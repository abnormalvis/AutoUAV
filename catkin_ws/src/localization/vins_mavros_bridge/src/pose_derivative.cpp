#include "vins_mavros_bridge/pose_derivative.h"

#include <algorithm>
#include <cmath>

namespace vins_mavros_bridge
{
namespace
{
constexpr double kEpsilon = 1e-12;
}  // namespace

void PoseDerivative::setParams(const Params& params)
{
  params_ = params;
  if (!(params_.window > 0.0))
  {
    params_.window = 1e-3;
  }
  if (!(params_.min_dt > 0.0))
  {
    params_.min_dt = 1e-3;
  }
  if (params_.capacity < 2)
  {
    params_.capacity = 2;
  }
  reset();
}

void PoseDerivative::reset()
{
  samples_.clear();
  has_estimate_ = false;
}

bool PoseDerivative::update(double stamp, const Eigen::Vector3d& position,
                            const Eigen::Quaterniond& orientation, Eigen::Vector3d* linear,
                            Eigen::Vector3d* angular)
{
  if (linear == nullptr || angular == nullptr)
  {
    return false;
  }
  if (!std::isfinite(stamp) || !position.allFinite())
  {
    reset();
    return false;
  }

  // A timestamp that moves backwards is an estimator restart, not motion. The
  // window would otherwise span two unrelated trajectories and produce a
  // velocity spike of arbitrary size.
  if (!samples_.empty() && stamp < samples_.back().stamp)
  {
    reset();
    ++resets_;
  }

  Sample sample;
  sample.stamp = stamp;
  sample.position = position;
  sample.orientation = orientation.normalized();
  samples_.push_back(sample);

  while (samples_.size() > params_.capacity)
  {
    samples_.pop_front();
  }

  // Drop samples older than the window, but never going below two: differentiating
  // needs a baseline, and the sample just before the window edge is the one that
  // gives it.
  while (samples_.size() > 2 && samples_.front().stamp < stamp - params_.window)
  {
    samples_.pop_front();
  }

  const Sample& oldest = samples_.front();
  const Sample& newest = samples_.back();
  const double baseline = newest.stamp - oldest.stamp;
  if (!(baseline >= params_.min_dt))
  {
    return false;
  }

  // Least-squares slope of position over the window. A plain two-point
  // difference would amplify pose noise by 1/dt; the fit averages it across the
  // whole window instead.
  double stamp_mean = 0.0;
  Eigen::Vector3d position_mean = Eigen::Vector3d::Zero();
  for (const Sample& s : samples_)
  {
    stamp_mean += s.stamp;
    position_mean += s.position;
  }
  const double inverse_count = 1.0 / static_cast<double>(samples_.size());
  stamp_mean *= inverse_count;
  position_mean *= inverse_count;

  double denominator = 0.0;
  Eigen::Vector3d numerator = Eigen::Vector3d::Zero();
  for (const Sample& s : samples_)
  {
    const double dt = s.stamp - stamp_mean;
    denominator += dt * dt;
    numerator += dt * (s.position - position_mean);
  }
  if (!(denominator > kEpsilon))
  {
    return false;
  }
  const Eigen::Vector3d linear_world = numerator / denominator;

  // Angular rate from the endpoint quaternion difference. q maps body to world,
  // so oldest^-1 * newest is the rotation accumulated in the body frame.
  Eigen::Quaterniond delta = oldest.orientation.conjugate() * newest.orientation;
  if (delta.w() < 0.0)
  {
    // q and -q are the same rotation; take the short way round so a slow
    // rotation never reads as a near-full turn.
    delta.coeffs() *= -1.0;
  }
  delta.normalize();

  const Eigen::AngleAxisd rotation(delta);
  if (!std::isfinite(rotation.angle()))
  {
    return false;
  }
  const Eigen::Vector3d angular_body = rotation.axis() * (rotation.angle() / baseline);

  if (!linear_world.allFinite() || !angular_body.allFinite())
  {
    return false;
  }

  *linear = linear_world;
  *angular = angular_body;
  has_estimate_ = true;
  return true;
}

}  // namespace vins_mavros_bridge
