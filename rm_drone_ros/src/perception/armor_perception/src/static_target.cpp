#include "armor_perception/static_target.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace armor_perception
{
namespace
{
constexpr int kStateDim = 6;   // px py pz vx vy vz
constexpr int kMeasDim = 3;    // px py pz

Eigen::MatrixXd makeF(double dt)
{
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(kStateDim, kStateDim);
  F.block<3, 3>(0, 3) = dt * Eigen::Matrix3d::Identity();
  return F;
}

// Wraps an angle to (-pi, pi].
double wrap(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}
}  // namespace

void StaticTarget::initialize(
  const Eigen::Vector3d & p_world, double yaw_world, const Eigen::Vector3d & p_std, double t_seconds)
{
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(kStateDim);
  x0.head<3>() = p_world;

  Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(kStateDim, kStateDim);
  const double ps = std::max(params_.initial_position_std, p_std.maxCoeff());
  P0.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * ps * ps;
  P0.block<3, 3>(3, 3) =
    Eigen::Matrix3d::Identity() * params_.initial_velocity_std * params_.initial_velocity_std;

  ekf_ = tools::ExtendedKalmanFilter(x0, P0);
  t_ = t_seconds;
  yaw_ = wrap(yaw_world);
  initialized_ = true;
  diverged_ = false;
  missed_frames_ = 0;
  consecutive_jumps_ = 0;
}

Eigen::MatrixXd StaticTarget::processNoise(double dt) const
{
  // Continuous white-acceleration model discretised over dt.
  const double q = params_.accel_noise;
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(kStateDim, kStateDim);
  Q.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (q * dt3 / 3.0);
  Q.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * (q * dt2 / 2.0);
  Q.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity() * (q * dt2 / 2.0);
  Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (q * dt);
  return Q;
}

void StaticTarget::propagate(double dt)
{
  if (!initialized_ || !(dt > 0.0)) return;
  // Cap the step: a long gap (e.g. the node was stalled) must not let the
  // covariance explode and then immediately accept a wild measurement.
  const double h = std::min(dt, 0.2);
  ekf_.predict(makeF(h), processNoise(h));
}

void StaticTarget::predict(double t_seconds)
{
  if (!initialized_) return;
  const double dt = t_seconds - t_;
  if (dt > 0.0) {
    propagate(dt);
    t_ = t_seconds;
  }
  ++missed_frames_;
}

bool StaticTarget::update(
  const Eigen::Vector3d & p_world, double yaw_world, const Eigen::Vector3d & p_std, double t_seconds)
{
  if (!initialized_) {
    initialize(p_world, yaw_world, p_std, t_seconds);
    return true;
  }

  const double dt = t_seconds - t_;
  if (dt > 0.0) {
    propagate(dt);
    t_ = t_seconds;
  }

  // Gating: a measurement far outside the predicted uncertainty is an
  // association error or a PnP flip, not target motion. Rejecting it keeps the
  // filter from being dragged across the room by one bad frame.
  const Eigen::Vector3d residual = p_world - ekf_.x.head<3>();
  Eigen::Matrix3d S = ekf_.P.block<3, 3>(0, 0);
  S += p_std.array().square().matrix().asDiagonal();
  const double mahalanobis = residual.transpose() * S.inverse() * residual;

  if (mahalanobis > gate_threshold_) {
    ++consecutive_jumps_;
    ++missed_frames_;
    if (consecutive_jumps_ >= params_.max_consecutive_jumps) diverged_ = true;
    return false;
  }
  consecutive_jumps_ = 0;

  // Position residual as a plain jump test, independent of the covariance:
  // catches the case where P has grown enough to make the gate permissive.
  if (residual.norm() > params_.jump_threshold) {
    ++consecutive_jumps_;
    if (consecutive_jumps_ >= params_.max_consecutive_jumps) diverged_ = true;
    return false;
  }

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(kMeasDim, kStateDim);
  H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

  Eigen::Matrix3d R = p_std.array().square().matrix().asDiagonal();
  const double floor_var = params_.min_measurement_std * params_.min_measurement_std;
  R += Eigen::Matrix3d::Identity() * floor_var;

  ekf_.update(p_world, H, R);

  // Yaw is low-passed rather than filtered: first-order lag with a fixed time
  // constant, using the actual elapsed time so a dropped frame does not slow
  // the response.
  const double alpha =
    (params_.yaw_time_constant > 0.0 && dt > 0.0)
      ? (1.0 - std::exp(-dt / params_.yaw_time_constant))
      : 1.0;
  yaw_ = wrap(yaw_ + alpha * wrap(yaw_world - yaw_));

  missed_frames_ = 0;
  return true;
}

Eigen::Vector3d StaticTarget::position() const
{
  if (!initialized_) return Eigen::Vector3d::Zero();
  return ekf_.x.head<3>();
}

Eigen::Vector3d StaticTarget::velocity() const
{
  if (!initialized_) return Eigen::Vector3d::Zero();
  return ekf_.x.tail<3>();
}

Eigen::Matrix3d StaticTarget::positionCovariance() const
{
  if (!initialized_) return Eigen::Matrix3d::Zero();
  return Eigen::Matrix3d(ekf_.P.block<3, 3>(0, 0));
}

double StaticTarget::positionStd() const
{
  if (!initialized_) return std::numeric_limits<double>::infinity();
  return std::sqrt(positionCovariance().diagonal().maxCoeff());
}

}  // namespace armor_perception
