#include "armor_perception/odometry_buffer.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace armor_perception
{

void OdometryBuffer::clear()
{
  samples_.clear();
  frame_id_ = "world";
}

bool OdometryBuffer::push(const nav_msgs::Odometry & odom)
{
  Sample s;
  s.stamp = odom.header.stamp.toSec();
  if (!(s.stamp > 0.0)) return false;

  const auto & p = odom.pose.pose.position;
  s.p = Eigen::Vector3d(p.x, p.y, p.z);

  const auto & q = odom.pose.pose.orientation;
  s.q = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
  if (s.q.norm() < 1e-9) return false;  // all-zero quaternion
  s.q.normalize();

  if (!std::isfinite(s.p.x()) || !std::isfinite(s.p.y()) || !std::isfinite(s.p.z())) return false;

  if (!odom.header.frame_id.empty()) frame_id_ = odom.header.frame_id;

  // Drop anything that arrives well behind the newest sample: out-of-order
  // delivery past the tolerance means the source is not monotonic, and keeping
  // such samples would break the binary-search precondition.
  if (!samples_.empty() && s.stamp < samples_.back().stamp - kReorderTolerance) return false;

  // Insert in sorted order. The common case (in-order arrival) is a push_back.
  if (samples_.empty() || s.stamp >= samples_.back().stamp) {
    samples_.push_back(s);
  } else {
    auto it = std::lower_bound(
      samples_.begin(), samples_.end(), s.stamp,
      [](const Sample & a, double t) { return a.stamp < t; });
    samples_.insert(it, s);
  }

  while (samples_.size() > capacity_) samples_.pop_front();

  return true;
}

bool OdometryBuffer::lookup(double stamp, Eigen::Isometry3d & T_world_base, std::string & error) const
{
  if (samples_.empty()) {
    error = "no odometry received yet";
    return false;
  }
  if (!(stamp > 0.0)) {
    error = "image stamp is zero";
    return false;
  }

  const double newest = samples_.back().stamp;
  const double oldest = samples_.front().stamp;

  // The frame is newer than anything we have. Extrapolating a pose is worse
  // than dropping the frame: the error is proportional to the velocity, so it
  // shows up exactly when the drone is moving fast.
  if (stamp > newest + max_age_) {
    std::ostringstream os;
    os << "image is " << (stamp - newest) << " s newer than the newest odometry"
       << " (max " << max_age_ << " s)";
    error = os.str();
    return false;
  }
  if (stamp < oldest) {
    std::ostringstream os;
    os << "image is " << (oldest - stamp) << " s older than the oldest retained odometry";
    error = os.str();
    return false;
  }

  // Clamp to the newest sample when the image is slightly ahead of it. Within
  // maxAge() the induced error is bounded and a hold is better than a drop.
  if (stamp >= newest) {
    const auto & s = samples_.back();
    T_world_base = Eigen::Isometry3d::Identity();
    T_world_base.linear() = s.q.toRotationMatrix();
    T_world_base.translation() = s.p;
    error.clear();
    return true;
  }

  // Find the bracketing pair. lower_bound gives the first sample with
  // stamp_i >= stamp; since stamp < newest, that element exists and is not begin
  // unless stamp == oldest (handled by the interpolation guard below).
  auto hi = std::lower_bound(
    samples_.begin(), samples_.end(), stamp,
    [](const Sample & a, double t) { return a.stamp < t; });
  if (hi == samples_.begin()) {
    const auto & s = samples_.front();
    T_world_base = Eigen::Isometry3d::Identity();
    T_world_base.linear() = s.q.toRotationMatrix();
    T_world_base.translation() = s.p;
    error.clear();
    return true;
  }

  const auto & b = *hi;
  const auto & a = *(hi - 1);
  const double span = b.stamp - a.stamp;
  const double alpha = (span > 1e-9) ? (stamp - a.stamp) / span : 0.0;

  T_world_base = Eigen::Isometry3d::Identity();
  T_world_base.translation() = a.p + alpha * (b.p - a.p);
  T_world_base.linear() = a.q.slerp(alpha, b.q).toRotationMatrix();
  error.clear();
  return true;
}

}  // namespace armor_perception
