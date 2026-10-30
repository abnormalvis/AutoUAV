#ifndef ARMOR_PERCEPTION__ODOMETRY_BUFFER_HPP
#define ARMOR_PERCEPTION__ODOMETRY_BUFFER_HPP

#include <Eigen/Geometry>
#include <deque>
#include <string>

#include <nav_msgs/Odometry.h>

namespace armor_perception
{

// Time-ordered buffer of body poses, used to evaluate T_world_base at an
// arbitrary timestamp (the image stamp). See plan §3.4.
//
// Why interpolate at all: the camera runs at 30 Hz while odometry arrives at
// whatever rate the VIO produces (typically 20-100 Hz). Using "the latest
// odometry" instead of the sample at the image stamp leaves a residual
// translation of v * dt, which at 1 m/s and 30 ms is 3 cm of target position
// error that grows with flight speed -- exactly the failure mode the
// ego-motion compensation is supposed to remove.
class OdometryBuffer
{
public:
  // Reject samples whose stamp is more than this far behind the newest one.
  // Beyond that the stream is considered reordered/garbage rather than jitter.
  static constexpr double kReorderTolerance = 0.5;  // s

  void setCapacity(std::size_t n) { capacity_ = n; }
  void setMaxAge(double age) { max_age_ = age; }

  // Returns false if the sample was discarded (stamp <= 0, or far out of order).
  bool push(const nav_msgs::Odometry & odom);

  // Interpolates the pose at `stamp`.
  //
  // Returns false and fills `error` when:
  //   * the buffer is empty,
  //   * `stamp` is newer than the newest sample by more than maxAge(),
  //   * `stamp` is older than the oldest retained sample,
  //   * `stamp` lies more than maxAge() ahead of the newest sample (the caller
  //     must drop the frame rather than extrapolate).
  bool lookup(double stamp, Eigen::Isometry3d & T_world_base, std::string & error) const;

  bool empty() const { return samples_.empty(); }
  std::size_t size() const { return samples_.size(); }
  double newestStamp() const { return samples_.empty() ? 0.0 : samples_.back().stamp; }
  double oldestStamp() const { return samples_.empty() ? 0.0 : samples_.front().stamp; }
  double maxAge() const { return max_age_; }

  // Frame id of the most recent sample (the world frame the poses live in).
  const std::string & frameId() const { return frame_id_; }

  void clear();

private:
  struct Sample
  {
    double stamp = 0.0;
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  };

  std::deque<Sample> samples_;
  std::size_t capacity_ = 200;
  double max_age_ = 0.05;  // s
  std::string frame_id_ = "world";
};

}  // namespace armor_perception

#endif  // ARMOR_PERCEPTION__ODOMETRY_BUFFER_HPP
