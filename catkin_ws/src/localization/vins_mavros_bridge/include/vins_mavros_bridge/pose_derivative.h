#ifndef VINS_MAVROS_BRIDGE_POSE_DERIVATIVE_H_
#define VINS_MAVROS_BRIDGE_POSE_DERIVATIVE_H_

#include <cstddef>
#include <deque>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace vins_mavros_bridge
{

/// Recovers velocity from a position-only estimate by differentiating it.
///
/// A pose-only source (ORB-SLAM3's PoseStamped topic, SVO-based estimators) has
/// no velocity to forward. Publishing zero is not neutral: px4ctrl's velocity
/// term Kv * (v_des - v) then contributes a constant false error, and
/// EGO-Planner's initial state is wrong. Differentiating is the only option, and
/// it is what the rest of the stack already assumes a VIO provides.
///
/// Two choices decide whether the result is usable:
///
///   - A two-point difference over one frame interval at 30 Hz turns
///     millimetre-level pose noise into metres per second of velocity noise.
///     The linear estimate is therefore a least-squares slope over a ~0.1 s
///     window, not a single interval.
///   - Angular rate comes from the quaternion difference across the same
///     window, q_prev^-1 * q_cur. That rotation is expressed in the body frame,
///     so its axis is the rotation axis and its angle over dt is the rate.
///
/// The output is deliberately world-frame linear velocity and body-frame
/// angular rate: those are the two conventions the bridge already knows how to
/// normalize, so a derived twist takes the same path as a measured one.
///
/// No ROS types here on purpose, so it can be tested without a master.
class PoseDerivative
{
public:
  struct Params
  {
    /// Temporal baseline. Longer is smoother but adds lag; 0.1 s at 30 Hz is
    /// three to five samples, roughly one control-loop time constant.
    double window{0.1};

    /// Below this baseline there is nothing to differentiate.
    double min_dt{0.02};

    /// Hard cap on retained samples, independent of `window`, so a stalled
    /// estimator cannot grow the buffer without bound.
    std::size_t capacity{64};
  };

  void setParams(const Params& params);
  const Params& params() const { return params_; }

  /// Discards the accumulated window. Call after a known discontinuity.
  void reset();

  /// Adds a sample and, once the window is long enough, writes the estimate.
  ///
  /// @param stamp        seconds; a stamp that moves backwards is treated as an
  ///                     estimator restart and clears the window
  /// @param position     world position of the estimated frame
  /// @param orientation  world orientation of the estimated frame (body -> world)
  /// @param linear       world-frame velocity; untouched unless true is returned
  /// @param angular      body-frame angular rate; untouched unless true is returned
  /// @return true when both estimates were produced
  bool update(double stamp, const Eigen::Vector3d& position,
              const Eigen::Quaterniond& orientation, Eigen::Vector3d* linear,
              Eigen::Vector3d* angular);

  std::size_t size() const { return samples_.size(); }

  /// Number of times a backwards timestamp forced a window reset.
  unsigned long long resets() const { return resets_; }

  /// True once at least one velocity estimate has been produced since the last
  /// reset. Callers use this to distinguish "not warmed up" from "zero motion".
  bool hasEstimate() const { return has_estimate_; }

private:
  struct Sample
  {
    double stamp{0.0};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  };

  Params params_;
  std::deque<Sample> samples_;
  bool has_estimate_{false};
  unsigned long long resets_{0};
};

}  // namespace vins_mavros_bridge

#endif  // VINS_MAVROS_BRIDGE_POSE_DERIVATIVE_H_
