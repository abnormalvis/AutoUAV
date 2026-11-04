#ifndef ARMOR_PERCEPTION__STATIC_TARGET_HPP
#define ARMOR_PERCEPTION__STATIC_TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <string>

#include "tasks/auto_aim/armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace armor_perception
{

// World-frame estimator for a *static* armor plate.
//
// Replaces sp_vision's `auto_aim::Target`, whose 11-state model
// [x vx y vy z vz a w r l h] describes a plate rotating on a robot's yaw axis:
// `r` is the plate's radius from the rotation centre and `w` the spin rate.
// For a plate bolted to a wall those parameters are unobservable -- they are
// carried by the initial value and random-walked by the process noise. Upstream
// then declares divergence when `r` leaves [0.05, 0.5] m (target.cpp:241-250),
// which for a static plate is a coin flip, and the tracker drops to "lost".
//
// This model is the smallest one that is actually observable from a monocular
// plate observation: a constant-velocity position in the world frame. The
// velocity states stay near zero for a static target and exist only so that a
// slow drift (a plate on a moving base, a slightly biased PnP) is tracked
// rather than fought.
//
// The plate orientation is *not* in the filter. A planar target seen from the
// front has a near-unobservable yaw (the classic flip ambiguity), so filtering
// it would only smooth noise into a slow random walk. Yaw is instead
// low-passed with a fixed time constant, which is honest about how much
// information there is.
class StaticTarget
{
public:
  struct Params
  {
    // Acceleration spectral density of the CV model, m^2/s^3. Small: the target
    // is static, and a large value lets the filter chase PnP jitter.
    double accel_noise = 0.05;

    // Floor on the per-axis measurement standard deviation, m. Derived from the
    // reprojection error at the measured range; this floor keeps very close
    // observations from producing an over-confident filter.
    double min_measurement_std = 0.005;

    // Yaw low-pass time constant, s.
    double yaw_time_constant = 0.2;

    // Initial covariance (position, velocity) diagonals.
    double initial_position_std = 0.5;
    double initial_velocity_std = 1.0;

    // Divergence test: this many consecutive updates with a position residual
    // above `jump_threshold` means the association is wrong.
    int max_consecutive_jumps = 5;
    double jump_threshold = 1.0;  // m
  };

  StaticTarget() = default;

  void setParams(const Params & params) { params_ = params; }
  const Params & params() const { return params_; }

  // (Re)initialise from a measurement. Safe to call while already initialised:
  // it resets the state, which the tracker does when it re-acquires.
  void initialize(
    const Eigen::Vector3d & p_world, double yaw_world, const Eigen::Vector3d & p_std,
    double t_seconds);

  bool initialized() const { return initialized_; }

  // Predict to `t_seconds` then fold in a measurement. Returns false when the
  // update was rejected as an outlier or the filter has diverged.
  bool update(
    const Eigen::Vector3d & p_world, double yaw_world, const Eigen::Vector3d & p_std,
    double t_seconds);

  // Advance the state without a measurement (a missed frame).
  void predict(double t_seconds);

  // State accessors, world frame.
  Eigen::Vector3d position() const;
  Eigen::Vector3d velocity() const;
  double yaw() const { return yaw_; }
  Eigen::Matrix3d positionCovariance() const;

  double positionStd() const;
  bool diverged() const { return diverged_; }

  // Frames since the last accepted measurement.
  int missedFrames() const { return missed_frames_; }

  // Reject updates whose Mahalanobis distance exceeds this. Chi-square, 3 DOF,
  // 99% -> 11.34. Configurable because SITL PnP noise is not the same as real.
  void setGateThreshold(double value) { gate_threshold_ = value; }

private:
  void propagate(double dt);
  Eigen::MatrixXd processNoise(double dt) const;

  Params params_;
  tools::ExtendedKalmanFilter ekf_;

  bool initialized_ = false;
  bool diverged_ = false;

  double t_ = 0.0;
  double yaw_ = 0.0;
  int missed_frames_ = 0;
  int consecutive_jumps_ = 0;
  double gate_threshold_ = 11.34;
};

}  // namespace armor_perception

#endif  // ARMOR_PERCEPTION__STATIC_TARGET_HPP
