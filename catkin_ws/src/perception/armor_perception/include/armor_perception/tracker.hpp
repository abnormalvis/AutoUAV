#ifndef ARMOR_PERCEPTION__TRACKER_HPP
#define ARMOR_PERCEPTION__TRACKER_HPP

#include <Eigen/Dense>
#include <list>
#include <opencv2/core.hpp>
#include <string>

#include "static_target.hpp"
#include "tasks/auto_aim/armor.hpp"

namespace armor_perception
{

// Detection-to-track association plus the acquire/lose state machine.
//
// The state machine is the part of sp_vision's `Tracker` worth keeping:
// `min_detect_count` stops a single spurious detection from starting a track,
// and `max_temp_lost_count` keeps a track alive through a few dropped frames
// instead of declaring loss at the first blink.
//
// What changed:
//   * the omni-perception overload is gone -- it dragged in
//     `tasks/omniperception/perceptron.hpp` -> `io/usbcamera` -> vendor SDKs;
//   * the estimator is `StaticTarget`, not the 11-state ring model;
//   * timestamps are `double` seconds (ROS time), because
//     `std::chrono::steady_clock` cannot be compared with a header stamp;
//   * the image centre used for association is a parameter, not the hardcoded
//     `1440/2, 1080/2` of the upstream gimbal camera.
class Tracker
{
public:
  enum class State
  {
    LOST = 0,
    DETECTING = 1,
    TRACKING = 2,
    TEMP_LOST = 3
  };

  struct Params
  {
    int min_detect_count = 5;      // consecutive detections before TRACKING
    int max_temp_lost_count = 15;  // consecutive misses before LOST
  };

  void setParams(const Params & params) { params_ = params; }
  const Params & params() const { return params_; }

  void setTargetParams(const StaticTarget::Params & params) { target_params_ = params; }
  const StaticTarget::Params & targetParams() const { return target_params_; }

  // Association gate: a detection further than this from the prediction is
  // treated as a different object rather than an update of the current track.
  void setAssociationRadius(double metres) { association_radius_ = metres; }

  // Folds one frame of solved armors into the track. `armors` must already
  // carry world-frame poses (ArmorSolver::solve). `img_center` is used only
  // before a track exists, to prefer the detection nearest the optical axis.
  // Returns true when the tracker is in TRACKING after this frame.
  bool track(
    const std::list<auto_aim::Armor> & armors, double t_seconds, const cv::Point2f & img_center,
    const Eigen::Vector3d & p_std);

  State state() const { return state_; }
  const char * stateName() const;

  const StaticTarget & target() const { return target_; }

  // True when a position estimate is available (tracking or briefly lost).
  bool hasTarget() const { return state_ == State::TRACKING || state_ == State::TEMP_LOST; }
  Eigen::Vector3d position() const { return target_.position(); }

  // Orientation of the most recently accepted observation. Kept outside the
  // filter because a fronto-parallel planar target has an unobservable yaw.
  const Eigen::Matrix3d & orientation() const { return last_R_; }

  void reset();

private:
  // Chooses the armor to associate with, or nullptr.
  const auto_aim::Armor * select(
    const std::list<auto_aim::Armor> & armors, const cv::Point2f & img_center) const;

  State state_ = State::LOST;
  Params params_;
  StaticTarget::Params target_params_;
  StaticTarget target_;

  double association_radius_ = 0.5;  // m

  int detect_count_ = 0;
  int temp_lost_count_ = 0;

  Eigen::Matrix3d last_R_ = Eigen::Matrix3d::Identity();
};

}  // namespace armor_perception

#endif  // ARMOR_PERCEPTION__TRACKER_HPP
