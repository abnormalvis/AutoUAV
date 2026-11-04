#include "armor_perception/tracker.hpp"

#include <algorithm>
#include <limits>

namespace armor_perception
{

void Tracker::reset()
{
  state_ = State::LOST;
  detect_count_ = 0;
  temp_lost_count_ = 0;
  target_ = StaticTarget();
  target_.setParams(target_params_);
  last_R_ = Eigen::Matrix3d::Identity();
}

const char * Tracker::stateName() const
{
  switch (state_) {
    case State::LOST:
      return "LOST";
    case State::DETECTING:
      return "DETECTING";
    case State::TRACKING:
      return "TRACKING";
    case State::TEMP_LOST:
      return "TEMP_LOST";
  }
  return "UNKNOWN";
}

const auto_aim::Armor * Tracker::select(
  const std::list<auto_aim::Armor> & armors, const cv::Point2f & img_center) const
{
  const auto_aim::Armor * best = nullptr;
  double best_score = std::numeric_limits<double>::max();

  const bool tracking = (state_ == State::TRACKING || state_ == State::TEMP_LOST) &&
                        target_.initialized();

  for (const auto & armor : armors) {
    double score;
    if (tracking) {
      // Nearest to the prediction, in metres.
      const double d = (armor.xyz_in_world - target_.position()).norm();
      if (d > association_radius_) continue;
      score = d;
    } else {
      // No track yet: nearest to the optical axis, in pixels. This is the
      // behaviour the upstream tracker had, minus the hardcoded resolution.
      score = cv::norm(armor.center - img_center);
    }
    if (score < best_score) {
      best_score = score;
      best = &armor;
    }
  }
  return best;
}

bool Tracker::track(
  const std::list<auto_aim::Armor> & armors, double t_seconds, const cv::Point2f & img_center,
  const Eigen::Vector3d & p_std)
{
  const auto_aim::Armor * armor = select(armors, img_center);

  if (armor == nullptr) {
    switch (state_) {
      case State::LOST:
      case State::DETECTING:
        // Nothing to hold on to; stay where we are.
        detect_count_ = 0;
        state_ = State::LOST;
        break;
      case State::TRACKING:
        state_ = State::TEMP_LOST;
        temp_lost_count_ = 0;
        target_.predict(t_seconds);
        break;
      case State::TEMP_LOST:
        ++temp_lost_count_;
        if (temp_lost_count_ > params_.max_temp_lost_count) {
          reset();
        } else {
          target_.predict(t_seconds);
        }
        break;
    }
    return state_ == State::TRACKING;
  }

  switch (state_) {
    case State::LOST:
      // First sighting: seed the estimator directly, no hysteresis yet. The
      // DETECTING state is what enforces min_detect_count, and seeding here
      // gives the association gate a prediction to work with.
      target_.setParams(target_params_);
      target_.initialize(
        armor->xyz_in_world,
        std::atan2(armor->R_armor2world(1, 0), armor->R_armor2world(0, 0)), p_std, t_seconds);
      detect_count_ = 1;
      last_R_ = armor->R_armor2world;
      state_ = (detect_count_ >= params_.min_detect_count) ? State::TRACKING : State::DETECTING;
      break;

    case State::DETECTING: {
      const bool ok = target_.update(
        armor->xyz_in_world,
        std::atan2(armor->R_armor2world(1, 0), armor->R_armor2world(0, 0)), p_std, t_seconds);
      if (ok) {
        ++detect_count_;
        last_R_ = armor->R_armor2world;
      } else {
        // Rejected by the gate: the new sighting is not the same object, so
        // restart the count rather than accumulate unrelated detections.
        detect_count_ = 1;
        target_.initialize(
          armor->xyz_in_world,
          std::atan2(armor->R_armor2world(1, 0), armor->R_armor2world(0, 0)), p_std, t_seconds);
      }
      if (detect_count_ >= params_.min_detect_count) state_ = State::TRACKING;
      break;
    }

    case State::TRACKING:
    case State::TEMP_LOST: {
      const bool ok = target_.update(
        armor->xyz_in_world,
        std::atan2(armor->R_armor2world(1, 0), armor->R_armor2world(0, 0)), p_std, t_seconds);
      if (ok) {
        last_R_ = armor->R_armor2world;
        temp_lost_count_ = 0;
        state_ = State::TRACKING;
      } else if (target_.diverged()) {
        // The estimator and the detections disagree persistently; start over
        // rather than publish a drifting pose.
        reset();
      } else {
        state_ = State::TEMP_LOST;
        ++temp_lost_count_;
        if (temp_lost_count_ > params_.max_temp_lost_count) reset();
      }
      break;
    }
  }

  return state_ == State::TRACKING;
}

}  // namespace armor_perception
