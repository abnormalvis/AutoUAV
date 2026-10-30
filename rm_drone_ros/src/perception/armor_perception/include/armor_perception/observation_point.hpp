#ifndef ARMOR_PERCEPTION__OBSERVATION_POINT_HPP
#define ARMOR_PERCEPTION__OBSERVATION_POINT_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>

namespace armor_perception
{

// Chooses where the drone should hover to look at the plate (plan §4.4).
//
// The point is `standoff` metres along the plate's outward normal, on the side
// the camera is already on. Generating it from the *normal* rather than from
// the current line of sight is what makes the drone end up square to the plate
// instead of merely close to it.
//
// Two guards keep the point from jumping around:
//   * the normal is only trusted when the estimated orientation is fresh and
//     the range is short enough for PnP yaw to be meaningful; otherwise the
//     point degrades to "back off along the current line of sight", which is
//     always well-defined;
//   * the resulting distance is clamped to [min_standoff, max_standoff], so a
//     bad normal can never command a collision course.
class ObservationPointGenerator
{
public:
  struct Params
  {
    // 1.5 m: at fx=455 the light bar is ~17 px tall, PnP lateral accuracy is
    // good, and the plate fills a comfortable fraction of the frame. See the
    // pixel budget in plan §零.
    double standoff = 1.5;

    // Hard limits. The minimum is a safety floor: with detection lost the
    // drone must never be commanded closer than this.
    double min_standoff = 0.8;
    double max_standoff = 3.0;

    // The normal is trusted only within this range. Beyond it the monocular
    // yaw estimate is too noisy to build a hover point on.
    double max_normal_range = 6.0;
  };

  void setParams(const Params & params) { params_ = params; }
  const Params & params() const { return params_; }

  struct Result
  {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    double yaw = 0.0;          // heading that points the body +X at the plate
    bool normal_used = false;  // false => degraded to the line-of-sight fallback
    double standoff = 0.0;     // achieved distance to the plate, m
  };

  // p_armor / R_armor2world: filtered target pose in the world frame.
  // p_camera: current camera (or body) position in the world frame, used only
  //           to pick the side of the plate and as the fallback direction.
  Result generate(
    const Eigen::Vector3d & p_armor, const Eigen::Matrix3d & R_armor2world,
    const Eigen::Vector3d & p_camera, bool normal_trusted) const;

private:
  Params params_;
};

}  // namespace armor_perception

#endif  // ARMOR_PERCEPTION__OBSERVATION_POINT_HPP
