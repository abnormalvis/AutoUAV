#include "armor_perception/observation_point.hpp"

#include <algorithm>
#include <cmath>

namespace armor_perception
{

ObservationPointGenerator::Result ObservationPointGenerator::generate(
  const Eigen::Vector3d & p_armor, const Eigen::Matrix3d & R_armor2world,
  const Eigen::Vector3d & p_camera, bool normal_trusted) const
{
  Result result;

  const Eigen::Vector3d to_camera = p_camera - p_armor;
  const double range = to_camera.norm();

  Eigen::Vector3d direction;
  if (normal_trusted && range > 1e-6) {
    // Column 0 of R_armor2world is the plate's outward normal, by construction
    // in ArmorSolver::objectPoints().
    direction = R_armor2world.col(0).normalized();

    // Pick the side the camera is already on, so the drone is never sent
    // through the wall the plate is mounted on.
    if (direction.dot(to_camera) < 0.0) direction = -direction;
    result.normal_used = true;
  } else {
    // Fallback: keep the current bearing and back off along it. Always defined,
    // and continuous with the normal-based point when the normal flips.
    direction = (range > 1e-6) ? Eigen::Vector3d(to_camera / range) : Eigen::Vector3d::UnitX();
    result.normal_used = false;
  }

  // Clamp the requested standoff. The achieved value is published, so a caller
  // that asked for something outside the limits can see what it got.
  const double standoff =
    std::min(std::max(params_.standoff, params_.min_standoff), params_.max_standoff);
  result.standoff = standoff;
  result.position = p_armor + direction * standoff;

  // Heading that points body +X at the plate, so the forward camera sees it.
  const Eigen::Vector2d look(p_armor.x() - result.position.x(), p_armor.y() - result.position.y());
  result.yaw = (look.norm() > 1e-6) ? std::atan2(look.y(), look.x()) : 0.0;

  return result;
}

}  // namespace armor_perception
