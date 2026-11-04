#include "armor_perception/armor_solver.hpp"

#include <cmath>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace armor_perception
{
namespace
{
// Object points in the armor frame: +X is the outward normal (towards the
// observer), +Y points to the observer's right, +Z is the plate's up axis.
//
// This ordering matches auto_aim::Armor::points, which the detector fills as
// {left.top, right.top, right.bottom, left.bottom} -- i.e. top-left,
// top-right, bottom-right, bottom-left as seen in the image.
//
// Upstream sp_vision used {0, +-W/2, +-L/2}, putting the normal on +X and the
// plate's up axis on +Z as well; the difference is only that we do not rely on
// a downstream "add +-15 deg pitch" step to recover the orientation.
std::vector<cv::Point3f> makeObjectPoints(double width, double lightbar_length)
{
  const float hw = static_cast<float>(width / 2.0);
  const float hl = static_cast<float>(lightbar_length / 2.0);
  return {
    {0.0f, -hw, hl},   // top-left
    {0.0f, hw, hl},    // top-right
    {0.0f, hw, -hl},   // bottom-right
    {0.0f, -hw, -hl},  // bottom-left
  };
}
}  // namespace

bool ArmorSolver::setCameraIntrinsics(
  const cv::Mat & camera_matrix, const cv::Mat & distort_coeffs)
{
  if (camera_matrix.rows != 3 || camera_matrix.cols != 3) return false;
  if (camera_matrix.type() != CV_64F) return false;

  const double fx = camera_matrix.at<double>(0, 0);
  const double fy = camera_matrix.at<double>(1, 1);
  if (!(fx > 0.0) || !(fy > 0.0) || !std::isfinite(fx) || !std::isfinite(fy)) return false;

  camera_matrix_ = camera_matrix.clone();
  distort_coeffs_ = distort_coeffs.clone();
  return true;
}

void ArmorSolver::resetContinuity()
{
  have_hint_ = false;
  hint_p_.setZero();
  hint_R_.setIdentity();
}

std::vector<cv::Point3f> ArmorSolver::objectPoints(auto_aim::ArmorType type) const
{
  const double width =
    (type == auto_aim::ArmorType::big) ? dims_.big_armor_width : dims_.small_armor_width;
  return makeObjectPoints(width, dims_.lightbar_length);
}

double ArmorSolver::score(
  const Eigen::Vector3d & p_world, const Eigen::Matrix3d & R_world, double reproj_error,
  bool have_hint, const Eigen::Vector3d & hint_p, const Eigen::Matrix3d & hint_R) const
{
  // Reprojection error is in pixels and is the primary signal: the two IPPE
  // solutions of a planar target often differ by a few pixels at most.
  double cost = reproj_error;

  // Prefer the branch whose normal faces the camera. The plate is observed from
  // the front, so a normal pointing away means the mirrored solution was picked.
  if (prefer_frontal_) {
    const Eigen::Vector3d n_world = R_world.col(0);
    const Eigen::Vector3d to_camera = T_world_camera_.translation() - p_world;
    if (n_world.dot(to_camera) < 0.0) cost += 1.0e3;
  }

  // Temporal continuity: a jump of more than ~1 m or ~60 deg between consecutive
  // frames is a branch flip, not motion. Weighted low so it only breaks ties.
  if (have_hint) {
    const double dp = (p_world - hint_p).norm();
    const Eigen::Matrix3d dR = hint_R.transpose() * R_world;
    const double dangle = std::acos(std::max(-1.0, std::min(1.0, (dR.trace() - 1.0) / 2.0)));
    cost += 50.0 * dp + 5.0 * dangle;
  }

  return cost;
}

bool ArmorSolver::solve(auto_aim::Armor & armor) const
{
  if (camera_matrix_.empty()) return false;
  if (!hasDimensions()) return false;
  if (armor.points.size() != 4) return false;

  const auto object_points = objectPoints(armor.type);

  std::vector<cv::Mat> rvecs, tvecs;
  std::vector<double> reproj_errors;
  try {
    cv::solvePnPGeneric(
      object_points, armor.points, camera_matrix_, distort_coeffs_, rvecs, tvecs, false,
      cv::SOLVEPNP_IPPE, cv::noArray(), cv::noArray(), reproj_errors);
  } catch (const cv::Exception &) {
    return false;
  }
  if (rvecs.empty()) return false;

  const Eigen::Matrix3d R_world_camera = T_world_camera_.linear();
  const Eigen::Vector3d p_world_camera = T_world_camera_.translation();

  bool found = false;
  double best_cost = 0.0;
  Eigen::Vector3d best_p = Eigen::Vector3d::Zero();
  Eigen::Matrix3d best_R = Eigen::Matrix3d::Identity();
  double best_reproj = 0.0;

  for (std::size_t i = 0; i < rvecs.size(); ++i) {
    const cv::Vec3d & tvec = tvecs[i];
    if (!(tvec[2] > 0.0)) continue;  // behind the camera

    cv::Mat rmat;
    cv::Rodrigues(rvecs[i], rmat);
    Eigen::Matrix3d R_camera_armor;
    cv::cv2eigen(rmat, R_camera_armor);

    const Eigen::Vector3d p_camera(tvec[0], tvec[1], tvec[2]);
    const Eigen::Vector3d p_world = R_world_camera * p_camera + p_world_camera;
    const Eigen::Matrix3d R_world = R_world_camera * R_camera_armor;

    // solvePnPGeneric returns per-solution reprojection errors in the same
    // order as the solutions; fall back to 0 if the vector is short.
    const double reproj = (i < reproj_errors.size()) ? reproj_errors[i] : 0.0;

    const double cost = score(p_world, R_world, reproj, have_hint_, hint_p_, hint_R_);
    if (!found || cost < best_cost) {
      found = true;
      best_cost = cost;
      best_p = p_world;
      best_R = R_world;
      best_reproj = reproj;
    }
  }

  if (!found) return false;

  armor.xyz_in_world = best_p;
  armor.R_armor2world = best_R;
  armor.ypr_in_world = Eigen::Vector3d(
    std::atan2(best_R(1, 0), best_R(0, 0)),
    std::asin(std::max(-1.0, std::min(1.0, -best_R(2, 0)))),
    std::atan2(best_R(2, 1), best_R(2, 2)));

  have_hint_ = true;
  hint_p_ = best_p;
  hint_R_ = best_R;
  last_reprojection_error_ = best_reproj;

  return true;
}

std::vector<cv::Point2f> ArmorSolver::reproject(
  const Eigen::Vector3d & xyz_in_world, const Eigen::Matrix3d & R_armor2world,
  auto_aim::ArmorType type) const
{
  std::vector<cv::Point2f> image_points;
  if (camera_matrix_.empty()) return image_points;

  const Eigen::Matrix3d R_camera_world = T_world_camera_.linear().transpose();
  const Eigen::Vector3d p_camera = R_camera_world * (xyz_in_world - T_world_camera_.translation());

  Eigen::Matrix3d R_camera_armor = R_camera_world * R_armor2world;

  cv::Mat R_cv;
  cv::eigen2cv(R_camera_armor, R_cv);
  cv::Vec3d rvec;
  cv::Rodrigues(R_cv, rvec);
  const cv::Vec3d tvec(p_camera.x(), p_camera.y(), p_camera.z());

  cv::projectPoints(
    objectPoints(type), rvec, tvec, camera_matrix_, distort_coeffs_, image_points);
  return image_points;
}

}  // namespace armor_perception
