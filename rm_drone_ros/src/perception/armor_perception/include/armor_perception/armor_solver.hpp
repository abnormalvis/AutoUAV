#ifndef ARMOR_PERCEPTION__ARMOR_SOLVER_HPP
#define ARMOR_PERCEPTION__ARMOR_SOLVER_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <list>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <vector>

#include "tasks/auto_aim/armor.hpp"

namespace armor_perception
{

// Dimensions of the plate being observed, in metres. Upstream sp_vision keeps
// these as `constexpr` in solver.cpp; a simulation model or a differently built
// plate will not match, so they are configuration here.
struct ArmorDimensions
{
  double lightbar_length = 0.056;
  double small_armor_width = 0.135;
  double big_armor_width = 0.230;
};

// Turns detected armor corners into a world-frame 6-DOF pose.
//
// Replaces sp_vision's `Solver`. The reason is not cosmetic: upstream computed
//
//     armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;
//
// with no translation term, because for a gimbal the rotation centre is the
// world origin. On a translating UAV that is silently wrong by the vehicle's
// own position. Here the caller supplies T_world_camera for the image
// timestamp, so
//
//     T_world_armor = T_world_camera * T_camera_armor
//
// and ego-motion compensation falls out of the timestamp alignment rather than
// needing a separate velocity term (see plan §3.5).
//
// Armor frame convention (deliberately different from upstream, see
// objectPoints()): +X is the plate's outward normal, +Y points to the
// observer's right, +Z is the plate's up axis.
class ArmorSolver
{
public:
  void setDimensions(const ArmorDimensions & dims) { dims_ = dims; }
  const ArmorDimensions & dimensions() const { return dims_; }

  // Prefer camera_info over the YAML fallback. Returns false for degenerate
  // intrinsics (fx or fy <= 0), in which case the previous matrix is kept.
  bool setCameraIntrinsics(const cv::Mat & camera_matrix, const cv::Mat & distort_coeffs);
  bool hasIntrinsics() const { return !camera_matrix_.empty(); }

  void set_T_world_camera(const Eigen::Isometry3d & T_world_camera)
  {
    T_world_camera_ = T_world_camera;
  }
  const Eigen::Isometry3d & T_world_camera() const { return T_world_camera_; }

  // Forget the previous solution used to break the planar-PnP flip ambiguity.
  void resetContinuity();

  // When true (default) a solution whose outward normal points away from the
  // camera is rejected in favour of its mirror. Only valid for a plate that is
  // observed from the front, which is what this task does.
  void setPreferFrontal(bool value) { prefer_frontal_ = value; }

  // Plate dimensions must be set before solve().
  bool hasDimensions() const { return dims_.lightbar_length > 0.0 && dims_.small_armor_width > 0.0; }

  // Fills armor.xyz_in_world and armor.R_armor2world. Returns false if PnP
  // failed, produced no solution in front of the camera, or the intrinsics are
  // unset.
  bool solve(auto_aim::Armor & armor) const;

  // Reprojects a world-frame plate pose into the current image. Used by the
  // coordinate-chain self-check (plan §5.3b): feeding back the solved pose must
  // land within a couple of pixels of the detected corners, which is the only
  // reliable way to catch an optical-frame convention error.
  std::vector<cv::Point2f> reproject(
    const Eigen::Vector3d & xyz_in_world, const Eigen::Matrix3d & R_armor2world,
    auto_aim::ArmorType type) const;

  // Mean per-corner reprojection error of the accepted solution, pixels.
  double lastReprojectionError() const { return last_reprojection_error_; }

private:
  std::vector<cv::Point3f> objectPoints(auto_aim::ArmorType type) const;

  // Scores the two IPPE solutions. Lower is better.
  double score(
    const Eigen::Vector3d & p_world, const Eigen::Matrix3d & R_world, double reproj_error,
    bool have_hint, const Eigen::Vector3d & hint_p, const Eigen::Matrix3d & hint_R) const;

  ArmorDimensions dims_;
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  Eigen::Isometry3d T_world_camera_ = Eigen::Isometry3d::Identity();
  bool prefer_frontal_ = true;

  // Last accepted solution, in the world frame. Mutable because solve() is
  // const w.r.t. the pose chain but needs to remember what it picked. The node
  // calls resetContinuity() when tracking drops, so no time-based expiry is
  // needed here.
  mutable bool have_hint_ = false;
  mutable Eigen::Vector3d hint_p_ = Eigen::Vector3d::Zero();
  mutable Eigen::Matrix3d hint_R_ = Eigen::Matrix3d::Identity();

  mutable double last_reprojection_error_ = 0.0;
};

}  // namespace armor_perception

#endif  // ARMOR_PERCEPTION__ARMOR_SOLVER_HPP
