#include <gtest/gtest.h>

#include <cmath>
#include <opencv2/calib3d.hpp>

#include "armor_perception/armor_solver.hpp"
#include "armor_perception/frames.hpp"
#include "tasks/auto_aim/armor.hpp"

using armor_perception::ArmorSolver;
using armor_perception::buildTBaseOptical;
using armor_perception::opticalFromBody;

namespace
{
// Intrinsics of the Gazebo iris_vio camera: 848x480, HFOV 1.5009831567 rad.
constexpr double kFx = 455.4;
constexpr double kFy = 455.4;
constexpr double kCx = 424.0;
constexpr double kCy = 240.0;

cv::Mat makeK()
{
  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  K.at<double>(0, 0) = kFx;
  K.at<double>(1, 1) = kFy;
  K.at<double>(0, 2) = kCx;
  K.at<double>(1, 2) = kCy;
  return K;
}

// Projects the plate's four corners through a given world pose, the way the
// camera would see them. Returns the image points.
std::vector<cv::Point2f> projectCorners(
  const ArmorSolver & solver, const Eigen::Isometry3d & T_world_camera,
  const Eigen::Vector3d & p_world, const Eigen::Matrix3d & R_world)
{
  // Build a throwaway solver bound to the same camera pose so reproject() can
  // be used for the forward projection too.
  ArmorSolver forward = solver;
  forward.set_T_world_camera(T_world_camera);
  return forward.reproject(p_world, R_world, auto_aim::ArmorType::small);
}

// World pose of the plate, and a camera standoff metres along the plate normal
// looking straight back at it.
struct Scene
{
  Eigen::Vector3d p_armor;
  Eigen::Matrix3d R_armor_world;
  Eigen::Isometry3d T_world_camera;
};

// Armor frame: +X outward normal, +Y observer's right, +Z up. With the normal
// along world +X and up along world +Z the plate frame is the world frame, and
// `yaw` rotates the plate about the vertical.
Eigen::Matrix3d plateRotation(double yaw)
{
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

// Optical frame columns are [right, down, forward]. For a camera looking
// straight at the plate:
//   forward = -normal,  down = -up,  right = down x forward = +Y_armor
Eigen::Matrix3d opticalLookingAt(const Eigen::Matrix3d & R_armor_world)
{
  Eigen::Matrix3d R;
  R.col(0) = R_armor_world.col(1);
  R.col(1) = -R_armor_world.col(2);
  R.col(2) = -R_armor_world.col(0);
  return R;
}

// Camera placed `standoff` metres along the plate normal, optionally shifted
// sideways along the plate's own right axis.
Eigen::Isometry3d cameraAt(
  const Eigen::Vector3d & p_armor, const Eigen::Matrix3d & R_armor_world, double standoff,
  double lateral = 0.0)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = opticalLookingAt(R_armor_world);
  T.translation() =
    p_armor + R_armor_world.col(0) * standoff + R_armor_world.col(1) * lateral;
  return T;
}

Scene makeScene(double yaw, double standoff = 2.0)
{
  Scene s;
  s.p_armor = Eigen::Vector3d(5.0, 1.0, 1.2);
  s.R_armor_world = plateRotation(yaw);
  s.T_world_camera = cameraAt(s.p_armor, s.R_armor_world, standoff);
  return s;
}
}  // namespace

// The optical-frame rotation is the single most error-prone step: if it is
// wrong, detection still looks perfect and every pose is rotated. Pin it down.
TEST(Frames, OpticalFromBodyMapsAxesCorrectly)
{
  const Eigen::Matrix3d R = opticalFromBody();

  // Body forward (+X) -> optical forward (+Z)
  EXPECT_TRUE((R * Eigen::Vector3d::UnitX()).isApprox(Eigen::Vector3d::UnitZ(), 1e-12));
  // Body left (+Y) -> optical left (-X)
  EXPECT_TRUE((R * Eigen::Vector3d::UnitY()).isApprox(-Eigen::Vector3d::UnitX(), 1e-12));
  // Body up (+Z) -> optical up (-Y)
  EXPECT_TRUE((R * Eigen::Vector3d::UnitZ()).isApprox(-Eigen::Vector3d::UnitY(), 1e-12));

  EXPECT_NEAR(R.determinant(), 1.0, 1e-12);
  EXPECT_TRUE((R.transpose() * R).isApprox(Eigen::Matrix3d::Identity(), 1e-12));
}

TEST(Frames, BaseOpticalPutsOpticalAxesInBodyFrame)
{
  const Eigen::Vector3d p(0.1, 0.0, 0.0);
  const Eigen::Isometry3d T = buildTBaseOptical(p, Eigen::Matrix3d::Identity());

  EXPECT_TRUE(T.translation().isApprox(p, 1e-12));

  // Columns of T.linear() are the optical axes expressed in body coordinates.
  EXPECT_TRUE(T.linear().col(0).isApprox(Eigen::Vector3d(0, -1, 0), 1e-12));  // right = -left
  EXPECT_TRUE(T.linear().col(1).isApprox(Eigen::Vector3d(0, 0, -1), 1e-12));  // down = -up
  EXPECT_TRUE(T.linear().col(2).isApprox(Eigen::Vector3d(1, 0, 0), 1e-12));   // forward
}

TEST(ArmorSolver, RefusesToSolveWithoutIntrinsics)
{
  ArmorSolver solver;
  auto_aim::Armor armor;
  armor.type = auto_aim::ArmorType::small;
  armor.points = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
  EXPECT_FALSE(solver.solve(armor));
}

TEST(ArmorSolver, RejectsDegenerateIntrinsics)
{
  ArmorSolver solver;
  cv::Mat K = cv::Mat::zeros(3, 3, CV_64F);
  EXPECT_FALSE(solver.setCameraIntrinsics(K, cv::Mat::zeros(1, 5, CV_64F)));
  EXPECT_FALSE(solver.hasIntrinsics());
}

TEST(ArmorSolver, AcceptsValidIntrinsics)
{
  ArmorSolver solver;
  EXPECT_TRUE(solver.setCameraIntrinsics(makeK(), cv::Mat::zeros(1, 5, CV_64F)));
  EXPECT_TRUE(solver.hasIntrinsics());
}

// The coordinate-chain self-check from the plan (§5.3b), done analytically: a
// known world pose is projected, then solved back. Position and orientation must
// return, and the reprojection error must be near zero.
TEST(ArmorSolver, RecoversKnownPose)
{
  ArmorSolver solver;
  ASSERT_TRUE(solver.setCameraIntrinsics(makeK(), cv::Mat::zeros(1, 5, CV_64F)));

  const Scene scene = makeScene(0.0);

  auto_aim::Armor armor;
  armor.type = auto_aim::ArmorType::small;
  armor.points = projectCorners(solver, scene.T_world_camera, scene.p_armor, scene.R_armor_world);

  solver.set_T_world_camera(scene.T_world_camera);
  ASSERT_TRUE(solver.solve(armor));

  EXPECT_LT((armor.xyz_in_world - scene.p_armor).norm(), 1e-3);
  EXPECT_LT(solver.lastReprojectionError(), 0.1);
}

// Ego-motion: the same plate seen from two different camera positions must land
// at the same world point. This is the whole reason the solver takes
// T_world_camera instead of a bare rotation.
TEST(ArmorSolver, SamePlateFromDifferentCameraPosesGivesSameWorldPoint)
{
  ArmorSolver solver;
  ASSERT_TRUE(solver.setCameraIntrinsics(makeK(), cv::Mat::zeros(1, 5, CV_64F)));

  const Eigen::Vector3d truth(6.0, -2.0, 1.0);
  const Eigen::Matrix3d R_truth = plateRotation(0.3);

  // Three camera positions: straight on, and 0.5 m to either side along the
  // plate's right axis. A correct world-frame chain puts the plate in the same
  // place from all three.
  for (double lateral : {-0.5, 0.0, 0.5}) {
    const Eigen::Isometry3d T_world_camera = cameraAt(truth, R_truth, 2.0, lateral);

    auto_aim::Armor armor;
    armor.type = auto_aim::ArmorType::small;
    armor.points = projectCorners(solver, T_world_camera, truth, R_truth);

    solver.set_T_world_camera(T_world_camera);
    solver.resetContinuity();
    ASSERT_TRUE(solver.solve(armor)) << "lateral offset " << lateral;

    // 2 m standoff, sub-pixel projection: expect millimetre agreement.
    EXPECT_LT((armor.xyz_in_world - truth).norm(), 0.01) << "lateral offset " << lateral;
  }
}

// The solved orientation's +X axis must be the plate normal, pointing at the
// camera. This is what the observation point is built on.
TEST(ArmorSolver, SolvedNormalPointsAtCamera)
{
  ArmorSolver solver;
  ASSERT_TRUE(solver.setCameraIntrinsics(makeK(), cv::Mat::zeros(1, 5, CV_64F)));

  const Scene scene = makeScene(0.2);

  auto_aim::Armor armor;
  armor.type = auto_aim::ArmorType::small;
  armor.points = projectCorners(solver, scene.T_world_camera, scene.p_armor, scene.R_armor_world);

  solver.set_T_world_camera(scene.T_world_camera);
  ASSERT_TRUE(solver.solve(armor));

  const Eigen::Vector3d normal = armor.R_armor2world.col(0);
  const Eigen::Vector3d to_camera = scene.T_world_camera.translation() - armor.xyz_in_world;
  EXPECT_GT(normal.dot(to_camera.normalized()), 0.9);
}

// Reprojection must be the exact inverse of the solve. If the two disagree, one
// of them has the frame convention wrong.
TEST(ArmorSolver, ReprojectionRoundTrips)
{
  ArmorSolver solver;
  ASSERT_TRUE(solver.setCameraIntrinsics(makeK(), cv::Mat::zeros(1, 5, CV_64F)));

  const Scene scene = makeScene(-0.15);
  solver.set_T_world_camera(scene.T_world_camera);

  const auto corners =
    solver.reproject(scene.p_armor, scene.R_armor_world, auto_aim::ArmorType::small);
  ASSERT_EQ(corners.size(), 4u);

  auto_aim::Armor armor;
  armor.type = auto_aim::ArmorType::small;
  armor.points = corners;
  ASSERT_TRUE(solver.solve(armor));

  EXPECT_LT((armor.xyz_in_world - scene.p_armor).norm(), 1e-3);
}
