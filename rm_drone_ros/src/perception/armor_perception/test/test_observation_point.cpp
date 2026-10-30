#include <gtest/gtest.h>

#include <cmath>

#include "armor_perception/observation_point.hpp"

using armor_perception::ObservationPointGenerator;

namespace
{
// Armor frame: +X outward normal, +Y to the observer's right, +Z up. With the
// normal along world +X and up along world +Z, the plate frame *is* the world
// frame; `yaw` then rotates the plate about the vertical.
Eigen::Matrix3d plateRotation(double yaw)
{
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}
}  // namespace

TEST(ObservationPoint, PlateFrameNormalIsX)
{
  // Guard the convention the other tests (and ArmorSolver) depend on.
  const Eigen::Matrix3d R = plateRotation(0.0);
  EXPECT_TRUE(R.col(0).isApprox(Eigen::Vector3d::UnitX(), 1e-12));
  EXPECT_TRUE(R.col(1).isApprox(Eigen::Vector3d::UnitY(), 1e-12));
  EXPECT_TRUE(R.col(2).isApprox(Eigen::Vector3d::UnitZ(), 1e-12));
}

TEST(ObservationPoint, SitsAlongNormalWhenTrusted)
{
  ObservationPointGenerator gen;
  const Eigen::Vector3d p_armor(5.0, 2.0, 1.5);
  const Eigen::Matrix3d R = plateRotation(0.0);
  const Eigen::Vector3d p_camera = p_armor + Eigen::Vector3d(4.0, 0, 0);

  const auto r = gen.generate(p_armor, R, p_camera, true);

  EXPECT_TRUE(r.normal_used);
  EXPECT_NEAR(r.standoff, 1.5, 1e-9);
  EXPECT_TRUE(r.position.isApprox(p_armor + Eigen::Vector3d(1.5, 0, 0), 1e-9));
}

// If the drone is already behind the plate, the point must flip to the near
// side rather than send it through the wall.
TEST(ObservationPoint, PicksTheSideTheCameraIsOn)
{
  ObservationPointGenerator gen;
  const Eigen::Vector3d p_armor(5.0, 2.0, 1.5);
  const Eigen::Matrix3d R = plateRotation(0.0);
  const Eigen::Vector3d p_camera = p_armor + Eigen::Vector3d(-4.0, 0, 0);

  const auto r = gen.generate(p_armor, R, p_camera, true);

  EXPECT_TRUE(r.normal_used);
  EXPECT_TRUE(r.position.isApprox(p_armor + Eigen::Vector3d(-1.5, 0, 0), 1e-9));
}

// With the normal untrusted the point degrades to "back off along the current
// bearing": always defined, and no sudden lateral jump.
TEST(ObservationPoint, FallsBackToLineOfSight)
{
  ObservationPointGenerator gen;
  const Eigen::Vector3d p_armor(0.0, 0.0, 0.0);
  const Eigen::Matrix3d R = plateRotation(0.0);
  const Eigen::Vector3d p_camera(0.0, 3.0, 0.0);  // off to the side

  const auto r = gen.generate(p_armor, R, p_camera, false);

  EXPECT_FALSE(r.normal_used);
  EXPECT_TRUE(r.position.isApprox(Eigen::Vector3d(0.0, 1.5, 0.0), 1e-9));
}

// A bad normal must never produce a collision course: the standoff is clamped.
TEST(ObservationPoint, ClampsStandoffToSafetyLimits)
{
  ObservationPointGenerator gen;
  ObservationPointGenerator::Params params;
  params.standoff = 0.1;  // far below the floor
  gen.setParams(params);

  const Eigen::Vector3d p_armor(0.0, 0.0, 0.0);
  const Eigen::Vector3d p_camera(3.0, 0.0, 0.0);
  const auto r = gen.generate(p_armor, plateRotation(0.0), p_camera, true);

  EXPECT_NEAR(r.standoff, params.min_standoff, 1e-9);
  EXPECT_GE((r.position - p_armor).norm(), params.min_standoff - 1e-9);

  params.standoff = 99.0;
  gen.setParams(params);
  const auto r2 = gen.generate(p_armor, plateRotation(0.0), p_camera, true);
  EXPECT_NEAR(r2.standoff, params.max_standoff, 1e-9);
}

TEST(ObservationPoint, HeadingFacesThePlate)
{
  ObservationPointGenerator gen;
  const Eigen::Vector3d p_armor(5.0, 2.0, 1.5);
  const Eigen::Vector3d p_camera = p_armor + Eigen::Vector3d(4.0, 0, 0);

  const auto r = gen.generate(p_armor, plateRotation(0.0), p_camera, true);

  // Body +X must point from the observation point towards the plate, i.e. -X.
  EXPECT_NEAR(std::abs(r.yaw), M_PI, 1e-6);
}

// The point must be continuous in the normal: a small rotation of the plate
// cannot teleport the observation point across the field.
TEST(ObservationPoint, IsContinuousInPlateYaw)
{
  ObservationPointGenerator gen;
  const Eigen::Vector3d p_armor(5.0, 2.0, 1.5);
  const Eigen::Vector3d p_camera = p_armor + Eigen::Vector3d(4.0, 0, 0);

  Eigen::Vector3d previous;
  for (int i = 0; i <= 20; ++i) {
    const double yaw = -0.2 + 0.02 * i;
    const auto r = gen.generate(p_armor, plateRotation(yaw), p_camera, true);
    if (i > 0) {
      EXPECT_LT((r.position - previous).norm(), 0.1) << "at step " << i;
    }
    previous = r.position;
  }
}
