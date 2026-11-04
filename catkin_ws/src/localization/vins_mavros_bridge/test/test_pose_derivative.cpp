#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "vins_mavros_bridge/pose_derivative.h"

using vins_mavros_bridge::PoseDerivative;

namespace
{

Eigen::Matrix3d rotZ(double angle)
{
  return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

Eigen::Matrix3d rotX(double angle)
{
  return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()).toRotationMatrix();
}

PoseDerivative makeDerivative(double window = 0.1, double min_dt = 0.02)
{
  PoseDerivative derivative;
  PoseDerivative::Params params;
  params.window = window;
  params.min_dt = min_dt;
  derivative.setParams(params);
  return derivative;
}

}  // namespace

TEST(PoseDerivative, ReportsNothingUntilTheWindowHasABaseline)
{
  // 50 ms is well above the 33 ms frame interval, so two consecutive samples
  // are not enough and the third is.
  PoseDerivative derivative = makeDerivative(0.1, 0.05);

  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
  EXPECT_FALSE(derivative.update(0.0, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                                 &linear, &angular));
  EXPECT_FALSE(derivative.update(0.033, Eigen::Vector3d::UnitX() * 0.033,
                                 Eigen::Quaterniond::Identity(), &linear, &angular));
  EXPECT_FALSE(derivative.hasEstimate());

  EXPECT_TRUE(derivative.update(0.066, Eigen::Vector3d::UnitX() * 0.066,
                                Eigen::Quaterniond::Identity(), &linear, &angular));
  EXPECT_TRUE(derivative.hasEstimate());
}

TEST(PoseDerivative, RecoversAConstantVelocity)
{
  PoseDerivative derivative = makeDerivative();
  const Eigen::Vector3d velocity(1.0, -0.5, 0.25);
  const Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
  bool ok = false;
  for (int i = 0; i < 20; ++i)
  {
    const double t = 0.033 * i;
    ok = derivative.update(t, velocity * t, orientation, &linear, &angular);
  }

  ASSERT_TRUE(ok);
  EXPECT_TRUE(linear.isApprox(velocity, 1e-9));
  EXPECT_NEAR(angular.norm(), 0.0, 1e-9);
}

TEST(PoseDerivative, StationaryInputIsExactlyZero)
{
  PoseDerivative derivative = makeDerivative();

  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
  bool ok = false;
  for (int i = 0; i < 10; ++i)
  {
    ok = derivative.update(0.033 * i, Eigen::Vector3d(2.0, 3.0, 1.0),
                           Eigen::Quaterniond::Identity(), &linear, &angular);
  }

  ASSERT_TRUE(ok);
  EXPECT_LT(linear.norm(), 1e-12);
  EXPECT_LT(angular.norm(), 1e-12);
}

// The window is the whole point: a two-point difference over one frame interval
// amplifies this noise by 1/dt, the least-squares slope averages it down.
TEST(PoseDerivative, LeastSquaresSlopeBeatsATwoPointDifferenceUnderNoise)
{
  PoseDerivative derivative = makeDerivative(0.1, 0.02);
  const Eigen::Vector3d velocity(0.5, 0.0, 0.0);
  const double dt = 0.033;
  const int count = 40;

  auto noisyPosition = [&](int i) {
    const double noise = (i % 2 == 0) ? 0.005 : -0.005;
    return velocity * (dt * i) + Eigen::Vector3d(noise, 0.0, 0.0);
  };

  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
  bool ok = false;
  for (int i = 0; i < count; ++i)
  {
    ok = derivative.update(dt * i, noisyPosition(i), Eigen::Quaterniond::Identity(), &linear,
                           &angular);
  }

  ASSERT_TRUE(ok);

  // Alternating noise is the worst case for a slope fit: it is not zero-mean
  // over a four-sample window, so this is a bound, not an equality.
  const double windowed_error = std::abs(linear.x() - velocity.x());
  EXPECT_LT(windowed_error, 0.1);

  // The single-interval estimate the naive implementation would have produced.
  const Eigen::Vector3d difference =
      (noisyPosition(count - 1) - noisyPosition(count - 2)) / dt;
  EXPECT_GT(std::abs(difference.x() - velocity.x()), 0.2);
  EXPECT_LT(windowed_error, std::abs(difference.x() - velocity.x()));
}

TEST(PoseDerivative, AngularRateIsExpressedInTheBodyFrame)
{
  // World orientation R(t) = Rz(w t) * Rx(90 deg). The spin is about the world
  // Z axis, but the body sees it about its own Y axis -- this is what
  // distinguishes a body-frame angular rate from a world-frame one.
  const double rate = 0.5;
  const Eigen::Matrix3d tilt = rotX(M_PI / 2.0);

  PoseDerivative derivative = makeDerivative(0.1, 0.02);
  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
  bool ok = false;
  for (int i = 0; i < 40; ++i)
  {
    const double t = 0.033 * i;
    Eigen::Quaterniond orientation(rotZ(rate * t) * tilt);
    ok = derivative.update(t, Eigen::Vector3d::Zero(), orientation, &linear, &angular);
  }

  ASSERT_TRUE(ok);
  EXPECT_NEAR(angular.x(), 0.0, 1e-6);
  EXPECT_NEAR(angular.y(), rate, 1e-6);
  EXPECT_NEAR(angular.z(), 0.0, 1e-6);
}

TEST(PoseDerivative, BackwardsTimestampClearsTheWindow)
{
  PoseDerivative derivative = makeDerivative();
  Eigen::Vector3d linear;
  Eigen::Vector3d angular;

  for (int i = 0; i < 10; ++i)
  {
    derivative.update(0.033 * i, Eigen::Vector3d::UnitX() * (0.033 * i),
                      Eigen::Quaterniond::Identity(), &linear, &angular);
  }
  ASSERT_EQ(derivative.resets(), 0ULL);

  // An estimator restart: same window would span two unrelated trajectories.
  EXPECT_FALSE(derivative.update(0.0, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                                 &linear, &angular));
  EXPECT_EQ(derivative.resets(), 1ULL);
  EXPECT_FALSE(derivative.hasEstimate());
  EXPECT_EQ(derivative.size(), 1U);
}

TEST(PoseDerivative, EmptyWindowIsNeverUsedForDifferentiation)
{
  PoseDerivative derivative = makeDerivative(0.1, 0.02);
  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
  EXPECT_FALSE(derivative.update(1.0, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                                 &linear, &angular));
  // Same stamp twice: zero baseline, must not divide by zero.
  EXPECT_FALSE(derivative.update(1.0, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                                 &linear, &angular));
}

TEST(PoseDerivative, RejectsNonFiniteInput)
{
  PoseDerivative derivative = makeDerivative();
  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
  Eigen::Vector3d bad;
  bad << std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0;
  EXPECT_FALSE(derivative.update(0.0, bad, Eigen::Quaterniond::Identity(), &linear, &angular));
  EXPECT_EQ(derivative.size(), 0U);
}
