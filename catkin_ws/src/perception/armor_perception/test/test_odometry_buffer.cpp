#include <gtest/gtest.h>

#include <nav_msgs/Odometry.h>

#include <Eigen/Geometry>

#include "armor_perception/odometry_buffer.hpp"

using armor_perception::OdometryBuffer;

namespace
{
nav_msgs::Odometry makeOdom(double t, double x, double y, double z, double yaw = 0.0)
{
  nav_msgs::Odometry o;
  o.header.stamp.fromSec(t);
  o.header.frame_id = "world";
  o.pose.pose.position.x = x;
  o.pose.pose.position.y = y;
  o.pose.pose.position.z = z;
  const Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  o.pose.pose.orientation.w = q.w();
  o.pose.pose.orientation.x = q.x();
  o.pose.pose.orientation.y = q.y();
  o.pose.pose.orientation.z = q.z();
  return o;
}
}  // namespace

TEST(OdometryBuffer, EmptyBufferFails)
{
  OdometryBuffer buffer;
  Eigen::Isometry3d T;
  std::string error;
  EXPECT_FALSE(buffer.lookup(1.0, T, error));
  EXPECT_FALSE(error.empty());
}

TEST(OdometryBuffer, RejectsNonPositiveStamp)
{
  OdometryBuffer buffer;
  nav_msgs::Odometry o;
  o.pose.pose.orientation.w = 1.0;
  EXPECT_FALSE(buffer.push(o));  // stamp 0
  EXPECT_TRUE(buffer.empty());
}

TEST(OdometryBuffer, RejectsZeroQuaternion)
{
  OdometryBuffer buffer;
  nav_msgs::Odometry o = makeOdom(1.0, 0, 0, 0);
  o.pose.pose.orientation.w = 0.0;
  EXPECT_FALSE(buffer.push(o));
}

// The core contract: a pose at an intermediate timestamp is interpolated, not
// taken from the nearest sample. Without this, ego-motion compensation leaves a
// residual of v * dt that grows with flight speed.
TEST(OdometryBuffer, InterpolatesPositionLinearly)
{
  OdometryBuffer buffer;
  buffer.setMaxAge(0.05);

  ASSERT_TRUE(buffer.push(makeOdom(1.000, 0.0, 0.0, 1.0)));
  ASSERT_TRUE(buffer.push(makeOdom(1.100, 1.0, 0.0, 1.0)));

  Eigen::Isometry3d T;
  std::string error;
  ASSERT_TRUE(buffer.lookup(1.025, T, error)) << error;

  EXPECT_NEAR(T.translation().x(), 0.25, 1e-9);
  EXPECT_NEAR(T.translation().z(), 1.0, 1e-9);
}

TEST(OdometryBuffer, InterpolatesOrientationBySlerp)
{
  OdometryBuffer buffer;
  buffer.setMaxAge(0.05);

  ASSERT_TRUE(buffer.push(makeOdom(1.0, 0, 0, 0, 0.0)));
  ASSERT_TRUE(buffer.push(makeOdom(1.1, 0, 0, 0, M_PI / 2.0)));

  Eigen::Isometry3d T;
  std::string error;
  ASSERT_TRUE(buffer.lookup(1.05, T, error)) << error;

  const Eigen::Vector3d x_axis = T.linear().col(0);
  EXPECT_NEAR(x_axis.x(), std::cos(M_PI / 4.0), 1e-6);
  EXPECT_NEAR(x_axis.y(), std::sin(M_PI / 4.0), 1e-6);
}

// An image older than the newest odometry by more than maxAge must be dropped:
// extrapolating is worse than skipping a frame.
TEST(OdometryBuffer, RejectsImageTooFarAheadOfOdometry)
{
  OdometryBuffer buffer;
  buffer.setMaxAge(0.05);

  ASSERT_TRUE(buffer.push(makeOdom(1.0, 0, 0, 0)));
  ASSERT_TRUE(buffer.push(makeOdom(1.1, 0, 0, 0)));

  Eigen::Isometry3d T;
  std::string error;
  EXPECT_FALSE(buffer.lookup(1.2, T, error));
  EXPECT_FALSE(error.empty());
}

// Within maxAge the newest sample is used as-is; that bound is the whole point
// of the tolerance, so it must not also be treated as an error.
TEST(OdometryBuffer, HoldsNewestWithinTolerance)
{
  OdometryBuffer buffer;
  buffer.setMaxAge(0.05);

  ASSERT_TRUE(buffer.push(makeOdom(1.0, 0, 0, 0)));
  ASSERT_TRUE(buffer.push(makeOdom(1.1, 2.0, 0, 0)));

  Eigen::Isometry3d T;
  std::string error;
  ASSERT_TRUE(buffer.lookup(1.12, T, error)) << error;
  EXPECT_NEAR(T.translation().x(), 2.0, 1e-9);
}

TEST(OdometryBuffer, RejectsImageOlderThanBuffer)
{
  OdometryBuffer buffer;
  buffer.setMaxAge(0.05);
  buffer.setCapacity(3);

  for (int i = 0; i < 10; ++i) {
    ASSERT_TRUE(buffer.push(makeOdom(1.0 + 0.01 * i, i, 0, 0)));
  }

  Eigen::Isometry3d T;
  std::string error;
  EXPECT_FALSE(buffer.lookup(0.5, T, error));
}

TEST(OdometryBuffer, CapacityIsBounded)
{
  OdometryBuffer buffer;
  buffer.setCapacity(5);
  for (int i = 0; i < 20; ++i) {
    ASSERT_TRUE(buffer.push(makeOdom(1.0 + 0.01 * i, i, 0, 0)));
  }
  EXPECT_LE(buffer.size(), 5u);
}

TEST(OdometryBuffer, OutOfOrderSamplesAreSorted)
{
  OdometryBuffer buffer;
  buffer.setMaxAge(0.05);

  ASSERT_TRUE(buffer.push(makeOdom(1.0, 0.0, 0, 0)));
  ASSERT_TRUE(buffer.push(makeOdom(1.2, 2.0, 0, 0)));
  ASSERT_TRUE(buffer.push(makeOdom(1.1, 1.0, 0, 0)));  // late arrival

  Eigen::Isometry3d T;
  std::string error;
  ASSERT_TRUE(buffer.lookup(1.1, T, error)) << error;
  EXPECT_NEAR(T.translation().x(), 1.0, 1e-9);
}

TEST(OdometryBuffer, FrameIdFollowsSource)
{
  OdometryBuffer buffer;
  nav_msgs::Odometry o = makeOdom(1.0, 0, 0, 0);
  o.header.frame_id = "map";
  ASSERT_TRUE(buffer.push(o));
  EXPECT_EQ(buffer.frameId(), "map");
}
