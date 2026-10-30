#include <gtest/gtest.h>

#include <cmath>

#include "armor_perception/static_target.hpp"

using armor_perception::StaticTarget;

namespace
{
const Eigen::Vector3d kNoise = Eigen::Vector3d::Constant(0.01);
}

TEST(StaticTarget, InitializesFromFirstMeasurement)
{
  StaticTarget target;
  const Eigen::Vector3d p(1.0, 2.0, 3.0);
  target.initialize(p, 0.5, kNoise, 10.0);

  ASSERT_TRUE(target.initialized());
  EXPECT_TRUE(target.position().isApprox(p, 1e-9));
  EXPECT_NEAR(target.yaw(), 0.5, 1e-9);
  EXPECT_FALSE(target.diverged());
}

// A static plate measured repeatedly must not drift. This is the property the
// 11-state ring model could not provide: with r unobservable, the estimate
// wandered and diverged() fired.
TEST(StaticTarget, StaticTargetStaysPut)
{
  StaticTarget target;
  StaticTarget::Params params;
  params.accel_noise = 0.05;
  target.setParams(params);

  const Eigen::Vector3d truth(3.0, -1.0, 1.5);
  target.initialize(truth, 0.0, kNoise, 0.0);

  double t = 0.0;
  for (int i = 0; i < 200; ++i) {
    t += 1.0 / 30.0;
    // A little sensor noise, as a real detector would produce.
    const double dither = (i % 2 == 0) ? 0.004 : -0.004;
    ASSERT_TRUE(target.update(truth + Eigen::Vector3d(dither, 0, 0), 0.0, kNoise, t));
  }

  EXPECT_LT((target.position() - truth).norm(), 0.02);
  EXPECT_LT(target.positionStd(), 0.05);
  EXPECT_FALSE(target.diverged());
  EXPECT_EQ(target.missedFrames(), 0);
}

// Velocity states must stay near zero for a static plate. If they do not, the
// filter is chasing noise and will overshoot when the estimate is used.
TEST(StaticTarget, VelocityStaysNearZeroForStaticTarget)
{
  StaticTarget target;
  const Eigen::Vector3d truth(2.0, 2.0, 1.0);
  target.initialize(truth, 0.0, kNoise, 0.0);

  double t = 0.0;
  for (int i = 0; i < 150; ++i) {
    t += 1.0 / 30.0;
    ASSERT_TRUE(target.update(truth, 0.0, kNoise, t));
  }

  EXPECT_LT(target.velocity().norm(), 0.1);
}

TEST(StaticTarget, PredictGrowsUncertainty)
{
  StaticTarget target;
  target.initialize(Eigen::Vector3d::Zero(), 0.0, kNoise, 0.0);
  const double before = target.positionStd();

  target.predict(0.5);

  EXPECT_GT(target.positionStd(), before);
  EXPECT_EQ(target.missedFrames(), 1);
}

// A single wild measurement is an association error or a PnP flip, not target
// motion. It must be rejected rather than dragging the estimate across the room.
TEST(StaticTarget, RejectsOutlierMeasurement)
{
  StaticTarget target;
  const Eigen::Vector3d truth(1.0, 1.0, 1.0);
  target.initialize(truth, 0.0, kNoise, 0.0);

  double t = 0.0;
  for (int i = 0; i < 10; ++i) {
    t += 1.0 / 30.0;
    ASSERT_TRUE(target.update(truth, 0.0, kNoise, t));
  }

  const Eigen::Vector3d before = target.position();
  EXPECT_FALSE(target.update(Eigen::Vector3d(5.0, 5.0, 5.0), 0.0, kNoise, t + 1.0 / 30.0));
  EXPECT_LT((target.position() - before).norm(), 0.05);
  EXPECT_FALSE(target.diverged());
}

// Persistent rejection means the track and the detections disagree; the filter
// must say so instead of silently publishing a stale pose.
TEST(StaticTarget, DivergesAfterRepeatedJumps)
{
  StaticTarget target;
  StaticTarget::Params params;
  params.max_consecutive_jumps = 3;
  params.jump_threshold = 0.5;
  target.setParams(params);

  const Eigen::Vector3d truth(1.0, 1.0, 1.0);
  target.initialize(truth, 0.0, kNoise, 0.0);

  double t = 0.0;
  for (int i = 0; i < 20; ++i) {
    t += 1.0 / 30.0;
    target.update(truth, 0.0, kNoise, t);
  }

  for (int i = 0; i < 3; ++i) {
    t += 1.0 / 30.0;
    EXPECT_FALSE(target.update(Eigen::Vector3d(9.0, 9.0, 9.0), 0.0, kNoise, t));
  }

  EXPECT_TRUE(target.diverged());
}

TEST(StaticTarget, YawIsLowPassedNotJumped)
{
  StaticTarget target;
  StaticTarget::Params params;
  params.yaw_time_constant = 0.2;
  target.setParams(params);

  target.initialize(Eigen::Vector3d::Zero(), 0.0, kNoise, 0.0);

  // One frame at 30 Hz: the estimate must move towards the measurement but not
  // jump all the way to it.
  ASSERT_TRUE(target.update(Eigen::Vector3d::Zero(), 1.0, kNoise, 1.0 / 30.0));
  EXPECT_GT(target.yaw(), 0.0);
  EXPECT_LT(target.yaw(), 1.0);
}

TEST(StaticTarget, YawWrapsAcrossPi)
{
  StaticTarget target;
  StaticTarget::Params params;
  params.yaw_time_constant = 0.05;
  target.setParams(params);

  target.initialize(Eigen::Vector3d::Zero(), M_PI - 0.1, kNoise, 0.0);

  // Measurements either side of the branch cut must not produce a full turn.
  double t = 0.0;
  for (int i = 0; i < 60; ++i) {
    t += 1.0 / 30.0;
    target.update(Eigen::Vector3d::Zero(), -M_PI + 0.1, kNoise, t);
  }

  const double yaw = target.yaw();
  EXPECT_TRUE(std::abs(std::abs(yaw) - M_PI) < 0.2) << "yaw = " << yaw;
}
