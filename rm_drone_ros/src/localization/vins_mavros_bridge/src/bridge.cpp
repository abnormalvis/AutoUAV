#include "vins_mavros_bridge/bridge.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <vector>

#include <XmlRpcValue.h>

namespace vins_mavros_bridge
{
namespace
{
constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kDegToRad = M_PI / 180.0;

/// Beyond this the input stamp cannot be a latency measurement: the VIO and the
/// bridge are simply using different clocks. Reported once as a configuration
/// problem instead of looking like permanent staleness.
constexpr double kClockSkewBound = 3600.0;

std::vector<double> readFixedArray(const ros::NodeHandle& nh, const std::string& name,
                                   const std::vector<double>& fallback)
{
  XmlRpc::XmlRpcValue value;
  if (!nh.getParam(name, value) || value.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      static_cast<size_t>(value.size()) != fallback.size())
  {
    return fallback;
  }
  std::vector<double> out(fallback.size());
  for (size_t i = 0; i < out.size(); ++i)
  {
    out[i] = static_cast<double>(value[static_cast<int>(i)]);
  }
  return out;
}

}  // namespace

Bridge::Bridge() : private_nh_("~")
{
  loadParameters();
  advertise();

  if (input_mode_ == InputMode::kOdometry)
  {
    input_sub_ = nh_.subscribe(input_topic_, 50, &Bridge::odometryCallback, this);
  }
  else
  {
    input_sub_ = nh_.subscribe(input_topic_, 50, &Bridge::poseStampedCallback, this);
  }

  if (consistency_check_)
  {
    px4_odom_sub_ = nh_.subscribe(px4_odom_topic_, 10, &Bridge::localPositionCallback, this);
  }

  watchdog_timer_ = nh_.createTimer(ros::Duration(0.02), &Bridge::onWatchdog, this);
  status_timer_ = nh_.createTimer(ros::Duration(1.0), &Bridge::onStatus, this);
  rate_window_start_ = ros::Time::now();

  const Eigen::Vector3d extrinsic_rpy = rpyFromRotation(base_to_input_.linear());
  ROS_INFO("vins_mavros_bridge: %s (%s) -> %s", input_topic_.c_str(),
           input_mode_ == InputMode::kOdometry ? "nav_msgs/Odometry" : "PoseStamped",
           output_topic_.c_str());
  ROS_INFO("vins_mavros_bridge: input child frame '%s' -> '%s', twist normalized from '%s' to "
           "world",
           input_child_frame_.c_str(), body_frame_.c_str(),
           twist_frame_ == TwistFrame::kBody ? "body" : "world");
  ROS_INFO("vins_mavros_bridge: T_base_input xyz=[%.4f %.4f %.4f] rpy_deg=[%.4f %.4f %.4f]",
           base_to_input_.translation().x(), base_to_input_.translation().y(),
           base_to_input_.translation().z(), extrinsic_rpy.x() * kRadToDeg,
           extrinsic_rpy.y() * kRadToDeg, extrinsic_rpy.z() * kRadToDeg);

  if (publish_vision_pose_)
  {
    ROS_INFO("vins_mavros_bridge: MAVROS external vision via %s", vision_pose_topic_.c_str());
  }
  else if (publish_mavros_odometry_)
  {
    ROS_INFO("vins_mavros_bridge: MAVROS external vision via %s",
             mavros_odometry_topic_.c_str());
  }
  else
  {
    ROS_WARN("vins_mavros_bridge: no MAVROS external vision output enabled; PX4 will not see "
             "this estimate");
  }

  if (twist_frame_ == TwistFrame::kBody && !input_child_frame_.empty())
  {
    ROS_INFO("vins_mavros_bridge: body-frame twist is rotated using the '%s' orientation",
             input_child_frame_.c_str());
  }
}

void Bridge::loadParameters()
{
  std::string input_mode = "odometry";
  private_nh_.param<std::string>("input_mode", input_mode, std::string("odometry"));
  if (input_mode == "pose_stamped")
  {
    input_mode_ = InputMode::kPoseStamped;
  }
  else if (input_mode == "odometry")
  {
    input_mode_ = InputMode::kOdometry;
  }
  else
  {
    fatal("input_mode must be 'odometry' or 'pose_stamped', got '" + input_mode + "'");
  }

  private_nh_.param<std::string>("input_topic", input_topic_, std::string("/ov_msckf/odomimu"));
  private_nh_.param<std::string>("input_child_frame", input_child_frame_, std::string("imu"));
  private_nh_.param<std::string>("world_frame", world_frame_, std::string("world"));
  private_nh_.param<std::string>("body_frame", body_frame_, std::string("base_link"));

  std::string twist_frame = "body";
  private_nh_.param<std::string>("input_twist_frame", twist_frame, std::string("body"));
  if (twist_frame == "body")
  {
    twist_frame_ = TwistFrame::kBody;
  }
  else if (twist_frame == "world")
  {
    twist_frame_ = TwistFrame::kWorld;
  }
  else
  {
    fatal("input_twist_frame must be 'body' or 'world', got '" + twist_frame + "'");
  }

  private_nh_.param("input_twist_angular", input_twist_angular_, true);

  std::string jump_action = "reject";
  private_nh_.param<std::string>("jump_action", jump_action, std::string("reject"));
  if (jump_action == "reject")
  {
    jump_action_ = JumpAction::kReject;
  }
  else if (jump_action == "reset_origin")
  {
    jump_action_ = JumpAction::kResetOrigin;
  }
  else
  {
    fatal("jump_action must be 'reject' or 'reset_origin', got '" + jump_action + "'");
  }

  private_nh_.param<std::string>("odometry_topic", output_topic_,
                                 std::string("/localization/odometry"));
  private_nh_.param<std::string>("vision_pose_topic", vision_pose_topic_,
                                 std::string("/mavros/vision_pose/pose"));
  private_nh_.param<std::string>("mavros_odometry_topic", mavros_odometry_topic_,
                                 std::string("/mavros/odometry/in"));
  private_nh_.param<std::string>("status_topic", status_topic_,
                                 std::string("/vins_mavros_bridge/status"));
  private_nh_.param<std::string>("path_topic", path_topic_,
                                 std::string("/vins_mavros_bridge/path"));
  private_nh_.param<std::string>("valid_topic", valid_topic_,
                                 std::string("/vins_mavros_bridge/valid"));

  private_nh_.param("publish_vision_pose", publish_vision_pose_, true);
  private_nh_.param("publish_mavros_odometry", publish_mavros_odometry_, false);
  private_nh_.param("publish_tf", publish_tf_, false);

  // Two external-vision feeds into one EKF2 double-count the measurement. This
  // is a configuration error, not something to resolve at runtime.
  if (publish_vision_pose_ && publish_mavros_odometry_)
  {
    fatal("publish_vision_pose and publish_mavros_odometry are mutually exclusive: "
          "enabling both feeds PX4 EKF2 from two paths");
  }

  private_nh_.param("input_timeout", input_timeout_, 0.5);
  private_nh_.param("max_jump_m", max_jump_m_, 1.0);
  private_nh_.param("max_jump_deg", max_jump_deg_, 30.0);
  private_nh_.param("output_rate", output_rate_, 0.0);
  private_nh_.param("max_republish_age", max_republish_age_, 0.2);
  private_nh_.param("path_rate", path_rate_, 1.0);
  private_nh_.param("recovery_frames", recovery_frames_, 5);
  private_nh_.param("path_max_poses", path_max_poses_, 1000);

  private_nh_.param("consistency_check", consistency_check_, true);
  private_nh_.param<std::string>("px4_odom_topic", px4_odom_topic_,
                                 std::string("/mavros/local_position/odom"));
  private_nh_.param("consistency_tolerance", consistency_tolerance_, 1.0);

  const std::vector<double> extrinsic_xyz =
      readFixedArray(private_nh_, "base_to_input_xyz", {0.0, 0.0, 0.0});
  const std::vector<double> extrinsic_rpy =
      readFixedArray(private_nh_, "base_to_input_rpy_deg", {0.0, 0.0, 0.0});

  // T_base_input: where the VIO's child frame sits on the airframe, expressed
  // in base_link. A sensor mounted 10 cm forward of the body origin is
  // [+0.10, 0, 0]. T_world_base = T_world_input * inv(T_base_input).
  Eigen::Vector3d rpy_rad;
  for (int i = 0; i < 3; ++i)
  {
    rpy_rad[i] = extrinsic_rpy[static_cast<size_t>(i)] * kDegToRad;
  }
  base_to_input_ =
      transformFromXyzRpy(Eigen::Vector3d(extrinsic_xyz[0], extrinsic_xyz[1], extrinsic_xyz[2]),
                          rpy_rad);

  const std::vector<double> pose_floor = readFixedArray(
      private_nh_, "pose_covariance_floor", {0.0025, 0.0025, 0.0025, 0.0004, 0.0004, 0.0004});
  const std::vector<double> twist_floor = readFixedArray(
      private_nh_, "twist_covariance_floor", {0.01, 0.01, 0.01, 0.001, 0.001, 0.001});
  for (size_t i = 0; i < 6; ++i)
  {
    pose_covariance_floor_[i] = pose_floor[i];
    twist_covariance_floor_[i] = twist_floor[i];
  }

  if (max_jump_m_ <= 0.0 && max_jump_deg_ <= 0.0)
  {
    ROS_WARN("vins_mavros_bridge: jump detection disabled (max_jump_m and max_jump_deg <= 0)");
  }
  if (input_timeout_ <= 0.0)
  {
    ROS_WARN("vins_mavros_bridge: input_timeout <= 0 disables staleness detection");
  }
  if (recovery_frames_ < 1)
  {
    recovery_frames_ = 1;
  }
}

void Bridge::advertise()
{
  odometry_pub_ = nh_.advertise<nav_msgs::Odometry>(output_topic_, 20);

  if (publish_vision_pose_)
  {
    vision_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(vision_pose_topic_, 20);
  }
  if (publish_mavros_odometry_)
  {
    mavros_odometry_pub_ = nh_.advertise<nav_msgs::Odometry>(mavros_odometry_topic_, 20);
  }

  status_pub_ = nh_.advertise<BridgeStatus>(status_topic_, 5, /*latch=*/true);
  path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 1);
  valid_pub_ = nh_.advertise<std_msgs::Bool>(valid_topic_, 5, /*latch=*/true);

  path_.header.frame_id = world_frame_;

  std_msgs::Bool valid;
  valid.data = false;
  valid_pub_.publish(valid);
}

void Bridge::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  Measurement measurement;
  measurement.stamp = msg->header.stamp;

  const geometry_msgs::Point& position = msg->pose.pose.position;
  const geometry_msgs::Quaternion& orientation = msg->pose.pose.orientation;
  measurement.position = Eigen::Vector3d(position.x, position.y, position.z);
  measurement.orientation =
      Eigen::Quaterniond(orientation.w, orientation.x, orientation.y, orientation.z);
  measurement.linear = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                                       msg->twist.twist.linear.z);
  measurement.angular = Eigen::Vector3d(msg->twist.twist.angular.x, msg->twist.twist.angular.y,
                                        msg->twist.twist.angular.z);
  measurement.pose_covariance = msg->pose.covariance;
  measurement.twist_covariance = msg->twist.covariance;
  measurement.has_twist = true;

  // Frame-name mismatches are reported but not fatal: the configured extrinsic
  // is authoritative, and a wrong child_frame_id is the usual reason a VIO
  // silently lands in the wrong frame.
  if (!input_child_frame_.empty() && !msg->child_frame_id.empty() &&
      msg->child_frame_id != input_child_frame_)
  {
    ROS_WARN_THROTTLE(5.0,
                      "vins_mavros_bridge: input child_frame_id is '%s' but input_child_frame is "
                      "'%s'; the configured extrinsic is being used",
                      msg->child_frame_id.c_str(), input_child_frame_.c_str());
  }
  if (!world_frame_.empty() && !msg->header.frame_id.empty() &&
      msg->header.frame_id != world_frame_)
  {
    ROS_WARN_THROTTLE(5.0,
                      "vins_mavros_bridge: input frame_id is '%s', output world_frame is '%s'; "
                      "assuming the two worlds are gravity-aligned",
                      msg->header.frame_id.c_str(), world_frame_.c_str());
  }

  process(measurement);
}

void Bridge::poseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  Measurement measurement;
  measurement.stamp = msg->header.stamp;

  const geometry_msgs::Point& position = msg->pose.position;
  const geometry_msgs::Quaternion& orientation = msg->pose.orientation;
  measurement.position = Eigen::Vector3d(position.x, position.y, position.z);
  measurement.orientation =
      Eigen::Quaterniond(orientation.w, orientation.x, orientation.y, orientation.z);
  measurement.has_twist = false;

  process(measurement);
}

void Bridge::localPositionCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  have_px4_odom_ = true;
  px4_position_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                  msg->pose.pose.position.z);
}

void Bridge::process(const Measurement& raw)
{
  ++received_;

  Measurement measurement = raw;

  if (measurement.stamp.isZero())
  {
    measurement.stamp = ros::Time::now();
    ROS_WARN_THROTTLE(5.0,
                      "vins_mavros_bridge: input has a zero timestamp; substituting now(), which "
                      "defeats latency and staleness accounting");
  }

  if (!last_input_stamp_.isZero() && measurement.stamp < last_input_stamp_)
  {
    ++timestamp_resets_;
    // The estimator restarted or its time base moved backwards. Comparing the
    // new sample against the previous output would always look like a jump, so
    // re-arm the jump check instead of rejecting a legitimate reset.
    have_last_odometry_ = false;
    consecutive_valid_ = 0;
    ROS_WARN("vins_mavros_bridge: input timestamp went backwards (%.3f -> %.3f), treating it as "
             "an estimator reset",
             last_input_stamp_.toSec(), measurement.stamp.toSec());
  }
  last_input_stamp_ = measurement.stamp;

  const double latency = (ros::Time::now() - measurement.stamp).toSec();
  if (latency <= kClockSkewBound)
  {
    last_latency_ = latency;
    if (input_timeout_ > 0.0 && latency > input_timeout_)
    {
      ++rejected_stale_;
      consecutive_valid_ = 0;
      setState(State::kStale);
      ROS_WARN_THROTTLE(2.0,
                        "vins_mavros_bridge: dropping input already %.3f s old (timeout %.3f s)",
                        latency, input_timeout_);
      return;
    }
  }
  else
  {
    ROS_WARN_THROTTLE(
        10.0,
        "vins_mavros_bridge: input stamp is %.0f s behind the ROS clock; the VIO and the bridge "
        "are using different clocks, so latency and staleness checks are skipped",
        latency);
  }

  if (!measurementIsFinite(measurement))
  {
    ++rejected_invalid_;
    consecutive_valid_ = 0;
    ROS_WARN_THROTTLE(2.0,
                      "vins_mavros_bridge: dropping input with non-finite values or a degenerate "
                      "quaternion");
    return;
  }
  measurement.orientation.normalize();

  // T_world_input, exactly as the VIO reports it.
  Eigen::Isometry3d world_input = Eigen::Isometry3d::Identity();
  world_input.translation() = measurement.position;
  world_input.linear() = measurement.orientation.toRotationMatrix();

  // T_world_body = T_world_input * inv(T_base_input), before loop-closure fixes.
  const Eigen::Isometry3d world_body_raw = world_input * base_to_input_.inverse();

  // --- twist normalization -------------------------------------------------
  // The whole reason this package exists: OpenVINS publishes body-frame
  // velocity while px4ctrl and EGO-Planner consume world-frame velocity. A
  // wrong choice here is silent; it shows up as hover drift.
  Eigen::Vector3d linear_raw = measurement.linear;
  Eigen::Vector3d angular_raw = measurement.angular;
  Eigen::Matrix3d twist_rotation = Eigen::Matrix3d::Identity();
  if (measurement.has_twist && twist_frame_ == TwistFrame::kBody)
  {
    // Body velocity is expressed in the input child frame; the output body
    // frame differs only by a fixed extrinsic, so rotating by the world
    // orientation of the input child frame is exact.
    const Eigen::Matrix3d world_input_rotation = world_body_raw.linear() * base_to_input_.linear();
    linear_raw = world_input_rotation * linear_raw;
    angular_raw = world_input_rotation * angular_raw;
    twist_rotation = world_input_rotation;
  }

  Eigen::Isometry3d world_body = correction_ * world_body_raw;

  if (have_last_odometry_)
  {
    const double position_jump = (world_body.translation() - last_output_position_).norm();
    const double orientation_jump_deg =
        last_output_orientation_.angularDistance(Eigen::Quaterniond(world_body.linear())) *
        kRadToDeg;

    const bool position_exceeded = max_jump_m_ > 0.0 && position_jump > max_jump_m_;
    const bool orientation_exceeded = max_jump_deg_ > 0.0 && orientation_jump_deg > max_jump_deg_;

    if (position_exceeded || orientation_exceeded)
    {
      ++rejected_jump_;
      consecutive_valid_ = 0;

      if (jump_action_ == JumpAction::kResetOrigin)
      {
        // Absorb the discontinuity into a world correction so the published
        // trajectory stays continuous. The raw VIO output is left untouched, so
        // the correction is reversible and inspectable.
        correction_ = last_output_transform_ * world_body_raw.inverse();
        world_body = correction_ * world_body_raw;
        linear_raw = correction_.linear() * linear_raw;
        angular_raw = correction_.linear() * angular_raw;
        ++origin_resets_;
        ROS_WARN("vins_mavros_bridge: absorbed a %.3f m / %.1f deg jump into a world correction "
                 "(reset #%llu)",
                 position_jump, orientation_jump_deg, origin_resets_);
      }
      else
      {
        // Rejecting the sample is not enough: the stream is no longer the
        // continuous local estimate the controller was consuming, so stop
        // advertising it as valid until it recovers or is restarted.
        setState(State::kStale);
        ROS_WARN_THROTTLE(1.0,
                          "vins_mavros_bridge: rejected a %.3f m / %.1f deg pose jump (limits "
                          "%.3f m / %.1f deg)",
                          position_jump, orientation_jump_deg, max_jump_m_, max_jump_deg_);
        return;
      }
    }
  }

  nav_msgs::Odometry odometry;
  odometry.header.stamp = measurement.stamp;
  odometry.header.frame_id = world_frame_;
  odometry.child_frame_id = body_frame_;
  setPoseFromTransform(world_body, odometry.pose.pose);

  // Twist is published in the world frame on purpose: that is what
  // uav_utils::extract_odometry and ego_replan_fsm assume. It deviates from
  // REP-105, which the README states explicitly.
  odometry.twist.twist.linear.x = linear_raw.x();
  odometry.twist.twist.linear.y = linear_raw.y();
  odometry.twist.twist.linear.z = linear_raw.z();
  if (measurement.has_twist && input_twist_angular_)
  {
    odometry.twist.twist.angular.x = angular_raw.x();
    odometry.twist.twist.angular.y = angular_raw.y();
    odometry.twist.twist.angular.z = angular_raw.z();
  }

  // Covariances: the pose covariance a VIO publishes is already world
  // referenced, so it only follows the world correction (identity in the normal
  // case). The twist covariance follows the same rotation that was applied to
  // the twist itself. Both diagonals are floored so downstream consumers never
  // see an over-confident estimate.
  rotateCovariance(measurement.pose_covariance, odometry.pose.covariance, correction_.linear());
  applyCovarianceFloor(odometry.pose.covariance, pose_covariance_floor_);

  rotateCovariance(measurement.twist_covariance, odometry.twist.covariance,
                   correction_.linear() * twist_rotation);
  applyCovarianceFloor(odometry.twist.covariance, twist_covariance_floor_);

  ++consecutive_valid_;
  if (state_ != State::kOk && consecutive_valid_ >= recovery_frames_)
  {
    setState(State::kOk);
  }

  publish(odometry);

  last_output_transform_ = world_body;
  last_output_position_ = world_body.translation();
  last_output_orientation_ = Eigen::Quaterniond(world_body.linear()).normalized();
  last_output_odometry_ = odometry;
  last_output_stamp_ = measurement.stamp;
  last_publish_wall_ = ros::Time::now();
  have_last_odometry_ = true;
}

void Bridge::publish(const nav_msgs::Odometry& odometry)
{
  // Nothing is forwarded until the stream has proven itself (startup or after a
  // recovery). Publishing a partially-validated estimate to the controller is
  // exactly the silent failure this package is meant to prevent.
  if (state_ != State::kOk)
  {
    return;
  }

  odometry_pub_.publish(odometry);
  ++published_;

  if (publish_vision_pose_ && vision_pose_pub_)
  {
    geometry_msgs::PoseStamped vision_pose;
    vision_pose.header = odometry.header;
    vision_pose.pose = odometry.pose.pose;
    vision_pose_pub_.publish(vision_pose);
  }

  if (publish_mavros_odometry_ && mavros_odometry_pub_)
  {
    // /mavros/odometry/in expects REP-105 body-frame twist, so convert back
    // from the world-frame twist this package publishes internally.
    nav_msgs::Odometry mavros_odometry = odometry;
    const Eigen::Quaterniond orientation(
        odometry.pose.pose.orientation.w, odometry.pose.pose.orientation.x,
        odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z);
    const Eigen::Matrix3d world_body_rotation = orientation.toRotationMatrix();
    const Eigen::Vector3d linear_body =
        world_body_rotation.transpose() *
        Eigen::Vector3d(odometry.twist.twist.linear.x, odometry.twist.twist.linear.y,
                        odometry.twist.twist.linear.z);
    const Eigen::Vector3d angular_body =
        world_body_rotation.transpose() *
        Eigen::Vector3d(odometry.twist.twist.angular.x, odometry.twist.twist.angular.y,
                        odometry.twist.twist.angular.z);
    mavros_odometry.twist.twist.linear.x = linear_body.x();
    mavros_odometry.twist.twist.linear.y = linear_body.y();
    mavros_odometry.twist.twist.linear.z = linear_body.z();
    mavros_odometry.twist.twist.angular.x = angular_body.x();
    mavros_odometry.twist.twist.angular.y = angular_body.y();
    mavros_odometry.twist.twist.angular.z = angular_body.z();
    mavros_odometry_pub_.publish(mavros_odometry);
  }

  if (publish_tf_)
  {
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = odometry.header.stamp;
    transform.header.frame_id = world_frame_;
    transform.child_frame_id = body_frame_;
    transform.transform.translation.x = odometry.pose.pose.position.x;
    transform.transform.translation.y = odometry.pose.pose.position.y;
    transform.transform.translation.z = odometry.pose.pose.position.z;
    transform.transform.rotation = odometry.pose.pose.orientation;
    tf_broadcaster_.sendTransform(transform);
  }

  if (path_pub_ && path_rate_ > 0.0)
  {
    const ros::Time now = ros::Time::now();
    const bool due = path_.poses.empty() ||
                     (now - path_.header.stamp).toSec() >= 1.0 / path_rate_;
    if (due)
    {
      geometry_msgs::PoseStamped pose;
      pose.header = odometry.header;
      pose.pose = odometry.pose.pose;
      path_.poses.push_back(pose);
      if (path_max_poses_ > 0 && static_cast<int>(path_.poses.size()) > path_max_poses_)
      {
        path_.poses.erase(path_.poses.begin());
      }
      path_.header.stamp = now;
      path_pub_.publish(path_);
    }
  }

  if (consistency_check_ && have_px4_odom_)
  {
    // Same-source cross-check: EKF2 fuses this stream, so its local position
    // should follow it. A constant offset is expected because the VIO origin and
    // the PX4 local origin differ; a growing one means EKF2 is not using it.
    const Eigen::Vector3d bridge_position(odometry.pose.pose.position.x,
                                          odometry.pose.pose.position.y,
                                          odometry.pose.pose.position.z);
    const double deviation = (px4_position_ - bridge_position).norm();
    if (deviation > consistency_tolerance_)
    {
      ROS_WARN_THROTTLE(5.0,
                        "vins_mavros_bridge: %.3f m deviation from %s (tolerance %.3f m); check "
                        "the PX4 origin offset before trusting the EKF2 fusion",
                        deviation, px4_odom_topic_.c_str(), consistency_tolerance_);
    }
  }
}

void Bridge::setState(State state)
{
  if (state_ == state)
  {
    return;
  }
  state_ = state;

  std_msgs::Bool valid;
  valid.data = (state == State::kOk);
  valid_pub_.publish(valid);

  switch (state)
  {
    case State::kInit:
      ROS_INFO("vins_mavros_bridge: INIT");
      break;
    case State::kOk:
      ROS_INFO("vins_mavros_bridge: OK, external vision active");
      break;
    case State::kStale:
      ROS_WARN("vins_mavros_bridge: STALE, external vision suspended");
      break;
  }
}

void Bridge::onWatchdog(const ros::TimerEvent& /*event*/)
{
  const ros::Time now = ros::Time::now();

  if (state_ == State::kOk && !last_publish_wall_.isZero() &&
      (now - last_publish_wall_).toSec() > input_timeout_)
  {
    ROS_WARN("vins_mavros_bridge: no accepted input for %.3f s",
             (now - last_publish_wall_).toSec());
    setState(State::kStale);
    return;
  }

  // Optional zero-order hold. It repeats only the newest sample and never
  // resurrects data older than max_republish_age_, so it cannot mask a dead
  // estimator.
  if (output_rate_ <= 0.0 || state_ != State::kOk || !have_last_odometry_)
  {
    return;
  }
  const double period = 1.0 / output_rate_;
  if ((now - last_publish_wall_).toSec() < period)
  {
    return;
  }
  if (max_republish_age_ > 0.0 && (now - last_output_stamp_).toSec() > max_republish_age_)
  {
    return;
  }
  odometry_pub_.publish(last_output_odometry_);
  ++published_;
  last_publish_wall_ = now;
}

void Bridge::onStatus(const ros::TimerEvent& /*event*/)
{
  const ros::Time now = ros::Time::now();
  const double elapsed = (now - rate_window_start_).toSec();
  if (elapsed > 1e-3)
  {
    input_rate_hz_ = static_cast<double>(received_ - rate_window_received_) / elapsed;
    output_rate_hz_ = static_cast<double>(published_ - rate_window_published_) / elapsed;
    rate_window_start_ = now;
    rate_window_received_ = received_;
    rate_window_published_ = published_;
  }

  BridgeStatus status;
  status.header.stamp = now;
  status.header.frame_id = world_frame_;
  status.state = (state_ == State::kOk)
                     ? BridgeStatus::STATE_OK
                     : ((state_ == State::kStale) ? BridgeStatus::STATE_STALE
                                                  : BridgeStatus::STATE_INIT);
  status.valid = (state_ == State::kOk);
  status.input_topic = input_topic_;
  status.world_frame = world_frame_;
  status.body_frame = body_frame_;
  status.input_rate_hz = input_rate_hz_;
  status.output_rate_hz = output_rate_hz_;
  status.input_latency = last_latency_;
  status.received = received_;
  status.published = published_;
  status.rejected_invalid = rejected_invalid_;
  status.rejected_stale = rejected_stale_;
  status.rejected_jump = rejected_jump_;
  status.timestamp_resets = timestamp_resets_;
  status.origin_resets = origin_resets_;
  status_pub_.publish(status);

  if (received_ > 0 && state_ != State::kOk)
  {
    ROS_WARN("vins_mavros_bridge: input %.1f Hz, output %.1f Hz, latency %.3f s, rejected "
             "(invalid %llu, stale %llu, jump %llu)",
             input_rate_hz_, output_rate_hz_, last_latency_, rejected_invalid_, rejected_stale_,
             rejected_jump_);
  }
}

Eigen::Isometry3d Bridge::transformFromXyzRpy(const Eigen::Vector3d& xyz,
                                              const Eigen::Vector3d& rpy_rad)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = xyz;
  transform.linear() =
      (Eigen::AngleAxisd(rpy_rad.z(), Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(rpy_rad.y(), Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(rpy_rad.x(), Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  return transform;
}

Eigen::Vector3d Bridge::rpyFromRotation(const Eigen::Matrix3d& rotation)
{
  // ZYX (yaw-pitch-roll), matching transformFromXyzRpy.
  const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
  const double pitch = std::asin(std::max(-1.0, std::min(1.0, -rotation(2, 0))));
  const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  return Eigen::Vector3d(roll, pitch, yaw);
}

void Bridge::setPoseFromTransform(const Eigen::Isometry3d& transform, geometry_msgs::Pose& pose)
{
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();

  const Eigen::Quaterniond orientation(transform.linear());
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.orientation.w = orientation.w();
}

void Bridge::rotateCovariance(const boost::array<double, 36>& input,
                              boost::array<double, 36>& output,
                              const Eigen::Matrix3d& rotation)
{
  Eigen::Matrix<double, 6, 6> covariance;
  for (int row = 0; row < 6; ++row)
  {
    for (int column = 0; column < 6; ++column)
    {
      covariance(row, column) = input[static_cast<size_t>(row * 6 + column)];
    }
  }

  Eigen::Matrix<double, 6, 6> transform = Eigen::Matrix<double, 6, 6>::Zero();
  transform.block<3, 3>(0, 0) = rotation;
  transform.block<3, 3>(3, 3) = rotation;

  covariance = transform * covariance * transform.transpose();

  for (int row = 0; row < 6; ++row)
  {
    for (int column = 0; column < 6; ++column)
    {
      output[static_cast<size_t>(row * 6 + column)] = covariance(row, column);
    }
  }
}

void Bridge::applyCovarianceFloor(boost::array<double, 36>& covariance,
                                  const std::array<double, 6>& floor_values)
{
  for (size_t i = 0; i < floor_values.size(); ++i)
  {
    const size_t index = i * 6 + i;
    // Also catches NaN: any comparison with NaN is false.
    if (!(covariance[index] >= floor_values[i]))
    {
      covariance[index] = floor_values[i];
    }
  }
}

bool Bridge::measurementIsFinite(const Measurement& measurement)
{
  if (!measurement.position.allFinite() || !measurement.linear.allFinite() ||
      !measurement.angular.allFinite())
  {
    return false;
  }
  if (!measurement.orientation.coeffs().allFinite())
  {
    return false;
  }
  const double norm = measurement.orientation.squaredNorm();
  return std::isfinite(norm) && norm > 1e-6;
}

void Bridge::fatal(const std::string& message)
{
  ROS_FATAL("vins_mavros_bridge: %s", message.c_str());
  ros::shutdown();
  std::exit(EXIT_FAILURE);
}

}  // namespace vins_mavros_bridge
