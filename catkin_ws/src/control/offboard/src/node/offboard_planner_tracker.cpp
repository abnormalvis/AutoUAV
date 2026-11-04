#include "offboard_planner_tracker.h"

#include <algorithm>
#include <cmath>

namespace offboard
{
PlannerTracker::PlannerTracker(const ros::NodeHandle& nh) : nh_(nh)
{
  nh_.param("takeoff_height", takeoff_height_, takeoff_height_);
  nh_.param("takeoff_vertical_speed", takeoff_vertical_speed_, takeoff_vertical_speed_);
  nh_.param("takeoff_tolerance", takeoff_tolerance_, takeoff_tolerance_);
  nh_.param("yaw_excitation_rate", yaw_excitation_rate_, yaw_excitation_rate_);
  nh_.param("yaw_excitation_duration", yaw_excitation_duration_, yaw_excitation_duration_);
  nh_.param("offboard_prestream_duration", offboard_prestream_duration_, offboard_prestream_duration_);
  nh_.param("sensor_timeout", sensor_timeout_, sensor_timeout_);
  nh_.param("vio_stable_duration", vio_stable_duration_, vio_stable_duration_);
  nh_.param("vio_init_timeout", vio_init_timeout_, vio_init_timeout_);
  nh_.param("bootstrap_timeout", bootstrap_timeout_, bootstrap_timeout_);
  nh_.param("land_request_retry", land_request_retry_, land_request_retry_);
  nh_.param("position_gain", position_gain_, position_gain_);
  nh_.param("max_velocity", max_velocity_, max_velocity_);
  nh_.param("command_timeout", command_timeout_, command_timeout_);

  state_sub_ = nh_.subscribe("/mavros/state", 10, &PlannerTracker::stateCallback, this);
  go_sub_ = nh_.subscribe("/offboard/go", 1, &PlannerTracker::goCallback, this);
  altitude_sub_ = nh_.subscribe("/mavros/altitude", 10, &PlannerTracker::altitudeCallback, this);
  estimator_status_sub_ = nh_.subscribe("/mavros/estimator_status", 10,
                                        &PlannerTracker::estimatorStatusCallback, this);
  odom_sub_ = nh_.subscribe("/ov_msckf/odomimu", 10, &PlannerTracker::odomCallback, this);
  command_sub_ = nh_.subscribe("/position_cmd", 10, &PlannerTracker::commandCallback, this);
  setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
  trigger_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 1, true);
  mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  arm_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
}

void PlannerTracker::stateCallback(const mavros_msgs::State::ConstPtr& msg)
{
  state_ = *msg;
  state_received_ = ros::Time::now();
}

void PlannerTracker::goCallback(const std_msgs::Empty::ConstPtr&)
{
  go_requested_ = true;
  ROS_WARN("Received /offboard/go; autonomous takeoff sequence is enabled");
}

void PlannerTracker::altitudeCallback(const mavros_msgs::Altitude::ConstPtr& msg)
{
  altitude_ = *msg;
  altitude_received_ = ros::Time::now();
  have_altitude_ = true;
}

void PlannerTracker::estimatorStatusCallback(const mavros_msgs::EstimatorStatus::ConstPtr& msg)
{
  estimator_status_ = *msg;
  estimator_status_received_ = ros::Time::now();
  have_estimator_status_ = true;
}

void PlannerTracker::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom_ = *msg;
  odom_received_ = ros::Time::now();
  have_odom_ = true;
}

void PlannerTracker::commandCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg)
{
  if (msg->trajectory_flag != quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY)
  {
    ROS_WARN_THROTTLE(2.0, "Ignoring EGO command with non-ready trajectory status %u", msg->trajectory_flag);
    return;
  }

  command_ = *msg;
  command_received_ = ros::Time::now();
  have_command_ = true;
  stale_hold_active_ = false;
}

double PlannerTracker::currentYaw() const
{
  const geometry_msgs::Quaternion& q = odom_.pose.pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

bool PlannerTracker::bootstrapEstimateReady(const ros::Time& now) const
{
  return have_estimator_status_ && have_altitude_ &&
         (now - estimator_status_received_).toSec() <= sensor_timeout_ &&
         (now - altitude_received_).toSec() <= sensor_timeout_ &&
         estimator_status_.attitude_status_flag &&
         estimator_status_.velocity_vert_status_flag &&
         std::isfinite(altitude_.relative);
}

bool PlannerTracker::vioOdometryFresh(const ros::Time& now) const
{
  return have_odom_ && (now - odom_received_).toSec() <= sensor_timeout_ &&
         std::isfinite(odom_.pose.pose.position.x) &&
         std::isfinite(odom_.pose.pose.position.y) &&
         std::isfinite(odom_.pose.pose.position.z) &&
         std::isfinite(odom_.pose.pose.orientation.w) &&
         std::isfinite(odom_.pose.pose.orientation.x) &&
         std::isfinite(odom_.pose.pose.orientation.y) &&
         std::isfinite(odom_.pose.pose.orientation.z);
}

bool PlannerTracker::positionEstimateReady(const ros::Time& now) const
{
  const bool horizontal_position_valid = estimator_status_.pos_horiz_rel_status_flag ||
                                         estimator_status_.pos_horiz_abs_status_flag;
  const bool vertical_position_valid = estimator_status_.pos_vert_abs_status_flag ||
                                       estimator_status_.pos_vert_agl_status_flag;
  return bootstrapEstimateReady(now) && vioOdometryFresh(now) &&
         estimator_status_.velocity_horiz_status_flag &&
         horizontal_position_valid && vertical_position_valid;
}

void PlannerTracker::publishPositionSetpoint(double x, double y, double z, double yaw)
{
  mavros_msgs::PositionTarget setpoint;
  setpoint.header.stamp = ros::Time::now();
  setpoint.header.frame_id = "map";
  setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  setpoint.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                       mavros_msgs::PositionTarget::IGNORE_VY |
                       mavros_msgs::PositionTarget::IGNORE_VZ |
                       mavros_msgs::PositionTarget::IGNORE_AFX |
                       mavros_msgs::PositionTarget::IGNORE_AFY |
                       mavros_msgs::PositionTarget::IGNORE_AFZ |
                       mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
  setpoint.position.x = x;
  setpoint.position.y = y;
  setpoint.position.z = z;
  setpoint.yaw = yaw;
  setpoint_pub_.publish(setpoint);
}

void PlannerTracker::publishVelocitySetpoint(double vx, double vy, double vz, double yaw,
                                             double yaw_rate, bool use_yaw_rate)
{
  mavros_msgs::PositionTarget setpoint;
  setpoint.header.stamp = ros::Time::now();
  setpoint.header.frame_id = "map";
  setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  setpoint.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                       mavros_msgs::PositionTarget::IGNORE_PY |
                       mavros_msgs::PositionTarget::IGNORE_PZ |
                       mavros_msgs::PositionTarget::IGNORE_AFX |
                       mavros_msgs::PositionTarget::IGNORE_AFY |
                       mavros_msgs::PositionTarget::IGNORE_AFZ |
                       mavros_msgs::PositionTarget::IGNORE_YAW;
  if (!use_yaw_rate)
    setpoint.type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
  setpoint.velocity.x = vx;
  setpoint.velocity.y = vy;
  setpoint.velocity.z = vz;
  setpoint.yaw = yaw;
  setpoint.yaw_rate = yaw_rate;
  setpoint_pub_.publish(setpoint);
}

bool PlannerTracker::requestMode(const std::string& mode)
{
  mavros_msgs::SetMode request;
  request.request.custom_mode = mode;
  if (!mode_client_.call(request) || !request.response.mode_sent)
  {
    ROS_WARN_THROTTLE(2.0, "PX4 did not accept mode %s", mode.c_str());
    return false;
  }
  return true;
}

bool PlannerTracker::arm()
{
  mavros_msgs::CommandBool request;
  request.request.value = true;
  if (!arm_client_.call(request) || !request.response.success)
  {
    ROS_WARN_THROTTLE(2.0, "PX4 arming request was rejected");
    return false;
  }
  return true;
}

void PlannerTracker::run()
{
  ros::Rate rate(50.0);
  const auto send_zero_velocity = [this]() {
    publishVelocitySetpoint(0.0, 0.0, 0.0, 0.0, 0.0, true);
  };
  const auto request_land = [this, &rate, &send_zero_velocity]() {
    ros::Time last_request(0);
    while (ros::ok() && state_.mode != "AUTO.LAND")
    {
      send_zero_velocity();
      ros::spinOnce();
      if ((ros::Time::now() - last_request).toSec() >= land_request_retry_)
      {
        requestMode("AUTO.LAND");
        last_request = ros::Time::now();
      }
      rate.sleep();
    }
  };

  ROS_INFO("Waiting for PX4 attitude/vertical-velocity estimate and barometric altitude");
  while (ros::ok())
  {
    ros::spinOnce();
    if (state_.connected && bootstrapEstimateReady(ros::Time::now()))
      break;
    rate.sleep();
  }
  if (!ros::ok())
    return;

  ROS_INFO("Waiting for explicit /offboard/go trigger; no setpoints or arming before GO");
  while (ros::ok() && !go_requested_)
  {
    ros::spinOnce();
    rate.sleep();
  }
  if (!ros::ok())
    return;

  const double takeoff_start_altitude = altitude_.relative;
  const ros::Time prestream_start = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - prestream_start).toSec() < offboard_prestream_duration_)
  {
    if (!bootstrapEstimateReady(ros::Time::now()))
    {
      ROS_ERROR("PX4 attitude/vertical-velocity or altitude estimate became stale before OFFBOARD prestream");
      return;
    }
    send_zero_velocity();
    ros::spinOnce();
    rate.sleep();
  }
  if (!ros::ok())
    return;

  ROS_INFO("Requesting PX4 OFFBOARD mode");
  ros::Time last_mode_request(0);
  while (ros::ok() && state_.mode != "OFFBOARD")
  {
    if (!bootstrapEstimateReady(ros::Time::now()))
    {
      ROS_ERROR("PX4 bootstrap estimate became invalid before OFFBOARD mode was accepted");
      return;
    }
    send_zero_velocity();
    ros::spinOnce();
    if ((ros::Time::now() - last_mode_request).toSec() > 1.0)
    {
      requestMode("OFFBOARD");
      last_mode_request = ros::Time::now();
    }
    rate.sleep();
  }
  if (!ros::ok())
    return;

  ROS_INFO("OFFBOARD active; arming PX4");
  while (ros::ok() && !state_.armed)
  {
    if (!bootstrapEstimateReady(ros::Time::now()))
    {
      ROS_ERROR("PX4 bootstrap estimate became invalid before arming");
      return;
    }
    send_zero_velocity();
    ros::spinOnce();
    arm();
    rate.sleep();
  }
  if (!ros::ok())
    return;

  const double target_altitude = takeoff_start_altitude + takeoff_height_;
  const ros::Time takeoff_start = ros::Time::now();
  ROS_WARN("Starting vertical-velocity bootstrap to %.2f m relative altitude", takeoff_height_);
  while (ros::ok())
  {
    ros::spinOnce();
    const ros::Time now = ros::Time::now();
    if (!bootstrapEstimateReady(now))
    {
      ROS_ERROR("PX4 bootstrap estimate became invalid during climb; requesting AUTO.LAND");
      request_land();
      return;
    }
    if ((now - takeoff_start).toSec() > bootstrap_timeout_)
    {
      ROS_ERROR("Vertical bootstrap timed out before reaching %.2f m; requesting AUTO.LAND", target_altitude);
      request_land();
      return;
    }

    if (altitude_.relative >= target_altitude - takeoff_tolerance_)
    {
      send_zero_velocity();
      break;
    }

    const double yaw_rate = (now - takeoff_start).toSec() < yaw_excitation_duration_
                                ? yaw_excitation_rate_
                                : 0.0;
    publishVelocitySetpoint(0.0, 0.0, takeoff_vertical_speed_, 0.0, yaw_rate, true);
    rate.sleep();
  }
  if (!ros::ok())
    return;

  ROS_WARN("Bootstrap altitude reached; waiting for fresh VIO and valid PX4 local position");
  const ros::Time vio_wait_start = ros::Time::now();
  ros::Time position_valid_since(0);
  while (ros::ok())
  {
    ros::spinOnce();
    const ros::Time now = ros::Time::now();
    if (!bootstrapEstimateReady(now))
    {
      ROS_ERROR("PX4 attitude/vertical-velocity or altitude estimate became invalid before VIO handoff");
      request_land();
      return;
    }
    if ((now - vio_wait_start).toSec() > vio_init_timeout_)
    {
      ROS_ERROR("VIO and PX4 local position were not valid within %.1f s; requesting AUTO.LAND",
                vio_init_timeout_);
      request_land();
      return;
    }

    if (positionEstimateReady(now))
    {
      if (position_valid_since.isZero())
        position_valid_since = now;
      if ((now - position_valid_since).toSec() >= vio_stable_duration_)
        break;
    }
    else
    {
      position_valid_since = ros::Time(0);
    }

    const double yaw_rate = (now - vio_wait_start).toSec() < yaw_excitation_duration_
                                ? yaw_excitation_rate_
                                : 0.0;
    publishVelocitySetpoint(0.0, 0.0, 0.0, 0.0, yaw_rate, true);
    rate.sleep();
  }
  if (!ros::ok())
    return;

  const double vio_hold_x = odom_.pose.pose.position.x;
  const double vio_hold_y = odom_.pose.pose.position.y;
  const double vio_hold_z = odom_.pose.pose.position.z;
  const double vio_hold_yaw = currentYaw();
  const ros::Time hold_start = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - hold_start).toSec() < 1.0)
  {
    if (!positionEstimateReady(ros::Time::now()))
    {
      ROS_ERROR("PX4 local position became invalid during position-control handoff; requesting AUTO.LAND");
      request_land();
      return;
    }
    publishPositionSetpoint(vio_hold_x, vio_hold_y, vio_hold_z, vio_hold_yaw);
    ros::spinOnce();
    rate.sleep();
  }
  if (!ros::ok())
    return;

  geometry_msgs::PoseStamped trigger;
  trigger.header.stamp = ros::Time::now();
  trigger.header.frame_id = "world";
  trigger.pose = odom_.pose.pose;
  trigger_pub_.publish(trigger);
  ROS_INFO("VIO odometry and PX4 local position are stable; triggered EGO preset planner");

  ros::Time last_offboard_warning(0);
  while (ros::ok())
  {
    ros::spinOnce();

    if (!positionEstimateReady(ros::Time::now()))
    {
      ROS_ERROR_THROTTLE(1.0, "VIO/PX4 position estimate became invalid; requesting AUTO.LAND");
      request_land();
      return;
    }

    if (state_.mode != "OFFBOARD" && (ros::Time::now() - last_offboard_warning).toSec() > 2.0)
    {
      ROS_WARN("PX4 left OFFBOARD mode; not requesting it again automatically");
      last_offboard_warning = ros::Time::now();
    }

    if (have_command_ && (ros::Time::now() - command_received_).toSec() <= command_timeout_)
    {
      const double ex = command_.position.x - odom_.pose.pose.position.x;
      const double ey = command_.position.y - odom_.pose.pose.position.y;
      const double ez = command_.position.z - odom_.pose.pose.position.z;
      double vx = command_.velocity.x + position_gain_ * ex;
      double vy = command_.velocity.y + position_gain_ * ey;
      double vz = command_.velocity.z + position_gain_ * ez;
      const double speed = std::sqrt(vx * vx + vy * vy + vz * vz);
      if (speed > max_velocity_ && speed > 0.0)
      {
        const double scale = max_velocity_ / speed;
        vx *= scale;
        vy *= scale;
        vz *= scale;
      }
      publishVelocitySetpoint(vx, vy, vz, command_.yaw);
    }
    else
    {
      if (!stale_hold_active_)
      {
        stale_hold_position_ = odom_.pose.pose.position;
        stale_hold_active_ = true;
        if (have_command_)
          ROS_WARN("EGO command timed out; holding current position");
        else
          ROS_INFO_THROTTLE(5.0, "Waiting for the first EGO position command");
      }
      publishPositionSetpoint(stale_hold_position_.x, stale_hold_position_.y,
                              stale_hold_position_.z, currentYaw());
    }
    rate.sleep();
  }
}
}