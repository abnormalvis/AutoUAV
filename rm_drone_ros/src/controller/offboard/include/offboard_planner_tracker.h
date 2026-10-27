#ifndef OFFBOARD_PLANNER_TRACKER_H
#define OFFBOARD_PLANNER_TRACKER_H

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/EstimatorStatus.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <string>

namespace offboard
{
class PlannerTracker
{
public:
  explicit PlannerTracker(const ros::NodeHandle& nh);
  void run();

private:
  void stateCallback(const mavros_msgs::State::ConstPtr& msg);
  void goCallback(const std_msgs::Empty::ConstPtr& msg);
  void altitudeCallback(const mavros_msgs::Altitude::ConstPtr& msg);
  void estimatorStatusCallback(const mavros_msgs::EstimatorStatus::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void commandCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg);
  void publishPositionSetpoint(double x, double y, double z, double yaw);
  void publishVelocitySetpoint(double vx, double vy, double vz, double yaw,
                               double yaw_rate = 0.0, bool use_yaw_rate = false);
  bool requestMode(const std::string& mode);
  bool arm();
  double currentYaw() const;
  bool bootstrapEstimateReady(const ros::Time& now) const;
  bool positionEstimateReady(const ros::Time& now) const;
  bool vioOdometryFresh(const ros::Time& now) const;

  ros::NodeHandle nh_;
  ros::Subscriber state_sub_;
  ros::Subscriber go_sub_;
  ros::Subscriber altitude_sub_;
  ros::Subscriber estimator_status_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber command_sub_;
  ros::Publisher setpoint_pub_;
  ros::Publisher trigger_pub_;
  ros::ServiceClient mode_client_;
  ros::ServiceClient arm_client_;

  mavros_msgs::State state_;
  mavros_msgs::Altitude altitude_;
  mavros_msgs::EstimatorStatus estimator_status_;
  nav_msgs::Odometry odom_;
  quadrotor_msgs::PositionCommand command_;
  ros::Time state_received_;
  ros::Time altitude_received_;
  ros::Time estimator_status_received_;
  ros::Time odom_received_;
  ros::Time command_received_;
  bool go_requested_ = false;
  bool have_altitude_ = false;
  bool have_estimator_status_ = false;
  bool have_odom_ = false;
  bool have_command_ = false;
  bool stale_hold_active_ = false;
  geometry_msgs::Point stale_hold_position_;

  double takeoff_height_ = 0.5;
  double takeoff_vertical_speed_ = 0.25;
  double takeoff_tolerance_ = 0.15;
  double yaw_excitation_rate_ = 0.12;
  double yaw_excitation_duration_ = 1.5;
  double offboard_prestream_duration_ = 1.5;
  double sensor_timeout_ = 0.5;
  double vio_stable_duration_ = 2.0;
  double vio_init_timeout_ = 30.0;
  double bootstrap_timeout_ = 15.0;
  double land_request_retry_ = 1.0;
  double position_gain_ = 0.8;
  double max_velocity_ = 0.8;
  double command_timeout_ = 0.5;
};
}

#endif