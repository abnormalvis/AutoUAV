#ifndef VINS_MAVROS_BRIDGE_BRIDGE_H_
#define VINS_MAVROS_BRIDGE_BRIDGE_H_

#include <array>
#include <string>

#include <boost/array.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_broadcaster.h>

#include <vins_mavros_bridge/BridgeStatus.h>

namespace vins_mavros_bridge
{

/// Normalizes one VIO source into the single local odometry interface used by
/// the flight stack.
///
/// One node plus a YAML adapter configuration, not one C++ class per algorithm:
/// every candidate VIO publishes nav_msgs/Odometry (or PoseStamped), and the
/// differences are limited to the topic name, the input child frame, the
/// reference frame of the twist, and whether the angular rate is usable. Those
/// are parameters, not code.
class Bridge
{
public:
  Bridge();

private:
  enum class InputMode { kOdometry, kPoseStamped };
  enum class TwistFrame { kBody, kWorld };
  enum class JumpAction { kReject, kResetOrigin };
  enum class State { kInit, kOk, kStale };

  struct Measurement
  {
    ros::Time stamp;
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d linear{Eigen::Vector3d::Zero()};
    Eigen::Vector3d angular{Eigen::Vector3d::Zero()};
    boost::array<double, 36> pose_covariance{};
    boost::array<double, 36> twist_covariance{};
    bool has_twist{false};
  };

  void loadParameters();
  void advertise();

  void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void poseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void localPositionCallback(const nav_msgs::Odometry::ConstPtr& msg);

  void process(const Measurement& measurement);
  void publish(const nav_msgs::Odometry& odometry);
  void setState(State state);

  void onWatchdog(const ros::TimerEvent& event);
  void onStatus(const ros::TimerEvent& event);

  static Eigen::Isometry3d transformFromXyzRpy(const Eigen::Vector3d& xyz,
                                               const Eigen::Vector3d& rpy_rad);
  static Eigen::Vector3d rpyFromRotation(const Eigen::Matrix3d& rotation);
  static void setPoseFromTransform(const Eigen::Isometry3d& transform,
                                   geometry_msgs::Pose& pose);
  static void rotateCovariance(const boost::array<double, 36>& input,
                               boost::array<double, 36>& output,
                               const Eigen::Matrix3d& rotation);
  static void applyCovarianceFloor(boost::array<double, 36>& covariance,
                                   const std::array<double, 6>& floor_values);
  static bool measurementIsFinite(const Measurement& measurement);
  static void fatal(const std::string& message);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // --- adapter configuration -----------------------------------------------
  InputMode input_mode_{InputMode::kOdometry};
  TwistFrame twist_frame_{TwistFrame::kBody};
  JumpAction jump_action_{JumpAction::kReject};
  bool input_twist_angular_{true};

  std::string input_topic_;
  std::string world_frame_;
  std::string body_frame_;
  std::string input_child_frame_;

  std::string output_topic_;
  std::string vision_pose_topic_;
  std::string mavros_odometry_topic_;
  std::string status_topic_;
  std::string path_topic_;
  std::string valid_topic_;

  bool publish_vision_pose_{true};
  bool publish_mavros_odometry_{false};
  bool publish_tf_{false};

  double input_timeout_{0.5};
  double max_jump_m_{1.0};
  double max_jump_deg_{30.0};
  double output_rate_{0.0};
  double max_republish_age_{0.2};
  double path_rate_{1.0};
  int recovery_frames_{5};
  int path_max_poses_{1000};

  bool consistency_check_{true};
  std::string px4_odom_topic_;
  double consistency_tolerance_{1.0};

  std::array<double, 6> pose_covariance_floor_{};
  std::array<double, 6> twist_covariance_floor_{};

  /// T_base_input: the pose of the VIO's child frame expressed in base_link,
  /// i.e. where the sensor is mounted relative to the body origin.
  Eigen::Isometry3d base_to_input_{Eigen::Isometry3d::Identity()};

  /// Left-multiplied world correction. Identity unless a reset_origin event
  /// occurred; loop-closure discontinuities land here, never in the raw stream.
  Eigen::Isometry3d correction_{Eigen::Isometry3d::Identity()};

  // --- runtime state --------------------------------------------------------
  State state_{State::kInit};
  int consecutive_valid_{0};
  ros::Time last_input_stamp_;
  ros::Time last_output_stamp_;
  /// ros::Time::now() at the last accepted sample. Uses the local clock, so the
  /// watchdog keeps working even when VIO stamps come from another epoch.
  ros::Time last_publish_wall_;
  Eigen::Isometry3d last_output_transform_{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d last_output_position_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond last_output_orientation_{Eigen::Quaterniond::Identity()};
  nav_msgs::Odometry last_output_odometry_;
  bool have_last_odometry_{false};

  // --- statistics -----------------------------------------------------------
  unsigned long long received_{0};
  unsigned long long published_{0};
  unsigned long long rejected_invalid_{0};
  unsigned long long rejected_stale_{0};
  unsigned long long rejected_jump_{0};
  unsigned long long timestamp_resets_{0};
  unsigned long long origin_resets_{0};
  double last_latency_{0.0};
  double input_rate_hz_{0.0};
  double output_rate_hz_{0.0};
  unsigned long long rate_window_received_{0};
  unsigned long long rate_window_published_{0};
  ros::Time rate_window_start_;

  // --- PX4 cross-check ------------------------------------------------------
  bool have_px4_odom_{false};
  Eigen::Vector3d px4_position_{Eigen::Vector3d::Zero()};

  // --- ROS interfaces -------------------------------------------------------
  ros::Subscriber input_sub_;
  ros::Subscriber px4_odom_sub_;
  ros::Publisher odometry_pub_;
  ros::Publisher vision_pose_pub_;
  ros::Publisher mavros_odometry_pub_;
  ros::Publisher status_pub_;
  ros::Publisher path_pub_;
  ros::Publisher valid_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  nav_msgs::Path path_;

  ros::Timer watchdog_timer_;
  ros::Timer status_timer_;
};

}  // namespace vins_mavros_bridge

#endif  // VINS_MAVROS_BRIDGE_BRIDGE_H_
