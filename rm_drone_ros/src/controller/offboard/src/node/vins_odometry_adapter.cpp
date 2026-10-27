#include <cmath>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

class VinsOdometryAdapter
{
public:
  VinsOdometryAdapter() : private_nh_("~"), initialized_(false)
  {
    std::string input_topic;
    std::string odometry_topic;
    std::string vision_pose_topic;
    private_nh_.param<std::string>("input_topic", input_topic, "/ov_srvins/odomimu");
    private_nh_.param<std::string>("odometry_topic", odometry_topic, "/uav1/vins/odom");
    private_nh_.param<std::string>("vision_pose_topic", vision_pose_topic,
                                   "/uav1/mavros/vision_pose/pose");
    private_nh_.param<std::string>("frame_id", frame_id_, "map");
    private_nh_.param<std::string>("child_frame_id", child_frame_id_, "base_link");
    double initial_x = 0.0;
    double initial_y = 0.0;
    double initial_z = 0.15;
    private_nh_.param("initial_x", initial_x, initial_x);
    private_nh_.param("initial_y", initial_y, initial_y);
    private_nh_.param("initial_z", initial_z, initial_z);
    private_nh_.param("initial_yaw", initial_yaw_, 0.0);
    initial_position_ = Eigen::Vector3d(initial_x, initial_y, initial_z);

    odometry_pub_ = nh_.advertise<nav_msgs::Odometry>(odometry_topic, 10);
    vision_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(vision_pose_topic, 10);
    odometry_sub_ = nh_.subscribe(input_topic, 20, &VinsOdometryAdapter::odometryCallback, this);
    ROS_INFO("VIO adapter: %s -> %s and %s", input_topic.c_str(), odometry_topic.c_str(),
             vision_pose_topic.c_str());
  }

private:
  static bool finiteVector(const Eigen::Vector3d& value)
  {
    return value.allFinite();
  }

  static void rotateCovariance(const boost::array<double, 36>& input,
                               boost::array<double, 36>& output,
                               const Eigen::Matrix3d& rotation)
  {
    Eigen::Matrix<double, 6, 6> covariance;
    Eigen::Matrix<double, 6, 6> transform = Eigen::Matrix<double, 6, 6>::Zero();
    for (int row = 0; row < 6; ++row)
    {
      for (int column = 0; column < 6; ++column)
        covariance(row, column) = input[row * 6 + column];
    }
    transform.block<3, 3>(0, 0) = rotation;
    transform.block<3, 3>(3, 3) = rotation;
    covariance = transform * covariance * transform.transpose();
    for (int row = 0; row < 6; ++row)
    {
      for (int column = 0; column < 6; ++column)
        output[row * 6 + column] = covariance(row, column);
    }
  }

  void odometryCallback(const nav_msgs::Odometry::ConstPtr& message)
  {
    const auto& pose = message->pose.pose;
    Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
    Eigen::Vector3d velocity(message->twist.twist.linear.x,
                             message->twist.twist.linear.y,
                             message->twist.twist.linear.z);
    Eigen::Quaterniond orientation(pose.orientation.w, pose.orientation.x,
                                   pose.orientation.y, pose.orientation.z);
    if (!finiteVector(position) || !finiteVector(velocity) || !orientation.coeffs().allFinite() ||
        orientation.squaredNorm() < 1e-12)
    {
      ROS_WARN_THROTTLE(2.0, "Ignoring sqrtVINS odometry with invalid pose, twist, or quaternion");
      return;
    }
    orientation.normalize();

    if (!initialized_)
    {
      first_position_ = position;
      const Eigen::Quaterniond target_orientation(
          Eigen::AngleAxisd(initial_yaw_, Eigen::Vector3d::UnitZ()));
      alignment_ = target_orientation * orientation.inverse();
      alignment_.normalize();
      initialized_ = true;
      ROS_INFO("Aligned sqrtVINS first pose to configured map origin and yaw");
    }

    const Eigen::Matrix3d rotation = alignment_.toRotationMatrix();
    const Eigen::Vector3d aligned_position =
        rotation * (position - first_position_) + initial_position_;
    const Eigen::Quaterniond aligned_orientation = (alignment_ * orientation).normalized();
    const Eigen::Vector3d aligned_velocity = rotation * velocity;
    if (!finiteVector(aligned_position) || !finiteVector(aligned_velocity) ||
        !aligned_orientation.coeffs().allFinite())
    {
      ROS_WARN_THROTTLE(2.0, "Ignoring aligned sqrtVINS odometry with non-finite values");
      return;
    }

    nav_msgs::Odometry odometry = *message;
    odometry.header.frame_id = frame_id_;
    odometry.child_frame_id = child_frame_id_;
    odometry.pose.pose.position.x = aligned_position.x();
    odometry.pose.pose.position.y = aligned_position.y();
    odometry.pose.pose.position.z = aligned_position.z();
    odometry.pose.pose.orientation.x = aligned_orientation.x();
    odometry.pose.pose.orientation.y = aligned_orientation.y();
    odometry.pose.pose.orientation.z = aligned_orientation.z();
    odometry.pose.pose.orientation.w = aligned_orientation.w();
    rotateCovariance(message->pose.covariance, odometry.pose.covariance, rotation);
    odometry.twist.twist.linear.x = aligned_velocity.x();
    odometry.twist.twist.linear.y = aligned_velocity.y();
    odometry.twist.twist.linear.z = aligned_velocity.z();
    odometry.twist.twist.angular.x = 0.0;
    odometry.twist.twist.angular.y = 0.0;
    odometry.twist.twist.angular.z = 0.0;
    rotateCovariance(message->twist.covariance, odometry.twist.covariance, rotation);
    odometry_pub_.publish(odometry);

    geometry_msgs::PoseStamped vision_pose;
    vision_pose.header = odometry.header;
    vision_pose.pose = odometry.pose.pose;
    vision_pose_pub_.publish(vision_pose);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber odometry_sub_;
  ros::Publisher odometry_pub_;
  ros::Publisher vision_pose_pub_;
  std::string frame_id_;
  std::string child_frame_id_;
  Eigen::Vector3d initial_position_{0.0, 0.0, 0.15};
  Eigen::Vector3d first_position_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond alignment_{Eigen::Quaterniond::Identity()};
  double initial_yaw_ = 0.0;
  bool initialized_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vins_odometry_adapter");
  VinsOdometryAdapter adapter;
  ros::spin();
  return 0;
}
