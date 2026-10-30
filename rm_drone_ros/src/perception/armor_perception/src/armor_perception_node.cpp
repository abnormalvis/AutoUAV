#include <algorithm>
#include <cmath>
#include <list>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include "armor_perception/ArmorTarget.h"
#include "armor_perception/ArmorTargetArray.h"

#include "armor_perception/armor_solver.hpp"
#include "armor_perception/frames.hpp"
#include "armor_perception/observation_point.hpp"
#include "armor_perception/odometry_buffer.hpp"
#include "armor_perception/static_target.hpp"
#include "armor_perception/tracker.hpp"
#include "tasks/auto_aim/detector.hpp"

namespace armor_perception
{

class ArmorPerceptionNode
{
public:
  ArmorPerceptionNode() : nh_(), pnh_("~")
  {
    loadParams();

    detector_.reset(new auto_aim::Detector(detector_config_path_, false));

    odom_sub_ = nh_.subscribe(odom_topic_, 50, &ArmorPerceptionNode::odomCb, this);
    valid_sub_ = nh_.subscribe(valid_topic_, 5, &ArmorPerceptionNode::validCb, this);
    info_sub_ = nh_.subscribe(info_topic_, 5, &ArmorPerceptionNode::infoCb, this);
    image_sub_ = nh_.subscribe(image_topic_, 5, &ArmorPerceptionNode::imageCb, this);

    targets_pub_ = nh_.advertise<ArmorTargetArray>(targets_topic_, 5);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_topic_, 5);
    obs_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(obs_topic_, 5);
    state_pub_ = nh_.advertise<std_msgs::String>(state_topic_, 5, true);
    if (publish_debug_) debug_pub_ = nh_.advertise<sensor_msgs::Image>(debug_topic_, 1);

    ROS_INFO(
      "[armor_perception] image=%s odom=%s standoff=%.2f m", image_topic_.c_str(),
      odom_topic_.c_str(), observation_.params().standoff);
    if (!solver_.hasIntrinsics()) {
      ROS_WARN("[armor_perception] no intrinsics yet, waiting for %s", info_topic_.c_str());
    }
  }

private:
  // ------------------------------------------------------------------ setup
  void loadParams()
  {
    pnh_.param<std::string>("detector_config", detector_config_path_, "");
    if (detector_config_path_.empty()) {
      throw std::runtime_error("~detector_config is required (path to the detector YAML)");
    }

    pnh_.param<std::string>("image_topic", image_topic_, "/camera/left/image_raw");
    pnh_.param<std::string>("camera_info_topic", info_topic_, "/camera/left/camera_info");
    pnh_.param<std::string>("odom_topic", odom_topic_, "/localization/odometry");
    pnh_.param<std::string>("valid_topic", valid_topic_, "/vins_mavros_bridge/valid");

    pnh_.param<std::string>("targets_topic", targets_topic_, "/perception/armor_targets");
    pnh_.param<std::string>("pose_topic", pose_topic_, "/perception/target_pose");
    pnh_.param<std::string>("observation_topic", obs_topic_, "/perception/observation_point");
    pnh_.param<std::string>("state_topic", state_topic_, "/perception/task_state");
    pnh_.param<std::string>("debug_topic", debug_topic_, "/perception/debug_image");
    pnh_.param<bool>("publish_debug", publish_debug_, false);

    loadExtrinsics();
    loadGeometry();
    loadFilterParams();
  }

  // T_base_camera. `cam_xyz` is the camera origin in base_link; the camera axes
  // come from `cam_rpy_deg` (degrees, ZYX order) or a full `R_base_camera`.
  void loadExtrinsics()
  {
    std::vector<double> cam_xyz{0.1, 0.0, 0.0};
    pnh_.getParam("cam_xyz", cam_xyz);
    if (cam_xyz.size() != 3) throw std::runtime_error("~cam_xyz must have 3 elements");

    Eigen::Matrix3d R_body_camera = Eigen::Matrix3d::Identity();
    std::vector<double> flat;
    if (pnh_.getParam("R_base_camera", flat) && flat.size() == 9) {
      R_body_camera = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(flat.data());
    } else {
      std::vector<double> rpy_deg{0.0, 0.0, 0.0};
      pnh_.getParam("cam_rpy_deg", rpy_deg);
      if (rpy_deg.size() != 3) throw std::runtime_error("~cam_rpy_deg must have 3 elements");
      const double roll = rpy_deg[0] * M_PI / 180.0;
      const double pitch = rpy_deg[1] * M_PI / 180.0;
      const double yaw = rpy_deg[2] * M_PI / 180.0;
      R_body_camera = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
                        .toRotationMatrix();
    }

    // The Gazebo camera_link uses the sensor convention (+X forward, +Y left,
    // +Z up); OpenCV PnP needs the optical convention (+Z forward, +X right,
    // +Y down). See frames.hpp -- getting this wrong leaves detection looking
    // perfect while every pose is rotated.
    T_base_camera_ =
      buildTBaseOptical(Eigen::Vector3d(cam_xyz[0], cam_xyz[1], cam_xyz[2]), R_body_camera);
  }

  void loadGeometry()
  {
    ArmorDimensions dims;
    pnh_.param("lightbar_length", dims.lightbar_length, dims.lightbar_length);
    pnh_.param("small_armor_width", dims.small_armor_width, dims.small_armor_width);
    pnh_.param("big_armor_width", dims.big_armor_width, dims.big_armor_width);
    solver_.setDimensions(dims);

    // YAML intrinsics are a fallback only; camera_info overrides them.
    std::vector<double> k;
    if (pnh_.getParam("camera_matrix", k) && k.size() == 9) {
      cv::Mat K(3, 3, CV_64F, k.data());
      cv::Mat D = cv::Mat::zeros(1, 5, CV_64F);
      if (solver_.setCameraIntrinsics(K.clone(), D)) {
        intrinsics_fx_ = K.at<double>(0, 0);
        ROS_WARN("[armor_perception] using YAML intrinsics until camera_info arrives");
      }
    }
  }

  void loadFilterParams()
  {
    double max_odom_age = 0.05;
    pnh_.param("max_odom_age", max_odom_age, max_odom_age);
    odom_buffer_.setMaxAge(max_odom_age);

    int capacity = 200;
    pnh_.param("odom_capacity", capacity, capacity);
    odom_buffer_.setCapacity(static_cast<std::size_t>(std::max(10, capacity)));

    pnh_.param<std::string>("fallback_odom_frame", odom_frame_, "world");

    Tracker::Params tp;
    pnh_.param("min_detect_count", tp.min_detect_count, tp.min_detect_count);
    pnh_.param("max_temp_lost_count", tp.max_temp_lost_count, tp.max_temp_lost_count);
    tracker_.setParams(tp);

    StaticTarget::Params sp;
    pnh_.param("accel_noise", sp.accel_noise, sp.accel_noise);
    pnh_.param("min_measurement_std", sp.min_measurement_std, sp.min_measurement_std);
    pnh_.param("yaw_time_constant", sp.yaw_time_constant, sp.yaw_time_constant);
    pnh_.param("max_consecutive_jumps", sp.max_consecutive_jumps, sp.max_consecutive_jumps);
    pnh_.param("jump_threshold", sp.jump_threshold, sp.jump_threshold);
    tracker_.setTargetParams(sp);

    double assoc = 0.5;
    pnh_.param("association_radius", assoc, assoc);
    tracker_.setAssociationRadius(assoc);

    ObservationPointGenerator::Params op;
    pnh_.param("standoff", op.standoff, op.standoff);
    pnh_.param("min_standoff", op.min_standoff, op.min_standoff);
    pnh_.param("max_standoff", op.max_standoff, op.max_standoff);
    pnh_.param("max_normal_range", op.max_normal_range, op.max_normal_range);
    observation_.setParams(op);

    pnh_.param("observe_position_std", observe_position_std_, observe_position_std_);
    pnh_.param("normal_trusted_std", normal_trusted_std_, normal_trusted_std_);
  }

  // -------------------------------------------------------------- callbacks
  void odomCb(const nav_msgs::OdometryConstPtr & msg) { odom_buffer_.push(*msg); }

  void validCb(const std_msgs::BoolConstPtr & msg) { vio_valid_ = msg->data; }

  void infoCb(const sensor_msgs::CameraInfoConstPtr & msg)
  {
    if (msg->width == 0 || msg->height == 0) return;
    if (intrinsics_from_info_) return;

    cv::Mat K(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) K.at<double>(r, c) = msg->K[r * 3 + c];

    cv::Mat D;
    if (!msg->D.empty()) {
      D = cv::Mat(1, static_cast<int>(msg->D.size()), CV_64F);
      for (std::size_t i = 0; i < msg->D.size(); ++i) {
        D.at<double>(0, static_cast<int>(i)) = msg->D[i];
      }
    } else {
      D = cv::Mat::zeros(1, 5, CV_64F);
    }

    if (!solver_.setCameraIntrinsics(K, D)) {
      ROS_ERROR_THROTTLE(5.0, "[armor_perception] camera_info intrinsics are degenerate");
      return;
    }

    intrinsics_from_info_ = true;
    intrinsics_fx_ = K.at<double>(0, 0);
    ROS_INFO(
      "[armor_perception] intrinsics from camera_info: fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
      K.at<double>(0, 0), K.at<double>(1, 1), K.at<double>(0, 2), K.at<double>(1, 2));
  }

  void imageCb(const sensor_msgs::ImageConstPtr & msg)
  {
    if (!solver_.hasIntrinsics()) return;  // warned at startup

    if (!vio_valid_) {
      ROS_WARN_THROTTLE(2.0, "[armor_perception] dropping frame: VIO not valid");
      return;
    }

    const double stamp = msg->header.stamp.toSec();
    if (!(stamp > 0.0)) {
      ROS_WARN_THROTTLE(2.0, "[armor_perception] dropping frame: image stamp is zero");
      return;
    }

    Eigen::Isometry3d T_world_base;
    std::string error;
    if (!odom_buffer_.lookup(stamp, T_world_base, error)) {
      ROS_WARN_THROTTLE(2.0, "[armor_perception] dropping frame: %s", error.c_str());
      return;
    }

    cv::Mat bgr;
    try {
      bgr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
    } catch (const cv_bridge::Exception & e) {
      ROS_ERROR_THROTTLE(2.0, "[armor_perception] cv_bridge: %s", e.what());
      return;
    }
    if (bgr.empty()) return;

    // Pose the camera at the image timestamp. This is the ego-motion
    // compensation: everything downstream is in the world frame, so a moving
    // drone does not look like a moving target.
    solver_.set_T_world_camera(T_world_base * T_base_camera_);

    ++frame_count_;
    cv::Mat debug;
    auto armors =
      detector_->detect(bgr, static_cast<int>(frame_count_), publish_debug_ ? &debug : nullptr);

    std::list<auto_aim::Armor> solved;
    double worst_reproj = 0.0;
    for (auto & armor : armors) {
      if (!solver_.solve(armor)) continue;
      worst_reproj = std::max(worst_reproj, solver_.lastReprojectionError());
      solved.push_back(armor);
    }

    const cv::Point2f img_center(bgr.cols / 2.0f, bgr.rows / 2.0f);
    tracker_.track(solved, stamp, img_center, measurementStd(solved, worst_reproj));

    publish(msg->header.stamp, solved, debug);
  }

  // Per-axis measurement standard deviation for the filter, from the
  // reprojection error and the observed range. A fixed constant would be wrong
  // in both directions: too loose up close, far too tight at range.
  //
  // Lateral error ~ range * err_px / fx. Depth error of a planar target
  // ~ range^2 * err_px / (fx * plate_width), growing quadratically -- the reason
  // monocular range is only usable at short distance.
  Eigen::Vector3d measurementStd(
    const std::list<auto_aim::Armor> & armors, double worst_reproj) const
  {
    if (armors.empty() || !(intrinsics_fx_ > 0.0)) return Eigen::Vector3d::Constant(0.05);

    const auto & armor = armors.front();
    const double range = (armor.xyz_in_world - solver_.T_world_camera().translation()).norm();
    if (!(range > 1e-3)) return Eigen::Vector3d::Constant(0.05);

    const double err_px = std::max(0.5, worst_reproj);
    const double width = (armor.type == auto_aim::ArmorType::big)
                           ? solver_.dimensions().big_armor_width
                           : solver_.dimensions().small_armor_width;

    const double lateral = range * err_px / intrinsics_fx_;
    const double depth = range * range * err_px / (intrinsics_fx_ * width);

    const double s = std::min(std::max(std::max(lateral, depth), 0.002), 0.5);
    return Eigen::Vector3d::Constant(s);
  }

  // ---------------------------------------------------------------- publish
  void publish(
    const ros::Time & stamp, const std::list<auto_aim::Armor> & armors, const cv::Mat & debug)
  {
    const std::string frame =
      odom_buffer_.frameId().empty() ? odom_frame_ : odom_buffer_.frameId();

    ArmorTargetArray array;
    array.header.stamp = stamp;
    array.header.frame_id = frame;
    array.valid = tracker_.hasTarget();

    const uint8_t state = toMsgState(tracker_.state());
    uint8_t id = 0;
    for (const auto & armor : armors) {
      ArmorTarget target;
      target.header = array.header;
      target.id = id++;
      target.armor_name = static_cast<uint8_t>(armor.name);
      target.confidence = static_cast<float>(armor.confidence);

      // PoseWithCovariance carries no header of its own; the enclosing
      // ArmorTarget::header already names the world frame.
      target.pose.pose.position.x = armor.xyz_in_world.x();
      target.pose.pose.position.y = armor.xyz_in_world.y();
      target.pose.pose.position.z = armor.xyz_in_world.z();

      Eigen::Quaterniond q(armor.R_armor2world);
      q.normalize();
      target.pose.pose.orientation.w = q.w();
      target.pose.pose.orientation.x = q.x();
      target.pose.pose.orientation.y = q.y();
      target.pose.pose.orientation.z = q.z();

      target.distance = static_cast<float>(
        (armor.xyz_in_world - solver_.T_world_camera().translation()).norm());
      target.tracking_state = state;

      array.targets.push_back(target);
    }
    targets_pub_.publish(array);

    if (tracker_.hasTarget()) publishTracked(stamp, frame);
    publishTaskState();

    if (publish_debug_ && !debug.empty() && debug_pub_) {
      cv_bridge::CvImage out;
      out.header = array.header;
      out.encoding = sensor_msgs::image_encodings::BGR8;
      out.image = debug;
      debug_pub_.publish(out.toImageMsg());
    }
  }

  void publishTracked(const ros::Time & stamp, const std::string & frame)
  {
    const Eigen::Vector3d p = tracker_.position();
    const Eigen::Matrix3d R = tracker_.orientation();
    const Eigen::Matrix3d P = tracker_.target().positionCovariance();
    const Eigen::Vector3d cam = solver_.T_world_camera().translation();

    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = frame;
    pose.pose.pose.position.x = p.x();
    pose.pose.pose.position.y = p.y();
    pose.pose.pose.position.z = p.z();

    Eigen::Quaterniond q(R);
    q.normalize();
    pose.pose.pose.orientation.w = q.w();
    pose.pose.pose.orientation.x = q.x();
    pose.pose.pose.orientation.y = q.y();
    pose.pose.pose.orientation.z = q.z();

    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) pose.pose.covariance[i * 6 + j] = P(i, j);
    pose_pub_.publish(pose);

    // The plate normal is only trusted when the position estimate is tight and
    // the plate is close enough for PnP yaw to mean anything; otherwise the
    // generator falls back to backing off along the current bearing.
    const double range = (p - cam).norm();
    const bool normal_trusted = tracker_.target().positionStd() < normal_trusted_std_ &&
                                range < observation_.params().max_normal_range;

    const auto point = observation_.generate(p, R, cam, normal_trusted);

    geometry_msgs::PoseStamped obs;
    obs.header.stamp = stamp;
    obs.header.frame_id = frame;
    obs.pose.position.x = point.position.x();
    obs.pose.position.y = point.position.y();
    obs.pose.position.z = point.position.z();

    const Eigen::Quaterniond qyaw(Eigen::AngleAxisd(point.yaw, Eigen::Vector3d::UnitZ()));
    obs.pose.orientation.w = qyaw.w();
    obs.pose.orientation.x = qyaw.x();
    obs.pose.orientation.y = qyaw.y();
    obs.pose.orientation.z = qyaw.z();
    obs_pub_.publish(obs);
  }

  // Perception reports state; it never commands flight. The mission layer
  // decides whether to fly the cruise waypoints, the observation point, or to
  // hold in place (plan §4.5).
  void publishTaskState()
  {
    std_msgs::String msg;
    switch (tracker_.state()) {
      case Tracker::State::LOST:
        // Never acquired, or lost for good: nothing to observe, the mission
        // layer may resume cruising.
        msg.data = "CRUISE";
        break;
      case Tracker::State::DETECTING:
        msg.data = "CAPTURE";
        break;
      case Tracker::State::TRACKING:
        msg.data =
          (tracker_.target().positionStd() < observe_position_std_) ? "OBSERVE" : "CAPTURE";
        break;
      case Tracker::State::TEMP_LOST:
        // Hold at the last known position. Do not fall back to cruise: that
        // oscillates between phases every time the plate blinks.
        msg.data = "LOST_HOLD";
        break;
    }
    state_pub_.publish(msg);
  }

  static uint8_t toMsgState(Tracker::State s)
  {
    switch (s) {
      case Tracker::State::LOST:
        return ArmorTarget::LOST;
      case Tracker::State::DETECTING:
        return ArmorTarget::DETECTING;
      case Tracker::State::TRACKING:
        return ArmorTarget::TRACKING;
      case Tracker::State::TEMP_LOST:
        return ArmorTarget::TEMP_LOST;
    }
    return ArmorTarget::LOST;
  }

  ros::NodeHandle nh_, pnh_;

  std::string detector_config_path_;
  std::string image_topic_, info_topic_, odom_topic_, valid_topic_;
  std::string targets_topic_, pose_topic_, obs_topic_, state_topic_, debug_topic_;
  bool publish_debug_ = false;
  std::string odom_frame_ = "world";

  std::unique_ptr<auto_aim::Detector> detector_;
  ArmorSolver solver_;
  OdometryBuffer odom_buffer_;
  Tracker tracker_;
  ObservationPointGenerator observation_;

  Eigen::Isometry3d T_base_camera_ = Eigen::Isometry3d::Identity();

  bool vio_valid_ = false;
  bool intrinsics_from_info_ = false;
  double intrinsics_fx_ = 0.0;

  double observe_position_std_ = 0.05;
  double normal_trusted_std_ = 0.15;

  long frame_count_ = 0;

  ros::Subscriber odom_sub_, valid_sub_, info_sub_, image_sub_;
  ros::Publisher targets_pub_, pose_pub_, obs_pub_, state_pub_, debug_pub_;
};

}  // namespace armor_perception

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "armor_perception");
  try {
    armor_perception::ArmorPerceptionNode node;
    ros::spin();
  } catch (const std::exception & e) {
    ROS_FATAL("[armor_perception] %s", e.what());
    return 1;
  }
  return 0;
}
