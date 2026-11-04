// #  Copyright © 2023 ROBOMASTER All Rights Reserved.
// #  You may use, distribute and modify this code under the
// #  terms of the MIT license, which unfortunately won't be
// #  written for another century.

#include "../../include/d430_slam.h"

#include <ros/package.h>


realsense::Slam::Slam(const ros::NodeHandle& nh):nh_(nh)
{
    slam_count_ = 0;

    // The paths used to be hardcoded to /home/nvidia/... which made the package
    // rebuildable only on the machine it was written on. They are parameters
    // now; the settings file defaults to the one shipped with this package.
    ros::NodeHandle pnh("~");
    pnh.param<std::string>(
        "vocabulary_path", vocabulary_path_,
        "/home/nvidia/Softwares/ORB_SLAM3/Vocabulary/ORBvoc.txt");
    pnh.param<std::string>("settings_path", settings_path_, "");
    if (settings_path_.empty()) {
        settings_path_ = ros::package::getPath("d430_slam") + "/config/settings.yaml";
    }
    pnh.param<std::string>("pose_topic", pose_topic_, "/d430_camera/pose");
    pnh.param<std::string>("frame_id", frame_id_, "map");

    d430_camera_ptr_ = std::make_unique<realsense::D430>();
    orbslam_ptr_ = std::make_unique<ORB_SLAM3::System>(
        vocabulary_path_, settings_path_, ORB_SLAM3::System::STEREO, false);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic_, 1);
    timer = nh_.createTimer(ros::Duration(0.077), std::bind(&Slam::image_grap, this));

    ROS_INFO("[d430_slam] vocabulary=%s settings=%s pose_topic=%s",
             vocabulary_path_.c_str(), settings_path_.c_str(), pose_topic_.c_str());
}

realsense::Slam::~Slam()
{
    timer.stop();
    timer.~Timer();
    d430_camera_ptr_->~D430();
    
}

void realsense::Slam::image_grap()
{
    auto res = d430_camera_ptr_->get_stereo_images(left_img_, right_img_, time_stamp_);
    // The camera driver fills these from a background thread; before the first
    // frameset they are empty and handing them to ORB-SLAM3 is a crash, not a
    // dropped frame.
    if(!res || left_img_.empty() || right_img_.empty()) return;
    {
        // uint64 nanoseconds -> seconds as a double. The previous
        // `time_stamp_/1000000000` was an integer division, so every frame of a
        // second carried the same stamp and ORB-SLAM3 saw a 1 Hz sensor.
        const double timestamp_seconds = 1e-9 * static_cast<double>(time_stamp_);
        Sophus::SE3f pose =  orbslam_ptr_->TrackStereo(left_img_, right_img_, timestamp_seconds);
        Eigen::Matrix3f Rw2c = pose.matrix3x4().block(0, 0, 3, 3).transpose();//w is the orbslam w;
        Eigen::Vector3f Pw2c = -1*Rw2c*pose.matrix3x4().col(3);
        Eigen::Quaternionf Qw2c(Rw2c);
        Eigen::Quaternionf Qflu2w(0.5, -0.50, 0.50, -0.50);
        Eigen::Vector3f p = Qflu2w*Pw2c;
        Eigen::Quaternionf q = Qflu2w*Qw2c*Qflu2w.inverse();
        pose_.header.frame_id = frame_id_;
        pose_.header.seq = slam_count_;
        // Full timestamp, not a sec/nsec split. `nsec = time_stamp_` put a
        // nanosecond count above 1e9 into a field that must stay below it, so
        // ros::Time::toSec() returned roughly twice the real time and every
        // downstream staleness or association check saw a stamp in the future.
        pose_.header.stamp = ros::Time().fromNSec(time_stamp_);
        pose_.pose.orientation.w = q.w();
        pose_.pose.orientation.x = q.x();
        pose_.pose.orientation.y = q.y();
        pose_.pose.orientation.z = q.z();
        pose_.pose.position.x = p[0];
        pose_.pose.position.y = p[1];
        pose_.pose.position.z = p[2];
        pose_pub_.publish(pose_);
        slam_count_ += 1;
    }
}

