#include <ros/ros.h>

#include "vins_mavros_bridge/bridge.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vins_mavros_bridge");

  // Constructed after ros::init so ROS_FATAL in the constructor reaches the
  // console; the node shuts itself down on an invalid configuration instead of
  // running with a silently wrong feedback chain.
  vins_mavros_bridge::Bridge bridge;
  ros::spin();
  return 0;
}
