#include <gtest/gtest.h>

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vins_mavros_bridge_test", ros::init_options::AnonymousName);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
