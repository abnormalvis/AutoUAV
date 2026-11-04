cmake_minimum_required(VERSION 3.0.2)
project(simple_offboard_forward)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  rospy
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/forward_30cm_offboard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
