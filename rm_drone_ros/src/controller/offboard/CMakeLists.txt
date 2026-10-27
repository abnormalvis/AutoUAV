#  Copyright © 2023 ROBOMASTER All Rights Reserved.
#  You may use, distribute and modify this code under the
#  terms of the MIT license, which unfortunately won't be
#  written for another century.

cmake_minimum_required(VERSION 3.0.2)
project(offboard)

add_definitions(-std=c++14)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
#   dynamic_reconfigure
  tf
#   pcl_ros
  mavros
  mavros_msgs
  mavlink
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  sensor_msgs
  std_msgs
)
# find_package(PCL 1.7 REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs mavros_msgs geometry_msgs nav_msgs quadrotor_msgs sensor_msgs message_runtime tf
#  DEPENDS system_lib
)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
#   ${PCL_INCLUDE_DIRS}
#   ${YAML_CPP_INCLUDE_DIR}
)

add_library(offboard_rover    src/node/offboard_rover.cpp)


add_dependencies(
  offboard_rover ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${EIGEN3_LIBS}
)

add_library(offboard_postest     src/node/offboard_postest.cpp)


add_dependencies(
  offboard_postest ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${EIGEN3_LIBS}
)

add_library(offboard_veltest    src/node/offboard_veltest.cpp  utils/pid.cpp)


add_dependencies(
  offboard_veltest ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${EIGEN3_LIBS}
)

add_executable(offboard_test_node src/node/offboard_test_node.cpp)

target_link_libraries(offboard_test_node
  offboard_rover
  offboard_postest
  offboard_veltest
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES})

add_executable(offboard_planner_tracker_node
  src/node/offboard_planner_tracker_node.cpp
  src/node/offboard_planner_tracker.cpp
)

add_dependencies(offboard_planner_tracker_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(offboard_planner_tracker_node
  ${catkin_LIBRARIES}
)

add_executable(vins_odometry_adapter src/node/vins_odometry_adapter.cpp)
add_dependencies(vins_odometry_adapter ${catkin_EXPORTED_TARGETS})
target_link_libraries(vins_odometry_adapter ${catkin_LIBRARIES})

install(TARGETS offboard_planner_tracker_node vins_odometry_adapter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

