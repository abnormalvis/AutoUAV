cmake_minimum_required(VERSION 3.0.2)
project(vins_mavros_bridge)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  message_generation
  nav_msgs
  std_msgs
  tf2_ros
)

find_package(Eigen3 REQUIRED)

add_message_files(FILES
  BridgeStatus.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs message_runtime nav_msgs std_msgs tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(vins_mavros_bridge_node
  src/bridge.cpp
  src/bridge_node.cpp
)

add_dependencies(vins_mavros_bridge_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(vins_mavros_bridge_node ${catkin_LIBRARIES})

install(TARGETS vins_mavros_bridge_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/synthetic_odom_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
