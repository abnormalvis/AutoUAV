cmake_minimum_required(VERSION 3.0.2)
project(armor_perception)

# The vendored sp_vision sources need C++17 (std::filesystem, structured
# bindings). Set it per target rather than globally, so nothing leaks into the
# rest of the catkin build.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -Wno-unused-parameter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  cv_bridge
)

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs calib3d dnn)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_message_files(FILES
  ArmorTarget.msg
  ArmorTargetArray.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs message_runtime
  INCLUDE_DIRS include
)

include_directories(
  include
  # The vendored tree keeps its upstream include roots, so the copies can keep
  # their original `#include "tasks/auto_aim/armor.hpp"` paths unchanged.
  thirdparty/sp_vision
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

set(VENDORED_SOURCES
  thirdparty/sp_vision/tasks/auto_aim/armor.cpp
  thirdparty/sp_vision/tasks/auto_aim/classifier.cpp
  thirdparty/sp_vision/tasks/auto_aim/detector.cpp
  thirdparty/sp_vision/tools/extended_kalman_filter.cpp
  thirdparty/sp_vision/tools/img_tools.cpp
  thirdparty/sp_vision/tools/math_tools.cpp
)

set(PERCEPTION_SOURCES
  src/armor_solver.cpp
  src/observation_point.cpp
  src/odometry_buffer.cpp
  src/static_target.cpp
  src/tracker.cpp
)

add_library(${PROJECT_NAME}_core STATIC ${VENDORED_SOURCES} ${PERCEPTION_SOURCES})

add_dependencies(${PROJECT_NAME}_core
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_core
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  yaml-cpp
)

add_executable(armor_perception_node src/armor_perception_node.cpp)
add_dependencies(armor_perception_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(armor_perception_node ${PROJECT_NAME}_core)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_odometry_buffer test/test_odometry_buffer.cpp test/test_main.cpp)
  if(TARGET test_odometry_buffer)
    target_link_libraries(test_odometry_buffer ${PROJECT_NAME}_core)
  endif()

  catkin_add_gtest(test_static_target test/test_static_target.cpp test/test_main.cpp)
  if(TARGET test_static_target)
    target_link_libraries(test_static_target ${PROJECT_NAME}_core)
  endif()

  catkin_add_gtest(test_armor_solver test/test_armor_solver.cpp test/test_main.cpp)
  if(TARGET test_armor_solver)
    target_link_libraries(test_armor_solver ${PROJECT_NAME}_core)
  endif()

  catkin_add_gtest(test_observation_point test/test_observation_point.cpp test/test_main.cpp)
  if(TARGET test_observation_point)
    target_link_libraries(test_observation_point ${PROJECT_NAME}_core)
  endif()
endif()

install(TARGETS armor_perception_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/evaluate_pose.py
  scripts/synthetic_chain_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
