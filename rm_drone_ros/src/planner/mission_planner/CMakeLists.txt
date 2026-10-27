cmake_minimum_required(VERSION 3.0.2)
project(mission_planner)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  mavros_msgs
  std_msgs
  std_srvs
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs mavros_msgs std_msgs std_srvs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(mission_manager_node src/mission_manager_node.cpp)
add_dependencies(mission_manager_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(mission_manager_node ${catkin_LIBRARIES})

install(TARGETS mission_manager_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)