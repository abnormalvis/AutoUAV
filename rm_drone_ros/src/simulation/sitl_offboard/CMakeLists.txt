cmake_minimum_required(VERSION 3.0.2)
project(sitl_offboard)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/keyboard_goal.py
  scripts/camera_pose_bridge.py
  scripts/depth_mm_bridge.py
  scripts/prometheus_truth_bridge.py
  scripts/prometheus_sitl_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/run_px4_sitl.sh
  scripts/run_prometheus_px4_sitl.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch worlds models config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)