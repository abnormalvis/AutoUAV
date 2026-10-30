#!/usr/bin/env bash
# Build and test armor_perception inside the project's Noetic container.
#
# The host has no ROS installation, so this is the only way to compile here.
#
#   ./build.sh              build the package and the node
#   ./build.sh --tests      build, then run the unit tests
#   ./build.sh --chain      build, then run the end-to-end synthetic chain check
#   ./build.sh --shell      interactive shell in a workspace with this package
#
# The package is built in an isolated catkin workspace inside the container, not
# in rm_drone_ros itself. The full workspace does not configure in this image:
# communication/apm_d430_bridge requires `mavros_extras`, which is not
# installed. armor_perception depends only on core ROS packages, so an isolated
# workspace is sufficient and much faster.
#
# To build it as part of rm_drone_ros, install the missing dependency first:
#   apt-get install ros-noetic-mavros-extras
# then run catkin_make in the usual way.
set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="${ARMOR_DOCKER_IMAGE:-common-ros-noetic-docker-main-ros1:latest}"
MODE="${1:-build}"

# The package is mounted read-only and copied into a scratch workspace inside
# the container, so nothing here depends on host file ownership.
common=(--rm -v "$PKG_DIR":/pkg:ro "$IMAGE")

build_ws='
  set -e
  source /opt/ros/noetic/setup.bash
  rm -rf /tmp/ws && mkdir -p /tmp/ws/src
  cp -r /pkg /tmp/ws/src/armor_perception
  cd /tmp/ws
  catkin_make -DCMAKE_BUILD_TYPE=Release
  cd build && make tests -j"$(nproc)" >/dev/null
'

case "$MODE" in
  --shell)
    exec docker run -it "${common[@]}" bash -lc "
      source /opt/ros/noetic/setup.bash
      $build_ws
      source /tmp/ws/devel/setup.bash
      cd /tmp/ws
      exec bash
    "
    ;;
  --tests)
    docker run "${common[@]}" bash -lc "
      $build_ws
      source /tmp/ws/devel/setup.bash
      rc=0
      for t in test_odometry_buffer test_static_target test_armor_solver test_observation_point; do
        echo \"=== \$t\"
        /tmp/ws/devel/lib/armor_perception/\$t || rc=1
      done
      exit \$rc
    "
    ;;
  --chain)
    docker run "${common[@]}" bash -lc "
      $build_ws
      source /tmp/ws/devel/setup.bash
      roscore >/tmp/roscore.log 2>&1 &
      sleep 5
      rosrun armor_perception armor_perception_node \
        _detector_config:=/tmp/ws/src/armor_perception/config/uav_sitl.yaml >/tmp/node.log 2>&1 &
      sleep 2
      rc=0
      rosrun armor_perception synthetic_chain_test.py || rc=\$?
      echo '--- node log ---'
      grep -E 'WARN|ERROR' /tmp/node.log | head -5 || true
      exit \$rc
    "
    ;;
  *)
    docker run "${common[@]}" bash -lc "$build_ws"
    ;;
esac
