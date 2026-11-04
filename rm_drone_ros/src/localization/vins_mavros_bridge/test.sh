#!/usr/bin/env bash
# Build and test vins_mavros_bridge inside the project's Noetic container.
#
#   ./test.sh              build the package and the node
#   ./test.sh --unit       build, then run the unit tests
#   ./test.sh --frames     build, then run the end-to-end frame checks
#   ./test.sh --shell      interactive shell in a workspace with this package
#
# The host has no ROS installation, so this is the only way to compile here.
#
# The package is built in an isolated catkin workspace inside the container
# rather than in rm_drone_ros. The full workspace does not configure in this
# image because communication/apm_d430_bridge requires `mavros_extras`, which
# is not installed. This package depends only on core ROS packages, so an
# isolated workspace is sufficient and much faster.
#
# --frames is the check that matters: it starts the node with each adapter's
# parameters and asserts on the velocity that comes out. A twist-frame mistake
# is invisible in the pose stream, so a pose-only test cannot catch it.
set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="${BRIDGE_DOCKER_IMAGE:-common-ros-noetic-docker-main-ros1:latest}"
MODE="${1:-build}"

common=(--rm -v "$PKG_DIR":/pkg:ro "$IMAGE")

build_ws='
  set -e
  source /opt/ros/noetic/setup.bash
  rm -rf /tmp/ws && mkdir -p /tmp/ws/src
  cp -r /pkg /tmp/ws/src/vins_mavros_bridge
  cd /tmp/ws
  catkin_make -DCMAKE_BUILD_TYPE=Release
  source /tmp/ws/devel/setup.bash
'

case "$MODE" in
  --shell)
    exec docker run -it "${common[@]}" bash -lc "$build_ws; cd /tmp/ws; exec bash"
    ;;
  --unit)
    docker run "${common[@]}" bash -lc "
      $build_ws
      cd /tmp/ws/build && make tests -j\"\$(nproc)\" >/dev/null
      /tmp/ws/devel/lib/vins_mavros_bridge/test_pose_derivative
    "
    ;;
  --frames)
    docker run "${common[@]}" bash -lc "
      $build_ws
      roscore >/tmp/roscore.log 2>&1 &
      sleep 5
      rc=0
      run_case() {
        name=\"\$1\"; shift
        rosrun vins_mavros_bridge vins_mavros_bridge_node \
          _publish_vision_pose:=false _publish_mavros_odometry:=false \
          _consistency_check:=false \"\$@\" >/tmp/bridge_\$name.log 2>&1 &
        node=\$!
        sleep 2
        rosrun vins_mavros_bridge frame_convention_check.py \"\$name\" || rc=1
        kill \$node 2>/dev/null || true
        wait \$node 2>/dev/null || true
        sleep 1
      }
      run_case openvins \
        _input_mode:=odometry _input_topic:=/check/odom _input_child_frame:=imu \
        _input_twist_frame:=body _input_twist_angular:=false
      run_case vins_fusion \
        _input_mode:=odometry _input_topic:=/check/odom _input_child_frame:=body \
        _input_twist_frame:=world _input_twist_angular:=false _check_child_frame:=false
      run_case derived \
        _input_mode:=pose_with_covariance _input_topic:=/check/pose _input_child_frame:=imu \
        _derive_twist:=true _input_twist_angular:=false
      exit \$rc
    "
    ;;
  *)
    docker run "${common[@]}" bash -lc "$build_ws"
    ;;
esac
