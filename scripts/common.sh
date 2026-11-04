#!/usr/bin/env bash
# Shared environment for the on-board start-up scripts. Source it, don't run it.
# Site-specific values go in /etc/rm_drone/rm_drone.env (see systemd/rm_drone.env.example).

RM_ENV_FILE="${RM_ENV_FILE:-/etc/rm_drone/rm_drone.env}"
# shellcheck disable=SC1090
[ -f "$RM_ENV_FILE" ] && . "$RM_ENV_FILE"

SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="${ROOT_DIR:-$(cd "$SCRIPTS_DIR/.." && pwd)}"
CATKIN_WS="${CATKIN_WS:-$ROOT_DIR/catkin_ws}"
SVO_WS="${SVO_WS:-$ROOT_DIR/svo_ws}"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-noetic}"

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-127.0.0.1}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/var/log/rm_drone/ros}"
mkdir -p "$ROS_LOG_DIR" 2>/dev/null || export ROS_LOG_DIR=/tmp/rm_drone_ros_log

# setup.bash reads unset variables, so relax -u while sourcing.
rm_source() {
  set +u
  # shellcheck disable=SC1090
  . "$1" || { echo "[rm_drone] cannot source $1" >&2; exit 1; }
  set -u
}

rm_source "/opt/ros/$ROS_DISTRO_NAME/setup.bash"

# Source only the workspace a stage needs: catkin_ws and svo_ws must not be overlaid by default.
rm_source_rm_drone() { rm_source "$CATKIN_WS/devel/setup.bash"; }
rm_source_svo()      { rm_source "$SVO_WS/devel/setup.bash"; }

rm_log() { echo "[rm_drone:${RM_STAGE:-?}] $*"; }

# rm_wait_ros: block until the master answers.
rm_wait_ros() {
  local t=${1:-60}
  until rosnode list >/dev/null 2>&1; do
    t=$((t - 1))
    [ "$t" -gt 0 ] || { rm_log "ROS master not reachable"; exit 1; }
    sleep 1
  done
}

# rm_wait_topic <topic> [timeout_s]: block until at least one message arrives.
rm_wait_topic() {
  local topic=$1 t=${2:-60}
  rm_log "waiting for $topic"
  until timeout 3 rostopic echo -n 1 "$topic" >/dev/null 2>&1; do
    t=$((t - 3))
    [ "$t" -gt 0 ] || { rm_log "timeout waiting for $topic"; exit 1; }
  done
}
