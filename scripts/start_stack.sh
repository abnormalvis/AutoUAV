#!/usr/bin/env bash
# vins_mavros_bridge + EGO-Planner + px4ctrl. Does not issue takeoff.
set -euo pipefail
RM_STAGE=stack
. "$(dirname "$(readlink -f "$0")")/common.sh"
rm_wait_ros
rm_source_rm_drone

LOCALIZATION="${LOCALIZATION:-schurvins}"
case "$LOCALIZATION" in
  schurvins) VIO_TOPIC="${VIO_TOPIC:-/svo/pose_imu}" ;;
  openvins)  VIO_TOPIC="${VIO_TOPIC:-/ov_msckf/odomimu}" ;;
  *) rm_log "unknown LOCALIZATION=$LOCALIZATION"; exit 2 ;;
esac

rm_wait_topic /mavros/state 60
rm_wait_topic "$VIO_TOPIC" 120

exec roslaunch "$ROOT_DIR/launch/realflight_ego_vio_px4ctrl.launch" \
  localization:="$LOCALIZATION" input_topic:="$VIO_TOPIC" drone_id:="${DRONE_ID:-0}"
