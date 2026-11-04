#!/usr/bin/env bash
# VIO estimator. LOCALIZATION=schurvins runs SchurVINS from svo_ws (separate
# workspace, must not share an overlay with rm_drone_ros); openvins runs ov_msckf.
set -euo pipefail
RM_STAGE=vio
. "$(dirname "$(readlink -f "$0")")/common.sh"
rm_wait_ros

LOCALIZATION="${LOCALIZATION:-schurvins}"
IMU_TOPIC="${IMU_TOPIC:-/mavros/imu/data_raw}"
CAM0_TOPIC="${CAM0_TOPIC:-/camera/infra1/image_rect_raw}"
CAM1_TOPIC="${CAM1_TOPIC:-/camera/infra2/image_rect_raw}"

rm_wait_topic "$CAM0_TOPIC" 90
rm_wait_topic "$IMU_TOPIC" 60

case "$LOCALIZATION" in
  schurvins)
    rm_source_svo
    : "${SVO_LAUNCH:=$RM_DRONE_WS/config/d430_schurvins/live_d430_px4imu.launch}"
    exec roslaunch "$SVO_LAUNCH" \
      cam0_topic:="$CAM0_TOPIC" cam1_topic:="$CAM1_TOPIC" imu_topic:="$IMU_TOPIC"
    ;;
  openvins)
    rm_source_rm_drone
    : "${OV_CONFIG:?set OV_CONFIG (estimator_config.yaml) in $RM_ENV_FILE}"
    exec roslaunch ov_msckf subscribe.launch config_path:="$OV_CONFIG" \
      topic_camera0:="$CAM0_TOPIC" topic_imu:="$IMU_TOPIC" use_stereo:=false max_cameras:=1
    ;;
  *) rm_log "unknown LOCALIZATION=$LOCALIZATION"; exit 2 ;;
esac
