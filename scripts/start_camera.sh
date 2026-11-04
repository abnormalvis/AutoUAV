#!/usr/bin/env bash
# RealSense driver is not part of this workspace; supply it via CAMERA_CMD
# (default: stock realsense2_camera launch, D430 stereo IR + depth, no IMU on this sensor).
set -euo pipefail
RM_STAGE=camera
. "$(dirname "$(readlink -f "$0")")/common.sh"
rm_wait_ros

if [ -n "${CAMERA_SETUP:-}" ]; then rm_source "$CAMERA_SETUP"; fi

CAMERA_CMD="${CAMERA_CMD:-roslaunch realsense2_camera rs_camera.launch \
enable_infra1:=true enable_infra2:=true enable_depth:=true enable_gyro:=false enable_accel:=false \
enable_color:=false infra_width:=848 infra_height:=480 infra_fps:=30}"

rm_log "$CAMERA_CMD"
# shellcheck disable=SC2086
exec $CAMERA_CMD
