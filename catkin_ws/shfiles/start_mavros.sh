#!/usr/bin/env bash
set -euo pipefail
RM_STAGE=mavros
. "$(dirname "$(readlink -f "$0")")/common.sh"
rm_wait_ros

FCU_URL="${FCU_URL:-/dev/ttyACM0:921600}"
GCS_URL="${GCS_URL:-}"
IMU_RATE_HZ="${IMU_RATE_HZ:-200}"

rm_log "fcu_url=$FCU_URL"
# Raise the IMU stream rate once MAVROS is up; VIO needs >=100 Hz and PX4 defaults are lower.
(
  for _ in $(seq 1 60); do
    if rostopic echo -n 1 /mavros/state >/dev/null 2>&1 && \
       rosservice call /mavros/set_stream_rate 0 "$IMU_RATE_HZ" 1 >/dev/null 2>&1; then
      rm_log "stream rate set to ${IMU_RATE_HZ}Hz"; exit 0
    fi
    sleep 2
  done
  rm_log "WARNING: could not set stream rate; /mavros/imu/data_raw may be too slow"
) &

exec roslaunch mavros px4.launch fcu_url:="$FCU_URL" gcs_url:="$GCS_URL"
