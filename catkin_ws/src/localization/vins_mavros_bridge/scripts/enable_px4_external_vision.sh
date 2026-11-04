#!/usr/bin/env bash
# Point PX4 EKF2 at the external vision that vins_mavros_bridge publishes.
#
# WHY THIS IS A SEPARATE STEP
#
# Delivering a pose to /mavros/vision_pose/pose is not the same as PX4 using it.
# MAVROS forwards it to the FCU as a VISION_POSITION_ESTIMATE message; EKF2 then
# ignores it until a parameter enables that measurement source. Nothing reports
# the difference: the topic is healthy, the flight controller is healthy, and the
# vehicle simply dead-reckons.
#
# The two parameter sets below differ because PX4 changed them. v1.13 (what the
# XTDrone SITL ships) accepts EKF2_EV_CTRL as a name but rejects writes on it, so
# the older EKF2_AID_MASK / EKF2_HGT_MODE pair is what actually takes effect
# there. This script tries the newer pair first and falls back.
#
# Usage:
#   ./enable_px4_external_vision.sh            read and report the current state
#   ./enable_px4_external_vision.sh --apply    write the parameters
#
# Requires a running mavros and, for the write path, a connected FCU. Whether
# EKF2 is actually fusing the stream is checked separately: /mavros/local_position
# should follow /localization/odometry to within a few centimetres. A growing
# offset means the parameter took but the measurement is being rejected.
set -uo pipefail

APPLY=0
if [ "${1:-}" = "--apply" ]; then
  APPLY=1
fi

MAVROS_NS="${MAVROS_NS:-/mavros}"
PARAM_TOPIC="${MAVROS_NS}/param"
PARAM_SERVICE="${PARAM_TOPIC}/set"
GET_SERVICE="${PARAM_TOPIC}/get"

if ! rosnode list >/dev/null 2>&1; then
  echo "ERROR: no ROS master. Start roscore and mavros first." >&2
  exit 1
fi

if ! rosservice list 2>/dev/null | grep -qx "$PARAM_SERVICE"; then
  echo "ERROR: $PARAM_SERVICE is not advertised. Is mavros running and connected to the FCU?" >&2
  exit 1
fi

get_param() {
  # rosservice-type field formatting: integers come back as {integer: N}
  rosservice call -1 "$GET_SERVICE" "{param_id: '$1', value: {integer: 0}}" 2>/dev/null \
    | awk '/value:/ {for (i = 1; i <= NF; i++) if ($i == "real" || $i == "integer") {print $(i+1); exit}}'
}

set_integer() {
  local name="$1" value="$2"
  local out
  out=$(rosservice call -1 "$PARAM_SERVICE" \
        "{param_id: '$name', value: {integer: $value}}" 2>&1)
  if printf '%s' "$out" | grep -q "success: True"; then
    echo "  set $name = $value"
    return 0
  fi
  echo "  REFUSED $name = $value (firmware rejected the write)"
  return 1
}

echo "=== current external-vision parameters"
for name in EKF2_EV_CTRL EKF2_AID_MASK EKF2_HGT_MODE EKF2_HGT_REF EKF2_EV_DELAY EKF2_EV_NOISE_MD; do
  value=$(get_param "$name")
  if [ -z "$value" ]; then
    echo "  $name: not readable (firmware may not define it)"
  else
    echo "  $name: $value"
  fi
done

if [ "$APPLY" -ne 1 ]; then
  echo
  echo "Read-only. Re-run with --apply to write the parameters."
  exit 0
fi

echo
echo "=== writing"
used_modern=0
# 7 = bit0 horizontal position + bit1 horizontal velocity + bit2 vertical position.
if set_integer EKF2_EV_CTRL 7; then
  used_modern=1
else
  echo "  falling back to the pre-1.14 parameter pair"
  # 24 = bit3 external-vision position + bit4 external-vision yaw.
  set_integer EKF2_AID_MASK 24 || true
  # 3 = height reference is vision.
  set_integer EKF2_HGT_MODE 3 || true
fi

echo
echo "=== after"
for name in EKF2_EV_CTRL EKF2_AID_MASK EKF2_HGT_MODE; do
  value=$(get_param "$name")
  [ -n "$value" ] && echo "  $name: $value"
done

echo
if [ "$used_modern" -eq 1 ]; then
  echo "Configured with EKF2_EV_CTRL. Confirm fusion:"
else
  echo "Configured with the pre-1.14 pair. Confirm fusion:"
fi
echo "  rostopic echo -n1 ${MAVROS_NS}/local_position/pose/pose/position"
echo "  rostopic echo -n1 /localization/odometry/pose/pose/position"
echo "They must agree to within a few centimetres and keep agreeing while moving."
echo
echo "NOTE: these parameters are latched at boot. They do not survive an FCU or"
echo "      PX4 restart. Persist them in the airframe parameter file."
echo "      EKF2_EV_DELAY and EKF2_EV_POS_* are calibration, not defaults: set"
echo "      them from the measured camera-to-IMU extrinsic and the real latency."
