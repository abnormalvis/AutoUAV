#!/usr/bin/env bash
# Read-only chain check: VIO -> bridge -> /localization/odometry -> EGO/px4ctrl, plus
# consistency with PX4. Prints PASS/FAIL per item; attach the output when reporting issues.
#   shfiles/check_chain.sh [schurvins|openvins]
set -uo pipefail
RM_STAGE=check
. "$(dirname "$(readlink -f "$0")")/common.sh"
rm_source_rm_drone

LOCALIZATION="${1:-${LOCALIZATION:-schurvins}}"
case "$LOCALIZATION" in
  schurvins) VIO_TOPIC=/svo/pose_imu ;;
  openvins)  VIO_TOPIC=/ov_msckf/odomimu ;;
  *) echo "unknown localization $LOCALIZATION"; exit 2 ;;
esac

FAILS=0
pass() { echo "PASS  $*"; }
fail() { echo "FAIL  $*"; FAILS=$((FAILS + 1)); }

rostopic list >/dev/null 2>&1 || { echo "FAIL  ROS master not reachable"; exit 1; }

# check_rate <topic> <min_hz>
check_rate() {
  local out hz
  out=$(timeout 6 rostopic hz -w 20 "$1" 2>/dev/null | grep "average rate" | tail -1)
  hz=$(echo "$out" | awk '{print $3}')
  if [ -z "$hz" ]; then fail "$1 no messages"; return; fi
  if awk -v h="$hz" -v m="$2" 'BEGIN{exit !(h>=m)}'; then pass "$1 ${hz}Hz (>= $2)"; else fail "$1 ${hz}Hz (< $2)"; fi
}

check_rate /camera/infra1/image_rect_raw 15
check_rate /camera/infra2/image_rect_raw 15
check_rate /mavros/imu/data_raw 90
check_rate "$VIO_TOPIC" 10
check_rate /localization/odometry 10
check_rate /mavros/vision_pose/pose 10

valid=$(timeout 3 rostopic echo -n 1 /vins_mavros_bridge/valid 2>/dev/null | grep -m1 "data:" | awk '{print $2}')
[ "$valid" = "true" ] && pass "/vins_mavros_bridge/valid true" || fail "/vins_mavros_bridge/valid = ${valid:-none}"

connected=$(timeout 3 rostopic echo -n 1 /mavros/state 2>/dev/null | grep -m1 "connected:" | awk '{print $2}')
[ "$connected" = "True" ] || [ "$connected" = "true" ] && pass "FCU connected" || fail "FCU not connected"

# Downstream nodes must read /localization/odometry, not a raw VIO topic.
for n in "/drone_${DRONE_ID:-0}_ego_planner_node" /px4ctrl; do
  if rosnode list 2>/dev/null | grep -q "^${n}$"; then
    if rosnode info "$n" 2>/dev/null | grep -q "/localization/odometry"; then
      pass "$n subscribes /localization/odometry"
    else
      fail "$n does not subscribe /localization/odometry"
    fi
  else
    fail "node $n not running"
  fi
done

# Offset between PX4 local position and the VIO output: grows if EKF2 rejects external vision.
read -r vx vy vz < <(timeout 3 rostopic echo -n 1 /localization/odometry/pose/pose/position 2>/dev/null | awk '/^x:|^y:|^z:/{printf "%s ", $2}')
read -r px py pz < <(timeout 3 rostopic echo -n 1 /mavros/local_position/pose/pose/position 2>/dev/null | awk '/^x:|^y:|^z:/{printf "%s ", $2}')
if [ -n "${vx:-}" ] && [ -n "${px:-}" ]; then
  d=$(awk -v a="$vx" -v b="$vy" -v c="$vz" -v x="$px" -v y="$py" -v z="$pz" 'BEGIN{printf "%.3f", sqrt((a-x)^2+(b-y)^2+(c-z)^2)}')
  if awk -v d="$d" 'BEGIN{exit !(d<0.3)}'; then pass "PX4 vs VIO position offset ${d}m"; else fail "PX4 vs VIO position offset ${d}m (EKF2 may not be fusing; see enable_px4_external_vision.sh)"; fi
else
  fail "could not read positions for consistency check"
fi

echo "---"
[ "$FAILS" -eq 0 ] && echo "ALL PASS" || echo "$FAILS check(s) failed"
exit "$FAILS"
