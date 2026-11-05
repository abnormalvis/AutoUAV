#!/usr/bin/env bash
# Read-only chain check: VIO -> bridge -> /localization/odometry -> EGO/px4ctrl, plus
# consistency with PX4. Prints PASS/FAIL per item; attach the output when reporting issues.
#   scripts/check_chain.sh [schurvins|openvins]
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
[ "$valid" = "True" ] || [ "$valid" = "true" ] \
  && pass "/vins_mavros_bridge/valid true" \
  || fail "/vins_mavros_bridge/valid = ${valid:-none}"

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

# The two estimators have independent local origins, so compare motion over a
# short interval rather than their absolute positions.
read_position() {
  timeout 3 rostopic echo -n 1 "$1" 2>/dev/null |
    awk '/^x:/{x=$2} /^y:/{y=$2} /^z:/{z=$2} END{if(x!="" && y!="" && z!="") print x, y, z}'
}

read -r vx1 vy1 vz1 <<< "$(read_position /localization/odometry/pose/pose/position)"
read -r px1 py1 pz1 <<< "$(read_position /mavros/local_position/pose/pose/position)"
if [ -n "${vx1:-}" ] && [ -n "${px1:-}" ]; then
  sleep 2
  read -r vx2 vy2 vz2 <<< "$(read_position /localization/odometry/pose/pose/position)"
  read -r px2 py2 pz2 <<< "$(read_position /mavros/local_position/pose/pose/position)"
fi

if [ -n "${vx1:-}" ] && [ -n "${px1:-}" ] && [ -n "${vx2:-}" ] && [ -n "${px2:-}" ]; then
  d=$(awk -v ax="$vx1" -v ay="$vy1" -v az="$vz1" \
          -v bx="$vx2" -v by="$vy2" -v bz="$vz2" \
          -v cx="$px1" -v cy="$py1" -v cz="$pz1" \
          -v dx="$px2" -v dy="$py2" -v dz="$pz2" \
          'BEGIN{printf "%.3f", sqrt(((bx-ax)-(dx-cx))^2+((by-ay)-(dy-cy))^2+((bz-az)-(dz-cz))^2)}')
  if awk -v d="$d" 'BEGIN{exit !(d<0.3)}'; then
    pass "PX4/VIO motion mismatch ${d}m over 2s (absolute local-origin offset ignored)"
  else
    fail "PX4/VIO motion mismatch ${d}m over 2s (check time sync and EKF2 external-vision fusion)"
  fi
else
  fail "could not read position samples for consistency check"
fi

echo "---"
[ "$FAILS" -eq 0 ] && echo "ALL PASS" || echo "$FAILS check(s) failed"
exit "$FAILS"
