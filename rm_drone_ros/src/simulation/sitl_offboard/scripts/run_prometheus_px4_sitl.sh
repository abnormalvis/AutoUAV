#!/usr/bin/env bash
set -euo pipefail

PX4_HOME="${PX4_HOME:-/home/idris-24-04/px4_ws/PX4-Autopilot}"
export PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-10015}"
export PX4_SIM_MODEL="${PX4_SIM_MODEL:-gazebo-classic_p450_D435i}"
export PX4_SIM_WORLD="${PX4_SIM_WORLD:-planning_easy}"
export PX4_SIM_SPEED_FACTOR="${PX4_SIM_SPEED_FACTOR:-1.0}"

exec "$(dirname "${BASH_SOURCE[0]}")/run_px4_sitl.sh" "$PX4_HOME"
