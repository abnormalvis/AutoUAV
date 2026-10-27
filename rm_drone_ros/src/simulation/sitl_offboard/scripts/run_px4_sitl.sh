#!/usr/bin/env bash
#
# Launch PX4 SITL standalone against a Gazebo Classic world that is already
# running (start the Gazebo side first with: roslaunch sitl_offboard sitl.launch).
#
# PX4 connects to the iris_vio model's mavlink interface plugin over TCP 4560;
# no simulator process is spawned here. The airframe is forced to 10015 (iris
# quad-x) via PX4_SYS_AUTOSTART so rcS skips the model-name -> airframe lookup.
#
# Usage:  ./run_px4_sitl.sh [px4_home]
#         (px4_home defaults to $PX4_HOME, then /home/idris-24-04/px4_ws/PX4-Autopilot)
set -euo pipefail

PX4_HOME="${1:-${PX4_HOME:-/home/idris-24-04/px4_ws/PX4-Autopilot}}"
BUILD_DIR="${PX4_BUILD_DIR:-$PX4_HOME/build/px4_sitl_default}"

# --- Path guards (fail fast with actionable messages) -------------------------
IRIS_SDF="$PX4_HOME/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf"
if [ ! -f "$IRIS_SDF" ]; then
    echo "ERROR: $IRIS_SDF is missing." >&2
    echo "       The gazebo-classic model SDFs have not been rendered." >&2
    echo "       Build the plugins standalone:" >&2
    echo "         cmake -S $PX4_HOME/Tools/simulation/gazebo-classic/sitl_gazebo-classic -B $BUILD_DIR/build_gazebo-classic -DSEND_ODOMETRY_DATA=ON -DGENERATE_ROS_MODELS=ON && make -C $BUILD_DIR/build_gazebo-classic -j4" >&2
    exit 1
fi

if ! ls "$BUILD_DIR"/build_gazebo-classic/libgazebo_*.so >/dev/null 2>&1; then
    echo "ERROR: no libgazebo_*.so found under $BUILD_DIR/build_gazebo-classic" >&2
    echo "       The PX4 gazebo-classic plugins have not been built." >&2
    echo "       Build the plugins standalone:" >&2
    echo "         cmake -S $PX4_HOME/Tools/simulation/gazebo-classic/sitl_gazebo-classic -B $BUILD_DIR/build_gazebo-classic -DSEND_ODOMETRY_DATA=ON -DGENERATE_ROS_MODELS=ON && make -C $BUILD_DIR/build_gazebo-classic -j4" >&2
    exit 1
fi

PX4_BIN="$BUILD_DIR/bin/px4"
if [ ! -x "$PX4_BIN" ]; then
    echo "ERROR: $PX4_BIN is missing or not executable." >&2
    exit 1
fi

# --- Gazebo + PX4 environment ------------------------------------------------
# Adds GAZEBO_PLUGIN_PATH and LD_LIBRARY_PATH for the PX4 plugin .so files, and
# GAZEBO_MODEL_PATH for the PX4 base models (iris, gps, depth_camera, ...).
# setup_gazebo.bash appends to these without a default; with `set -u` we must
# pre-initialise them or sourcing it fails on an unbound variable.
export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"
# shellcheck disable=SC1091
source "$PX4_HOME/Tools/simulation/gazebo-classic/setup_gazebo.bash" "$PX4_HOME" "$BUILD_DIR"

# Make our own model (iris_vio) resolvable alongside the PX4 base models.
PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export GAZEBO_MODEL_PATH="$PKG_DIR/models:$GAZEBO_MODEL_PATH"

# Airframe and simulator identity.
export PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-10015}"
export PX4_SIM_MODEL="${PX4_SIM_MODEL:-gazebo-classic_iris_vio}"
export PX4_SIM_WORLD="${PX4_SIM_WORLD:-indoor_room}"
export PX4_SIM_SPEED_FACTOR="${PX4_SIM_SPEED_FACTOR:-1.0}"

echo "Launching PX4 SITL standalone:"
echo "  airframe PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART"
echo "  model    PX4_SIM_MODEL=$PX4_SIM_MODEL"
echo "  world    PX4_SIM_WORLD=$PX4_SIM_WORLD"
echo "Expecting a running Gazebo with the iris_vio model (mavlink TCP 4560)."

cd "$BUILD_DIR/rootfs"
exec "$PX4_BIN" -d "$BUILD_DIR/etc"
