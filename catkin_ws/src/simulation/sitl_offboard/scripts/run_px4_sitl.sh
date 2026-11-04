#!/usr/bin/env bash
#
# Launch PX4 SITL standalone against a Gazebo Classic world that is already
# running (start the Gazebo side first with: roslaunch sitl_offboard sitl.launch).
#
# PX4 connects to the iris_vio model's mavlink interface plugin over TCP 4560;
# no simulator process is spawned here.
#
# Two PX4 layouts are supported, because upstream moved the gazebo-classic tree
# at v1.14:
#
#   v1.13 and older  Tools/sitl_gazebo, plugins in build/.../build_gazebo
#                    airframe selected by PX4_SIM_MODEL name lookup
#   v1.14 and newer  Tools/simulation/gazebo-classic/sitl_gazebo-classic,
#                    plugins in build/.../build_gazebo-classic
#                    airframe selected by PX4_SYS_AUTOSTART
#
# The layout is detected from the checkout rather than a version string, because
# the XTDrone installer ships a v1.13.2 tree with no reliable version marker.
#
# Usage:  ./run_px4_sitl.sh [px4_home]
#         (px4_home defaults to $PX4_HOME; there is no built-in path because the
#          PX4 checkout location is machine-specific)
set -euo pipefail

PX4_HOME="${1:-${PX4_HOME:-}}"
if [ -z "$PX4_HOME" ]; then
    echo "ERROR: PX4_HOME is not set." >&2
    echo "       Export PX4_HOME=/path/to/PX4-Autopilot or pass it as \$1." >&2
    exit 1
fi
BUILD_DIR="${PX4_BUILD_DIR:-$PX4_HOME/build/px4_sitl_default}"

# --- Detect the gazebo-classic layout ----------------------------------------
if [ -d "$PX4_HOME/Tools/simulation/gazebo-classic/sitl_gazebo-classic" ]; then
    GAZEBO_SRC="$PX4_HOME/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
    GAZEBO_BUILD="$BUILD_DIR/build_gazebo-classic"
    SETUP_GAZEBO="$PX4_HOME/Tools/simulation/gazebo-classic/setup_gazebo.bash"
    LEGACY_LAYOUT=0
elif [ -d "$PX4_HOME/Tools/sitl_gazebo" ]; then
    GAZEBO_SRC="$PX4_HOME/Tools/sitl_gazebo"
    GAZEBO_BUILD="$BUILD_DIR/build_gazebo"
    SETUP_GAZEBO="$PX4_HOME/Tools/setup_gazebo.bash"
    LEGACY_LAYOUT=1
else
    echo "ERROR: no gazebo-classic source tree found under $PX4_HOME/Tools." >&2
    echo "       Looked for Tools/simulation/gazebo-classic/sitl_gazebo-classic" >&2
    echo "       and Tools/sitl_gazebo." >&2
    exit 1
fi

# --- Path guards (fail fast with actionable messages) -------------------------
IRIS_SDF="$GAZEBO_SRC/models/iris/iris.sdf"
if [ ! -f "$IRIS_SDF" ]; then
    echo "ERROR: $IRIS_SDF is missing." >&2
    echo "       The gazebo-classic model SDFs have not been rendered." >&2
    echo "       Build the plugins (this also renders the SDFs):" >&2
    echo "         source /opt/ros/noetic/setup.bash" >&2
    echo "         make -C $PX4_HOME px4_sitl_default sitl_gazebo -j4" >&2
    exit 1
fi

if ! ls "$GAZEBO_BUILD"/libgazebo_*.so >/dev/null 2>&1; then
    echo "ERROR: no libgazebo_*.so found under $GAZEBO_BUILD" >&2
    echo "       The PX4 gazebo-classic plugins have not been built." >&2
    echo "       Build them (source ROS first; the plugins need geometry_msgs):" >&2
    echo "         source /opt/ros/noetic/setup.bash" >&2
    echo "         make -C $PX4_HOME px4_sitl_default sitl_gazebo -j4" >&2
    exit 1
fi

PX4_BIN="$BUILD_DIR/bin/px4"
if [ ! -x "$PX4_BIN" ]; then
    echo "ERROR: $PX4_BIN is missing or not executable." >&2
    echo "         make -C $PX4_HOME px4_sitl_default -j4" >&2
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
# shellcheck disable=SC1090
source "$SETUP_GAZEBO" "$PX4_HOME" "$BUILD_DIR"

# Make our own model (iris_vio) resolvable alongside the PX4 base models.
PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export GAZEBO_MODEL_PATH="$PKG_DIR/models:$GAZEBO_MODEL_PATH"

# --- Airframe identity -------------------------------------------------------
# Two separate mechanisms, and only one is about the vehicle model.
#
# 1. Which PX4 airframe (parameter set / mixer) rcS loads.
#    v1.14+ : rcS honours PX4_SYS_AUTOSTART, so 10015 (iris quad-x) works.
#    v1.13  : rcS ignores PX4_SYS_AUTOSTART and instead looks up a file named
#             "<id>_${PX4_SIM_MODEL}" under etc/init.d-posix/airframes. v1.13
#             ships 10016_iris and has no 10015, so the name must be "iris".
# 2. Which Gazebo model PX4 expects. That is not PX4's business here: Gazebo
#    spawns the vehicle from our own iris_vio.sdf and PX4 only connects to the
#    mavlink plugin on TCP 4560. PX4_SIM_MODEL is otherwise read only by
#    sitl_run.sh, which this script bypasses.
if [ "$LEGACY_LAYOUT" = "1" ]; then
    export PX4_SIM_MODEL="${PX4_SIM_MODEL:-iris}"
    unset PX4_SYS_AUTOSTART || true
else
    export PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-10015}"
    export PX4_SIM_MODEL="${PX4_SIM_MODEL:-gazebo-classic_iris_vio}"
fi
export PX4_SIM_WORLD="${PX4_SIM_WORLD:-indoor_room}"
export PX4_SIM_SPEED_FACTOR="${PX4_SIM_SPEED_FACTOR:-1.0}"

echo "Launching PX4 SITL standalone:"
echo "  gazebo src  $GAZEBO_SRC"
echo "  plugins     $GAZEBO_BUILD"
echo "  airframe    ${PX4_SYS_AUTOSTART:+PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART }PX4_SIM_MODEL=$PX4_SIM_MODEL"
echo "  world       PX4_SIM_WORLD=$PX4_SIM_WORLD"
echo "Expecting a running Gazebo with the iris_vio model (mavlink TCP 4560)."

# --- Working directory -------------------------------------------------------
# sitl_run.sh normally creates this and copies the rootfs in; this script starts
# the binary directly, so create it here or PX4 writes parameters and logs into
# whatever directory we happen to be in.
ROOTFS="$BUILD_DIR/tmp/rootfs"
mkdir -p "$ROOTFS"
cd "$ROOTFS"

# --- Startup script ----------------------------------------------------------
# Both layouts ship etc/init.d-posix/rcS. Prefer it over etc/init.d/rcS, which is
# the generic (non-SITL) startup script.
if [ -f "$BUILD_DIR/etc/init.d-posix/rcS" ]; then
    RC_S="etc/init.d-posix/rcS"
else
    RC_S="etc/init.d/rcS"
fi

exec "$PX4_BIN" -d "$BUILD_DIR/etc" -s "$RC_S"
