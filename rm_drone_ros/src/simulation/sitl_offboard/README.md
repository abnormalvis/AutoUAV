# sitl_offboard

ROS 1 launch and script entry points for wiring OpenVINS, MAVROS, EGO-Planner,
and `px4ctrl` into a PX4 SITL / Gazebo Classic simulation.

## Sensor baseline

The `iris_vio` model (see `models/iris_vio/`) extends the stock PX4 Iris with a
forward-facing sensor link `camera_link` (pose `0.1 0 0`, looking along +X). All
four sensors are co-located on that link, so `T_cam_imu = identity`:

| Sensor | Plugin | Topic | Notes |
|--------|--------|-------|-------|
| RGB | `libgazebo_ros_camera.so` | `/camera/left/image_raw` | OpenVINS mono, 30 Hz |
| Depth | `libgazebo_ros_openni_kinect.so` | `/camera/depth/image_rect_raw` | 16UC1 mm, 15 Hz |
| IMU | `libgazebo_ros_imu_sensor.so` | `/camera/imu` | 200 Hz |
| Pose | `libgazebo_ros_p3d.so` | `/camera/pose_odom` | ground-truth Odometry, world frame |

Canonical intrinsics (consistent across the SDF and `config/sitl_iris/`):
848x480, `horizontal_fov = 1.5009831567 rad` (D455-like), centered pinhole
`fx = fy = 455.4`, `cx = 424.0`, `cy = 240.0`, zero distortion. Gazebo Classic
keeps the optical axis centered, so `cx/cy` are exactly `width/2, height/2`.

IMU noise densities (also in `config/sitl_iris/kalibr_imu_chain.yaml`):
gyro `1.7e-4 rad/s/√Hz`, accel `3.0e-3 m/s²/√Hz`. The per-sample Gazebo gaussian
stddevs in the SDF are these densities scaled by `sqrt(200 Hz)`.

Airframe: `run_px4_sitl.sh` picks the right one for the PX4 version in use -
`PX4_SYS_AUTOSTART=10015` (iris quad-x) on v1.14+, and the `iris` model name on
v1.13, whose `rcS` does the airframe lookup by file name instead and ships
`10016_iris` rather than `10015`. See the script header for why the two differ.

## Prerequisites

### 1. Build the PX4 gazebo-classic plugins (do this first)

This renders `*.sdf.jinja` -> `*.sdf` in place (so `model://iris` resolves) and
builds the PX4 Gazebo plugin `.so` files:

```bash
source /opt/ros/noetic/setup.bash   # required: the plugins link geometry_msgs etc.
export PX4_HOME=/path/to/PX4-Autopilot
make -C "$PX4_HOME" px4_sitl_default sitl_gazebo -j4
```

Verify:

```bash
ls "$PX4_HOME"/Tools/sitl_gazebo/models/iris/iris.sdf                      # v1.13 and older
ls "$PX4_HOME"/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf  # v1.14+
ls "$PX4_HOME"/build/px4_sitl_default/build_gazebo/libgazebo_*.so          # v1.13 and older
ls "$PX4_HOME"/build/px4_sitl_default/build_gazebo-classic/libgazebo_*.so  # v1.14+
```

`run_px4_sitl.sh` and `sitl.launch` accept either layout: both the legacy
(`Tools/sitl_gazebo` + `build_gazebo`) and the current one
(`Tools/simulation/gazebo-classic/sitl_gazebo-classic` + `build_gazebo-classic`)
are detected/placed on the search path. `run_px4_sitl.sh` fails fast with these
exact messages if the SDFs or plugins are missing.

**Sourcing ROS is not optional.** `sitl_gazebo` links against `geometry_msgs`,
`nav_msgs` and `mavros_msgs`; without `/opt/ros/noetic/setup.bash` the external
project fails at its configure step with
`Could not find a package configuration file provided by "geometry_msgs"`.

PX4 build prerequisites that a bare ROS container is missing (Ubuntu 20.04):

```bash
sudo apt-get install -y ninja-build bc python3-jinja2 python3-toml \
                        python3-empy python3-packaging python3-jsonschema \
                        python3-future libgstreamer1.0-dev \
                        libgstreamer-plugins-base1.0-dev
pip3 install --user kconfiglib pyros-genmsg
```

`bc` is not a build dependency: PX4's `etc/init.d-posix/rcS` calls it at
runtime and dies with `rcS: 174: bc: not found` without it. `pyros-genmsg`
provides `genmsg`, without which the uORB message generation fails. The
GStreamer dev packages are needed by `gst_camera_plugin`; the build otherwise
fails with `GSTREAMER_APP_LIBRARIES (ADVANCED)` set to NOTFOUND.

MAVROS also needs the GeographicLib geoid dataset at runtime. Without it
`mavros_node` aborts with `GeographicLib exception: File not readable
/usr/share/GeographicLib/geoids/egm96-5.pgm`. The XTDrone bundle ships it:

```bash
sudo cp -r <xtdrone>/sources/GeographicLib/* /usr/share/GeographicLib/
```

If `PX4_Firmware` was copied out of the XTDrone installer rather than cloned,
its `.git` has a different owner than the container user and every `git
describe` fails with `detected dubious ownership`. That breaks PX4's git version
header generation:

```bash
git config --global --add safe.directory "$PX4_HOME"
```

### 2. Build and source the catkin workspace

One catkin workspace, `rm_drone_ros`, is used. The `ego_planner`/`px4ctrl`
flight stack is symlinked into the `planner/` and `controller/` categories from
`third_party/Fast-Drone-250`, so a single build covers everything:

```bash
cd <path-to-rmUAV>/rm_drone_ros
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="ov_core;ov_msckf;ov_init;ov_data;ov_eval;vins_mavros_bridge;sitl_offboard;ego_planner;plan_env;path_searching;bspline_opt;traj_utils;quadrotor_msgs;px4ctrl;uav_utils;cmake_utils;offboard" -j4
source devel/setup.bash
```

The whitelist skips `apm_d430_bridge`/`px4_d430_bridge`, which still contain
machine-specific absolute paths and are not part of this chain. `d430_slam` and
the RealSense driver packages are excluded from the catkin build entirely (see
the workspace README).

Then confirm all of these resolve:

```bash
for p in ego_planner px4ctrl quadrotor_msgs ov_msckf vins_mavros_bridge sitl_offboard mavros; do rospack find "$p"; done
```

## SITL bring-up sequence

Open one terminal per step. All ROS terminals must be in the sourced ROS Noetic
environment (both `devel/setup.bash` files loaded).

### 1. Gazebo + vehicle + sensors

```bash
roslaunch sitl_offboard sitl.launch
```

This starts Gazebo Classic with the lightweight `worlds/indoor_room.sdf`
(12 x 10 m), spawns `iris_vio` (0.3 m up), and publishes the ground-truth
`/camera/pose`. To load the larger RM2027 arena instead, pass
`world:=$(rospack find sitl_offboard)/worlds/rm2027_gazebo/rm2027_arena_classic.world`.
`sitl.launch` sets
`GAZEBO_MODEL_PATH` / `GAZEBO_PLUGIN_PATH` itself via the `px4_src`/`px4_build`
args, which default to `$PX4_HOME` / `$PX4_BUILD_DIR`; export them first.

The indoor world has simple box geometry and a small number of textured walls
and obstacles to reduce Gazebo rendering load while retaining nearby visual
features for VIO. Camera/depth rendering and OpenVINS still consume CPU/GPU;
if the simulator remains slow, try Gazebo headless (`gui:=false`) or reduce the
camera update rate/resolution in the vehicle model.

### 2. PX4 autopilot (separate terminal)

```bash
rosrun sitl_offboard run_px4_sitl.sh
```

PX4 connects to the iris_vio model's mavlink interface over TCP 4560. Confirm
the connection with `rostopic echo -n 1 /mavros/state` once MAVROS is up.

### 3. VIO -> PX4 EKF and px4ctrl

```bash
roslaunch sitl_offboard px4_vio_ctrl.launch \
  fcu_url:=udp://:14550@127.0.0.1:18570 \
  camera_topic:=/camera/left/image_raw \
  imu_topic:=/camera/imu \
  openvins_config:=$(rospack find sitl_offboard)/config/sitl_iris/estimator_config.yaml
```

`fcu_url` must match the MAVROS-side UDP port PX4 opens for the GCS link
(`18570` for instance 0, from `px4-rc.mavlink`). The PX4 v1.13 defaults
`14557` belong to the offboard link and do not answer here.

Verify `/mavros/state` (connected), `/ov_msckf/odomimu`, and
`/localization/odometry` tracking it. The bridge forwards the normalized estimate
to `/mavros/vision_pose/pose` and to px4ctrl; the raw VIO topic is no longer
consumed by anything downstream. For an isolated RC-free SITL run, add
`enable_rc_free_takeoff:=true` to px4ctrl.

If OpenVINS is not wanted yet, the same chain can be exercised against the
simulator's ground-truth odometry. This proves the wiring without waiting for
VIO to converge:

```bash
roslaunch vins_mavros_bridge bridge.launch \
  config:=$(rospack find vins_mavros_bridge)/config/gazebo_groundtruth.yaml
```

Verified on this machine with that setup: bridge status `state=OK`,
`input_rate_hz=50`, `/localization/odometry` in `world -> base_link`,
`/mavros/vision_pose/pose` at 50 Hz, and px4ctrl consuming the estimate (it
logs its `ODOM frequency seems lower than 100Hz` warning, which is the expected
complaint for a 50 Hz source, and never `No odom!`).

#### Fusing external vision in PX4 v1.13

The chain above only *delivers* vision to MAVROS. PX4 does not fuse it until
told to, and v1.13 uses a different parameter set from the one usually quoted
for newer firmware. `EKF2_EV_CTRL` exists as a name but rejects writes on
v1.13; the working pair is `EKF2_AID_MASK` + `EKF2_HGT_MODE`:

```bash
rosservice call /mavros/param/set "{param_id: EKF2_AID_MASK, value: {integer: 24}}"  # bit3 EV pos + bit4 EV yaw
rosservice call /mavros/param/set "{param_id: EKF2_HGT_MODE, value: {integer: 3}}"   # height from vision
```

`AID_MASK` bit 0 is GPS, bit 3 external-vision position, bit 4 external-vision
yaw, bit 6 external-vision velocity. Both calls return `success: True` and take
effect within a few seconds. After that `/mavros/local_position/pose` follows
`/localization/odometry` to within a few centimetres, which is the check that
EKF2 is actually consuming this stream rather than dead-reckoning.

**This parameter is latched at boot.** Setting `EKF2_AID_MASK` at runtime and
then letting PX4 restart (or rebooting the FCU) silently reverts it. For a run
that must survive a PX4 restart, seed the airframe's parameter file instead:

```bash
PARAM_FILE=$PX4_HOME/build/px4_sitl_default/tmp/rootfs/eeprom/parameters_10016
# append/overwrite: EKF2_AID_MASK 24, EKF2_HGT_MODE 3,
#                   COM_RC_IN_MODE 4, NAV_RCL_ACT 0
```

`COM_RC_IN_MODE 4` and `NAV_RCL_ACT 0` matter because there is no RC
transmitter in SITL: with the defaults PX4 drops into `AUTO.RTL` on RC loss and
refuses to arm. Those two are SITL-only; never carry them to a real airframe.

The same values belong in the PX4 airframe/parameter file for real flight.
`EKF2_EV_DELAY` and `EKF2_EV_POS_*` still need the measured camera-to-IMU
extrinsic and the real vision latency - those are calibration, not defaults.

### 4. Take off

```bash
rostopic pub -1 /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1"
```

**Not yet verified end to end.** px4ctrl accepts the command, switches PX4 to
OFFBOARD, arms, and then the vehicle disarms and falls back to `AUTO.LOITER`
within about 5 s without climbing. The odometry side is not the cause - the
bridge is at `state=OK` with `odom_data.v.norm()` near zero and EKF2 is
tracking it, which are the preconditions px4ctrl checks. Suspects not yet
excluded: the thrust mapping (`controller.resetThrustMapping()` needs a hover
throttle estimate that SITL has no way to provide), and the `get_landed()`
check against a Gazebo model that is spawned 0.3 m up. Treat takeoff in SITL as
open work; everything up to and including "px4ctrl is receiving a valid
estimate" is verified.

### 5. EGO-Planner

```bash
roslaunch sitl_offboard ego_px4ctrl.launch \
  odom_topic:=/localization/odometry \
  camera_pose_topic:=/camera/pose \
  depth_topic:=/camera/depth/image_rect_raw
```

The intrinsics default to the baseline (cx 424.0, cy 240.0, fx/fy 455.4). Use
RViz's **2D Nav Goal** tool to publish `/move_base_simple/goal`, or set
`start_keyboard:=true` for the keyboard goal node.

### 6. EGO preset waypoints with the standalone offboard tracker

This alternative uses `offboard_planner_tracker_node` instead of `px4ctrl`.
Do not run both controllers, `mission_planner`, or a manual setpoint publisher
at the same time. Stop any existing `/mavros/setpoint_position/local`
publisher before enabling flight.

First, with Gazebo and PX4 already running, inspect the combined graph without
arming:

```bash
roslaunch sitl_offboard ego_offboard_tracker_sitl.launch enable_flight:=false
rostopic info /position_cmd
rostopic info /mavros/setpoint_raw/local
rostopic hz /ov_msckf/odomimu
```

Stop that launch, then explicitly enable the flight sequence:

```bash
roslaunch sitl_offboard ego_offboard_tracker_sitl.launch enable_flight:=true \
  takeoff_height:=0.5 takeoff_vertical_speed:=0.25 max_velocity:=0.6
```

The tracker waits for a single explicit operator trigger before publishing any
setpoints or arming:

```bash
rostopic pub -1 /offboard/go std_msgs/Empty "{}"
```

After GO, it pre-streams zero local-velocity setpoints, requests OFFBOARD and
arms, then climbs using a bounded vertical-velocity setpoint. Bootstrap uses
PX4 attitude/vertical-velocity validity and MAVROS relative altitude; it does
not use Gazebo ground-truth `/camera/pose_odom` as a PX4 position estimate.
The default climb is 0.5 m with a short bounded yaw-rate excitation to help
OpenVINS initialize. The tracker continues velocity control until both fresh
`/ov_msckf/odomimu` and PX4 horizontal/vertical local-position validity flags
remain valid for `vio_stable_duration`. It then holds the VIO pose, triggers
EGO's preset route, and tracks `/position_cmd`. Stale trajectory commands
cause a hold at the current VIO position. Invalid bootstrap estimates, failed
VIO initialization, or position-estimate loss request PX4 `AUTO.LAND` while
the tracker continues streaming zero-velocity setpoints.

The launch sets EGO to preset mode (`flight_type=2`) and waits for the tracker
trigger before planning. Flight remains opt-in (`enable_flight:=false` by
default). Do not run `px4ctrl`, `mission_planner`, or a manual setpoint
publisher concurrently with this tracker.

## Data flow and constraints

```text
sim camera + IMU -> OpenVINS /ov_msckf/odomimu
                 -> vins_mavros_bridge -> /localization/odometry
                      |-> /mavros/vision_pose/pose -> PX4 EKF2
                      |-> px4ctrl / EGO odometry -> B-spline -> PositionCommand
sim depth + unified pose ----------------------> EGO -> traj_server -> /position_cmd
MAVROS estimator flags + altitude -------------> bootstrap gate
offboard_planner_tracker -> /mavros/setpoint_raw/local -> PX4 SITL
```

- **EGO camera pose is the unified odometry.** The included
  `advanced_param_exp.xml` sets `grid_map/pose_type = 2` (ODOMETRY), so the
  depth-fusion camera pose comes from `~grid_map/odom` -> `/localization/odometry`,
  not `/camera/pose`. That is consistent because `T_cam_imu = identity`.
  `/camera/pose` is a ground-truth cross-check (via `camera_pose_bridge.py`),
  and is also the topic EGO would use if switched to `pose_type = 1`.
- **Depth encoding.** The SDF publishes `16UC1` millimeters directly
  (`<useDepth16UC1Format>true`), matching EGO's `k_depth_scaling_factor = 1000`.
  EGO also handles `32FC1` meters natively, so if a different sensor emits
  meters, either path works. The `depth_mm_bridge.py` node is a fallback
  converter; enable it with `start_depth_bridge:=true` only if needed.
- Do not start the EGO launch until px4ctrl has finished takeoff. px4ctrl
  intentionally rejects command traffic while in `MANUAL_CTRL` or auto-takeoff.
- `mission_planner` also publishes MAVROS setpoints. Do not run it with
  `px4ctrl`; there must be one active flight controller/setpoint source.
- OpenVINS initializes on motion (`init_dyn_use: true`); the drone is stationary
  at spawn, so `/ov_msckf/odomimu` starts only after takeoff motion — expected.
  `/localization/odometry` follows once the bridge has accepted
  `recovery_frames` consecutive samples.
- Frame alignment between OpenVINS `global` and Gazebo `world` (gravity-aligned
  z-up, possible yaw offset) is acceptable for first validation; document any
  offset as a follow-up. The bridge publishes into `world` and keeps the input
  world untouched, so a yaw offset shows up directly in the output.
- **Camera optical-axis convention.** The Gazebo camera looks along `+X` (body
  forward), while OpenVINS's pinhole model treats `+Z` as the optical axis. The
  approved baseline uses `T_cam_imu = identity`; if VIO initializes with a
  ~90 deg pitch/roll offset or the EGO depth map looks rotated, apply the
  standard `+Z -> +X` rotation to `T_cam_imu` in
  `config/sitl_iris/kalibr_imucam_chain.yaml` (and keep the depth camera frame
  consistent). This is a runtime-tuning item, not a static error.
- This integration is SITL-only until frame alignment, estimator loss behavior,
  mode fallback, and emergency takeover have been tested on the exact
  PX4/MAVROS versions. Never test an unverified launch on a physical vehicle.

## Interface checks

```bash
rostopic hz /camera/left/image_raw
rostopic hz /camera/imu
rostopic hz /camera/depth/image_rect_raw
rostopic echo -n 1 /camera/depth/image_rect_raw   # expect encoding mono16/16UC1
rostopic hz /camera/pose
rostopic hz /ov_msckf/odomimu
rostopic hz /mavros/vision_pose/pose
rostopic echo -n 1 /mavros/state
rostopic info /move_base_simple/goal
rostopic info /position_cmd
rostopic info /mavros/setpoint_raw/attitude
```

## Package layout

- `models/iris_vio/` — vehicle model (iris + VIO sensor suite).
- `config/sitl_iris/` — self-contained OpenVINS SITL config (3 YAML files).
- `scripts/run_px4_sitl.sh` — standalone PX4 launcher (airframe 10015).
- `scripts/camera_pose_bridge.py` — `/camera/pose_odom` -> `/camera/pose`.
- `scripts/depth_mm_bridge.py` — optional 32FC1 -> 16UC1 mm converter.
- `launch/sitl.launch` — Gazebo world + model spawn + pose bridge.
- `launch/px4_vio_ctrl.launch` — MAVROS + OpenVINS + vision bridge + px4ctrl.
- `launch/ego_px4ctrl.launch` — EGO-Planner + traj_server + RViz + keyboard goal.
- `launch/ego_offboard_tracker_sitl.launch` — EGO preset waypoints + standalone offboard tracker; flight is opt-in.
- `worlds/indoor_room.sdf` — textured indoor room (features for VIO).
