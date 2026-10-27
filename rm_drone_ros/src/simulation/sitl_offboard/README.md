# sitl_offboard

ROS 1 launch and script entry points for wiring OpenVINS, MAVROS, EGO-Planner,
and `px4ctrl` into a PX4 SITL / Gazebo Classic simulation. It also contains an
opt-in Prometheus basic-demo integration for exercising Prometheus command flow
on the existing Iris SITL airframe.

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

Airframe: `PX4_SYS_AUTOSTART=10015` (iris quad-x), forced by
`run_px4_sitl.sh` so the model-name -> airframe lookup is skipped.

## Prerequisites

### 1. Build the PX4 gazebo-classic plugins (do this first)

This renders `*.sdf.jinja` -> `*.sdf` in place (so `model://iris` resolves) and
builds the PX4 Gazebo plugin `.so` files:

```bash
cd /home/idris-24-04/px4_ws/PX4-Autopilot
cmake -S Tools/simulation/gazebo-classic/sitl_gazebo-classic \
      -B build/px4_sitl_default/build_gazebo-classic \
      -DSEND_ODOMETRY_DATA=ON -DGENERATE_ROS_MODELS=ON
cmake --build build/px4_sitl_default/build_gazebo-classic -j4
```

> The standalone `cmake` above is what this environment uses: it builds into
> `build/px4_sitl_default/build_gazebo-classic/` (reusing the mavlink headers the
> main PX4 build already generated there) and renders the SDFs in place. The
> canonical `make px4_sitl sitl_gazebo-classic` also works, but only if the main
> `build/px4_sitl_default` dir was configured with Gazebo found; this
> environment's was not (`gazebo_DIR-NOTFOUND`).

Verify:

```bash
ls Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf
ls build/px4_sitl_default/build_gazebo-classic/libgazebo_*.so
```

`run_px4_sitl.sh` fails fast with these exact messages if either is missing.

### 2. Build and source the catkin workspace

One catkin workspace, `rm_drone_ros`, is used. The `ego_planner`/`px4ctrl`
flight stack is symlinked into the `planner/` and `controller/` categories from
`third_party/Fast-Drone-250`, so a single build covers everything:

```bash
cd /home/idris-24-04/uav_ws/src/rmUAV/rm_drone_ros
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="ov_core;ov_msckf;ov_init;ov_data;ov_eval;vision_to_mavros;sitl_offboard;ego_planner;plan_env;path_searching;bspline_opt;traj_utils;quadrotor_msgs;px4ctrl;uav_utils;cmake_utils;offboard" -j4
source devel/setup.bash
```

The whitelist skips `d430_slam`/`apm_d430_bridge`, which still contain
machine-specific absolute paths and are not part of this chain.

Then confirm all of these resolve:

```bash
for p in ego_planner px4ctrl quadrotor_msgs ov_msckf vision_to_mavros sitl_offboard mavros; do rospack find "$p"; done
```

### Prometheus basic demos on Iris SITL

This is a software-interface/task-flow reproduction using the PX4 Iris airframe
and the `iris_vio` Gazebo sensor model. It is **not** a P450 dynamics match or a
hardware-flight validation. The Prometheus controller uses PX4 SITL local pose
for control feedback and Gazebo ground truth only for its `odom_valid` gate; VIO
is not part of this first pass.

Build the required Prometheus packages and only the basic demo executables (the
advanced SpireCV/gimbal demos remain excluded because their optional SDK is not
needed here):

```bash
catkin_make \
  -DCATKIN_WHITELIST_PACKAGES="prometheus_msgs;prometheus_gazebo;prometheus_uav_control;prometheus_demo;sitl_offboard" \
  -DPROMETHEUS_DEMO_BUILD_ADVANCED=OFF -j4
source devel/setup.bash
```

Start the simulation stack with one basic task selected:

```bash
roslaunch sitl_offboard prometheus_basic_sitl.launch demo:=enu_xyz_pos_control
```

Supported values for `demo` are `takeoff_land`, `enu_xyz_pos_control`,
`body_xyz_pos_control`, and `circular_trajectory_control`. Before starting the
task, verify `/uav1/mavros/state`, `/uav1/prometheus/state`, and
`/uav1/prometheus/control_state`. The launcher waits without arming until an
operator explicitly publishes GO:

```bash
rostopic pub -1 /prometheus/sitl_demo/go std_msgs/Empty "{}"
```

After GO, the SITL-only runner simulates the RC mode edges, arms, requests
Prometheus `RC_POS_CONTROL` then `COMMAND_CONTROL`, commands the configured
takeoff height, and starts the unmodified upstream demo. The selected demo then
performs its own task and land command. Stop the launch with Ctrl-C on any
unexpected movement; PX4 QGroundControl console/log output and Gazebo state are
the authoritative checks.

Do not use this runner on hardware. It injects fake RC input and automatically
arms after GO. The separate Prometheus source package is built in this same
catkin workspace; no joystick device or second setpoint publisher is launched.

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
args (defaults point at `/home/idris-24-04/px4_ws/PX4-Autopilot`).

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
  fcu_url:=udp://:14540@127.0.0.1:14557 \
  camera_topic:=/camera/left/image_raw \
  imu_topic:=/camera/imu \
  openvins_config:=$(rospack find sitl_offboard)/config/sitl_iris/estimator_config.yaml
```

Verify `/mavros/state` (connected), `/ov_msckf/odomimu`, and
`/mavros/vision_pose/pose` tracking it. For an isolated RC-free SITL run, add
`enable_rc_free_takeoff:=true` to px4ctrl.

### 4. Take off

```bash
rostopic pub -1 /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1"
```

Wait for PX4 to report OFFBOARD/armed and the takeoff to finish.

### 5. EGO-Planner

```bash
roslaunch sitl_offboard ego_px4ctrl.launch \
  odom_topic:=/ov_msckf/odomimu \
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
sim camera + IMU -> OpenVINS /ov_msckf/odomimu -> MAVROS /mavros/vision_pose/pose -> PX4 EKF2
                                             -> EGO odometry -> B-spline -> PositionCommand
sim depth + OpenVINS pose -------------------> EGO -> traj_server -> /position_cmd
MAVROS estimator flags + altitude -----------------------------------------> bootstrap gate
offboard_planner_tracker -> /mavros/setpoint_raw/local -> PX4 SITL
```

- **EGO camera pose is the VIO odometry.** The included
  `advanced_param_exp.xml` sets `grid_map/pose_type = 2` (ODOMETRY), so the
  depth-fusion camera pose comes from `~grid_map/odom` -> `/ov_msckf/odomimu`,
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
- Frame alignment between OpenVINS `global` and Gazebo `world` (gravity-aligned
  z-up, possible yaw offset) is acceptable for first validation; document any
  offset as a follow-up.
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
