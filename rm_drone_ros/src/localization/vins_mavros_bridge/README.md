# vins_mavros_bridge

The single source of local odometry for the flight stack. Any VIO/VINS writes
into it; `px4ctrl`, EGO-Planner and MAVROS external vision read out of it. The
controller and the planner never depend on a specific VIO package name.

## Why this package exists

Wiring a VIO topic straight into `px4ctrl` or EGO-Planner looks like a topic
remap but is not one. Two things break silently:

**1. The twist reference frame is not standardized.**

- `ov_msckf` publishes `/ov_msckf/odomimu` with `child_frame_id = "imu"` and
  `twist.linear` **in the body frame** (`Propagator.cpp` computes
  `quat_2_Rot(q_Gtoi) * v_iinG`; `ROS1Visualizer.cpp` marks it
  `// vel in local frame`).
- `uav_utils::extract_odometry` and `ego_replan_fsm.cpp` treat
  `twist.twist.linear` as **world frame**.
- VINS-Fusion publishes world-frame twist.

So the same controller receives a different velocity field depending on which
VIO is plugged in, with no error message. The symptom is hover drift and
sluggish velocity tracking. The bridge makes the frame an explicit parameter
(`input_twist_frame: body | world`) and always publishes **world-frame** twist,
which is what FD250's `px4ctrl` and EGO-Planner expect.

**2. Loop closure and re-initialization produce discontinuities.** A controller
integrating a continuous local estimate cannot absorb a jump. The bridge detects
them (`max_jump_m`, `max_jump_deg`) and either rejects the sample or absorbs it
into a world correction, depending on `jump_action`.

## Interface

### Input

| `input_mode` | Type | Notes |
|---|---|---|
| `odometry` (default) | `nav_msgs/Odometry` | Preferred: carries velocity and covariance |
| `pose_stamped` | `geometry_msgs/PoseStamped` | Position-only mode; zero twist is published |

The adapter configuration is entirely YAML — there is no per-algorithm C++
class. The candidates differ only in topic name, input child frame, twist
reference frame, and whether the angular rate is usable; all of those are
parameters.

### Output

| Topic | Type | Default | Purpose |
|---|---|---|---|
| `/localization/odometry` | `nav_msgs/Odometry` | on | Main interface. `frame_id = world`, `child_frame_id = base_link`, twist in the **world** frame |
| `/mavros/vision_pose/pose` | `PoseStamped` | on | MAVROS external vision (`publish_vision_pose`) |
| `/mavros/odometry/in` | `nav_msgs/Odometry` | off | Alternative MAVROS external vision (`publish_mavros_odometry`); twist converted back to body frame per REP-105 |
| `/vins_mavros_bridge/status` | `BridgeStatus` | on | State machine, measured rates, latency, rejection counters |
| `/vins_mavros_bridge/valid` | `std_msgs/Bool` | on | Latched; false means "do not trust this stream" |
| `/vins_mavros_bridge/path` | `nav_msgs/Path` | on | Debug trail, bounded by `path_max_poses` |
| TF `world -> base_link` | — | off | `publish_tf`; off by default to avoid fighting the VIO's own TF |

`publish_vision_pose` and `publish_mavros_odometry` are **mutually exclusive**.
Enabling both feeds one EKF2 from two paths and double-counts the measurement;
the node refuses to start if both are true.

### Frame conventions

- ROS world is ENU, body is FLU, per REP-105.
- The bridge does **not** perform ENU↔NED or FLU↔FRD conversion. MAVROS owns
  that boundary; doing it in both places is the classic double-rotation bug.
- `world_frame` defaults to `world`, not `odom`, because EGO-Planner hardcodes
  `grid_map/frame_id = world` and `traj_server` stamps commands with `"world"`.
  Keep this parameter equal to `grid_map/frame_id`.

## Adapter parameters

| Parameter | Meaning |
|---|---|
| `input_topic` | VIO output topic |
| `input_child_frame` | The frame the VIO's pose actually describes (`imu` for OpenVINS, `body` for VINS-Fusion) |
| `input_twist_frame` | `body` or `world` — the reference frame of `twist.linear` in the input |
| `input_twist_angular` | `false` when the VIO does not estimate angular rate (OpenVINS) |
| `base_to_input_xyz` / `base_to_input_rpy_deg` | Extrinsic from `base_link` to `input_child_frame`. **Use measured values, not the defaults** |

The pose chain applied is `T_world_base = T_world_input * inv(T_base_input)`.

## Safety behaviour

- Zero timestamp → substituted with `now()` and warned (this defeats latency
  accounting, so it is an upstream defect, not a supported mode).
- Timestamp moving backwards → treated as an estimator reset; the jump check is
  re-armed instead of firing spuriously.
- Non-finite values or a degenerate quaternion → dropped and counted.
- Input older than `input_timeout` → state goes `STALE`, **MAVROS output stops**,
  `/vins_mavros_bridge/valid` goes false. Stale poses are never silently
  republished. Recovery needs `recovery_frames` consecutive accepted samples.
- Jump beyond `max_jump_m` / `max_jump_deg` → `reject` (default) or
  `reset_origin`, which absorbs the discontinuity into a world correction and
  leaves the raw VIO output untouched. A rejected jump also drops the bridge to
  `STALE`: once the estimate has jumped, the stream is no longer the continuous
  local estimate the controller was consuming, so it must not keep being
  advertised as valid.
- Covariance diagonals are floored (`pose_covariance_floor`,
  `twist_covariance_floor`) so downstream consumers never see an
  over-confident estimate.
- Optional `output_rate` republishes the newest sample only, capped by
  `max_republish_age` so it cannot mask a dead estimator. Default 0 (off).
- `consistency_check` compares the output against `/mavros/local_position/odom`
  and warns past `consistency_tolerance`. A constant offset is expected (the VIO
  origin and the PX4 local origin differ); a growing one means EKF2 is not
  actually following this stream.

## Usage

```bash
roslaunch vins_mavros_bridge openvins.launch          # OpenVINS adapter
roslaunch vins_mavros_bridge vins_fusion.launch       # template; VIO not built yet
roslaunch vins_mavros_bridge bridge.launch config:=/path/to/adapter.yaml
```

Downstream consumers point at `/localization/odometry`:

```xml
<remap from="~odom" to="/localization/odometry" />             <!-- px4ctrl -->
<arg name="odometry_topic" value="/localization/odometry" />   <!-- EGO -->
```

## Verification without hardware

`scripts/synthetic_odom_publisher.py` reproduces OpenVINS's frame conventions,
so the normalization can be checked with no camera and no PX4:

```bash
rosrun vins_mavros_bridge synthetic_odom_publisher.py --mode yaw_test
roslaunch vins_mavros_bridge openvins.launch
rostopic echo -n1 /localization/odometry/twist/twist/linear
```

With yaw = 90 deg and input `twist.linear = (1, 0, 0)`, `input_twist_frame:
body` must produce world `(0, 1, 0)`; `world` must pass `(1, 0, 0)` through
unchanged. This is the only test that distinguishes the two modes — replaying a
bag cannot, because a pure rotation preserves the vector norm.

Other modes: `hover`, `translate`, `jump` (5 m step at t = 5 s), `divergent`
(runaway position, like the one available VIO bag), `silent` (stops after 3 s to
exercise the timeout).
