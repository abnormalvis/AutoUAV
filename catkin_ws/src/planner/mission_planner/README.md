# mission_planner

This package provides an explicitly triggered PX4 waypoint baseline for SITL validation. It does not perform target detection, target localization, automatic landing, or autonomous takeoff on node startup.

## Inputs and outputs

- Subscribes to `/mavros/state` (`mavros_msgs/State`).
- Subscribes to `/mavros/local_position/pose` (`geometry_msgs/PoseStamped`).
- Publishes the only mission position setpoint stream to `/mavros/setpoint_position/local`.
- Publishes a latched state string on `/mission/state`.
- Exposes `/mission/start` and `/mission/return` as `std_srvs/Trigger` services.

The mission start service captures the current local pose, pre-streams the takeoff setpoint, requests `OFFBOARD`, then arms after PX4 reports the requested mode. It climbs to the configured relative altitude, flies along local +X until the configured horizontal displacement is reached, and holds at the search waypoint. Call `/mission/return` to return horizontally to the captured takeoff position at the same altitude. Landing remains manual.

## SITL-only initial validation

Build this package in a ROS Noetic workspace with MAVROS dependencies available, then launch PX4 SITL and MAVROS with a valid local-position estimate before starting the mission node:

```bash
roslaunch mission_planner waypoint_mission.launch
rostopic echo /mavros/state
rostopic echo /mavros/local_position/pose
rostopic info /mavros/setpoint_position/local
rosservice call /mission/start
rostopic echo /mission/state
rosservice call /mission/return
```

The default search displacement is 11 m, with a 1.5 m takeoff rise. Both are private launch parameters and should first be exercised in SITL.

## Safety limits

- The node does not start, arm, or request a flight mode until `/mission/start` is explicitly called.
- Stale local pose or FCU disconnect moves the mission to `ABORTED` and requests the configured fallback mode (`POSCTL` by default), while continuing to stream the last setpoint until PX4 reports the fallback mode. Verify the fallback and RC takeover behavior for the exact PX4 build and estimator setup.
- If the local estimate is lost, the vehicle may not be able to safely hold or return. Maintain a tested RC takeover path and do not use this node on hardware until the SITL failure cases and vehicle-specific failsafes are validated.
- This baseline has no target detector, relative target pose, obstacle avoidance, geofence, battery handling, or automatic landing. Do not connect a vision controller as a second publisher to the same setpoint topic.
- The mission coordinate axes are those of MAVROS local position. Confirm the local frame orientation and +X direction in SITL before interpreting the displacement as a physical search direction.