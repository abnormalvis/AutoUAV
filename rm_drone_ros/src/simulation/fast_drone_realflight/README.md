# fast_drone_realflight

`ego_vio_px4ctrl.launch` connects an already-running OpenVINS or SchurVINS
estimator to the Fast-Drone-250 EGO-Planner and `px4ctrl` flight stack through
`vins_mavros_bridge`.

The launch starts the selected bridge, EGO-Planner, and `px4ctrl`. It does not
start MAVROS, the RealSense driver, or either VIO estimator. Start those
upstream processes first and verify the selected raw VIO topic is publishing.
This avoids accidentally starting a second MAVROS or camera instance.

For OpenVINS, start MAVROS and the camera once, then start `ov_msckf` with the
calibrated estimator config and the actual image/IMU topics, for example:

```bash
roslaunch ov_msckf subscribe.launch \
  config_path:=/path/to/estimator_config.yaml \
  topic_camera0:=/camera/infra2/image_rect_raw \
  topic_imu:=/camera/imu \
  use_stereo:=false max_cameras:=1
```

The topic names above are examples: match them to the active camera driver and
OpenVINS calibration. Do not use the existing `rspx4.sh` unchanged for this
setup, because it also starts VINS-Fusion; start only its required MAVROS and
camera components instead. For SchurVINS, start its estimator from its own
workspace and confirm it publishes the configured pose topic before continuing.

With the ROS workspace sourced, use one of:

```bash
roslaunch fast_drone_realflight ego_vio_px4ctrl.launch localization:=openvins
roslaunch fast_drone_realflight ego_vio_px4ctrl.launch localization:=schurvins
```

OpenVINS must publish `/ov_msckf/odomimu`; SchurVINS must publish
`/svo/pose_imu`. Override `input_topic` or `bridge_config` if the selected
estimator uses a different topic or needs a different adapter configuration.
The SchurVINS estimator itself is external to this workspace and must be
started in its own environment.

Both downstream nodes consume `/localization/odometry`. The bridge also
publishes MAVROS external vision by default. Confirm `/vins_mavros_bridge/valid`
is `true`, inspect `/localization/odometry`, and confirm PX4 is fusing external
vision before issuing the `/px4ctrl/takeoff_land` command. Calibrate the
base-to-IMU extrinsics in the selected bridge config before flight; the
repository defaults are placeholders.

The EGO-Planner experiment launch uses `flight_type=1`, so select a goal in
RViz after takeoff/arming is handled according to the aircraft's procedure.
This launch does not issue the takeoff command.
