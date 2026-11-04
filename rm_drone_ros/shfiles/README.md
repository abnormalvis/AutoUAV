# shfiles：机载启动脚本与 systemd

面向香橙派 5（Ubuntu 20.04 + ROS noetic 原生）。链路：

```text
roscore → mavros(PX4, /dev/ttyACM0) → D430 红外双目 → SchurVINS(/svo/pose_imu)
        → vins_mavros_bridge(/localization/odometry, /mavros/vision_pose/pose) → EGO-Planner + px4ctrl
```

D430 没有 IMU，SchurVINS 使用飞控 IMU `/mavros/imu/data_raw`。

## 文件

| 文件 | 作用 |
|---|---|
| `common.sh` | 公共环境与等待函数（被 source） |
| `start_roscore.sh` / `start_mavros.sh` / `start_camera.sh` / `start_vio.sh` / `start_stack.sh` | 各阶段入口，每个阶段先等待上游话题 |
| `check_chain.sh` | 只读链路检查，输出 PASS/FAIL |
| `install_systemd.sh` | 安装/卸载 systemd 单元 |
| `systemd/` | `rm-drone@.service` 模板、`rm-drone.target`、`rm_drone.env.example` |

## 安装

```bash
cd <rmUAV>/rm_drone_ros
sudo shfiles/install_systemd.sh --user <运行用户>   # 加 --enable 则开机自启
sudo nano /etc/rm_drone/rm_drone.env                # 按 env 示例修改
sudo usermod -aG dialout <运行用户>                 # 访问 /dev/ttyACM0
sudo systemctl start rm-drone.target
journalctl -fu 'rm-drone@*'
```

先分别编译 `rm_drone_ros`（catkin_make）和 `svo_ws`（catkin build），两者不要互相 overlay。RealSense 驱动在独立工作区，用 `CAMERA_SETUP` 指向其 `setup.bash`。

## 飞行前必须完成

1. **相机-IMU 标定**：用 Kalibr 标定红外双目 + `/mavros/imu/data_raw`，填入
   `config/d430_schurvins/d430_stereo.yaml`（内参、`T_B_C`、`delay_imu_cam`、噪声）。文件里的值全是占位。
2. **桥接外参**：`vins_mavros_bridge/config/schurvins.yaml` 的 `base_to_input_*`（IMU 在 base_link 中的位置/姿态）。按文件内注释平移机体验证轴向。
3. **EGO-Planner 深度内参**：`third_party/Fast-Drone-250` 的 `single_run_in_exp.launch` 写死了 `fx/fy/cx/cy` 与 `/camera/depth/image_rect_raw`。请用 `/camera/depth/camera_info` 的实际值（与相机分辨率一致）。该文件属于 submodule，需要时通过 launch 参数覆盖，不要直接改第三方源码。
4. **EKF2 外部视觉**：`localization/vins_mavros_bridge/scripts/enable_px4_external_vision.sh` 默认只读检查，`--apply` 才写参数；不会在开机时自动执行。
5. IMU 速率：`start_mavros.sh` 启动后通过 `set_stream_rate` 设为 `IMU_RATE_HZ`（默认 200）。若日志出现 WARNING，请确认 `/mavros/imu/data_raw` ≥ 100 Hz。
6. 同一时刻只能有一个飞控控制者和一个外部视觉来源。

## 检查与排错

```bash
shfiles/check_chain.sh schurvins
```

上机反馈时请附：`check_chain.sh` 输出、`journalctl -u 'rm-drone@*' --no-pager -n 200`、`rostopic hz` 的 IMU/图像/`/svo/pose_imu`。

常见问题：
- `rm-drone@vio` 反复重启：多半在等相机或 IMU 话题，先看 camera/mavros 服务。
- `/dev/ttyACM0` 无权限或设备名变化：加入 dialout 组，或用 udev 规则固定名称后设置 `FCU_URL`。
- 脚本不发起飞指令，起飞需按机体流程手动触发。
