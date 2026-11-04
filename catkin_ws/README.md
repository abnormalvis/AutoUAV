# catkin_ws

本目录是项目唯一的 ROS 1/catkin 工作区。开发机与机载电脑应检出相同主仓库提交，并使用 Git 记录的 submodule 提交，以保证源码一致。

## 目录

```text
rmUAV/
├── catkin_ws/src/              # 唯一 catkin 源码空间，只放 ROS 包与符号链接
│   ├── localization/
│   │   ├── vins_mavros_bridge/   # 统一 VIO 里程计接口
│   │   └── vins_fusion/          # 指向 third_party/VINS-Fusion 的符号链接
│   ├── planning/
│   │   ├── mission_planner/      # 显式触发的 PX4 航点任务基线
│   │   └── ego_planner, plan_env, path_searching, bspline_opt, traj_utils -> third_party/Fast-Drone-250
│   ├── control/
│   │   ├── offboard/             # 本项目 OFFBOARD 示例与 tracker
│   │   ├── simple_offboard_forward/
│   │   └── px4ctrl -> third_party/Fast-Drone-250
│   ├── perception/
│   │   ├── armor_perception/     # 装甲板检测 → 世界系位姿 → 观察点
│   │   └── d430_slam/            # 含 CATKIN_IGNORE，见下文构建限制
│   ├── communication/            # apm_d430_bridge、px4_d430_bridge
│   ├── common/                   # quadrotor_msgs、uav_utils、cmake_utils -> Fast-Drone-250
│   ├── simulation/               # sitl_offboard，以及旧的 fast_drone_realflight（见 docs/deploy.md）
│   └── vendor/                   # open_vins submodule 及其 ov_* 符号链接
├── config/                     # d430_schurvins/（SchurVINS 标定与 launch）、systemd/（单元与 env 示例）
├── launch/                     # 根级入口，如 realflight_ego_vio_px4ctrl.launch
├── scripts/                    # start_*.sh、check_chain.sh、install_systemd.sh、set_origin.py
├── docs/                       # 部署、布局、集成计划等文档
├── data/ bags/ logs/ tmp/      # 本地运行数据，被 .gitignore 排除
├── third_party/                # Fast-Drone-250、VINS-Fusion 子模块；参考检出被忽略
└── svo_ws/ ob_ws/ rm_drone_sim/ files/ models/   # SchurVINS 独立工作区、归档与素材
```

类别目录仅用于组织 ROS 包，不改变 OpenVINS 原有包名、ROS topic/service 或第三方源码。OpenVINS 的源码 checkout 位于 `vendor/open_vins`，五个 ROS 包通过 `vendor/` 下的相对链接接入 catkin。Fast-Drone-250 的 EGO-Planner 依赖、`px4ctrl` 和公共工具包通过各职能目录下的相对符号链接参与构建。

`third_party/` 下的 `fast-drone-250_cxr`、`raicom-air-recon-2026`、`NMPC-QUA`、`Prometheus` 只作参考，已被 `.gitignore` 排除，不参与构建。

### 定位接口

`localization/vins_mavros_bridge` 是唯一对外提供 odometry/pose 的模块。它订阅某一种 VIO 的输出，统一输出：

```text
/localization/odometry        nav_msgs/Odometry   world -> base_link
/mavros/vision_pose/pose      geometry_msgs/PoseStamped
```

`px4ctrl` 的 `~odom`、EGO-Planner 的 `~odom_world` / `~grid_map/odom` 都应指向 `/localization/odometry`，不再直接订阅某个 VIO 包的话题。桥接层负责三件下游不该重复做的事：坐标外参合成、把 `twist.linear` 统一到世界系、超时/跳变/非法值检测。详见该包 README。

`vision_to_mavros` 仍在工作区，但 OpenVINS 主链路已不再使用它。它的 `openvins_to_mavros_node` 只是把 `/ov_msckf/odomimu` 原样转发到 `/mavros/vision_pose/pose`，而 OpenVINS 的 `twist.linear` 是机体系 —— 位姿本身没问题，速度参考系是错的。这条路径现在由 `vins_mavros_bridge` 承担，它在转发前把速度归一到世界系，并补上超时、跳变和非法值检查。`mavros_realsense_openvins.launch` 与 `start_mavros_realsense_openvins.sh` 已改为直接启动桥接层。

它剩下的独有功能是 T265 / AprilTag / D430 的 launch 和 `calibrate_extrinsics.py`。注意两者都能发 `/mavros/vision_pose/pose`：`vins_mavros_bridge` 只能拦住自己的两个出口（`publish_vision_pose` 与 `publish_mavros_odometry`），拦不住另一个进程，**同时启动会让 EKF2 从两条路径收到外部视觉**，只能靠 launch 纪律保证。

`mission_planner`、`px4ctrl` 和 `offboard` 中的 tracker 都可能向 PX4 请求模式或发送控制设定值。一次飞行只能启动一个飞控控制者；具体输入输出与 SITL 启动顺序见 `simulation/sitl_offboard/README.md`。

Fast-Drone-250 实机 EGO-Planner 链路可用 `roslaunch launch/realflight_ego_vio_px4ctrl.launch localization:=openvins`（在仓库根目录执行） 或 `localization:=schurvins` 启动。先单独启动 MAVROS、相机和所选 VIO；该入口负责启动对应桥接器、EGO-Planner 与 `px4ctrl`。详细顺序及验证项见 `simulation/fast_drone_realflight/README.md`。VINS-Fusion 用户仍可直接使用原 `single_run_in_exp.launch` 与 `run_ctrl.launch`，不改参数时保持原话题默认值。

### 感知接口

`perception/armor_perception` 是装甲板定点观察任务的感知节点。它订阅 `/camera/left/image_raw`、`/localization/odometry` 和 `/vins_mavros_bridge/valid`，输出：

```text
/perception/armor_targets        armor_perception/ArmorTargetArray   本帧全部检测
/perception/target_pose          PoseWithCovarianceStamped           跟踪目标的世界系位姿
/perception/observation_point    PoseStamped                         观察点
/perception/task_state           std_msgs/String                     CRUISE/CAPTURE/OBSERVE/LOST_HOLD
```

**它不发布 `/position_cmd`。** 感知只报告位姿与观察点，由任务层决定飞巡航航点、飞观察点还是原地保持 —— 上面「一次飞行只能启动一个飞控控制者」的约束同样适用于轨迹来源。定位链路接入方式是：本包把 `/localization/odometry` 在**图像时间戳**处插值出 `T_world_base`，再乘静态外参得到 `T_world_camera`，所有量测直接落到世界系，因此不需要单独的 VIO 自运动补偿模块。

该包的检测核心复用自已归档的 `ob_ws/src/sp_vision_25`（云台自瞄工程），但复制到包内 `thirdparty/sp_vision/` 并逐处打补丁，因为上游顶层 CMake 无法作为 catkin 子目录嵌入（全局 `CMAKE_BUILD_TYPE`、硬编码 OpenVINO、厂商相机 SDK）。改动清单见该目录的 README。

## 机载启动（scripts/ 与 config/）

顶层 `scripts/` 提供 systemd 启动脚本，链路为 MAVROS → D430 红外双目 → SchurVINS（使用飞控 IMU `/mavros/imu/data_raw`）→ `vins_mavros_bridge` → EGO-Planner + `px4ctrl`。SchurVINS 的 D430 实时入口在 顶层 `config/d430_schurvins/live_d430_px4imu.launch`（svo_ws 是独立仓库，故放在本工作区跟踪），标定模板为同目录的 `d430_stereo.yaml`（占位值，飞行前必须标定）。安装、配置和检查见 [`docs/deploy.md`](../docs/deploy.md)。

## 克隆与更新

从主仓库根目录克隆：

```bash
git clone --recurse-submodules <rmUAV-repository-url>
```

已有检出则运行：

```bash
git submodule update --init --recursive
```

切换主仓库提交后也应执行 submodule 更新。请提交主仓库中的 submodule gitlink 变更；不要在机载端自行跟踪 submodule 的最新分支，否则两端源码可能不一致。

## 构建

使用与机载电脑相同的 Ubuntu、ROS 1 发行版和系统架构。除 ROS Noetic、catkin 工具和 RealSense SDK `librealsense2` 外，还需要：

- `ros-noetic-mavros-extras`：提供 `/mavros/vision_pose/pose` 与 `/mavros/odometry/in` 插件。缺少它时这两个话题根本不会出现，外部视觉链路是断的。
- `libceres-dev`：`ov_msckf` 和 `ov_init` 的 CMakeLists 都有 `find_package(Ceres REQUIRED)`。

从工作区根目录构建：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd <path-to-rmUAV>/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

如果使用 `catkin build`，需先安装 `catkin_tools`，并在该工作区初始化构建配置。

只构建视觉里程计：

```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="ov_core;ov_data;ov_eval;ov_init;ov_msckf" -j4
```

`perception/armor_perception` 只依赖核心 ROS 包（`roscpp`、`cv_bridge`、`nav_msgs`、`sensor_msgs`、Eigen、yaml-cpp），不依赖 `mavros_extras`、Ceres 或 OpenVINO。因此在上面第 109 行的 `mavros_extras` 装好之前，它仍可单独构建：

```bash
cd catkin_ws && catkin_make --pkg armor_perception
```

它的源码树还自带一个独立的容器内构建脚本，用于在没有完整工作区（或本机没装 ROS）时验证：

```bash
catkin_ws/src/perception/armor_perception/build.sh --tests
catkin_ws/src/perception/armor_perception/build.sh --chain   # 端到端坐标链自检
```

## 当前构建限制

- `perception/d430_slam` 已加 `CATKIN_IGNORE`。其 CMake 和 ORB-SLAM3 初始化代码仍含旧设备上的 `/home/nvidia/...` 绝对路径，并依赖外部安装的 ORB-SLAM3、Pangolin 和特定 OpenCV 版本。该包是一个 RealSense D430 双目 ORB-SLAM3 节点，只发布 `PoseStamped`（`/d430_camera/pose`），不含速度；接入时用 `vins_mavros_bridge` 的 `input_mode: pose_stamped`。当前不参与构建。
- SchurVINS 源码及其依赖不参与 `catkin_ws` 构建，需在独立工作区启动估计器。控制链路适配器已提供 `schurvins.yaml`，默认订阅 `/svo/pose_imu` 并基于位姿差分估计速度；使用前确认实际 SchurVINS 输出与坐标轴约定符合配置，并标定 IMU 外参。通过 `fast_drone_realflight` 启动器可选 `localization:=schurvins` 接入 EGO-Planner 和 `px4ctrl`。
- RealSense 驱动包（`realsense2_camera`、`realsense2_description`）的符号链接已移除，不再从本工作区构建。需要时由独立的 RealSense 工作区提供。
- `ov_init` 和 `ov_msckf` 需要 Ceres Solver；未安装 `libceres-dev` 时 CMake 配置阶段会直接失败。
- `communication/` 下的 `apm_d430_bridge`、`px4_d430_bridge` 含旧机器（Jetson）的串口默认值，不参与主构建；`px4_d430_bridge` 的 `fcu_url` 默认已改为 `/dev/ttyACM0`，可用 launch 参数覆盖。
- PX4 与 APM 的 MAVROS 启动配置目前仍需分开核对；包被 catkin 发现不代表飞控链路已配置或已验证。
- `planner/mission_planner` 当前只提供显式触发的 PX4 OFFBOARD 起飞、局部水平航点、悬停和返航基线；不含目标检测、目标定位、避障或自动降落。先在 PX4 SITL 验证，不要直接用于实机。
- `localization/vins_mavros_bridge` 的接口行为已用合成输入验证，并在 Gazebo/PX4 SITL 里跑通了 `Gazebo → PX4 → MAVROS → 桥接层 → px4ctrl` 全链路（EKF2 跟随误差约 5 mm）。尚未在实机或数据集上验证定位质量；`config/*.yaml` 里的外参是占位值，必须用实机标定结果替换。SITL 里的起飞动作还没跑通，见 `simulation/sitl_offboard/README.md`。
- 首次部署应先做干净检出和构建验证，再接入飞控。不要将未验证的工作区直接用于实机自主飞行。
