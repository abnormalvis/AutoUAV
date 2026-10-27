# rm_drone_ros

本目录是项目唯一的 ROS 1/catkin 工作区。开发机与机载电脑应检出相同主仓库提交，并使用 Git 记录的 submodule 提交，以保证源码一致。

## 目录

```text
rmUAV/
├── .gitmodules
├── rm_drone_ros/
│   └── src/
│       ├── perception/
│       │   ├── d430_slam/        # 本项目相机/VIO 包
│       │   ├── ov_core -> ../open_vins/ov_core
│       │   ├── ov_data -> ../open_vins/ov_data
│       │   ├── ov_eval -> ../open_vins/ov_eval
│       │   ├── ov_init -> ../open_vins/ov_init
│       │   ├── ov_msckf -> ../open_vins/ov_msckf
│       │   ├── realsense2_camera -> ../../../third_party/Fast-Drone-250/...
│       │   └── realsense2_description -> ../../../third_party/Fast-Drone-250/...
│       ├── planning/
│       │   ├── mission_planner/  # 显式触发的 PX4 航点任务基线
│       │   ├── ego_planner -> ../../../third_party/Fast-Drone-250/...
│       │   ├── plan_env -> ../../../third_party/Fast-Drone-250/...
│       │   ├── path_searching -> ../../../third_party/Fast-Drone-250/...
│       │   ├── bspline_opt -> ../../../third_party/Fast-Drone-250/...
│       │   └── traj_utils -> ../../../third_party/Fast-Drone-250/...
│       ├── controller/
│       │   ├── offboard/         # 本项目 OFFBOARD 示例与 tracker
│       │   └── px4ctrl -> ../../../third_party/Fast-Drone-250/...
│       ├── communication/
│       │   ├── apm_d430_bridge/
│       │   └── px4_d430_bridge/
│       ├── common/
│       │   ├── quadrotor_msgs -> ../../../third_party/Fast-Drone-250/...
│       │   ├── uav_utils -> ../../../third_party/Fast-Drone-250/...
│       │   └── cmake_utils -> ../../../third_party/Fast-Drone-250/...
│       ├── simulation/
│       │   └── sitl_offboard/    # SITL 模型、launch、config、脚本
│       ├── open_vins/            # OpenVINS submodule
│       ├── vision_to_mavros/     # VIO 到 MAVROS 的 submodule
│       ├── sqrtVINS -> ../../third_party/sqrtVINS
│       └── Prometheus/           # 控制/估计参考 submodule
└── third_party/
    └── Fast-Drone-250/           # 完整仓库；通过分类目录的符号链接暴露需要的包
```

类别目录仅用于组织 ROS 包，不改变 OpenVINS 原有包名、ROS topic/service 或第三方源码。OpenVINS、`vision_to_mavros` 和 Prometheus 的源码 checkout 保留在各自原位置；OpenVINS 的五个 ROS 包通过 `perception/` 下的相对链接接入 catkin。SqrtVINS 的唯一正式源码位于 `third_party/sqrtVINS`，`src/sqrtVINS` 是指向它的相对链接。SqrtVINS 是 OpenVINS 的派生实现，其重复基础包在 `package.xml`、CMake project 和内部依赖中分别命名为 `sqrt_vins_core`、`sqrt_vins_data`、`sqrt_vins_eval`，算法包保持 `ov_srvins`，从而允许两套 VIO 在同一工作区并存。其 CMake library/executable targets 加了 SqrtVINS 前缀，安装后的节点/工具文件名保持兼容。RealSense 只通过链接暴露驱动及描述包。Fast-Drone-250 的 EGO-Planner 依赖、`px4ctrl` 和公共工具包通过各职能目录下的相对符号链接参与构建。

`mission_planner`、`px4ctrl` 和 `offboard` 中的 tracker 都可能向 PX4 请求模式或发送控制设定值。一次飞行只能启动一个飞控控制者；具体输入输出与 SITL 启动顺序见 `simulation/sitl_offboard/README.md`。

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

使用与机载电脑相同的 Ubuntu、ROS 1 发行版和系统架构。先安装对应 ROS 发行版、catkin 工具以及 RealSense SDK `librealsense2`，然后从工作区根目录构建：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd <path-to-rmUAV>/rm_drone_ros
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

如果使用 `catkin build`，需先安装 `catkin_tools`，并在该工作区初始化构建配置。

OpenVINS 和 SqrtVINS 可一起编译。`ov_init` 与 `ov_srvins` 需要 Ceres Solver；确认安装匹配的 Ceres 开发包后，可显式构建两套视觉里程计：

```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="ov_core;ov_data;ov_eval;ov_init;ov_msckf;sqrt_vins_core;sqrt_vins_data;sqrt_vins_eval;ov_srvins" -j4
```

SqrtVINS 的 D455 配置和 live launch 可作为起点，图像/IMU 话题需先与相机实际输出核对，外参及噪声参数需按本机标定更新：

```bash
roslaunch ov_srvins subscribe.launch \
    config_path:=$(rospack find ov_srvins)/../config/rs_d455/estimator_config.yaml \
    topic_camera0:=/camera/color/image_raw topic_imu:=/camera/imu \
    max_cameras:=1 use_stereo:=false
```

这只启动 SqrtVINS 估计器，不启动 RealSense、MAVROS 或 PX4 控制器；不要将示例 D455 配置视为已完成的实机标定。

## 当前构建限制

- `prometheus_demo` 默认需要可选 SpireCV headers 才能构建其高级 gimbal demo。本工作区提供 `PROMETHEUS_DEMO_BUILD_ADVANCED=OFF` 选项用于只构建 basic tutorials；Prometheus SITL 任务入口见 `simulation/sitl_offboard/README.md`。
- `ov_init` 和 `ov_srvins` 需要 Ceres Solver；当前开发环境未提供 `CeresConfig.cmake`，因此包含这些包的完整白名单构建会在 CMake 配置阶段失败。部署环境需安装与 ROS/Ubuntu 版本匹配的 Ceres 开发包后再执行构建。
- SqrtVINS 的唯一源码入口已统一到 `third_party/sqrtVINS` submodule，包名兼容改动已应用到该 submodule 工作树。当前尚未创建 submodule commit 或更新主仓库 gitlink；在提交这两项之前，其他干净检出不会包含本次修改。
- `d430_slam` 的 CMake 和 ORB-SLAM3 初始化代码仍含旧设备上的 `/home/nvidia/...` 绝对路径，并依赖外部安装的 ORB-SLAM3、Pangolin 和特定 OpenCV 版本。在这些路径/依赖改为可配置之前，不能承诺在新机器上全量构建成功。
- PX4 与 APM 的 MAVROS 启动配置目前仍需分开核对；包被 catkin 发现不代表飞控链路已配置或已验证。
- `planner/mission_planner` 当前只提供显式触发的 PX4 OFFBOARD 起飞、局部水平航点、悬停和返航基线；不含目标检测、目标定位、避障或自动降落。先在 PX4 SITL 验证，不要直接用于实机。
- 首次部署应先做干净检出和构建验证，再接入飞控。不要将未验证的工作区直接用于实机自主飞行。