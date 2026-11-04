# armor_perception

装甲板定点观察的感知节点：检测静止装甲板，估计其在世界系中的 6-DOF 位姿，
并生成一个安全的观察点。**非接触**——只发布位姿与观察点，从不发布飞行指令。

## 这个包解决什么

上游 `ob_ws/src/sp_vision_25` 是云台自瞄工程，它的 `Solver` / `Target` /
`Tracker` 有三个前提在无人机上不成立：

1. `solver.cpp` 的 `xyz_in_world = R_gimbal2world_ * xyz_in_gimbal` **没有平移项**。
   云台的旋转中心就是世界原点时成立；无人机一平移，目标世界坐标整体偏掉，
   而且不报错——看起来像检测精度问题。
2. `Target` 的 11 维状态 `[x vx y vy z vz a w r l h]` 建模的是"绕自身 yaw 轴
   旋转的四面装甲"。静止单板的 `r` 不可观测，靠初值维持、被过程噪声推着走，
   一旦漂出 `[0.05, 0.5]` 就被 `diverged()` 判定发散，跟踪被打断。
3. `Tracker` 引入了 `omniperception` → `io/usbcamera` → 厂商相机 SDK。

所以这里**只复用无状态的检测核心**（灯条检测、装甲板几何、数字分类），
重写了位姿求解与状态估计。逐文件的改动记录在
[`thirdparty/sp_vision/README.md`](thirdparty/sp_vision/README.md)。

| 复用（复制 + 打补丁） | 新写 |
|---|---|
| `auto_aim::Detector` 传统灯条检测 | `ArmorSolver`：世界系 PnP，带平移 |
| `auto_aim::Armor` / `Lightbar` 数据结构 | `StaticTarget`：笛卡尔常速模型 |
| `auto_aim::Classifier` 数字分类（默认关闭） | `Tracker`：保留上游状态机，换掉估计器 |
| `tools::ExtendedKalmanFilter` | `OdometryBuffer`：图像时刻插值 |
| `tools::{math,img}_tools` | `ObservationPointGenerator` |

## 自运动补偿怎么做的

没有单独的"补偿模块"。每帧在**图像时间戳**处取 `T_world_base`，再乘上静态外参：

```
T_world_camera(t) = T_world_base(t) · T_base_camera
T_world_armor    = T_world_camera(t) · T_camera_armor
```

所有量测直接进世界系，滤波器看到的就是静止目标，相机运动被自然消掉。
补偿质量完全取决于时间对齐精度——这就是 `OdometryBuffer` 存在的理由：

- 位置线性插值、姿态 slerp，不是"取最新一帧"。1 m/s 飞行、30 ms 偏差就是
  3 cm 的目标位置误差，且随速度增长。
- 图像比最新里程计新超过 `~max_odom_age`（默认 0.05 s）→ **丢弃该帧**。
  外推位姿比丢帧更糟，误差正比于速度。
- `/vins_mavros_bridge/valid` 为 false → 同样丢弃。

## 坐标系（最容易错的地方）

Gazebo 相机 link 用传感器坐标系：**+X 前、+Y 左、+Z 上**。
OpenCV PnP 要光学坐标系：**+Z 前、+X 右、+Y 下**。

```
R_optical_from_body = [[0, -1,  0],
                       [0,  0, -1],
                       [1,  0,  0]]
```

搞错这一步，检测依然完美、debug 图依然正常，但每一个位姿都是转过的。
`frames.hpp` 单独把它抽出来并有单测，端到端还有 `scripts/synthetic_chain_test.py`
用独立写出的 REP-103 约定反算一遍。

## 话题

| 方向 | 话题 | 类型 |
|---|---|---|
| sub | `/camera/left/image_raw` | `sensor_msgs/Image` |
| sub | `/camera/left/camera_info` | `sensor_msgs/CameraInfo` |
| sub | `/localization/odometry` | `nav_msgs/Odometry` |
| sub | `/vins_mavros_bridge/valid` | `std_msgs/Bool`（latched） |
| pub | `/perception/armor_targets` | `armor_perception/ArmorTargetArray` |
| pub | `/perception/target_pose` | `geometry_msgs/PoseWithCovarianceStamped` |
| pub | `/perception/observation_point` | `geometry_msgs/PoseStamped` |
| pub | `/perception/task_state` | `std_msgs/String`（latched） |
| pub | `/perception/debug_image` | `sensor_msgs/Image`（`~publish_debug`） |

`/perception/task_state` 取值 `CRUISE` / `CAPTURE` / `OBSERVE` / `LOST_HOLD`。
**感知节点不控制飞行**：由任务层决定飞巡航航点、飞观察点，还是原地保持。
这样任一时刻只有一个轨迹生产者（`catkin_ws/README.md` 的既有约束）。

`LOST_HOLD` 的语义是在最后已知位置悬停，**不回退巡航**——回退会让状态在
每个闪断之间来回切换。

## 构建

本机没有 ROS，构建在项目 Docker 镜像里做：

```bash
./build.sh            # 构建
./build.sh --tests    # 构建并跑单测
./build.sh --chain    # 构建并跑端到端合成链路自检
```

脚本在**独立**的 catkin 工作区里构建（容器内 `/tmp/ws`），不是 `catkin_ws` 本身。
原因：整个工作区在当前镜像里配不通过——`communication/apm_d430_bridge`
需要 `mavros_extras`，镜像里没装。本包只依赖核心 ROS 包，独立工作区足够，
而且快得多。要把本包接进 `catkin_ws` 一起构建，先装
`ros-noetic-mavros-extras`。

## 验证

分四级，每级通过再进下一级。

### a. 单元测试

```bash
./build.sh --tests
```

35 个用例，覆盖：里程计插值/陈旧丢弃/乱序、静止目标不漂移/离群值门限/发散判据/
yaw 低通与跨 π 回绕、PnP 往返（含"同一块板从三个不同机位看过去必须落到同一个
世界点"）、观察点法向与限位、光学坐标系转换。

### b. 坐标链自检（不需要 Gazebo）

```bash
./build.sh --chain
```

用独立写出的 REP-103 约定渲染装甲板、发布合成相机流与里程计，断言
`/perception/target_pose` 落到真值上。**这是抓光学坐标系错误和外参错误的
唯一可靠手段**——检测在 debug 图上看着完全正常。0.9 m 处实测误差 2.8 mm。

也可以在有 ROS 的环境里手动跑：

```bash
roslaunch armor_perception armor_perception_sitl.launch &
rosrun armor_perception synthetic_chain_test.py
```

### c. SITL 位姿精度

```bash
roslaunch sitl_offboard sitl.launch \
  world:=$(find sitl_offboard)/worlds/armor_observe.sdf
# 另开终端，接上 VIO
roslaunch vins_mavros_bridge openvins.launch
roslaunch armor_perception armor_perception_sitl.launch publish_debug:=true
```

`worlds/armor_observe.sdf` 把装甲板放在 `(3.0, 0, 1.2)`、yaw = π，正对起飞点。
真值直接量：

```bash
rosrun armor_perception evaluate_pose.py --truth 3.0 0.0 1.2 --truth-yaw 3.14159265358979
```

判据：位置误差均值 < 0.10 m（1–2 m 工作距离），yaw 误差用于判断观察点是否可信。

> **SITL 的检测距离只有约 3 m。** `iris_vio` 是 86° 宽角、fx≈455，灯条 56 mm
> 在 3 m 处只有 8.5 px，正好卡在 `min_lightbar_length: 8` 上。实机窄视场
> （`uav.yaml` 的 fx≈2925）约 20 m，两者差 6 倍，**SITL 验证不了远距离检测**。
> 详见实施计划的像素预算表。

### d. 自运动补偿

让无人机绕目标飞 0.5 m/s 圆弧，看 `/perception/target_pose` 的世界坐标漂移；
标准差应 < 5 cm。把 `~max_odom_age` 调大（例如 5.0）让节点退化成"用最新一帧
里程计"，误差应明显变大——这是这个设计有效的证据。

### e. 闭环

观察点 → `/position_cmd` → `px4ctrl`。**前提是 SITL 起飞链路可用**；
`sitl_offboard/README.md` 记录起飞尚未端到端验证（解锁后约 5 s 内 disarm）。
a–d 都不需要飞行，可以先做完。

## 已知限制

1. **单目深度误差随距离平方增长**：`σ_depth ≈ r²·σ_px / (f·W)`。0.9 m 处 2.8 mm，
   1.9 m 处约 78 mm（实测）。所以工作距离限制在 1–3 m，距离只作接近判断。
2. **近正视时 yaw 不可观测**（平面 PnP 翻转二义性）。用 IPPE 双解 + 时序选解，
   并在法向不可信时把观察点退化为"沿当前视线后退"。没有用上游的
   `optimize_yaw`：它在 140° 内搜索使重投影最小的 yaw，而 `reproject_armor`
   把装甲板 pitch 硬编码为 ±15°（"装在竖直圆柱上"的假设），静止平板不成立。
3. **数字分类默认关闭**（`use_classifier: false`）。SITL 的板没有可读数字，
   而分类器要拖一个模型文件进来。几何足够定位一块已知的板。
4. **`vendor/ov_*` 五个符号链接指向 `open_vins/`**，即 `catkin_ws/src/vendor/open_vins`
   这个 submodule。它们此前是悬空的（checkout 被移到 `third_party/open_vins` 且
   `.git/modules` 里的 `core.worktree` 未同步），现在已移回原路径并解析正常。
   移动这些路径前先跑 `git submodule status`，它会把 `core.worktree` 与实际位置
   不一致直接报出来。
