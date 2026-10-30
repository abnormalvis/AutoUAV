# UAV Stack 集成方案

## 1. 背景与目标

当前工作区位于 `rmUAV`，目标目录为：

- `/home/idris/uav_ws/src/rmUAV/rm_drone_ros`
- 参考目录：
  - `/home/idris/uav_ws/src/rmUAV/third_party/fast-drone-250_cxr`
  - `/home/idris/uav_ws/src/rmUAV/third_party/raicom-air-recon-2026`

当前已经具备多套可参考结构：

- OpenVINS 相关包
- `vision_to_mavros`
- Prometheus 相关控制/规划模块
- Fast-Drone-250 中的 px4ctrl、EGO-Planner 相关算法代码
- RAICOM 的 `px4ctrl`、`simple_offboard_forward`、`fastlio_mavros_bridge`

本次方案的核心目标是：

1. 不再继续沿用 `sqrtVINS` 作为主线，优先评估 VINS-Fusion / SchurVINS / ORB-SLAM3 的实机可用性。
2. `px4ctrl` 和 `ego_planner` 采用 Fast-Drone-250 CXR 的演进版本进行适配，并统一到 ROS1 / PX4 1.14 生态。
3. `simple_offboard_forward` 作为飞控验证和离线测试工具，负责短距离 OFFBOARD 基础链路验证。
4. 新增 `vins_mavros_bridge`，统一封装 VIO/VINS 输出到统一里程计/位姿接口。
5. 形成一个“可替换 VIO + 稳定 PX4 控制 + 统一 odometry bridge”的 ROS1 无人机基础架构。

---

## 2. 当前现状盘点

### 2.1 现有工程结构

当前 `rm_drone_ros` 的源代码目录中，已经有以下模块：

- `src/open_vins/`
- `src/vision_to_mavros/`
- `src/Prometheus/`
- `src/controller/offboard/`
- `src/controller/px4ctrl`（通过符号链接指向 Fast-Drone-250 的 px4ctrl）
- `src/planner/ego_planner`（通过符号链接指向 Fast-Drone-250 的 planner）
- `src/common/quadrotor_msgs`、`uav_utils`、`cmake_utils`
- `src/simulation/sitl_offboard/`

另外，`third_party` 中已有两个重要参考工程：

- `fast-drone-250_cxr`：包含 VINS-Fusion 与 Fast-Drone-250 的相关模块
- `raicom-air-recon-2026`：有 `px4ctrl`、`fastlio_mavros_bridge`、`simple_offboard_forward`、EGO-Planner 一整套较完整的实用化组件

### 2.2 当前问题

通过现状检查，当前工作区还有以下关键问题：

1. `sqrtVINS` 已不再作为主目标，且当前 `.gitmodules` 与实际路径不一致。
2. `Fast-Drone-250`、`RAICOM`、`Prometheus` 三套代码并存，容易导致包名冲突、重复依赖和符号链接混乱。
3. `vision_to_mavros` 仍是一个较通用的旧接口，适合兼容层，但不建议作为最终正式接口。
4. `rm_drone_ros` 中存在大量 build、devel、备份目录，后续需统一整理为正式源码工程。
5. 当前仍缺少统一的 VIO/VINS 对 PX4 的抽象层；即使算法更换，控制器和规划器理论上也不应感知具体算法实现。

---

## 3. 总体架构建议

建议最终采用如下层次结构：

```text
传感器层
  ├── RealSense / D430 / 双目相机
  ├── Livox / Mid-360S
  └── IMU

状态估计层
  ├── VINS-Fusion
  ├── SchurVINS
  ├── ORB-SLAM3
  └── FAST-LIO（独立备选）

状态接口抽象层
  └── vins_mavros_bridge

规划层
  └── EGO-Planner 改进版

飞控控制层
  ├── px4ctrl
  ├── simple_offboard_forward
  └── MAVROS / PX4
```

核心原则：

> VIO 算法可以替换，但规划器和控制器不应该感知具体 VIO 实现。

因此，VINS-Fusion、SchurVINS、ORB-SLAM3 都应统一输出到 `vins_mavros_bridge`，由其提供统一的 odometry/pose 接口给下游模块使用。

---

## 4. 目录调整建议

建议正式目录框架如下：

```text
rm_drone_ros/src/
├── perception/
│   ├── vio_vins_fusion/
│   ├── vio_schurvins/
│   ├── vio_orbslam3/
│   ├── d430_slam/
│   └── realsense2_camera/
│
├── localization/
│   └── vins_mavros_bridge/
│
├── controller/
│   ├── px4ctrl/
│   ├── simple_offboard_forward/
│   └── offboard/
│
├── planner/
│   ├── ego_planner/
│   ├── plan_env/
│   ├── path_searching/
│   ├── bspline_opt/
│   └── traj_utils/
│
├── communication/
│   ├── px4_d430_bridge/
│   └── apm_d430_bridge/
│
├── common/
│   ├── quadrotor_msgs/
│   ├── uav_utils/
│   └── cmake_utils/
│
└── simulation/
    └── sitl_offboard/
```

这里不建议直接把多个 VIO 项目的源码完全混在一个目录下，而是保留其原始包结构，按功能模块接入正式工作区。正式约定：

- `vins_mavros_bridge` 是唯一对外提供 odometry/pose 的模块。
- `vision_to_mavros` 可作为兼容层或历史兼容包，不作为正式桥接入口。
- `px4ctrl` 和 `ego_planner` 只保留一套正式版本，不允许多个实现同时参与同一 catkin 构建。

---

## 5. VIO 算法选型方案

**本阶段不锁定 VIO 算法。** VINS-Fusion、SchurVINS、ORB-SLAM3 均需经过同一套实机指标（见 5.4）比较后再决定主飞算法。VINS-Fusion 仅因接入成本最低而被**最先接入作为基线**，并不代表选型结论。

### 5.1 基线接入：VINS-Fusion

当前 Fast-Drone-250 CXR 已经包含 VINS-Fusion 代码：

- `/third_party/fast-drone-250_cxr/fast-drone-250-master/src/VINS-Fusion`

作为最先接入的基线，理由是：

- 与现有 Fast-Drone-250 体系衔接成本低；
- 适合双目/单目 + IMU 的无人机定位；
- 在 ROS1 + PX4 场景中最容易先形成工程闭环，用于验证统一接口。

接入步骤：

1. 将 VINS-Fusion 纳入正式工作区；
2. 通过其适配器与 `vins_mavros_bridge` 接到统一里程计接口；
3. 完成仿真与实机静态/低速测试；
4. 其余两种算法按同样接口接入后，用同一套指标比较。

### 5.2 候选方案：SchurVINS

SchurVINS 作为候选方案，重点评估：

- RK3588 上的 CPU 占用；
- 实时性；
- 初始化成功率；
- 低纹理 / 高速运动下的稳定性；
- 是否能稳定输出位姿与速度；
- 是否能在 ROS1 环境中稳定编译。

应先做统一输入输出接口，随后换算法时只改对应适配器，不改控制器/规划器。

### 5.3 备选方案：ORB-SLAM3

ORB-SLAM3 更适合：

- 研究型对比；
- 回环和长期轨迹评估；
- 双目与 RGB-D 场景；
- 离线数据集验证。

但是它在实机飞行中的风险包括：

- 初始化和尺度问题；
- 回环重定位造成位姿跳变；
- 仅使用单目时的尺度不稳定；
- 动态环境下的跟踪失败；
- 输出频率和延迟可能不满足飞控要求。

因此，ORB-SLAM3 先用于离线 rosbag 和悬停评估；是否进入实机飞行取决于下述评估结果，尤其是跳变处理。

### 5.4 统一评估指标

三种算法（以及独立备选 FAST-LIO）使用同一批 rosbag 和同一条实机航线，按以下指标比较，不只看轨迹精度：

| 指标 | 说明 |
|---|---|
| 定位漂移 | 悬停 30/60/120 秒漂移、闭合航线误差 |
| 跟踪稳定性 | 快速转向、机体振动下是否丢失或发散 |
| 丢失后恢复 | 遮挡、短时丢帧后能否恢复，恢复时是否跳变 |
| 端到端延迟 | 传感器时间戳到统一里程计输出 |
| 输出频率 | 实测稳定频率，而非配置频率 |
| 初始化 | 耗时与成功率 |
| 资源占用 | RK3588 上与规划器、MAVROS、相机驱动同时运行时的 CPU/内存/温度 |
| 位姿跳变 | 回环或重定位是否造成位姿不连续 |
| PX4 兼容性 | EKF2 融合是否稳定 |

### 5.5 回环/重定位跳变的处理原则

依赖连续局部里程计的飞控和控制器不能直接接收跳变位姿。因此：

- 区分局部连续里程计（`odom -> base_link`）与全局修正（`map -> odom`）；
- 回环/重定位产生的修正只能进入 `map -> odom`，**不得进入 PX4 与控制器的反馈链**；
- 无法区分的算法输出，桥接层必须做跳变检测，检测到跳变时按 6.4 的异常策略处理，而不是直接转发。

---

## 6. `vins_mavros_bridge` 设计

### 6.0 职责边界

- **对上**：提供与算法无关的 `nav_msgs/Odometry`（必要时再提供 `PoseStamped`）。
- **对下**：每种 VIO 使用独立适配器处理话题和消息差异（VINS-Fusion / SchurVINS / ORB-SLAM3 各一个），适配器之外的桥接核心不感知具体算法。
- **必须明确处理**：`map/odom/base_link` 的定义、相机到机体外参、ENU↔NED 与 FLU↔FRD 转换（明确由谁负责）、时间戳、协方差。
- **不能只靠改话题名把 VIO 位姿送入 MAVROS。** 是否向 MAVROS 输出、输出什么，取决于 6.5 的决策。

### 6.1 输入接口

`vins_mavros_bridge` 应该统一接收如下输入：

```text
nav_msgs/Odometry
```

这是最推荐的统一输入类型，因为它包含：

- 位置
- 姿态
- 速度
- 协方差
- 时间戳
- `frame_id`
- `child_frame_id`

兼容输入也应支持：

```text
geometry_msgs/PoseStamped
```

但仅有 `PoseStamped` 的算法不具备速度相关信息，需要桥接层单独标识为“有限状态模式”。

### 6.2 输出接口

推荐输出：

```text
/mavros/odometry/in
nav_msgs/Odometry
```

兼容输出：

```text
/mavros/vision_pose/pose
geometry_msgs/PoseStamped
```

调试输出建议包括：

- `/vins_mavros_bridge/odometry`
- `/vins_mavros_bridge/pose`
- `/vins_mavros_bridge/status`
- `/vins_mavros_bridge/path`

### 6.3 坐标系与 TF

桥接层必须明确处理：

```text
map / odom
  └── base_link
        └── camera_link
              └── camera_optical_frame
```

建议约定：

- ROS 世界坐标：ENU；机体坐标：`base_link`（FLU）
- 相机输出：按实际相机坐标系和外参转换到 `base_link`
- MAVROS 负责 ROS（ENU/FLU）与 PX4（NED/FRD）之间的转换
- 桥接层不能重复做 ENU↔NED、FLU↔FRD 转换；若某算法输出非 ENU/FLU，只在对应适配器内转换到 ENU/FLU

配置示例：

```yaml
world_frame: odom
body_frame: base_link
vio_frame: camera_link
publish_tf: false

base_to_vio_xyz: [0.0, 0.0, 0.0]
base_to_vio_rpy_deg: [0.0, 0.0, 0.0]
```

必须使用实机标定值更新这些参数，不能直接使用默认值。

### 6.4 关键安全逻辑

桥接层不能只是简单做消息转发，必须至少具备：

- 时间戳检查
- 零时间戳处理
- 输入超时检测
- 位姿跳变检测
- 非法四元数检测
- 协方差配置
- 输出频率统计
- 输入/输出延迟统计
- 重置状态检测

当 VIO 超时或输出异常时，桥接包应：

1. 停止发布有效外部里程计；
2. 发布明显异常状态；
3. 由上层控制器决定降级、悬停或退出 OFFBOARD；
4. 不应静默继续发布旧位姿。

### 6.5 状态反馈链决策（编码前必须定下）

两条位置反馈链不能互相矛盾。需要先决定并写入配置文档：

1. **PX4 是否融合外部视觉**：若融合，需配置 EKF2 相关参数（如 `EKF2_EV_CTRL`、`EKF2_HGT_REF`、`EKF2_EV_DELAY` 等，具体以 PX4 1.14 参数文档核对），并确认 GPS/气压计等其他源的取舍。
2. **px4ctrl 使用哪个状态作为反馈**：
   - 方案 A：PX4 融合 VIO，px4ctrl 使用 `/mavros/local_position/odom`；
   - 方案 B：px4ctrl 直接使用 VIO 统一里程计，PX4 仍需有自己可用的位置估计。
3. 无论选哪种，规划器、控制器与 PX4 必须使用同一个局部连续状态来源，禁止 PX4 融合 VIO 的同时，px4ctrl 又使用另一套未经一致性验证的位姿。

当前倾向（待评审确认）：方案 A，由 PX4 EKF2 统一融合，px4ctrl 以 PX4 估计为反馈，规划器使用统一里程计做地图与规划。

---

## 7. `px4ctrl` 与 `ego_planner` 的迁移

### 7.0 移植原则：先核对接口，再移植

包能编译不等于移植成功。移植前逐项核对 `fast-drone-250_cxr` 的 px4ctrl / EGO-Planner 与现有系统是否一致：

| 核对项 | 内容 |
|---|---|
| 里程计输入 | 订阅的话题、消息类型、`frame_id/child_frame_id` |
| 指令输出 | 发布的轨迹/控制指令（`PositionCommand`、`bspline` 等）及消息定义版本 |
| 坐标系 | world / body 定义，ENU/FLU 约定 |
| 时间戳 | 使用 header 时间还是接收时间，超时阈值 |
| 控制频率 | 指令频率、里程计频率、OFFBOARD 要求的持续发布频率 |

**先固定最小链路，再接入 EGO-Planner：**

```text
相机/IMU → VIO → 统一局部里程计 → 控制器 → MAVROS/PX4
```

### 7.1 `px4ctrl`

建议以 `fast-drone-250_cxr` 的版本为改进基础，RAICOM 版本作为对照参考，并将确定的唯一版本放入正式工作区：

- `rm_drone_ros/src/controller/px4ctrl`

迁移时重点核对：

- PX4 1.14 接口兼容性
- MAVROS 话题名
- 位置 / 速度 / 加速度控制模式
- 推力标定和参数文件
- 起飞、悬停、降落、解锁逻辑
- VIO 超时处理逻辑

注意：

- 不要直接保留多个 `px4ctrl` 包；
- 一次飞行只能启动一个飞控控制者；
- 不能因为“能编译”就认为控制链路已经验证。

### 7.2 `ego_planner`

建议以 `fast-drone-250_cxr` 的 planner 为改进基础，保留对照：

- `/third_party/raicom-air-recon-2026/.../ego_planner`
- `/rm_drone_ros/src/Prometheus/.../ego_planner_swarm`

（最终采用哪一份，以 7.0 接口核对结果为准。）

统一内容包括：

- `quadrotor_msgs` 版本一致
- odom 输入接口统一
- `base_link` / `world` 坐标系一致
- RViz 配置统一
- launch 参数统一
- 输入必须从 `vins_mavros_bridge` 或统一定位接口获取

EGO-Planner 不应直接依赖某个 VIO 包名，而是依赖统一 odometry 接口。

---

## 8. `simple_offboard_forward` 的定位

`simple_offboard_forward` 作为独立的“验收包”保留，**只验证基础 OFFBOARD 链路**，不依赖 VIO，也不依赖规划器或 px4ctrl。这样飞不起来时，可先排除基础 offboard 链路问题。

它的职责仅限：

1. 检查 MAVROS / FCU 连接状态；
2. PX4 模式切换（OFFBOARD）与解锁；
3. 持续发布设定值（含进入 OFFBOARD 前的预热发布）；
4. 失联保护：状态异常、连接丢失或设定值中断时中止；
5. 悬停与短距离位置指令。

**首次实机验证不直接采用前飞任务**，按以下顺序渐进：

```text
无桨测试
  ↓
SITL
  ↓
实机上电但不解锁
  ↓
限位条件下的悬停
  ↓
限位条件下的短距离位置指令
  ↓
（以上通过后）短距离前飞
```

注意：它与 px4ctrl、mission_planner、offboard 不可同时启动（见 12.1）。

---

## 9. ROS 话题与接口契约

建议实现统一接口：

### 9.1 统一定位接口

```text
/localization/odometry
nav_msgs/Odometry
```

建议字段：

```text
header.frame_id       = odom
child_frame_id        = base_link
pose.pose             = base_link 在 odom 中的位姿
twist.twist           = base_link 速度
pose.covariance       = 位姿协方差
twist.covariance      = 速度协方差
```

### 9.2 MAVROS 接口

```text
/mavros/odometry/in
nav_msgs/Odometry
```

兼容输出：

```text
/mavros/vision_pose/pose
geometry_msgs/PoseStamped
```

### 9.3 控制层输入

```text
/mavros/state
/mavros/local_position/odom
/mavros/imu/data
/localization/odometry
```

### 9.4 规划层输入

```text
/localization/odometry
/local_map/...
```

### 9.5 规划层输出

```text
/position_cmd
/bspline
```

具体消息定义仍需统一 `quadrotor_msgs` 版本，避免在同一工作区同时存在多个不同定义。

---

## 10. 分阶段实施计划

### 阶段 0：仓库整理与基线确认

目标：

- 确认正式主仓库范围；
- 排除 build、devel、日志、备份目录；
- 修正 `.gitmodules` 与实际子模块状态；
- 确认当前可编译最小包集合；
- 记录当前包依赖关系。

输出：

- 干净检出状态
- 最小 catkin 构建清单
- 当前包依赖清单
- 当前接口清单

### 阶段 1：建立统一里程计接口

目标：

- 新增 `vins_mavros_bridge`
- 先接入已有 OpenVINS / `nav_msgs/Odometry`
- 验证 `/mavros/odometry/in` 与 `/mavros/vision_pose/pose`
- 不改变规划器和控制器逻辑

验收：

- rosbag 回放稳定输出
- 时间戳正确
- 坐标系正确
- 无重复 ENU/NED 转换
- MAVROS / PX4 能正常接收外部视觉

### 阶段 2：接入 VINS-Fusion（基线，不锁定选型）

目标：

- 将 VINS-Fusion 纳入工作区
- 编写其独立适配器，输出接入 `vins_mavros_bridge`
- 使用统一 launch 和配置
- 先完成 6.5 的反馈链决策
- 完成静态、手持、低速飞行测试

验收：

- 初始化成功率
- 输出频率
- 延迟
- 短时遮挡恢复
- 悬停漂移
- PX4 EKF 稳定性

### 阶段 3：迁移 `px4ctrl` 与 `ego_planner`

目标：

- 先完成 7.0 的接口核对表
- 先固定最小链路（相机/IMU → VIO → 统一里程计 → px4ctrl → MAVROS/PX4）并验证，再接入 EGO-Planner
- 确认并落地唯一正式 px4ctrl 与 ego_planner 版本
- 统一消息接口和 odometry 输入
- 先做 SITL 验证，再进行实机验证

验收：

- PX4 1.14 OFFBOARD 正常
- 起飞、悬停、轨迹跟踪、降落正常
- VIO 短时异常时有安全处理
- 规划器与控制器消息频率正常

### 阶段 4：引入 SchurVINS 和 ORB-SLAM3 并完成选型

目标：

- 只增加对应适配器，不改规划器和控制器接口
- 用同一组 rosbag 和实机航线，按 5.4 指标评估三种算法
- 输出选型报告，此时才锁定主飞算法

验收：

- 5.4 全部指标有实测数据
- 回环/重定位跳变按 5.5 处理并验证
- 与规划器同时运行时 RK3588 资源满足余量

### 阶段 5：形成正式飞行启动入口

建议最终产生：

```text
bringup/
├── px4_mavros.launch
├── vio_vins_fusion.launch
├── vio_schurvins.launch
├── vio_orbslam3.launch
├── localization.launch
├── px4ctrl.launch
├── ego_planner.launch
├── simple_offboard_forward.launch
└── full_stack.launch
```

其中 `full_stack.launch` 不应默认直接解锁和 OFFBOARD；启动系统与允许飞行必须拆分。

---

## 11. 实机评估顺序

建议严格按以下顺序执行：

```text
1. 相机与 IMU 驱动单独验证
2. VIO 单独运行并录制 rosbag
3. VIO 输出 TF 与 Odometry 检查
4. vins_mavros_bridge 单独运行
5. MAVROS / PX4 接收 external vision
6. PX4 EKF 状态检查
7. simple_offboard_forward 无桨测试
8. simple_offboard_forward 限位悬停，再限位短距离指令（不直接前飞）
9. px4ctrl 悬停测试（最小链路，不含规划器）
10. EGO-Planner SITL
11. EGO-Planner 实机低速测试
12. 复杂环境与高速飞行测试
```

任何一步没有通过，都不应继续下一步。

---

## 12. 关键风险

### 12.1 多个控制器同时运行

当前工程中 `mission_planner`、`offboard`、`px4ctrl` 都可能向 PX4 发送控制指令。一次飞行只能启用一个控制器。

### 12.2 VIO 输出跳变

ORB-SLAM3 或重新初始化/回环重定位可能造成位姿跳变。不能把未检测的输出直接送入 PX4。

### 12.3 坐标系重复转换

常见错误：

- VIO 已转换为 NED；
- bridge 再转换一次；
- MAVROS 再转换一次。

建议整个 ROS 系统统一使用 ENU，并由 MAVROS 负责与 PX4 NED 之间的适配。

### 12.4 消息包重复

当前工程中 `quadrotor_msgs`、`uav_utils`、`traj_utils` 可能来自多套源码。必须最终保留一套正式包，不能依赖 compile 顺序 “碰巧编译成功”。

### 12.5 RK3588 资源限制

在 VIO、相机驱动、规划器和 MAVROS 同时运行时，应记录：

- CPU 占用
- 内存使用
- 温度
- 时间延迟
- ROS callback 延迟
- 图像和 IMU 丢帧情况

---

## 13. 最终实施建议

建议采用最终路线：

```text
VIO：
  暂不锁定；VINS-Fusion 最先接入作为基线
  VINS-Fusion / SchurVINS / ORB-SLAM3 用同一套实机指标比较后再选定

状态接口：
  新增 vins_mavros_bridge（对上算法无关 nav_msgs/Odometry，对下每种 VIO 一个适配器）
  MAVROS 作为唯一 PX4 外部视觉入口
  先定 6.5 反馈链决策，避免两条位置反馈链矛盾

控制器：
  以 fast-drone-250_cxr 的 px4ctrl 为改进基础，先核对接口再移植

规划器：
  以 fast-drone-250_cxr 的 EGO-Planner 为改进基础
  最小链路验证通过后再接入

验证工具：
  simple_offboard_forward 作为独立 OFFBOARD 链路验收包
  先限位悬停与短距离指令，不直接前飞

仓库管理：
  rm_drone_ros 作为唯一 ROS1 工作区
  third_party 只保存参考源码或正式依赖
  清理 build/devel/备份目录和重复源码入口
```

这条路线能够最大程度兼容当前工程，避开 `sqrtVINS` 的历史问题，并且能在 ROS1/PX4 1.14 上形成稳定的飞行基础架构。

---

## 14. 下一步动作建议

以下是建议的下一步顺序：

1. 统一清理 `rmUAV` 仓库结构，仅保留正式源码和 `third_party` 参考目录；
2. 修正 `.gitmodules` 与第三方代码的实际状态；
3. 按 7.0 核对 `fast-drone-250_cxr` 与 RAICOM 的 `px4ctrl`、`ego_planner` 接口，并检视 `simple_offboard_forward`；
4. 评审并确定 6.5 的状态反馈链决策；
5. 实现 `vins_mavros_bridge` 及 VINS-Fusion 基线适配器（不代表锁定选型）；
6. 先通过 SITL 和无桨测试，再进入实机飞行验证。

该方案适合在当前 ROS1/PX4 1.14 条件下逐步落地，并为后续 SchurVINS / ORB-SLAM3 的横向评估预留统一接口。

---

## 15. 待评审决策项

| # | 决策 | 位置 | 当前倾向 |
|---|---|---|---|
| 1 | PX4 是否融合外部视觉，EKF2 参数如何配置 | 6.5 | 融合（方案 A） |
| 2 | px4ctrl 反馈来源：`/mavros/local_position/odom` 还是 VIO 直出 | 6.5 | PX4 估计（方案 A） |
| 3 | px4ctrl / EGO-Planner 采用 `fast-drone-250_cxr` 还是 RAICOM 版本 | 7.0 | 以 cxr 为基础，接口核对后定 |
| 4 | 回环/重定位修正只进 `map→odom` 还是由桥接层做跳变拒绝 | 5.5 | 两者都做 |
| 5 | `vision_to_mavros` 保留为兼容层还是废弃 | 4 | **已废弃并移除**（`vins_mavros_bridge` 取代） |
