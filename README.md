# Robomaster自主无人机

## 注意
组装文档中采用了悬挂方案安装飞控，该方案能够极大地减小震动对imu的影响。但是长时间的拉伸会使得皮筋产生脆化而失去弹性。将皮筋更换为工业橡皮筋可缓解该问题。同时碳纤维板上预留了飞控安装孔位，也可以选择直接将飞控安装到碳板上，但是该做法的imu减震性能不佳。

## 简介
为了降低选手参与自主无人机赛事的门槛，减少用于调试安装设备的精力，推出了基于DJI Avata平台，Nvidia Jetson Xiaver NX 和 Ardupilot的开源自主无人机方案。  
*docs/RMUA2023自主无人机开源手册.md*详细介绍了安装，调试，参数配置，程序开发的全过程。按照该手册的指引，能够从0开始获得可以自主定位，手动或者离线控制的自主无人机。    
*models*中包含有所需的非标件模型文件。   
`rm_drone_ros` 含有多个基本离线控制程序案例以供参考：
1. 视觉定位 (`rm_drone_ros/src/perception/d430_slam`)；
2. 定点悬浮 (`rm_drone_ros/src/controller/offboard`)；
3. 位置控制巡航 (`rm_drone_ros/src/controller/offboard`)；
4. 速度控制巡航 (`rm_drone_ros/src/controller/offboard`)。

## 软件工作区

ROS 1 软件统一放在 [`rm_drone_ros`](rm_drone_ros/README.md)。OpenVINS、SchurVINS 和 Fast-Drone-250 作为 Git submodule 管理；克隆后先初始化 submodule：

```bash
git submodule update --init --recursive
```

`rm_drone_ros/src` 是唯一 catkin 源码空间，自有 ROS 包按感知、规划、控制、通信、公共组件和仿真分类。OpenVINS 与 SchurVINS 通过不同 ROS 包名在同一工作区并存（SchurVINS 当前不参与构建，见 `rm_drone_ros/README.md` 的构建限制）。Fast-Drone-250 完整仓库位于 `third_party/Fast-Drone-250`，所需 EGO-Planner、`px4ctrl` 及公共依赖通过分类目录下的相对符号链接加入 catkin，避免无意构建该仓库中的其他包。仿真环境素材放在 `rm_drone_sim/`（XTDrone 安装包，2.7 GB，被 `.gitignore` 排除，需单独获取）。构建与部署步骤见 [`rm_drone_ros/README.md`](rm_drone_ros/README.md)。

## License
本项目使用*GNU GPLv3*，查看*LICENSE*文件获得更多信息。


