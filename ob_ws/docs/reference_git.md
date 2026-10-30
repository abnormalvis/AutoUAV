可以按**民用、非接触式跟踪**和自主飞行来筛选：我不能推荐或协助实现无人机自动识别并打击、攻击目标的项目。不过，下面这些资源适合研究**目标跟随、巡检、搜救、目标位姿估计、避障和轨迹规划**。

| 方向 | 项目 / 资源 | 适合做什么 |
|---|---|---|
| 视觉目标跟随 | [PixEagle（GitHub）](https://github.com/alireza787b/PixEagle) | PX4 无人机的视觉跟踪与跟随项目，结合 OpenCV、YOLO、MAVSDK 和 MAVLink；可作为“检测—跟踪—非接触跟随”的入门参考。 |
| 自主导航与避障 | [PX4 ROS 2 Drone Navigation（GitHub）](https://github.com/formiat/px4-ros2-drone-nav) | PX4、ROS 2、Gazebo 仿真结合，涉及 VIO/SLAM、三维地图、路径规划和局部避障；偏导航，不是目标攻击系统。 |
| 轨迹规划 | [Fast-Planner（GitHub）](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) | 多旋翼快速规划与避障，适合研究从当前位置到安全航点的轨迹生成。 |
| 轨迹规划与避障 | [EGO-Planner（GitHub）](https://github.com/ZJU-FAST-Lab/ego-planner) | 面向无人机的局部轨迹规划与避障，可用于理解规划器如何根据环境生成可执行轨迹。 |
| 飞控基础 | [PX4 Autopilot（GitHub）](https://github.com/PX4/PX4-Autopilot) | 常用开源飞控；适合作为位置、速度控制和仿真测试的底层平台。 |
| 视觉里程计 / 位姿估计 | [VINS-Fusion（GitHub）](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) | 用相机与 IMU 估计无人机自身位姿；它解决的是“无人机在哪里”，不能直接替代目标识别或目标位姿估计。 |

一个较稳妥的研究路线是：

1. **先在 Gazebo 等仿真环境中**验证检测、跟踪和安全跟随，不直接上真实无人机。  
2. 将视觉模块输出限定为目标的**图像位置、距离或相对位姿**，用于巡检或保持安全间距。  
3. 由规划器生成**无碰撞轨迹**，由飞控执行，并保留限速、地理围栏、失联返航和人工接管。  
4. 用静态标志物、模拟目标或授权的地面车辆测试；避免把目标跟踪链路连接到伤害性载荷或攻击动作。

若你重点是“**根据目标相对位置调整无人机速度**”，可以先看 PixEagle 的非接触跟随思路，再搭配 PX4 和 Fast-Planner/EGO-Planner；若重点是“**目标位姿估计后规划自身轨迹**”，则要区分目标相对位姿估计、无人机自身定位和轨迹规划这三个模块分别验证。