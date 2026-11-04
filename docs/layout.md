# 目录职责

| 目录 | 放什么 | 不放什么 |
|---|---|---|
| `catkin_ws/src/<层>/` | ROS 包或指向 `third_party` 的符号链接 | 脚本、运行配置、文档 |
| `config/` | 与具体机体/部署相关的配置（标定、systemd 单元、env 示例） | 随包发布的默认参数（留在包内 `config/`） |
| `launch/` | 跨包的根级入口 launch | 单个包自己的 launch |
| `scripts/` | 启动、检查、安装等 shell/python 脚本 | ROS 节点脚本（留在包内 `scripts/`） |
| `docs/` | 部署、布局、集成计划、记录 | |
| `third_party/` | submodule 与参考检出 | 本项目代码 |
| `data/ bags/ logs/ tmp/` | 运行产物（被忽略，仅 `.gitkeep`） | 需要入库的文件 |
| `svo_ws/` | SchurVINS 独立 catkin 工作区（独立仓库，被忽略） | 本项目配置（放 `config/`） |

## src 分层

- `localization`：VIO 到 `/localization/odometry` 的统一接口。
- `planning`：EGO-Planner 及任务规划。
- `control`：px4ctrl、offboard 控制示例。
- `perception`：目标检测与 SLAM 相关节点。
- `communication`：与飞控的桥接。
- `common`：消息与工具包。
- `simulation`：SITL 资源。
- `vendor`：第三方 ROS 栈（OpenVINS）及其符号链接。

新增包按链路位置放入对应层；第三方源码用 submodule 加相对符号链接，不整份拷贝入库。

## 路径约定

脚本通过 `scripts/common.sh` 推导 `ROOT_DIR`、`CATKIN_WS`（`$ROOT_DIR/catkin_ws`）、`SVO_WS`（`$ROOT_DIR/svo_ws`），可在 `/etc/rm_drone/rm_drone.env` 覆盖。`catkin_ws` 与 `svo_ws` 分别编译、分别 source，互不叠加。

## 迁移说明

`rm_drone_ros` 已改名为 `catkin_ws`，旧的 `build/`、`devel/` 缓存含旧绝对路径，需删除后重编。
