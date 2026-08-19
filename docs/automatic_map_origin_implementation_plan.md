# 永久地图原点自动初始化与多 LIO 重定位实施计划

## 实施状态（2026-08-18）

已完成首个可运行版本：

- 独立 `map_frame_manager` catkin 包；
- RA-LIO/Point-LIO 参数化输入适配；
- 静止窗口检测与自动永久原点锁定；
- yaw-only 水平地图坐标定义；
- 点云累计、周期体素压缩及 PCD/YAML 原子保存；
- NDT+ICP 重定位、fitness 与最大修正量拒绝条件；
- `map→local` TF、`/Odometry_map`、状态与 ready 输出；
- lock/save/reset/relocalize 服务；
- 坐标变换单元测试和运行指南。

尚待目标 ROS Noetic 设备完成编译、rosbag 和现场验收；Point-LIO wrapper 中的话题与
frame 仍需用 `sml-core-machine` 的实际接口替换。地图 SHA-256、重叠率指标和
px4_estimator 自动门控保留为后续增强项，当前版本不应绕过 ready 状态直接用于自动飞行。

## 1. 项目目标

建立一套与具体 LIO 算法解耦的地图坐标初始化系统，实现：

1. 首次建图时，程序自动以飞机初始位置和朝向建立永久 `map` 坐标系；
2. 自动保存 PCD 地图及配套坐标元数据，不需要使用 CloudCompare 手工计算原点；
3. 后续启动时，通过当前点云与历史地图配准，恢复固定的 `map` 坐标；
4. RA-LIO 和 Point-LIO 使用同一个初始化与重定位模块；
5. 重定位失败时禁止向飞控输出“已初始化”状态，避免错误全局位姿进入控制链路。

本项目只负责全局坐标初始化与重定位，不修改 RA-LIO 或 Point-LIO 内部滤波器。

## 2. 坐标系定义

统一采用 ROS ENU 约定：

- `map`：永久全局坐标系；
- `local`：LIO 本次启动产生的局部里程计坐标系。RA-LIO 通常为 `world`，Point-LIO 通常为 `camera_init`；
- `body`：飞机机体/IMU 坐标系；
- `lidar`：雷达坐标系；
- `pcd`：PCD 文件保存时的原始坐标系。

所有位姿采用 `T_parent_child` 记法。例如：

```text
T_map_body = T_map_local × T_local_body
```

### 2.1 首次建图的永久原点

在飞机静止且 LIO 初始化完成后，记录：

```text
p0     = 飞机在 local 中的位置
yaw0   = 飞机机头在 local 水平面内的方向
```

永久坐标系定义为：

```text
map 原点 = p0
map +X   = 初始机头在水平面内的投影
map +Z   = 重力反方向
map +Y   = +Z × +X
```

只使用初始 yaw 定义水平朝向，不把起飞架上的微小 roll/pitch 固化到地图中。变换为：

```text
R_map_local = Rz(-yaw0)
t_map_local = -R_map_local × p0
```

若使用外部绝对航向，则将 `yaw0` 替换为经过 NED→ENU、FRD→FLU 和磁偏角修正后的航向。

## 3. 总体架构

```text
                 ┌──────────────────────────────┐
LiDAR + IMU ────→│ RA-LIO 或 Point-LIO          │
                 │                              │
                 │ local odom + registered cloud│
                 └──────────────┬───────────────┘
                                │
                ┌───────────────▼────────────────┐
                │ map_frame_manager              │
                │                                │
首次建图模式 ──→│ 稳定检测 → 锁定永久原点        │
                │                                │
重定位模式 ────→│ 加载 PCD → NDT → ICP           │
                └───────────────┬────────────────┘
                                │
             ┌──────────────────┼──────────────────┐
             ▼                  ▼                  ▼
       map → local TF    /Odometry_map      初始化状态/质量
```

模块放入独立 ROS 包 `map_frame_manager`，不继续把核心功能绑定在 `ra_lio` 包内。RA-LIO 和 Point-LIO 只提供不同的配置文件或 launch wrapper。

## 4. 工作模式

### 4.1 `create_map`：首次建图

流程：

1. 启动选定的 LIO；
2. 等待里程计、registered cloud 和 IMU 数据连续有效；
3. 判断飞机是否持续静止；
4. 自动锁定 `T_map_local`；
5. 建图期间持续把 registered cloud 转换到 `map` 坐标并累计；
6. 调用保存服务或正常退出时保存 PCD；
7. 同时原子写入地图 YAML 元数据；
8. PCD 与 YAML 均成功后发布 `map_ready=true`。

默认要求飞机在启动架上静止并朝向场地正 X。原点锁定后，即使飞机移动也不再修改 `T_map_local`。

### 4.2 `relocalize`：加载已有地图

流程：

1. 加载 PCD 和 YAML；
2. 校验地图版本、点数、文件路径和 SHA-256；
3. 等待 LIO 初始化；
4. 积累若干帧 registered cloud；
5. 由配置的起飞区域或外部粗位置产生平移初值；
6. 使用飞机/LIO 姿态产生旋转初值；
7. NDT 粗配准；
8. ICP 精配准；
9. 通过 fitness、重叠率、位移修正量和姿态修正量进行验收；
10. 冻结 `T_map_local`，持续发布地图坐标里程计。

重定位只校正 `map→local`，不直接重置 LIO 内部状态，因此局部里程计保持连续。

## 5. 状态机

```text
IDLE
  ↓
WAITING_FOR_LIO
  ↓
WAITING_FOR_STABILITY
  ├── create_map ──→ ORIGIN_LOCKED ──→ MAPPING ──→ SAVING ──→ READY
  └── relocalize ──→ COLLECTING_SCAN ─→ MATCHING ─→ VALIDATING ─→ READY
                                         │               │
                                         └────失败───────→ FAILED
```

从 `FAILED` 恢复需要重新触发服务或重新满足自动重试条件。任何非 `READY` 状态均不得宣告全局定位可用。

## 6. ROS 接口

### 6.1 通用输入

| 参数 | 类型 | 说明 |
|---|---|---|
| `odom_topic` | `nav_msgs/Odometry` | LIO 局部里程计 |
| `cloud_topic` | `sensor_msgs/PointCloud2` | 已变换到 `local_frame` 的点云 |
| `attitude_topic` | `sensor_msgs/Imu`，可选 | FCU 姿态或航向初值 |
| `local_frame` | string | LIO 局部父坐标系 |
| `body_frame` | string | 里程计子坐标系 |

必须验证：

```text
odom.header.frame_id == local_frame
cloud.header.frame_id == local_frame
odom.child_frame_id  == body_frame
```

### 6.2 输出

| 话题/TF | 类型 | 说明 |
|---|---|---|
| `map -> local` | TF | 永久地图到当前 LIO 局部系的变换 |
| `/Odometry_map` | `nav_msgs/Odometry` | 永久地图坐标下的飞机位姿 |
| `/prior_map` | `sensor_msgs/PointCloud2` | 历史地图，latched |
| `/map_frame_manager/state` | `std_msgs/String` | 当前状态 |
| `/map_frame_manager/ready` | `std_msgs/Bool` | 是否允许使用全局定位 |
| `/map_frame_manager/quality` | 自定义消息或 diagnostics | fitness、重叠率、点数、耗时 |
| `/map_frame_manager/aligned_cloud` | `sensor_msgs/PointCloud2` | 配准结果检查 |

### 6.3 服务

| 服务 | 作用 |
|---|---|
| `/map_frame_manager/lock_origin` | 手动确认/重新锁定首次建图原点 |
| `/map_frame_manager/save_map` | 原子保存 PCD 和 YAML |
| `/map_frame_manager/relocalize` | 重新执行点云匹配 |
| `/map_frame_manager/reset` | 清除当前初始化结果，回到等待状态 |

自动模式下不要求操作这些服务，但保留它们用于调试和异常恢复。

## 7. 地图元数据格式

每张地图必须成对保存：

```text
competition_map.pcd
competition_map.yaml
```

建议 YAML 格式：

```yaml
format_version: 1
map_id: jiangyin_2026_competition
created_at: 2026-08-18T10:00:00+08:00
pcd_file: competition_map.pcd
pcd_sha256: "..."
point_count: 1234567
frame_convention: ENU
origin_definition: initial_vehicle_position_and_heading

transform_map_from_pcd:
  translation: [0.0, 0.0, 0.0]
  quaternion: [0.0, 0.0, 0.0, 1.0]

creation:
  localization_source: ra_lio
  odom_topic: /Odometry
  cloud_topic: /cloud_registered
  local_frame: world
  body_frame: body
  heading_source: initial_vehicle_x

filters:
  map_leaf_size: 0.10
```

保存时先写临时文件，完成并校验后再重命名，避免断电留下半张地图或 PCD/YAML 不一致。

## 8. 稳定检测与自动锁定

默认连续 3 秒满足以下条件后自动锁定原点：

```text
里程计持续时间       >= 3 s
registered cloud 帧数 >= 20
线速度模长           < 0.05 m/s
角速度模长           < 0.03 rad/s
位置窗口标准差       < 0.03 m
yaw 窗口标准差        < 1.0 deg
```

阈值全部参数化。若超时 30 秒仍不稳定，进入 `FAILED` 并给出具体失败指标，不使用不可靠初值继续运行。

## 9. 航向策略

提供三种可配置策略：

### `initial_vehicle_x`（默认）

首次建图时飞机机头方向定义为地图正 X。最适合室内比赛场地，避免磁罗盘干扰。

### `fcu_heading`

使用 FCU 姿态的绝对 yaw。实现时必须显式配置消息坐标约定，并完成 NED/ENU、FRD/FLU 转换。检测到磁航向跳变或协方差过大时拒绝锁定。

### `configured_yaw`

由配置提供场地 yaw，适用于固定安装和已测量的场地坐标。

无论哪种策略，地图 Z 轴均由重力方向确定，不固化飞机静止时的微小 roll/pitch。

## 10. RA-LIO 与 Point-LIO 适配

### 10.1 RA-LIO

```yaml
odom_topic: /Odometry
cloud_topic: /cloud_registered
local_frame: world
body_frame: body
```

### 10.2 Point-LIO

示例配置，最终以 `sml-core-machine` 实际消息为准：

```yaml
odom_topic: /aft_mapped_to_init
cloud_topic: /cloud_registered
local_frame: camera_init
body_frame: body
```

若 Point-LIO 只发布机体/雷达系点云，则增加一个基于时间同步里程计的 cloud transformer，将点云转换到 `local_frame` 后再交给通用模块。不能只修改 `frame_id` 字符串。

## 11. 实施阶段

### 阶段 A：接口与坐标约定确认

1. 记录 RA-LIO 实际 odom/cloud 的 header 和 TF；
2. 在 `sml-core-machine` 记录 Point-LIO 对应信息；
3. 确认 PX4 姿态来源、ENU/NED 约定和是否使用磁航向；
4. 固化 `map/local/body/lidar` TF 规范。

交付物：接口表、TF 树和两个 LIO 的配置文件。

### 阶段 B：独立通用包

1. 新建 `src/map_frame_manager` catkin 包；
2. 将现有 `map_relocalization.cpp` 从 RA-LIO 包迁移到通用包；
3. 实现严格的 frame 校验和时间戳校验；
4. 提供 RA-LIO、Point-LIO 两套 launch wrapper。

交付物：可使用人工粗初值完成两种 LIO 的已有地图重定位。

### 阶段 C：首次建图自动定原点

1. 实现稳定检测窗口；
2. 实现 yaw-only 永久原点计算；
3. 实现地图点云累计和体素降采样；
4. 实现 PCD/YAML 原子保存；
5. 实现地图 checksum 与格式版本。

交付物：飞机摆正并静止后，可全自动生成带永久坐标定义的地图包。

### 阶段 D：自动重定位

1. 使用 LIO/FCU 姿态生成旋转初值；
2. 使用固定起飞区域配置生成平移初值；
3. 实现局部地图裁剪、NDT 和 ICP；
4. 增加 fitness、重叠率和最大修正量验证；
5. 实现失败重试和质量诊断。

交付物：从指定起飞区域启动，无需 RViz `/initialpose` 即可恢复永久坐标。

### 阶段 E：控制链路接入

1. 保持 `/Odometry` 作为原始诊断输出；
2. 仅在 `ready=true` 后允许 `/Odometry_map` 进入 px4_estimator；
3. 增加初始化超时、定位失效和 TF 冲突保护；
4. 分别完成 RA-LIO 与 Point-LIO 飞行前静态验证；
5. 最后进行低高度、限速飞行验证。

## 12. 测试计划

### 12.1 单元测试

- `T_map_local` 计算与逆变换；
- yaw 在 `±π` 附近的处理；
- NED/ENU 与 FRD/FLU 转换；
- 稳定窗口判定；
- PCD/YAML 一致性和 checksum；
- 错误 frame、空点云、非法四元数拒绝；
- 地图保存中断后的恢复。

### 12.2 Rosbag 回放

至少准备：

1. RA-LIO 首次建图包；
2. RA-LIO 同地图重定位包；
3. Point-LIO 同地图重定位包；
4. 静止、缓慢移动、快速转动和点云遮挡异常包；
5. 对称走廊或重复结构失败样例。

回放必须支持 `use_sim_time=true`，相同 bag 重复运行应产生确定性结果。

### 12.3 现场测试

1. 同一起飞点重复上电 10 次；
2. 起飞区域内改变位置和航向；
3. RA-LIO 与 Point-LIO 交叉验证；
4. 遮挡部分雷达视场；
5. 故意给出错误粗位置，确认系统拒绝错误匹配；
6. 重定位成功后静置和低速移动，检查全局位姿连续性。

## 13. 验收标准

### 首次建图

- 自动原点锁定无需人工计算；
- 初始机体位置在 `map` 中误差小于 2 cm；
- 初始机头相对 `map +X` 误差小于 1°；
- PCD 和 YAML 能配对加载并通过 checksum；
- 非正常中断不会覆盖上一版有效地图。

### 重定位

- 指定起飞区域内成功率不低于 95%；
- 初始化时间不超过 10 秒；
- 静态平移重复性优于 10 cm；
- 静态 yaw 重复性优于 2°；
- 错误匹配不得发布 `ready=true`；
- RA-LIO 和 Point-LIO 输出统一为 `map -> body` 语义。

### 控制安全

- `ready=false` 时全局位姿不进入自动飞行控制；
- 初始化成功后 `map→local` 不随单帧点云跳变；
- LIO 重启、时间戳回退或 frame 改变时立即撤销 ready；
- 系统中只能存在一个 `map→local` TF 发布者。

## 14. 主要风险与对策

| 风险 | 对策 |
|---|---|
| 重复走廊导致错误匹配 | 限制起飞区域、增加重叠率和修正量检查 |
| 室内磁航向漂移 | 默认使用首次建图机头方向，不依赖磁罗盘 |
| Point-LIO 点云不在 local 坐标系 | 增加带时间同步的点云变换适配器 |
| PCD 太大导致 NDT 缓慢 | 地图体素化，并按粗初值裁剪局部子图 |
| LIO 尚未稳定就锁定原点 | 使用速度、角速度、位置和 yaw 多指标窗口 |
| 地图与 YAML 不匹配 | 文件版本、点数和 SHA-256 校验 |
| TF 重复发布 | 启动时检测并拒绝冲突发布者 |
| 重定位结果直接导致飞控跳变 | ready 门控，先验证再切换 px4_estimator |

## 15. 预期交付文件

```text
src/map_frame_manager/
├── CMakeLists.txt
├── package.xml
├── include/map_frame_manager/
├── src/
│   ├── map_frame_manager_node.cpp
│   ├── origin_initializer.cpp
│   ├── prior_map_localizer.cpp
│   └── map_storage.cpp
├── config/
│   ├── common.yaml
│   ├── ra_lio.yaml
│   └── point_lio.yaml
├── launch/
│   ├── create_map_ra_lio.launch
│   ├── relocalize_ra_lio.launch
│   └── relocalize_point_lio.launch
└── test/
    ├── test_frame_math.cpp
    ├── test_stability_detector.cpp
    └── test_map_metadata.cpp

docs/
├── automatic_map_origin_implementation_plan.md
└── map_frame_manager_runtime_guide.md
```

## 16. 实施前需要确认的信息

进入编码阶段前需要从 `sml-core-machine` 获取：

1. Point-LIO odometry 话题、消息类型和 `header.frame_id`；
2. Point-LIO registered cloud 话题和 `header.frame_id`；
3. Point-LIO `child_frame_id`；
4. FCU 姿态话题以及 NED/ENU、FRD/FLU 约定；
5. 是否固定从指定起飞区域启动；
6. 首次建图时是否采用“机头指向场地正 X”的操作规范。

其中第 5、6 项建议采用“固定起飞区域 + 首次建图机头朝正 X”，这是当前场景下最可靠、最容易验收的方案。
