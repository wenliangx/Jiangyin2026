# 永久地图创建与重定位运行指南

手举建图、人员轨迹清理、CloudCompare 复核和 SUPER 静态底图的完整流程见
`docs/competition_map_workflow.md`。

## 1. 构建

`map_frame_manager` 是标准 catkin 包：

```bash
cd ~/Jiangyin2026
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

## 2. 手持创建临时地图并确定永久原点

建图阶段不再固定永久原点。RA-LIO 模式：

```bash
roslaunch map_frame_manager create_map_ra_lio.launch \
  output_directory:=/data/maps/jiangyin \
  map_name:=competition_map \
  rviz:=true
```

查看状态：

```bash
rostopic echo /map_frame_manager/state
rostopic echo /map_frame_manager/ready
```

状态进入 `MAPPING` 后移动飞机完成建图。结束前显式保存：

```bash
rosservice call /map_frame_manager/save_map
```

也可以正常按 `Ctrl+C`，节点默认在退出时保存。输出为：

```text
/data/maps/jiangyin/competition_map.pcd
/data/maps/jiangyin/competition_map.yaml
```

YAML 是最后写入的有效性标志；保存失败时状态会变为 `FAILED_SAVE`。
保存结果的 `origin_definition` 为 `unanchored_lio_local_frame`。随后按
`docs/competition_map_workflow.md` 的“确定永久原点”步骤生成永久坐标 PCD。

Point-LIO 首次建图时先启动 `sml-core-machine` 的 Point-LIO，再执行：

```bash
roslaunch map_frame_manager create_map_point_lio.launch \
  odom_topic:=/aft_mapped_to_init \
  cloud_topic:=/cloud_registered \
  local_frame:=camera_init \
  body_frame:=body \
  output_directory:=/data/maps/jiangyin \
  map_name:=competition_map
```

话题和 frame 必须按 Point-LIO 的实际输出替换。

## 3. 使用已有地图重定位

比赛运行只支持固定原点 yaw 初始化。飞机必须放在永久 `(0,0,0)`，roll、pitch
为零；节点自动在全角度范围搜索 yaw，不接收 `/initialpose`，也不允许配准算法
修改位置、roll 或 pitch。

由本系统创建的 PCD 已经位于永久场地坐标，不再需要手动转换。
创建地图时该坐标在文件中记为 `lio_global`；重定位运行时，同一组
坐标数值对外使用 `world` frame。
`map` 坐标系不由本模块发布，可继续留给其他模块使用：

```yaml
map_transform: [0, 0, 0, 0, 0, 0]
```

RA-LIO：

```bash
roslaunch map_frame_manager relocalize_ra_lio.launch \
  map_file:=/data/maps/jiangyin/competition_map.pcd \
  rviz:=true
```

若要让现有控制、EGO 和 SUPER 直接使用重定位后的永久 `world`，开启统一全局
输出开关：

```bash
roslaunch map_frame_manager relocalize_ra_lio.launch \
  map_file:=/data/maps/jiangyin/competition_map.pcd \
  publish_to_original_odom_topic:=true
```

此时 RA-LIO 的 IEKF 和 ikd-Tree 仍在 `lio_local` 内运行，并持续提供内部调试
话题：

```text
/Odometry_local           lio_local -> body
/cloud_registered_local   lio_local 点云
/Laser_map_local          lio_local 地图
/path_local               lio_local 路径
```

重定位成功并进入 `READY` 后，RA-LIO 发布层统一应用 `world -> lio_local`
变换，对外提供：

```text
/Odometry                 world -> body
/cloud_registered         world 点云
/Laser_map                world 地图
/path                     world 路径
```

因此现有下游话题名不需要修改，且里程计、点云和轨迹不会混用坐标系。`map`
坐标系不由此流程占用。`/Odometry_lio_global` 作为 map_frame_manager 的独立校验
输出保留，其数值应与公共 `/Odometry` 一致。

全局输出模式下，RA-LIO 内置的 PCD 保存仍保存 `lio_local` 数值。
需要可复用的永久地图时，继续使用 `map_frame_manager` 的
`/map_frame_manager/save_map` 服务。

Point-LIO：

```bash
roslaunch map_frame_manager relocalize_point_lio.launch \
  map_file:=/data/maps/jiangyin/competition_map.pcd \
  odom_topic:=/aft_mapped_to_init \
  cloud_topic:=/cloud_registered \
  local_frame:=camera_init \
  body_frame:=body
```

Point-LIO 由外部 launch 启动，若启用同一兼容方式，必须先将 Point-LIO 的原始
输出改到独立话题，再指定输入和原输出话题，例如：

```bash
roslaunch map_frame_manager relocalize_point_lio.launch \
  map_file:=/data/maps/jiangyin/competition_map.pcd \
  odom_topic:=/aft_mapped_to_init_local \
  publish_to_original_odom_topic:=true \
  original_odom_topic:=/aft_mapped_to_init
```

禁止让 `odom_topic` 与兼容输出话题相同；节点会主动拒绝这种配置，避免订阅自身
输出形成反馈。

节点积累点云后执行 yaw 粗搜索、`0.05°` 细搜索和连续一致性验证。RA-LIO
全局输出模式成功后输出：

- TF `world -> lio_local`；
- `/Odometry` 与 `/cloud_registered`（永久 `world`）；
- `/Odometry_lio_global`；
- `/relocalization/world_from_lio_local`；
- `/prior_map`；
- `/relocalization/aligned_cloud`；
- `/relocalization/initialized = true`；
- `/map_frame_manager/state = READY`。

重新匹配或清除结果：

```bash
rosservice call /map_frame_manager/relocalize
rosservice call /map_frame_manager/reset
```

运行状态依次为 `COLLECTING_SCAN -> MATCHING_YAW -> VERIFYING_YAW -> READY`。
场景航向有歧义、重叠率不足或连续结果不一致时拒绝初始化，不提供任意地点的
人工粗定位回退。

## 4. 输入接口约束

重定位与 LIO 算法无关，但以下条件必须成立：

```text
odom.header.frame_id  == local_frame
cloud.header.frame_id == local_frame
odom.child_frame_id   == body_frame
```

registered cloud 必须真正转换到了 `local_frame`，不能只修改消息中的
`header.frame_id`。节点遇到错误父坐标系会拒绝数据。

## 5. 飞控接入

开启 RA-LIO 统一全局输出后，PX4 estimator、现有控制链、EGO 与 SUPER 可继续
使用原话题 `/Odometry` 和 `/cloud_registered`，二者均为永久 `world`。必须在
以下条件全部满足后才允许解锁：

1. `/relocalization/initialized` 为 true；
2. aligned cloud 与 prior map 重合；
3. 飞机静止时 `/Odometry` 与 `/Odometry_lio_global` 没有跳变；
4. `world -> lio_local` 保持固定；
5. 重复启动的位置和 yaw 满足精度要求。

自动飞行必须由 ready 状态门控，任何 `WAITING`、`MATCHING` 或 `FAILED` 状态均
不能使用全局位姿解锁任务。
