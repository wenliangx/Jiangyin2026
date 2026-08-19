# 永久地图创建与重定位运行指南

## 1. 构建

`map_frame_manager` 是标准 catkin 包：

```bash
cd ~/Jiangyin2026
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

## 2. 首次创建永久地图

把飞机放在希望的永久原点，机头对准场地正 X，并保持静止。RA-LIO 模式：

```bash
roslaunch map_frame_manager create_map_ra_lio.launch \
  output_directory:=/data/maps/jiangyin \
  map_name:=competition_map \
  rviz:=true
```

节点默认等待 3 秒稳定窗口和至少 20 帧点云，然后自动锁定：

```text
初始飞机位置 → map (0,0,0)
初始机头水平投影 → map +X
重力反方向 → map +Z
```

查看状态：

```bash
rostopic echo /map_frame_manager/state
rostopic echo /map_frame_manager/ready
```

原点锁定后移动飞机完成建图。结束前显式保存：

```bash
rosservice call /map_frame_manager/save_map
```

也可以正常按 `Ctrl+C`，节点默认在退出时保存。输出为：

```text
/data/maps/jiangyin/competition_map.pcd
/data/maps/jiangyin/competition_map.yaml
```

YAML 是最后写入的有效性标志；保存失败时状态会变为 `FAILED_SAVE`。

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

从固定起飞区域启动时，在 `config/relocalize.yaml` 中填写飞机在地图内的大致
起始位姿：

```yaml
auto_initialize: true
initial_pose: [x, y, z, roll, pitch, yaw]
```

由本系统创建的 PCD 已经位于永久 `map` 坐标，不再需要手动转换：

```yaml
map_transform: [0, 0, 0, 0, 0, 0]
```

RA-LIO：

```bash
roslaunch map_frame_manager relocalize_ra_lio.launch \
  map_file:=/data/maps/jiangyin/competition_map.pcd \
  rviz:=true
```

Point-LIO：

```bash
roslaunch map_frame_manager relocalize_point_lio.launch \
  map_file:=/data/maps/jiangyin/competition_map.pcd \
  odom_topic:=/aft_mapped_to_init \
  cloud_topic:=/cloud_registered \
  local_frame:=camera_init \
  body_frame:=body
```

节点积累点云后依次执行 NDT 和 ICP。成功输出：

- TF `map -> local_frame`；
- `/Odometry_map`；
- `/prior_map`；
- `/relocalization/aligned_cloud`；
- `/relocalization/initialized = true`；
- `/map_frame_manager/state = READY`。

重新匹配或清除结果：

```bash
rosservice call /map_frame_manager/relocalize
rosservice call /map_frame_manager/reset
```

若不使用固定起飞粗位姿，将 `auto_initialize` 设为 false，并在 RViz 使用
`2D Pose Estimate` 发布 `/initialpose`。

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

初期保持 PX4 estimator 使用原始 LIO 里程计。确认以下条件后再切换到
`/Odometry_map`：

1. `/relocalization/initialized` 为 true；
2. aligned cloud 与 prior map 重合；
3. 飞机静止时 `/Odometry_map` 没有跳变；
4. `map -> local_frame` 保持固定；
5. 重复启动的位置和 yaw 满足精度要求。

自动飞行必须由 ready 状态门控，任何 `WAITING`、`MATCHING` 或 `FAILED` 状态均
不能使用全局位姿解锁任务。
