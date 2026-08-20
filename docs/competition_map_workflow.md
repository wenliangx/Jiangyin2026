# 赛前手举建图与比赛地图工作流

## 1. 手举建图

本阶段不再决定永久原点。启动后等待状态进入 `MAPPING`，再把无人机举到
约 1.2--1.8 m，缓慢行走并至少从两个方向覆盖每条通道。保存的 PCD 使用
本次 LIO 的临时局部坐标系；起步位置和机头方向都不会成为比赛原点。

```bash
roslaunch map_frame_manager create_map_ra_lio.launch \
  output_directory:=$HOME/maps/jiangyin \
  map_name:=competition_map \
  rviz:=false
```

建图配置默认开启手举模式，会排除机体下方的持机人区域。排除只作用于
工作地图；系统同时保留未过滤原图。

```bash
rostopic echo /map_frame_manager/state
rosservice call /map_frame_manager/save_map
```

输出：

```text
competition_map_raw.pcd    未过滤备份
competition_map.pcd        已排除持机人的工作图
competition_map.yaml       原点、坐标系与过滤参数
```

此时 YAML 中的 `origin_definition` 应为 `unanchored_lio_local_frame`。

## 2. 在建好的地图中确定永久原点

完成建图后，把飞机拿到希望的永久原点，机头对准永久 `lio_global +X`，保持静止。
使用 RA-LIO 时可由定原点工具一并启动 LIO：

```bash
roslaunch map_frame_manager set_origin_ra_lio.launch \
  map_file:=$HOME/maps/jiangyin/competition_map.pcd \
  output_directory:=$HOME/maps/jiangyin/permanent \
  map_name:=competition_map \
  auto_initialize:=false
```

如果 LIO 已经启动，则使用通用入口 `set_origin.launch`，参数相同。

在 RViz 中用 `2D Pose Estimate` 给出飞机在临时 PCD 中的大致位置和朝向。
该值只作为 NDT+ICP 的搜索种子，不会成为永久原点。等待匹配成功：

```bash
rostopic echo -n 1 /relocalization/initialized
rostopic echo /map_origin_setter/state
```

当状态为 `READY_TO_SET_ORIGIN` 时，确认飞机仍位于原点且机头朝 `+X`：

```bash
rosservice call /map_origin_setter/set_origin
```

工具把匹配到的当前机体位置设为 `(0,0,0)`，把当前航向设为零航向；滚转和
俯仰不会倾斜地图。输出为
`$HOME/maps/jiangyin/permanent/competition_map.pcd` 和对应 YAML。后续清理、
重定位和规划只使用这份永久坐标地图。重复走廊等场景不能保证无先验全局
匹配唯一，因此推荐始终在 RViz 中给出粗略种子并检查 ICP fitness。

## 3. 自动清理和派生

先根据实际赛场修改 `map_frame_manager/config/process_map.yaml` 中的
`venue_bounds`，然后执行：

```bash
roslaunch map_frame_manager process_map.launch \
  input_pcd:=$HOME/maps/jiangyin/permanent/competition_map.pcd \
  output_directory:=$HOME/maps/jiangyin \
  map_name:=competition_map
```

处理包括非法点删除、赛场边界裁剪、体素化、SOR 和半径离群点过滤。输出：

```text
competition_map_clean.pcd
competition_map_relocal.pcd
competition_map_planner.pcd
competition_map.yaml
```

## 4. CloudCompare 复核

1. 备份 `clean.pcd`，不覆盖 `raw.pcd`。
2. 同时打开 raw 和 clean，确认原点、方向和缩放完全重合。
3. 用顶视图和 `Segment` 多边形小范围删除人体行走拖影；再切换侧视图
   确认没有误删门框、柱子和低矮障碍。
4. 场外点使用 `Cross Section/Crop Box` 裁剪，墙外保留 0.3--0.5 m 余量。
5. SOR 只做保守复核：邻居 20--30，标准差 1.5--2.0；每次先预览。
6. 不使用 Align/Translate/Rotate 改变永久坐标，不人工制造缺失墙面。
7. 以 Binary PCD 保存，单位保持为米，保留 XYZ 和需要的 intensity。

人工修订后，将文件作为 `competition_map_clean.pcd`，再从该文件运行一次
派生流程。不要分别手工编辑 relocal 和 planner 版本。

## 5. 比赛启动

先启动重定位：

```bash
roslaunch map_frame_manager relocalize_ra_lio.launch \
  map_file:=$HOME/maps/jiangyin/competition_map_relocal.pcd \
  publish_to_original_odom_topic:=true \
  rviz:=false
```

比赛定位只支持固定原点 yaw 初始化：飞机放在永久 `(0,0,0)`，保持水平，
程序在全角度范围自动搜索 yaw。运行时不订阅 `/initialpose`，不允许 NDT/ICP
修改平移、roll 或 pitch。状态依次为 `COLLECTING_SCAN`、`MATCHING_YAW`、
`VERIFYING_YAW`、`READY`；连续多次航向不一致或场景有歧义时拒绝初始化。

确认：

```bash
rostopic echo -n 1 /relocalization/initialized
rostopic echo -n 1 /map_frame_manager/state
```

然后启动 SUPER：

```bash
roslaunch mission_planner flag_happy_fly.launch \
  static_map_file:=$HOME/maps/jiangyin/competition_map_planner.pcd \
  require_relocalization_ready:=true
```

SUPER 启动时把规划 PCD 加入 ROG-Map，运行中仍订阅 `/cloud_registered`
加入临时障碍。静态 PCD 只读，不会被 RA-LIO 或 SUPER 改写。mission planner
在 `/relocalization/initialized=true` 前保留航点但不发布给 SUPER。
