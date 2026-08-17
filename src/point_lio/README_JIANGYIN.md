# 江阴机第一阶段接入说明

本阶段保留 PX4 EKF2：MID360 按 10 Hz 发布完整点云帧，Point-LIO 以约 20 Hz 向 SUPER 发布 `/Odometry`；桥接器每个雷达帧只把最新校正状态送入 `/mavros/odometry/out` 供飞控融合。旧的 `px4_estimator` 不应同时运行。

## 启动

完整实机流程由 `jiangyin` 命令读取 `~/.config/zellij/layouts/jiangyin.kdl` 启动。该流程使用 Point-LIO 自带的 MID360 launch，驱动按物理 10 Hz 发布：每约 100 ms 一帧、每帧约 2 万点、总点频约 20 万点/秒。

当前参数按室内飞行配置：保留 0.4 m 近场过滤、0.2 m 点云/地图降采样和全部有效原始点，最大有效测距限制为 50 m。不要为了提高“信任度”继续降低 `lidar_meas_cov`；先通过动态飞行日志检查匹配退化、速度抖动和 PX4 innovation。

单独调试定位时可以执行：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch point_lio msg_mid360.launch
# 另开终端：
roslaunch point_lio stage1_mid360.launch publish_to_px4:=false
```

先用 `publish_to_px4:=false` 做地面检查。点云、姿态、速度和频率正常后，再改为 `true` 送入 PX4。

SUPER 使用已有接口：

- 地图点云：`/cloud_registered`
- 规划里程计：`/Odometry`
- Point-LIO 原始输出：`/point_lio/odometry`
- PX4 外部里程计：`/mavros/odometry/out`
- 桥接健康状态：`/pointlio_mavros_bridge/healthy`

如果手动启动控制侧，应关闭旧桥：

```bash
roslaunch fsm_ctrl swarm.launch start_estimator:=false
roslaunch mission_planner flag_happy_fly.launch
```

## 上机前检查

```bash
rostopic hz /point_lio/odometry
rostopic hz /Odometry
rostopic hz /mavros/odometry/out
rostopic echo /pointlio_mavros_bridge/healthy
```

预期 `/livox/lidar` 和 `/cloud_registered` 约 10 Hz，Point-LIO 与 SUPER 里程计平均约 20 Hz，PX4 外部里程计约 10 Hz，健康状态为 `true`。20 Hz定位状态会随每个100 ms雷达帧批量到达；桥接器只向PX4发送每帧最后一个校正状态，由飞控IMU在两次外部观测之间传播。静止放平时重点确认：

1. `/point_lio/odometry` 的 roll、pitch 接近 0，移动机头时姿态方向正确。
2. 手推飞机时位置轴和 `/cloud_registered` 在同一个坐标系内，不出现反向或镜像。
3. `/mavros/local_position/odom` 在开启 PX4 输入后无跳变、无持续 innovation reject。
4. 确认 ROS 中只有一个节点发布 `/mavros/odometry/out`，再解锁飞行。

MID360 配置中的 `body_R_wrt_IMU` 使用当前机器的 +30° 绕 Y 安装补偿；如果实际支架角度变化，需要先修改 `config/mid360.yaml`，不能带着错误安装角起飞。
