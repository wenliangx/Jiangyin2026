# Zellij 航点打点会话

该会话用于实机打点，一次启动以下节点：

- ROS master、MID360 驱动、RA-LIO 定位；
- `px4_estimator`，将 `/Odometry` 送入 PX4 融合；
- 前视和下视相机、前视目标分类、下视 AprilTag 识别；
- 航点记录程序和识别结果监视器。

## 启动

在工作空间根目录执行：

```bash
./scripts/start_waypoint_recording_zellij.sh
```

默认写入：

```text
src/fsm_ctrl/config/mission_super_waypoints.yaml
```

也可以把输出文件作为第一个参数传入：

```bash
./scripts/start_waypoint_recording_zellij.sh /home/flag/mission_super_waypoints.yaml
```

脚本会检查 zellij、ROS Noetic 和工作空间的 `devel/setup.bash`。已有 ROS
master 时会复用它。RA-LIO 发布 `/Odometry` 后才启动 PX4 融合；打点程序会
继续等待 `/mavros/local_position/pose`，因此看到交互提示符时融合位姿已经可用。

## 页面

- `打点`：左侧运行交互式航点记录，右侧同时打印两路识别结果；
- `定位`：ROS master、MID360、RA-LIO、PX4 融合日志；
- `视觉`：双相机、下视 AprilTag、前视分类日志。

打点命令沿用 `record_super_waypoints.py`：Enter 或 `a` 添加，`u` 撤销，
`seg N` 切换任务段，`n`/`p` 切换前后任务段，`s` 保存，`q` 保存并退出。

该会话不启动飞行状态机，所以双相机以 `always_enabled:=true` 运行。退出时按
`Ctrl-o`、`d` 分离会话；需要彻底结束所有 pane 时在 zellij 内按 `Ctrl-q`。
