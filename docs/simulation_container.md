# PX4 + MID360 容器仿真

该环境在一个无 GPU 容器中运行 PX4 SITL、Gazebo Classic、MAVROS、MID360
点云桥接、RA-LIO、PX4 外部视觉转发以及 `flight_fsm`/NMPC。雷达使用 Gazebo
CPU ray 传感器，因此远程服务器不需要 X11、VirtualGL 或 NVIDIA 运行时。

## 镜像分层

- `docker/sim/Dockerfile`：从 ROS Noetic 基础环境构建完整 PX4/Gazebo 基础镜像。
  仅在 PX4 版本或系统依赖变化时使用。
- `docker/sim/Dockerfile.stack`：日常使用的薄派生镜像。复用
  `localhost/jiangyin_px4_mid360:latest`，只安装仓库内的预编译 deb、更新 MID360
  模型并编译当前工作区。航点封存也在这一层生成。
- `docker/sim/build-stack-podman.sh`：启用 Podman layers 的构建入口。
- `docker/sim/scripts/configure_px4_mid360.py`：幂等生成 PX4 airframe 和 Gazebo
  模型，并修补 Gazebo ROS API 插件。
- `docker/sim/scripts/start_sim_stack.sh`：单容器进程编排和逐级就绪检查。
- `docker/sim/scripts/smoke_sim_stack.py`：消息、安装位姿和可选飞行回归检查；
  `smoke_sim_stack.sh` 自动加载 ROS 环境。
- `docker-compose.yml`：最终运行入口，仅包含 `sim-stack` 服务。

已有基础镜像时，在仓库根目录执行：

```bash
BUILD_JOBS=4 ./docker/sim/build-stack-podman.sh
podman compose up -d
podman compose exec sim-stack smoke_sim_stack
```

若基础镜像标签不同，可以覆盖 `PX4_SIM_BASE`；输出标签可用
`SIM_STACK_IMAGE` 覆盖。派生镜像构建全程使用本地 deb，不依赖构建时访问 apt
仓库。服务器实测基础镜像为 6.88 GB，派生镜像为约 7.16 GB。

## 雷达安装与坐标

MID360 相对机体/IMU 的安装位姿统一为：

- 平移：`[0.0, 0.0, 0.12]` m
- 绕 Y 轴俯仰：`-15 deg`
- RA-LIO 外参旋转：
  `[0.9659258263, 0, -0.2588190451; 0, 1, 0; 0.2588190451, 0, 0.9659258263]`

模型生成器和 `src/RA-LIO/config/mid360.yaml` 使用同一组数值。冒烟脚本通过
Gazebo `/gazebo/get_link_state` 服务独立检查安装位姿，而不是只检查配置文件。
点云桥将 `PointCloud2` 转为 Livox `CustomMsg`，`timebase` 和每点
`offset_time` 均使用纳秒。

## 服务器验证结果

在 `yh_server_wenliang` 上已经验证：

- 镜像可由缓存层完成构建，PX4 增量模型生成和 catkin 工作区编译成功；
- Gazebo CPU ray 点云约 46080 点/帧，Livox 转换后约 17745 个有效点；
- 雷达实测安装高度 `0.120 m`、俯仰 `-15.00 deg`；
- MAVROS 已连接，RA-LIO `/Odometry`、IMU、PX4 本地位姿和状态机节点均持续可用；
- 普通冒烟检查输出 `SIM_STACK_SMOKE_OK`。

完整起飞回归当前仍失败。PX4 原生健康报告指出
`local_position_estimate` 起飞前检查失败，RA-LIO 静止初始化后的
`vehicle_local_position.heading_good_for_control` 为 false。测试中仅为诊断强制解锁
后，PX4 能进入 OFFBOARD，NMPC 推力和四路电机输出均已到达仿真器，但 RA-LIO
反馈姿态与 Gazebo 真值存在约 180 度的航向/四元数差异，控制器产生较大角速度和
不均衡电机输出，无人机未离地。因此 `smoke_sim_stack --flight` 应保留为后续坐标系
对齐修复的回归门槛，不应在 CI 中绕过 PX4 起飞前检查。

排查日志位于容器内 `/tmp/jiangyin_sim/`。入口默认设置 `NO_PXH=1`，避免 PX4
交互提示符在无 TTY 环境中持续写入日志。
