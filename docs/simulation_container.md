# PX4 + MID360 容器仿真

该环境在一个无 GPU 容器中运行 PX4 SITL、Gazebo Classic、MAVROS、MID360
点云桥接、RA-LIO、PX4 外部视觉转发以及 `flight_fsm`/NMPC。雷达使用 Gazebo
CPU ray 传感器，因此远程服务器不需要 X11、VirtualGL 或 NVIDIA 运行时。

## 镜像结构

仓库只保留三个镜像构建文件：依赖包、开发编译环境和完整仿真环境。

- `docker/Dockerfile.debs`：构建可复用的第三方依赖 deb；
- `docker/dev/Dockerfile.dev`：开发和编译工作区，由 devcontainer 自动构建；
- `docker/sim/Dockerfile`：单文件构建完整 PX4/Gazebo、MID360、RA-LIO 和控制栈。

开发镜像复用 `localhost/jiangyin_dev:v0.2` 中稳定的 ROS、编译器和 IDE 工具层；
仿真镜像复用 `localhost/jiangyin_px4_mid360:latest` 中昂贵且稳定的 PX4/Gazebo
工具链。仓库只维护项目相关的 deb、模型、算法和入口层，日常重建可直接命中缓存。

- `docker/sim/build-sim-podman.sh`：启用 Podman layers 的仿真构建入口。
- `docker/sim/scripts/configure_px4_mid360.py`：幂等生成 PX4 airframe 和 Gazebo
  模型，并修补 Gazebo ROS API 插件。
- `docker/sim/scripts/start_sim_stack.sh`：单容器进程编排和逐级就绪检查。
- `docker/sim/scripts/smoke_sim_stack.py`：消息、安装位姿和可选飞行回归检查；
  `smoke_sim_stack.sh` 自动加载 ROS 环境。
- `docker-compose.yml`：最终运行入口，仅包含 `sim-stack` 服务。

在仓库根目录执行：

```bash
BUILD_JOBS=4 ./docker/sim/build-sim-podman.sh
podman compose up -d
podman compose exec sim-stack smoke_sim_stack
```

基础标签可用 `PX4_SIM_BASE` 覆盖，输出标签可用 `SIM_IMAGE` 覆盖，编译并行度
可用 `BUILD_JOBS` 调整。CasADi、Livox SDK/ROS 驱动和 Sophus 使用仓库内 deb。

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
- 普通冒烟检查输出 `SIM_STACK_SMOKE_OK`；
- 正常解锁、OFFBOARD 和 NMPC 起飞通过，没有绕过 PX4 起飞前检查；
- 最终精简镜像中正常起飞至 `0.587 m`，Gazebo 与 RA-LIO 三轴位移最大误差为
  `0.016 m`；
- FSM 测试共 84 项，0 错误、0 失败。

排查日志位于容器内 `/tmp/jiangyin_sim/`。入口默认设置 `NO_PXH=1`，避免 PX4
交互提示符在无 TTY 环境中持续写入日志。
