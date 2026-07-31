# Docker 开发指南 — 单容器工作流

> 一个镜像，一个入口，一个容器。PX4 SITL 已烘焙进镜像，无需雷达仿真。

## 快速开始

```bash
# 1. 构建 dev 镜像（首次较慢，之后 PX4 层缓存）
docker build -f docker/Dockerfile -t jiangyin_jy2026 .

# 2. 跑 SITL（headless，PX4 iris + Gazebo）
docker run --rm -it --network=host jiangyin_jy2026 sitl

# 3. 跑权威冒烟测试（gzserver + px4 + IMU + MAVROS）
docker run --rm -it --network=host jiangyin_jy2026 smoke

# 4. 开发模式（编译 /ws/src + RA-LIO，然后交互 shell）
docker run --rm -it --network=host -v $PWD:/ws:Z jiangyin_jy2026 dev
```

## 入口模式 (jy-docker.sh)

| 模式 | 作用 |
|------|------|
| `shell` | 交互 bash（默认） |
| `sitl` | roscore → MAVROS → px4 + gzserver（headless） |
| `dev` | 编译 /ws/src（catkin_make --use-ninja + RA-LIO），然后 bash |
| `stack` | 算法栈（RA-LIO → px4_estimator → FSM+NMPC） |
| `all` | sitl（后台）+ stack（前台） |
| `takeoff` / `land` / `reset` | FSM UDP 指令 + Gazebo 复位 |
| `smoke` | 运行 jy-smoke-test |

## 开发循环（改代码 → 测试）

```bash
# 方式 A：dev 模式（源码挂载 + 容器内编译）
docker run --rm -it --network=host -v $PWD:/ws:Z jiangyin_jy2026 dev
#   → 容器内自动 catkin_make --use-ninja + RA-LIO，然后进 shell
#   → 改代码后重跑：cd /ws && catkin_make --use-ninja && source devel/setup.bash

# 方式 B：compose / devcontainer（编译 + 交互 shell）
podman-compose -f docker/compose.yml up -d --build  # jy 服务 = jy-docker.sh dev
podman-compose -f docker/compose.yml logs -f jy
```

## 常用操作

```bash
# 单独起 SITL 测试飞行控制
docker run --rm -it --network=host jiangyin_jy2026 sitl
# 另一终端：
docker exec -it <容器名> jy-docker.sh takeoff
docker exec -it <容器名> jy-docker.sh land

# 构建 prod 镜像（比赛用：烘焙源码 + 算法栈）
docker build -f docker/Dockerfile -t jiangyin_jy2026:prod --target prod .

# 限制 PX4 编译内存（低配机器）
docker build -f docker/Dockerfile --build-arg PX4_BUILD_JOBS=4 .
```

## 说明

- **无雷达仿真**：SITL 用 PX4 原生 iris 模型 + empty world。mid360 LiDAR 仅用于实机（RA-LIO 栈）。
- **ROS master 在 localhost**：单容器内 `http://localhost:11311`，无跨容器 DNS。
- **GUI**：图形化 Gazebo 模式已移除，全部 headless 运行。headless SITL 纯 CPU。
- **PX4 迭代**：高级用法 —— 将宿主 PX4 源码挂载到镜像内相同路径
  `-v $PWD/PX4-Autopilot:/opt/PX4-Autopilot:ro`（需与烘焙版本 ABI 兼容）。
- **镜像大小**：Podman 可用 `--squash` 压缩发布版
  `podman build --squash -f docker/Dockerfile -t jiangyin_jy2026:release .`
- **构建上下文**：`.dockerignore` 排除 build/devel/logs/.git 等。
