# docker/ — Container Image Pipeline

**Single unified image + 1 compose service.** One Dockerfile builds a dev image with PX4 SITL baked in; a `prod` target bakes the algorithm stack for competition day.

## IMAGE

```
osrf/ros:noetic-desktop-full (Ubuntu 20.04 + ROS Noetic, digest-pinned)
└── localhost/jiangyin_jy2026:latest   # single image: deps + PX4 SITL + Gazebo + jy-docker.sh
```

| File | Role |
|------|------|
| `Dockerfile` | The ONE Dockerfile. Default target = dev; `--target prod` = baked algo stack |
| `scripts/jy-docker.sh` | Single entrypoint mode router (shell/sitl/dev/stack/all/takeoff/land/reset/smoke) |
| `scripts/jy-smoke-test` | Authoritative runtime gate (gzserver + px4 + /mavros/imu/data + /mavros/state) |

## BUILD

```bash
# dev image (default; PX4 SITL baked, jobs = core count)
docker build -f docker/Dockerfile -t jiangyin_jy2026 .

# prod image (bakes src/ + catkin_make + RA-LIO for competition day)
docker build -f docker/Dockerfile -t jiangyin_jy2026:prod --target prod .

# optional: cap PX4 make jobs (RAM-limited machines), or run smoke at build
docker build -f docker/Dockerfile --build-arg PX4_BUILD_JOBS=4 .
docker build -f docker/Dockerfile --build-arg RUN_SMOKE=1 .
```

Context = repo root (`.dockerignore` there keeps it ~310MB).

## RUNTIME

Entrypoint `jy-docker.sh <mode>` — single container, ROS master on localhost:

| Mode | What it does |
|------|--------------|
| `shell` | interactive bash (default) |
| `sitl` | roscore → MAVROS → px4 + gzserver (headless, stock iris model) |
| `dev` | compile /ws/src (catkin_make --use-ninja + RA-LIO), then bash |
| `stack` | algorithm stack (RA-LIO → px4_estimator → FSM+NMPC) |
| `all` | sitl (bg) + stack (fg) |
| `takeoff` / `land` / `reset` | FSM UDP commands + Gazebo reset |
| `smoke` | run jy-smoke-test |

**No lidar/radar simulation** — SITL uses the stock PX4 iris model + empty world. The mid360 LiDAR is real-hardware only (RA-LIO stack in `dev`/`stack` modes).

## SERVICES (docker/compose.yml)

| Service | Profile | Role |
|---------|---------|------|
| `jy` | default | everything in one container (`jy-docker.sh dev`) |

```bash
podman-compose -f docker/compose.yml up -d --build  # build image, start jy (headless dev shell)
podman-compose -f docker/compose.yml down    # stop
```

## NOTES

- **PX4**: v1.14.3 pinned at SHA `de8a295af4d8192a3e85b2565040367378a07d8e` (not tag). Baked via `DONT_RUN=1 make px4_sitl gazebo-classic` with `j=$(nproc)` (PX4's Makefile reads the `j` variable, not `-j`).
- **GUI**: Graphical Gazebo mode removed — all runtime is headless. Headless SITL is CPU-only (stock iris has no GPU sensors).
- **Networking**: single-container localhost model (ROS_MASTER_URI=http://localhost:11311). No jy-net bridge.
- **Deployment**: prod target for field machines; `tmux-real.sh` covers bare-metal real-robot.
- **No CI/CD**: images built manually.
