# docker/ — Container Image Pipeline

**5 layered images, 4 compose services, 6 deployment scripts.** Builds PX4 SITL + full algorithm stack.

## IMAGE HIERARCHY

```
osrf/ros:noetic-desktop-full (Ubuntu 20.04 + ROS Noetic)
├── localhost/jiangyin_px4_mid360:latest    # PX4 v1.14.3 + Gazebo Classic + iris_mid360 model
└── localhost/jiangyin_core:latest           # CasADi+IPOPT, Livox SDK2, Sophus (from .core or .core.prebuilt)
    ├── localhost/jiangyin_jy2026:with-mid360-sim  # Copies src/, runs catkin_make + RA-LIO at build time
    └── localhost/jiangyin_jy-dev:latest     # Adds clangd-22, libpcl-dev, Gazebo plugins, plotjuggler
```

## SERVICES (docker-compose.yml, jy-net bridge)

| Service | Image | Role | GPU |
|---------|-------|------|-----|
| `sim` | jiangyin_px4_mid360 | PX4 SITL headless (gzserver) | yes |
| `gui` | jiangyin_px4_mid360 | Gazebo GUI (gzclient, XRDP) | yes |
| `jy2026` | jiangyin_jy2026 | Algorithm pipeline (production) | no |
| `jy-dev` | jiangyin_jy-dev | Dev workspace (source mount, compile on start) | no |

## KEY SCRIPTS

| Script | Role |
|--------|------|
| `Dockerfile.core` / `.core.prebuilt` | Build core image from source or prebuilt debs |
| `Dockerfile.prod` | Build production image: copy src/, catkin_make, RA-LIO |
| `docker/dev/Dockerfile.dev` | Build dev image with clangd, PCL, Gazebo plugins |
| `docker/sim/Dockerfile` | Build simulation image (PX4 v1.14.3 + Gazebo Classic) |
| `dev/scripts/start_jy2026.sh` | Pipeline launcher (dev mode compiles on start) |
| `dev/scripts/jy-stack.sh` | Stack lifecycle: start/stop/status for all algorithm nodes |
| `dev/scripts/jy-sim-control.sh` | Sim controls: `jy-takeoff`, `jy-land`, `jy-reset` |
| `sim/scripts/start_px4_mid360.sh` | PX4 SITL entrypoint (roscore → MAVROS → px4+gzserver) |
| `sim/scripts/configure_px4_mid360.py` | Build-time: creates iris_mid360 model with simulated LiDAR |
| `build-debs.sh` / `build-debs-podman.sh` | Build CasADi/Sophus/Livox SDK2 .deb packages |

## NOTES

- **Build order**: debs (optional) → core → prod/dev (parallel) ↔ sim (standalone independent)
- **Dev vs Prod**: dev mounts source as volume (hot-reload), prod bakes compiled binaries at image build time
- **GPU**: sim and gui require NVIDIA GPU with nvidia-container-runtime; jy2026 and jy-dev do not
- **Network**: All containers communicate via `jy-net` bridge DNS (ROS_MASTER_URI=http://sim:11311)
- **No CI/CD**: All images built manually. No GitHub/GitLab CI pipeline.
