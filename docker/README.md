# Docker — Jiangyin2026 Container Pipeline

One image, one entrypoint (`docker/scripts/jy-docker.sh`), one headless container. PX4 SITL is baked into the image; no LiDAR simulation.

Graphical Gazebo mode has been removed. All runtime is headless.

## Build

```bash
docker build -f docker/Dockerfile -t jiangyin_jy2026 .
docker build -f docker/Dockerfile -t jiangyin_jy2026:prod --target prod .
docker build -f docker/Dockerfile --build-arg PX4_BUILD_JOBS=4 .
docker build -f docker/Dockerfile --build-arg RUN_SMOKE=1 .
```

## Run

```bash
# PX4 SITL headless: roscore -> MAVROS -> px4 + gzserver
docker run --rm -it --network=host jiangyin_jy2026 sitl

# Runtime gate: gzserver + px4 + /mavros/imu/data + /mavros/state
docker run --rm -it --network=host jiangyin_jy2026 smoke

# Dev shell: compile /ws/src (catkin_make --use-ninja + RA-LIO), then bash
docker run --rm -it --network=host -v "$PWD:/ws:Z" jiangyin_jy2026 dev

# Full sim + algorithm stack in one headless container
docker run --rm -it --network=host -v "$PWD:/ws:Z" jiangyin_jy2026 all

# Algorithm stack lifecycle / FSM commands inside a running container
docker exec -it <container> jy-docker.sh stack start|stop|status
docker exec -it <container> jy-docker.sh takeoff|land|reset
```

Modes (`docker/scripts/jy-docker.sh --help`): `shell`, `sitl`, `dev`, `stack`, `all`, `takeoff`, `land`, `reset`, `smoke`.

## Compose / Devcontainer

`docker/compose.yml` defines the single `jy` service. It builds `docker/Dockerfile`, bind-mounts the repository to `/ws`, and passes `dev` to the image entrypoint (`jy-docker.sh`). Its primary consumer is the VS Code devcontainer (`.devcontainer/devcontainer.json`).

```bash
podman-compose -f docker/compose.yml up -d --build
podman-compose -f docker/compose.yml down
```

## Notes

- PX4: v1.14.3 pinned at SHA `de8a295af4d8192a3e85b2565040367378a07d8e`.
- Headless: stock PX4 iris model + empty world, CPU-only. The mid360 LiDAR is real-hardware only.
- No CI/CD: images are built manually.

## ARM64 (NVIDIA Jetson)

Arm64 debs and images are built on an x86_64 host via QEMU emulation:

```bash
./docker/build-arm64.sh debs    # build arm64 debs -> deb/arm64/ (slow)
./docker/build-arm64.sh image   # build jiangyin_jy2026:arm64 (PX4 bake: hours)
PX4_BUILD_JOBS=4 ./docker/build-arm64.sh   # both, jobs capped
```

- Base image: `ros:noetic-ros-base-focal` (official arm64) + OSRF gazebo repo for `gazebo11`.
- Tag: `jiangyin_jy2026:arm64` (amd64 stays `:latest`).
- **Smoke-testing**: QEMU emulation does not satisfy PX4 lockstep timing — run `jy-smoke-test` on the Jetson, not the x86 host.
- Run on the Jetson: `podman run --rm -it --network=host jiangyin_jy2026:arm64 sitl` (or `all`).
