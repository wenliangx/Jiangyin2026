---
title: "Round 1 — Minimalism Analysis: ONE Dockerfile for Dev + Full PX4 SITL"
author: Minimalist (hyperplan team member)
date: 2026-07-31
TeamRunId: 72ea875e-e8bc-40de-881a-65b65a745db3
---

## The Problem Space

**6 Dockerfiles → 1.** User wants a single Dockerfile at repo root that:
1. Does dev build (source mount → compile on container start)
2. Has baked-in PX4 SITL + Gazebo Classic for simple testing

---

## Redundancy Map Across 6 Current Dockerfiles

### 1. `docker/Dockerfile.debs` — KEEP AS-IS (detached build tool)
- **Purpose**: Build .deb packages for CasADi/Sophus/Livox-SDK2/Livox-ROS-Driver2
- **Verdict**: MUST KEEP. `build-debs.sh` and `build-debs-podman.sh` explicitly depend on it.
- But it must stay DECOUPLED — not merged into the single Dockerfile. It's a build tool, not a runtime image.

### 2. Dual core: `Dockerfile.core` (source) + `Dockerfile.core.prebuilt` (debs)
- Both produce `localhost/jiangyin_core:latest` — same base image, different installation paths
- `.core` installs from source (~40min build, ~1.5GB wasted on build artifacts)
- `.core.prebuilt` installs from `deb/` directory (~5.1MB total, instant)
- **Verdict**: Use the `.core.prebuilt` pattern ONLY. The `deb/` directory has all 5 prebuilt packages.

### 3. `docker/sim/Dockerfile` — CONTENTS BELONG IN SINGLE FILE
- 147 lines: apt deps (~35 pkg), cmake upgrade, CasADi build (redundant!), Sophus/Livox build, PX4 clone+submodules, pip deps, iris_mid360 model, SITL build, entrypoint script
- **Key insight**: builds OLD `Livox-SDK`, not `Livox-SDK2` — this is a bug
- The PX4 SITL build stage (lines 101-139: clone PX4, submodules, pip deps, configure_px4_mid360.py, `DONT_RUN=1 make px4_sitl gazebo-classic`) is the CORE addition for the single file

### 4. `docker/Dockerfile.prod` — ENTIRELY DEFLETED BY DEV MODEL
- Copies `src/`, bakes `catkin_make` binaries, bakes RA-LIO at build time
- **Fatal flaw**: user explicitly said "dev compiles /ws/src at container start" — "baked at build" is the opposite of what's needed
- **Verdict**: DISCARDED. The dev model (source mount + compile-on-start) replaces prod entirely

### 5. `docker/dev/Dockerfile.dev` — THE TEMPLATE
- 59 lines: `FROM jiangyin_core`, clangd-22, libpcl-dev, Gazebo plugins, symlink shortcuts → `CMD ["bash", "/opt/start_jy2026.sh", "--dev"]`
- **Verdict**: This is the structural template for the single Dockerfile (minus the separate jiangyin_core dependency)

---

## Layer-by-Layer Specification for The ONE Dockerfile

### L0: BASE
```
FROM docker.io/osrf/ros:noetic-desktop-full
```

### L1: APT DEPS (MERGED FROM ALL + DEDUPED)
From `.core` line 10-51 + `sim` line 23-66 + `dev` line 33-44

**Remove from merged list**:
- `clangd` → L2 installs clangd-22 specifically from llvm repo
- `libapriltag-dev` → no actual project usage post-refactor

### L2: CMAKE UPGRADE
Download/extract CMake 3.28.3 to `/opt/cmake`, set `PATH=/opt/cmake/bin:${PATH}`

### L3: PREBUILT DEBS INSTALL
```
COPY deb/ /tmp/jiangyin-debs/
RUN apt install /tmp/jiangyin-debs/*.deb && rm -rf /tmp/jiangyin-debs
```
Installs: casadi-dev, livox-sdk2-dev, livox-ros-driver2, sophus-dev, source-deps meta, ros1-deps meta

### L4: GIS DATASETS + PX4 PYTHON DEPS
```
RUN /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
RUN pip3 install argcomplete cerberus empy future jsonschema kconfiglib lxml matplotlib packaging psutil pyros-genmsg pyserial pyyaml six toml tqdm uavcan wheel
```

### L5: PX4 SOURCES + SUBMODULES
```
RUN git clone --depth 1 --branch v1.14.3 https://github.com/PX4/PX4-Autopilot.git /opt/PX4-Autopilot
RUN cd /opt/PX4-Autopilot && git submodule update --init --depth 1 \
    Tools/simulation/gazebo-classic/sitl_gazebo-classic \
    src/drivers/gps/devices src/lib/events/libevents \
    src/modules/mavlink/mavlink src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client
```

### L6: IRIS_MID360 MODEL + SITL BUILD (THE CORE)
```
COPY docker/sim/scripts/configure_px4_mid360.py /usr/local/bin/
COPY docker/sim/worlds/obstacle_test.world /opt/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/
RUN python3 /usr/local/bin/configure_px4_mid360.py
RUN cd /opt/PX4-Autopilot && DONT_RUN=1 make px4_sitl gazebo-classic
```

### L7: CLANGD-22 + PCL + GAZEBO PLUGINS + PLOTJUGGLER
From `dev` Dockerfile: LLVM apt source → clangd-22 → libdw-dev, libpcl-dev, gazebo-msgs, gazebo-ros, gazebo-plugins, gazebo-ros-control, velodyne-gazebo-plugins, plotjuggler

### L8: ENTRYPOINT ROUTER SCRIPTS
Merge 4 scripts → 1 `/usr/local/bin/jy-docker.sh` mode router (see below)

### L9: FINAL SETUP
```
WORKDIR /ws
ENTRYPOINT ["/usr/local/bin/jy-docker.sh"]
CMD ["shell"]
```

---

## Merged Entryoint Router: `jy-docker.sh <mode> [args?]`

Replaces 4 scripts: `start_px4_mid360.sh`, `start_jy2026.sh`, `jy-stack.sh`, `jy-sim-control.sh`

| Mode | Behavior |
|------|----------|
| `shell` | Interactive bash (dev default, with source mount) |
| `sitl` | Start PX4 SITL headless (roscore → MAVROS → px4+gzserver) |
| `gui` | Start PX4 SITL + gzclient (needs GPU/X11) |
| `dev` | Mount source → catkin_make → source setup.bash → bash |
| `stack` | Start algorithm stack (RA-LIO → px4_estimator → FSM+NMPC) |
| `all` | sitl + stack together |
| `takeoff` | Send FSM cmd=3 (UDP) |
| `land` | Send FSM cmd=4 (UDP) |
| `reset` | Reset Gazebo + stop FSM |

---

## compose.yml Target: 4 services → 1 service + profiles

Current: 4 services (sim, gui, jy2026, jy-dev) → separate images, separate networks
Target: 1 service with profile-based commands

```yaml
services:
  jy:
    image: localhost/jiangyin_jy2026:latest
    profiles: ["default"]      # dev shell with source mount (95% usage)
    volumes: [".:/ws:Z"]      # hot-reload source
    # Default mode: shell

  # --profile sim: PX4 SITL headless
  # --profile gui: PX4 SITL + gzclient  
  # --profile dev: compile source + start stack

  # Usage:
  #   podman-compose up -d                    # dev shell
  #   podman-compose -p sim up -d             # SITL headless + GUI in same container
  #   podman-compose -p dev up -d             # source mount + compile + stack
```

---

## Estimated Size: ~8.5 GB

| Layer | Accumulated Size |
|-------|-----------------|
| L0 base | ~3.5 GB |
| L1 L2 apt+cmake | +~1.2 GB |
| L3 debs | +50 MB |
| L4 GIS+pip | +100 MB |
| L5 PX4 sources | +800 MB |
| L6 SITL build+models | +2.5 GB |
| L7 clangd+tools | +200 MB |
| L8-L9 scripts+entrypoint | +5 MB |
| **TOTAL** | **~8.5 GB** |

---

## Files Changed: Summary

| Action | File(s) |
|--------|---------|
| **NEW** | `Dockerfile` (repo root, ~150 lines), `.dockerignore`, `docker-compose.yml` |
| **NEW** | `docker/jy-docker.sh` (replaces 4 scripts) |
| **MOVED** | `configure_px4_mid360.py`, `obstacle_test.world` → stay `docker/sim/` (build-time COPY) |
| **DELETED** | `Dockerfile.core`, `Dockerfile.core.prebuilt`, `Dockerfile.prod`, `Dockerfile.dev`, `docker/sim/Dockerfile` |
| **KEPT** | `Dockerfile.debs`, `build-debs.sh`, `build-debs-podman.sh`, `deb/` directory |

---

## 3 Strongest Positions to Defend

### Position 1: USE THE .DEBS — NEVER BUILD CASADi/SOPHUS/LIVOX IN-IMAGE
The `deb/` directory already contains all 5 packages (4.1MB + 400KB + 568KB + 40KB + meta = ~5.1MB total). Building them from source in a Dockerfile wastes ~40min build time + ~1.5GB on build artifacts. The `.core.prebuilt` pattern is strictly superior: `COPY deb/ && apt install *.deb`. Dockerfile.debs stays as a detached build tool only.

**Evidence**: `Dockerfile.core` has 50+ lines of source build with git clone + cmake + make. `Dockerfile.core.prebuilt` is 16 lines with apt install. Same end result.

### Position 2: MERGE ALL ENTRYPOINT SCRIPTS INTO ONE ROUTER
Four nearly-duplicate scripts (`start_px4_mid360.sh`, `start_jy2026.sh`, `jy-stack.sh`, `jy-sim-control.sh`) share ~70% boilerplate: ROS_MASTER_URI defaults, roscore wait loops, launch patterns, PID tracking, log formatting. One `jy-docker.sh <mode>` with case-driven subcommands is cleaner and easier to maintain than 4 separate 80-160 line scripts with duplicated logic.

**Evidence**: `start_jy2026.sh` line 23-26 (ROS_MASTER_URI logic) is nearly identical to `start_px4_mid360.sh` line 33. Both wait for roscore with identical 30-60 second loops. Both source `/opt/ros/noetic/setup.bash`.

### Position 3: DEFER THE PROD BAKED IMAGE ENTIRELY
`Dockerfile.prod` (catkin_make baked at build time) is dead weight for dev workflow. The source-mount + compile-on-start model is strictly better: one command, hot-reload, no rebuild needed for code changes. If production deployment (baked binaries) is ever needed, a 3-line multi-stage target from the dev image handles it. The current 4-service compose (sim+gui share one image, jy2026+jy-dev forked from prod) is an anti-pattern — one service + profiled command-line modes solves all four use cases.

**Evidence**: `jy-dev` already does source mount with volume bind (`- .:/ws:Z`). `jy2026` bakes at build time but adds ZERO value over `jy-dev` with a `RUN catkin_make` step done lazily on first container start.

