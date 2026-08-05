# arm64 Image Port — Design

**Date:** 2026-08-05
**Status:** Approved
**Branch:** codex/ra-lio-pcd-localization (work on feature branch)
**Related:** `docs/2026-07-31-docker-compose-simplify-design.md` (single-image model), commit `1632adb` (removed legacy docker assets, incl. deb builder)

## 1. Overview

Produce an **arm64 (aarch64) version of the Jiangyin2026 Docker image**, targeting an **NVIDIA Jetson (Orin/NX)** flight computer. Deliverables, in order:

1. **arm64 `.deb` packages** for the four source-built dependencies (CasADi, Livox-SDK2, livox_ros_driver2, Sophus) + the two `_all.deb` metapackages.
2. A **fully runnable arm64 image** equivalent to the x86 dev image: ROS Noetic + Gazebo + PX4 SITL baked + all `jy-docker.sh` modes.

The x86 pipeline (currently committed binaries in `deb/`, one arch-aware `docker/Dockerfile`) is the reference. The deb builder (`docker/Dockerfile.debs` + `docker/build-debs.sh`) was deleted in `1632adb` and is **restored from git** (`1632adb^`), then made arch-parameterized.

## 2. Constraints & Decisions (confirmed with user)

| # | Decision | Value |
|---|----------|-------|
| D1 | Target hardware | NVIDIA Jetson (Orin/NX), Ubuntu 20.04 arm64 (L4T r35). CPU-only stack — generic arm64 Ubuntu base is acceptable now; CUDA/L4T-specific base is out of scope. |
| D2 | Build environment | This x86 host via `podman build --platform linux/arm64` + QEMU aarch64 binfmt (registered). No docker/buildx. |
| D3 | Image scope | **Full dev image incl. PX4 SITL + Gazebo** (sitl/all modes) — faithful to the x86 reference. |
| D4 | Deb pipeline | **Restore arch-aware deb builder**; arm64 debs stored in `deb/arm64/`; main Dockerfile made arch-aware (single file, no fork). |
| D5 | Image tag | `jiangyin_jy2026:arm64` (amd64 keeps `:latest`). |

## 3. Verified Facts (2026-08-05)

- `osrf/ros:noetic-desktop-full` is **amd64-only** on Docker Hub — cannot be used for arm64.
- Official `ros:noetic-ros-base-focal` **has an arm64 manifest** (verified: pulls fine via podman on this host).
- **All required ROS/OSRF packages exist for focal arm64** (verified in `packages.ros.org` pool):
  - `ros-noetic-gazebo-ros`, `gazebo-ros-control`, `gazebo-msgs`, `gazebo-plugins` (2.9.3-1focal…`_arm64.deb`, built 2025-05-21)
  - `ros-noetic-velodyne-gazebo-plugins` (1.0.13-1focal…`_arm64.deb`)
  - `ros-noetic-mavros(-extras)`, `ros-noetic-plotjuggler(-ros)`, `ros-noetic-rosfmt` — arm64 in pool
- `gazebo11` focal arm64 available from **OSRF gazebo apt repo** (`packages.osrfoundation.org`, e.g. `11.15.1-1~focal_arm64.deb`). Ubuntu universe arm64 availability unverified → **add OSRF gazebo repo** in the arm64 build (safe either way).
- **PX4 v1.14.3 SITL builds natively on aarch64** (PX4 CI builds arm64 debs; official `px4io/px4-sitl` arm64 image exists; M1/Jetson community reports working). No multilib issue — our Dockerfile does not run `ubuntu.sh`.
- CMake `cmake-3.28.3-linux-aarch64.tar.gz` exists on GitHub releases (verified, 302 → release asset).
- **QEMU emulation**: builds work but ~20–100× slower (PX4 bake = hours). SITL **runtime** under QEMU is unreliable (PX4 lockstep + wall-clock timing) — runtime verification must happen on the real Jetson.
- apt/OSRF **GPG key rotation** (2025) — use 2026-era base images and current key install.

## 4. Architecture

```
                        ┌─────────────────────────────────────────────┐
   podman build         │  docker/Dockerfile.debs  (arch-aware)        │
   --platform arm64 ──► │  CasADi + Livox-SDK2 + livox_ros_driver2    │
                        │  + Sophus → DESTDIR → build-debs.sh         │
                        │  (dpkg --print-architecture → arm64)        │
                        └─────────────────────────────────────────────┘
                                        │ deb/arm64/*_arm64.deb
                                        ▼
                        ┌─────────────────────────────────────────────┐
   podman build         │  docker/Dockerfile  (arch-aware, single file)│
   --platform arm64 ──► │  ros:noetic-ros-base-focal@arm64            │
                        │  + gazebo11(OSRF) + gazebo-ros-pkgs(arm64)  │
                        │  + CMake aarch64 + PX4 SITL bake            │
                        │  + jy-docker.sh → jiangyin_jy2026:arm64     │
                        └─────────────────────────────────────────────┘
```

## 5. Components

### 5.1 Deb builder — `docker/Dockerfile.debs` + `docker/build-debs.sh` (restored + parameterized)

- Restore both files from git `1632adb^` unchanged as the baseline.
- **Parameterize base image**: `ARG ROS_BASE_IMAGE` defaulting to `docker.io/osrf/ros:noetic-desktop-full` (preserves x86 behavior); arm64 build passes `ros:noetic-ros-base-focal`.
- Everything downstream is already arch-agnostic: `dpkg-deb --build` writes `Architecture: $(dpkg --print-architecture)` (arm64 inside the arm64 build container).
- **Output**: run `build-jiangyin-debs /pkg /out/debs` → `_arm64.deb` files for the 4 arch-specific packages; copy the 2 `_all.deb` metapackages (arch-independent, version-identical — no rebuild).
- **Destination**: `deb/arm64/`. Existing amd64 files in `deb/` untouched.
- **Verification gate in builder**: after build, assert `dpkg --info <deb> | grep Architecture: arm64` for each arch-specific deb.

### 5.2 Main Dockerfile — `docker/Dockerfile` (arch-aware, single file)

| Location | Today (x86-hardcoded) | Change |
|---|---|---|
| L0 base | `osrf/ros:noetic-desktop-full@sha256:7dbf…` (amd64-only digest) | `ARG ROS_BASE_IMAGE` (arm64 default `ros:noetic-ros-base-focal`; amd64 keeps pinned osrf image). Digest pin only for amd64 default. |
| L0 apt (gazebo11) | Ubuntu universe | arm64: add OSRF gazebo apt repo (`packages.osrfoundation.org` focal) + its key before `gazebo11` install; keep Ubuntu universe as fallback (no-op if present). |
| L2 CMake | `cmake-3.28.3-linux-x86_64.tar.gz` | `ARG CMake_ARCH` / `TARGETARCH` → `…-linux-aarch64.tar.gz` on arm64. |
| L3 deb COPY | `COPY deb/jiangyin-*.deb /tmp/` (globs match both archs) | `ARG DEB_DIR=deb` → `deb/arm64` on arm64 so only arm64 debs enter the image. Add arch sanity check before install (dpkg-query / grep). |
| L9 plugin symlink + L-ENV `GAZEBO_PLUGIN_PATH` | `/usr/lib/x86_64-linux-gnu/gazebo-11/plugins` | `ARG GZ_LIBDIR` → `aarch64-linux-gnu` on arm64. |
| L7 PX4 bake | `make px4_sitl gazebo-classic -j… j=…` | Unchanged (builds natively on aarch64). Keep `PX4_BUILD_JOBS` cap; note arm64 bake is slow under QEMU. |
| Everything else | unchanged | unchanged (jy-docker.sh, entrypoint, prod stage, smoke gate). |

Arch plumbing: use the standard platform build args (`TARGETARCH`, `TARGETPLATFORM`) if podman/buildah provide them with `--platform`; otherwise pass explicit `--build-arg` (verify at implementation; fallback = explicit args).

### 5.3 `docker/scripts/jy-docker.sh` — runtime arch-agnostic paths

Lines 55–56 hardcode `/usr/lib/x86_64-linux-gnu/gazebo-11/plugins` into `GAZEBO_PLUGIN_PATH`/`LD_LIBRARY_PATH`. Since the Dockerfile ENV already sets the correct arch path, jy-docker.sh's append would inject a bogus x86 dir on arm64. Fix: derive at runtime —

```bash
gazebo_libdir="/usr/lib/$(dpkg --print-architecture)-linux-gnu/gazebo-11/plugins"
export GAZEBO_PLUGIN_PATH="/opt/ros/noetic/lib:${gazebo_libdir}${GAZEBO_PLUGIN_PATH:+:${GAZEBO_PLUGIN_PATH}}"
export LD_LIBRARY_PATH="/opt/ros/noetic/lib:${gazebo_libdir}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
```

(Behavior unchanged on amd64: `x86_64-linux-gnu`.)

### 5.4 Build orchestration — `docker/build-arm64.sh` (new)

1. `podman build --platform linux/arm64 -f docker/Dockerfile.debs -t jiangyin-debs-builder:arm64 .`
2. Extract debs → stage into `deb/arm64/` (copy `_all.deb` from `deb/`).
3. `podman build --platform linux/arm64 --build-arg PX4_BUILD_JOBS=<cap> -f docker/Dockerfile -t jiangyin_jy2026:arm64 .`
4. Print a README-style summary: expected long build (QEMU), verification steps, note that SITL smoke must run on the Jetson.

## 6. Verification Plan

| Level | Check | Where |
|---|---|---|
| Deb metadata | `dpkg --info` → `Architecture: arm64` on all 4 arch debs | host (after build) |
| Deb installability | `podman run --platform linux/arm64` + `dpkg -i` all debs in a throwaway arm64 container; `dpkg-query -W` confirms installed | host (QEMU) |
| Deb usability | Compile a minimal consumer in the arm64 container: `#include <casadi/…>` + link, Sophus header include, `find_package(livox_ros_driver2 CONFIG)` | host (QEMU) |
| Image build | dev stage completes all RUN asserts (PX4 `bin/px4`, gazebo `.so`, livox cmake config, `*.o` cleanup count) | host (QEMU) |
| Image runtime | `podman run --platform linux/arm64 jiangyin_jy2026:arm64 shell` boots; `sitl` mode starts roscore + MAVROS + px4/gzserver (best-effort under QEMU) | host (QEMU) |
| **Full SITL smoke** | `jy-smoke-test` gate (gzserver + px4 + /mavros/imu/data + /mavros/state) | **Jetson (native)** — QEMU timing unreliable |

## 7. Risks & Mitigations

| Risk | Impact | Mitigation |
|---|---|---|
| QEMU build time (PX4 ~1000 files, 20–100× slower) | Hours per build | Cap `PX4_BUILD_JOBS`; run in tmux/background; layer caching makes rebuilds incremental; document expected time in `build-arm64.sh` header |
| SITL runtime under QEMU unreliable (lockstep timing) | False negatives in smoke | Boot-level verification only on host; authoritative smoke on Jetson |
| Ubuntu universe may lack gazebo11 arm64 | apt install failure | Add OSRF gazebo repo (authoritative, verified arm64) |
| apt/OSRF GPG key rotation (2025) | repo auth failure | 2026-era base images carry current keys; install OSRF key explicitly in builder + main image |
| Platform ARGs unavailable in podman/buildah | arch plumbing breaks | Verify `TARGETARCH` support; fallback to explicit `--build-arg` |
| Jetson L4T quirks (generic Ubuntu vs L4T) | runtime surprises | CPU-only stack → low risk; document; GPU/CUDA out of scope |

## 8. Out of Scope

- CUDA / L4T-specific base image or GPU acceleration.
- Running SITL under QEMU on the host as an accepted workflow.
- Changing the amd64 image behavior (must stay byte-compatible where the arch-independent parts are concerned).
- CI/CD for image builds.
