# arm64 Image Port Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce arm64 (aarch64) `.deb` packages for the four source-built dependencies and a fully runnable `jiangyin_jy2026:arm64` image (ROS Noetic + Gazebo + PX4 SITL baked) by making the deb builder and main Dockerfile arch-aware, built on this x86 host via `podman build --platform linux/arm64` + QEMU emulation.

**Architecture:** Restore the deleted deb builder (`docker/Dockerfile.debs` + `docker/build-debs.sh`) from git `1632adb^` (already arch-parameterized via `ARG ROS_BASE_IMAGE` + `dpkg --print-architecture`), build it with `--platform linux/arm64` to emit `deb/arm64/*_arm64.deb`. Parameterize the single `docker/Dockerfile` with `ARG`s so the same file builds amd64 (defaults = current byte-identical values) and arm64 (overridden args). Add `docker/build-arm64.sh` to orchestrate both long QEMU builds.

**Tech Stack:** Dockerfile (Buildah-compatible), bash, podman 4.9 / buildah 1.33, QEMU aarch64 binfmt (registered on host), dpkg-deb, ROS Noetic (focal), PX4 v1.14.3 SITL.

## Global Constraints

- Spec: `docs/superpowers/specs/2026-08-05-arm64-image-port-design.md` (approved).
- All work on new branch `codex/arm64-image-port` created from current HEAD.
- **amd64 behavior must stay byte-identical** wherever arch-independent: every Dockerfile change must default to the current hardcoded value when `TARGETARCH` is unset/empty.
- arm64 base image: `docker.io/library/ros:noetic-ros-base-focal` (official, arm64 manifest verified).
- arm64 debs land in `deb/arm64/`; existing amd64 debs in `deb/` untouched.
- arm64 image tag: `jiangyin_jy2026:arm64`; amd64 keeps `:latest`.
- Gazebo plugin multiarch dir mapping: `uname -m` → `x86_64`→`/usr/lib/x86_64-linux-gnu/`, `aarch64`→`/usr/lib/aarch64-linux-gnu/`. **Do NOT use `dpkg --print-architecture`** (returns `amd64`/`arm64`, which do NOT match GNU multiarch dir names).
- CMake arch mapping: `TARGETARCH=arm64` → `cmake-3.28.3-linux-aarch64.tar.gz`; default (`amd64`/empty) → `linux-x86_64`.
- All `ros-noetic-gazebo-*`, `mavros`, `plotjuggler`, `velodyne-gazebo-plugins` have focal arm64 binaries (verified in pool 2026-08-05) — apt installs work on arm64.
- `gazebo11` for focal arm64 comes from the OSRF gazebo apt repo (`http://packages.osrfoundation.org/gazebo/ubuntu-stable focal main`, key `http://packages.osrfoundation.org/gazebo.key`).
- Long QEMU builds (deb build, PX4 bake) run with `PX4_BUILD_JOBS` capped (`--build-arg PX4_BUILD_JOBS=4`) and generous timeouts; document expected multi-hour durations.
- Full SITL smoke (`jy-smoke-test`) is **deferred to the Jetson** — QEMU runtime timing is unreliable for PX4 lockstep. Host-side verification is boot-level only.
- Every task ends with a commit. Force-add under `docs/superpowers/` if needed (`git add -f` — the dir is gitignored).

---

### Task 0: Create feature branch

**Files:** none (git only)

- [ ] **Step 1: Create branch from current HEAD**

```bash
git checkout -b codex/arm64-image-port
git status   # expect: on branch codex/arm64-image-port, clean tree
```

- [ ] **Step 2: Confirm build prerequisites on host**

```bash
test -d /proc/sys/fs/binfmt_misc && grep -q enabled /proc/sys/fs/binfmt_misc/qemu-aarch64
podman --version        # expect 4.9.3
buildah --version       # expect 1.33.7
```

Expected: all pass. (If qemu-aarch64 binfmt missing, stop and install `qemu-user-static` first.)

---

### Task 1: Restore arch-aware deb builder

**Files:**
- Create: `docker/Dockerfile.debs` (from git `1632adb^`, plus `ARG` visibility fix — see Step 2)
- Create: `docker/build-debs.sh` (verbatim from git `1632adb^`)

**Interfaces:**
- Consumes: nothing (standalone).
- Produces: `docker/Dockerfile.debs` — a buildable image named by the caller, containing `/usr/local/bin/build-jiangyin-debs`; `docker/build-debs.sh` — `build-jiangyin-debs <package_root> <output_dir>`, reads arch from `dpkg --print-architecture`. Task 4's `build-arm64.sh` consumes both.

- [ ] **Step 1: Extract both files from git history**

```bash
git show 1632adb^:docker/build-debs.sh > docker/build-debs.sh
git show 1632adb^:docker/Dockerfile.debs > docker/Dockerfile.debs
git diff --stat HEAD -- docker/build-debs.sh docker/Dockerfile.debs
# expect: two new files (restored)
```

- [ ] **Step 2: Fix ARG visibility in `docker/Dockerfile.debs`**

The restored file already has `ARG ROS_BASE_IMAGE=docker.io/osrf/ros:noetic-desktop-full` before `FROM`. Add a **second** declaration after `FROM` so the value is visible inside the stage (needed if any later line references it):

Replace the top of the file with:

```dockerfile
ARG ROS_BASE_IMAGE=docker.io/osrf/ros:noetic-desktop-full
FROM ${ROS_BASE_IMAGE}

ARG ROS_BASE_IMAGE
```

(Keep everything below `FROM` exactly as restored — source builds, DESTDIR installs, and the final `build-jiangyin-debs /pkg /out/debs` call. Verify the rest of the file is untouched: `diff <(git show 1632adb^:docker/Dockerfile.debs) docker/Dockerfile.debs` should show only the added `ARG ROS_BASE_IMAGE` line.)

- [ ] **Step 3: Syntax-check both files**

```bash
bash -n docker/build-debs.sh && echo "build-debs.sh OK"
grep -q 'ARG ROS_BASE_IMAGE' docker/Dockerfile.debs && echo "Dockerfile.debs ARG OK"
```

- [ ] **Step 4: Commit**

```bash
git add docker/Dockerfile.debs docker/build-debs.sh
git commit -m "build(docker): restore arch-aware deb builder (Dockerfile.debs + build-debs.sh)"
```

---

### Task 2: Pre-flight — arm64 emulation end-to-end sanity

**Files:** temp only (in `/tmp/plat-test`, removed at end)

**Interfaces:**
- Consumes: nothing.
- Produces: a **verified decision** the later tasks rely on: `podman build --platform linux/arm64` + QEMU works on this host, and an explicit `--build-arg TARGETARCH=arm64` is the robust arch plumbing (never depend on buildah auto-populating it).

- [ ] **Step 1: Build a tiny arm64 image through the whole pipeline**

```bash
mkdir -p /tmp/plat-test
cat > /tmp/plat-test/Containerfile <<'EOF'
FROM docker.io/library/ros:noetic-ros-base-focal
ARG TARGETARCH
RUN echo "TARGETARCH=[${TARGETARCH}] uname=$(uname -m)" \
 && uname -m | grep -q aarch64 \
 && test -n "${TARGETARCH}"
EOF
podman build --platform linux/arm64 \
  --build-arg TARGETARCH=arm64 \
  -f /tmp/plat-test/Containerfile -t plat-test /tmp/plat-test
```

- [ ] **Step 2: Run it under QEMU and verify**

```bash
podman run --rm --platform linux/arm64 plat-test
# expect: "TARGETARCH=[arm64] uname=aarch64" (container exited 0)
podman rmi plat-test
rm -rf /tmp/plat-test
```

Expected: exit 0 with the echo line. This proves: arm64 pull + emulated RUN + explicit TARGETARCH plumbing all work. If it fails, stop and report (do not continue to Task 3).

- [ ] **Step 3: Commit (no repo changes — skip commit, nothing to stage)**

`git status` → clean. Note the verified result in the task log.

---

### Task 3: Parameterize the main Dockerfile + fix jy-docker.sh gazebo path

**Files:**
- Modify: `docker/Dockerfile` (L0 base, L0 apt, L2 CMake, L3 deb COPY, L9 symlink + ENV)
- Modify: `docker/scripts/jy-docker.sh:55-56`

**Interfaces:**
- Consumes: `deb/arm64/*.deb` from Task 5 (path referenced via `DEB_DIR` arg); `docker/Dockerfile.debs`/`build-debs.sh` from Task 1.
- Produces: an arch-aware `docker/Dockerfile` where **every default equals today's hardcoded value**; `docker/build-arm64.sh` (Task 4) passes overrides for arm64.

- [ ] **Step 1: Base image (L0) — parameterize `FROM`**

Replace lines 45 (`FROM docker.io/osrf/ros:noetic-desktop-full@sha256:7dbf…`) with:

```dockerfile
ARG ROS_BASE_IMAGE=docker.io/osrf/ros:noetic-desktop-full@sha256:7dbfb9576d8e6d226c31e06129a82aaab8702695f38eca2116918cb9b9308797
FROM ${ROS_BASE_IMAGE} AS dev
```

Default (amd64) = the current pinned image, byte-identical.

- [ ] **Step 2: L0 apt RUN — add arm64 OSRF gazebo repo + declare `TARGETARCH`**

Above the `RUN apt-get update && apt-get install -y …` block (line 53), add:

```dockerfile
ARG TARGETARCH
```

Then wrap the existing apt RUN so arm64 first registers the OSRF gazebo repo (needed for `gazebo11` arm64). Replace `RUN apt-get update && apt-get install -y --no-install-recommends \` with:

```dockerfile
RUN if [ "${TARGETARCH}" = "arm64" ]; then \
        apt-get update \
        && apt-get install -y --no-install-recommends curl ca-certificates gnupg \
        && curl -fsSL http://packages.osrfoundation.org/gazebo.key | gpg --dearmor -o /usr/share/keyrings/gazebo-archive-keyring.gpg \
        && echo "deb [signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable focal main" > /etc/apt/sources.list.d/gazebo-stable.list; \
    fi \
    && apt-get update && apt-get install -y --no-install-recommends \
```

The existing package list stays unchanged (it already includes `gazebo11`, `curl`, `gnupg`). On amd64 (`TARGETARCH` empty) the `if` is a no-op → identical behavior.

- [ ] **Step 3: CMake (L2) — arch-aware URL**

Replace line 120's hardcoded URL. Above the L2 RUN block (before line 119 `RUN curl -fsSL`), add:

```dockerfile
ARG TARGETARCH
ARG CMAKE_TARBALL=cmake-3.28.3-linux-x86_64.tar.gz
```

Then inside the RUN, select the aarch64 tarball when arm64. Replace `"https://github.com/Kitware/CMake/releases/download/v3.28.3/cmake-3.28.3-linux-x86_64.tar.gz"` with:

```dockerfile
RUN if [ "${TARGETARCH}" = "arm64" ]; then CMAKE_TARBALL=cmake-3.28.3-linux-aarch64.tar.gz; fi \
    && curl -fsSL \
      "https://github.com/Kitware/CMake/releases/download/v3.28.3/${CMAKE_TARBALL}" \
      -o /tmp/cmake.tar.gz \
```

(rest of the L2 RUN unchanged).

- [ ] **Step 4: deb COPY (L3) — arch-aware source dir**

Before the L3 block (before line 133 `COPY deb/jiangyin-sophus-dev_*.deb /tmp/`), add:

```dockerfile
ARG TARGETARCH
ARG DEB_DIR=deb
```

Replace the five COPY lines (133-137) — note `${DEB_DIR}` expansion in COPY source paths:

```dockerfile
COPY ${DEB_DIR}/jiangyin-sophus-dev_*.deb /tmp/
COPY ${DEB_DIR}/jiangyin-casadi-dev_*.deb /tmp/
COPY ${DEB_DIR}/jiangyin-livox-sdk2-dev_*.deb /tmp/
COPY ${DEB_DIR}/jiangyin-livox-ros-driver2_*.deb /tmp/
COPY ${DEB_DIR}/jiangyin-ros1-deps_*.deb ${DEB_DIR}/jiangyin-source-deps_*.deb /tmp/
```

Default (`DEB_DIR=deb`) = current behavior. Arm64 build passes `--build-arg DEB_DIR=deb/arm64`.

- [ ] **Step 5: Gazebo plugin libdir (L9 symlink + ENV)**

Above line 246 (the L9 RUN), add:

```dockerfile
ARG TARGETARCH
ARG GZ_LIBDIR=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins
```

Replace the symlink target in the L9 RUN (`/usr/lib/x86_64-linux-gnu/gazebo-11/plugins`) with `${GZ_LIBDIR}`, and replace the ENV lines 256-258:

```dockerfile
ENV GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:${GZ_LIBDIR}
ENV GAZEBO_MODEL_PATH=/home/user/.gazebo/models:/usr/share/gazebo-11/models
ENV GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11/models:/home/user/.gazebo/models
```

Defaults = current values; arm64 passes `--build-arg GZ_LIBDIR=/usr/lib/aarch64-linux-gnu/gazebo-11/plugins`.

- [ ] **Step 6: jy-docker.sh — derive gazebo libdir from `uname -m`**

Replace lines 55-56 in `docker/scripts/jy-docker.sh`:

```bash
# Gazebo plugins and ROS library paths (multiarch dir derived from uname -m:
# x86_64 -> /usr/lib/x86_64-linux-gnu, aarch64 -> /usr/lib/aarch64-linux-gnu)
machine="$(uname -m)"
gazebo_libdir="/usr/lib/${machine}-linux-gnu/gazebo-11/plugins"
export GAZEBO_PLUGIN_PATH="/opt/ros/noetic/lib:${gazebo_libdir}${GAZEBO_PLUGIN_PATH:+:${GAZEBO_PLUGIN_PATH}}"
export LD_LIBRARY_PATH="/opt/ros/noetic/lib:${gazebo_libdir}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
```

(Behavior on amd64 is identical: `uname -m` = `x86_64`.)

- [ ] **Step 7: Static verification**

```bash
bash -n docker/scripts/jy-docker.sh && echo "jy-docker.sh OK"
grep -c 'TARGETARCH' docker/Dockerfile     # expect >= 4 (L0, L2, L3, L9)
grep -n 'linux-aarch64' docker/Dockerfile  # expect 1 (CMake conditional)
grep -n 'DEB_DIR' docker/Dockerfile        # expect 6 (1 ARG + 5 COPY)
grep -n 'GZ_LIBDIR' docker/Dockerfile      # expect 3 (1 ARG + symlink + ENV)
# sanity: default args must reproduce current values
sed -n '/^ARG ROS_BASE_IMAGE=/,+0p' docker/Dockerfile   # shows pinned osrf digest
sed -n '/^ARG GZ_LIBDIR=/p' docker/Dockerfile           # shows x86_64 path
sed -n '/^ARG DEB_DIR=/p' docker/Dockerfile             # shows deb
```

- [ ] **Step 8: Commit**

```bash
git add docker/Dockerfile docker/scripts/jy-docker.sh
git commit -m "build(docker): make Dockerfile + jy-docker.sh arch-aware (amd64 defaults unchanged)"
```

---

### Task 4: Add `docker/build-arm64.sh` orchestration

**Files:**
- Create: `docker/build-arm64.sh` (chmod +x)

**Interfaces:**
- Consumes: `docker/Dockerfile.debs`, `docker/build-debs.sh` (Task 1); arch-aware `docker/Dockerfile` (Task 3).
- Produces: executable stages `debs | image | all` (default `all`); used by Task 5 (`debs`) and Task 6 (`image`).

- [ ] **Step 1: Write the script**

```bash
cat > docker/build-arm64.sh <<'SCRIPT'
#!/usr/bin/env bash
# ============================================================================
# build-arm64.sh — build the Jiangyin2026 arm64 deb packages and image on an
# x86_64 host via podman --platform linux/arm64 + QEMU aarch64 emulation.
#
# WARNING: emulated builds are ~20-100x slower than native. The PX4 SITL bake
# alone can take multiple hours. Cap PX4_BUILD_JOBS on RAM-limited machines:
#   PX4_BUILD_JOBS=4 ./docker/build-arm64.sh
# Run under tmux/screen; layers are cached so rebuilds resume incrementally.
#
# Usage:
#   docker/build-arm64.sh [debs|image|all]   (default: all)
#
# Stages:
#   debs   build arm64 debs via docker/Dockerfile.debs -> deb/arm64/
#   image  build jiangyin_jy2026:arm64 via docker/Dockerfile
#
# Runtime note: full SITL smoke (jy-smoke-test) must run on the Jetson.
# QEMU emulation does not satisfy PX4 lockstep timing for reliable SITL.
# ============================================================================
set -euo pipefail

STAGE="${1:-all}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEBS_IMAGE="jiangyin-debs-builder:arm64"
MAIN_IMAGE="jiangyin_jy2026:arm64"
ROS_BASE="docker.io/library/ros:noetic-ros-base-focal"
BUILD_JOBS="${PX4_BUILD_JOBS:-4}"

build_debs() {
  echo "[arm64] Building deb packages (QEMU emulation, this takes a while)..."
  podman build --platform linux/arm64 \
    --build-arg ROS_BASE_IMAGE="${ROS_BASE}" \
    -f "${ROOT}/docker/Dockerfile.debs" \
    -t "${DEBS_IMAGE}" "${ROOT}"

  echo "[arm64] Extracting debs into deb/arm64/ ..."
  mkdir -p "${ROOT}/deb/arm64"
  cid="$(podman create "${DEBS_IMAGE}" true)"
  podman cp "${cid}:/out/debs/." "${ROOT}/deb/arm64/"
  podman rm "${cid}"
  # arch-independent metapackages: reuse existing _all debs from deb/
  cp -n "${ROOT}"/deb/jiangyin-source-deps_*_all.deb "${ROOT}/deb/arm64/" 2>/dev/null || true
  cp -n "${ROOT}"/deb/jiangyin-ros1-deps_*_all.deb "${ROOT}/deb/arm64/" 2>/dev/null || true

  echo "[arm64] Verifying deb architecture..."
  for f in "${ROOT}"/deb/arm64/jiangyin-*-dev_*.deb; do
    dpkg --info "$f" | grep -q '^ Architecture: arm64$' \
      || { echo "FAIL: $f is not arm64"; exit 1; }
    echo "  OK: $(basename "$f")"
  done
  echo "[arm64] Debs ready in deb/arm64/"
}

build_image() {
  echo "[arm64] Building ${MAIN_IMAGE} (PX4 SITL bake is the long pole)..."
  podman build --platform linux/arm64 \
    --build-arg ROS_BASE_IMAGE="${ROS_BASE}" \
    --build-arg TARGETARCH=arm64 \
    --build-arg DEB_DIR=deb/arm64 \
    --build-arg GZ_LIBDIR=/usr/lib/aarch64-linux-gnu/gazebo-11/plugins \
    --build-arg PX4_BUILD_JOBS="${BUILD_JOBS}" \
    -f "${ROOT}/docker/Dockerfile" \
    -t "${MAIN_IMAGE}" "${ROOT}"

  echo "[arm64] Image built: ${MAIN_IMAGE}"
  echo "[arm64] Next: run boot-level verification, then jy-smoke-test on the Jetson."
}

case "${STAGE}" in
  debs)  build_debs ;;
  image) build_image ;;
  all)   build_debs && build_image ;;
  *)     echo "Usage: $0 [debs|image|all]"; exit 2 ;;
esac
SCRIPT
chmod +x docker/build-arm64.sh
```

- [ ] **Step 2: Syntax-check and smoke the usage path**

```bash
bash -n docker/build-arm64.sh && echo "build-arm64.sh OK"
./docker/build-arm64.sh bogus 2>&1 | tail -1   # expect: Usage line, exit 2
./docker/build-arm64.sh --help 2>&1 | tail -1  # expect: Usage line, exit 2
```

- [ ] **Step 3: Commit**

```bash
git add docker/build-arm64.sh
git commit -m "build(docker): add build-arm64.sh orchestration (QEMU emulated arm64 builds)"
```

---

### Task 5: Build + verify the arm64 debs (long-running)

**Files:** generated `deb/arm64/*.deb` (committed)

**Interfaces:**
- Consumes: `build-arm64.sh` stage `debs` (Task 4).
- Produces: `deb/arm64/jiangyin-casadi-dev_3.7.0-1_arm64.deb`, `deb/arm64/jiangyin-livox-sdk2-dev_0.0.0-1_arm64.deb`, `deb/arm64/jiangyin-livox-ros-driver2_1.0.0-5_arm64.deb`, `deb/arm64/jiangyin-sophus-dev_1.22.10-1_arm64.deb`, + the 2 `_all.deb` copies. Task 6 consumes these via `DEB_DIR=deb/arm64`.

- [ ] **Step 1: Run the deb build (hours under QEMU — use tmux/long timeout)**

```bash
tmux new -d -s arm64-debs 'PX4_BUILD_JOBS=4 ./docker/build-arm64.sh debs > /tmp/arm64-debs.log 2>&1'
tmux attach -t arm64-debs   # watch; or poll the log
tail -f /tmp/arm64-debs.log
```

Expected end state: log prints `[arm64] Debs ready in deb/arm64/` and the arch-verification loop shows `OK:` for all 4 arch-specific debs. If a source build fails under emulation, fix in the builder image (keep Task 1 files authoritative) and re-run.

- [ ] **Step 2: Verify metadata + contents**

```bash
ls -la deb/arm64/                                   # 6 debs: 4 *_arm64.deb + 2 *_all.deb
dpkg --info deb/arm64/jiangyin-casadi-dev_3.7.0-1_arm64.deb | grep -E '^(Package|Version|Architecture|Depends):'
# expect: Architecture: arm64
dpkg -c deb/arm64/jiangyin-livox-ros-driver2_1.0.0-5_arm64.deb | grep -c livox_ros_driver2Config.cmake   # expect >= 1
```

- [ ] **Step 3: Install-verification in an arm64 container**

```bash
podman run --rm --platform linux/arm64 \
  -v "$PWD/deb/arm64:/debs:ro" \
  jiangyin-debs-builder:arm64 bash -lc \
  'dpkg -i /debs/*.deb && dpkg-query -W jiangyin-* && echo DEBS_INSTALL_OK'
```

Expected: `DEBS_INSTALL_OK` + the 6 `jiangyin-*` packages listed installed.

- [ ] **Step 4: Consumer smoke-compile (CasADi + Sophus + livox find_package)**

```bash
podman run --rm --platform linux/arm64 jiangyin-debs-builder:arm64 bash -lc '
source /opt/ros/noetic/setup.bash
cat >/tmp/consumer.cpp <<EOF
#include <casadi/casadi.hpp>
#include <sophus/se3.hpp>
int main(){ casadi::MX x = casadi::MX::sym("x"); Sophus::SE3d T; return (int)T.matrix().rows(); }
EOF
g++ -std=c++17 /tmp/consumer.cpp -o /tmp/consumer -I/usr/local/include -L/usr/local/lib -lcasadi -lcasadi_ipopt -lipopt
LD_LIBRARY_PATH=/usr/local/lib /tmp/consumer && echo CASADI_SOPHUS_OK
mkdir -p /tmp/livox-test && cat >/tmp/livox-test/CMakeLists.txt <<EOC
cmake_minimum_required(VERSION 3.16)
project(livox_test CXX)
find_package(livox_ros_driver2 REQUIRED)
message(STATUS "livox dir: ${livox_ros_driver2_DIR}")
EOC
cmake -S /tmp/livox-test -B /tmp/livox-test/build -DCMAKE_PREFIX_PATH=/opt/ros/noetic >/dev/null && echo LIVOX_FIND_OK
'
```

Expected: `CASADI_SOPHUS_OK` and `LIVOX_FIND_OK` (exit 0).

- [ ] **Step 5: Commit debs**

```bash
git add -f deb/arm64/
git commit -m "build(docker): add arm64 deb packages (CasADi, Livox-SDK2, livox_ros_driver2, Sophus, metapackages)"
```

---

### Task 6: Build + boot-verify the arm64 image (very long-running)

**Files:** none (image artifact `jiangyin_jy2026:arm64`)

**Interfaces:**
- Consumes: `deb/arm64/` (Task 5), arch-aware `docker/Dockerfile` (Task 3).
- Produces: runnable `localhost/jiangyin_jy2026:arm64` image.

- [ ] **Step 1: Run the full image build (multi-hour PX4 bake — tmux)**

```bash
tmux new -d -s arm64-img 'PX4_BUILD_JOBS=4 ./docker/build-arm64.sh image > /tmp/arm64-img.log 2>&1'
tail -f /tmp/arm64-img.log
```

Expected end state: log prints `[arm64] Image built: jiangyin_jy2026:arm64`. The in-Dockerfile asserts (PX4 `bin/px4`, gazebo `.so`, livox cmake config, `*.o` count 0) all pass — the build fails loudly otherwise.

- [ ] **Step 2: Boot-level verification under QEMU**

```bash
podman run --rm --platform linux/arm64 jiangyin_jy2026:arm64 \
  bash -lc 'source /opt/ros/noetic/setup.bash && roscore --version && \
  test -x /opt/PX4-Autopilot/build/px4_sitl_default/bin/px4 && \
  ls /usr/lib/aarch64-linux-gnu/gazebo-11/plugins/*.so >/dev/null && \
  dpkg-query -W jiangyin-livox-ros-driver2 && \
  test -f /opt/ros/noetic/share/livox_ros_driver2/cmake/livox_ros_driver2Config.cmake && \
  echo BOOT_OK'
```

Expected: `BOOT_OK`.

- [ ] **Step 3: Best-effort `sitl` mode launch (may be timing-limited under QEMU)**

```bash
timeout 300 podman run --rm --platform linux/arm64 jiangyin_jy2026:arm64 sitl 2>&1 | tail -20
```

If roscore/MAVROS/px4 all spawn and `/mavros/state` connects → success. If it times out with lockstep/timing errors → **document it, do not treat as failure** (expected under QEMU; full gate on Jetson). Record the observation in the task log.

- [ ] **Step 4: (Optional, recommended) amd64 regression build**

```bash
podman build -f docker/Dockerfile -t jiangyin_jy2026:regression . 2>&1 | tail -5
```

All defaults reproduce the previous amd64 image. If the full bake is too long for a regression run, at minimum confirm the L0-L3 layers build with defaults (first ~30 min) and note the bake is unchanged.

---

### Task 7: Update docs

**Files:**
- Modify: `docker/README.md`
- Modify: `docker/AGENTS.md`

**Interfaces:** consumes final state of Tasks 1-6.

- [ ] **Step 1: Add arm64 section to `docker/README.md`**

Append (after the existing "Notes" section):

````markdown
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
````

- [ ] **Step 2: Add arm64 note to `docker/AGENTS.md`**

In `## BUILD`, add after the existing build commands:

```markdown
# arm64 (Jetson) — QEMU-emulated on x86 host; debs in deb/arm64/
./docker/build-arm64.sh debs
./docker/build-arm64.sh image   # -> jiangyin_jy2026:arm64
```

And in `## NOTES`, add one line: **arm64 image uses `ros:noetic-ros-base-focal` + OSRF gazebo repo; full SITL smoke must run on Jetson (QEMU timing).**

- [ ] **Step 3: Commit**

```bash
git add docker/README.md docker/AGENTS.md
git commit -m "docs(docker): document arm64 build + verification flow"
```

---

### Task 8: Final review pass

**Files:** none

- [ ] **Step 1: Verify the working tree matches the spec**

```bash
git log --oneline codex/arm64-image-port ^codex/ra-lio-pcd-localization
# expect 5 feature commits: Tasks 1, 3, 4, 5, 7 (+ any fixups).
# Task 2 commits nothing (no repo changes); Task 6 commits nothing (image artifact only).
git status   # clean except untracked build logs (if any)
```

- [ ] **Step 2: Re-read spec `docs/superpowers/specs/2026-08-05-arm64-image-port-design.md` and confirm each section has a task**

| Spec section | Plan task |
|---|---|
| 5.1 deb builder restored + parameterized | Task 1, 5 |
| 5.2 main Dockerfile arch-aware | Task 3 |
| 5.3 jy-docker.sh arch-agnostic paths | Task 3 (Step 6) |
| 5.4 build-arm64.sh | Task 4 |
| 6 deb metadata/install/consumer verify | Task 5 |
| 6 image build + boot verify | Task 6 |
| docs | Task 7 |

- [ ] **Step 3: Report completion with the "next steps on Jetson" handoff**

Summarize: debs in `deb/arm64/`, image `jiangyin_jy2026:arm64` built, and the Jetson-side commands (import image, `podman run --network=host jiangyin_jy2026:arm64 sitl`, then `jy-smoke-test`).
