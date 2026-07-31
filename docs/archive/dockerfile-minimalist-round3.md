---
title: "Round 3 — Final Defense & Verdicts"
author: Minimalist
TeamRunId: 72ea875e-e8bc-40de-881a-65b65a745db3
---

## VERDICT 1: PRODUCTION STAGE → CONCEDED WITH CONDITIONS

**Pragmatist and Artistry argue:** A prod stage in the merged Dockerfile is necessary for competition-day determinism. Compile-on-start risks -j$(nproc) OOM failure. The working tree has build/=877MB + devel/=859MB as direct evidence of the cost.

**Verdict: CONCEDED.**

The prod stage argument is valid. Here's why I change my position:

1. **The 877MB build/ + 859MB devel/ directories** are exactly the evidence I should have prioritized. They're the working tree contamination that causes the 2.2GB docker context problem and the .dockerignore overhead. A baked prod image solves both: one small deployable /ws/devel that's gitignored on production machines.

2. **Competition-day determinism matters.** `catkin_make -j$(nproc)` on a fresh container is a non-deterministic event. OOM on a RAM-limited VM, disk space exhaustion in tmpfs, missing cached deps from a different apt snapshot — these are non-zero risks at a competition. A baked image (built the night before) is deterministic, auditable, and reproducible.

3. **My original position ("dev only") was too narrow.** The user said "最终只留一个Dockerfile用来做开发" — ONE Dockerfile for development. But a single Dockerfile can have a dev default AND a prod stage without violating "one file." The user never said "ONLY dev."

**Concession terms:**

- The merged Dockerfile keeps a **~15-line prod multi-stage target**:
  ```dockerfile
  # syntax=docker/dockerfile:1
  FROM jiangyin-base:latest AS prod
  WORKDIR /ws
  RUN --mount=type=bind,src=src,dst=/ws/src <<'EOF'
  source /opt/ros/noetic/setup.bash && catkin_make -j$(nproc) -DROS_EDITION=ROS1
  source /ws/devel/setup.bash && cd /ws/src/RA-LIO/build && cmake .. && make -j$(nproc)
  cp -rn devel/bin/* /ws/devel/bin/ && cp -rn devel/lib/* /ws/devel/lib/ && cp -rn devel/share/* /ws/devel/share/
  EOF
  entrypoint: /usr/local/bin/jy-docker.sh
  CMD: stack
  ```
- **Trigger:** `--target prod` during build (`docker build --target prod ...`)
- **Prod default CMD:** `["stack"]` (starts algo stack automatically, no shell)
- **Dev default CMD:** `["shell"]` (interactive dev, compile-on-start)

**But I fight one thing:** The prod stage should be OPTIONAL, guarded by a build flag. Not all users need it. The file has it, the stage exists, but it's activated only via `--target prod` or `-f Dockerfile --target prod`. The default build remains dev.

---

## VERDICT 2: RUNTIME HEALTH-GATES → PARTIAL CONCESSION

**Pragmatist:** runtime health-gate (rostopic list loop) before stack.
**Deep-Researcher:** baked inline grep at build + runtime smoke test as jy-smoke-test gate in sitl mode.

**Verdict: PARTIAL CONCESSION.**

I'll accept:

1. **Inline grep asserts at build time** (my Round 2 concession still stands). After `configure_px4_mid360.py`, verify:
   ```dockerfile
   RUN test -f /opt/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_mid360/model.config \
       && grep -q "1020" /opt/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt \
       && grep -q "libgazebo_ros_api_plugin.so" /opt/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_run.sh
   ```
   These are 3 lines that prevent 40-minute failures downstream.

2. **Runtime smoke test in jy-docker.sh sitl mode** (new acceptance): When running in `sitl` or `all` mode, include a brief health gate:
   ```bash
   # After starting gzserver, wait for /mid360/points to appear or timeout after 15s
   for i in $(seq 1 15); do
     if rostopic list 2>/dev/null | grep -q '/mid360/points'; then
       echo "SITL health check passed after ${i}s"
       break
     fi
     sleep 1
   done
   ```
   This is NOT the 60-second rostopic loop that Pragmatist proposed for the algorithm stack. This is a 15-second focused check on a single topic that proves PX4+Gazebo+MAVROS are actually connected. If it fails, the container exits with a clear error message instead of silently running broken.

**What I still reject:**

- **Health-gate before algo stack startup.** In a single-container mode-router, the stack launches after the user has explicitly chosen `dev` or `stack` mode. If the stack fails, the algorithm node's own error handling catches it. A 60-second rostopic loop is over-engineering for a dev workflow where the user is watching the terminal.

**Modified position:** Build-time greps: YES. Runtime SITL smoke test (15s, focused): YES. Runtime algo-stack health gate (60s loop): NO.

---

## VERDICT 3: BIND-MOUNT OF /opt/PX4-AUTOPilot → CONCEDED AS OPTION

**Deep-Researcher + UltraBrain:** bind-mount of /opt/PX4-Autopilot is optional and additive. Bake by default, override only for PX4-source iteration.

**Verdict: CONCEDED.**

This is a clean concession because the positions align:

1. **Bake by default:** The image includes baked PX4 + SITL binaries. Default CMD mode is `shell` or `dev` — PX4 source is NOT mounted. All SITL tests work out-of-the-box.

2. **Bind-mount override is a documented advanced workflow:** Users who want to iterate on PX4 source can mount their host PX4 tree:
   ```yaml
   # docker-compose.override.yml (documented, not committed)
   volumes:
     - /home/user/PX4-Autopilot:/opt/PX4-Autopilot:ro
   ```
   This overrides the baked content. The user accepts the risk that their host PX4 version may not match the baked SITL binary compatibility.

3. **This doesn't change the default image at all.** The image is identical. The bind-mount is an external compose override — outside the Dockerfile, outside the committed compose file. It's truly additive.

**Condition:** Document this in the Dockerfile as a comment. Don't put it in the default compose.yml.

---

## WINNING ARCHITECTURE IN 5 LINES

```
1 Dockerfile (repo root) — base stage (L0-L7: apt+cmake+debs+PX4+SITL+clangd), prod multi-stage target (bak
e src + catkin_make, only triggered via --target prod)
1 entrypoint script (jy-docker.sh) — modes: shell, sitl, gui, dev, stack, all, takeoff, land, reset
1 compose.yml — single service "jy" with volume mount .:/ws:Z (profiles: default/dev, sim, gui)
prod stage optional (docker build --target prod) — baked binaries for competition/day-0 determinism
Runtime: build-time greps after configure_px4_mid360.py + 15s SITL smoke test in sitl/all modes; algo-stack
health-gate removed (failures caught by node logs); /opt/PX4-Autopilot bind-mount documented as advanced override
```
