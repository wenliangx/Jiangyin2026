# PROJECT KNOWLEDGE BASE

**Generated:** 2026-07-27 19:55
**Commit:** ed6c3f1
**Branch:** codex/flight-fsm-clean

## OVERVIEW

Jiangyin2026 — autonomous UAV competition code. ROS Noetic (Catkin) monorepo with nested sub-workspaces. C++ primary, Python tooling/scripts. Controllers: FSM+NMPC (fsm_ctrl), ego-planner-v2 (local replanning), SUPER (global+local corridor planning). LiDAR-inertial odometry via RA-LIO, visual localization via AprilTag. Docker/Podman-based deployment with PX4 SITL + Gazebo simulation.

## STRUCTURE

```
./
├── src/               # Primary catkin workspace (12 packages)
│   ├── SUPER/         #  Core planning: super_planner, rog_map, mission_planner, mars_uav_sim
│   ├── ego-planner-v2/ #  Local trajectory planning + simulation (22 packages)
│   ├── fsm_ctrl/      #  Finite state machine + NMPC controller
│   ├── RA-LIO/        #  LiDAR-inertial odometry
│   ├── apriltag_ros/  #  AprilTag visual localization
│   └── ...            #  pose_to_odom, plane_Det, libs/px4_plugs, etc.
├── docker/            # Container images: core, dev, prod, sim, debs
├── deb/               # Pre-built .deb packages (CasADi, Sophus, Livox SDK2)
├── build/             # Catkin build artifacts (gitignored)
├── devel/             # Catkin devel space (gitignored)
├── logs/              # Per-package log storage (gitignored)
├── tmux-sim.sh        # Local simulation launcher (tmux)
├── tmux-real.sh       # Local real-robot launcher (tmux)
├── docker-compose.yml # Container orchestration (sim/gui/jy2026/jy-dev)
└── .opencode/         # AI coding assistant config (Node.js)
```

## WHERE TO LOOK

| Task | Location | Notes |
|------|----------|-------|
| FSM + NMPC controller | `src/fsm_ctrl/` | single_offboard_fsm.cpp, px4_estimator.cpp, NMPC_test |
| Local trajectory planning | `src/ego-planner-v2/planner/` | ego_planner_node, path_searching, traj_opt |
| Mission planning (SUPER) | `src/SUPER/super_planner/` | CIRI corridors, A*, trajectory opt |
| LiDAR-inertial odometry | `src/RA-LIO/` | laserMapping.cpp (ralio_mapping node) |
| Visual localization | `src/apriltag_ros/` | Continuous AprilTag detection |
| Simulation (PX4 SITL) | `docker/sim/` | start_px4_mid360.sh, Dockerfile |
| Deployment pipeline | `docker/dev/scripts/start_jy2026.sh` | Start script for full stack |
| Stack lifecycle | `docker/dev/scripts/jy-stack.sh` | start/stop/status for all nodes |
| Unit tests | `src/fsm_ctrl/test/` | GTest (26 TEST_F), rostest smoke test |
| Simulation tests | `src/ego-planner-v2/uav_simulator/` | mockamap, map_generator |
| ROS msg definitions | `src/*/msg/` | 17 custom .msg packages |
| PX4 plugins | `src/libs/px4_plugs/` | px4_link_monitor, px4_log_manager, px4_param_migrator |
| Build (container) | `docker/Dockerfile.*` | 5 layered images |
| Build (local) | `catkin_make` | ROS1 workspace |
| Dev container | `.devcontainer/devcontainer.json` | VS Code remote dev |

## CODE MAP

Core binaries (ROS nodes):

| Binary | Package | Source | Role |
|--------|---------|--------|------|
| `single_offboard_fsm` | fsm_ctrl | `src/single_offboard_fsm.cpp` | FSM + NMPC controller (primary) |
| `single_offboard_sml` | fsm_ctrl | `src/single_offboard_sml.cpp` | FSM (Boost.SML alternative) |
| `px4_estimator` | fsm_ctrl | `src/px4_estimator.cpp` | Odometry fusion for PX4 EKF2 |
| `ralio_mapping` | RA-LIO | `src/laserMapping.cpp` | LiDAR-inertial SLAM |
| `ego_planner_node` | ego-planner | `plan_manage/src/ego_planner_node.cpp` | Local trajectory planner |
| `fsm_node` | super_planner | `Apps/fsm_node_ros1.cpp` | SUPER planner FSM |
| `apriltag_ros_continuous_node` | apriltag_ros | `src/continuous_detector.cpp` | Continuous tag detection |
| `plane_Det` | plane_Det | `src/plane_Det_next.cpp` | Visual plane detection |
| `pose_to_odom_node` | pose_to_odom | `src/pose_to_odom_node.cpp` | Pose → odometry conversion |
| `waypoint_mission` | mission_planner | `Apps/ros1_waypoint_mission.cpp` | SUPER waypoint execution |

## CONVENTIONS

- **Language**: C++17 (SUPER, apriltag_ros), C++14 (fsm_ctrl, RA-LIO), C++11 (ego-planner legacy)
- **Build system**: `catkin_make` (not catkin_tools, not colcon)
- **Naming**: Classes `CamelCase`, functions `camelBack`, variables `lower_case`, private members `_prefixed`, constants `UPPER_CASE`
- **Nulls**: `NULL` preferred over `nullptr` (clang-tidy config)
- **Braces**: Allman style (`.clang-format` in mockamap submodule, Mozilla based)
- **Optimization**: Aggressive `-O3` with fast-math, loop unrolling in apriltag_ros and SUPER
- **ROS messaging**: lenient QoS (best_effort, durability_volatile) used widely in ROS2 SUPER builds
- **Testing**: GTest + unittest + rostest. Hand-rolled fake classes (no gmock)
- **Container**: Docker/Podman with layered images (core → dev → prod, separate sim image)
- **Git**: Submodules for livox_ros_driver2 (uninitialized) and px4_plugs (forked branch)

## ANTI-PATTERNS (THIS PROJECT)

- **NEVER** mix position + velocity + acceleration + yaw/yaw_rate in single `Set_TargetPosition` call — corrupts target
- **NEVER** assign high-precision 64-bit timestamps to `double` — loses precision, breaks trajectory timing
- **NEVER** use Horner's scheme for polynomial evaluation near zeros — catastrophic cancellation
- **NEVER** change `rho = 0.998` forgetting factor in ctrl_math.hpp — breaks thrust estimation LSE fit
- **NEVER** use LBFGS epsilon convergence test on nonsmooth cost functions
- **DO NOT** use `aim_pos` / `aim_vel` for attitude control — position/velocity only
- Submodule `src/libs/livox_ros_driver2` is **uninitialized** — must `git submodule update --init` before build
- `src/mid360_gazebo/` has restrictive `drwx------` permissions and no package definition
- `plane_Det` currently has `CATKIN_IGNORE` — disabled from build
- 14+ packages carry `CATKIN_IGNORE` markers — expect them excluded from default builds

## COMMANDS

```bash
# Local build
catkin_make -j$(nproc) -DROS_EDITION=ROS1                           # Full workspace
catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DROS_EDITION=ROS1 -j$(nproc)  # With compile_commands

# Dev container
podman-compose up -d jy-dev                          # Start dev container
# Inside dev container: catkin_make runs automatically on start

# Production
podman-compose up -d                                 # sim + jy2026 (headless)
podman-compose up -d sim gui                         # sim + Gazebo GUI

# Simulation
docker build -t jiangyin_px4_mid360:latest -f docker/sim/Dockerfile docker/sim/
docker build -t jiangyin_jy2026:latest -f docker/Dockerfile.prod .

# Tests (run inside dev container after catkin_make)
catkin_make run_tests single_offboard_sml_test       # FSM unit tests
rostest fsm_ctrl single_offboard_sml_smoke.test      # ROS integration test

# SUPER ROS version switch
bash src/SUPER/scripts/select_ros_version.sh ROS1    # or ROS2

# Stack management (inside running container)
jy-start-stack                                       # Start all algorithm pipeline nodes
jy-stack status                                      # Check node status
jy-takeoff / jy-land / jy-reset                      # Runtime commands

# Local tmux sessions
bash tmux-sim.sh                                     # Simulation (4 panes)
bash tmux-real.sh                                    # Real robot (8 panes)
```

## NOTES

- **3 planners coexist**: ego-planner-v2 (local replanning), SUPER (corridor-based), NMPC (in fsm_ctrl). Pipeline: RA-LIO (odom) → px4_estimator (fusion) → FSM (guidance) → PX4 (actuation)
- **No CI/CD** — all build verification is manual via Docker images
- **No coverage configuration** — gcov/lcov not set up
- `.opencode/package.json` is AI tooling only, not project code
- RA-LIO is built standalone (not via catkin) in `src/RA-LIO/build/` — binaries copied into devel/ manually
