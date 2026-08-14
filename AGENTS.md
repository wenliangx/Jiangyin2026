# PROJECT KNOWLEDGE BASE

**Generated:** 2026-07-27 19:55
**Commit:** ed6c3f1
**Branch:** codex/flight-fsm-clean

## OVERVIEW

Jiangyin2026 — autonomous UAV competition code. ROS Noetic (Catkin) monorepo with nested sub-workspaces. C++ primary, Python tooling/scripts. Controllers: FSM+NMPC (fsm_ctrl), ego-planner-v2 (local replanning), SUPER (global+local corridor planning). LiDAR-inertial odometry via RA-LIO. Vision landing (AprilTag) and target classification via uav_vision. Docker/Podman-based deployment with PX4 SITL + Gazebo simulation.

- **7 package-level AGENTS.md**: `src/`, `src/fsm_ctrl/`, `src/ego-planner-v2/`, `src/SUPER/`, `src/RA-LIO/`, `src/uav_vision/` (planned).
- **After ed6c3f1**: refactor SML landing flow, add precision landing handoff (uav_vision_msgs integration), fix yaw tracking and orientation transmission for nmpc_ref/nmpc_fdb, remove legacy mission/ego flight tracks.

## STRUCTURE

```
./
├── src/               # Primary catkin workspace (14 packages)
│   ├── SUPER/         # Core planning: super_planner, rog_map, mission_planner, mars_uav_sim
│   ├── ego-planner-v2/ # Local trajectory planning (22 pkgs, 12 CATKIN_IGNORE)
│   ├── fsm_ctrl/      # FSM + NMPC controller (sml + legacy)
│   ├── RA-LIO/        # LiDAR-inertial odometry (standalone build)
│   ├── uav_vision/    # Vision: landing tag detection + target classification (C++/Python)
│   ├── uav_vision_msgs/ # Vision message definitions
│   ├── apriltag_ros/  # AprilTag visual localization (CATKIN_IGNORE)
│   ├── apriltag_echo_message/  # AprilTag→laser echo bridge (CATKIN_IGNORE)
│   ├── gz_external_pose/  # Gazebo→VRPN pose bridge (Python)
│   ├── libs/px4_plugs/  # PX4 plugins: link_monitor, log_manager, param_migrator
│   ├── mid360_gazebo/ # Incomplete MID360 plugin
│   └── livox_ros_driver/  # Legacy driver dir
├── docker/            # Container images: core, dev, prod, sim, debs (5 layered)
├── deb/               # Pre-built .deb packages (CasADi, Sophus, Livox SDK2)
├── vrpn/              # VRPN client ROS integration (external)
├── sim_config/        # Simulation pipeline config+launch
├── docs/              # Runtime guides, plans
├── build/ devel/ logs/       # Catkin artifacts (gitignored)
├── tmux-sim.sh / tmux-real.sh # Simulation (5 panes) / real-robot (9 panes) launchers
├── docker-compose.yml # Container orchestration (sim/gui/jy2026/jy-dev)
└── .opencode/         # AI coding assistant config (Node.js)
```

## WHERE TO LOOK

| Task | Location | Notes |
|------|----------|-------|
| FSM + NMPC controller | `src/fsm_ctrl/` | single_offboard_sml.cpp (959L), single_offboard_fsm.cpp (2829L), px4_estimator.cpp (224L) |
| Local trajectory planning | `src/ego-planner-v2/planner/` | ego_planner_node, path_searching, traj_opt (MINCO) |
| Mission planning (SUPER) | `src/SUPER/super_planner/` | CIRI corridors, A*, trajectory opt |
| LiDAR-inertial odometry | `src/RA-LIO/` | ralio_mapping node, FAST-LIO2-style IEKF |
| Landing vision | `src/uav_vision/` | AprilTag landing + target template matching (C++/Python) |
| Vision message defs | `src/uav_vision_msgs/` | LandingOffset, TargetMatch, TargetMatchArray |
| Simulation (PX4 SITL) | `docker/sim/` | start_px4_mid360.sh, Dockerfile |
| Unit tests | `src/fsm_ctrl/test/` | GTest (36 TEST_F), rostest smoke test |
| Vision tests | `src/uav_vision/test/` | GTest (test_landing_core) + 5 Python tests |
| ROS msg definitions | `src/*/msg/` | 17+ custom .msg packages |
| PX4 plugins | `src/libs/px4_plugs/` | px4_link_monitor, px4_log_manager, px4_param_migrator (Python) |
| Build (container) | `docker/Dockerfile.*` | 5 layered images: core→dev→prod, sim (standalone) |
| Build (local) | `catkin_make` | ROS1 workspace + RA-LIO standalone |
| NMPC hover tuning | `sim_config/params/nmpc_hover_tune.yaml` | Thrust tuning config |

## CODE MAP

Core binaries (ROS nodes) — C++:

| Binary | Package | Source | Role |
|--------|---------|--------|------|
| `single_offboard_fsm` | fsm_ctrl | `src/single_offboard_fsm.cpp` (2829L) | Legacy FSM + NMPC controller |
| `single_offboard_sml` | fsm_ctrl | `src/single_offboard_sml.cpp` (959L) | Boost.SML FSM with precision landing handoff |
| `px4_estimator` | fsm_ctrl | `src/px4_estimator.cpp` (224L) | Odometry fusion for PX4 EKF2 |
| `ralio_mapping` | RA-LIO | `src/laserMapping.cpp` (~1000L) | LiDAR-inertial SLAM (FAST-LIO2) |
| `ego_planner_node` | ego-planner | `plan_manage/src/ego_planner_node.cpp` | Local trajectory planner (EGOReplanFSM) |
| `traj_server` | ego-planner | `plan_manage/src/traj_server.cpp` | Trajectory publishing server |
| `fsm_node` | super_planner | `Apps/fsm_node_ros1.cpp` | SUPER planner FSM |
| `waypoint_mission` | mission_planner | `Apps/ros1_waypoint_mission.cpp` | SUPER waypoint execution |
| `landing_tag_node` | uav_vision | `src/landing_tag_node.cpp` | AprilTag landing target detection |

Python ROS nodes:

| Node | Package | Script | Role |
|------|---------|--------|------|
| `px4_link_monitor` | px4_plugs | `scripts/link_monitor_node.py` | MAVLink link health monitoring |
| `px4_log_manager` | px4_plugs | `scripts/log_manager_node.py` | PX4 flight log download/parse/erase |
| `px4_param_migrator` | px4_plugs | `scripts/param_migrator_node.py` | PX4 parameter export/import |
| `gazebo_pose_to_vrpn` | gz_external_pose | `scripts/gazebo_pose_to_vrpn.py` | Gazebo → VRPN pose bridge |
| `landing_debug_web` | uav_vision | `scripts/landing_debug_web.py` | MJPEG landing debug web UI |
| `target_match_node` | uav_vision | `scripts/target_match_node.py` | Front-camera target classification |

## CONVENTIONS

- **Language**: C++17 (SUPER), C++14 (fsm_ctrl, RA-LIO), C++11 (ego-planner legacy), Python 3 (tooling)
- **Build**: `catkin_make` (not colcon). RA-LIO built standalone, binaries manually copied to devel/
- **Naming** (clang-tidy): Classes `CamelCase`, functions `camelBack`, variables `lower_case`, private `_prefixed`, constants `UPPER_CASE`
- **Nulls**: `NULL` preferred over `nullptr` (clang-tidy `NullMacros`)
- **Optimization**: Aggressive `-O3` with fast-math, loop unrolling in apriltag_ros and SUPER
- **Package-level AGENTS.md**: Deep-dive docs for `src/`, `src/fsm_ctrl/`, `src/ego-planner-v2/`, `src/SUPER/`, `src/RA-LIO/`.
- **Testing**: GTest (36 TEST_F in fsm_ctrl) + rostest smoke test + uav_vision (1 CTest + 5 Python tests). Hand-rolled fake classes.
- **ROS package naming**: `*_msgs` suffix for message packages, `<package format="2">` schema

## ANTI-PATTERNS (THIS PROJECT)

- **NEVER** mix position + velocity + acceleration + yaw/yaw_rate in single `Set_TargetPosition` call — corrupts target
- **NEVER** assign high-precision 64-bit timestamps to `double` — loses precision, breaks trajectory timing
- **NEVER** use Horner's scheme for polynomial evaluation near zeros — catastrophic cancellation
- **NEVER** change `rho = 0.998` forgetting factor in ctrl_math.hpp — breaks thrust estimation LSE fit
- **NEVER** use LBFGS epsilon convergence test on nonsmooth cost functions
- **DO NOT** use `aim_pos` / `aim_vel` for attitude control — position/velocity only
- **DO NOT** edit SUPER template CMakeLists.txt/package.xml under `ros/` — always use `select_ros_version.sh`
- Submodule `src/libs/livox_ros_driver2` is **uninitialized** — must `git submodule update --init`
- `src/mid360_gazebo/` has `drwx------` permissions, no package definition
- **17 CATKIN_IGNORE** packages: ego-planner sim/Utils (12), apriltag_ros, apriltag_echo_message, plane_Det, RA-LIO/build
- 15+ package.xml files have `<license>TODO</license>` — legal/audit risk
- RA-LIO build is standalone — no automatic dependency resolution
- **SML → legacy mismatch**: single_offboard_sml no longer subscribes to `/ego_planner/flag_state`, `/target_pose`, or `/tf_output`. Replaced with `uav_vision_msgs/LandingOffset`. Legacy callbacks removed.

## COMMANDS

```bash
# Local build
catkin_make -j$(nproc) -DROS_EDITION=ROS1                           # Full workspace
catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DROS_EDITION=ROS1 -j$(nproc)  # With compile_commands

# RA-LIO standalone
cd src/RA-LIO && cmake . -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)
cp build/bin/ralio_mapping ../../devel/lib/ra_lio/

# Containers
podman-compose up -d jy-dev                          # Dev container
podman-compose up -d                                 # sim + jy2026 (headless)
podman-compose up -d sim gui                         # sim + Gazebo GUI
docker build -t jiangyin_px4_mid360:latest -f docker/sim/Dockerfile docker/sim/
docker build -t jiangyin_jy2026:latest -f docker/Dockerfile.prod .

# Tests
catkin_make run_tests single_offboard_sml_test       # FSM unit tests (36 GTest)
rostest fsm_ctrl single_offboard_sml_smoke.test      # ROS integration test

# Stack / analysis
jy-start-stack                                       # Start all algorithm nodes
bash src/SUPER/scripts/select_ros_version.sh ROS1    # SUPER ROS1↔ROS2 toggle
python3 analyze_bag.py /path/to/robot.bag            # Bag analysis
```

## NOTES

- **3 planners coexist**: ego-planner-v2 (local replanning), SUPER (corridor-based), NMPC (in fsm_ctrl). Pipeline: RA-LIO (odom) → px4_estimator (fusion) → FSM (guidance) → PX4 (actuation)
- **No CI/CD** — manual build verification via Docker images
- **Test coverage**: Only fsm_ctrl (36 GTest) and uav_vision (1 CTest + 5 Python) have active tests. SUPER, ego-planner, RA-LIO have none.
- **Ubuntu 20.04 + ROS Noetic** is Tier 1; ROS2 is experimental
- **Latest changes**: SML node uses `SegmentedMissionMachine`; precision landing receives `LandingObservation` from `uav_vision_msgs/LandingOffset`; obsolete CoreFlight/FullMission machines and legacy reference tracking were removed.
