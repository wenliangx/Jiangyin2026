# ego-planner-v2 — Local Trajectory Planning Stack

**Legacy planner** — C++11, CMake 2.8.3, Eigen + PCL. Pipeline: RA-LIO odom → grid_map → path_searching → traj_opt (MINCO) → EGOReplanFSM → PX4 via mavros.

## PACKAGE GROUPS

### planner/ (7 pkgs) — Core planning pipeline

| Package | Role |
|---------|------|
| `plan_manage` | `ego_planner_node` + `traj_server`; EGOReplanFSM + EGOPlannerManager; FSM: INIT→WAIT_TARGET→GEN_TRAJ→EXEC_TRAJ→EMERGENCY_STOP |
| `path_searching` | Graph search, road-map path generation, waypoint planning |
| `plan_env` | GridMap collision world; inflation radius too large degrades perf |
| `traj_opt` | MINCO polynomial optimizer; `poly_traj_optimizer.cpp` (1658 LOC), `root_finder.hpp`, `lbfgs.hpp`, `gcopter.hpp`; LBFGS 200 iters, mem=16 |
| `traj_utils` | Message types (PolyTraj, MINCOTraj, Flag), visualization helpers, PlanParameters |
| `drone_detect` | **CATKIN_IGNORE** — ego obstacle detection |
| `swarm_bridge` | `traj2odom_node` — deserializes MINCOTraj from swarm peers, reconstructs MinJerkOpt |

### uav_simulator/ (7 pkgs) — Simulation (all CATKIN_IGNORE)

All 7 disabled: `so3_quadrotor_simulator`, `so3_control`, `local_sensing`, `map_generator`, `mockamap`, `fake_drone`, `flag_pubtest`.

### Utils/ (9 pkgs) — Supporting infrastructure

| Package | Role |
|---------|------|
| `uav_utils` | Core utilities; CMake 2.8.3 with runtime `CheckCXXCompilerFlag` for C++11/0x autodetect; only pkg with gtest coverage |
| `quadrotor_msgs` | Custom msg (GoalSet) |
| `pose_utils` | Transform helpers |
| `odom_visualization` | **CATKIN_IGNORE** — RViz odometry viz |
| `rviz_plugins` | **CATKIN_IGNORE** — custom RViz plugins |
| `assign_goals` / `random_goals` | **CATKIN_IGNORE** ×2 — waypoint assignment |
| `moving_obstacles` | **CATKIN_IGNORE** — simulate moving obstacles |
| `manual_take_over` / `selected_points_publisher` | **CATKIN_IGNORE** ×2 — manual control + point viz |

## CONVENTIONS

- **Language**: C++11/0x (older than rest of project C++14/17)
- **CMake**: `cmake_minimum_required(VERSION 2.8.3)` everywhere; `ADD_COMPILE_OPTIONS(-std=c++11)`
- **Optimization**: `-O3 -Wall -g` Release builds
- **Math**: Eigen with `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` on every Eigen-heavy class; PCL 1.7+
- **Naming**: `namespace ego_planner`, files `snake_case`, classes `CamelCase`
- **Test targets**: commented out in many CMakeLists; not integrated into CI. Only `uav_utils` (7 GTest cases for geometry) has active tests
- **12 CATKIN_IGNORE pkgs** excluded from catkin_make (6 in uav_simulator, 5 in Utils, drone_detect)
- **`test_dynamics.cpp`** is a standalone benchmark, not a proper test (no assertions)

## ANTI-PATTERNS

- **NEVER assign high-precision timestamps to double** (`poly_traj_optimizer.cpp:1472`) — loses precision, breaks trajectory timing and virtual-real time mapping
- **NEVER use Horner's scheme in root_finder.hpp** (`root_finder.hpp:65`) — catastrophic cancellation near polynomial zeros; breaks LBFGS convergence
- **Inflation radius too large** in GridMap degrades collision checking and path search
- **NEVER change LBFGS convergence epsilon for nonsmooth costs** — MINCO cost has discontinuities at collision boundaries
- Multiple pkg CMakeLists have commented-out `catkin_package()` and test targets
- `uav_utils` CMake 2.8.3 with runtime C++ detection — do not upgrade without confirming build servers
- No tests cover `optimizeTrajectory()` or `EGOPlannerManager` — largest blast radius package
- `traj2odom_node` swarm time sync tolerance: warns >0.25s, errors >10s offset

## CALL GRAPH

`ego_planner_node` → `EGOReplanFSM::execFSMCallback()` (timer) → `EGOPlannerManager::reboundReplan()` → `PolyTrajOptimizer::optimizeTrajectory()` → `lbfgs::lbfgs_optimize()`

`reboundReplan()` allows 3 restarts + 20 LBFGS rebounds. Uses `MinJerkOpt` init, `RealT2VirtualT`/`VirtualT2RealT` time mapping for LBFGS feasibility.