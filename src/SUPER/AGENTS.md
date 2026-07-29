# SUPER - Mission Planning System (Mars Lab, HKU)

Dual ROS1/ROS2 codebase selected at CMake configure time via `select_ros_version.sh` template swap (CMakeLists.txt/package.xml templates). DUSE_ROS1 macro controls compile branches.

## PACKAGES

| Package | Source | Role |
|---------|--------|------|
| `super_planner/` | `src/SUPER/super_planner/` | CIRI corridor generation, A* path search, trajectory optimization (yaw/expansion/backup) |
| `rog_map/` | `src/SUPER/rog_map/` | Collision distance map, ESDF, raycasting (AGNES/ROG-Map) |
| `mission_planner/` | `src/SUPER/mission_planner/` | Waypoint mission execution (ROS1/ROS2/flag variants) |
| `mars_uav_sim/` | `src/SUPER/mars_uav_sim/` | Simulation: marsim_render (GL), mars_super_msgs, perfect_drone_sim |
| `scripts/` | `src/SUPER/scripts/` | `select_ros_version.sh` - swaps CMakeLists.xml and package.xml templates |

### Hot files

- `Apps/fsm_node_ros1.cpp` - FSM entry node, launches planning pipeline
- `src/super_core/super_planner.cpp` - core planner orchestration. Line 573 is critical (TODO hot init)
- `src/traj_opt/*.cpp` - trajectory optimization modules (yaw_opt, expansion, backup)
- `src/corridor_generator/` - CIRI-based corridor construction. Line 211 known TODO (clearance bug)
- `rog_map/` - collision distance via raycasting and ESDF propagation

## CONVENTIONS

- C++17, `-O3 -Wall -g -fPIC` with `-Werror=return-type` and `-Werror=unused-variable`
- Dual ROS1/ROS2: `select_ros_version.sh` swaps CMakeLists.txt and package.xml at CMake configure time. No runtime detection.
- `DUSE_ROS1` preprocessor macro guards ROS1-specific code paths in source files

## ANTI-PATTERNS

- ROS2 messaging uses lenient QoS (best_effort, durability_volatile) - messages can be silently dropped
- `corridor_generator.cpp:211` - known clearance bug, TODO in hot path. Do not change without tests.
- `super_planner.cpp:573` - critical initialization area with TODO ("hot init"). High risk of regressions.
- `super_planner.cpp:589` - TODO: "Why cannot directly replan on cmd traj?"
- `super_planner.cpp:1018` - TODO: commented-out early-exit optimization for backup trajectory
- Never edit template CMakeLists.txt or package.xml files under `ros/` manually - always use `select_ros_version.sh`
- ros2 branch files are CMake copy templates, not standalone ROS2 builds

## COMMANDS

```bash
# Toggle target ROS version in src/SUPER/
bash scripts/select_ros_version.sh ROS1    # or ROS2
bash scripts/select_ros_version.sh ROS2

# Build from workspace root (after template swap)
catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DROS_EDITION=ROS1 -j$(nproc)

# Plot command logs (Python)
python3 log/cmd_logs/plotCmdLog.py
python3 log/plot_time.py