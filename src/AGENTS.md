# src/ — ROS Catkin Workspaces

**Meta:** `catkin_make` workspace root. 11 top-level entries (dirs), ~50 ROS packages total.

## PACKAGE MAP

| Package | Lang | Role |
|---------|------|------|
| `SUPER/` | C++17 | Global + local corridor planning (CIRI) + mission FSM + waypoint execution (6 sub-pkgs) |
| `ego-planner-v2/` | C++11 | Local replanning: path search, traj opt, collision avoidance, state machine (22 sub-pkgs, 12 CATKIN_IGNORE) |
| `fsm_ctrl/` | C++14 | FSM state machine + NMPC guidance + PX4 estimator bridge (4 binaries, 6 libs, 38 GTest) |
| `RA-LIO/` | C++14 | LiDAR-inertial odometry (FAST-LIO2 IEKF). Built standalone outside catkin |
| `uav_vision/` | C++/Python | Vision: AprilTag landing detection (C++) + target classification (Python, multi-template) |
| `uav_vision_msgs/` | — | Message contracts: LandingOffset, TargetMatch, TargetMatchArray |
| `apriltag_ros/` | C++17 | Continuous AprilTag detector (ROS1, aggressive -O3). **CATKIN_IGNORE** |
| `apriltag_echo_message/` | — | Bridge: AprilTag → laser msg. **CATKIN_IGNORE** |
| `libs/px4_plugs/` | Python | PX4 plugins: link_monitor, log_manager, param_migrator (3 Python ROS nodes) |
| `livox_ros_driver/` | — | Legacy driver. No ROS pkg structure |

## NOTES

- **Build order matters:** RA-LIO built separately → `src/RA-LIO/build/` → copy binaries to `devel/lib/` manually. Does not go through catkin_make.
- **SUPER ROS dual-mode:** `src/SUPER/scripts/select_ros_version.sh` toggles between ROS1 and ROS2 builds. Switch affects package deps and msg types across all 6 sub-packages. Never edit template files directly.
- **17+ CATKIN_IGNORE markers** across apriltag_ros, apriltag_echo_message, 12+ in ego-planner-v2 — expect excluded from default builds.
- **Submodules:** `libs/px4_plugs` is forked-branch clone; `libs/livox_ros_driver2` uninitialized (needs `git submodule update --init`).
- **3 planners in pipeline:** RA-LIO (odom) → px4_estimator (fusion) → FSM/NMPC (guidance) → PX4 (actuation). ego-planner and SUPER provide corridor/global inputs to FSM.
- **Python ROS nodes** in px4_plugs (3), uav_vision (3) — all via catkin_install_python.
- **uav_vision** has full runtime guide at `docs/uav_vision_runtime_guide.md` (Chinese).
