# src/ — ROS Catkin Workspaces

**Meta:** `catkin_make` workspace root. 13 top-level entries (dirs), ~50 ROS packages total.

## PACKAGE MAP

| Package | Lang | Role |
|---------|------|------|
| `SUPER/` | C++17 | Global + local corridor planning (CIRI) + mission FSM + waypoint execution (6 sub-pkgs) |
| `ego-planner-v2/` | C++11 | Local replanning: path search, traj opt, collision avoidance, state machine (22 sub-pkgs, 3 dirs) |
| `fsm_ctrl/` | C++14 | FSM state machine + NMPC guidance + PX4 estimator bridge (primary controller, 26 GTest cases) |
| `RA-LIO/` | C++14 | LiDAR-inertial odometry. Built standalone outside catkin — copy binaries to devel/ manually |
| `apriltag_ros/` | C++17 | Continuous AprilTag detector (ROS1 node, aggressive -O3 unrolling) |
| `apriltag_echo_message/` | — | Bridge: AprilTag message → laser_msg for plane_Det |
| `pose_to_odom/` | — | PoseStamped → Odometry converter |
| `plane_Det/` | — | **CATKIN_IGNORE** — disabled. Visual plane detection for nav |
| `gz_external_pose/` | — | Gazebo → ROS pose bridge |
| `libs/px4_plugs/` | C++17 | PX4 plugin submodule: link_monitor, log_manager, param_migrator 3 nodes |
| `mid360_gazebo/` | — | **No package.xml/CMakeLists.txt** — incomplete MID360 Gazebo plugin. `drwx------` permissions |
| `livox_ros_driver/` | — | Legacy driver directory. No ROS pkg structure |

## NOTES

- **Build order matters:** RA-LIO built separately → `src/RA-LIO/build/` → copy binaries to `devel/lib/` manually. Does not go through catkin_make.
- **SUPER ROS dual-mode:** `src/SUPER/scripts/select_ros_version.sh` toggles between ROS1 and ROS2 builds. Switch affects package deps and msg types across all 6 sub-packages.
- **14+ packages carry CATKIN_IGNORE:** expect them excluded from default catkin_make run (plane_Det, mid360_gazebo, livox_ros_driver, and others).
- **Submodules:** `libs/px4_plugs` is forked-branch clone; `libs/livox_ros_driver2` does not exist as tracked submodule.
- **3 planners in pipeline:** RA-LIO (odom) → px4_estimator (fusion) → FSM/NMPC (guidance) → PX4 (actuation). ego-planner and SUPER provide corridor/global inputs to FSM.