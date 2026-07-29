# RA-LIO — LiDAR-Inertial Odometry

**Based on:** FAST-LIO2 | **Manifold:** SO(3) IEKF (Iterated Error-State Kalman Filter) | **State:** 24D

RA-LIO is the first perception stage in the Jiangyin2026 pipeline:
`RA-LIO (odom) → px4_estimator (fusion) → FSM/NMPC (control) → PX4 (actuation)`.

## KEY FILES

| File | Role |
|------|------|
| `src/laserMapping.cpp` | Main node entry (241 Hz loop). Synchronizes LiDAR+IMU, manages local map, runs IEKF, publishes odometry/path/point cloud |
| `src/preprocess.cpp` | Point cloud preprocessing for 5 radar types: Livox AVIA/MID360, Velodyne VLP-16, Ouster OS1-64, RS-32, Vanjee-16. Blind range filter, feature classification |
| `include/esekfom.hpp` | IEKF on SO(3): `predict()` (IMU propagation), `update_iterated_dyn_share_modified()` (point-to-plane residual with global observation buffers) |
| `include/use-ikfom.hpp` | 24D state `state_ikfom` (pos3, rot3, lidar2imu_off6, vel3, bg3, ba3, grav3), process model `get_f()`, noise cov Q |
| `include/IMU_Processing.hpp` | `ImuProcess` class: IMU preintegration for point cloud undistortion, gravity alignment over first ~20 frames, bias estimation |
| `include/ikd-Tree/ikd_Tree.h` | Incremental dynamic KD-tree for local map management. Dynamic FOV sliding, node add/delete/rebuild |
| `config/mid360.yaml` | Noise covariances, extrinsic T/R, blind range, detection range, feature flags |

## BUILD

RA-LIO is built **standalone**, NOT via `catkin_make`.

```bash
cd src/RA-LIO
mkdir -p build && cd build
cmake .. && make -j$(nproc)
# Copy binary + msg to catkin devel space
cp ralio_mapping ~/Jiangyin2026/devel/lib/ra_lio/
cp msg/Pose6D.msg ~/Jiangyin2026/devel/share/ra_lio/msg/
```

**Dependencies:** Eigen3, PCL 1.8+, Sophus (/usr/local), livox_ros_driver2, rosfmt (/opt/ros/noetic/lib/librosfmt9.so)
**Flags:** C++14, -O3, OpenMP (auto-detected CPU cores, max 3 threads)

## ROS TOPICS

| Type | Topic | Message |
|------|-------|---------|
| Sub | `/livox/lidar` | `livox_ros_driver2/CustomMsg` or `sensor_msgs/PointCloud2` |
| Sub | `/livox/imu` | `sensor_msgs/Imu` |
| Pub | `/Odometry` | `nav_msgs/Odometry` (primary output) |
| Pub | `/path` | `nav_msgs/Path` |
| Pub | `/cloud_registered`, `/cloud_registered_body` | `sensor_msgs/PointCloud2` |
| Pub | `/Laser_map` | `sensor_msgs/PointCloud2` (ikd-Tree local map) |
| Pub | `/speed_vector` | `visualization_msgs/Marker` (RViz arrow) |
| Pub | TF `world → body` | transform (IMU frame pose) |

## NOTES

- **Standalone build:** Binary lives in `RA-LIO/build/` after `cmake+make`. Must be manually copied to `devel/lib/` — it is NOT produced by catkin.
- **Coordinate chain:** `LiDAR frame → extrinsic_R/T → IMU frame (body) → ESKF pose → world frame`. All state is IMU-oriented.
- **Initialization:** First ~20 frames align gravity and estimate biases. No odometry output until init completes.
- **Thread model:** Main loop at ~241 Hz (LiDAR rate) manages map + ESKF. OpenMP parallelism (2-3 cores) for point-to-plane feature matching.
- **Publish flags:** All output topics (path, cloud, speed_vector, body-frame cloud) controlled independently in `mid360.yaml` under `publish:` section. Disable to save bandwidth in deployment.
- **`build/` directory:** RA-LIO stores its own CMake artifacts in `src/RA-LIO/build/` — separate from the catkin `build/` at repo root. Do not confuse them.