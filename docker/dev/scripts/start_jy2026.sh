#!/usr/bin/env bash
# ============================================================================
# start_jy2026.sh — Entrypoint for jy2026 / jy-dev container
#
# Mode:
#   production  — source & binaries are baked in the image, ready to run
#   --dev       — source is mounted at /ws/src/, compile on startup
#
# Pipeline:
#   1. mid360_bridge  (Gazebo PointCloud2 → /livox/lidar)
#   2. IMU relay      (/mavros/imu/data → /livox/imu)
#   3. RA-LIO         (LiDAR SLAM → /Odometry)
#   4. px4_estimator  (/Odometry → /mavros/vision_pose/pose → PX4 EKF2)
#   5. FSM+NMPC       (PX4 control → NMPC hover)
# ============================================================================
set -euo pipefail

DEV_MODE=false
if [[ "${1:-}" == "--dev" ]]; then
  DEV_MODE=true
  shift
fi

source /opt/ros/noetic/setup.bash

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://sim:11311}"
export ROS_IP="${ROS_IP:-jy-dev}"

log_info()  { printf '[jy2026] %s\n' "$*"; }
log_error() { printf '[jy2026 ERROR] %s\n' "$*" >&2; }


prepare_livox_ros_driver2_ros1() {
  local driver_dir="/ws/src/livox_ros_driver2"
  if [[ -f "${driver_dir}/package_ROS1.xml" ]]; then
    cp -f "${driver_dir}/package_ROS1.xml" "${driver_dir}/package.xml"
  fi
}

launch_service() {
  local name="$1" cmd="$2" delay="${3:-2}"
  log_info "Starting ${name}..."
  bash -c "${cmd}" &
  sleep "${delay}"
  if kill -0 $! 2>/dev/null; then
    log_info "${name} running (PID $!)"
  else
    log_error "${name} failed to start"
  fi
}

# ---- Dev mode: compile from mounted source --------------------------------
if [[ "${DEV_MODE}" == "true" ]]; then
  log_info "Dev mode: compiling from /ws/src/..."

  rm -f /ws/src/ego-planner-v2/CATKIN_IGNORE
  prepare_livox_ros_driver2_ros1

  cd /ws
  log_info "catkin_make..."
  catkin_make -j"$(nproc)" -DROS_EDITION=ROS1 2>&1 | tail -20
  source /ws/devel/setup.bash

  log_info "Building RA-LIO..."
  mkdir -p /ws/src/RA-LIO/build && cd /ws/src/RA-LIO/build
  cmake .. -DCMAKE_BUILD_TYPE=Release \
    -Dlivox_ros_driver2_DIR=/ws/devel/share/livox_ros_driver2/cmake 2>&1 | tail -3
  make -j"$(nproc)" 2>&1 | tail -3
  cp -rn devel/bin/* /ws/devel/bin/ 2>/dev/null || true
  cp -rn devel/lib/* /ws/devel/lib/ 2>/dev/null || true
  cp -rn devel/share/* /ws/devel/share/ 2>/dev/null || true

  log_info "Dev build complete."
else
  source /ws/devel/setup.bash
fi

# ---- Wait for ROS master ------------------------------------------------
log_info "Waiting for ROS master (${ROS_MASTER_URI})..."
for i in $(seq 1 60); do
  if rostopic list >/dev/null 2>&1; then
    log_info "ROS master ready after ${i}s"; break
  fi
  [[ $i -eq 60 ]] && { log_error "ROS master not reachable after 60s"; exit 1; }
  sleep 1
done

if [[ "${DEV_MODE}" == "true" && "${AUTO_START_STACK:-0}" != "1" ]]; then
  log_info "Dev container ready. Algorithm stack is stopped by default."
  log_info "Run jy-start-stack to launch mid360_bridge, fake mocap, RA-LIO, px4_estimator, and FSM+NMPC."
  exec tail -f /dev/null
fi

if [[ "${AUTO_START_STACK:-0}" == "1" && -x /usr/local/bin/jy-stack ]]; then
  /usr/local/bin/jy-stack start
  exec tail -f /dev/null
fi

# ---- Launch pipeline (production fallback) --------------------------------
launch_service "mid360_bridge" \
  "rosrun mid360_gazebo mid360_bridge.py" 2

launch_service "IMU relay" \
  "rosrun topic_tools relay /mavros/imu/data /livox/imu" 2

if [[ "${GZ_MOCAP_ENABLED:-0}" == "1" ]]; then
  launch_service "Gazebo mocap bridge" \
    "roslaunch gz_external_pose gazebo_pose_to_vrpn.launch model_name:=${GZ_MOCAP_MODEL:-iris_mid360} output_topic:=/vrpn_client_node/jy0/pose odom_topic:=/ground_truth/state ready_topic:=/gz_mocap/ready zero_origin:=${GZ_MOCAP_ZERO_ORIGIN:-true}" 2
fi

launch_service "RA-LIO" \
  "roslaunch ra_lio mapping_mid360.launch use_sim_time:=false rviz:=false" 5

launch_service "px4_estimator" \
  "roslaunch fsm_ctrl px4_estimator.launch" 3

launch_service "FSM+NMPC" \
  "roslaunch fsm_ctrl single.launch start_mavros:=false use_external_odom:=true" 8

log_info "Pipeline ready: mid360_bridge -> RA-LIO -> px4_estimator -> FSM+NMPC -> PX4 EKF2"

exec tail -f /dev/null
