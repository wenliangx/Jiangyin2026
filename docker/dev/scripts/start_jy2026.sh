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

  cd /ws
  log_info "catkin_make..."
  catkin_make -j"$(nproc)" 2>&1 | tail -20
  source /ws/devel/setup.bash

  log_info "Building RA-LIO..."
  mkdir -p /ws/src/RA-LIO/build && cd /ws/src/RA-LIO/build
  cmake .. -DCMAKE_BUILD_TYPE=Release \
    -Dlivox_ros_driver_DIR=/ws/devel/share/livox_ros_driver/cmake 2>&1 | tail -3
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

# ---- Launch pipeline -----------------------------------------------------
launch_service "mid360_bridge" \
  "rosrun mid360_gazebo mid360_bridge.py" 2

launch_service "IMU relay" \
  "rosrun topic_tools relay /mavros/imu/data /livox/imu" 2

launch_service "RA-LIO" \
  "roslaunch ra_lio mapping_mid360.launch use_sim_time:=false rviz:=false" 5

launch_service "px4_estimator" \
  "roslaunch fsm_ctrl px4_estimator.launch" 3

launch_service "FSM+NMPC" \
  "roslaunch fsm_ctrl single.launch start_mavros:=false use_external_odom:=true" 8

# ---- Wait for EKF convergence --------------------------------------------
log_info "Waiting for EKF convergence (up to 120s)..."
ekf_ready="false"
for i in $(seq 1 120); do
  if rostopic echo /fsm_ctrl/ekf_ready -n 1 2>/dev/null | grep -q "data: True"; then
    log_info "EKF converged after ${i}s"
    ekf_ready="true"
    break
  fi
  sleep 1
done

if [[ "${ekf_ready}" == "false" ]]; then
  log_error "EKF did not converge within 120s — skipping auto-takeoff"
fi

# ---- Send NMPC hover command ---------------------------------------------
if [[ "${ekf_ready}" == "true" ]]; then
  python3 -c "
import socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(b'2,0,0,1.0,0,0,0,0,0,0', ('127.0.0.1', 12001))
sock.close()
"
  log_info "NMPC hover command sent"
fi

log_info "Pipeline ready: mid360_bridge → RA-LIO → px4_estimator → FSM+NMPC → PX4 EKF2"

exec tail -f /dev/null
