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
BUILD_ONLY=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --dev) DEV_MODE=true ;;
    --build-only) BUILD_ONLY=true ;;
    *) printf '[jy2026 ERROR] Unknown option: %s\n' "$1" >&2; exit 2 ;;
  esac
  shift
done

if [[ "${DEV_MODE}" == "true" ]]; then
  export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
else
  export ROS_MASTER_URI="${ROS_MASTER_URI:-http://sim:11311}"
fi
export ROS_IP="${ROS_IP:-jy-dev}"

source /opt/ros/noetic/setup.bash

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

  if ! dpkg-query -W -f='${Status}' jiangyin-livox-ros-driver2 2>/dev/null \
      | grep -qx 'install ok installed'; then
    log_error "jiangyin-livox-ros-driver2 is not installed in the jy-dev image."
    log_error "Rebuild localhost/jiangyin_core:latest from docker/Dockerfile.core.prebuilt, then rebuild jy-dev."
    exit 1
  fi

  livox_driver_share="/opt/ros/noetic/share/livox_ros_driver2"
  livox_driver_cmake="${livox_driver_share}/cmake"
  if [[ ! -f "${livox_driver_cmake}/livox_ros_driver2Config.cmake" ]]; then
    log_error "Missing Livox CMake package: ${livox_driver_cmake}/livox_ros_driver2Config.cmake"
    exit 1
  fi

  stale_livox_paths=(
    /ws/build/livox_ros_driver2
    /ws/devel/include/livox_ros_driver2
    /ws/devel/lib/livox_ros_driver2
    /ws/devel/lib/pkgconfig/livox_ros_driver2.pc
    /ws/devel/lib/python3/dist-packages/livox_ros_driver2
    /ws/devel/share/common-lisp/ros/livox_ros_driver2
    /ws/devel/share/gennodejs/ros/livox_ros_driver2
    /ws/devel/share/livox_ros_driver2
    /ws/devel/share/roseus/ros/livox_ros_driver2
  )
  for path in "${stale_livox_paths[@]}"; do
    if [[ -e "${path}" ]]; then
      log_info "Removing stale source-built Livox artifact: ${path}"
      rm -rf "${path}"
    fi
  done
  log_info "Using Livox ROS driver deb from ${livox_driver_share}"

  if [[ -f /ws/build/Makefile || -f /ws/build/CMakeCache.txt ]]; then
    cached_cmake=""
    cached_source=""
    if [[ -f /ws/build/Makefile ]]; then
      cached_cmake="$(sed -n 's/^CMAKE_COMMAND = //p' /ws/build/Makefile)"
    fi
    if [[ -z "${cached_cmake}" && -f /ws/build/CMakeCache.txt ]]; then
      cached_cmake="$(sed -n 's/^CMAKE_COMMAND:INTERNAL=//p' /ws/build/CMakeCache.txt)"
    fi
    if [[ -f /ws/build/CMakeCache.txt ]]; then
      cached_source="$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' /ws/build/CMakeCache.txt)"
    fi
    if [[ -n "${cached_cmake}" && ! -x "${cached_cmake}" ]] ||
       [[ -n "${cached_source}" && "${cached_source}" != "/ws/src" ]]; then
      log_info "Removing stale catkin build cache (cmake=${cached_cmake:-unknown}, source=${cached_source:-unknown})"
      rm -rf /ws/build
    fi
  fi

  rm -f /ws/src/ego-planner-v2/CATKIN_IGNORE
  cd /ws

  log_info "catkin_make..."
  catkin_make -j"$(nproc)" \
    -DROS_EDITION=ROS1 \
    -DCATKIN_BLACKLIST_PACKAGES=livox_ros_driver2 2>&1 | tail -20
  source /ws/devel/setup.bash

  log_info "Building RA-LIO..."
  if [[ -f /ws/src/RA-LIO/build/CMakeCache.txt ]]; then
    ralio_cached_source="$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' /ws/src/RA-LIO/build/CMakeCache.txt)"
    if [[ -n "${ralio_cached_source}" && "${ralio_cached_source}" != "/ws/src/RA-LIO" ]]; then
      log_info "Removing stale RA-LIO build cache (source=${ralio_cached_source})"
      rm -rf /ws/src/RA-LIO/build
    fi
  fi
  mkdir -p /ws/src/RA-LIO/build && cd /ws/src/RA-LIO/build
  cmake .. -DCMAKE_BUILD_TYPE=Release \
    -Dlivox_ros_driver2_DIR="${livox_driver_cmake}" 2>&1 | tail -3
  make -j"$(nproc)" 2>&1 | tail -3
  cp -rn devel/bin/* /ws/devel/bin/ 2>/dev/null || true
  cp -rn devel/lib/* /ws/devel/lib/ 2>/dev/null || true
  cp -rn devel/share/* /ws/devel/share/ 2>/dev/null || true

  log_info "Dev build complete."
  if [[ "${BUILD_ONLY}" == "true" ]]; then
    exit 0
  fi
else
  source /ws/devel/setup.bash
fi

if [[ "${DEV_MODE}" == "true" && "${AUTO_START_STACK:-0}" != "1" ]]; then
  log_info "Dev container ready. Algorithm stack is stopped by default."
  log_info "Run jy-start-stack to launch mid360_bridge, fake mocap, RA-LIO, px4_estimator, and FSM+NMPC."
  exec tail -f /dev/null
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
  "roslaunch fsm_ctrl flight_fsm.launch start_mavros:=false" 8

log_info "Pipeline ready: mid360_bridge -> RA-LIO -> px4_estimator -> FSM+NMPC -> PX4 EKF2"

exec tail -f /dev/null
