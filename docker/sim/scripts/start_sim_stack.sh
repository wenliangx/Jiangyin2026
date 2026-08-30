#!/usr/bin/env bash
set -euo pipefail

mode="${1:-all}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_IP="${ROS_IP:-127.0.0.1}"
export PX4_HOME="${PX4_HOME:-/opt/PX4-Autopilot}"
export PX4_SIM_MODEL="${PX4_SIM_MODEL:-iris_mid360}"
export PX4_SIM_WORLD="${PX4_SIM_WORLD:-obstacle_test}"
export HEADLESS="${HEADLESS:-1}"
export NO_PXH="${NO_PXH:-1}"
export QT_X11_NO_MITSHM=1
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}"
export GAZEBO_PLUGIN_PATH="/opt/ros/noetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins${GAZEBO_PLUGIN_PATH:+:${GAZEBO_PLUGIN_PATH}}"
export LD_LIBRARY_PATH="/opt/ros/noetic/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

source /opt/ros/noetic/setup.bash
source /ws/devel/setup.bash
source "${PX4_HOME}/Tools/simulation/gazebo-classic/setup_gazebo.bash" \
  "${PX4_HOME}" "${PX4_HOME}/build/px4_sitl_default"

log_dir="${SIM_LOG_DIR:-/tmp/jiangyin_sim}"
mkdir -p "${log_dir}"
pids=()

log() { printf '[sim-stack] %s\n' "$*"; }

cleanup() {
  local pid
  for pid in "${pids[@]:-}"; do
    kill "${pid}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

launch() {
  local name="$1"
  shift
  log "starting ${name}"
  "$@" >"${log_dir}/${name}.log" 2>&1 &
  pids+=("$!")
}

wait_for() {
  local description="$1" timeout_seconds="$2"
  shift 2
  local attempt
  for attempt in $(seq 1 "${timeout_seconds}"); do
    if "$@" >/dev/null 2>&1; then
      log "${description} ready after ${attempt}s"
      return 0
    fi
    sleep 1
  done
  log "ERROR: timed out waiting for ${description}"
  return 1
}

start_ros() {
  launch roscore roscore
  wait_for "ROS master" 30 rostopic list
}

start_sitl() {
  launch mavros roslaunch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557

  local px4_build="${PX4_HOME}/build/px4_sitl_default"
  launch px4_gazebo bash -lc \
    "cd '${px4_build}/src/modules/simulation/simulator_mavlink' && \
     exec '${PX4_HOME}/Tools/simulation/gazebo-classic/sitl_run.sh' \
       '${px4_build}/bin/px4' none '${PX4_SIM_MODEL}' '${PX4_SIM_WORLD}' \
       '${PX4_HOME}' '${px4_build}'"

  wait_for "Gazebo point cloud" 90 rostopic info /mid360/points
  wait_for "PX4/MAVROS link" 60 bash -lc \
    "timeout 2 rostopic echo -n1 /mavros/state | grep -q 'connected: True'"
}

start_algorithms() {
  launch mid360_bridge rosrun mid360_sim pointcloud_to_livox
  wait_for "Livox simulation cloud" 30 rostopic info /livox/lidar

  launch imu_relay rosrun topic_tools relay /mavros/imu/data /livox/imu
  wait_for "IMU relay" 30 rostopic info /livox/imu

  launch ra_lio roslaunch ra_lio mapping_mid360.launch use_sim_time:=false rviz:=false
  wait_for "RA-LIO odometry" 90 bash -lc "timeout 3 rostopic echo -n1 /Odometry"

  launch px4_estimator roslaunch fsm_ctrl px4_estimator.launch
  launch flight_fsm roslaunch fsm_ctrl flight_fsm.launch start_mavros:=false nmpc_hover_thrust:=0.400
  wait_for "flight FSM" 30 rosnode info /flight_fsm
  sleep 2
  rosnode info /flight_fsm >/dev/null
}

case "${mode}" in
  all)
    start_ros
    start_sitl
    start_algorithms
    log "simulation stack ready; logs: ${log_dir}"
    while kill -0 "${pids[0]}" 2>/dev/null; do sleep 5; done
    ;;
  sitl)
    start_ros
    start_sitl
    log "PX4 SITL ready; logs: ${log_dir}"
    while kill -0 "${pids[0]}" 2>/dev/null; do sleep 5; done
    ;;
  shell)
    trap - EXIT INT TERM
    exec bash
    ;;
  *)
    printf 'Usage: %s [all|sitl|shell]\n' "${0##*/}" >&2
    exit 2
    ;;
esac
