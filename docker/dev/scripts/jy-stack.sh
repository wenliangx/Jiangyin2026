#!/usr/bin/env bash
set -euo pipefail

command_name="${0##*/}"
if [[ "${command_name}" == "jy-stack" || "${command_name}" == "jy-stack.sh" ]]; then
  command_name="${1:-status}"
  [[ $# -gt 0 ]] && shift
fi

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://sim:11311}"
export ROS_IP="${ROS_IP:-jy-dev}"
source /opt/ros/noetic/setup.bash
if [[ -f /ws/devel/setup.bash ]]; then
  source /ws/devel/setup.bash
fi

STATE_DIR="${JY_STACK_STATE_DIR:-/tmp/jy2026_stack}"
PID_FILE="${STATE_DIR}/pids"
LOG_DIR="${STATE_DIR}/logs"
mkdir -p "${LOG_DIR}"

log_info() { printf '[jy-stack] %s\n' "$*"; }
log_error() { printf '[jy-stack ERROR] %s\n' "$*" >&2; }

usage() {
  cat <<'EOF'
Usage:
  jy-start-stack
  jy-stack start|stop|status
EOF
}

is_running() {
  [[ -s "${PID_FILE}" ]] || return 1
  while IFS=: read -r pid _; do
    [[ -n "${pid}" ]] || continue
    if kill -0 "${pid}" 2>/dev/null; then
      return 0
    fi
  done < "${PID_FILE}"
  return 1
}

launch_service() {
  local name="$1" cmd="$2" delay="${3:-2}"
  local log_file="${LOG_DIR}/${name}.log"
  log_info "Starting ${name}..."
  bash -lc "source /opt/ros/noetic/setup.bash; source /ws/devel/setup.bash 2>/dev/null || true; exec ${cmd}" >"${log_file}" 2>&1 &
  local pid=$!
  printf '%s:%s\n' "${pid}" "${name}" >> "${PID_FILE}"
  sleep "${delay}"
  if kill -0 "${pid}" 2>/dev/null; then
    log_info "${name} running (PID ${pid}, log ${log_file})"
  else
    log_error "${name} failed to start; see ${log_file}"
  fi
}

wait_for_ros_master() {
  log_info "Waiting for ROS master (${ROS_MASTER_URI})..."
  for i in $(seq 1 60); do
    if rostopic list >/dev/null 2>&1; then
      log_info "ROS master ready after ${i}s"
      return 0
    fi
    sleep 1
  done
  log_error "ROS master not reachable after 60s"
  return 1
}

start_stack() {
  if is_running; then
    log_info "Stack already appears to be running. Use jy-stack stop first if needed."
    return 0
  fi

  : > "${PID_FILE}"
  wait_for_ros_master

  launch_service "mid360_bridge" \
    "rosrun mid360_gazebo mid360_bridge.py" 2

  launch_service "imu_relay" \
    "rosrun topic_tools relay /mavros/imu/data /livox/imu" 2

  if [[ "${GZ_MOCAP_ENABLED:-0}" == "1" ]]; then
    launch_service "gz_mocap_bridge" \
      "roslaunch gz_external_pose gazebo_pose_to_vrpn.launch model_name:=${GZ_MOCAP_MODEL:-iris_mid360} output_topic:=/vrpn_client_node/jy0/pose odom_topic:=/ground_truth/state ready_topic:=/gz_mocap/ready zero_origin:=${GZ_MOCAP_ZERO_ORIGIN:-true}" 2
  fi

  launch_service "ra_lio" \
    "roslaunch ra_lio mapping_mid360.launch use_sim_time:=false rviz:=false" 5

  launch_service "px4_estimator" \
    "roslaunch fsm_ctrl px4_estimator.launch" 3

  launch_service "fsm_nmpc" \
    "roslaunch fsm_ctrl flight_fsm.launch start_mavros:=false" 8

  log_info "Stack started. Logs are in ${LOG_DIR}."
}

stop_stack() {
  log_info "Stopping algorithm stack..."

  for node in \
    /mid360_bridge \
    /relay \
    /gazebo_pose_to_vrpn \
    /laserMapping \
    /px4_estimator \
    /flight_fsm; do
    rosnode kill "${node}" >/dev/null 2>&1 || true
  done

  if [[ -f "${PID_FILE}" ]]; then
    while IFS=: read -r pid name; do
      [[ -n "${pid}" ]] || continue
      if kill -0 "${pid}" 2>/dev/null; then
        log_info "Stopping ${name:-process} (PID ${pid})"
        kill "${pid}" 2>/dev/null || true
      fi
    done < "${PID_FILE}"
    sleep 2
    while IFS=: read -r pid _; do
      [[ -n "${pid}" ]] || continue
      if kill -0 "${pid}" 2>/dev/null; then
        kill -9 "${pid}" 2>/dev/null || true
      fi
    done < "${PID_FILE}"
    : > "${PID_FILE}"
  fi

  log_info "Algorithm stack stopped."
}

status_stack() {
  if [[ ! -s "${PID_FILE}" ]]; then
    log_info "No recorded stack processes."
    return 0
  fi

  while IFS=: read -r pid name; do
    [[ -n "${pid}" ]] || continue
    if kill -0 "${pid}" 2>/dev/null; then
      printf '%s running PID %s\n' "${name}" "${pid}"
    else
      printf '%s stopped PID %s\n' "${name}" "${pid}"
    fi
  done < "${PID_FILE}"
}

case "${command_name}" in
  jy-start-stack|start)
    start_stack
    ;;
  jy-stop-stack|stop)
    stop_stack
    ;;
  status)
    status_stack
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    log_error "Unknown command: ${command_name}"
    usage >&2
    exit 2
    ;;
esac
