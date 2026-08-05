#!/usr/bin/env bash
# ============================================================================
# jy-docker.sh — single entrypoint mode router for the Jiangyin2026 stack.
#
# Consolidates four legacy entrypoint scripts into one:
#   docker/dev/scripts/start_jy2026.sh       -> dev mode
#   docker/dev/scripts/jy-stack.sh           -> stack mode
#   docker/dev/scripts/jy-sim-control.sh     -> takeoff / land / reset modes
#
# Usage:
#   jy-docker.sh <mode> [args]
#   jy-docker.sh -h|--help
#
# Single-container model: ROS master defaults to localhost, no container
# hostnames are used anywhere. External overrides respected via ${VAR:-}.
# ============================================================================
set -euo pipefail

# ---- help (handled before sourcing ROS so -h works on any host) -----------
usage() {
  cat <<'EOF'
Usage: jy-docker.sh <mode> [args]

Modes:
  shell     Interactive dev shell (default when no mode given)
  sitl      PX4 SITL + Gazebo, headless (roscore -> MAVROS -> px4+gzserver)
  dev       Compile-from-source dev shell (livox deb check, catkin_make, RA-LIO)
  stack     Algorithm stack lifecycle: start (default) | stop | status
  all       SITL in background + algorithm stack in foreground
  takeoff   Send FSM takeoff command (UDP 12001 cmd=3)
  land      Send FSM land command (UDP 12001 cmd=4)
  reset     Stop algorithm stack + reset Gazebo simulation
  smoke     Run the smoke test (/usr/local/bin/jy-smoke-test)

Options:
  -h, --help   Show this help and exit
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || "${1:-}" == "help" ]]; then
  usage
  exit 0
fi

# ---- Global environment (all modes) ---------------------------------------
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_IP="${ROS_IP:-127.0.0.1}"
export PX4_HOME="${PX4_HOME:-/opt/PX4-Autopilot}"           # PX4 source path
export PX4_SIM_MODEL="${PX4_SIM_MODEL:-iris}"               # drone SDF model (stock PX4)
export PX4_SIM_WORLD="${PX4_SIM_WORLD:-empty}"              # gazebo world (stock)
export HEADLESS="${HEADLESS:-1}"
export QT_X11_NO_MITSHM=1                                    # avoid X11 shm issues

# Gazebo plugins and ROS library paths (multiarch dir derived from uname -m:
# x86_64 -> /usr/lib/x86_64-linux-gnu, aarch64 -> /usr/lib/aarch64-linux-gnu)
machine="$(uname -m)"
gazebo_libdir="/usr/lib/${machine}-linux-gnu/gazebo-11/plugins"
export GAZEBO_PLUGIN_PATH="/opt/ros/noetic/lib:${gazebo_libdir}${GAZEBO_PLUGIN_PATH:+:${GAZEBO_PLUGIN_PATH}}"
export LD_LIBRARY_PATH="/opt/ros/noetic/lib:${gazebo_libdir}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

source /opt/ros/noetic/setup.bash

log_info()  { printf '[jy] %s\n' "$*"; }
log_error() { printf '[jy ERROR] %s\n' "$*"; }

start_sitl() {
  # Load ROS and Gazebo environments
  source "${PX4_HOME}/Tools/simulation/gazebo-classic/setup_gazebo.bash" \
    "${PX4_HOME}" "${PX4_HOME}/build/px4_sitl_default"

  # ---- 1. Start roscore ----------------------------------------------------
  if ! pgrep -f "roscore" >/dev/null 2>&1; then
    roscore >/dev/null 2>&1 &
  fi

  # Wait for roscore to be ready (up to 30 seconds)
  for i in $(seq 1 30); do
    if rostopic list >/dev/null 2>&1; then break; fi
    [[ $i -eq 30 ]] && { printf 'roscore did not start within 30s\n'; exit 1; }
    sleep 1
  done

  # ---- 2. Start MAVROS (PX4 <-> ROS bridge) --------------------------------
  # fcu_url: udp://:14540@127.0.0.1:14557
  #   listen on local 14540 -> forward to PX4 SITL 14557
  roslaunch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557 >/dev/null 2>&1 &
  echo "MAVROS started"

  # ---- 3. Start PX4 SITL + Gazebo ------------------------------------------
  # sitl_run.sh args: <px4_binary> <vehicle_type(none)> <model> <world(none)> <px4_home> <build_dir>
  local px4_build="${PX4_HOME}/build/px4_sitl_default"
  cd "${px4_build}/src/modules/simulation/simulator_mavlink"
  exec "${PX4_HOME}/Tools/simulation/gazebo-classic/sitl_run.sh" \
    "${px4_build}/bin/px4" none "${PX4_SIM_MODEL}" "${PX4_SIM_WORLD}" \
    "${PX4_HOME}" "${px4_build}"
}

# 15s SITL health gate: /mavros/state must become connected (PX4 link up)
wait_sitl_ready() {
  for i in $(seq 1 15); do
    if timeout 2 rostopic echo -n1 /mavros/state 2>/dev/null | grep -q 'connected: True'; then
      printf 'SITL health check passed after %ss\n' "${i}"
      return 0
    fi
    sleep 1
  done
  printf '[jy ERROR] SITL failed to connect MAVROS within 15s. Check: gzserver running, bin/px4 alive, MAVLink port 14557 listening.\n'
  exit 1
}

# ============================================================================
# stack mode body — docker/dev/scripts/jy-stack.sh
# ============================================================================
STATE_DIR="${JY_STACK_STATE_DIR:-/tmp/jy2026_stack}"
PID_FILE="${STATE_DIR}/pids"
LOG_DIR="${STATE_DIR}/logs"
mkdir -p "${LOG_DIR}"

stack_usage() {
  cat <<'EOF'
Usage:
  jy-docker.sh stack start|stop|status
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
    log_info "Stack already appears to be running. Use jy-docker.sh stack stop first if needed."
    return 0
  fi

  : > "${PID_FILE}"
  wait_for_ros_master

  launch_service "imu_relay" \
    "rosrun topic_tools relay /mavros/imu/data /livox/imu" 2

  launch_service "ra_lio" \
    "roslaunch ra_lio mapping_mid360.launch use_sim_time:=false rviz:=false" 5

  launch_service "px4_estimator" \
    "roslaunch fsm_ctrl px4_estimator.launch" 3

  launch_service "fsm_nmpc" \
    "roslaunch fsm_ctrl single.launch start_mavros:=false use_external_odom:=true" 8

  log_info "Stack started. Logs are in ${LOG_DIR}."
}

stop_stack() {
  log_info "Stopping algorithm stack..."

  for node in \
    /relay \
    /laserMapping \
    /px4_estimator \
    /single_offboard_fsm; do
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

stack_main() {
  local sub="${1:-start}"
  [[ $# -gt 0 ]] && shift
  case "${sub}" in
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
      stack_usage
      ;;
    *)
      log_error "Unknown command: ${sub}"
      stack_usage
      exit 2
      ;;
  esac
}

# ============================================================================
# takeoff / land / reset mode body — docker/dev/scripts/jy-sim-control.sh
# ============================================================================
send_fsm_cmd() {
  local cmd="$1"
  python3 - "$cmd" <<'INNER_PY'
import socket
import sys

cmd = int(sys.argv[1])
payload = f"{cmd},0.000,0.000,0.000".encode()
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(payload, ("127.0.0.1", 12001))
sock.close()
print(payload.decode())
INNER_PY
}

wait_for_service() {
  local service="$1"
  local timeout="${2:-20}"
  for _ in $(seq 1 "${timeout}"); do
    if rosservice list 2>/dev/null | grep -qx "${service}"; then
      return 0
    fi
    sleep 1
  done
  printf 'Timed out waiting for %s\n' "${service}"
  return 1
}

# ============================================================================
# dev mode body — docker/dev/scripts/start_jy2026.sh (--dev)
# livox deb check -> stale-cache purge -> catkin_make -> RA-LIO -> bash
# ============================================================================
dev_main() {
  log_info "Dev mode: compiling from /ws/src/..."

  if ! dpkg-query -W -f='${Status}' jiangyin-livox-ros-driver2 2>/dev/null \
      | grep -qx 'install ok installed'; then
    log_error "jiangyin-livox-ros-driver2 is not installed in the image."
    log_error "Rebuild the core image from docker/Dockerfile.core.prebuilt, then rebuild the dev image."
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
    cached_generator=""
    if [[ -f /ws/build/CMakeCache.txt ]]; then
      cached_generator="$(sed -n 's/^CMAKE_GENERATOR:INTERNAL=//p' /ws/build/CMakeCache.txt)"
    fi
    if [[ -n "${cached_generator}" && "${cached_generator}" != "Ninja" ]]; then
      log_info "Removing stale catkin build cache (generator ${cached_generator})"
      rm -rf /ws/build
    fi
  fi

  if [[ -f /ws/build/Makefile || -f /ws/build/CMakeCache.txt ]]; then
    cached_cmake=""
    if [[ -f /ws/build/Makefile ]]; then
      cached_cmake="$(sed -n 's/^CMAKE_COMMAND = //p' /ws/build/Makefile)"
    fi
    if [[ -z "${cached_cmake}" && -f /ws/build/CMakeCache.txt ]]; then
      cached_cmake="$(sed -n 's/^CMAKE_COMMAND:INTERNAL=//p' /ws/build/CMakeCache.txt)"
    fi
    if [[ -n "${cached_cmake}" && ! -x "${cached_cmake}" ]]; then
      log_info "Removing stale catkin build cache (missing ${cached_cmake})"
      rm -rf /ws/build
    fi
  fi

  rm -f /ws/src/ego-planner-v2/CATKIN_IGNORE
  cd /ws

  log_info "catkin_make..."
  catkin_make --use-ninja -j"$(nproc)" \
    -DROS_EDITION=ROS1 \
    -DCATKIN_BLACKLIST_PACKAGES=livox_ros_driver2 2>&1 | tail -20
  source /ws/devel/setup.bash

  log_info "Building RA-LIO..."
  mkdir -p /ws/src/RA-LIO/build && cd /ws/src/RA-LIO/build
  cmake .. -DCMAKE_BUILD_TYPE=Release \
    -Dlivox_ros_driver2_DIR="${livox_driver_cmake}" 2>&1 | tail -3
  make -j"$(nproc)" 2>&1 | tail -3
  cp -rn devel/bin/* /ws/devel/bin/ 2>/dev/null || true
  cp -rn devel/lib/* /ws/devel/lib/ 2>/dev/null || true
  cp -rn devel/share/* /ws/devel/share/ 2>/dev/null || true

  log_info "Dev build complete."
  source /ws/devel/setup.bash
  exec bash
}

# ============================================================================
# mode router
# ============================================================================
mode="${1:-shell}"
[[ $# -gt 0 ]] && shift

case "${mode}" in
  -h|--help|help)
    usage
    ;;
  shell)
    exec bash
    ;;
  sitl)
    start_sitl &
    wait_sitl_ready
    wait
    ;;
  dev)
    dev_main
    ;;
  stack)
    stack_main "$@"
    ;;
  all)
    start_sitl &
    wait_sitl_ready
    start_stack
    wait
    ;;
  takeoff)
    send_fsm_cmd 3 >/dev/null
    printf 'Sent takeoff command to FSM (cmd=3).\n'
    ;;
  land)
    send_fsm_cmd 4 >/dev/null
    printf 'Sent land command to FSM (cmd=4).\n'
    ;;
  reset)
    stop_stack
    send_fsm_cmd 0 >/dev/null || true
    if wait_for_service /mavros/cmd/arming 5; then
      rosservice call /mavros/cmd/arming "value: false" >/dev/null || true
    fi
    wait_for_service /gazebo/reset_simulation 20
    rosservice call /gazebo/reset_simulation "{}" >/dev/null
    printf 'Stopped algorithm stack and reset Gazebo simulation. Algorithms remain stopped.\n'
    ;;
  smoke)
    if [[ -x /usr/local/bin/jy-smoke-test ]]; then
      exec /usr/local/bin/jy-smoke-test
    else
      printf '[jy ERROR] /usr/local/bin/jy-smoke-test not found: the smoke test script is not installed yet. Build and install it into the image first.\n'
      exit 1
    fi
    ;;
  *)
    printf '[jy ERROR] Unknown mode: %s\n' "${mode}"
    usage
    exit 2
    ;;
esac
