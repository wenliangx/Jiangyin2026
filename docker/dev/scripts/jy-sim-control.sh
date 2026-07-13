#!/usr/bin/env bash
set -euo pipefail

command_name="${0##*/}"
if [[ "${command_name}" == "jy-sim-control" || "${command_name}" == "jy-sim-control.sh" ]]; then
  command_name="${1:-}"
  [[ $# -gt 0 ]] && shift
fi

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://sim:11311}"
export ROS_IP="${ROS_IP:-jy-dev}"
source /opt/ros/noetic/setup.bash

usage() {
  cat <<'EOF'
Usage:
  jy-takeoff
  jy-land
  jy-reset
  jy-sim-control takeoff|land|reset
EOF
}

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
  printf 'Timed out waiting for %s\n' "${service}" >&2
  return 1
}

case "${command_name}" in
  jy-takeoff|takeoff)
    send_fsm_cmd 3 >/dev/null
    printf 'Sent takeoff command to FSM (cmd=3).\n'
    ;;
  jy-land|land)
    send_fsm_cmd 4 >/dev/null
    printf 'Sent land command to FSM (cmd=4).\n'
    ;;
  jy-reset|reset)
    if command -v jy-stack >/dev/null 2>&1; then
      jy-stack stop >/dev/null || true
    elif [[ -x /ws/docker/dev/scripts/jy-stack.sh ]]; then
      /ws/docker/dev/scripts/jy-stack.sh stop >/dev/null || true
    fi
    send_fsm_cmd 0 >/dev/null || true
    if wait_for_service /mavros/cmd/arming 5; then
      rosservice call /mavros/cmd/arming "value: false" >/dev/null || true
    fi
    wait_for_service /gazebo/reset_simulation 20
    rosservice call /gazebo/reset_simulation "{}" >/dev/null
    printf 'Stopped algorithm stack and reset Gazebo simulation. Algorithms remain stopped.\n'
    ;;
  -h|--help|help|"")
    usage
    ;;
  *)
    printf 'Unknown command: %s\n' "${command_name}" >&2
    usage >&2
    exit 2
    ;;
esac
