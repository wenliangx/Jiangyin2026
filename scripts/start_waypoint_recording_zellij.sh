#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd -- "${script_dir}/.." && pwd)"
layout_file="${workspace_dir}/config/zellij/record_super_waypoints.kdl"

if ! command -v zellij >/dev/null 2>&1; then
  echo "错误：未找到 zellij。请先安装 zellij。" >&2
  exit 127
fi

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "错误：未找到 /opt/ros/noetic/setup.bash。该会话需要 ROS Noetic。" >&2
  exit 1
fi

if [[ ! -f "${workspace_dir}/devel/setup.bash" ]]; then
  echo "错误：未找到 ${workspace_dir}/devel/setup.bash，请先编译工作空间。" >&2
  exit 1
fi

export JY_WAYPOINT_OUTPUT="${1:-${workspace_dir}/src/fsm_ctrl/config/mission_super_waypoints.yaml}"
mkdir -p "$(dirname -- "${JY_WAYPOINT_OUTPUT}")"

echo "航点将保存到：${JY_WAYPOINT_OUTPUT}"
cd "${workspace_dir}"
exec zellij --session waypoint-record --layout "${layout_file}"
