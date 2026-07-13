#!/usr/bin/env bash
# ============================================================================
# start_px4_mid360.sh — PX4 SITL + Gazebo 仿真容器入口脚本
#
# 启动顺序:
#   1. roscore          — ROS Master（如果尚未运行）
#   2. MAVROS           — PX4 ↔ ROS 通信桥接 (mavros px4.launch)
#   3. PX4 SITL + Gazebo — 启动 iris_mid360 无人机模型的仿真
#
# 用法:
#   start_px4_mid360.sh            # 无头模式（headless，不渲染画面）
#   start_px4_mid360.sh --gui      # 带 Gazebo GUI 渲染
#
# ROS Master 默认地址: http://localhost:11311（roscore 与本容器同启）
# 可通过环境变量 ROS_MASTER_URI 覆盖
# ============================================================================
set -euo pipefail

HEADLESS=1
while (($#)); do
  case "$1" in
    --gui)  unset HEADLESS; shift ;;                         # 启用Gazebo GUI渲染
    --gui=false) export HEADLESS=1; shift ;;                 # 显式禁用GUI
    -h|--help) printf 'Usage: %s [--gui]\n' "${0##*/}"; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2
       printf 'Usage: %s [--gui]\n' "${0##*/}" >&2; exit 2 ;;
  esac
done

export PX4_HOME="${PX4_HOME:-/opt/PX4-Autopilot}"           # PX4源码路径
export PX4_SIM_MODEL="${PX4_SIM_MODEL:-iris_mid360}"        # 使用的无人机SDF模型名
export PX4_SIM_WORLD="${PX4_SIM_WORLD:-obstacle_test}"      # 默认障碍物测试地图
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export HEADLESS
export QT_X11_NO_MITSHM=1                                    # 避免X11共享内存问题

# Gazebo插件和ROS库路径
export GAZEBO_PLUGIN_PATH="/opt/ros/noetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins${GAZEBO_PLUGIN_PATH:+:${GAZEBO_PLUGIN_PATH}}"
export LD_LIBRARY_PATH="/opt/ros/noetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

# 加载ROS和Gazebo环境
source /opt/ros/noetic/setup.bash
source "${PX4_HOME}/Tools/simulation/gazebo-classic/setup_gazebo.bash" \
  "${PX4_HOME}" "${PX4_HOME}/build/px4_sitl_default"

# ---- 1. 启动 roscore -------------------------------------------------------
if ! pgrep -f "roscore" >/dev/null 2>&1; then
  roscore >/tmp/roscore.log 2>&1 &
fi

# 等待 roscore 就绪（最多等待30秒）
for i in $(seq 1 30); do
  if rostopic list >/dev/null 2>&1; then break; fi
  [[ $i -eq 30 ]] && { echo "roscore did not start within 30s" >&2; exit 1; }
  sleep 1
done

# ---- 2. 启动 MAVROS（PX4 ↔ ROS 通信桥）-------------------------------------
# fcu_url: udp://:14540@127.0.0.1:14557
#   本地14540端口监听 → 转发到PX4 SITL的14557端口
roslaunch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557 >/tmp/mavros.log 2>&1 &
echo "MAVROS started"

# ---- 3. 启动 PX4 SITL + Gazebo --------------------------------------------
# 使用 sitl_run.sh 脚本启动仿真环境
# 参数: <px4_binary> <vehicle_type(none)> <model> <world(none)> <px4_home> <build_dir>
px4_build="${PX4_HOME}/build/px4_sitl_default"
cd "${px4_build}/src/modules/simulation/simulator_mavlink"
exec "${PX4_HOME}/Tools/simulation/gazebo-classic/sitl_run.sh" \
  "${px4_build}/bin/px4" none "${PX4_SIM_MODEL}" "${PX4_SIM_WORLD}" \
  "${PX4_HOME}" "${px4_build}"
