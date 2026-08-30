#!/usr/bin/env bash
# =============================================================================
# start_sim_pipeline.sh — 仿真全流程启动脚本
# =============================================================================
# 启动整套定位、控制、规划 pipeline，连接至 PX4 SITL + Gazebo。
#
# 前置条件:
#   1. PX4 SITL + Gazebo + MAVROS 已在运行 (docker-compose up -d sim)
#   2. 或本地启动了: roscore + mavros + PX4 SITL + Gazebo
#
# 用法:
#   ./sim_config/scripts/start_sim_pipeline.sh                  # 默认参数
#   ./sim_config/scripts/start_sim_pipeline.sh --hover 0.420   # 指定悬停推力
#   ./sim_config/scripts/start_sim_pipeline.sh --no-lio         # 不启动RA-LIO
#   ./sim_config/scripts/start_sim_pipeline.sh --tune-hover     # 悬停推力调参模式
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
CONFIG_DIR="${WORKSPACE_DIR}/sim_config"
LAUNCH_DIR="${CONFIG_DIR}/launch"

HOVER_THRUST=0.400
USE_RA_LIO=true
TUNE_HOVER=false

log_info()  { printf '[sim] %s\n' "$*"; }
log_error() { printf '[sim ERROR] %s\n' "$*" >&2; }

# ---- Parse arguments -------------------------------------------------------
while [[ $# -gt 0 ]]; do
    case "$1" in
        --hover) HOVER_THRUST="$2"; shift 2 ;;
        --no-lio) USE_RA_LIO=false; shift ;;
        --tune-hover) TUNE_HOVER=true; HOVER_THRUST="$2"; shift 2 ;;
        -h|--help)
            sed -n 's/^# //p; s/^#$//p' "$0"
            exit 0 ;;
        *) log_error "Unknown arg: $1"; exit 2 ;;
    esac
done

# ---- Source environment ----------------------------------------------------
source /opt/ros/noetic/setup.bash >/dev/null 2>&1 || {
    log_error "ROS Noetic not found. Is /opt/ros/noetic/setup.bash sourced?"
    exit 1
}

if [[ -f "${WORKSPACE_DIR}/devel/setup.bash" ]]; then
    source "${WORKSPACE_DIR}/devel/setup.bash"
fi

# ---- Check ROS master ------------------------------------------------------
log_info "Waiting for ROS master..."
for i in $(seq 1 30); do
    if rostopic list >/dev/null 2>&1; then
        log_info "ROS master ready after ${i}s"
        break
    fi
    [[ $i -eq 30 ]] && { log_error "ROS master not reachable"; exit 1; }
    sleep 1
done

# ---- Check prerequisite topics (Gazebo + PX4) ------------------------------
log_info "Checking prerequisite topics..."
PREREQ_TOPICS=("/mavros/state" "/gazebo/model_states")
for topic in "${PREREQ_TOPICS[@]}"; do
    for i in $(seq 1 30); do
        if rostopic info "${topic}" >/dev/null 2>&1; then
            break
        fi
        [[ $i -eq 30 ]] && { log_error "Topic ${topic} not found. Is PX4 SITL + Gazebo running?"; exit 1; }
        sleep 1
    done
done
log_info "Prerequisites OK."

# =============================================================================
# Pipeline Launch
# =============================================================================
PIDS=()

cleanup() {
    log_info "Shutting down pipeline..."
    for pid in "${PIDS[@]}"; do
        kill "${pid}" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    log_info "Pipeline stopped."
}
trap cleanup EXIT INT TERM

launch() {
    local name="$1" cmd="$2" delay="${3:-3}"
    log_info "Starting ${name}..."
    bash -c "source /opt/ros/noetic/setup.bash && source ${WORKSPACE_DIR}/devel/setup.bash 2>/dev/null; exec ${cmd}" &
    PIDS+=($!)
    sleep "${delay}"
    if kill -0 "${PIDS[-1]}" 2>/dev/null; then
        log_info "${name} running (PID ${PIDS[-1]})"
    else
        log_error "${name} failed to start"
    fi
}

# Step 1: mid360_bridge (Gazebo PointCloud2 → Livox CustomMsg)
launch "mid360_bridge" \
    "rosrun mid360_sim pointcloud_to_livox" 2

# Step 2: IMU relay (MAVROS IMU → RA-LIO input)
launch "imu_relay" \
    "rosrun topic_tools relay /mavros/imu/data /livox/imu" 2

# Step 3: RA-LIO (LiDAR-inertial odometry)
if [[ "${USE_RA_LIO}" == "true" ]]; then
    launch "ra_lio" \
        "roslaunch ra_lio mapping_mid360.launch use_sim_time:=false rviz:=false" 5
fi

# Step 4: px4_estimator (odometry fusion for PX4 EKF2)
launch "px4_estimator" \
    "roslaunch fsm_ctrl px4_estimator.launch" 3

# Step 5: FSM + NMPC (controller with hover thrust tuning)
launch "fsm_nmpc" \
    "roslaunch ${LAUNCH_DIR}/pipeline_sim.launch \
        hover_thrust:=${HOVER_THRUST} \
        use_ra_lio:=${USE_RA_LIO}" 5

log_info ""
log_info "=== Pipeline ready ==="
log_info "  hover_thrust: ${HOVER_THRUST}"
log_info "  RA-LIO:       ${USE_RA_LIO}"
log_info ""
log_info "Send commands:"
log_info "  jy-takeoff                    # 起飞"
log_info "  rostopic pub /fsm_cmd std_msgs/Int32 3   # 起飞 (cmd=3)"
log_info "  rostopic pub /fsm_cmd std_msgs/Int32 5   # 轨迹跟踪 (cmd=5)"
log_info "  rostopic pub /fsm_cmd std_msgs/Int32 4   # 降落 (cmd=4)"
log_info ""

wait
