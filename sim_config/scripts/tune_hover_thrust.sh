#!/usr/bin/env bash
# =============================================================================
# tune_hover_thrust.sh — NMPC 悬停推力自动调参脚本
# =============================================================================
# 依次测试不同 hover_thrust 值，记录飞行表现。
# 每次测试: 重启 pipeline → 起飞 → 悬停 10s → 记录高度误差 → 降落
#
# 用法:
#   ./sim_config/scripts/tune_hover_thrust.sh              # 测试默认值列表
#   ./sim_config/scripts/tune_hover_thrust.sh 0.35 0.40 0.45 0.50  # 自定义列表
#
# 评估指标 (手动观察):
#   1. 起飞后10s内高度变化 < ±0.1m → 悬停推力合适
#   2. 持续下降 → 增加推力
#   3. 持续上升 → 减小推力
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PARAMS_DIR="${WORKSPACE_DIR}/sim_config/params"

log_info()  { printf '[tune] %s\n' "$*"; }
log_error() { printf '[tune ERROR] %s\n' "$*" >&2; }

# 默认测试值 (仿真 iris 典型范围)
if [[ $# -gt 0 ]]; then
    THRUST_VALUES=("$@")
else
    THRUST_VALUES=(0.350 0.375 0.400 0.425 0.450 0.475 0.500)
fi

cleanup() {
    log_info "Cleaning up..."
    # 停止当前 pipeline
    if command -v jy-stack >/dev/null 2>&1; then
        jy-stack stop 2>/dev/null || true
    fi
    # 发送降落命令
    python3 -c "
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.sendto(b'4,0,0,0', ('127.0.0.1', 12001))
" 2>/dev/null || true
    log_info "Done."
}
trap cleanup EXIT INT TERM

log_info "=== NMPC Hover Thrust Tuning ==="
log_info "Testing values: ${THRUST_VALUES[*]}"
log_info ""

for thrust in "${THRUST_VALUES[@]}"; do
    log_info "========================================"
    log_info "Testing hover_thrust = ${thrust}"
    log_info "========================================"

    # 1. 更新参数文件中的 hover_thrust
    PARAM_FILE="${PARAMS_DIR}/nmpc_hover_tune.yaml"
    if [[ -f "${PARAM_FILE}" ]]; then
        # 替换 YAML 中的 hover_thrust 值
        sed -i "s/^nmpc_hover_thrust:.*/nmpc_hover_thrust: ${thrust}/" "${PARAM_FILE}"
        log_info "Updated ${PARAM_FILE} → nmpc_hover_thrust: ${thrust}"
    fi

    # 2. 启动 pipeline
    log_info "Starting pipeline with hover_thrust=${thrust}..."
    "${WORKSPACE_DIR}/sim_config/scripts/start_sim_pipeline.sh" \
        --hover "${thrust}" &

    PIPELINE_PID=$!
    sleep 15  # 等待 pipeline 就绪 + RA-LIO 初始化

    # 3. 发送起飞命令 (cmd=3)
    log_info "Sending takeoff (cmd=3)..."
    python3 -c "
import socket, time
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.sendto(b'3,0.000,0.000,0.000', ('127.0.0.1', 12001))
print('Takeoff sent')
"

    # 4. 悬停 15 秒并记录信息
    log_info "Hovering for 15s..."
    for i in $(seq 1 15); do
        sleep 1
        # 可选: 记录当前高度 (需要 rostopic echo)
        # height=$(rostopic echo /mavros/local_position/pose -n1 2>/dev/null | grep -A1 'position' | tail -1 | awk '{print $3}')
        # log_info "  t=${i}s height=${height}"
    done

    # 5. 发送降落命令 (cmd=4)
    log_info "Sending land (cmd=4)..."
    python3 -c "
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.sendto(b'4,0.000,0.000,0.000', ('127.0.0.1', 12001))
print('Land sent')
"

    sleep 5  # 等待降落完成

    # 6. 停止 pipeline
    kill "${PIPELINE_PID}" 2>/dev/null || true
    sleep 3

    log_info "Test hover_thrust=${thrust} complete."
    log_info ""
done

log_info "=== All tests complete ==="
log_info "Review height data and choose the best hover_thrust value."
log_info "Update nmpc_hover_tune.yaml or single.launch with the chosen value."
