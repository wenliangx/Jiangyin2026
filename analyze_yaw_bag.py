#!/usr/bin/env python3
"""
分析 2026-07-26-22-36-00.bag 中 yaw 控制性能。

从 bag 中提取：
  - /super/flag_cmd   → 参考 yaw（super planner 输出）
  - /mavros/local_position/pose → 实际 yaw
  - /nmpc_posref / nmpc_posfdb → NMPC 位置参考/反馈

输出：
  1. yaw_ref_vs_actual.png  — 参考 yaw vs 实际 yaw 时序图
  2. yaw_error.png           — yaw 跟踪误差
  3. yaw_error_hist.png      — yaw 误差分布直方图
  4. 终端统计摘要
"""

import math
import sys
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

try:
    import rosbag
except ImportError:
    print("rosbag not available, trying bagpy or manual parse...")
    sys.exit(1)

BAG_PATH = "/home/flag/Jiangyin2026/2026-07-26-22-39-09.bag"

# --- 工具函数 ---

def quat_to_yaw(w, x, y, z):
    """四元数转 yaw (rad)，范围 [-pi, pi]"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

def unwrap_yaw(rad):
    """解缠绕，保持连续性"""
    return np.unwrap(np.array(rad))

# --- 读取数据 ---

print("Reading bag...")
bag = rosbag.Bag(BAG_PATH)

# 各时间序列
t_super = []      # super/flag_cmd 时间戳
yaw_refs = []     # 每个 cmd 点对应的参考 yaw（取 horizon 第一个点的 yaw）

t_pose = []       # pose 时间戳
yaw_actual = []   # 实际 yaw
pos_actual = []   # 实际位置

t_ref = []        # nmpc 位置参考
pos_ref = []

t_fdb = []        # nmpc 位置反馈
pos_fdb = []

print("Extracting messages...")

for topic, msg, t in bag.read_messages():
    ts = t.to_sec()

    if topic == "/super/flag_cmd":
        t_super.append(ts)
        # 取 horizon 第一个点（当前参考点）的 yaw
        if len(msg.cmd) > 0:
            yaw_refs.append(msg.cmd[0].yaw)
        else:
            yaw_refs.append(math.nan)

    elif topic == "/mavros/local_position/pose":
        t_pose.append(ts)
        q = msg.pose.orientation
        yaw = quat_to_yaw(q.w, q.x, q.y, q.z)
        yaw_actual.append(yaw)
        pos_actual.append((msg.pose.position.x,
                           msg.pose.position.y,
                           msg.pose.position.z))

    elif topic == "/nmpc_posref":
        t_ref.append(ts)
        pos_ref.append((msg.pose.position.x,
                        msg.pose.position.y,
                        msg.pose.position.z))

    elif topic == "/nmpc_posfdb":
        t_fdb.append(ts)
        pos_fdb.append((msg.pose.position.x,
                        msg.pose.position.y,
                        msg.pose.position.z))

bag.close()
print(f"super/flag_cmd: {len(t_super)} msgs")
print(f"pose:            {len(t_pose)} msgs")
print(f"nmpc_posref:     {len(t_ref)} msgs")
print(f"nmpc_posfdb:     {len(t_fdb)} msgs")

# 归一化时间（相对起点秒数）
t0 = min(t_super[0] if t_super else float("inf"),
         t_pose[0] if t_pose else float("inf"))
t_super = np.array(t_super) - t0
t_pose = np.array(t_pose) - t0
t_ref = np.array(t_ref) - t0
t_fdb = np.array(t_fdb) - t0

yaw_refs = np.array(yaw_refs, dtype=np.float64)
yaw_actual = np.array(yaw_actual, dtype=np.float64)
pos_actual = np.array(pos_actual)
pos_ref = np.array(pos_ref)
pos_fdb = np.array(pos_fdb)

# --- 对齐 yaw 参考和实际 yaw（最近邻插值）---

print("\nAligning yaw reference to pose timestamps...")
yaw_ref_aligned = np.full_like(yaw_actual, np.nan)
for i, tp in enumerate(t_pose):
    # 找最近的 super 消息
    idx = np.argmin(np.abs(t_super - tp))
    if abs(t_super[idx] - tp) < 0.1:  # 100ms 以内
        yaw_ref_aligned[i] = yaw_refs[idx]

# 去掉未对齐的点
valid = ~np.isnan(yaw_ref_aligned)
t_valid = t_pose[valid]
yaw_ref_valid = yaw_ref_aligned[valid]
yaw_actual_valid = yaw_actual[valid]

# yaw 误差 = 实际 - 参考（需要做角度规范化到 [-pi, pi]）
yaw_error_raw = yaw_actual_valid - yaw_ref_valid
yaw_error = np.arctan2(np.sin(yaw_error_raw), np.cos(yaw_error_raw))

print(f"Aligned points: {len(t_valid)} / {len(t_pose)}")

# --- 统计 ---

rmse = np.sqrt(np.mean(yaw_error**2))
mae = np.mean(np.abs(yaw_error))
max_err = np.max(np.abs(yaw_error))
std_err = np.std(yaw_error)

print(f"\n{'='*60}")
print(f"Yaw 跟踪性能统计")
print(f"{'='*60}")
print(f"RMS 误差:          {math.degrees(rmse):.2f} deg  ({rmse:.4f} rad)")
print(f"平均绝对误差:      {math.degrees(mae):.2f} deg  ({mae:.4f} rad)")
print(f"最大绝对误差:      {math.degrees(max_err):.2f} deg  ({max_err:.4f} rad)")
print(f"误差标准差:        {math.degrees(std_err):.2f} deg  ({std_err:.4f} rad)")

# 分析 yaw 参考的变化率
yaw_ref_rate = np.diff(yaw_refs) / np.diff(t_super)
print(f"\n参考 yaw 最大变化率: {np.max(np.abs(yaw_ref_rate)):.4f} rad/s")
print(f"参考 yaw 平均变化率: {np.mean(np.abs(yaw_ref_rate)):.4f} rad/s")

# --- 分析：检查 yaw 参考是否几乎为零（即修改前的行为） ---
near_zero_ref = np.abs(yaw_refs) < 0.001
print(f"\n参考 yaw 接近 0 (<0.001 rad) 的比例: {np.mean(near_zero_ref)*100:.1f}%")

# 检查参考 yaw 的范围
print(f"参考 yaw min: {np.min(yaw_refs):.4f} rad ({math.degrees(np.min(yaw_refs)):.1f} deg)")
print(f"参考 yaw max: {np.max(yaw_refs):.4f} rad ({math.degrees(np.max(yaw_refs)):.1f} deg)")
print(f"参考 yaw mean: {np.mean(yaw_refs):.4f} rad ({math.degrees(np.mean(yaw_refs)):.1f} deg)")

# --- 分段分析 ---
# 将数据分为 5 个等长时间段
n_segments = 5
segment_len = (t_valid[-1] - t_valid[0]) / n_segments
print(f"\n{'='*60}")
print(f"分段 yaw 误差分析 ({n_segments} 段)")
print(f"{'='*60}")
for i in range(n_segments):
    seg_start = t_valid[0] + i * segment_len
    seg_end = seg_start + segment_len
    seg_mask = (t_valid >= seg_start) & (t_valid < seg_end)
    if np.sum(seg_mask) > 0:
        seg_err = yaw_error[seg_mask]
        seg_rmse = math.degrees(np.sqrt(np.mean(seg_err**2)))
        seg_yaw_ref_mean = math.degrees(np.mean(yaw_ref_valid[seg_mask]))
        print(f"  [{seg_start:5.0f}s - {seg_end:5.0f}s]: "
              f"RMS={seg_rmse:.1f}deg, mean_ref_yaw={seg_yaw_ref_mean:.1f}deg, "
              f"samples={np.sum(seg_mask)}")

# --- 绘图 ---

fig, axes = plt.subplots(3, 2, figsize=(16, 14))

# 1. 参考 yaw vs 实际 yaw（全时域）
ax = axes[0, 0]
ax.plot(t_super, np.degrees(yaw_refs), 'b-', alpha=0.6, linewidth=0.5, label='ref yaw (super)')
ax.plot(t_pose, np.degrees(yaw_actual), 'r-', alpha=0.6, linewidth=0.5, label='actual yaw')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Yaw (deg)')
ax.set_title('Reference vs Actual Yaw (Full Timeline)')
ax.legend(loc='best')
ax.grid(True, alpha=0.3)

# 2. yaw 误差随时间变化
ax = axes[0, 1]
ax.plot(t_valid, np.degrees(yaw_error), 'k-', alpha=0.5, linewidth=0.3)
ax.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Yaw Error (deg)')
ax.set_title('Yaw Tracking Error Over Time')
ax.grid(True, alpha=0.3)

# 3. yaw 误差分布直方图
ax = axes[1, 0]
ax.hist(np.degrees(yaw_error), bins=60, color='steelblue', edgecolor='white', alpha=0.8)
ax.axvline(x=0, color='red', linestyle='--', alpha=0.5)
ax.set_xlabel('Yaw Error (deg)')
ax.set_ylabel('Count')
ax.set_title('Yaw Error Distribution')
ax.grid(True, alpha=0.3, axis='y')

# 4. 参考 yaw 变化率 vs 实际 yaw 变化率
ax = axes[1, 1]
ref_rate = np.diff(np.degrees(yaw_refs)) / np.diff(t_super)
actual_rate = np.diff(np.degrees(unwrap_yaw(yaw_actual))) / np.diff(t_pose)
# 对齐时间轴
t_rate_ref = (t_super[1:] + t_super[:-1]) / 2
t_rate_act = (t_pose[1:] + t_pose[:-1]) / 2
ax.plot(t_rate_ref, ref_rate, 'b-', alpha=0.5, linewidth=0.5, label='ref yaw rate')
ax.plot(t_rate_act, actual_rate, 'r-', alpha=0.5, linewidth=0.5, label='actual yaw rate')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Yaw Rate (deg/s)')
ax.set_title('Reference vs Actual Yaw Rate')
ax.legend(loc='best')
ax.grid(True, alpha=0.3)

# 5. 位置跟踪 - XY 轨迹 vs reference
ax = axes[2, 0]
pos_ref_arr = np.array(pos_ref)
pos_fdb_arr = np.array(pos_fdb)
pos_actual_arr = np.array(pos_actual)
if len(pos_ref_arr) > 0:
    ax.plot(pos_ref_arr[:, 0], pos_ref_arr[:, 1], 'b-', alpha=0.5, linewidth=0.5, label='ref')
if len(pos_actual_arr) > 0:
    ax.plot(pos_actual_arr[:, 0], pos_actual_arr[:, 1], 'r-', alpha=0.5, linewidth=0.5, label='actual')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_title('Position XY Trajectory')
ax.legend(loc='best')
ax.axis('equal')
ax.grid(True, alpha=0.3)

# 6. yaw 误差 vs 飞行器速度（判断是否在机动时误差变大）
ax = axes[2, 1]
if len(pos_actual_arr) > 1:
    vel = np.sqrt(np.sum(np.diff(pos_actual_arr, axis=0)**2, axis=1)) / np.diff(t_pose)
    t_vel = (t_pose[1:] + t_pose[:-1]) / 2
    # 对齐 yaw 误差到速度时间
    yaw_err_vel = np.full_like(vel, np.nan)
    for i, tv in enumerate(t_vel):
        idx_err = np.argmin(np.abs(t_valid - tv))
        if abs(t_valid[idx_err] - tv) < 0.1:
            yaw_err_vel[i] = yaw_error[idx_err]
    valid_vel = ~np.isnan(yaw_err_vel)
    scatter = ax.scatter(vel[valid_vel], np.degrees(yaw_err_vel[valid_vel]),
                         c=t_vel[valid_vel], s=1, alpha=0.6, cmap='viridis')
    plt.colorbar(scatter, ax=ax, label='Time (s)')
    ax.set_xlabel('Speed (m/s)')
    ax.set_ylabel('Yaw Error (deg)')
    ax.set_title('Yaw Error vs Speed')
    ax.axhline(y=0, color='gray', linestyle='--', alpha=0.3)
    ax.grid(True, alpha=0.3)

plt.tight_layout()
fig.savefig("/home/flag/Jiangyin2026/yaw_analysis.png", dpi=150)
print(f"\nPlot saved to /home/flag/Jiangyin2026/yaw_analysis.png")

# --- yaw 参考的自相关分析 ---
# 检查 yaw 参考是否有振荡或高频成分
print(f"\n{'='*60}")
print(f"Yaw 参考信号频域特征")
print(f"{'='*60}")
if len(yaw_refs) > 100:
    # 简单的频谱分析（yaw_refs 是 50Hz 采样的）
    dt = np.median(np.diff(t_super))
    fs = 1.0 / dt
    n = len(yaw_refs)
    yaw_ref_fft = np.abs(np.fft.rfft(yaw_refs - np.mean(yaw_refs)))
    freqs = np.fft.rfftfreq(n, dt)
    # 找前 5 个主导频率
    top_indices = np.argsort(yaw_ref_fft)[-10:]
    top_indices = top_indices[np.argsort(freqs[top_indices])]
    for idx in top_indices:
        if freqs[idx] < fs / 2:
            print(f"  {freqs[idx]:.3f} Hz: amplitude={yaw_ref_fft[idx]:.4f}")

    # 参考 yaw 的峰峰值
    print(f"\n参考 yaw 峰峰值: {math.degrees(np.ptp(yaw_refs)):.2f} deg")

# --- NMPC yaw 权重分析 ---
print(f"\n{'='*60}")
print(f"NMPC yaw 权重参数（需要对照参数文件）")
print(f"{'='*60}")
print("""
yaw 控制效果不好，可能涉及以下 NMPC 调参方向：

1. **姿态权重 (Q_quat)**: 在 RosNmpcPort 构造函数中加载
   - nmpc_Qquatx, nmpc_Qquaty, nmpc_Qquatz
   - 对应 q_attitude << q_quat_x, q_quat_y, q_quat_z
   - 这三个值控制 NMPC 对姿态（包括 yaw）跟踪的惩罚权重
   - **如果 q_quat_z 太小**，NMPC 不关心 yaw 误差

2. **角速度权重 (R_w)**:
   - nmpc_Rwx, nmpc_Rwy, nmpc_Rwz
   - 对应 r_angular << r_w_x, r_w_y, r_w_z
   - R_wz 控制偏航角速度的代价
   - **如果 R_wz 太大**，控制器不敢用偏航角速度来纠正 yaw

3. **NMPC 控制器内部的 yaw 约束**:
   - NmpcController 构造参数中有 yaw 范围 [-3.14, 3.14]
   - 检查是否有内部饱和

4. **参考轨迹的 yaw 平滑性**:
   - 如果 super planner 给的 yaw 轨迹不够平滑
   - 或者 yaw 变化率超过飞行器能力

关键是找到 Q_quat_z 和 R_wz 的平衡点
""")

print("Done.")
