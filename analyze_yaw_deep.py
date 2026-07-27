#!/usr/bin/env python3
"""
深入分析 yaw 控制问题：
1. 参考 yaw 信号的跳变和不连续性
2. 检查每个 cmd[index] 的 yaw 是否有规律
3. 位置跟踪 vs yaw 跟踪对比
4. 查看 NMPC 实际使用的参数
"""

import math
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

try:
    import rosbag
except ImportError:
    print("rosbag not available")
    sys.exit(1)

BAG_PATH = "/home/flag/Jiangyin2026/2026-07-26-22-39-09.bag"

def quat_to_yaw(w, x, y, z):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

print("Reading bag...")
bag = rosbag.Bag(BAG_PATH)

t_super, yaw_cmd0, yaw_cmd1, yaw_cmd4, yaw_cmd8 = [], [], [], [], []
t_pose, yaw_act, pos_act = [], [], []
t_ref, pos_ref = [], []
t_fdb, pos_fdb = [], []

for topic, msg, t in bag.read_messages():
    ts = t.to_sec()
    if topic == "/super/flag_cmd":
        t_super.append(ts)
        sz = len(msg.cmd)
        yaw_cmd0.append(msg.cmd[0].yaw if sz > 0 else math.nan)
        yaw_cmd1.append(msg.cmd[1].yaw if sz > 1 else math.nan)
        yaw_cmd4.append(msg.cmd[4].yaw if sz > 4 else math.nan)
        yaw_cmd8.append(msg.cmd[8].yaw if sz > 8 else math.nan)
    elif topic == "/mavros/local_position/pose":
        t_pose.append(ts)
        q = msg.pose.orientation
        yaw_act.append(quat_to_yaw(q.w, q.x, q.y, q.z))
        pos_act.append((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
    elif topic == "/nmpc_posref":
        t_ref.append(ts)
        pos_ref.append((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
    elif topic == "/nmpc_posfdb":
        t_fdb.append(ts)
        pos_fdb.append((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))

bag.close()

t0 = min(t_super[0], t_pose[0])
t_super = np.array(t_super) - t0
t_pose = np.array(t_pose) - t0
t_ref = np.array(t_ref) - t0
t_fdb = np.array(t_fdb) - t0

yaw_cmd0 = np.array(yaw_cmd0); yaw_cmd1 = np.array(yaw_cmd1)
yaw_cmd4 = np.array(yaw_cmd4); yaw_cmd8 = np.array(yaw_cmd8)
yaw_act_arr = np.array(yaw_act)
pos_act_arr = np.array(pos_act)

# ===== 1. 分析参考 yaw 的跳变 =====
print("=" * 60)
print("Yaw 参考信号跳变分析")
print("=" * 60)

# 计算相邻两帧之间的 yaw 变化（每帧间隔 ~0.02s = 50Hz）
yaw_diff = np.diff(yaw_cmd0)
t_diff = np.diff(t_super)
yaw_rate = yaw_diff / t_diff

# 计算角度差（处理 ±pi 环绕问题）
yaw_diff_wrapped = np.arctan2(np.sin(yaw_diff), np.cos(yaw_diff))
yaw_rate_wrapped = yaw_diff_wrapped / t_diff

# 找到大幅跳变点
jump_threshold = 0.1  # rad per timestep (0.02s => 5 rad/s)
jumps = np.abs(yaw_diff_wrapped) > jump_threshold
print(f"大幅跳变 (> {jump_threshold:.1f} rad/step = {jump_threshold/0.02:.1f} rad/s) 次数: {np.sum(jumps)} / {len(yaw_diff)}")
print(f"跳变比例: {np.sum(jumps)/len(yaw_diff)*100:.1f}%")

if np.sum(jumps) > 0:
    jump_indices = np.where(jumps)[0]
    print(f"前10次跳变:")
    for idx in jump_indices[:10]:
        print(f"  t={t_super[idx]:.2f}s: yaw {yaw_cmd0[idx]:.3f} -> {yaw_cmd0[idx+1]:.3f}, "
              f"diff={math.degrees(yaw_diff_wrapped[idx]):.1f}deg")

# yaw 参考中 NaN 的比例
nan_ratio = np.mean(np.isnan(yaw_cmd0))
print(f"\nNaN 比例: {nan_ratio*100:.1f}%")

# ===== 2. 分析不同 horizon 位置的 yaw =====
print(f"\n{'='*60}")
print(f"Horizon 不同位置的 yaw 对比（取几何平均值附近的时间点）")
print(f"{'='*60}")
mid = len(t_super) // 2
for label, arr in [("cmd[0] (当前)", yaw_cmd0), ("cmd[1]", yaw_cmd1),
                    ("cmd[4]", yaw_cmd4), ("cmd[8] (末端)", yaw_cmd8)]:
    valid = arr[~np.isnan(arr)]
    print(f"  {label:20s}: mean={math.degrees(np.mean(valid)):7.1f}deg, "
          f"std={math.degrees(np.std(valid)):7.1f}deg, "
          f"range=[{math.degrees(np.min(valid)):.1f}, {math.degrees(np.max(valid)):.1f}]")

# ===== 3. 位置跟踪 vs yaw 跟踪对比 =====
print(f"\n{'='*60}")
print(f"位置跟踪性能")
print(f"{'='*60}")
if len(pos_ref) > 0 and len(pos_act_arr) > 0:
    # 对齐位置参考和实际
    pos_ref_arr = np.array(pos_ref)
    pos_errs = []
    for i, tp in enumerate(t_pose):
        idx = np.argmin(np.abs(t_ref - tp))
        if abs(t_ref[idx] - tp) < 0.1:
            err = np.sqrt(np.sum((pos_act_arr[i] - pos_ref_arr[idx])**2))
            pos_errs.append(err)
    if pos_errs:
        pos_errs = np.array(pos_errs)
        print(f"  位置 RMS 误差: {np.sqrt(np.mean(pos_errs**2)):.3f} m")
        print(f"  位置 最大误差: {np.max(pos_errs):.3f} m")
        print(f"  位置 平均误差: {np.mean(pos_errs):.3f} m")

# ===== 4. 检查 yaw 是否在 ±pi 处频繁穿越导致环绕 =====
print(f"\n{'='*60}")
print(f"Yaw 环绕分析")
print(f"{'='*60}")
# 统计 yaw 参考穿越 ±pi 的次数
yaw_unwrapped = np.unwrap(yaw_cmd0[~np.isnan(yaw_cmd0)])
total_wraps = (yaw_unwrapped[-1] - yaw_unwrapped[0]) / (2 * math.pi)
print(f"  参考 yaw 总环绕圈数: {total_wraps:.1f}")
print(f"  即 yaw 参考在 {t_super[-1]-t_super[0]:.1f}s 内转了 {total_wraps:.1f} 圈")

# 看看实际 yaw 是否有跟随趋势
yaw_act_unwrapped = np.unwrap(yaw_act_arr)
print(f"  实际 yaw 总环绕圈数: {(yaw_act_unwrapped[-1] - yaw_act_unwrapped[0])/(2*math.pi):.1f}")

# ===== 5. 实际 yaw 变化率分析 =====
actual_yaw_diff = np.diff(yaw_act_unwrapped)
actual_yaw_rate = actual_yaw_diff / np.diff(t_pose)
print(f"  实际 yaw 最大变化率: {np.max(np.abs(actual_yaw_rate)):.2f} rad/s ({math.degrees(np.max(np.abs(actual_yaw_rate))):.0f} deg/s)")

# ===== 绘图 =====
fig, axes = plt.subplots(3, 2, figsize=(16, 14))

# 图1: cmd[0] yaw 和 cmd[8] yaw 对比（展示 horizon 首尾的 yaw 差异）
ax = axes[0, 0]
ax.plot(t_super, np.degrees(yaw_cmd0), 'b-', alpha=0.7, lw=0.5, label='cmd[0] yaw (当前)')
ax.plot(t_super, np.degrees(yaw_cmd8), 'g-', alpha=0.5, lw=0.5, label='cmd[8] yaw (末端)')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Yaw (deg)')
ax.set_title('Reference Yaw: cmd[0] vs cmd[8] (Horizon End Points)')
ax.legend()
ax.grid(True, alpha=0.3)

# 图2: 解缠绕后的参考 yaw vs 实际 yaw
ax = axes[0, 1]
yaw_ref_unwrap = np.unwrap(yaw_cmd0[~np.isnan(yaw_cmd0)])
t_ref_valid = t_super[~np.isnan(yaw_cmd0)]
ax.plot(t_ref_valid, np.degrees(yaw_ref_unwrap), 'b-', alpha=0.7, lw=0.8, label='ref yaw (unwrapped)')
ax.plot(t_pose, np.degrees(yaw_act_unwrapped), 'r-', alpha=0.7, lw=0.5, label='actual yaw (unwrapped)')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Yaw (deg, unwrapped)')
ax.set_title('Unwrapped Reference vs Actual Yaw')
ax.legend()
ax.grid(True, alpha=0.3)

# 图3: yaw 变化率
ax = axes[1, 0]
ref_rate = np.diff(np.degrees(yaw_cmd0)) / np.diff(t_super)
t_rate = (t_super[1:] + t_super[:-1]) / 2
ax.plot(t_rate, ref_rate, 'b-', alpha=0.5, lw=0.5, label='ref yaw rate')
# 过滤掉环绕导致的大跳变
mask = np.abs(ref_rate) < 500
ax.plot(t_rate[mask], ref_rate[mask], 'b-', alpha=0.8, lw=0.3)
ax.plot((t_pose[1:]+t_pose[:-1])/2, np.degrees(actual_yaw_rate), 'r-', alpha=0.5, lw=0.5, label='actual yaw rate')
ax.set_ylabel('Yaw Rate (deg/s)')
ax.set_xlabel('Time (s)')
ax.set_title('Yaw Rate Comparison')
ax.legend()
ax.grid(True, alpha=0.3)
# 放大的内嵌图：只看 ±200 deg/s 范围
ax.set_ylim(-400, 400)

# 图4: 位置 XY 轨迹
ax = axes[1, 1]
if len(pos_ref) > 0:
    pr = np.array(pos_ref)
    ax.plot(pr[:, 0], pr[:, 1], 'b-', alpha=0.6, lw=1, label='NMPC ref')
ax.plot(pos_act_arr[:, 0], pos_act_arr[:, 1], 'r-', alpha=0.6, lw=0.5, label='actual')
ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
ax.set_title('Position XY Trajectory')
ax.legend(); ax.axis('equal'); ax.grid(True, alpha=0.3)

# 图5: 位置 Z 跟踪
ax = axes[2, 0]
if len(pos_ref) > 0:
    pr = np.array(pos_ref)
    ax.plot(t_ref, pr[:, 2], 'b-', alpha=0.6, lw=1, label='NMPC ref Z')
ax.plot(t_pose, pos_act_arr[:, 2], 'r-', alpha=0.6, lw=0.5, label='actual Z')
ax.set_xlabel('Time (s)'); ax.set_ylabel('Z (m)')
ax.set_title('Position Z Tracking')
ax.legend(); ax.grid(True, alpha=0.3)

# 图6: 实际 yaw vs 实际位置 X（观察是否在某些位置 yaw 偏差大）
ax = axes[2, 1]
sc = ax.scatter(pos_act_arr[:, 0], pos_act_arr[:, 1], c=np.degrees(yaw_act_arr),
                s=1, alpha=0.8, cmap='hsv')
plt.colorbar(sc, ax=ax, label='Actual Yaw (deg)')
# 叠加参考轨迹
if len(pos_ref) > 0:
    pr = np.array(pos_ref)
    ax.plot(pr[:, 0], pr[:, 1], 'k-', alpha=0.3, lw=0.8, label='ref path')
ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
ax.set_title('Actual Yaw Color on XY Trajectory')
ax.legend(); ax.axis('equal'); ax.grid(True, alpha=0.3)

plt.tight_layout()
fig.savefig("/home/flag/Jiangyin2026/yaw_deep_analysis.png", dpi=150)
print(f"\nPlot saved to /home/flag/Jiangyin2026/yaw_deep_analysis.png")

print("\nDone.")
