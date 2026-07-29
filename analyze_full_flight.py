#!/usr/bin/env python3
"""
全面分析飞行性能：位置 (X/Y/Z) + Yaw 跟踪。
针对 Q_quatz=19 调高后，位置跟踪是否退化的诊断。
"""

import math, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

try:
    import rosbag
except ImportError:
    print("rosbag not available")
    sys.exit(1)

BAG_PATH = "/home/flag/Jiangyin2026/2026-07-27-14-53-07.bag"

def quat_to_yaw(w, x, y, z):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

def angle_diff(a, b):
    return np.arctan2(np.sin(a - b), np.cos(a - b))

print("Reading bag...")
bag = rosbag.Bag(BAG_PATH)

t_super, yaw_refs, pos_refs_super = [], [], []
t_pose, yaw_act, pos_act = [], [], []
t_ref, pos_ref = [], []
t_fdb, pos_fdb = [], []

for topic, msg, t in bag.read_messages():
    ts = t.to_sec()
    if topic == "/super/flag_cmd":
        t_super.append(ts)
        if len(msg.cmd) > 0:
            c = msg.cmd[0]
            yaw_refs.append(c.yaw)
            pos_refs_super.append((c.position.x, c.position.y, c.position.z))
        else:
            yaw_refs.append(math.nan)
            pos_refs_super.append((math.nan, math.nan, math.nan))
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

yaw_refs = np.array(yaw_refs, dtype=np.float64)
yaw_act_arr = np.array(yaw_act, dtype=np.float64)
pos_refs_super_arr = np.array(pos_refs_super)
pos_act_arr = np.array(pos_act)
pos_ref_arr = np.array(pos_ref)
pos_fdb_arr = np.array(pos_fdb)

# ===== 对齐 yaw 参考 =====
yaw_ref_aligned = np.full_like(yaw_act_arr, np.nan)
for i, tp in enumerate(t_pose):
    idx = np.argmin(np.abs(t_super - tp))
    if abs(t_super[idx] - tp) < 0.1:
        yaw_ref_aligned[i] = yaw_refs[idx]
valid_yaw = ~np.isnan(yaw_ref_aligned)
t_yaw_valid = t_pose[valid_yaw]
yaw_ref_v = yaw_ref_aligned[valid_yaw]
yaw_act_v = yaw_act_arr[valid_yaw]
yaw_err = angle_diff(yaw_act_v, yaw_ref_v)

# ===== 对齐位置参考 =====
pos_ref_x_aligned = np.full_like(t_pose, np.nan)
pos_ref_y_aligned = np.full_like(t_pose, np.nan)
pos_ref_z_aligned = np.full_like(t_pose, np.nan)
for i, tp in enumerate(t_pose):
    idx = np.argmin(np.abs(t_super - tp))
    if abs(t_super[idx] - tp) < 0.1:
        pos_ref_x_aligned[i] = pos_refs_super_arr[idx, 0]
        pos_ref_y_aligned[i] = pos_refs_super_arr[idx, 1]
        pos_ref_z_aligned[i] = pos_refs_super_arr[idx, 2]
valid_pxy = ~np.isnan(pos_ref_x_aligned)

# ===== 统计 =====
print("=" * 60)
print("综合跟踪性能对比")
print("=" * 60)

def stats(name, err, unit, is_angle=False):
    rmse = np.sqrt(np.mean(err**2))
    mae = np.mean(np.abs(err))
    mx = np.max(np.abs(err))
    std = np.std(err)
    if is_angle:
        print(f"  {name:12s}: RMS={rmse*180/math.pi:6.1f}deg  MAE={mae*180/math.pi:6.1f}deg  "
              f"MAX={mx*180/math.pi:6.1f}deg  STD={std*180/math.pi:6.1f}deg")
    else:
        print(f"  {name:12s}: RMS={rmse:6.3f}{unit}  MAE={mae:6.3f}{unit}  "
              f"MAX={mx:6.3f}{unit}  STD={std:6.3f}{unit}")

stats("Yaw",       yaw_err, "rad", is_angle=True)
stats("X Pos",     pos_act_arr[valid_pxy,0] - pos_ref_x_aligned[valid_pxy], "m")
stats("Y Pos",     pos_act_arr[valid_pxy,1] - pos_ref_y_aligned[valid_pxy], "m")
stats("Z Pos",     pos_act_arr[valid_pxy,2] - pos_ref_z_aligned[valid_pxy], "m")

# ===== 时间段分析 =====
print(f"\n{'='*60}")
print(f"分段性能 (2秒窗口)")
print(f"{'='*60}")
print(f"{'Time':>8s}  {'Yaw RMS':>8s}  {'X RMS':>8s}  {'Y RMS':>8s}  {'Z RMS':>8s}")

window = 2.0
t_start = max(t_yaw_valid[0], t_pose[0])
t_end = min(t_yaw_valid[-1], t_pose[-1])
t_w = t_start
while t_w + window <= t_end:
    mask_y = (t_yaw_valid >= t_w) & (t_yaw_valid < t_w + window)
    y_rms = math.degrees(np.sqrt(np.mean(yaw_err[mask_y]**2))) if np.sum(mask_y) > 10 else float('nan')
    mask_p = (t_pose >= t_w) & (t_pose < t_w + window) & valid_pxy
    x_rms = np.sqrt(np.mean((pos_act_arr[mask_p,0] - pos_ref_x_aligned[mask_p])**2)) if np.sum(mask_p) > 10 else float('nan')
    y_rms = np.sqrt(np.mean((pos_act_arr[mask_p,1] - pos_ref_y_aligned[mask_p])**2)) if np.sum(mask_p) > 10 else float('nan')
    z_rms = np.sqrt(np.mean((pos_act_arr[mask_p,2] - pos_ref_z_aligned[mask_p])**2)) if np.sum(mask_p) > 10 else float('nan')

    print(f"{t_w:7.1f}s  {y_rms:7.1f}deg  {x_rms:7.3f}m  {y_rms:7.3f}m  {z_rms:7.3f}m")
    t_w += window

# ===== 参考信号特征 =====
print(f"\n参考 yaw 信号统计:")
yaw_clean = yaw_refs[~np.isnan(yaw_refs)]
t_clean = t_super[~np.isnan(yaw_refs)]
yaw_rate_ref = np.diff(np.unwrap(yaw_clean)) / np.diff(t_clean)
print(f"  参考 yaw 平均变化率: {np.mean(np.abs(yaw_rate_ref)):.2f} rad/s")
print(f"  参考 yaw 最大变化率: {np.max(np.abs(yaw_rate_ref)):.2f} rad/s")
print(f"  参考 yaw 范围: {math.degrees(np.min(yaw_clean)):.1f} ~ {math.degrees(np.max(yaw_clean)):.1f} deg")

# ===== X/Y 位置误差与速度的相关性 =====
if len(pos_act_arr) > 1:
    vel = np.sqrt(np.sum(np.diff(pos_act_arr, axis=0)**2, axis=1)) / np.diff(t_pose)
    t_vel = (t_pose[1:] + t_pose[:-1]) / 2
    # 找机动最大的时段
    top_vel_idx = np.argsort(vel)[-int(len(vel)*0.1):]  # top 10% speed
    print(f"\n最高速度10%时段: 速度 > {vel[top_vel_idx[0]]:.2f} m/s")
    print(f"  位置 RMS (高速度段): 见上（如果高速度段误差显著大于低速段，说明需要增大 Q_vel 或 R_w）")

# ===== NMPC 参数分析 =====
print(f"\n{'='*60}")
print(f"调参指南")
print(f"{'='*60}")

Q_quatz = 19.0
Q_pos_x, Q_pos_y = 30.0, 50.0

print(f"""
当前权重比: Q_quatz/Q_posx = {Q_quatz/Q_pos_x:.2f}, Q_quatz/Q_posy = {Q_quatz/Q_pos_y:.2f}

如果 X/Y 位置跟踪变差，有 3 种调整思路:

  A) 微降 Q_quatz (19→12~15): 
     折中方案，yaw 仍显著优于旧值(1.0)，但释放一些优化空间给位置

  B) 同步提升 Q_pos (30→50, 50→80):
     保持 yaw 权重不变，同时增强位置跟踪，但可能降低整体鲁棒性

  C) 增大 R_wz (0.3→0.5~0.8):
     抑制 yaw 角速度，减少剧烈 yaw 机动对位置的耦合干扰
     （yaw 响应会稍慢，但位置更稳定）

推荐: 先试 A (Q_quatz=15)，如果 yaw 仍然可接受且位置改善，就是最佳折中;
      如果位置仍然差，再试 C (R_wz=0.6)。
""")

# ===== 绘图 =====
fig = plt.figure(figsize=(18, 12))
gs = GridSpec(3, 3, figure=fig, hspace=0.35, wspace=0.35)

# 图1: Yaw 跟踪
ax = fig.add_subplot(gs[0, 0])
yaw_ref_uw = np.unwrap(yaw_refs[~np.isnan(yaw_refs)])
t_ref_uw = t_super[~np.isnan(yaw_refs)]
yaw_act_uw = np.unwrap(yaw_act_arr)
ax.plot(t_ref_uw, np.degrees(yaw_ref_uw), 'b-', alpha=0.7, lw=0.8, label='ref yaw')
ax.plot(t_pose, np.degrees(yaw_act_uw), 'r-', alpha=0.7, lw=0.5, label='actual yaw')
ax.set_ylabel('Yaw (deg, unwrapped)'); ax.set_xlabel('Time (s)')
ax.set_title('Yaw Tracking'); ax.legend(loc='best'); ax.grid(True, alpha=0.3)

# 图2: Yaw 误差
ax = fig.add_subplot(gs[0, 1])
ax.plot(t_yaw_valid, np.degrees(yaw_err), 'k-', alpha=0.5, lw=0.3)
ax.axhline(y=0, color='gray', ls='--', alpha=0.5)
ax.set_ylabel('Yaw Error (deg)'); ax.set_xlabel('Time (s)')
ax.set_title('Yaw Error'); ax.grid(True, alpha=0.3)

# 图3: Yaw 误差直方图
ax = fig.add_subplot(gs[0, 2])
ax.hist(np.degrees(yaw_err), bins=50, color='steelblue', edgecolor='white', alpha=0.8)
ax.axvline(x=0, color='red', ls='--', alpha=0.5)
ax.set_xlabel('Yaw Error (deg)'); ax.set_ylabel('Count')
ax.set_title('Yaw Error Distribution'); ax.grid(True, alpha=0.3, axis='y')

# 图4: X 位置
ax = fig.add_subplot(gs[1, 0])
ax.plot(t_super, pos_refs_super_arr[:,0], 'b-', alpha=0.6, lw=0.8, label='ref X')
ax.plot(t_pose, pos_act_arr[:,0], 'r-', alpha=0.6, lw=0.5, label='actual X')
if len(pos_ref_arr) > 0:
    ax.plot(t_ref, pos_ref_arr[:,0], 'g--', alpha=0.5, lw=0.8, label='NMPC ref X')
ax.set_ylabel('X (m)'); ax.set_xlabel('Time (s)')
ax.set_title('X Position'); ax.legend(loc='best'); ax.grid(True, alpha=0.3)

# 图5: Y 位置
ax = fig.add_subplot(gs[1, 1])
ax.plot(t_super, pos_refs_super_arr[:,1], 'b-', alpha=0.6, lw=0.8, label='ref Y')
ax.plot(t_pose, pos_act_arr[:,1], 'r-', alpha=0.6, lw=0.5, label='actual Y')
if len(pos_ref_arr) > 0:
    ax.plot(t_ref, pos_ref_arr[:,1], 'g--', alpha=0.5, lw=0.8, label='NMPC ref Y')
ax.set_ylabel('Y (m)'); ax.set_xlabel('Time (s)')
ax.set_title('Y Position'); ax.legend(loc='best'); ax.grid(True, alpha=0.3)

# 图6: Z 位置
ax = fig.add_subplot(gs[1, 2])
ax.plot(t_super, pos_refs_super_arr[:,2], 'b-', alpha=0.6, lw=0.8, label='ref Z')
ax.plot(t_pose, pos_act_arr[:,2], 'r-', alpha=0.6, lw=0.5, label='actual Z')
if len(pos_ref_arr) > 0:
    ax.plot(t_ref, pos_ref_arr[:,2], 'g--', alpha=0.5, lw=0.8, label='NMPC ref Z')
ax.set_ylabel('Z (m)'); ax.set_xlabel('Time (s)')
ax.set_title('Z Position'); ax.legend(loc='best'); ax.grid(True, alpha=0.3)

# 图7: X 误差
ax = fig.add_subplot(gs[2, 0])
x_err = pos_act_arr[valid_pxy,0] - pos_ref_x_aligned[valid_pxy]
ax.plot(t_pose[valid_pxy], x_err, 'r-', alpha=0.5, lw=0.3)
ax.axhline(y=0, color='gray', ls='--', alpha=0.5)
ax.set_ylabel('X Error (m)'); ax.set_xlabel('Time (s)')
ax.set_title('X Position Error'); ax.grid(True, alpha=0.3)

# 图8: Y 误差
ax = fig.add_subplot(gs[2, 1])
y_err = pos_act_arr[valid_pxy,1] - pos_ref_y_aligned[valid_pxy]
ax.plot(t_pose[valid_pxy], y_err, 'g-', alpha=0.5, lw=0.3)
ax.axhline(y=0, color='gray', ls='--', alpha=0.5)
ax.set_ylabel('Y Error (m)'); ax.set_xlabel('Time (s)')
ax.set_title('Y Position Error'); ax.grid(True, alpha=0.3)

# 图9: XY 轨迹
ax = fig.add_subplot(gs[2, 2])
ax.plot(pos_refs_super_arr[:,0], pos_refs_super_arr[:,1], 'b-', alpha=0.5, lw=1.0, label='ref path')
ax.plot(pos_act_arr[:,0], pos_act_arr[:,1], 'r-', alpha=0.5, lw=0.5, label='actual path')
ax.plot(pos_refs_super_arr[0,0], pos_refs_super_arr[0,1], 'bo', ms=6, label='start')
ax.plot(pos_refs_super_arr[-1,0], pos_refs_super_arr[-1,1], 'bx', ms=10, label='end')
ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
ax.set_title('XY Trajectory'); ax.legend(loc='best'); ax.axis('equal'); ax.grid(True, alpha=0.3)

plt.suptitle(f'Flight Performance — Q_quatz=19 — {BAG_PATH.split("/")[-1]}', fontsize=14)
plt.savefig("/home/flag/Jiangyin2026/full_flight_analysis.png", dpi=150, bbox_inches='tight')
print(f"\nPlot saved to /home/flag/Jiangyin2026/full_flight_analysis.png")
print("Done.")
