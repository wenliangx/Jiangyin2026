#!/usr/bin/env python3
"""
分析 NMPC 悬停起飞 bag 数据，诊断：
1. pitch回弹/仰头现象
2. Z轴顿挫/暂停现象
3. NMPC 动态响应问题的根因

用法: python3 analyze_bag.py [bag文件路径]
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 — register 3D projection
from matplotlib.gridspec import GridSpec
from collections import defaultdict

try:
    import rosbag
except ImportError:
    print("需要 rosbag 库: pip install rosbag 或 source /opt/ros/noetic/setup.bash")
    sys.exit(1)

# ========== 配置 ==========
BAG_PATH = sys.argv[1] if len(sys.argv) > 1 else "/home/flag/Jiangyin2026/2026-07-18-17-19-23.bag"
OUTPUT_DIR = os.path.dirname(BAG_PATH)

# ========== 读取数据 ==========
print(f"读取 bag: {BAG_PATH}")
bag = rosbag.Bag(BAG_PATH)

# /mavros/local_position/odom — EKF融合里程计 (位置+姿态+速度)
# /mavros/local_position/pose — EKF融合位姿 (位置+姿态)
# /mavros/setpoint_position/local — 位置设定点
# /Odometry — 自定义里程计

data = {
    'odom_t': [],       'odom_px': [], 'odom_py': [], 'odom_pz': [],
    'odom_vx': [], 'odom_vy': [], 'odom_vz': [],
    'odom_qw': [], 'odom_qx': [], 'odom_qy': [], 'odom_qz': [],
    'odom_wx': [], 'odom_wy': [], 'odom_wz': [],

    'pose_t': [],       'pose_px': [], 'pose_py': [], 'pose_pz': [],
    'pose_qw': [], 'pose_qx': [], 'pose_qy': [], 'pose_qz': [],

    'sp_t': [],          'sp_px': [], 'sp_py': [], 'sp_pz': [],
}

t0 = bag.get_start_time()

for topic, msg, t in bag.read_messages():
    ts = t.to_sec() - t0

    if topic == '/mavros/local_position/odom':
        data['odom_t'].append(ts)
        data['odom_px'].append(msg.pose.pose.position.x)
        data['odom_py'].append(msg.pose.pose.position.y)
        data['odom_pz'].append(msg.pose.pose.position.z)
        data['odom_qw'].append(msg.pose.pose.orientation.w)
        data['odom_qx'].append(msg.pose.pose.orientation.x)
        data['odom_qy'].append(msg.pose.pose.orientation.y)
        data['odom_qz'].append(msg.pose.pose.orientation.z)
        data['odom_vx'].append(msg.twist.twist.linear.x)
        data['odom_vy'].append(msg.twist.twist.linear.y)
        data['odom_vz'].append(msg.twist.twist.linear.z)
        data['odom_wx'].append(msg.twist.twist.angular.x)
        data['odom_wy'].append(msg.twist.twist.angular.y)
        data['odom_wz'].append(msg.twist.twist.angular.z)

    elif topic == '/mavros/local_position/pose':
        data['pose_t'].append(ts)
        data['pose_px'].append(msg.pose.position.x)
        data['pose_py'].append(msg.pose.position.y)
        data['pose_pz'].append(msg.pose.position.z)
        data['pose_qw'].append(msg.pose.orientation.w)
        data['pose_qx'].append(msg.pose.orientation.x)
        data['pose_qy'].append(msg.pose.orientation.y)
        data['pose_qz'].append(msg.pose.orientation.z)

    elif topic == '/mavros/setpoint_position/local':
        data['sp_t'].append(ts)
        data['sp_px'].append(msg.pose.position.x)
        data['sp_py'].append(msg.pose.position.y)
        data['sp_pz'].append(msg.pose.position.z)

bag.close()

# 转为 numpy
for k in data:
    data[k] = np.array(data[k])

print(f"原始 odom 消息数: {len(data['odom_t'])}")
print(f"分析时长: {data['odom_t'][-1]:.1f}s")

# ========== 关键指标计算 ==========

def quat_to_euler(qw, qx, qy, qz):
    """四元数→欧拉角 (roll, pitch, yaw)"""
    roll = np.arctan2(2.0*(qw*qx + qy*qz), 1.0 - 2.0*(qx*qx + qy*qy))
    sinp = 2.0*(qw*qy - qz*qx)
    pitch = np.where(np.abs(sinp) >= 1, np.copysign(np.pi/2, sinp), np.arcsin(sinp))
    yaw = np.arctan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
    return np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw)

# 计算欧拉角
roll, pitch, yaw = quat_to_euler(
    data['odom_qw'], data['odom_qx'], data['odom_qy'], data['odom_qz'])

# ========== 自动检测起飞段 ==========
# 起飞特征: z从接近0 → 上升到0.2m以上
dz = data['odom_pz']
takeoff_idx = np.where(dz > 0.15)[0]
if len(takeoff_idx) > 0:
    t_start = data['odom_t'][takeoff_idx[0]]
    # 向前扩展2s看准备阶段
    t_analysis_start = max(0, t_start - 2.0)
else:
    t_analysis_start = 0

# 找出hover段：z接近1.0m ± 0.1的区间
hover_mask = (dz > 0.85) & (dz < 1.2)
hover_idx = np.where(hover_mask)[0]
if len(hover_idx) > 0:
    t_hover_start = data['odom_t'][hover_idx[0]]
else:
    t_hover_start = data['odom_t'][-1]

# 截取关键段: 起飞前2s 到 hover稳定后3s
t_plot_start = t_analysis_start
t_plot_end = min(data['odom_t'][-1], t_hover_start + 5.0)
mask = (data['odom_t'] >= t_plot_start) & (data['odom_t'] <= t_plot_end)

print(f"\n=== 关键时间点 ===")
print(f"起飞开始 (z>0.15m): t={t_start:.1f}s")
print(f"到达hover (z≈1.0m): t={t_hover_start:.1f}s")
print(f"上升用时: {t_hover_start - t_start:.2f}s")

# ========== 详细分析 ==========

# 1. Z轴运动分析
z_sel = data['odom_pz'][mask]
t_sel = data['odom_t'][mask]
vz_sel = data['odom_vz'][mask]

# 找到Z轴最大速度
vz_max_idx = np.argmax(vz_sel)
print(f"\n=== Z轴运动分析 ===")
print(f"最大上升速度: {vz_sel[vz_max_idx]:.3f} m/s @ t={t_sel[vz_max_idx]:.2f}s")
print(f"最大上升速度时 Z={z_sel[vz_max_idx]:.3f}m")

# 检测Z轴"顿挫"：速度变化率突变
dvz = np.diff(vz_sel) / np.diff(t_sel)
dvz = np.append(dvz, 0)
# 找出大幅减速段（加速度< -2 m/s²，即突然减速）
jerk_mask = dvz < -3.0
jerk_times = t_sel[jerk_mask]
if len(jerk_times) > 0:
    print(f"检测到急减速(jerk<-3m/s²): t={jerk_times[0]:.2f}s ~ {jerk_times[-1]:.2f}s")

# 检查Z是否越过目标(overshoot)
overshoot = z_sel - 1.0  # 目标1.0m
overshoot_positive = overshoot > 0.03
if np.any(overshoot_positive):
    max_overshoot = np.max(overshoot)
    max_overshoot_t = t_sel[np.argmax(overshoot)]
    print(f"Z过冲: 最大{max_overshoot:.3f}m @ t={max_overshoot_t:.2f}s")
else:
    print("Z无过冲")

# Z达到目标1.0m附近（±3cm）的时间
reached_1m = np.where(np.abs(z_sel - 1.0) < 0.03)[0]
if len(reached_1m) > 0:
    consecutive = np.split(reached_1m, np.where(np.diff(reached_1m) != 1)[0] + 1)
    for seg in consecutive:
        if len(seg) > 20:  # 持续0.2s以上
            print(f"Z稳定在1.0±0.03m: t={t_sel[seg[0]]:.2f}s")
            break

# 2. Pitch 分析
pitch_sel = pitch[mask]
roll_sel = roll[mask]

pitch_max_idx = np.argmax(np.abs(pitch_sel))
print(f"\n=== Pitch分析 ===")
print(f"Pitch最大绝对值: {pitch_sel[pitch_max_idx]:.2f}° @ t={t_sel[pitch_max_idx]:.2f}s")
print(f"Pitch最大绝对值时 Z={z_sel[pitch_max_idx]:.3f}m")

# Pitch 震荡检测：寻找pitch符号变化
pitch_sign_change = np.where(np.diff(np.signbit(pitch_sel)))[0]
if len(pitch_sign_change) > 0:
    print(f"Pitch过零次数: {len(pitch_sign_change)}")

# 3. 加速度分析 (从速度差分)
# 计算Z向加速度
acc_z = np.gradient(data['odom_vz'], data['odom_t'])
acc_z_sel = acc_z[mask]
print(f"\n=== Z向加速度分析 ===")
print(f"Z向最大加速度: {np.max(acc_z_sel):.3f} m/s²")
print(f"Z向最小加速度(最大减速度): {np.min(acc_z_sel):.3f} m/s²")

# 4. 悬停稳定段分析
hover_stable = (data['odom_t'] >= t_hover_start + 1.0) & (data['odom_t'] <= t_hover_start + 4.0)
if np.sum(hover_stable) > 10:
    print(f"\n=== 悬停稳定段 (t={t_hover_start+1:.1f}s ~ {t_hover_start+4:.1f}s) ===")
    print(f"Z均值: {np.mean(data['odom_pz'][hover_stable]):.4f} m")
    print(f"Z标准差: {np.std(data['odom_pz'][hover_stable]):.4f} m")
    print(f"Pitch均值: {np.mean(pitch[hover_stable]):.2f}°")
    print(f"Pitch标准差: {np.std(pitch[hover_stable]):.2f}°")
    print(f"Roll均值: {np.mean(roll[hover_stable]):.2f}°")
    print(f"Roll标准差: {np.std(roll[hover_stable]):.2f}°")

# ========== 绘图 ==========
print("\n生成分析图表...")

fig = plt.figure(figsize=(20, 18))
gs = GridSpec(5, 2, figure=fig, hspace=0.35, wspace=0.3)

# ---- Panel 1: 3D轨迹 ----
ax1 = fig.add_subplot(gs[0, 0], projection='3d')
ax1.plot(data['odom_px'][mask], data['odom_py'][mask], data['odom_pz'][mask],
         linewidth=0.8, color='steelblue')
ax1.scatter(0, 0, 1.0, c='red', s=80, marker='*', label='Target (0,0,1.0)')
ax1.set_xlabel('X [m]')
ax1.set_ylabel('Y [m]')
ax1.set_zlabel('Z [m]')
ax1.set_title('3D Position Trajectory (takeoff → hover)')
ax1.legend()

# ---- Panel 2: Z位置 vs 时间 ----
ax2 = fig.add_subplot(gs[0, 1])
ax2.plot(t_sel, z_sel, 'b-', linewidth=1.2, label='Z position')
ax2.axhline(y=1.0, color='r', linestyle='--', alpha=0.7, label='Target 1.0m')
# 标注起飞点
ax2.axvline(x=t_start, color='green', linestyle=':', alpha=0.7, label=f'Takeoff t={t_start:.1f}s')
ax2.axvline(x=t_hover_start, color='orange', linestyle=':', alpha=0.7, label=f'Reach hover t={t_hover_start:.1f}s')
ax2.set_xlabel('Time [s]')
ax2.set_ylabel('Z [m]')
ax2.set_title('Z Position vs Time')
ax2.legend(loc='best')
ax2.grid(True, alpha=0.3)

# ---- Panel 3: Z速度 vs 时间 ----
ax3 = fig.add_subplot(gs[1, 0])
ax3.plot(t_sel, vz_sel, 'b-', linewidth=1.2, label='Vz')
ax3.fill_between(t_sel, 0, vz_sel, alpha=0.2)
ax3.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
ax3.axvline(x=t_start, color='green', linestyle=':', alpha=0.7)
ax3.axvline(x=t_hover_start, color='orange', linestyle=':', alpha=0.7)
ax3.set_xlabel('Time [s]')
ax3.set_ylabel('Vz [m/s]')
ax3.set_title('Z Velocity vs Time')
ax3.legend(loc='best')
ax3.grid(True, alpha=0.3)

# ---- Panel 4: Z加速度 vs 时间 ----
ax4 = fig.add_subplot(gs[1, 1])
ax4.plot(t_sel, acc_z_sel, 'r-', linewidth=1.0, label='Az (from diff Vz)')
ax4.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
ax4.axvline(x=t_start, color='green', linestyle=':', alpha=0.7)
ax4.axvline(x=t_hover_start, color='orange', linestyle=':', alpha=0.7)
ax4.set_xlabel('Time [s]')
ax4.set_ylabel('Az [m/s²]')
ax4.set_title('Z Acceleration vs Time (numerical derivative)')
ax4.legend(loc='best')
ax4.grid(True, alpha=0.3)

# ---- Panel 5: Pitch vs 时间 ----
ax5 = fig.add_subplot(gs[2, 0])
ax5.plot(t_sel, pitch_sel, 'r-', linewidth=1.2, label='Pitch')
ax5.fill_between(t_sel, -2, 2, alpha=0.1, color='green', label='±2° band')
ax5.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
ax5.axvline(x=t_start, color='green', linestyle=':', alpha=0.7)
ax5.axvline(x=t_hover_start, color='orange', linestyle=':', alpha=0.7)
ax5.set_xlabel('Time [s]')
ax5.set_ylabel('Pitch [°]')
ax5.set_title('Pitch Angle vs Time')
ax5.legend(loc='best')
ax5.grid(True, alpha=0.3)

# ---- Panel 6: Roll vs Pitch (XY散点) ----
ax6 = fig.add_subplot(gs[2, 1])
sc = ax6.scatter(roll_sel, pitch_sel, c=t_sel, s=5, cmap='viridis', alpha=0.6)
ax6.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
ax6.axvline(x=0, color='gray', linestyle='--', alpha=0.5)
ax6.set_xlabel('Roll [°]')
ax6.set_ylabel('Pitch [°]')
ax6.set_title('Attitude: Pitch vs Roll (colored by time)')
ax6.grid(True, alpha=0.3)
plt.colorbar(sc, ax=ax6, label='Time [s]')

# ---- Panel 7: Z位置 vs Z速度 (相图) ----
ax7 = fig.add_subplot(gs[3, 0])
points = ax7.scatter(z_sel, vz_sel, c=t_sel, s=5, cmap='plasma', alpha=0.7)
ax7.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
ax7.axvline(x=1.0, color='r', linestyle='--', alpha=0.5, label='Target Z=1.0')
ax7.set_xlabel('Z Position [m]')
ax7.set_ylabel('Vz [m/s]')
ax7.set_title('Phase Portrait: Z vs Vz (colored by time)')
ax7.legend(loc='best')
ax7.grid(True, alpha=0.3)
plt.colorbar(points, ax=ax7, label='Time [s]')

# ---- Panel 8: 水平位置 ----
ax8 = fig.add_subplot(gs[3, 1])
ax8.plot(t_sel, data['odom_px'][mask], 'b-', linewidth=1.0, label='X')
ax8.plot(t_sel, data['odom_py'][mask], 'r-', linewidth=1.0, label='Y')
ax8.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
ax8.axvline(x=t_start, color='green', linestyle=':', alpha=0.7)
ax8.axvline(x=t_hover_start, color='orange', linestyle=':', alpha=0.7)
ax8.set_xlabel('Time [s]')
ax8.set_ylabel('Position [m]')
ax8.set_title('Horizontal Position vs Time')
ax8.legend(loc='best')
ax8.grid(True, alpha=0.3)

# ---- Panel 9: 全段Z全景 ----
ax9 = fig.add_subplot(gs[4, 0])
ax9.plot(data['odom_t'], data['odom_pz'], 'b-', linewidth=0.8)
ax9.axhline(y=1.0, color='r', linestyle='--', alpha=0.7)
ax9.axvline(x=t_start, color='green', linestyle=':', alpha=0.7)
ax9.axvline(x=t_hover_start, color='orange', linestyle=':', alpha=0.7)
ax9.set_xlabel('Time [s]')
ax9.set_ylabel('Z [m]')
ax9.set_title('Full Z Trajectory')

# ---- Panel 10: 全段Pitch全景 ----
ax10 = fig.add_subplot(gs[4, 1])
ax10.plot(data['odom_t'], pitch, 'r-', linewidth=0.8)
ax10.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
ax10.axvline(x=t_start, color='green', linestyle=':', alpha=0.7)
ax10.axvline(x=t_hover_start, color='orange', linestyle=':', alpha=0.7)
ax10.set_xlabel('Time [s]')
ax10.set_ylabel('Pitch [°]')
ax10.set_title('Full Pitch Trajectory')

fig.suptitle(f'NMPC Hover Takeoff Analysis\nBag: {os.path.basename(BAG_PATH)}',
             fontsize=14, fontweight='bold', y=0.98)

out_png = os.path.join(OUTPUT_DIR, 'nmpc_hover_analysis.png')
plt.savefig(out_png, dpi=150, bbox_inches='tight')
print(f"图表已保存: {out_png}")

# ========== 生成详细文本报告 ==========
print("\n" + "="*70)
print("                    根因分析报告")
print("="*70)

# 分析Z轴"顿挫"
# 现象: Z在接近目标时减速，然后"顿"一下
# 可能原因: NMPC 同时受到位置误差和姿态误差的拉扯
# 当Z接近目标时，位置误差变小，姿态误差（如果此时有pitch）变成主导
# NMPC会降低acc_z来减少过冲，同时纠正attitude

# 分析Pitch回弹
# 计算pitch峰值时间与Z速度峰值时间的相对关系
if len(vz_sel) > 0:
    vz_peak_t = t_sel[np.argmax(vz_sel)]
if len(pitch_sel) > 0:
    pitch_peak_t = t_sel[np.argmax(np.abs(pitch_sel))]

print(f"\n时序对比:")
if 'vz_peak_t' in dir():
    print(f"  Vz峰值时间: t={vz_peak_t:.2f}s")
    print(f"  Z此时: {z_sel[np.argmax(vz_sel)]:.3f}m")
print(f"  Pitch峰值时间: t={pitch_peak_t:.2f}s")
print(f"  Z此时: {z_sel[np.argmax(np.abs(pitch_sel))]:.3f}m")
if 'vz_peak_t' in dir():
    print(f"  时间差(Pitch_peak - Vz_peak): {pitch_peak_t - vz_peak_t:.2f}s")

# 分析上升段Z速度profile
# 理想情况: 加速→匀速→减速→到达
# 检查是否有"双峰"现象(加速-减速-再加速-再减速)
print(f"\nVz速度峰分析:")
# Simple peak finding without scipy
peak_indices = []
for i in range(1, len(vz_sel) - 1):
    if vz_sel[i] > 0.05 and vz_sel[i] > vz_sel[i-1] and vz_sel[i] > vz_sel[i+1]:
        peak_indices.append(i)
# merge close peaks
filtered_peaks = []
for p in peak_indices:
    if not filtered_peaks or p - filtered_peaks[-1] > 30:
        filtered_peaks.append(p)
if len(filtered_peaks) > 1:
    print(f"  检测到 {len(filtered_peaks)} 个速度峰:")
    for i, p in enumerate(filtered_peaks):
        print(f"    峰{i}: t={t_sel[p]:.2f}s, Vz={vz_sel[p]:.3f}m/s, Z={z_sel[p]:.3f}m")
elif len(filtered_peaks) == 1:
    print(f"  检测到 1 个速度峰: t={t_sel[filtered_peaks[0]]:.2f}s, Vz={vz_sel[filtered_peaks[0]]:.3f}m/s")
else:
    p = np.argmax(vz_sel)
    print(f"  (使用最大值): t={t_sel[p]:.2f}s, Vz={vz_sel[p]:.3f}m/s")

# 分析Z从0.2→0.8的profile
climb_mask = (z_sel > 0.2) & (z_sel < 0.95)
if np.sum(climb_mask) > 5:
    print(f"\n上升段(0.2→0.95m)分析:")
    print(f"  平均速度: {np.mean(vz_sel[climb_mask]):.3f} m/s")
    print(f"  耗时: {t_sel[climb_mask][-1] - t_sel[climb_mask][0]:.2f}s")

# 分析pitch和Z加速度的耦合关系
# NMPC模型: v̇_x = 2(q₀q₂+q₁q₃)*acc_z  ≈ 2*q2*acc_z
#            v̇_z = (q₀²-q₁²-q₂²+q₃²)*acc_z - g  ≈ acc_z - g (当水平时)
# 当pitch≠0时，acc_z会产生水平加速度分量
# 这导致: pitch → 水平推力分量 → 位置偏差 → NMPC用pitch纠正 → 耦合振荡
print(f"\n姿态-位置耦合分析:")
# 检查: pitch 和 X加速度的相关性
ax_from_v = np.gradient(data['odom_vx'], data['odom_t'])
ax_sel = ax_from_v[mask]
# acc_z_sel 中去除重力
if len(ax_sel) == len(acc_z_sel):
    # 理论: ax ≈ -pitch * acc_z (小角度近似)
    pitch_rad = np.deg2rad(pitch_sel)
    theoretical_ax = -np.sin(pitch_rad) * acc_z_sel  # body z acc 在world x的分量
    # 检查相关性
    if len(ax_sel) > 10:
        corr = np.corrcoef(ax_sel, theoretical_ax)[0, 1]
        print(f"  X加速度 vs -sin(pitch)*acc_z 相关系数: {corr:.3f}")
        print(f"  (如果接近1，说明水平运动主要是pitch引起的)")

print("\n" + "="*70)
print("分析完成！请查看图表和上述数据。")
print("="*70)
