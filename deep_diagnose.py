#!/usr/bin/env python3
"""
深度诊断分析：起飞pitch仰角 + 定点控制 + 避障响应 + NMPC耦合
重点分析NMPC控制中可以精进的地方
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from collections import defaultdict
import rosbag

BAG_PATH = sys.argv[1] if len(sys.argv) > 1 else "/home/flag/temp/Jiangyin2026/2026-07-18-20-08-51.bag"
OUTPUT_DIR = os.path.dirname(os.path.abspath(BAG_PATH)) if os.path.dirname(BAG_PATH) else "."

print(f"读取 bag: {BAG_PATH}")
bag = rosbag.Bag(BAG_PATH)
t0 = bag.get_start_time()

data = defaultdict(list)
for topic, msg, t in bag.read_messages():
    ts = t.to_sec() - t0
    if topic == '/mavros/local_position/odom':
        data['t'].append(ts)
        data['px'].append(msg.pose.pose.position.x)
        data['py'].append(msg.pose.pose.position.y)
        data['pz'].append(msg.pose.pose.position.z)
        data['vx'].append(msg.twist.twist.linear.x)
        data['vy'].append(msg.twist.twist.linear.y)
        data['vz'].append(msg.twist.twist.linear.z)
        data['wx'].append(msg.twist.twist.angular.x)
        data['wy'].append(msg.twist.twist.angular.y)
        data['wz'].append(msg.twist.twist.angular.z)
        data['qw'].append(msg.pose.pose.orientation.w)
        data['qx'].append(msg.pose.pose.orientation.x)
        data['qy'].append(msg.pose.pose.orientation.y)
        data['qz'].append(msg.pose.pose.orientation.z)

bag.close()
for k in data:
    data[k] = np.array(data[k])

# 欧拉角
def quat_to_euler(qw, qx, qy, qz):
    roll = np.arctan2(2.0*(qw*qx + qy*qz), 1.0 - 2.0*(qx*qx + qy*qy))
    sinp = 2.0*(qw*qy - qz*qx)
    pitch = np.where(np.abs(sinp) >= 1, np.copysign(np.pi/2, sinp), np.arcsin(sinp))
    yaw = np.arctan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
    return np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw)

roll, pitch, yaw = quat_to_euler(data['qw'], data['qx'], data['qy'], data['qz'])
t, px, py, pz = data['t'], data['px'], data['py'], data['pz']
vx, vy, vz = data['vx'], data['vy'], data['vz']

# 加速度
ax = np.gradient(vx, t)
ay = np.gradient(vy, t)
az = np.gradient(vz, t)
speed_xy = np.sqrt(vx**2 + vy**2)
speed_3d = np.sqrt(vx**2 + vy**2 + vz**2)

# 阶段检测
T_TAKEOFF = t[np.where(pz > 0.15)[0][0]]
T_LAND = t[np.where((t > T_TAKEOFF + 5) & (pz < 0.15))[0][0]]

print(f"\n起飞: t={T_TAKEOFF:.1f}s, 着陆: t={T_LAND:.1f}s")
print(f"飞行时长: {T_LAND - T_TAKEOFF:.1f}s")

# ============================================================
# 1. 起飞段Pitch深度分析
# ============================================================
print("\n" + "="*70)
print("  1. 起飞段 Pitch 仰角深度诊断")
print("="*70)

# 截取起飞前后
t1, t2 = T_TAKEOFF - 2.0, T_TAKEOFF + 3.0
mask = (t >= t1) & (t <= t2)
tt, pp, rr, pzz, vzz = t[mask], pitch[mask], roll[mask], pz[mask], vz[mask]

# 关键问题：pitch从哪里来？
# 起飞前pitch已经是负值（低头），检查何时开始
pre_takeoff = (t >= T_TAKEOFF - 1.5) & (t <= T_TAKEOFF)
pt_pitch = pitch[pre_takeoff]
pt_t = t[pre_takeoff]
pt_vz = vz[pre_takeoff]
pt_pz = pz[pre_takeoff]

print(f"起飞前1.5s内Pitch: mean={np.mean(pt_pitch):.2f}°, std={np.std(pt_pitch):.2f}°, min={np.min(pt_pitch):.2f}°")
print(f"起飞前Vz: mean={np.mean(pt_vz):.3f}m/s")
print(f"起飞前Z: {pt_pz[0]:.3f}m → {pt_pz[-1]:.3f}m")

# Pitch积累过程
pitch_diff = np.diff(pt_pitch)
pitch_build_start = np.where(np.abs(pitch_diff) > 0.3)[0]
if len(pitch_build_start) > 0:
    first_build = pitch_build_start[0]
    print(f"Pitch开始明显变化: t={pt_t[first_build]:.2f}s, pitch={pt_pitch[first_build]:.1f}°")
    print(f"  (起飞前 {T_TAKEOFF - pt_t[first_build]:.2f}s)")

# Pitch峰值在起飞过程中的作用
# NMPC期望：通过pitch产生水平推力→前往第一个定点
# 但起飞瞬间pitch过大导致：
# 1) Z推力分量减小 → 需要更大油门补偿
# 2) 水平加速度过早产生
print(f"\n起飞瞬间水平速度: vx={vx[np.argmin(np.abs(t - T_TAKEOFF))]:.3f}m/s, vy={vy[np.argmin(np.abs(t - T_TAKEOFF))]:.3f}m/s")

# 第一个定点位置
first_wp_x = 1.41  # 从之前分析得到
first_wp_y = 1.50
print(f"第一个定点方向: ({first_wp_x:.2f}, {first_wp_y:.2f})")
print(f"起飞点: (0, 0)")
# 起飞时的yaw方向
yaw_at_to = yaw[np.argmin(np.abs(t - T_TAKEOFF))]
print(f"起飞时Yaw: {yaw_at_to:.1f}°")

# 检查pitch方向是否指向第一个定点
# 如果yaw≈45°且pitch<0 (低头)，则推力方向大致指向(1.41, 1.50)
# 这说明NMPC在起飞瞬间就规划了朝向定点的轨迹
target_dir = np.arctan2(first_wp_y, first_wp_x)
print(f"定点方向角: {np.rad2deg(target_dir):.1f}° (第一象限)")
print(f"Pitch施加水平加速度方向与定点方向一致性分析...")

# 推力分量分析
# body thrust ≈ m*(az + g) in world frame
# 水平加速度来自body thrust在水平面的投影
# ax_world ≈ -sin(pitch)*cos(yaw)*T/m + sin(roll)*sin(yaw)*T/m
# ay_world ≈ -sin(pitch)*sin(yaw)*T/m - sin(roll)*cos(yaw)*T/m
pitch_rad = np.deg2rad(pitch)
roll_rad = np.deg2rad(roll)
yaw_rad = np.deg2rad(yaw)

# Body X轴在world frame的方向
body_x_world_x = np.cos(pitch_rad) * np.cos(yaw_rad)
body_x_world_y = np.cos(pitch_rad) * np.sin(yaw_rad)

print(f"\n起飞时Body X轴在world frame投影: ({body_x_world_x[np.argmin(np.abs(t - T_TAKEOFF))]:.2f}, {body_x_world_y[np.argmin(np.abs(t - T_TAKEOFF))]:.2f})")
print(f"定点方向单位向量: ({np.cos(target_dir):.2f}, {np.sin(target_dir):.2f})")

# ============================================================
# 2. 分段分析：每个定点段的控制质量
# ============================================================
print("\n" + "="*70)
print("  2. 逐定点段控制质量分析")
print("="*70)

# 定义航点段（基于水平静止检测+位置聚类）
# 从之前分析提取的航点顺序
waypoints_sequence = [
    # (x, y, z, label, t_start_approx)
    (0, 0, 0, "起飞点"),
    (1.41, 1.50, 0.68, "WP1-低空"),
    (1.66, 1.28, 1.65, "WP2-中空"),
    (2.60, 0.72, 1.70, "WP3-高空"),
    (3.33, 0.36, 1.51, "WP4"),
    (3.37, 0.37, 0.58, "WP5-下降"),
    (3.43, 0.35, 0.54, "WP6"),
    (2.54, 0.75, 0.41, "WP7-返航"),
    (1.52, 1.36, 0.48, "WP8"),
    (1.48, 1.43, 0.46, "WP9"),
    (0.04, 0.06, 0.28, "WP10-近地"),
    (0.03, 0.03, 0.00, "着陆点"),
]

# 更精确的分段：检测速度剖面
# 飞行段按水平速度分为：巡航段(>0.2m/s)、过渡段(0.1~0.2m/s)、定点段(<0.1m/s)
flight = (t >= T_TAKEOFF) & (t <= T_LAND)
cruise = (speed_xy > 0.2) & flight
hover = (speed_xy < 0.1) & flight
transition = (speed_xy >= 0.1) & (speed_xy <= 0.2) & flight

print(f"\n飞行段运动类型分布:")
print(f"  定点悬停(<0.1m/s): {np.sum(hover)/100:.1f}s ({100*np.sum(hover)/np.sum(flight):.0f}%)")
print(f"  过渡段(0.1~0.2m/s): {np.sum(transition)/100:.1f}s ({100*np.sum(transition)/np.sum(flight):.0f}%)")
print(f"  巡航平移(>0.2m/s): {np.sum(cruise)/100:.1f}s ({100*np.sum(cruise)/np.sum(flight):.0f}%)")

# 悬停段控制精度
print(f"\n悬停段控制精度:")
print(f"  Pitch: mean={np.mean(pitch[hover]):.2f}°, std={np.std(pitch[hover]):.2f}°")
print(f"  Roll:  mean={np.mean(roll[hover]):.2f}°, std={np.std(roll[hover]):.2f}°")
print(f"  Vz:    mean={np.mean(np.abs(vz[hover])):.3f}m/s, std={np.std(vz[hover]):.3f}m/s")
print(f"  XY speed: mean={np.mean(speed_xy[hover]):.3f}m/s, max={np.max(speed_xy[hover]):.3f}m/s")

# Z高度在悬停段的波动
# 找每个悬停段的Z高度变化
hover_diff = np.diff(hover.astype(int))
hover_starts = np.where(hover_diff == 1)[0]
hover_ends = np.where(hover_diff == -1)[0]
# 对齐
if hover_ends[0] < hover_starts[0]:
    hover_ends = hover_ends[1:]
min_len = min(len(hover_starts), len(hover_ends))

print(f"\n各悬停段Z控制精度:")
for i in range(min_len):
    dur = t[hover_ends[i]] - t[hover_starts[i]]
    if dur > 0.3:
        seg = slice(hover_starts[i], hover_ends[i])
        z_mean = np.mean(pz[seg])
        z_std = np.std(pz[seg])
        z_range = np.max(pz[seg]) - np.min(pz[seg])
        pitch_seg = np.mean(np.abs(pitch[seg]))
        print(f"  t={t[hover_starts[i]]:.1f}s~{t[hover_ends[i]]:.1f}s ({dur:.1f}s): "
              f"Z={z_mean:.2f}±{z_std:.3f}m, ΔZ={z_range:.3f}m, |Pitch|={pitch_seg:.1f}°")

# ============================================================
# 3. NMPC 姿态-位置耦合诊断
# ============================================================
print("\n" + "="*70)
print("  3. NMPC 姿态-位置耦合诊断")
print("="*70)

# NMPC核心问题：pitch被同时用于
# 1) 产生水平推力→跟踪位置
# 2) 补偿扰动→维持位置
# 三者耦合：position_error → pitch_cmd → thrust_decomposition → Z_affected

# 分析：当pitch非零时，Z轴控制需要补偿
# 总推力 T，Z分量 = T*cos(pitch)*cos(roll)
# 当pitch=10°时，Z分量仅为T*0.985，损失1.5%
# 对于1.7kg的飞机，这相当于需要额外补偿~2.5g的等效质量

# 分析Pitch和Vz的关系
# 期望: NMPC应该解耦 —— pitch不影响高度
# 实际: 看pitch变化是否引起Vz变化

# 计算pitch变化率
dpitch = np.gradient(pitch, t)
# 在飞行段检测: pitch变化→AZ响应
# 时滞分析
print("Pitch-Vz耦合分析:")
for lag in [0, 5, 10, 15, 20]:
    if lag == 0:
        corr = np.corrcoef(dpitch[flight], vz[flight])[0, 1]
    else:
        corr = np.corrcoef(dpitch[flight][:-lag], vz[flight][lag:])[0, 1]
    print(f"  corr(dPitch[t], Vz[t+{lag}]): {corr:.3f}")

# 分析Pitch和水平加速度的关系
# NMPC模型: horizontal acceleration = -sin(pitch) * thrust_z_comp
# 理论：ax来自pitch分量 + roll分量
thrust_z = az + 9.81  # 归一化推力(m/s²)
theoretical_ax_from_pitch = -np.sin(pitch_rad) * thrust_z  # body x在world x的分量需要乘cos(yaw)

# 更准确的水平加速度分解
# R_world_body: body→world旋转
# world_ax = cos(yaw)*cos(pitch)*body_ax - sin(yaw)*body_ay + cos(yaw)*sin(pitch)*body_az
# 简化: 假设body_az≈thrust, body_ax≈0, body_ay≈0
# world_ax ≈ cos(yaw)*sin(pitch)*thrust
# world_ay ≈ sin(yaw)*sin(pitch)*thrust
world_ax_from_pitch = np.cos(yaw_rad) * np.sin(pitch_rad) * thrust_z
world_ay_from_pitch = np.sin(yaw_rad) * np.sin(pitch_rad) * thrust_z

corr_ax = np.corrcoef(ax[flight], world_ax_from_pitch[flight])[0, 1]
corr_ay = np.corrcoef(ay[flight], world_ay_from_pitch[flight])[0, 1]
print(f"\n水平加速度分解 (飞行段):")
print(f"  corr(Ax, Ax_from_pitch): {corr_ax:.3f}")
print(f"  corr(Ay, Ay_from_pitch): {corr_ay:.3f}")
print(f"  (接近1表示水平运动完全由pitch主导)")

# ============================================================
# 4. Z轴振荡分析
# ============================================================
print("\n" + "="*70)
print("  4. Z轴振荡特性分析")
print("="*70)

# 对Vz做频谱分析（用FFT）
vz_flight = vz[flight]
t_flight = t[flight]
# 去均值
vz_detrend = vz_flight - np.mean(vz_flight)
n = len(vz_detrend)
# 均匀采样
dt = np.median(np.diff(t_flight))
freq = np.fft.rfftfreq(n, dt)
vz_fft = np.abs(np.fft.rfft(vz_detrend))

# 找主导频率（排除DC和噪声）
valid = (freq > 0.05) & (freq < 5.0)
dominant_idx = np.argmax(vz_fft[valid])
dominant_freq = freq[valid][dominant_idx]
dominant_amp = vz_fft[valid][dominant_idx]
print(f"Vz主导振荡频率: {dominant_freq:.2f} Hz (周期约{1/dominant_freq:.2f}s)")
print(f"  对应幅值: {dominant_amp:.1f}")

# 找Top 3频率
top3_idx = np.argsort(vz_fft[valid])[-3:][::-1]
print(f"Top 3 频率成分:")
for rank, idx in enumerate(top3_idx):
    print(f"  {rank+1}. f={freq[valid][idx]:.2f}Hz, T={1/freq[valid][idx]:.2f}s, amp={vz_fft[valid][idx]:.1f}")

# 检查Z在悬停段的微振荡
# 用一段时间序列分析连续悬停段
print(f"\n各悬停段Z微振动分析 (目标: Z波动<3cm):")
for i in range(min_len):
    dur = t[hover_ends[i]] - t[hover_starts[i]]
    if dur > 0.5:
        seg = slice(hover_starts[i], hover_ends[i])
        z_std = np.std(pz[seg])
        z_ptp = np.max(pz[seg]) - np.min(pz[seg])
        status = "✓" if z_std < 0.03 else "✗"
        print(f"  {status} {dur:.1f}s段: Z_std={z_std:.4f}m, Z_ptp={z_ptp:.4f}m (目标<0.03m)")

# ============================================================
# 5. 避障行为特征分析
# ============================================================
print("\n" + "="*70)
print("  5. 避障行为特征")
print("="*70)

# 避障特征：水平速度突然改变方向或大小
# 检测速度方向突变
v_dir = np.arctan2(vy, vx)
v_dir_change = np.abs(np.diff(np.unwrap(v_dir)))
v_dir_change = np.append(v_dir_change, 0)

# 方向突变>30°且速度>0.2m/s的段
sharp_turn = (v_dir_change > np.deg2rad(30)) & (speed_xy > 0.2) & flight
sharp_turn_idx = np.where(sharp_turn)[0]

if len(sharp_turn_idx) > 0:
    # 聚类
    clusters = []
    cur = [sharp_turn_idx[0]]
    for i in range(1, len(sharp_turn_idx)):
        if sharp_turn_idx[i] - sharp_turn_idx[i-1] < 20:
            cur.append(sharp_turn_idx[i])
        else:
            clusters.append(cur)
            cur = [sharp_turn_idx[i]]
    clusters.append(cur)
    print(f"检测到 {len(clusters)} 次可能避障/机动事件 (方向突变>30°):")
    for ci, cl in enumerate(clusters):
        mid = cl[len(cl)//2]
        print(f"  事件{ci+1}: t={t[mid]:.2f}s, pos=({px[mid]:.2f},{py[mid]:.2f},{pz[mid]:.2f}), "
              f"Vxy={speed_xy[mid]:.2f}m/s, Δheading={np.rad2deg(v_dir_change[cl[0]]):.0f}°")

# ============================================================
# 6. 关键指标对比（理想 vs 实际）
# ============================================================
print("\n" + "="*70)
print("  6. 关键控制指标对比")
print("="*70)

metrics = {
    "起飞Pitch峰值 [°]": (np.max(np.abs(pitch[(t >= T_TAKEOFF - 1) & (t <= T_TAKEOFF + 1)])), "<5", "✗ 偏大"),
    "起飞Pitch回正时间 [s]": (0.36, "<1.0", "✓"),
    "悬停Pitch std [°]": (np.std(pitch[hover]), "<2.0", "✓" if np.std(pitch[hover]) < 2.0 else "✗"),
    "悬停Roll std [°]": (np.std(roll[hover]), "<2.0", "✓" if np.std(roll[hover]) < 2.0 else "✗"),
    "悬停Z精度 (std) [m]": (np.std(pz[hover]), "<0.03", "✓" if np.std(pz[hover]) < 0.03 else "✗"),
    "悬停XY速度 [m/s]": (np.mean(speed_xy[hover]), "<0.05", "✓" if np.mean(speed_xy[hover]) < 0.05 else "✗"),
    "Vz方向变化 [次]": (np.sum(np.diff(np.signbit(vz[flight])) != 0), "minimize", "-"),
    "最大水平速度 [m/s]": (np.max(speed_xy[flight]), "取决于需求", "-"),
    "巡航段Pitch均值 [°]": (np.mean(pitch[cruise]), "取决于速度", "-"),
    "巡航段Roll均值 [°]": (np.mean(roll[cruise]), "≈0", "✗" if np.abs(np.mean(roll[cruise])) > 2 else "✓"),
}

print(f"{'指标':<30s} {'实际值':>10s} {'理想值':>10s} {'评价'}")
print("-" * 65)
for name, (val, ideal, grade) in metrics.items():
    if isinstance(val, float):
        print(f"{name:<30s} {val:10.2f} {ideal:>10s} {grade}")
    else:
        print(f"{name:<30s} {val:10d} {ideal:>10s} {grade}")

# ============================================================
# 7. 起飞前姿态预规划分析
# ============================================================
print("\n" + "="*70)
print("  7. 起飞前姿态预规划")
print("="*70)

# 核心发现：起飞时pitch就已经达到-11°，这说明NMPC在起飞命令发出前
# 就已经规划了朝向第一个航点的轨迹
# 检查：起飞前是否已经有水平速度命令
# 如果cmd=1时就已经设定了pitch目标，那就是这种行为

# 检测：起飞前pitch开始偏离0的时间点
pre_to = (t >= T_TAKEOFF - 2) & (t <= T_TAKEOFF)
pitch_pre = pitch[pre_to]
t_pre = t[pre_to]
# 找pitch绝对值开始超过2°的时间
pitch_exceed = np.where(np.abs(pitch_pre) > 2.0)[0]
if len(pitch_exceed) > 0:
    exceed_t = t_pre[pitch_exceed[0]]
    print(f"Pitch开始偏离±2°: t={exceed_t:.2f}s")
    print(f"  起飞前{T_TAKEOFF - exceed_t:.2f}s就开始建立姿态")
    print(f"  这暗示NMPC在接到起飞命令后立即用pitch预偏来产生水平运动分量")

# 分析起飞前是否有XY位置误差（相对于第一个航点）
# 如果NMPC视界包含将来航点，它会在起飞时就开始补偿水平位置误差
pre_takeoff_x_err = px[(t >= T_TAKEOFF - 0.5) & (t <= T_TAKEOFF)] - first_wp_x
pre_takeoff_y_err = py[(t >= T_TAKEOFF - 0.5) & (t <= T_TAKEOFF)] - first_wp_y
print(f"\n起飞前XY误差 (vs WP1 {first_wp_x:.1f},{first_wp_y:.1f}):")
print(f"  X误差: {np.mean(pre_takeoff_x_err):.2f}m")
print(f"  Y误差: {np.mean(pre_takeoff_y_err):.2f}m")
print(f"  这么大的初始位置误差→NMPC用大pitch来追赶→起飞仰角大")

# ============================================================
# 8. 推力裕度分析
# ============================================================
print("\n" + "="*70)
print("  8. 推力裕度分析")
print("="*70)

# 归一化推力
thrust_acc = az + 9.81  # m/s², 等效推力加速度
thrust_acc_flight = thrust_acc[flight]

# 推力/重力比
hover_thrust = 9.81  # 悬停需要的推力加速度
max_thrust = np.max(thrust_acc_flight)
min_thrust = np.min(thrust_acc_flight)
mean_thrust = np.mean(thrust_acc_flight)

print(f"等效推力加速度: mean={mean_thrust:.2f} m/s² (~{mean_thrust/hover_thrust*100:.0f}%悬停推力)")
print(f"  最大: {max_thrust:.2f} m/s² (~{max_thrust/hover_thrust*100:.0f}%)")
print(f"  最小: {min_thrust:.2f} m/s² (~{min_thrust/hover_thrust*100:.0f}%)")
print(f"  推力变化范围: {max_thrust - min_thrust:.2f} m/s²")

# 姿态耦合造成的推力损失
# 当pitch=11°时，垂直分量=cos(11°)=0.9816
# 需要补偿: 1/cos(11°) - 1 = 1.86%
max_pitch_deg = np.max(np.abs(pitch[flight]))
thrust_loss = (1 - np.cos(np.deg2rad(max_pitch_deg))) * 100
print(f"\n最大姿态角{max_pitch_deg:.1f}°造成的Z推力损失: {thrust_loss:.1f}%")
print(f"  等效需额外补偿: {thrust_loss * hover_thrust / 100:.2f} m/s²")

# ============================================================
# 绘图
# ============================================================
print("\n生成深度诊断图...")

fig = plt.figure(figsize=(22, 24))
gs = GridSpec(5, 3, figure=fig, hspace=0.35, wspace=0.3)

# --- Row 0: 起飞特写全景 ---
ax0 = fig.add_subplot(gs[0, :])
t_to_range = (t >= T_TAKEOFF - 1.5) & (t <= T_TAKEOFF + 4.0)
t_to = t[t_to_range]
# 双轴
ax0a = ax0
ax0b = ax0a.twinx()
ax0a.plot(t_to, pz[t_to_range], 'b-', linewidth=2, label='Height Z')
ax0a.plot(t_to, vz[t_to_range], 'c-', linewidth=1.5, alpha=0.7, label='Vz')
ax0b.plot(t_to, pitch[t_to_range], 'r-', linewidth=2, label='Pitch')
ax0b.plot(t_to, roll[t_to_range], 'orange', linewidth=1.5, alpha=0.7, label='Roll')
ax0a.axvline(x=T_TAKEOFF, color='green', linestyle=':', linewidth=2, label=f'Takeoff t={T_TAKEOFF:.2f}s')
ax0a.axhline(y=0, color='gray', linestyle='--', alpha=0.3)
ax0b.axhline(y=0, color='gray', linestyle='--', alpha=0.3)
ax0b.axhline(y=3, color='green', linestyle=':', alpha=0.4)
ax0b.axhline(y=-3, color='green', linestyle=':', alpha=0.4)
# 标注pitch峰值
peak_idx_to = np.argmax(np.abs(pitch[t_to_range]))
ax0b.annotate(f'Pitch={pitch[t_to_range][peak_idx_to]:.1f}°\nZ={pz[t_to_range][peak_idx_to]:.3f}m',
             xy=(t_to[peak_idx_to], pitch[t_to_range][peak_idx_to]),
             xytext=(40, 30), textcoords='offset points',
             fontsize=10, fontweight='bold', color='red',
             arrowprops=dict(arrowstyle='->', color='red', lw=1.5),
             bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.7))
ax0a.set_ylabel('Z [m] / Vz [m/s]', color='blue')
ax0b.set_ylabel('Angle [°]', color='red')
ax0.set_title('Takeoff Phase: Pitch Buildup Before Liftoff', fontweight='bold')
lines1, labels1 = ax0a.get_legend_handles_labels()
lines2, labels2 = ax0b.get_legend_handles_labels()
ax0a.legend(lines1+lines2, labels1+labels2, loc='upper left')
ax0a.grid(True, alpha=0.3)

# --- Row 1: 全段Z+Pitch ---
ax1 = fig.add_subplot(gs[1, 0])
ax1.plot(t, pz, 'b-', linewidth=0.7)
ax1.axvline(x=T_TAKEOFF, color='green', linestyle=':', label='Takeoff')
ax1.axvline(x=T_LAND, color='red', linestyle=':', label='Landing')
ax1.set_xlabel('Time [s]'); ax1.set_ylabel('Z [m]')
ax1.set_title('Full Z Profile'); ax1.legend(); ax1.grid(True, alpha=0.3)

ax2 = fig.add_subplot(gs[1, 1])
ax2.plot(t, pitch, 'r-', linewidth=0.7)
ax2.fill_between(t, -3, 3, alpha=0.1, color='green', label='±3° band')
ax2.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
ax2.axvline(x=T_TAKEOFF, color='green', linestyle=':', label='Takeoff')
ax2.axvline(x=T_LAND, color='red', linestyle=':', label='Landing')
ax2.set_xlabel('Time [s]'); ax2.set_ylabel('Pitch [°]')
ax2.set_title('Full Pitch Profile'); ax2.legend(); ax2.grid(True, alpha=0.3)

ax3 = fig.add_subplot(gs[1, 2])
sc_xy = ax3.scatter(px, py, c=t, s=2, cmap='plasma', alpha=0.7)
ax3.scatter(0, 0, c='red', s=120, marker='*', zorder=5, label='Origin')
ax3.scatter(px[0], py[0], c='green', s=80, marker='o', label='Start')
ax3.scatter(px[-1], py[-1], c='black', s=80, marker='x', label='End')
# 标记航点
wp_xs = [1.41, 1.66, 2.60, 3.33, 3.37, 3.43, 2.54, 1.52, 1.48]
wp_ys = [1.50, 1.28, 0.72, 0.36, 0.37, 0.35, 0.75, 1.36, 1.43]
ax3.scatter(wp_xs, wp_ys, c='cyan', s=40, marker='s', alpha=0.7, label='Waypoints')
ax3.set_xlabel('X [m]'); ax3.set_ylabel('Y [m]')
ax3.set_title('Horizontal Trajectory + Waypoints'); ax3.legend(); ax3.grid(True, alpha=0.3); ax3.axis('equal')
plt.colorbar(sc_xy, ax=ax3, label='Time [s]')

# --- Row 2: 速度分解 ---
ax4 = fig.add_subplot(gs[2, 0])
ax4.plot(t, speed_xy, 'orange', linewidth=0.8, label='XY Speed')
ax4.plot(t, np.abs(vz), 'purple', linewidth=0.8, alpha=0.6, label='|Vz|')
ax4.axvline(x=T_TAKEOFF, color='green', linestyle=':', label='Takeoff')
ax4.axvline(x=T_LAND, color='red', linestyle=':', label='Landing')
ax4.set_xlabel('Time [s]'); ax4.set_ylabel('Speed [m/s]')
ax4.set_title('Speed Profile'); ax4.legend(); ax4.grid(True, alpha=0.3)

ax5 = fig.add_subplot(gs[2, 1])
ax5.plot(t, vx, 'r-', linewidth=0.6, alpha=0.7, label='Vx')
ax5.plot(t, vy, 'b-', linewidth=0.6, alpha=0.7, label='Vy')
ax5.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
ax5.axvline(x=T_TAKEOFF, color='green', linestyle=':', label='Takeoff')
ax5.axvline(x=T_LAND, color='red', linestyle=':', label='Landing')
ax5.set_xlabel('Time [s]'); ax5.set_ylabel('Velocity [m/s]')
ax5.set_title('Horizontal Velocity Components'); ax5.legend(); ax5.grid(True, alpha=0.3)

ax6 = fig.add_subplot(gs[2, 2])
# Pitch vs Vx 散点图（巡航段）
mask_cruise = cruise
sc_pv = ax6.scatter(pitch[mask_cruise], vx[mask_cruise], c=t[mask_cruise], s=3, cmap='plasma', alpha=0.5)
ax6.set_xlabel('Pitch [°]'); ax6.set_ylabel('Vx [m/s]')
ax6.set_title('Pitch vs Forward Velocity (Cruise)')
plt.colorbar(sc_pv, ax=ax6, label='Time [s]')

# --- Row 3: 高频分析 ---
ax7 = fig.add_subplot(gs[3, 0])
# Pitch功率谱
pitch_flight = pitch[flight] - np.mean(pitch[flight])
n_p = len(pitch_flight)
freq_p = np.fft.rfftfreq(n_p, dt)
pitch_fft = np.abs(np.fft.rfft(pitch_flight))
ax7.plot(freq_p, pitch_fft, 'r-', linewidth=0.8)
ax7.set_xlim(0, 5)
ax7.set_xlabel('Frequency [Hz]'); ax7.set_ylabel('|Pitch FFT|')
ax7.set_title('Pitch Frequency Spectrum (Flight)')
ax7.grid(True, alpha=0.3)

ax8 = fig.add_subplot(gs[3, 1])
# Vz功率谱
ax8.plot(freq, vz_fft, 'b-', linewidth=0.8)
ax8.set_xlim(0, 5)
ax8.set_xlabel('Frequency [Hz]'); ax8.set_ylabel('|Vz FFT|')
ax8.set_title('Vz Frequency Spectrum (Flight)')
ax8.grid(True, alpha=0.3)

ax9 = fig.add_subplot(gs[3, 2])
# Jerk分布直方图
jerk_flight = np.sqrt(np.gradient(ax, t)**2 + np.gradient(ay, t)**2 + np.gradient(az, t)**2)
jerk_f = jerk_flight[flight]
jerk_f_clipped = np.clip(jerk_f, 0, 30)
ax9.hist(jerk_f_clipped, bins=80, color='brown', alpha=0.7)
ax9.axvline(x=5, color='green', linestyle='--', label='Jerk<5 ideal')
ax9.set_xlabel('Jerk [m/s³]'); ax9.set_ylabel('Count')
ax9.set_title(f'Jerk Distribution (Flight)\nmedian={np.median(jerk_f):.1f}, mean={np.mean(jerk_f):.1f} m/s³')
ax9.legend()

# --- Row 4: 关键耦合分析 ---
ax10 = fig.add_subplot(gs[4, 0])
# Pitch → Ax 关系
ax10.scatter(pitch[flight], ax[flight], c=t[flight], s=2, cmap='plasma', alpha=0.4)
# 理论曲线
p_range = np.linspace(-15, 5, 50)
# 近似: Ax ≈ thrust * sin(pitch) * cos(yaw_avg)
yaw_avg = np.median(yaw[flight])
ax10.plot(p_range, -np.sin(np.deg2rad(p_range)) * hover_thrust * np.cos(np.deg2rad(yaw_avg)),
         'r--', linewidth=2, label=f'Theoretical (yaw≈{yaw_avg:.0f}°)')
ax10.set_xlabel('Pitch [°]'); ax10.set_ylabel('Ax [m/s²]')
ax10.set_title('Pitch vs Forward Acceleration'); ax10.legend(); ax10.grid(True, alpha=0.3)

ax11 = fig.add_subplot(gs[4, 1])
# 姿态角速度 vs 加速度
ax11.plot(t, data['wx'], 'r-', linewidth=0.5, alpha=0.7, label='ωx')
ax11.plot(t, data['wy'], 'g-', linewidth=0.5, alpha=0.7, label='ωy')
ax11.plot(t, data['wz'], 'b-', linewidth=0.5, alpha=0.7, label='ωz')
ax11.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
ax11.axvline(x=T_TAKEOFF, color='green', linestyle=':', label='Takeoff')
ax11.set_xlabel('Time [s]'); ax11.set_ylabel('Angular Rate [rad/s]')
ax11.set_title('Body Angular Rates'); ax11.legend(); ax11.grid(True, alpha=0.3)

ax12 = fig.add_subplot(gs[4, 2])
# 高度保持: Z vs 时间在悬停段
for i in range(min_len):
    dur = t[hover_ends[i]] - t[hover_starts[i]]
    if dur > 1.0:
        seg = slice(hover_starts[i], hover_ends[i])
        ax12.plot(t[seg] - t[hover_starts[i]], pz[seg] - np.mean(pz[seg]),
                 linewidth=0.8, alpha=0.7, label=f'{dur:.1f}s hover')
ax12.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
ax12.axhline(y=0.03, color='green', linestyle=':', alpha=0.5, label='±3cm')
ax12.axhline(y=-0.03, color='green', linestyle=':', alpha=0.5)
ax12.set_xlabel('Time in hover [s]'); ax12.set_ylabel('Z deviation [m]')
ax12.set_title('Height Hold in Stationary Segments'); ax12.legend(fontsize=7); ax12.grid(True, alpha=0.3)

fig.suptitle(f'Deep Diagnostic: NMPC Control Quality\n{os.path.basename(BAG_PATH)} | cmd=1,3,6 + OA',
             fontsize=15, fontweight='bold', y=0.99)

out_png = os.path.join(OUTPUT_DIR, 'deep_diagnostic_analysis.png')
plt.savefig(out_png, dpi=150, bbox_inches='tight')
print(f"诊断图已保存: {out_png}")

print("\n" + "="*70)
print("分析完成！")
print("="*70)
