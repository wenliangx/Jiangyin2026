#!/usr/bin/env python3
"""
Comprehensive flight analysis for cmd=1→3→5 sequence (hover + waypoint).
Analyzes:
  - Position tracking error (NMPC ref vs actual)
  - Z-axis takeoff/hover/landing performance
  - Horizontal waypoint tracking accuracy
  - Attitude stability (roll, pitch, yaw)
  - Velocity profiles
  - NMPC reference-following quality
  - Phase-by-phase metrics

Usage: python3 analyze_flight.py [bag_path]
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from collections import defaultdict

try:
    import rosbag
except ImportError:
    print("需要 rosbag 库: source /opt/ros/noetic/setup.bash")
    sys.exit(1)

# ========== Config ==========
BAG_PATH = sys.argv[1] if len(sys.argv) > 1 else "/home/flag/Jiangyin2026/2026-07-21-18-43-11.bag"
OUT_DIR = os.path.dirname(os.path.abspath(BAG_PATH))
BAG_NAME = os.path.basename(BAG_PATH)

print(f"Reading: {BAG_PATH}")
bag = rosbag.Bag(BAG_PATH)
t0 = bag.get_start_time()

# ========== Data Extraction ==========
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
    elif topic == '/nmpc_posref':
        data['ref_t'].append(ts)
        data['ref_x'].append(msg.pose.position.x)
        data['ref_y'].append(msg.pose.position.y)
        data['ref_z'].append(msg.pose.position.z)
    elif topic == '/nmpc_posfdb':
        data['fdb_t'].append(ts)
        data['fdb_x'].append(msg.pose.position.x)
        data['fdb_y'].append(msg.pose.position.y)
        data['fdb_z'].append(msg.pose.position.z)
    elif topic == '/mavros/setpoint_position/local':
        data['sp_t'].append(ts)
        data['sp_x'].append(msg.pose.position.x)
        data['sp_y'].append(msg.pose.position.y)
        data['sp_z'].append(msg.pose.position.z)

bag.close()

# Convert to numpy
for k in list(data.keys()):
    data[k] = np.array(data[k])

print(f"Odom msgs: {len(data['t']):d}, duration: {data['t'][-1]:.1f}s")
print(f"NMPC ref msgs: {len(data['ref_t']):d}")
print(f"NMPC fdb msgs: {len(data['fdb_t']):d}")

# ========== Quaternion → Euler ==========
def quat_to_euler(qw, qx, qy, qz):
    roll = np.arctan2(2.0*(qw*qx + qy*qz), 1.0 - 2.0*(qx*qx + qy*qy))
    sinp = 2.0*(qw*qy - qz*qx)
    pitch = np.where(np.abs(sinp) >= 1, np.copysign(np.pi/2, sinp), np.arcsin(sinp))
    yaw = np.arctan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
    return np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw)

roll, pitch, yaw = quat_to_euler(data['qw'], data['qx'], data['qy'], data['qz'])

# ========== Detect Flight Phases ==========
print("\n" + "="*70)
print("                    FLIGHT PHASE DETECTION")
print("="*70)

# Phase 0: Ground idle (Z < 0.1m)
# Phase 1: Takeoff (Z: 0.1 → 0.95m)
# Phase 2: Hover at 1m (before waypoint, ref=(0,0,1))
# Phase 3: X waypoint leg (X: 0→1, Y≈0)
# Phase 4: Y waypoint leg (Y: 0→1, X≈1)
# Phase 5: Hold at (1,1,1)
# Phase 6: Landing

pz = data['pz']
t = data['t']

# Ground idle
ground_mask = pz < 0.1
ground_end_idx = np.where(pz > 0.15)[0]
t_takeoff_start = t[ground_end_idx[0]] if len(ground_end_idx) > 0 else 0

# Reach hover
hover_reach_idx = np.where(pz > 0.95)[0]
t_hover_reach = t[hover_reach_idx[0]] if len(hover_reach_idx) > 0 else t[-1]

# Waypoint phases from NMPC ref
if len(data['ref_t']) > 0:
    ref_x, ref_y = data['ref_x'], data['ref_y']
    ref_t = data['ref_t']

    # X leg
    x_move = np.where(ref_x > 0.01)[0]
    t_x_start = ref_t[x_move[0]] if len(x_move) > 0 else None
    x_done = np.where(ref_x >= 0.99)[0]
    t_x_end = ref_t[x_done[0]] if len(x_done) > 0 else None

    # Y leg
    y_move = np.where(ref_y > 0.01)[0]
    t_y_start = ref_t[y_move[0]] if len(y_move) > 0 else None
    y_done = np.where(ref_y >= 0.99)[0]
    t_y_end = ref_t[y_done[0]] if len(y_done) > 0 else None

    ref_appears = ref_t[0]
else:
    t_x_start = t_x_end = t_y_start = t_y_end = ref_appears = None

# Landing
hover_mask_late = pz > 0.9
hover_late_idx = np.where(hover_mask_late)[0]
if len(hover_late_idx) > 0:
    last_hover_idx = hover_late_idx[-1]
    descent_idx = np.where(pz[last_hover_idx:] < 0.8)[0]
    t_descent_start = t[last_hover_idx + descent_idx[0]] if len(descent_idx) > 0 else None
else:
    t_descent_start = None

print(f"Phase 0 - Ground idle:      t=0.0s ~ t={t_takeoff_start:.1f}s")
print(f"Phase 1 - Takeoff:          t={t_takeoff_start:.1f}s ~ t={t_hover_reach:.1f}s ({(t_hover_reach-t_takeoff_start):.2f}s)")
print(f"Phase 2 - Hover @1m:        t={t_hover_reach:.1f}s ~ t={ref_appears:.1f}s")
if t_x_start:
    print(f"Phase 3 - X waypoint leg:   t={t_x_start:.1f}s ~ t={t_x_end:.1f}s ({(t_x_end-t_x_start):.2f}s, ref X: 0→1)")
if t_y_start:
    print(f"Phase 4 - Y waypoint leg:   t={t_y_start:.1f}s ~ t={t_y_end:.1f}s ({(t_y_end-t_y_start):.2f}s, ref Y: 0→1)")
if t_y_end:
    print(f"Phase 5 - Hold @(1,1,1):    t={t_y_end:.1f}s ~ t={t_descent_start:.1f}s" if t_descent_start else f"Phase 5 - Hold: t={t_y_end:.1f}s ~ end")
if t_descent_start:
    print(f"Phase 6 - Landing:          t={t_descent_start:.1f}s ~ t={t[-1]:.1f}s")

# ========== Align NMPC ref to odom time ==========
# Interpolate NMPC ref onto odom timeline for error calculation
if len(data['ref_t']) > 2:
    ref_x_on_odom = np.interp(t, data['ref_t'], data['ref_x'], left=np.nan, right=np.nan)
    ref_y_on_odom = np.interp(t, data['ref_t'], data['ref_y'], left=np.nan, right=np.nan)
    ref_z_on_odom = np.interp(t, data['ref_t'], data['ref_z'], left=np.nan, right=np.nan)
    # Mask out times before ref appeared
    valid_ref = t >= data['ref_t'][0]
else:
    ref_x_on_odom = ref_y_on_odom = ref_z_on_odom = None
    valid_ref = np.zeros_like(t, dtype=bool)

# ========== Key Metrics ==========
print("\n" + "="*70)
print("                    KEY PERFORMANCE METRICS")
print("="*70)

# --- Z-Axis Takeoff ---
takeoff_mask = (t >= t_takeoff_start) & (t <= t_hover_reach)
if np.sum(takeoff_mask) > 5:
    print(f"\n--- Takeoff Phase (t={t_takeoff_start:.1f}s ~ {t_hover_reach:.1f}s) ---")
    print(f"  Duration: {t_hover_reach - t_takeoff_start:.2f}s")
    print(f"  Max Vz: {np.max(data['vz'][takeoff_mask]):.3f} m/s")
    print(f"  Avg Vz (climb): {np.mean(data['vz'][takeoff_mask]):.3f} m/s")
    acc_z_takeoff = np.gradient(data['vz'], t)
    print(f"  Max Az (up): {np.max(acc_z_takeoff[takeoff_mask]):.3f} m/s²")
    print(f"  Min Az (down): {np.min(acc_z_takeoff[takeoff_mask]):.3f} m/s²")

# --- Hover Stability ---
hover_pre_wp = (t >= t_hover_reach + 0.5)  # 0.5s settling time after reaching hover
if ref_appears is not None:
    hover_pre_wp &= (t < ref_appears)
if np.sum(hover_pre_wp) > 20:
    print(f"\n--- Pre-waypoint Hover Stability (t={t_hover_reach+0.5:.1f}s ~ t={ref_appears:.1f}s) ---")
    print(f"  Z mean: {np.mean(pz[hover_pre_wp]):.4f} m")
    print(f"  Z std:  {np.std(pz[hover_pre_wp]):.4f} m")
    print(f"  Z RMS error vs 1.0m: {np.sqrt(np.mean((pz[hover_pre_wp] - 1.0)**2)):.4f} m")
    print(f"  X std:  {np.std(data['px'][hover_pre_wp]):.4f} m")
    print(f"  Y std:  {np.std(data['py'][hover_pre_wp]):.4f} m")
    print(f"  Roll mean:  {np.mean(roll[hover_pre_wp]):.2f}°, std: {np.std(roll[hover_pre_wp]):.2f}°")
    print(f"  Pitch mean: {np.mean(pitch[hover_pre_wp]):.2f}°, std: {np.std(pitch[hover_pre_wp]):.2f}°")

# --- X Waypoint Leg ---
if t_x_start and t_x_end:
    x_leg_mask = (t >= t_x_start) & (t <= t_x_end + 1.0)  # +1s settling
    if np.sum(x_leg_mask) > 20:
        print(f"\n--- X Waypoint Leg (t={t_x_start:.1f}s ~ {t_x_end:.1f}s) ---")
        print(f"  Duration: {t_x_end - t_x_start:.2f}s")
        print(f"  Ref X speed: {(ref_x[x_done[0]] - ref_x[x_move[0]]) / (t_x_end - t_x_start):.3f} m/s")
        # Tracking error during X motion
        x_err = data['px'][x_leg_mask] - ref_x_on_odom[x_leg_mask]
        x_err_valid = ~np.isnan(x_err)
        print(f"  X tracking RMS error: {np.sqrt(np.mean(x_err[x_err_valid]**2)):.4f} m")
        print(f"  X max tracking error: {np.max(np.abs(x_err[x_err_valid])):.4f} m")
        print(f"  Y deviation RMS: {np.sqrt(np.mean((data['py'][x_leg_mask] - 0)**2)):.4f} m")
        print(f"  Y max deviation: {np.max(np.abs(data['py'][x_leg_mask])):.4f} m")
        print(f"  Z deviation RMS: {np.sqrt(np.mean((pz[x_leg_mask] - 1.0)**2)):.4f} m")
        # Max velocity
        print(f"  Max Vx: {np.max(data['vx'][x_leg_mask]):.3f} m/s")
        print(f"  Max pitch during X leg: {np.max(np.abs(pitch[x_leg_mask])):.2f}°")

# --- Y Waypoint Leg ---
if t_y_start and t_y_end:
    y_leg_mask = (t >= t_y_start) & (t <= t_y_end + 1.0)
    if np.sum(y_leg_mask) > 20:
        print(f"\n--- Y Waypoint Leg (t={t_y_start:.1f}s ~ {t_y_end:.1f}s) ---")
        print(f"  Duration: {t_y_end - t_y_start:.2f}s")
        print(f"  Ref Y speed: {(ref_y[y_done[0]] - ref_y[y_move[0]]) / (t_y_end - t_y_start):.3f} m/s")
        y_err = data['py'][y_leg_mask] - ref_y_on_odom[y_leg_mask]
        y_err_valid = ~np.isnan(y_err)
        print(f"  Y tracking RMS error: {np.sqrt(np.mean(y_err[y_err_valid]**2)):.4f} m")
        print(f"  Y max tracking error: {np.max(np.abs(y_err[y_err_valid])):.4f} m")
        x_dev_mask = y_leg_mask & ~np.isnan(ref_x_on_odom)
        if np.sum(x_dev_mask) > 5:
            print(f"  X deviation RMS: {np.sqrt(np.mean((data['px'][x_dev_mask] - 1.0)**2)):.4f} m")
        print(f"  Z deviation RMS: {np.sqrt(np.mean((pz[y_leg_mask] - 1.0)**2)):.4f} m")
        print(f"  Max Vy: {np.max(data['vy'][y_leg_mask]):.3f} m/s")
        print(f"  Max roll during Y leg: {np.max(np.abs(roll[y_leg_mask])):.2f}°")

# --- Hold Phase ---
if t_y_end and t_descent_start:
    hold_mask = (t >= t_y_end + 1.0) & (t <= t_descent_start)
elif t_y_end:
    hold_mask = (t >= t_y_end + 1.0)
else:
    hold_mask = np.zeros_like(t, dtype=bool)

if np.sum(hold_mask) > 50:
    print(f"\n--- Hold @(1,1,1) Phase (n={np.sum(hold_mask)} pts) ---")
    print(f"  X mean: {np.mean(data['px'][hold_mask]):.4f} m, std: {np.std(data['px'][hold_mask]):.4f} m")
    print(f"  Y mean: {np.mean(data['py'][hold_mask]):.4f} m, std: {np.std(data['py'][hold_mask]):.4f} m")
    print(f"  Z mean: {np.mean(pz[hold_mask]):.4f} m, std: {np.std(pz[hold_mask]):.4f} m")
    print(f"  X range: [{np.min(data['px'][hold_mask]):.3f}, {np.max(data['px'][hold_mask]):.3f}]")
    print(f"  Y range: [{np.min(data['py'][hold_mask]):.3f}, {np.max(data['py'][hold_mask]):.3f}]")
    print(f"  Z mean: {np.mean(pz[hold_mask]):.4f} m, std: {np.std(pz[hold_mask]):.4f} m")
    print(f"  Position hold RMS: {np.sqrt(np.mean((data['px'][hold_mask]-1)**2 + (data['py'][hold_mask]-1)**2 + (pz[hold_mask]-1)**2)):.4f} m")
    print(f"  Roll std: {np.std(roll[hold_mask]):.2f}°")
    print(f"  Pitch std: {np.std(pitch[hold_mask]):.2f}°")

# --- Overall NMPC Tracking ---
if ref_x_on_odom is not None:
    ref_valid = valid_ref & (t >= ref_appears)
    if t_descent_start:
        ref_valid &= (t <= t_descent_start)
    if np.sum(ref_valid) > 100:
        x_err_v = data['px'][ref_valid] - ref_x_on_odom[ref_valid]
        y_err_v = data['py'][ref_valid] - ref_y_on_odom[ref_valid]
        z_err_v = pz[ref_valid] - ref_z_on_odom[ref_valid]
        # Remove any remaining NaN
        good = ~np.isnan(x_err_v) & ~np.isnan(y_err_v) & ~np.isnan(z_err_v)
        x_err_v, y_err_v, z_err_v = x_err_v[good], y_err_v[good], z_err_v[good]
        total_pos_err = np.sqrt(x_err_v**2 + y_err_v**2 + z_err_v**2)
        print(f"\n--- Overall NMPC Tracking (t={t[ref_valid][0]:.1f}s ~ {t[ref_valid][-1]:.1f}s) ---")
        print(f"  3D RMS tracking error: {np.sqrt(np.mean(total_pos_err**2)):.4f} m")
        print(f"  3D max tracking error: {np.max(total_pos_err):.4f} m")
        print(f"  Mean abs X error: {np.mean(np.abs(x_err_v)):.4f} m")
        print(f"  Mean abs Y error: {np.mean(np.abs(y_err_v)):.4f} m")
        print(f"  Mean abs Z error: {np.mean(np.abs(z_err_v)):.4f} m")

# --- Landing ---
if t_descent_start:
    land_mask = (t >= t_descent_start)
    if np.sum(land_mask) > 10:
        print(f"\n--- Landing Phase (t={t_descent_start:.1f}s ~ end) ---")
        print(f"  Duration: {t[-1] - t_descent_start:.2f}s")
        print(f"  Min Vz (descent): {np.min(data['vz'][land_mask]):.3f} m/s")
        print(f"  Final Z: {pz[-1]:.4f} m")
        print(f"  Touchdown Z speed: {data['vz'][-1]:.3f} m/s")

# ========== Attitude Analysis ==========
print(f"\n--- Full Flight Attitude ---")
print(f"  Max |Roll|:  {np.max(np.abs(roll)):.2f}°")
print(f"  Max |Pitch|: {np.max(np.abs(pitch)):.2f}°")
print(f"  Roll std:    {np.std(roll):.2f}°")
print(f"  Pitch std:   {np.std(pitch):.2f}°")
print(f"  Yaw range:   [{np.min(yaw):.1f}°, {np.max(yaw):.1f}°]")

# ========== Generate Plots ==========
print("\n" + "="*70)
print("                    GENERATING PLOTS")
print("="*70)

fig = plt.figure(figsize=(24, 22))
gs = GridSpec(6, 3, figure=fig, hspace=0.40, wspace=0.35)

# Find plot range: from takeoff to landing
t_plot_start = max(0, t_takeoff_start - 2)
t_plot_end = min(t[-1], (t_descent_start + 3) if t_descent_start else t[-1])
plot_mask = (t >= t_plot_start) & (t <= t_plot_end)
t_plot = t[plot_mask]

# Helper to shade phases
def shade_phases(ax, ymin, ymax):
    """Shade flight phases on an axis"""
    colors = ['#e8f5e9', '#fff3e0', '#e3f2fd', '#fce4ec', '#f3e5f5']
    labels = ['Takeoff', 'Hover', 'X leg', 'Y leg', 'Hold']
    phases = [
        (t_takeoff_start, t_hover_reach),
        (t_hover_reach, t_x_start if t_x_start else t_hover_reach),
        (t_x_start, t_x_end) if t_x_start else None,
        (t_y_start, t_y_end) if t_y_start else None,
        (t_y_end, t_descent_start if t_descent_start else t_plot_end) if t_y_end else None,
    ]
    for i, p in enumerate(phases):
        if p is None:
            continue
        ax.axvspan(p[0], p[1], alpha=0.15, color=colors[i % len(colors)], label=labels[i] if i < len(labels) else None)

# ---- Panel 1: XZ Trajectory (side view) ----
ax1 = fig.add_subplot(gs[0:2, 0])
ax1.plot(data['px'][plot_mask], pz[plot_mask],
         linewidth=0.6, color='steelblue', alpha=0.8)
ax1.scatter([0], [1.0], c='orange', s=80, marker='*', label='WP1 (0,0,1)')
ax1.scatter([1.0], [1.0], c='red', s=80, marker='*', label='WP2 (1,1,1)')
ax1.scatter([data['px'][0]], [pz[0]], c='green', s=60, marker='o', label='Start')
ax1.scatter([data['px'][-1]], [pz[-1]], c='purple', s=60, marker='s', label='End')
ax1.set_xlabel('X [m]')
ax1.set_ylabel('Z [m]')
ax1.set_title('XZ Trajectory (Side View)')
ax1.legend(loc='best', fontsize=7)
ax1.grid(True, alpha=0.3)

# ---- Panel 2: X Position vs Time (with NMPC ref) ----
ax2 = fig.add_subplot(gs[0, 1])
ax2.plot(t_plot, data['px'][plot_mask], 'b-', linewidth=1.0, label='Actual X')
if ref_x_on_odom is not None:
    ax2.plot(t_plot, ref_x_on_odom[plot_mask], 'r--', linewidth=1.0, alpha=0.8, label='NMPC Ref X')
shade_phases(ax2, -0.5, 1.5)
ax2.set_ylabel('X [m]')
ax2.set_title('X Position vs Time')
ax2.legend(loc='best', fontsize=7)
ax2.grid(True, alpha=0.3)
ax2.set_ylim(-0.3, 1.5)

# ---- Panel 3: Y Position vs Time (with NMPC ref) ----
ax3 = fig.add_subplot(gs[0, 2])
ax3.plot(t_plot, data['py'][plot_mask], 'b-', linewidth=1.0, label='Actual Y')
if ref_y_on_odom is not None:
    ax3.plot(t_plot, ref_y_on_odom[plot_mask], 'r--', linewidth=1.0, alpha=0.8, label='NMPC Ref Y')
shade_phases(ax3, -0.5, 1.5)
ax3.set_ylabel('Y [m]')
ax3.set_title('Y Position vs Time')
ax3.legend(loc='best', fontsize=7)
ax3.grid(True, alpha=0.3)
ax3.set_ylim(-0.5, 1.5)

# ---- Panel 4: Z Position vs Time ----
ax4 = fig.add_subplot(gs[1, 1])
ax4.plot(t_plot, pz[plot_mask], 'b-', linewidth=1.0, label='Actual Z')
ax4.axhline(y=1.0, color='g', linestyle=':', alpha=0.7, label='Target 1.0m')
shade_phases(ax4, -0.1, 1.3)
ax4.set_ylabel('Z [m]')
ax4.set_title('Z Position vs Time')
ax4.legend(loc='best', fontsize=7)
ax4.grid(True, alpha=0.3)

# ---- Panel 5: X Tracking Error ----
ax5 = fig.add_subplot(gs[1, 2])
if ref_x_on_odom is not None:
    x_err_plot = data['px'] - ref_x_on_odom
    ax5.plot(t_plot, x_err_plot[plot_mask], 'r-', linewidth=0.8, label='X tracking error')
ax5.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
ax5.fill_between(t_plot, -0.05, 0.05, alpha=0.1, color='green', label='±5cm band')
shade_phases(ax5, -0.3, 0.3)
ax5.set_ylabel('X Error [m]')
ax5.set_title('X Position Tracking Error (Actual - Ref)')
ax5.legend(loc='best', fontsize=7)
ax5.grid(True, alpha=0.3)

# ---- Panel 6: Y Tracking Error ----
ax6 = fig.add_subplot(gs[2, 0])
if ref_y_on_odom is not None:
    y_err_plot = data['py'] - ref_y_on_odom
    ax6.plot(t_plot, y_err_plot[plot_mask], 'r-', linewidth=0.8, label='Y tracking error')
ax6.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
ax6.fill_between(t_plot, -0.05, 0.05, alpha=0.1, color='green', label='±5cm band')
shade_phases(ax6, -0.3, 0.3)
ax6.set_ylabel('Y Error [m]')
ax6.set_title('Y Position Tracking Error (Actual - Ref)')
ax6.legend(loc='best', fontsize=7)
ax6.grid(True, alpha=0.3)

# ---- Panel 7: 3D Tracking Error (combined) ----
ax7 = fig.add_subplot(gs[2, 1])
if ref_x_on_odom is not None:
    pos_err_3d = np.sqrt(x_err_plot**2 + y_err_plot**2 + (pz - ref_z_on_odom)**2)
    ax7.plot(t_plot, pos_err_3d[plot_mask], 'purple', linewidth=1.0, label='3D position error')
ax7.axhline(y=0.05, color='orange', linestyle=':', alpha=0.7, label='5cm threshold')
ax7.axhline(y=0.10, color='red', linestyle=':', alpha=0.7, label='10cm threshold')
shade_phases(ax7, -0.05, 0.5)
ax7.set_ylabel('3D Error [m]')
ax7.set_title('3D Position Tracking Error')
ax7.legend(loc='best', fontsize=7)
ax7.grid(True, alpha=0.3)

# ---- Panel 8: Velocity X ----
ax8 = fig.add_subplot(gs[2, 2])
ax8.plot(t_plot, data['vx'][plot_mask], 'b-', linewidth=0.8, label='Vx')
ax8.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
shade_phases(ax8, -1.0, 1.0)
ax8.set_ylabel('Vx [m/s]')
ax8.set_title('X Velocity vs Time')
ax8.legend(loc='best', fontsize=7)
ax8.grid(True, alpha=0.3)

# ---- Panel 9: Velocity Y ----
ax9 = fig.add_subplot(gs[3, 0])
ax9.plot(t_plot, data['vy'][plot_mask], 'b-', linewidth=0.8, label='Vy')
ax9.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
shade_phases(ax9, -1.0, 1.0)
ax9.set_ylabel('Vy [m/s]')
ax9.set_title('Y Velocity vs Time')
ax9.legend(loc='best', fontsize=7)
ax9.grid(True, alpha=0.3)

# ---- Panel 10: Velocity Z ----
ax10 = fig.add_subplot(gs[3, 1])
ax10.plot(t_plot, data['vz'][plot_mask], 'b-', linewidth=0.8, label='Vz')
ax10.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
shade_phases(ax10, -1.0, 1.0)
ax10.set_ylabel('Vz [m/s]')
ax10.set_title('Z Velocity vs Time')
ax10.legend(loc='best', fontsize=7)
ax10.grid(True, alpha=0.3)

# ---- Panel 11: Attitude (Roll, Pitch) ----
ax11 = fig.add_subplot(gs[3, 2])
ax11.plot(t_plot, roll[plot_mask], 'b-', linewidth=0.8, alpha=0.8, label='Roll')
ax11.plot(t_plot, pitch[plot_mask], 'r-', linewidth=0.8, alpha=0.8, label='Pitch')
ax11.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
shade_phases(ax11, -20, 20)
ax11.set_ylabel('Angle [°]')
ax11.set_title('Attitude (Roll, Pitch) vs Time')
ax11.legend(loc='best', fontsize=7)
ax11.grid(True, alpha=0.3)

# ---- Panel 12: XY Trajectory (Top-down) ----
ax12 = fig.add_subplot(gs[4, 0])
sc12 = ax12.scatter(data['px'][plot_mask], data['py'][plot_mask], c=t_plot, s=3,
                    cmap='plasma', alpha=0.7)
ax12.plot([0, 0, 1, 1], [0, 1, 1, 0], 'g--', linewidth=1.0, alpha=0.5, label='Reference path')
ax12.scatter([0], [0], c='green', s=60, marker='o', label='Start')
ax12.scatter([1], [1], c='red', s=60, marker='*', label='Target')
ax12.axis('equal')
ax12.set_xlabel('X [m]')
ax12.set_ylabel('Y [m]')
ax12.set_title('XY Trajectory (Top-down)')
ax12.legend(loc='best', fontsize=7)
ax12.grid(True, alpha=0.3)
plt.colorbar(sc12, ax=ax12, label='Time [s]')

# ---- Panel 13: Pitch vs Vx (coupling) ----
ax13 = fig.add_subplot(gs[4, 1])
# Only during X leg
if t_x_start and t_x_end:
    xleg = (t >= t_x_start) & (t <= t_x_end + 1.0)
    sc13 = ax13.scatter(pitch[xleg], data['vx'][xleg], c=t[xleg], s=5,
                        cmap='viridis', alpha=0.6)
    ax13.set_xlabel('Pitch [°]')
    ax13.set_ylabel('Vx [m/s]')
    ax13.set_title('Pitch-Vx Coupling (X leg)')
    ax13.grid(True, alpha=0.3)
    plt.colorbar(sc13, ax=ax13, label='Time [s]')

# ---- Panel 14: Roll vs Vy (coupling) ----
ax14 = fig.add_subplot(gs[4, 2])
if t_y_start and t_y_end:
    yleg = (t >= t_y_start) & (t <= t_y_end + 1.0)
    sc14 = ax14.scatter(roll[yleg], data['vy'][yleg], c=t[yleg], s=5,
                        cmap='viridis', alpha=0.6)
    ax14.set_xlabel('Roll [°]')
    ax14.set_ylabel('Vy [m/s]')
    ax14.set_title('Roll-Vy Coupling (Y leg)')
    ax14.grid(True, alpha=0.3)
    plt.colorbar(sc14, ax=ax14, label='Time [s]')

# ---- Panel 15: Z-Axis detail (takeoff hover landing) ----
ax15 = fig.add_subplot(gs[5, 0])
ax15.plot(t_plot, pz[plot_mask], 'b-', linewidth=1.0)
ax15.axhline(y=1.0, color='g', linestyle=':', alpha=0.7, label='1.0m target')
shade_phases(ax15, -0.1, 1.3)
ax15.set_xlabel('Time [s]')
ax15.set_ylabel('Z [m]')
ax15.set_title('Full Z Profile (Takeoff → Hover → Landing)')
ax15.legend(loc='best', fontsize=7)
ax15.grid(True, alpha=0.3)

# ---- Panel 16: 3D Error histogram ----
ax16 = fig.add_subplot(gs[5, 1])
if ref_x_on_odom is not None:
    valid = valid_ref & (t >= ref_appears)
    if np.sum(valid) > 50:
        err_hist = pos_err_3d[valid]
        ax16.hist(err_hist, bins=50, color='steelblue', alpha=0.7, edgecolor='navy', linewidth=0.3)
        ax16.axvline(x=np.mean(err_hist), color='red', linestyle='--', linewidth=1.5, label=f'Mean={np.mean(err_hist):.3f}m')
        ax16.axvline(x=np.sqrt(np.mean(err_hist**2)), color='orange', linestyle='--', linewidth=1.5, label=f'RMS={np.sqrt(np.mean(err_hist**2)):.3f}m')
        ax16.set_xlabel('3D Position Error [m]')
        ax16.set_ylabel('Count')
        ax16.set_title('Distribution of 3D Tracking Error')
        ax16.legend(loc='best', fontsize=7)
        ax16.grid(True, alpha=0.3)

# ---- Panel 17: Speed profile ----
ax17 = fig.add_subplot(gs[5, 2])
speed = np.sqrt(data['vx']**2 + data['vy']**2 + data['vz']**2)
ax17.plot(t_plot, speed[plot_mask], 'darkblue', linewidth=0.8, label='Speed')
shade_phases(ax17, -0.2, 2.0)
ax17.set_xlabel('Time [s]')
ax17.set_ylabel('Speed [m/s]')
ax17.set_title('Total Speed vs Time')
ax17.legend(loc='best', fontsize=7)
ax17.grid(True, alpha=0.3)

fig.suptitle(f'Flight Analysis: cmd=1→3→5 (Hover + Waypoint)\nBag: {BAG_NAME}',
             fontsize=14, fontweight='bold', y=0.99)

out_png = os.path.join(OUT_DIR, 'flight_analysis_cmd135.png')
plt.savefig(out_png, dpi=150, bbox_inches='tight')
print(f"Plot saved: {out_png}")

# ========== Additional Diagnostics ==========
print("\n" + "="*70)
print("                    ADDITIONAL DIAGNOSTICS")
print("="*70)

# Corner negotiation: Check how well the drone transitions from X leg to Y leg
if t_x_end and t_y_start:
    corner = (t >= t_x_end - 0.5) & (t <= t_y_start + 1.0)
    if np.sum(corner) > 30:
        print(f"\n--- Corner Transition (t={t_x_end-0.5:.1f}s ~ {t_y_start+1:.1f}s) ---")
        # Speed through corner
        print(f"  Min speed in corner: {np.min(speed[corner]):.4f} m/s")
        print(f"  Max speed in corner: {np.max(speed[corner]):.4f} m/s")
        # Position error at corner
        if ref_x_on_odom is not None:
            valid_c = corner & ~np.isnan(ref_x_on_odom)
            if np.sum(valid_c) > 10:
                corner_err = np.sqrt(x_err_plot[valid_c]**2 + y_err_plot[valid_c]**2 + (pz[valid_c] - 1.0)**2)
                print(f"  Max 3D error at corner: {np.max(corner_err):.4f} m")
                print(f"  RMS 3D error at corner: {np.sqrt(np.mean(corner_err**2)):.4f} m")

# Overshoot analysis
print(f"\n--- Overshoot Analysis ---")
if t_x_end:
    x_after = (t >= t_x_end) & (t <= t_x_end + 3.0)
    if np.sum(x_after) > 10:
        x_max_after = np.max(data['px'][x_after])
        x_overshoot = x_max_after - 1.0
        print(f"  X max after reaching 1.0: {x_max_after:.4f} m (overshoot: {x_overshoot*100:.1f} cm)")

if t_y_end:
    y_after = (t >= t_y_end) & (t <= t_y_end + 3.0)
    if np.sum(y_after) > 10:
        y_max_after = np.max(data['py'][y_after])
        y_overshoot = y_max_after - 1.0
        print(f"  Y max after reaching 1.0: {y_max_after:.4f} m (overshoot: {y_overshoot*100:.1f} cm)")

# Z overshoot
z_after_takeoff = (t >= t_hover_reach) & (t <= t_hover_reach + 3.0)
if np.sum(z_after_takeoff) > 10:
    z_max_after_to = np.max(pz[z_after_takeoff])
    z_overshoot_to = z_max_after_to - 1.0
    print(f"  Z max after reaching 1.0m: {z_max_after_to:.4f} m (overshoot: {z_overshoot_to*100:.1f} cm)")

# Yaw check
yaw_range = np.max(yaw) - np.min(yaw)
print(f"\n--- Yaw ---")
print(f"  Yaw range: {yaw_range:.1f}°")
print(f"  Yaw min: {np.min(yaw):.1f}°")
print(f"  Yaw max: {np.max(yaw):.1f}°")

# Velocity analysis during waypoint legs
print(f"\n--- Velocity Statistics ---")
if t_x_start and t_x_end:
    x_leg_v = (t >= t_x_start) & (t <= t_x_end)
    if np.sum(x_leg_v) > 10:
        print(f"  X leg - Mean Vx: {np.mean(data['vx'][x_leg_v]):.3f} m/s, Max Vx: {np.max(data['vx'][x_leg_v]):.3f} m/s")
        print(f"  X leg - Mean Vy: {np.mean(np.abs(data['vy'][x_leg_v])):.3f} m/s (crosstrack)")
if t_y_start and t_y_end:
    y_leg_v = (t >= t_y_start) & (t <= t_y_end)
    if np.sum(y_leg_v) > 10:
        print(f"  Y leg - Mean Vy: {np.mean(data['vy'][y_leg_v]):.3f} m/s, Max Vy: {np.max(data['vy'][y_leg_v]):.3f} m/s")
        print(f"  Y leg - Mean Vx: {np.mean(np.abs(data['vx'][y_leg_v])):.3f} m/s (crosstrack)")

# Average acceleration (jerk proxy)
# Calculate jerk as derivative of acceleration
acc_x = np.gradient(data['vx'], t)
acc_y = np.gradient(data['vy'], t)
acc_z = np.gradient(data['vz'], t)
jerk_x = np.gradient(acc_x, t)
jerk_y = np.gradient(acc_y, t)
jerk_z = np.gradient(acc_z, t)
jerk_mag = np.sqrt(jerk_x**2 + jerk_y**2 + jerk_z**2)
print(f"\n--- Jerk (Smoothness) ---")
print(f"  Max jerk magnitude: {np.max(jerk_mag):.1f} m/s³")
print(f"  Mean jerk magnitude: {np.mean(jerk_mag):.1f} m/s³")
if t_x_start and t_x_end:
    x_leg_j = (t >= t_x_start) & (t <= t_x_end)
    print(f"  X leg mean jerk: {np.mean(jerk_mag[x_leg_j]):.1f} m/s³")
if t_y_start and t_y_end:
    y_leg_j = (t >= t_y_start) & (t <= t_y_end)
    print(f"  Y leg mean jerk: {np.mean(jerk_mag[y_leg_j]):.1f} m/s³")

print(f"\n{'='*70}")
print("                    ANALYSIS COMPLETE")
print(f"{'='*70}")
