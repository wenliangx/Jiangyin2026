#!/usr/bin/env python3
"""
分析 NMPC 控制回路的 rosbag，回答三个问题：
  1. 飞控什么时候断的？
  2. 为什么断了？
  3. NMPC 求解器算了多久？收敛了吗？
"""

import rosbag
import sys
import numpy as np
from collections import defaultdict

def analyze(bag_path):
    bag = rosbag.Bag(bag_path)

    # ===== 数据结构 =====
    state_events = []       # (t, connected, armed, mode)
    attitude_cmds = []      # (t, thrust, wx, wy, wz, type_mask)
    rosout_msgs = []        # (t, level, msg)
    poses = []              # (t, x, y, z, qw, qx, qy, qz)
    nmpc_cmds = []          # (t, cmd_name, pos, quat, thrust, wx, wy, wz)

    # ===== 读取 =====
    for topic, msg, t in bag.read_messages():
        ts = t.to_sec()

        if topic == "/mavros/state":
            state_events.append((ts, msg.connected, msg.armed, msg.mode))

        elif topic == "/mavros/setpoint_raw/attitude":
            attitude_cmds.append((ts,
                msg.thrust,
                msg.body_rate.x, msg.body_rate.y, msg.body_rate.z,
                msg.type_mask))

        elif topic == "/rosout":
            rosout_msgs.append((ts, msg.level, msg.msg))

        elif topic == "/mavros/local_position/pose":
            poses.append((ts,
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                msg.pose.orientation.w, msg.pose.orientation.x,
                msg.pose.orientation.y, msg.pose.orientation.z))

    bag.close()

    # ===== 确定 NMPC 活跃时间窗口 =====
    nmpc_start = None
    nmpc_end = None
    for ts, level, msg in rosout_msgs:
        if "CMD" in msg and "NMPC" in msg:
            if nmpc_start is None:
                nmpc_start = ts
            nmpc_end = ts

    print("=" * 70)
    print("1. 飞控状态变迁 (state)")
    print("=" * 70)
    prev_connected, prev_armed, prev_mode = None, None, None
    for ts, connected, armed, mode in state_events:
        if connected != prev_connected or armed != prev_armed or mode != prev_mode:
            flag = ""
            if prev_connected is not None and prev_connected and not connected:
                flag = "  <<<<<< FCU/MAVLink 真断联! connected=False"
            elif prev_mode is not None and prev_mode == "OFFBOARD" and mode != "OFFBOARD":
                flag = "  <<<<<< 离开 OFFBOARD 模式! (超时/控制指令中断)"
            elif prev_armed is not None and prev_armed and not armed:
                flag = "  <<<<<< 已 DISARM"
            prev_connected, prev_armed, prev_mode = connected, armed, mode
            if nmpc_start and ts > nmpc_start:
                flag += "  [NMPC活跃期]"
            print(f"  t={ts:14.3f}s  connected={connected}  armed={armed}  mode={mode}{flag}")

    # ===== 找出 NMPC 活跃期的 OFFBOARD armed 段 =====
    active_segments = []
    in_seg = False
    seg_start = None
    for ts, connected, armed, mode in state_events:
        if armed and mode == "OFFBOARD":
            if not in_seg:
                seg_start = ts
                in_seg = True
        else:
            if in_seg:
                active_segments.append((seg_start, ts))
                in_seg = False
    if in_seg:
        active_segments.append((seg_start, None))

    print(f"\n  OFFBOARD+Armed 时间窗: {len(active_segments)} 段")
    for i, (s, e) in enumerate(active_segments):
        dur = (e - s) if e else float('inf')
        if e:
            print(f"    段{i+1}: {s:.3f}s ~ {e:.3f}s  (持续 {dur:.1f}s)")
        else:
            print(f"    段{i+1}: {s:.3f}s ~ (bag结束)  (持续 {dur:.1f}s)")

    # ===== 2. NMPC 求解诊断 =====
    print()
    print("=" * 70)
    print("2. NMPC 求解器性能 (/rosout 中 NMPC 相关日志)")
    print("=" * 70)
    nmpc_logs = [(t, l, m) for t, l, m in rosout_msgs if "NMPC" in m.upper()]
    if not nmpc_logs:
        print("  (无 NMPC 相关日志)")
    else:
        solve_times = []
        costs = []
        for ts, level, msg in nmpc_logs:
            tag = {8: "  <<< ERROR", 4: "  < WARN", 2: ""}.get(level, "")
            print(f"  t={ts:14.3f}s [{level}]{tag} {msg}")
            import re
            m = re.search(r'(\d+\.?\d*)\s*ms', msg)
            if m:
                solve_times.append(float(m.group(1)))
            m2 = re.search(r'cost=(\d+\.?\d*)', msg)
            if m2:
                costs.append(float(m2.group(1)))

        if solve_times:
            print(f"\n  求解时间统计 ({len(solve_times)}次):")
            print(f"    min={min(solve_times):.1f}ms  max={max(solve_times):.1f}ms  "
                  f"mean={np.mean(solve_times):.1f}ms  median={np.median(solve_times):.1f}ms")
            over_20 = sum(1 for t in solve_times if t > 20)
            over_10 = sum(1 for t in solve_times if t > 10)
            if over_20 > 0:
                print(f"    *** 超20ms(回路崩溃): {over_20}/{len(solve_times)} ***")
            if over_10 > 0:
                print(f"    超10ms(余量紧): {over_10}/{len(solve_times)}")
        if costs:
            print(f"  cost 范围: [{min(costs):.4f}, {max(costs):.4f}]")

    # ===== 3. 控制指令分析 =====
    print()
    print("=" * 70)
    print("3. 控制指令分析 (/mavros/setpoint_raw/attitude)")
    print("=" * 70)
    if len(attitude_cmds) < 2:
        print("  (控制指令不足)")
    else:
        times = np.array([c[0] for c in attitude_cmds])
        intervals = np.diff(times) * 1000  # ms

        print(f"  控制指令总数: {len(attitude_cmds)}")

        # 全时段统计
        print(f"  全时段: 平均间隔 {np.mean(intervals):.1f}ms  最大间隔 {np.max(intervals):.0f}ms")

        # 逐段分析
        print(f"\n  逐 OFFBOARD+Armed 段分析:")
        for i, (seg_s, seg_e) in enumerate(active_segments):
            seg_end = seg_e if seg_e else max(times)
            seg_mask = (times >= seg_s) & (times <= seg_end)
            seg_times = times[seg_mask]
            if len(seg_times) >= 2:
                seg_intervals = np.diff(seg_times) * 1000
                max_gap = np.max(seg_intervals)
                max_gap_idx = np.argmax(seg_intervals)
                avg_gap = np.mean(seg_intervals)
                end_label = f"{seg_end:.1f}s" if seg_e else "bag结束"
                print(f"    段{i+1} [{seg_s:.1f}s ~ {end_label}, {len(seg_times)}条指令]: "
                      f"平均间隔 {avg_gap:.1f}ms  最大间隔 {max_gap:.0f}ms")
                if max_gap > 100:
                    max_gap_t = seg_times[max_gap_idx]
                    print(f"      *** 间隙 {max_gap:.0f}ms @ t={max_gap_t:.3f}s "
                          f"(offboard timeout 通常 500ms) ***")
                if avg_gap > 30:
                    print(f"      *** 平均间隔 {avg_gap:.0f}ms > 30ms, 控制回路明显卡顿 ***")
            elif len(seg_times) == 1:
                end_label = f"{seg_end:.1f}s" if seg_e else "bag结束"
                print(f"    段{i+1} [{seg_s:.1f}s ~ {end_label}]: 仅1条指令")
            else:
                end_label = f"{seg_end:.1f}s" if seg_e else "bag结束"
                print(f"    段{i+1} [{seg_s:.1f}s ~ {end_label}]: 无控制指令! ***")

        # NMPC 活跃期单独分析
        if nmpc_start and nmpc_end:
            print(f"\n  NMPC 活跃期 [{nmpc_start:.3f}s ~ {nmpc_end:.3f}s] 控制指令:")
            nmpc_att_mask = (times >= nmpc_start) & (times <= nmpc_end)
            nmpc_att = [attitude_cmds[i] for i in range(len(attitude_cmds)) if nmpc_att_mask[i]]
            if nmpc_att:
                nmpc_times = np.array([c[0] for c in nmpc_att])
                nmpc_intervals = np.diff(nmpc_times) * 1000
                print(f"    指令数: {len(nmpc_att)}  平均间隔: {np.mean(nmpc_intervals):.1f}ms  "
                      f"最大间隔: {np.max(nmpc_intervals):.0f}ms")

        # 推力/角速度极值（NMPC 活跃期）
        if nmpc_start:
            nmpc_mask_arr = (times >= nmpc_start)
            thrusts = [attitude_cmds[i][1] for i in range(len(attitude_cmds)) if nmpc_mask_arr[i]]
            wxs = [attitude_cmds[i][2] for i in range(len(attitude_cmds)) if nmpc_mask_arr[i]]
            wys = [attitude_cmds[i][3] for i in range(len(attitude_cmds)) if nmpc_mask_arr[i]]
            wzs = [attitude_cmds[i][4] for i in range(len(attitude_cmds)) if nmpc_mask_arr[i]]
            tt = [attitude_cmds[i][0] for i in range(len(attitude_cmds)) if nmpc_mask_arr[i]]

            if thrusts:
                print(f"\n  NMPC 活跃期控制量:")
                print(f"    推力范围: [{min(thrusts):.4f}, {max(thrusts):.4f}]")
                print(f"    wx 范围: [{min(wxs):.3f}, {max(wxs):.3f}]")
                print(f"    wy 范围: [{min(wys):.3f}, {max(wys):.3f}]")
                print(f"    wz 范围: [{min(wzs):.3f}, {max(wzs):.3f}]")

                abnormal_w = [(tt[i], wxs[i], wys[i], wzs[i])
                              for i in range(len(tt))
                              if abs(wxs[i]) > 1.0 or abs(wys[i]) > 1.0]
                if abnormal_w:
                    print(f"    *** 异常大角速度 (>1 rad/s): {len(abnormal_w)} 次 ***")
                    for t, wx, wy, wz in abnormal_w[:5]:
                        print(f"      t={t:.3f}s  wx={wx:.2f}  wy={wy:.2f}  wz={wz:.2f}")

    # ===== 4. 位姿分析（NMPC 活跃期） =====
    print()
    print("=" * 70)
    print("4. 位姿反馈 (/mavros/local_position/pose)")
    print("=" * 70)
    if nmpc_start and len(poses) >= 2:
        ptimes = np.array([p[0] for p in poses])
        nmpc_pose_mask = ptimes >= nmpc_start
        nmpc_poses = [poses[i] for i in range(len(poses)) if nmpc_pose_mask[i]]

        if nmpc_poses:
            print(f"  NMPC 活跃期位姿样本: {len(nmpc_poses)}")
            zs = [p[3] for p in nmpc_poses]
            qws = [p[4] for p in nmpc_poses]
            print(f"    Z 范围: [{min(zs):.3f}, {max(zs):.3f}]  "
                  f"初始Z={nmpc_poses[0][3]:.3f}  最终Z={nmpc_poses[-1][3]:.3f}")
            print(f"    qw 范围: [{min(qws):.4f}, {max(qws):.4f}]  "
                  f"(越接近1.0姿态越水平)")

            bad_q = [(nmpc_poses[i][0], qws[i])
                     for i in range(len(qws)) if abs(qws[i]) < 0.5]
            if bad_q:
                print(f"    *** 四元数 w < 0.5 (极度倾斜): {len(bad_q)} 次 ***")
                for t, qw in bad_q[:5]:
                    print(f"      t={t:.3f}s  qw={qw:.4f}")
        else:
            print("  (NMPC 活跃期无位姿数据)")
    else:
        print(f"  总位姿样本: {len(poses)}")
        if len(poses) >= 2:
            print(f"  初始: ({poses[0][1]:.2f},{poses[0][2]:.2f},{poses[0][3]:.2f})  "
                  f"最终: ({poses[-1][1]:.2f},{poses[-1][2]:.2f},{poses[-1][3]:.2f})")

    # ===== 5. 回传链路分析 =====
    print()
    print("=" * 70)
    print("5. FCU回传链路")
    print("=" * 70)

    last_state_ts = state_events[-1][0] if state_events else None
    last_state = state_events[-1] if state_events else None
    last_pose_ts = poses[-1][0] if poses else None
    last_att_ts = attitude_cmds[-1][0] if attitude_cmds else None

    if last_state_ts is not None:
        _, last_connected, last_armed, last_mode = last_state
        print(f"  最后 state: t={last_state_ts:.3f}s  connected={last_connected}  armed={last_armed}  mode={last_mode}")
    if last_pose_ts is not None:
        print(f"  最后 pose:  t={last_pose_ts:.3f}s")
    if last_att_ts is not None:
        print(f"  最后 setpoint_raw/attitude: t={last_att_ts:.3f}s")

    telemetry_stop = False
    if last_att_ts is not None and last_pose_ts is not None and (last_att_ts - last_pose_ts) > 0.20:
        telemetry_stop = True
        print(f"  *** FCU回传在 t≈{last_pose_ts:.3f}s 后停止，但控制指令继续发到 {last_att_ts:.3f}s ***")
        if last_state_ts is not None and abs(last_state_ts - last_pose_ts) < 0.1:
            print("  这更像 FCU->MAVROS 回传流中断，而不是 FSM 停发 setpoint。")

    # ===== 6. 总结 =====
    print()
    print("=" * 70)
    print("6. 诊断总结")
    print("=" * 70)

    issues = []

    # 分析状态段
    for i, (s, e) in enumerate(active_segments):
        seg_dur = (e - s) if e else None
        if seg_dur and seg_dur < 1.0:
            issues.append(f"段{i+1} OFFBOARD+Armed 仅 {seg_dur:.1f}s → 太短，offboard 可能立刻超时")

    # 检查每段是否有控制指令
    if len(attitude_cmds) >= 2:
        for i, (seg_s, seg_e) in enumerate(active_segments):
            seg_end = seg_e if seg_e else max(times)
            seg_mask = (times >= seg_s) & (times <= seg_end)
            if np.sum(seg_mask) == 0:
                issues.append(f"段{i+1} OFFBOARD+Armed 期间无任何控制指令 → 必然触发 offboard timeout!")
            elif np.sum(seg_mask) == 1:
                issues.append(f"段{i+1} OFFBOARD+Armed 期间仅1条指令 → 大概率触发 timeout")

    # Check NMPC solve time
    if 'solve_times' in dir() and solve_times:
        if max(solve_times) > 20:
            issues.append(f"NMPC 冷启动 {max(solve_times):.0f}ms → 超出 20ms 回路预算，但在热启动后正常")
        if np.mean(solve_times) > 15:
            issues.append(f"NMPC 平均求解 {np.mean(solve_times):.0f}ms")

    if telemetry_stop:
        issues.append("NMPC开始后 FCU回传流停止，但 setpoint 仍在持续发送 → 故障点在 FCU/MAVROS回传链路一侧")

    if not issues:
        print("  未发现明显问题")
    else:
        for i, issue in enumerate(issues, 1):
            print(f"  {i}. {issue}")

    print()
    print("=" * 70)
    print("6. 关键结论")
    print("=" * 70)
    if nmpc_start and active_segments:
        # 找包住 NMPC 的 OFFBOARD 段
        nmpc_in_offboard = False
        for seg_s, seg_e in active_segments:
            seg_end = seg_e if seg_e else float('inf')
            if seg_s <= nmpc_start <= seg_end:
                nmpc_in_offboard = True
                dur_from_nmpc = seg_end - nmpc_start
                print(f"  NMPC 开始于 OFFBOARD+Armed 段内，距段结束 {dur_from_nmpc:.1f}s")
                break
        if not nmpc_in_offboard:
            print(f"  *** NMPC 开始时飞控已不在 OFFBOARD 模式! ***")
            print(f"  这意味着 NMPC 算出的控制指令 PX4 不会执行。")
    print()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"用法: python3 {sys.argv[0]} <bag_file>")
        sys.exit(1)
    analyze(sys.argv[1])
