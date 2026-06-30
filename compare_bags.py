#!/usr/bin/env python3
"""
对比两个 bag：
  - 正常悬停的 bag
  - 翻车/不起飞的 bag
找出关键差异。
"""

import rosbag
import sys
import numpy as np

def load_bag(bag_path, label):
    bag = rosbag.Bag(bag_path)

    data = {
        'label': label,
        'state_events': [],
        'attitude_cmds': [],
        'rosout_msgs': [],
        'poses': [],
    }

    for topic, msg, t in bag.read_messages():
        ts = t.to_sec()
        if topic == "/mavros/state":
            data['state_events'].append((ts, msg.armed, msg.mode))
        elif topic == "/mavros/setpoint_raw/attitude":
            data['attitude_cmds'].append((ts, msg.thrust,
                msg.body_rate.x, msg.body_rate.y, msg.body_rate.z, msg.type_mask))
        elif topic == "/rosout":
            data['rosout_msgs'].append((ts, msg.level, msg.msg))
        elif topic == "/mavros/local_position/pose":
            data['poses'].append((ts,
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                msg.pose.orientation.w, msg.pose.orientation.x,
                msg.pose.orientation.y, msg.pose.orientation.z))
    bag.close()
    return data

def analyze_one(data):
    """返回关键指标"""
    r = {}

    # 找第一个 OFFBOARD+Armed 段
    state = data['state_events']
    offb_start = offb_end = None
    for ts, armed, mode in state:
        if armed and mode == "OFFBOARD":
            if offb_start is None:
                offb_start = ts
            offb_end = ts
        elif offb_start is not None:
            break

    r['offb_start'] = offb_start
    r['offb_end'] = offb_end
    r['offb_dur'] = (offb_end - offb_start) if (offb_start and offb_end) else 0

    # 找第一个 NMPC 日志时间
    nmpc_start = None
    nmpc_cmds = []
    for ts, level, msg in data['rosout_msgs']:
        if "NMPC" in msg.upper() and "CMD" in msg:
            if nmpc_start is None:
                nmpc_start = ts
            # 解析控制指令
            import re
            m_thrust = re.search(r'thrust=([\d.]+)', msg)
            m_wx = re.search(r'wx=([\-\d.]+)', msg)
            m_wy = re.search(r'wy=([\-\d.]+)', msg)
            m_wz = re.search(r'wz=([\-\d.]+)', msg)
            m_qw = re.search(r'quat=\(([\-\d.]+),', msg)
            m_qx = re.search(r'quat=\([^,]+,([\-\d.]+),', msg)
            if m_thrust:
                nmpc_cmds.append((ts,
                    float(m_thrust.group(1)),
                    float(m_wx.group(1)) if m_wx else 0,
                    float(m_wy.group(1)) if m_wy else 0,
                    float(m_wz.group(1)) if m_wz else 0,
                    float(m_qw.group(1)) if m_qw else 1,
                    float(m_qx.group(1)) if m_qx else 0))

    r['nmpc_start'] = nmpc_start
    r['nmpc_cmd_count'] = len(nmpc_cmds)
    if nmpc_cmds:
        r['first_thrust'] = nmpc_cmds[0][1]
        r['first_wx'] = nmpc_cmds[0][2]
        r['first_qw'] = nmpc_cmds[0][4]
        r['first_qx'] = nmpc_cmds[0][5]

    # NMPC 求解时间
    solve_times = []
    for ts, level, msg in data['rosout_msgs']:
        if ("solve:" in msg.lower() or "solve time:" in msg.lower() or "TOOK" in msg):
            m = re.search(r'(\d+\.?\d*)\s*ms', msg)
            if m:
                solve_times.append((ts, float(m.group(1))))
    r['solve_times'] = solve_times

    # NMPC 活跃期位姿
    if nmpc_start and data['poses']:
        ptimes = np.array([p[0] for p in data['poses']])
        nmpc_mask = ptimes >= nmpc_start
        nmpc_poses = [data['poses'][i] for i in range(len(data['poses'])) if nmpc_mask[i]]
        r['pose_count'] = len(nmpc_poses)
        if nmpc_poses:
            r['pose_first_z'] = nmpc_poses[0][3]
            r['pose_last_z'] = nmpc_poses[-1][3]
            r['pose_max_z'] = max(p[3] for p in nmpc_poses)
    else:
        r['pose_count'] = 0

    # NMPC 活跃期控制指令间隔
    if nmpc_start:
        att_times = np.array([c[0] for c in data['attitude_cmds']])
        att_mask = att_times >= nmpc_start
        att_after = att_times[att_mask]
        if len(att_after) >= 2:
            intervals = np.diff(att_after) * 1000
            r['att_avg_ms'] = np.mean(intervals)
            r['att_max_ms'] = np.max(intervals)
            r['att_count'] = len(att_after)

    # 离地时间检测：找 Z > 0.05 的第一个时间
    if data['poses']:
        for ts, x, y, z, qw, qx, qy, qz in data['poses']:
            if z > 0.05:
                r['takeoff_time'] = ts
                break
        if 'takeoff_time' not in r:
            r['takeoff_time'] = None

    return r


def print_comparison(r1, r2):
    print("=" * 70)
    print("对比：FAIL (bag1) vs OK (bag2)")
    print("=" * 70)

    keys = [
        ('offb_dur',        'OFFBOARD+Armed 持续时间'),
        ('nmpc_start',      '首次 NMPC 求解时间'),
        ('nmpc_cmd_count',  'NMPC 日志条数'),
        ('first_thrust',    '第一帧 thrust'),
        ('first_wx',        '第一帧 wx'),
        ('first_qw',        '第一帧 qw (姿态)'),
        ('first_qx',        '第一帧 qx (姿态)'),
        ('pose_count',      'NMPC 活跃期 位姿样本数'),
        ('pose_first_z',    'NMPC 活跃期 初始Z'),
        ('pose_max_z',      'NMPC 活跃期 最大Z'),
        ('takeoff_time',    '首次离地时间 (Z>0.05)'),
    ]

    print(f"\n{'指标':<30s} {'FAIL':>20s} {'OK':>20s} {'差异':>20s}")
    print("-" * 90)
    for key, name in keys:
        v1 = r1.get(key, 'N/A')
        v2 = r2.get(key, 'N/A')

        if isinstance(v1, float) and isinstance(v2, float):
            diff = v2 - v1
            diff_str = f"{diff:+.4f}"
            v1s = f"{v1:.4f}"
            v2s = f"{v2:.4f}"
        elif v1 is None and v2 is None:
            diff_str = "均无"
            v1s = "无"
            v2s = "无"
        elif v1 is None:
            diff_str = "FAIL无"
            v1s = "无"
            v2s = str(v2)
        elif v2 is None:
            diff_str = "OK无"
            v1s = str(v1)
            v2s = "无"
        else:
            diff_str = ""
            v1s = str(v1)
            v2s = str(v2)

        print(f"{name:<30s} {v1s:>20s} {v2s:>20s} {diff_str:>20s}")

    # NMPC 求解时间
    print(f"\n--- NMPC 求解时间 ---")
    st1 = r1.get('solve_times', [])
    st2 = r2.get('solve_times', [])
    if st1:
        t1 = [s[1] for s in st1]
        print(f"  FAIL: {len(t1)}次, min={min(t1):.1f}ms max={max(t1):.1f}ms mean={np.mean(t1):.1f}ms")
    if st2:
        t2 = [s[1] for s in st2]
        print(f"  OK:   {len(t2)}次, min={min(t2):.1f}ms max={max(t2):.1f}ms mean={np.mean(t2):.1f}ms")

    # 控制指令间隔
    print(f"\n--- NMPC 活跃期控制指令间隔 ---")
    for label, r in [('FAIL', r1), ('OK', r2)]:
        if 'att_avg_ms' in r:
            print(f"  {label}: avg={r['att_avg_ms']:.1f}ms max={r['att_max_ms']:.0f}ms "
                  f"({r['att_count']}条)")

    # 关键诊断
    print(f"\n--- 关键诊断 ---")
    issues = []
    if r1.get('pose_count', 0) < 10 and r2.get('pose_count', 0) > 10:
        issues.append("位姿样本差异巨大! FAIL仅{}帧, OK有{}帧 → FAIL时 mavros 几乎不发 /local_position/pose".format(
            r1.get('pose_count', 0), r2.get('pose_count', 0)))
    if r1.get('takeoff_time') is None and r2.get('takeoff_time') is not None:
        issues.append("FAIL 没有离地(Z始终≈0), OK 正常起飞 → FAIL时飞控没有执行 NMPC 指令")
    if r1.get('offb_dur', 0) < 10 and r2.get('offb_dur', 0) > 10:
        issues.append("FAIL OFFBOARD仅{:.1f}s就超时退出, OK持续{:.1f}s → FAIL时 offboard 提前超时导致 disarm".format(
            r1.get('offb_dur', 0), r2.get('offb_dur', 0)))

    for i, iss in enumerate(issues, 1):
        print(f"  {i}. {iss}")

    if not issues:
        print("  需要更多数据来定位差异")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(f"用法: python3 {sys.argv[0]} <fail_bag> <ok_bag>")
        sys.exit(1)

    d1 = load_bag(sys.argv[1], "FAIL")
    d2 = load_bag(sys.argv[2], "OK")

    r1 = analyze_one(d1)
    r2 = analyze_one(d2)

    print_comparison(r1, r2)
