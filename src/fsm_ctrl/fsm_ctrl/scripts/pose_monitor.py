#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pose_monitor.py — 实时对比多源位姿，用于飞前验证
  订阅:
    /vrpn_client_node/jy0/pose        原始动捕位姿
    /mavros/vision_pose/pose          px4_estimator 转发给飞控的视觉位姿
    /mavros/local_position/pose       PX4 EKF2 融合后的位姿
    /mavros/setpoint_raw/attitude     FSM发出的控制指令(推力+角速度)
  显示:
    终端原地刷新（不滚动），对比三源位置/姿态/推力指令/坐标系
"""

import sys
import rospy
import math
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import AttitudeTarget

# 显示区域固定行数（与 _render 输出严格一致）
DISPLAY_LINES = 35


class PoseMonitor:
    def __init__(self):
        self.mocap_pos = None
        self.mocap_quat = None
        self.vision_pos = None
        self.vision_quat = None
        self.fcu_pos = None
        self.fcu_quat = None
        self.target_wx = None      # FSM发出的角速度指令
        self.target_wy = None
        self.target_wz = None
        self.target_thrust = None  # FSM发出的推力指令 [0,1]

        rospy.Subscriber("/vrpn_client_node/jy0/pose", PoseStamped, self.mocap_cb)
        rospy.Subscriber("/mavros/vision_pose/pose", PoseStamped, self.vision_cb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.fcu_cb)
        rospy.Subscriber("/mavros/setpoint_raw/attitude", AttitudeTarget, self.att_cb)

        self.rate = rospy.Rate(10)

        # 首帧：先打出空白占位，防止之后向上移动时光标越界
        for _ in range(DISPLAY_LINES + 2):
            print()
        self._first = True

    # ---------- callbacks ----------
    def mocap_cb(self, msg):
        self.mocap_pos = msg.pose.position
        self.mocap_quat = msg.pose.orientation

    def vision_cb(self, msg):
        self.vision_pos = msg.pose.position
        self.vision_quat = msg.pose.orientation

    def fcu_cb(self, msg):
        self.fcu_pos = msg.pose.position
        self.fcu_quat = msg.pose.orientation

    def att_cb(self, msg):
        self.target_wx = msg.body_rate.x
        self.target_wy = msg.body_rate.y
        self.target_wz = msg.body_rate.z
        self.target_thrust = msg.thrust
        self._att_type_mask = msg.type_mask  # 0=ATTITUDE, 128=IGNORE_ATTITUDE(只用bodyrate)

    # ---------- helpers ----------
    @staticmethod
    def _euler(q):
        return tft.euler_from_quaternion([q.x, q.y, q.z, q.w])

    @staticmethod
    def _cln(s):
        """行尾清空，防止上一帧残留"""
        return s + "\033[0K"

    def _maybe(self, val, fmt, na="--"):
        return fmt.format(val) if val is not None else na

    # ---------- 核心渲染 ----------
    def _render(self):
        lines = []

        # --- 头部 ---
        lines.append(self._cln("=" * 80))
        lines.append(self._cln("  位姿实时监控 (Pose Monitor) — 飞前坐标系验证"))
        lines.append(self._cln("=" * 80))
        lines.append(self._cln(""))

        # --- 位置 ---
        lines.append(self._cln("┌─ 位置 (Position) ───────────────────────────────────────────────────────────┐"))
        lines.append(self._cln("  {:<18} {:>10} {:>10} {:>10} {:>10}".format(
            "数据源", "X(m)", "Y(m)", "Z(m)", "Yaw(°)")))
        lines.append(self._cln("  " + "-" * 64))

        # Mocap
        if self.mocap_pos and self.mocap_quat:
            _, _, y = self._euler(self.mocap_quat)
            lines.append(self._cln("  {:<18} {:>10.3f} {:>10.3f} {:>10.3f} {:>10.2f}".format(
                "Mocap(原始)", self.mocap_pos.x, self.mocap_pos.y, self.mocap_pos.z, math.degrees(y))))
        else:
            lines.append(self._cln("  {:<18} {:>10} {:>10} {:>10} {:>10}".format(
                "Mocap(原始)", "--", "--", "--", "--")))

        # Vision
        if self.vision_pos and self.vision_quat:
            _, _, y = self._euler(self.vision_quat)
            lines.append(self._cln("  {:<18} {:>10.3f} {:>10.3f} {:>10.3f} {:>10.2f}".format(
                "Vision(PX4输入)", self.vision_pos.x, self.vision_pos.y, self.vision_pos.z, math.degrees(y))))
        else:
            lines.append(self._cln("  {:<18} {:>10} {:>10} {:>10} {:>10}".format(
                "Vision(PX4输入)", "--", "--", "--", "--")))

        # FCU
        if self.fcu_pos and self.fcu_quat:
            _, _, y = self._euler(self.fcu_quat)
            lines.append(self._cln("  {:<18} {:>10.3f} {:>10.3f} {:>10.3f} {:>10.2f}".format(
                "FCU(EKF融合)", self.fcu_pos.x, self.fcu_pos.y, self.fcu_pos.z, math.degrees(y))))
        else:
            lines.append(self._cln("  {:<18} {:>10} {:>10} {:>10} {:>10}".format(
                "FCU(EKF融合)", "--", "--", "--", "--")))

        # 偏差
        lines.append(self._cln("  " + "-" * 64))
        if self.vision_pos and self.fcu_pos and self.vision_quat and self.fcu_quat:
            dx = self.vision_pos.x - self.fcu_pos.x
            dy = self.vision_pos.y - self.fcu_pos.y
            dz = self.vision_pos.z - self.fcu_pos.z
            _, _, yv = self._euler(self.vision_quat)
            _, _, yf = self._euler(self.fcu_quat)
            dyaw = math.degrees(yv - yf)
            ok = "✓ EKF已收敛" if (abs(dx)<0.05 and abs(dy)<0.05 and abs(dz)<0.05 and abs(dyaw)<9.0) else "✗ 未收敛"
            lines.append(self._cln("  {:<18} {:>10.3f} {:>10.3f} {:>10.3f} {:>10.2f}   {}".format(
                "Vision-FCU偏差", dx, dy, dz, dyaw, ok)))
        else:
            lines.append(self._cln("  {:<18} {:>10} {:>10} {:>10} {:>10}".format(
                "Vision-FCU偏差", "--", "--", "--", "--")))

        lines.append(self._cln("└" + "─" * 63 + "┘"))
        lines.append(self._cln(""))

        # --- 姿态 ---
        lines.append(self._cln("┌─ 姿态 (Attitude) ───────────────────────────────────────────────────────────┐"))
        lines.append(self._cln("  {:<18} {:>10} {:>10} {:>10}".format(
            "数据源", "Roll(°)", "Pitch(°)", "Yaw(°)")))
        lines.append(self._cln("  " + "-" * 50))

        for label, q in [("Mocap(原始)", self.mocap_quat),
                         ("Vision(PX4输入)", self.vision_quat),
                         ("FCU(EKF融合)", self.fcu_quat)]:
            if q:
                r, p, y = self._euler(q)
                lines.append(self._cln("  {:<18} {:>10.2f} {:>10.2f} {:>10.2f}".format(
                    label, math.degrees(r), math.degrees(p), math.degrees(y))))
            else:
                lines.append(self._cln("  {:<18} {:>10} {:>10} {:>10}".format(label, "--", "--", "--")))

        lines.append(self._cln("└" + "─" * 49 + "┘"))
        lines.append(self._cln(""))

        # --- 推力 & 角速度指令 (FSM→PX4) ---
        lines.append(self._cln("┌─ 控制指令 (FSM → PX4) ───────────────────────────────────────────────────────┐"))
        lines.append(self._cln("  {:<18} {:>10} {:>10} {:>10} {:>10}".format(
            "指令", "Wx(rad/s)", "Wy(rad/s)", "Wz(rad/s)", "Thrust(%)")))
        lines.append(self._cln("  " + "-" * 64))

        if self.target_thrust is not None:
            tpct = self.target_thrust * 100.0
            # 用颜色提示推力大小
            bar = "█" * int(tpct / 5) + "░" * (20 - int(tpct / 5))
            lines.append(self._cln("  {:<18} {:>10.3f} {:>10.3f} {:>10.3f} {:>9.1f}%  {}".format(
                "FSM输出",
                self.target_wx or 0, self.target_wy or 0, self.target_wz or 0,
                tpct, bar)))
        else:
            lines.append(self._cln("  {:<18} {:>10} {:>10} {:>10} {:>10}".format(
                "FSM输出", "--", "--", "--", "--")))

        # 悬停参考线
        hover_ref = rospy.get_param("/single_offboard_fsm/nmpc_hover_thrust", 0.385) * 100
        lines.append(self._cln("  " + "-" * 64))
        lines.append(self._cln("  悬停参考推力: {:.1f}%   (高于此值=上升, 低于=下降)".format(hover_ref)))

        lines.append(self._cln("└" + "─" * 63 + "┘"))
        lines.append(self._cln(""))

        # --- 坐标系验证提示 ---
        lines.append(self._cln("  └─ 坐标系验证:"))
        lines.append(self._cln("     ✓ 右手定则: X=前, Y=左, Z=上"))
        lines.append(self._cln("     ✓ 向前移动→X增大 | 向左移动→Y增大 | 向上→Z增大"))
        lines.append(self._cln("     ✓ 机头右偏→Yaw增大 | 前倾→Pitch正 | 右滚→Roll正"))

        # 保证行数严格固定，多余用空行填补
        while len(lines) < DISPLAY_LINES:
            lines.append(self._cln(""))

        return "\n".join(lines[:DISPLAY_LINES])

    # ---------- 主循环 ----------
    def run(self):
        while not rospy.is_shutdown():
            # 向上移动光标到预留区域的起始行
            if not self._first:
                sys.stdout.write("\033[{}A".format(DISPLAY_LINES + 1))
            self._first = False

            sys.stdout.write(self._render())
            sys.stdout.write("\n")   # 最后一行后换行（被下一次 \033[A 抵消）
            sys.stdout.flush()

            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("pose_monitor")
    PoseMonitor().run()
