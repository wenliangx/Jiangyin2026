#!/usr/bin/env python3

import socket
import threading
import time
import unittest
import math

import rosnode
import rospy
from fsm_ctrl.msg import nmpc_state
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.msg import RCIn
from super_msgs.msg import Flag as SuperFlag
from traj_utils.msg import Flag as EgoFlag
from uav_vision_msgs.msg import LandingOffset


class SingleOffboardSmlSmoke(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._positions = []
        self._super_flags = []
        self._reference_positions = []
        self._feedback_positions = []
        self._nmpc_states = []
        self._attitudes = []
        self._position_sub = rospy.Subscriber(
            "/mavros/setpoint_position/local", PoseStamped,
            self._position_callback, queue_size=100)
        self._super_flag_sub = rospy.Subscriber(
            "/super/flag_waypoint", SuperFlag,
            self._super_flag_callback, queue_size=100)
        self._reference_sub = rospy.Subscriber(
            "/nmpc_posref", PoseStamped,
            self._reference_callback, queue_size=100)
        self._feedback_sub = rospy.Subscriber(
            "/nmpc_posfdb", PoseStamped,
            self._feedback_callback, queue_size=100)
        self._nmpc_state_sub = rospy.Subscriber(
            "/nmpc_state", nmpc_state,
            self._nmpc_state_callback, queue_size=100)
        self._attitude_sub = rospy.Subscriber(
            "/mavros/setpoint_raw/attitude", AttitudeTarget,
            self._attitude_callback, queue_size=100)
        self._rc_pub = rospy.Publisher("/mavros/rc/in", RCIn, queue_size=1)
        self._planner_pub = rospy.Publisher(
            "/position_cmd_nmpc", EgoFlag, queue_size=1)
        self._local_pose_pub = rospy.Publisher(
            "/mavros/local_position/pose", PoseStamped, queue_size=1)
        self._target_pose_pub = rospy.Publisher(
            "/target_pose", PoseStamped, queue_size=1)
        self._landing_offset_pub = rospy.Publisher(
            "/vision/landing/offset", LandingOffset, queue_size=1)

    def _position_callback(self, message):
        with self._lock:
            self._positions.append((time.monotonic(), message.pose.position.z))

    def _super_flag_callback(self, message):
        with self._lock:
            self._super_flags.append((time.monotonic(), message.id))

    def _reference_callback(self, message):
        with self._lock:
            self._reference_positions.append(
                (time.monotonic(), message.pose.position.x,
                 message.pose.position.y, message.pose.position.z))

    def _feedback_callback(self, message):
        with self._lock:
            self._feedback_positions.append(
                (time.monotonic(), message.pose.position.x,
                 message.pose.position.y, message.pose.position.z))

    def _nmpc_state_callback(self, message):
        with self._lock:
            self._nmpc_states.append(
                (time.monotonic(), message.pos_ref[0].x,
                 message.pos_ref[0].y, message.pos_ref[0].z,
                 message.target.thrust))

    def _attitude_callback(self, message):
        with self._lock:
            self._attitudes.append(
                (time.monotonic(), message.body_rate.x, message.body_rate.y,
                 message.body_rate.z, message.thrust))

    def _send_udp_command(self, command):
        payload = str(command).encode("ascii")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
            for _ in range(10):
                udp.sendto(payload, ("127.0.0.1", 12001))
                time.sleep(0.02)

    def _select_udp_command(self, command):
        # Force a fresh typed-command transition, independent of whatever
        # command the persistent test node held before this assertion window.
        self._send_udp_command(0)
        time.sleep(0.15)
        self._send_udp_command(command)

    def _wait_for_node(self):
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            if "/single_offboard_fsm" in rosnode.get_node_names():
                break
            time.sleep(0.05)
        self.assertIn("/single_offboard_fsm", rosnode.get_node_names())
        time.sleep(0.5)

    def _wait_for(self, predicate, timeout=6.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if predicate():
                    return True
            time.sleep(0.02)
        with self._lock:
            return predicate()

    def _clear_observations(self):
        with self._lock:
            self._positions.clear()
            self._super_flags.clear()
            self._reference_positions.clear()
            self._feedback_positions.clear()
            self._nmpc_states.clear()
            self._attitudes.clear()

    def _publish_planner_z(self, z):
        message = EgoFlag()
        for command in message.cmd:
            command.position.z = z
        for _ in range(5):
            self._planner_pub.publish(message)
            time.sleep(0.02)

    def _publish_planner_position(self, x, y, z):
        message = EgoFlag()
        for command in message.cmd:
            command.position.x = x
            command.position.y = y
            command.position.z = z
        for _ in range(5):
            self._planner_pub.publish(message)
            time.sleep(0.02)

    def _publish_pose(self, publisher, x, y, z):
        message = PoseStamped()
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.position.z = z
        message.pose.orientation.w = 1.0
        for _ in range(20):
            publisher.publish(message)
            time.sleep(0.01)

    def _publish_landing_offset(self, dx=0.0, dy=0.0):
        message = LandingOffset()
        message.header.stamp = rospy.Time.now()
        message.valid = True
        message.dx = dx
        message.dy = dy
        message.tag_count = 5
        for _ in range(10):
            message.header.stamp = rospy.Time.now()
            self._landing_offset_pub.publish(message)
            time.sleep(0.02)

    def _has_finite_attitude_since(self, start_index):
        return any(math.isfinite(wx) and math.isfinite(wy) and
                   math.isfinite(wz) and math.isfinite(thrust)
                   for _, wx, wy, wz, thrust in
                   self._attitudes[start_index:])

    def test_contract_udp_rate_and_short_rc(self):
        self._wait_for_node()

        # A short RC array must be rejected without indexing channels 8/10.
        self._rc_pub.publish(RCIn(channels=[1000]))
        # Flag.cmd is a fixed-size ROS array; publishing its default value
        # exercises all planner indices without relying on external planners.
        self._planner_pub.publish(EgoFlag())

        self._send_udp_command(2)

        self.assertTrue(self._wait_for(
            lambda: any(abs(z - 1.0) < 1e-9 and thrust > 0.0
                        for _, x, y, z, thrust in self._nmpc_states)))
        self.assertTrue(self._wait_for(
            lambda: any(thrust > 0.0 for _, _, _, _, thrust in
                        self._attitudes)))

        start = time.monotonic()
        time.sleep(0.6)
        with self._lock:
            recent = sum(1 for timestamp, _, _, _, _ in self._attitudes
                         if timestamp >= start)
        self.assertGreaterEqual(recent, 20)  # Allows scheduling jitter at 50 Hz.

    def test_segmented_mission_cmd3_cmd4_cmd6_contracts(self):
        self._clear_observations()
        self._wait_for_node()

        self._publish_pose(self._local_pose_pub, 0.0, 0.0, 1.0)
        self._send_udp_command(0)
        time.sleep(0.3)
        self._send_udp_command(3)
        self.assertTrue(self._wait_for(
            lambda: len(self._super_flags) >= 1))
        self.assertTrue(self._wait_for(
            lambda: any(z > 0.0
                        for _, _, _, z in self._reference_positions) and
            len(self._feedback_positions) >= 1))
        self.assertTrue(self._wait_for(
            lambda: any(z > 0.0 and thrust > 0.0
                        for _, _, _, z, thrust in self._nmpc_states)))
        self.assertTrue(self._wait_for(
            lambda: any(thrust > 0.0 for _, _, _, _, thrust in
                        self._attitudes)))

        self._send_udp_command(0)
        time.sleep(0.3)
        with self._lock:
            super_before_cmd4 = len(self._super_flags)
            state_before_cmd4 = len(self._nmpc_states)
        self._send_udp_command(4)
        self.assertTrue(self._wait_for(
            lambda: len(self._super_flags) > super_before_cmd4))
        self.assertTrue(self._wait_for(
            lambda: len(self._nmpc_states) > state_before_cmd4))

        self._send_udp_command(0)
        time.sleep(0.3)
        with self._lock:
            ref_before_cmd6 = len(self._reference_positions)
            state_before_cmd6 = len(self._nmpc_states)
            att_before_cmd6 = len(self._attitudes)
        self._publish_pose(self._local_pose_pub, 0.2, -0.1, 0.8)
        self._publish_landing_offset()
        self._send_udp_command(6)
        deadline = time.monotonic() + 6.0
        saw_landing_output = False
        while time.monotonic() < deadline and not saw_landing_output:
            self._publish_landing_offset()
            with self._lock:
                saw_landing_output = (
                    len(self._reference_positions) > ref_before_cmd6 and
                    len(self._nmpc_states) > state_before_cmd6)
        self.assertTrue(saw_landing_output)
        self.assertTrue(self._wait_for(
            lambda: len(self._attitudes) > att_before_cmd6 and
            any(thrust > 0.0 for _, _, _, _, thrust in
                self._attitudes[att_before_cmd6:])))


if __name__ == "__main__":
    rospy.init_node("single_offboard_sml_smoke_test")
    import rostest
    rostest.rosrun("fsm_ctrl", "single_offboard_sml_smoke",
                   SingleOffboardSmlSmoke)
