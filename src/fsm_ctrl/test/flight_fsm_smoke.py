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
from uav_vision_msgs.msg import LandingOffset, VisionControl


class FlightFsmSmoke(unittest.TestCase):
    UDP_PORT = 12991

    def setUp(self):
        self._lock = threading.Lock()
        self._positions = []
        self._super_flags = []
        self._reference_positions = []
        self._feedback_positions = []
        self._nmpc_states = []
        self._attitudes = []
        self._vision_controls = []
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
        self._vision_control_sub = rospy.Subscriber(
            "/vision/control", VisionControl,
            self._vision_control_callback, queue_size=10)
        self._rc_pub = rospy.Publisher("/mavros/rc/in", RCIn, queue_size=1)
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

    def _vision_control_callback(self, message):
        with self._lock:
            self._vision_controls.append(
                (message.front_camera_enabled,
                 message.down_camera_enabled))

    def _send_udp_command(self, command):
        payload = str(command).encode("ascii")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
            for _ in range(10):
                udp.sendto(payload, ("127.0.0.1", self.UDP_PORT))
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
            if "/flight_fsm" in rosnode.get_node_names():
                break
            time.sleep(0.05)
        self.assertIn("/flight_fsm", rosnode.get_node_names())
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
            self._vision_controls.clear()

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
        self._select_udp_command(2)

        self.assertTrue(self._wait_for(
            lambda: any(abs(z - 0.5) < 1e-9 and thrust > 0.0
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

    def test_camera_control_heartbeat_and_latch_contract(self):
        self._wait_for_node()

        self._send_udp_command(0)
        self.assertTrue(self._wait_for(
            lambda: self._vision_controls and
            self._vision_controls[-1] == (True, True)))

        self._send_udp_command(3)
        self.assertTrue(self._wait_for(
            lambda: self._vision_controls and
            self._vision_controls[-1] == (True, False)))

        late_controls = []
        late_lock = threading.Lock()

        def late_callback(message):
            with late_lock:
                late_controls.append(
                    (message.front_camera_enabled,
                     message.down_camera_enabled))

        late_subscriber = rospy.Subscriber(
            "/vision/control", VisionControl, late_callback, queue_size=1)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            with late_lock:
                if late_controls:
                    break
            time.sleep(0.02)
        with late_lock:
            self.assertTrue(late_controls)
            self.assertEqual((True, False), late_controls[-1])
        late_subscriber.unregister()

        with self._lock:
            heartbeat_start = len(self._vision_controls)
        self.assertTrue(self._wait_for(
            lambda: len(self._vision_controls) >= heartbeat_start + 3))
        with self._lock:
            heartbeats = self._vision_controls[heartbeat_start:]
        self.assertTrue(heartbeats)
        self.assertTrue(all(control == (True, False)
                            for control in heartbeats))

        with self._lock:
            control_count = len(self._vision_controls)
        self._send_udp_command(4)
        self.assertTrue(self._wait_for(
            lambda: len(self._vision_controls) > control_count and
            self._vision_controls[-1] == (False, True)))

        self._send_udp_command(5)
        self.assertTrue(self._wait_for(
            lambda: self._vision_controls[-1] == (False, False)))

        with self._lock:
            control_count = len(self._vision_controls)
        self._send_udp_command(6)
        self.assertTrue(self._wait_for(
            lambda: len(self._vision_controls) > control_count and
            self._vision_controls[-1] == (False, False)))

        self._send_udp_command(9)
        self.assertTrue(self._wait_for(
            lambda: self._vision_controls[-1] == (False, False)))

    def test_active_mission_high_hover_super_and_landing_contracts(self):
        self._clear_observations()
        self._wait_for_node()

        self._publish_pose(self._local_pose_pub, 0.0, 0.0, 1.0)
        self._send_udp_command(0)
        time.sleep(0.3)
        self._send_udp_command(2)
        self.assertTrue(self._wait_for(
            lambda: any(abs(x) < 1e-9 and
                        abs(y) < 1e-9 and abs(z - 0.5) < 1e-9 and
                        thrust > 0.0
                        for _, x, y, z, thrust in self._nmpc_states)))
        self.assertTrue(self._wait_for(
            lambda: any(thrust > 0.0 for _, _, _, _, thrust in
                        self._attitudes)))

        self._send_udp_command(3)
        self.assertTrue(self._wait_for(
            lambda: len(self._super_flags) >= 1))

        self._send_udp_command(0)
        time.sleep(0.3)
        with self._lock:
            state_before_landing = len(self._nmpc_states)
            att_before_landing = len(self._attitudes)
        self._publish_pose(self._local_pose_pub, 0.2, -0.1, 0.8)
        self._send_udp_command(6)
        deadline = time.monotonic() + 6.0
        saw_landing_output = False
        while time.monotonic() < deadline and not saw_landing_output:
            self._publish_landing_offset()
            with self._lock:
                saw_landing_output = any(
                    abs(x - 0.2) < 1e-6 and abs(y + 0.1) < 1e-6 and
                    0.0 < z < 0.8 and thrust > 0.0
                    for _, x, y, z, thrust in
                    self._nmpc_states[state_before_landing:])
        self.assertTrue(saw_landing_output)
        self.assertTrue(self._wait_for(
            lambda: len(self._attitudes) > att_before_landing and
            any(thrust > 0.0 for _, _, _, _, thrust in
                self._attitudes[att_before_landing:])))


if __name__ == "__main__":
    rospy.init_node("flight_fsm_smoke_test")
    import rostest
    rostest.rosrun("fsm_ctrl", "flight_fsm_smoke",
                   FlightFsmSmoke)
