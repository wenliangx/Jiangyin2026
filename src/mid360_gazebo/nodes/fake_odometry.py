#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Quaternion, Point
from std_msgs.msg import Header
import math
import random


class FakeOdometryNode:
    """Simulates RA-LIO odometry output with realistic drift/noise for testing."""

    def __init__(self):
        rospy.init_node('fake_odometry', anonymous=True)

        # Hover position (meters)
        self.hover_x = rospy.get_param('~hover_x', 0.0)
        self.hover_y = rospy.get_param('~hover_y', 0.0)
        self.hover_z = rospy.get_param('~hover_z', 1.0)

        # Drift parameters (simulates LiDAR odometry errors)
        self.position_noise_std = rospy.get_param('~position_noise_std', 0.02)  # 2cm std
        self.yaw_noise_std = rospy.get_param('~yaw_noise_std', 0.01)  # ~0.57 degrees
        self.drift_rate = rospy.get_param('~drift_rate', 0.001)  # m/s drift accumulation

        # Publish rate
        self.publish_rate = rospy.get_param('~publish_rate', 30.0)

        # State tracking
        self.cumulative_drift_x = 0.0
        self.cumulative_drift_y = 0.0
        self.cumulative_drift_z = 0.0
        self.cumulative_yaw_drift = 0.0

        self.pub_odom = rospy.Publisher('/Odometry', Odometry, queue_size=100)
        self.rate = rospy.Rate(self.publish_rate)

    def add_noise(self, std):
        return random.gauss(0, std)

    def run(self):
        while not rospy.is_shutdown():
            # Accumulate small drift over time (simulates LiDAR odometry drift)
            dt = 1.0 / self.publish_rate
            self.cumulative_drift_x += self.add_noise(self.drift_rate * dt)
            self.cumulative_drift_y += self.add_noise(self.drift_rate * dt)
            self.cumulative_drift_z += self.add_noise(self.drift_rate * dt * 0.5)
            self.cumulative_yaw_drift += self.add_noise(self.drift_rate * 0.1 * dt)

            # Current pose with noise + drift
            current_x = self.hover_x + self.cumulative_drift_x + self.add_noise(self.position_noise_std)
            current_y = self.hover_y + self.cumulative_drift_y + self.add_noise(self.position_noise_std)
            current_z = self.hover_z + self.cumulative_drift_z + self.add_noise(self.position_noise_std * 0.5)

            # Yaw stays near zero with small drift
            current_yaw = self.cumulative_yaw_drift + self.add_noise(self.yaw_noise_std)

            # Create odometry message
            odom = Odometry()
            odom.header = Header()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "map"
            odom.child_frame_id = "base_link"

            odom.pose.pose = Pose(
                position=Point(x=current_x, y=current_y, z=current_z),
                orientation=Quaternion(*self.quaternion_from_euler(0, 0, current_yaw))
            )

            odom.twist.twist = Twist()  # Zero velocity (hovering)

            self.pub_odom.publish(odom)
            self.rate.sleep()

    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - \
             math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + \
             math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - \
             math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + \
             math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]


if __name__ == '__main__':
    try:
        node = FakeOdometryNode()
        rospy.loginfo("Fake odometry node started. Hovering at (%.2f, %.2f, %.2f)",
                      node.hover_x, node.hover_y, node.hover_z)
        node.run()
    except rospy.ROSInterruptException:
        pass
