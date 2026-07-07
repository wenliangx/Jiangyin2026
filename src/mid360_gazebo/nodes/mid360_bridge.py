#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Bridge node: converts Gazebo MID360 PointCloud2 (/mid360/points)
to Livox CustomMsg format (/livox/lidar).

Gazebo velodyne plugin publishes sensor_msgs/PointCloud2 with fields:
  x, y, z (float32), intensity (float32), ring (uint16)

Livox CustomMsg format:
  header, timebase (uint64), point_num (uint32), lidar_id (uint8),
  rsvd (uint8[3]), points[] where each point has:
    offset_time (uint32 us), x/y/z (float32 m), reflectivity (uint8), tag (uint8), line (uint8)
"""

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from livox_ros_driver.msg import CustomMsg, CustomPoint
from sensor_msgs import point_cloud2
import numpy as np


class Mid360Bridge:
    def __init__(self):
        rospy.init_node("mid360_bridge", anonymous=True)

        self.horizontal_samples = rospy.get_param("~horizontal_samples", 720)
        self.vertical_samples = rospy.get_param("~vertical_samples", 64)
        self.lidar_id = rospy.get_param("~lidar_id", 0)
        self.scan_period = 1.0 / rospy.get_param("~scan_rate", 10.0)

        self.sub = rospy.Subscriber(
            "/mid360/points", PointCloud2, self.cloud_callback, queue_size=1
        )
        self.pub = rospy.Publisher("/livox/lidar", CustomMsg, queue_size=1)

        rospy.loginfo("Mid360Bridge started: /mid360/points -> /livox/lidar")
        rospy.loginfo(
            "  Horizontal samples: %d, Vertical samples: %d, Scan rate: %.1f Hz",
            self.horizontal_samples,
            self.vertical_samples,
            1.0 / self.scan_period,
        )

    def cloud_callback(self, msg):
        custom_msg = CustomMsg()
        custom_msg.header = msg.header
        custom_msg.timebase = int(msg.header.stamp.to_sec() * 1e6)
        custom_msg.lidar_id = self.lidar_id
        custom_msg.rsvd = [0, 0, 0]

        points_list = []
        point_num = 0

        for pt in point_cloud2.read_points(
            msg, field_names=("x", "y", "z", "intensity", "ring"), skip_nans=True
        ):
            x, y, z = float(pt[0]), float(pt[1]), float(pt[2])
            intensity = float(pt[3]) if pt[3] is not None else 0.0
            ring = int(pt[4]) if pt[4] is not None else 0

            # Skip invalid points (NaN, Inf, or out of range)
            if not np.isfinite(x) or not np.isfinite(y) or not np.isfinite(z):
                continue
            if x == 0.0 and y == 0.0 and z == 0.0:
                continue

            # Calculate offset_time based on horizontal angle
            angle = np.arctan2(y, x)
            normalized_angle = (angle + np.pi) / (2.0 * np.pi)
            offset_us = int(normalized_angle * self.scan_period * 1e6)

            reflectivity = min(255, max(0, int(intensity)))
            line = min(63, max(0, ring))
            tag = 0

            pt_msg = CustomPoint()
            pt_msg.offset_time = offset_us
            pt_msg.x = x
            pt_msg.y = y
            pt_msg.z = z
            pt_msg.reflectivity = reflectivity
            pt_msg.tag = tag
            pt_msg.line = line
            points_list.append(pt_msg)
            point_num += 1

        custom_msg.point_num = point_num
        custom_msg.points = points_list
        self.pub.publish(custom_msg)


def main():
    try:
        bridge = Mid360Bridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
