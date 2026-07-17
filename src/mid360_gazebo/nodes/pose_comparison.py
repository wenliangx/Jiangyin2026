#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math


class PoseComparisonNode:
    def __init__(self):
        rospy.init_node('pose_comparison', anonymous=True)

        self.publish_rate = rospy.get_param('~publish_rate', 10.0)
        self.print_interval = rospy.get_param('~print_interval', 0.5)

        self.last_print_time = 0.0
        self.lidar_odom = None
        self.gt_pose = None

        self.sub_lidar = rospy.Subscriber(
            '/Odometry', Odometry, self.lidar_callback, queue_size=1000
        )
        self.sub_gt = rospy.Subscriber(
            '/ground_truth/state', Odometry, self.gt_callback, queue_size=1000
        )

        self.pub_dx = rospy.Publisher('/pose_comparison/dx', Float64, queue_size=10)
        self.pub_dy = rospy.Publisher('/pose_comparison/dy', Float64, queue_size=10)
        self.pub_dz = rospy.Publisher('/pose_comparison/dz', Float64, queue_size=10)
        self.pub_d_yaw = rospy.Publisher('/pose_comparison/d_yaw', Float64, queue_size=10)
        self.pub_horizontal_error = rospy.Publisher(
            '/pose_comparison/horizontal_error', Float64, queue_size=10
        )
        self.pub_total_error = rospy.Publisher(
            '/pose_comparison/total_error', Float64, queue_size=10
        )

        self.rate = rospy.Rate(self.publish_rate)

    def lidar_callback(self, msg):
        self.lidar_odom = msg

    def gt_callback(self, msg):
        self.gt_pose = msg

    def compute_error(self, odom_msg, gt_msg):
        if odom_msg is None or gt_msg is None:
            return None

        px_lidar = odom_msg.pose.pose.position.x
        py_lidar = odom_msg.pose.pose.position.y
        pz_lidar = odom_msg.pose.pose.position.z

        px_gt = gt_msg.pose.pose.position.x
        py_gt = gt_msg.pose.pose.position.y
        pz_gt = gt_msg.pose.pose.position.z

        qx_lidar = odom_msg.pose.pose.orientation.x
        qy_lidar = odom_msg.pose.pose.orientation.y
        qz_lidar = odom_msg.pose.pose.orientation.z
        qw_lidar = odom_msg.pose.pose.orientation.w

        qx_gt = gt_msg.pose.pose.orientation.x
        qy_gt = gt_msg.pose.pose.orientation.y
        qz_gt = gt_msg.pose.pose.orientation.z
        qw_gt = gt_msg.pose.pose.orientation.w

        yaw_lidar = math.atan2(2 * (qw_lidar * qz_lidar + qx_lidar * qy_lidar),
                               1 - 2 * (qx_lidar * qx_lidar + qy_lidar * qy_lidar))
        yaw_gt = math.atan2(2 * (qw_gt * qz_gt + qx_gt * qy_gt),
                            1 - 2 * (qx_gt * qx_gt + qy_gt * qy_gt))

        # Compute errors
        dx = px_lidar - px_gt
        dy = py_lidar - py_gt
        dz = pz_lidar - pz_gt
        d_yaw = yaw_lidar - yaw_gt

        # Normalize yaw error to [-pi, pi]
        while d_yaw > math.pi:
            d_yaw -= 2 * math.pi
        while d_yaw < -math.pi:
            d_yaw += 2 * math.pi

        horizontal_error = math.sqrt(dx ** 2 + dy ** 2)
        total_error = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

        return {
            'dx': dx, 'dy': dy, 'dz': dz,
            'd_yaw': d_yaw,
            'horizontal_error': horizontal_error,
            'total_error': total_error,
            'px_lidar': px_lidar, 'py_lidar': py_lidar, 'pz_lidar': pz_lidar,
            'px_gt': px_gt, 'py_gt': py_gt, 'pz_gt': pz_gt,
            'yaw_lidar': yaw_lidar, 'yaw_gt': yaw_gt,
        }

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

            if self.lidar_odom is None or self.gt_pose is None:
                continue

            error = self.compute_error(self.lidar_odom, self.gt_pose)
            if error is None:
                continue

            current_time = rospy.get_time()

            # Publish error topics for rqt_plot
            self.pub_dx.publish(Float64(error['dx']))
            self.pub_dy.publish(Float64(error['dy']))
            self.pub_dz.publish(Float64(error['dz']))
            self.pub_d_yaw.publish(Float64(error['d_yaw']))
            self.pub_horizontal_error.publish(Float64(error['horizontal_error']))
            self.pub_total_error.publish(Float64(error['total_error']))

            # Print to terminal at interval
            if current_time - self.last_print_time >= self.print_interval:
                self.last_print_time = current_time
                print("\n" + "=" * 60)
                print(f"Time: {current_time:.3f}")
                print("-" * 60)
                print("LiDAR Pose (Odometry):")
                print(f"  x={error['px_lidar']:+.4f} m, "
                      f"y={error['py_lidar']:+.4f} m, "
                      f"z={error['pz_lidar']:+.4f} m, "
                      f"yaw={math.degrees(error['yaw_lidar']):+.2f}°")
                print("GT   Pose (Ground Truth):")
                print(f"  x={error['px_gt']:+.4f} m, "
                      f"y={error['py_gt']:+.4f} m, "
                      f"z={error['pz_gt']:+.4f} m, "
                      f"yaw={math.degrees(error['yaw_gt']):+.2f}°")
                print("-" * 60)
                print("Error:")
                print(f"  dx={error['dx']:+.4f} m, "
                      f"dy={error['dy']:+.4f} m, "
                      f"dz={error['dz']:+.4f} m")
                print(f"  horizontal_error={error['horizontal_error']:.4f} m")
                print(f"  total_error={error['total_error']:.4f} m")
                print(f"  yaw_error={math.degrees(error['d_yaw']):+.2f}°")
                print("=" * 60)


if __name__ == '__main__':
    try:
        node = PoseComparisonNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
