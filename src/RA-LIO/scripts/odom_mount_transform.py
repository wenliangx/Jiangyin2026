#!/usr/bin/env python3
import math

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf.transformations as tft


def transform_from_rpy_xyz(roll_deg, pitch_deg, yaw_deg, x, y, z):
    quat = tft.quaternion_from_euler(
        math.radians(roll_deg),
        math.radians(pitch_deg),
        math.radians(yaw_deg),
        axes='sxyz',
    )
    mat = tft.quaternion_matrix(quat)
    mat[0:3, 3] = [x, y, z]
    return mat


def quaternion_from_matrix(mat):
    quat = tft.quaternion_from_matrix(mat)
    norm = math.sqrt(sum(value * value for value in quat))
    if not math.isfinite(norm) or norm < 1e-12:
        return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    quat = [value / norm for value in quat]
    return Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])


class OdomMountTransform:
    def __init__(self):
        self.input_topic = rospy.get_param('~input_topic', '/Odometry')
        self.output_topic = rospy.get_param('~output_topic', '/Odometry_transformed')
        self.output_frame_id = rospy.get_param('~output_frame_id', '')
        self.output_child_frame_id = rospy.get_param('~output_child_frame_id', '')
        self.mode = rospy.get_param('~mode', 'child')

        roll = float(rospy.get_param('~roll_deg', 0.0))
        pitch = float(rospy.get_param('~pitch_deg', 0.0))
        yaw = float(rospy.get_param('~yaw_deg', 0.0))
        x = float(rospy.get_param('~x', 0.0))
        y = float(rospy.get_param('~y', 0.0))
        z = float(rospy.get_param('~z', 0.0))
        self.mount_tf = transform_from_rpy_xyz(roll, pitch, yaw, x, y, z)
        self.mount_tf_inv = tft.inverse_matrix(self.mount_tf)

        if self.mode not in ('child', 'world'):
            raise rospy.ROSException("~mode must be 'child' or 'world'")

        self.pub = rospy.Publisher(self.output_topic, Odometry, queue_size=10)
        self.sub = rospy.Subscriber(self.input_topic, Odometry, self.callback, queue_size=10)
        rospy.loginfo(
            'odom_mount_transform: %s -> %s, mode=%s, rpy=(%.3f %.3f %.3f), xyz=(%.3f %.3f %.3f)',
            self.input_topic, self.output_topic, self.mode, roll, pitch, yaw, x, y, z,
        )

    def callback(self, msg):
        pose = msg.pose.pose
        trans = [pose.position.x, pose.position.y, pose.position.z]
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        odom_tf = tft.quaternion_matrix(quat)
        odom_tf[0:3, 3] = trans

        if self.mode == 'child':
            out_tf = odom_tf.dot(self.mount_tf)
        else:
            out_tf = self.mount_tf.dot(odom_tf).dot(self.mount_tf_inv)

        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        if self.output_frame_id:
            out.header.frame_id = self.output_frame_id
        if self.output_child_frame_id:
            out.child_frame_id = self.output_child_frame_id

        out.pose = msg.pose
        out.twist = msg.twist
        out.pose.pose.position.x = out_tf[0, 3]
        out.pose.pose.position.y = out_tf[1, 3]
        out.pose.pose.position.z = out_tf[2, 3]
        out.pose.pose.orientation = quaternion_from_matrix(out_tf)

        if self.mode == 'world':
            linear = self.mount_tf[0:3, 0:3].dot([
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            ])
            angular = self.mount_tf[0:3, 0:3].dot([
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z,
            ])
            out.twist.twist.linear.x = linear[0]
            out.twist.twist.linear.y = linear[1]
            out.twist.twist.linear.z = linear[2]
            out.twist.twist.angular.x = angular[0]
            out.twist.twist.angular.y = angular[1]
            out.twist.twist.angular.z = angular[2]

        self.pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('odom_mount_transform')
    OdomMountTransform()
    rospy.spin()
