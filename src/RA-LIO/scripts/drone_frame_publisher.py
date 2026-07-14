#!/usr/bin/env python3
import copy
import math

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf.transformations as tft
import tf2_ros


def quat_msg_from_xyzw(q):
    norm = math.sqrt(sum(v * v for v in q))
    if not math.isfinite(norm) or norm < 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return tuple(v / norm for v in q)


class DroneFramePublisher:
    def __init__(self):
        self.input_topic = rospy.get_param('~input_topic', '/Odometry')
        self.output_topic = rospy.get_param('~output_topic', '/drone/odometry')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.world_frame = rospy.get_param('~world_frame', 'world')
        self.body_frame = rospy.get_param('~body_frame', 'body')
        self.drone_frame = rospy.get_param('~drone_frame', 'drone')

        self.map_world_q = tft.quaternion_from_euler(math.pi, 0.0, 0.0, axes='sxyz')
        self.body_drone_q = tft.quaternion_from_euler(0.0, math.radians(-40.0), 0.0, axes='sxyz')
        self.body_drone_t = (0.07, 0.0, 0.0)
        self.body_drone_mat = tft.quaternion_matrix(self.body_drone_q)
        self.body_drone_mat[0:3, 3] = self.body_drone_t

        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.pub = rospy.Publisher(self.output_topic, Odometry, queue_size=10)
        self.sub = rospy.Subscriber(self.input_topic, Odometry, self.odom_callback, queue_size=10)

        self.publish_static_transforms()
        rospy.loginfo('drone_frame_publisher: %s -> %s, map=%s world=%s body=%s drone=%s',
                      self.input_topic, self.output_topic, self.map_frame, self.world_frame,
                      self.body_frame, self.drone_frame)

    def publish_static_transforms(self):
        now = rospy.Time.now()
        map_world = TransformStamped()
        map_world.header.stamp = now
        map_world.header.frame_id = self.map_frame
        map_world.child_frame_id = self.world_frame
        map_world.transform.translation.x = 0.0
        map_world.transform.translation.y = 0.0
        map_world.transform.translation.z = 0.0
        q = quat_msg_from_xyzw(self.map_world_q)
        map_world.transform.rotation.x = q[0]
        map_world.transform.rotation.y = q[1]
        map_world.transform.rotation.z = q[2]
        map_world.transform.rotation.w = q[3]

        body_drone = TransformStamped()
        body_drone.header.stamp = now
        body_drone.header.frame_id = self.body_frame
        body_drone.child_frame_id = self.drone_frame
        body_drone.transform.translation.x = self.body_drone_t[0]
        body_drone.transform.translation.y = self.body_drone_t[1]
        body_drone.transform.translation.z = self.body_drone_t[2]
        q = quat_msg_from_xyzw(self.body_drone_q)
        body_drone.transform.rotation.x = q[0]
        body_drone.transform.rotation.y = q[1]
        body_drone.transform.rotation.z = q[2]
        body_drone.transform.rotation.w = q[3]

        self.static_broadcaster.sendTransform([map_world, body_drone])

    def odom_callback(self, msg):
        pose = msg.pose.pose
        world_body = tft.quaternion_matrix([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ])
        world_body[0:3, 3] = [pose.position.x, pose.position.y, pose.position.z]

        map_world = tft.quaternion_matrix(self.map_world_q)
        map_drone = map_world.dot(world_body).dot(self.body_drone_mat)

        out = Odometry()
        out.header = copy.deepcopy(msg.header)
        out.header.frame_id = self.map_frame
        out.child_frame_id = self.drone_frame
        out.pose = copy.deepcopy(msg.pose)
        out.twist = copy.deepcopy(msg.twist)
        out.pose.pose.position.x = map_drone[0, 3]
        out.pose.pose.position.y = map_drone[1, 3]
        out.pose.pose.position.z = map_drone[2, 3]
        q = quat_msg_from_xyzw(tft.quaternion_from_matrix(map_drone))
        out.pose.pose.orientation.x = q[0]
        out.pose.pose.orientation.y = q[1]
        out.pose.pose.orientation.z = q[2]
        out.pose.pose.orientation.w = q[3]

        self.pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('drone_frame_publisher')
    DroneFramePublisher()
    rospy.spin()
