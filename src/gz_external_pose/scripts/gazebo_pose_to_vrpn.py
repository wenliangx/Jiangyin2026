#!/usr/bin/env python3
import copy

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class GazeboPoseToVrpn:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "iris_mid360")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.child_frame_id = rospy.get_param("~child_frame_id", "base_link")
        self.output_topic = rospy.get_param("~output_topic", "/vrpn_client_node/jy0/pose")
        self.odom_topic = rospy.get_param("~odom_topic", "/camera/odom/sample")
        self.ready_topic = rospy.get_param("~ready_topic", "/fsm_ctrl/ekf_ready")
        self.zero_origin = rospy.get_param("~zero_origin", True)
        self.warn_interval = rospy.Duration(rospy.get_param("~warn_interval", 5.0))
        self.last_warn = rospy.Time(0)
        self.origin = None

        self.pose_pub = rospy.Publisher(self.output_topic, PoseStamped, queue_size=10)
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        self.ready_pub = rospy.Publisher(self.ready_topic, Bool, queue_size=1, latch=True)
        self.states_sub = rospy.Subscriber(
            "/gazebo/model_states",
            ModelStates,
            self.model_states_cb,
            queue_size=10,
        )

        rospy.loginfo(
            "Publishing Gazebo model '%s' pose to %s and odometry to %s",
            self.model_name,
            self.output_topic,
            self.odom_topic,
        )

    def model_states_cb(self, msg):
        try:
            index = msg.name.index(self.model_name)
        except ValueError:
            now = rospy.Time.now()
            if now - self.last_warn > self.warn_interval:
                rospy.logwarn(
                    "Model '%s' not found in /gazebo/model_states. Available models: %s",
                    self.model_name,
                    ", ".join(msg.name),
                )
                self.last_warn = now
            return

        pose = msg.pose[index]
        twist = msg.twist[index]
        if self.origin is None:
            self.origin = copy.deepcopy(pose.position)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose = copy.deepcopy(pose)
        if self.zero_origin:
            pose_msg.pose.position.x = pose.position.x - self.origin.x
            pose_msg.pose.position.y = pose.position.y - self.origin.y
            pose_msg.pose.position.z = pose.position.z - self.origin.z
        self.pose_pub.publish(pose_msg)

        odom_msg = Odometry()
        odom_msg.header = pose_msg.header
        odom_msg.child_frame_id = self.child_frame_id
        odom_msg.pose.pose = pose_msg.pose
        odom_msg.twist.twist = twist
        self.odom_pub.publish(odom_msg)
        self.ready_pub.publish(Bool(data=True))


def main():
    rospy.init_node("gazebo_pose_to_vrpn")
    GazeboPoseToVrpn()
    rospy.spin()


if __name__ == "__main__":
    main()
