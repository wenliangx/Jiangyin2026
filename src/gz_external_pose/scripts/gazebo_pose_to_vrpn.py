#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped


class GazeboPoseToVrpn:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "iris_mid360")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.output_topic = rospy.get_param("~output_topic", "/vrpn_client_node/jy0/pose")
        self.warn_interval = rospy.Duration(rospy.get_param("~warn_interval", 5.0))
        self.last_warn = rospy.Time(0)

        self.pose_pub = rospy.Publisher(self.output_topic, PoseStamped, queue_size=10)
        self.states_sub = rospy.Subscriber(
            "/gazebo/model_states",
            ModelStates,
            self.model_states_cb,
            queue_size=10,
        )

        rospy.loginfo(
            "Publishing Gazebo model '%s' pose to %s",
            self.model_name,
            self.output_topic,
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

        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose = msg.pose[index]
        self.pose_pub.publish(pose_msg)


def main():
    rospy.init_node("gazebo_pose_to_vrpn")
    GazeboPoseToVrpn()
    rospy.spin()


if __name__ == "__main__":
    main()
