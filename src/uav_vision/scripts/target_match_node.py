#!/usr/bin/env python3
"""ROS1 adapter for square-board multi-template classification."""

import os

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
from uav_vision_msgs.msg import TargetMatch, TargetMatchArray

from uav_vision.target_matcher import MatcherConfig, TargetMatcher
from uav_vision.temporal_vote import TemporalVoter


class TargetMatchNode:
    def __init__(self):
        matcher_values = rospy.get_param("~matcher")
        temporal_values = rospy.get_param("~temporal")
        templates_dir = rospy.get_param("~templates_dir")
        if not os.path.isdir(templates_dir):
            raise RuntimeError(f"templates directory does not exist: {templates_dir}")

        self._bridge = CvBridge()
        self._matcher = TargetMatcher(
            MatcherConfig.from_mapping(matcher_values),
            templates_dir,
        )
        self._voter = TemporalVoter(
            window_size=int(temporal_values["vote_window"]),
            min_votes=int(temporal_values["min_stable_votes"]),
            lost_frames=int(temporal_values["target_lost_frames"]),
        )
        self._last_stable_result = None
        self._publish_debug = bool(
            rospy.get_param("~publish_debug_image", True)
        )

        result_topic = rospy.get_param(
            "~result_topic", "/vision/target/result"
        )
        debug_topic = rospy.get_param(
            "~debug_topic", "/vision/target/debug_image"
        )
        image_topic = rospy.get_param(
            "~image_topic", "/vision/front/image_raw"
        )
        self._result_publisher = rospy.Publisher(
            result_topic, TargetMatchArray, queue_size=1
        )
        self._debug_publisher = rospy.Publisher(
            debug_topic, Image, queue_size=1
        )
        self._subscriber = rospy.Subscriber(
            image_topic,
            Image,
            self._image_callback,
            queue_size=1,
            buff_size=2**24,
        )
        rospy.loginfo(
            "target_match_node ready: image=%s result=%s templates=%s",
            image_topic,
            result_topic,
            templates_dir,
        )

    @staticmethod
    def _to_message(result):
        message = TargetMatch()
        message.label = result.label
        message.score = result.score
        message.gray_score = result.gray_score
        message.hog_score = result.hog_score
        message.color_score = result.color_score
        message.margin = result.margin
        message.sharpness = result.sharpness
        message.target_side_px = int(
            max(0, min(round(result.target_side_px), 65535))
        )
        message.corners = [
            float(value)
            for value in result.corners.reshape(-1).tolist()
        ]
        if len(message.corners) != 8:
            raise ValueError("valid target result must contain four corners")
        return message

    def _image_callback(self, image_message):
        output = TargetMatchArray()
        output.header = image_message.header
        output.valid = False
        output.matches = []
        frame = None
        result = None
        stable_label = None
        try:
            frame = self._bridge.imgmsg_to_cv2(
                image_message, desired_encoding="bgr8"
            )
            result = self._matcher.match_frame(frame)
            stable_label = self._voter.update(
                result.label if result.valid else None
            )
            published_result = None
            if stable_label is not None:
                if result.valid and result.label == stable_label:
                    self._last_stable_result = result
                if (
                    self._last_stable_result is not None
                    and self._last_stable_result.label == stable_label
                ):
                    published_result = self._last_stable_result
            else:
                self._last_stable_result = None
            if published_result is not None:
                output.valid = True
                output.matches = [self._to_message(published_result)]
        except (CvBridgeError, ValueError, cv2.error) as error:
            rospy.logwarn_throttle(2.0, "target matching failed: %s", error)

        self._result_publisher.publish(output)

        if self._publish_debug and frame is not None and result is not None:
            debug = self._matcher.annotate(frame, result, stable_label)
            debug_message = self._bridge.cv2_to_imgmsg(
                debug, encoding="bgr8"
            )
            debug_message.header = image_message.header
            self._debug_publisher.publish(debug_message)


def main():
    rospy.init_node("target_match_node")
    try:
        TargetMatchNode()
    except Exception as error:
        rospy.logfatal("failed to initialize target_match_node: %s", error)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
