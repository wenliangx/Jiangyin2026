#!/usr/bin/env python3
"""ROS1 adapter for square-board multi-template classification."""

import os

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
from uav_vision_msgs.msg import TargetMatch, TargetMatchArray, VisionControl

from uav_vision.board_pipeline import DetectorConfig, TargetRecognitionPipeline
from uav_vision.target_matcher import MatcherConfig
from uav_vision.temporal_vote import TemporalVoter
from uav_vision.video_recorder import (
    AsyncVideoPairRecorder,
    RecorderConfig,
    beijing_timestamp,
)


def normalize_camera_role(camera_role):
    camera_role = str(camera_role).strip().lower()
    if camera_role == "down":
        camera_role = "rear"
    if camera_role not in ("front", "rear"):
        raise ValueError("camera_role must be 'front' or 'rear'")
    return camera_role


def requested_camera_enabled(message, camera_role):
    camera_role = normalize_camera_role(camera_role)
    if camera_role == "front":
        return bool(message.front_camera_enabled)
    # Keep the legacy message field until all non-vision publishers migrate.
    return bool(message.down_camera_enabled)


class TargetMatchNode:
    def __init__(self):
        matcher_values = rospy.get_param("~matcher")
        detector_values = rospy.get_param("~detector")
        temporal_values = rospy.get_param("~temporal")
        templates_dir = rospy.get_param("~templates_dir")
        model_path = rospy.get_param("~model_path")
        if not os.path.isdir(templates_dir):
            raise RuntimeError(f"templates directory does not exist: {templates_dir}")

        detector_config = DetectorConfig.from_mapping(detector_values)
        self._bridge = CvBridge()
        self._pipeline = TargetRecognitionPipeline(
            detector_config,
            MatcherConfig.from_mapping(matcher_values),
            templates_dir,
            model_path,
        )
        self._processing_period = 1.0 / detector_config.processing_fps
        self._last_processed_stamp = None
        self._voter = TemporalVoter(
            window_size=int(temporal_values["vote_window"]),
            min_votes=int(temporal_values["min_stable_votes"]),
            lost_frames=int(temporal_values["target_lost_frames"]),
        )
        self._last_stable_result = None
        self._camera_role = normalize_camera_role(
            rospy.get_param("~camera_role", "front")
        )
        self._always_enabled = bool(
            rospy.get_param("~always_enabled", False)
        )
        self._enabled = self._always_enabled
        self._publish_debug = bool(
            rospy.get_param("~publish_debug_image", True)
        )
        self._recorder = AsyncVideoPairRecorder(
            RecorderConfig.from_mapping(rospy.get_param("~recording", {})),
            rospy.get_param(
                "~recording_stream_name",
                f"{self._camera_role}_target",
            ),
            log_info=rospy.loginfo,
            log_warning=rospy.logwarn,
            log_error=rospy.logerr,
        )
        rospy.on_shutdown(self._recorder.close)

        result_topic = rospy.get_param(
            "~result_topic", "/vision/target/result"
        )
        debug_topic = rospy.get_param(
            "~debug_topic", "/vision/target/debug_image"
        )
        image_topic = rospy.get_param(
            "~image_topic", "/vision/front/image_raw"
        )
        control_topic = rospy.get_param(
            "~control_topic", "/vision/control"
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
        self._control_subscriber = rospy.Subscriber(
            control_topic,
            VisionControl,
            self._control_callback,
            queue_size=1,
        )
        # Clear any result latch in downstream consumers when this node is
        # restarted while its camera is disabled.
        self._publish_invalid()
        rospy.loginfo(
            "target_match_node ready: role=%s image=%s result=%s "
            "control=%s enabled=%s templates=%s model=%s",
            self._camera_role,
            image_topic,
            result_topic,
            control_topic,
            self._enabled,
            templates_dir,
            model_path,
        )

    def _publish_invalid(self, stamp=None):
        output = TargetMatchArray()
        output.header.stamp = stamp or rospy.Time.now()
        output.valid = False
        output.matches = []
        self._result_publisher.publish(output)

    def _reset_matching_state(self):
        self._voter.reset()
        self._last_stable_result = None
        self._last_processed_stamp = None

    def _should_process(self, stamp):
        stamp_seconds = stamp.to_sec()
        if stamp_seconds <= 0.0:
            stamp_seconds = rospy.get_time()
        if self._last_processed_stamp is not None:
            elapsed = stamp_seconds - self._last_processed_stamp
            if 0.0 <= elapsed < self._processing_period * 0.95:
                return False
        self._last_processed_stamp = stamp_seconds
        return True

    def _control_callback(self, message):
        requested = requested_camera_enabled(message, self._camera_role)
        enabled = self._always_enabled or requested
        if enabled == self._enabled:
            return
        self._enabled = enabled
        self._reset_matching_state()
        if not enabled:
            self._publish_invalid(message.header.stamp)
        rospy.loginfo(
            "%s target matcher %s",
            self._camera_role,
            "enabled" if enabled else "disabled",
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
        if not self._enabled:
            return
        if not self._should_process(image_message.header.stamp):
            return
        output = TargetMatchArray()
        output.header = image_message.header
        output.valid = False
        output.matches = []
        frame = None
        result = None
        pipeline_result = None
        stable_label = None
        published_result = None
        try:
            frame = self._bridge.imgmsg_to_cv2(
                image_message, desired_encoding="bgr8"
            )
            pipeline_result = self._pipeline.process(frame)
            result = pipeline_result.match
            stable_label = self._voter.update(
                result.label if result.valid else None
            )
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
            rospy.loginfo_throttle(
                5.0,
                "%s target pipeline: detector=%.1f ms classify=%.1f ms "
                "total=%.1f ms method=%s current=%s stable=%s",
                self._camera_role,
                pipeline_result.detector_ms,
                pipeline_result.classification_ms,
                pipeline_result.total_ms,
                pipeline_result.method,
                result.label,
                stable_label or "none",
            )
        except (CvBridgeError, ValueError, RuntimeError, cv2.error) as error:
            rospy.logwarn_throttle(2.0, "target matching failed: %s", error)

        self._result_publisher.publish(output)

        if (
            self._publish_debug
            and frame is not None
            and pipeline_result is not None
        ):
            debug = self._pipeline.annotate(
                frame, pipeline_result, stable_label
            )
            debug_message = self._bridge.cv2_to_imgmsg(
                debug, encoding="bgr8"
            )
            debug_message.header = image_message.header
            self._debug_publisher.publish(debug_message)

        if frame is not None and self._recorder.enabled:
            stamp_seconds = image_message.header.stamp.to_sec()
            recording_frame = self._pipeline.annotate_recording(
                frame,
                published_result,
                beijing_timestamp(stamp_seconds),
            )
            self._recorder.submit(
                frame, recording_frame, stamp_seconds
            )


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
