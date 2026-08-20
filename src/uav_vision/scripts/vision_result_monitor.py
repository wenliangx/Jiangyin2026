#!/usr/bin/env python3

import rospy

from uav_vision_msgs.msg import LandingOffset, TargetMatchArray


class VisionResultMonitor:
    def __init__(self):
        self._landing_topic = rospy.get_param(
            "~landing_topic", "/vision/landing/offset")
        self._target_topic = rospy.get_param(
            "~target_topic", "/vision/target/result")

        rospy.Subscriber(
            self._landing_topic, LandingOffset, self._landing_callback,
            queue_size=1)
        rospy.Subscriber(
            self._target_topic, TargetMatchArray, self._target_callback,
            queue_size=1)

        rospy.loginfo("Landing results: %s", self._landing_topic)
        rospy.loginfo("Target results:  %s", self._target_topic)

    @staticmethod
    def _stamp(message):
        stamp = message.header.stamp
        return stamp.to_sec() if stamp else 0.0

    def _landing_callback(self, message):
        if message.valid:
            rospy.loginfo(
                "[下视] t=%.3f valid=1 tags=%s dx=%+.1fpx dy=%+.1fpx "
                "center=(%.1f, %.1f)",
                self._stamp(message), list(message.tag_ids), message.dx,
                message.dy, message.center_x, message.center_y)
        else:
            rospy.loginfo_throttle(1.0, "[下视] valid=0（未识别到降落标记）")

    def _target_callback(self, message):
        if not message.valid or not message.matches:
            rospy.loginfo_throttle(1.0, "[前视] valid=0（未识别到目标）")
            return

        matches = sorted(message.matches, key=lambda item: item.score,
                         reverse=True)
        summary = ", ".join(
            "{} score={:.3f} margin={:.3f}".format(
                match.label, match.score, match.margin)
            for match in matches)
        rospy.loginfo("[前视] t=%.3f valid=1 %s", self._stamp(message), summary)


def main():
    rospy.init_node("vision_result_monitor")
    VisionResultMonitor()
    rospy.spin()


if __name__ == "__main__":
    main()
