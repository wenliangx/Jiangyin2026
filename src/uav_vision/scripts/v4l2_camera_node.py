#!/usr/bin/env python3
"""Publish one V4L2 camera with fixed manual exposure controls."""

import fcntl
import os
import struct


V4L2_CID_GAIN = 0x00980913
V4L2_CID_EXPOSURE_AUTO = 0x009A0901
V4L2_CID_EXPOSURE_ABSOLUTE = 0x009A0902
V4L2_CID_EXPOSURE_AUTO_PRIORITY = 0x009A0903
_IOC_READ = 2
_IOC_WRITE = 1
_VIDIOC_G_CTRL = (
    ((_IOC_READ | _IOC_WRITE) << 30)
    | (8 << 16)
    | (ord("V") << 8)
    | 27
)
_VIDIOC_S_CTRL = (
    ((_IOC_READ | _IOC_WRITE) << 30)
    | (8 << 16)
    | (ord("V") << 8)
    | 28
)


def validate_settings(width, height, fps, exposure, gain):
    width = int(width)
    height = int(height)
    fps = float(fps)
    exposure = int(exposure)
    gain = int(gain)
    if width <= 0 or height <= 0 or fps <= 0.0:
        raise ValueError("width, height and fps must be positive")
    if not 1 <= exposure <= 5000:
        raise ValueError("exposure must be in [1, 5000]")
    if not 0 <= gain <= 100:
        raise ValueError("gain must be in [0, 100]")
    return width, height, fps, exposure, gain


def control_requests(exposure, gain):
    return [
        (V4L2_CID_EXPOSURE_AUTO, 1),
        (V4L2_CID_EXPOSURE_ABSOLUTE, int(exposure)),
        (V4L2_CID_GAIN, int(gain)),
        (V4L2_CID_EXPOSURE_AUTO_PRIORITY, 0),
    ]


class V4L2Controls:
    def __init__(self, device):
        self._fd = os.open(device, os.O_RDWR | os.O_NONBLOCK)

    def close(self):
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None

    def set(self, control_id, value):
        data = bytearray(struct.pack("<Ii", control_id, value))
        fcntl.ioctl(self._fd, _VIDIOC_S_CTRL, data, True)

    def get(self, control_id):
        data = bytearray(struct.pack("<Ii", control_id, 0))
        fcntl.ioctl(self._fd, _VIDIOC_G_CTRL, data, True)
        return struct.unpack("<Ii", data)[1]


def apply_controls(controls, exposure, gain):
    for control_id, requested in control_requests(exposure, gain):
        controls.set(control_id, requested)
        actual = controls.get(control_id)
        if actual != requested:
            raise RuntimeError(
                f"control 0x{control_id:08x}: "
                f"requested {requested}, got {actual}"
            )


def main():
    import cv2
    from cv_bridge import CvBridge
    import rospy
    from sensor_msgs.msg import Image

    rospy.init_node("v4l2_camera_node")
    device = rospy.get_param("~device")
    image_topic = rospy.get_param("~image_topic")
    frame_id = rospy.get_param(
        "~frame_id", rospy.get_name().strip("/")
    )
    pixel_format = str(rospy.get_param("~pixel_format", "MJPG"))
    settings = validate_settings(
        rospy.get_param("~width", 1280),
        rospy.get_param("~height", 720),
        rospy.get_param("~fps", 30.0),
        rospy.get_param("~exposure", 150),
        rospy.get_param("~gain", 5),
    )
    width, height, fps, exposure, gain = settings
    if len(pixel_format) != 4:
        raise ValueError("pixel_format must contain four characters")

    capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
    capture.set(
        cv2.CAP_PROP_FOURCC,
        cv2.VideoWriter_fourcc(*pixel_format),
    )
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    capture.set(cv2.CAP_PROP_FPS, fps)
    if not capture.isOpened():
        capture.release()
        raise RuntimeError(f"cannot open camera device: {device}")

    controls = V4L2Controls(device)
    try:
        apply_controls(controls, exposure, gain)
        control_state = {
            f"0x{control_id:08x}": controls.get(control_id)
            for control_id, _ in control_requests(exposure, gain)
        }
    finally:
        controls.close()

    rospy.loginfo(
        "camera ready: device=%s topic=%s size=%dx%d fps=%.1f "
        "format=%s controls=%s",
        device,
        image_topic,
        int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
        int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        capture.get(cv2.CAP_PROP_FPS),
        pixel_format,
        control_state,
    )

    publisher = rospy.Publisher(image_topic, Image, queue_size=1)
    bridge = CvBridge()
    rate = rospy.Rate(fps)
    try:
        while not rospy.is_shutdown():
            ok, frame = capture.read()
            if not ok or frame is None:
                rospy.logwarn_throttle(
                    1.0, "camera returned no frame: %s", device
                )
                rate.sleep()
                continue
            message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = frame_id
            publisher.publish(message)
            rate.sleep()
    finally:
        capture.release()


if __name__ == "__main__":
    main()
