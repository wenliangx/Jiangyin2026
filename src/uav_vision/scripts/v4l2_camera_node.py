#!/usr/bin/env python3
"""Publish one V4L2 camera with fixed manual exposure controls."""

import fcntl
import os
import struct
import threading
import time


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


def orient_frame(frame, rotate_180):
    if not rotate_180:
        return frame
    return frame[::-1, ::-1].copy()


def validate_camera_role(camera_role):
    camera_role = str(camera_role).strip().lower()
    # Keep "down" as a compatibility alias while VisionControl still uses
    # the legacy down_camera_enabled field for the rear camera.
    if camera_role == "down":
        camera_role = "rear"
    if camera_role not in ("front", "rear"):
        raise ValueError("camera_role must be 'front' or 'rear'")
    return camera_role


def requested_camera_enabled(message, camera_role):
    camera_role = validate_camera_role(camera_role)
    if camera_role == "front":
        return bool(message.front_camera_enabled)
    # The message field is intentionally retained to avoid changing the ROS
    # message MD5. It now carries the rear-camera request.
    return bool(message.down_camera_enabled)


class CameraControlState:
    """Thread-safe desired camera state selected from VisionControl."""

    def __init__(self, camera_role, always_enabled=False):
        self._camera_role = validate_camera_role(camera_role)
        self._always_enabled = bool(always_enabled)
        self._desired_enabled = self._always_enabled
        self._lock = threading.Lock()

    @property
    def desired_enabled(self):
        with self._lock:
            return self._desired_enabled

    def update(self, message):
        requested = requested_camera_enabled(message, self._camera_role)
        desired = self._always_enabled or requested
        with self._lock:
            changed = desired != self._desired_enabled
            self._desired_enabled = desired
        return changed, desired

    def run_if_enabled(self, action):
        """Run a short action atomically with the final enabled check."""
        with self._lock:
            if not self._desired_enabled:
                return False
            action()
            return True


def open_capture(
    cv2,
    device,
    pixel_format,
    width,
    height,
    fps,
    exposure,
    gain,
):
    capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
    try:
        capture.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*pixel_format),
        )
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        capture.set(cv2.CAP_PROP_FPS, fps)
        if not capture.isOpened():
            raise RuntimeError(f"cannot open camera device: {device}")

        controls = V4L2Controls(device)
        try:
            apply_controls(controls, exposure, gain)
            control_values = {
                f"0x{control_id:08x}": controls.get(control_id)
                for control_id, _ in control_requests(exposure, gain)
            }
        finally:
            controls.close()
        return capture, control_values
    except Exception:
        capture.release()
        raise


def main():
    import cv2
    from cv_bridge import CvBridge
    import rospy
    from sensor_msgs.msg import Image
    from uav_vision_msgs.msg import VisionControl

    rospy.init_node("v4l2_camera_node")
    device = rospy.get_param("~device")
    image_topic = rospy.get_param("~image_topic")
    frame_id = rospy.get_param(
        "~frame_id", rospy.get_name().strip("/")
    )
    pixel_format = str(rospy.get_param("~pixel_format", "MJPG"))
    rotate_180 = bool(rospy.get_param("~rotate_180", False))
    camera_role = validate_camera_role(
        rospy.get_param("~camera_role")
    )
    control_topic = rospy.get_param(
        "~control_topic", "/vision/control"
    )
    always_enabled = bool(rospy.get_param("~always_enabled", False))
    reopen_retry_seconds = float(
        rospy.get_param("~reopen_retry_seconds", 1.0)
    )
    if reopen_retry_seconds < 0.0:
        raise ValueError("reopen_retry_seconds must be non-negative")
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

    publisher = rospy.Publisher(image_topic, Image, queue_size=1)
    bridge = CvBridge()
    camera_control = CameraControlState(camera_role, always_enabled)

    def control_callback(message):
        changed, enabled = camera_control.update(message)
        if changed:
            rospy.loginfo(
                "%s camera requested %s",
                camera_role,
                "enabled" if enabled else "disabled",
            )

    control_subscriber = rospy.Subscriber(
        control_topic,
        VisionControl,
        control_callback,
        queue_size=1,
    )
    rospy.loginfo(
        "camera controller ready: role=%s device=%s topic=%s "
        "control=%s enabled=%s",
        camera_role,
        device,
        image_topic,
        control_topic,
        camera_control.desired_enabled,
    )

    rate = rospy.Rate(fps)
    capture = None
    next_open_attempt = 0.0
    try:
        while not rospy.is_shutdown():
            if not camera_control.desired_enabled:
                next_open_attempt = 0.0
                if capture is not None:
                    capture.release()
                    capture = None
                    rospy.loginfo("%s camera stopped", camera_role)
                rate.sleep()
                continue

            if capture is None:
                now = time.monotonic()
                if now < next_open_attempt:
                    rate.sleep()
                    continue
                try:
                    capture, control_values = open_capture(
                        cv2,
                        device,
                        pixel_format,
                        width,
                        height,
                        fps,
                        exposure,
                        gain,
                    )
                except (OSError, RuntimeError, cv2.error) as error:
                    next_open_attempt = now + reopen_retry_seconds
                    rospy.logwarn_throttle(
                        1.0,
                        "%s camera start failed: %s",
                        camera_role,
                        error,
                    )
                    rate.sleep()
                    continue

                if not camera_control.desired_enabled:
                    capture.release()
                    capture = None
                    continue
                rospy.loginfo(
                    "%s camera started: device=%s size=%dx%d fps=%.1f "
                    "format=%s rotate_180=%s controls=%s",
                    camera_role,
                    device,
                    int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
                    int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                    capture.get(cv2.CAP_PROP_FPS),
                    pixel_format,
                    rotate_180,
                    control_values,
                )

            try:
                ok, frame = capture.read()
            except cv2.error as error:
                ok, frame = False, None
                rospy.logwarn_throttle(
                    1.0, "%s camera read failed: %s", camera_role, error
                )
            if not ok or frame is None:
                rospy.logwarn_throttle(
                    1.0, "%s camera returned no frame: %s", camera_role, device
                )
                capture.release()
                capture = None
                next_open_attempt = time.monotonic() + reopen_retry_seconds
                rate.sleep()
                continue

            if not camera_control.desired_enabled:
                capture.release()
                capture = None
                rospy.loginfo("%s camera stopped", camera_role)
                continue

            frame = orient_frame(frame, rotate_180)
            message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = frame_id
            camera_control.run_if_enabled(
                lambda: publisher.publish(message)
            )
            rate.sleep()
    finally:
        if capture is not None:
            capture.release()
        control_subscriber.unregister()


if __name__ == "__main__":
    main()
