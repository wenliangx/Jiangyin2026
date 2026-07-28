#!/usr/bin/env python3
"""Serve the ROS landing debug image as a small MJPEG web page."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
import time

import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image


INDEX_HTML = b"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>UAV Landing Debug</title>
  <style>
    body { margin: 0; background: #111; color: #eee; font-family: sans-serif; }
    main { max-width: 1280px; margin: auto; padding: 12px; }
    h1 { margin: 0 0 10px; font-size: 20px; font-weight: 500; }
    img { display: block; width: 100%; height: auto; background: #000; }
  </style>
</head>
<body>
  <main>
    <h1>UAV Landing Debug</h1>
    <img src="/stream.mjpg" alt="landing debug stream">
  </main>
</body>
</html>
"""


class LatestJpegFrame:
    def __init__(self):
        self._lock = threading.Lock()
        self._frame = None

    def update(self, frame):
        with self._lock:
            self._frame = frame

    def get(self):
        with self._lock:
            return self._frame


class DebugWebServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, frame_buffer):
        super().__init__(address, DebugWebHandler)
        self.frame_buffer = frame_buffer


class DebugWebHandler(BaseHTTPRequestHandler):
    def log_message(self, format_string, *args):
        rospy.loginfo("%s %s", self.client_address[0], format_string % args)

    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(INDEX_HTML)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(INDEX_HTML)
            return
        if self.path != "/stream.mjpg":
            self.send_error(404)
            return
        self._serve_stream()

    def _serve_stream(self):
        self.send_response(200)
        self.send_header(
            "Content-Type", "multipart/x-mixed-replace; boundary=frame"
        )
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            while not rospy.is_shutdown():
                frame = self.server.frame_buffer.get()
                if frame is None:
                    time.sleep(0.05)
                    continue
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(
                    f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii")
                )
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
                time.sleep(0.03)
        except (BrokenPipeError, ConnectionResetError):
            return


def main():
    rospy.init_node("landing_debug_web")
    image_topic = rospy.get_param(
        "~image_topic", "/vision/landing/debug_image"
    )
    host = rospy.get_param("~host", "0.0.0.0")
    port = int(rospy.get_param("~port", 8080))
    jpeg_quality = int(rospy.get_param("~jpeg_quality", 85))

    bridge = CvBridge()
    frames = LatestJpegFrame()

    def image_callback(message):
        image = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        ok, encoded = cv2.imencode(
            ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality]
        )
        if ok:
            frames.update(encoded.tobytes())

    rospy.Subscriber(image_topic, Image, image_callback, queue_size=1)
    server = DebugWebServer((host, port), frames)
    server.timeout = 0.2
    rospy.loginfo(
        "landing debug web ready: http://%s:%d topic=%s",
        host,
        port,
        image_topic,
    )
    try:
        while not rospy.is_shutdown():
            server.handle_request()
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
