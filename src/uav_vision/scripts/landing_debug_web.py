#!/usr/bin/env python3
"""Serve the ROS landing debug image as a small MJPEG web page."""

import html
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import threading
import time

import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image


def validate_stream_settings(max_width, stream_fps):
    max_width = int(max_width)
    stream_fps = float(stream_fps)
    if max_width < 0:
        raise ValueError("max_width must be zero or positive")
    if stream_fps <= 0.0:
        raise ValueError("stream_fps must be positive")
    return max_width, stream_fps


def resize_to_max_width(image, max_width):
    if max_width == 0 or image.shape[1] <= max_width:
        return image
    scale = float(max_width) / image.shape[1]
    height = max(1, int(round(image.shape[0] * scale)))
    return cv2.resize(image, (max_width, height), interpolation=cv2.INTER_AREA)


def validate_extrinsic_rpy(value):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError("extrinsic RPY must contain three values")
    result = [float(component) for component in value]
    if not all(math.isfinite(component) for component in result):
        raise ValueError("extrinsic RPY must be finite")
    if any(abs(component) > 360.0 for component in result):
        raise ValueError("extrinsic RPY must be within +/-360 degrees")
    return result


def build_index_html(page_title, enable_extrinsic_controls=False):
    safe_title = html.escape(str(page_title))
    controls = ""
    if enable_extrinsic_controls:
        controls = """
    <section class="controls">
      <div class="controls-title">相机 → 机体外参（度）</div>
      <div class="axis-grid">
        <label for="roll-range">Roll</label>
        <input id="roll-range" type="range" min="-180" max="180" step="0.5" value="0">
        <input id="roll-number" type="number" min="-360" max="360" step="0.5" value="0">
        <label for="pitch-range">Pitch</label>
        <input id="pitch-range" type="range" min="-180" max="180" step="0.5" value="0">
        <input id="pitch-number" type="number" min="-360" max="360" step="0.5" value="0">
        <label for="yaw-range">Yaw</label>
        <input id="yaw-range" type="range" min="-180" max="180" step="0.5" value="0">
        <input id="yaw-number" type="number" min="-360" max="360" step="0.5" value="0">
      </div>
      <div class="buttons">
        <button id="apply" type="button">应用外参</button>
        <button id="down-default" type="button" class="secondary">正装下视 180, 0, 90</button>
        <button id="identity" type="button" class="secondary">恢复 0, 0, 0</button>
        <span id="status">正在读取当前参数…</span>
      </div>
      <div class="hint">旋转顺序：Rz(yaw) · Ry(pitch) · Rx(roll)；应用后约 0.2 秒生效。</div>
    </section>
    <script>
      const axes = ['roll', 'pitch', 'yaw'];
      function setValues(values) {
        axes.forEach((axis, index) => {
          document.getElementById(axis + '-range').value = values[index];
          document.getElementById(axis + '-number').value = values[index];
        });
      }
      function values() {
        return axes.map(axis => Number(document.getElementById(axis + '-number').value));
      }
      axes.forEach(axis => {
        const range = document.getElementById(axis + '-range');
        const number = document.getElementById(axis + '-number');
        range.addEventListener('input', () => { number.value = range.value; });
        number.addEventListener('input', () => { range.value = number.value; });
      });
      async function readCurrent() {
        const response = await fetch('/api/extrinsic', {cache: 'no-store'});
        const data = await response.json();
        if (!response.ok) throw new Error(data.error || '读取失败');
        setValues(data.rpy_deg);
        document.getElementById('status').textContent = '当前：' + data.rpy_deg.join(', ') + '°';
      }
      async function apply(valuesToApply) {
        const status = document.getElementById('status');
        status.textContent = '正在应用…';
        const response = await fetch('/api/extrinsic', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({rpy_deg: valuesToApply})
        });
        const data = await response.json();
        if (!response.ok) throw new Error(data.error || '应用失败');
        setValues(data.rpy_deg);
        status.textContent = '已应用：' + data.rpy_deg.join(', ') + '°';
      }
      document.getElementById('apply').addEventListener('click', () => {
        apply(values()).catch(error => { document.getElementById('status').textContent = error.message; });
      });
      document.getElementById('identity').addEventListener('click', () => {
        apply([0, 0, 0]).catch(error => { document.getElementById('status').textContent = error.message; });
      });
      document.getElementById('down-default').addEventListener('click', () => {
        apply([180, 0, 90]).catch(error => { document.getElementById('status').textContent = error.message; });
      });
      readCurrent().catch(error => { document.getElementById('status').textContent = error.message; });
    </script>
"""
    return f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>{safe_title}</title>
  <style>
    body {{ margin: 0; background: #111; color: #eee; font-family: sans-serif; }}
    main {{ max-width: 1280px; margin: auto; padding: 12px; }}
    h1 {{ margin: 0 0 10px; font-size: 20px; font-weight: 500; }}
    img {{ display: block; width: 100%; height: auto; background: #000; }}
    .controls {{ margin-bottom: 12px; padding: 12px; background: #20242a; border-radius: 6px; }}
    .controls-title {{ margin-bottom: 8px; font-weight: 600; }}
    .axis-grid {{ display: grid; grid-template-columns: 52px 1fr 92px; gap: 8px 10px; align-items: center; }}
    .axis-grid input[type=number] {{ width: 80px; padding: 5px; color: #eee; background: #111; border: 1px solid #555; }}
    .buttons {{ display: flex; align-items: center; gap: 10px; margin-top: 10px; flex-wrap: wrap; }}
    button {{ padding: 7px 14px; cursor: pointer; color: #111; background: #5de1e6; border: 0; border-radius: 4px; font-weight: 600; }}
    button.secondary {{ color: #eee; background: #555; }}
    #status {{ color: #f5d76e; }}
    .hint {{ margin-top: 8px; color: #aaa; font-size: 13px; }}
  </style>
</head>
<body>
  <main>
    <h1>{safe_title}</h1>
{controls}
    <img src="/stream.mjpg" alt="ROS debug image stream">
  </main>
</body>
</html>
""".encode("utf-8")


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

    def __init__(
        self,
        address,
        frame_buffer,
        page_title,
        stream_fps,
        enable_extrinsic_controls=False,
        extrinsic_param="/landing_tag_node/pose/camera_to_body_rpy_deg",
    ):
        super().__init__(address, DebugWebHandler)
        self.frame_buffer = frame_buffer
        self.enable_extrinsic_controls = bool(enable_extrinsic_controls)
        self.extrinsic_param = str(extrinsic_param)
        self.index_html = build_index_html(
            page_title, self.enable_extrinsic_controls
        )
        self.stream_period = 1.0 / stream_fps


class DebugWebHandler(BaseHTTPRequestHandler):
    def log_message(self, format_string, *args):
        rospy.loginfo("%s %s", self.client_address[0], format_string % args)

    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header(
                "Content-Length", str(len(self.server.index_html))
            )
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(self.server.index_html)
            return
        if self.path == "/api/extrinsic":
            if not self.server.enable_extrinsic_controls:
                self.send_error(404)
                return
            try:
                value = validate_extrinsic_rpy(
                    rospy.get_param(self.server.extrinsic_param, [0, 0, 0])
                )
                self._send_json(200, {"rpy_deg": value})
            except (TypeError, ValueError) as error:
                self._send_json(500, {"error": str(error)})
            return
        if self.path != "/stream.mjpg":
            self.send_error(404)
            return
        self._serve_stream()

    def do_POST(self):
        if (
            self.path != "/api/extrinsic"
            or not self.server.enable_extrinsic_controls
        ):
            self.send_error(404)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > 4096:
                raise ValueError("invalid request length")
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
            value = validate_extrinsic_rpy(payload.get("rpy_deg"))
            rospy.set_param(self.server.extrinsic_param, value)
            self._send_json(200, {"rpy_deg": value})
        except (AttributeError, json.JSONDecodeError, TypeError, ValueError) as error:
            self._send_json(400, {"error": str(error)})

    def _send_json(self, status, payload):
        encoded = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(encoded)

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
                time.sleep(self.server.stream_period)
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
    max_width, stream_fps = validate_stream_settings(
        rospy.get_param("~max_width", 0),
        rospy.get_param("~stream_fps", 30.0),
    )
    page_title = rospy.get_param("~page_title", "UAV Landing Debug")
    enable_extrinsic_controls = bool(
        rospy.get_param("~enable_extrinsic_controls", False)
    )
    extrinsic_param = rospy.get_param(
        "~extrinsic_param",
        "/landing_tag_node/pose/camera_to_body_rpy_deg",
    )

    bridge = CvBridge()
    frames = LatestJpegFrame()

    def image_callback(message):
        image = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        image = resize_to_max_width(image, max_width)
        ok, encoded = cv2.imencode(
            ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality]
        )
        if ok:
            frames.update(encoded.tobytes())

    rospy.Subscriber(image_topic, Image, image_callback, queue_size=1)
    server = DebugWebServer(
        (host, port),
        frames,
        page_title,
        stream_fps,
        enable_extrinsic_controls,
        extrinsic_param,
    )
    server.timeout = 0.2
    rospy.loginfo(
        "landing debug web ready: http://%s:%d topic=%s "
        "max_width=%d stream_fps=%.1f jpeg_quality=%d",
        host,
        port,
        image_topic,
        max_width,
        stream_fps,
        jpeg_quality,
    )
    try:
        while not rospy.is_shutdown():
            server.handle_request()
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
