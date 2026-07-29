#!/usr/bin/env python3
"""Serve a USB camera as MJPEG with a live focus sharpness metric."""

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import time

import cv2


INDEX_HTML = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>UAV Camera Focus</title>
  <style>
    body { margin: 0; background: #111; color: #eee; font-family: sans-serif; }
    main { max-width: 1280px; margin: auto; padding: 16px; }
    h1 { font-size: 20px; font-weight: 500; }
    img { display: block; width: 100%; height: auto; background: #000; }
    p { color: #bbb; }
  </style>
</head>
<body>
  <main>
    <h1>UAV Camera Focus</h1>
    <img src="/stream.mjpg" alt="USB camera live stream">
    <p>缓慢调节镜头，尽量让画面细节清晰，并观察 Sharpness 数值升高。</p>
  </main>
</body>
</html>
""".encode("utf-8")


def measure_sharpness(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F, ksize=3).var())


def parse_device(value):
    return int(value) if value.isdigit() else value


class FrameRateMeter:
    def __init__(self, smoothing=0.9):
        if not 0.0 <= smoothing < 1.0:
            raise ValueError("smoothing must be in [0, 1)")
        self.smoothing = smoothing
        self.last_time = None
        self.value = 0.0

    def update(self, now=None):
        now = time.monotonic() if now is None else float(now)
        if self.last_time is None:
            self.last_time = now
            return 0.0

        instantaneous = 1.0 / max(now - self.last_time, 1e-6)
        self.last_time = now
        self.value = (
            instantaneous
            if self.value == 0.0
            else self.smoothing * self.value
            + (1.0 - self.smoothing) * instantaneous
        )
        return self.value


def open_capture(device, width, height, fps):
    capture = cv2.VideoCapture(parse_device(device), cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    capture.set(cv2.CAP_PROP_FPS, fps)
    if not capture.isOpened():
        capture.release()
        raise RuntimeError(f"cannot open camera device: {device}")
    ok, frame = capture.read()
    if not ok or frame is None:
        capture.release()
        raise RuntimeError(f"camera opened but returned no frame: {device}")
    return capture, frame


class FocusServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, handler, capture, first_frame, device):
        super().__init__(address, handler)
        self.capture = capture
        self.first_frame = first_frame
        self.device = device


class FocusHandler(BaseHTTPRequestHandler):
    def log_message(self, format_string, *args):
        print(
            f"{self.client_address[0]} "
            f"{format_string % args}",
            flush=True,
        )

    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(INDEX_HTML)))
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

        frame = self.server.first_frame
        self.server.first_frame = None
        fps_meter = FrameRateMeter()
        try:
            while True:
                if frame is None:
                    ok, frame = self.server.capture.read()
                    if not ok or frame is None:
                        time.sleep(0.02)
                        continue
                smoothed_fps = fps_meter.update()

                sharpness = measure_sharpness(frame)
                height, width = frame.shape[:2]
                display = frame.copy()
                cv2.putText(
                    display,
                    (
                        f"{self.server.device}  {width}x{height}  "
                        f"FPS {smoothed_fps:.1f}  Sharpness {sharpness:.1f}"
                    ),
                    (16, 32),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.68,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                ok, encoded = cv2.imencode(
                    ".jpg", display, [cv2.IMWRITE_JPEG_QUALITY, 85]
                )
                frame = None
                if not ok:
                    continue
                payload = encoded.tobytes()
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(
                    f"Content-Length: {len(payload)}\r\n\r\n".encode("ascii")
                )
                self.wfile.write(payload)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            return


def build_argument_parser():
    parser = argparse.ArgumentParser(
        description="View a V4L2 camera in a browser while adjusting focus."
    )
    parser.add_argument(
        "--device",
        required=True,
        help="V4L2 path such as /dev/v4l/by-path/...-video-index0",
    )
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=30.0)
    return parser


def main():
    args = build_argument_parser().parse_args()
    capture, first_frame = open_capture(
        args.device, args.width, args.height, args.fps
    )
    server = FocusServer(
        (args.host, args.port),
        FocusHandler,
        capture,
        first_frame,
        args.device,
    )
    print(
        f"Camera ready: http://<NUC-IP>:{args.port} "
        f"(device={args.device})",
        flush=True,
    )
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        print("Stopping camera viewer.", flush=True)
    finally:
        server.server_close()
        capture.release()


if __name__ == "__main__":
    main()
