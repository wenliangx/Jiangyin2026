"""Low-latency paired video recording for vision debug sessions."""

from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
import os
import queue
import re
import threading
import time
from typing import Callable, Mapping, Optional

import cv2
import numpy as np


BEIJING_TIMEZONE = timezone(timedelta(hours=8), name="Asia/Shanghai")


def beijing_timestamp(stamp_seconds: float) -> str:
    """Format a Unix timestamp as a millisecond Beijing wall-clock time."""
    stamp_seconds = float(stamp_seconds)
    if stamp_seconds <= 0.0:
        stamp_seconds = time.time()
    value = datetime.fromtimestamp(stamp_seconds, BEIJING_TIMEZONE)
    return value.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3] + " Beijing"


def _date_folder(stamp_seconds: float) -> str:
    value = datetime.fromtimestamp(stamp_seconds, BEIJING_TIMEZONE)
    return value.strftime("%Y-%m-%d")


def _allocate_session_directory(root, date_folder, stream_name):
    pipeline_dir = os.path.join(root, date_folder, stream_name)
    os.makedirs(pipeline_dir, exist_ok=True)
    indices = [
        int(name)
        for name in os.listdir(pipeline_dir)
        if re.fullmatch(r"\d{3}", name)
        and os.path.isdir(os.path.join(pipeline_dir, name))
    ]
    next_index = max(indices, default=0) + 1
    while True:
        session_dir = os.path.join(pipeline_dir, f"{next_index:03d}")
        try:
            os.mkdir(session_dir)
            return session_dir
        except FileExistsError:
            next_index += 1


@dataclass(frozen=True)
class RecorderConfig:
    enabled: bool = True
    root: str = "/home/flag/vision_recordings"
    fps: float = 10.0
    codec: str = "mp4v"
    extension: str = ".mp4"
    segment_seconds: float = 300.0
    inactivity_seconds: float = 3.0
    queue_size: int = 2

    @classmethod
    def from_mapping(cls, values: Mapping) -> "RecorderConfig":
        config = cls(
            enabled=bool(values.get("enabled", True)),
            root=os.path.abspath(os.path.expanduser(str(
                values.get("root", "/home/flag/vision_recordings")
            ))),
            fps=float(values.get("fps", 10.0)),
            codec=str(values.get("codec", "mp4v")),
            extension=str(values.get("extension", ".mp4")),
            segment_seconds=float(values.get("segment_seconds", 300.0)),
            inactivity_seconds=float(values.get("inactivity_seconds", 3.0)),
            queue_size=int(values.get("queue_size", 2)),
        )
        config.validate()
        return config

    def validate(self) -> None:
        if not self.root:
            raise ValueError("recording root must not be empty")
        if self.fps <= 0.0:
            raise ValueError("recording fps must be positive")
        if len(self.codec) != 4:
            raise ValueError("recording codec must contain four characters")
        if not self.extension.startswith("."):
            raise ValueError("recording extension must start with '.'")
        if self.segment_seconds <= 0.0:
            raise ValueError("recording segment_seconds must be positive")
        if self.inactivity_seconds <= 0.0:
            raise ValueError("recording inactivity_seconds must be positive")
        if self.queue_size <= 0:
            raise ValueError("recording queue_size must be positive")


@dataclass
class _FramePair:
    raw: np.ndarray
    processed: np.ndarray
    stamp_seconds: float


class AsyncVideoPairRecorder:
    """Write synchronized raw/result videos without blocking image callbacks.

    The callback only retains bounded image-buffer references and performs a
    non-blocking queue operation. If encoding falls behind, the oldest queued
    recording frame is dropped so recognition latency never grows with disk or
    codec latency.
    """

    def __init__(
        self,
        config: RecorderConfig,
        stream_name: str,
        log_info: Optional[Callable[[str], None]] = None,
        log_warning: Optional[Callable[[str], None]] = None,
        log_error: Optional[Callable[[str], None]] = None,
    ):
        self.config = config
        self.stream_name = str(stream_name).strip().replace("/", "_")
        if not self.stream_name:
            raise ValueError("recording stream_name must not be empty")
        self._log_info = log_info or (lambda message: None)
        self._log_warning = log_warning or (lambda message: None)
        self._log_error = log_error or (lambda message: None)
        self._queue = queue.Queue(maxsize=config.queue_size)
        self._submit_lock = threading.Lock()
        self._last_accepted_stamp = None
        self._dropped_frames = 0
        self._stop = threading.Event()
        self._thread = None
        if config.enabled:
            self._thread = threading.Thread(
                target=self._run,
                name=f"{self.stream_name}_video_writer",
                daemon=True,
            )
            self._thread.start()

    @property
    def enabled(self) -> bool:
        return self.config.enabled

    @property
    def dropped_frames(self) -> int:
        return self._dropped_frames

    def submit(
        self,
        raw: np.ndarray,
        processed: np.ndarray,
        stamp_seconds: float,
    ) -> bool:
        """Queue a synchronized pair; return False when sampled or disabled."""
        if not self.enabled or self._stop.is_set():
            return False
        stamp_seconds = float(stamp_seconds)
        if stamp_seconds <= 0.0:
            stamp_seconds = time.time()
        with self._submit_lock:
            minimum_interval = 1.0 / self.config.fps
            if self._last_accepted_stamp is not None:
                elapsed = stamp_seconds - self._last_accepted_stamp
                if 0.0 <= elapsed < minimum_interval * 0.95:
                    return False
            self._last_accepted_stamp = stamp_seconds

        item = _FramePair(
            # CvBridge frames and annotated outputs own/reference immutable
            # buffers for the rest of this callback. Keeping those references
            # avoids two extra 1280x720 copies on the recognition thread.
            raw=np.ascontiguousarray(raw),
            processed=np.ascontiguousarray(processed),
            stamp_seconds=stamp_seconds,
        )
        try:
            self._queue.put_nowait(item)
        except queue.Full:
            try:
                self._queue.get_nowait()
                self._queue.task_done()
            except queue.Empty:
                pass
            self._dropped_frames += 1
            if self._dropped_frames == 1 or self._dropped_frames % 100 == 0:
                self._log_warning(
                    f"{self.stream_name} recorder dropped "
                    f"{self._dropped_frames} queued frame(s)"
                )
            try:
                self._queue.put_nowait(item)
            except queue.Full:
                return False
        return True

    def close(self) -> None:
        if not self.enabled or self._thread is None:
            return
        self._stop.set()
        self._thread.join(timeout=10.0)
        if self._thread.is_alive():
            self._log_warning(
                f"{self.stream_name} video writer did not stop within 10 seconds"
            )
        self._thread = None

    def _open_writers(self, item, session_dir, part_index):
        if part_index == 0:
            raw_name = f"raw{self.config.extension}"
            result_name = f"result{self.config.extension}"
        else:
            raw_name = f"raw_part{part_index:03d}{self.config.extension}"
            result_name = f"result_part{part_index:03d}{self.config.extension}"
        raw_path = os.path.join(session_dir, raw_name)
        result_path = os.path.join(session_dir, result_name)
        fourcc = cv2.VideoWriter_fourcc(*self.config.codec)
        raw_size = (int(item.raw.shape[1]), int(item.raw.shape[0]))
        result_size = (
            int(item.processed.shape[1]),
            int(item.processed.shape[0]),
        )
        raw_writer = cv2.VideoWriter(
            raw_path, fourcc, self.config.fps, raw_size
        )
        result_writer = cv2.VideoWriter(
            result_path, fourcc, self.config.fps, result_size
        )
        if not raw_writer.isOpened() or not result_writer.isOpened():
            raw_writer.release()
            result_writer.release()
            for path in (raw_path, result_path):
                try:
                    if os.path.exists(path) and os.path.getsize(path) == 0:
                        os.unlink(path)
                except OSError:
                    pass
            raise RuntimeError(
                f"cannot open paired video writers with codec "
                f"{self.config.codec}"
            )
        self._log_info(
            f"{self.stream_name} recording: {raw_path} | {result_path}"
        )
        return raw_writer, result_writer, raw_size, result_size

    def _run(self) -> None:
        raw_writer = None
        result_writer = None
        raw_size = None
        result_size = None
        session_dir = None
        segment_stamp = None
        last_stamp = None
        last_activity_monotonic = None
        part_index = 0
        try:
            while not self._stop.is_set() or not self._queue.empty():
                try:
                    item = self._queue.get(timeout=0.2)
                except queue.Empty:
                    if (
                        raw_writer is not None
                        and last_activity_monotonic is not None
                        and time.monotonic() - last_activity_monotonic
                        >= self.config.inactivity_seconds
                    ):
                        raw_writer.release()
                        result_writer.release()
                        raw_writer = None
                        result_writer = None
                        raw_size = None
                        result_size = None
                        session_dir = None
                        segment_stamp = None
                        last_stamp = None
                        last_activity_monotonic = None
                    continue
                try:
                    item_raw_size = (item.raw.shape[1], item.raw.shape[0])
                    item_result_size = (
                        item.processed.shape[1], item.processed.shape[0]
                    )
                    new_session = (
                        last_stamp is None
                        or item.stamp_seconds < last_stamp
                        or item.stamp_seconds - last_stamp
                        > self.config.inactivity_seconds
                    )
                    new_segment = (
                        segment_stamp is not None
                        and item.stamp_seconds - segment_stamp
                        >= self.config.segment_seconds
                    )
                    size_changed = (
                        raw_size is not None
                        and (raw_size != item_raw_size
                             or result_size != item_result_size)
                    )
                    if (raw_writer is None or new_session
                            or new_segment or size_changed):
                        if raw_writer is not None:
                            raw_writer.release()
                            result_writer.release()
                        if new_session:
                            session_dir = _allocate_session_directory(
                                self.config.root,
                                _date_folder(item.stamp_seconds),
                                self.stream_name,
                            )
                            part_index = 0
                        else:
                            part_index += 1
                        segment_stamp = item.stamp_seconds
                        (
                            raw_writer,
                            result_writer,
                            raw_size,
                            result_size,
                        ) = self._open_writers(
                            item, session_dir, part_index
                        )
                    raw_writer.write(item.raw)
                    result_writer.write(item.processed)
                    last_stamp = item.stamp_seconds
                    last_activity_monotonic = time.monotonic()
                except (OSError, RuntimeError, cv2.error) as error:
                    self._log_error(
                        f"{self.stream_name} video recording failed: {error}"
                    )
                    if raw_writer is not None:
                        raw_writer.release()
                        result_writer.release()
                    raw_writer = None
                    result_writer = None
                    raw_size = None
                    result_size = None
                    last_stamp = item.stamp_seconds
                finally:
                    self._queue.task_done()
        finally:
            if raw_writer is not None:
                raw_writer.release()
                result_writer.release()
