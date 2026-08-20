#!/usr/bin/env python3
import pathlib
import tempfile
import unittest

import cv2
import numpy as np

from uav_vision.video_recorder import (
    AsyncVideoPairRecorder,
    RecorderConfig,
    beijing_timestamp,
)


class VideoRecorderTest(unittest.TestCase):
    def test_formats_beijing_timestamp_independent_of_host_timezone(self):
        self.assertEqual(
            beijing_timestamp(0.001),
            "1970-01-01 08:00:00.001 Beijing",
        )

    def test_writes_synchronized_raw_and_result_mp4_files(self):
        with tempfile.TemporaryDirectory() as directory:
            config = RecorderConfig(
                root=directory,
                fps=10.0,
                codec="mp4v",
                extension=".mp4",
                segment_seconds=60.0,
                inactivity_seconds=3.0,
                queue_size=8,
            )
            recorder = AsyncVideoPairRecorder(config, "test_stream")
            stamp = 1700000000.0
            for index in range(5):
                raw = np.full((48, 64, 3), index * 20, np.uint8)
                result = raw.copy()
                cv2.rectangle(result, (8, 8), (32, 32), (0, 255, 0), 2)
                self.assertTrue(
                    recorder.submit(raw, result, stamp + index * 0.1)
                )
            recorder.close()

            files = sorted(pathlib.Path(directory).rglob("*.mp4"))
            self.assertEqual(len(files), 2)
            self.assertEqual(files[0].name, "raw.mp4")
            self.assertEqual(files[1].name, "result.mp4")
            self.assertEqual(files[0].parent.name, "001")
            self.assertEqual(files[1].parent.name, "001")

            frame_counts = []
            for path in files:
                capture = cv2.VideoCapture(str(path))
                self.assertTrue(capture.isOpened(), path)
                frame_counts.append(int(capture.get(cv2.CAP_PROP_FRAME_COUNT)))
                capture.release()
            self.assertEqual(frame_counts, [5, 5])

    def test_camera_gap_allocates_next_numbered_session_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            config = RecorderConfig(
                root=directory,
                fps=10.0,
                codec="mp4v",
                extension=".mp4",
                segment_seconds=60.0,
                inactivity_seconds=1.0,
                queue_size=8,
            )
            recorder = AsyncVideoPairRecorder(config, "test_stream")
            frame = np.zeros((24, 32, 3), np.uint8)
            self.assertTrue(recorder.submit(frame, frame, 1700000000.0))
            self.assertTrue(recorder.submit(frame, frame, 1700000002.0))
            recorder.close()
            session_dirs = sorted(
                path.parent.name
                for path in pathlib.Path(directory).rglob("raw.mp4")
            )
            self.assertEqual(session_dirs, ["001", "002"])

    def test_rejects_invalid_codec(self):
        with self.assertRaises(ValueError):
            RecorderConfig(codec="bad").validate()


if __name__ == "__main__":
    unittest.main()
