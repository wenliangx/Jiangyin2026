#!/usr/bin/env python3
import pathlib
import sys
import unittest


SCRIPTS_DIR = pathlib.Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

import landing_debug_web


class LatestJpegFrameTest(unittest.TestCase):
    def test_stores_latest_frame(self):
        frame = landing_debug_web.LatestJpegFrame()
        self.assertIsNone(frame.get())

        frame.update(b"first")
        frame.update(b"latest")

        self.assertEqual(frame.get(), b"latest")


if __name__ == "__main__":
    unittest.main()
