#!/usr/bin/env python3
import pathlib
import sys
import unittest

import cv2
import numpy as np


SCRIPTS_DIR = pathlib.Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

import camera_focus_view


class FocusMetricTest(unittest.TestCase):
    def test_uniform_image_has_zero_sharpness(self):
        frame = np.full((64, 64, 3), 127, np.uint8)
        self.assertEqual(camera_focus_view.measure_sharpness(frame), 0.0)

    def test_checkerboard_is_sharper_than_uniform_image(self):
        uniform = np.full((64, 64, 3), 127, np.uint8)
        checker = (np.indices((64, 64)).sum(axis=0) // 8) % 2
        checker = cv2.cvtColor(
            (checker * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR
        )
        self.assertGreater(
            camera_focus_view.measure_sharpness(checker),
            camera_focus_view.measure_sharpness(uniform),
        )

    def test_first_fps_sample_is_zero_then_uses_elapsed_time(self):
        meter = camera_focus_view.FrameRateMeter(smoothing=0.5)
        self.assertEqual(meter.update(10.0), 0.0)
        self.assertAlmostEqual(meter.update(10.1), 10.0)
        self.assertAlmostEqual(meter.update(10.2), 10.0)


if __name__ == "__main__":
    unittest.main()
