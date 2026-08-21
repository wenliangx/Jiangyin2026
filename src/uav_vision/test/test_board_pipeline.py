#!/usr/bin/env python3
import unittest

import numpy as np

from uav_vision.board_pipeline import (
    Detection,
    DetectorConfig,
    TargetRecognitionPipeline,
)
from uav_vision.target_matcher import MatchResult


class DetectorConfigTest(unittest.TestCase):
    def test_competition_defaults_use_wide_roi(self):
        config = DetectorConfig()
        self.assertEqual(config.input_size, 512)
        self.assertAlmostEqual(config.roi_padding, 0.75)
        self.assertAlmostEqual(config.confidence, 0.10)
        self.assertAlmostEqual(config.processing_fps, 10.0)

    def test_rejects_unknown_settings(self):
        with self.assertRaises(ValueError):
            DetectorConfig.from_mapping({"not_a_setting": 1})


class TargetRecognitionGeometryTest(unittest.TestCase):
    def test_detector_box_becomes_clockwise_quad(self):
        detection = Detection((10.0, 20.0, 50.0, 70.0), 0.9)
        corners = TargetRecognitionPipeline._box_corners(detection)
        np.testing.assert_allclose(
            corners,
            np.asarray(
                ((10.0, 20.0), (50.0, 20.0), (50.0, 70.0), (10.0, 70.0)),
                dtype=np.float32,
            ),
        )

    def test_padded_crop_is_clipped_to_frame(self):
        frame = np.zeros((100, 200, 3), dtype=np.uint8)
        crop, x_offset, y_offset = TargetRecognitionPipeline._clip_crop(
            frame, (0.0, 20.0, 40.0, 60.0), 0.75
        )
        self.assertEqual((x_offset, y_offset), (0, 0))
        self.assertEqual(crop.shape[:2], (90, 70))

    def test_roi_corners_are_translated_back_to_full_frame(self):
        result = MatchResult(
            valid=True,
            label="car",
            corners=np.asarray(
                ((1.0, 2.0), (11.0, 2.0), (11.0, 12.0), (1.0, 12.0)),
                dtype=np.float32,
            ),
        )
        moved = TargetRecognitionPipeline._move_to_frame(result, 30, 40)
        np.testing.assert_allclose(
            moved.corners,
            np.asarray(
                ((31.0, 42.0), (41.0, 42.0), (41.0, 52.0), (31.0, 52.0)),
                dtype=np.float32,
            ),
        )


if __name__ == "__main__":
    unittest.main()
