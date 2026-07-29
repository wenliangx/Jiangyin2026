#!/usr/bin/env python3
import pathlib
import tempfile
import unittest

import cv2
import numpy as np

from uav_vision.target_matcher import MatcherConfig, TargetMatcher, order_quad


LABELS = ("plane", "car", "ship", "house")


def make_pattern(label):
    image = np.full((256, 256, 3), 245, np.uint8)
    if label == "plane":
        cv2.line(image, (30, 128), (226, 128), (30, 30, 30), 18)
        cv2.line(image, (128, 35), (128, 220), (30, 30, 30), 12)
    elif label == "car":
        cv2.rectangle(image, (40, 85), (216, 175), (180, 80, 20), -1)
        cv2.circle(image, (80, 185), 24, (20, 20, 20), -1)
        cv2.circle(image, (180, 185), 24, (20, 20, 20), -1)
    elif label == "ship":
        image[:] = (180, 120, 40)
        cv2.fillConvexPoly(
            image,
            np.array([[35, 170], [220, 170], [180, 220], [70, 220]]),
            (240, 240, 240),
        )
        cv2.rectangle(image, (95, 70), (165, 170), (240, 240, 240), -1)
    else:
        image[:] = (210, 180, 130)
        cv2.rectangle(image, (60, 80), (196, 225), (230, 230, 230), -1)
        cv2.fillConvexPoly(
            image,
            np.array([[45, 90], [128, 25], [211, 90]]),
            (40, 40, 160),
        )
        cv2.rectangle(image, (108, 155), (148, 225), (50, 80, 120), -1)
    return image


def write_templates(directory):
    directory = pathlib.Path(directory)
    for label in LABELS:
        ok = cv2.imwrite(str(directory / f"{label}.png"), make_pattern(label))
        if not ok:
            raise RuntimeError(f"failed to write {label}")


def test_config(**overrides):
    values = dict(
        canonical_size=256,
        border_crop_fraction=0.0,
        canny_low=30,
        canny_high=100,
        min_area_ratio=0.01,
        max_area_ratio=0.8,
        polygon_epsilon_ratio=0.03,
        min_side_ratio=0.65,
        max_angle_cosine=0.35,
        max_candidates=5,
        min_target_side_px=50.0,
        min_sharpness=0.0,
        gray_weight=0.50,
        hog_weight=0.30,
        color_weight=0.20,
        min_class_score=0.55,
        min_class_margin=0.02,
        min_candidate_margin=0.01,
        augmentations=({},),
    )
    values.update(overrides)
    return MatcherConfig(**values)


class TargetMatcherTest(unittest.TestCase):
    def test_orders_quad_corners_clockwise_from_top_left(self):
        points = np.float32([[90, 90], [10, 10], [90, 10], [10, 90]])
        ordered = order_quad(points)
        np.testing.assert_allclose(
            ordered,
            np.float32([[10, 10], [90, 10], [90, 90], [10, 90]]),
        )

    def test_exact_template_scores_higher_than_other_class(self):
        with tempfile.TemporaryDirectory() as directory:
            write_templates(directory)
            matcher = TargetMatcher(test_config(), directory)
            result = matcher.classify_patch(make_pattern("plane"))
            self.assertTrue(result.valid, result.reason)
            self.assertEqual(result.label, "plane")
            self.assertGreater(result.score, result.second_score)

    def test_rejects_ambiguous_patch(self):
        with tempfile.TemporaryDirectory() as directory:
            write_templates(directory)
            matcher = TargetMatcher(
                test_config(min_class_margin=0.20), directory
            )
            result = matcher.classify_patch(
                np.full((256, 256, 3), 127, np.uint8)
            )
            self.assertFalse(result.valid)
            self.assertEqual(result.label, "unknown")

    def test_match_frame_finds_embedded_square(self):
        with tempfile.TemporaryDirectory() as directory:
            write_templates(directory)
            matcher = TargetMatcher(test_config(), directory)
            frame = np.full((480, 640, 3), 25, np.uint8)
            frame[100:356, 180:436] = make_pattern("car")
            result = matcher.match_frame(frame)
            self.assertTrue(result.valid, result.reason)
            self.assertEqual(result.label, "car")
            self.assertEqual(result.corners.shape, (4, 2))
            self.assertGreater(result.target_side_px, 200.0)

    def test_blank_frame_is_unknown(self):
        with tempfile.TemporaryDirectory() as directory:
            write_templates(directory)
            matcher = TargetMatcher(test_config(), directory)
            result = matcher.match_frame(np.full((480, 640, 3), 80, np.uint8))
            self.assertFalse(result.valid)
            self.assertEqual(result.label, "unknown")
            self.assertEqual(result.reason, "no_square_candidate")


if __name__ == "__main__":
    unittest.main()
