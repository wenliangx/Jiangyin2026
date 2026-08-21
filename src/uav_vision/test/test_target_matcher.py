#!/usr/bin/env python3
import pathlib
import tempfile
import unittest
from unittest import mock

import cv2
import numpy as np

from uav_vision.target_matcher import (
    MatchResult,
    MatcherConfig,
    TargetMatcher,
    order_quad,
)


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
            for label in LABELS:
                with self.subTest(label=label):
                    result = matcher.classify_patch(make_pattern(label))
                    self.assertTrue(result.valid, result.reason)
                    self.assertEqual(result.label, label)
                    self.assertGreater(result.score, result.second_score)

    def test_vectorized_score_matches_legacy_template_loop(self):
        with tempfile.TemporaryDirectory() as directory:
            write_templates(directory)
            matcher = TargetMatcher(test_config(), directory)
            patch = make_pattern("ship")
            result = matcher.classify_patch(patch)
            candidate = matcher._extract_features(patch)
            legacy_scores = []
            for label in LABELS:
                best = max(
                    matcher._score_features(candidate, template)
                    for template in matcher._templates[label]
                )
                legacy_scores.append((best[0], label))
            legacy_scores.sort(reverse=True)

            self.assertEqual(result.label, legacy_scores[0][1])
            self.assertAlmostEqual(result.score, legacy_scores[0][0], places=5)
            self.assertAlmostEqual(
                result.second_score, legacy_scores[1][0], places=5
            )

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

    def test_white_board_candidate_survives_brick_mortar_background(self):
        with tempfile.TemporaryDirectory() as directory:
            write_templates(directory)
            matcher = TargetMatcher(test_config(), directory)
            frame = np.full((480, 640, 3), (35, 35, 150), np.uint8)
            for y in range(30, 470, 30):
                cv2.line(frame, (0, y), (239, y), (235, 235, 235), 2)
                cv2.line(frame, (400, y), (639, y), (235, 235, 235), 2)
            board = cv2.resize(make_pattern("plane"), (160, 160))
            frame[150:310, 240:400] = board

            candidates = matcher._find_white_board_candidates(
                frame, float(frame.shape[0] * frame.shape[1])
            )
            self.assertTrue(candidates)
            centers = [candidate[2].mean(axis=0) for candidate in candidates]
            self.assertTrue(
                any(np.linalg.norm(center - (320, 230)) < 10 for center in centers)
            )

    def test_blue_fallback_finds_blue_poster(self):
        with tempfile.TemporaryDirectory() as directory:
            write_templates(directory)
            matcher = TargetMatcher(test_config(), directory)
            frame = np.full((480, 640, 3), (35, 35, 150), np.uint8)
            frame[155:305, 245:395] = (180, 120, 40)

            candidates = matcher._find_blue_board_fallback_candidates(frame)

            self.assertTrue(candidates)
            centers = [candidate[2].mean(axis=0) for candidate in candidates]
            self.assertTrue(
                any(np.linalg.norm(center - (320, 230)) < 10 for center in centers)
            )

    def test_blank_frame_is_unknown(self):
        with tempfile.TemporaryDirectory() as directory:
            write_templates(directory)
            matcher = TargetMatcher(test_config(), directory)
            result = matcher.match_frame(np.full((480, 640, 3), 80, np.uint8))
            self.assertFalse(result.valid)
            self.assertEqual(result.label, "unknown")
            self.assertIn(
                result.reason,
                ("no_square_candidate", "akaze_no_scene_features"),
            )

    def test_white_fallback_accepts_strong_candidate_after_primary_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            write_templates(directory)
            matcher = TargetMatcher(test_config(), directory)
            frame = np.full((480, 640, 3), 80, np.uint8)
            corners = np.float32(
                [[200, 120], [360, 120], [360, 280], [200, 280]]
            )
            fallback = MatchResult(
                valid=True,
                label="car",
                score=0.78,
                second_score=0.56,
                margin=0.22,
            )
            with mock.patch.object(
                matcher,
                "_match_orb_fallback",
                return_value=MatchResult(reason="orb_no_geometric_match"),
            ), mock.patch.object(
                matcher,
                "_match_akaze_fallback",
                return_value=MatchResult(reason="akaze_no_geometric_match"),
            ), mock.patch.object(
                matcher, "find_square_candidates", return_value=[]
            ), mock.patch.object(
                matcher,
                "_find_white_board_fallback_candidates",
                return_value=[(1.0, 160.0, corners)],
            ), mock.patch.object(
                matcher, "classify_patch", return_value=fallback
            ):
                result = matcher.match_frame(frame)

            self.assertTrue(result.valid, result.reason)
            self.assertEqual(result.label, "car")
            self.assertEqual(result.reason, "accepted_white_fallback")

    def test_white_fallback_rejects_weak_candidate(self):
        with tempfile.TemporaryDirectory() as directory:
            write_templates(directory)
            matcher = TargetMatcher(test_config(), directory)
            frame = np.full((480, 640, 3), 80, np.uint8)
            corners = np.float32(
                [[200, 120], [360, 120], [360, 280], [200, 280]]
            )
            weak = MatchResult(
                valid=True,
                label="ship",
                score=0.69,
                second_score=0.50,
                margin=0.19,
            )
            with mock.patch.object(
                matcher,
                "_match_orb_fallback",
                return_value=MatchResult(reason="orb_no_geometric_match"),
            ), mock.patch.object(
                matcher,
                "_match_akaze_fallback",
                return_value=MatchResult(reason="akaze_no_geometric_match"),
            ), mock.patch.object(
                matcher, "find_square_candidates", return_value=[]
            ), mock.patch.object(
                matcher,
                "_find_white_board_fallback_candidates",
                return_value=[(1.0, 160.0, corners)],
            ), mock.patch.object(
                matcher, "classify_patch", return_value=weak
            ):
                result = matcher.match_frame(frame)

            self.assertFalse(result.valid)
            self.assertEqual(result.label, "unknown")
            self.assertEqual(result.reason, "white_fallback_score_too_low")

    def test_white_fallback_does_not_override_primary_result(self):
        with tempfile.TemporaryDirectory() as directory:
            write_templates(directory)
            matcher = TargetMatcher(test_config(), directory)
            frame = np.full((480, 640, 3), 80, np.uint8)
            corners = np.float32(
                [[200, 120], [360, 120], [360, 280], [200, 280]]
            )
            primary = MatchResult(
                valid=True,
                label="plane",
                score=0.75,
                second_score=0.55,
                margin=0.20,
            )
            with mock.patch.object(
                matcher,
                "_match_orb_fallback",
                return_value=MatchResult(reason="orb_no_geometric_match"),
            ), mock.patch.object(
                matcher,
                "_match_akaze_fallback",
                return_value=MatchResult(reason="akaze_no_geometric_match"),
            ), mock.patch.object(
                matcher,
                "find_square_candidates",
                return_value=[(1.0, 160.0, corners)],
            ), mock.patch.object(
                matcher, "classify_patch", return_value=primary
            ), mock.patch.object(
                matcher, "_find_white_board_fallback_candidates"
            ) as fallback_mock:
                result = matcher.match_frame(frame)

            self.assertTrue(result.valid, result.reason)
            self.assertEqual(result.label, "plane")
            fallback_mock.assert_not_called()

    def test_recording_annotation_uses_stable_class_and_box(self):
        with tempfile.TemporaryDirectory() as directory:
            write_templates(directory)
            matcher = TargetMatcher(test_config(), directory)
            result = matcher.classify_patch(make_pattern("ship"))
            result.corners = np.float32(
                [[20, 20], [120, 20], [120, 120], [20, 120]]
            )
            frame = np.zeros((160, 180, 3), np.uint8)
            annotated = matcher.annotate_recording(
                frame, result, "2026-08-20 01:02:03.456 Beijing"
            )
            self.assertEqual(annotated.shape, frame.shape)
            self.assertGreater(np.count_nonzero(annotated), 0)
            self.assertTrue(np.array_equal(frame, np.zeros_like(frame)))


if __name__ == "__main__":
    unittest.main()
