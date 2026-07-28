#!/usr/bin/env python3
import pathlib
import sys
import unittest


SCRIPTS_DIR = pathlib.Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

import v4l2_camera_node


class CameraSettingsTest(unittest.TestCase):
    def test_accepts_competition_settings(self):
        self.assertEqual(
            v4l2_camera_node.validate_settings(1280, 720, 30.0, 150, 5),
            (1280, 720, 30.0, 150, 5),
        )

    def test_rejects_invalid_ranges(self):
        invalid = [
            (0, 720, 30.0, 150, 5),
            (1280, 0, 30.0, 150, 5),
            (1280, 720, 0.0, 150, 5),
            (1280, 720, 30.0, 0, 5),
            (1280, 720, 30.0, 5001, 5),
            (1280, 720, 30.0, 150, -1),
            (1280, 720, 30.0, 150, 101),
        ]
        for settings in invalid:
            with self.subTest(settings=settings):
                with self.assertRaises(ValueError):
                    v4l2_camera_node.validate_settings(*settings)

    def test_builds_manual_exposure_controls(self):
        self.assertEqual(
            v4l2_camera_node.control_requests(150, 5),
            [
                (v4l2_camera_node.V4L2_CID_EXPOSURE_AUTO, 1),
                (v4l2_camera_node.V4L2_CID_EXPOSURE_ABSOLUTE, 150),
                (v4l2_camera_node.V4L2_CID_GAIN, 5),
                (
                    v4l2_camera_node.V4L2_CID_EXPOSURE_AUTO_PRIORITY,
                    0,
                ),
            ],
        )


class ControlApplicationTest(unittest.TestCase):
    def test_applies_and_verifies_all_controls(self):
        class FakeControls:
            def __init__(self):
                self.values = {}

            def set(self, control_id, value):
                self.values[control_id] = value

            def get(self, control_id):
                return self.values[control_id]

        controls = FakeControls()
        v4l2_camera_node.apply_controls(controls, 150, 5)
        self.assertEqual(
            controls.values,
            dict(v4l2_camera_node.control_requests(150, 5)),
        )

    def test_rejects_control_readback_mismatch(self):
        class MismatchedControls:
            def set(self, control_id, value):
                pass

            def get(self, control_id):
                return -1

        with self.assertRaises(RuntimeError):
            v4l2_camera_node.apply_controls(
                MismatchedControls(),
                150,
                5,
            )


if __name__ == "__main__":
    unittest.main()
