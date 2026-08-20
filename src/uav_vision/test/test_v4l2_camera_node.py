#!/usr/bin/env python3
import pathlib
import sys
import unittest


SCRIPTS_DIR = pathlib.Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

import v4l2_camera_node


class FakeVisionControl:
    def __init__(self, front=False, down=False):
        self.front_camera_enabled = front
        self.down_camera_enabled = down


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


class CameraControlStateTest(unittest.TestCase):
    def test_rejects_unknown_camera_role(self):
        with self.assertRaises(ValueError):
            v4l2_camera_node.CameraControlState("side")

    def test_front_camera_uses_only_front_field(self):
        state = v4l2_camera_node.CameraControlState("front")
        self.assertFalse(state.desired_enabled)

        changed, enabled = state.update(FakeVisionControl(down=True))
        self.assertFalse(changed)
        self.assertFalse(enabled)

        changed, enabled = state.update(FakeVisionControl(front=True))
        self.assertTrue(changed)
        self.assertTrue(enabled)
        self.assertTrue(state.desired_enabled)

    def test_down_camera_updates_are_idempotent(self):
        state = v4l2_camera_node.CameraControlState("down")

        changed, enabled = state.update(FakeVisionControl(down=True))
        self.assertTrue(changed)
        self.assertTrue(enabled)

        changed, enabled = state.update(FakeVisionControl(down=True))
        self.assertFalse(changed)
        self.assertTrue(enabled)

        changed, enabled = state.update(FakeVisionControl(down=False))
        self.assertTrue(changed)
        self.assertFalse(enabled)

    def test_always_enabled_ignores_disable_requests(self):
        state = v4l2_camera_node.CameraControlState(
            "front", always_enabled=True
        )
        self.assertTrue(state.desired_enabled)

        changed, enabled = state.update(FakeVisionControl(front=False))
        self.assertFalse(changed)
        self.assertTrue(enabled)

    def test_run_if_enabled_guards_the_final_publish(self):
        state = v4l2_camera_node.CameraControlState("front")
        calls = []

        self.assertFalse(state.run_if_enabled(lambda: calls.append("frame")))
        self.assertEqual([], calls)

        state.update(FakeVisionControl(front=True))
        self.assertTrue(state.run_if_enabled(lambda: calls.append("frame")))
        self.assertEqual(["frame"], calls)

        state.update(FakeVisionControl(front=False))
        self.assertFalse(state.run_if_enabled(lambda: calls.append("frame")))
        self.assertEqual(["frame"], calls)


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
