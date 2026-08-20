#!/usr/bin/env python3
import pathlib
import sys
import unittest


SCRIPTS_DIR = pathlib.Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

import target_match_node


class FakeVisionControl:
    def __init__(self, front=False, rear=False):
        self.front_camera_enabled = front
        # Rear keeps the legacy wire field until the state-machine publisher
        # is migrated in a coordinated message-definition change.
        self.down_camera_enabled = rear


class TargetMatchCameraControlTest(unittest.TestCase):
    def test_normalizes_legacy_down_role_to_rear(self):
        self.assertEqual(target_match_node.normalize_camera_role("down"), "rear")

    def test_rejects_unknown_role(self):
        with self.assertRaises(ValueError):
            target_match_node.normalize_camera_role("side")

    def test_front_and_rear_use_independent_control_fields(self):
        front_only = FakeVisionControl(front=True)
        rear_only = FakeVisionControl(rear=True)
        self.assertTrue(
            target_match_node.requested_camera_enabled(front_only, "front")
        )
        self.assertFalse(
            target_match_node.requested_camera_enabled(front_only, "rear")
        )
        self.assertFalse(
            target_match_node.requested_camera_enabled(rear_only, "front")
        )
        self.assertTrue(
            target_match_node.requested_camera_enabled(rear_only, "rear")
        )


if __name__ == "__main__":
    unittest.main()
