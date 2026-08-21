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


class FakeStamp:
    def __init__(self, seconds):
        self._seconds = float(seconds)

    def to_sec(self):
        return self._seconds


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

    def test_processing_rate_gate_keeps_ten_hz_from_thirty_hz_input(self):
        node = target_match_node.TargetMatchNode.__new__(
            target_match_node.TargetMatchNode
        )
        node._processing_period = 0.1
        node._last_processed_stamp = None
        accepted = [
            node._should_process(FakeStamp(100.0 + index / 30.0))
            for index in range(7)
        ]
        self.assertEqual(accepted, [True, False, False, True, False, False, True])


if __name__ == "__main__":
    unittest.main()
