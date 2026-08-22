#!/usr/bin/env python3
import math
import pathlib
import sys
import unittest


PACKAGE_SRC_DIR = pathlib.Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(PACKAGE_SRC_DIR))

from uav_vision.control_state import DelayedDisableState


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class DelayedDisableStateTest(unittest.TestCase):
    def test_disable_stays_effective_for_exact_grace_window(self):
        clock = FakeClock()
        state = DelayedDisableState(
            disable_delay_seconds=1.0, monotonic=clock
        )

        self.assertEqual(state.update(True), (True, True))
        self.assertEqual(state.update(False), (True, False))
        self.assertFalse(state.requested_enabled)
        self.assertTrue(state.effective_enabled)
        self.assertTrue(state.grace_active)
        calls = []
        self.assertFalse(
            state.run_if_requested_enabled(lambda: calls.append("result"))
        )
        self.assertTrue(
            state.run_if_effectively_enabled(lambda: calls.append("frame"))
        )
        self.assertEqual(calls, ["frame"])

        clock.advance(0.999)
        self.assertTrue(state.effective_enabled)
        clock.advance(0.001)
        self.assertFalse(state.effective_enabled)
        self.assertFalse(state.grace_active)

    def test_repeated_disable_does_not_extend_deadline(self):
        clock = FakeClock()
        state = DelayedDisableState(
            initial_enabled=True,
            disable_delay_seconds=1.0,
            monotonic=clock,
        )
        state.update(False)
        clock.advance(0.75)
        self.assertEqual(state.update(False), (False, False))
        clock.advance(0.25)
        self.assertFalse(state.effective_enabled)

    def test_reenable_cancels_pending_disable(self):
        clock = FakeClock()
        state = DelayedDisableState(
            initial_enabled=True,
            disable_delay_seconds=1.0,
            monotonic=clock,
        )
        state.update(False)
        clock.advance(0.5)
        self.assertEqual(state.update(True), (True, True))
        clock.advance(2.0)
        self.assertTrue(state.effective_enabled)
        self.assertFalse(state.grace_active)

    def test_zero_delay_preserves_immediate_disable(self):
        state = DelayedDisableState(initial_enabled=True)
        state.update(False)
        self.assertFalse(state.effective_enabled)

    def test_rejects_invalid_delay(self):
        for value in (-1.0, math.inf, math.nan):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    DelayedDisableState(disable_delay_seconds=value)


if __name__ == "__main__":
    unittest.main()
