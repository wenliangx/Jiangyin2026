#!/usr/bin/env python3
import unittest

from uav_vision.temporal_vote import TemporalVoter


class TemporalVoterTest(unittest.TestCase):
    def test_unknown_occupies_window_without_voting(self):
        voter = TemporalVoter(window_size=3, min_votes=2, lost_frames=3)
        self.assertIsNone(voter.update("plane"))
        self.assertIsNone(voter.update(None))
        self.assertEqual(voter.update("plane"), "plane")

    def test_stable_label_bridges_short_unknown_gap(self):
        voter = TemporalVoter(window_size=3, min_votes=2, lost_frames=3)
        self.assertIsNone(voter.update("plane"))
        self.assertEqual(voter.update("plane"), "plane")
        self.assertEqual(voter.update(None), "plane")
        self.assertEqual(voter.update(None), "plane")
        self.assertIsNone(voter.update(None))

    def test_consecutive_unknown_clears_history(self):
        voter = TemporalVoter(window_size=5, min_votes=2, lost_frames=2)
        self.assertIsNone(voter.update("car"))
        self.assertEqual(voter.update("car"), "car")
        self.assertEqual(voter.update(None), "car")
        self.assertIsNone(voter.update(None))
        self.assertIsNone(voter.update("car"))

    def test_tie_is_not_stable(self):
        voter = TemporalVoter(window_size=4, min_votes=2, lost_frames=4)
        voter.update("plane")
        voter.update("car")
        voter.update("plane")
        self.assertIsNone(voter.update("car"))

    def test_invalid_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            TemporalVoter(window_size=0, min_votes=1, lost_frames=1)
        with self.assertRaises(ValueError):
            TemporalVoter(window_size=3, min_votes=4, lost_frames=1)
        with self.assertRaises(ValueError):
            TemporalVoter(window_size=3, min_votes=1, lost_frames=0)


if __name__ == "__main__":
    unittest.main()
