#!/usr/bin/env python3

import copy
import importlib.util
import pathlib
import tempfile
import types
import unittest


PACKAGE = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "waypoint_validator", PACKAGE / "scripts" / "validate_super_waypoints.py")
VALIDATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VALIDATOR)


class WaypointValidationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _, cls.waypoints = VALIDATOR.load_yaml(
            PACKAGE / "config" / "mission_super_waypoints.yaml")
        _, cls.rules = VALIDATOR.load_yaml(
            PACKAGE / "config" / "waypoint_validation_rules.yaml")

    def test_repository_mission_is_valid(self):
        self.assertEqual([], VALIDATOR.validate(self.waypoints, self.rules))

    def test_rejects_wrong_segment_count(self):
        waypoints = copy.deepcopy(self.waypoints)
        waypoints["segments"].pop()
        self.assertTrue(VALIDATOR.validate(waypoints, self.rules))

    def test_rejects_non_numeric_waypoint(self):
        waypoints = copy.deepcopy(self.waypoints)
        waypoints["segments"][0]["waypoints"][0]["x"] = "not-a-number"
        self.assertTrue(VALIDATOR.validate(waypoints, self.rules))

    def test_rejects_relationship_violation(self):
        waypoints = copy.deepcopy(self.waypoints)
        points = waypoints["segments"][2]["waypoints"]
        points[2]["x"] += 0.1
        self.assertTrue(VALIDATOR.validate(waypoints, self.rules))

    def test_sealed_writable_file_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            seal_args = types.SimpleNamespace(
                waypoints=str(PACKAGE / "config" / "mission_super_waypoints.yaml"),
                rules=str(PACKAGE / "config" / "waypoint_validation_rules.yaml"),
                output_dir=directory,
            )
            self.assertEqual(0, VALIDATOR.seal(seal_args))
            waypoint = pathlib.Path(directory) / "mission_super_waypoints.validated.yaml"
            waypoint.chmod(0o644)
            verify_args = types.SimpleNamespace(
                verify_manifest=str(pathlib.Path(directory) /
                                    "waypoint_validation_manifest.yaml"),
                expected_waypoints=str(waypoint),
            )
            self.assertEqual(1, VALIDATOR.verify(verify_args))


if __name__ == "__main__":
    unittest.main()
