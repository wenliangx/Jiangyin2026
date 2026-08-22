#!/usr/bin/env python3
"""Validate and seal SUPER waypoint YAML files for flight use."""

import argparse
import datetime
import hashlib
import math
import os
import pathlib
import shutil
import stat
import sys
import tempfile

import yaml


FIELDS = {"x", "y", "z", "yaw", "desired_speed"}


class ValidationError(Exception):
    pass


def load_yaml(path):
    try:
        with open(path, "rb") as stream:
            raw = stream.read()
        data = yaml.safe_load(raw)
    except (OSError, yaml.YAMLError) as error:
        raise ValidationError(f"{path}: {error}")
    if not isinstance(data, dict):
        raise ValidationError(f"{path}: root must be a mapping")
    return raw, data


def number(value, location, integer=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValidationError(f"{location}: expected numeric value")
    result = int(value) if integer else float(value)
    if integer and result != value:
        raise ValidationError(f"{location}: expected integer")
    if not math.isfinite(float(result)):
        raise ValidationError(f"{location}: value must be finite")
    return result


def circular_difference(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def validate(waypoints, rules):
    errors = []

    def check(condition, message):
        if not condition:
            errors.append(message)

    check(rules.get("schema_version") == 1,
          "rules schema_version must equal 1")
    segments = waypoints.get("segments")
    check(isinstance(waypoints.get("frame_id"), str), "frame_id must be a string")
    check(isinstance(segments, list), "segments must be a sequence")
    if not isinstance(segments, list):
        return errors

    expected_frame = rules.get("required_frame_id")
    if expected_frame is not None:
        check(waypoints.get("frame_id") == expected_frame,
              f"frame_id must equal {expected_frame!r}")
    expected_segments = rules.get("required_segment_count")
    if expected_segments is not None:
        try:
            expected_segments = number(expected_segments,
                                       "required_segment_count", True)
            check(len(segments) == expected_segments,
                  f"segment count must be {expected_segments}, got {len(segments)}")
        except ValidationError as error:
            errors.append(str(error))

    waypoint_rules = rules.get("waypoint_rules", {})
    segment_rules = rules.get("segment_rules", [])
    tolerance = float(rules.get("equality_tolerance", 1e-4))
    parsed = []
    for si, segment in enumerate(segments):
        location = f"segments[{si}]"
        if not isinstance(segment, dict):
            errors.append(f"{location}: must be a mapping")
            parsed.append([])
            continue
        points = segment.get("waypoints")
        if not isinstance(points, list) or not points:
            errors.append(f"{location}.waypoints: must be a non-empty sequence")
            parsed.append([])
            continue
        parsed_segment = []
        for wi, point in enumerate(points):
            ploc = f"{location}.waypoints[{wi}]"
            if not isinstance(point, dict):
                errors.append(f"{ploc}: must be a mapping")
                continue
            parsed_point = {}
            try:
                for field in waypoint_rules.get("required_fields", []):
                    if field not in point:
                        raise ValidationError(f"{ploc}.{field}: required")
                for field in ("x", "y", "z"):
                    if field not in point:
                        raise ValidationError(f"{ploc}.{field}: required")
                    parsed_point[field] = number(point[field], f"{ploc}.{field}")
                parsed_point["id"] = number(point.get("id", wi), f"{ploc}.id", True)
                parsed_point["mode"] = number(point.get("mode", 2), f"{ploc}.mode", True)
                parsed_point["is_map"] = number(point.get("is_map", 1), f"{ploc}.is_map", True)
                parsed_point["desired_speed"] = number(
                    point.get("desired_speed", 0.0), f"{ploc}.desired_speed")
                yaw = point.get("yaw", float("nan"))
                if isinstance(yaw, str) and yaw.lower() in ("nan", ".nan", "free", "auto"):
                    parsed_point["yaw"] = float("nan")
                else:
                    parsed_point["yaw"] = number(yaw, f"{ploc}.yaw")
                if (not waypoint_rules.get("allow_auto_yaw", True) and
                        not math.isfinite(parsed_point["yaw"])):
                    raise ValidationError(f"{ploc}.yaw: must be numeric")
                check(parsed_point["mode"] in waypoint_rules.get("allowed_modes", [0, 1, 2]),
                      f"{ploc}.mode: not allowed")
                check(parsed_point["is_map"] in waypoint_rules.get("allowed_is_map", [0, 1]),
                      f"{ploc}.is_map: not allowed")
                check(parsed_point["desired_speed"] >= waypoint_rules.get("min_desired_speed", 0.0),
                      f"{ploc}.desired_speed: below minimum")
                for field, limit_key in (("x", "max_abs_x"), ("y", "max_abs_y")):
                    if limit_key in waypoint_rules:
                        check(abs(parsed_point[field]) <= float(waypoint_rules[limit_key]),
                              f"{ploc}.{field}: exceeds {limit_key}")
                if "min_z" in waypoint_rules:
                    check(parsed_point["z"] >= float(waypoint_rules["min_z"]),
                          f"{ploc}.z: below minimum")
                if "max_z" in waypoint_rules:
                    check(parsed_point["z"] <= float(waypoint_rules["max_z"]),
                          f"{ploc}.z: above maximum")
                if waypoint_rules.get("require_sequential_ids", False):
                    first_id = int(waypoint_rules.get("first_id", 0))
                    check(parsed_point["id"] == first_id + wi,
                          f"{ploc}.id: expected {first_id + wi}")
                parsed_segment.append(parsed_point)
            except ValidationError as error:
                errors.append(str(error))
        parsed.append(parsed_segment)

    for ri, rule in enumerate(segment_rules):
        try:
            si = number(rule["segment"], f"segment_rules[{ri}].segment", True)
            if si < 0 or si >= len(segments):
                raise ValidationError(f"segment_rules[{ri}]: segment index out of range")
            if "required_name" in rule:
                check(segments[si].get("name") == rule["required_name"],
                      f"segments[{si}].name must equal {rule['required_name']!r}")
            if "waypoint_count" in rule:
                expected = number(rule["waypoint_count"],
                                  f"segment_rules[{ri}].waypoint_count", True)
                actual = len(segments[si].get("waypoints", []))
                check(actual == expected,
                      f"segments[{si}] waypoint count must be {expected}, got {actual}")
        except (KeyError, TypeError, ValidationError) as error:
            errors.append(f"segment_rules[{ri}]: {error}")

    for ri, relation in enumerate(rules.get("relationships", [])):
        try:
            si = int(relation["segment"])
            ai = int(relation["waypoint_a"])
            bi = int(relation["waypoint_b"])
            a, b = parsed[si][ai], parsed[si][bi]
            for mode in ("same", "different"):
                for field in relation.get(mode, []):
                    if field not in FIELDS:
                        raise ValidationError(f"unknown relationship field {field!r}")
                    av, bv = a[field], b[field]
                    if not math.isfinite(av) or not math.isfinite(bv):
                        raise ValidationError(f"{field} cannot compare auto/free yaw")
                    difference = (circular_difference(av, bv)
                                  if field == "yaw" else abs(av - bv))
                    valid = difference <= tolerance if mode == "same" else difference > tolerance
                    check(valid,
                          f"relationships[{ri}]: {field} must be {mode} "
                          f"(difference={difference:.9g}, tolerance={tolerance})")
        except (IndexError, KeyError, TypeError, ValueError, ValidationError) as error:
            errors.append(f"relationships[{ri}]: {error}")
    return errors


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def atomic_copy_bytes(raw, destination):
    destination.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=destination.name + ".", dir=str(destination.parent))
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(raw)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, 0o444)
        os.replace(temporary, destination)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def seal(args):
    waypoint_raw, waypoints = load_yaml(args.waypoints)
    rules_raw, rules = load_yaml(args.rules)
    errors = validate(waypoints, rules)
    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1
    output = pathlib.Path(args.output_dir).resolve()
    waypoint_path = output / "mission_super_waypoints.validated.yaml"
    rules_path = output / "waypoint_validation_rules.validated.yaml"
    manifest_path = output / "waypoint_validation_manifest.yaml"
    atomic_copy_bytes(waypoint_raw, waypoint_path)
    atomic_copy_bytes(rules_raw, rules_path)
    manifest = {
        "schema_version": 1,
        "waypoint_file": waypoint_path.name,
        "rules_file": rules_path.name,
        "waypoint_sha256": sha256(waypoint_path),
        "rules_sha256": sha256(rules_path),
        "validated_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "validator_version": 1,
    }
    atomic_copy_bytes(yaml.safe_dump(manifest, sort_keys=False).encode(), manifest_path)
    print(f"VALID: {len(waypoints['segments'])} segments")
    print(f"SEALED: {waypoint_path}")
    print(f"MANIFEST: {manifest_path}")
    return 0


def verify(args):
    manifest_path = pathlib.Path(args.verify_manifest).resolve()
    _, manifest = load_yaml(manifest_path)
    base = manifest_path.parent
    waypoint_path = (base / manifest["waypoint_file"]).resolve()
    rules_path = (base / manifest["rules_file"]).resolve()
    errors = []
    if stat.S_IMODE(manifest_path.stat().st_mode) & 0o222:
        errors.append(f"manifest is writable: {manifest_path}")
    if manifest.get("schema_version") != 1:
        errors.append("manifest schema_version must equal 1")
    if args.expected_waypoints and waypoint_path != pathlib.Path(args.expected_waypoints).resolve():
        errors.append("manifest waypoint path does not match mission_super_waypoints_file")
    for path, key in ((waypoint_path, "waypoint_sha256"), (rules_path, "rules_sha256")):
        if not path.is_file():
            errors.append(f"sealed file missing: {path}")
            continue
        if stat.S_IMODE(path.stat().st_mode) & 0o222:
            errors.append(f"sealed file is writable: {path}")
        if sha256(path) != manifest.get(key):
            errors.append(f"SHA-256 mismatch: {path}")
    if not errors:
        _, waypoints = load_yaml(waypoint_path)
        _, rules = load_yaml(rules_path)
        errors.extend(validate(waypoints, rules))
    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print(f"VALIDATED SEALED WAYPOINTS: {waypoint_path}")
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--waypoints")
    parser.add_argument("--rules")
    parser.add_argument("--output-dir")
    parser.add_argument("--verify-manifest")
    parser.add_argument("--expected-waypoints")
    args = parser.parse_args()
    if args.verify_manifest:
        return verify(args)
    if not args.waypoints or not args.rules or not args.output_dir:
        parser.error("seal mode requires --waypoints, --rules, and --output-dir")
    return seal(args)


if __name__ == "__main__":
    sys.exit(main())
