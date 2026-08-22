# SUPER waypoint preflight validation

Edit only the source waypoint and rule files:

- `src/fsm_ctrl/config/mission_super_waypoints.yaml`
- `src/fsm_ctrl/config/waypoint_validation_rules.yaml`

Seal a validated snapshot before flight:

```bash
rosrun fsm_ctrl validate_super_waypoints.py \
  --waypoints "$(rospack find fsm_ctrl)/config/mission_super_waypoints.yaml" \
  --rules "$(rospack find fsm_ctrl)/config/waypoint_validation_rules.yaml" \
  --output-dir "$(rospack find fsm_ctrl)/config/validated"
```

The command validates YAML syntax, the exact segment count and names, each
segment's waypoint count, numeric waypoint fields, ID continuity, numeric
limits, and configured same/different field relationships. It then writes a
read-only waypoint snapshot, a read-only rules snapshot, and a read-only
SHA-256 manifest.

`single_sml.launch` and `single_sml2.launch` load only the sealed waypoint
snapshot. At startup `single_offboard_sml` invokes the same validator in
manifest verification mode. A writable file, hash mismatch, invalid rule, or
invalid waypoint aborts node startup; there is no built-in waypoint fallback.

After changing either source file, rerun the sealing command and restart the
node. Do not edit files under `config/validated/` directly.
