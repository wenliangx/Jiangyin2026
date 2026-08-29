#!/usr/bin/env bash
set -euo pipefail

repo_root="$(git rev-parse --show-toplevel)"
cd "${repo_root}"

echo "Tracked files: $(git ls-files | wc -l | tr -d ' ')"
echo "Tracked Catkin logs: $(git ls-files logs | wc -l | tr -d ' ')"
echo
echo "Largest tracked files:"
git ls-files -z |
  xargs -0 du -k 2>/dev/null |
  sort -n |
  tail -20 |
  awk '{printf "%8.1f MiB  %s\n", $1 / 1024, $2}'

echo
echo "Ignored Catkin packages:"
find src -name CATKIN_IGNORE -print | sort

echo
echo "Removed legacy control references:"
legacy_node='single_offboard_'fsm
legacy_launch='single.'launch
rg -n "${legacy_node}|${legacy_launch}" . || true

echo
echo "Tracked assistant artifacts:"
git ls-files | rg '(^|/)(AGENTS\.md|\.omo/|\.opencode/|\.superpowers/|docs/superpowers/)' || true
