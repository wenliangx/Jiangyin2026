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
echo "Legacy control references:"
rg -n 'single_offboard_fsm|swarm_user_cmd|single\.launch|swarm\.launch' \
  --glob '!src/fsm_ctrl/src/single_offboard_fsm.cpp' \
  --glob '!src/fsm_ctrl/include/fsm_ctrl/single_offboard_fsm.hpp' \
  --glob '!src/fsm_ctrl/src/swarm_user_cmd.cpp' \
  --glob '!src/fsm_ctrl/include/fsm_ctrl/swarm_user_cmd.hpp' \
  --glob '!docs/repository_cleanup.md' \
  . || true
