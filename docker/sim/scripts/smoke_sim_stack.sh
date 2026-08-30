#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /ws/devel/setup.bash
exec python3 /usr/local/libexec/smoke_sim_stack.py "$@"
