#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "$0")/../.." && pwd)"
base_image="${PX4_SIM_BASE:-localhost/jiangyin_px4_mid360:latest}"
output_image="${SIM_STACK_IMAGE:-localhost/jiangyin_sim_stack:latest}"
build_jobs="${BUILD_JOBS:-4}"

cd "${repo_root}"
exec podman build --layers --format docker \
  --build-arg "PX4_SIM_BASE=${base_image}" \
  --build-arg "BUILD_JOBS=${build_jobs}" \
  -f docker/sim/Dockerfile.stack \
  -t "${output_image}" \
  .
