#!/usr/bin/env bash
# ============================================================================
# build-arm64.sh — build the Jiangyin2026 arm64 deb packages and image on an
# x86_64 host via podman --platform linux/arm64 + QEMU aarch64 emulation.
#
# WARNING: emulated builds are ~20-100x slower than native. The PX4 SITL bake
# alone can take multiple hours. Cap PX4_BUILD_JOBS on RAM-limited machines:
#   PX4_BUILD_JOBS=4 ./docker/build-arm64.sh
# Run under tmux/screen; layers are cached so rebuilds resume incrementally.
#
# Usage:
#   docker/build-arm64.sh [debs|image|all]   (default: all)
#
# Stages:
#   debs   build arm64 debs via docker/Dockerfile.debs -> deb/arm64/
#   image  build jiangyin_jy2026:arm64 via docker/Dockerfile
#
# Runtime note: full SITL smoke (jy-smoke-test) must run on the Jetson.
# QEMU emulation does not satisfy PX4 lockstep timing for reliable SITL.
# ============================================================================
set -euo pipefail

STAGE="${1:-all}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEBS_IMAGE="jiangyin-debs-builder:arm64"
MAIN_IMAGE="jiangyin_jy2026:arm64"
ROS_BASE="docker.io/library/ros:noetic-ros-base-focal"
BUILD_JOBS="${PX4_BUILD_JOBS:-4}"

build_debs() {
  echo "[arm64] Building deb packages (QEMU emulation, this takes a while)..."
  podman build --platform linux/arm64 \
    --build-arg ROS_BASE_IMAGE="${ROS_BASE}" \
    -f "${ROOT}/docker/Dockerfile.debs" \
    -t "${DEBS_IMAGE}" "${ROOT}"

  echo "[arm64] Extracting debs into deb/arm64/ ..."
  mkdir -p "${ROOT}/deb/arm64"
  cid="$(podman create "${DEBS_IMAGE}" true)"
  podman cp "${cid}:/out/debs/." "${ROOT}/deb/arm64/"
  podman rm "${cid}"
  # arch-independent metapackages: reuse existing _all debs from deb/
  cp -n "${ROOT}"/deb/jiangyin-source-deps_*_all.deb "${ROOT}/deb/arm64/" 2>/dev/null || true
  cp -n "${ROOT}"/deb/jiangyin-ros1-deps_*_all.deb "${ROOT}/deb/arm64/" 2>/dev/null || true

  echo "[arm64] Verifying deb architecture..."
  for f in "${ROOT}"/deb/arm64/jiangyin-*-dev_*.deb; do
    dpkg --info "$f" | grep -q '^ Architecture: arm64$' \
      || { echo "FAIL: $f is not arm64"; exit 1; }
    echo "  OK: $(basename "$f")"
  done
  echo "[arm64] Debs ready in deb/arm64/"
}

build_image() {
  echo "[arm64] Building ${MAIN_IMAGE} (PX4 SITL bake is the long pole)..."
  podman build --platform linux/arm64 \
    --build-arg ROS_BASE_IMAGE="${ROS_BASE}" \
    --build-arg TARGETARCH=arm64 \
    --build-arg DEB_DIR=deb/arm64 \
    --build-arg GZ_LIBDIR=/usr/lib/aarch64-linux-gnu/gazebo-11/plugins \
    --build-arg PX4_BUILD_JOBS="${BUILD_JOBS}" \
    -f "${ROOT}/docker/Dockerfile" \
    -t "${MAIN_IMAGE}" "${ROOT}"

  echo "[arm64] Image built: ${MAIN_IMAGE}"
  echo "[arm64] Next: run boot-level verification, then jy-smoke-test on the Jetson."
}

case "${STAGE}" in
  debs)  build_debs ;;
  image) build_image ;;
  all)   build_debs && build_image ;;
  *)     echo "Usage: $0 [debs|image|all]"; exit 2 ;;
esac