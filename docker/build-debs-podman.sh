#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image="${IMAGE:-localhost/jiangyin-source-debs:latest}"
base_image="${ROS_BASE_IMAGE:-docker.io/osrf/ros:noetic-desktop-full}"
build_jobs="${BUILD_JOBS:-4}"
container="jiangyin-source-debs-export"

rm -rf "$root_dir/dist/debs"
mkdir -p "$root_dir/dist"
podman build \
    --build-arg "ROS_BASE_IMAGE=$base_image" \
    --build-arg "BUILD_JOBS=$build_jobs" \
    -f "$root_dir/docker/Dockerfile.debs" \
    -t "$image" \
    "$root_dir"
podman rm -f "$container" >/dev/null 2>&1 || true
podman create --name "$container" "$image" >/dev/null
podman cp "$container:/out/debs" "$root_dir/dist/debs"
podman rm "$container" >/dev/null
for deb in "$root_dir"/dist/debs/*.deb; do
    dpkg-deb -I "$deb"
done
