#!/usr/bin/env bash
set -euo pipefail

package_root="$1"
output_dir="$2"
architecture="$(dpkg --print-architecture)"

mkdir -p "$output_dir"

make_package() {
    local name="$1"
    local version="$2"
    local source_dir="$3"
    local description="$4"
    local depends="${5:-}"
    local staging

    staging="$(mktemp -d)"
    mkdir -p "$staging/DEBIAN"
    cp -a "$source_dir/." "$staging/"

    cat >"$staging/DEBIAN/control" <<EOF
Package: $name
Version: $version
Section: devel
Priority: optional
Architecture: $architecture
Maintainer: Jiangyin2026 <maintainers@jiangyin2026.local>
Description: $description
EOF
    if [[ -n "$depends" ]]; then
        printf 'Depends: %s\n' "$depends" >>"$staging/DEBIAN/control"
    fi

    if [[ "$name" == "jiangyin-casadi-dev" ]]; then
        cat >"$staging/DEBIAN/postinst" <<'EOF'
#!/bin/sh
set -e
ldconfig
EOF
        chmod 0755 "$staging/DEBIAN/postinst"
    fi

    dpkg-deb --build "$staging" "$output_dir/${name}_${version}_${architecture}.deb"
    rm -rf "$staging"
}

# CasADi is linked against IPOPT and the Ubuntu BLAS/LAPACK and compiler
# runtimes. Keep these as package metadata so APT can resolve them on install.
casadi_depends='coinor-libipopt1v5, libblas3, liblapack3, libgfortran5, libgcc-s1, libstdc++6'
make_package \
    jiangyin-casadi-dev \
    3.7.0-1 \
    "$package_root/casadi" \
    'CasADi built with IPOPT for Jiangyin2026.' \
    "$casadi_depends"

make_package \
    jiangyin-livox-sdk2-dev \
    0.0.0-1 \
    "$package_root/livox" \
    'Livox SDK2 development files for Jiangyin2026.'

make_package \
    jiangyin-livox-ros-driver2 \
    1.0.0-1 \
    "$package_root/livox-ros-driver2" \
    'Livox ROS Driver 2 for ROS Noetic and Jiangyin2026.' \
    'jiangyin-livox-sdk2-dev (= 0.0.0-1), ros-noetic-roscpp, ros-noetic-rospy, ros-noetic-sensor-msgs, ros-noetic-std-msgs, ros-noetic-message-runtime, ros-noetic-rosbag, ros-noetic-pcl-ros, libapr1'

make_package \
    jiangyin-sophus-dev \
    1.22.10-1 \
    "$package_root/sophus" \
    'Sophus development files for Jiangyin2026.' \
    'libeigen3-dev'

meta="$(mktemp -d)"
mkdir -p "$meta/DEBIAN"
cat >"$meta/DEBIAN/control" <<EOF
Package: jiangyin-source-deps
Version: 1.0.0-1
Section: metapackages
Priority: optional
Architecture: all
Maintainer: Jiangyin2026 <maintainers@jiangyin2026.local>
Depends: jiangyin-casadi-dev (= 3.7.0-1), jiangyin-livox-sdk2-dev (= 0.0.0-1), jiangyin-sophus-dev (= 1.22.10-1)
Description: Source-built runtime and development dependencies for Jiangyin2026.
EOF
dpkg-deb --build "$meta" "$output_dir/jiangyin-source-deps_1.0.0-1_all.deb"
rm -rf "$meta"

meta="$(mktemp -d)"
mkdir -p "$meta/DEBIAN"
cat >"$meta/DEBIAN/control" <<EOF
Package: jiangyin-ros1-deps
Version: 1.0.0-1
Section: metapackages
Priority: optional
Architecture: all
Maintainer: Jiangyin2026 <maintainers@jiangyin2026.local>
Depends: jiangyin-livox-ros-driver2 (= 1.0.0-1), ros-noetic-mavros, ros-noetic-mavros-extras, ros-noetic-rosfmt, ros-noetic-gazebo-msgs, ros-noetic-gazebo-ros, ros-noetic-gazebo-plugins, ros-noetic-gazebo-ros-control, ros-noetic-plotjuggler, ros-noetic-plotjuggler-ros
Recommends: ros-noetic-velodyne-gazebo-plugins
Description: ROS Noetic runtime and simulation dependencies for Jiangyin2026.
EOF
dpkg-deb --build "$meta" "$output_dir/jiangyin-ros1-deps_1.0.0-1_all.deb"
rm -rf "$meta"
