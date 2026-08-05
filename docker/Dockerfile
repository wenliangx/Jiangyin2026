# =============================================================================
# Jiangyin2026 — single Dockerfile (multi-stage: dev + prod)
# =============================================================================
# Located at docker/Dockerfile; build context = repo root (.dockerignore there).
#
#   dev stage (default target) : apt deps + clangd-22 + CMake 3.28.3 +
#                                per-library debs + GeographicLib + PX4 v1.14.3
#                                SITL (stock iris model, baked make — no lidar
#                                simulation) + Gazebo plugins
#                                + jy-docker.sh entrypoint
#   prod stage (--target prod) : bakes src/ via catkin_make + RA-LIO (day-0
#                                deterministic competition image)
#
# BUILD COMMANDS (run at repo root; build context ~310MB via .dockerignore)
#   docker build -f docker/Dockerfile -t jiangyin_jy2026 .               # dev stage (default)
#   docker build -f docker/Dockerfile -t jiangyin_jy2026:prod --target prod .  # baked algo stack
#   docker build -f docker/Dockerfile -t jiangyin_jy2026 --build-arg PX4_BUILD_JOBS=4 .  # cap make jobs
#   docker build -f docker/Dockerfile -t jiangyin_jy2026 --build-arg RUN_SMOKE=1 .  # run jy-smoke-test at build
#
# RUNTIME MODES — entrypoint jy-docker.sh <mode> (default: shell)
#   shell   interactive bash inside the image
#   sitl    roscore + MAVROS + PX4 SITL + Gazebo (headless)
#   dev     compile /ws/src (source-mount model) then start the stack
#   stack   algorithm stack only (RA-LIO → px4_estimator → FSM+NMPC)
#   all     sitl + stack together
#   takeoff send FSM cmd=3 (UDP)
#   land    send FSM cmd=4 (UDP)
#   reset   reset Gazebo + stop FSM
#
# ADVANCED WORKFLOW — PX4 source iteration via bind-mount at the SAME path that
# is baked into the image (overrides baked content; host tree must stay ABI-
# compatible with the baked SITL binary — user accepts the risk):
#   podman run -v $PWD/PX4-Autopilot:/opt/PX4-Autopilot:ro jiangyin_jy2026 sitl
#
# RELEASE SIZE — Podman: --squash collapses the PX4 build layers into one:
#   podman build --squash -t jiangyin_jy2026:release .
#
# ARBITRATION — PX4 + SITL are baked with an in-Dockerfile make (round 1-3
# verdict); tarball-based distribution was rejected.
# =============================================================================

# ---------------------------------------------------------------------------
# L0 — base image (Ubuntu 20.04 + ROS Noetic desktop-full, pinned digest)
# ---------------------------------------------------------------------------
ARG ROS_BASE_IMAGE=docker.io/osrf/ros:noetic-desktop-full@sha256:7dbfb9576d8e6d226c31e06129a82aaab8702695f38eca2116918cb9b9308797
FROM ${ROS_BASE_IMAGE} AS dev

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
ARG TARGETARCH
RUN if [ "${TARGETARCH}" = "arm64" ]; then \
        apt-get update \
        && apt-get install -y --no-install-recommends curl ca-certificates gnupg \
        && curl -fsSL http://packages.osrfoundation.org/gazebo.key | gpg --dearmor -o /usr/share/keyrings/gazebo-archive-keyring.gpg \
        && echo "deb [signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable focal main" > /etc/apt/sources.list.d/gazebo-stable.list; \
    fi \
    && apt-get update && apt-get install -y --no-install-recommends \
    bash \
    build-essential \
    ca-certificates \
    clangd \
    cmake \
    coinor-libipopt-dev \
    curl \
    gazebo11 \
    genromfs \
    gfortran \
    geographiclib-tools \
    git \
    gnupg \
    libapriltag-dev \
    libdw-dev \
    libeigen3-dev \
    libgazebo11-dev \
    libglfw3-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev \
    liblapack-dev \
    libopencv-dev \
    libpcl-dev \
    libprotobuf-dev \
    lsb-release \
    ninja-build \
    pkg-config \
    protobuf-compiler \
    python3-dev \
    python3-empy \
    python3-jinja2 \
    python3-numpy \
    python3-pip \
    python3-setuptools \
    python3-toml \
    python3-wheel \
    python3-yaml \
    ros-noetic-gazebo-msgs \
    ros-noetic-gazebo-plugins \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-ros-control \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-mavros \
    ros-noetic-mavros-extras \
    ros-noetic-mavros-msgs \
    ros-noetic-plotjuggler \
    ros-noetic-plotjuggler-ros \
    ros-noetic-rosfmt \
    ros-noetic-velodyne-gazebo-plugins \
    software-properties-common \
    swig \
    tmux \
    wget \
    && rm -rf /var/lib/apt/lists/*

# clangd-22 from apt.llvm.org (LLVM 22 toolchain repo).
# apt.llvm.org publishes no focal arm64 packages (LLVM 22 nor older), so on
# arm64 the Ubuntu clangd (10.x) from the L0 apt list is used instead.
RUN if [ "${TARGETARCH}" != "arm64" ]; then \
        wget -qO- https://apt.llvm.org/llvm.sh | bash -s 22 \
        && apt-get update \
        && apt-get install -y --no-install-recommends clangd-22 \
        && update-alternatives --install /usr/local/bin/clangd clangd /usr/bin/clangd-22 100 \
        && rm -rf /var/lib/apt/lists/*; \
    fi

# ---------------------------------------------------------------------------
# L2 — CMake 3.28.3 (Ubuntu 20.04 ships 3.16; Sophus requires 3.24+)
# ---------------------------------------------------------------------------
ARG TARGETARCH
ARG CMAKE_TARBALL=cmake-3.28.3-linux-x86_64.tar.gz
RUN if [ "${TARGETARCH}" = "arm64" ]; then CMAKE_TARBALL=cmake-3.28.3-linux-aarch64.tar.gz; fi \
    && curl -fsSL \
      "https://github.com/Kitware/CMake/releases/download/v3.28.3/${CMAKE_TARBALL}" \
      -o /tmp/cmake.tar.gz \
    && mkdir -p /opt/cmake \
    && tar -xzf /tmp/cmake.tar.gz --strip-components=1 -C /opt/cmake \
    && rm /tmp/cmake.tar.gz \
    && /opt/cmake/bin/cmake --version | grep -q "3.28.3"

ENV PATH=/opt/cmake/bin:${PATH}

# ---------------------------------------------------------------------------
# L3 — per-library debs (one COPY per family for cache granularity)
#      runtime apt deps for CasADi (IPOPT), Livox-SDK2, Sophus, apriltag, PCL
# ---------------------------------------------------------------------------
ARG TARGETARCH
ARG DEB_DIR=deb
COPY ${DEB_DIR}/jiangyin-sophus-dev_*.deb /tmp/
COPY ${DEB_DIR}/jiangyin-casadi-dev_*.deb /tmp/
COPY ${DEB_DIR}/jiangyin-livox-sdk2-dev_*.deb /tmp/
COPY ${DEB_DIR}/jiangyin-livox-ros-driver2_*.deb /tmp/
COPY ${DEB_DIR}/jiangyin-ros1-deps_*.deb ${DEB_DIR}/jiangyin-source-deps_*.deb /tmp/
RUN apt-get update && apt-get install -y --no-install-recommends \
    /tmp/*.deb \
    coinor-libipopt1v5 \
    libblas3 \
    liblapack3 \
    libgfortran5 \
    libomp-dev \
    libeigen3-dev \
    libapriltag-dev \
    libglfw3-dev \
    libdw-dev \
    libpcl-dev \
    && rm -rf /tmp/*.deb /var/lib/apt/lists/* \
    && dpkg-query -W jiangyin-livox-ros-driver2 \
    && test -f /opt/ros/noetic/share/livox_ros_driver2/cmake/livox_ros_driver2Config.cmake

# ---------------------------------------------------------------------------
# L4 — GeographicLib geoid datasets (MAVROS GPS→ENU) + PX4 Python deps
# ---------------------------------------------------------------------------
RUN /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh \
    && test -r /usr/share/GeographicLib/geoids/egm96-5.pgm \
    && pip3 install --no-cache-dir \
    argcomplete \
    cerberus \
    empy==3.3.4 \
    future \
    jsonschema \
    kconfiglib \
    lxml \
    matplotlib \
    packaging \
    psutil \
    pyros-genmsg \
    pyserial \
    pyyaml \
    six \
    toml \
    tqdm \
    uavcan \
    wheel

# ---------------------------------------------------------------------------
# L5 — PX4-Autopilot v1.14.3 source (pinned SHA, shallow, fetch retry loop)
#      + the 5 gazebo-classic submodules
# ---------------------------------------------------------------------------
ARG PX4_SHA=de8a295af4d8192a3e85b2565040367378a07d8e
ARG PX4_TAG=v1.14.3

# NOTE: the shallow fetch is NOT enough on its own — PX4's top-level
# CMakeLists.txt runs `git describe` to derive the version and splits it on
# '.', so a tag-less shallow clone (bare abbreviated SHA like "1dacb4c") breaks
# the configure (list index out of range). Fix: fetch the release tag (shallow)
# and deepen the pinned SHA to 50 commits so the tag is reachable from HEAD.
RUN git init /opt/PX4-Autopilot \
    && git -C /opt/PX4-Autopilot remote add origin https://github.com/PX4/PX4-Autopilot.git \
    && for i in 1 2 3; do \
         git -C /opt/PX4-Autopilot fetch --depth 1 origin "${PX4_SHA}" \
           && { FETCHED=1; break; } \
           || { echo "[PX4] fetch attempt ${i}/3 failed, retrying in 5s"; sleep 5; }; \
       done \
    && test "${FETCHED:-0}" = "1" \
    && git -C /opt/PX4-Autopilot checkout --detach FETCH_HEAD \
    && git -C /opt/PX4-Autopilot fetch --depth 1 origin tag "${PX4_TAG}" \
    && git -C /opt/PX4-Autopilot fetch --depth 50 origin "${PX4_SHA}" \
    && git -C /opt/PX4-Autopilot submodule update --init --recursive --depth 1 \
         Tools/simulation/gazebo-classic/sitl_gazebo-classic \
         src/drivers/gps/devices \
         src/lib/events/libevents \
         src/modules/mavlink/mavlink \
         src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client \
    && test "$(git -C /opt/PX4-Autopilot describe --exclude ext/* --always --tags)" = "${PX4_TAG}" \
    && echo "${PX4_SHA}" > /opt/PX4-Autopilot/.jy-px4-sha

# ---------------------------------------------------------------------------
# L6 — (removed: mid360 LiDAR simulation. SITL uses stock PX4 iris model +
#      default gazebo world; no radar/lidar simulation in sim.)
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# L7 — bake PX4 SITL (jobs = core count; DONT_RUN=1: build only, no launch)
#      NOTE: PX4's Makefile takes the job count from the make variable `j`
#      (`ifneq ($(j),) PX4_MAKE_ARGS += -j$(j)`) — the `-j` flag alone is
#      ignored and ninja silently uses ALL cores. `j=$(nproc)` must be passed
#      so the job count is honored. PX4_BUILD_JOBS overrides (e.g. cap it on
#      RAM-limited machines: --build-arg PX4_BUILD_JOBS=4).
# ---------------------------------------------------------------------------
ARG PX4_BUILD_JOBS

RUN cd /opt/PX4-Autopilot \
    && DONT_RUN=1 make px4_sitl gazebo-classic -j"${PX4_BUILD_JOBS:-$(nproc)}" j="${PX4_BUILD_JOBS:-$(nproc)}"

# ---------------------------------------------------------------------------
# L8 — conservative cleanup (keep src/ module dirs, rootfs/, *.so, bin/,
#      build.ninja, CMakeCache.txt) + asserts that the SITL artifacts survive
# ---------------------------------------------------------------------------
RUN find /opt/PX4-Autopilot/build/px4_sitl_default -name '*.o' -delete \
    && rm -rf /opt/PX4-Autopilot/build/px4_sitl_default/logs \
    && cd /opt/PX4-Autopilot/build/px4_sitl_default \
    && test -f bin/px4 \
    && test -d src/modules/simulation/simulator_mavlink \
    && ls build_gazebo-classic/*.so >/dev/null \
    && test -f build.ninja \
    && test -f CMakeCache.txt \
    && [ "$(find . -name '*.o' | wc -l)" -eq 0 ]

# ---------------------------------------------------------------------------
# L9 — dev tooling: gazebo plugin symlinks, jy-* convenience symlinks, ENV
# ---------------------------------------------------------------------------
ARG TARGETARCH
ARG GZ_LIBDIR=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins
RUN for so in /opt/ros/noetic/lib/*.so; do \
        ln -sf "$so" "${GZ_LIBDIR}/$(basename $so)"; \
    done 2>/dev/null || true \
    && ln -sf /usr/local/bin/jy-docker.sh /usr/local/bin/jy-takeoff \
    && ln -sf /usr/local/bin/jy-docker.sh /usr/local/bin/jy-land \
    && ln -sf /usr/local/bin/jy-docker.sh /usr/local/bin/jy-reset \
    && ln -sf /usr/local/bin/jy-docker.sh /usr/local/bin/jy-stack \
    && ln -sf /usr/local/bin/jy-docker.sh /usr/local/bin/jy-start-stack

# Single-container networking + Gazebo/PX4 runtime env
ENV GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:${GZ_LIBDIR}
ENV GAZEBO_MODEL_PATH=/home/user/.gazebo/models:/usr/share/gazebo-11/models
ENV GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11/models:/home/user/.gazebo/models
ENV SIM_LOCKSTEP=1
ENV QT_X11_NO_MITSHM=1
ENV ROS_MASTER_URI=http://localhost:11311
ENV ROS_IP=127.0.0.1
ENV PX4_HOME=/opt/PX4-Autopilot
ENV PX4_VERSION=v1.14.3

# ---------------------------------------------------------------------------
# L10 — entrypoint (jy-docker.sh mode router) + optional build-time smoke gate
# ---------------------------------------------------------------------------
COPY docker/scripts/jy-docker.sh /usr/local/bin/jy-docker.sh
COPY docker/scripts/jy-smoke-test /usr/local/bin/jy-smoke-test
RUN chmod +x /usr/local/bin/jy-docker.sh /usr/local/bin/jy-smoke-test

WORKDIR /ws

ENTRYPOINT ["jy-docker.sh"]
CMD ["shell"]

ARG RUN_SMOKE=0
RUN if [ "${RUN_SMOKE}" = "1" ]; then jy-smoke-test; fi

# ---------------------------------------------------------------------------
# PROD STAGE — bake the algorithm stack (src/ + catkin_make + RA-LIO)
#              only built when requested via --target prod
# ---------------------------------------------------------------------------
FROM dev AS prod

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

WORKDIR /ws
COPY src/ /ws/src/

# # package_ROS1.xml → package.xml (no-clobber) + enable ego-planner-v2
# RUN if [ -f /ws/src/libs/livox_ros_driver2/package_ROS1.xml ]; then \
#         cp -n /ws/src/libs/livox_ros_driver2/package_ROS1.xml /ws/src/libs/livox_ros_driver2/package.xml; \
#     fi \
#     && rm -f /ws/src/ego-planner-v2/CATKIN_IGNORE

ARG JOBS=64

# NOTE: podman/buildah ignores the SHELL instruction under the OCI image
# format, so RUN steps execute under /bin/sh — ROS setup.bash is bash-only;
# use the POSIX setup.sh variant and '.' instead of 'source'.
RUN . /opt/ros/noetic/setup.sh \
    && catkin_make -j"${JOBS}" --use-ninja

# # RA-LIO standalone build (needs livox_ros_driver2 from /ws/devel), then merge
# # the binaries into /ws/devel (Dockerfile.prod steps)
# RUN . /opt/ros/noetic/setup.sh \
#     && . /ws/devel/setup.sh \
#     && mkdir -p /ws/src/RA-LIO/build \
#     && cd /ws/src/RA-LIO/build \
#     && cmake .. -DCMAKE_BUILD_TYPE=Release -Dlivox_ros_driver2_DIR=/ws/devel/share/livox_ros_driver2/cmake \
#     && make -j"${JOBS}" \
#     && cp -rn devel/bin/* /ws/devel/bin/ 2>/dev/null || true; \
#     cp -rn devel/lib/* /ws/devel/lib/ 2>/dev/null || true; \
#     cp -rn devel/share/* /ws/devel/share/ 2>/dev/null || true; \
#     echo 'RA-LIO built and merged into /ws/devel/'

RUN echo "source /ws/devel/setup.bash" >> /root/.bashrc

CMD ["stack"]

# ---------------------------------------------------------------------------
# DEFAULT TARGET — dev stage. Docker/podman default to the LAST stage in the
# file, so an empty stage re-tagging dev must come after prod to keep
# `docker build` / `podman build` producing the dev image (prod is opt-in via
# --target prod).
# ---------------------------------------------------------------------------
FROM dev
