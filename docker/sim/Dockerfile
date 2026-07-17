# =============================================================================
# PX4 SITL + Gazebo 仿真环境 Dockerfile
# =============================================================================
# 构建 PX4 SITL v1.14.3 + Gazebo Classic 11 仿真镜像
# 附带自定义 iris_mid360 无人机模型（PX4 Iris + 模拟 Livox MID360 LiDAR）
#
# 构建命令（在 Jiangyin2026 目录下执行）:
#   docker build -t jiangyin_px4_mid360:latest -f docker/sim/Dockerfile docker/sim/
#
# 启动命令:
#   docker run --rm --net=host jiangyin_px4_mid360:latest          # 无头模式
#   docker run --rm --net=host jiangyin_px4_mid360:latest --gui    # 带GUI
# =============================================================================

FROM docker.io/osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV PATH=/opt/cmake-3.28.3/bin:${PATH}
ENV PX4_HOME=/opt/PX4-Autopilot
ENV PX4_VERSION=v1.14.3

# === 阶段1: 系统依赖和ROS包 ===
RUN apt update && apt install -y --no-install-recommends \
    bash \
    ca-certificates \
    coinor-libipopt-dev \
    curl \
    git \
    gfortran \
    gnupg \
    lsb-release \
    wget \
    build-essential \
    cmake \
    ninja-build \
    pkg-config \
    protobuf-compiler \
    genromfs \
    swig \
    libdw-dev \
    libeigen3-dev \
    libgazebo11-dev \
    libglfw3-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    liblapack-dev \
    libopencv-dev \
    libprotobuf-dev \
    python3-dev \
    python3-empy \
    python3-jinja2 \
    python3-numpy \
    python3-pip \
    python3-setuptools \
    python3-toml \
    python3-wheel \
    python3-yaml \
    gazebo11 \
    geographiclib-tools \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-mavros \
    ros-noetic-mavros-extras \
    ros-noetic-rosfmt \
    ros-noetic-velodyne-gazebo-plugins \
    && rm -rf /var/lib/apt/lists/*

# === 阶段2: 编译第三方库（CasADi, Sophus, Livox-SDK）+ 升级CMake ===
WORKDIR /tmp

RUN git clone --depth 1 https://github.com/casadi/casadi.git
RUN git clone --depth 1 https://github.com/strasdat/Sophus.git
RUN git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK.git

RUN wget -q https://github.com/Kitware/CMake/releases/download/v3.28.3/cmake-3.28.3-linux-x86_64.tar.gz && \
    tar -zxf cmake-3.28.3-linux-x86_64.tar.gz && \
    mv cmake-3.28.3-linux-x86_64 /opt/cmake-3.28.3 && \
    rm cmake-3.28.3-linux-x86_64.tar.gz

# Livox-SDK（Livox LiDAR 驱动所需的底层SDK）
WORKDIR /tmp/Livox-SDK
RUN rm -fr build && \
    mkdir build && cd build && \
    cmake .. && make -j"$(nproc)" && \
    make install && \
    ldconfig

# CasADi（非线性优化和最优控制库，NMPC依赖）
WORKDIR /tmp/casadi
RUN rm -fr build && \
    mkdir build && cd build && \
    cmake .. -DWITH_IPOPT=ON && make -j"$(nproc)" && \
    make install && \
    ldconfig

# MAVROS 地理库数据（GPS坐标转换所需）
RUN /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh && \
    test -r /usr/share/GeographicLib/geoids/egm96-5.pgm

# === 阶段3: 克隆 PX4-Autopilot v1.14.3 源码 ===
RUN git clone --depth 1 --branch ${PX4_VERSION} https://github.com/PX4/PX4-Autopilot.git ${PX4_HOME} && \
    cd ${PX4_HOME} && \
    git submodule update --init --recursive --depth 1 \
      Tools/simulation/gazebo-classic/sitl_gazebo-classic \
      src/drivers/gps/devices \
      src/lib/events/libevents \
      src/modules/mavlink/mavlink \
      src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client

WORKDIR ${PX4_HOME}

# PX4 Python 依赖
RUN pip3 install --no-cache-dir \
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

# === 阶段4: 创建 iris_mid360 无人机模型 + 编译 PX4 SITL ===
COPY worlds/obstacle_test.world ${PX4_HOME}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/obstacle_test.world

COPY scripts/configure_px4_mid360.py /usr/local/bin/configure_px4_mid360.py
RUN python3 /usr/local/bin/configure_px4_mid360.py

RUN DONT_RUN=1 make px4_sitl gazebo-classic

# === 入口脚本: 启动 roscore → MAVROS → PX4 SITL + Gazebo ===
COPY scripts/start_px4_mid360.sh /usr/local/bin/start_px4_mid360.sh
RUN chmod +x /usr/local/bin/start_px4_mid360.sh

WORKDIR /ws

ENTRYPOINT ["/usr/local/bin/start_px4_mid360.sh"]
