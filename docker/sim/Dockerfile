# Complete Jiangyin simulation stack. The expensive PX4/Gazebo toolchain is a
# stable shared base; this file owns every project-specific simulation layer.

ARG PX4_SIM_BASE=localhost/jiangyin_px4_mid360:latest
FROM ${PX4_SIM_BASE}

ARG BUILD_JOBS=4

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_MASTER_URI=http://localhost:11311
ENV ROS_IP=127.0.0.1
ENV PX4_SIM_MODEL=iris_mid360
ENV PX4_SIM_WORLD=obstacle_test
ENV HEADLESS=1

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

COPY deb/jiangyin-casadi-dev_*.deb \
     deb/jiangyin-livox-sdk2-dev_*.deb \
     deb/jiangyin-livox-ros-driver2_*.deb \
     deb/jiangyin-sophus-dev_*.deb \
     /tmp/jiangyin-debs/
RUN dpkg -i /tmp/jiangyin-debs/*.deb \
    && rm -rf /tmp/jiangyin-debs \
    && ldconfig \
    && test -f /opt/ros/noetic/share/livox_ros_driver2/cmake/livox_ros_driver2Config.cmake

COPY docker/sim/worlds/obstacle_test.world \
     /opt/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/obstacle_test.world
COPY docker/sim/scripts/configure_px4_mid360.py /usr/local/bin/configure_px4_mid360.py
RUN python3 /usr/local/bin/configure_px4_mid360.py \
    && cd /opt/PX4-Autopilot \
    && DONT_RUN=1 make px4_sitl gazebo-classic -j2 j=2 \
    && model=/opt/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_mid360/iris_mid360.sdf \
    && grep -q '<sensor name="mid360" type="ray">' "${model}" \
    && grep -q '<pose>0 0 0.12 0 -0.2617993877991494 0</pose>' "${model}" \
    && test "$(grep -c '<motorConstant>2.77e-05</motorConstant>' "${model}")" -eq 4 \
    && airframe=/opt/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1020_gazebo-classic_iris_mid360 \
    && grep -q '^param set-default EKF2_EV_CTRL 11$' "${airframe}" \
    && grep -q '^param set-default EKF2_HGT_REF 3$' "${airframe}" \
    && grep -q '^param set-default EKF2_GPS_CTRL 0$' "${airframe}" \
    && grep -q '^param set-default EKF2_MAG_TYPE 5$' "${airframe}"

WORKDIR /ws
COPY src/ /ws/src/
RUN python3 /ws/src/fsm_ctrl/scripts/validate_super_waypoints.py \
      --waypoints /ws/src/fsm_ctrl/config/mission_super_waypoints.yaml \
      --rules /ws/src/fsm_ctrl/config/waypoint_validation_rules.yaml \
      --output-dir /ws/src/fsm_ctrl/config/validated \
    && source /opt/ros/noetic/setup.bash \
    && catkin_make -j"${BUILD_JOBS}" \
      -DROS_EDITION=ROS1 \
      '-DCATKIN_WHITELIST_PACKAGES=super_msgs;uav_vision_msgs;mid360_sim;fsm_ctrl;ra_lio'

COPY docker/sim/scripts/start_sim_stack.sh /usr/local/bin/start_sim_stack
COPY docker/sim/scripts/smoke_sim_stack.py /usr/local/libexec/smoke_sim_stack.py
COPY docker/sim/scripts/smoke_sim_stack.sh /usr/local/bin/smoke_sim_stack
RUN chmod +x /usr/local/bin/start_sim_stack /usr/local/bin/smoke_sim_stack \
    && echo 'source /ws/devel/setup.bash' >> /root/.bashrc

ENTRYPOINT ["/usr/local/bin/start_sim_stack"]
CMD ["all"]
