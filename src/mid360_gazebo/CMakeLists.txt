cmake_minimum_required(VERSION 2.8.3)
project(mid360_gazebo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  gazebo_ros
  livox_ros_driver
)

catkin_package()

install(DIRECTORY models launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  nodes/mid360_bridge.py
  nodes/pose_comparison.py
  nodes/fake_odometry.py
  nodes/spawn_quadrotor.py
  nodes/record_data.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
