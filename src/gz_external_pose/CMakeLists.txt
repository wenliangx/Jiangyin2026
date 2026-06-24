cmake_minimum_required(VERSION 3.0.2)
project(gz_external_pose)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  gazebo_msgs
  nav_msgs
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/gazebo_pose_to_vrpn.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
