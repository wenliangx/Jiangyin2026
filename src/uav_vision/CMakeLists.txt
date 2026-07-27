cmake_minimum_required(VERSION 3.0.2)
project(uav_vision)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  std_msgs
  uav_vision_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS cv_bridge rospy sensor_msgs std_msgs uav_vision_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/camera_focus_view.py
    scripts/target_match_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch templates
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
