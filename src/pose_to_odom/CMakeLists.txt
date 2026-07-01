cmake_minimum_required(VERSION 3.0.2)
project(pose_to_odom)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    nav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pose_to_odom_node src/pose_to_odom_node.cpp)

target_link_libraries(pose_to_odom_node
  ${catkin_LIBRARIES}
)

install(TARGETS pose_to_odom_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
