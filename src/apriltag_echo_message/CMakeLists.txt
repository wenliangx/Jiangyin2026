cmake_minimum_required(VERSION 3.0.2)
project(apriltag_echo_message)

# set(OpenCV_DIR /home/flag/opencv-4.2.0/build)
# set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake)

find_package(catkin REQUIRED COMPONENTS
  OpenCV REQUIRED

  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  apriltag_ros
  geometry_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}${OpenCV_INCLUDE_DIRS}
)

add_executable(echo_message src/echo_message.cpp)
add_dependencies(echo_message  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(echo_message  ${catkin_LIBRARIES})

add_executable(echo_message_mul src/echo_message_mul.cpp)
add_dependencies(echo_message_mul  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(echo_message_mul  ${catkin_LIBRARIES})

add_executable(tf_pub src/tf_pub.cpp)
add_dependencies(tf_pub  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_pub  ${catkin_LIBRARIES})
