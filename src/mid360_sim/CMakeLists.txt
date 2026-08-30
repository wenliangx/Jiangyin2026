cmake_minimum_required(VERSION 3.0.2)
project(mid360_sim)

find_package(catkin REQUIRED COMPONENTS
  livox_ros_driver2
  roscpp
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS livox_ros_driver2 roscpp sensor_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(pointcloud_to_livox src/pointcloud_to_livox.cpp)
target_compile_features(pointcloud_to_livox PRIVATE cxx_std_14)
add_dependencies(pointcloud_to_livox ${catkin_EXPORTED_TARGETS})
target_link_libraries(pointcloud_to_livox ${catkin_LIBRARIES})

install(TARGETS pointcloud_to_livox
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
