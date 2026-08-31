cmake_minimum_required(VERSION 3.10)
project(ra_lio)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  livox_ros_driver2
  message_generation
  nav_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Threads REQUIRED)
find_package(rosfmt REQUIRED)

find_library(ROSFMT_LIBRARY NAMES rosfmt9 PATHS /opt/ros/noetic/lib)
if(NOT ROSFMT_LIBRARY)
  message(FATAL_ERROR "rosfmt9 library not found")
endif()

add_message_files(FILES Pose6D.msg)
generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    nav_msgs
    roscpp
    rospy
    std_msgs
  DEPENDS EIGEN3 PCL
)

set(RA_LIO_COMPILE_DEFINITIONS
  ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/"
)
set(RA_LIO_MP_PROC_NUM 1)

message(STATUS "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(RA_LIO_PROCESSOR_COUNT)
  message(STATUS "Processor count: ${RA_LIO_PROCESSOR_COUNT}")
  if(RA_LIO_PROCESSOR_COUNT GREATER 4)
    list(APPEND RA_LIO_COMPILE_DEFINITIONS MP_EN)
    set(RA_LIO_MP_PROC_NUM 3)
  elseif(RA_LIO_PROCESSOR_COUNT GREATER 3)
    list(APPEND RA_LIO_COMPILE_DEFINITIONS MP_EN)
    set(RA_LIO_MP_PROC_NUM 2)
  endif()
endif()
list(APPEND RA_LIO_COMPILE_DEFINITIONS MP_PROC_NUM=${RA_LIO_MP_PROC_NUM})

add_executable(ralio_mapping
  src/laser_mapping.cpp
  src/preprocess.cpp
  include/ikd-Tree/ikd_Tree.cpp
)
add_dependencies(ralio_mapping
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_compile_features(ralio_mapping PRIVATE cxx_std_14)
target_compile_options(ralio_mapping PRIVATE -O3 -fexceptions)
target_compile_definitions(ralio_mapping PRIVATE ${RA_LIO_COMPILE_DEFINITIONS})
target_include_directories(ralio_mapping PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  /usr/local/include
)
target_link_libraries(ralio_mapping PRIVATE
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Sophus_LIBRARIES}
  ${ROSFMT_LIBRARY}
  OpenMP::OpenMP_CXX
  Threads::Threads
)
