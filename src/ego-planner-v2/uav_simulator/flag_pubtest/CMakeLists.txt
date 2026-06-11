cmake_minimum_required(VERSION 2.8.3)
project(flag_pubtest)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  traj_utils
)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS traj_utils 
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable (flag_pubtest src/flag_pubtest.cpp )
target_link_libraries(flag_pubtest
  ${catkin_LIBRARIES})
