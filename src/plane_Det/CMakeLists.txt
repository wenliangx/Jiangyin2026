cmake_minimum_required(VERSION 3.4.1)
set(CMAKE_CXX_STANDARD 14)
project(plane_Det)



# SET(CMAKE_BUILD_TYPE Release)
# MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
# set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")



find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  std_msgs
  rospy
  sensor_msgs
  
)


find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
# find_package(octomap REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(InferenceEngine REQUIRED)
# find_package(ngraph REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp  
    std_msgs
    rospy
    sensor_msgs
  DEPENDS
    cv_bridge
    image_transport
    
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  /home/flag/local/include
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
)


add_executable(plane_Det src/plane_Det_next.cpp)

target_link_libraries(plane_Det
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OpenCV_LIBS}
)

