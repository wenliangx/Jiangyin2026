cmake_minimum_required(VERSION 3.0.2)
project(map_frame_manager)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf
)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common filters io registration)
find_package(Boost REQUIRED COMPONENTS filesystem system)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs pcl_conversions pcl_ros roscpp sensor_msgs
                 std_msgs std_srvs tf
  DEPENDS EIGEN3 PCL
)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS})

add_executable(map_frame_manager_node src/map_frame_manager_node.cpp)
target_link_libraries(map_frame_manager_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(map_origin_recorder_node src/map_origin_recorder_node.cpp)
target_link_libraries(map_origin_recorder_node ${catkin_LIBRARIES} ${PCL_LIBRARIES}
                      ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})

add_executable(map_pcd_processor_node src/map_pcd_processor_node.cpp)
target_link_libraries(map_pcd_processor_node ${catkin_LIBRARIES} ${PCL_LIBRARIES}
                      ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})

add_executable(map_origin_setter_node src/map_origin_setter_node.cpp)
target_link_libraries(map_origin_setter_node ${catkin_LIBRARIES} ${PCL_LIBRARIES}
                      ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})

install(TARGETS map_frame_manager_node map_origin_recorder_node map_pcd_processor_node
                map_origin_setter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_frame_math test/test_frame_math.cpp)
  if(TARGET test_frame_math)
    target_link_libraries(test_frame_math ${catkin_LIBRARIES})
  endif()
endif()
