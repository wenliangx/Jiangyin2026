cmake_minimum_required(VERSION 3.0.2)
project(uav_vision)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  uav_vision_msgs
)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(APRILTAG REQUIRED apriltag)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES uav_landing_core
  CATKIN_DEPENDS
    cv_bridge image_transport roscpp rospy sensor_msgs std_msgs uav_vision_msgs
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${APRILTAG_INCLUDE_DIRS}
)

add_library(uav_landing_core
  src/landing_debug_overlay.cpp
  src/landing_tag_detector.cpp
  src/landing_offset_estimator.cpp
)
target_compile_features(uav_landing_core PUBLIC cxx_std_14)
target_compile_options(uav_landing_core PRIVATE -Wall -Wextra)
add_dependencies(uav_landing_core ${catkin_EXPORTED_TARGETS})
target_link_libraries(uav_landing_core
  ${OpenCV_LIBRARIES}
  ${APRILTAG_LIBRARIES}
)

add_executable(landing_tag_node src/landing_tag_node.cpp)
add_dependencies(landing_tag_node uav_landing_core ${catkin_EXPORTED_TARGETS})
target_compile_features(landing_tag_node PRIVATE cxx_std_14)
target_compile_options(landing_tag_node PRIVATE -Wall -Wextra)
target_link_libraries(landing_tag_node
  uav_landing_core
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

catkin_install_python(
  PROGRAMS
    scripts/camera_focus_view.py
    scripts/landing_debug_web.py
    scripts/target_match_node.py
    scripts/v4l2_camera_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch templates udev
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  TARGETS uav_landing_core landing_tag_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
  catkin_add_gtest(test_landing_core test/test_landing_core.cpp)
  if(TARGET test_landing_core)
    target_compile_features(test_landing_core PRIVATE cxx_std_14)
    target_compile_options(test_landing_core PRIVATE -Wall -Wextra)
    add_dependencies(test_landing_core uav_landing_core ${catkin_EXPORTED_TARGETS})
    target_link_libraries(test_landing_core
      uav_landing_core
      ${OpenCV_LIBRARIES}
      ${APRILTAG_LIBRARIES}
    )
  endif()
endif()
