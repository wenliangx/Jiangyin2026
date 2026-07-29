cmake_minimum_required(VERSION 3.1)
project(fsm_ctrl)

## ------------------------------------------------------------------
## Boost.SML v1.2.0 (vendored, header-only)
## ------------------------------------------------------------------
add_library(boost_sml INTERFACE)
target_include_directories(boost_sml SYSTEM INTERFACE ${PROJECT_SOURCE_DIR}/third_party/boost_sml/include)
target_compile_features(boost_sml INTERFACE cxx_std_14)
find_package(Threads REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  roscpp
  mavros_msgs
  std_msgs
  nav_msgs
  tf
  visualization_msgs
  quadrotor_msgs
  super_msgs
  traj_utils
  uav_vision_msgs
)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
if(TARGET yaml-cpp::yaml-cpp)
  set(FSM_CTRL_YAML_CPP_TARGET yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
  set(FSM_CTRL_YAML_CPP_TARGET yaml-cpp)
else()
  set(FSM_CTRL_YAML_CPP_TARGET ${YAML_CPP_LIBRARIES})
endif()

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  nmpc_state.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  mavros_msgs
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES fsm
#LIBRARIES motordriver
   CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf message_runtime quadrotor_msgs super_msgs uav_vision_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

#add_executable(single_offboard src/single_offboard.cpp)
#target_link_libraries(single_offboard ${catkin_LIBRARIES} boost_system boost_filesystem)

add_library(ctrl_math
  src/ctrl_math.cpp
)

add_library(dfbc
  src/dfbc.cpp
)

add_library(att_nmpc
  src/att_nmpc.cpp
)

add_library(nmpc_ctrl
  src/NMPC_Controller.cpp
)

add_library(FLAG_NMPC
  src/NMPC_test.cpp
)

add_library(traj_gen
  src/traj_gen.cpp
)

add_executable(single_offboard_fsm
  src/single_offboard_fsm.cpp
)
target_link_libraries(single_offboard_fsm
  ctrl_math dfbc att_nmpc nmpc_ctrl FLAG_NMPC traj_gen ${catkin_LIBRARIES} boost_system boost_filesystem
)
target_link_libraries(single_offboard_fsm 
  /usr/local/lib/libcasadi.so.3.7
) 

add_executable(px4_estimator
  src/px4_estimator.cpp
)
target_link_libraries(px4_estimator
  ctrl_math ${catkin_LIBRARIES} boost_system boost_filesystem
)

add_executable(swarm_user_cmd
  src/swarm_user_cmd.cpp
)
target_link_libraries(swarm_user_cmd
  ctrl_math ${catkin_LIBRARIES} boost_system boost_filesystem
)

# ------------------------------------------------------------------
# single_offboard_sml  (C++14, Boost.SML FSM)
# ------------------------------------------------------------------
add_executable(single_offboard_sml
  src/single_offboard_sml.cpp
)
target_compile_features(single_offboard_sml PRIVATE cxx_std_14)
add_dependencies(single_offboard_sml
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(single_offboard_sml
  boost_sml
  dfbc att_nmpc FLAG_NMPC traj_gen nmpc_ctrl ctrl_math
  ${catkin_LIBRARIES}
  ${FSM_CTRL_YAML_CPP_TARGET}
  boost_system boost_filesystem
  Threads::Threads
  /usr/local/lib/libcasadi.so.3.7
)

catkin_install_python(PROGRAMS
  scripts/pose_monitor.py
  scripts/record_super_waypoints.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## ------------------------------------------------------------------
## Tests  (GTest, no CasADi / no ROS control linkage)
## ------------------------------------------------------------------
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(single_offboard_sml_test
    test/single_offboard_sml_test.cpp
  )
  target_link_libraries(single_offboard_sml_test
    boost_sml
  )
  target_compile_features(single_offboard_sml_test PRIVATE cxx_std_14)
  add_rostest(test/single_offboard_sml_smoke.test)
endif()
