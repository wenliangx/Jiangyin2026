cmake_minimum_required(VERSION 3.0.2)
project(fsm_ctrl)

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
  # super_msgs
  traj_utils
)
find_package(Eigen3 REQUIRED)

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
   CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf message_runtime quadrotor_msgs super_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
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

add_executable(single_offboard_fsm 
  src/single_offboard_fsm.cpp
)
target_link_libraries(single_offboard_fsm
  ctrl_math dfbc att_nmpc nmpc_ctrl FLAG_NMPC ${catkin_LIBRARIES} boost_system boost_filesystem
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

catkin_install_python(PROGRAMS
  scripts/pose_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

