#ifndef _SINGLE_OFFBOARD_FSM_HPP_
#define _SINGLE_OFFBOARD_FSM_HPP_



#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <vector>
#include <deque>
#include <mavros_msgs/PositionTarget.h>
#include <tf/tf.h>
#include "std_msgs/Int8.h"
#include "std_msgs/Int64.h"
#include <mavros_msgs/AttitudeTarget.h>
#include <fsm_ctrl/dfbc.hpp>
#include <fsm_ctrl/NMPC_test.hpp>
#include <fsm_ctrl/NMPC_Controller.hpp>
#include <quadrotor_msgs/PositionCommand.h>
#include <visualization_msgs/Marker.h>
// SYX FSM TEST
#include <traj_utils/Flag.h>
#include <traj_utils/FlagState.h>
#include <fsm_ctrl/ctrl_math.hpp>
// #include <super_msgs/PositionCommand.h>
// #include <super_msgs/Flag.h>
#include <fsm_ctrl/nmpc_state.h>
// SYX FSM TEST DONE
#include <nav_msgs/Path.h>



struct TypePoint
{
    int id;
    int mode;
    int is_map;
    double x,y,z;
    double yaw;
    int perc_mode;
};



#endif