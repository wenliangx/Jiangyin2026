#ifndef _DFBC_HPP_
#define _DFBC_HPP_

#include <iostream>
#include <cmath>
#include <vector>
#include <queue>
#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <fsm_ctrl/ctrl_math.hpp>



class DFBC
{
    public:
        
        double ctrl_interv;
        ThrEst thrust_estor;
        CtrlPt ctrl_des;

        Eigen::Vector3d kp_pos, ki_pos, kd_pos;         // position PID 
        Eigen::Vector3d kp_vel, ki_vel, kd_vel;         // velocity PID
        double kp_yaw, kd_yaw;                          // yaw PD
        Eigen::Vector3d ierr_pos, derr_pos, ierr_vel, derr_vel;
        double derr_yaw;
        
        DFBC(int _ctrl_rate, double _hover_thrust, Eigen::Vector3d _kp_pos, Eigen::Vector3d _ki_pos, Eigen::Vector3d _kd_pos,
             Eigen::Vector3d _kp_vel, Eigen::Vector3d _ki_vel, Eigen::Vector3d _kd_vel, double _kp_yaw, double _kd_yaw);
        mavros_msgs::AttitudeTarget DFBCtrl(CtrlPt _ctrl_ref, Eigen::Vector3d _pos_fdb, Eigen::Vector3d _vel_fdb);
        geometry_msgs::TwistStamped VelCtrl(CtrlPt _ctrl_ref, Eigen::Vector3d _pos_fdb, Eigen::Vector3d _vel_fdb, double _yaw_fdb);
        mavros_msgs::AttitudeTarget AttCmd(double _yaw);
        geometry_msgs::TwistStamped VelCmd();
        // airsim_ros::VelCmd VelCmd();
};



#endif