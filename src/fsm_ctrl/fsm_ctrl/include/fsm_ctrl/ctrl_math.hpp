#ifndef _CTRL_MATH_HPP_
#define _CTRL_MATH_HPP_

#include <iostream>
#include <cmath>
#include <vector>
#include <queue>
#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <mavros_msgs/AttitudeTarget.h>

#define GRAVITY 9.8015



class CtrlPt
{
    public:
        Eigen::Vector3d pos, vel, acc;
        Eigen::Quaterniond quat;
        Eigen::Vector3d euler;
        Eigen::Vector3d rate;
        double thrust;
};


class ThrEst
{
    public:
        double p_est = 1e6;
        double rho = 0.998;         // forgetting factor, Do Not Change!!!
        double ctrl_dt;
        double hover_thr;
        double thr_to_acc;
        std::queue<std::pair<ros::Time, double>> thr_stamped;
        
        void Set_Estor(int _ctrl_rate, double _hover_thr);
        double LSE(double _acc);
};

Eigen::Vector3d QuatToEuler(double _w, double _x, double _y, double _z);
Eigen::Vector3d QuatToEuler(Eigen::Quaterniond _quat);

Eigen::Quaterniond EulerToQuat(double _roll, double _pitch, double _yaw);
Eigen::Quaterniond EulerToQuat(Eigen::Vector3d _euler);

Eigen::Matrix3d QuatToMat(double _w, double _x, double _y, double _z);
Eigen::Matrix3d QuatToMat(Eigen::Quaterniond _quat);

Eigen::Matrix3d EulerToMat(double _roll, double _pitch, double _yaw);
Eigen::Matrix3d EulerToMat(Eigen::Vector3d _euler);

Eigen::Vector3d MatToEuler(Eigen::Matrix3d _mat);
Eigen::Quaterniond MatToQuat(Eigen::Matrix3d _mat);

double Clamp_Single(double data, double max);
double Clamp_Double(double data, double min, double max);
double Sign(double _data);

mavros_msgs::AttitudeTarget ArmCmd();
double YawSmooth(double &_yaw, double _yaw_ref);



#endif