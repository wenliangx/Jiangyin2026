#ifndef _ATT_NMPC_HPP_
#define _ATT_NMPC_HPP_

#include <iostream>
#include <cmath>
#include <vector>
#include <queue>
#include <map>
#include <ros/ros.h>
#include <casadi/casadi.hpp>
#include <eigen3/Eigen/Eigen>
#include <mavros_msgs/AttitudeTarget.h>
#include <fsm_ctrl/ctrl_math.hpp>

/*    prediction step and sample time interval    */
#define STEP 5
#define INTERV 0.1

#define STATE_NUM 7
#define INPUT_NUM 4


class NMPC
{
    public:
    
        /* variable */
        Eigen::Matrix<double, STATE_NUM, 1> mat_Q;
        Eigen::Matrix<double, INPUT_NUM, 1> mat_R;
        double acc_limit;
        double att_limit;

        ThrEst thrust_estor;
        std::vector<double> init_solution;
        casadi::Function solver;
        std::map<std::string, casadi::DM> arg;
        std::map<std::string, casadi::DM> solution;
        Eigen::Vector4d solution_input;

        /* function */
        NMPC(Eigen::Matrix<double, STATE_NUM, 1> _mat_Q, Eigen::Matrix<double, INPUT_NUM, 1> _mat_R,
             double _acc_limit, double _att_limit, int _ctrl_rate, double _hover_thrust);
        void Set_Solver();
        mavros_msgs::AttitudeTarget OptSolve(Eigen::Vector3d _pos_fdb, Eigen::Vector3d _vel_fdb, double _yaw_fdb, std::vector<CtrlPt> _state_ref);
        mavros_msgs::AttitudeTarget AttCmd(double _yaw);
};



#endif