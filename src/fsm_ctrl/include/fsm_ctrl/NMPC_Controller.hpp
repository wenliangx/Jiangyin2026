#ifndef __NMPC_NEW_H__
#define __NMPC_NEW_H__

#include <casadi/casadi.hpp>
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <fsm_ctrl/ctrl_math.hpp>

class NMPC_Ctrller_simple
{
    private:
        /*unchanged parameters*/
        const double drone_mass = 1.874; //无人机质量0.9
        const double world_gravity_acc = 9.8015; //重力加速度
        const double drone_arm_L = 0.125; //无人机机臂长度0.18

        /*NLP parameters*/
        double ctrl_T; //控制器周期
        std::array<double, 2> acc_z_limit; //输入z轴加速度约束
        std::array<double, 2> w_limit; //无人机角速度约束
        int NLP_predict_step; //NLP问题预测步长
        double NLP_onestep_time; //NLP问题每步预测时间
        int NLP_state_num; //NLP问题状态变量个数
        int NLP_input_num; //NLP问题输入变量个数

        Eigen::Matrix<float, 3, 1> NLP_costQ_pos; //位置代价
        Eigen::Matrix<float, 3, 1> NLP_costQ_vel; //速度代价
        Eigen::Matrix<float, 3, 1> NLP_costQ_quat; //姿态代价
        Eigen::Matrix<float, 3, 1> NLP_costR_w; //角速度代价
        double NLP_costR_acc_z; //输入z轴加速度代价

        /* CASADI basic variables and functions */
        casadi::Function NLP_solver; //NLP问题求解器
        casadi::Function model_predict_function; //NLP问题迭代方程
        std::map<std::string, casadi::DM> NLP_result; //NLP问题最优求解值
        std::map<std::string, casadi::DM> NLP_args; //NLP求解器参数
        std::vector<double> NLP_initial_u0; //NLP问题每次开始迭代时输入变量初始值

        /* NMPC outputs */
        Eigen::Vector3d w_command; //求解结果，最优角速度
        double acc_z_command; //求解结果，最优力
        std::vector<double> optimal_command; //所有最优输出总集合

        ThrEst thr_est;

    public:
        NMPC_Ctrller_simple(double _ctrl_T,
                            std::array<double, 2> _acc_z_limit,
                            std::array<double, 2> _w_limit,
                            int _NLP_predict_step,
                            double _NLP_onestep_time,
                            int _NLP_state_num,
                            int _NLP_input_num,
                            Eigen::Matrix<float, 3, 1> _NLP_costQ_pos,
                            Eigen::Matrix<float, 3, 1> _NLP_costQ_vel,
                            Eigen::Matrix<float, 3, 1> _NLP_costQ_quat,
                            Eigen::Matrix<float, 3, 1> _NLP_costR_w,
                            double _NLP_costR_acc_z,
                            double _hover_thrust); //构造函数

        NMPC_Ctrller_simple(const NMPC_Ctrller_simple& other); // 拷贝构造函数
        NMPC_Ctrller_simple& operator=(const NMPC_Ctrller_simple& other); //赋值运算符重载

        double getCtrlT() const {return ctrl_T;}  //
		std::array<double, 2> getAcc_zLimit() const {return acc_z_limit;}  //
		std::array<double, 2> getWLimit() const {return w_limit;}  //

        int getNLPPredictStep() const {return NLP_predict_step;}  //
		double getNLPOnestepTime() const {return NLP_onestep_time;}  //
		int getNLPStateNum() const {return NLP_state_num;}  //
		int getNLPInputNum() const {return NLP_input_num;}  //

        Eigen::Matrix<float, 3, 1> getNLPcostQpos() const {return NLP_costQ_pos;}  //
        Eigen::Matrix<float, 3, 1> getNLPcostQvel() const {return NLP_costQ_vel;}  //
        Eigen::Matrix<float, 3, 1> getNLPcostQquat() const {return NLP_costQ_quat;}  //
        Eigen::Matrix<float, 3, 1> getNLPcostRw() const {return NLP_costR_w;}  //
        double getNLPcostRacc_z() const {return NLP_costR_acc_z;}  //

        Eigen::Vector3d getwCommand() const {return w_command;}  //
        double getAcc_zCommand() const {return acc_z_command;}  //
        std::vector<double> getOptimalCommand() const {return optimal_command;}  //

        void setNLPCostQpos(Eigen::Matrix<float, 3, 1> _NLP_costQpos) {NLP_costQ_pos = _NLP_costQpos;}  //
        void setNLPCostQvel(Eigen::Matrix<float, 3, 1> _NLP_costQvel) {NLP_costQ_vel = _NLP_costQvel;}  //
        void setNLPCostQquat(Eigen::Matrix<float, 3, 1> _NLP_costQquat) {NLP_costQ_quat = _NLP_costQquat;}  //
        void setNLPCostRw(Eigen::Matrix<float, 3, 1> _NLP_costRw) {NLP_costR_w = _NLP_costRw;}  //
        void setNLPCostRacc_z(double _NLP_costR_acc_z) {NLP_costR_acc_z = _NLP_costR_acc_z;}  // 
        
        void optimal_solution(std::vector<double> _current_states, std::vector<double> _desired_params);
};

#endif
