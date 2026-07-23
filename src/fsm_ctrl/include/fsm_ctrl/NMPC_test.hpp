#ifndef __NMPC_H__
#define __NMPC_H__

#include <casadi/casadi.hpp>
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <Eigen/Dense>

class NMPC_Ctrller
{
private:
  /*pysical parameters*/
  double model_mass = 1.0;
  double gravity = 9.8015;
  /*normal control parameters*/
  double ctrl_interv;       //控制间隔
  double p_estimate = 1e6;  // TODO这个东西这么大吗？？
  double thrust_to_force;   // 

  /* NMPC parameters */

  float acc_limit = 20;
  float omega_limit = M_PI;
  int m_predict_step;
  float m_sample_time;
  int m_predict_stage;
  int n_states;
  int n_controls;
  std::vector<Eigen::Matrix<float, 3, 1>> m_predict_trajectory;
  std::vector<float> m_initial_guess;
  std::queue<std::pair<ros::Time, double>> thrust_stamped;  // 推力估计

  /* NMPC variables and functions */
  casadi::Function m_solver;
  casadi::Function model_predict_function;
  std::map<std::string, casadi::DM> m_result;
  std::map<std::string, casadi::DM> m_args;

  /* NMPC outputs */
  Eigen::Matrix<float, 4, 1> m_control_command;

public:
  NMPC_Ctrller(int _predict_step, float _sample_time, double _ctrl_interv, double _hover_thrust,
               Eigen::Matrix<float, 10, 1> _m_Q_param, Eigen::Matrix<float, 4, 1> _m_R_param);
  // ~NMPC_Ctrller();
  /* NMPC solver set */
  void set_my_nmpc_solver(Eigen::Matrix<float, 10, 1> _m_Q_param, Eigen::Matrix<float, 4, 1> _m_R_param);
  /* NMPC solver */
  void optimal_solution(Eigen::Matrix<float, 10, 1> _current_states, Eigen::Matrix<float, 60, 1> _desired_states,
                        Eigen::Matrix<float, 4, 1> _desired_controls);

  /* NMPC outputs */
  Eigen::Matrix<float, 4, 1> get_control_command();

  bool EstimateThrust(Eigen::Vector3d& _acc);
  bool Acc2Trust();
  casadi::SX CostMatrix_Q(Eigen::Matrix<float, 10, 1> _m_Q_param);
  casadi::SX CostMatrix_R(Eigen::Matrix<float, 4, 1> _m_R_param);
};

#endif  // __NMPC_H__