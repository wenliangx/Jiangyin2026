#include "fsm_ctrl/nmpc_controller.hpp"

#include <iostream>

namespace {

constexpr double kGravity = 9.8015;

}  // namespace

NmpcController::NmpcController(
    const std::array<double, 2>& vertical_acceleration_limits,
    const std::array<double, 2>& angular_rate_limits, int prediction_steps,
    double prediction_step_seconds, int state_size, int input_size,
    const Eigen::Matrix<float, 3, 1>& position_weights,
    const Eigen::Matrix<float, 3, 1>& velocity_weights,
    const Eigen::Matrix<float, 3, 1>& attitude_weights,
    const Eigen::Matrix<float, 3, 1>& angular_rate_weights,
    double acceleration_weight, double hover_thrust)
    : vertical_acceleration_limits_(vertical_acceleration_limits),
      angular_rate_limits_(angular_rate_limits),
      prediction_steps_(prediction_steps),
      prediction_step_seconds_(prediction_step_seconds),
      state_size_(state_size),
      input_size_(input_size),
      position_weights_(position_weights),
      velocity_weights_(velocity_weights),
      attitude_weights_(attitude_weights),
      angular_rate_weights_(angular_rate_weights),
      acceleration_weight_(acceleration_weight) {
  thrust_estimator_.configure(50, hover_thrust);

  for (int j = 0; j < prediction_steps_; j++)
    initial_guess_.push_back(0);  // wx[0..N-1]
  for (int j = 0; j < prediction_steps_; j++)
    initial_guess_.push_back(0);  // wy[0..N-1]
  for (int j = 0; j < prediction_steps_; j++)
    initial_guess_.push_back(0);  // wz[0..N-1]
  for (int j = 0; j < prediction_steps_; j++)
    initial_guess_.push_back(0);  // az[0..N-1]

  /* states */
  casadi::SX p = casadi::SX::sym("p", 3);  // position in world frame 3*1
  casadi::SX v = casadi::SX::sym("v", 3);  // velocity in world frame 3*1
  casadi::SX q = casadi::SX::sym(
      "q", 4);  // attitude quaternion from world frame to body frame 3*1
  /* inputs */
  casadi::SX w = casadi::SX::sym("w", 3);  // angle velocity in body frame 3*1
  casadi::SX acc_z =
      casadi::SX::sym("acc_z", 1);  // motor thrust in body frame 4*1

  /* total states and total inputs */
  casadi::SX x = casadi::SX::vertcat({p, v, q});   // x:10*1
  casadi::SX u = casadi::SX::vertcat({w, acc_z});  // u:4*1

  /* dynamics model */
  casadi::SX x_dot = casadi::SX::vertcat({
      v(0),
      v(1),
      v(2),
      2 * (q(0) * q(2) + q(1) * q(3)) * acc_z,
      2 * (q(2) * q(3) - q(0) * q(1)) * acc_z,
      (q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3)) * acc_z -
          kGravity,
      0.5 * (-q(1) * w(0) - q(2) * w(1) - q(3) * w(2)),
      0.5 * (q(0) * w(0) + q(2) * w(2) - q(3) * w(1)),
      0.5 * (q(0) * w(1) - q(1) * w(2) + q(3) * w(0)),
      0.5 * (q(0) * w(2) + q(1) * w(1) - q(2) * w(0)),
  });  // x_dot:13*1

  // 定义模型函数
  casadi::Function model_function = casadi::Function("f", {x, u}, {x_dot});

  // 求解问题符号表示
  casadi::SX input_trajectory = casadi::SX::sym(
      "input_trajectory", input_size_,
      prediction_steps_);  // 控制输入，本案例中为 4*(predict_step)
  casadi::SX state_trajectory = casadi::SX::sym(
      "state_trajectory", state_size_,
      prediction_steps_ + 1);  // 状态输出，本案例中为 13*(predict_step+1)

  // 优化问题所需参数
  // 当前所有状态+期望轨迹点对应位置、速度、姿态、加速度+期望控制量
  // 10+(1+predict_step)*10+predict_step*4
  casadi::SX opt_param =
      casadi::SX::sym("opt_param", (prediction_steps_ + 2) * state_size_ +
                                       prediction_steps_ * input_size_);
  // std::cout << "SX 变量 opt_param:" << opt_param << std::endl;

  // 优化变量
  // (predict_step*input_num)*1，从上到下分别为第一个控制量的predict_step步，第二个控制量的predict_step步，...，第input_num个控制量的predict_step步
  casadi::SX opt_var = casadi::SX::reshape(input_trajectory.T(), -1, 1);
  // std::cout << "SX 变量 opt_var:" << opt_var << std::endl;

  // 根据模型函数前向预测无人机运动状态
  state_trajectory(casadi::Slice(), 0) =
      opt_param(casadi::Slice(0, state_size_, 1));  // 状态初始值

  for (int i = 0; i < prediction_steps_; i++) {
    std::vector<casadi::SX> model_inputs;  // 最终为10*1
    casadi::SX current_state = state_trajectory(casadi::Slice(), i);
    casadi::SX current_input =
        casadi::SX::vertcat({opt_var(i), opt_var(i + prediction_steps_),
                             opt_var(i + 2 * prediction_steps_),
                             opt_var(i + 3 * prediction_steps_)});
    // std::cout << "SX 变量 current_input:" << current_input << std::endl;
    model_inputs.push_back(current_state);
    model_inputs.push_back(current_input);
    state_trajectory(casadi::Slice(), i + 1) =
        model_function(model_inputs).at(0) * prediction_step_seconds_ +
        current_state;
  }

  // 惩罚矩阵
  casadi::SX position_cost = casadi::SX::diag({casadi::SX::vertcat(
      {position_weights_(0), position_weights_(1), position_weights_(2)})});
  casadi::SX velocity_cost = casadi::SX::diag({casadi::SX::vertcat(
      {velocity_weights_(0), velocity_weights_(1), velocity_weights_(2)})});
  casadi::SX attitude_cost = casadi::SX::diag({casadi::SX::vertcat(
      {attitude_weights_(0), attitude_weights_(1), attitude_weights_(2)})});
  casadi::SX angular_rate_cost = casadi::SX::diag(
      {casadi::SX::vertcat({angular_rate_weights_(0), angular_rate_weights_(1),
                            angular_rate_weights_(2)})});

  // 计算代价函数
  casadi::SX cost_function = casadi::SX::sym("cost_function");
  cost_function = 0;
  for (int i = 0; i < prediction_steps_; i++) {
    casadi::SX error_pos =
        opt_param(casadi::Slice(state_size_ * (i + 1),
                                state_size_ * (i + 1) + 3, 1)) -
        state_trajectory(casadi::Slice(0, 3, 1), i);
    casadi::SX error_vel =
        opt_param(casadi::Slice(state_size_ * (i + 1) + 3,
                                state_size_ * (i + 1) + 6, 1)) -
        state_trajectory(casadi::Slice(3, 6, 1), i);
    casadi::SX error_quat = casadi::SX::vertcat(
        {state_trajectory(6, i) * opt_param(state_size_ * (i + 1) + 7) -
             state_trajectory(7, i) * opt_param(state_size_ * (i + 1) + 6) +
             state_trajectory(8, i) * opt_param(state_size_ * (i + 1) + 9) -
             state_trajectory(9, i) * opt_param(state_size_ * (i + 1) + 8),
         state_trajectory(6, i) * opt_param(state_size_ * (i + 1) + 8) -
             state_trajectory(7, i) * opt_param(state_size_ * (i + 1) + 9) -
             state_trajectory(8, i) * opt_param(state_size_ * (i + 1) + 6) +
             state_trajectory(9, i) * opt_param(state_size_ * (i + 1) + 7),
         state_trajectory(6, i) * opt_param(state_size_ * (i + 1) + 9) +
             state_trajectory(7, i) * opt_param(state_size_ * (i + 1) + 8) -
             state_trajectory(8, i) * opt_param(state_size_ * (i + 1) + 7) -
             state_trajectory(9, i) * opt_param(state_size_ * (i + 1) + 6)});
    casadi::SX error_w =
        opt_param(casadi::Slice(
            (state_size_ * (prediction_steps_ + 2) + input_size_ * i),
            (state_size_ * (prediction_steps_ + 2) + input_size_ * i + 3), 1)) -
        input_trajectory(casadi::Slice(0, 3, 1), i);
    casadi::SX error_acc_z =
        opt_param(state_size_ * (prediction_steps_ + 2) + input_size_ * i + 3) -
        input_trajectory(3, i);
    cost_function =
        cost_function +
        casadi::SX::mtimes({error_pos.T(), position_cost, error_pos}) +
        casadi::SX::mtimes({error_vel.T(), velocity_cost, error_vel}) +
        casadi::SX::mtimes({error_quat.T(), attitude_cost, error_quat}) +
        casadi::SX::mtimes({error_w.T(), angular_rate_cost, error_w}) +
        acceleration_weight_ * error_acc_z * error_acc_z;
  }

  casadi::SX error_pos =
      opt_param(casadi::Slice(state_size_ * (prediction_steps_ + 1),
                              state_size_ * (prediction_steps_ + 1) + 3, 1)) -
      state_trajectory(casadi::Slice(0, 3, 1), prediction_steps_);
  casadi::SX error_vel =
      opt_param(casadi::Slice(state_size_ * (prediction_steps_ + 1) + 3,
                              state_size_ * (prediction_steps_ + 1) + 6, 1)) -
      state_trajectory(casadi::Slice(3, 6, 1), prediction_steps_);
  casadi::SX error_quat = casadi::SX::vertcat(
      {state_trajectory(6, prediction_steps_) *
               opt_param(state_size_ * (prediction_steps_ + 1) + 7) -
           state_trajectory(7, prediction_steps_) *
               opt_param(state_size_ * (prediction_steps_ + 1) + 6) +
           state_trajectory(8, prediction_steps_) *
               opt_param(state_size_ * (prediction_steps_ + 1) + 9) -
           state_trajectory(9, prediction_steps_) *
               opt_param(state_size_ * (prediction_steps_ + 1) + 8),
       state_trajectory(6, prediction_steps_) *
               opt_param(state_size_ * (prediction_steps_ + 1) + 8) -
           state_trajectory(7, prediction_steps_) *
               opt_param(state_size_ * (prediction_steps_ + 1) + 9) -
           state_trajectory(8, prediction_steps_) *
               opt_param(state_size_ * (prediction_steps_ + 1) + 6) +
           state_trajectory(9, prediction_steps_) *
               opt_param(state_size_ * (prediction_steps_ + 1) + 7),
       state_trajectory(6, prediction_steps_) *
               opt_param(state_size_ * (prediction_steps_ + 1) + 9) +
           state_trajectory(7, prediction_steps_) *
               opt_param(state_size_ * (prediction_steps_ + 1) + 8) -
           state_trajectory(8, prediction_steps_) *
               opt_param(state_size_ * (prediction_steps_ + 1) + 7) -
           state_trajectory(9, prediction_steps_) *
               opt_param(state_size_ * (prediction_steps_ + 1) + 6)});
  cost_function =
      cost_function +
      casadi::SX::mtimes({error_pos.T(), position_cost, error_pos}) +
      casadi::SX::mtimes({error_vel.T(), velocity_cost, error_vel}) +
      casadi::SX::mtimes({error_quat.T(), attitude_cost, error_quat});

  // 构建求解器（不考虑优化问题）
  // 这里的变量说明可以找Casadi C++ API手册
  casadi::SXDict nlp_problem = {
      {"f", cost_function},
      {"x", opt_var},    // 系统输出
      {"p", opt_param},  // 优化参数,即当前状态、目标状态、目标控制
  };

  std::string solver_name = "ipopt";
  casadi::Dict nlp_opts;
  nlp_opts["expand"] = true;
  nlp_opts["ipopt.max_iter"] = 5000;
  nlp_opts["ipopt.print_level"] = 5;
  nlp_opts["print_time"] = 1;
  nlp_opts["ipopt.acceptable_tol"] = 1e-4;
  nlp_opts["ipopt.acceptable_obj_change_tol"] = 1e-4;
  nlp_opts["ipopt.acceptable_dual_inf_tol"] = 1e-4;

  solver_ = nlpsol("solver", solver_name, nlp_problem, nlp_opts);
}

void NmpcController::solve(const std::vector<double>& current_states,
                           const std::vector<double>& desired_parameters) {
  size_t current_states_size = static_cast<size_t>(state_size_);
  size_t desired_params_size = static_cast<size_t>(
      (prediction_steps_ + 1) * state_size_ + prediction_steps_ * input_size_);
  // 向量大小检查
  if (current_states.size() != current_states_size) {
    std::cerr << "Warning! Current States Vector size changes!" << std::endl;
  }
  if (desired_parameters.size() != desired_params_size) {
    std::cerr << "Warning! Desired Params Vector size changes!" << std::endl;
  }

  // 控制约束
  std::vector<double> lbx;         // 控制下限
  std::vector<double> ubx;         // 控制上限
  std::vector<double> parameters;  // 包括当前状态、目标状态、目标控制

  for (int i = 0; i < input_size_ - 1; i++) {
    for (int j = 0; j < prediction_steps_; j++) {
      lbx.push_back(angular_rate_limits_.at(0));
      ubx.push_back(angular_rate_limits_.at(1));
    }
  }
  for (int j = 0; j < prediction_steps_; j++)  // 力限制
  {
    lbx.push_back(vertical_acceleration_limits_.at(0));
    ubx.push_back(vertical_acceleration_limits_.at(1));
  }

  // size_t current_states_size = _current_states.size();
  // size_t desired_params_size = _desired_params.size();
  // std::cout << "current_states_size" << lbx.size() << std::endl;

  for (size_t i = 0; i < current_states_size; i++)  // 传入参数
  {
    parameters.push_back(current_states[i]);
  }
  for (size_t i = 0; i < desired_params_size; i++) {
    parameters.push_back(desired_parameters[i]);
  }
  // std::cout << parameters << std::endl;

  // 求解参数设置
  solver_arguments_["lbx"] = lbx;
  solver_arguments_["ubx"] = ubx;
  solver_arguments_["p"] = parameters;
  solver_arguments_["x0"] = initial_guess_;

  // 求解 — 计时并诊断
  ros::WallTime t_solve_start = ros::WallTime::now();
  result_ = solver_(solver_arguments_);
  double solve_time_ms =
      (ros::WallTime::now() - t_solve_start).toSec() * 1000.0;

  double final_cost = static_cast<double>(result_.at("f"));
  if (solve_time_ms > 20.0) {
    ROS_ERROR(
        "NMPC SOLVE TOOK %.1f ms (>20ms! OFFBOARD TIMEOUT RISK!) cost=%.4f",
        solve_time_ms, final_cost);
  } else if (solve_time_ms > 10.0) {
    ROS_WARN("NMPC solve time: %.1f ms (>10ms, loop budget tight) cost=%.4f",
             solve_time_ms, final_cost);
  } else {
    ROS_INFO_THROTTLE(2.0, "NMPC solve: %.1f ms  cost=%.4f", solve_time_ms,
                      final_cost);
  }

  // 获取优化变量
  std::vector<double> cmd_control_all(result_.at("x"));
  // std::cout << "cmd_control_all: " << cmd_control_all << std::endl;
  std::vector<double> cmd_control_wx, cmd_control_wy, cmd_control_wz,
      cmd_control_acc_z;
  cmd_control_wx.assign(cmd_control_all.begin(),
                        cmd_control_all.begin() + prediction_steps_);
  cmd_control_wy.assign(cmd_control_all.begin() + prediction_steps_,
                        cmd_control_all.begin() + 2 * prediction_steps_);
  cmd_control_wz.assign(cmd_control_all.begin() + 2 * prediction_steps_,
                        cmd_control_all.begin() + 3 * prediction_steps_);
  cmd_control_acc_z.assign(cmd_control_all.begin() + 3 * prediction_steps_,
                           cmd_control_all.begin() + 4 * prediction_steps_);

  // 存储下一时刻最初优化解猜测
  std::vector<double> initial_guess;

  for (int i = 0; i < prediction_steps_; i++) {
    initial_guess.push_back(cmd_control_wx.at(i));
  }
  for (int i = 0; i < prediction_steps_; i++) {
    initial_guess.push_back(cmd_control_wy.at(i));
  }
  for (int i = 0; i < prediction_steps_; i++) {
    initial_guess.push_back(cmd_control_wz.at(i));
  }
  for (int i = 0; i < prediction_steps_; i++) {
    initial_guess.push_back(cmd_control_acc_z.at(i));
  }

  initial_guess_ = initial_guess;

  // 控制序列的第一组作为当前控制量
  angular_rate_command_ << cmd_control_wx.front(), cmd_control_wy.front(),
      cmd_control_wz.front();
  thrust_command_ = thrust_estimator_.estimateThrust(cmd_control_acc_z.front());
  // std::cout << "thrust_command_: " << thrust_command_ << std::endl;
}
