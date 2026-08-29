#pragma once

#include <array>
#include <casadi/casadi.hpp>
#include <eigen3/Eigen/Dense>
#include <fsm_ctrl/ctrl_math.hpp>
#include <map>
#include <string>
#include <vector>

namespace fsm_ctrl {

class NmpcController {
 public:
  NmpcController(const std::array<double, 2>& vertical_acceleration_limits,
                 const std::array<double, 2>& angular_rate_limits, int prediction_steps,
                 double prediction_step_seconds, int state_size, int input_size,
                 const Eigen::Matrix<float, 3, 1>& position_weights,
                 const Eigen::Matrix<float, 3, 1>& velocity_weights,
                 const Eigen::Matrix<float, 3, 1>& attitude_weights,
                 const Eigen::Matrix<float, 3, 1>& angular_rate_weights, double acceleration_weight,
                 double hover_thrust);

  void solve(const std::vector<double>& current_states,
             const std::vector<double>& desired_parameters);

  Eigen::Vector3d angularRateCommand() const { return angular_rate_command_; }
  double thrustCommand() const { return thrust_command_; }

 private:
  std::array<double, 2> vertical_acceleration_limits_;
  std::array<double, 2> angular_rate_limits_;
  int prediction_steps_;
  double prediction_step_seconds_;
  int state_size_;
  int input_size_;

  Eigen::Matrix<float, 3, 1> position_weights_;
  Eigen::Matrix<float, 3, 1> velocity_weights_;
  Eigen::Matrix<float, 3, 1> attitude_weights_;
  Eigen::Matrix<float, 3, 1> angular_rate_weights_;
  double acceleration_weight_;

  casadi::Function solver_;
  std::map<std::string, casadi::DM> result_;
  std::map<std::string, casadi::DM> solver_arguments_;
  std::vector<double> initial_guess_;

  Eigen::Vector3d angular_rate_command_;
  double thrust_command_{0.0};
  ThrustEstimator thrust_estimator_;
};

}  // namespace fsm_ctrl
