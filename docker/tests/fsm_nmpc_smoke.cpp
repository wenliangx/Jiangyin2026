#include <fsm_ctrl/NMPC_Controller.hpp>

#include <cmath>
#include <iostream>
#include <vector>

int main() {
  ros::Time::init();

  Eigen::Matrix<float, 3, 1> q_position;
  Eigen::Matrix<float, 3, 1> q_velocity;
  Eigen::Matrix<float, 3, 1> q_attitude;
  Eigen::Matrix<float, 3, 1> r_angular;
  q_position.setOnes();
  q_velocity.setOnes();
  q_attitude.setOnes();
  r_angular.setOnes();

  constexpr int predict_steps = 8;
  NMPC_Ctrller_simple controller(
      0.02, std::array<double, 2>{{0.0, 15.0}},
      std::array<double, 2>{{-3.14, 3.14}}, predict_steps, 0.05, 10, 4,
      q_position, q_velocity, q_attitude, r_angular, 1.0, 0.196);

  const std::vector<double> current{
      0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  std::vector<double> desired;
  desired.reserve((predict_steps + 1) * 10 + predict_steps * 4);
  for (int index = 0; index <= predict_steps; ++index) {
    desired.insert(desired.end(),
                   {0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0});
  }
  for (int index = 0; index < predict_steps; ++index) {
    desired.insert(desired.end(), {0.0, 0.0, 0.0, 9.8015});
  }

  controller.optimal_solution(current, desired);
  const Eigen::Vector3d angular_rate = controller.getwCommand();
  const double thrust = controller.getAcc_zCommand();
  const std::vector<double> solution = controller.getOptimalCommand();

  std::cout << "NMPC body_rate=" << angular_rate.transpose()
            << " thrust=" << thrust
            << " variables=" << solution.size() << std::endl;
  if (solution.size() != static_cast<std::size_t>(predict_steps * 4) ||
      !angular_rate.allFinite() || !std::isfinite(thrust)) {
    return 1;
  }
  return 0;
}
