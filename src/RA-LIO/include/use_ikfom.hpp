#pragma once

#include <Eigen/Core>
#include <sophus/so3.hpp>

#include "common_lib.hpp"

namespace ra_lio {

inline constexpr int kStateDimension = 24;
inline constexpr int kNoiseDimension = 12;
using StateVector = Eigen::Matrix<double, kStateDimension, 1>;
using StateCovariance = Eigen::Matrix<double, kStateDimension, kStateDimension>;
using ProcessNoise = Eigen::Matrix<double, kNoiseDimension, kNoiseDimension>;

struct State {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d pos{Eigen::Vector3d::Zero()};
  Sophus::SO3d rot{};
  Sophus::SO3d offset_R_L_I{};
  Eigen::Vector3d offset_T_L_I{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bg{Eigen::Vector3d::Zero()};
  Eigen::Vector3d ba{Eigen::Vector3d::Zero()};
  Eigen::Vector3d grav{0.0, 0.0, -kGravity};
};

struct ImuInput {
  Eigen::Vector3d acc{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
};

[[nodiscard]] inline ProcessNoise makeProcessNoise() {
  ProcessNoise noise = ProcessNoise::Zero();
  noise.block<3, 3>(0, 0) = 0.0001 * Eigen::Matrix3d::Identity();
  noise.block<3, 3>(3, 3) = 0.0001 * Eigen::Matrix3d::Identity();
  noise.block<3, 3>(6, 6) = 0.00001 * Eigen::Matrix3d::Identity();
  noise.block<3, 3>(9, 9) = 0.00001 * Eigen::Matrix3d::Identity();
  return noise;
}

[[nodiscard]] inline StateVector processModel(const State& state, const ImuInput& input) {
  StateVector derivative = StateVector::Zero();
  const Eigen::Vector3d angular_velocity = input.gyro - state.bg;
  const Eigen::Vector3d acceleration = state.rot.matrix() * (input.acc - state.ba);
  derivative.segment<3>(0) = state.vel;
  derivative.segment<3>(3) = angular_velocity;
  derivative.segment<3>(12) = acceleration + state.grav;
  return derivative;
}

[[nodiscard]] inline StateCovariance processJacobian(const State& state, const ImuInput& input) {
  StateCovariance jacobian = StateCovariance::Zero();
  jacobian.block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d acceleration = input.acc - state.ba;
  jacobian.block<3, 3>(12, 3) = -state.rot.matrix() * Sophus::SO3d::hat(acceleration);
  jacobian.block<3, 3>(12, 18) = -state.rot.matrix();
  jacobian.block<3, 3>(12, 21) = Eigen::Matrix3d::Identity();
  jacobian.block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();
  return jacobian;
}

[[nodiscard]] inline Eigen::Matrix<double, kStateDimension, kNoiseDimension> noiseJacobian(
    const State& state, const ImuInput&) {
  Eigen::Matrix<double, kStateDimension, kNoiseDimension> jacobian =
      Eigen::Matrix<double, kStateDimension, kNoiseDimension>::Zero();
  jacobian.block<3, 3>(12, 3) = -state.rot.matrix();
  jacobian.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
  jacobian.block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();
  jacobian.block<3, 3>(18, 9) = Eigen::Matrix3d::Identity();
  return jacobian;
}

// Transitional aliases are package-private and keep the mathematical implementation
// reviewable while callers move to the descriptive type names above.
using state_ikfom = State;
using input_ikfom = ImuInput;
inline ProcessNoise process_noise_cov() { return makeProcessNoise(); }
inline StateVector get_f(const State& state, const ImuInput& input) {
  return processModel(state, input);
}
inline StateCovariance df_dx(const State& state, const ImuInput& input) {
  return processJacobian(state, input);
}
inline Eigen::Matrix<double, kStateDimension, kNoiseDimension> df_dw(const State& state,
                                                                     const ImuInput& input) {
  return noiseJacobian(state, input);
}

}  // namespace ra_lio
