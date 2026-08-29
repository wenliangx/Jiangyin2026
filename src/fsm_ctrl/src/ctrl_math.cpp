#include "fsm_ctrl/ctrl_math.hpp"

#include <cmath>

namespace {

constexpr double kGravity = 9.8015;

}  // namespace

Eigen::Vector3d quaternionToEuler(double w, double x, double y, double z) {
  Eigen::Vector3d euler;

  const double sin_roll = 2.0 * (w * x + y * z);
  const double cos_roll = 1.0 - 2.0 * (x * x + y * y);
  euler[0] = std::atan2(sin_roll, cos_roll);

  const double sin_pitch = 2.0 * (w * y - z * x);
  if (std::abs(sin_pitch) >= 1.0) {
    euler[1] = std::copysign(M_PI / 2.0, sin_pitch);
  } else {
    euler[1] = std::asin(sin_pitch);
  }

  const double sin_yaw = 2.0 * (w * z + x * y);
  const double cos_yaw = 1.0 - 2.0 * (y * y + z * z);
  euler[2] = std::atan2(sin_yaw, cos_yaw);
  return euler;
}

Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond& quaternion) {
  return quaternionToEuler(quaternion.w(), quaternion.x(), quaternion.y(),
                           quaternion.z());
}

Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw) {
  Eigen::Quaterniond quaternion;
  const double cos_roll = std::cos(roll * 0.5);
  const double sin_roll = std::sin(roll * 0.5);
  const double cos_pitch = std::cos(pitch * 0.5);
  const double sin_pitch = std::sin(pitch * 0.5);
  const double cos_yaw = std::cos(yaw * 0.5);
  const double sin_yaw = std::sin(yaw * 0.5);
  quaternion.w() =
      cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw;
  quaternion.x() =
      sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw;
  quaternion.y() =
      cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw;
  quaternion.z() =
      cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw;
  return quaternion;
}

Eigen::Quaterniond eulerToQuaternion(const Eigen::Vector3d& euler) {
  return eulerToQuaternion(euler[0], euler[1], euler[2]);
}

Eigen::Matrix3d quaternionToRotationMatrix(double w, double x, double y,
                                           double z) {
  Eigen::Matrix3d matrix;
  matrix << w * w + x * x - y * y - z * z, 2.0 * (x * y - w * z),
      2.0 * (x * z + w * y), 2.0 * (x * y + w * z),
      w * w - x * x + y * y - z * z, 2.0 * (y * z - w * x),
      2.0 * (x * z - w * y), 2.0 * (y * z + w * x),
      w * w - x * x - y * y + z * z;
  return matrix;
}

Eigen::Matrix3d quaternionToRotationMatrix(
    const Eigen::Quaterniond& quaternion) {
  return quaternionToRotationMatrix(quaternion.w(), quaternion.x(),
                                    quaternion.y(), quaternion.z());
}

Eigen::Matrix3d eulerToRotationMatrix(double roll, double pitch, double yaw) {
  Eigen::Matrix3d roll_matrix;
  Eigen::Matrix3d pitch_matrix;
  Eigen::Matrix3d yaw_matrix;
  roll_matrix << 1.0, 0.0, 0.0, 0.0, std::cos(roll), -std::sin(roll), 0.0,
      std::sin(roll), std::cos(roll);
  pitch_matrix << std::cos(pitch), 0.0, std::sin(pitch), 0.0, 1.0, 0.0,
      -std::sin(pitch), 0.0, std::cos(pitch);
  yaw_matrix << std::cos(yaw), -std::sin(yaw), 0.0, std::sin(yaw),
      std::cos(yaw), 0.0, 0.0, 0.0, 1.0;
  return yaw_matrix * pitch_matrix * roll_matrix;
}

Eigen::Matrix3d eulerToRotationMatrix(const Eigen::Vector3d& euler) {
  return eulerToRotationMatrix(euler[0], euler[1], euler[2]);
}

Eigen::Vector3d rotationMatrixToEuler(const Eigen::Matrix3d& matrix) {
  Eigen::Vector3d euler;
  euler[0] = std::atan2(matrix(2, 1), matrix(2, 2));
  euler[1] = std::asin(-matrix(2, 0));
  euler[2] = std::atan2(matrix(1, 0), matrix(0, 0));
  return euler;
}

Eigen::Quaterniond rotationMatrixToQuaternion(const Eigen::Matrix3d& matrix) {
  Eigen::Quaterniond quaternion;
  quaternion.w() =
      std::sqrt(1.0 + matrix(0, 0) + matrix(1, 1) + matrix(2, 2)) * 0.5;
  quaternion.x() = (matrix(2, 1) - matrix(1, 2)) / (4.0 * quaternion.w());
  quaternion.y() = (matrix(0, 2) - matrix(2, 0)) / (4.0 * quaternion.w());
  quaternion.z() = (matrix(1, 0) - matrix(0, 1)) / (4.0 * quaternion.w());
  return quaternion;
}

void ThrustEstimator::configure(int control_rate, double hover_thrust) {
  control_period_ = 1.0 / control_rate;
  thrust_to_acceleration_ = kGravity / hover_thrust;
}

double ThrustEstimator::estimateThrust(double acceleration) {
  const ros::Time now = ros::Time::now();
  if (thrust_history_.empty()) {
    const double thrust = acceleration / thrust_to_acceleration_;
    thrust_history_.push({now, thrust});
    ROS_WARN("No data for thrust estimation! Slope unchanged!");
    return thrust;
  }

  const std::pair<ros::Time, double> time_thrust = thrust_history_.front();
  const double elapsed = (now - time_thrust.first).toSec();
  if (elapsed > 1.5 * control_period_) {
    ROS_WARN("Control frequency lower than ROS rate!");
  }

  double thrust = time_thrust.second;
  thrust_history_.pop();

  const double gamma =
      1.0 / (forgetting_factor_ + thrust * covariance_ * thrust);
  const double gain = gamma * covariance_ * thrust;
  thrust_to_acceleration_ +=
      gain * (acceleration - thrust * thrust_to_acceleration_);
  covariance_ = (1.0 - gain * thrust) * covariance_ / forgetting_factor_;
  thrust = acceleration / thrust_to_acceleration_;

  thrust_history_.push({now, thrust});
  return thrust;
}
