#ifndef FSM_CTRL_CTRL_MATH_HPP_
#define FSM_CTRL_CTRL_MATH_HPP_

#include <ros/ros.h>

#include <eigen3/Eigen/Eigen>
#include <queue>
#include <utility>

class ThrustEstimator {
 public:
  void configure(int control_rate, double hover_thrust);
  double estimateThrust(double acceleration);

 private:
  double covariance_{1e6};
  const double forgetting_factor_{0.998};
  double control_period_{0.0};
  double thrust_to_acceleration_{0.0};
  std::queue<std::pair<ros::Time, double>> thrust_history_;
};

Eigen::Vector3d quaternionToEuler(double w, double x, double y, double z);
Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond& quaternion);

Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw);
Eigen::Quaterniond eulerToQuaternion(const Eigen::Vector3d& euler);

Eigen::Matrix3d quaternionToRotationMatrix(double w, double x, double y,
                                           double z);
Eigen::Matrix3d quaternionToRotationMatrix(
    const Eigen::Quaterniond& quaternion);

Eigen::Matrix3d eulerToRotationMatrix(double roll, double pitch, double yaw);
Eigen::Matrix3d eulerToRotationMatrix(const Eigen::Vector3d& euler);

Eigen::Vector3d rotationMatrixToEuler(const Eigen::Matrix3d& matrix);
Eigen::Quaterniond rotationMatrixToQuaternion(const Eigen::Matrix3d& matrix);

#endif  // FSM_CTRL_CTRL_MATH_HPP_
