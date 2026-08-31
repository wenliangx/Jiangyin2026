#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ra_lio/Pose6D.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/QR>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <vector>

namespace ra_lio {

inline constexpr double kPi = 3.14159265358;
inline constexpr double kGravity = 9.801;
inline constexpr std::size_t kNearestNeighborCount = 5;

using Point = pcl::PointXYZINormal;
using PointCloud = pcl::PointCloud<Point>;
using PointVector = std::vector<Point, Eigen::aligned_allocator<Point>>;
using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using Vector3f = Eigen::Vector3f;
using Matrix3f = Eigen::Matrix3f;

// Temporary internal aliases keep the numerical code readable during migration.
using PointType = Point;
using PointCloudXYZI = PointCloud;
using V3D = Vector3d;
using M3D = Matrix3d;
using V3F = Vector3f;
using M3F = Matrix3f;

inline const Matrix3d kIdentity3d = Matrix3d::Identity();
inline const Matrix3f kIdentity3f = Matrix3f::Identity();
inline const Vector3d kZero3d = Vector3d::Zero();
inline const Vector3f kZero3f = Vector3f::Zero();

struct MeasureGroup {
  double lidar_beg_time{0.0};
  double lidar_end_time{0.0};
  PointCloud::Ptr lidar{new PointCloud()};
  std::deque<sensor_msgs::Imu::ConstPtr> imu;
};

template <typename Derived>
[[nodiscard]] inline Eigen::Vector3d vectorFromArray(const Derived& values) {
  return {values[0], values[1], values[2]};
}

template <typename Derived>
[[nodiscard]] inline Eigen::Matrix3d matrixFromArray(const Derived& values) {
  Eigen::Matrix3d matrix;
  matrix << values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7],
      values[8];
  return matrix;
}

template <typename Derived>
[[nodiscard]] inline Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(
    const Eigen::MatrixBase<Derived>& vector) {
  using Scalar = typename Derived::Scalar;
  Eigen::Matrix<Scalar, 3, 3> matrix;
  matrix << Scalar{0}, -vector[2], vector[1], vector[2], Scalar{0}, -vector[0], -vector[1],
      vector[0], Scalar{0};
  return matrix;
}

template <typename T>
[[nodiscard]] inline Pose6D makePose6d(double time, const Eigen::Matrix<T, 3, 1>& acceleration,
                                       const Eigen::Matrix<T, 3, 1>& angular_velocity,
                                       const Eigen::Matrix<T, 3, 1>& velocity,
                                       const Eigen::Matrix<T, 3, 1>& position,
                                       const Eigen::Matrix<T, 3, 3>& rotation) {
  Pose6D pose;
  pose.offset_time = time;
  for (int row = 0; row < 3; ++row) {
    pose.acc[row] = acceleration(row);
    pose.gyr[row] = angular_velocity(row);
    pose.vel[row] = velocity(row);
    pose.pos[row] = position(row);
    for (int column = 0; column < 3; ++column) {
      pose.rot[row * 3 + column] = rotation(row, column);
    }
  }
  return pose;
}

[[nodiscard]] inline float squaredDistance(const Point& lhs, const Point& rhs) noexcept {
  const float dx = lhs.x - rhs.x;
  const float dy = lhs.y - rhs.y;
  const float dz = lhs.z - rhs.z;
  return dx * dx + dy * dy + dz * dz;
}

template <typename T>
[[nodiscard]] inline bool estimatePlane(Eigen::Matrix<T, 4, 1>& plane, const PointVector& points,
                                        T threshold) {
  if (points.size() < kNearestNeighborCount) {
    return false;
  }
  Eigen::Matrix<T, kNearestNeighborCount, 3> coefficients;
  Eigen::Matrix<T, kNearestNeighborCount, 1> constants;
  constants.setConstant(T{-1});
  for (std::size_t index = 0; index < kNearestNeighborCount; ++index) {
    coefficients(index, 0) = points[index].x;
    coefficients(index, 1) = points[index].y;
    coefficients(index, 2) = points[index].z;
  }
  const Eigen::Matrix<T, 3, 1> normal = coefficients.colPivHouseholderQr().solve(constants);
  const T norm = normal.norm();
  if (norm <= std::numeric_limits<T>::epsilon()) {
    return false;
  }
  plane.template head<3>() = normal / norm;
  plane(3) = T{1} / norm;
  for (std::size_t index = 0; index < kNearestNeighborCount; ++index) {
    const T residual = plane(0) * points[index].x + plane(1) * points[index].y +
                       plane(2) * points[index].z + plane(3);
    if (std::abs(residual) > threshold) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] inline Eigen::Vector3d rotationToYpr(const Eigen::Matrix3d& rotation) {
  const Eigen::Vector3d forward = rotation.col(0);
  const Eigen::Vector3d left = rotation.col(1);
  const Eigen::Vector3d up = rotation.col(2);
  const double yaw = std::atan2(forward.y(), forward.x());
  const double pitch =
      std::atan2(-forward.z(), forward.x() * std::cos(yaw) + forward.y() * std::sin(yaw));
  const double roll = std::atan2(up.x() * std::sin(yaw) - up.y() * std::cos(yaw),
                                 -left.x() * std::sin(yaw) + left.y() * std::cos(yaw));
  return Eigen::Vector3d{yaw, pitch, roll} * (180.0 / kPi);
}

template <typename Derived>
[[nodiscard]] inline Eigen::Matrix<typename Derived::Scalar, 3, 3> yprToRotation(
    const Eigen::MatrixBase<Derived>& ypr) {
  using Scalar = typename Derived::Scalar;
  const Scalar yaw = ypr(0) * Scalar{kPi / 180.0};
  const Scalar pitch = ypr(1) * Scalar{kPi / 180.0};
  const Scalar roll = ypr(2) * Scalar{kPi / 180.0};
  const Eigen::AngleAxis<Scalar> yaw_rotation(yaw, Eigen::Matrix<Scalar, 3, 1>::UnitZ());
  const Eigen::AngleAxis<Scalar> pitch_rotation(pitch, Eigen::Matrix<Scalar, 3, 1>::UnitY());
  const Eigen::AngleAxis<Scalar> roll_rotation(roll, Eigen::Matrix<Scalar, 3, 1>::UnitX());
  return (yaw_rotation * pitch_rotation * roll_rotation).toRotationMatrix();
}

[[nodiscard]] inline Eigen::Matrix3d gravityToRotation(const Eigen::Vector3d& gravity) {
  Eigen::Matrix3d rotation =
      Eigen::Quaterniond::FromTwoVectors(gravity.normalized(), Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  const double yaw = rotationToYpr(rotation).x();
  return yprToRotation(Eigen::Vector3d{-yaw, 0.0, 0.0}) * rotation;
}

}  // namespace ra_lio
