#pragma once

#include <sensor_msgs/Imu.h>

#include <Eigen/Core>
#include <memory>
#include <vector>

#include "common_lib.hpp"
#include "esekfom.hpp"

namespace ra_lio {

struct ImuProcessorConfig {
  Eigen::Vector3d lidar_to_imu_translation{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d lidar_to_imu_rotation{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d gyro_covariance{Eigen::Vector3d::Constant(0.1)};
  Eigen::Vector3d acceleration_covariance{Eigen::Vector3d::Constant(0.1)};
  Eigen::Vector3d gyro_bias_covariance{Eigen::Vector3d::Constant(0.0001)};
  Eigen::Vector3d acceleration_bias_covariance{Eigen::Vector3d::Constant(0.0001)};
};

class ImuProcessor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcessor();
  ~ImuProcessor();
  ImuProcessor(ImuProcessor&&) noexcept;
  ImuProcessor& operator=(ImuProcessor&&) noexcept;
  ImuProcessor(const ImuProcessor&) = delete;
  ImuProcessor& operator=(const ImuProcessor&) = delete;
  void reset();
  void configure(const ImuProcessorConfig& config);
  void process(const MeasureGroup& measurements, ErrorStateKalmanFilter& filter,
               PointCloud::Ptr& undistorted_cloud);

  [[nodiscard]] double firstLidarTime() const noexcept;
  void setFirstLidarTime(double time) noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ra_lio
