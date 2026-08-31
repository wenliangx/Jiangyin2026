#pragma once

#include <ros/node_handle.h>

#include <Eigen/Core>
#include <string>

#include "preprocess.hpp"

namespace ra_lio {

struct MappingConfig {
  bool path_enabled{true};
  bool speed_vector_enabled{true};
  bool scan_publish_enabled{true};
  bool dense_publish_enabled{true};
  bool body_frame_publish_enabled{true};
  bool time_sync_enabled{false};
  bool extrinsic_estimation_enabled{true};
  bool pcd_save_enabled{false};
  int maximum_iterations{4};
  int pcd_save_interval{-1};
  std::string map_file_path;
  std::string lidar_topic{"/livox/lidar"};
  std::string imu_topic{"/livox/imu"};
  double lidar_to_imu_time_offset{0.0};
  double corner_leaf_size{0.5};
  double surface_leaf_size{0.5};
  double map_leaf_size{0.5};
  double cube_side_length{200.0};
  float detection_range{300.0F};
  double field_of_view_degrees{180.0};
  double gyro_covariance{0.1};
  double acceleration_covariance{0.1};
  double gyro_bias_covariance{0.0001};
  double acceleration_bias_covariance{0.0001};
  Eigen::Vector3d lidar_to_imu_translation{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d lidar_to_imu_rotation{Eigen::Matrix3d::Identity()};
  bool has_complete_extrinsics{false};
  PreprocessorConfig preprocessor;

  [[nodiscard]] static MappingConfig load(const ros::NodeHandle& node);
  [[nodiscard]] bool valid(std::string& reason) const;
};

}  // namespace ra_lio
