#include "mapping_config.hpp"

#include <array>
#include <vector>

#include "common_lib.hpp"

namespace ra_lio {

MappingConfig MappingConfig::load(const ros::NodeHandle& node) {
  MappingConfig config;
  node.param("publish/path_en", config.path_enabled, config.path_enabled);
  node.param("publish/speed_vector_en", config.speed_vector_enabled, config.speed_vector_enabled);
  node.param("publish/scan_publish_en", config.scan_publish_enabled, config.scan_publish_enabled);
  node.param("publish/dense_publish_en", config.dense_publish_enabled,
             config.dense_publish_enabled);
  node.param("publish/scan_bodyframe_pub_en", config.body_frame_publish_enabled,
             config.body_frame_publish_enabled);
  node.param("max_iteration", config.maximum_iterations, config.maximum_iterations);
  node.param("map_file_path", config.map_file_path, config.map_file_path);
  node.param("common/lid_topic", config.lidar_topic, config.lidar_topic);
  node.param("common/imu_topic", config.imu_topic, config.imu_topic);
  node.param("common/time_sync_en", config.time_sync_enabled, config.time_sync_enabled);
  node.param("common/time_offset_lidar_to_imu", config.lidar_to_imu_time_offset,
             config.lidar_to_imu_time_offset);
  node.param("filter_size_corner", config.corner_leaf_size, config.corner_leaf_size);
  node.param("filter_size_surf", config.surface_leaf_size, config.surface_leaf_size);
  node.param("filter_size_map", config.map_leaf_size, config.map_leaf_size);
  node.param("cube_side_length", config.cube_side_length, config.cube_side_length);
  node.param("mapping/det_range", config.detection_range, config.detection_range);
  node.param("mapping/fov_degree", config.field_of_view_degrees, config.field_of_view_degrees);
  node.param("mapping/gyr_cov", config.gyro_covariance, config.gyro_covariance);
  node.param("mapping/acc_cov", config.acceleration_covariance, config.acceleration_covariance);
  node.param("mapping/b_gyr_cov", config.gyro_bias_covariance, config.gyro_bias_covariance);
  node.param("mapping/b_acc_cov", config.acceleration_bias_covariance,
             config.acceleration_bias_covariance);
  node.param("preprocess/blind", config.preprocessor.blind, config.preprocessor.blind);
  node.param("preprocess/lidar_type", config.preprocessor.lidar_type,
             config.preprocessor.lidar_type);
  node.param("preprocess/scan_line", config.preprocessor.scan_lines,
             config.preprocessor.scan_lines);
  node.param("preprocess/timestamp_unit", config.preprocessor.timestamp_unit,
             config.preprocessor.timestamp_unit);
  node.param("preprocess/scan_rate", config.preprocessor.scan_rate, config.preprocessor.scan_rate);
  node.param("point_filter_num", config.preprocessor.point_filter_num,
             config.preprocessor.point_filter_num);
  node.param("feature_extract_enable", config.preprocessor.feature_enabled,
             config.preprocessor.feature_enabled);
  node.param("mapping/extrinsic_est_en", config.extrinsic_estimation_enabled,
             config.extrinsic_estimation_enabled);
  node.param("pcd_save/pcd_save_en", config.pcd_save_enabled, config.pcd_save_enabled);
  node.param("pcd_save/interval", config.pcd_save_interval, config.pcd_save_interval);

  std::vector<double> translation;
  std::vector<double> rotation;
  node.param("mapping/extrinsic_T", translation, std::vector<double>{});
  node.param("mapping/extrinsic_R", rotation, std::vector<double>{});
  if (translation.size() == 3) {
    config.lidar_to_imu_translation = vectorFromArray(translation);
  }
  if (rotation.size() == 9) {
    config.lidar_to_imu_rotation = matrixFromArray(rotation);
  }
  config.has_complete_extrinsics = translation.size() == 3 && rotation.size() == 9;
  return config;
}

bool MappingConfig::valid(std::string& reason) const {
  if (!has_complete_extrinsics) {
    reason = "mapping/extrinsic_T must have 3 values and mapping/extrinsic_R must have 9 values";
  } else if (maximum_iterations <= 0) {
    reason = "max_iteration must be positive";
  } else if (corner_leaf_size <= 0.0 || surface_leaf_size <= 0.0 || map_leaf_size <= 0.0) {
    reason = "voxel filter sizes must be positive";
  } else if (cube_side_length <= 0.0 || detection_range <= 0.0F) {
    reason = "mapping ranges must be positive";
  } else if (field_of_view_degrees <= 0.0 || field_of_view_degrees > 360.0) {
    reason = "mapping/fov_degree must be in (0, 360]";
  } else if (gyro_covariance <= 0.0 || acceleration_covariance <= 0.0 ||
             gyro_bias_covariance <= 0.0 || acceleration_bias_covariance <= 0.0) {
    reason = "IMU covariance parameters must be positive";
  } else if (preprocessor.lidar_type < AVIA || preprocessor.lidar_type > VANJEE16) {
    reason = "preprocess/lidar_type must be in [1, 5]";
  } else if (preprocessor.scan_lines <= 0 || preprocessor.scan_lines > 128) {
    reason = "preprocess/scan_line must be in [1, 128]";
  } else if (preprocessor.timestamp_unit < SEC || preprocessor.timestamp_unit > NS) {
    reason = "preprocess/timestamp_unit must be in [0, 3]";
  } else if (preprocessor.scan_rate <= 0) {
    reason = "preprocess/scan_rate must be positive";
  } else if (preprocessor.point_filter_num <= 0) {
    reason = "point_filter_num must be positive";
  } else if (preprocessor.blind < 0.0) {
    reason = "preprocess/blind must be non-negative";
  } else {
    reason.clear();
    return true;
  }
  return false;
}

}  // namespace ra_lio
