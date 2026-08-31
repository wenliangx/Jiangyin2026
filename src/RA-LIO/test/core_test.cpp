#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/make_shared.hpp>
#include <cmath>

#include "common_lib.hpp"
#include "esekfom.hpp"
#include "mapping_config.hpp"
#include "preprocess.hpp"
#include "use_ikfom.hpp"

namespace ra_lio {
namespace {

template <typename VendorPoint>
sensor_msgs::PointCloud2::ConstPtr makePointCloudMessage(const std::vector<VendorPoint>& points) {
  pcl::PointCloud<VendorPoint> cloud;
  cloud.reserve(points.size());
  for (const auto& point : points) {
    cloud.push_back(point);
  }
  cloud.width = points.size();
  cloud.height = 1;
  auto message = boost::make_shared<sensor_msgs::PointCloud2>();
  pcl::toROSMsg(cloud, *message);
  return message;
}

TEST(CommonMath, ComputesSquaredDistance) {
  Point first;
  first.x = 1.0F;
  first.y = 2.0F;
  first.z = 3.0F;
  Point second;
  second.x = 4.0F;
  second.y = 6.0F;
  second.z = 3.0F;
  EXPECT_FLOAT_EQ(squaredDistance(first, second), 25.0F);
}

TEST(CommonMath, EstimatesNormalizedPlane) {
  PointVector points(kNearestNeighborCount);
  const double coordinates[kNearestNeighborCount][2] = {
      {0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}, {-1.0, 0.5}};
  for (std::size_t index = 0; index < points.size(); ++index) {
    points[index].x = coordinates[index][0];
    points[index].y = coordinates[index][1];
    points[index].z = 2.0;
  }
  Eigen::Vector4d plane;
  ASSERT_TRUE(estimatePlane(plane, points, 1e-6));
  EXPECT_NEAR(std::abs(plane.z()), 1.0, 1e-9);
  EXPECT_NEAR(std::abs(plane.w()), 2.0, 1e-9);
}

TEST(ProcessModel, PreservesOriginalStateLayout) {
  State state;
  state.vel = {1.0, 2.0, 3.0};
  state.bg = {0.1, 0.2, 0.3};
  state.ba = {0.4, 0.5, 0.6};
  ImuInput input;
  input.gyro = {1.1, 2.2, 3.3};
  input.acc = {0.4, 0.5, kGravity + 0.6};

  const StateVector derivative = processModel(state, input);
  EXPECT_TRUE(derivative.segment<3>(0).isApprox(state.vel));
  EXPECT_TRUE(derivative.segment<3>(3).isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_TRUE(derivative.segment<3>(12).isApprox(Eigen::Vector3d::Zero(), 1e-12));
}

TEST(ErrorStateKalmanFilter, BoxPlusUpdatesTranslationWithoutChangingRotations) {
  ErrorStateKalmanFilter filter;
  State initial;
  StateVector delta = StateVector::Zero();
  delta.segment<3>(0) = Eigen::Vector3d(1.0, -2.0, 3.0);
  delta.segment<3>(12) = Eigen::Vector3d(0.5, 0.25, -0.5);
  const State updated = filter.boxplus(initial, delta);
  EXPECT_TRUE(updated.pos.isApprox(Eigen::Vector3d(1.0, -2.0, 3.0)));
  EXPECT_TRUE(updated.vel.isApprox(Eigen::Vector3d(0.5, 0.25, -0.5)));
  EXPECT_TRUE(updated.rot.matrix().isApprox(Eigen::Matrix3d::Identity()));
}

TEST(MappingConfig, RejectsIncompleteExtrinsicsAndInvalidRanges) {
  MappingConfig config;
  std::string reason;
  EXPECT_FALSE(config.valid(reason));
  EXPECT_NE(reason.find("extrinsic"), std::string::npos);

  config.has_complete_extrinsics = true;
  config.surface_leaf_size = 0.0;
  EXPECT_FALSE(config.valid(reason));
  EXPECT_NE(reason.find("voxel"), std::string::npos);

  config.surface_leaf_size = 0.5;
  EXPECT_TRUE(config.valid(reason));
  EXPECT_TRUE(reason.empty());

  config.preprocessor.timestamp_unit = 4;
  EXPECT_FALSE(config.valid(reason));
  EXPECT_NE(reason.find("timestamp_unit"), std::string::npos);
}

TEST(Preprocessor, PreservesAllSupportedLidarMessageFormats) {
  Preprocessor preprocessor;
  PreprocessorConfig config;
  config.blind = 0.01;
  config.point_filter_num = 1;
  config.feature_enabled = false;
  PointCloud::Ptr output{new PointCloud()};

  auto livox = boost::make_shared<livox_ros_driver2::CustomMsg>();
  livox->point_num = 3;
  livox->points.resize(livox->point_num);
  for (std::size_t index = 0; index < livox->points.size(); ++index) {
    auto& point = livox->points[index];
    point.x = static_cast<float>(index + 1);
    point.y = 0.25F;
    point.z = 0.5F;
    point.reflectivity = static_cast<std::uint8_t>(10 + index);
    point.tag = 0x10;
    point.line = 0;
    point.offset_time = static_cast<std::uint32_t>(index * 1'000'000);
  }
  config.lidar_type = AVIA;
  config.scan_lines = 6;
  preprocessor.configure(config);
  preprocessor.process(livox, output);
  ASSERT_EQ(output->size(), 2U);
  EXPECT_FLOAT_EQ(output->back().curvature, 2.0F);

  ouster_ros::Point ouster_first{};
  ouster_first.x = 1.0F;
  ouster_first.y = 0.25F;
  ouster_first.z = 0.5F;
  ouster_first.intensity = 11.0F;
  ouster_first.t = 1'000'000;
  ouster_first.ring = 0;
  auto ouster_second = ouster_first;
  ouster_second.x = 2.0F;
  ouster_second.t = 2'000'000;
  config.lidar_type = OUST64;
  config.timestamp_unit = NS;
  config.scan_lines = 64;
  preprocessor.configure(config);
  preprocessor.process(makePointCloudMessage(std::vector{ouster_first, ouster_second}), output);
  ASSERT_EQ(output->size(), 2U);
  EXPECT_FLOAT_EQ(output->back().curvature, 2.0F);

  velodyne_ros::Point velodyne_first{};
  velodyne_first.x = 1.0F;
  velodyne_first.y = 0.25F;
  velodyne_first.z = 0.5F;
  velodyne_first.intensity = 12.0F;
  velodyne_first.time = 0.001F;
  velodyne_first.ring = 0;
  auto velodyne_second = velodyne_first;
  velodyne_second.x = 2.0F;
  velodyne_second.time = 0.002F;
  config.lidar_type = VELO16;
  config.timestamp_unit = SEC;
  config.scan_lines = 16;
  preprocessor.configure(config);
  preprocessor.process(makePointCloudMessage(std::vector{velodyne_first, velodyne_second}), output);
  ASSERT_EQ(output->size(), 2U);
  EXPECT_FLOAT_EQ(output->back().curvature, 2.0F);

  rslidar_ros::Point robosense_first{};
  robosense_first.x = 1.0F;
  robosense_first.y = 0.25F;
  robosense_first.z = 0.5F;
  robosense_first.intensity = 13;
  robosense_first.ring = 0;
  robosense_first.timestamp = 10.0;
  auto robosense_second = robosense_first;
  robosense_second.x = 2.0F;
  robosense_second.timestamp = 10.002;
  config.lidar_type = RS32;
  config.scan_lines = 32;
  preprocessor.configure(config);
  preprocessor.process(makePointCloudMessage(std::vector{robosense_first, robosense_second}),
                       output);
  ASSERT_EQ(output->size(), 2U);
  EXPECT_NEAR(output->back().curvature, 2.0F, 1e-3F);

  vanjee_ros::Point vanjee_first{};
  vanjee_first.x = 1.0F;
  vanjee_first.y = 0.25F;
  vanjee_first.z = 0.5F;
  vanjee_first.intensity = 14.0F;
  vanjee_first.ring = 0;
  vanjee_first.timestamp = 1'000.0;
  auto vanjee_second = vanjee_first;
  vanjee_second.x = 2.0F;
  vanjee_second.timestamp = 2'000.0;
  config.lidar_type = VANJEE16;
  config.timestamp_unit = US;
  config.scan_lines = 16;
  preprocessor.configure(config);
  preprocessor.process(makePointCloudMessage(std::vector{vanjee_first, vanjee_second}), output);
  ASSERT_EQ(output->size(), 2U);
  EXPECT_FLOAT_EQ(output->back().curvature, 2.0F);
}

}  // namespace
}  // namespace ra_lio

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
