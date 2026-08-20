#include <cmath>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

namespace {

Eigen::Matrix4d makeMapFromLocal(const Eigen::Vector3d& initial_position,
                                 double initial_yaw) {
  Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
  result.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(-initial_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  result.block<3, 1>(0, 3) = -result.block<3, 3>(0, 0) * initial_position;
  return result;
}

TEST(FrameMath, InitialPositionBecomesOrigin) {
  const Eigen::Vector3d initial_position(12.0, -3.0, 1.2);
  const Eigen::Matrix4d map_from_local = makeMapFromLocal(initial_position, 0.7);
  Eigen::Vector4d homogeneous;
  homogeneous << initial_position, 1.0;
  const Eigen::Vector4d in_map = map_from_local * homogeneous;
  EXPECT_NEAR(in_map.x(), 0.0, 1e-12);
  EXPECT_NEAR(in_map.y(), 0.0, 1e-12);
  EXPECT_NEAR(in_map.z(), 0.0, 1e-12);
}

TEST(FrameMath, InitialHeadingBecomesPositiveX) {
  const double yaw = -1.1;
  const Eigen::Matrix4d map_from_local =
      makeMapFromLocal(Eigen::Vector3d(4.0, 5.0, 0.0), yaw);
  const Eigen::Vector3d heading_local(std::cos(yaw), std::sin(yaw), 0.0);
  const Eigen::Vector3d heading_map = map_from_local.block<3, 3>(0, 0) * heading_local;
  EXPECT_NEAR(heading_map.x(), 1.0, 1e-12);
  EXPECT_NEAR(heading_map.y(), 0.0, 1e-12);
  EXPECT_NEAR(heading_map.z(), 0.0, 1e-12);
}

TEST(FrameMath, MapZRemainsGravityAligned) {
  const Eigen::Matrix4d map_from_local =
      makeMapFromLocal(Eigen::Vector3d(0.0, 0.0, 2.0), 2.2);
  const Eigen::Vector3d z_map =
      map_from_local.block<3, 3>(0, 0) * Eigen::Vector3d::UnitZ();
  EXPECT_NEAR(z_map.x(), 0.0, 1e-12);
  EXPECT_NEAR(z_map.y(), 0.0, 1e-12);
  EXPECT_NEAR(z_map.z(), 1.0, 1e-12);
}

TEST(FrameMath, GlobalPoseAndCloudUseSameRigidTransform) {
  Eigen::Matrix4d global_from_local = Eigen::Matrix4d::Identity();
  global_from_local.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  global_from_local.block<3, 1>(0, 3) = Eigen::Vector3d(10.0, -2.0, 0.5);
  Eigen::Vector4d local;
  local << 2.0, 1.0, 3.0, 1.0;
  const Eigen::Vector4d pose_global = global_from_local * local;
  const Eigen::Vector4d cloud_global = global_from_local * local;
  EXPECT_NEAR(pose_global.x(), 9.0, 1e-12);
  EXPECT_NEAR(pose_global.y(), 0.0, 1e-12);
  EXPECT_NEAR(pose_global.z(), 3.5, 1e-12);
  EXPECT_TRUE(pose_global.isApprox(cloud_global, 1e-12));
}

TEST(FrameMath, WorldExpressedVelocityUsesRotationOnly) {
  const Eigen::Matrix3d rotation =
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d global_velocity = rotation * Eigen::Vector3d(2.0, 0.0, 0.0);
  EXPECT_NEAR(global_velocity.x(), 0.0, 1e-12);
  EXPECT_NEAR(global_velocity.y(), 2.0, 1e-12);
  EXPECT_NEAR(global_velocity.z(), 0.0, 1e-12);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
