#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <set>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

extern "C" {
#include <apriltag/apriltag.h>
#include <apriltag/common/image_u8.h>
#include <apriltag/tag36h11.h>
}

#include "uav_vision/landing_offset_estimator.hpp"
#include "uav_vision/landing_debug_overlay.hpp"
#include "uav_vision/landing_frame_transformer.hpp"
#include "uav_vision/landing_tag_detector.hpp"

namespace uav_vision {
namespace {

TagObservation observation(
    int id, double x, double y, int hamming = 0, double margin = 50.0) {
  TagObservation value;
  value.id = id;
  value.center = cv::Point2d(x, y);
  value.hamming = hamming;
  value.decision_margin = margin;
  return value;
}

std::vector<TagObservation> fiveAt(double x, double y) {
  return {
      observation(0, x - 20, y - 20),
      observation(1, x + 20, y - 20),
      observation(2, x + 20, y + 20),
      observation(3, x - 20, y + 20),
      observation(4, x, y),
  };
}

EstimatorConfig estimatorConfig() {
  EstimatorConfig config;
  config.expected_ids = {0, 1, 2, 3, 4};
  config.center_id = 4;
  config.max_hamming = 1;
  config.min_decision_margin = 20.0;
  config.filter_window = 3;
  config.max_pixel_jump = 50.0;
  config.loss_frames = 5;
  config.reset_frames = 2;
  return config;
}

TEST(LandingEstimator, FiveIdsAreOrderIndependent) {
  auto first = fiveAt(300, 200);
  auto second = first;
  std::reverse(second.begin(), second.end());

  LandingOffsetEstimator estimator_a(estimatorConfig());
  LandingOffsetEstimator estimator_b(estimatorConfig());
  const auto a = estimator_a.update(640, 480, first);
  const auto b = estimator_b.update(640, 480, second);

  ASSERT_TRUE(a.valid);
  ASSERT_TRUE(b.valid);
  EXPECT_DOUBLE_EQ(a.center_x, 300.0);
  EXPECT_DOUBLE_EQ(a.center_y, 200.0);
  EXPECT_DOUBLE_EQ(a.center_x, b.center_x);
  EXPECT_DOUBLE_EQ(a.center_y, b.center_y);
  EXPECT_EQ(a.tag_ids, (std::vector<int>{0, 1, 2, 3, 4}));
}

TEST(LandingEstimator, AnyNonEmptySubsetIsAveraged) {
  LandingOffsetEstimator single_estimator(estimatorConfig());
  const auto single = single_estimator.update(
      640, 480, {observation(2, 350, 260)});
  ASSERT_TRUE(single.valid);
  EXPECT_DOUBLE_EQ(single.center_x, 350.0);
  EXPECT_DOUBLE_EQ(single.center_y, 260.0);
  EXPECT_DOUBLE_EQ(single.dx, 30.0);
  EXPECT_DOUBLE_EQ(single.dy, 20.0);
  EXPECT_EQ(single.tag_ids, (std::vector<int>{2}));

  LandingOffsetEstimator subset_estimator(estimatorConfig());
  const auto subset = subset_estimator.update(
      640,
      480,
      {observation(0, 100, 120), observation(3, 300, 320)});
  ASSERT_TRUE(subset.valid);
  EXPECT_DOUBLE_EQ(subset.center_x, 200.0);
  EXPECT_DOUBLE_EQ(subset.center_y, 220.0);
  EXPECT_DOUBLE_EQ(subset.dx, -120.0);
  EXPECT_DOUBLE_EQ(subset.dy, -20.0);
  EXPECT_EQ(subset.tag_ids, (std::vector<int>{0, 3}));
}

TEST(LandingEstimator, CenterTagDirectlyDefinesLandingCenter) {
  LandingOffsetEstimator estimator(estimatorConfig());
  const auto result = estimator.update(
      640,
      480,
      {observation(0, 100, 100),
       observation(3, 500, 400),
       observation(4, 330, 245)});

  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.center_x, 330.0);
  EXPECT_DOUBLE_EQ(result.center_y, 245.0);
  EXPECT_DOUBLE_EQ(result.dx, 10.0);
  EXPECT_DOUBLE_EQ(result.dy, 5.0);
  EXPECT_EQ(result.reason, "center_tag");
}

TEST(LandingEstimator, ThreeCornersUseLongestDiagonalMidpoint) {
  LandingOffsetEstimator estimator(estimatorConfig());
  const auto result = estimator.update(
      640,
      480,
      {observation(0, 100, 100),
       observation(1, 300, 100),
       observation(2, 300, 300)});

  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.center_x, 200.0);
  EXPECT_DOUBLE_EQ(result.center_y, 200.0);
  EXPECT_EQ(result.reason, "three_corner_midpoint");
}

TEST(LandingEstimator, FourCornersUseProjectiveDiagonalIntersection) {
  LandingOffsetEstimator estimator(estimatorConfig());
  const auto result = estimator.update(
      640,
      480,
      {observation(0, 100, 100),
       observation(1, 320, 80),
       observation(2, 300, 300),
       observation(3, 80, 320)});

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.center_x, 200.0, 1e-9);
  EXPECT_NEAR(result.center_y, 200.0, 1e-9);
  EXPECT_EQ(result.reason, "four_corner_intersection");
}

TEST(LandingEstimator, InvalidFramesReturnNan) {
  LandingOffsetEstimator estimator(estimatorConfig());
  auto result = estimator.update(640, 480, {});
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(std::isnan(result.dx));
  EXPECT_TRUE(result.tag_ids.empty());

  auto poor = fiveAt(300, 200);
  poor[2].decision_margin = 5.0;
  result = estimator.update(640, 480, poor);
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(std::isnan(result.center_x));
  EXPECT_EQ(result.tag_ids, (std::vector<int>{0, 1, 2, 3, 4}));
}

TEST(LandingEstimator, FiveConsecutiveMissingFramesCauseLoss) {
  LandingOffsetEstimator estimator(estimatorConfig());
  const auto detected = estimator.update(640, 480, fiveAt(300, 200));
  ASSERT_TRUE(detected.valid);

  for (int missing_frame = 1; missing_frame < 5; ++missing_frame) {
    const auto held = estimator.update(640, 480, {});
    ASSERT_TRUE(held.valid) << "missing frame " << missing_frame;
    EXPECT_DOUBLE_EQ(held.center_x, 300.0);
    EXPECT_DOUBLE_EQ(held.center_y, 200.0);
    EXPECT_DOUBLE_EQ(held.dx, -20.0);
    EXPECT_DOUBLE_EQ(held.dy, -40.0);
    EXPECT_EQ(held.tag_ids, (std::vector<int>{0, 1, 2, 3, 4}));
    EXPECT_EQ(
        held.reason,
        "missing_hold_" + std::to_string(missing_frame) + "_of_5");
  }

  const auto lost = estimator.update(640, 480, {});
  EXPECT_FALSE(lost.valid);
  EXPECT_TRUE(std::isnan(lost.center_x));
  EXPECT_TRUE(std::isnan(lost.dx));
  EXPECT_EQ(lost.reason, "missing_id");

  const auto still_lost = estimator.update(640, 480, {});
  EXPECT_FALSE(still_lost.valid);
  EXPECT_TRUE(std::isnan(still_lost.dx));

  const auto reacquired = estimator.update(640, 480, fiveAt(400, 300));
  ASSERT_TRUE(reacquired.valid);
  EXPECT_DOUBLE_EQ(reacquired.center_x, 400.0);
  EXPECT_DOUBLE_EQ(reacquired.center_y, 300.0);
}

TEST(LandingEstimator, MedianAndJumpResetWork) {
  LandingOffsetEstimator estimator(estimatorConfig());
  EXPECT_TRUE(estimator.update(640, 480, fiveAt(100, 100)).valid);
  const auto filtered = estimator.update(640, 480, fiveAt(110, 110));
  ASSERT_TRUE(filtered.valid);
  EXPECT_DOUBLE_EQ(filtered.center_x, 105.0);

  const auto jump = estimator.update(640, 480, fiveAt(250, 250));
  EXPECT_FALSE(jump.valid);
  EXPECT_TRUE(std::isnan(jump.dx));
  EXPECT_FALSE(estimator.update(640, 480, {}).valid);

  const auto reset = estimator.update(640, 480, fiveAt(250, 250));
  ASSERT_TRUE(reset.valid);
  EXPECT_DOUBLE_EQ(reset.center_x, 250.0);
}

TEST(LandingFrameTransformer, IdentityAttitudeAndExtrinsicPreserveOffset) {
  const std::array<double, 9> identity{{
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0,
  }};
  LandingFrameTransformer transformer(identity);
  const auto result = transformer.cameraToLeveledImage(
      12.0, -8.0, 640.0, AttitudeQuaternion{});

  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.x, 12.0);
  EXPECT_DOUBLE_EQ(result.y, -8.0);
}

TEST(LandingFrameTransformer, RpyDegreesBuildCameraToBodyRotation) {
  const auto matrix = cameraToBodyFromRpyDegrees(180.0, 0.0, -90.0);
  const std::array<double, 9> expected{{
      0.0, -1.0, 0.0,
      -1.0, 0.0, 0.0,
      0.0, 0.0, -1.0,
  }};
  for (std::size_t index = 0; index < matrix.size(); ++index) {
    EXPECT_NEAR(matrix[index], expected[index], 1e-12);
  }
}

TEST(LandingFrameTransformer, FusedYawPreservesImageAxisOffset) {
  const std::array<double, 9> identity{{
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0,
  }};
  LandingFrameTransformer transformer(identity);
  AttitudeQuaternion yaw_ninety;
  yaw_ninety.w = std::sqrt(0.5);
  yaw_ninety.z = std::sqrt(0.5);
  const auto result = transformer.cameraToLeveledImage(
      10.0, 4.0, 640.0, yaw_ninety);

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.x, 10.0, 1e-12);
  EXPECT_NEAR(result.y, 4.0, 1e-12);
}

TEST(LandingFrameTransformer, InvalidQuaternionIsRejected) {
  const std::array<double, 9> identity{{
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0,
  }};
  LandingFrameTransformer transformer(identity);
  AttitudeQuaternion invalid;
  invalid.w = 0.0;
  const auto result = transformer.cameraToLeveledImage(
      10.0, 4.0, 640.0, invalid);

  EXPECT_FALSE(result.valid);
}

TEST(LandingFrameTransformer, RollMotionKeepsStationaryRayStable) {
  const std::array<double, 9> identity{{
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0,
  }};
  constexpr double focal_length_px = 640.0;
  constexpr double roll = 0.15;
  LandingFrameTransformer transformer(identity);
  AttitudeQuaternion rolled;
  rolled.w = std::cos(roll * 0.5);
  rolled.x = std::sin(roll * 0.5);

  // A fixed ray along world Z appears at +f*tan(roll) in the current image.
  // Rotating the complete [dx/f, dy/f, 1] ray must recover zero world error.
  const auto result = transformer.cameraToLeveledImage(
      0.0,
      focal_length_px * std::tan(roll),
      focal_length_px,
      rolled);

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.x, 0.0, 1e-9);
  EXPECT_NEAR(result.y, 0.0, 1e-9);
}

TEST(LandingFrameTransformer, ObservedDownCameraAxesCorrectRollAndPitch) {
  constexpr double focal_length_px = 640.0;
  constexpr double angle = 0.15;
  const auto down_camera = cameraToBodyFromRpyDegrees(180.0, 0.0, 90.0);
  LandingFrameTransformer transformer(down_camera);

  AttitudeQuaternion rolled;
  rolled.w = std::cos(angle * 0.5);
  rolled.x = std::sin(angle * 0.5);
  const auto roll_result = transformer.cameraToLeveledImage(
      -focal_length_px * std::tan(angle),
      0.0,
      focal_length_px,
      rolled);
  ASSERT_TRUE(roll_result.valid);
  EXPECT_NEAR(roll_result.x, 0.0, 1e-9);
  EXPECT_NEAR(roll_result.y, 0.0, 1e-9);

  AttitudeQuaternion pitched;
  pitched.w = std::cos(angle * 0.5);
  pitched.y = std::sin(angle * 0.5);
  const auto pitch_result = transformer.cameraToLeveledImage(
      0.0,
      focal_length_px * std::tan(angle),
      focal_length_px,
      pitched);
  ASSERT_TRUE(pitch_result.valid);
  EXPECT_NEAR(pitch_result.x, 0.0, 1e-9);
  EXPECT_NEAR(pitch_result.y, 0.0, 1e-9);
}

TEST(LandingDetector, SyntheticFiveTagImageIsDetected) {
  cv::Mat canvas(720, 1280, CV_8UC1, cv::Scalar(255));
  const std::vector<cv::Point> origins = {
      {120, 100}, {520, 100}, {320, 300}, {120, 500}, {520, 500}};

  apriltag_family_t* family = tag36h11_create();
  ASSERT_NE(family, nullptr);
  for (int id = 0; id < 5; ++id) {
    image_u8_t* raw = apriltag_to_image(family, id);
    ASSERT_NE(raw, nullptr);
    cv::Mat small(
        raw->height, raw->width, CV_8UC1, raw->buf, raw->stride);
    cv::Mat printable(small.size(), CV_8UC1, cv::Scalar(255));
    printable(cv::Rect(1, 1, 8, 8)).setTo(cv::Scalar(0));
    small(cv::Rect(2, 2, 6, 6))
        .copyTo(printable(cv::Rect(2, 2, 6, 6)));
    cv::Mat scaled;
    cv::resize(
        printable, scaled, cv::Size(), 12.0, 12.0, cv::INTER_NEAREST);
    scaled.copyTo(canvas(cv::Rect(
        origins[id].x, origins[id].y, scaled.cols, scaled.rows)));
    std::free(raw->buf);
    std::free(raw);
  }
  tag36h11_destroy(family);

  DetectorConfig config;
  config.tag_family = "tag36h11";
  LandingTagDetector detector(config);
  const auto detections = detector.detect(canvas);

  std::set<int> ids;
  for (const auto& detection : detections) {
    ids.insert(detection.id);
  }
  EXPECT_EQ(ids, (std::set<int>{0, 1, 2, 3, 4}));
  EXPECT_TRUE(cv::imwrite("/tmp/uav_landing_test.png", canvas));
}

TEST(LandingDebugOverlay, FormatsMissingTagAndPixelOffset) {
  const std::vector<TagObservation> observations = {
      observation(0, 100, 100),
      observation(1, 200, 100),
      observation(3, 100, 200),
      observation(4, 200, 200),
  };
  EXPECT_EQ(
      formatTagSummary({0, 1, 2, 3, 4}, observations),
      "tags: 4/5  missing: 2");

  LandingEstimate valid;
  valid.valid = true;
  valid.dx = 12.36;
  valid.dy = -8.04;
  EXPECT_EQ(
      formatPixelOffset(valid),
      "dx: +12.4 px  dy: -8.0 px");

  LandingEstimate invalid;
  EXPECT_EQ(
      formatPixelOffset(invalid),
      "dx: -- px  dy: -- px");
}

}  // namespace
}  // namespace uav_vision

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
