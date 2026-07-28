#include <algorithm>
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
      observation(2, x, y),
      observation(3, x - 20, y + 20),
      observation(4, x + 20, y + 20),
  };
}

EstimatorConfig estimatorConfig() {
  EstimatorConfig config;
  config.expected_ids = {0, 1, 2, 3, 4};
  config.max_hamming = 1;
  config.min_decision_margin = 20.0;
  config.filter_window = 3;
  config.max_pixel_jump = 50.0;
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

TEST(LandingEstimator, InvalidFramesReturnNan) {
  LandingOffsetEstimator estimator(estimatorConfig());
  auto missing = fiveAt(300, 200);
  missing.pop_back();
  auto result = estimator.update(640, 480, missing);
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(std::isnan(result.dx));
  EXPECT_EQ(result.tag_ids, (std::vector<int>{0, 1, 2, 3}));

  auto poor = fiveAt(300, 200);
  poor[2].decision_margin = 5.0;
  result = estimator.update(640, 480, poor);
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(std::isnan(result.center_x));
  EXPECT_EQ(result.tag_ids, (std::vector<int>{0, 1, 2, 3, 4}));
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
