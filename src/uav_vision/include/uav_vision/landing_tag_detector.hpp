#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace uav_vision {

struct DetectorConfig {
  std::string tag_family = "tag36h11";
  int threads = 2;
  double decimate = 1.0;
  double blur = 0.0;
  bool refine_edges = true;
};

struct TagObservation {
  int id = -1;
  cv::Point2d center;
  std::array<cv::Point2d, 4> corners{};
  int hamming = 0;
  double decision_margin = 0.0;
};

class LandingTagDetector {
 public:
  explicit LandingTagDetector(const DetectorConfig& config);
  ~LandingTagDetector();

  LandingTagDetector(const LandingTagDetector&) = delete;
  LandingTagDetector& operator=(const LandingTagDetector&) = delete;

  std::vector<TagObservation> detect(const cv::Mat& gray);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace uav_vision
