#pragma once

#include <deque>
#include <string>
#include <vector>

#include "uav_vision/landing_tag_detector.hpp"

namespace uav_vision {

struct EstimatorConfig {
  std::vector<int> expected_ids{0, 1, 2, 3, 4};
  int center_id = 4;
  int max_hamming = 1;
  double min_decision_margin = 20.0;
  int filter_window = 5;
  double max_pixel_jump = 80.0;
  int loss_frames = 5;
  int reset_frames = 3;
};

struct LandingEstimate {
  bool valid = false;
  double center_x = 0.0;
  double center_y = 0.0;
  double dx = 0.0;
  double dy = 0.0;
  std::vector<int> tag_ids;
  std::string reason;
};

class LandingOffsetEstimator {
 public:
  explicit LandingOffsetEstimator(const EstimatorConfig& config);

  LandingEstimate update(
      int image_width,
      int image_height,
      const std::vector<TagObservation>& observations);

 private:
  LandingEstimate invalid(
      const std::vector<int>& observed_ids, const std::string& reason);
  LandingEstimate missing(int image_width, int image_height);
  static double median(const std::deque<double>& values);
  void clearFilter();

  EstimatorConfig config_;
  std::deque<double> x_values_;
  std::deque<double> y_values_;
  bool has_filtered_center_ = false;
  double filtered_x_ = 0.0;
  double filtered_y_ = 0.0;
  std::vector<int> last_valid_tag_ids_;
  int missing_frames_ = 0;
  int invalid_frames_ = 0;
};

}  // namespace uav_vision
