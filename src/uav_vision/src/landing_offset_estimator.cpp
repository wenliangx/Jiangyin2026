#include "uav_vision/landing_offset_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>

namespace uav_vision {

LandingOffsetEstimator::LandingOffsetEstimator(const EstimatorConfig& config)
    : config_(config) {
  const std::set<int> unique_ids(
      config_.expected_ids.begin(), config_.expected_ids.end());
  if (config_.expected_ids.size() != 5 || unique_ids.size() != 5 ||
      *unique_ids.begin() < 0) {
    throw std::invalid_argument(
        "expected_ids must contain five distinct non-negative IDs");
  }
  if (config_.max_hamming < 0 || config_.min_decision_margin < 0.0 ||
      config_.filter_window <= 0 || config_.filter_window % 2 == 0 ||
      config_.max_pixel_jump < 0.0 || config_.reset_frames <= 0) {
    throw std::invalid_argument("invalid landing estimator configuration");
  }
}

LandingEstimate LandingOffsetEstimator::update(
    int image_width,
    int image_height,
    const std::vector<TagObservation>& observations) {
  const std::set<int> expected(
      config_.expected_ids.begin(), config_.expected_ids.end());
  std::map<int, std::vector<const TagObservation*>> targets;
  for (const auto& observation : observations) {
    if (expected.count(observation.id) != 0) {
      targets[observation.id].push_back(&observation);
    }
  }

  std::vector<int> observed_ids;
  for (const auto& item : targets) {
    observed_ids.push_back(item.first);
  }

  if (image_width <= 0 || image_height <= 0) {
    return invalid(observed_ids, "invalid_image_size");
  }
  for (const auto& item : targets) {
    if (item.second.size() != 1) {
      return invalid(observed_ids, "duplicate_id");
    }
  }
  if (targets.empty()) {
    return invalid(observed_ids, "missing_id");
  }

  double raw_x = 0.0;
  double raw_y = 0.0;
  for (const auto& item : targets) {
    const TagObservation& observation = *item.second.front();
    if (observation.hamming > config_.max_hamming) {
      return invalid(observed_ids, "hamming");
    }
    if (observation.decision_margin < config_.min_decision_margin) {
      return invalid(observed_ids, "decision_margin");
    }
    raw_x += observation.center.x;
    raw_y += observation.center.y;
  }
  raw_x /= targets.size();
  raw_y /= targets.size();

  if (has_filtered_center_ &&
      std::hypot(raw_x - filtered_x_, raw_y - filtered_y_) >
          config_.max_pixel_jump) {
    return invalid(observed_ids, "pixel_jump");
  }

  x_values_.push_back(raw_x);
  y_values_.push_back(raw_y);
  while (static_cast<int>(x_values_.size()) > config_.filter_window) {
    x_values_.pop_front();
    y_values_.pop_front();
  }

  filtered_x_ = median(x_values_);
  filtered_y_ = median(y_values_);
  has_filtered_center_ = true;
  invalid_frames_ = 0;

  LandingEstimate result;
  result.valid = true;
  result.center_x = filtered_x_;
  result.center_y = filtered_y_;
  result.dx = filtered_x_ - image_width / 2.0;
  result.dy = filtered_y_ - image_height / 2.0;
  result.tag_ids = observed_ids;
  result.reason = "accepted";
  return result;
}

LandingEstimate LandingOffsetEstimator::invalid(
    const std::vector<int>& observed_ids, const std::string& reason) {
  ++invalid_frames_;
  if (invalid_frames_ >= config_.reset_frames) {
    clearFilter();
  }

  const double nan = std::numeric_limits<double>::quiet_NaN();
  LandingEstimate result;
  result.valid = false;
  result.center_x = nan;
  result.center_y = nan;
  result.dx = nan;
  result.dy = nan;
  result.tag_ids = observed_ids;
  result.reason = reason;
  return result;
}

double LandingOffsetEstimator::median(const std::deque<double>& values) {
  std::vector<double> sorted(values.begin(), values.end());
  std::sort(sorted.begin(), sorted.end());
  const std::size_t middle = sorted.size() / 2;
  if (sorted.size() % 2 == 1) {
    return sorted[middle];
  }
  return (sorted[middle - 1] + sorted[middle]) / 2.0;
}

void LandingOffsetEstimator::clearFilter() {
  x_values_.clear();
  y_values_.clear();
  has_filtered_center_ = false;
  invalid_frames_ = 0;
}

}  // namespace uav_vision
