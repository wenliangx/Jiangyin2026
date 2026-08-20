#include "uav_vision/landing_offset_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace uav_vision {
namespace {

double cross2d(const cv::Point2d& first, const cv::Point2d& second) {
  return first.x * second.y - first.y * second.x;
}

bool diagonalIntersection(
    const std::vector<cv::Point2d>& corners, cv::Point2d* center) {
  if (corners.size() != 4 || center == nullptr) {
    return false;
  }

  std::vector<cv::Point2f> float_corners;
  float_corners.reserve(corners.size());
  for (const auto& corner : corners) {
    float_corners.emplace_back(
        static_cast<float>(corner.x), static_cast<float>(corner.y));
  }
  std::vector<int> hull;
  cv::convexHull(float_corners, hull, false, false);
  if (hull.size() != 4) {
    return false;
  }

  const cv::Point2d first_start = corners[hull[0]];
  const cv::Point2d first_direction = corners[hull[2]] - first_start;
  const cv::Point2d second_start = corners[hull[1]];
  const cv::Point2d second_direction = corners[hull[3]] - second_start;
  const double denominator = cross2d(first_direction, second_direction);
  if (std::abs(denominator) < 1e-9) {
    return false;
  }

  const double distance =
      cross2d(second_start - first_start, second_direction) / denominator;
  *center = first_start + distance * first_direction;
  return std::isfinite(center->x) && std::isfinite(center->y);
}

cv::Point2d longestPairMidpoint(
    const std::vector<cv::Point2d>& points) {
  double longest_squared = -1.0;
  cv::Point2d midpoint;
  for (std::size_t first = 0; first < points.size(); ++first) {
    for (std::size_t second = first + 1; second < points.size(); ++second) {
      const cv::Point2d difference = points[first] - points[second];
      const double squared = difference.dot(difference);
      if (squared > longest_squared) {
        longest_squared = squared;
        midpoint = 0.5 * (points[first] + points[second]);
      }
    }
  }
  return midpoint;
}

}  // namespace

LandingOffsetEstimator::LandingOffsetEstimator(const EstimatorConfig& config)
    : config_(config) {
  const std::set<int> unique_ids(
      config_.expected_ids.begin(), config_.expected_ids.end());
  if (config_.expected_ids.size() != 5 || unique_ids.size() != 5 ||
      *unique_ids.begin() < 0) {
    throw std::invalid_argument(
        "expected_ids must contain five distinct non-negative IDs");
  }
  if (unique_ids.count(config_.center_id) == 0) {
    throw std::invalid_argument("center_id must be one of expected_ids");
  }
  if (config_.max_hamming < 0 || config_.min_decision_margin < 0.0 ||
      config_.filter_window <= 0 || config_.filter_window % 2 == 0 ||
      config_.max_pixel_jump < 0.0 || config_.loss_frames <= 0 ||
      config_.reset_frames <= 0) {
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
    return missing(image_width, image_height);
  }

  cv::Point2d visible_sum;
  std::vector<cv::Point2d> corner_centers;
  for (const auto& item : targets) {
    const TagObservation& observation = *item.second.front();
    if (observation.hamming > config_.max_hamming) {
      return invalid(observed_ids, "hamming");
    }
    if (observation.decision_margin < config_.min_decision_margin) {
      return invalid(observed_ids, "decision_margin");
    }
    visible_sum += observation.center;
    if (item.first != config_.center_id) {
      corner_centers.push_back(observation.center);
    }
  }

  cv::Point2d raw_center;
  std::string accepted_reason;
  const auto center_tag = targets.find(config_.center_id);
  if (center_tag != targets.end()) {
    raw_center = center_tag->second.front()->center;
    accepted_reason = "center_tag";
  } else if (diagonalIntersection(corner_centers, &raw_center)) {
    accepted_reason = "four_corner_intersection";
  } else if (corner_centers.size() >= 3) {
    raw_center = longestPairMidpoint(corner_centers);
    accepted_reason = "three_corner_midpoint";
  } else {
    raw_center = visible_sum * (1.0 / targets.size());
    accepted_reason = "visible_mean";
  }

  const double raw_x = raw_center.x;
  const double raw_y = raw_center.y;

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
  last_valid_tag_ids_ = observed_ids;
  missing_frames_ = 0;
  invalid_frames_ = 0;

  LandingEstimate result;
  result.valid = true;
  result.center_x = filtered_x_;
  result.center_y = filtered_y_;
  result.dx = filtered_x_ - image_width / 2.0;
  result.dy = filtered_y_ - image_height / 2.0;
  result.tag_ids = observed_ids;
  result.reason = accepted_reason;
  return result;
}

LandingEstimate LandingOffsetEstimator::invalid(
    const std::vector<int>& observed_ids, const std::string& reason) {
  missing_frames_ = 0;
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

LandingEstimate LandingOffsetEstimator::missing(
    int image_width, int image_height) {
  ++missing_frames_;
  if (has_filtered_center_ && invalid_frames_ == 0 &&
      missing_frames_ < config_.loss_frames) {
    LandingEstimate result;
    result.valid = true;
    result.center_x = filtered_x_;
    result.center_y = filtered_y_;
    result.dx = filtered_x_ - image_width / 2.0;
    result.dy = filtered_y_ - image_height / 2.0;
    result.tag_ids = last_valid_tag_ids_;
    result.reason =
        "missing_hold_" + std::to_string(missing_frames_) + "_of_" +
        std::to_string(config_.loss_frames);
    return result;
  }

  if (has_filtered_center_) {
    clearFilter();
  }
  return invalid({}, "missing_id");
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
  last_valid_tag_ids_.clear();
  missing_frames_ = 0;
  invalid_frames_ = 0;
}

}  // namespace uav_vision
