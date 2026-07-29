#pragma once

#include <string>
#include <vector>

#include "uav_vision/landing_offset_estimator.hpp"
#include "uav_vision/landing_tag_detector.hpp"

namespace uav_vision {

std::string formatTagSummary(
    const std::vector<int>& expected_ids,
    const std::vector<TagObservation>& observations);

std::string formatPixelOffset(const LandingEstimate& estimate);

}  // namespace uav_vision
