#include "uav_vision/landing_debug_overlay.hpp"

#include <cmath>
#include <iomanip>
#include <set>
#include <sstream>

namespace uav_vision {

std::string formatTagSummary(
    const std::vector<int>& expected_ids,
    const std::vector<TagObservation>& observations) {
  const std::set<int> expected(expected_ids.begin(), expected_ids.end());
  std::set<int> observed;
  for (const auto& observation : observations) {
    if (expected.count(observation.id) != 0) {
      observed.insert(observation.id);
    }
  }

  std::ostringstream output;
  output << "tags: " << observed.size() << "/" << expected.size();
  bool first_missing = true;
  for (const int id : expected) {
    if (observed.count(id) != 0) {
      continue;
    }
    output << (first_missing ? "  missing: " : ",");
    output << id;
    first_missing = false;
  }
  return output.str();
}

std::string formatPixelOffset(const LandingEstimate& estimate) {
  if (!estimate.valid || !std::isfinite(estimate.dx) ||
      !std::isfinite(estimate.dy)) {
    return "dx: -- px  dy: -- px";
  }

  std::ostringstream output;
  output << std::fixed << std::setprecision(1) << std::showpos
         << "dx: " << estimate.dx << " px  dy: " << estimate.dy << " px";
  return output.str();
}

}  // namespace uav_vision
