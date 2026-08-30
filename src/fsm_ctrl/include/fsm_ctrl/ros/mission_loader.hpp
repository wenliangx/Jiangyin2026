#pragma once

#include <fsm_ctrl/flight_fsm/types.hpp>
#include <string>
#include <vector>

namespace fsm_ctrl {
namespace ros_adapter {

struct MissionPoint {
  int id{0};
  int mode{0};
  int is_map{0};
  Vec3 position;
  double yaw{0.0};
  double desired_speed{0.0};
  int perc_mode{0};
};

std::vector<std::vector<MissionPoint>> loadMissionSegments(const std::string& path);

}  // namespace ros_adapter
}  // namespace fsm_ctrl
