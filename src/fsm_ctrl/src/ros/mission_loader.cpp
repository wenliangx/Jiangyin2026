#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fsm_ctrl/ros/mission_loader.hpp>
#include <limits>
#include <stdexcept>

namespace fsm_ctrl {
namespace ros_adapter {
namespace {

double optionalWaypointYaw(const YAML::Node& node) {
  if (!node || node.IsNull()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  try {
    return node.as<double>();
  } catch (const YAML::Exception&) {
    std::string value = node.as<std::string>();
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
      return static_cast<char>(std::tolower(character));
    });
    if (value == "nan" || value == ".nan" || value == "free" || value == "auto") {
      return std::numeric_limits<double>::quiet_NaN();
    }
    throw;
  }
}

MissionPoint parseMissionPoint(const YAML::Node& node, std::size_t fallback_id) {
  MissionPoint point;
  point.id = node["id"] ? node["id"].as<int>() : static_cast<int>(fallback_id);
  point.mode = node["mode"] ? node["mode"].as<int>() : 2;
  point.is_map = node["is_map"] ? node["is_map"].as<int>() : 1;
  point.position.x = node["x"].as<double>();
  point.position.y = node["y"].as<double>();
  point.position.z = node["z"].as<double>();
  point.yaw = optionalWaypointYaw(node["yaw"]);
  point.desired_speed = node["desired_speed"] ? node["desired_speed"].as<double>() : 0.0;

  if (point.mode < 0 || point.mode > 2) {
    throw std::runtime_error("waypoint mode must be 0, 1, or 2");
  }
  if (point.is_map != 0 && point.is_map != 1) {
    throw std::runtime_error("waypoint is_map must be 0 or 1");
  }
  if (!std::isfinite(point.position.x) || !std::isfinite(point.position.y) ||
      !std::isfinite(point.position.z)) {
    throw std::runtime_error("waypoint x/y/z must be finite");
  }
  if (!std::isfinite(point.desired_speed) || point.desired_speed < 0.0) {
    throw std::runtime_error("waypoint desired_speed must be finite and non-negative");
  }
  return point;
}

std::vector<MissionPoint> loadMissionPoints(const std::string& path) {
  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node waypoint_nodes = root["waypoints"];
  if (!waypoint_nodes || !waypoint_nodes.IsSequence() || waypoint_nodes.size() == 0) {
    throw std::runtime_error("waypoint file must contain non-empty waypoints");
  }

  std::vector<MissionPoint> waypoints;
  waypoints.reserve(waypoint_nodes.size());
  for (std::size_t index = 0; index < waypoint_nodes.size(); ++index) {
    waypoints.push_back(parseMissionPoint(waypoint_nodes[index], index));
  }
  return waypoints;
}

}  // namespace

std::vector<std::vector<MissionPoint>> loadMissionSegments(const std::string& path) {
  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node segment_nodes = root["segments"];
  if (!segment_nodes || !segment_nodes.IsSequence()) {
    return std::vector<std::vector<MissionPoint>>{loadMissionPoints(path)};
  }

  std::vector<std::vector<MissionPoint>> segments;
  segments.reserve(segment_nodes.size());
  for (std::size_t index = 0; index < segment_nodes.size(); ++index) {
    const YAML::Node waypoints = segment_nodes[index]["waypoints"];
    if (!waypoints || !waypoints.IsSequence() || waypoints.size() == 0) {
      throw std::runtime_error("each segment must contain non-empty waypoints");
    }

    std::vector<MissionPoint> segment;
    segment.reserve(waypoints.size());
    for (std::size_t waypoint_index = 0; waypoint_index < waypoints.size(); ++waypoint_index) {
      segment.push_back(parseMissionPoint(waypoints[waypoint_index], waypoint_index));
    }
    segments.push_back(segment);
  }
  return segments;
}

}  // namespace ros_adapter
}  // namespace fsm_ctrl
