#pragma once

#include <string>
#include <vector>

namespace fsm_ctrl {

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Quaternion {
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct TelemetrySnapshot {
  std::string mode;
  bool armed{false};
  Vec3 position;
  Vec3 velocity;
  Quaternion attitude;
};

struct ReferencePoint {
  Vec3 position;
  Vec3 velocity;
  Quaternion attitude;
};

struct BodyRateThrust {
  Vec3 body_rate;
  double thrust{0.0};
};

struct PositionSetpoint {
  Vec3 position;
  double yaw{0.0};
};

struct AttitudeSetpoint {
  Quaternion attitude;
  double thrust{0.0};
};

struct NmpcMonitor {
  std::vector<ReferencePoint> references;
  TelemetrySnapshot feedback;
  BodyRateThrust target;
};

struct LandingObservation {
  bool valid{false};
  double dx{0.0};
  double dy{0.0};
  int tag_count{0};
  double stamp{0.0};
  double age{0.0};
};

// 视觉相机的期望运行状态。它是状态快照而不是一次性的启停脉冲，
// 因而 ROS 适配层可以安全地使用 latched topic 交付给晚启动的节点。
struct CameraControlState {
  bool front_camera_enabled{false};
  bool down_camera_enabled{false};
};

struct Config {
  double service_retry_seconds{5.0};
  double low_thrust{0.02};
  double hover_thrust{0.196};
  double position_hold_z{0.4};
  double landing_target_z{0.00};
  double landing_reference_z{-0.15};
  double landing_tolerance_z{0.25};
};

}  // namespace fsm_ctrl
