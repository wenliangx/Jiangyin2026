#ifndef FSM_CTRL_SINGLE_OFFBOARD_SML_TYPES_CANONICAL_HPP_
#define FSM_CTRL_SINGLE_OFFBOARD_SML_TYPES_CANONICAL_HPP_

#include <string>
#include <vector>

namespace fsm_ctrl {
namespace single_sml {

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

struct Config {
  double service_retry_seconds{5.0};
  double low_thrust{0.02};
  double hover_thrust{0.196};
  double position_hold_z{0.4};
  double landing_target_z{0.005};
  double landing_reference_z{0.05};
  double landing_tolerance_z{0.05};
};

}  // namespace single_sml
}  // namespace fsm_ctrl

#endif  // FSM_CTRL_SINGLE_OFFBOARD_SML_TYPES_CANONICAL_HPP_
