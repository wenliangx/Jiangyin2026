#ifndef FSM_CTRL_SINGLE_OFFBOARD_SML_CONTEXT_HPP_
#define FSM_CTRL_SINGLE_OFFBOARD_SML_CONTEXT_HPP_

#include <fsm_ctrl/single_offboard_sml_ports.hpp>

#include <cmath>

namespace fsm_ctrl {
namespace single_sml {

struct Context {
  Context(Clock& clock_in, AutopilotPort& autopilot_in,
          SetpointPort& setpoint_in, NmpcPort& nmpc_in,
          ReferenceProvider& reference_in, MissionPort& mission_in,
          PrecisionLandingPort& landing_in,
          const Config& config_in = Config{})
      : clock(clock_in),
        autopilot(autopilot_in),
        setpoint(setpoint_in),
        nmpc(nmpc_in),
        reference(reference_in),
        mission(mission_in),
        landing(landing_in),
        config(config_in),
        last_service_request(clock_in.now()) {}

  void ensureOffboardArm() {
    const double current_time = clock.now();
    if (telemetry.mode != "OFFBOARD" &&
        current_time - last_service_request > config.service_retry_seconds) {
      autopilot.requestOffboard();
      last_service_request = current_time;
    } else if (!telemetry.armed &&
               current_time - last_service_request >
                   config.service_retry_seconds) {
      autopilot.requestArm();
      last_service_request = current_time;
    }
  }

  static bool finite(const BodyRateThrust& command) {
    return std::isfinite(command.body_rate.x) &&
           std::isfinite(command.body_rate.y) &&
           std::isfinite(command.body_rate.z) &&
           std::isfinite(command.thrust);
  }

  Clock& clock;
  AutopilotPort& autopilot;
  SetpointPort& setpoint;
  NmpcPort& nmpc;
  ReferenceProvider& reference;
  MissionPort& mission;
  PrecisionLandingPort& landing;
  Config config;
  TelemetrySnapshot telemetry;
  double last_service_request;
  bool landing_reached{false};
};

}  // namespace single_sml
}  // namespace fsm_ctrl

#endif  // FSM_CTRL_SINGLE_OFFBOARD_SML_CONTEXT_HPP_
