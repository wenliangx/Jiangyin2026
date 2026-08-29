#ifndef FSM_CTRL_FLIGHT_FSM_CONTEXT_HPP_
#define FSM_CTRL_FLIGHT_FSM_CONTEXT_HPP_

#include <fsm_ctrl/flight_fsm/ports.hpp>

#include <array>
#include <cmath>
#include <string>

namespace fsm_ctrl {
namespace flight_fsm {

struct Context {
  Context(Clock& clock_in, AutopilotPort& autopilot_in,
          SetpointPort& setpoint_in, NmpcPort& nmpc_in,
          MissionPort& mission_in, PrecisionLandingPort& landing_in,
          CameraControlPort& camera_control_in,
          const Config& config_in = Config{})
      : clock(clock_in),
        autopilot(autopilot_in),
        setpoint(setpoint_in),
        nmpc(nmpc_in),
        mission(mission_in),
        landing(landing_in),
        camera_control(camera_control_in),
        config(config_in),
        last_service_request(clock_in.now()) {}

  void ensureOffboardArm() {
    if (permanent_landing_lock) {
      return;
    }
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

  void ensureArm() {
    if (permanent_landing_lock) {
      return;
    }
    const double current_time = clock.now();
    if (!telemetry.armed &&
        current_time - last_service_request > config.service_retry_seconds) {
      autopilot.requestArm();
      last_service_request = current_time;
    }
  }

  void ensureDisarm() {
    if (!telemetry.armed) {
      return;
    }
    const double current_time = clock.now();
    if (!disarm_request_started ||
        current_time - last_service_request > config.service_retry_seconds) {
      autopilot.requestDisarm();
      last_service_request = current_time;
      disarm_request_started = true;
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
  MissionPort& mission;
  PrecisionLandingPort& landing;
  CameraControlPort& camera_control;
  Config config;
  TelemetrySnapshot telemetry;
  double last_service_request;
  bool landing_reached{false};
  bool disarm_request_started{false};
  // 首次判定落地后永久置位；进程生命周期内任何动作都不得清除此锁。
  bool permanent_landing_lock{false};
  // 三段任务之间的两个识别卡槽：slot 0 触发 1->2，slot 1 触发 2->3。
  std::array<std::string, 2> recognized_targets;
};

}  // namespace flight_fsm
}  // namespace fsm_ctrl

#endif  // FSM_CTRL_FLIGHT_FSM_CONTEXT_HPP_
