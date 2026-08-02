#ifndef FSM_CTRL_SINGLE_OFFBOARD_SML_ACTIONS_COMMON_HPP_
#define FSM_CTRL_SINGLE_OFFBOARD_SML_ACTIONS_COMMON_HPP_

#include <fsm_ctrl/single_offboard_sml/context.hpp>
#include <fsm_ctrl/single_offboard_sml/states.hpp>

#include <cmath>
#include <vector>

namespace fsm_ctrl {
namespace single_sml {

struct HorizonLogFields {
  LogSeverity severity{LogSeverity::Info};
  LogEvent event{LogEvent::CommandNew};
  const std::vector<ReferencePoint>& horizon;
  int segment_index{-1};
};

struct TrackLogFields {
  LogEvent event{LogEvent::CommandNew};
  const std::vector<ReferencePoint>& horizon;
  const BodyRateThrust& command;
};

inline void writeLog(Context& context, LogSeverity severity, LogEvent event) {
  LogRecord record;
  record.severity = severity;
  record.event = event;
  record.stamp = context.clock.now();
  record.position = context.telemetry.position;
  context.log.write(record);
}

inline void writeHorizonLog(Context& context, const HorizonLogFields& fields) {
  LogRecord record;
  record.severity = fields.severity;
  record.event = fields.event;
  record.stamp = context.clock.now();
  record.position = context.telemetry.position;
  record.horizon_size = fields.horizon.size();
  record.segment_index = fields.segment_index;
  if (!fields.horizon.empty()) {
    record.reference_position = fields.horizon.front().position;
  }
  context.log.write(record);
}

inline void writeNmpcLog(Context& context, const TrackLogFields& fields) {
  LogRecord record;
  record.severity = fields.event == LogEvent::NmpcPublishSuccess
                        ? LogSeverity::Debug
                        : LogSeverity::Error;
  record.event = fields.event;
  record.stamp = context.clock.now();
  record.position = context.telemetry.position;
  record.horizon_size = fields.horizon.size();
  if (!fields.horizon.empty()) {
    record.reference_position = fields.horizon.front().position;
  }
  record.command_output = fields.command;
  context.log.write(record);
}

struct Noop {
  void operator()() const {}
};

inline bool publishTrackCommand(Context& context,
                                const std::vector<ReferencePoint>& horizon) {
  BodyRateThrust command;
  if (horizon.empty()) {
    writeHorizonLog(
        context, HorizonLogFields{LogSeverity::Warn, LogEvent::EmptyHorizon,
                                  horizon});
    return false;
  }
  if (!context.nmpc.solveTrack(context.telemetry, horizon, command)) {
    writeNmpcLog(context,
                 TrackLogFields{LogEvent::NmpcSolveFailure, horizon, command});
    return false;
  }
  if (!Context::finite(command)) {
    writeNmpcLog(
        context, TrackLogFields{LogEvent::NmpcNonFiniteOutput, horizon,
                                command});
    return false;
  }
  context.setpoint.publishBodyRateThrust(command);
  NmpcMonitor monitor;
  monitor.references = horizon;
  monitor.feedback = context.telemetry;
  monitor.target = command;
  context.setpoint.publishNmpcMonitor(monitor);
  if (context.clock.now() - context.last_success_log_time >= 1.0) {
    writeNmpcLog(context,
                 TrackLogFields{LogEvent::NmpcPublishSuccess, horizon,
                                command});
    context.last_success_log_time = context.clock.now();
  }
  return true;
}

inline std::vector<ReferencePoint> fixedPositionHorizon(const Vec3& position) {
  std::vector<ReferencePoint> horizon(10);
  for (auto& point : horizon) {
    point.position = position;
  }
  return horizon;
}

struct TickArmOnly {
  void operator()(Context& context) const {
    writeLog(context, LogSeverity::Debug, LogEvent::ActionArmOnly);
    context.setpoint.publishBodyRateThrust(
        BodyRateThrust{Vec3{}, context.config.low_thrust});
    context.ensureOffboardArm();
  }
};

struct TickCoreHoverToOneMeter {
  void operator()(Context& context) const {
    context.ensureOffboardArm();
    const Vec3 target{context.telemetry.position.x, context.telemetry.position.y,
                      1.0};
    const std::vector<ReferencePoint> horizon = fixedPositionHorizon(target);
    writeHorizonLog(
        context, HorizonLogFields{LogSeverity::Debug,
                                  LogEvent::ActionHoverToOneMeter, horizon});
    publishTrackCommand(context, horizon);
  }
};

struct TickLanding {
  void operator()(Context& context) const {
    if (!context.landing_reached) {
      context.ensureOffboardArm();
      std::vector<ReferencePoint> horizon;
      if (context.landing.prepareLanding(context.clock.now(),
                                         context.telemetry, horizon)) {
        publishTrackCommand(context, horizon);
      }
      if (context.landing.isComplete() ||
          std::abs(context.telemetry.position.z -
                   context.config.landing_reference_z) <
              context.config.landing_tolerance_z) {
        context.landing_reached = true;
      }
      return;
    }

    if (context.telemetry.mode != "OFFBOARD" && context.telemetry.armed) {
      context.autopilot.requestDisarm();
    }
  }
};

struct ResetSuperTrack {
  void operator()(Context& context) const {
    context.mission.reset();
  }
};

// SML 转换 guard：任务航点不可用（YAML 加载失败）时禁止进入 super 状态。
struct MissionAvailable {
  bool operator()(const Context& context) const {
    return context.mission.available();
  }
};

struct ResetLanding {
  void operator()(Context& context) const {
    context.landing_reached = false;
    context.landing.reset();
  }
};

struct TickEmergency {
  void operator()(Context& context) const {
    const AttitudeSetpoint setpoint{Quaternion{},
                                    context.config.hover_thrust - 0.03};
    LogRecord record;
    record.severity = LogSeverity::Warn;
    record.event = LogEvent::ActionEmergency;
    record.stamp = context.clock.now();
    record.position = context.telemetry.position;
    record.command_output.thrust = setpoint.thrust;
    context.log.write(record);
    context.setpoint.publishAttitude(setpoint);
  }
};

}  // namespace single_sml
}  // namespace fsm_ctrl

#endif  // FSM_CTRL_SINGLE_OFFBOARD_SML_ACTIONS_COMMON_HPP_
