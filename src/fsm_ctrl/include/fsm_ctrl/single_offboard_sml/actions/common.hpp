#ifndef FSM_CTRL_SINGLE_OFFBOARD_SML_ACTIONS_COMMON_HPP_
#define FSM_CTRL_SINGLE_OFFBOARD_SML_ACTIONS_COMMON_HPP_

#include <fsm_ctrl/single_offboard_sml/context.hpp>
#include <fsm_ctrl/single_offboard_sml/states.hpp>

#include <cmath>
#include <vector>

namespace fsm_ctrl {
namespace single_sml {

struct Noop {
  void operator()() const {}
};

inline bool publishTrackCommand(Context& context,
                                const std::vector<ReferencePoint>& horizon) {
  BodyRateThrust command;
  if (horizon.empty() ||
      !context.nmpc.solveTrack(context.telemetry, horizon, command) ||
      !Context::finite(command)) {
    return false;
  }
  context.setpoint.publishBodyRateThrust(command);
  NmpcMonitor monitor;
  monitor.references = horizon;
  monitor.feedback = context.telemetry;
  monitor.target = command;
  context.setpoint.publishNmpcMonitor(monitor);
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
    publishTrackCommand(context, fixedPositionHorizon(target));
  }
};

struct TickCoreLanding {
  void operator()(Context& context) const {
    if (!context.landing_reached) {
      context.ensureOffboardArm();
      std::vector<ReferencePoint> horizon;
      if (context.landing.prepareLanding(context.clock.now(),
                                         context.telemetry, horizon) &&
          !horizon.empty()) {
        context.setpoint.publishFeedbackPosition(context.telemetry.position,
                                                 context.telemetry.attitude);
        context.setpoint.publishReferencePosition(horizon.front().position,
                                                  horizon.front().attitude);
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

struct ResetLanding {
  void operator()(Context& context) const {
    context.landing_reached = false;
    context.landing.reset();
  }
};

struct TickEmergency {
  void operator()(Context& context) const {
    context.setpoint.publishAttitude(
        AttitudeSetpoint{Quaternion{}, context.config.hover_thrust - 0.03});
  }
};

}  // namespace single_sml
}  // namespace fsm_ctrl

#endif  // FSM_CTRL_SINGLE_OFFBOARD_SML_ACTIONS_COMMON_HPP_
