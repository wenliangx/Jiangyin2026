#ifndef FSM_CTRL_SINGLE_OFFBOARD_SML_ACTIONS_HPP_
#define FSM_CTRL_SINGLE_OFFBOARD_SML_ACTIONS_HPP_

#include <fsm_ctrl/single_offboard_sml_context.hpp>
#include <fsm_ctrl/single_offboard_sml_states.hpp>

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

inline bool publishTrackDebugOnly(Context& context,
                                  const std::vector<ReferencePoint>& horizon) {
  BodyRateThrust command;
  if (horizon.empty() ||
      !context.nmpc.solveTrack(context.telemetry, horizon, command) ||
      !Context::finite(command)) {
    return false;
  }
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

struct TickLowThrust {
  void operator()(Context& context) const {
    context.ensureOffboardArm();
    context.setpoint.publishBodyRateThrust(
        BodyRateThrust{Vec3{}, context.config.low_thrust});
  }
};

struct TickArmOnly {
  void operator()(Context& context) const { context.ensureArm(); }
};

struct TickCoreHoverToOneMeter {
  void operator()(Context& context) const {
    context.ensureOffboardArm();
    const Vec3 target{context.telemetry.position.x, context.telemetry.position.y,
                      1.0};
    publishTrackCommand(context, fixedPositionHorizon(target));
  }
};

struct TickPositionHold {
  void operator()(Context& context) const {
    context.ensureOffboardArm();
    context.setpoint.publishPosition(
        PositionSetpoint{Vec3{0.0, 0.0, context.config.position_hold_z}, 0.0});
  }
};

struct TickNmpcHover {
  void operator()(Context& context) const {
    context.ensureOffboardArm();
    BodyRateThrust command;
    if (context.nmpc.solveHover(context.telemetry, command) &&
        Context::finite(command)) {
      context.setpoint.publishBodyRateThrust(command);
    }
  }
};

struct TickCoreSuperLandingDebug {
  void operator()(Context& context) const {
    context.ensureOffboardArm();
    const Vec3 goal{1.0, 0.0, 1.0};
    std::vector<ReferencePoint> super_horizon;
    if (context.mission.prepareCoreSuperGoal(context.clock.now(),
                                             context.telemetry, goal,
                                             super_horizon) &&
        !super_horizon.empty()) {
      context.setpoint.publishFeedbackPosition(context.telemetry.position,
                                               context.telemetry.attitude);
      context.setpoint.publishReferencePosition(super_horizon.front().position,
                                                super_horizon.front().attitude);
      publishTrackCommand(context, super_horizon);
    }

    std::vector<ReferencePoint> landing_horizon;
    if (context.landing.prepareLanding(context.clock.now(), context.telemetry,
                                       landing_horizon) &&
        !landing_horizon.empty()) {
      context.setpoint.publishFeedbackPosition(context.telemetry.position,
                                               context.telemetry.attitude);
      context.setpoint.publishReferencePosition(landing_horizon.front().position,
                                                landing_horizon.front().attitude);
      publishTrackDebugOnly(context, landing_horizon);
    }
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

struct ResetNmpcTrack {
  void operator()(Context& context) const { context.reference.reset(); }
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

struct TickNmpcTrack {
  void operator()(Context& context) const {
    context.ensureOffboardArm();
    std::vector<ReferencePoint> horizon;
    if (context.reference.horizon(context.clock.now(), horizon)) {
      publishTrackCommand(context, horizon);
    }
  }
};

struct TickSuperTrack {
  void operator()(Context& context) const {
    context.ensureOffboardArm();
    std::vector<ReferencePoint> horizon;
    const bool prepared =
        context.mission.prepareSuper(context.clock.now(), context.telemetry,
                                     horizon);
    if (prepared && !horizon.empty()) {
      context.setpoint.publishFeedbackPosition(context.telemetry.position,
                                               context.telemetry.attitude);
      context.setpoint.publishReferencePosition(horizon.front().position,
                                                horizon.front().attitude);
      publishTrackCommand(context, horizon);
    }
  }
};

inline void tickSuperSegment(Context& context, int segment_index) {
  context.ensureOffboardArm();
  std::vector<ReferencePoint> horizon;
  if (context.mission.prepareSuperSegment(segment_index, context.clock.now(),
                                          context.telemetry, horizon) &&
      !horizon.empty()) {
    context.setpoint.publishFeedbackPosition(context.telemetry.position,
                                             context.telemetry.attitude);
    context.setpoint.publishReferencePosition(horizon.front().position,
                                              horizon.front().attitude);
    publishTrackCommand(context, horizon);
  }
}

struct TickSuperSegment1 {
  void operator()(Context& context) const { tickSuperSegment(context, 0); }
};

struct TickSuperSegment2 {
  void operator()(Context& context) const { tickSuperSegment(context, 1); }
};

struct TickSuperSegment3 {
  void operator()(Context& context) const { tickSuperSegment(context, 2); }
};

struct TickEmergency {
  void operator()(Context& context) const {
    context.setpoint.publishAttitude(
        AttitudeSetpoint{Quaternion{}, context.config.hover_thrust - 0.03});
  }
};

}  // namespace single_sml
}  // namespace fsm_ctrl

#endif  // FSM_CTRL_SINGLE_OFFBOARD_SML_ACTIONS_HPP_
