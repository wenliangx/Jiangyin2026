#ifndef FSM_CTRL_SINGLE_OFFBOARD_SML_ACTIONS_HPP_
#define FSM_CTRL_SINGLE_OFFBOARD_SML_ACTIONS_HPP_

#include <fsm_ctrl/single_offboard_sml_context.hpp>
#include <fsm_ctrl/single_offboard_sml_landing.hpp>
#include <fsm_ctrl/single_offboard_sml_states.hpp>

#include <cmath>
#include <vector>
#include <iostream>
namespace fsm_ctrl {
namespace single_sml {

inline void publishCameraControl(Context& context, bool front_enabled,
                                 bool down_enabled) {
  context.camera_control.publishControl(
      CameraControlState{front_enabled, down_enabled});
}

struct DisableCameras {
  void operator()(Context& context) const {
    publishCameraControl(context, false, false);
  }
};

struct EnableFrontCamera {
  void operator()(Context& context) const {
    publishCameraControl(context, true, false);
  }
};

struct EnableDownCamera {
  void operator()(Context& context) const {
    publishCameraControl(context, false, true);
  }
};

struct EnableBothCameras {
  void operator()(Context& context) const {
    publishCameraControl(context, true, true);
  }
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

inline void publishLowThrust(Context& context) {
  context.setpoint.publishBodyRateThrust(
      BodyRateThrust{Vec3{}, context.config.low_thrust});
}

inline bool handleLandingCompletion(Context& context) {
  const bool landed =
      context.landing_reached ||
      std::abs(context.telemetry.position.z -
               context.config.landing_reference_z) <
          context.config.landing_tolerance_z;
  if (!landed) {
    return false;
  }
  context.landing_reached = true;
  publishLowThrust(context);
  return true;
}

struct TickArmOnly {
  void operator()(Context& context) const {
    publishLowThrust(context);
    context.ensureOffboardArm();
  }
};

struct TickLowerHover {
  void operator()(Context& context) const {
    context.ensureOffboardArm();
    const Vec3 target{0.0, 0.0, 0.5};
    publishTrackCommand(context, fixedPositionHorizon(target));
  }
};
struct TickHighHover {
  void operator()(Context& context) const {
    context.ensureOffboardArm();
    const Vec3 target{1.0, 0.0, 1.5};
    publishTrackCommand(context, fixedPositionHorizon(target));
  }
};

struct TickLanding {
  void operator()(Context& context) const {
    if (context.landing_reached) {
      handleLandingCompletion(context);
      return;
    }

    context.ensureOffboardArm();
    std::vector<ReferencePoint> horizon;
    const bool prepared = context.landing.prepareLanding(
        context.clock.now(), context.telemetry, horizon);
    if (handleLandingCompletion(context)) {
      return;
    }
    if (prepared) {
      publishTrackCommand(context, horizon);
    }
  }
};

// 活动任务使用的视觉闭环版本。旧 TickLanding 保留给 MissionMachine，
// 便于独立回退和对比。
struct TickClosedLoopLanding {
  void operator()(Context& context) const {
    if (context.landing_reached) {
      handleLandingCompletion(context);
      return;
    }

    context.ensureOffboardArm();
    std::vector<ReferencePoint> horizon;
    const bool prepared = context.landing.prepareClosedLoopLanding(
        context.clock.now(), context.telemetry, horizon);
    if (handleLandingCompletion(context)) {
      return;
    }
    if (prepared) {
      publishTrackCommand(context, horizon);
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

struct ResetClosedLoopLanding {
  void operator()(Context& context) const {
    context.landing_reached = false;
    context.landing.reset();
    context.landing.startClosedLoopLanding(context.telemetry);
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
        std::cout<<"segment_index: "<<segment_index<<"\n";
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
