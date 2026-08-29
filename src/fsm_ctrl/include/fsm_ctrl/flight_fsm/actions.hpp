#pragma once

#include <cmath>
#include <fsm_ctrl/flight_fsm/context.hpp>
#include <fsm_ctrl/flight_fsm/events.hpp>
#include <fsm_ctrl/flight_fsm/landing_planner.hpp>
#include <fsm_ctrl/flight_fsm/states.hpp>
#include <vector>

namespace fsm_ctrl {

inline void publishCameraControl(Context& context, bool front_enabled, bool down_enabled) {
  context.camera_control.publishControl(CameraControlState{front_enabled, down_enabled});
}

struct DisableCameras {
  void operator()(Context& context) const { publishCameraControl(context, false, false); }
};

struct EnableFrontCamera {
  void operator()(Context& context) const { publishCameraControl(context, true, false); }
};

struct EnableDownCamera {
  void operator()(Context& context) const { publishCameraControl(context, false, true); }
};

struct EnableBothCameras {
  void operator()(Context& context) const { publishCameraControl(context, true, true); }
};

inline bool publishTrackCommand(Context& context, const std::vector<ReferencePoint>& horizon) {
  BodyRateThrust command;
  if (horizon.empty() || !context.nmpc.solveTrack(context.telemetry, horizon, command) ||
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
  context.setpoint.publishBodyRateThrust(BodyRateThrust{Vec3{}, context.config.low_thrust});
}

inline bool handleLandingCompletion(Context& context) {
  const bool landed = context.landing_reached ||
                      std::abs(context.telemetry.position.z - context.config.landing_reference_z) <
                          context.config.landing_tolerance_z;
  if (!landed) {
    return false;
  }
  context.landing_reached = true;
  context.permanent_landing_lock = true;
  if (context.telemetry.armed) {
    publishLowThrust(context);
    context.ensureDisarm();
  }
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

    std::vector<ReferencePoint> horizon;
    const bool prepared =
        context.landing.prepareLanding(context.clock.now(), context.telemetry, horizon);
    if (handleLandingCompletion(context)) {
      return;
    }
    if (prepared) {
      publishTrackCommand(context, horizon);
    }
  }
};

// 备用视觉闭环版本；当前两个状态机都使用 TickLanding。
struct TickClosedLoopLanding {
  void operator()(Context& context) const {
    if (context.landing_reached) {
      handleLandingCompletion(context);
      return;
    }

    std::vector<ReferencePoint> horizon;
    const bool prepared =
        context.landing.prepareClosedLoopLanding(context.clock.now(), context.telemetry, horizon);
    if (handleLandingCompletion(context)) {
      return;
    }
    if (prepared) {
      publishTrackCommand(context, horizon);
    }
  }
};

struct ResetSuperTrack {
  void operator()(Context& context) const { context.mission.reset(); }
};

struct StartSegmentedMission {
  void operator()(Context& context) const {
    context.recognized_targets = {};
    context.mission.reset();
  }
};

struct FirstTargetAvailable {
  bool operator()(const OnTargetRecognized& event) const { return !event.label.empty(); }
};

struct NewTargetAvailable {
  bool operator()(const OnTargetRecognized& event, const Context& context) const {
    return !event.label.empty() &&
           (context.recognized_targets[0].empty() || event.label != context.recognized_targets[0]);
  }
};

// 永久落地锁置位后拒绝所有状态命令，使状态机无法离开 Landing。
struct TerminalSafetyUnlocked {
  bool operator()(const Context& context) const { return !context.permanent_landing_lock; }
};

struct StoreFirstTargetAndResetSuper {
  void operator()(const OnTargetRecognized& event, Context& context) const {
    context.recognized_targets[0] = event.label;
    context.mission.selectCommand(4);
    context.mission.reset();
  }
};

struct StoreNextTargetAndResetSuper {
  void operator()(const OnTargetRecognized& event, Context& context) const {
    if (context.recognized_targets[0].empty()) {
      context.recognized_targets[0] = event.label;
    } else {
      context.recognized_targets[1] = event.label;
    }
    context.mission.selectCommand(5);
    context.mission.reset();
  }
};

struct StartSuperSegment2 {
  void operator()(Context& context) const {
    context.mission.selectCommand(4);
    context.mission.reset();
  }
};

struct StartSuperSegment3 {
  void operator()(Context& context) const {
    context.mission.selectCommand(5);
    context.mission.reset();
  }
};

struct ResetLanding {
  void operator()(Context& context) const {
    context.landing_reached = false;
    context.disarm_request_started = false;
    context.landing.reset();
  }
};

struct TickSuperTrack {
  void operator()(Context& context) const {
    context.ensureOffboardArm();
    std::vector<ReferencePoint> horizon;
    const bool prepared =
        context.mission.prepareSuper(context.clock.now(), context.telemetry, horizon);
    if (prepared && !horizon.empty()) {
      context.setpoint.publishFeedbackPosition(context.telemetry.position,
                                               context.telemetry.attitude);
      context.setpoint.publishReferencePosition(horizon.front().position, horizon.front().attitude);
      publishTrackCommand(context, horizon);
    }
  }
};

inline void tickSuperSegment(Context& context, int segment_index) {
  context.ensureOffboardArm();
  std::vector<ReferencePoint> horizon;
  if (context.mission.prepareSuperSegment(segment_index, context.clock.now(), context.telemetry,
                                          horizon) &&
      !horizon.empty()) {
    context.setpoint.publishFeedbackPosition(context.telemetry.position,
                                             context.telemetry.attitude);
    context.setpoint.publishReferencePosition(horizon.front().position, horizon.front().attitude);
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

}  // namespace fsm_ctrl
