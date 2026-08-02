#ifndef FSM_CTRL_SINGLE_OFFBOARD_SML_ACTIONS_MISSION_HPP_
#define FSM_CTRL_SINGLE_OFFBOARD_SML_ACTIONS_MISSION_HPP_

#include <fsm_ctrl/single_offboard_sml/actions/common.hpp>

#include <vector>

namespace fsm_ctrl {
namespace single_sml {

struct TickSuperTrack {
  void operator()(Context& context) const {
    context.ensureOffboardArm();
    std::vector<ReferencePoint> horizon;
    const bool prepared =
        context.mission.prepareSuper(context.clock.now(), context.telemetry,
                                     horizon);
    if (prepared && !horizon.empty()) {
      writeHorizonLog(
          context, HorizonLogFields{LogSeverity::Debug,
                                    LogEvent::ActionSuperTrack, horizon});
      context.setpoint.publishFeedbackPosition(context.telemetry.position,
                                               context.telemetry.attitude);
      context.setpoint.publishReferencePosition(horizon.front().position,
                                                horizon.front().attitude);
      publishTrackCommand(context, horizon);
    } else {
      writeHorizonLog(
          context, HorizonLogFields{LogSeverity::Warn, LogEvent::EmptyHorizon,
                                    horizon});
    }
  }
};

}  // namespace single_sml
}  // namespace fsm_ctrl

#endif  // FSM_CTRL_SINGLE_OFFBOARD_SML_ACTIONS_MISSION_HPP_
