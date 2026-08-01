#ifndef FSM_CTRL_SINGLE_OFFBOARD_SML_ACTIONS_SEGMENTED_MISSION_HPP_
#define FSM_CTRL_SINGLE_OFFBOARD_SML_ACTIONS_SEGMENTED_MISSION_HPP_

#include <fsm_ctrl/single_offboard_sml/actions/common.hpp>

#include <vector>

namespace fsm_ctrl {
namespace single_sml {

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

}  // namespace single_sml
}  // namespace fsm_ctrl

#endif  // FSM_CTRL_SINGLE_OFFBOARD_SML_ACTIONS_SEGMENTED_MISSION_HPP_
