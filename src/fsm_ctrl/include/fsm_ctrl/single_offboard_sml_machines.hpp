#ifndef FSM_CTRL_SINGLE_OFFBOARD_SML_MACHINES_HPP_
#define FSM_CTRL_SINGLE_OFFBOARD_SML_MACHINES_HPP_

#include <fsm_ctrl/single_offboard_sml_actions.hpp>

#include <boost/sml.hpp>

namespace fsm_ctrl {
namespace single_sml {

#define FSM_CTRL_SML_CORE_COMMAND_TRANSITIONS(source_state)                    \
  boost::sml::state<source_state> + boost::sml::event<OnCommand0> =           \
      boost::sml::state<Idle>,                                                \
  boost::sml::state<source_state> + boost::sml::event<OnCommand1> =           \
      boost::sml::state<ArmOnly>,                                             \
  boost::sml::state<source_state> + boost::sml::event<OnCommand2> =           \
      boost::sml::state<CoreHover>,                                           \
  boost::sml::state<source_state> + boost::sml::event<OnCommand3> =           \
      boost::sml::state<CoreSuperLanding>,                                    \
  boost::sml::state<source_state> + boost::sml::event<OnCommand4> /           \
      ResetLanding{} =                                                        \
      boost::sml::state<CoreLanding>,                                         \
  boost::sml::state<source_state> + boost::sml::event<OnCommand9> =           \
      boost::sml::state<Emergency>,                                           \
  boost::sml::state<source_state> + boost::sml::event<OnCommand7> =           \
      boost::sml::state<SafeNoop>,                                            \
  boost::sml::state<source_state> + boost::sml::event<OnCommand8> =           \
      boost::sml::state<SafeNoop>,                                            \
  boost::sml::state<source_state> + boost::sml::event<OnUnsupportedCommand> = \
      boost::sml::state<SafeNoop>

#define FSM_CTRL_SML_FULL_COMMAND_TRANSITIONS(source_state)                    \
  boost::sml::state<source_state> + boost::sml::event<OnCommand0> =           \
      boost::sml::state<Idle>,                                                \
  boost::sml::state<source_state> + boost::sml::event<OnCommand1> =           \
      boost::sml::state<LowThrust>,                                           \
  boost::sml::state<source_state> + boost::sml::event<OnCommand2> =           \
      boost::sml::state<PositionHold>,                                        \
  boost::sml::state<source_state> + boost::sml::event<OnCommand3> =           \
      boost::sml::state<NmpcHover>,                                           \
  boost::sml::state<source_state> + boost::sml::event<OnCommand4> /           \
      ResetLanding{} =                                                        \
      boost::sml::state<Landing>,                                             \
  boost::sml::state<source_state> + boost::sml::event<OnCommand9> =           \
      boost::sml::state<Emergency>,                                           \
  boost::sml::state<source_state> + boost::sml::event<OnCommand7> =           \
      boost::sml::state<SafeNoop>,                                            \
  boost::sml::state<source_state> + boost::sml::event<OnCommand8> =           \
      boost::sml::state<SafeNoop>,                                            \
  boost::sml::state<source_state> + boost::sml::event<OnUnsupportedCommand> = \
      boost::sml::state<SafeNoop>,                                            \
  boost::sml::state<source_state> + boost::sml::event<OnCommand5> /           \
      ResetNmpcTrack{} =                                                      \
      boost::sml::state<NmpcTrack>,                                           \
  boost::sml::state<source_state> + boost::sml::event<OnCommand6> /           \
      ResetSuperTrack{} =                                                     \
      boost::sml::state<SuperTrack>

#define FSM_CTRL_SML_MISSION_COMMAND_TRANSITIONS(source_state)                 \
  boost::sml::state<source_state> + boost::sml::event<OnCommand0> =           \
      boost::sml::state<Idle>,                                                \
  boost::sml::state<source_state> + boost::sml::event<OnCommand1> =           \
      boost::sml::state<ArmOnly>,                                             \
  boost::sml::state<source_state> + boost::sml::event<OnCommand2> =           \
      boost::sml::state<NmpcHover>,                                           \
  boost::sml::state<source_state> + boost::sml::event<OnCommand3> /           \
      ResetSuperTrack{} =                                                     \
      boost::sml::state<SuperTrack>,                                          \
  boost::sml::state<source_state> + boost::sml::event<OnCommand4> /           \
      ResetLanding{} =                                                        \
      boost::sml::state<Landing>,                                             \
  boost::sml::state<source_state> + boost::sml::event<OnCommand9> =           \
      boost::sml::state<Emergency>,                                           \
  boost::sml::state<source_state> + boost::sml::event<OnCommand5> =           \
      boost::sml::state<SafeNoop>,                                            \
  boost::sml::state<source_state> + boost::sml::event<OnCommand6> =           \
      boost::sml::state<SafeNoop>,                                            \
  boost::sml::state<source_state> + boost::sml::event<OnCommand7> =           \
      boost::sml::state<SafeNoop>,                                            \
  boost::sml::state<source_state> + boost::sml::event<OnCommand8> =           \
      boost::sml::state<SafeNoop>,                                            \
  boost::sml::state<source_state> + boost::sml::event<OnUnsupportedCommand> = \
      boost::sml::state<SafeNoop>

#define FSM_CTRL_SML_SEGMENTED_MISSION_COMMAND_TRANSITIONS(source_state)       \
  boost::sml::state<source_state> + boost::sml::event<OnCommand0> =           \
      boost::sml::state<Idle>,                                                \
  boost::sml::state<source_state> + boost::sml::event<OnCommand1> =           \
      boost::sml::state<ArmOnly>,                                             \
  boost::sml::state<source_state> + boost::sml::event<OnCommand2> =           \
      boost::sml::state<NmpcHover>,                                           \
  boost::sml::state<source_state> + boost::sml::event<OnCommand3> /           \
      ResetSuperTrack{} =                                                     \
      boost::sml::state<SuperSegment1>,                                       \
  boost::sml::state<source_state> + boost::sml::event<OnCommand4> /           \
      ResetSuperTrack{} =                                                     \
      boost::sml::state<SuperSegment2>,                                       \
  boost::sml::state<source_state> + boost::sml::event<OnCommand5> /           \
      ResetSuperTrack{} =                                                     \
      boost::sml::state<SuperSegment3>,                                       \
  boost::sml::state<source_state> + boost::sml::event<OnCommand6> /           \
      ResetLanding{} =                                                        \
      boost::sml::state<Landing>,                                             \
  boost::sml::state<source_state> + boost::sml::event<OnCommand9> =           \
      boost::sml::state<Emergency>,                                           \
  boost::sml::state<source_state> + boost::sml::event<OnCommand7> =           \
      boost::sml::state<SafeNoop>,                                            \
  boost::sml::state<source_state> + boost::sml::event<OnCommand8> =           \
      boost::sml::state<SafeNoop>,                                            \
  boost::sml::state<source_state> + boost::sml::event<OnUnsupportedCommand> = \
      boost::sml::state<SafeNoop>

struct CoreFlightMachine {
  auto operator()() const {
    using namespace boost::sml;
    return make_transition_table(
        *state<Idle> + event<Tick> / Noop{},
        state<ArmOnly> + event<Tick> / TickArmOnly{},
        state<CoreHover> + event<Tick> / TickCoreHoverToOneMeter{},
        state<CoreSuperLanding> + event<Tick> / TickCoreSuperLandingDebug{},
        state<CoreLanding> + event<Tick> / TickCoreLanding{},
        state<Emergency> + event<Tick> / TickEmergency{},
        state<SafeNoop> + event<Tick> / Noop{},
        FSM_CTRL_SML_CORE_COMMAND_TRANSITIONS(Idle),
        FSM_CTRL_SML_CORE_COMMAND_TRANSITIONS(ArmOnly),
        FSM_CTRL_SML_CORE_COMMAND_TRANSITIONS(CoreHover),
        FSM_CTRL_SML_CORE_COMMAND_TRANSITIONS(CoreSuperLanding),
        FSM_CTRL_SML_CORE_COMMAND_TRANSITIONS(CoreLanding),
        FSM_CTRL_SML_CORE_COMMAND_TRANSITIONS(Emergency),
        FSM_CTRL_SML_CORE_COMMAND_TRANSITIONS(SafeNoop));
  }
};

struct MissionMachine {
  auto operator()() const {
    using namespace boost::sml;
    return make_transition_table(
        *state<Idle> + event<Tick> / Noop{},
        state<ArmOnly> + event<Tick> / TickArmOnly{},
        state<NmpcHover> + event<Tick> / TickCoreHoverToOneMeter{},
        state<SuperTrack> + event<Tick> / TickSuperTrack{},
        state<Landing> + event<Tick> / TickCoreLanding{},
        state<Emergency> + event<Tick> / TickEmergency{},
        state<SafeNoop> + event<Tick> / Noop{},
        FSM_CTRL_SML_MISSION_COMMAND_TRANSITIONS(Idle),
        FSM_CTRL_SML_MISSION_COMMAND_TRANSITIONS(ArmOnly),
        FSM_CTRL_SML_MISSION_COMMAND_TRANSITIONS(NmpcHover),
        FSM_CTRL_SML_MISSION_COMMAND_TRANSITIONS(SuperTrack),
        FSM_CTRL_SML_MISSION_COMMAND_TRANSITIONS(Landing),
        FSM_CTRL_SML_MISSION_COMMAND_TRANSITIONS(Emergency),
        FSM_CTRL_SML_MISSION_COMMAND_TRANSITIONS(SafeNoop));
  }
};

struct SegmentedMissionMachine {
  auto operator()() const {
    using namespace boost::sml;
    return make_transition_table(
        *state<Idle> + event<Tick> / Noop{},
        state<ArmOnly> + event<Tick> / TickArmOnly{},
        state<NmpcHover> + event<Tick> / TickCoreHoverToOneMeter{},
        state<SuperSegment1> + event<Tick> / TickSuperSegment1{},
        state<SuperSegment2> + event<Tick> / TickSuperSegment2{},
        state<SuperSegment3> + event<Tick> / TickSuperSegment3{},
        state<Landing> + event<Tick> / TickCoreLanding{},
        state<Emergency> + event<Tick> / TickEmergency{},
        state<SafeNoop> + event<Tick> / Noop{},
        FSM_CTRL_SML_SEGMENTED_MISSION_COMMAND_TRANSITIONS(Idle),
        FSM_CTRL_SML_SEGMENTED_MISSION_COMMAND_TRANSITIONS(ArmOnly),
        FSM_CTRL_SML_SEGMENTED_MISSION_COMMAND_TRANSITIONS(NmpcHover),
        FSM_CTRL_SML_SEGMENTED_MISSION_COMMAND_TRANSITIONS(SuperSegment1),
        FSM_CTRL_SML_SEGMENTED_MISSION_COMMAND_TRANSITIONS(SuperSegment2),
        FSM_CTRL_SML_SEGMENTED_MISSION_COMMAND_TRANSITIONS(SuperSegment3),
        FSM_CTRL_SML_SEGMENTED_MISSION_COMMAND_TRANSITIONS(Landing),
        FSM_CTRL_SML_SEGMENTED_MISSION_COMMAND_TRANSITIONS(Emergency),
        FSM_CTRL_SML_SEGMENTED_MISSION_COMMAND_TRANSITIONS(SafeNoop));
  }
};

struct FullMissionMachine {
  auto operator()() const {
    using namespace boost::sml;
    return make_transition_table(
        *state<Idle> + event<Tick> / Noop{},
        state<LowThrust> + event<Tick> / TickLowThrust{},
        state<PositionHold> + event<Tick> / TickPositionHold{},
        state<NmpcHover> + event<Tick> / TickNmpcHover{},
        state<Landing> + event<Tick> / TickLanding{},
        state<NmpcTrack> + event<Tick> / TickNmpcTrack{},
        state<SuperTrack> + event<Tick> / TickSuperTrack{},
        state<Emergency> + event<Tick> / TickEmergency{},
        state<SafeNoop> + event<Tick> / Noop{},
        FSM_CTRL_SML_FULL_COMMAND_TRANSITIONS(Idle),
        FSM_CTRL_SML_FULL_COMMAND_TRANSITIONS(LowThrust),
        FSM_CTRL_SML_FULL_COMMAND_TRANSITIONS(PositionHold),
        FSM_CTRL_SML_FULL_COMMAND_TRANSITIONS(NmpcHover),
        FSM_CTRL_SML_FULL_COMMAND_TRANSITIONS(Landing),
        FSM_CTRL_SML_FULL_COMMAND_TRANSITIONS(NmpcTrack),
        FSM_CTRL_SML_FULL_COMMAND_TRANSITIONS(SuperTrack),
        FSM_CTRL_SML_FULL_COMMAND_TRANSITIONS(Emergency),
        FSM_CTRL_SML_FULL_COMMAND_TRANSITIONS(SafeNoop));
  }
};

#undef FSM_CTRL_SML_FULL_COMMAND_TRANSITIONS
#undef FSM_CTRL_SML_CORE_COMMAND_TRANSITIONS
#undef FSM_CTRL_SML_MISSION_COMMAND_TRANSITIONS
#undef FSM_CTRL_SML_SEGMENTED_MISSION_COMMAND_TRANSITIONS

using Machine = FullMissionMachine;
using StateMachine = boost::sml::sm<Machine>;
using CoreFlightStateMachine = boost::sml::sm<CoreFlightMachine>;
using MissionStateMachine = boost::sml::sm<MissionMachine>;
using SegmentedMissionStateMachine = boost::sml::sm<SegmentedMissionMachine>;

}  // namespace single_sml
}  // namespace fsm_ctrl

#endif  // FSM_CTRL_SINGLE_OFFBOARD_SML_MACHINES_HPP_
