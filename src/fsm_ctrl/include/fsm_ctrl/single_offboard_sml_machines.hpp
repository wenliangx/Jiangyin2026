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
  boost::sml::state<source_state> + boost::sml::event<OnCommand5> =           \
      boost::sml::state<SafeNoop>,                                            \
  boost::sml::state<source_state> + boost::sml::event<OnCommand6> =           \
      boost::sml::state<SafeNoop>,                                            \
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
        *state<Idle> + event<Tick> / DisableCameras{},
        state<ArmOnly> + event<Tick> /
            (DisableCameras{}, TickArmOnly{}),
        state<CoreHover> + event<Tick> /
            (DisableCameras{}, TickCoreHoverToOneMeter{}),
        state<CoreSuperLanding> + event<Tick> /
            (EnableDownCamera{}, TickCoreSuperLandingDebug{}),
        state<CoreLanding> + event<Tick> /
            (EnableDownCamera{}, TickCoreLanding{}),
        state<Emergency> + event<Tick> /
            (DisableCameras{}, TickEmergency{}),
        state<SafeNoop> + event<Tick> / DisableCameras{},
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
        *state<Idle> + event<Tick> / DisableCameras{},
        state<ArmOnly> + event<Tick> /
            (DisableCameras{}, TickArmOnly{}),
        state<NmpcHover> + event<Tick> /
            (DisableCameras{}, TickLowerHover{}),
        state<SuperTrack> + event<Tick> /
            (EnableFrontCamera{}, TickSuperTrack{}),
        state<Landing> + event<Tick> /
            (EnableDownCamera{}, TickLanding{}),
        state<Emergency> + event<Tick> /
            (DisableCameras{}, TickEmergency{}),
        state<SafeNoop> + event<Tick> / DisableCameras{},
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
        *state<Idle> + event<Tick> / DisableCameras{},
        state<ArmOnly> + event<Tick> /
            (DisableCameras{}, TickArmOnly{}),
        state<NmpcHover> + event<Tick> /
            (DisableCameras{}, TickLowerHover{}),
        state<SuperSegment1> + event<Tick> /
            (EnableFrontCamera{}, TickSuperSegment1{}),
        state<SuperSegment2> + event<Tick> /
            (EnableFrontCamera{}, TickSuperSegment2{}),
        state<SuperSegment3> + event<Tick> /
            (EnableDownCamera{}, TickSuperSegment3{}),
        state<Landing> + event<Tick> /
            (EnableDownCamera{}, TickLanding{}),
        state<Emergency> + event<Tick> /
            (DisableCameras{}, TickEmergency{}),
        state<SafeNoop> + event<Tick> / DisableCameras{},
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
        *state<Idle> + event<Tick> / DisableCameras{},
        state<LowThrust> + event<Tick> /
            (DisableCameras{}, TickLowThrust{}),
        state<PositionHold> + event<Tick> /
            (DisableCameras{}, TickPositionHold{}),
        state<NmpcHover> + event<Tick> /
            (DisableCameras{}, TickNmpcHover{}),
        state<Landing> + event<Tick> /
            (EnableDownCamera{}, TickLanding{}),
        state<NmpcTrack> + event<Tick> /
            (DisableCameras{}, TickNmpcTrack{}),
        state<SuperTrack> + event<Tick> /
            (EnableFrontCamera{}, TickSuperTrack{}),
        state<Emergency> + event<Tick> /
            (DisableCameras{}, TickEmergency{}),
        state<SafeNoop> + event<Tick> / DisableCameras{},
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
using ActiveMachine = SegmentedMissionMachine;
using ActiveStateMachine = boost::sml::sm<ActiveMachine>;

}  // namespace single_sml
}  // namespace fsm_ctrl

#endif  // FSM_CTRL_SINGLE_OFFBOARD_SML_MACHINES_HPP_
