#ifndef FSM_CTRL_SINGLE_OFFBOARD_SML_MACHINES_HPP_
#define FSM_CTRL_SINGLE_OFFBOARD_SML_MACHINES_HPP_

#include <fsm_ctrl/single_offboard_sml_actions.hpp>

#include <boost/sml.hpp>

namespace fsm_ctrl {
namespace single_sml {

#define FSM_CTRL_SML_MISSION_COMMAND_TRANSITIONS(source_state)                 \
  boost::sml::state<source_state> + boost::sml::event<OnCommand0>            \
      [TerminalSafetyUnlocked{}] =                                           \
      boost::sml::state<Idle>,                                                \
  boost::sml::state<source_state> + boost::sml::event<OnCommand1>            \
      [TerminalSafetyUnlocked{}] =                                           \
      boost::sml::state<ArmOnly>,                                             \
  boost::sml::state<source_state> + boost::sml::event<OnCommand2>            \
      [TerminalSafetyUnlocked{}] =                                           \
      boost::sml::state<NmpcHover>,                                           \
  boost::sml::state<source_state> + boost::sml::event<OnCommand3>            \
      [TerminalSafetyUnlocked{}] /                                            \
      ResetSuperTrack{} =                                                     \
      boost::sml::state<SuperTrack>,                                          \
  boost::sml::state<source_state> + boost::sml::event<OnCommand4>            \
      [TerminalSafetyUnlocked{}] /                                            \
      ResetLanding{} =                                                        \
      boost::sml::state<Landing>,                                             \
  boost::sml::state<source_state> + boost::sml::event<OnCommand9>            \
      [TerminalSafetyUnlocked{}] =                                           \
      boost::sml::state<Emergency>,                                           \
  boost::sml::state<source_state> + boost::sml::event<OnCommand5>            \
      [TerminalSafetyUnlocked{}] =                                           \
      boost::sml::state<SafeNoop>,                                            \
  boost::sml::state<source_state> + boost::sml::event<OnCommand6>            \
      [TerminalSafetyUnlocked{}] =                                           \
      boost::sml::state<SafeNoop>,                                            \
  boost::sml::state<source_state> + boost::sml::event<OnCommand7>            \
      [TerminalSafetyUnlocked{}] =                                           \
      boost::sml::state<SafeNoop>,                                            \
  boost::sml::state<source_state> + boost::sml::event<OnCommand8>            \
      [TerminalSafetyUnlocked{}] =                                           \
      boost::sml::state<SafeNoop>,                                            \
  boost::sml::state<source_state> + boost::sml::event<OnUnsupportedCommand>  \
      [TerminalSafetyUnlocked{}] =                                           \
      boost::sml::state<SafeNoop>

#define FSM_CTRL_SML_SEGMENTED_MISSION_COMMAND_TRANSITIONS(source_state)       \
  boost::sml::state<source_state> + boost::sml::event<OnCommand0>            \
      [TerminalSafetyUnlocked{}] =                                           \
      boost::sml::state<Idle>,                                                \
  boost::sml::state<source_state> + boost::sml::event<OnCommand1>            \
      [TerminalSafetyUnlocked{}] =                                           \
      boost::sml::state<ArmOnly>,                                             \
  boost::sml::state<source_state> + boost::sml::event<OnCommand2>            \
      [TerminalSafetyUnlocked{}] =                                           \
      boost::sml::state<NmpcHover>,                                           \
  boost::sml::state<source_state> + boost::sml::event<OnCommand3>            \
      [TerminalSafetyUnlocked{}] /                                            \
      StartSegmentedMission{} =                                               \
      boost::sml::state<SuperSegment1>,                                       \
  boost::sml::state<source_state> + boost::sml::event<OnCommand4>            \
      [TerminalSafetyUnlocked{}] /                                            \
      ResetSuperTrack{} =                                                     \
      boost::sml::state<SuperSegment2>,                                       \
  boost::sml::state<source_state> + boost::sml::event<OnCommand5>            \
      [TerminalSafetyUnlocked{}] /                                            \
      ResetSuperTrack{} =                                                     \
      boost::sml::state<SuperSegment3>,                                       \
  boost::sml::state<source_state> + boost::sml::event<OnCommand6>            \
      [TerminalSafetyUnlocked{}] /                                            \
      ResetLanding{} =                                                        \
      boost::sml::state<Landing>,                                             \
  boost::sml::state<source_state> + boost::sml::event<OnCommand9>            \
      [TerminalSafetyUnlocked{}] =                                           \
      boost::sml::state<Emergency>,                                           \
  boost::sml::state<source_state> + boost::sml::event<OnCommand7>            \
      [TerminalSafetyUnlocked{}] =                                           \
      boost::sml::state<SafeNoop>,                                            \
  boost::sml::state<source_state> + boost::sml::event<OnCommand8>            \
      [TerminalSafetyUnlocked{}] =                                           \
      boost::sml::state<SafeNoop>,                                            \
  boost::sml::state<source_state> + boost::sml::event<OnUnsupportedCommand>  \
      [TerminalSafetyUnlocked{}] =                                           \
      boost::sml::state<SafeNoop>

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
            (EnableDownCamera{}, TickSuperSegment2{}),
        state<SuperSegment3> + event<Tick> /
            (DisableCameras{}, TickSuperSegment3{}),
        state<SuperSegment1> + event<OnTargetRecognized>
            [FirstTargetAvailable{}] /
            StoreFirstTargetAndResetSuper{} = state<SuperSegment2>,
        state<SuperSegment2> + event<OnTargetRecognized>
            [NewTargetAvailable{}] /
            StoreNextTargetAndResetSuper{} = state<SuperSegment3>,
        state<SuperSegment1> + event<OnSegmentTimeout> /
            StartSuperSegment2{} = state<SuperSegment2>,
        state<SuperSegment2> + event<OnSegmentTimeout> /
            StartSuperSegment3{} = state<SuperSegment3>,
        state<SuperSegment3> + event<OnFinalSegmentComplete> /
            ResetLanding{} = state<Landing>,
        state<Landing> + event<Tick> /
            (DisableCameras{}, TickLanding{}),
        state<Emergency> + event<Tick> /
            (DisableCameras{}, TickLanding{}),
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

#undef FSM_CTRL_SML_MISSION_COMMAND_TRANSITIONS
#undef FSM_CTRL_SML_SEGMENTED_MISSION_COMMAND_TRANSITIONS

using MissionStateMachine = boost::sml::sm<MissionMachine>;
using SegmentedMissionStateMachine = boost::sml::sm<SegmentedMissionMachine>;
using ActiveMachine = SegmentedMissionMachine;
using ActiveStateMachine = boost::sml::sm<ActiveMachine>;

}  // namespace single_sml
}  // namespace fsm_ctrl

#endif  // FSM_CTRL_SINGLE_OFFBOARD_SML_MACHINES_HPP_
