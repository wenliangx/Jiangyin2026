#ifndef FSM_CTRL_SINGLE_OFFBOARD_SML_MACHINES_MISSION_HPP_
#define FSM_CTRL_SINGLE_OFFBOARD_SML_MACHINES_MISSION_HPP_

#include <fsm_ctrl/single_offboard_sml/actions/mission.hpp>

#include <boost/sml.hpp>

namespace fsm_ctrl {
namespace single_sml {

#define FSM_CTRL_SML_MISSION_COMMAND_TRANSITIONS(source_state)                 \
  boost::sml::state<source_state> + boost::sml::event<OnCommand0> =           \
      boost::sml::state<Idle>,                                                \
  boost::sml::state<source_state> + boost::sml::event<OnCommand1> =           \
      boost::sml::state<ArmOnly>,                                             \
  boost::sml::state<source_state> + boost::sml::event<OnCommand2> =           \
      boost::sml::state<NmpcHover>,                                           \
  boost::sml::state<source_state> + boost::sml::event<OnCommand3> [MissionAvailable{}] / \
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

struct MissionMachine {
  auto operator()() const {
    using namespace boost::sml;
    return make_transition_table(
        *state<Idle> + event<Tick> / Noop{},
        state<ArmOnly> + event<Tick> / TickArmOnly{},
        state<NmpcHover> + event<Tick> / TickCoreHoverToOneMeter{},
        state<SuperTrack> + event<Tick> / TickSuperTrack{},
        state<Landing> + event<Tick> / TickLanding{},
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

#undef FSM_CTRL_SML_MISSION_COMMAND_TRANSITIONS

using MissionStateMachine = boost::sml::sm<MissionMachine>;
using ActiveMachine = MissionMachine;
using ActiveStateMachine = boost::sml::sm<ActiveMachine>;

}  // namespace single_sml
}  // namespace fsm_ctrl

#endif  // FSM_CTRL_SINGLE_OFFBOARD_SML_MACHINES_MISSION_HPP_
