#ifndef FSM_CTRL_SINGLE_OFFBOARD_SML_MACHINES_HPP_
#define FSM_CTRL_SINGLE_OFFBOARD_SML_MACHINES_HPP_

#include <fsm_ctrl/single_offboard_sml_actions.hpp>

#include <boost/sml.hpp>

namespace fsm_ctrl {
namespace single_sml {

#define FSM_CTRL_SML_CORE_SELECT_TRANSITIONS(source_state)                    \
  boost::sml::state<source_state> + boost::sml::event<SelectIdle> =           \
      boost::sml::state<Idle>,                                                \
  boost::sml::state<source_state> + boost::sml::event<SelectLowThrust> =      \
      boost::sml::state<ArmOnly>,                                             \
  boost::sml::state<source_state> + boost::sml::event<SelectPositionHold> =   \
      boost::sml::state<CoreHover>,                                           \
  boost::sml::state<source_state> + boost::sml::event<SelectNmpcHover> =      \
      boost::sml::state<CoreSuperLanding>,                                    \
  boost::sml::state<source_state> + boost::sml::event<SelectLanding> /        \
      ResetLanding{} =                                                        \
      boost::sml::state<CoreLanding>,                                         \
  boost::sml::state<source_state> + boost::sml::event<SelectEmergency> =      \
      boost::sml::state<Emergency>,                                           \
  boost::sml::state<source_state> + boost::sml::event<SelectSafeNoop> =       \
      boost::sml::state<SafeNoop>

#define FSM_CTRL_SML_FULL_SELECT_TRANSITIONS(source_state)                    \
  boost::sml::state<source_state> + boost::sml::event<SelectIdle> =           \
      boost::sml::state<Idle>,                                                \
  boost::sml::state<source_state> + boost::sml::event<SelectLowThrust> =      \
      boost::sml::state<LowThrust>,                                           \
  boost::sml::state<source_state> + boost::sml::event<SelectPositionHold> =   \
      boost::sml::state<PositionHold>,                                        \
  boost::sml::state<source_state> + boost::sml::event<SelectNmpcHover> =      \
      boost::sml::state<NmpcHover>,                                           \
  boost::sml::state<source_state> + boost::sml::event<SelectLanding> /        \
      ResetLanding{} =                                                        \
      boost::sml::state<Landing>,                                             \
  boost::sml::state<source_state> + boost::sml::event<SelectEmergency> =      \
      boost::sml::state<Emergency>,                                           \
  boost::sml::state<source_state> + boost::sml::event<SelectSafeNoop> =       \
      boost::sml::state<SafeNoop>,                                            \
  boost::sml::state<source_state> + boost::sml::event<SelectNmpcTrack> /      \
      ResetNmpcTrack{} =                                                      \
      boost::sml::state<NmpcTrack>,                                           \
  boost::sml::state<source_state> + boost::sml::event<SelectSuperTrack> /     \
      ResetSuperTrack{} =                                                     \
      boost::sml::state<SuperTrack>

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
        FSM_CTRL_SML_CORE_SELECT_TRANSITIONS(Idle),
        FSM_CTRL_SML_CORE_SELECT_TRANSITIONS(ArmOnly),
        FSM_CTRL_SML_CORE_SELECT_TRANSITIONS(CoreHover),
        FSM_CTRL_SML_CORE_SELECT_TRANSITIONS(CoreSuperLanding),
        FSM_CTRL_SML_CORE_SELECT_TRANSITIONS(CoreLanding),
        FSM_CTRL_SML_CORE_SELECT_TRANSITIONS(Emergency),
        FSM_CTRL_SML_CORE_SELECT_TRANSITIONS(SafeNoop));
  }
};

struct MissionMachine {
  auto operator()() const {
    using namespace boost::sml;
    return make_transition_table(
        *state<Idle> + event<Tick> / Noop{},
        state<SuperTrack> + event<Tick> / TickSuperTrack{},
        state<Landing> + event<Tick> / TickLanding{},
        state<Emergency> + event<Tick> / TickEmergency{},
        state<SafeNoop> + event<Tick> / Noop{},
        FSM_CTRL_SML_FULL_SELECT_TRANSITIONS(Idle),
        FSM_CTRL_SML_FULL_SELECT_TRANSITIONS(SuperTrack),
        FSM_CTRL_SML_FULL_SELECT_TRANSITIONS(Landing),
        FSM_CTRL_SML_FULL_SELECT_TRANSITIONS(Emergency),
        FSM_CTRL_SML_FULL_SELECT_TRANSITIONS(SafeNoop));
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
        FSM_CTRL_SML_FULL_SELECT_TRANSITIONS(Idle),
        FSM_CTRL_SML_FULL_SELECT_TRANSITIONS(LowThrust),
        FSM_CTRL_SML_FULL_SELECT_TRANSITIONS(PositionHold),
        FSM_CTRL_SML_FULL_SELECT_TRANSITIONS(NmpcHover),
        FSM_CTRL_SML_FULL_SELECT_TRANSITIONS(Landing),
        FSM_CTRL_SML_FULL_SELECT_TRANSITIONS(NmpcTrack),
        FSM_CTRL_SML_FULL_SELECT_TRANSITIONS(SuperTrack),
        FSM_CTRL_SML_FULL_SELECT_TRANSITIONS(Emergency),
        FSM_CTRL_SML_FULL_SELECT_TRANSITIONS(SafeNoop));
  }
};

#undef FSM_CTRL_SML_FULL_SELECT_TRANSITIONS
#undef FSM_CTRL_SML_CORE_SELECT_TRANSITIONS

using Machine = FullMissionMachine;
using StateMachine = boost::sml::sm<Machine>;
using CoreFlightStateMachine = boost::sml::sm<CoreFlightMachine>;
using MissionStateMachine = boost::sml::sm<MissionMachine>;

}  // namespace single_sml
}  // namespace fsm_ctrl

#endif  // FSM_CTRL_SINGLE_OFFBOARD_SML_MACHINES_HPP_
