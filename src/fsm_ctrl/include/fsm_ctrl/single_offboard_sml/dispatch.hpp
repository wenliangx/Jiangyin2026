#ifndef FSM_CTRL_SINGLE_OFFBOARD_SML_DISPATCH_CANONICAL_HPP_
#define FSM_CTRL_SINGLE_OFFBOARD_SML_DISPATCH_CANONICAL_HPP_

#include <fsm_ctrl/single_offboard_sml/machines/mission.hpp>
#include <fsm_ctrl/single_offboard_sml/machines/segmented_mission.hpp>

namespace fsm_ctrl {
namespace single_sml {

template <typename StateMachineT>
class CommandDispatcherT {
 public:
  explicit CommandDispatcherT(StateMachineT& machine,
                              ReferenceProvider* reference = nullptr,
                              MissionPort* mission = nullptr)
      : machine_(machine), reference_(reference), mission_(mission) {}

  bool update(int command) {
    if (has_previous_ && command == previous_) {
      return false;
    }
    previous_ = command;
    has_previous_ = true;
    if (reference_) {
      reference_->selectCommand(command);
    }
    if (mission_) {
      mission_->selectCommand(command);
    }
    dispatch(command);
    return true;
  }

 private:
  void dispatch(int command) {
    switch (command) {
      case 0: machine_.process_event(OnCommand0{}); break;
      case 1: machine_.process_event(OnCommand1{}); break;
      case 2: machine_.process_event(OnCommand2{}); break;
      case 3: machine_.process_event(OnCommand3{}); break;
      case 4: machine_.process_event(OnCommand4{}); break;
      case 5: machine_.process_event(OnCommand5{}); break;
      case 6: machine_.process_event(OnCommand6{}); break;
      case 7: machine_.process_event(OnCommand7{}); break;
      case 8: machine_.process_event(OnCommand8{}); break;
      case 9: machine_.process_event(OnCommand9{}); break;
      default: machine_.process_event(OnUnsupportedCommand{}); break;
    }
  }

  StateMachineT& machine_;
  ReferenceProvider* reference_{nullptr};
  MissionPort* mission_{nullptr};
  bool has_previous_{false};
  int previous_{0};
};

using MissionCommandDispatcher = CommandDispatcherT<MissionStateMachine>;
using SegmentedMissionCommandDispatcher =
    CommandDispatcherT<SegmentedMissionStateMachine>;
using ActiveCommandDispatcher = CommandDispatcherT<ActiveStateMachine>;

}  // namespace single_sml
}  // namespace fsm_ctrl

#endif  // FSM_CTRL_SINGLE_OFFBOARD_SML_DISPATCH_CANONICAL_HPP_
