#pragma once

#include <fsm_ctrl/flight_fsm/events.hpp>
#include <fsm_ctrl/flight_fsm/machine.hpp>
#include <fsm_ctrl/flight_fsm/ports.hpp>
#include <fsm_ctrl/fsm/distinct_command_dispatcher.hpp>

namespace fsm_ctrl {

// 命令分发器：把 UDP/用户输入的整数 cmd 转成 SML 的 command 事件。
// 连续重复的 cmd 会被忽略，避免重复执行进入状态时的 reset 逻辑；
// 主循环仍会继续发送 Tick，让当前状态保持周期执行。
struct CommandEventRouter {
  template <typename StateMachineT>
  void operator()(StateMachineT& machine, int command) const {
    switch (command) {
      case 0:
        machine.process(OnCommand0{});
        break;
      case 1:
        machine.process(OnCommand1{});
        break;
      case 2:
        machine.process(OnCommand2{});
        break;
      case 3:
        machine.process(OnCommand3{});
        break;
      case 4:
        machine.process(OnCommand4{});
        break;
      case 5:
        machine.process(OnCommand5{});
        break;
      case 6:
        machine.process(OnCommand6{});
        break;
      case 7:
        machine.process(OnCommand7{});
        break;
      case 8:
        machine.process(OnCommand8{});
        break;
      case 9:
        machine.process(OnCommand9{});
        break;
      default:
        machine.process(OnUnsupportedCommand{});
        break;
    }
  }
};

struct MissionCommandObserver {
  MissionPort* mission{nullptr};

  void operator()(int command) const {
    if (mission != nullptr) {
      mission->selectCommand(command);
    }
  }
};

template <typename StateMachineT>
class CommandDispatcherT
    : public DistinctCommandDispatcher<StateMachineT, CommandEventRouter, MissionCommandObserver> {
 private:
  using Base = DistinctCommandDispatcher<StateMachineT, CommandEventRouter, MissionCommandObserver>;

 public:
  explicit CommandDispatcherT(StateMachineT& machine, MissionPort* mission = nullptr)
      : Base(machine, CommandEventRouter{}, MissionCommandObserver{mission}) {}
};

using MissionCommandDispatcher = CommandDispatcherT<MissionStateMachine>;
using SegmentedMissionCommandDispatcher = CommandDispatcherT<SegmentedMissionStateMachine>;
using ActiveCommandDispatcher = CommandDispatcherT<ActiveStateMachine>;

}  // namespace fsm_ctrl
