#ifndef FSM_CTRL_SINGLE_OFFBOARD_SML_DISPATCH_HPP_
#define FSM_CTRL_SINGLE_OFFBOARD_SML_DISPATCH_HPP_

#include <fsm_ctrl/single_offboard_sml.hpp>

namespace fsm_ctrl {
namespace single_sml {

// 命令分发器：把 UDP/用户输入的整数 cmd 转成 SML 的 command 事件。
// 连续重复的 cmd 会被忽略，避免重复执行进入状态时的 reset 逻辑；
// 主循环仍会继续发送 Tick，让当前状态保持周期执行。
template <typename StateMachineT>
class CommandDispatcherT {
 public:
  // reference 和 mission 为可选适配器；它们先收到原始 cmd，
  // 用于在 FSM Tick 前选择对应的参考源或任务模式。
  explicit CommandDispatcherT(StateMachineT& machine,
                              ReferenceProvider* reference = nullptr,
                              MissionPort* mission = nullptr)
      : machine_(machine), reference_(reference), mission_(mission) {}

  // 处理一条 cmd；只有 cmd 变化并触发 command 事件时返回 true。
  // cmd 的实际含义由当前 Machine 的 transition table 决定。
  // dispatcher 只负责把整数转成统一事件。
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

  // 被驱动的 Boost.SML 状态机实例。
  StateMachineT& machine_;
  // cmd5 参考轨迹适配器；允许为空，便于纯状态机测试。
  ReferenceProvider* reference_{nullptr};
  // cmd6/7/8 任务轨迹适配器；允许为空，便于纯状态机测试。
  MissionPort* mission_{nullptr};
  // 上一次是否已经收到过 cmd。
  bool has_previous_{false};
  // 上一次处理的 cmd，用于重复命令抑制。
  int previous_{0};
};

using CommandDispatcher = CommandDispatcherT<StateMachine>;
using CoreFlightCommandDispatcher = CommandDispatcherT<CoreFlightStateMachine>;
using MissionCommandDispatcher = CommandDispatcherT<MissionStateMachine>;
using SegmentedMissionCommandDispatcher =
    CommandDispatcherT<SegmentedMissionStateMachine>;
using ActiveCommandDispatcher = CommandDispatcherT<ActiveStateMachine>;

}  // namespace single_sml
}  // namespace fsm_ctrl

#endif  // FSM_CTRL_SINGLE_OFFBOARD_SML_DISPATCH_HPP_
