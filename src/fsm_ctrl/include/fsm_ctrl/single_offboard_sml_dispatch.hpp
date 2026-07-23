#ifndef FSM_CTRL_SINGLE_OFFBOARD_SML_DISPATCH_HPP_
#define FSM_CTRL_SINGLE_OFFBOARD_SML_DISPATCH_HPP_

#include <fsm_ctrl/single_offboard_sml.hpp>

namespace fsm_ctrl {
namespace single_sml {

// 命令分发器：把 UDP/用户输入的整数 cmd 转成 SML 的 Select 事件。
// 连续重复的 cmd 会被忽略，避免重复执行进入状态时的 reset 逻辑；
// 主循环仍会继续发送 Tick，让当前状态保持周期执行。
class CommandDispatcher {
 public:
  // reference 和 mission 为可选适配器；它们先收到原始 cmd，
  // 用于在 FSM Tick 前选择对应的参考源或任务模式。
  explicit CommandDispatcher(StateMachine& machine,
                             ReferenceProvider* reference = nullptr,
                             MissionPort* mission = nullptr)
      : machine_(machine), reference_(reference), mission_(mission) {}

  // 处理一条 cmd；只有 cmd 变化并触发 Select 事件时返回 true。
  // 映射关系：
  //   0 Idle，1 LowThrust，2 PositionHold，3 NmpcHover，4 Landing，
  //   5 NmpcTrack，6 SuperTrack，7 MissionTrack，8 EgoTrack，
  //   9 Emergency，其它值进入 SafeNoop。
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
    switch (command) {
      case 0: machine_.process_event(SelectIdle{}); break;
      case 1: machine_.process_event(SelectLowThrust{}); break;
      case 2: machine_.process_event(SelectPositionHold{}); break;
      case 3: machine_.process_event(SelectNmpcHover{}); break;
      case 4: machine_.process_event(SelectLanding{}); break;
      case 5: machine_.process_event(SelectNmpcTrack{}); break;
      case 6: machine_.process_event(SelectSuperTrack{}); break;
      case 7: machine_.process_event(SelectMissionTrack{}); break;
      case 8: machine_.process_event(SelectEgoTrack{}); break;
      case 9: machine_.process_event(SelectEmergency{}); break;
      default: machine_.process_event(SelectSafeNoop{}); break;
    }
    return true;
  }

 private:
  // 被驱动的 Boost.SML 状态机实例。
  StateMachine& machine_;
  // cmd5 参考轨迹适配器；允许为空，便于纯状态机测试。
  ReferenceProvider* reference_{nullptr};
  // cmd6/7/8 任务轨迹适配器；允许为空，便于纯状态机测试。
  MissionPort* mission_{nullptr};
  // 上一次是否已经收到过 cmd。
  bool has_previous_{false};
  // 上一次处理的 cmd，用于重复命令抑制。
  int previous_{0};
};

}  // namespace single_sml
}  // namespace fsm_ctrl

#endif  // FSM_CTRL_SINGLE_OFFBOARD_SML_DISPATCH_HPP_
