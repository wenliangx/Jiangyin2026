#ifndef FSM_CTRL_FSM_STATE_MACHINE_HPP_
#define FSM_CTRL_FSM_STATE_MACHINE_HPP_

#include <boost/sml.hpp>

#include <utility>

namespace fsm_ctrl {
namespace fsm {

// Boost.SML 的轻量运行时外壳。Definition 只描述转换表，TickEvent 只描述
// 周期事件；ROS、时钟、控制器等依赖由构造函数注入，框架本身不依赖 ROS。
template <typename Definition, typename TickEvent>
class StateMachine {
 public:
  using DefinitionType = Definition;
  using TickEventType = TickEvent;
  using NativeMachine = boost::sml::sm<Definition>;

  template <typename... Dependencies>
  explicit StateMachine(Dependencies&&... dependencies)
      : machine_(std::forward<Dependencies>(dependencies)...) {}

  template <typename Event>
  bool process(Event&& event) {
    return machine_.process_event(std::forward<Event>(event));
  }

  bool tick() { return process(TickEvent{}); }

  template <typename State>
  bool isState() const {
    return machine_.is(boost::sml::state<State>);
  }

  // 兼容原生 Boost.SML API，便于现有状态机渐进迁移。
  template <typename Event>
  bool process_event(Event&& event) {
    return process(std::forward<Event>(event));
  }

  template <typename StateExpression>
  bool is(const StateExpression& state) const {
    return machine_.is(state);
  }

  NativeMachine& native() { return machine_; }
  const NativeMachine& native() const { return machine_; }

 private:
  NativeMachine machine_;
};

}  // namespace fsm
}  // namespace fsm_ctrl

#endif  // FSM_CTRL_FSM_STATE_MACHINE_HPP_
