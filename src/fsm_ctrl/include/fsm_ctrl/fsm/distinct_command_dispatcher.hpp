#pragma once

#include <utility>

namespace fsm_ctrl {

struct IgnoreCommand {
  void operator()(int) const {}
};

// 将整数/枚举等外部命令的边沿转换成类型化事件。Router 负责领域映射，
// Observer 可在事件进入状态机前同步任务选择器、审计日志等外部适配器。
template <typename Machine, typename Router, typename Observer = IgnoreCommand>
class DistinctCommandDispatcher {
 public:
  explicit DistinctCommandDispatcher(Machine& machine, Router router = Router{},
                                     Observer observer = Observer{})
      : machine_(machine), router_(std::move(router)), observer_(std::move(observer)) {}

  bool update(int command) {
    if (has_previous_ && command == previous_) {
      return false;
    }
    previous_ = command;
    has_previous_ = true;
    observer_(command);
    router_(machine_, command);
    return true;
  }

  void reset() { has_previous_ = false; }
  bool hasPrevious() const { return has_previous_; }
  int previous() const { return previous_; }

 private:
  Machine& machine_;
  Router router_;
  Observer observer_;
  bool has_previous_{false};
  int previous_{0};
};

}  // namespace fsm_ctrl
