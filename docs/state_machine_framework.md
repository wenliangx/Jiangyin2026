# 可复用状态机框架

`fsm_ctrl` 提供一个不依赖 ROS、MAVROS、CasADi 的 C++14 头文件框架，位于
`include/fsm_ctrl/fsm/`。它保留 Boost.SML 的类型安全转换表，同时统一了任务
节点最容易重复实现的两部分：状态机生命周期与外部整数命令的边沿分发。

## 组成

- `StateMachine<Definition, TickEvent>`：持有 SML 实例，注入端口/上下文依赖，
  提供 `process(event)`、`tick()` 和 `isState<State>()`。
- `DistinctCommandDispatcher<Machine, Router, Observer>`：仅在命令变化时调用
  Router；Observer 可在事件前同步任务选择器、日志或指标。
- Definition、Event、State、Action、Guard 与所有端口仍由具体业务声明。框架
  不读取 ROS 参数，也不拥有线程，因此单元测试无需启动 roscore。

## 最小示例

```cpp
#include <fsm_ctrl/fsm/state_machine.hpp>

struct Idle {};
struct Running {};
struct Start {};
struct Tick {};
struct Context { int ticks{0}; };
struct Count { void operator()(Context& c) const { ++c.ticks; } };

struct MachineDefinition {
  auto operator()() const {
    using namespace boost::sml;
    return make_transition_table(
        *state<Idle> + event<Start> = state<Running>,
        state<Running> + event<Tick> / Count{});
  }
};

Context context;
fsm_ctrl::fsm::StateMachine<MachineDefinition, Tick> machine(context);
machine.process(Start{});
machine.tick();
```

外部协议到事件的映射应放在独立 Router 中。这样 UDP、RC、ROS topic 或测试
输入可以复用同一转换表，且协议编号不会渗入动作和守卫。

## 在飞行状态机中的落地方式

比赛状态机由 `include/fsm_ctrl/flight_fsm/machine.hpp` 定义转换表，
`MissionStateMachine`、`SegmentedMissionStateMachine` 与 `ActiveStateMachine`
使用通用 `StateMachine`。`flight_fsm/command_dispatcher.hpp` 只保留
比赛命令到类型化事件的 Router 以及任务端口 Observer。

新增状态机时建议按以下边界组织：

1. `types`：纯数据结构，不含 ROS 消息。
2. `ports`：外部能力的抽象接口。
3. `context`：端口引用与运行时状态。
4. `states` / `events`：状态标签与带载荷事件分文件定义。
5. `actions/guards`：只依赖 context 和 event。
6. `machine`：仅包含转换表与 active machine 选择。
7. ROS 节点：消息转换、参数加载、订阅发布和 50 Hz 调度。

测试示例见 `test/state_machine_framework_test.cpp`；完整的端口 fake 与安全终态
测试见 `test/flight_fsm_test.cpp`。
