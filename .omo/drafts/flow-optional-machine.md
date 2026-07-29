# Draft: flow-optional-machine

## TL;DR

Replace PipelineRegistry + PipelineState tick with **self-guarding data flows**. Each flow uses `optional<T>` short-circuit: if any stage returns `nullopt`, the chain stops. Each flow's first stage is a **state guard** — if the owning state is not active, the whole flow produces nothing. States define flows as part of their type. Main loop runs ALL flows linearly; only the active state's flows execute. No more `process_event(Tick{})`.

## Core idea

```
source | guard<State> | transform | transform | ... | sink
  │        │             │           │             │
  │        └─ 状态不活跃 → nullopt → 短路         │
  └─ optional<T>                                    └─ optional<U> | void
```

## Key types

```cpp
namespace flow {

// Flow<T>: ctx → optional<T>
template <typename T = bool>
class Flow {
public:
  using Fn = std::function<std::optional<T>(Context&)>;
  Flow(Fn fn) : fn_(std::move(fn)) {}
  std::optional<T> run(Context& ctx) const { return fn_(ctx); }
private:
  Fn fn_;
};

// make_source: 数据入口
template <typename T>
Flow<T> source(std::function<std::optional<T>(Context&)> reader);

// guard: 状态守卫（最头）
template <typename State, typename T>
Flow<T> guard(Flow<T> upstream);

// Flow<T> | then(T,ctx→optional<U>) → Flow<U>
template <typename T, typename U, typename Fn>
Flow<U> operator|(Flow<T> flow, Fn&& fn);

// Flow<T> | sink(T,ctx→void) → void
template <typename T, typename Fn>
??? operator|(Flow<T> flow, Fn&& fn);

}

struct Context {
  std::type_index active_state{typeid(NopState)};
  // optional DataPort members (if still needed)
};

// FlowRegistry
class FlowRegistry {
  std::vector<std::function<void(Context&)>> flows_;
public:
  template <typename... States>
  void register_all();
  void run(Context& ctx);  // for (auto& f : flows_) f(ctx);
};
```

## State definition

```cpp
struct TakeoffState {
  static auto flows() {
    using namespace flow;
    return std::make_tuple(
      source<Telemetry>(telemetry_fn)
        | guard<TakeoffState>
        | [](Telemetry t, Context& ctx) -> std::optional<bool> {
            ctx.ensure_offboard_arm();
            publish_position({0,0,1.0});
            return true;
          }
    );
  }
};

struct HoverState {
  static auto flows() {
    using namespace flow;
    return std::make_tuple(
      source<NmpcOutput>(nmpc_solve_fn)
        | guard<HoverState>
        | [](NmpcOutput o, Context&) -> std::optional<bool> {
            publish_body_rate(o.rate, o.thrust);
            return true;
          }
    );
  }
};
// etc.
```

## Changes

### New files (2)
- `core/flow.hpp` — Flow<T>, source(), guard(), operator| (optional short-circuit)
- `core/flow_registry.hpp` — FlowRegistry, flow registration

### Modified files (3)
- `core/context.hpp` — add `active_state: type_index`
- `example/machine.hpp` — states define `static flows()`, transition table adds active_state actions
- `src/example_node.cpp` — main loop replaces `process_event(Tick{})` with `flow_registry_.run(ctx)`

### Removed files (3)
- `core/pipe.hpp` — replaced by flow.hpp
- `core/subscription_manager.hpp` — replaced by flow_registry.hpp
- `core/dataflow.hpp` — DataPort/Pipeline/ServicePort/PipelineRegistry replaced by FlowRegistry

### Deleted types
- `PipelineRegistry`, `PipelineState`, `DataflowNmpcState`, `DataflowLandingState`
- `PipelineTraits`, `register_pipelines`

## Open questions
1. Flow compositor operator — `|` or `.then()`?
2. Transition action for active_state — manual per-transition or macro-generated?
3. Sink overload — void or returns optional<bool>?
