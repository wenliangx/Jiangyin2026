# auto-subscription-machine

## TL;DR (For humans)

Move pipeline definitions from `example_node.cpp` (node constructor) into `machine.hpp` (state definitions). Add a `SubscriptionManager` that auto-registers pipelines via fold expression over state types. States declare `static pipeline()` as a method; `SubscriptionManager::registerAll<States...>(registry)` registers them in one line. PipelineRegistry and PipelineState stay unchanged. ExampleNode constructor drops 6 manual `add<>()` calls.

**Files changed:** ~4 files, ~+80 LOC, ~-40 LOC
**States adding `pipeline()`:** DfTakeoff, DfHover, DfLanding, DfSuperTrack, DfSuperTrackB — all 5
**Automation target:** `pipeline_registry_->add<State>(pipeline)` — 6 calls → 1 fold-expression call

---

## Must-Have

1. States declare `static auto pipeline()` that returns `std::function<bool(Context&)>` or lambda
2. SubscriptionManager auto-registers all state pipelines via `registerAll<States...>(registry)`
3. ExampleNode constructor calls ONE registration line instead of six
4. All 11 existing tests pass
5. PipelineRegistry interface unchanged (backward compatible)

## Must-NOT-Have

- No change to Boost.SML transition table format
- No change to PipelineState / PipelineRegistry / Context types
- No subscribe/unsubscribe semantics (lifecycle = activate/deactivate only)
- No change to ros_backends or Singletons

---

## Todos

- [ ] 1. Create `core/subscription_manager.hpp` — `register_pipelines<States...>(PipelineRegistry&)` using fold expression. Includes `has_pipeline` detection trait. ~25 LOC. Depends: none. Parallel: task 2.
- [ ] 2. Add `pipeline()` static methods to all 5 states in `machine.hpp`. DfTakeoff and DfLanding use `+(Source<Telemetry> | ...)`. DfHover/DfSuperTrack/DfSuperTrackB return lambdas that call Singletons directly (no arm_svc_/nmpc_flow_/landing_ctrl_ dependency — controllers are static locals inside the lambdas). ~50 LOC. Depends: task 1. Blocks: task 3.
- [ ] 3. Simplify `example_node.cpp` — remove 6 `registry.add<>()` calls, replace with `register_pipelines<DfTakeoff, DfHover, DfLanding, DfSuperTrack, DfSuperTrackB>(*registry);`. Remove `arm_svc_`, `nmpc_flow_`, `landing_ctrl_` members (moved into pipeline bodies as static locals). ~-40 LOC. Depends: task 2. Blocks: task 4.
- [ ] 4. `catkin_make flight_fsm && flight_fsm_core_test` — verify compile and all tests pass. Depends: task 3.

## Final verification wave

- [ ] F1. `catkin_make flight_fsm` — 0 errors, 0 warnings
- [ ] F2. `flight_fsm_core_test` — all 11+ tests pass
- [ ] F3. Spot-check: all 5 pipelines registered (registry contains 5+ entries)
- [ ] F4. No dangling `arm_svc_`/`nmpc_flow_`/`landing_ctrl_` in DataflowNode class
