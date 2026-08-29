#include <gtest/gtest.h>

#include <fsm_ctrl/fsm/distinct_command_dispatcher.hpp>
#include <fsm_ctrl/fsm/state_machine.hpp>

namespace {

struct Idle {};
struct Running {};
struct Start {};
struct Stop {};
struct Tick {};

struct Counter {
  int ticks{0};
};

struct CountTick {
  void operator()(Counter& counter) const { ++counter.ticks; }
};

struct ExampleMachine {
  auto operator()() const {
    using namespace boost::sml;
    return make_transition_table(
        *state<Idle> + event<Start> = state<Running>,
        state<Running> + event<Tick> / CountTick{},
        state<Running> + event<Stop> = state<Idle>);
  }
};

using Runtime = fsm_ctrl::fsm::StateMachine<ExampleMachine, Tick>;

struct ExampleRouter {
  void operator()(Runtime& machine, int command) const {
    if (command == 1) {
      machine.process(Start{});
    } else {
      machine.process(Stop{});
    }
  }
};

struct RecordCommand {
  int* value;
  void operator()(int command) const { *value = command; }
};

TEST(StateMachineFramework, OwnsMachineAndProvidesTypedTick) {
  Counter counter;
  Runtime machine(counter);

  EXPECT_TRUE(machine.isState<Idle>());
  machine.process(Start{});
  EXPECT_TRUE(machine.isState<Running>());
  EXPECT_TRUE(machine.tick());
  EXPECT_EQ(1, counter.ticks);
}

TEST(StateMachineFramework, DispatchesOnlyCommandEdges) {
  Counter counter;
  Runtime machine(counter);
  int observed = -1;
  fsm_ctrl::fsm::DistinctCommandDispatcher<Runtime, ExampleRouter,
                                           RecordCommand>
      dispatcher(machine, ExampleRouter{}, RecordCommand{&observed});

  EXPECT_TRUE(dispatcher.update(1));
  EXPECT_TRUE(machine.isState<Running>());
  EXPECT_EQ(1, observed);
  EXPECT_FALSE(dispatcher.update(1));

  dispatcher.reset();
  EXPECT_TRUE(dispatcher.update(1));
  EXPECT_TRUE(dispatcher.update(0));
  EXPECT_TRUE(machine.isState<Idle>());
  EXPECT_EQ(0, dispatcher.previous());
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
