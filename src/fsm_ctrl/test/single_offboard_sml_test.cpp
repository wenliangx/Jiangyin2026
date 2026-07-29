#include <gtest/gtest.h>

#include <limits>
#include <string>
#include <vector>

#include <fsm_ctrl/single_offboard_sml.hpp>
#include <fsm_ctrl/single_offboard_sml_dispatch.hpp>

namespace {
using namespace fsm_ctrl::single_sml;

class FakeClock final : public Clock {
 public:
  double now() const override { return value; }
  double value{0.0};
};

class FakeAutopilot final : public AutopilotPort {
 public:
  bool requestOffboard() override {
    calls.push_back("offboard");
    return result;
  }
  bool requestArm() override {
    calls.push_back("arm");
    return result;
  }
  bool requestDisarm() override {
    calls.push_back("disarm");
    return result;
  }
  bool result{true};
  std::vector<std::string> calls;
};

class FakeSetpoint final : public SetpointPort {
 public:
  void publishPosition(const PositionSetpoint& value) override {
    positions.push_back(value);
  }
  void publishBodyRateThrust(const BodyRateThrust& value) override {
    body_rates.push_back(value);
  }
  void publishAttitude(const AttitudeSetpoint& value) override {
    attitudes.push_back(value);
  }
  void publishReferencePosition(const Vec3& value,
                                 const Quaternion& /*attitude*/) override {
    reference_positions.push_back(value);
  }
  void publishFeedbackPosition(const Vec3& value,
                                const Quaternion& /*attitude*/) override {
    feedback_positions.push_back(value);
  }
  void publishNmpcMonitor(const NmpcMonitor& value) override {
    monitors.push_back(value);
  }
  std::vector<PositionSetpoint> positions;
  std::vector<BodyRateThrust> body_rates;
  std::vector<AttitudeSetpoint> attitudes;
  std::vector<Vec3> reference_positions;
  std::vector<Vec3> feedback_positions;
  std::vector<NmpcMonitor> monitors;
};

class FakeNmpc final : public NmpcPort {
 public:
  bool solveHover(const TelemetrySnapshot& value,
                  BodyRateThrust& output) override {
    ++hover_calls;
    last_telemetry = value;
    output = hover_output;
    return hover_result;
  }
  bool solveTrack(const TelemetrySnapshot& value,
                  const std::vector<ReferencePoint>& points,
                  BodyRateThrust& output) override {
    ++track_calls;
    last_telemetry = value;
    last_horizon = points;
    output = track_output;
    return track_result;
  }
  int hover_calls{0};
  int track_calls{0};
  bool hover_result{true};
  bool track_result{true};
  BodyRateThrust hover_output{{0.1, 0.2, 0.3}, 0.4};
  BodyRateThrust track_output{{0.4, 0.5, 0.6}, 0.7};
  TelemetrySnapshot last_telemetry;
  std::vector<ReferencePoint> last_horizon;
};

class FakeReference final : public ReferenceProvider {
 public:
  void selectCommand(int command) override { selected_commands.push_back(command); }
  void reset() override { ++resets; }
  bool horizon(double value, std::vector<ReferencePoint>& output) override {
    ++horizon_calls;
    last_time = value;
    output = points;
    return result;
  }
  bool result{true};
  int resets{0};
  int horizon_calls{0};
  double last_time{0.0};
  std::vector<ReferencePoint> points{ReferencePoint{}};
  std::vector<int> selected_commands;
};

class FakeMission final : public MissionPort {
 public:
  void selectCommand(int command) override { selected_commands.push_back(command); }
  void reset() override {
    ++resets;
  }
  bool prepareSuper(double value, const TelemetrySnapshot& telemetry,
                    std::vector<ReferencePoint>& output) override {
    ++super_calls;
    last_time = value;
    last_telemetry = telemetry;
    output = super_points;
    return super_result;
  }
  bool prepareCoreSuperGoal(double value, const TelemetrySnapshot& telemetry,
                            const Vec3& goal,
                            std::vector<ReferencePoint>& output) override {
    ++core_super_calls;
    last_time = value;
    last_telemetry = telemetry;
    last_core_super_goal = goal;
    output = core_super_points;
    return core_super_result;
  }
  int resets{0};
  int super_calls{0};
  int core_super_calls{0};
  double last_time{0.0};
  TelemetrySnapshot last_telemetry;
  bool super_result{true};
  bool core_super_result{true};
  std::vector<ReferencePoint> super_points{ReferencePoint{}};
  std::vector<ReferencePoint> core_super_points{ReferencePoint{}};
  std::vector<int> selected_commands;
  Vec3 last_core_super_goal;
};

class FakeLanding final : public PrecisionLandingPort {
 public:
  void reset() override { ++resets; }
  void updateObservation(const LandingObservation& value) override {
    last_observation = value;
  }
  bool prepareLanding(double value, const TelemetrySnapshot& telemetry,
                      std::vector<ReferencePoint>& output) override {
    ++prepare_calls;
    last_time = value;
    last_telemetry = telemetry;
    output = points;
    return result;
  }
  bool isComplete() const override { return complete; }

  int resets{0};
  int prepare_calls{0};
  double last_time{0.0};
  bool result{true};
  bool complete{false};
  std::vector<ReferencePoint> points{ReferencePoint{}};
  TelemetrySnapshot last_telemetry;
  LandingObservation last_observation;
};

struct Fixture : testing::Test {
  Fixture()
      : context(clock, autopilot, setpoint, nmpc, reference, mission, landing),
        sm(context) {}

  static const char* StateName(int index) {
    switch (index) {
      case 0: return "Idle";
      case 1: return "LowThrust";
      case 2: return "PositionHold";
      case 3: return "NmpcHover";
      case 4: return "Landing";
      case 5: return "NmpcTrack";
      case 6: return "SuperTrack";
      case 7: return "Emergency";
      case 8: return "SafeNoop";
      default: return "Unknown";
    }
  }

  void SendCommandEventByStateIndex(StateMachine& machine, int index) {
    switch (index) {
      case 0: machine.process_event(OnCommand0{}); break;
      case 1: machine.process_event(OnCommand1{}); break;
      case 2: machine.process_event(OnCommand2{}); break;
      case 3: machine.process_event(OnCommand3{}); break;
      case 4: machine.process_event(OnCommand4{}); break;
      case 5: machine.process_event(OnCommand5{}); break;
      case 6: machine.process_event(OnCommand6{}); break;
      case 7: machine.process_event(OnCommand9{}); break;
      case 8: machine.process_event(OnCommand7{}); break;
      default: ADD_FAILURE() << "Bad state index " << index; break;
    }
  }

  void ExpectStateByIndex(StateMachine& machine, int index) {
    switch (index) {
      case 0: EXPECT_TRUE(machine.is(boost::sml::state<Idle>)); break;
      case 1: EXPECT_TRUE(machine.is(boost::sml::state<LowThrust>)); break;
      case 2: EXPECT_TRUE(machine.is(boost::sml::state<PositionHold>)); break;
      case 3: EXPECT_TRUE(machine.is(boost::sml::state<NmpcHover>)); break;
      case 4: EXPECT_TRUE(machine.is(boost::sml::state<Landing>)); break;
      case 5: EXPECT_TRUE(machine.is(boost::sml::state<NmpcTrack>)); break;
      case 6: EXPECT_TRUE(machine.is(boost::sml::state<SuperTrack>)); break;
      case 7: EXPECT_TRUE(machine.is(boost::sml::state<Emergency>)); break;
      case 8: EXPECT_TRUE(machine.is(boost::sml::state<SafeNoop>)); break;
      default: ADD_FAILURE() << "Bad state index " << index; break;
    }
  }

  void ClearOutputs() {
    autopilot.calls.clear();
    setpoint.positions.clear();
    setpoint.body_rates.clear();
    setpoint.attitudes.clear();
    setpoint.reference_positions.clear();
    setpoint.feedback_positions.clear();
    setpoint.monitors.clear();
    nmpc.hover_calls = 0;
    nmpc.track_calls = 0;
    nmpc.last_horizon.clear();
    reference.horizon_calls = 0;
    mission.super_calls = 0;
    mission.core_super_calls = 0;
    landing.prepare_calls = 0;
  }

  void SetOffboardAndArmed() {
    context.telemetry.mode = "OFFBOARD";
    context.telemetry.armed = true;
  }

  FakeClock clock;
  FakeAutopilot autopilot;
  FakeSetpoint setpoint;
  FakeNmpc nmpc;
  FakeReference reference;
  FakeMission mission;
  FakeLanding landing;
  Context context;
  StateMachine sm;
};

TEST_F(Fixture, InitialAndEveryCommandEventReachExpectedState) {
  EXPECT_TRUE(sm.is(boost::sml::state<Idle>));
  sm.process_event(OnCommand1{});
  EXPECT_TRUE(sm.is(boost::sml::state<LowThrust>));
  sm.process_event(OnCommand2{});
  EXPECT_TRUE(sm.is(boost::sml::state<PositionHold>));
  sm.process_event(OnCommand3{});
  EXPECT_TRUE(sm.is(boost::sml::state<NmpcHover>));
  sm.process_event(OnCommand4{});
  EXPECT_TRUE(sm.is(boost::sml::state<Landing>));
  sm.process_event(OnCommand5{});
  EXPECT_TRUE(sm.is(boost::sml::state<NmpcTrack>));
  sm.process_event(OnCommand6{});
  EXPECT_TRUE(sm.is(boost::sml::state<SuperTrack>));
  sm.process_event(OnCommand9{});
  EXPECT_TRUE(sm.is(boost::sml::state<Emergency>));
  sm.process_event(OnCommand7{});
  EXPECT_TRUE(sm.is(boost::sml::state<SafeNoop>));
  sm.process_event(OnCommand8{});
  EXPECT_TRUE(sm.is(boost::sml::state<SafeNoop>));
  sm.process_event(OnCommand0{});
  EXPECT_TRUE(sm.is(boost::sml::state<Idle>));
}

TEST_F(Fixture, EveryCommandEventWorksFromEveryState) {
  for (int source = 0; source < 9; ++source) {
    for (int target = 0; target < 9; ++target) {
      SCOPED_TRACE(std::string(StateName(source)) + " -> " +
                   StateName(target));
      StateMachine machine(context);
      SendCommandEventByStateIndex(machine, source);
      ExpectStateByIndex(machine, source);
      SendCommandEventByStateIndex(machine, target);
      ExpectStateByIndex(machine, target);
    }
  }
}

TEST_F(Fixture, DirectOnCommand5ResetsOnceFromEverySourceState) {
  for (int source = 0; source < 9; ++source) {
    SCOPED_TRACE(StateName(source));
    StateMachine machine(context);
    SendCommandEventByStateIndex(machine, source);
    const int resets_after_source_command = reference.resets;
    SendCommandEventByStateIndex(machine, 5);
    EXPECT_EQ(resets_after_source_command + 1, reference.resets);
    ExpectStateByIndex(machine, 5);
  }
}

TEST_F(Fixture, DirectOnCommand4ResetsOnceFromEverySourceState) {
  for (int source = 0; source < 9; ++source) {
    SCOPED_TRACE(StateName(source));
    StateMachine machine(context);
    SendCommandEventByStateIndex(machine, source);
    const int resets_after_source_command = landing.resets;
    SendCommandEventByStateIndex(machine, 4);
    EXPECT_EQ(resets_after_source_command + 1, landing.resets);
    ExpectStateByIndex(machine, 4);
  }
}

TEST_F(Fixture, CommandDispatcherMapsCoreAndSafeNoopCommands) {
  CommandDispatcher dispatcher(sm, &reference);
  EXPECT_TRUE(dispatcher.update(0));
  EXPECT_TRUE(sm.is(boost::sml::state<Idle>));
  EXPECT_TRUE(dispatcher.update(1));
  EXPECT_TRUE(sm.is(boost::sml::state<LowThrust>));
  EXPECT_TRUE(dispatcher.update(2));
  EXPECT_TRUE(sm.is(boost::sml::state<PositionHold>));
  EXPECT_TRUE(dispatcher.update(3));
  EXPECT_TRUE(sm.is(boost::sml::state<NmpcHover>));
  EXPECT_TRUE(dispatcher.update(4));
  EXPECT_TRUE(sm.is(boost::sml::state<Landing>));
  EXPECT_TRUE(dispatcher.update(5));
  EXPECT_TRUE(sm.is(boost::sml::state<NmpcTrack>));
  EXPECT_TRUE(dispatcher.update(6));
  EXPECT_TRUE(sm.is(boost::sml::state<SuperTrack>));
  EXPECT_TRUE(dispatcher.update(7));
  EXPECT_TRUE(sm.is(boost::sml::state<SafeNoop>));
  EXPECT_TRUE(dispatcher.update(8));
  EXPECT_TRUE(sm.is(boost::sml::state<SafeNoop>));
  EXPECT_TRUE(dispatcher.update(9));
  EXPECT_TRUE(sm.is(boost::sml::state<Emergency>));
  for (const int command : {-1, 42}) {
    EXPECT_TRUE(dispatcher.update(command));
    EXPECT_TRUE(sm.is(boost::sml::state<SafeNoop>));
  }
  const std::vector<int> expected_commands{0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
                                           -1, 42};
  EXPECT_EQ(expected_commands, reference.selected_commands);
}

TEST_F(Fixture, CommandDispatcherSuppressesRepeatedCoreAndUnknownCommands) {
  CommandDispatcher dispatcher(sm, &reference);
  EXPECT_TRUE(dispatcher.update(6));
  EXPECT_TRUE(sm.is(boost::sml::state<SuperTrack>));
  EXPECT_FALSE(dispatcher.update(6));
  EXPECT_TRUE(sm.is(boost::sml::state<SuperTrack>));

  EXPECT_TRUE(dispatcher.update(7));
  EXPECT_TRUE(sm.is(boost::sml::state<SafeNoop>));
  EXPECT_FALSE(dispatcher.update(7));

  EXPECT_TRUE(dispatcher.update(8));
  EXPECT_TRUE(sm.is(boost::sml::state<SafeNoop>));
  EXPECT_FALSE(dispatcher.update(8));

  EXPECT_TRUE(dispatcher.update(42));
  EXPECT_TRUE(sm.is(boost::sml::state<SafeNoop>));
  EXPECT_FALSE(dispatcher.update(42));

  EXPECT_TRUE(dispatcher.update(5));
  EXPECT_TRUE(sm.is(boost::sml::state<NmpcTrack>));
  EXPECT_EQ(1, reference.resets);
  EXPECT_FALSE(dispatcher.update(5));
  EXPECT_EQ(1, reference.resets);

  EXPECT_TRUE(dispatcher.update(-1));
  EXPECT_TRUE(sm.is(boost::sml::state<SafeNoop>));
  EXPECT_FALSE(dispatcher.update(-1));
  const std::vector<int> expected_commands{6, 7, 8, 42, 5, -1};
  EXPECT_EQ(expected_commands, reference.selected_commands);
}

TEST_F(Fixture, CommandDispatcherProcessesFirstIntMinCommand) {
  CommandDispatcher dispatcher(sm, &reference);
  EXPECT_TRUE(dispatcher.update(std::numeric_limits<int>::min()));
  EXPECT_TRUE(sm.is(boost::sml::state<SafeNoop>));
  EXPECT_FALSE(dispatcher.update(std::numeric_limits<int>::min()));
}

TEST_F(Fixture, RepeatedCommandDoesNotReenterTrack) {
  CommandDispatcher dispatcher(sm, &reference);
  EXPECT_TRUE(dispatcher.update(5));
  EXPECT_EQ(1, reference.resets);
  EXPECT_FALSE(dispatcher.update(5));
  EXPECT_EQ(1, reference.resets);
  EXPECT_TRUE(dispatcher.update(0));
  EXPECT_TRUE(dispatcher.update(5));
  EXPECT_EQ(2, reference.resets);
}

TEST_F(Fixture, IdleAndSafeNoopTicksHaveNoOutput) {
  sm.process_event(Tick{});
  sm.process_event(OnUnsupportedCommand{});
  sm.process_event(Tick{});
  EXPECT_TRUE(autopilot.calls.empty());
  EXPECT_TRUE(setpoint.positions.empty());
  EXPECT_TRUE(setpoint.body_rates.empty());
  EXPECT_TRUE(setpoint.attitudes.empty());
}

TEST_F(Fixture, LowThrustTickPublishesLegacyMessage) {
  SetOffboardAndArmed();
  context.config.low_thrust = 0.123;
  sm.process_event(OnCommand1{});
  sm.process_event(Tick{});
  ASSERT_EQ(1u, setpoint.body_rates.size());
  EXPECT_DOUBLE_EQ(context.config.low_thrust, setpoint.body_rates[0].thrust);
  EXPECT_DOUBLE_EQ(0.0, setpoint.body_rates[0].body_rate.x);
  EXPECT_DOUBLE_EQ(0.0, setpoint.body_rates[0].body_rate.y);
  EXPECT_DOUBLE_EQ(0.0, setpoint.body_rates[0].body_rate.z);
  EXPECT_TRUE(setpoint.positions.empty());
  EXPECT_TRUE(setpoint.attitudes.empty());
}

TEST_F(Fixture, PositionHoldTickPublishesLegacyTarget) {
  SetOffboardAndArmed();
  context.config.position_hold_z = 1.75;
  sm.process_event(OnCommand2{});
  sm.process_event(Tick{});
  ASSERT_EQ(1u, setpoint.positions.size());
  EXPECT_DOUBLE_EQ(0.0, setpoint.positions[0].position.x);
  EXPECT_DOUBLE_EQ(0.0, setpoint.positions[0].position.y);
  EXPECT_DOUBLE_EQ(1.75, setpoint.positions[0].position.z);
  EXPECT_DOUBLE_EQ(0.0, setpoint.positions[0].yaw);
  EXPECT_TRUE(setpoint.body_rates.empty());
  EXPECT_TRUE(setpoint.attitudes.empty());
}

TEST_F(Fixture, OffboardArmSharedLogicRunsOnlyInActiveOffboardStates) {
  for (const int state : {1, 2, 3, 4, 5, 6}) {
    SCOPED_TRACE(StateName(state));
    ClearOutputs();
    context.telemetry.mode = "MANUAL";
    context.telemetry.armed = false;
    context.last_service_request = 0.0;
    clock.value = 5.001;
    SendCommandEventByStateIndex(sm, state);
    sm.process_event(Tick{});
    ASSERT_FALSE(autopilot.calls.empty());
    EXPECT_EQ("offboard", autopilot.calls[0]);
  }

  for (const int state : {0, 7, 8}) {
    SCOPED_TRACE(StateName(state));
    ClearOutputs();
    context.telemetry.mode = "MANUAL";
    context.telemetry.armed = false;
    context.landing_reached = false;
    context.last_service_request = 0.0;
    clock.value = 5.001;
    SendCommandEventByStateIndex(sm, state);
    sm.process_event(Tick{});
    EXPECT_TRUE(autopilot.calls.empty());
  }
}

TEST_F(Fixture, ActiveOffboardStatesDoNotRequestServicesWhenAlreadyReady) {
  for (const int state : {1, 2, 3, 4, 5, 6}) {
    SCOPED_TRACE(StateName(state));
    ClearOutputs();
    SetOffboardAndArmed();
    clock.value = 100.0;
    SendCommandEventByStateIndex(sm, state);
    sm.process_event(Tick{});
    EXPECT_TRUE(autopilot.calls.empty());
  }
}

TEST_F(Fixture, ServiceCooldownUsesStrictFiveSecondBoundary) {
  sm.process_event(OnCommand1{});
  clock.value = 4.999;
  sm.process_event(Tick{});
  clock.value = 5.000;
  sm.process_event(Tick{});
  EXPECT_TRUE(autopilot.calls.empty());
  clock.value = 5.001;
  sm.process_event(Tick{});
  ASSERT_EQ(1u, autopilot.calls.size());
  EXPECT_EQ("offboard", autopilot.calls[0]);
}

TEST_F(Fixture, FailedRequestAlsoRefreshesSharedCooldown) {
  autopilot.result = false;
  sm.process_event(OnCommand1{});
  clock.value = 5.001;
  sm.process_event(Tick{});
  clock.value = 10.001;
  sm.process_event(Tick{});
  ASSERT_EQ(1u, autopilot.calls.size());
  clock.value = 10.002;
  sm.process_event(Tick{});
  ASSERT_EQ(2u, autopilot.calls.size());
  EXPECT_EQ("offboard", autopilot.calls[0]);
  EXPECT_EQ("offboard", autopilot.calls[1]);
}

TEST_F(Fixture, OffboardThenArmRequestsAreOrderedAndShareTimestamp) {
  sm.process_event(OnCommand2{});
  clock.value = 5.001;
  sm.process_event(Tick{});
  context.telemetry.mode = "OFFBOARD";
  clock.value = 10.001;
  sm.process_event(Tick{});
  ASSERT_EQ(1u, autopilot.calls.size());
  clock.value = 10.002;
  sm.process_event(Tick{});
  ASSERT_EQ(2u, autopilot.calls.size());
  EXPECT_EQ("offboard", autopilot.calls[0]);
  EXPECT_EQ("arm", autopilot.calls[1]);
}

TEST_F(Fixture, NmpcHoverForwardsTelemetryAndFiniteOutput) {
  context.telemetry.position = {1.0, 2.0, 3.0};
  context.telemetry.velocity = {4.0, 5.0, 6.0};
  context.telemetry.attitude = {0.7, 0.1, 0.2, 0.3};
  SetOffboardAndArmed();
  sm.process_event(OnCommand3{});
  sm.process_event(Tick{});
  EXPECT_EQ(1, nmpc.hover_calls);
  EXPECT_DOUBLE_EQ(1.0, nmpc.last_telemetry.position.x);
  EXPECT_DOUBLE_EQ(5.0, nmpc.last_telemetry.velocity.y);
  EXPECT_DOUBLE_EQ(0.2, nmpc.last_telemetry.attitude.y);
  ASSERT_EQ(1u, setpoint.body_rates.size());
  EXPECT_DOUBLE_EQ(0.1, setpoint.body_rates[0].body_rate.x);
  EXPECT_DOUBLE_EQ(0.2, setpoint.body_rates[0].body_rate.y);
  EXPECT_DOUBLE_EQ(0.3, setpoint.body_rates[0].body_rate.z);
  EXPECT_DOUBLE_EQ(0.4, setpoint.body_rates[0].thrust);
}

TEST_F(Fixture, NmpcHoverRejectsFailureAndNonFiniteOutput) {
  context.telemetry.mode = "OFFBOARD";
  context.telemetry.armed = true;
  sm.process_event(OnCommand3{});
  nmpc.hover_result = false;
  sm.process_event(Tick{});
  nmpc.hover_result = true;
  nmpc.hover_output.thrust = std::numeric_limits<double>::quiet_NaN();
  sm.process_event(Tick{});
  EXPECT_TRUE(setpoint.body_rates.empty());
}

TEST_F(Fixture, NmpcHoverRejectsAnyNonFiniteOutputField) {
  SetOffboardAndArmed();
  sm.process_event(OnCommand3{});
  for (int field = 0; field < 4; ++field) {
    SCOPED_TRACE(field);
    ClearOutputs();
    nmpc.hover_output = BodyRateThrust{{0.1, 0.2, 0.3}, 0.4};
    if (field == 0) {
      nmpc.hover_output.body_rate.x =
          std::numeric_limits<double>::quiet_NaN();
    } else if (field == 1) {
      nmpc.hover_output.body_rate.y =
          std::numeric_limits<double>::infinity();
    } else if (field == 2) {
      nmpc.hover_output.body_rate.z =
          -std::numeric_limits<double>::infinity();
    } else {
      nmpc.hover_output.thrust = std::numeric_limits<double>::quiet_NaN();
    }
    sm.process_event(Tick{});
    EXPECT_EQ(1, nmpc.hover_calls);
    EXPECT_TRUE(setpoint.body_rates.empty());
  }
}

TEST_F(Fixture, TrackUsesReferenceAndResetsExactlyOnReentry) {
  SetOffboardAndArmed();
  reference.points[0].position.x = 7.0;
  reference.points[0].velocity.y = 8.0;
  context.telemetry.position = {1.0, 2.0, 3.0};
  context.telemetry.velocity = {4.0, 5.0, 6.0};
  sm.process_event(OnCommand5{});
  EXPECT_EQ(1, reference.resets);
  clock.value = 3.0;
  sm.process_event(Tick{});
  EXPECT_EQ(1, reference.horizon_calls);
  EXPECT_EQ(1, nmpc.track_calls);
  ASSERT_EQ(1u, nmpc.last_horizon.size());
  EXPECT_DOUBLE_EQ(7.0, nmpc.last_horizon[0].position.x);
  EXPECT_DOUBLE_EQ(8.0, nmpc.last_horizon[0].velocity.y);
  EXPECT_DOUBLE_EQ(1.0, nmpc.last_telemetry.position.x);
  EXPECT_DOUBLE_EQ(5.0, nmpc.last_telemetry.velocity.y);
  EXPECT_DOUBLE_EQ(3.0, reference.last_time);
  ASSERT_EQ(1u, setpoint.body_rates.size());
  EXPECT_DOUBLE_EQ(0.4, setpoint.body_rates[0].body_rate.x);
  EXPECT_DOUBLE_EQ(0.5, setpoint.body_rates[0].body_rate.y);
  EXPECT_DOUBLE_EQ(0.6, setpoint.body_rates[0].body_rate.z);
  EXPECT_DOUBLE_EQ(0.7, setpoint.body_rates[0].thrust);
  ASSERT_EQ(1u, setpoint.monitors.size());
  ASSERT_EQ(1u, setpoint.monitors[0].references.size());
  EXPECT_DOUBLE_EQ(7.0, setpoint.monitors[0].references[0].position.x);
  EXPECT_DOUBLE_EQ(8.0, setpoint.monitors[0].references[0].velocity.y);
  EXPECT_DOUBLE_EQ(1.0, setpoint.monitors[0].feedback.position.x);
  EXPECT_DOUBLE_EQ(5.0, setpoint.monitors[0].feedback.velocity.y);
  EXPECT_DOUBLE_EQ(0.4, setpoint.monitors[0].target.body_rate.x);
  EXPECT_DOUBLE_EQ(0.7, setpoint.monitors[0].target.thrust);
  sm.process_event(OnCommand0{});
  sm.process_event(OnCommand5{});
  EXPECT_EQ(2, reference.resets);
}

TEST_F(Fixture, Cmd6UsesSuperMissionReferenceAndNmpcMonitor) {
  SetOffboardAndArmed();
  context.telemetry.position = {1.0, 2.0, 3.0};
  context.telemetry.velocity = {4.0, 5.0, 6.0};
  mission.super_points = {ReferencePoint{}};
  mission.super_points[0].position = {16.0, 26.0, 36.0};
  mission.super_points[0].velocity = {6.0, 12.0, 18.0};
  nmpc.track_output = BodyRateThrust{{0.6, 1.2, 1.8}, 0.46};

  sm.process_event(OnCommand6{});
  EXPECT_EQ(1, mission.resets);
  clock.value = 6.0;
  sm.process_event(Tick{});

  EXPECT_EQ(1, mission.super_calls);
  EXPECT_EQ(0, reference.horizon_calls);
  EXPECT_EQ(1, nmpc.track_calls);
  ASSERT_EQ(1u, nmpc.last_horizon.size());
  EXPECT_DOUBLE_EQ(16.0, nmpc.last_horizon[0].position.x);
  EXPECT_DOUBLE_EQ(12.0, nmpc.last_horizon[0].velocity.y);
  ASSERT_EQ(1u, setpoint.body_rates.size());
  EXPECT_DOUBLE_EQ(0.6, setpoint.body_rates[0].body_rate.x);
  EXPECT_DOUBLE_EQ(1.2, setpoint.body_rates[0].body_rate.y);
  EXPECT_DOUBLE_EQ(1.8, setpoint.body_rates[0].body_rate.z);
  EXPECT_DOUBLE_EQ(0.46, setpoint.body_rates[0].thrust);
  ASSERT_EQ(1u, setpoint.feedback_positions.size());
  ASSERT_EQ(1u, setpoint.reference_positions.size());
  EXPECT_DOUBLE_EQ(1.0, setpoint.feedback_positions[0].x);
  EXPECT_DOUBLE_EQ(16.0, setpoint.reference_positions[0].x);
  ASSERT_EQ(1u, setpoint.monitors.size());
  EXPECT_DOUBLE_EQ(16.0, setpoint.monitors[0].references[0].position.x);
  EXPECT_DOUBLE_EQ(1.0, setpoint.monitors[0].feedback.position.x);
  EXPECT_DOUBLE_EQ(4.0, setpoint.monitors[0].feedback.velocity.x);
  EXPECT_DOUBLE_EQ(0.46, setpoint.monitors[0].target.thrust);
}

TEST_F(Fixture, Cmd6RejectsMissingReferenceSolveFailureAndBadOutput) {
  SetOffboardAndArmed();
  sm.process_event(OnCommand6{});

  mission.super_result = false;
  sm.process_event(Tick{});
  EXPECT_EQ(0, nmpc.track_calls);
  EXPECT_TRUE(setpoint.body_rates.empty());
  EXPECT_TRUE(setpoint.monitors.empty());

  mission.super_result = true;
  mission.super_points.clear();
  sm.process_event(Tick{});
  EXPECT_EQ(0, nmpc.track_calls);

  mission.super_points = {ReferencePoint{}};
  nmpc.track_result = false;
  sm.process_event(Tick{});
  EXPECT_EQ(1, nmpc.track_calls);
  EXPECT_TRUE(setpoint.body_rates.empty());
  EXPECT_TRUE(setpoint.monitors.empty());
  EXPECT_EQ(1u, setpoint.reference_positions.size());
  EXPECT_EQ(1u, setpoint.feedback_positions.size());

  ClearOutputs();
  mission.super_points = {ReferencePoint{}};
  nmpc.track_result = true;
  nmpc.track_output.thrust = std::numeric_limits<double>::quiet_NaN();
  sm.process_event(Tick{});
  EXPECT_EQ(1, nmpc.track_calls);
  EXPECT_TRUE(setpoint.body_rates.empty());
  EXPECT_TRUE(setpoint.monitors.empty());
  EXPECT_EQ(1u, setpoint.reference_positions.size());
  EXPECT_EQ(1u, setpoint.feedback_positions.size());
}

TEST_F(Fixture, TrackRejectsMissingReferenceAndBadNmpcOutput) {
  context.telemetry.mode = "OFFBOARD";
  context.telemetry.armed = true;
  sm.process_event(OnCommand5{});
  reference.result = false;
  sm.process_event(Tick{});
  reference.result = true;
  reference.points.clear();
  sm.process_event(Tick{});
  reference.points.push_back(ReferencePoint{});
  nmpc.track_output.body_rate.x =
      std::numeric_limits<double>::infinity();
  sm.process_event(Tick{});
  EXPECT_TRUE(setpoint.body_rates.empty());
  EXPECT_TRUE(setpoint.monitors.empty());
}

TEST_F(Fixture, TrackDoesNotCallNmpcWithoutValidReference) {
  SetOffboardAndArmed();
  sm.process_event(OnCommand5{});

  reference.result = false;
  sm.process_event(Tick{});
  EXPECT_EQ(1, reference.horizon_calls);
  EXPECT_EQ(0, nmpc.track_calls);
  EXPECT_TRUE(setpoint.body_rates.empty());
  EXPECT_TRUE(setpoint.monitors.empty());

  ClearOutputs();
  reference.result = true;
  reference.points.clear();
  sm.process_event(Tick{});
  EXPECT_EQ(1, reference.horizon_calls);
  EXPECT_EQ(0, nmpc.track_calls);
  EXPECT_TRUE(setpoint.body_rates.empty());
  EXPECT_TRUE(setpoint.monitors.empty());
}

TEST_F(Fixture, TrackRejectsNmpcSolveFailure) {
  SetOffboardAndArmed();
  sm.process_event(OnCommand5{});
  nmpc.track_result = false;
  sm.process_event(Tick{});
  EXPECT_EQ(1, reference.horizon_calls);
  EXPECT_EQ(1, nmpc.track_calls);
  EXPECT_TRUE(setpoint.body_rates.empty());
  EXPECT_TRUE(setpoint.monitors.empty());
}

TEST_F(Fixture, TrackRejectsAnyNonFiniteOutputField) {
  SetOffboardAndArmed();
  sm.process_event(OnCommand5{});
  for (int field = 0; field < 4; ++field) {
    SCOPED_TRACE(field);
    ClearOutputs();
    reference.points = {ReferencePoint{}};
    nmpc.track_output = BodyRateThrust{{0.4, 0.5, 0.6}, 0.7};
    if (field == 0) {
      nmpc.track_output.body_rate.x =
          std::numeric_limits<double>::quiet_NaN();
    } else if (field == 1) {
      nmpc.track_output.body_rate.y =
          std::numeric_limits<double>::infinity();
    } else if (field == 2) {
      nmpc.track_output.body_rate.z =
          -std::numeric_limits<double>::infinity();
    } else {
      nmpc.track_output.thrust = std::numeric_limits<double>::quiet_NaN();
    }
    sm.process_event(Tick{});
    EXPECT_EQ(1, nmpc.track_calls);
    EXPECT_TRUE(setpoint.body_rates.empty());
    EXPECT_TRUE(setpoint.monitors.empty());
  }
}

TEST_F(Fixture, LandingUsesPrecisionHorizonAndNmpcMonitor) {
  SetOffboardAndArmed();
  context.telemetry.position = {2.0, 3.0, 0.11};
  landing.points[0].position = {2.1, 2.9, 0.10};
  nmpc.track_output = BodyRateThrust{{0.2, 0.3, 0.4}, 0.5};
  clock.value = 12.0;

  sm.process_event(OnCommand4{});
  EXPECT_EQ(1, landing.resets);
  sm.process_event(Tick{});

  EXPECT_EQ(1, landing.prepare_calls);
  EXPECT_DOUBLE_EQ(12.0, landing.last_time);
  EXPECT_DOUBLE_EQ(2.0, landing.last_telemetry.position.x);
  EXPECT_EQ(1, nmpc.track_calls);
  ASSERT_EQ(1u, nmpc.last_horizon.size());
  EXPECT_DOUBLE_EQ(2.1, nmpc.last_horizon[0].position.x);
  ASSERT_EQ(1u, setpoint.body_rates.size());
  EXPECT_DOUBLE_EQ(0.2, setpoint.body_rates[0].body_rate.x);
  EXPECT_DOUBLE_EQ(0.5, setpoint.body_rates[0].thrust);
  ASSERT_EQ(1u, setpoint.monitors.size());
  EXPECT_DOUBLE_EQ(2.1, setpoint.monitors[0].references[0].position.x);
  EXPECT_FALSE(context.landing_reached);
}

TEST_F(Fixture, LandingRejectsMissingReferenceSolveFailureAndBadOutput) {
  SetOffboardAndArmed();
  sm.process_event(OnCommand4{});

  landing.result = false;
  sm.process_event(Tick{});
  EXPECT_EQ(1, landing.prepare_calls);
  EXPECT_EQ(0, nmpc.track_calls);
  EXPECT_TRUE(setpoint.body_rates.empty());
  EXPECT_TRUE(setpoint.monitors.empty());

  ClearOutputs();
  landing.result = true;
  landing.points.clear();
  sm.process_event(Tick{});
  EXPECT_EQ(1, landing.prepare_calls);
  EXPECT_EQ(0, nmpc.track_calls);

  ClearOutputs();
  landing.points = {ReferencePoint{}};
  nmpc.track_result = false;
  sm.process_event(Tick{});
  EXPECT_EQ(1, nmpc.track_calls);
  EXPECT_TRUE(setpoint.body_rates.empty());

  ClearOutputs();
  nmpc.track_result = true;
  nmpc.track_output.thrust = std::numeric_limits<double>::quiet_NaN();
  sm.process_event(Tick{});
  EXPECT_EQ(1, nmpc.track_calls);
  EXPECT_TRUE(setpoint.body_rates.empty());
  EXPECT_TRUE(setpoint.monitors.empty());
}

TEST_F(Fixture, LandingCompletionKeepsLegacyLatchAndDisarmSemantics) {
  SetOffboardAndArmed();
  context.telemetry.position = {2.0, 3.0, 0.11};
  sm.process_event(OnCommand4{});
  landing.complete = true;
  sm.process_event(Tick{});
  EXPECT_TRUE(context.landing_reached);
  sm.process_event(Tick{});
  EXPECT_TRUE(autopilot.calls.empty());
  context.telemetry.mode = "POSCTL";
  sm.process_event(Tick{});
  ASSERT_EQ(1u, autopilot.calls.size());
  EXPECT_EQ("disarm", autopilot.calls[0]);
}

TEST_F(Fixture, LandingHeightToleranceCanStillLatchAsFallback) {
  SetOffboardAndArmed();
  context.config.landing_reference_z = 0.05;
  context.config.landing_tolerance_z = 0.05;
  context.telemetry.position = {4.0, 5.0, 0.10};
  sm.process_event(OnCommand4{});

  sm.process_event(Tick{});
  context.telemetry.position.z = 0.06;
  sm.process_event(Tick{});
  EXPECT_TRUE(context.landing_reached);
}

TEST_F(Fixture, LandingDisarmsOnlyWhenLatchedNonOffboardAndArmed) {
  sm.process_event(OnCommand4{});
  context.landing_reached = true;

  context.telemetry.mode = "OFFBOARD";
  context.telemetry.armed = true;
  sm.process_event(Tick{});
  EXPECT_TRUE(autopilot.calls.empty());

  context.telemetry.mode = "POSCTL";
  context.telemetry.armed = false;
  sm.process_event(Tick{});
  EXPECT_TRUE(autopilot.calls.empty());

  context.telemetry.armed = true;
  sm.process_event(Tick{});
  ASSERT_EQ(1u, autopilot.calls.size());
  EXPECT_EQ("disarm", autopilot.calls[0]);
}

TEST_F(Fixture, CoreFlightMachineRunsCoreStates) {
  CoreFlightStateMachine machine(context);
  EXPECT_TRUE(machine.is(boost::sml::state<Idle>));

  machine.process_event(OnCommand1{});
  EXPECT_TRUE(machine.is(boost::sml::state<ArmOnly>));
  clock.value = 5.1;
  machine.process_event(Tick{});
  EXPECT_FALSE(autopilot.calls.empty());
  EXPECT_TRUE(setpoint.body_rates.empty());

  ClearOutputs();
  SetOffboardAndArmed();
  context.telemetry.position = {2.0, 3.0, 0.2};
  machine.process_event(OnCommand2{});
  EXPECT_TRUE(machine.is(boost::sml::state<CoreHover>));
  machine.process_event(Tick{});
  EXPECT_EQ(1, nmpc.track_calls);
  ASSERT_EQ(1u, setpoint.body_rates.size());
  ASSERT_FALSE(nmpc.last_horizon.empty());
  EXPECT_DOUBLE_EQ(2.0, nmpc.last_horizon.front().position.x);
  EXPECT_DOUBLE_EQ(3.0, nmpc.last_horizon.front().position.y);
  EXPECT_DOUBLE_EQ(1.0, nmpc.last_horizon.front().position.z);

  ClearOutputs();
  machine.process_event(OnCommand9{});
  machine.process_event(Tick{});
  ASSERT_EQ(1u, setpoint.attitudes.size());
  EXPECT_DOUBLE_EQ(context.config.hover_thrust - 0.03,
                   setpoint.attitudes[0].thrust);
}

TEST_F(Fixture, MissionMachineMapsCommandsToArmHoverSuperLanding) {
  MissionStateMachine machine(context);
  MissionCommandDispatcher dispatcher(machine, &reference, &mission);
  EXPECT_TRUE(machine.is(boost::sml::state<Idle>));

  EXPECT_TRUE(dispatcher.update(1));
  EXPECT_TRUE(machine.is(boost::sml::state<ArmOnly>));

  EXPECT_TRUE(dispatcher.update(2));
  EXPECT_TRUE(machine.is(boost::sml::state<NmpcHover>));

  EXPECT_TRUE(dispatcher.update(3));
  EXPECT_TRUE(machine.is(boost::sml::state<SuperTrack>));
  EXPECT_EQ(1, mission.resets);

  EXPECT_TRUE(dispatcher.update(4));
  EXPECT_TRUE(machine.is(boost::sml::state<Landing>));
  EXPECT_EQ(1, landing.resets);

  EXPECT_TRUE(dispatcher.update(5));
  EXPECT_TRUE(machine.is(boost::sml::state<SafeNoop>));
}

TEST_F(Fixture, SegmentedMissionMachineMapsCommandsToSegmentsAndLanding) {
  SegmentedMissionStateMachine machine(context);
  SegmentedMissionCommandDispatcher dispatcher(machine, &reference, &mission);
  EXPECT_TRUE(machine.is(boost::sml::state<Idle>));

  EXPECT_TRUE(dispatcher.update(1));
  EXPECT_TRUE(machine.is(boost::sml::state<ArmOnly>));

  EXPECT_TRUE(dispatcher.update(2));
  EXPECT_TRUE(machine.is(boost::sml::state<NmpcHover>));

  EXPECT_TRUE(dispatcher.update(3));
  EXPECT_TRUE(machine.is(boost::sml::state<SuperSegment1>));
  EXPECT_EQ(1, mission.resets);

  EXPECT_TRUE(dispatcher.update(4));
  EXPECT_TRUE(machine.is(boost::sml::state<SuperSegment2>));
  EXPECT_EQ(2, mission.resets);

  EXPECT_TRUE(dispatcher.update(5));
  EXPECT_TRUE(machine.is(boost::sml::state<SuperSegment3>));
  EXPECT_EQ(3, mission.resets);

  EXPECT_TRUE(dispatcher.update(6));
  EXPECT_TRUE(machine.is(boost::sml::state<Landing>));
  EXPECT_EQ(1, landing.resets);
}

TEST_F(Fixture, CoreFlightCmd3PublishesSuperControlAndLandingDebug) {
  CoreFlightStateMachine machine(context);
  SetOffboardAndArmed();
  mission.core_super_points = {ReferencePoint{}};
  mission.core_super_points.front().position = {1.0, 0.0, 1.0};
  landing.points = {ReferencePoint{}};
  landing.points.front().position = {4.0, 5.0, 0.8};
  landing.result = true;

  machine.process_event(OnCommand3{});
  EXPECT_TRUE(machine.is(boost::sml::state<CoreSuperLanding>));
  machine.process_event(Tick{});

  EXPECT_EQ(1, mission.core_super_calls);
  EXPECT_DOUBLE_EQ(1.0, mission.last_core_super_goal.x);
  EXPECT_DOUBLE_EQ(0.0, mission.last_core_super_goal.y);
  EXPECT_DOUBLE_EQ(1.0, mission.last_core_super_goal.z);
  EXPECT_EQ(1, landing.prepare_calls);
  EXPECT_EQ(2, nmpc.track_calls);
  EXPECT_EQ(2u, setpoint.monitors.size());
  ASSERT_EQ(1u, setpoint.body_rates.size());
  EXPECT_DOUBLE_EQ(nmpc.track_output.thrust, setpoint.body_rates[0].thrust);
}

TEST_F(Fixture, CoreFlightCmd4PublishesLandingControl) {
  CoreFlightStateMachine machine(context);
  SetOffboardAndArmed();
  landing.points = {ReferencePoint{}};
  landing.points.front().position = {2.0, 2.0, 0.5};
  landing.result = true;

  machine.process_event(OnCommand4{});
  EXPECT_TRUE(machine.is(boost::sml::state<CoreLanding>));
  machine.process_event(Tick{});

  EXPECT_EQ(1, landing.prepare_calls);
  EXPECT_EQ(1, nmpc.track_calls);
  EXPECT_EQ(1u, setpoint.monitors.size());
  ASSERT_EQ(1u, setpoint.body_rates.size());
  EXPECT_DOUBLE_EQ(nmpc.track_output.thrust, setpoint.body_rates[0].thrust);
}

TEST_F(Fixture, CoreFlightDispatcherMapsTrackCommandsToSafeNoop) {
  CoreFlightStateMachine machine(context);
  CoreFlightCommandDispatcher dispatcher(machine, &reference, &mission);

  for (const int command : {5, 6, 7, 8}) {
    SCOPED_TRACE(command);
    EXPECT_TRUE(dispatcher.update(command));
    EXPECT_TRUE(machine.is(boost::sml::state<SafeNoop>));
  }
  const std::vector<int> expected_commands{5, 6, 7, 8};
  EXPECT_EQ(expected_commands, reference.selected_commands);
  EXPECT_EQ(expected_commands, mission.selected_commands);
}

TEST_F(Fixture, EmergencyPublishesIdentityAttitudeAndLegacyThrust) {
  context.config.hover_thrust = 0.4;
  sm.process_event(OnCommand9{});
  sm.process_event(Tick{});
  ASSERT_EQ(1u, setpoint.attitudes.size());
  EXPECT_DOUBLE_EQ(1.0, setpoint.attitudes[0].attitude.w);
  EXPECT_DOUBLE_EQ(0.0, setpoint.attitudes[0].attitude.x);
  EXPECT_DOUBLE_EQ(0.0, setpoint.attitudes[0].attitude.y);
  EXPECT_DOUBLE_EQ(0.0, setpoint.attitudes[0].attitude.z);
  EXPECT_DOUBLE_EQ(0.37, setpoint.attitudes[0].thrust);
  EXPECT_TRUE(autopilot.calls.empty());
  EXPECT_TRUE(setpoint.positions.empty());
  EXPECT_TRUE(setpoint.body_rates.empty());
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
