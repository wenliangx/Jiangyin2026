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
  bool solveLegacy(const LegacyNmpcRequest& request,
                   BodyRateThrust& output) override {
    ++legacy_calls;
    last_legacy = request;
    output = legacy_output;
    return legacy_result;
  }
  int hover_calls{0};
  int track_calls{0};
  int legacy_calls{0};
  bool hover_result{true};
  bool track_result{true};
  bool legacy_result{true};
  BodyRateThrust hover_output{{0.1, 0.2, 0.3}, 0.4};
  BodyRateThrust track_output{{0.4, 0.5, 0.6}, 0.7};
  BodyRateThrust legacy_output{{0.7, 0.8, 0.9}, 1.0};
  TelemetrySnapshot last_telemetry;
  std::vector<ReferencePoint> last_horizon;
  LegacyNmpcRequest last_legacy;
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
  void reset(MissionTrackMode value) override {
    ++resets;
    last_reset_mode = value;
  }
  bool prepareSuper(double value, const TelemetrySnapshot& telemetry,
                    std::vector<ReferencePoint>& output) override {
    ++super_calls;
    last_time = value;
    last_telemetry = telemetry;
    output = super_points;
    return super_result;
  }
  bool prepareMission(double value, const TelemetrySnapshot& telemetry,
                      std::vector<ReferencePoint>& output) override {
    ++mission_calls;
    last_time = value;
    last_telemetry = telemetry;
    output = mission_points;
    return mission_result;
  }
  bool prepareEgo(double value, const TelemetrySnapshot& telemetry,
                  std::vector<ReferencePoint>& output) override {
    ++ego_calls;
    last_time = value;
    last_telemetry = telemetry;
    output = ego_points;
    return ego_result;
  }

  int resets{0};
  MissionTrackMode last_reset_mode{MissionTrackMode::Super};
  int super_calls{0};
  int mission_calls{0};
  int ego_calls{0};
  double last_time{0.0};
  TelemetrySnapshot last_telemetry;
  bool super_result{true};
  bool mission_result{true};
  bool ego_result{true};
  std::vector<ReferencePoint> super_points{ReferencePoint{}};
  std::vector<ReferencePoint> mission_points{ReferencePoint{}};
  std::vector<ReferencePoint> ego_points{ReferencePoint{}};
  std::vector<int> selected_commands;
};

struct Fixture : testing::Test {
  Fixture()
      : context(clock, autopilot, setpoint, nmpc, reference, mission),
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
      case 7: return "MissionTrack";
      case 8: return "EgoTrack";
      case 9: return "Emergency";
      case 10: return "SafeNoop";
      default: return "Unknown";
    }
  }

  void SelectStateByIndex(StateMachine& machine, int index) {
    switch (index) {
      case 0: machine.process_event(SelectIdle{}); break;
      case 1: machine.process_event(SelectLowThrust{}); break;
      case 2: machine.process_event(SelectPositionHold{}); break;
      case 3: machine.process_event(SelectNmpcHover{}); break;
      case 4: machine.process_event(SelectLanding{}); break;
      case 5: machine.process_event(SelectNmpcTrack{}); break;
      case 6: machine.process_event(SelectSuperTrack{}); break;
      case 7: machine.process_event(SelectMissionTrack{}); break;
      case 8: machine.process_event(SelectEgoTrack{}); break;
      case 9: machine.process_event(SelectEmergency{}); break;
      case 10: machine.process_event(SelectSafeNoop{}); break;
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
      case 7: EXPECT_TRUE(machine.is(boost::sml::state<MissionTrack>)); break;
      case 8: EXPECT_TRUE(machine.is(boost::sml::state<EgoTrack>)); break;
      case 9: EXPECT_TRUE(machine.is(boost::sml::state<Emergency>)); break;
      case 10: EXPECT_TRUE(machine.is(boost::sml::state<SafeNoop>)); break;
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
    nmpc.legacy_calls = 0;
    nmpc.last_horizon.clear();
    nmpc.last_legacy = LegacyNmpcRequest{};
    reference.horizon_calls = 0;
    mission.super_calls = 0;
    mission.mission_calls = 0;
    mission.ego_calls = 0;
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
  Context context;
  StateMachine sm;
};

TEST_F(Fixture, InitialAndEverySelectionReachExpectedState) {
  EXPECT_TRUE(sm.is(boost::sml::state<Idle>));
  sm.process_event(SelectLowThrust{});
  EXPECT_TRUE(sm.is(boost::sml::state<LowThrust>));
  sm.process_event(SelectPositionHold{});
  EXPECT_TRUE(sm.is(boost::sml::state<PositionHold>));
  sm.process_event(SelectNmpcHover{});
  EXPECT_TRUE(sm.is(boost::sml::state<NmpcHover>));
  sm.process_event(SelectLanding{});
  EXPECT_TRUE(sm.is(boost::sml::state<Landing>));
  sm.process_event(SelectNmpcTrack{});
  EXPECT_TRUE(sm.is(boost::sml::state<NmpcTrack>));
  sm.process_event(SelectSuperTrack{});
  EXPECT_TRUE(sm.is(boost::sml::state<SuperTrack>));
  sm.process_event(SelectMissionTrack{});
  EXPECT_TRUE(sm.is(boost::sml::state<MissionTrack>));
  sm.process_event(SelectEgoTrack{});
  EXPECT_TRUE(sm.is(boost::sml::state<EgoTrack>));
  sm.process_event(SelectEmergency{});
  EXPECT_TRUE(sm.is(boost::sml::state<Emergency>));
  sm.process_event(SelectSafeNoop{});
  EXPECT_TRUE(sm.is(boost::sml::state<SafeNoop>));
  sm.process_event(SelectIdle{});
  EXPECT_TRUE(sm.is(boost::sml::state<Idle>));
}

TEST_F(Fixture, EverySelectionEventWorksFromEveryState) {
  for (int source = 0; source < 11; ++source) {
    for (int target = 0; target < 11; ++target) {
      SCOPED_TRACE(std::string(StateName(source)) + " -> " +
                   StateName(target));
      StateMachine machine(context);
      SelectStateByIndex(machine, source);
      ExpectStateByIndex(machine, source);
      SelectStateByIndex(machine, target);
      ExpectStateByIndex(machine, target);
    }
  }
}

TEST_F(Fixture, DirectSelectNmpcTrackResetsOnceFromEverySourceState) {
  for (int source = 0; source < 11; ++source) {
    SCOPED_TRACE(StateName(source));
    StateMachine machine(context);
    SelectStateByIndex(machine, source);
    const int resets_after_source_selection = reference.resets;
    SelectStateByIndex(machine, 5);
    EXPECT_EQ(resets_after_source_selection + 1, reference.resets);
    ExpectStateByIndex(machine, 5);
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
  EXPECT_TRUE(sm.is(boost::sml::state<MissionTrack>));
  EXPECT_TRUE(dispatcher.update(8));
  EXPECT_TRUE(sm.is(boost::sml::state<EgoTrack>));
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
  EXPECT_TRUE(sm.is(boost::sml::state<MissionTrack>));
  EXPECT_FALSE(dispatcher.update(7));

  EXPECT_TRUE(dispatcher.update(8));
  EXPECT_TRUE(sm.is(boost::sml::state<EgoTrack>));
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
  sm.process_event(SelectSafeNoop{});
  sm.process_event(Tick{});
  EXPECT_TRUE(autopilot.calls.empty());
  EXPECT_TRUE(setpoint.positions.empty());
  EXPECT_TRUE(setpoint.body_rates.empty());
  EXPECT_TRUE(setpoint.attitudes.empty());
}

TEST_F(Fixture, LowThrustTickPublishesLegacyMessage) {
  SetOffboardAndArmed();
  context.config.low_thrust = 0.123;
  sm.process_event(SelectLowThrust{});
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
  sm.process_event(SelectPositionHold{});
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
  for (const int state : {1, 2, 3, 5, 6}) {
    SCOPED_TRACE(StateName(state));
    ClearOutputs();
    context.telemetry.mode = "MANUAL";
    context.telemetry.armed = false;
    context.last_service_request = 0.0;
    clock.value = 5.001;
    SelectStateByIndex(sm, state);
    sm.process_event(Tick{});
    ASSERT_FALSE(autopilot.calls.empty());
    EXPECT_EQ("offboard", autopilot.calls[0]);
  }

  for (const int state : {0, 4, 7, 8, 9, 10}) {
    SCOPED_TRACE(StateName(state));
    ClearOutputs();
    context.telemetry.mode = "MANUAL";
    context.telemetry.armed = false;
    context.landing_reached = false;
    context.last_service_request = 0.0;
    clock.value = 5.001;
    SelectStateByIndex(sm, state);
    sm.process_event(Tick{});
    EXPECT_TRUE(autopilot.calls.empty());
  }
}

TEST_F(Fixture, ActiveOffboardStatesDoNotRequestServicesWhenAlreadyReady) {
  for (const int state : {1, 2, 3, 5, 6}) {
    SCOPED_TRACE(StateName(state));
    ClearOutputs();
    SetOffboardAndArmed();
    clock.value = 100.0;
    SelectStateByIndex(sm, state);
    sm.process_event(Tick{});
    EXPECT_TRUE(autopilot.calls.empty());
  }
}

TEST_F(Fixture, ServiceCooldownUsesStrictFiveSecondBoundary) {
  sm.process_event(SelectLowThrust{});
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
  sm.process_event(SelectLowThrust{});
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
  sm.process_event(SelectPositionHold{});
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
  sm.process_event(SelectNmpcHover{});
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
  sm.process_event(SelectNmpcHover{});
  nmpc.hover_result = false;
  sm.process_event(Tick{});
  nmpc.hover_result = true;
  nmpc.hover_output.thrust = std::numeric_limits<double>::quiet_NaN();
  sm.process_event(Tick{});
  EXPECT_TRUE(setpoint.body_rates.empty());
}

TEST_F(Fixture, NmpcHoverRejectsAnyNonFiniteOutputField) {
  SetOffboardAndArmed();
  sm.process_event(SelectNmpcHover{});
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
  sm.process_event(SelectNmpcTrack{});
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
  sm.process_event(SelectIdle{});
  sm.process_event(SelectNmpcTrack{});
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

  sm.process_event(SelectSuperTrack{});
  EXPECT_EQ(1, mission.resets);
  EXPECT_EQ(MissionTrackMode::Super, mission.last_reset_mode);
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

TEST_F(Fixture, Cmd7And8UseLegacyMissionNmpcPipeline) {
  for (const int state : {7, 8}) {
    SCOPED_TRACE(StateName(state));
    ClearOutputs();
    context.telemetry.mode = "MANUAL";
    context.telemetry.armed = false;
    context.last_service_request = 0.0;
    clock.value = 5.001;
    auto& points = state == 7 ? mission.mission_points : mission.ego_points;
    points = {ReferencePoint{}};
    points[0].position = {10.0 + state, 20.0 + state, 30.0 + state};
    points[0].velocity = {1.0 * state, 2.0 * state, 3.0 * state};
    points[0].attitude = {0.5, 0.1, 0.2, 0.3};
    nmpc.legacy_output = BodyRateThrust{{0.1 * state, 0.2 * state,
                                         0.3 * state},
                                        0.4 + 0.01 * state};
    SelectStateByIndex(sm, state);
    sm.process_event(Tick{});
    EXPECT_TRUE(autopilot.calls.empty());
    EXPECT_EQ(0, reference.horizon_calls);
    EXPECT_EQ(0, nmpc.track_calls);
    EXPECT_EQ(1, nmpc.legacy_calls);
    if (state == 7) {
      EXPECT_EQ(1, mission.mission_calls);
      EXPECT_EQ(MissionTrackMode::Mission, mission.last_reset_mode);
    } else {
      EXPECT_EQ(1, mission.ego_calls);
      EXPECT_EQ(MissionTrackMode::Ego, mission.last_reset_mode);
    }
    ASSERT_EQ(1u, nmpc.last_legacy.horizon.size());
    EXPECT_DOUBLE_EQ(10.0 + state, nmpc.last_legacy.horizon[0].position.x);
    EXPECT_DOUBLE_EQ(2.0 * state, nmpc.last_legacy.horizon[0].velocity.y);
    EXPECT_DOUBLE_EQ(0.5, nmpc.last_legacy.horizon[0].attitude.w);
    EXPECT_DOUBLE_EQ(0.3, nmpc.last_legacy.horizon[0].attitude.z);
    EXPECT_DOUBLE_EQ(9.8, nmpc.last_legacy.desired_controls[0]);
    EXPECT_DOUBLE_EQ(0.0, nmpc.last_legacy.desired_controls[1]);
    EXPECT_DOUBLE_EQ(0.0, nmpc.last_legacy.desired_controls[2]);
    EXPECT_DOUBLE_EQ(0.0, nmpc.last_legacy.desired_controls[3]);
    ASSERT_EQ(1u, setpoint.body_rates.size());
    EXPECT_DOUBLE_EQ(0.1 * state, setpoint.body_rates[0].body_rate.x);
    EXPECT_DOUBLE_EQ(0.2 * state, setpoint.body_rates[0].body_rate.y);
    EXPECT_DOUBLE_EQ(0.3 * state, setpoint.body_rates[0].body_rate.z);
    EXPECT_DOUBLE_EQ(0.4 + 0.01 * state, setpoint.body_rates[0].thrust);
    ASSERT_EQ(1u, setpoint.feedback_positions.size());
    ASSERT_EQ(1u, setpoint.reference_positions.size());
    EXPECT_DOUBLE_EQ(10.0 + state, setpoint.reference_positions[0].x);
  }
}

TEST_F(Fixture, Cmd6RejectsMissingReferenceSolveFailureAndBadOutput) {
  SetOffboardAndArmed();
  sm.process_event(SelectSuperTrack{});

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

TEST_F(Fixture, Cmd7And8RejectMissingReferenceSolveFailureAndBadOutput) {
  SetOffboardAndArmed();
  for (const int state : {7, 8}) {
    SCOPED_TRACE(StateName(state));
    ClearOutputs();
    SelectStateByIndex(sm, state);
    auto& result = state == 7 ? mission.mission_result : mission.ego_result;
    auto& points = state == 7 ? mission.mission_points : mission.ego_points;

    result = false;
    sm.process_event(Tick{});
    EXPECT_EQ(0, nmpc.legacy_calls);
    EXPECT_TRUE(setpoint.body_rates.empty());
    EXPECT_TRUE(setpoint.reference_positions.empty());
    EXPECT_TRUE(setpoint.feedback_positions.empty());

    result = true;
    points.clear();
    sm.process_event(Tick{});
    EXPECT_EQ(0, nmpc.legacy_calls);

    points = {ReferencePoint{}};
    nmpc.legacy_result = false;
    sm.process_event(Tick{});
    EXPECT_EQ(1, nmpc.legacy_calls);
    EXPECT_TRUE(setpoint.body_rates.empty());
    EXPECT_EQ(1u, setpoint.reference_positions.size());
    EXPECT_EQ(1u, setpoint.feedback_positions.size());

    ClearOutputs();
    points = {ReferencePoint{}};
    nmpc.legacy_result = true;
    nmpc.legacy_output.body_rate.z =
        std::numeric_limits<double>::infinity();
    sm.process_event(Tick{});
    EXPECT_EQ(1, nmpc.legacy_calls);
    EXPECT_TRUE(setpoint.body_rates.empty());
  }
}

TEST_F(Fixture, TrackRejectsMissingReferenceAndBadNmpcOutput) {
  context.telemetry.mode = "OFFBOARD";
  context.telemetry.armed = true;
  sm.process_event(SelectNmpcTrack{});
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
  sm.process_event(SelectNmpcTrack{});

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
  sm.process_event(SelectNmpcTrack{});
  nmpc.track_result = false;
  sm.process_event(Tick{});
  EXPECT_EQ(1, reference.horizon_calls);
  EXPECT_EQ(1, nmpc.track_calls);
  EXPECT_TRUE(setpoint.body_rates.empty());
  EXPECT_TRUE(setpoint.monitors.empty());
}

TEST_F(Fixture, TrackRejectsAnyNonFiniteOutputField) {
  SetOffboardAndArmed();
  sm.process_event(SelectNmpcTrack{});
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

TEST_F(Fixture, LandingKeepsLegacyLatchAndDisarmSemantics) {
  context.telemetry.position = {2.0, 3.0, 0.11};
  context.telemetry.mode = "OFFBOARD";
  context.telemetry.armed = true;
  sm.process_event(SelectLanding{});
  sm.process_event(Tick{});
  ASSERT_EQ(1u, setpoint.positions.size());
  EXPECT_DOUBLE_EQ(2.0, setpoint.positions[0].position.x);
  EXPECT_DOUBLE_EQ(3.0, setpoint.positions[0].position.y);
  EXPECT_FALSE(context.landing_reached);
  context.telemetry.position.z = 0.06;
  sm.process_event(Tick{});
  EXPECT_TRUE(context.landing_reached);
  sm.process_event(Tick{});
  EXPECT_TRUE(autopilot.calls.empty());
  context.telemetry.mode = "POSCTL";
  sm.process_event(Tick{});
  ASSERT_EQ(1u, autopilot.calls.size());
  EXPECT_EQ("disarm", autopilot.calls[0]);
}

TEST_F(Fixture, LandingUsesStrictToleranceAndStopsPublishingAfterLatch) {
  context.config.landing_reference_z = 0.05;
  context.config.landing_tolerance_z = 0.05;
  context.config.landing_target_z = 0.005;
  context.telemetry.position = {4.0, 5.0, 0.10};
  context.telemetry.mode = "OFFBOARD";
  context.telemetry.armed = true;
  sm.process_event(SelectLanding{});

  sm.process_event(Tick{});
  ASSERT_EQ(1u, setpoint.positions.size());
  EXPECT_DOUBLE_EQ(4.0, setpoint.positions[0].position.x);
  EXPECT_DOUBLE_EQ(5.0, setpoint.positions[0].position.y);
  EXPECT_DOUBLE_EQ(0.005, setpoint.positions[0].position.z);
  EXPECT_DOUBLE_EQ(0.0, setpoint.positions[0].yaw);
  EXPECT_FALSE(context.landing_reached);

  context.telemetry.position.z = 0.099;
  sm.process_event(Tick{});
  EXPECT_TRUE(context.landing_reached);
  const std::size_t published_before_latched_tick = setpoint.positions.size();
  sm.process_event(Tick{});
  EXPECT_EQ(published_before_latched_tick, setpoint.positions.size());
}

TEST_F(Fixture, LandingDisarmsOnlyWhenLatchedNonOffboardAndArmed) {
  sm.process_event(SelectLanding{});
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

TEST_F(Fixture, EmergencyPublishesIdentityAttitudeAndLegacyThrust) {
  context.config.hover_thrust = 0.4;
  sm.process_event(SelectEmergency{});
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
