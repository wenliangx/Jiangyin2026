#include <gtest/gtest.h>

#include <limits>
#include <string>
#include <type_traits>
#include <vector>

#include <fsm_ctrl/single_offboard_sml.hpp>
#include <fsm_ctrl/single_offboard_sml/actions/common.hpp>
#include <fsm_ctrl/single_offboard_sml/actions/mission.hpp>
#include <fsm_ctrl/single_offboard_sml/actions/segmented_mission.hpp>
#include <fsm_ctrl/single_offboard_sml/context.hpp>
#include <fsm_ctrl/single_offboard_sml/dispatch.hpp>
#include <fsm_ctrl/single_offboard_sml/machines/mission.hpp>
#include <fsm_ctrl/single_offboard_sml/machines/segmented_mission.hpp>
#include <fsm_ctrl/single_offboard_sml/ports.hpp>
#include <fsm_ctrl/single_offboard_sml/states.hpp>
#include <fsm_ctrl/single_offboard_sml/types.hpp>

namespace {
using namespace fsm_ctrl::single_sml;

TEST(HeaderLayoutCompatibility,
     OldWrappersAndCanonicalHeadersExposePublicTypes) {
  static_assert(std::is_same<ActiveMachine, MissionMachine>::value,
                "ActiveMachine must remain the mission machine alias");
  static_assert(std::is_same<ActiveStateMachine,
                             boost::sml::sm<ActiveMachine>>::value,
                "ActiveStateMachine must remain compatible with ActiveMachine");
  static_assert(std::is_same<ActiveCommandDispatcher,
                             CommandDispatcherT<ActiveStateMachine>>::value,
                "ActiveCommandDispatcher must remain the active dispatcher");
}

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

class FakeLogger final : public LogPort {
 public:
  void write(const LogRecord& value) override { records.push_back(value); }

  std::vector<LogRecord> records;
};

std::size_t CountEvent(const FakeLogger& logger, LogEvent event) {
  std::size_t count = 0u;
  for (const LogRecord& record : logger.records) {
    if (record.event == event) {
      ++count;
    }
  }
  return count;
}

const LogRecord& FirstEvent(const FakeLogger& logger, LogEvent event) {
  for (const LogRecord& record : logger.records) {
    if (record.event == event) {
      return record;
    }
  }
  ADD_FAILURE() << "missing log event";
  return logger.records.front();
}

TEST(LoggingPort, FakeLoggerCapturesStructuredRecord) {
  FakeLogger logger;
  LogRecord record;
  record.severity = LogSeverity::Warn;
  record.event = LogEvent::CommandUnsupported;
  record.command = 42;
  record.stamp = 12.5;

  logger.write(record);

  ASSERT_EQ(1u, logger.records.size());
  EXPECT_EQ(LogSeverity::Warn, logger.records[0].severity);
  EXPECT_EQ(LogEvent::CommandUnsupported, logger.records[0].event);
  EXPECT_EQ(42, logger.records[0].command);
  EXPECT_DOUBLE_EQ(12.5, logger.records[0].stamp);
}

struct Fixture : testing::Test {
  Fixture()
      : context(clock, autopilot, setpoint, nmpc, reference, mission, landing,
                logger),
        sm(context) {}

  static const char* StateName(int index) {
    switch (index) {
      case 0: return "Idle";
      case 1: return "ArmOnly";
      case 2: return "NmpcHover";
      case 3: return "SuperTrack";
      case 4: return "Landing";
      case 5: return "Emergency";
      case 6: return "SafeNoop";
      default: return "Unknown";
    }
  }

  void SendCommandEventByStateIndex(ActiveStateMachine& machine, int index) {
    switch (index) {
      case 0: machine.process_event(OnCommand0{}); break;
      case 1: machine.process_event(OnCommand1{}); break;
      case 2: machine.process_event(OnCommand2{}); break;
      case 3: machine.process_event(OnCommand3{}); break;
      case 4: machine.process_event(OnCommand4{}); break;
      case 5: machine.process_event(OnCommand9{}); break;
      case 6: machine.process_event(OnCommand5{}); break;
      default: ADD_FAILURE() << "Bad state index " << index; break;
    }
  }

  void ExpectStateByIndex(ActiveStateMachine& machine, int index) {
    switch (index) {
      case 0: EXPECT_TRUE(machine.is(boost::sml::state<Idle>)); break;
      case 1: EXPECT_TRUE(machine.is(boost::sml::state<ArmOnly>)); break;
      case 2: EXPECT_TRUE(machine.is(boost::sml::state<NmpcHover>)); break;
      case 3: EXPECT_TRUE(machine.is(boost::sml::state<SuperTrack>)); break;
      case 4: EXPECT_TRUE(machine.is(boost::sml::state<Landing>)); break;
      case 5: EXPECT_TRUE(machine.is(boost::sml::state<Emergency>)); break;
      case 6: EXPECT_TRUE(machine.is(boost::sml::state<SafeNoop>)); break;
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
  FakeLogger logger;
  Context context;
  ActiveStateMachine sm;
};

TEST_F(Fixture, InitialAndEveryCommandEventReachExpectedState) {
  EXPECT_TRUE(sm.is(boost::sml::state<Idle>));
  sm.process_event(OnCommand1{});
  EXPECT_TRUE(sm.is(boost::sml::state<ArmOnly>));
  sm.process_event(OnCommand2{});
  EXPECT_TRUE(sm.is(boost::sml::state<NmpcHover>));
  sm.process_event(OnCommand3{});
  EXPECT_TRUE(sm.is(boost::sml::state<SuperTrack>));
  sm.process_event(OnCommand4{});
  EXPECT_TRUE(sm.is(boost::sml::state<Landing>));
  sm.process_event(OnCommand5{});
  EXPECT_TRUE(sm.is(boost::sml::state<SafeNoop>));
  sm.process_event(OnCommand6{});
  EXPECT_TRUE(sm.is(boost::sml::state<SafeNoop>));
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
  for (int source = 0; source < 7; ++source) {
    for (int target = 0; target < 7; ++target) {
      SCOPED_TRACE(std::string(StateName(source)) + " -> " +
                   StateName(target));
      ActiveStateMachine machine(context);
      SendCommandEventByStateIndex(machine, source);
      ExpectStateByIndex(machine, source);
      SendCommandEventByStateIndex(machine, target);
      ExpectStateByIndex(machine, target);
    }
  }
}

TEST_F(Fixture, DirectOnCommand3ResetsSuperOnceFromEverySourceState) {
  for (int source = 0; source < 7; ++source) {
    SCOPED_TRACE(StateName(source));
    ActiveStateMachine machine(context);
    SendCommandEventByStateIndex(machine, source);
    const int resets_after_source_command = mission.resets;
    SendCommandEventByStateIndex(machine, 3);
    EXPECT_EQ(resets_after_source_command + 1, mission.resets);
    ExpectStateByIndex(machine, 3);
  }
}

TEST_F(Fixture, DirectOnCommand4ResetsOnceFromEverySourceState) {
  for (int source = 0; source < 7; ++source) {
    SCOPED_TRACE(StateName(source));
    ActiveStateMachine machine(context);
    SendCommandEventByStateIndex(machine, source);
    const int resets_after_source_command = landing.resets;
    SendCommandEventByStateIndex(machine, 4);
    EXPECT_EQ(resets_after_source_command + 1, landing.resets);
    ExpectStateByIndex(machine, 4);
  }
}

TEST_F(Fixture, ActiveCommandDispatcherMapsMissionAndSafeNoopCommands) {
  ActiveCommandDispatcher dispatcher(sm, &reference, &mission, &logger);
  EXPECT_TRUE(dispatcher.update(0));
  EXPECT_TRUE(sm.is(boost::sml::state<Idle>));
  EXPECT_TRUE(dispatcher.update(1));
  EXPECT_TRUE(sm.is(boost::sml::state<ArmOnly>));
  EXPECT_TRUE(dispatcher.update(2));
  EXPECT_TRUE(sm.is(boost::sml::state<NmpcHover>));
  EXPECT_TRUE(dispatcher.update(3));
  EXPECT_TRUE(sm.is(boost::sml::state<SuperTrack>));
  EXPECT_EQ(1, mission.resets);
  EXPECT_TRUE(dispatcher.update(4));
  EXPECT_TRUE(sm.is(boost::sml::state<Landing>));
  EXPECT_EQ(1, landing.resets);
  EXPECT_TRUE(dispatcher.update(5));
  EXPECT_TRUE(sm.is(boost::sml::state<SafeNoop>));
  EXPECT_TRUE(dispatcher.update(6));
  EXPECT_TRUE(sm.is(boost::sml::state<SafeNoop>));
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
  EXPECT_EQ(expected_commands, mission.selected_commands);
}

TEST_F(Fixture, ActiveCommandDispatcherLogsCommandDecisions) {
  ActiveCommandDispatcher dispatcher(sm, &reference, &mission, &logger,
                                     &clock);
  clock.value = 7.5;

  EXPECT_TRUE(dispatcher.update(3));
  ASSERT_EQ(1u, logger.records.size());
  EXPECT_EQ(LogEvent::CommandNew, logger.records[0].event);
  EXPECT_EQ(LogSeverity::Info, logger.records[0].severity);
  EXPECT_EQ(3, logger.records[0].command);
  EXPECT_DOUBLE_EQ(7.5, logger.records[0].stamp);

  EXPECT_FALSE(dispatcher.update(3));
  ASSERT_EQ(2u, logger.records.size());
  EXPECT_EQ(LogEvent::CommandRepeatedSuppressed, logger.records[1].event);
  EXPECT_EQ(LogSeverity::Warn, logger.records[1].severity);
  EXPECT_EQ(3, logger.records[1].command);
  EXPECT_EQ(1u, reference.selected_commands.size());
  EXPECT_EQ(1u, mission.selected_commands.size());

  EXPECT_TRUE(dispatcher.update(42));
  ASSERT_EQ(4u, logger.records.size());
  EXPECT_EQ(LogEvent::CommandNew, logger.records[2].event);
  EXPECT_EQ(LogEvent::CommandUnsupported, logger.records[3].event);
  EXPECT_EQ(LogSeverity::Warn, logger.records[3].severity);
  EXPECT_EQ(42, logger.records[3].command);
  EXPECT_TRUE(sm.is(boost::sml::state<SafeNoop>));
}

TEST_F(Fixture, ActiveCommandDispatcherSuppressesRepeatedMissionAndUnknownCommands) {
  ActiveCommandDispatcher dispatcher(sm, &reference, &mission, &logger);
  EXPECT_TRUE(dispatcher.update(3));
  EXPECT_TRUE(sm.is(boost::sml::state<SuperTrack>));
  EXPECT_FALSE(dispatcher.update(3));
  EXPECT_TRUE(sm.is(boost::sml::state<SuperTrack>));
  EXPECT_EQ(1, mission.resets);

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
  EXPECT_TRUE(sm.is(boost::sml::state<SafeNoop>));
  EXPECT_EQ(1, mission.resets);
  EXPECT_FALSE(dispatcher.update(5));
  EXPECT_EQ(1, mission.resets);

  EXPECT_TRUE(dispatcher.update(-1));
  EXPECT_TRUE(sm.is(boost::sml::state<SafeNoop>));
  EXPECT_FALSE(dispatcher.update(-1));
  const std::vector<int> expected_commands{3, 7, 8, 42, 5, -1};
  EXPECT_EQ(expected_commands, reference.selected_commands);
  EXPECT_EQ(expected_commands, mission.selected_commands);
}

TEST_F(Fixture, CommandDispatcherProcessesFirstIntMinCommand) {
  ActiveCommandDispatcher dispatcher(sm, &reference, &mission, &logger);
  EXPECT_TRUE(dispatcher.update(std::numeric_limits<int>::min()));
  EXPECT_TRUE(sm.is(boost::sml::state<SafeNoop>));
  EXPECT_FALSE(dispatcher.update(std::numeric_limits<int>::min()));
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

TEST_F(Fixture, OffboardArmSharedLogicRunsOnlyInActiveOffboardStates) {
  for (const int state : {1, 2, 3, 4}) {
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

  for (const int state : {0, 5, 6}) {
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
  for (const int state : {1, 2, 3, 4}) {
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

TEST_F(Fixture, NmpcHoverTracksCurrentXyAtOneMeter) {
  context.telemetry.position = {1.0, 2.0, 3.0};
  context.telemetry.velocity = {4.0, 5.0, 6.0};
  context.telemetry.attitude = {0.7, 0.1, 0.2, 0.3};
  SetOffboardAndArmed();
  sm.process_event(OnCommand2{});
  sm.process_event(Tick{});
  EXPECT_EQ(0, nmpc.hover_calls);
  EXPECT_EQ(1, nmpc.track_calls);
  EXPECT_DOUBLE_EQ(1.0, nmpc.last_telemetry.position.x);
  EXPECT_DOUBLE_EQ(5.0, nmpc.last_telemetry.velocity.y);
  EXPECT_DOUBLE_EQ(0.2, nmpc.last_telemetry.attitude.y);
  ASSERT_EQ(10u, nmpc.last_horizon.size());
  EXPECT_DOUBLE_EQ(1.0, nmpc.last_horizon.front().position.x);
  EXPECT_DOUBLE_EQ(2.0, nmpc.last_horizon.front().position.y);
  EXPECT_DOUBLE_EQ(1.0, nmpc.last_horizon.front().position.z);
  ASSERT_EQ(1u, setpoint.body_rates.size());
  EXPECT_DOUBLE_EQ(0.4, setpoint.body_rates[0].body_rate.x);
  EXPECT_DOUBLE_EQ(0.5, setpoint.body_rates[0].body_rate.y);
  EXPECT_DOUBLE_EQ(0.6, setpoint.body_rates[0].body_rate.z);
  EXPECT_DOUBLE_EQ(0.7, setpoint.body_rates[0].thrust);
  ASSERT_EQ(1u, setpoint.monitors.size());
  ASSERT_EQ(10u, setpoint.monitors[0].references.size());
  EXPECT_DOUBLE_EQ(1.0, setpoint.monitors[0].references[0].position.x);
  EXPECT_DOUBLE_EQ(1.0, setpoint.monitors[0].references[0].position.z);
  EXPECT_DOUBLE_EQ(1.0, setpoint.monitors[0].feedback.position.x);
  EXPECT_DOUBLE_EQ(5.0, setpoint.monitors[0].feedback.velocity.y);
  EXPECT_DOUBLE_EQ(0.4, setpoint.monitors[0].target.body_rate.x);
  EXPECT_DOUBLE_EQ(0.7, setpoint.monitors[0].target.thrust);
  ASSERT_EQ(1u, CountEvent(logger, LogEvent::ActionHoverToOneMeter));
  const LogRecord& action = FirstEvent(logger, LogEvent::ActionHoverToOneMeter);
  EXPECT_EQ(LogSeverity::Debug, action.severity);
  EXPECT_DOUBLE_EQ(1.0, action.reference_position.x);
  EXPECT_DOUBLE_EQ(2.0, action.reference_position.y);
  EXPECT_DOUBLE_EQ(1.0, action.reference_position.z);
  ASSERT_EQ(1u, CountEvent(logger, LogEvent::NmpcPublishSuccess));
  const LogRecord& publish = FirstEvent(logger, LogEvent::NmpcPublishSuccess);
  EXPECT_EQ(LogSeverity::Debug, publish.severity);
  EXPECT_EQ(10u, publish.horizon_size);
  EXPECT_DOUBLE_EQ(0.7, publish.command_output.thrust);
}

TEST_F(Fixture, Cmd3UsesSuperMissionReferenceAndNmpcMonitor) {
  SetOffboardAndArmed();
  context.telemetry.position = {1.0, 2.0, 3.0};
  context.telemetry.velocity = {4.0, 5.0, 6.0};
  mission.super_points = {ReferencePoint{}};
  mission.super_points[0].position = {16.0, 26.0, 36.0};
  mission.super_points[0].velocity = {6.0, 12.0, 18.0};
  nmpc.track_output = BodyRateThrust{{0.6, 1.2, 1.8}, 0.46};

  sm.process_event(OnCommand3{});
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
  ASSERT_EQ(1u, CountEvent(logger, LogEvent::ActionSuperTrack));
  const LogRecord& action = FirstEvent(logger, LogEvent::ActionSuperTrack);
  EXPECT_EQ(LogSeverity::Debug, action.severity);
  EXPECT_EQ(1u, action.horizon_size);
  EXPECT_DOUBLE_EQ(16.0, action.reference_position.x);
  ASSERT_EQ(1u, CountEvent(logger, LogEvent::NmpcPublishSuccess));
}

TEST_F(Fixture, Cmd3RejectsMissingReferenceSolveFailureAndBadOutput) {
  SetOffboardAndArmed();
  sm.process_event(OnCommand3{});

  mission.super_result = false;
  sm.process_event(Tick{});
  EXPECT_EQ(0, nmpc.track_calls);
  EXPECT_TRUE(setpoint.body_rates.empty());
  EXPECT_TRUE(setpoint.monitors.empty());
  ASSERT_EQ(1u, CountEvent(logger, LogEvent::EmptyHorizon));
  EXPECT_EQ(LogSeverity::Warn,
            FirstEvent(logger, LogEvent::EmptyHorizon).severity);

  mission.super_result = true;
  mission.super_points.clear();
  sm.process_event(Tick{});
  EXPECT_EQ(0, nmpc.track_calls);
  ASSERT_EQ(2u, CountEvent(logger, LogEvent::EmptyHorizon));

  mission.super_points = {ReferencePoint{}};
  nmpc.track_result = false;
  sm.process_event(Tick{});
  EXPECT_EQ(1, nmpc.track_calls);
  EXPECT_TRUE(setpoint.body_rates.empty());
  EXPECT_TRUE(setpoint.monitors.empty());
  EXPECT_EQ(1u, setpoint.reference_positions.size());
  EXPECT_EQ(1u, setpoint.feedback_positions.size());
  ASSERT_EQ(1u, CountEvent(logger, LogEvent::NmpcSolveFailure));
  EXPECT_EQ(LogSeverity::Error,
            FirstEvent(logger, LogEvent::NmpcSolveFailure).severity);

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
  ASSERT_EQ(1u, CountEvent(logger, LogEvent::NmpcNonFiniteOutput));
  EXPECT_EQ(LogSeverity::Error,
            FirstEvent(logger, LogEvent::NmpcNonFiniteOutput).severity);
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
  ASSERT_EQ(1u, CountEvent(logger, LogEvent::ActionLanding));
  const LogRecord& action = FirstEvent(logger, LogEvent::ActionLanding);
  EXPECT_EQ(LogSeverity::Debug, action.severity);
  EXPECT_EQ(1u, action.horizon_size);
  EXPECT_DOUBLE_EQ(2.1, action.reference_position.x);
  ASSERT_EQ(1u, CountEvent(logger, LogEvent::NmpcPublishSuccess));
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
  ASSERT_EQ(1u, CountEvent(logger, LogEvent::EmptyHorizon));

  ClearOutputs();
  landing.result = true;
  landing.points.clear();
  sm.process_event(Tick{});
  EXPECT_EQ(1, landing.prepare_calls);
  EXPECT_EQ(0, nmpc.track_calls);
  ASSERT_EQ(2u, CountEvent(logger, LogEvent::EmptyHorizon));

  ClearOutputs();
  landing.points = {ReferencePoint{}};
  nmpc.track_result = false;
  sm.process_event(Tick{});
  EXPECT_EQ(1, nmpc.track_calls);
  EXPECT_TRUE(setpoint.body_rates.empty());
  ASSERT_EQ(1u, CountEvent(logger, LogEvent::NmpcSolveFailure));

  ClearOutputs();
  nmpc.track_result = true;
  nmpc.track_output.thrust = std::numeric_limits<double>::quiet_NaN();
  sm.process_event(Tick{});
  EXPECT_EQ(1, nmpc.track_calls);
  EXPECT_TRUE(setpoint.body_rates.empty());
  EXPECT_TRUE(setpoint.monitors.empty());
  ASSERT_EQ(1u, CountEvent(logger, LogEvent::NmpcNonFiniteOutput));
}

TEST_F(Fixture, LandingCompletionKeepsLegacyLatchAndDisarmSemantics) {
  SetOffboardAndArmed();
  context.telemetry.position = {2.0, 3.0, 0.11};
  sm.process_event(OnCommand4{});
  landing.complete = true;
  sm.process_event(Tick{});
  EXPECT_TRUE(context.landing_reached);
  ASSERT_EQ(1u, CountEvent(logger, LogEvent::LandingLatched));
  sm.process_event(Tick{});
  EXPECT_TRUE(autopilot.calls.empty());
  context.telemetry.mode = "POSCTL";
  sm.process_event(Tick{});
  ASSERT_EQ(1u, autopilot.calls.size());
  EXPECT_EQ("disarm", autopilot.calls[0]);
  ASSERT_EQ(1u, CountEvent(logger, LogEvent::LandingDisarmRequested));
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
  ASSERT_EQ(1u, CountEvent(logger, LogEvent::LandingLatched));
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
  ASSERT_EQ(1u, CountEvent(logger, LogEvent::LandingDisarmRequested));
}

TEST_F(Fixture, MissionMachineMapsCommandsToArmHoverSuperLanding) {
  MissionStateMachine machine(context);
  MissionCommandDispatcher dispatcher(machine, &reference, &mission, &logger);
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
  SegmentedMissionCommandDispatcher dispatcher(machine, &reference, &mission,
                                               &logger);
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
  ASSERT_EQ(1u, CountEvent(logger, LogEvent::ActionEmergency));
  const LogRecord& action = FirstEvent(logger, LogEvent::ActionEmergency);
  EXPECT_EQ(LogSeverity::Warn, action.severity);
  EXPECT_DOUBLE_EQ(0.37, action.command_output.thrust);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
