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
  bool solveTrack(const TelemetrySnapshot& value,
                  const std::vector<ReferencePoint>& points,
                  BodyRateThrust& output) override {
    ++track_calls;
    last_telemetry = value;
    last_horizon = points;
    output = track_output;
    return track_result;
  }
  int track_calls{0};
  bool track_result{true};
  BodyRateThrust track_output{{0.4, 0.5, 0.6}, 0.7};
  TelemetrySnapshot last_telemetry;
  std::vector<ReferencePoint> last_horizon;
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
  bool prepareSuperSegment(int segment_index, double value,
                           const TelemetrySnapshot& telemetry,
                           std::vector<ReferencePoint>& output) override {
    ++segment_calls;
    last_segment_index = segment_index;
    last_time = value;
    last_telemetry = telemetry;
    output = super_points;
    return super_result;
  }
  int resets{0};
  int super_calls{0};
  int segment_calls{0};
  int last_segment_index{-1};
  double last_time{0.0};
  TelemetrySnapshot last_telemetry;
  bool super_result{true};
  std::vector<ReferencePoint> super_points{ReferencePoint{}};
  std::vector<int> selected_commands;
};

class FakeLanding final : public PrecisionLandingPort {
 public:
  void reset() override { ++resets; }
  void updateObservation(const LandingObservation& value) override {
    last_observation = value;
  }
  void startClosedLoopLanding(
      const TelemetrySnapshot& telemetry) override {
    ++start_closed_loop_calls;
    start_telemetry = telemetry;
  }
  bool prepareLanding(double value, const TelemetrySnapshot& telemetry,
                      std::vector<ReferencePoint>& output) override {
    ++prepare_calls;
    last_time = value;
    last_telemetry = telemetry;
    output = points;
    return result;
  }
  bool prepareClosedLoopLanding(
      double value, const TelemetrySnapshot& telemetry,
      std::vector<ReferencePoint>& output) override {
    ++closed_loop_prepare_calls;
    return prepareLanding(value, telemetry, output);
  }
  int resets{0};
  int start_closed_loop_calls{0};
  int prepare_calls{0};
  int closed_loop_prepare_calls{0};
  double last_time{0.0};
  bool result{true};
  std::vector<ReferencePoint> points{ReferencePoint{}};
  TelemetrySnapshot last_telemetry;
  TelemetrySnapshot start_telemetry;
  LandingObservation last_observation;
};

class FakeCameraControl final : public CameraControlPort {
 public:
  void publishControl(const CameraControlState& value) override {
    controls.push_back(value);
  }

  std::vector<CameraControlState> controls;
};

struct Fixture : testing::Test {
  Fixture()
      : context(clock, autopilot, setpoint, nmpc, mission, landing,
                camera_control),
        sm(context) {}

  void ClearOutputs() {
    autopilot.calls.clear();
    setpoint.positions.clear();
    setpoint.body_rates.clear();
    setpoint.attitudes.clear();
    setpoint.reference_positions.clear();
    setpoint.feedback_positions.clear();
    setpoint.monitors.clear();
    nmpc.track_calls = 0;
    nmpc.last_horizon.clear();
    mission.super_calls = 0;
    mission.segment_calls = 0;
    landing.prepare_calls = 0;
    landing.closed_loop_prepare_calls = 0;
    camera_control.controls.clear();
  }

  void SetOffboardAndArmed() {
    context.telemetry.mode = "OFFBOARD";
    context.telemetry.armed = true;
  }

  void ExpectLatestCameraControl(bool front_enabled,
                                 bool down_enabled) const {
    ASSERT_FALSE(camera_control.controls.empty());
    EXPECT_EQ(front_enabled,
              camera_control.controls.back().front_camera_enabled);
    EXPECT_EQ(down_enabled,
              camera_control.controls.back().down_camera_enabled);
  }

  FakeClock clock;
  FakeAutopilot autopilot;
  FakeSetpoint setpoint;
  FakeNmpc nmpc;
  FakeMission mission;
  FakeLanding landing;
  FakeCameraControl camera_control;
  Context context;
  ActiveStateMachine sm;
};

TEST_F(Fixture, ActiveMachineMapsCommandsToSegmentedMissionStates) {
  ActiveCommandDispatcher dispatcher(sm, &mission);
  EXPECT_TRUE(sm.is(boost::sml::state<Idle>));
  EXPECT_TRUE(dispatcher.update(1));
  EXPECT_TRUE(sm.is(boost::sml::state<ArmOnly>));
  EXPECT_TRUE(dispatcher.update(2));
  EXPECT_TRUE(sm.is(boost::sml::state<NmpcHover>));
  EXPECT_TRUE(dispatcher.update(3));
  EXPECT_TRUE(sm.is(boost::sml::state<SuperSegment1>));
  EXPECT_TRUE(dispatcher.update(4));
  EXPECT_TRUE(sm.is(boost::sml::state<SuperSegment2>));
  EXPECT_TRUE(dispatcher.update(5));
  EXPECT_TRUE(sm.is(boost::sml::state<SuperSegment3>));
  EXPECT_TRUE(dispatcher.update(6));
  EXPECT_TRUE(sm.is(boost::sml::state<Landing>));
  EXPECT_TRUE(dispatcher.update(9));
  EXPECT_TRUE(sm.is(boost::sml::state<Emergency>));
  EXPECT_TRUE(dispatcher.update(7));
  EXPECT_TRUE(sm.is(boost::sml::state<SafeNoop>));
  EXPECT_TRUE(dispatcher.update(8));
  EXPECT_TRUE(sm.is(boost::sml::state<SafeNoop>));
  EXPECT_TRUE(dispatcher.update(42));
  EXPECT_TRUE(sm.is(boost::sml::state<SafeNoop>));
  EXPECT_TRUE(dispatcher.update(0));
  EXPECT_TRUE(sm.is(boost::sml::state<Idle>));
  EXPECT_EQ(3, mission.resets);
  EXPECT_EQ(1, landing.resets);
  EXPECT_EQ(1, landing.start_closed_loop_calls);
}

TEST_F(Fixture, InitialStatePublishesCamerasDisabledOnEveryTick) {
  EXPECT_TRUE(camera_control.controls.empty());

  sm.process_event(Tick{});
  ASSERT_EQ(1u, camera_control.controls.size());
  ExpectLatestCameraControl(false, false);

  sm.process_event(Tick{});
  ASSERT_EQ(2u, camera_control.controls.size());
  ExpectLatestCameraControl(false, false);
}

TEST_F(Fixture, ActiveDispatcherSuppressesRepeatedCommands) {
  ActiveCommandDispatcher dispatcher(sm, &mission);
  EXPECT_TRUE(dispatcher.update(std::numeric_limits<int>::min()));
  EXPECT_TRUE(sm.is(boost::sml::state<SafeNoop>));
  EXPECT_FALSE(dispatcher.update(std::numeric_limits<int>::min()));
  EXPECT_TRUE(dispatcher.update(3));
  EXPECT_EQ(1, mission.resets);
  EXPECT_FALSE(dispatcher.update(3));
  EXPECT_EQ(1, mission.resets);
  EXPECT_TRUE(dispatcher.update(0));
  EXPECT_TRUE(dispatcher.update(3));
  EXPECT_EQ(2, mission.resets);
}

TEST_F(Fixture, RecognitionSlotsAdvanceOnlyForFirstAndDifferentSecondTarget) {
  sm.process_event(OnCommand3{});
  EXPECT_TRUE(sm.is(boost::sml::state<SuperSegment1>));
  sm.process_event(Tick{});
  ExpectLatestCameraControl(true, false);

  sm.process_event(OnTargetRecognized{"plane"});
  EXPECT_TRUE(sm.is(boost::sml::state<SuperSegment2>));
  EXPECT_EQ("plane", context.recognized_targets[0]);
  EXPECT_TRUE(context.recognized_targets[1].empty());
  sm.process_event(Tick{});
  ExpectLatestCameraControl(false, true);

  sm.process_event(OnTargetRecognized{"plane"});
  EXPECT_TRUE(sm.is(boost::sml::state<SuperSegment2>));
  EXPECT_TRUE(context.recognized_targets[1].empty());

  sm.process_event(OnTargetRecognized{"car"});
  EXPECT_TRUE(sm.is(boost::sml::state<SuperSegment3>));
  EXPECT_EQ("car", context.recognized_targets[1]);
  sm.process_event(Tick{});
  ExpectLatestCameraControl(false, false);
  EXPECT_EQ(3, mission.resets);
}

TEST_F(Fixture, SegmentTimeoutAdvancesFirstTwoSegmentsOnly) {
  sm.process_event(OnCommand3{});
  EXPECT_TRUE(sm.is(boost::sml::state<SuperSegment1>));

  sm.process_event(OnSegmentTimeout{});
  EXPECT_TRUE(sm.is(boost::sml::state<SuperSegment2>));
  sm.process_event(OnSegmentTimeout{});
  EXPECT_TRUE(sm.is(boost::sml::state<SuperSegment3>));
  sm.process_event(OnSegmentTimeout{});
  EXPECT_TRUE(sm.is(boost::sml::state<SuperSegment3>));
  EXPECT_EQ(3, mission.resets);
}

TEST_F(Fixture, FirstRecognitionAfterInitialTimeoutStillAdvances) {
  sm.process_event(OnCommand3{});
  sm.process_event(OnSegmentTimeout{});
  ASSERT_TRUE(sm.is(boost::sml::state<SuperSegment2>));

  sm.process_event(OnTargetRecognized{"ship"});
  EXPECT_TRUE(sm.is(boost::sml::state<SuperSegment3>));
  EXPECT_EQ("ship", context.recognized_targets[0]);
  EXPECT_TRUE(context.recognized_targets[1].empty());
}

TEST_F(Fixture, FinalSegmentCompletionStartsClosedLoopLanding) {
  sm.process_event(OnCommand5{});
  ASSERT_TRUE(sm.is(boost::sml::state<SuperSegment3>));

  sm.process_event(OnFinalSegmentComplete{});
  EXPECT_TRUE(sm.is(boost::sml::state<Landing>));
  EXPECT_EQ(1, landing.resets);
  EXPECT_EQ(1, landing.start_closed_loop_calls);
}

TEST_F(Fixture, IdleAndSafeNoopTicksHaveNoFlightOutput) {
  sm.process_event(Tick{});
  sm.process_event(OnUnsupportedCommand{});
  sm.process_event(Tick{});
  EXPECT_TRUE(autopilot.calls.empty());
  EXPECT_TRUE(setpoint.positions.empty());
  EXPECT_TRUE(setpoint.body_rates.empty());
  EXPECT_TRUE(setpoint.attitudes.empty());
  ASSERT_EQ(2u, camera_control.controls.size());
  ExpectLatestCameraControl(false, false);
}

TEST_F(Fixture, ArmOnlyTickPublishesLowThrust) {
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

TEST_F(Fixture, OffboardArmSharedLogicRunsOnlyInActiveOffboardStates) {
  for (const int command : {1, 2, 3, 4, 5, 6}) {
    SCOPED_TRACE(command);
    ClearOutputs();
    context.telemetry.mode = "MANUAL";
    context.telemetry.armed = false;
    context.landing_reached = false;
    context.last_service_request = 0.0;
    clock.value = 5.001;
    ActiveStateMachine machine(context);
    ActiveCommandDispatcher dispatcher(machine, &mission);
    dispatcher.update(command);
    machine.process_event(Tick{});
    ASSERT_FALSE(autopilot.calls.empty());
    EXPECT_EQ("offboard", autopilot.calls[0]);
  }

  for (const int command : {0, 7, 8, 9}) {
    SCOPED_TRACE(command);
    ClearOutputs();
    context.telemetry.mode = "MANUAL";
    context.telemetry.armed = false;
    context.landing_reached = false;
    context.last_service_request = 0.0;
    clock.value = 5.001;
    ActiveStateMachine machine(context);
    ActiveCommandDispatcher dispatcher(machine, &mission);
    dispatcher.update(command);
    machine.process_event(Tick{});
    EXPECT_TRUE(autopilot.calls.empty());
  }
}

TEST_F(Fixture, ActiveOffboardStatesDoNotRequestServicesWhenAlreadyReady) {
  for (const int command : {1, 2, 3, 4, 5, 6}) {
    SCOPED_TRACE(command);
    ClearOutputs();
    SetOffboardAndArmed();
    context.landing_reached = false;
    clock.value = 100.0;
    ActiveStateMachine machine(context);
    ActiveCommandDispatcher dispatcher(machine, &mission);
    dispatcher.update(command);
    machine.process_event(Tick{});
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

TEST_F(Fixture, MissionSuperUsesMissionReferenceAndNmpcMonitor) {
  MissionStateMachine machine(context);
  SetOffboardAndArmed();
  context.telemetry.position = {1.0, 2.0, 3.0};
  context.telemetry.velocity = {4.0, 5.0, 6.0};
  mission.super_points = {ReferencePoint{}};
  mission.super_points[0].position = {16.0, 26.0, 36.0};
  mission.super_points[0].velocity = {6.0, 12.0, 18.0};
  nmpc.track_output = BodyRateThrust{{0.6, 1.2, 1.8}, 0.46};

  machine.process_event(OnCommand3{});
  EXPECT_EQ(1, mission.resets);
  clock.value = 6.0;
  machine.process_event(Tick{});

  EXPECT_EQ(1, mission.super_calls);
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

TEST_F(Fixture, MissionSuperRejectsMissingReferenceSolveFailureAndBadOutput) {
  MissionStateMachine machine(context);
  SetOffboardAndArmed();
  machine.process_event(OnCommand3{});

  mission.super_result = false;
  machine.process_event(Tick{});
  EXPECT_EQ(0, nmpc.track_calls);
  EXPECT_TRUE(setpoint.body_rates.empty());
  EXPECT_TRUE(setpoint.monitors.empty());

  mission.super_result = true;
  mission.super_points.clear();
  machine.process_event(Tick{});
  EXPECT_EQ(0, nmpc.track_calls);

  mission.super_points = {ReferencePoint{}};
  nmpc.track_result = false;
  machine.process_event(Tick{});
  EXPECT_EQ(1, nmpc.track_calls);
  EXPECT_TRUE(setpoint.body_rates.empty());
  EXPECT_TRUE(setpoint.monitors.empty());
  EXPECT_EQ(1u, setpoint.reference_positions.size());
  EXPECT_EQ(1u, setpoint.feedback_positions.size());

  ClearOutputs();
  mission.super_points = {ReferencePoint{}};
  nmpc.track_result = true;
  nmpc.track_output.thrust = std::numeric_limits<double>::quiet_NaN();
  machine.process_event(Tick{});
  EXPECT_EQ(1, nmpc.track_calls);
  EXPECT_TRUE(setpoint.body_rates.empty());
  EXPECT_TRUE(setpoint.monitors.empty());
  EXPECT_EQ(1u, setpoint.reference_positions.size());
  EXPECT_EQ(1u, setpoint.feedback_positions.size());
}

TEST_F(Fixture, LandingUsesPrecisionHorizonAndNmpcMonitor) {
  SetOffboardAndArmed();
  context.telemetry.position = {2.0, 3.0, 0.11};
  landing.points[0].position = {2.1, 2.9, 0.10};
  nmpc.track_output = BodyRateThrust{{0.2, 0.3, 0.4}, 0.5};
  clock.value = 12.0;

  sm.process_event(OnCommand6{});
  EXPECT_EQ(1, landing.resets);
  EXPECT_EQ(1, landing.start_closed_loop_calls);
  EXPECT_DOUBLE_EQ(2.0, landing.start_telemetry.position.x);
  EXPECT_DOUBLE_EQ(3.0, landing.start_telemetry.position.y);
  sm.process_event(Tick{});

  EXPECT_EQ(1, landing.prepare_calls);
  EXPECT_EQ(1, landing.closed_loop_prepare_calls);
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
  sm.process_event(OnCommand6{});

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

TEST_F(Fixture, LandingCompletionIgnoresPreparedResultAndKeepsLowThrust) {
  SetOffboardAndArmed();
  context.config.low_thrust = 0.031;
  context.telemetry.position =
      {2.0, 3.0, context.config.landing_reference_z};
  sm.process_event(OnCommand6{});
  landing.result = false;
  sm.process_event(Tick{});
  EXPECT_TRUE(context.landing_reached);
  ASSERT_EQ(1u, setpoint.body_rates.size());
  EXPECT_DOUBLE_EQ(0.031, setpoint.body_rates.back().thrust);
  EXPECT_DOUBLE_EQ(0.0, setpoint.body_rates.back().body_rate.x);
  EXPECT_DOUBLE_EQ(0.0, setpoint.body_rates.back().body_rate.y);
  EXPECT_DOUBLE_EQ(0.0, setpoint.body_rates.back().body_rate.z);

  sm.process_event(Tick{});
  ASSERT_EQ(2u, setpoint.body_rates.size());
  EXPECT_DOUBLE_EQ(0.031, setpoint.body_rates.back().thrust);
  EXPECT_TRUE(autopilot.calls.empty());

  context.telemetry.mode = "POSCTL";
  sm.process_event(Tick{});
  ASSERT_EQ(3u, setpoint.body_rates.size());
  EXPECT_DOUBLE_EQ(0.031, setpoint.body_rates.back().thrust);
  EXPECT_TRUE(autopilot.calls.empty());
}

TEST_F(Fixture, LandingHeightToleranceCanStillLatchAsFallback) {
  SetOffboardAndArmed();
  context.config.landing_reference_z = 0.05;
  context.config.landing_tolerance_z = 0.05;
  context.telemetry.position = {4.0, 5.0, 0.10};
  sm.process_event(OnCommand6{});
  landing.result = false;

  sm.process_event(Tick{});
  EXPECT_FALSE(context.landing_reached);
  EXPECT_TRUE(setpoint.body_rates.empty());
  context.telemetry.position.z = 0.06;
  sm.process_event(Tick{});
  EXPECT_TRUE(context.landing_reached);
  ASSERT_EQ(1u, setpoint.body_rates.size());
  EXPECT_DOUBLE_EQ(context.config.low_thrust,
                   setpoint.body_rates.back().thrust);
}

TEST_F(Fixture, LandingReachedKeepsLowThrustAcrossFlightModesAndArmStates) {
  sm.process_event(OnCommand6{});
  context.landing_reached = true;
  context.config.low_thrust = 0.027;

  context.telemetry.mode = "OFFBOARD";
  context.telemetry.armed = true;
  sm.process_event(Tick{});
  ASSERT_EQ(1u, setpoint.body_rates.size());
  EXPECT_DOUBLE_EQ(0.027, setpoint.body_rates.back().thrust);
  EXPECT_TRUE(autopilot.calls.empty());

  context.telemetry.mode = "POSCTL";
  context.telemetry.armed = false;
  sm.process_event(Tick{});
  ASSERT_EQ(2u, setpoint.body_rates.size());
  EXPECT_DOUBLE_EQ(0.027, setpoint.body_rates.back().thrust);
  EXPECT_TRUE(autopilot.calls.empty());

  context.telemetry.armed = true;
  sm.process_event(Tick{});
  ASSERT_EQ(3u, setpoint.body_rates.size());
  EXPECT_DOUBLE_EQ(0.027, setpoint.body_rates.back().thrust);
  EXPECT_TRUE(autopilot.calls.empty());
}

TEST_F(Fixture, MissionMachineMapsCommandsToArmHoverSuperLanding) {
  MissionStateMachine machine(context);
  MissionCommandDispatcher dispatcher(machine, &mission);
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
  SegmentedMissionCommandDispatcher dispatcher(machine, &mission);
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

TEST_F(Fixture, SegmentedMissionPublishesCameraStateOnEveryTick) {
  SegmentedMissionStateMachine machine(context);
  SegmentedMissionCommandDispatcher dispatcher(machine, &mission);
  EXPECT_TRUE(camera_control.controls.empty());

  machine.process_event(Tick{});
  ASSERT_EQ(1u, camera_control.controls.size());
  ExpectLatestCameraControl(false, false);

  EXPECT_TRUE(dispatcher.update(2));
  machine.process_event(Tick{});
  ExpectLatestCameraControl(false, false);

  const std::size_t initial_count = camera_control.controls.size();
  EXPECT_TRUE(dispatcher.update(3));
  EXPECT_EQ(initial_count, camera_control.controls.size());
  machine.process_event(Tick{});
  ASSERT_EQ(initial_count + 1u, camera_control.controls.size());
  ExpectLatestCameraControl(true, false);

  machine.process_event(Tick{});
  ASSERT_EQ(initial_count + 2u, camera_control.controls.size());
  ExpectLatestCameraControl(true, false);

  EXPECT_FALSE(dispatcher.update(3));
  EXPECT_EQ(initial_count + 2u, camera_control.controls.size());
  machine.process_event(Tick{});
  ASSERT_EQ(initial_count + 3u, camera_control.controls.size());
  ExpectLatestCameraControl(true, false);

  EXPECT_TRUE(dispatcher.update(4));
  machine.process_event(Tick{});
  ExpectLatestCameraControl(false, true);

  EXPECT_TRUE(dispatcher.update(5));
  machine.process_event(Tick{});
  ExpectLatestCameraControl(false, false);

  EXPECT_TRUE(dispatcher.update(6));
  machine.process_event(Tick{});
  ExpectLatestCameraControl(false, false);

  EXPECT_TRUE(dispatcher.update(9));
  machine.process_event(Tick{});
  ExpectLatestCameraControl(false, false);

  EXPECT_TRUE(dispatcher.update(7));
  machine.process_event(Tick{});
  ExpectLatestCameraControl(false, false);

  EXPECT_TRUE(dispatcher.update(0));
  machine.process_event(Tick{});
  ExpectLatestCameraControl(false, false);
}

TEST_F(Fixture, MissionMachinePublishesCameraStateOnTick) {
  MissionStateMachine mission_machine(context);
  MissionCommandDispatcher mission_dispatcher(mission_machine, &mission);
  EXPECT_TRUE(mission_dispatcher.update(3));
  mission_machine.process_event(Tick{});
  ExpectLatestCameraControl(true, false);
  EXPECT_TRUE(mission_dispatcher.update(4));
  mission_machine.process_event(Tick{});
  ExpectLatestCameraControl(false, true);
}

TEST(ClosedLoopLandingPlannerTest, HoldsEntryPositionWithoutVision) {
  ClosedLoopLandingConfig config;
  config.horizon_points = 3;
  ClosedLoopLandingPlanner planner(config);
  TelemetrySnapshot entry;
  entry.position = {1.0, 2.0, 1.5};
  planner.start(entry);

  TelemetrySnapshot drifted = entry;
  drifted.position = {4.0, 5.0, 1.2};
  std::vector<ReferencePoint> horizon;
  ASSERT_TRUE(planner.prepare(10.0, drifted, horizon));
  ASSERT_EQ(3u, horizon.size());
  EXPECT_DOUBLE_EQ(1.0, horizon.front().position.x);
  EXPECT_DOUBLE_EQ(2.0, horizon.front().position.y);
  EXPECT_DOUBLE_EQ(1.5, horizon.front().position.z);
  EXPECT_FALSE(planner.descending());
}

TEST(ClosedLoopLandingPlannerTest,
     AppliesEachFreshObservationOnceAndThenHoldsTarget) {
  ClosedLoopLandingConfig config;
  config.align_px_threshold = 50.0;
  config.xy_step = 0.1;
  config.observation_timeout = 0.5;
  ClosedLoopLandingPlanner planner(config);
  TelemetrySnapshot telemetry;
  telemetry.position = {1.0, 2.0, 1.5};
  planner.start(telemetry);
  planner.updateObservation(
      LandingObservation{true, 80.0, 20.0, 5, 9.9, 0.0});

  std::vector<ReferencePoint> horizon;
  ASSERT_TRUE(planner.prepare(10.0, telemetry, horizon));
  EXPECT_DOUBLE_EQ(1.1, horizon.front().position.x);
  EXPECT_DOUBLE_EQ(2.0, horizon.front().position.y);

  telemetry.position = {1.04, 2.03, 1.5};
  ASSERT_TRUE(planner.prepare(10.1, telemetry, horizon));
  EXPECT_DOUBLE_EQ(1.1, horizon.front().position.x);
  EXPECT_DOUBLE_EQ(2.0, horizon.front().position.y);
}

TEST(ClosedLoopLandingPlannerTest, SupportsAxisSwapAndIndependentSigns) {
  ClosedLoopLandingConfig config;
  config.align_px_threshold = 50.0;
  config.xy_step = 0.1;
  config.swap_xy = true;
  config.x_sign = -1.0;
  config.y_sign = 1.0;
  ClosedLoopLandingPlanner planner(config);
  TelemetrySnapshot telemetry;
  telemetry.position = {4.0, 5.0, 1.0};
  planner.start(telemetry);
  planner.updateObservation(
      LandingObservation{true, -80.0, 90.0, 5, 2.0, 0.0});

  std::vector<ReferencePoint> horizon;
  ASSERT_TRUE(planner.prepare(2.1, telemetry, horizon));
  EXPECT_DOUBLE_EQ(3.9, horizon.front().position.x);
  EXPECT_DOUBLE_EQ(4.9, horizon.front().position.y);
}

TEST(ClosedLoopLandingPlannerTest, LocksCurrentXyAndDescendsWhenAligned) {
  ClosedLoopLandingConfig config;
  config.align_px_threshold = 50.0;
  config.descent_rate = 0.2;
  config.control_rate_hz = 10.0;
  config.min_z = 0.1;
  ClosedLoopLandingPlanner planner(config);
  TelemetrySnapshot telemetry;
  telemetry.position = {1.5, 2.5, 2.0};
  planner.start(TelemetrySnapshot{});
  planner.updateObservation(
      LandingObservation{true, 50.0, -50.0, 5, 4.0, 0.0});

  std::vector<ReferencePoint> horizon;
  ASSERT_TRUE(planner.prepare(4.1, telemetry, horizon));
  EXPECT_TRUE(planner.descending());
  EXPECT_DOUBLE_EQ(1.5, horizon.front().position.x);
  EXPECT_DOUBLE_EQ(2.5, horizon.front().position.y);
  EXPECT_NEAR(1.98, horizon.front().position.z, 1e-12);

  telemetry.position = {1.7, 2.7, 1.99};
  ASSERT_TRUE(planner.prepare(4.2, telemetry, horizon));
  EXPECT_DOUBLE_EQ(1.5, horizon.front().position.x);
  EXPECT_DOUBLE_EQ(2.5, horizon.front().position.y);
  EXPECT_NEAR(1.96, horizon.front().position.z, 1e-12);
}

TEST(ClosedLoopLandingPlannerTest, FourAlignedTagsLockXyAndStartDescent) {
  ClosedLoopLandingConfig config;
  config.align_px_threshold = 50.0;
  config.descent_rate = 0.2;
  config.control_rate_hz = 10.0;
  ClosedLoopLandingPlanner planner(config);
  TelemetrySnapshot telemetry;
  telemetry.position = {2.0, 3.0, 1.5};
  planner.start(telemetry);
  planner.updateObservation(
      LandingObservation{true, 40.0, -30.0, 4, 5.0, 0.0});

  std::vector<ReferencePoint> horizon;
  ASSERT_TRUE(planner.prepare(5.1, telemetry, horizon));
  EXPECT_TRUE(planner.descending());
  EXPECT_FALSE(planner.adjusting());
  EXPECT_DOUBLE_EQ(2.0, horizon.front().position.x);
  EXPECT_DOUBLE_EQ(3.0, horizon.front().position.y);
  EXPECT_NEAR(1.48, horizon.front().position.z, 1e-12);
}

TEST(ClosedLoopLandingPlannerTest,
     PartialTagsAdjustButCannotStartBlindDescent) {
  ClosedLoopLandingConfig config;
  config.align_px_threshold = 50.0;
  config.xy_step = 0.1;
  config.adjust_duration_tag3 = 0.8;
  ClosedLoopLandingPlanner planner(config);
  TelemetrySnapshot telemetry;
  telemetry.position = {1.0, 2.0, 1.5};
  planner.start(telemetry);
  planner.updateObservation(
      LandingObservation{true, 80.0, 20.0, 3, 10.0, 0.0});

  std::vector<ReferencePoint> horizon;
  ASSERT_TRUE(planner.prepare(10.1, telemetry, horizon));
  EXPECT_TRUE(planner.adjusting());
  EXPECT_FALSE(planner.descending());
  EXPECT_DOUBLE_EQ(1.1, horizon.front().position.x);
  EXPECT_DOUBLE_EQ(2.0, horizon.front().position.y);
}

TEST(ClosedLoopLandingPlannerTest,
     IgnoresVisionDuringAdjustmentAndRequiresANewFrameAfterward) {
  ClosedLoopLandingConfig config;
  config.align_px_threshold = 50.0;
  config.xy_step = 0.1;
  config.observation_timeout = 0.5;
  config.adjust_duration_tag1 = 1.0;
  ClosedLoopLandingPlanner planner(config);
  TelemetrySnapshot telemetry;
  telemetry.position = {1.0, 2.0, 1.5};
  planner.start(telemetry);
  planner.updateObservation(
      LandingObservation{true, 80.0, 0.0, 1, 10.0, 0.0});

  std::vector<ReferencePoint> horizon;
  ASSERT_TRUE(planner.prepare(10.1, telemetry, horizon));
  ASSERT_TRUE(planner.adjusting());
  EXPECT_DOUBLE_EQ(1.1, horizon.front().position.x);

  planner.updateObservation(
      LandingObservation{true, 0.0, 0.0, 5, 10.5, 0.0});
  telemetry.position = {1.08, 2.0, 1.5};
  ASSERT_TRUE(planner.prepare(11.2, telemetry, horizon));
  EXPECT_FALSE(planner.adjusting());
  EXPECT_FALSE(planner.descending());
  EXPECT_DOUBLE_EQ(1.1, horizon.front().position.x);

  planner.updateObservation(
      LandingObservation{true, 0.0, 0.0, 5, 11.25, 0.0});
  ASSERT_TRUE(planner.prepare(11.3, telemetry, horizon));
  EXPECT_TRUE(planner.descending());
  EXPECT_DOUBLE_EQ(1.08, horizon.front().position.x);
  EXPECT_DOUBLE_EQ(2.0, horizon.front().position.y);
}

TEST(ClosedLoopLandingPlannerTest, FewerTagsUseLongerAdjustmentTime) {
  ClosedLoopLandingConfig config;
  config.align_px_threshold = 50.0;
  config.adjust_duration_tag1 = 1.2;
  config.adjust_duration_tag5 = 0.4;
  ClosedLoopLandingPlanner one_tag_planner(config);
  ClosedLoopLandingPlanner five_tag_planner(config);
  TelemetrySnapshot telemetry;
  telemetry.position = {1.0, 2.0, 1.5};
  one_tag_planner.start(telemetry);
  five_tag_planner.start(telemetry);
  one_tag_planner.updateObservation(
      LandingObservation{true, 80.0, 0.0, 1, 20.0, 0.0});
  five_tag_planner.updateObservation(
      LandingObservation{true, 80.0, 0.0, 5, 20.0, 0.0});

  std::vector<ReferencePoint> horizon;
  ASSERT_TRUE(one_tag_planner.prepare(20.1, telemetry, horizon));
  ASSERT_TRUE(five_tag_planner.prepare(20.1, telemetry, horizon));
  ASSERT_TRUE(one_tag_planner.prepare(20.6, telemetry, horizon));
  ASSERT_TRUE(five_tag_planner.prepare(20.6, telemetry, horizon));
  EXPECT_TRUE(one_tag_planner.adjusting());
  EXPECT_FALSE(five_tag_planner.adjusting());
  EXPECT_FALSE(one_tag_planner.descending());
  EXPECT_FALSE(five_tag_planner.descending());
}

TEST(ClosedLoopLandingPlannerTest, IgnoresStaleVisionAndKeepsRecordedHold) {
  ClosedLoopLandingConfig config;
  config.observation_timeout = 0.25;
  config.xy_step = 0.1;
  ClosedLoopLandingPlanner planner(config);
  TelemetrySnapshot entry;
  entry.position = {3.0, 4.0, 1.0};
  planner.start(entry);
  planner.updateObservation(
      LandingObservation{true, 100.0, -100.0, 5, 1.0, 0.0});

  TelemetrySnapshot telemetry = entry;
  telemetry.position = {3.2, 4.2, 1.0};
  std::vector<ReferencePoint> horizon;
  ASSERT_TRUE(planner.prepare(1.3, telemetry, horizon));
  EXPECT_DOUBLE_EQ(3.0, horizon.front().position.x);
  EXPECT_DOUBLE_EQ(4.0, horizon.front().position.y);
  EXPECT_FALSE(planner.descending());
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
