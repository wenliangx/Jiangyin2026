#ifndef FSM_CTRL_SINGLE_OFFBOARD_SML_PORTS_CANONICAL_HPP_
#define FSM_CTRL_SINGLE_OFFBOARD_SML_PORTS_CANONICAL_HPP_

#include <fsm_ctrl/single_offboard_sml/types.hpp>

#include <cstddef>
#include <vector>

namespace fsm_ctrl {
namespace single_sml {

class Clock {
 public:
  virtual ~Clock() = default;
  virtual double now() const = 0;
};

class AutopilotPort {
 public:
  virtual ~AutopilotPort() = default;
  virtual bool requestOffboard() = 0;
  virtual bool requestArm() = 0;
  virtual bool requestDisarm() = 0;
};

class SetpointPort {
 public:
  virtual ~SetpointPort() = default;
  virtual void publishPosition(const PositionSetpoint& setpoint) = 0;
  virtual void publishBodyRateThrust(const BodyRateThrust& setpoint) = 0;
  virtual void publishAttitude(const AttitudeSetpoint& setpoint) = 0;
  virtual void publishReferencePosition(const Vec3& position,
                                         const Quaternion& attitude) {}
  virtual void publishFeedbackPosition(const Vec3& position,
                                        const Quaternion& attitude) {}
  virtual void publishNmpcMonitor(const NmpcMonitor& monitor) {}
};

class NmpcPort {
 public:
  virtual ~NmpcPort() = default;
  virtual bool solveHover(const TelemetrySnapshot& telemetry,
                          BodyRateThrust& command) = 0;
  virtual bool solveTrack(const TelemetrySnapshot& telemetry,
                          const std::vector<ReferencePoint>& horizon,
                          BodyRateThrust& command) = 0;
};

class ReferenceProvider {
 public:
  virtual ~ReferenceProvider() = default;
  virtual void selectCommand(int command) {}
  virtual void reset() = 0;
  virtual bool horizon(double now,
                       std::vector<ReferencePoint>& points) = 0;
};

class MissionPort {
 public:
  virtual ~MissionPort() = default;
  virtual void selectCommand(int command) {}
  virtual void reset() = 0;
  // 任务航点是否可用（YAML 加载成功）。super 任务转换用它做 guard。
  virtual bool available() const { return true; }
  virtual bool prepareSuper(double now, const TelemetrySnapshot& telemetry,
                            std::vector<ReferencePoint>& horizon) = 0;
  virtual bool prepareCoreSuperGoal(double now,
                                    const TelemetrySnapshot& telemetry,
                                    const Vec3& goal,
                                    std::vector<ReferencePoint>& horizon) {
    (void)now;
    (void)telemetry;
    (void)goal;
    horizon.clear();
    return false;
  }
  virtual bool prepareSuperSegment(
      int segment_index, double now, const TelemetrySnapshot& telemetry,
      std::vector<ReferencePoint>& horizon) {
    (void)segment_index;
    (void)now;
    (void)telemetry;
    horizon.clear();
    return false;
  }
};

class PrecisionLandingPort {
 public:
  virtual ~PrecisionLandingPort() = default;
  virtual void reset() = 0;
  virtual void updateObservation(const LandingObservation& observation) = 0;
  virtual bool prepareLanding(double now, const TelemetrySnapshot& telemetry,
                              std::vector<ReferencePoint>& horizon) = 0;
  virtual bool isComplete() const = 0;
};

enum class LogSeverity { Debug, Info, Warn, Error };

enum class LogEvent {
  CommandNew,
  CommandRepeatedSuppressed,
  CommandUnsupported,
  ActionArmOnly,
  ActionHoverToOneMeter,
  ActionSuperTrack,
  ActionSuperSegment,
  ActionLanding,
  ActionEmergency,
  EmptyHorizon,
  NmpcSolveFailure,
  NmpcNonFiniteOutput,
  NmpcPublishSuccess,
  LandingLatched,
  LandingDisarmRequested,
};

struct LogRecord {
  LogSeverity severity{LogSeverity::Info};
  LogEvent event{LogEvent::CommandNew};
  double stamp{0.0};
  int command{0};
  int segment_index{-1};
  std::size_t horizon_size{0u};
  Vec3 position;
  Vec3 reference_position;
  BodyRateThrust command_output;
};

class LogPort {
 public:
  virtual ~LogPort() = default;
  virtual void write(const LogRecord& record) = 0;
};

}  // namespace single_sml
}  // namespace fsm_ctrl

#endif  // FSM_CTRL_SINGLE_OFFBOARD_SML_PORTS_CANONICAL_HPP_
