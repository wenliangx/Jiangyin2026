#ifndef FSM_CTRL_SINGLE_OFFBOARD_SML_PORTS_HPP_
#define FSM_CTRL_SINGLE_OFFBOARD_SML_PORTS_HPP_

#include <fsm_ctrl/single_offboard_sml_types.hpp>

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
  virtual bool solveLegacy(const LegacyNmpcRequest& request,
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
  virtual void reset(MissionTrackMode mode) = 0;
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
  virtual bool prepareMission(double now, const TelemetrySnapshot& telemetry,
                              std::vector<ReferencePoint>& horizon) = 0;
  virtual bool prepareEgo(double now, const TelemetrySnapshot& telemetry,
                          std::vector<ReferencePoint>& horizon) = 0;
  virtual bool wantsPrecisionLanding() const { return false; }
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

}  // namespace single_sml
}  // namespace fsm_ctrl

#endif  // FSM_CTRL_SINGLE_OFFBOARD_SML_PORTS_HPP_
