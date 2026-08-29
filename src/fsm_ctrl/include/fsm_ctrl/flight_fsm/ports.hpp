#ifndef FSM_CTRL_FLIGHT_FSM_PORTS_HPP_
#define FSM_CTRL_FLIGHT_FSM_PORTS_HPP_

#include <fsm_ctrl/flight_fsm/types.hpp>

#include <vector>

namespace fsm_ctrl {
namespace flight_fsm {

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
  virtual bool solveTrack(const TelemetrySnapshot& telemetry,
                          const std::vector<ReferencePoint>& horizon,
                          BodyRateThrust& command) = 0;
};

class MissionPort {
 public:
  virtual ~MissionPort() = default;
  virtual void selectCommand(int command) {}
  virtual void reset() = 0;
  virtual bool prepareSuper(double now, const TelemetrySnapshot& telemetry,
                            std::vector<ReferencePoint>& horizon) = 0;
  virtual bool prepareSuperSegment(
      int segment_index, double now, const TelemetrySnapshot& telemetry,
      std::vector<ReferencePoint>& horizon) {
    (void)segment_index;
    (void)now;
    (void)telemetry;
    horizon.clear();
    return false;
  }
  virtual bool isSuperSegmentTimedOut(double now) const {
    (void)now;
    return false;
  }
  virtual bool isFinalSuperSegmentComplete() const { return false; }
};

class PrecisionLandingPort {
 public:
  virtual ~PrecisionLandingPort() = default;
  virtual void reset() = 0;
  virtual void updateObservation(const LandingObservation& observation) = 0;
  virtual void startClosedLoopLanding(
      const TelemetrySnapshot& telemetry) {
    (void)telemetry;
  }
  virtual bool prepareLanding(double now, const TelemetrySnapshot& telemetry,
                              std::vector<ReferencePoint>& horizon) = 0;
  virtual bool prepareClosedLoopLanding(
      double now, const TelemetrySnapshot& telemetry,
      std::vector<ReferencePoint>& horizon) {
    return prepareLanding(now, telemetry, horizon);
  }
};

class CameraControlPort {
 public:
  virtual ~CameraControlPort() = default;
  virtual void publishControl(const CameraControlState& control) = 0;
};

}  // namespace flight_fsm
}  // namespace fsm_ctrl

#endif  // FSM_CTRL_FLIGHT_FSM_PORTS_HPP_
