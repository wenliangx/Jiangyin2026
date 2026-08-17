#ifndef FSM_CTRL_SINGLE_OFFBOARD_SML_INTEGRATOR_HPP_
#define FSM_CTRL_SINGLE_OFFBOARD_SML_INTEGRATOR_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <fsm_ctrl/single_offboard_sml_types.hpp>

namespace fsm_ctrl {
namespace single_sml {

struct HorizontalIntegralConfig {
  double gain{0.20};
  double limit{0.20};
  double deadband{0.003};
  double hold_reference_velocity{0.05};
  double hold_horizon_spread{0.02};
  double max_feedback_velocity{0.20};
  double max_update_gap{0.50};
};

// Offset-free outer loop for the simple NMPC. The NMPC plant has no
// disturbance state, so a constant attitude/thrust-axis modelling error
// otherwise requires a permanent position error. This integrator learns the
// virtual reference displacement only while the complete horizon is a hold.
class HorizontalReferenceIntegrator {
 public:
  explicit HorizontalReferenceIntegrator(
      const HorizontalIntegralConfig& config = HorizontalIntegralConfig{}) {
    configure(config);
  }

  void configure(const HorizontalIntegralConfig& config) {
    config_ = config;
    config_.gain = std::max(0.0, config_.gain);
    config_.limit = std::max(0.0, config_.limit);
    config_.deadband = std::max(0.0, config_.deadband);
    config_.hold_reference_velocity =
        std::max(0.0, config_.hold_reference_velocity);
    config_.hold_horizon_spread =
        std::max(0.0, config_.hold_horizon_spread);
    config_.max_feedback_velocity =
        std::max(0.0, config_.max_feedback_velocity);
    config_.max_update_gap = std::max(1e-3, config_.max_update_gap);
    reset();
  }

  Vec3 update(double now, const TelemetrySnapshot& telemetry,
              const std::vector<ReferencePoint>& horizon) {
    if (!telemetry.armed || !std::isfinite(now) || horizon.empty() ||
        !std::isfinite(telemetry.position.x) ||
        !std::isfinite(telemetry.position.y) ||
        !std::isfinite(telemetry.velocity.x) ||
        !std::isfinite(telemetry.velocity.y)) {
      reset();
      return bias_;
    }

    if (!std::isfinite(last_update_)) {
      last_update_ = now;
      return bias_;
    }

    const double dt = now - last_update_;
    last_update_ = now;
    if (!(dt > 0.0) || dt > config_.max_update_gap) {
      if (dt > config_.max_update_gap) {
        bias_ = Vec3{};
      }
      return bias_;
    }

    if (!isHoldHorizon(horizon) ||
        std::hypot(telemetry.velocity.x, telemetry.velocity.y) >
            config_.max_feedback_velocity) {
      return bias_;
    }

    const double error_x =
        applyDeadband(horizon.front().position.x - telemetry.position.x);
    const double error_y =
        applyDeadband(horizon.front().position.y - telemetry.position.y);
    bias_.x = clamp(bias_.x + config_.gain * error_x * dt,
                    -config_.limit, config_.limit);
    bias_.y = clamp(bias_.y + config_.gain * error_y * dt,
                    -config_.limit, config_.limit);
    return bias_;
  }

  const Vec3& bias() const { return bias_; }

  void reset() {
    bias_ = Vec3{};
    last_update_ = std::numeric_limits<double>::quiet_NaN();
  }

 private:
  static double clamp(double value, double low, double high) {
    return std::max(low, std::min(high, value));
  }

  double applyDeadband(double error) const {
    if (error > config_.deadband) return error - config_.deadband;
    if (error < -config_.deadband) return error + config_.deadband;
    return 0.0;
  }

  bool isHoldHorizon(const std::vector<ReferencePoint>& horizon) const {
    const Vec3& origin = horizon.front().position;
    for (const auto& point : horizon) {
      if (!std::isfinite(point.position.x) ||
          !std::isfinite(point.position.y) ||
          !std::isfinite(point.velocity.x) ||
          !std::isfinite(point.velocity.y) ||
          std::hypot(point.velocity.x, point.velocity.y) >
              config_.hold_reference_velocity ||
          std::hypot(point.position.x - origin.x,
                     point.position.y - origin.y) >
              config_.hold_horizon_spread) {
        return false;
      }
    }
    return true;
  }

  HorizontalIntegralConfig config_;
  Vec3 bias_;
  double last_update_{std::numeric_limits<double>::quiet_NaN()};
};

}  // namespace single_sml
}  // namespace fsm_ctrl

#endif  // FSM_CTRL_SINGLE_OFFBOARD_SML_INTEGRATOR_HPP_
