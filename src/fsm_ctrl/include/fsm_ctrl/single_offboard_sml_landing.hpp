#ifndef FSM_CTRL_SINGLE_OFFBOARD_SML_LANDING_HPP_
#define FSM_CTRL_SINGLE_OFFBOARD_SML_LANDING_HPP_

#include <fsm_ctrl/single_offboard_sml_types.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace fsm_ctrl {
namespace single_sml {

struct ClosedLoopLandingConfig {
  double observation_timeout{0.25};
  double align_px_threshold{50.0};
  double xy_step{0.03};
  bool swap_xy{false};
  double x_sign{1.0};
  double y_sign{1.0};
  double descent_rate{0.15};
  double min_z{0.03};
  double control_rate_hz{50.0};
  std::size_t horizon_points{6};
};

// 纯 C++ 的视觉闭环降落参考规划器。每条新鲜观测最多产生一次水平
// 步进；观测丢失时持续输出上一次记录的目标，避免相同消息在 50Hz Tick
// 中被重复累加。
class ClosedLoopLandingPlanner {
 public:
  explicit ClosedLoopLandingPlanner(
      const ClosedLoopLandingConfig& config = ClosedLoopLandingConfig{}) {
    configure(config);
    reset();
  }

  void configure(const ClosedLoopLandingConfig& config) {
    config_ = config;
    config_.observation_timeout =
        std::max(0.0, config_.observation_timeout);
    config_.align_px_threshold =
        std::max(0.0, config_.align_px_threshold);
    config_.xy_step = std::abs(config_.xy_step);
    config_.x_sign = config_.x_sign < 0.0 ? -1.0 : 1.0;
    config_.y_sign = config_.y_sign < 0.0 ? -1.0 : 1.0;
    config_.descent_rate = std::max(0.0, config_.descent_rate);
    config_.control_rate_hz = std::max(1.0, config_.control_rate_hz);
    config_.horizon_points = std::max<std::size_t>(1, config_.horizon_points);
  }

  void reset() {
    initialized_ = false;
    descending_ = false;
    target_ = Vec3{};
    last_observation_ = LandingObservation{};
    last_consumed_stamp_ = -std::numeric_limits<double>::infinity();
  }

  void start(const TelemetrySnapshot& telemetry) {
    reset();
    initialized_ = true;
    target_ = telemetry.position;
  }

  void updateObservation(const LandingObservation& observation) {
    last_observation_ = observation;
  }

  bool prepare(double now, const TelemetrySnapshot& telemetry,
               std::vector<ReferencePoint>& horizon) {
    if (!initialized_) {
      start(telemetry);
    }

    if (!descending_ && hasNewFreshObservation(now)) {
      last_consumed_stamp_ = last_observation_.stamp;
      target_.x = telemetry.position.x;
      target_.y = telemetry.position.y;

      const bool aligned =
          std::abs(last_observation_.dx) <= config_.align_px_threshold &&
          std::abs(last_observation_.dy) <= config_.align_px_threshold;
      if (aligned) {
        // 对准时锁住实时位置和高度，从这里开始只沿 z 下降。
        target_.z = telemetry.position.z;
        descending_ = true;
      } else {
        const double x_error =
            config_.swap_xy ? last_observation_.dy : last_observation_.dx;
        const double y_error =
            config_.swap_xy ? last_observation_.dx : last_observation_.dy;
        if (std::abs(x_error) > config_.align_px_threshold) {
          target_.x += config_.x_sign * signedStep(x_error);
        }
        if (std::abs(y_error) > config_.align_px_threshold) {
          target_.y += config_.y_sign * signedStep(y_error);
        }
      }
    }

    if (descending_) {
      target_.z = std::max(
          config_.min_z,
          target_.z - config_.descent_rate / config_.control_rate_hz);
    }

    horizon.assign(config_.horizon_points, ReferencePoint{});
    for (auto& point : horizon) {
      point.position = target_;
      point.attitude = telemetry.attitude;
    }
    return true;
  }

  bool descending() const { return descending_; }
  const Vec3& target() const { return target_; }
  const LandingObservation& lastObservation() const {
    return last_observation_;
  }

 private:
  bool hasNewFreshObservation(double now) const {
    return last_observation_.valid && std::isfinite(last_observation_.stamp) &&
           last_observation_.stamp >= 0.0 &&
           now >= last_observation_.stamp &&
           now - last_observation_.stamp <= config_.observation_timeout &&
           last_observation_.stamp > last_consumed_stamp_;
  }

  double signedStep(double error) const {
    return error < 0.0 ? -config_.xy_step : config_.xy_step;
  }

  ClosedLoopLandingConfig config_;
  bool initialized_{false};
  bool descending_{false};
  Vec3 target_;
  LandingObservation last_observation_;
  double last_consumed_stamp_{-std::numeric_limits<double>::infinity()};
};

}  // namespace single_sml
}  // namespace fsm_ctrl

#endif  // FSM_CTRL_SINGLE_OFFBOARD_SML_LANDING_HPP_
