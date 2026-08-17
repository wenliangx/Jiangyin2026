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
  double align_px_threshold{150.0};
  double xy_step{0.03};
  int lock_min_tag_count{3};
  double adjust_duration_tag1{1.2};
  double adjust_duration_tag2{1.0};
  double adjust_duration_tag3{0.8};
  double adjust_duration_tag4{0.6};
  double adjust_duration_tag5{0.4};
  bool swap_xy{false};
  double x_sign{1.0};
  double y_sign{1.0};
  double descent_rate{0.15};
  double min_z{0.01};
  double control_rate_hz{50.0};
  std::size_t horizon_points{6};
};

// 纯 C++ 的视觉闭环降落参考规划器。一条新鲜观测只产生一次
// 水平步进，随后在按标签数量选择的时间内保持该目标并忽略视觉。
// 4/5 个标签且偏差达标后锁定实时 x/y，之后只沿 z 盲降。
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
    config_.lock_min_tag_count =
        std::max(1, std::min(5, config_.lock_min_tag_count));
    config_.adjust_duration_tag1 =
        std::max(0.0, config_.adjust_duration_tag1);
    config_.adjust_duration_tag2 =
        std::max(0.0, config_.adjust_duration_tag2);
    config_.adjust_duration_tag3 =
        std::max(0.0, config_.adjust_duration_tag3);
    config_.adjust_duration_tag4 =
        std::max(0.0, config_.adjust_duration_tag4);
    config_.adjust_duration_tag5 =
        std::max(0.0, config_.adjust_duration_tag5);
    config_.x_sign = config_.x_sign < 0.0 ? -1.0 : 1.0;
    config_.y_sign = config_.y_sign < 0.0 ? -1.0 : 1.0;
    config_.descent_rate = std::max(0.0, config_.descent_rate);
    config_.control_rate_hz = std::max(1.0, config_.control_rate_hz);
    config_.horizon_points = std::max<std::size_t>(1, config_.horizon_points);
  }

  void reset() {
    initialized_ = false;
    stage_ = Stage::Observe;
    adjust_until_ = -std::numeric_limits<double>::infinity();
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
    // 调整期间必须完全忽略视觉，不缓存这段时间内的最后一帧。
    // 否则调整截止的同一 Tick 会立即消费运动中的旧观测。
    if (stage_ != Stage::Observe) {
      return;
    }
    last_observation_ = observation;
  }

  bool prepare(double now, const TelemetrySnapshot& telemetry,
               std::vector<ReferencePoint>& horizon) {
    if (!initialized_) {
      start(telemetry);
    }

    if (stage_ == Stage::Adjust && now >= adjust_until_) {
      stage_ = Stage::Observe;
    }

    if (stage_ == Stage::Observe && hasNewFreshObservation(now)) {
      last_consumed_stamp_ = last_observation_.stamp;
      target_.x = telemetry.position.x;
      target_.y = telemetry.position.y;

      const bool aligned =
          last_observation_.tag_count >= config_.lock_min_tag_count &&
          std::abs(last_observation_.dx) <= config_.align_px_threshold &&
          std::abs(last_observation_.dy) <= config_.align_px_threshold;
      if (aligned) {
        // 对准时锁住实时位置和高度，从这里开始只沿 z 下降。
        target_.z = telemetry.position.z;
        stage_ = Stage::Descend;
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
        adjust_until_ =
            now + adjustmentDuration(last_observation_.tag_count);
        stage_ = Stage::Adjust;
      }
    }

    if (stage_ == Stage::Descend) {
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

  bool adjusting() const { return stage_ == Stage::Adjust; }
  bool descending() const { return stage_ == Stage::Descend; }
  const Vec3& target() const { return target_; }
  const LandingObservation& lastObservation() const {
    return last_observation_;
  }

 private:
  bool hasNewFreshObservation(double now) const {
    return last_observation_.valid && std::isfinite(last_observation_.stamp) &&
           last_observation_.tag_count >= 1 &&
           last_observation_.tag_count <= 5 &&
           last_observation_.stamp >= 0.0 &&
           now >= last_observation_.stamp &&
           now - last_observation_.stamp <= config_.observation_timeout &&
           last_observation_.stamp > last_consumed_stamp_;
  }

  double signedStep(double error) const {
    return error < 0.0 ? -config_.xy_step : config_.xy_step;
  }

  double adjustmentDuration(int tag_count) const {
    switch (tag_count) {
      case 1: return config_.adjust_duration_tag1;
      case 2: return config_.adjust_duration_tag2;
      case 3: return config_.adjust_duration_tag3;
      case 4: return config_.adjust_duration_tag4;
      case 5: return config_.adjust_duration_tag5;
      default: return config_.adjust_duration_tag1;
    }
  }

  enum class Stage { Observe, Adjust, Descend };

  ClosedLoopLandingConfig config_;
  bool initialized_{false};
  Stage stage_{Stage::Observe};
  double adjust_until_{-std::numeric_limits<double>::infinity()};
  Vec3 target_;
  LandingObservation last_observation_;
  double last_consumed_stamp_{-std::numeric_limits<double>::infinity()};
};

}  // namespace single_sml
}  // namespace fsm_ctrl

#endif  // FSM_CTRL_SINGLE_OFFBOARD_SML_LANDING_HPP_
