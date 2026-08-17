#ifndef FSM_CTRL_SINGLE_OFFBOARD_SML_TYPES_HPP_
#define FSM_CTRL_SINGLE_OFFBOARD_SML_TYPES_HPP_

#include <string>
#include <vector>

namespace fsm_ctrl {
namespace single_sml {

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Quaternion {
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct TelemetrySnapshot {
  std::string mode;
  bool armed{false};
  Vec3 position;
  Vec3 velocity;
  Quaternion attitude;
};

struct ReferencePoint {
  Vec3 position;
  Vec3 velocity;
  Quaternion attitude;
  // NMPC 输入前馈：SUPER 轨迹已根据 acceleration/jerk 计算出
  // body-rate 和机体 Z 轴总加速度。静态参考默认为零角速度和重力。
  Vec3 body_rate;
  double total_acceleration{9.8015};
};

struct BodyRateThrust {
  Vec3 body_rate;
  double thrust{0.0};
  // NMPC 内部为消除恒定模型偏差而施加的虚拟 XY 参考偏置，仅用于监视。
  Vec3 reference_bias;
};

struct PositionSetpoint {
  Vec3 position;
  double yaw{0.0};
};

struct AttitudeSetpoint {
  Quaternion attitude;
  double thrust{0.0};
};

struct NmpcMonitor {
  std::vector<ReferencePoint> references;
  TelemetrySnapshot feedback;
  BodyRateThrust target;
};

struct LandingObservation {
  bool valid{false};
  double dx{0.0};
  double dy{0.0};
  int tag_count{0};
  double stamp{0.0};
  double age{0.0};
};

// 视觉相机的期望运行状态。它是状态快照而不是一次性的启停脉冲，
// 因而 ROS 适配层可以安全地使用 latched topic 交付给晚启动的节点。
struct CameraControlState {
  bool front_camera_enabled{false};
  bool down_camera_enabled{false};
};

struct Config {
  double service_retry_seconds{5.0};
  double low_thrust{0.02};
  double hover_thrust{0.196};
  double position_hold_z{0.4};
  double landing_target_z{0.005};
  double landing_reference_z{0.01};
  double landing_tolerance_z{0.01};
};

}  // namespace single_sml
}  // namespace fsm_ctrl

#endif  // FSM_CTRL_SINGLE_OFFBOARD_SML_TYPES_HPP_
