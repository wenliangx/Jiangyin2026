#ifndef FSM_CTRL_SINGLE_OFFBOARD_SML_HPP_
#define FSM_CTRL_SINGLE_OFFBOARD_SML_HPP_

#include <boost/sml.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

// single_offboard_sml 的 Boost.SML 状态机核心。
//
// 本文件只保留纯 C++ 状态机：状态机只接收 typed event、遥测快照和 Port
// 接口，不直接依赖 ROS、CasADi 或 MAVROS。ROS adapter 负责把 UDP 整数命令
// 转成 Select* 事件；50Hz 主循环负责发送 Tick 事件执行当前状态动作。
namespace fsm_ctrl {
namespace single_sml {

// 三维向量：统一表示位置、速度、角速度等 xyz 数据。
struct Vec3 {
  double x{0.0};  // x 轴分量。
  double y{0.0};  // y 轴分量。
  double z{0.0};  // z 轴分量。
};

// 四元数姿态：默认值为单位姿态。
struct Quaternion {
  double w{1.0};  // 实部。
  double x{0.0};  // x 轴虚部分量。
  double y{0.0};  // y 轴虚部分量。
  double z{0.0};  // z 轴虚部分量。
};

// 飞机当前状态快照：由 ROS adapter 从 MAVROS 回调中整理后写入。
struct TelemetrySnapshot {
  std::string mode;       // 当前飞控模式，例如 OFFBOARD。
  bool armed{false};      // 当前是否已解锁。
  Vec3 position;          // 当前本地位置。
  Vec3 velocity;          // 当前本地速度。
  Quaternion attitude;    // 当前姿态。
};

// 一帧轨迹参考点：NMPC 跟踪 horizon 由多个 ReferencePoint 组成。
struct ReferencePoint {
  Vec3 position;        // 期望位置。
  Vec3 velocity;        // 期望速度。
  Quaternion attitude;  // 期望姿态。
};

// 角速度加推力控制量：对应 MAVROS body_rate setpoint。
struct BodyRateThrust {
  Vec3 body_rate;       // 机体系角速度指令。
  double thrust{0.0};   // 归一化推力指令。
};

// 位置控制量：用于 cmd2 定点和 cmd4 下降位置点。
struct PositionSetpoint {
  Vec3 position;      // 期望位置。
  double yaw{0.0};    // 期望偏航角。
};

// 姿态加推力控制量：用于 cmd9 应急姿态输出。
struct AttitudeSetpoint {
  Quaternion attitude;  // 期望姿态。
  double thrust{0.0};   // 归一化推力指令。
};

// NMPC 监视消息的纯 C++ 中间格式。
struct NmpcMonitor {
  std::vector<ReferencePoint> references;  // 当前求解使用的参考轨迹。
  TelemetrySnapshot feedback;              // 当前反馈状态。
  BodyRateThrust target;                   // NMPC 求出的控制量。
};

// 旧版 younger_ctrl NMPC 请求的纯 C++ 中间格式。
struct LegacyNmpcRequest {
  TelemetrySnapshot telemetry;             // 求解时使用的反馈状态。
  std::vector<ReferencePoint> horizon;     // 求解时使用的参考轨迹。
  std::array<double, 4> desired_controls{{9.8, 0.0, 0.0, 0.0}};  // 旧接口默认期望控制量。
};

enum class MissionTrackMode {
  Super,    // cmd6：simple NMPC 任务轨迹跟踪。
  Mission,  // cmd7：legacy mission 跟踪。
  Ego,      // cmd8：legacy ego planner 跟踪。
};

// Port 接口是状态机和外部系统之间的边界。
// 实机代码用 ROS/MAVROS/NMPC 实现这些接口；单元测试用 fake 实现，
// 因此可以不启动 PX4、ROS 或 CasADi 就测试状态机行为。
class Clock {
 public:
  virtual ~Clock() = default;
  virtual double now() const = 0;
};

// 飞控服务接口：封装 OFFBOARD、解锁和上锁请求。
class AutopilotPort {
 public:
  virtual ~AutopilotPort() = default;
  virtual bool requestOffboard() = 0;
  virtual bool requestArm() = 0;
  virtual bool requestDisarm() = 0;
};

// 控制输出接口：封装位置、角速度/推力、姿态/推力等 setpoint 发布。
// 监视量和参考/反馈发布函数默认空实现，测试或简化 adapter 可只实现需要的输出。
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

// NMPC 求解接口：
//   solveHover  -> cmd3 固定点悬停
//   solveTrack  -> cmd5/cmd6 simple NMPC 轨迹跟踪
//   solveLegacy -> cmd7/cmd8 旧版 younger_ctrl 路径
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

// cmd5 参考轨迹接口：负责选择 planner / 内部轨迹 / fallback 悬停参考，
// 并在重新进入 cmd5 时重置内部轨迹状态。
class ReferenceProvider {
 public:
  virtual ~ReferenceProvider() = default;
  virtual void selectCommand(int command) {}
  virtual void reset() = 0;
  virtual bool horizon(double now,
                       std::vector<ReferencePoint>& points) = 0;
};

// cmd6/7/8 任务轨迹接口：封装 waypoint 发布、感知滤波和 legacy mission
// horizon 生成逻辑，避免 SML 转移表直接依赖 ROS topic 或任务细节。
class MissionPort {
 public:
  virtual ~MissionPort() = default;
  virtual void selectCommand(int command) {}
  virtual void reset(MissionTrackMode mode) = 0;
  virtual bool prepareSuper(double now, const TelemetrySnapshot& telemetry,
                            std::vector<ReferencePoint>& horizon) = 0;
  virtual bool prepareMission(double now, const TelemetrySnapshot& telemetry,
                              std::vector<ReferencePoint>& horizon) = 0;
  virtual bool prepareEgo(double now, const TelemetrySnapshot& telemetry,
                          std::vector<ReferencePoint>& horizon) = 0;
  virtual bool wantsPrecisionLanding() const { return false; }
};

// 下视视觉降落观测。dx/dy 是图像像素偏差，不是机体系或世界系距离。
struct LandingObservation {
  bool valid{false};
  double dx{0.0};
  double dy{0.0};
  int tag_count{0};
  double stamp{0.0};
  double age{0.0};
};

// 精准降落接口：ROS adapter 负责把视觉消息写入该端口，SML 核心只请求
// 当前 Tick 可执行的 NMPC horizon。
class PrecisionLandingPort {
 public:
  virtual ~PrecisionLandingPort() = default;
  virtual void reset() = 0;
  virtual void updateObservation(const LandingObservation& observation) = 0;
  virtual bool prepareLanding(double now, const TelemetrySnapshot& telemetry,
                              std::vector<ReferencePoint>& horizon) = 0;
  virtual bool isComplete() const = 0;
};

struct Config {
  // OFFBOARD/arm 服务请求的冷却时间，复现旧版严格大于 5s 的节奏。
  double service_retry_seconds{5.0};
  // cmd1 低油门输出。
  double low_thrust{0.02};
  // cmd9 应急输出使用的悬停油门基准。
  double hover_thrust{0.196};
  // cmd2 位置保持高度，也是启动 warmup 的高度。
  double position_hold_z{0.4};
  // cmd4 降落位置目标高度。
  double landing_target_z{0.005};
  // cmd4 判断接近地面的参考高度。
  double landing_reference_z{0.05};
  // cmd4 进入 landing_reached latch 的高度容差。
  double landing_tolerance_z{0.05};
};

struct Context {
  Context(Clock& clock_in, AutopilotPort& autopilot_in,
          SetpointPort& setpoint_in, NmpcPort& nmpc_in,
          ReferenceProvider& reference_in, MissionPort& mission_in,
          PrecisionLandingPort& landing_in,
          const Config& config_in = Config{})
      : clock(clock_in),
        autopilot(autopilot_in),
        setpoint(setpoint_in),
        nmpc(nmpc_in),
        reference(reference_in),
        mission(mission_in),
        landing(landing_in),
        config(config_in),
        last_service_request(clock_in.now()) {}

  // 共用 OFFBOARD/arm 逻辑：需要 OFFBOARD 时先请求模式切换；
  // 已经是 OFFBOARD 但未解锁时再请求 arm。请求失败也刷新冷却时间。
  void ensureOffboardArm() {
    const double current_time = clock.now();
    if (telemetry.mode != "OFFBOARD" &&
        current_time - last_service_request > config.service_retry_seconds) {
      autopilot.requestOffboard();
      last_service_request = current_time;
    } else if (!telemetry.armed &&
               current_time - last_service_request >
                   config.service_retry_seconds) {
      autopilot.requestArm();
      last_service_request = current_time;
    }
  }

  // 检查 NMPC 输出是否全为有限值；失败时不发布控制量。
  static bool finite(const BodyRateThrust& command) {
    return std::isfinite(command.body_rate.x) &&
           std::isfinite(command.body_rate.y) &&
           std::isfinite(command.body_rate.z) &&
           std::isfinite(command.thrust);
  }

  // 外部依赖接口引用；Context 不拥有这些对象。
  Clock& clock;
  AutopilotPort& autopilot;
  SetpointPort& setpoint;
  NmpcPort& nmpc;
  ReferenceProvider& reference;
  MissionPort& mission;
  PrecisionLandingPort& landing;
  // 状态机配置参数。
  Config config;
  // ROS adapter 每次回调更新的最新遥测快照。
  TelemetrySnapshot telemetry;
  // 最近一次 OFFBOARD/arm 请求时间，用于服务调用冷却。
  double last_service_request;
  // cmd4 降落 latch；一旦置位就停止继续发布下降位置点。
  bool landing_reached{false};
};

struct Idle {};
struct LowThrust {};
struct PositionHold {};
struct NmpcHover {};
struct Landing {};
struct NmpcTrack {};
struct SuperTrack {};
struct MissionTrack {};
struct EgoTrack {};
struct Emergency {};
struct SafeNoop {};

// Select 事件用于切换状态，由 CommandDispatcher 根据 UDP 整数命令生成。
// 状态机内部不直接 switch 原始 int 命令。
// Tick 事件不选择状态，只执行当前状态的周期动作。
struct SelectIdle {};
struct SelectLowThrust {};
struct SelectPositionHold {};
struct SelectNmpcHover {};
struct SelectLanding {};
struct SelectNmpcTrack {};
struct SelectSuperTrack {};
struct SelectMissionTrack {};
struct SelectEgoTrack {};
struct SelectEmergency {};
struct SelectSafeNoop {};
struct Tick {};

// Tick 动作函数：当前状态保持激活时以 50Hz 调用。
// 转移表中的 event<Tick> 行没有目标状态，因此只执行动作、不切状态。
struct Noop {
  void operator()() const {}
};

// cmd1 周期动作：请求 OFFBOARD/arm，然后发布固定低推力。
struct TickLowThrust {
  void operator()(Context& context) const {
    context.ensureOffboardArm();
    context.setpoint.publishBodyRateThrust(
        BodyRateThrust{Vec3{}, context.config.low_thrust});
  }
};

// cmd2 周期动作：请求 OFFBOARD/arm，然后发布固定高度的位置保持点。
struct TickPositionHold {
  void operator()(Context& context) const {
    context.ensureOffboardArm();
    context.setpoint.publishPosition(
        PositionSetpoint{Vec3{0.0, 0.0, context.config.position_hold_z}, 0.0});
  }
};

// cmd3 周期动作：请求 OFFBOARD/arm，调用 NMPC 悬停求解并发布有效结果。
struct TickNmpcHover {
  void operator()(Context& context) const {
    context.ensureOffboardArm();
    BodyRateThrust command;
    if (context.nmpc.solveHover(context.telemetry, command) &&
        Context::finite(command)) {
      context.setpoint.publishBodyRateThrust(command);
    }
  }
};

// cmd4 周期动作：精准降落优先生成 NMPC horizon；完成后复用旧版近地 latch
// 和“已离开 OFFBOARD 且仍 armed”时请求上锁的收尾语义。
struct TickLanding {
  void operator()(Context& context) const {
    if (!context.landing_reached) {
      context.ensureOffboardArm();
      std::vector<ReferencePoint> horizon;
      BodyRateThrust command;
      if (context.landing.prepareLanding(context.clock.now(),
                                         context.telemetry, horizon) &&
          !horizon.empty() &&
          context.nmpc.solveTrack(context.telemetry, horizon, command) &&
          Context::finite(command)) {
        context.setpoint.publishBodyRateThrust(command);
        NmpcMonitor monitor;
        monitor.references = horizon;
        monitor.feedback = context.telemetry;
        monitor.target = command;
        context.setpoint.publishNmpcMonitor(monitor);
      }
      if (context.landing.isComplete() ||
          std::abs(context.telemetry.position.z -
                   context.config.landing_reference_z) <
              context.config.landing_tolerance_z) {
        context.landing_reached = true;
      }
      return;
    }

    if (context.telemetry.mode != "OFFBOARD" && context.telemetry.armed) {
      context.autopilot.requestDisarm();
    }
  }
};

// 进入 cmd5 时重置参考轨迹提供器，复现旧版离开再进入 Track 的语义。
struct ResetNmpcTrack {
  void operator()(Context& context) const { context.reference.reset(); }
};

// 进入 cmd6 时重置任务轨迹适配器到 Super 模式。
struct ResetSuperTrack {
  void operator()(Context& context) const {
    context.mission.reset(MissionTrackMode::Super);
  }
};

// 进入 cmd7 时重置任务轨迹适配器到 Mission 模式。
struct ResetMissionTrack {
  void operator()(Context& context) const {
    context.mission.reset(MissionTrackMode::Mission);
  }
};

// 进入 cmd8 时重置任务轨迹适配器到 Ego 模式。
struct ResetEgoTrack {
  void operator()(Context& context) const {
    context.mission.reset(MissionTrackMode::Ego);
  }
};

struct ResetLanding {
  void operator()(Context& context) const {
    context.landing_reached = false;
    context.landing.reset();
  }
};

// cmd5 周期动作：请求 OFFBOARD/arm，获取参考 horizon，调用 NMPC 跟踪并发布有效结果。
// 成功求解时同时发布 NMPC 监视量，保持旧模块 /nmpc_state 调试接口可用。
struct TickNmpcTrack {
  void operator()(Context& context) const {
    context.ensureOffboardArm();
    std::vector<ReferencePoint> horizon;
    BodyRateThrust command;
    if (context.reference.horizon(context.clock.now(), horizon) &&
        !horizon.empty() &&
        context.nmpc.solveTrack(context.telemetry, horizon, command) &&
        Context::finite(command)) {
      context.setpoint.publishBodyRateThrust(command);
      NmpcMonitor monitor;
      monitor.references = horizon;
      monitor.feedback = context.telemetry;
      monitor.target = command;
      context.setpoint.publishNmpcMonitor(monitor);
    }
  }
};

// cmd6 SuperTrack：使用 simple NMPC 轨迹跟踪，并保留旧版 cmd==6 的
// OFFBOARD/arm 请求行为。
struct TickSuperTrack {
  void operator()(Context& context) const {
    context.ensureOffboardArm();
    std::vector<ReferencePoint> horizon;
    BodyRateThrust command;
    const bool prepared =
        context.mission.prepareSuper(context.clock.now(), context.telemetry,
                                     horizon);
    if (prepared && !horizon.empty()) {
      context.setpoint.publishFeedbackPosition(context.telemetry.position,
                                               context.telemetry.attitude);
      context.setpoint.publishReferencePosition(horizon.front().position,
                                                horizon.front().attitude);
    }
    if (prepared && !horizon.empty() &&
        context.nmpc.solveTrack(context.telemetry, horizon, command) &&
        Context::finite(command)) {
      context.setpoint.publishBodyRateThrust(command);
      NmpcMonitor monitor;
      monitor.references = horizon;
      monitor.feedback = context.telemetry;
      monitor.target = command;
      context.setpoint.publishNmpcMonitor(monitor);
    }
  }
};

// cmd7/cmd8 共用的 legacy mission 执行动作。
// 这里刻意不调用 ensureOffboardArm()，因为旧版 cmd==7/8 分支不主动请求
// OFFBOARD/arm。
struct TickLegacyMissionTrack {
  bool solve(Context& context, MissionTrackMode mode) const {
    std::vector<ReferencePoint> horizon;
    const double now = context.clock.now();
    const bool prepared =
        mode == MissionTrackMode::Mission
            ? context.mission.prepareMission(now, context.telemetry, horizon)
            : context.mission.prepareEgo(now, context.telemetry, horizon);
    BodyRateThrust command;
    LegacyNmpcRequest request;
    request.telemetry = context.telemetry;
    request.horizon = horizon;
    if (prepared && !request.horizon.empty()) {
      context.setpoint.publishFeedbackPosition(context.telemetry.position,
                                               context.telemetry.attitude);
      context.setpoint.publishReferencePosition(request.horizon.front().position,
                                                request.horizon.front().attitude);
    }
    if (prepared && context.mission.wantsPrecisionLanding()) {
      std::vector<ReferencePoint> landing_horizon;
      BodyRateThrust landing_command;
      if (context.landing.prepareLanding(context.clock.now(),
                                         context.telemetry, landing_horizon) &&
          !landing_horizon.empty() &&
          context.nmpc.solveTrack(context.telemetry, landing_horizon,
                                  landing_command) &&
          Context::finite(landing_command)) {
        context.setpoint.publishBodyRateThrust(landing_command);
        NmpcMonitor monitor;
        monitor.references = landing_horizon;
        monitor.feedback = context.telemetry;
        monitor.target = landing_command;
        context.setpoint.publishNmpcMonitor(monitor);
        if (context.landing.isComplete()) {
          context.landing_reached = true;
        }
        return true;
      }
      return false;
    }
    if (prepared && !request.horizon.empty() &&
        context.nmpc.solveLegacy(request, command) &&
        Context::finite(command)) {
      context.setpoint.publishBodyRateThrust(command);
      return true;
    }
    return false;
  }
};

struct TickMissionTrack : TickLegacyMissionTrack {
  // cmd7 周期动作：按 Mission 模式执行 legacy mission 跟踪。
  void operator()(Context& context) const {
    solve(context, MissionTrackMode::Mission);
  }
};

struct TickEgoTrack : TickLegacyMissionTrack {
  // cmd8 周期动作：按 Ego 模式执行 legacy ego planner 跟踪。
  void operator()(Context& context) const { solve(context, MissionTrackMode::Ego); }
};

// cmd9 周期动作：发布单位姿态和略低于悬停油门的应急推力。
struct TickEmergency {
  void operator()(Context& context) const {
    context.setpoint.publishAttitude(
        AttitudeSetpoint{Quaternion{}, context.config.hover_thrust - 0.03});
  }
};

// 为每个源状态生成完整 Select 转移，避免手写 11x11 份重复表项。
// 带 "/ ResetXxx{}" 的转移会在进入目标状态时执行对应 reset 动作。
#define FSM_CTRL_SML_SELECT_TRANSITIONS(source_state)                         \
  boost::sml::state<source_state> + boost::sml::event<SelectIdle> =           \
      boost::sml::state<Idle>,                                                 \
  boost::sml::state<source_state> + boost::sml::event<SelectLowThrust> =       \
      boost::sml::state<LowThrust>,                                            \
  boost::sml::state<source_state> + boost::sml::event<SelectPositionHold> =    \
      boost::sml::state<PositionHold>,                                         \
  boost::sml::state<source_state> + boost::sml::event<SelectNmpcHover> =       \
      boost::sml::state<NmpcHover>,                                            \
  boost::sml::state<source_state> + boost::sml::event<SelectLanding> /         \
      ResetLanding{} =                                                         \
      boost::sml::state<Landing>,                                              \
  boost::sml::state<source_state> + boost::sml::event<SelectNmpcTrack> /       \
      ResetNmpcTrack{} =                                                       \
      boost::sml::state<NmpcTrack>,                                            \
  boost::sml::state<source_state> + boost::sml::event<SelectSuperTrack> /      \
      ResetSuperTrack{} =                                                       \
      boost::sml::state<SuperTrack>,                                           \
  boost::sml::state<source_state> + boost::sml::event<SelectMissionTrack> /    \
      ResetMissionTrack{} =                                                     \
      boost::sml::state<MissionTrack>,                                         \
  boost::sml::state<source_state> + boost::sml::event<SelectEgoTrack> /        \
      ResetEgoTrack{} =                                                         \
      boost::sml::state<EgoTrack>,                                             \
  boost::sml::state<source_state> + boost::sml::event<SelectEmergency> =       \
      boost::sml::state<Emergency>,                                            \
      boost::sml::state<source_state> + boost::sml::event<SelectSafeNoop> =        \
      boost::sml::state<SafeNoop>

// SML 转移表：
//   *state<Idle> 表示初始状态为 Idle。
//   Tick 行执行当前状态动作并保持原状态。
//   Select 行允许从任意状态切到目标状态。
struct Machine {
  auto operator()() const {
    using namespace boost::sml;
    return make_transition_table(
        *state<Idle> + event<Tick> / Noop{},
        state<LowThrust> + event<Tick> / TickLowThrust{},
        state<PositionHold> + event<Tick> / TickPositionHold{},
        state<NmpcHover> + event<Tick> / TickNmpcHover{},
        state<Landing> + event<Tick> / TickLanding{},
        state<NmpcTrack> + event<Tick> / TickNmpcTrack{},
        state<SuperTrack> + event<Tick> / TickSuperTrack{},
        state<MissionTrack> + event<Tick> / TickMissionTrack{},
        state<EgoTrack> + event<Tick> / TickEgoTrack{},
        state<Emergency> + event<Tick> / TickEmergency{},
        state<SafeNoop> + event<Tick> / Noop{},
        FSM_CTRL_SML_SELECT_TRANSITIONS(Idle),
        FSM_CTRL_SML_SELECT_TRANSITIONS(LowThrust),
        FSM_CTRL_SML_SELECT_TRANSITIONS(PositionHold),
        FSM_CTRL_SML_SELECT_TRANSITIONS(NmpcHover),
        FSM_CTRL_SML_SELECT_TRANSITIONS(Landing),
        FSM_CTRL_SML_SELECT_TRANSITIONS(NmpcTrack),
        FSM_CTRL_SML_SELECT_TRANSITIONS(SuperTrack),
        FSM_CTRL_SML_SELECT_TRANSITIONS(MissionTrack),
        FSM_CTRL_SML_SELECT_TRANSITIONS(EgoTrack),
        FSM_CTRL_SML_SELECT_TRANSITIONS(Emergency),
        FSM_CTRL_SML_SELECT_TRANSITIONS(SafeNoop));
  }
};

#undef FSM_CTRL_SML_SELECT_TRANSITIONS

using StateMachine = boost::sml::sm<Machine>;

}  // namespace single_sml
}  // namespace fsm_ctrl

#endif  // FSM_CTRL_SINGLE_OFFBOARD_SML_HPP_
