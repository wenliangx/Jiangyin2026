#include <fsm_ctrl/single_offboard_sml.hpp>
#include <fsm_ctrl/single_offboard_sml_dispatch.hpp>
#include <fsm_ctrl/NMPC_Controller.hpp>
#include <fsm_ctrl/NMPC_test.hpp>
#include <fsm_ctrl/ctrl_math.hpp>
#include <fsm_ctrl/nmpc_state.h>
#include <fsm_ctrl/traj_gen.hpp>

// 旧版 NMPC 头文件会导出通用重力宏；这里清掉，避免污染 Boost.SML 和适配层头文件。
#ifdef G
#undef G
#endif
#ifdef GRAVITY
#undef GRAVITY
#endif

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <super_msgs/Flag.h>
#include <traj_utils/Flag.h>
#include <traj_utils/FlagState.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace smlfsm = fsm_ctrl::single_sml;

namespace {

// 主循环频率，与原 single_offboard_fsm 的 50Hz 控制节奏保持一致。
constexpr double kRateHz = 50.0;
// 单个控制周期时长，传给 NMPC 控制器。
constexpr double kInterval = 1.0 / kRateHz;
// simple NMPC 使用的参考 horizon 点数。
constexpr std::size_t kHorizonPoints = 9;
// planner topic 中按旧逻辑读取的参考点数。
constexpr std::size_t kPlannerPoints = 6;
// legacy younger_ctrl NMPC 使用的参考 horizon 点数。
constexpr std::size_t kLegacyHorizonPoints = 6;

// 旧任务航点的纯 C++ 表示，用于生成 cmd6/7/8 的 waypoint 消息。
struct MissionPoint {
  int id{0};              // 航点编号。
  int mode{0};            // 下游 planner 使用的任务模式。
  int is_map{0};          // 是否使用 map 坐标。
  smlfsm::Vec3 position;  // 航点位置。
  double yaw{0.0};        // 航点期望 yaw。
  int perc_mode{0};       // 感知模式字段，保留旧结构含义。
};

// 带时间戳的位姿缓存，用于把 apriltag 相机局部测量转换到全局坐标。
struct TimedPose {
  double stamp{0.0};      // 位姿时间戳。
  smlfsm::Vec3 position;  // 当时的全局位置。
  double roll{0.0};       // 当时的 roll。
  double pitch{0.0};      // 当时的 pitch。
  double yaw{0.0};        // 当时的 yaw。
};

// 构造旧版 ego/mission 任务航点表。
std::vector<MissionPoint> createLegacyEgoTrajectory() {
  return std::vector<MissionPoint>{
      {0, 2, 1, {1.4, 1.5, 1.0}, 0.0, 3},
      {1, 2, 1, {1.6, 1.2, 1.65}, 0.0, 0},
      {2, 2, 1, {2.6, 0.7, 1.7}, 0.0, 0},
      {3, 2, 1, {3.6, 0.2, 1.65}, 0.0, 0},
      {4, 2, 1, {3.6, 0.2, 0.5}, 0.0, 0},
      {5, 2, 1, {2.6, 0.7, 0.45}, 0.0, 0},
      {6, 2, 1, {1.6, 1.2, 0.5}, 0.0, 0},
      {7, 2, 1, {0.0, 0.0, 0.5}, 0.0, 0},
  };
}

// 对单个标量做对称限幅，主要用于约束横向修正量。
double clampSingle(double value, double limit) {
  if (value > limit) {
    return limit;
  }
  if (value < -limit) {
    return -limit;
  }
  return value;
}

// Clock 的 ROS 实现，给纯 C++ Context 提供当前时间。
class RosClock final : public smlfsm::Clock {
 public:
  double now() const override { return ros::Time::now().toSec(); }
};

// AutopilotPort 的 ROS/MAVROS 实现，负责 OFFBOARD、arm 和 disarm 服务调用。
class RosAutopilotPort final : public smlfsm::AutopilotPort {
 public:
  explicit RosAutopilotPort(ros::NodeHandle& node)
      : arming_client_(
            node.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming")),
        mode_client_(node.serviceClient<mavros_msgs::SetMode>(
            "mavros/set_mode")) {}

  bool requestOffboard() override {
    mavros_msgs::SetMode request;
    request.request.custom_mode = "OFFBOARD";
    return mode_client_.call(request) && request.response.mode_sent;
  }

  bool requestArm() override { return requestArmed(true); }
  bool requestDisarm() override { return requestArmed(false); }

 private:
  // 根据 armed 参数复用 MAVROS arming 服务。
  bool requestArmed(bool armed) {
    mavros_msgs::CommandBool request;
    request.request.value = armed;
    return arming_client_.call(request) && request.response.success;
  }

  ros::ServiceClient arming_client_;  // /mavros/cmd/arming 服务客户端。
  ros::ServiceClient mode_client_;    // /mavros/set_mode 服务客户端。
};

// SetpointPort 的 ROS/MAVROS 实现，集中管理状态机所有控制输出 topic。
class RosSetpointPort final : public smlfsm::SetpointPort {
 public:
  explicit RosSetpointPort(ros::NodeHandle& node)
      : position_pub_(node.advertise<geometry_msgs::PoseStamped>(
            "/mavros/setpoint_position/local", 10)),
        attitude_pub_(node.advertise<mavros_msgs::AttitudeTarget>(
            "/mavros/setpoint_raw/attitude", 10)),
        nmpc_posref_pub_(node.advertise<geometry_msgs::PoseStamped>(
            "/nmpc_posref", 10)),
        nmpc_posfdb_pub_(node.advertise<geometry_msgs::PoseStamped>(
            "/nmpc_posfdb", 10)),
        nmpc_state_pub_(
            node.advertise<fsm_ctrl::nmpc_state>("/nmpc_state", 10)) {}

  void publishPosition(const smlfsm::PositionSetpoint& setpoint) override {
    geometry_msgs::PoseStamped message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "map";
    message.pose.position.x = setpoint.position.x;
    message.pose.position.y = setpoint.position.y;
    message.pose.position.z = setpoint.position.z;
    message.pose.orientation.w = std::cos(setpoint.yaw * 0.5);
    message.pose.orientation.z = std::sin(setpoint.yaw * 0.5);
    position_pub_.publish(message);
  }

  void publishBodyRateThrust(
      const smlfsm::BodyRateThrust& setpoint) override {
    mavros_msgs::AttitudeTarget message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "FCU";
    message.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
    message.body_rate.x = setpoint.body_rate.x;
    message.body_rate.y = setpoint.body_rate.y;
    message.body_rate.z = setpoint.body_rate.z;
    message.thrust = setpoint.thrust;
    attitude_pub_.publish(message);
  }

  void publishAttitude(const smlfsm::AttitudeSetpoint& setpoint) override {
    mavros_msgs::AttitudeTarget message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "FCU";
    message.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                        mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                        mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
    message.orientation.w = setpoint.attitude.w;
    message.orientation.x = setpoint.attitude.x;
    message.orientation.y = setpoint.attitude.y;
    message.orientation.z = setpoint.attitude.z;
    message.thrust = setpoint.thrust;
    attitude_pub_.publish(message);
  }

  void publishReferencePosition(const smlfsm::Vec3& position) override {
    nmpc_posref_pub_.publish(toPose(position));
  }

  void publishFeedbackPosition(const smlfsm::Vec3& position) override {
    nmpc_posfdb_pub_.publish(toPose(position));
  }

  void publishNmpcMonitor(const smlfsm::NmpcMonitor& monitor) override {
    fsm_ctrl::nmpc_state message;
    const std::size_t count = std::min<std::size_t>(monitor.references.size(), 9);
    for (std::size_t index = 0; index < count; ++index) {
      message.pos_ref[index].x = monitor.references[index].position.x;
      message.pos_ref[index].y = monitor.references[index].position.y;
      message.pos_ref[index].z = monitor.references[index].position.z;
      message.vel_ref[index].x = monitor.references[index].velocity.x;
      message.vel_ref[index].y = monitor.references[index].velocity.y;
      message.vel_ref[index].z = monitor.references[index].velocity.z;
    }
    message.pos_fdb.x = monitor.feedback.position.x;
    message.pos_fdb.y = monitor.feedback.position.y;
    message.pos_fdb.z = monitor.feedback.position.z;
    message.vel_fdb.x = monitor.feedback.velocity.x;
    message.vel_fdb.y = monitor.feedback.velocity.y;
    message.vel_fdb.z = monitor.feedback.velocity.z;
    message.attitude_fdb.w = monitor.feedback.attitude.w;
    message.attitude_fdb.x = monitor.feedback.attitude.x;
    message.attitude_fdb.y = monitor.feedback.attitude.y;
    message.attitude_fdb.z = monitor.feedback.attitude.z;
    message.target.header.stamp = ros::Time::now();
    message.target.header.frame_id = "FCU";
    message.target.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                               mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE;
    message.target.body_rate.x = monitor.target.body_rate.x;
    message.target.body_rate.y = monitor.target.body_rate.y;
    message.target.body_rate.z = monitor.target.body_rate.z;
    message.target.thrust = monitor.target.thrust;
    nmpc_state_pub_.publish(message);
  }

 private:
  // 将纯 C++ 位置向量转换为 ROS PoseStamped，供监视/参考 topic 复用。
  static geometry_msgs::PoseStamped toPose(const smlfsm::Vec3& position) {
    geometry_msgs::PoseStamped message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "world";
    message.pose.position.x = position.x;
    message.pose.position.y = position.y;
    message.pose.position.z = position.z;
    message.pose.orientation.w = 1.0;
    return message;
  }

  ros::Publisher position_pub_;     // 位置 setpoint 输出。
  ros::Publisher attitude_pub_;     // 姿态或 body-rate setpoint 输出。
  ros::Publisher nmpc_posref_pub_;  // NMPC 参考位置监视输出。
  ros::Publisher nmpc_posfdb_pub_;  // NMPC 反馈位置监视输出。
  ros::Publisher nmpc_state_pub_;   // NMPC 完整监视消息输出。
};

// NmpcPort 的 ROS adapter 实现，封装 simple NMPC 和旧 younger_ctrl NMPC。
class RosNmpcPort final : public smlfsm::NmpcPort {
 public:
  explicit RosNmpcPort(ros::NodeHandle& private_node) {
    double q_pos_x = 1.0, q_pos_y = 1.0, q_pos_z = 1.0;
    double q_vel_x = 1.0, q_vel_y = 1.0, q_vel_z = 1.0;
    double q_quat_x = 1.0, q_quat_y = 1.0, q_quat_z = 1.0;
    double r_w_x = 1.0, r_w_y = 1.0, r_w_z = 1.0;
    double r_thrust = 1.0, hover_thrust = 0.196;
    private_node.param("nmpc_Qposx", q_pos_x, q_pos_x);
    private_node.param("nmpc_Qposy", q_pos_y, q_pos_y);
    private_node.param("nmpc_Qposz", q_pos_z, q_pos_z);
    private_node.param("nmpc_Qvelx", q_vel_x, q_vel_x);
    private_node.param("nmpc_Qvely", q_vel_y, q_vel_y);
    private_node.param("nmpc_Qvelz", q_vel_z, q_vel_z);
    private_node.param("nmpc_Qquatx", q_quat_x, q_quat_x);
    private_node.param("nmpc_Qquaty", q_quat_y, q_quat_y);
    private_node.param("nmpc_Qquatz", q_quat_z, q_quat_z);
    private_node.param("nmpc_Rwx", r_w_x, r_w_x);
    private_node.param("nmpc_Rwy", r_w_y, r_w_y);
    private_node.param("nmpc_Rwz", r_w_z, r_w_z);
    private_node.param("nmpc_RtotalF", r_thrust, r_thrust);
    private_node.param("nmpc_hover_thrust", hover_thrust, hover_thrust);

    Eigen::Matrix<float, 3, 1> q_position, q_velocity, q_attitude, r_angular;
    q_position << q_pos_x, q_pos_y, q_pos_z;
    q_velocity << q_vel_x, q_vel_y, q_vel_z;
    q_attitude << q_quat_x, q_quat_y, q_quat_z;
    r_angular << r_w_x, r_w_y, r_w_z;
    controller_.reset(new NMPC_Ctrller_simple(
        kInterval, std::array<double, 2>{{0.0, 15.0}},
        std::array<double, 2>{{-3.14, 3.14}}, 8, 0.05, 10, 4,
        q_position, q_velocity, q_attitude, r_angular, r_thrust,
        hover_thrust));

    double mq_px = 0.0, mq_py = 0.0, mq_pz = 0.0, mq_vx = 0.0, mq_vy = 0.0;
    double mq_vz = 0.0, mq_q1 = 0.0, mq_q2 = 0.0, mq_q3 = 0.0, mq_q4 = 0.0;
    double mr_a_bz = 0.0, mr_roll = 0.0, mr_pitch = 0.0, mr_yaw = 0.0;
    private_node.param("nmpc_mQ_px", mq_px, mq_px);
    private_node.param("nmpc_mQ_py", mq_py, mq_py);
    private_node.param("nmpc_mQ_pz", mq_pz, mq_pz);
    private_node.param("nmpc_mQ_vx", mq_vx, mq_vx);
    private_node.param("nmpc_mQ_vy", mq_vy, mq_vy);
    private_node.param("nmpc_mQ_vz", mq_vz, mq_vz);
    private_node.param("nmpc_mQ_q1", mq_q1, mq_q1);
    private_node.param("nmpc_mQ_q2", mq_q2, mq_q2);
    private_node.param("nmpc_mQ_q3", mq_q3, mq_q3);
    private_node.param("nmpc_mQ_q4", mq_q4, mq_q4);
    private_node.param("nmpc_mR_a_BZ", mr_a_bz, mr_a_bz);
    private_node.param("nmpc_mR_roll", mr_roll, mr_roll);
    private_node.param("nmpc_mR_pitch", mr_pitch, mr_pitch);
    private_node.param("nmpc_mR_yaw", mr_yaw, mr_yaw);
    Eigen::Matrix<float, 10, 1> legacy_q;
    Eigen::Matrix<float, 4, 1> legacy_r;
    legacy_q << mq_px, mq_py, mq_pz, mq_vx, mq_vy, mq_vz, mq_q1, mq_q2,
        mq_q3, mq_q4;
    legacy_r << mr_a_bz, mr_roll, mr_pitch, mr_yaw;
    legacy_controller_.reset(
        new NMPC_Ctrller(5, 0.1, kInterval, hover_thrust, legacy_q, legacy_r));
  }

  bool solveHover(const smlfsm::TelemetrySnapshot& telemetry,
                  smlfsm::BodyRateThrust& command) override {
    std::vector<smlfsm::ReferencePoint> horizon(kHorizonPoints);
    for (auto& point : horizon) {
      point.position.z = 0.5;
    }
    return solve(telemetry, horizon, command);
  }

  bool solveTrack(const smlfsm::TelemetrySnapshot& telemetry,
                  const std::vector<smlfsm::ReferencePoint>& horizon,
                  smlfsm::BodyRateThrust& command) override {
    return solve(telemetry, horizon, command);
  }

  bool solveLegacy(const smlfsm::LegacyNmpcRequest& request,
                   smlfsm::BodyRateThrust& command) override {
    if (!legacy_controller_ ||
        request.horizon.size() < kLegacyHorizonPoints) {
      return false;
    }
    Eigen::Matrix<float, 10, 1> current;
    current << request.telemetry.position.x, request.telemetry.position.y,
        request.telemetry.position.z, request.telemetry.velocity.x,
        request.telemetry.velocity.y, request.telemetry.velocity.z,
        request.telemetry.attitude.w, request.telemetry.attitude.x,
        request.telemetry.attitude.y, request.telemetry.attitude.z;

    Eigen::Matrix<float, 60, 1> desired;
    for (std::size_t index = 0; index < kLegacyHorizonPoints; ++index) {
      const auto& point = request.horizon[index];
      const std::size_t offset = index * 10;
      desired(offset + 0) = point.position.x;
      desired(offset + 1) = point.position.y;
      desired(offset + 2) = point.position.z;
      desired(offset + 3) = point.velocity.x;
      desired(offset + 4) = point.velocity.y;
      desired(offset + 5) = point.velocity.z;
      desired(offset + 6) = point.attitude.w;
      desired(offset + 7) = point.attitude.x;
      desired(offset + 8) = point.attitude.y;
      desired(offset + 9) = point.attitude.z;
    }

    Eigen::Matrix<float, 4, 1> desired_controls;
    desired_controls << request.desired_controls[0], request.desired_controls[1],
        request.desired_controls[2], request.desired_controls[3];

    try {
      legacy_controller_->optimal_solution(current, desired, desired_controls);
      if (!legacy_controller_->Acc2Trust()) {
        return false;
      }
    } catch (const std::exception& error) {
      ROS_ERROR_THROTTLE(1.0, "Legacy NMPC solve failed: %s", error.what());
      return false;
    }
    const Eigen::Matrix<float, 4, 1> output =
        legacy_controller_->get_control_command();
    command.thrust = output(0);
    command.body_rate = {output(1), output(2), output(3)};
    return smlfsm::Context::finite(command);
  }

 private:
  // simple NMPC 共用求解路径：把遥测和参考 horizon 展平成旧控制器输入。
  bool solve(const smlfsm::TelemetrySnapshot& telemetry,
             const std::vector<smlfsm::ReferencePoint>& horizon,
             smlfsm::BodyRateThrust& command) {
    if (!controller_ || horizon.empty()) {
      return false;
    }
    const std::vector<double> current{
        telemetry.position.x, telemetry.position.y, telemetry.position.z,
        telemetry.velocity.x, telemetry.velocity.y, telemetry.velocity.z,
        telemetry.attitude.w, telemetry.attitude.x, telemetry.attitude.y,
        telemetry.attitude.z};
    std::vector<double> desired;
    desired.reserve(kHorizonPoints * 10 + (kHorizonPoints - 1) * 4);
    for (std::size_t index = 0; index < kHorizonPoints; ++index) {
      const auto& point = horizon[std::min(index, horizon.size() - 1)];
      desired.insert(desired.end(),
                     {point.position.x, point.position.y, point.position.z,
                      point.velocity.x, point.velocity.y, point.velocity.z,
                      point.attitude.w, point.attitude.x, point.attitude.y,
                      point.attitude.z});
    }
    for (std::size_t index = 0; index + 1 < kHorizonPoints; ++index) {
      desired.insert(desired.end(), {0.0, 0.0, 0.0, 9.8015});
    }
    try {
      controller_->optimal_solution(current, desired);
    } catch (const std::exception& error) {
      ROS_ERROR_THROTTLE(1.0, "NMPC solve failed: %s", error.what());
      return false;
    }
    const Eigen::Vector3d angular = controller_->getwCommand();
    command.body_rate = {angular.x(), angular.y(), angular.z()};
    command.thrust = controller_->getAcc_zCommand();
    return smlfsm::Context::finite(command);
  }

  std::unique_ptr<NMPC_Ctrller_simple> controller_;  // cmd3/5/6 使用的 simple NMPC。
  std::unique_ptr<NMPC_Ctrller> legacy_controller_;  // cmd7/8 使用的旧版 NMPC。
};

// cmd5 参考轨迹提供器：按 planner、内部轨迹、固定悬停的优先级生成 horizon。
class RosReferenceProvider final : public smlfsm::ReferenceProvider {
 public:
  explicit RosReferenceProvider(ros::NodeHandle& private_node)
      : private_node_(private_node) {
    private_node_.param("use_traj_gen", use_internal_, false);
    private_node_.param("traj_type", trajectory_type_, std::string("circle"));
    private_node_.param("traj_cx", center_x_, 0.0);
    private_node_.param("traj_cy", center_y_, 0.0);
    private_node_.param("traj_alt", altitude_, 1.0);
    private_node_.param("traj_param1", parameter_one_, 2.0);
    private_node_.param("traj_param2", parameter_two_, 0.5);
    private_node_.param("traj_hold_time", hold_seconds_, 3.0);
  }

  // 记录当前命令，用于 fallback 高度等兼容旧行为的分支。
  void selectCommand(int command) override {
    switch (command) {
      case 6: mode_ = Mode::kSuperTrack; break;
      case 7: mode_ = Mode::kMissionTrack; break;
      case 8: mode_ = Mode::kEgoTrack; break;
      default: mode_ = Mode::kNmpcTrack; break;
    }
  }

  // 接收 planner 轨迹；保持旧行为：一旦收到有效 planner，就一直复用最后一帧。
  void updatePlanner(const traj_utils::Flag& message) {
    planner_.clear();
    for (std::size_t index = 0; index < kPlannerPoints; ++index) {
      smlfsm::ReferencePoint point;
      point.position = {message.cmd[index].position.x,
                        message.cmd[index].position.y,
                        message.cmd[index].position.z};
      point.velocity = {message.cmd[index].velocity.x,
                        message.cmd[index].velocity.y,
                        message.cmd[index].velocity.z};
      planner_.push_back(point);
    }
    planner_valid_ = true;
  }

  // 重新进入 cmd5 时重置内部轨迹起点和计时。
  void reset() override {
    start_time_ = ros::Time::now().toSec();
    internal_.reset();
    if (use_internal_) {
      createInternalTrajectory();
    }
  }

  // 生成给 cmd5 NMPC 的参考 horizon。
  bool horizon(double now,
               std::vector<smlfsm::ReferencePoint>& points) override {
    points.clear();
    if (planner_valid_) {
      for (std::size_t index = 0; index < kHorizonPoints; ++index) {
        points.push_back(planner_[std::min(index, planner_.size() - 1)]);
      }
      return true;
    }
    if (use_internal_ && internal_) {
      for (std::size_t index = 0; index < kHorizonPoints; ++index) {
        const double elapsed = now - start_time_ + index * 0.05;
        traj_gen::TrajPoint sample;
        if (elapsed < hold_seconds_) {
          sample.pos = Eigen::Vector3d(0.0, 0.0, altitude_);
        } else {
          sample = internal_->sample(elapsed - hold_seconds_);
        }
        smlfsm::ReferencePoint point;
        point.position = {sample.pos.x(), sample.pos.y(), sample.pos.z()};
        point.velocity = {sample.vel.x(), sample.vel.y(), sample.vel.z()};
        point.attitude = {std::cos(sample.yaw * 0.5), 0.0, 0.0,
                          std::sin(sample.yaw * 0.5)};
        points.push_back(point);
      }
      return true;
    }
    points.resize(kHorizonPoints);
    for (auto& point : points) {
      point.position.z = fallbackAltitude();
    }
    return true;
  }

 private:
  // 参考提供器内部模式，用于区分 cmd5 和任务态 fallback。
  enum class Mode { kNmpcTrack, kSuperTrack, kMissionTrack, kEgoTrack };

  // 没有 planner/内部轨迹时使用的固定高度。
  double fallbackAltitude() const {
    return mode_ == Mode::kNmpcTrack ? 0.5 : 1.0;
  }

  // 根据参数创建内部轨迹；参数无效时回退到 circle。
  void createInternalTrajectory() {
    try {
      if (trajectory_type_ == "waypoint") {
        XmlRpc::XmlRpcValue waypoints;
        if (private_node_.getParam("traj_waypoints", waypoints) &&
            waypoints.getType() == XmlRpc::XmlRpcValue::TypeArray &&
            waypoints.size() >= 2) {
          auto trajectory = std::make_shared<traj_gen::WaypointTrajectory>(
              altitude_, parameter_two_, true);
          for (int index = 0; index < waypoints.size(); ++index) {
            const double x = static_cast<double>(waypoints[index][0]);
            const double y = static_cast<double>(waypoints[index][1]);
            const double yaw = waypoints[index].size() > 2
                                   ? static_cast<double>(waypoints[index][2])
                                   : 0.0;
            trajectory->addWaypoint(x, y, yaw);
          }
          internal_ = trajectory;
          return;
        }
        ROS_WARN("Invalid traj_waypoints; falling back to circle");
      }
      const std::string type =
          trajectory_type_ == "waypoint" ? "circle" : trajectory_type_;
      internal_ = traj_gen::createTrajectory(
          type, center_x_, center_y_, altitude_, parameter_one_,
          parameter_two_);
    } catch (const std::exception& error) {
      ROS_ERROR("Cannot create internal trajectory: %s", error.what());
      internal_.reset();
    }
  }

  ros::NodeHandle& private_node_;  // 私有参数命名空间。
  bool use_internal_{false};       // 是否启用内部轨迹生成器。
  bool planner_valid_{false};      // 是否已经收到 planner 参考。
  double center_x_{0.0}, center_y_{0.0}, altitude_{1.0};  // 内部轨迹几何参数。
  double parameter_one_{2.0}, parameter_two_{0.5}, hold_seconds_{3.0};  // 内部轨迹形状/起飞等待参数。
  double start_time_{0.0};         // 内部轨迹的起始时间。
  std::string trajectory_type_;    // 内部轨迹类型。
  std::vector<smlfsm::ReferencePoint> planner_;  // 最近一帧 planner 轨迹。
  std::shared_ptr<traj_gen::Trajectory> internal_;  // 内部轨迹对象。
  Mode mode_{Mode::kNmpcTrack};    // 当前参考模式。
};

// cmd6/7/8 任务轨迹适配器：负责 waypoint 发布、planner 缓存、感知滤波和 legacy horizon。
class RosMissionPort final : public smlfsm::MissionPort {
 public:
  explicit RosMissionPort(ros::NodeHandle& node)
      : super_waypoint_pub_(node.advertise<super_msgs::Flag>(
            "/super/flag_waypoint", 10)),
        ego_waypoint_pub_(node.advertise<traj_utils::Flag>(
            "/ego_planner/flag_msg", 10)) {
    reset(smlfsm::MissionTrackMode::Super);
  }

  void selectCommand(int command) override { selected_command_ = command; }

  void reset(smlfsm::MissionTrackMode mode) override {
    (void)mode;
    // 旧版 cmd6/7/8 的任务进度本来由模块级静态变量保存。
    // 为了行为兼容，状态切换时不重置 need_generate_traj_、ego_traj_count_
    // 或 cmd7 阶段计数；感知回调会在需要时标记重新生成轨迹。
  }

  // cmd6：发布任务航点，并优先使用 super planner 返回的 horizon。
  bool prepareSuper(double /*now*/, const smlfsm::TelemetrySnapshot& /*telemetry*/,
                    std::vector<smlfsm::ReferencePoint>& horizon) override {
    publishNextWaypoint(smlfsm::MissionTrackMode::Super);
    if (super_planner_valid_) {
      horizon = super_planner_;
      return true;
    }
    horizon.assign(kHorizonPoints, smlfsm::ReferencePoint{});
    for (auto& point : horizon) {
      point.position.z = 1.0;
    }
    return true;
  }

  // cmd7：发布 mission 航点，按旧版穿环/降落流程生成 legacy horizon。
  bool prepareMission(double /*now*/, const smlfsm::TelemetrySnapshot& telemetry,
                      std::vector<smlfsm::ReferencePoint>& horizon) override {
    publishNextWaypoint(smlfsm::MissionTrackMode::Mission);
    buildMissionHorizon(telemetry);
    horizon = last_mission_horizon_;
    applyMissionYaw(telemetry.attitude, horizon);
    return !horizon.empty();
  }

  // cmd8：发布 ego 航点，并使用 ego planner 返回的 horizon。
  bool prepareEgo(double /*now*/, const smlfsm::TelemetrySnapshot& telemetry,
                  std::vector<smlfsm::ReferencePoint>& horizon) override {
    publishNextWaypoint(smlfsm::MissionTrackMode::Ego);
    if (!ego_planner_valid_) {
      horizon.clear();
      return false;
    }
    horizon = ego_planner_;
    applyMissionYaw(telemetry.attitude, horizon);
    return !horizon.empty();
  }

  // 接收 ego planner 的轨迹输出。
  void updatePlanner(const traj_utils::Flag& message) {
    ego_planner_.clear();
    for (std::size_t index = 0; index < kLegacyHorizonPoints; ++index) {
      smlfsm::ReferencePoint point;
      point.position = {message.cmd[index].position.x,
                        message.cmd[index].position.y,
                        message.cmd[index].position.z};
      point.velocity = {message.cmd[index].velocity.x,
                        message.cmd[index].velocity.y,
                        message.cmd[index].velocity.z};
      ego_planner_.push_back(point);
    }
    ego_planner_valid_ = true;
  }

  // 接收 super planner 的轨迹输出。
  void updateSuperPlanner(const super_msgs::Flag& message) {
    super_planner_.clear();
    for (std::size_t index = 0; index < kHorizonPoints; ++index) {
      smlfsm::ReferencePoint point;
      point.position = {message.cmd[index].position.x,
                        message.cmd[index].position.y,
                        message.cmd[index].position.z};
      point.velocity = {message.cmd[index].velocity.x,
                        message.cmd[index].velocity.y,
                        message.cmd[index].velocity.z};
      super_planner_.push_back(point);
    }
    super_planner_valid_ = true;
  }

  // 接收 ego planner 状态，用于旧任务完成/阶段切换判断。
  void updateEgoState(const traj_utils::FlagState& message) {
    ego_now_id_ = message.now_id;
    ego_touch_goal_ = message.touch_goal;
    if (ego_now_id_ == static_cast<int>(createLegacyEgoTrajectory().size()) - 1 &&
        ego_touch_goal_) {
      task1_ = 6;
    }
  }

  // 记录飞行器历史位姿，供 apriltag 测量按时间戳匹配。
  void recordTelemetryPose(double stamp,
                           const smlfsm::TelemetrySnapshot& telemetry) {
    TimedPose pose;
    pose.stamp = stamp;
    pose.position = telemetry.position;
    quaternionToEuler(telemetry.attitude, pose.roll, pose.pitch, pose.yaw);
    if (pose_history_.size() >= kPoseHistorySize) {
      pose_history_.erase(pose_history_.begin());
    }
    pose_history_.push_back(pose);
  }

  // 更新圆环位置观测；通过中值均值滤波后触发重新生成任务轨迹。
  void updateRingPose(const geometry_msgs::PoseStamped& message) {
    const smlfsm::Vec3 updated{message.pose.position.x,
                               message.pose.position.y,
                               message.pose.position.z};
    if (!near(updated, ring2_pose_, 1.0)) {
      return;
    }
    smlfsm::Vec3 filtered;
    if (!medianAverageFilter(ring2_filter_, updated, kRing2FilterSize,
                             filtered)) {
      return;
    }
    if (!near(filtered, ring2_pose_, 0.10)) {
      need_generate_traj_ = true;
      ring2_pose_ = filtered;
    }
  }

  // 更新 apriltag 观测：把相机局部测量转换到全局坐标并滤波。
  void updateApriltagPose(const geometry_msgs::PoseStamped& message) {
    if (ctl_land_count_ >= 230) {
      return;
    }
    const double stamp = message.header.stamp.isZero()
                             ? ros::Time::now().toSec()
                             : message.header.stamp.toSec();
    const TimedPose base = nearestPose(stamp);
    const smlfsm::Vec3 local_flipped{-message.pose.position.x,
                                     -message.pose.position.y,
                                     message.pose.position.z};
    const smlfsm::Vec3 global = localToGlobal(base, local_flipped);
    if (!near(global, ap_pose_, 2.0)) {
      return;
    }
    smlfsm::Vec3 filtered;
    if (!medianFilter(ap_filter_, global, kApriltagFilterSize, filtered)) {
      return;
    }
    ap_pose_ = filtered;
    is_found_ap_ = true;
  }

 private:
  // 如果任务轨迹需要刷新，就向 super/ego planner 发布下一个 waypoint。
  void publishNextWaypoint(smlfsm::MissionTrackMode mode) {
    if (!need_generate_traj_) {
      return;
    }
    ego_traj_ = createLegacyEgoTrajectory();
    if (ego_traj_count_ < ego_traj_.size()) {
      if (mode == smlfsm::MissionTrackMode::Super) {
        publishSuperWaypoint(ego_traj_[ego_traj_count_]);
      } else {
        publishEgoWaypoint(ego_traj_[ego_traj_count_]);
      }
    }
    ++ego_traj_count_;
    if (ego_traj_count_ >= ego_traj_.size()) {
      need_generate_traj_ = false;
      ego_traj_count_ = mode == smlfsm::MissionTrackMode::Mission
                            ? static_cast<std::size_t>(std::max(0, ego_now_id_))
                            : 0u;
    }
  }

  // 发布 super planner 使用的 waypoint 消息。
  void publishSuperWaypoint(const MissionPoint& point) {
    super_msgs::Flag message;
    fillCommonFlag(point, message);
    super_waypoint_pub_.publish(message);
  }

  // 发布 ego planner 使用的 waypoint 消息。
  void publishEgoWaypoint(const MissionPoint& point) {
    traj_utils::Flag message;
    fillCommonFlag(point, message);
    ego_waypoint_pub_.publish(message);
  }

  // 填充 super_msgs::Flag 和 traj_utils::Flag 共有字段。
  template <typename Message>
  void fillCommonFlag(const MissionPoint& point, Message& message) {
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "world";
    message.id = point.id;
    message.mode = point.mode;
    message.is_map = point.is_map;
    message.position.x = point.position.x;
    message.position.y = point.position.y;
    message.position.z = point.position.z;
  }

  // 将 legacy horizon 设置成同一个固定位置。
  void setConstantHorizon(const smlfsm::Vec3& position) {
    last_mission_horizon_.assign(kLegacyHorizonPoints, smlfsm::ReferencePoint{});
    for (auto& point : last_mission_horizon_) {
      point.position = position;
    }
  }

  // 按旧版 cmd7 阶段机生成穿第二个环、飞向 apriltag、下降的参考 horizon。
  void buildMissionHorizon(const smlfsm::TelemetrySnapshot& telemetry) {
    if (selected_command_ != 7 || task1_ != 6) {
      if (last_mission_horizon_.empty()) {
        setConstantHorizon({0.0, 0.0, 1.0});
      }
      return;
    }
    const smlfsm::Vec3 dynringpost{ring2_pose_.x - 1.0, ring2_pose_.y,
                                   ring2_pose_.z};
    if (!is_arrive_ring2_ && !is_crossed_ring2_) {
      last_mission_horizon_.clear();
      const double delta_y =
          0.4 * clampSingle(ring2_pose_.y - telemetry.position.y, 0.1);
      for (std::size_t index = 0; index < kLegacyHorizonPoints; ++index) {
        smlfsm::ReferencePoint point;
        point.position.x = ring2_pose_.x + 1.0 - 0.01 * ring2_approach_count_;
        point.position.y = ring2_pose_.y + index * delta_y / 3.0;
        point.position.z = 1.65;
        last_mission_horizon_.push_back(point);
      }
      if (++ring2_approach_count_ == 100) {
        is_arrive_ring2_ = true;
      }
      return;
    }
    if (is_arrive_ring2_ && !is_crossed_ring2_) {
      last_mission_horizon_.clear();
      for (std::size_t index = 0; index < kLegacyHorizonPoints; ++index) {
        smlfsm::ReferencePoint point;
        point.position.x = ring2_pose_.x - 0.01 * ring2_cross_count_;
        point.position.y = ring2_pose_.y;
        point.position.z = 1.65;
        last_mission_horizon_.push_back(point);
      }
      if (++ring2_cross_count_ == 100) {
        is_crossed_ring2_ = true;
      }
      return;
    }
    if (!is_arrive_r2_postpoint_) {
      if (count1_ < 150) {
        ++count1_;
        const double delta_x = (appre_pose_.x - dynringpost.x) / 150.0;
        const double delta_y = (appre_pose_.y - dynringpost.y) / 150.0;
        setConstantHorizon({dynringpost.x + delta_x * count1_,
                            dynringpost.y + delta_y * count1_, 1.65});
      } else {
        is_arrive_r2_postpoint_ = true;
      }
      return;
    }
    if (!is_found_ap_ && !is_arrive_ap_prepoint_) {
      return;
    }
    if (is_found_ap_ && !is_arrive_ap_prepoint_) {
      if (count2_ < 150) {
        ++count2_;
        const double delta_x = (ap_pose_.x - appre_pose_.x) / 150.0;
        const double delta_y = (ap_pose_.y - appre_pose_.y) / 150.0;
        setConstantHorizon({appre_pose_.x + delta_x * count2_,
                            appre_pose_.y + delta_y * count2_, 1.65});
      } else {
        is_arrive_ap_prepoint_ = true;
      }
      return;
    }
    if (is_found_ap_ && is_arrive_ap_prepoint_) {
      apland_ = true;
      last_mission_horizon_.clear();
      const double delta_y =
          0.4 * clampSingle(ap_pose_.y - telemetry.position.y, 0.1);
      for (std::size_t index = 0; index < kLegacyHorizonPoints; ++index) {
        smlfsm::ReferencePoint point;
        point.position.x = ap_pose_.x;
        point.position.y = ap_pose_.y + index * delta_y / 3.0;
        point.position.z = 1.65 - 0.005 * ctl_land_count_;
        last_mission_horizon_.push_back(point);
      }
      if (ctl_land_count_ < 330) {
        ++ctl_land_count_;
      } else {
        land_success_ = true;
      }
    }
  }

  // 给 mission/ego horizon 补 yaw 参考，沿用旧版 YawSmooth 行为。
  void applyMissionYaw(const smlfsm::Quaternion& current_attitude,
                       std::vector<smlfsm::ReferencePoint>& horizon) {
    if (horizon.empty()) {
      return;
    }
    if (ego_traj_.empty()) {
      ego_traj_ = createLegacyEgoTrajectory();
    }
    const int clamped_id =
        std::max(0, std::min<int>(ego_now_id_, ego_traj_.size() - 1));
    yaw_now_ = yawFromQuaternion(current_attitude);
    const double yaw = YawSmooth(yaw_now_, ego_traj_[clamped_id].yaw);
    const Eigen::Quaterniond quat = EulerToQuat(0.0, 0.0, yaw);
    for (auto& point : horizon) {
      point.attitude = {quat.w(), quat.x(), quat.y(), quat.z()};
    }
  }

  // 从四元数中提取 yaw。
  static double yawFromQuaternion(const smlfsm::Quaternion& quat) {
    return std::atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                      1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z));
  }

  // 判断三维向量是否在给定容差范围内。
  static bool near(const smlfsm::Vec3& value, const smlfsm::Vec3& reference,
                   double tolerance) {
    return std::abs(value.x - reference.x) < tolerance &&
           std::abs(value.y - reference.y) < tolerance &&
           std::abs(value.z - reference.z) < tolerance;
  }

  // 中值滤波：窗口满后输出每个轴的中位数。
  static bool medianFilter(std::vector<smlfsm::Vec3>& filter,
                           const smlfsm::Vec3& sample, std::size_t size,
                           smlfsm::Vec3& result) {
    pushFilterSample(filter, sample, size);
    if (filter.size() < size) {
      return false;
    }
    result.x = medianComponent(filter, 0);
    result.y = medianComponent(filter, 1);
    result.z = medianComponent(filter, 2);
    return true;
  }

  // 去掉最大/最小值后的均值滤波，降低圆环位置跳变影响。
  static bool medianAverageFilter(std::vector<smlfsm::Vec3>& filter,
                                  const smlfsm::Vec3& sample,
                                  std::size_t size, smlfsm::Vec3& result) {
    pushFilterSample(filter, sample, size);
    if (filter.size() < size) {
      return false;
    }
    result.x = trimmedAverageComponent(filter, 0);
    result.y = trimmedAverageComponent(filter, 1);
    result.z = trimmedAverageComponent(filter, 2);
    return true;
  }

  // 维护固定长度滤波窗口。
  static void pushFilterSample(std::vector<smlfsm::Vec3>& filter,
                               const smlfsm::Vec3& sample, std::size_t size) {
    if (filter.size() >= size) {
      filter.erase(filter.begin());
    }
    filter.push_back(sample);
  }

  // 取向量指定轴的分量，供滤波排序复用。
  static double component(const smlfsm::Vec3& value, int axis) {
    if (axis == 0) {
      return value.x;
    }
    if (axis == 1) {
      return value.y;
    }
    return value.z;
  }

  // 计算某个轴的中位数。
  static double medianComponent(const std::vector<smlfsm::Vec3>& filter,
                                int axis) {
    std::vector<smlfsm::Vec3> sorted = filter;
    std::sort(sorted.begin(), sorted.end(),
              [axis](const smlfsm::Vec3& left, const smlfsm::Vec3& right) {
                return component(left, axis) > component(right, axis);
              });
    return component(sorted[sorted.size() / 2], axis);
  }

  // 计算某个轴去掉首尾极值后的均值。
  static double trimmedAverageComponent(
      const std::vector<smlfsm::Vec3>& filter, int axis) {
    std::vector<smlfsm::Vec3> sorted = filter;
    std::sort(sorted.begin(), sorted.end(),
              [axis](const smlfsm::Vec3& left, const smlfsm::Vec3& right) {
                return component(left, axis) > component(right, axis);
              });
    double sum = 0.0;
    for (std::size_t index = 1; index + 1 < sorted.size(); ++index) {
      sum += component(sorted[index], axis);
    }
    return sum / static_cast<double>(sorted.size() - 2);
  }

  // 从历史位姿中查找时间戳最近的一帧。
  TimedPose nearestPose(double stamp) const {
    if (pose_history_.empty()) {
      TimedPose fallback;
      fallback.stamp = stamp;
      return fallback;
    }
    auto best = pose_history_.begin();
    double best_delta = std::abs(stamp - best->stamp);
    for (auto it = pose_history_.begin() + 1; it != pose_history_.end(); ++it) {
      const double delta = std::abs(stamp - it->stamp);
      if (delta < best_delta) {
        best = it;
        best_delta = delta;
      }
    }
    return *best;
  }

  // 将相机/机体系局部坐标转换为全局坐标。
  static smlfsm::Vec3 localToGlobal(const TimedPose& base,
                                    const smlfsm::Vec3& local) {
    const double cr = std::cos(base.roll);
    const double sr = std::sin(base.roll);
    const double cp = std::cos(base.pitch);
    const double sp = std::sin(base.pitch);
    const double cy = std::cos(base.yaw);
    const double sy = std::sin(base.yaw);

    smlfsm::Vec3 global;
    global.x = cp * cy * local.x + (sr * sp * cy - cr * sy) * local.y +
               (cr * sp * cy + sr * sy) * local.z + base.position.x;
    global.y = cp * sy * local.x + (sr * sp * sy + cr * cy) * local.y +
               (cr * sp * sy - sr * cy) * local.z + base.position.y;
    global.z = sp * local.x + sr * cp * local.y + cr * cp * local.z +
               base.position.z;
    return global;
  }

  // 四元数转欧拉角，供 apriltag 坐标转换使用。
  static void quaternionToEuler(const smlfsm::Quaternion& quat, double& roll,
                                double& pitch, double& yaw) {
    roll = std::atan2(2.0 * (quat.w * quat.x + quat.y * quat.z),
                      1.0 - 2.0 * (quat.x * quat.x + quat.y * quat.y));
    const double sinp = 2.0 * (quat.w * quat.y - quat.z * quat.x);
    pitch = std::abs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp)
                                  : std::asin(sinp);
    yaw = yawFromQuaternion(quat);
  }

  ros::Publisher super_waypoint_pub_;  // 发给 super planner 的 waypoint topic。
  ros::Publisher ego_waypoint_pub_;    // 发给 ego planner 的 waypoint topic。
  static constexpr std::size_t kPoseHistorySize = 200;
  static constexpr std::size_t kRing2FilterSize = 7;
  static constexpr std::size_t kApriltagFilterSize = 5;
  int selected_command_{0};           // 当前选择的任务命令。
  bool need_generate_traj_{true};     // 是否需要重新向 planner 发布 waypoint。
  std::size_t ego_traj_count_{0};     // 下一个要发布的任务航点序号。
  int ego_now_id_{0};                 // ego planner 当前航点 id。
  bool ego_touch_goal_{false};        // ego planner 是否到达当前目标。
  int task1_{0};                      // 旧版 cmd7 阶段触发变量。
  bool super_planner_valid_{false};   // 是否已有 super planner 输出。
  bool ego_planner_valid_{false};     // 是否已有 ego planner 输出。
  bool is_arrive_ring2_{false};       // 是否到达第二个圆环前。
  bool is_crossed_ring2_{false};      // 是否已穿过第二个圆环。
  bool is_arrive_r2_postpoint_{false};  // 是否到达圆环后置点。
  bool is_arrive_ap_prepoint_{false};   // 是否到达 apriltag 预降落点。
  bool is_found_ap_{false};           // 是否已发现 apriltag。
  bool apland_{false};                // 是否进入 apriltag 降落阶段。
  bool land_success_{false};          // 旧版降落完成标志。
  int ring2_approach_count_{0};       // 接近第二个圆环的插值计数。
  int ring2_cross_count_{0};          // 穿过第二个圆环的插值计数。
  int count1_{0};                     // 飞向圆环后置点的插值计数。
  int count2_{0};                     // 飞向 apriltag 预降落点的插值计数。
  int ctl_land_count_{0};             // apriltag 降落下降计数。
  double yaw_now_{0.0};               // 当前 yaw，用于 yaw 平滑。
  smlfsm::Vec3 ring2_pose_{2.0, 0.0, 1.0};  // 第二个圆环的估计位置。
  smlfsm::Vec3 appre_pose_{4.8, 0.8, 1.0};  // apriltag 预降落点。
  smlfsm::Vec3 ap_pose_{5.0, 1.1, 1.0};     // apriltag 的估计位置。
  std::vector<TimedPose> pose_history_;     // 飞机历史位姿缓存。
  std::vector<smlfsm::Vec3> ring2_filter_;  // 圆环位置滤波窗口。
  std::vector<smlfsm::Vec3> ap_filter_;     // apriltag 位置滤波窗口。
  std::vector<MissionPoint> ego_traj_;      // 旧版任务航点表。
  std::vector<smlfsm::ReferencePoint> super_planner_;  // 最近一帧 super planner 轨迹。
  std::vector<smlfsm::ReferencePoint> ego_planner_;    // 最近一帧 ego planner 轨迹。
  std::vector<smlfsm::ReferencePoint> last_mission_horizon_;  // 最近生成的 cmd7 horizon。
};

// UDP 命令邮箱：网络线程只更新原子 latest_，状态机仍由 ROS 主线程串行驱动。
class UdpCommandMailbox {
 public:
  explicit UdpCommandMailbox(int port) : port_(port) {}
  ~UdpCommandMailbox() { stop(); }

  // 启动 UDP 接收线程。
  void start() { worker_ = std::thread(&UdpCommandMailbox::run, this); }
  // 读取最近一次收到的命令。
  int latest() const { return latest_.load(std::memory_order_acquire); }

 private:
  // 停止接收线程并关闭 socket。
  void stop() {
    running_.store(false, std::memory_order_release);
    const int descriptor = socket_.exchange(-1);
    if (descriptor >= 0) {
      ::shutdown(descriptor, SHUT_RDWR);
      ::close(descriptor);
    }
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  // UDP 接收循环：只解析整数并写入 mailbox，不直接调用状态机。
  void run() {
    const int descriptor = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (descriptor < 0) {
      ROS_ERROR("Cannot create UDP command socket: %s", std::strerror(errno));
      return;
    }
    socket_.store(descriptor);
    int reuse = 1;
    ::setsockopt(descriptor, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(static_cast<uint16_t>(port_));
    if (::bind(descriptor, reinterpret_cast<sockaddr*>(&address),
               sizeof(address)) < 0) {
      ROS_ERROR("Cannot bind UDP command port %d: %s", port_,
                std::strerror(errno));
      int expected = descriptor;
      if (socket_.compare_exchange_strong(expected, -1)) {
        ::close(descriptor);
      }
      return;
    }
    char buffer[256];
    while (running_.load(std::memory_order_acquire) && ros::ok()) {
      const ssize_t length =
          ::recvfrom(descriptor, buffer, sizeof(buffer) - 1, 0, nullptr, nullptr);
      if (length <= 0) {
        continue;
      }
      buffer[length] = '\0';
      char* end = nullptr;
      const long command = std::strtol(buffer, &end, 10);
      if (end != buffer) {
        latest_.store(static_cast<int>(command), std::memory_order_release);
      }
    }
  }

  int port_;                         // UDP 监听端口，默认 12001。
  std::atomic<int> latest_{0};        // 最近一次收到的 cmd。
  std::atomic<int> socket_{-1};       // 当前 UDP socket fd，用于跨线程关闭。
  std::atomic<bool> running_{true};   // 接收线程运行标志。
  std::thread worker_;                // UDP 接收线程。
};

// 新 SML 节点的 ROS adapter：负责订阅/发布、命令分发和 50Hz Tick 调度。
class SingleOffboardNode {
 public:
  SingleOffboardNode()
      : private_node_("~"),
        autopilot_(node_),
        setpoint_(node_),
        nmpc_(private_node_),
        reference_(private_node_),
        mission_(node_),
        config_(loadConfig()),
        context_(clock_, autopilot_, setpoint_, nmpc_, reference_, mission_,
                 config_),
        machine_(context_),
        dispatcher_(machine_, &reference_, &mission_),
        mailbox_(loadUdpPort()) {
    state_sub_ = node_.subscribe("/mavros/state", 10,
                                &SingleOffboardNode::stateCallback, this);
    pose_sub_ = node_.subscribe("/mavros/local_position/pose", 10,
                               &SingleOffboardNode::poseCallback, this);
    velocity_sub_ =
        node_.subscribe("/mavros/local_position/velocity_local", 10,
                        &SingleOffboardNode::velocityCallback, this);
    planner_sub_ = node_.subscribe("/position_cmd_nmpc", 10,
                                  &SingleOffboardNode::plannerCallback, this);
    super_planner_sub_ = node_.subscribe(
        "/super/flag_cmd", 10, &SingleOffboardNode::superPlannerCallback, this);
    ego_state_sub_ = node_.subscribe(
        "/ego_planner/flag_state", 10, &SingleOffboardNode::egoStateCallback,
        this);
    ring_sub_ = node_.subscribe("/target_pose", 10,
                                &SingleOffboardNode::ringCallback, this);
    apriltag_sub_ = node_.subscribe("/tf_output", 10,
                                    &SingleOffboardNode::apriltagCallback, this);
    rc_sub_ = node_.subscribe("/mavros/rc/in", 10,
                             &SingleOffboardNode::rcCallback, this);
  }

  // 节点主循环：先发布 100 个位置 setpoint warmup，再按 UDP cmd 驱动 FSM。
  void run() {
    mailbox_.start();
    ros::Rate rate(kRateHz);
    smlfsm::PositionSetpoint warmup;
    warmup.position.z = config_.position_hold_z;
    for (int count = 0; count < 100 && ros::ok(); ++count) {
      ros::spinOnce();
      setpoint_.publishPosition(warmup);
      rate.sleep();
    }
    while (ros::ok()) {
      ros::spinOnce();
      const int command = mailbox_.latest();
      dispatcher_.update(command);
      machine_.process_event(smlfsm::Tick{});
      rate.sleep();
    }
  }

 private:
  // 加载 SML 状态机配置；未暴露的参数沿用 Config 默认值。
  smlfsm::Config loadConfig() {
    smlfsm::Config config;
    private_node_.param("nmpc_hover_thrust", config.hover_thrust,
                        config.hover_thrust);
    return config;
  }

  // 加载 UDP 命令端口，默认兼容旧节点的 12001。
  int loadUdpPort() {
    int port = 12001;
    private_node_.param("udp_port", port, port);
    return port;
  }

  // MAVROS 状态回调：更新飞控模式和解锁状态。
  void stateCallback(const mavros_msgs::State::ConstPtr& message) {
    context_.telemetry.mode = message->mode;
    context_.telemetry.armed = message->armed;
  }
  // 本地位姿回调：更新位置/姿态，并把历史位姿交给任务适配器。
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& message) {
    context_.telemetry.position = {message->pose.position.x,
                                   message->pose.position.y,
                                   message->pose.position.z};
    context_.telemetry.attitude = {
        message->pose.orientation.w, message->pose.orientation.x,
        message->pose.orientation.y, message->pose.orientation.z};
    const double stamp = message->header.stamp.isZero()
                             ? ros::Time::now().toSec()
                             : message->header.stamp.toSec();
    mission_.recordTelemetryPose(stamp, context_.telemetry);
  }
  // 本地速度回调：更新状态机遥测快照中的速度。
  void velocityCallback(
      const geometry_msgs::TwistStamped::ConstPtr& message) {
    context_.telemetry.velocity = {message->twist.linear.x,
                                   message->twist.linear.y,
                                   message->twist.linear.z};
  }
  // planner 回调：同时喂给 cmd5 参考提供器和任务适配器。
  void plannerCallback(const traj_utils::Flag::ConstPtr& message) {
    reference_.updatePlanner(*message);
    mission_.updatePlanner(*message);
  }
  // super planner 回调：短数组直接忽略，防止越界。
  void superPlannerCallback(const super_msgs::Flag::ConstPtr& message) {
    if (message->cmd.size() < kHorizonPoints) {
      ROS_WARN_THROTTLE(1.0, "Ignoring short super planner array (%zu points)",
                        message->cmd.size());
      return;
    }
    mission_.updateSuperPlanner(*message);
  }
  // ego planner 状态回调：更新任务阶段。
  void egoStateCallback(const traj_utils::FlagState::ConstPtr& message) {
    mission_.updateEgoState(*message);
  }
  // 圆环观测回调：更新任务适配器中的圆环估计。
  void ringCallback(const geometry_msgs::PoseStamped::ConstPtr& message) {
    mission_.updateRingPose(*message);
  }
  // apriltag 观测回调：更新任务适配器中的降落标记估计。
  void apriltagCallback(const geometry_msgs::PoseStamped::ConstPtr& message) {
    mission_.updateApriltagPose(*message);
  }
  // RC 回调当前只做短数组保护，保留旧 topic 契约。
  void rcCallback(const mavros_msgs::RCIn::ConstPtr& message) {
    if (message->channels.size() <= 10) {
      ROS_WARN_THROTTLE(1.0, "Ignoring short RC array (%zu channels)",
                        message->channels.size());
    }
  }

  ros::NodeHandle node_;          // 全局 ROS 命名空间。
  ros::NodeHandle private_node_;  // 节点私有参数命名空间。
  RosClock clock_;                // 状态机时间源。
  RosAutopilotPort autopilot_;    // 飞控服务适配器。
  RosSetpointPort setpoint_;      // 控制输出适配器。
  RosNmpcPort nmpc_;              // NMPC 求解适配器。
  RosReferenceProvider reference_;  // cmd5 参考轨迹适配器。
  RosMissionPort mission_;        // cmd6/7/8 任务轨迹适配器。
  smlfsm::Config config_;         // 状态机配置。
  smlfsm::Context context_;       // 状态机运行上下文。
  smlfsm::StateMachine machine_;  // Boost.SML 状态机实例。
  smlfsm::CommandDispatcher dispatcher_;  // cmd 到 Select 事件的分发器。
  UdpCommandMailbox mailbox_;     // UDP 命令 mailbox。
  ros::Subscriber state_sub_, pose_sub_, velocity_sub_, planner_sub_;  // 基础状态/参考订阅。
  ros::Subscriber super_planner_sub_, ego_state_sub_, ring_sub_, apriltag_sub_;  // 任务相关订阅。
  ros::Subscriber rc_sub_;        // RC 输入订阅。
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "single_offboard_fsm");
  SingleOffboardNode node;
  node.run();
  return 0;
}
