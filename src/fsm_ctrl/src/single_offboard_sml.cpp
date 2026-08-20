#include <fsm_ctrl/single_offboard_sml.hpp>
#include <fsm_ctrl/single_offboard_sml_dispatch.hpp>
#include <fsm_ctrl/NMPC_Controller.hpp>
#include <fsm_ctrl/NMPC_test.hpp>
#include <fsm_ctrl/ctrl_math.hpp>
#include <fsm_ctrl/nmpc_params.h>
#include <fsm_ctrl/nmpc_state.h>
#include <iostream>
#include <ostream>

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
#include <uav_vision_msgs/LandingOffset.h>
#include <uav_vision_msgs/TargetMatchArray.h>
#include <uav_vision_msgs/VisionControl.h>
#include <yaml-cpp/yaml.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <limits>
#include <memory>
#include <stdexcept>
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
// 精降调试 horizon 点数。
constexpr std::size_t kLandingHorizonPoints = 6;

// Super 任务航点的纯 C++ 表示。
struct MissionPoint {
  int id{0};              // 任务表内局部编号；发布时替换为全任务递增 id。
  int mode{0};            // 下游 planner 使用的任务模式。
  int is_map{0};          // 是否使用 map 坐标。
  smlfsm::Vec3 position;  // 航点位置。
  double yaw{0.0};        // 航点期望 yaw。
  double desired_speed{0.0};  // SUPER 期望速度，0 表示使用下游默认。
  int perc_mode{0};       // 感知模式字段，保留旧结构含义。
};

// 构造 Super 任务航点表。
std::vector<MissionPoint> createSuperTrajectory() {
  return std::vector<MissionPoint>{
      {0, 2, 1, {1.4, 1.5, 1.0}, 0.0, 0.0, 3},
      {1, 2, 1, {1.6, 1.2, 1.65}, 0.0, 0.0, 0},
      {2, 2, 1, {2.6, 0.7, 1.7}, 0.0, 0.0, 0},
      {3, 2, 1, {3.6, 0.2, 1.65}, 0.0, 0.0, 0},
      {4, 2, 1, {3.6, 0.2, 0.5}, 3.14, 0.0, 0},
      {5, 2, 1, {2.6, 0.7, 0.45}, 3.14, 0.0, 0},
      {6, 2, 1, {1.6, 1.2, 0.5}, 3.14, 0.0, 0},
      {7, 2, 1, {0.0, 0.0, 0.5}, 3.14, 0.0, 0},
  };
}

std::vector<std::vector<MissionPoint>> createSuperTrajectorySegments() {
  const std::vector<MissionPoint> points = createSuperTrajectory();
  return std::vector<std::vector<MissionPoint>>{
      {points[0], points[1]},
      {points[2], points[3]},
      {points[4], points[5], points[6], points[7]},
  };
}

double optionalWaypointYaw(const YAML::Node& node) {
  if (!node || node.IsNull()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  try {
    return node.as<double>();
  } catch (const YAML::Exception&) {
    std::string value = node.as<std::string>();
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) {
                     return static_cast<char>(std::tolower(c));
                   });
    if (value == "nan" || value == ".nan" || value == "free" ||
        value == "auto") {
      return std::numeric_limits<double>::quiet_NaN();
    }
    throw;
  }
}

MissionPoint parseMissionPoint(const YAML::Node& node,
                               std::size_t fallback_id) {
  MissionPoint point;
  point.id = node["id"] ? node["id"].as<int>() : static_cast<int>(fallback_id);
  point.mode = node["mode"] ? node["mode"].as<int>() : 2;
  point.is_map = node["is_map"] ? node["is_map"].as<int>() : 1;
  point.position.x = node["x"].as<double>();
  point.position.y = node["y"].as<double>();
  point.position.z = node["z"].as<double>();
  point.yaw = optionalWaypointYaw(node["yaw"]);
  point.desired_speed =
      node["desired_speed"] ? node["desired_speed"].as<double>() : 0.0;

  if (point.mode < 0 || point.mode > 2) {
    throw std::runtime_error("waypoint mode must be 0, 1, or 2");
  }
  if (point.is_map != 0 && point.is_map != 1) {
    throw std::runtime_error("waypoint is_map must be 0 or 1");
  }
  if (!std::isfinite(point.position.x) ||
      !std::isfinite(point.position.y) ||
      !std::isfinite(point.position.z)) {
    throw std::runtime_error("waypoint x/y/z must be finite");
  }
  if (!std::isfinite(point.desired_speed) || point.desired_speed < 0.0) {
    throw std::runtime_error(
        "waypoint desired_speed must be finite and non-negative");
  }
  return point;
}

std::vector<MissionPoint> loadSuperTrajectoryFile(const std::string& path) {
  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node waypoint_nodes = root["waypoints"];
  if (!waypoint_nodes || !waypoint_nodes.IsSequence() ||
      waypoint_nodes.size() == 0) {
    throw std::runtime_error("waypoint file must contain non-empty waypoints");
  }

  std::vector<MissionPoint> waypoints;
  waypoints.reserve(waypoint_nodes.size());
  for (std::size_t index = 0; index < waypoint_nodes.size(); ++index) {
    waypoints.push_back(parseMissionPoint(waypoint_nodes[index], index));
  }
  return waypoints;
}

std::vector<std::vector<MissionPoint>> loadSuperTrajectorySegmentsFile(
    const std::string& path) {
  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node segment_nodes = root["segments"];
  if (!segment_nodes || !segment_nodes.IsSequence()) {
    return std::vector<std::vector<MissionPoint>>{
        loadSuperTrajectoryFile(path)};
  }

  std::vector<std::vector<MissionPoint>> segments;
  segments.reserve(segment_nodes.size());
  for (std::size_t index = 0; index < segment_nodes.size(); ++index) {
    const YAML::Node waypoints = segment_nodes[index]["waypoints"];
    if (!waypoints || !waypoints.IsSequence() || waypoints.size() == 0) {
      throw std::runtime_error("each segment must contain non-empty waypoints");
    }

    std::vector<MissionPoint> segment;
    segment.reserve(waypoints.size());
    for (std::size_t waypoint_index = 0;
         waypoint_index < waypoints.size(); ++waypoint_index) {
      segment.push_back(parseMissionPoint(waypoints[waypoint_index],
                                          waypoint_index));
    }
    segments.push_back(segment);
  }
  return segments;
}

// Clock 的 ROS 实现，给纯 C++ Context 提供当前时间。
class RosClock final : public smlfsm::Clock {
 public:
  double now() const override { return ros::Time::now().toSec(); }
};

// AutopilotPort 的 ROS/MAVROS 实现，负责 OFFBOARD 和 arm 服务调用。
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

  void publishReferencePosition(const smlfsm::Vec3& position,
                                const smlfsm::Quaternion& attitude) override {
    nmpc_posref_pub_.publish(toPose(position, attitude));
  }

  void publishFeedbackPosition(const smlfsm::Vec3& position,
                                const smlfsm::Quaternion& attitude) override {
    nmpc_posfdb_pub_.publish(toPose(position, attitude));
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
  static geometry_msgs::PoseStamped toPose(const smlfsm::Vec3& position,
                                           const smlfsm::Quaternion& attitude =
                                               {1.0, 0.0, 0.0, 0.0}) {
    geometry_msgs::PoseStamped message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "world";
    message.pose.position.x = position.x;
    message.pose.position.y = position.y;
    message.pose.position.z = position.z;
    message.pose.orientation.w = attitude.w;
    message.pose.orientation.x = attitude.x;
    message.pose.orientation.y = attitude.y;
    message.pose.orientation.z = attitude.z;
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

    // 控制器固定构造参数，提取为具名常量，保证与参数镜像 topic 一致。
    constexpr std::array<double, 2> kAccZLimit{{0.0, 15.0}};
    constexpr std::array<double, 2> kWLimit{{-3.14, 3.14}};
    constexpr int kNlpPredictStep = 8;
    constexpr double kNlpOnestepTime = 0.05;
    constexpr int kNlpStateNum = 10;
    constexpr int kNlpInputNum = 4;

    Eigen::Matrix<float, 3, 1> q_position, q_velocity, q_attitude, r_angular;
    q_position << q_pos_x, q_pos_y, q_pos_z;
    q_velocity << q_vel_x, q_vel_y, q_vel_z;
    q_attitude << q_quat_x, q_quat_y, q_quat_z;
    r_angular << r_w_x, r_w_y, r_w_z;
    std::cout << q_pos_x << q_pos_y << q_pos_z << std::endl;
    controller_.reset(new NMPC_Ctrller_simple(
        kInterval, kAccZLimit, kWLimit, kNlpPredictStep, kNlpOnestepTime,
        kNlpStateNum, kNlpInputNum, q_position, q_velocity, q_attitude,
        r_angular, r_thrust, hover_thrust));

    // 镜像实际生效的 NMPC 参数到 /nmpc_params，仅用于录包观察，
    // 不参与任何控制计算。
    params_msg_.q_pos_x = q_pos_x;
    params_msg_.q_pos_y = q_pos_y;
    params_msg_.q_pos_z = q_pos_z;
    params_msg_.q_vel_x = q_vel_x;
    params_msg_.q_vel_y = q_vel_y;
    params_msg_.q_vel_z = q_vel_z;
    params_msg_.q_quat_x = q_quat_x;
    params_msg_.q_quat_y = q_quat_y;
    params_msg_.q_quat_z = q_quat_z;
    params_msg_.r_w_x = r_w_x;
    params_msg_.r_w_y = r_w_y;
    params_msg_.r_w_z = r_w_z;
    params_msg_.r_acc_z = r_thrust;
    params_msg_.hover_thrust = hover_thrust;
    params_msg_.ctrl_t = kInterval;
    params_msg_.acc_z_limit_low = kAccZLimit[0];
    params_msg_.acc_z_limit_high = kAccZLimit[1];
    params_msg_.w_limit_low = kWLimit[0];
    params_msg_.w_limit_high = kWLimit[1];
    params_msg_.nlp_predict_step = kNlpPredictStep;
    params_msg_.nlp_onestep_time = kNlpOnestepTime;
    params_msg_.nlp_state_num = kNlpStateNum;
    params_msg_.nlp_input_num = kNlpInputNum;

    // 锁存发布：后启动的录包/echo 也能立即收到当前参数。
    params_pub_ = private_node.advertise<fsm_ctrl::nmpc_params>(
        "/nmpc_params", 10, true);
    params_pub_.publish(params_msg_);
    // 1Hz 低频重发，保证 bag 中任意时间段都能看到这段参数。
    params_timer_ = private_node.createTimer(
        ros::Duration(1.0),
        [this](const ros::TimerEvent&) { params_pub_.publish(params_msg_); });
  }

  bool solveTrack(const smlfsm::TelemetrySnapshot& telemetry,
                  const std::vector<smlfsm::ReferencePoint>& horizon,
                  smlfsm::BodyRateThrust& command) override {
    return solve(telemetry, horizon, command);
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
      std::cout<<"x:"<<point.position.x<<", y:"<<point.position.y<<"\n";
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
  ros::Publisher params_pub_;          // /nmpc_params 参数镜像观察 topic。
  ros::Timer params_timer_;            // 参数镜像 1Hz 重发定时器。
  fsm_ctrl::nmpc_params params_msg_;   // 已加载 NMPC 参数的镜像消息。
};

class RosPrecisionLandingPort final : public smlfsm::PrecisionLandingPort {
 public:
  explicit RosPrecisionLandingPort(ros::NodeHandle& private_node) {
    private_node.param("landing_valid_timeout", valid_timeout_,
                       valid_timeout_);
    private_node.param("landing_align_px_threshold", align_px_threshold_,
                       align_px_threshold_);
    private_node.param("landing_stable_frames", stable_frames_,
                       stable_frames_);
    private_node.param("landing_pixel_gain_x", pixel_gain_x_, pixel_gain_x_);
    private_node.param("landing_pixel_gain_y", pixel_gain_y_, pixel_gain_y_);
    private_node.param("landing_max_xy_step", max_xy_step_, max_xy_step_);
    private_node.param("landing_xy_step", closed_loop_config_.xy_step,
                       closed_loop_config_.xy_step);
    private_node.param("landing_lock_min_tag_count",
                       closed_loop_config_.lock_min_tag_count,
                       closed_loop_config_.lock_min_tag_count);
    private_node.param("landing_adjust_duration_tag1",
                       closed_loop_config_.adjust_duration_tag1,
                       closed_loop_config_.adjust_duration_tag1);
    private_node.param("landing_adjust_duration_tag2",
                       closed_loop_config_.adjust_duration_tag2,
                       closed_loop_config_.adjust_duration_tag2);
    private_node.param("landing_adjust_duration_tag3",
                       closed_loop_config_.adjust_duration_tag3,
                       closed_loop_config_.adjust_duration_tag3);
    private_node.param("landing_adjust_duration_tag4",
                       closed_loop_config_.adjust_duration_tag4,
                       closed_loop_config_.adjust_duration_tag4);
    private_node.param("landing_adjust_duration_tag5",
                       closed_loop_config_.adjust_duration_tag5,
                       closed_loop_config_.adjust_duration_tag5);
    private_node.param("landing_swap_xy", closed_loop_config_.swap_xy,
                       closed_loop_config_.swap_xy);
    private_node.param("landing_x_sign", closed_loop_config_.x_sign,
                       closed_loop_config_.x_sign);
    private_node.param("landing_y_sign", closed_loop_config_.y_sign,
                       closed_loop_config_.y_sign);
    private_node.param("landing_descent_rate", descent_rate_, descent_rate_);
    private_node.param("landing_min_z", min_z_, min_z_);
    private_node.param("landing_loss_hold_seconds", loss_hold_seconds_,
                       loss_hold_seconds_);
    closed_loop_config_.observation_timeout = valid_timeout_;
    closed_loop_config_.align_px_threshold = align_px_threshold_;
    closed_loop_config_.descent_rate = descent_rate_;
    closed_loop_config_.min_z = min_z_;
    closed_loop_config_.control_rate_hz = kRateHz;
    closed_loop_config_.horizon_points = kLandingHorizonPoints;
    closed_loop_planner_.configure(closed_loop_config_);
  }

  void reset() override {
    stage_ = Stage::Acquire;
    stable_count_ = 0;
    lost_since_ = -1.0;
    target_z_ = 0.0;
    has_target_z_ = false;
    locked_xy_ = false;
    lock_x_ = 0.0;
    lock_y_ = 0.0;
    last_observation_ = smlfsm::LandingObservation{};
    closed_loop_planner_.reset();
  }

  void updateObservation(
      const smlfsm::LandingObservation& observation) override {
    last_observation_ = observation;
    closed_loop_planner_.updateObservation(observation);
  }

  void startClosedLoopLanding(
      const smlfsm::TelemetrySnapshot& telemetry) override {
    closed_loop_planner_.start(telemetry);
  }

  bool prepareLanding(double now, const smlfsm::TelemetrySnapshot& telemetry,
                      std::vector<smlfsm::ReferencePoint>& horizon) override {
    const double observation_age =
        last_observation_.stamp > 0.0 && now >= last_observation_.stamp
            ? now - last_observation_.stamp
            : -1.0;
    ROS_INFO_THROTTLE(
        0.5,
        "Landing vision monitor only: valid=%s tags=%d dx=%.2f dy=%.2f "
        "age=%.3fs",
        last_observation_.valid ? "true" : "false",
        last_observation_.tag_count, last_observation_.dx,
        last_observation_.dy, observation_age);

    // const bool observation_fresh =
    //     last_observation_.valid && now >= last_observation_.stamp &&
    //     now - last_observation_.stamp <= valid_timeout_;
    // if (!observation_fresh || std::abs(last_observation_.dx) > 50.0 ||
    //     std::abs(last_observation_.dy) > 50.0) {
    //   horizon.clear();
    //   return false;
    // }

    if (!locked_xy_) {
      lock_x_ = telemetry.position.x;
      lock_y_ = telemetry.position.y;
      target_z_ = telemetry.position.z;
      has_target_z_ = true;
      locked_xy_ = true;
    }

    target_z_ = std::max(min_z_, target_z_ - descent_rate_ / kRateHz);
    buildConstantHorizon({lock_x_, lock_y_, target_z_}, telemetry.attitude,
                         horizon);
    return true;

    /*
    // 原视觉伺服实现暂时保留：先等待有效观测，水平按 dx/dy 修正，
    // 连续稳定后进入下降。当前比赛调试改为“偏差达标后锁 x/y 下降”。
    if (!has_target_z_) {
      target_z_ = telemetry.position.z;
      has_target_z_ = true;
    }
    ...
    */
  }

  bool prepareClosedLoopLanding(
      double now, const smlfsm::TelemetrySnapshot& telemetry,
      std::vector<smlfsm::ReferencePoint>& horizon) override {
    const bool prepared =
        closed_loop_planner_.prepare(now, telemetry, horizon);
    const smlfsm::LandingObservation& observation =
        closed_loop_planner_.lastObservation();
    const smlfsm::Vec3& target = closed_loop_planner_.target();
    const double observation_age =
        observation.stamp > 0.0 && now >= observation.stamp
            ? now - observation.stamp
            : -1.0;
    ROS_INFO_THROTTLE(
        0.5,
        "Closed-loop landing: stage=%s valid=%s tags=%d dx=%.2f dy=%.2f "
        "age=%.3fs target=(%.3f, %.3f, %.3f)",
        closed_loop_planner_.descending()
            ? "descend"
            : (closed_loop_planner_.adjusting() ? "adjust" : "observe"),
        observation.valid ? "true" : "false", observation.tag_count,
        observation.dx, observation.dy, observation_age, target.x, target.y,
        target.z);
    return prepared;
  }

 private:
  enum class Stage { Acquire, Align, Descend, Touchdown };

  void buildConstantHorizon(
      const smlfsm::Vec3& target, const smlfsm::Quaternion& attitude,
      std::vector<smlfsm::ReferencePoint>& horizon) const {
    horizon.assign(kLandingHorizonPoints, smlfsm::ReferencePoint{});
    for (auto& point : horizon) {
      point.position = target;
      point.attitude = attitude;
    }
  }

  Stage stage_{Stage::Acquire};
  int stable_count_{0};
  double lost_since_{-1.0};
  double target_z_{0.0};
  bool has_target_z_{false};
  bool locked_xy_{false};
  double lock_x_{0.0};
  double lock_y_{0.0};
  smlfsm::LandingObservation last_observation_;

  double valid_timeout_{0.25};
  double align_px_threshold_{50.0};
  int stable_frames_{10};
  double pixel_gain_x_{0.001};
  double pixel_gain_y_{0.001};
  double max_xy_step_{0.08};
  double descent_rate_{0.15};
  double min_z_{0.03};
  double loss_hold_seconds_{0.5};
  smlfsm::ClosedLoopLandingConfig closed_loop_config_;
  smlfsm::ClosedLoopLandingPlanner closed_loop_planner_;
};

// 发布前视/下视相机的完整期望状态并锁存，保证晚启动的视觉节点也能立即
// 获得当前模式，而不依赖可能丢失的一次性启停脉冲。
class RosCameraControlPort final : public smlfsm::CameraControlPort {
 public:
  RosCameraControlPort(ros::NodeHandle& node,
                       ros::NodeHandle& private_node) {
    std::string topic = "/vision/control";
    private_node.param("vision_control_topic", topic, topic);
    publisher_ =
        node.advertise<uav_vision_msgs::VisionControl>(topic, 1, true);
  }

  void publishControl(const smlfsm::CameraControlState& control) override {
    uav_vision_msgs::VisionControl message;
    message.header.stamp = ros::Time::now();
    message.front_camera_enabled = control.front_camera_enabled;
    message.down_camera_enabled = control.down_camera_enabled;
    publisher_.publish(message);

    const bool changed =
        !has_last_control_ ||
        last_control_.front_camera_enabled != control.front_camera_enabled ||
        last_control_.down_camera_enabled != control.down_camera_enabled;
    if (changed) {
      ROS_INFO("Camera control: front=%s down=%s",
               message.front_camera_enabled ? "enabled" : "disabled",
               message.down_camera_enabled ? "enabled" : "disabled");
    } else {
      ROS_DEBUG_THROTTLE(1.0, "Camera control heartbeat: front=%s down=%s",
                         message.front_camera_enabled ? "enabled" : "disabled",
                         message.down_camera_enabled ? "enabled" : "disabled");
    }
    last_control_ = control;
    has_last_control_ = true;
  }

 private:
  ros::Publisher publisher_;
  smlfsm::CameraControlState last_control_;
  bool has_last_control_{false};
};

// Super 任务轨迹适配器：负责 waypoint 发布和 planner 缓存。
class RosMissionPort final : public smlfsm::MissionPort {
 public:
  RosMissionPort(ros::NodeHandle& node, ros::NodeHandle& private_node)
      : super_waypoint_pub_(node.advertise<super_msgs::Flag>(
            "/super/flag_waypoint", 10)) {
    private_node.param("mission_super_waypoints_file", waypoints_file_,
                       waypoints_file_);
    ROS_INFO("Mission super waypoints file: %s", waypoints_file_.c_str());
    private_node.param("mission_super_arrival_tolerance",
                       arrival_tolerance_, arrival_tolerance_);
    private_node.param("mission_segment_timeout_seconds",
                       segment_timeout_seconds_, segment_timeout_seconds_);
    loadSuperTrajectory();
    reset();
    // 构造阶段只初始化状态，不消耗第一段的全局航点 id。
    next_super_waypoint_id_ = 0;
    current_batch_start_id_ = 0;
    current_batch_end_id_ = 0;
  }

  void selectCommand(int command) override {
    if (command >= 3 && command <= 5) {
      active_segment_index_ = command - 3;
    }
  }

  void reset() override {
    super_waypoint_upload_active_ = true;
    super_waypoint_upload_index_ = 0u;
    if (active_segment_index_ >= 0 &&
        static_cast<std::size_t>(active_segment_index_) <
            super_segments_.size()) {
      current_batch_start_id_ = next_super_waypoint_id_;
      current_batch_end_id_ = current_batch_start_id_ +
          static_cast<int>(super_segments_[active_segment_index_].size());
      next_super_waypoint_id_ = current_batch_end_id_;
    }
    segment_arrived_ = false;
    segment_completed_at_ = -1.0;
    super_goal_reached_ = false;
    super_planner_valid_ = false;
  }

  // cmd6：发布任务航点，并优先使用 super planner 返回的 horizon。
  bool prepareSuper(double now, const smlfsm::TelemetrySnapshot& telemetry,
                    std::vector<smlfsm::ReferencePoint>& horizon) override {
    return prepareSuperSegment(active_segment_index_, now, telemetry, horizon);
  }

  bool prepareSuperSegment(
      int segment_index, double now,
      const smlfsm::TelemetrySnapshot& telemetry,
      std::vector<smlfsm::ReferencePoint>& horizon) override {
    active_segment_index_ = segment_index;
    std::cout<<"active_segment_index_:"<<active_segment_index_<<"\n";
    if (super_segments_.empty()) {
      std::cout<<"here\n";
      loadSuperTrajectory();
    }
    if (segment_index < 0 ||
        static_cast<std::size_t>(segment_index) >= super_segments_.size() ||
        super_segments_[segment_index].empty()) {
      horizon.clear();
      return false;
    }

    const MissionPoint& last = super_segments_[segment_index].back();
    if (!segment_arrived_ && super_goal_reached_ &&
        reached(last.position, telemetry.position)) {
      segment_arrived_ = true;
      segment_completed_at_ = now;
    }
    if (segment_arrived_) {
      buildHoldHorizon(last, horizon);
      return true;
    }

    publishNextWaypoint();
    if (super_planner_valid_) {
      horizon = super_planner_;
      return true;
    }
    buildCurrentHoldHorizon(telemetry, horizon);
    return true;
  }

  bool isSuperSegmentTimedOut(double now) const override {
    return active_segment_index_ < 2 && segment_arrived_ &&
           segment_completed_at_ >= 0.0 &&
           now - segment_completed_at_ >= segment_timeout_seconds_;
  }

  bool isFinalSuperSegmentComplete() const override {
    return active_segment_index_ == 2 && segment_arrived_;
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
      // 将 yaw 归一化到 [-π, π]，避免 super planner 输出的越界 yaw
      // （如 230°）导致四元数表示歧义和 NMPC 优化跳变。
      double yaw = message.cmd[index].yaw;
      yaw = std::fmod(yaw + M_PI, 2.0 * M_PI);
      if (yaw < 0.0) {
        yaw += 2.0 * M_PI;
      }
      yaw -= M_PI;
      point.attitude = {std::cos(yaw * 0.5), 0.0, 0.0, std::sin(yaw * 0.5)};
      super_planner_.push_back(point);
      
    }
    super_goal_reached_ = message.touch_goal != 0;
    super_planner_valid_ = true;
  }

 private:
  // 每个 tick 最多上传一个 waypoint，避免一次性打满下游订阅缓存。
  void publishNextWaypoint() {
    if (!super_waypoint_upload_active_) {
      return;
    }
    if (super_segments_.empty()) {
      loadSuperTrajectory();
    }
    if (active_segment_index_ < 0 ||
        static_cast<std::size_t>(active_segment_index_) >=
            super_segments_.size()) {
      return;
    }
    const std::vector<MissionPoint>& segment =
        super_segments_[active_segment_index_];
    if (super_waypoint_upload_index_ < segment.size()) {
      publishSuperWaypoint(segment[super_waypoint_upload_index_]);
      ++super_waypoint_upload_index_;
    }
    if (super_waypoint_upload_index_ >= segment.size()) {
      super_waypoint_upload_active_ = false;
      super_waypoint_upload_index_ = 0u;
    }
  }

  // 发布 super planner 使用的 waypoint 消息。
  void publishSuperWaypoint(const MissionPoint& point) {
    super_msgs::Flag message;
    fillCommonFlag1(point, message);
    message.total_waypoint = static_cast<int16_t>(current_batch_end_id_);
    super_waypoint_pub_.publish(message);
    ROS_INFO("Published mission super waypoint id=%d segment=%d index=%zu "
             "position=(%.3f, %.3f, %.3f) yaw=%.3f source=%s",
             message.id, active_segment_index_, super_waypoint_upload_index_,
             point.position.x, point.position.y, point.position.z, point.yaw,
             waypoints_file_.empty() ? "built-in" : waypoints_file_.c_str());
  }

  void loadSuperTrajectory() {

    if (waypoints_file_.empty()) {
      std::cout<<"waypoints_file_ is empty\n";
      super_segments_ = createSuperTrajectorySegments();
      return;
    }
    try {
      super_segments_ = loadSuperTrajectorySegmentsFile(waypoints_file_);
      std::size_t waypoint_count = 0u;
      for (const auto& segment : super_segments_) {
        waypoint_count += segment.size();
      }
      ROS_INFO("Loaded %zu mission super waypoints in %zu segments from %s",
               waypoint_count, super_segments_.size(), waypoints_file_.c_str());
    } catch (const std::exception& error) {
      ROS_ERROR("Failed to load mission super waypoints from %s: %s; "
                "falling back to built-in waypoints",
                waypoints_file_.c_str(), error.what());
      super_segments_ = createSuperTrajectorySegments();
    }
  }

  bool reached(const smlfsm::Vec3& target, const smlfsm::Vec3& position) const {
    const double dx = target.x - position.x;
    const double dy = target.y - position.y;
    const double dz = target.z - position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz) <= arrival_tolerance_;
  }

  void buildHoldHorizon(const MissionPoint& point,
                        std::vector<smlfsm::ReferencePoint>& horizon) const {
    horizon.assign(kHorizonPoints, smlfsm::ReferencePoint{});
    const smlfsm::Quaternion attitude{
        std::cos(point.yaw * 0.5), 0.0, 0.0, std::sin(point.yaw * 0.5)};
    for (auto& reference : horizon) {
      reference.position = point.position;
      reference.attitude = attitude;
    }
  }

  void buildCurrentHoldHorizon(
      const smlfsm::TelemetrySnapshot& telemetry,
      std::vector<smlfsm::ReferencePoint>& horizon) const {
    horizon.assign(kHorizonPoints, smlfsm::ReferencePoint{});
    for (auto& reference : horizon) {
      reference.position = telemetry.position;
      reference.attitude = telemetry.attitude;
    }
  }

  void fillCommonFlag1(const MissionPoint& point, super_msgs::Flag& message) {
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "world";
    message.id = current_batch_start_id_ +
                 static_cast<int>(super_waypoint_upload_index_);
    message.mode = point.mode;
    message.is_map = point.is_map;
    message.yaw = point.yaw;
    message.desired_speed = point.desired_speed;
    message.position.x = point.position.x;
    message.position.y = point.position.y;
    message.position.z = point.position.z;
  }

  ros::Publisher super_waypoint_pub_;  // 发给 super planner 的 waypoint topic。
  bool super_waypoint_upload_active_{true};  // 是否正在逐 tick 上传 waypoint。
  std::size_t super_waypoint_upload_index_{0};  // 本组待上传 waypoint 下标。
  int next_super_waypoint_id_{0};  // 下一个批次可使用的全局 id，切段不归零。
  int current_batch_start_id_{0}; // 当前分段首个全局 id。
  int current_batch_end_id_{0};   // 当前分段最后一个 id 的后一位。
  int active_segment_index_{0};    // cmd3/4/5 选择的当前段。
  bool segment_arrived_{false};    // 当前段是否已到末点并切为 NMPC 定点。
  bool super_goal_reached_{false}; // SUPER 是否报告当前轨迹已经完成。
  double arrival_tolerance_{0.2};  // 判定到达段末点的三维距离阈值。
  double segment_timeout_seconds_{5.0};  // 末点无识别结果时自动切段等待。
  double segment_completed_at_{-1.0};   // 到达末点时刻，负值表示未到达。
  bool super_planner_valid_{false};   // 是否已有 super planner 输出。
  std::string waypoints_file_;     // mission super waypoint YAML 文件。
  std::vector<std::vector<MissionPoint>> super_segments_;  // 分段 Super 航点表。
  std::vector<smlfsm::ReferencePoint> super_planner_;  // 最近一帧 super planner 轨迹。
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
        landing_(private_node_),
        mission_(node_, private_node_),
        camera_control_(node_, private_node_),
        config_(loadConfig()),
        context_(clock_, autopilot_, setpoint_, nmpc_, mission_, landing_,
                 camera_control_, config_),
        machine_(context_),
        dispatcher_(machine_, &mission_),
        mailbox_(loadUdpPort()) {
    state_sub_ = node_.subscribe("/mavros/state", 10,
                                &SingleOffboardNode::stateCallback, this);
    pose_sub_ = node_.subscribe("/mavros/local_position/pose", 10,
                               &SingleOffboardNode::poseCallback, this);
    velocity_sub_ =
        node_.subscribe("/mavros/local_position/velocity_local", 10,
                        &SingleOffboardNode::velocityCallback, this);
    super_planner_sub_ = node_.subscribe(
        "/super/flag_cmd", 10, &SingleOffboardNode::superPlannerCallback, this);
    std::string landing_topic = "/vision/landing/offset";
    private_node_.param("precision_landing_topic", landing_topic,
                        landing_topic);
    landing_sub_ = node_.subscribe(landing_topic, 10,
                                   &SingleOffboardNode::landingCallback, this);
    std::string target_topic = "/vision/target/result";
    private_node_.param("target_recognition_topic", target_topic,
                        target_topic);
    target_sub_ = node_.subscribe(target_topic, 10,
                                  &SingleOffboardNode::targetCallback, this);
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
      const bool command_changed = dispatcher_.update(command);
      if (command_changed && command >= 3 && command <= 5) {
        final_landing_dispatched_ = false;
      }
      machine_.process_event(smlfsm::Tick{});
      if (mission_.isSuperSegmentTimedOut(clock_.now())) {
        ROS_WARN("No qualifying target recognized within the segment timeout; "
                 "advancing to the next SUPER segment");
        machine_.process_event(smlfsm::OnSegmentTimeout{});
      }
      if (!final_landing_dispatched_ &&
          mission_.isFinalSuperSegmentComplete()) {
        final_landing_dispatched_ = true;
        ROS_INFO("Final SUPER segment complete; starting landing");
        machine_.process_event(smlfsm::OnFinalSegmentComplete{});
      }
      rate.sleep();
    }
  }

 private:
  // 加载 SML 状态机配置；默认值继续来自 Config，保证未配置时行为不变。
  smlfsm::Config loadConfig() {
    smlfsm::Config config;
    private_node_.param("service_retry_seconds", config.service_retry_seconds,
                        config.service_retry_seconds);
    private_node_.param("low_thrust", config.low_thrust, config.low_thrust);
    private_node_.param("nmpc_hover_thrust", config.hover_thrust,
                        config.hover_thrust);
    private_node_.param("position_hold_z", config.position_hold_z,
                        config.position_hold_z);
    private_node_.param("landing_target_z", config.landing_target_z,
                        config.landing_target_z);
    private_node_.param("landing_reference_z", config.landing_reference_z,
                        config.landing_reference_z);
    private_node_.param("landing_tolerance_z", config.landing_tolerance_z,
                        config.landing_tolerance_z);
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
  // 本地位姿回调：更新位置/姿态。
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& message) {
    context_.telemetry.position = {message->pose.position.x,
                                   message->pose.position.y,
                                   message->pose.position.z};
    context_.telemetry.attitude = {
        message->pose.orientation.w, message->pose.orientation.x,
        message->pose.orientation.y, message->pose.orientation.z};
  }
  // 本地速度回调：更新状态机遥测快照中的速度。
  void velocityCallback(
      const geometry_msgs::TwistStamped::ConstPtr& message) {
    context_.telemetry.velocity = {message->twist.linear.x,
                                   message->twist.linear.y,
                                   message->twist.linear.z};
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
  // 下视 AprilTag 像素偏差回调：缓存观测供活动任务的闭环降落使用。
  void landingCallback(
      const uav_vision_msgs::LandingOffset::ConstPtr& message) {
    smlfsm::LandingObservation observation;
    observation.valid =
        message->valid && message->tag_count >= 1 &&
        message->tag_count <= 5 &&
        std::isfinite(message->dx) && std::isfinite(message->dy);
    observation.dx = message->dx;
    observation.dy = message->dy;
    observation.tag_count = message->tag_count;
    observation.stamp = message->header.stamp.isZero()
                            ? ros::Time::now().toSec()
                            : message->header.stamp.toSec();
    observation.age = ros::Time::now().toSec() - observation.stamp;
    landing_.updateObservation(observation);
  }

  void targetCallback(
      const uav_vision_msgs::TargetMatchArray::ConstPtr& message) {
    const bool valid = message->valid && !message->matches.empty();
    if (!valid) {
      last_target_label_.clear();
      return;
    }
    const std::string& label = message->matches.front().label;
    if (label.empty() || label == last_target_label_) {
      return;
    }
    last_target_label_ = label;
    ROS_INFO("Stable target recognized: %s", label.c_str());
    machine_.process_event(smlfsm::OnTargetRecognized{label});
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
  RosPrecisionLandingPort landing_;  // 下视视觉精准降落适配器。
  RosMissionPort mission_;        // cmd6/7/8 任务轨迹适配器。
  RosCameraControlPort camera_control_;  // 前视/下视相机启停状态发布器。
  smlfsm::Config config_;         // 状态机配置。
  smlfsm::Context context_;       // 状态机运行上下文。
  smlfsm::ActiveStateMachine machine_;  // 当前节点使用的状态机实例。
  smlfsm::ActiveCommandDispatcher dispatcher_;  // 当前节点 cmd 分发器。
  UdpCommandMailbox mailbox_;     // UDP 命令 mailbox。
  ros::Subscriber state_sub_, pose_sub_, velocity_sub_;  // 基础状态订阅。
  ros::Subscriber super_planner_sub_;  // super planner 订阅。
  ros::Subscriber landing_sub_;  // 下视视觉降落偏差订阅。
  ros::Subscriber target_sub_;   // 前视稳定目标识别结果订阅。
  ros::Subscriber rc_sub_;        // RC 输入订阅。
  std::string last_target_label_;  // 抑制分类节点连续发布的同类别结果。
  bool final_landing_dispatched_{false};  // 防止末段完成事件在 50Hz 重复发送。
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "single_offboard_fsm");
  SingleOffboardNode node;
  node.run();
  return 0;
}
