#include <fsm_ctrl/ros/parameter_loader.hpp>

namespace fsm_ctrl {
namespace ros_adapter {

Config loadConfig(ros::NodeHandle& private_node) {
  Config config;
  private_node.param("service_retry_seconds", config.service_retry_seconds,
                     config.service_retry_seconds);
  private_node.param("low_thrust", config.low_thrust, config.low_thrust);
  private_node.param("nmpc_hover_thrust", config.hover_thrust, config.hover_thrust);
  private_node.param("position_hold_z", config.position_hold_z, config.position_hold_z);
  private_node.param("landing_target_z", config.landing_target_z, config.landing_target_z);
  private_node.param("landing_reference_z", config.landing_reference_z, config.landing_reference_z);
  private_node.param("landing_tolerance_z", config.landing_tolerance_z, config.landing_tolerance_z);
  return config;
}

int loadUdpPort(ros::NodeHandle& private_node) {
  int port = 12001;
  private_node.param("udp_port", port, port);
  return port;
}

}  // namespace ros_adapter
}  // namespace fsm_ctrl
