#pragma once

#include <ros/node_handle.h>

#include <fsm_ctrl/flight_fsm/types.hpp>

namespace fsm_ctrl {
namespace ros_adapter {

Config loadConfig(ros::NodeHandle& private_node);
int loadUdpPort(ros::NodeHandle& private_node);

}  // namespace ros_adapter
}  // namespace fsm_ctrl
