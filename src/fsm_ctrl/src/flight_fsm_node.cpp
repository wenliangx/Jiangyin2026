#include <ros/ros.h>

#include <fsm_ctrl/ros/flight_runtime.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "flight_fsm");
  return fsm_ctrl::ros_adapter::runFlightRuntime();
}
