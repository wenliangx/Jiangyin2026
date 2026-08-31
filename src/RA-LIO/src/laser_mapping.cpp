#include <ros/ros.h>

#include "mapping_node.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle node;
  ra_lio::MappingNode mapping_node(std::move(node));
  return mapping_node.run();
}
