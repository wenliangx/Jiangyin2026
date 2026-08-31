#pragma once

#include <ros/node_handle.h>

#include <memory>

namespace ra_lio {

class MappingNode {
 public:
  explicit MappingNode(ros::NodeHandle node);
  ~MappingNode();
  MappingNode(MappingNode&&) noexcept;
  MappingNode& operator=(MappingNode&&) noexcept;
  MappingNode(const MappingNode&) = delete;
  MappingNode& operator=(const MappingNode&) = delete;

  int run();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ra_lio
