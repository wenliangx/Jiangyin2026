#pragma once

#include <atomic>
#include <thread>

namespace fsm_ctrl {
namespace ros_adapter {

class UdpCommandMailbox {
 public:
  explicit UdpCommandMailbox(int port);
  ~UdpCommandMailbox();

  void start();
  int latest() const;

 private:
  void stop();
  void run();

  int port_;
  std::atomic<int> latest_{0};
  std::atomic<int> socket_{-1};
  std::atomic<bool> running_{true};
  std::thread worker_;
};

}  // namespace ros_adapter
}  // namespace fsm_ctrl
