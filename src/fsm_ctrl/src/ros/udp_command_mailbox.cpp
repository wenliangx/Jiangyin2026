#include <arpa/inet.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fsm_ctrl/ros/udp_command_mailbox.hpp>

namespace fsm_ctrl {
namespace ros_adapter {

UdpCommandMailbox::UdpCommandMailbox(int port) : port_(port) {}

UdpCommandMailbox::~UdpCommandMailbox() { stop(); }

void UdpCommandMailbox::start() { worker_ = std::thread(&UdpCommandMailbox::run, this); }

int UdpCommandMailbox::latest() const { return latest_.load(std::memory_order_acquire); }

void UdpCommandMailbox::stop() {
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

void UdpCommandMailbox::run() {
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
  if (::bind(descriptor, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
    ROS_ERROR("Cannot bind UDP command port %d: %s", port_, std::strerror(errno));
    int expected = descriptor;
    if (socket_.compare_exchange_strong(expected, -1)) {
      ::close(descriptor);
    }
    return;
  }

  char buffer[256];
  while (running_.load(std::memory_order_acquire) && ros::ok()) {
    const ssize_t length = ::recvfrom(descriptor, buffer, sizeof(buffer) - 1, 0, nullptr, nullptr);
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

}  // namespace ros_adapter
}  // namespace fsm_ctrl
