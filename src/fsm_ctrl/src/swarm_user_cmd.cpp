/**
 * @file    swarm_user_cmd
 * @brief   finite state machine user interaction node
 * @author  FLAG Lab, BIT
 * @version 2.1
 * @date    2024-07-18
 */

#include <arpa/inet.h>
#include <geometry_msgs/PoseStamped.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <sys/socket.h>
#include <unistd.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <fsm_ctrl/ctrl_math.hpp>
#include <iomanip>
#include <iostream>
#include <thread>

namespace {

std::atomic<int> command{-1};
Eigen::Vector3d pos_fcu = Eigen::Vector3d::Zero();
Eigen::Vector3d pos_vio = Eigen::Vector3d::Zero();
Eigen::Vector3d euler_fcu = Eigen::Vector3d::Zero();
Eigen::Vector3d euler_vio = Eigen::Vector3d::Zero();

/**
 * @brief  fusion pose subscriber callback
 * @param  _msg: geometry_msgs::PoseStamped from FCU
 * @return NULL
 */
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  pos_fcu = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  const Eigen::Quaterniond attitude(msg->pose.orientation.w, msg->pose.orientation.x,
                                    msg->pose.orientation.y, msg->pose.orientation.z);
  euler_fcu = fsm_ctrl::quaternionToEuler(attitude);
}

/**
 * @brief  odometry subscriber callback
 * @param  msg: geometry_msgs::PoseStamped from odometry
 * @return NULL
 */
void odometryCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  pos_vio = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  const Eigen::Quaterniond attitude(msg->pose.orientation.w, msg->pose.orientation.x,
                                    msg->pose.orientation.y, msg->pose.orientation.z);
  euler_vio = fsm_ctrl::quaternionToEuler(attitude);
}

void udpServer(const char* ip, const uint16_t client_port, const int uav_id) {
  // ROS_INFO("UDP %d", uav_id);
  geometry_msgs::PoseStamped offset;
  int sock_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_fd < 0) {
    ROS_ERROR("Network Error");
    return;
  }
  sockaddr_in addr_client{};
  addr_client.sin_family = AF_INET;
  if (::inet_pton(AF_INET, ip, &addr_client.sin_addr) != 1) {
    ROS_ERROR("Invalid UDP address: %s", ip);
    ::close(sock_fd);
    return;
  }
  addr_client.sin_port = htons(client_port);
  const socklen_t address_length = sizeof(addr_client);
  switch (uav_id) {
    case 1: {
      offset.pose.position.x = 0.0;
      offset.pose.position.y = 0.0;
      offset.pose.position.z = 0.0;
      break;
    }
    case 2: {
      offset.pose.position.x = -0.3;
      offset.pose.position.y = 0.3;
      offset.pose.position.z = 0.0;
      break;
    }
    case 3: {
      offset.pose.position.x = -0.3;
      offset.pose.position.y = -0.3;
      offset.pose.position.z = 0.0;
      break;
    }
    case 4: {
      offset.pose.position.x = 0.3;
      offset.pose.position.y = -0.3;
      offset.pose.position.z = 0.0;
      break;
    }
    default:
      break;
  }
  char send_buf[100];
  ros::Rate rate(10.0);
  while (ros::ok()) {
    const int current_command = command.load(std::memory_order_relaxed);
    const int message_length =
        std::snprintf(send_buf, sizeof(send_buf), "%d,%.3f,%.3f,%.3f",
                      current_command >= 10 ? 10 : current_command, offset.pose.position.x,
                      offset.pose.position.y, offset.pose.position.z);
    if (message_length < 0 || static_cast<std::size_t>(message_length) >= sizeof(send_buf)) {
      ROS_ERROR("Cannot format UDP command");
      break;
    }
    const ssize_t send_num = ::sendto(sock_fd, send_buf, static_cast<std::size_t>(message_length),
                                      0, reinterpret_cast<sockaddr*>(&addr_client), address_length);
    if (send_num < 0) {
      ROS_ERROR("Send Fail!, UAV = %d", uav_id);
      // perror("sendto error:");
      // exit(1);
    }
    // ROS_INFO("Current Pub: %s", send_buf);
    // cout << ssize_t(std::strlen(send_buf)) << endl;
    if (current_command == 10) {
      break;
    }
    rate.sleep();
  }
  ::close(sock_fd);
}

/**
 * @brief  enter user command
 * @param  NULL
 * @return NULL
 */
void readUserCommand() {
  int input = -1;
  std::cin >> input;
  command.store(input, std::memory_order_relaxed);
  std::cout << "\033[A";
  // switch(command)
  // {
  //     case 1:
  //         ROS_INFO("Arm publish!");
  //         break;
  //     case 2:
  //         ROS_INFO("Disarm publish!");
  //         break;
  //     case 3:
  //         ROS_INFO("Takeoff publish!");
  //         break;
  //     case 4:
  //         ROS_INFO("Land publish!");
  //         break;
  // }
}

/**
 * @brief  user command thread
 * @param  NULL
 * @return NULL
 */
void commandListener() {
  ros::Rate rate(10.0);
  while (ros::ok()) {
    readUserCommand();
    if (command.load(std::memory_order_relaxed) == 0) {
      break;
    }
    rate.sleep();
  }
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "swarm_user_cmd");
  ros::NodeHandle nh("~");

  ros::Subscriber pose_suber =
      nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, poseCallback);
  ros::Subscriber odom_suber =
      nh.subscribe<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1, odometryCallback);

  std::thread(commandListener).detach();
  std::thread(udpServer, "127.0.0.1", 12001, 1).detach();
  // new std::thread(&udpServer,"192.168.1.11",12001,1);
  // new std::thread(&udpServer,"192.168.1.12",12001,2);
  // new std::thread(&udpServer,"192.168.1.13",12001,3);
  // new std::thread(&udpServer,"192.168.1.14",12001,4);

  for (int i = 0; i < 9; i++) {
    std::cout << std::endl;
  }

  ros::Rate rate(10.0);
  while (ros::ok()) {
    for (int i = 0; i < 8; i++) {
      std::cout << "\033[A";
    }

    std::cout.setf(std::ios::fixed);      // 固定的浮点显示
    std::cout << std::setprecision(3);    // 固定显示精度为2位
    std::cout.setf(std::ios::left);       // 左对齐
    std::cout.setf(std::ios::showpoint);  // 强制显示小数点
    std::cout.setf(std::ios::showpos);    // 强制显示符号
    std::cout << "\r" << "\x1B[0K";
    std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>- Pose Info -<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    std::cout << "\r" << "\x1B[0K";
    std::cout << "[FCU] x: " << pos_fcu(0) << "(m)   y: " << pos_fcu(1) << "(m)   z: " << pos_fcu(2)
              << "(m)   yaw: " << euler_fcu(2) * 180.0 / M_PI << "(deg)" << std::endl;
    std::cout << "\r" << "\x1B[0K";
    std::cout << "[VIO] x: " << pos_vio(0) << "(m)   y: " << pos_vio(1) << "(m)   z: " << pos_vio(2)
              << "(m)   yaw: " << euler_vio(2) * 180.0 / M_PI << "(deg)" << std::endl;
    std::cout << "\r" << "\x1B[0K";
    std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>- User Info -<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    std::cout << "\r" << "\x1B[0K";
    std::cout << "1: Arm         2: Disarm      3: Takeoff     4: Land        5: Debug"
              << std::endl;
    std::cout << "\r" << "\x1B[0K";
    std::cout << "6:             7:             8:             9:             0: Exit" << std::endl;

    std::cout.unsetf(std::ios::showpos);
    const int current_command = command.load(std::memory_order_relaxed);
    if (current_command == -1) {
      std::cout << "\r" << "\x1B[0K";
      std::cout << "[CMD] " << std::endl;
    } else {
      std::cout << "\r" << "\x1B[0K";
      std::cout << "[CMD] " << current_command << std::endl;
    }
    std::cout << "\r" << "\x1B[0K";
    std::cout << "Enter User Command: " << std::endl;

    if (current_command == 0) {
      break;
    }
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
