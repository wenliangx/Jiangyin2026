/**
 * @file    px4_estimator
 * @brief   subscribe odometry from sensors & publish pose to mavros for odometry - imu fusion
 * @author  FLAG Lab, BIT
 * @version 1.1
 * @date    2024-05-26
 */

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <Eigen/Dense>
#include <cmath>
#include <fsm_ctrl/ctrl_math.hpp>

namespace {

bool is_mocap_initialized = false;
Eigen::Vector3d mocap_position = Eigen::Vector3d::Zero();
Eigen::Quaterniond mocap_attitude = Eigen::Quaterniond::Identity();
Eigen::Vector3d initial_mocap_position = Eigen::Vector3d::Zero();
Eigen::Quaterniond initial_mocap_attitude = Eigen::Quaterniond::Identity();

Eigen::Vector3d lidar_position = Eigen::Vector3d::Zero();
Eigen::Quaterniond lidar_attitude = Eigen::Quaterniond::Identity();

Eigen::Vector3d camera_position = Eigen::Vector3d::Zero();
Eigen::Quaterniond camera_attitude = Eigen::Quaterniond::Identity();

Eigen::Vector3d fcu_position = Eigen::Vector3d::Zero();
Eigen::Vector3d fcu_euler = Eigen::Vector3d::Zero();

bool has_new_source_sample = false;
int vision_source = 0;
std_msgs::Bool ekf_ready;
geometry_msgs::PoseStamped vision_pose;

/**
 * @brief  motion capture pose subscriber callback
 * @param  msg: from MoCap
 * @return NULL
 */
void mocapCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  const Eigen::Vector3d raw_position(msg->pose.position.x, msg->pose.position.y,
                                     msg->pose.position.z);
  const Eigen::Quaterniond raw_attitude(msg->pose.orientation.w, msg->pose.orientation.x,
                                        msg->pose.orientation.y, msg->pose.orientation.z);
  if (!is_mocap_initialized) {
    initial_mocap_position = raw_position;
    initial_mocap_attitude = raw_attitude;
    is_mocap_initialized = true;
  }

  mocap_position = initial_mocap_attitude.inverse() * (raw_position - initial_mocap_position);
  mocap_attitude = initial_mocap_attitude.inverse() * raw_attitude;

  if (vision_source == 0) {
    vision_pose.header.stamp = msg->header.stamp;
    vision_pose.header.frame_id = "map";
    vision_pose.pose.position.x = mocap_position(0);
    vision_pose.pose.position.y = mocap_position(1);
    vision_pose.pose.position.z = mocap_position(2);
    vision_pose.pose.orientation.w = mocap_attitude.w();
    vision_pose.pose.orientation.x = mocap_attitude.x();
    vision_pose.pose.orientation.y = mocap_attitude.y();
    vision_pose.pose.orientation.z = mocap_attitude.z();
    has_new_source_sample = true;
  }
}

/**
 * @brief  lidar odometry subscriber callback
 * @param  msg: from LIO
 * @return NULL
 */
void lidarCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  lidar_position = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                   msg->pose.pose.position.z);
  lidar_attitude = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

  if (vision_source == 1) {
    vision_pose.header.stamp = msg->header.stamp;
    vision_pose.header.frame_id = "map";
    vision_pose.pose.position.x = lidar_position(0);
    vision_pose.pose.position.y = lidar_position(1);
    vision_pose.pose.position.z = lidar_position(2);
    vision_pose.pose.orientation.w = lidar_attitude.w();
    vision_pose.pose.orientation.x = lidar_attitude.x();
    vision_pose.pose.orientation.y = lidar_attitude.y();
    vision_pose.pose.orientation.z = lidar_attitude.z();
    has_new_source_sample = true;
  }
}

/**
 * @brief  camera odometry subscriber callback
 * @param  msg: from VIO
 * @return NULL
 */
void cameraCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  camera_position = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                    msg->pose.pose.position.z);
  camera_attitude = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                       msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

  if (vision_source == 2) {
    vision_pose.header.stamp = msg->header.stamp;
    vision_pose.header.frame_id = "map";
    vision_pose.pose.position.x = camera_position(0);
    vision_pose.pose.position.y = camera_position(1);
    vision_pose.pose.position.z = camera_position(2);
    vision_pose.pose.orientation.w = camera_attitude.w();
    vision_pose.pose.orientation.x = camera_attitude.x();
    vision_pose.pose.orientation.y = camera_attitude.y();
    vision_pose.pose.orientation.z = camera_attitude.z();
    has_new_source_sample = true;
  }
}

/**
 * @brief  fusion pose subscriber callback
 * @param  msg: from FCU EKF
 * @return NULL
 */
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  fcu_position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  const Eigen::Quaterniond fcu_attitude(msg->pose.orientation.w, msg->pose.orientation.x,
                                        msg->pose.orientation.y, msg->pose.orientation.z);
  fcu_euler = fsm_ctrl::quaternionToEuler(fcu_attitude);
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "px4_estimator");
  ros::NodeHandle nh;

  /* parameter */
  nh.param("/px4_estimator/vision_source", vision_source, 0);

  /* publisher */
  ros::Publisher ready_pub = nh.advertise<std_msgs::Bool>("/fsm_ctrl/ekf_ready", 1);
  ros::Publisher vision_pub =
      nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1);

  /* subscriber */
  ros::Subscriber mocap_sub =
      nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/jy0/pose", 1, mocapCallback);
  ros::Subscriber lidar_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 1, lidarCallback);
  ros::Subscriber camera_sub = nh.subscribe<nav_msgs::Odometry>("camera_odom", 1, cameraCallback);
  ros::Subscriber pose_sub =
      nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, poseCallback);

  ros::Rate rate(100.0);
  while (ros::ok()) {
    ros::spinOnce();

    if (has_new_source_sample) {
      vision_pub.publish(vision_pose);
      has_new_source_sample = false;
    }

    Eigen::Vector3d euler_vision =
        fsm_ctrl::quaternionToEuler(vision_pose.pose.orientation.w, vision_pose.pose.orientation.x,
                                    vision_pose.pose.orientation.y, vision_pose.pose.orientation.z);
    if (std::abs(vision_pose.pose.position.x - fcu_position(0)) < 0.05 &&
        std::abs(vision_pose.pose.position.y - fcu_position(1)) < 0.05 &&
        std::abs(vision_pose.pose.position.z - fcu_position(2)) < 0.05 &&
        std::abs(euler_vision(2) - fcu_euler(2)) < M_PI / 20.0) {
      ekf_ready.data = true;
    }  // else {ekf_ready.data = false;}
    ready_pub.publish(ekf_ready);

    rate.sleep();
  }

  return 0;
}
