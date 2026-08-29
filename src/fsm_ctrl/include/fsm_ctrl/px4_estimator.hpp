#ifndef FSM_CTRL_PX4_ESTIMATOR_HPP_
#define FSM_CTRL_PX4_ESTIMATOR_HPP_

#include <iostream>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <fsm_ctrl/ctrl_math.hpp>



/* MoCap */
static bool is_mocap_init = false;
static Eigen::Vector3d pos_mocap = Eigen::Vector3d::Zero();
static Eigen::Quaterniond quat_mocap = Eigen::Quaterniond::Identity();
static Eigen::Vector3d euler_mocap = Eigen::Vector3d::Zero();
static Eigen::Vector3d pos_mocap_init = Eigen::Vector3d::Zero();
static Eigen::Quaterniond quat_mocap_init = Eigen::Quaterniond::Identity();

/* lidar */
static Eigen::Vector3d pos_lidar = Eigen::Vector3d::Zero();
static Eigen::Vector3d vel_lidar = Eigen::Vector3d::Zero();
static Eigen::Quaterniond quat_lidar = Eigen::Quaterniond::Identity();
static Eigen::Vector3d euler_lidar = Eigen::Vector3d::Zero();

/* camera */
static Eigen::Vector3d pos_camera = Eigen::Vector3d::Zero();
static Eigen::Vector3d vel_camera = Eigen::Vector3d::Zero();
static Eigen::Quaterniond quat_camera = Eigen::Quaterniond::Identity();
static Eigen::Vector3d euler_camera = Eigen::Vector3d::Zero();

/* FCU */
static Eigen::Vector3d pos_fcu = Eigen::Vector3d::Zero();
static Eigen::Vector3d vel_fcu = Eigen::Vector3d::Zero();
static Eigen::Quaterniond quat_fcu = Eigen::Quaterniond::Identity();
static Eigen::Vector3d euler_fcu = Eigen::Vector3d::Zero();

/* TFmini */
static double tfmini_raw = 0.0;
static double tfmini_map = 0.0;

/* EKF */
static bool is_source_new = false;
static int flag_vision_source = 0;
std_msgs::Bool ekf_ready;
geometry_msgs::PoseStamped vision_pose;


/* Callback */
void mocapCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
void lidarCallback(const nav_msgs::Odometry::ConstPtr &msg);
void cameraCallback(const nav_msgs::Odometry::ConstPtr &msg);
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
void rangeCallback(const sensor_msgs::Range::ConstPtr &msg);
double mapHeight(double range, double roll, double pitch);


#endif  // FSM_CTRL_PX4_ESTIMATOR_HPP_
