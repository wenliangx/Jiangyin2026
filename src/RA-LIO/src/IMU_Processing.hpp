// IMU_Processing.hpp - IMU数据处理模块
// 功能：
//   1. IMU初始化：利用静止状态的多帧IMU数据估计初始姿态（重力方向对齐）、陀螺仪bias
//   2. 前向传播：将点云变换到世界坐标系（使用当前EKF状态进行点云"去畸变"）
//   此版本相对于FAST-LIO2做了简化，不进行逐IMU点前向传播，直接用EKF状态变换整个点云帧

#ifndef IMU_PROCESSING_HPP
#define IMU_PROCESSING_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <deque>
#include <cmath>
#include <sophus/so3.hpp>

#include "common_lib.h"
#include "use-ikfom.hpp"
#include "esekfom.hpp"

using namespace std;
using namespace Eigen;

#define MAX_INI_COUNT (20)  // IMU初始化所需的最大帧数

// 按点的时间曲率排序的比较函数
const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();

  void Reset();  // 重置IMU处理状态（重新初始化）
  // 设置IMU处理参数：LiDAR-IMU外参 (transl, rot)、噪声协方差参数
  void set_param(const V3D &transl, const M3D &rot, const V3D &gyr, const V3D &acc, const V3D &gyr_bias, const V3D &acc_bias);
  Eigen::Matrix<double, 12, 12> Q;  // 过程噪声协方差矩阵

  // 主处理函数：对measurement中的点云进行去畸变/坐标变换
  void Process(const MeasureGroup &meas, esekfom::esekf &kf_state, PointCloudXYZI::Ptr &pcl_un_);

  // 噪声协方差参数
  V3D cov_acc;       // 加速度计测量噪声
  V3D cov_gyr;       // 陀螺仪测量噪声
  V3D cov_acc_scale; // 加速度计初始噪声（初始化阶段的放大值）
  V3D cov_gyr_scale; // 陀螺仪初始噪声（初始化阶段的放大值）
  V3D cov_bias_gyr;  // 陀螺仪bias随机游走噪声
  V3D cov_bias_acc;  // 加速度计bias随机游走噪声
  double first_lidar_time;  // 第一帧LiDAR的时间戳
  Eigen::Vector4d wq;       // 四元数形式的姿态（调试用）

 private:
  // IMU初始化函数：计算初始姿态、陀螺仪bias、重力方向
  void IMU_init(const MeasureGroup &meas, esekfom::esekf &kf_state, int &N);

  PointCloudXYZI::Ptr cur_pcl_un_;  // 当前处理的"去畸变"后点云
  sensor_msgs::ImuConstPtr last_imu_;  // 上一帧IMU数据
  M3D Lidar_R_wrt_IMU;  // LiDAR -> IMU 旋转外参
  V3D Lidar_T_wrt_IMU;  // LiDAR -> IMU 平移外参
  V3D mean_acc;          // 累计平均加速度（用于初始化重力估计）
  V3D mean_gyr;          // 累计平均角速度（用于初始化陀螺仪bias估计）
  V3D angvel_last;       // 上一次角速度
  V3D acc_s_last;        // 上一次加速度
  double start_timestamp_;       // 起始时间戳
  double last_lidar_end_time_;   // 上一帧LiDAR结束时间
  int init_iter_num = 1;         // 初始化迭代计数器
  bool b_first_frame_ = true;    // 是否第一帧
  bool imu_need_init_ = true;    // 是否需要初始化IMU
};

#endif

// === 实现部分 ===

ImuProcess::ImuProcess()
{
  cov_acc = V3D(0.1, 0.1, 0.1);            // 加速度计噪声（默认值，后续从配置读取）
  cov_gyr = V3D(0.1, 0.1, 0.1);            // 陀螺仪噪声
  cov_acc_scale = V3D(0.1, 0.1, 0.1);     // 加速度计初始缩放噪声（初始化阶段放大用）
  cov_gyr_scale = V3D(0.1, 0.1, 0.1);     // 陀螺仪初始缩放噪声
  cov_bias_gyr = V3D(0.0001, 0.0001, 0.0001);  // 陀螺仪bias随机游走
  cov_bias_acc = V3D(0.0001, 0.0001, 0.0001);  // 加速度计bias随机游走
  mean_acc = V3D(0, 0, -G_m_s2);           // 平均加速度初始化为世界系重力（向下）
  mean_gyr = V3D(0, 0, 0);                 // 平均角速度初始化为0
  angvel_last = V3D(0, 0, 0);
  start_timestamp_ = -1;
  last_lidar_end_time_ = 0;
  Lidar_R_wrt_IMU = M3D::Identity();       // 外参旋转初始化为单位阵
  Lidar_T_wrt_IMU = V3D(0, 0, 0);         // 外参平移初始化为零向量
  Q = process_noise_cov();
}

ImuProcess::~ImuProcess() {}

// 重置状态：恢复为初始值，等待重新初始化
void ImuProcess::Reset()
{
  mean_acc = V3D(0, 0, -G_m_s2);
  mean_gyr = V3D(0, 0, 0);
  imu_need_init_ = true;
  init_iter_num = 1;
}

// 从外部设置IMU处理参数
void ImuProcess::set_param(const V3D &transl, const M3D &rot, const V3D &gyr, const V3D &acc, const V3D &gyr_bias, const V3D &acc_bias)
{
  Lidar_T_wrt_IMU = transl;   // LiDAR相对IMU的平移
  Lidar_R_wrt_IMU = rot;      // LiDAR相对IMU的旋转
  cov_gyr = gyr;               // 陀螺仪测量噪声协方差
  cov_acc = acc;               // 加速度计测量噪声协方差
  cov_gyr_scale = gyr;         // 保留原始值用于初始化结束后恢复
  cov_acc_scale = acc;
  cov_bias_gyr = gyr_bias;     // 陀螺仪bias噪声协方差
  cov_bias_acc = acc_bias;     // 加速度计bias噪声协方差

  Q = Eigen::Matrix<double, 12, 12>::Zero();
  Q.block<3, 3>(0, 0) = cov_gyr.asDiagonal();
  Q.block<3, 3>(3, 3) = cov_acc.asDiagonal();
  Q.block<3, 3>(6, 6) = cov_bias_gyr.asDiagonal();
  Q.block<3, 3>(9, 9) = cov_bias_acc.asDiagonal();
}

// === IMU初始化 ===
// 利用多帧IMU数据的平均值估计：
//   1. 陀螺仪bias（均值即为bias，假设静止）
//   2. 重力方向（加速度均值的方向）
//   3. 初始姿态（将重力与世界系z轴对齐）
void ImuProcess::IMU_init(const MeasureGroup &meas, esekfom::esekf &kf_state, int &N)
{
  const auto &imu_dec = meas.imu;
  // 递推更新平均值：先恢复到N-1的和，再逐帧累加
  mean_acc *= (N - 1);
  mean_gyr *= (N - 1);

  for (const auto &imu : imu_dec)
  {
    const auto &acc = imu->linear_acceleration;
    const auto &gyr = imu->angular_velocity;
    mean_acc(0) += acc.x; mean_acc(1) += acc.y; mean_acc(2) += acc.z;
    mean_gyr(0) += gyr.x; mean_gyr(1) += gyr.y; mean_gyr(2) += gyr.z;
  }
  mean_acc /= N;    // 求平均值
  mean_gyr /= N;

  const double raw_acc_norm = mean_acc.norm();
  if (!std::isfinite(raw_acc_norm) || raw_acc_norm < 1e-3)
  {
    ROS_WARN_THROTTLE(1.0, "Invalid IMU acceleration during initialization: [%.6f %.6f %.6f], skip this packet",
                      mean_acc.x(), mean_acc.y(), mean_acc.z());
    mean_acc.setZero();
    mean_gyr.setZero();
    return;
  }

  // Livox ROS Driver 2 publishes accelerometer data in g, while ROS/RA-LIO math
  // expects m/s^2. Detect g-scale data by its static norm and convert it.
  const double acc_unit_scale = raw_acc_norm < 3.0 ? G_m_s2 : 1.0;
  const V3D mean_acc_ms2 = mean_acc * acc_unit_scale;
  const double acc_norm = mean_acc_ms2.norm();

  state_ikfom init_state = kf_state.get_x();
  // g2R(mean_acc) already aligns the measured static acceleration to world +Z.
  // Gravity is a world-frame state, so keep it fixed in world -Z.
  init_state.grav = V3D(0, 0, -G_m_s2);
  // 陀螺仪bias等于静止状态下角速度的均值
  init_state.bg = mean_gyr;
  init_state.ba = V3D(0,0,0);    // 加速度计bias初始为0
  init_state.vel = V3D(0,0,0);   // 速度初始为0

  // 利用重力方向计算初始旋转矩阵，避免只初始化一列矩阵导致 NaN 四元数
  M3D R0 = g2R(mean_acc);
  init_state.rot = Sophus::SO3d(Eigen::Quaterniond(R0));

  // 外参初始化（旋转为单位阵，平移使用配置值）
  init_state.offset_R_L_I = Sophus::SO3d(Eigen::Quaterniond(Lidar_R_wrt_IMU));
  init_state.offset_T_L_I = Lidar_T_wrt_IMU;

  kf_state.change_x(init_state);  // 将初始化结果写入EKF
  // 根据实际测量的加速度大小缩放加速度噪声协方差
  cov_acc *= pow(G_m_s2 / acc_norm, 2);

  N++;
  last_imu_ = imu_dec.back();
}

// === IMU主处理函数 ===
//   初始化阶段：调用IMU_init进行姿态和bias初始化
//   正常运行阶段：用IMU做状态前向预测，同时保留当前帧点云在LiDAR/body系，
//              由后续IEKF观测模型统一变换到世界系
void ImuProcess::Process(const MeasureGroup &meas, esekfom::esekf &kf_state, PointCloudXYZI::Ptr &cur_pcl_un_)
{
  if(meas.imu.empty()) return;           // 没有IMU数据则跳过
  ROS_ASSERT(meas.lidar != nullptr);   // 确保LiDAR数据有效

  // --- 初始化阶段 ---
  if (imu_need_init_)
  {
    IMU_init(meas, kf_state, init_iter_num);
    imu_need_init_ = true;              // 标记仍需初始化
    last_imu_ = meas.imu.back();

    state_ikfom imu_state = kf_state.get_x();

    if (init_iter_num > MAX_INI_COUNT)  // 达到初始化帧数上限
    {
      imu_need_init_ = false;
      cov_acc = cov_acc_scale;           // 恢复正常的噪声协方差
      cov_gyr = cov_gyr_scale;
      ROS_INFO("IMU Initial Done");
    }
    return;
  }

  // --- 正常运行阶段：用IMU前向预测位姿与协方差 ---
  double last_timestamp = last_imu_ ? last_imu_->header.stamp.toSec() : meas.imu.front()->header.stamp.toSec();
  for (const auto &imu : meas.imu)
  {
    const double imu_time = imu->header.stamp.toSec();
    double dt = imu_time - last_timestamp;
    last_timestamp = imu_time;

    if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.1)
    {
      continue;
    }

    input_ikfom in;
    in.gyro << imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z;
    in.acc << imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z;

    const double acc_norm = in.acc.norm();
    if (!std::isfinite(acc_norm) || acc_norm < 1e-3)
    {
      continue;
    }
    if (acc_norm < 3.0)
    {
      in.acc *= G_m_s2;
    }

    kf_state.predict(dt, Q, in);
  }
  last_imu_ = meas.imu.back();

  // --- 保留LiDAR/body系点云 ---
  cur_pcl_un_->clear();
  if (meas.lidar->empty()) return;
  *cur_pcl_un_ = *meas.lidar;
}
