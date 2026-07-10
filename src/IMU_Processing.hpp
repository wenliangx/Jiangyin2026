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
#include <mutex>
#include <cmath>
#include <algorithm>
#include <sophus/so3.hpp>

#include "common_lib.h"
#include "use-ikfom.hpp"
#include "esekfom.hpp"

using namespace std;
using namespace Eigen;

#define MAX_INI_COUNT (20)

const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess(); 
  ~ImuProcess(); 
  
  void Reset(); 
  void set_param(const V3D &transl, const M3D &rot, const V3D &gyr, const V3D &acc, const V3D &gyr_bias, const V3D &acc_bias);
  Eigen::Matrix<double, 12, 12> Q;    

  void Process(const MeasureGroup &meas, esekfom::esekf &kf_state, PointCloudXYZI::Ptr &pcl_un_);

  V3D cov_acc;           
  V3D cov_gyr;            
  V3D cov_acc_scale;       
  V3D cov_gyr_scale;      
  V3D cov_bias_gyr;       
  V3D cov_bias_acc;        
  double first_lidar_time; 

  Eigen::Vector4d wq; 

 private:
  void IMU_init(const MeasureGroup &meas, esekfom::esekf &kf_state, int &N);
  void UndistortPcl(const MeasureGroup &meas, esekfom::esekf &kf_state, PointCloudXYZI &pcl_in_out);

  PointCloudXYZI::Ptr cur_pcl_un_;        
  sensor_msgs::ImuConstPtr last_imu_;     
  vector<Pose6D> IMUpose;              
  M3D Lidar_R_wrt_IMU;                    
  V3D Lidar_T_wrt_IMU;                    
  V3D mean_acc;                           
  V3D mean_gyr;                           
  V3D angvel_last;                        
  V3D acc_s_last;                         
  double start_timestamp_;                
  double last_lidar_end_time_;            
  int init_iter_num = 1;                  
  bool b_first_frame_ = true;
  bool imu_need_init_ = true;
};

#endif

// ==================== Implementation ====================

ImuProcess::ImuProcess()
{
  cov_acc = V3D(0.1, 0.1, 0.1);
  cov_gyr = V3D(0.1, 0.1, 0.1);
  cov_acc_scale = V3D(0.1, 0.1, 0.1);
  cov_gyr_scale = V3D(0.1, 0.1, 0.1);
  cov_bias_gyr = V3D(0.0001, 0.0001, 0.0001);
  cov_bias_acc = V3D(0.0001, 0.0001, 0.0001);
  mean_acc = V3D(0, 0, -G_m_s2);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = V3D(0, 0, 0);
  start_timestamp_ = -1;
  last_lidar_end_time_ = 0;
  Lidar_R_wrt_IMU = M3D::Identity();
  Lidar_T_wrt_IMU = V3D(0, 0, 0);
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() 
{
  mean_acc = V3D(0, 0, -G_m_s2);
  mean_gyr = V3D(0, 0, 0);
  imu_need_init_ = true;
  init_iter_num = 1;
  IMUpose.clear();
}

void ImuProcess::set_param(const V3D &transl, const M3D &rot, const V3D &gyr, const V3D &acc, const V3D &gyr_bias, const V3D &acc_bias) 
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU = rot;
  cov_gyr = gyr;
  cov_acc = acc;
  cov_gyr_scale = gyr;
  cov_acc_scale = acc;
  cov_bias_gyr = gyr_bias;
  cov_bias_acc = acc_bias;
}

void ImuProcess::IMU_init(const MeasureGroup &meas, esekfom::esekf &kf_state, int &N)
{
  /* accumulate initial IMU measurements */
  const auto &imu_dec = meas.imu;
  mean_acc *=(N-1);
  mean_gyr *=(N-1);
  for (const auto &imu : imu_dec)   
  {
    const auto &acc = imu->linear_acceleration;
    const auto &gyr = imu->angular_velocity;
    mean_acc(0) += acc.x; mean_acc(1) += acc.y; mean_acc(2) += acc.z;
    mean_gyr(0) += gyr.x; mean_gyr(1) += gyr.y; mean_gyr(2) += gyr.z;
  }
  mean_acc/=N;
  mean_gyr/=N;

  /* initial attitude */
  state_ikfom init_state = kf_state.get_x();
  init_state.grav = -mean_acc.norm() * mean_acc.normalized();
  init_state.bg = mean_gyr;
  init_state.ba = V3D(0,0,0);
  init_state.vel = V3D(0,0,0);
  
  /* use mean_acc to compute initial rotation */
  V3D g_world = V3D(0, 0, -G_m_s2);
  /* SVD approach: find R such that R * mean_acc ≈ g_world */
  /* Simplified: use the fact that mean_acc ≈ -g in the IMU frame when stationary */
  M3D R_imu;
  /* Use a simple approximation: align mean_acc with gravity */
  R_imu.col(2) = init_state.grav.normalized();
  V3D cross_g;
  cross_g = g_world.cross(R_imu.col(2));
  double sin_g = cross_g.norm();
  double cos_g = g_world.dot(R_imu.col(2));
  M3D Sk;
  Sk << 0.0, -cross_g.z(), cross_g.y(),
        cross_g.z(), 0.0, -cross_g.x(),
        -cross_g.y(), cross_g.x(), 0.0;
  if (sin_g > 1e-6) {
    R_imu += Sk * (R_imu - M3D::Identity()) + (Sk * Sk) * ((1.0 - cos_g) / (sin_g * sin_g)) * (R_imu - M3D::Identity());
  }
  init_state.rot = Sophus::SO3d(R_imu);

  /* offset transformation between lidar and imu */
  init_state.offset_R_L_I = Sophus::SO3d(Lidar_R_wrt_IMU);
  init_state.offset_T_L_I = Lidar_T_wrt_IMU;

  kf_state.change_x(init_state);

  cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);

  N++;
  last_imu_ = imu_dec.back();
}

void ImuProcess::UndistortPcl(const MeasureGroup &meas, esekfom::esekf &kf_state, PointCloudXYZI &pcl_in_out)
{
  auto v_imu = meas.imu;
  
  /* sort IMU measurements by time */
  v_imu.push_front(last_imu_);
  
  int Lidar_frame_n = v_imu.size();
  if(Lidar_frame_n == 0) return;
  
  double time_vel = v_imu.front()->header.stamp.toSec();
  last_imu_ = v_imu.back();
  
  state_ikfom imu_state = kf_state.get_x();
  IMUpose.clear();
  IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.matrix()));
  
  /* check gravity alignment */
  V3D tmp_grav = -mean_acc.normalized() * G_m_s2;
  double ang_acc = M3D::Identity().col(2).dot(tmp_grav);
  double ang_vel = 0.0;
  
  /* IMU forward integration */
  double dt = 0;
  for (int j = (int)v_imu.size() - 2; j >= 0; j--)
  {
    if(v_imu[j+1]->header.stamp.toSec() < meas.lidar_beg_time - first_lidar_time)
    {
      continue;
    }
    
    auto head = v_imu[j]; 
    auto tail = v_imu[j+1]; 
    
    const auto &acc = head->linear_acceleration;
    const auto &gyr = head->angular_velocity;
    
    /* IMU preintegration */
    V3D acc_imu(head->linear_acceleration.x, head->linear_acceleration.y, head->linear_acceleration.z);
    V3D angvel(head->angular_velocity.x, head->angular_velocity.y, head->angular_velocity.z);
    
    /* remove bias */
    acc_imu -= imu_state.ba;
    angvel -= imu_state.bg;
    
    dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
    if(dt < 0) dt = 0;
    
    /* predict IMU pose */
    if (IMUpose.size() > 0) {
      auto last_pose = IMUpose.back();
      
      /* orientation propagation */
      M3D RDelta = Sophus::SO3d::exp(angvel * dt).matrix();
      M3D R_imu = last_pose.rot * RDelta;
      
      /* velocity propagation */
      V3D vel_imu(last_pose.vel);
      V3D acc_world = imu_state.rot.matrix() * acc_imu + imu_state.grav;
      vel_imu += acc_world * dt;
      
      /* position propagation */
      V3D pos_imu(last_pose.pos);
      pos_imu += vel_imu * dt + 0.5 * acc_world * dt * dt;
      
      IMUpose.push_back(set_pose6d(dt, acc_imu, angvel, vel_imu, pos_imu, R_imu));
    }
  }
  
  /* de-distort points */
  pcl_in_out = meas.lidar->points;
  sort(pcl_in_out.points.begin(), pcl_in_out.points.end(), time_list);
  
  double pcl_beg_time = meas.lidar_beg_time - first_lidar_time;
  double pcl_end_time = meas.lidar_end_time - first_lidar_time;
  
  int size = pcl_in_out.size();
  
  /* find the first IMU pose after the first lidar point */
  int imu_front_n = 0;
  for (int i = 0; i < IMUpose.size() - 1; i++) {
    if (IMUpose[i].offset_time < pcl_beg_time && IMUpose[i+1].offset_time >= pcl_beg_time) {
      imu_front_n = i;
      break;
    }
  }
  
  /* de-distortion */
  for (int i = 0; i < size; i++)
  {
    double time = pcl_in_out.points[i].curvature / 1000.0;
    if(time == 0) {
      /* first point - no compensation needed */
      continue;
    }
    
    /* find the IMU pose for this point */
    int imu_next_n = imu_front_n + 1;
    while (imu_next_n < IMUpose.size() && IMUpose[imu_next_n].offset_time < time)
    {
      imu_next_n++;
    }
    
    if (imu_next_n >= IMUpose.size()) imu_next_n = IMUpose.size() - 1;
    
    auto head_pose = IMUpose[imu_next_n];
    auto tail_pose = IMUpose[imu_next_n - 1];
    
    /* interpolation ratio */
    float ratio = (time - tail_pose.offset_time) / (head_pose.offset_time - tail_pose.offset_time);
    if(ratio < 0) ratio = 0;
    if(ratio > 1) ratio = 1;
    
    /* interpolate IMU pose */
    V3D offset_T_L_I(imu_state.offset_T_L_I);
    M3D offset_R_L_I(imu_state.offset_R_L_I.matrix());
    
    /* orientation: spherical linear interpolation */
    M3D R_head(head_pose.rot);
    M3D R_tail(tail_pose.rot);
    M3D R_delta = R_tail.transpose() * R_head;
    M3D R_interp = R_tail * Sophus::SO3d(R_delta).log() * ratio;
    /* simplified: just use the head rotation */
    R_interp = R_head;
    
    V3D vel_interp(tail_pose.vel + ratio * (head_pose.vel - tail_pose.vel));
    V3D pos_interp(tail_pose.pos + ratio * (head_pose.pos - tail_pose.pos));
    
    /* compensate point */
    V3D P_i(pcl_in_out.points[i].x, pcl_in_out.points[i].y, pcl_in_out.points[i].z);
    
    M3D R_imu_current(R_interp);
    V3D T_imu_current(pos_interp);
    
    /* transform point from lidar frame to IMU frame */
    V3D P_imu = offset_R_L_I * P_i + offset_T_L_I;
    
    /* compensate motion */
    V3D P_compensate = R_imu_current.transpose() * (P_imu - T_imu_current);
    
    pcl_in_out.points[i].x = P_compensate(0);
    pcl_in_out.points[i].y = P_compensate(1);
    pcl_in_out.points[i].z = P_compensate(2);
  }
  
  last_lidar_end_time_ = pcl_end_time;
  last_imu_ = v_imu.front();
}

void ImuProcess::Process(const MeasureGroup &meas, esekfom::esekf &kf_state, PointCloudXYZI::Ptr &cur_pcl_un_)
{
  if(meas.imu.empty()) {return};
  ROS_ASSERT(meas.lidar != nullptr);

  if (imu_need_init_)   
  {
    IMU_init(meas, kf_state, init_iter_num);
    imu_need_init_ = true;
    last_imu_ = meas.imu.back();
    
    state_ikfom imu_state = kf_state.get_x();
    
    if (init_iter_num > MAX_INI_COUNT)   /* IMU init done */
    {
      imu_need_init_ = false;
      cov_acc = cov_acc_scale;
      cov_gyr = cov_gyr_scale;
      ROS_INFO("IMU Initial Done");
    }
    
    return;
  }

  /* After IMU init, undistort point cloud */
  cur_pcl_un_->clear();
  *cur_pcl_un_ = *(meas.lidar);
  
  if(cur_pcl_un_->empty()) {
    ROS_WARN("Empty point cloud after IMU init");
    return;
  }
  
  /* Simple de-distortion: just use the EKF state */
  state_ikfom imu_state = kf_state.get_x();
  int size = cur_pcl_un_->points.size();
  
  /* Convert points from lidar frame to world frame */
  for (int i = 0; i < size; i++)
  {
    V3D P_lidar(cur_pcl_un_->points[i].x, cur_pcl_un_->points[i].y, cur_pcl_un_->points[i].z);
    
    /* Transform: world = R_l2i * (R_i2w * P_lidar + T_i2w) + T_lidar */
    V3D P_imu = imu_state.offset_R_L_I.matrix() * P_lidar + imu_state.offset_T_L_I;
    V3D P_world = imu_state.rot.matrix() * P_imu + imu_state.pos;
    
    cur_pcl_un_->points[i].x = P_world(0);
    cur_pcl_un_->points[i].y = P_world(1);
    cur_pcl_un_->points[i].z = P_world(2);
  }
}
IMU_EOF

echo "Total lines: $(wc -l < /home/wenliang/Jiangyin_image/Jiangyin2026/src/RA-LIO/src/IMU_Processing.hpp)"