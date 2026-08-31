// use_ikfom.hpp - IKFoM (Iterated Kalman Filter on Manifold) 状态定义和过程模型
// 功能：
//   1. 定义24维系统状态量 state_ikfom：位置、姿态、外参（LiDAR到IMU）、速度、IMU偏置、重力向量
//   2. 定义IMU输入量 input_ikfom：加速度、角速度
//   3. 定义系统过程模型函数 get_f（状态导数）、df_dx（状态雅可比）、df_dw（噪声雅可比）
//   4. 定义过程噪声协方差 process_noise_cov

#pragma once

#include <vector>
#include <cstdlib>
#include <boost/bind.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "common_lib.hpp"
#include <sophus/so3.hpp>

// === 状态量结构体 ===
// 共24维状态:
//   pos (3)      - 位置（世界坐标系）
//   rot (SO3)    - 姿态旋转（IMU系 -> 世界系）
//   offset_R_L_I (SO3) - LiDAR到IMU的外参旋转
//   offset_T_L_I (3)   - LiDAR到IMU的外参平移
//   vel (3)      - 速度（世界坐标系）
//   bg (3)       - 陀螺仪bias
//   ba (3)       - 加速度计bias
//   grav (3)     - 重力向量（世界坐标系）
struct state_ikfom
{
	Eigen::Vector3d pos = Eigen::Vector3d(0,0,0);                               // IMU在世界系下的位置
	Sophus::SO3d rot = Sophus::SO3d(Eigen::Matrix3d::Identity());               // IMU姿态旋转矩阵 R_I^W
	Sophus::SO3d offset_R_L_I = Sophus::SO3d(Eigen::Matrix3d::Identity());     // LiDAR -> IMU 外参旋转
	Eigen::Vector3d offset_T_L_I = Eigen::Vector3d(0,0,0);                     // LiDAR -> IMU 外参平移
	Eigen::Vector3d vel = Eigen::Vector3d(0,0,0);                               // IMU在世界系下的速度
	Eigen::Vector3d bg = Eigen::Vector3d(0,0,0);                                 // 陀螺仪偏差（bias）
	Eigen::Vector3d ba = Eigen::Vector3d(0,0,0);                                 // 加速度计偏差（bias）
	Eigen::Vector3d grav = Eigen::Vector3d(0,0,-G_m_s2);                        // 世界系下的重力向量（初值向下）
};

// === 输入量结构体 ===
struct input_ikfom
{
	Eigen::Vector3d acc = Eigen::Vector3d(0,0,0);   // IMU加速度测量值 a_m（IMU坐标系）
	Eigen::Vector3d gyro = Eigen::Vector3d(0,0,0);  // IMU角速度测量值 ω_m（IMU坐标系）
};

// === 过程噪声协方差矩阵 Q ===
// 12维噪声: [陀螺仪噪声(3), 加速度计噪声(3), 陀螺仪bias噪声(3), 加速度计bias噪声(3)]
Eigen::Matrix<double, 12, 12> process_noise_cov()
{
	Eigen::Matrix<double, 12, 12> Q = Eigen::MatrixXd::Zero(12, 12);
	Q.block<3, 3>(0, 0) = 0.0001 * Eigen::Matrix3d::Identity();   // 陀螺仪测量噪声协方差
	Q.block<3, 3>(3, 3) = 0.0001 * Eigen::Matrix3d::Identity();   // 加速度计测量噪声协方差
	Q.block<3, 3>(6, 6) = 0.00001 * Eigen::Matrix3d::Identity();  // 陀螺仪bias随机游走协方差
	Q.block<3, 3>(9, 9) = 0.00001 * Eigen::Matrix3d::Identity();  // 加速度计bias随机游走协方差

	return Q;
}

// === 系统状态微分方程 f(x, u) ===
// 输入: s(状态量), in(IMU输入量)
// 输出: 24维状态导数向量 [位置导数(3), 姿态导数(3), 外参旋转导数(3), 外参平移导数(3),
//                        速度导数(3), bg导数(3), ba导数(3), g导数(3)]
// 对应于论文公式(3):
//   dR/dt = R * [ω_m - b_g]_×     (pose以4个分量表示在SO(3)上)
//   dp/dt = v
//   dv/dt = R * (a_m - b_a) + g
//   db_g/dt = 0,  db_a/dt = 0,  dg/dt = 0
Eigen::Matrix<double, 24, 1> get_f(state_ikfom s, input_ikfom in)
{
	Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();
	// 去除bias后的角速度 ω = ω_m - b_g
	Eigen::Vector3d omega = in.gyro - s.bg;
	// 将加速度转到世界系：a_inertial = R * (a_m - b_a)
	Eigen::Vector3d a_inertial = s.rot.matrix() * (in.acc - s.ba);

	for (int i = 0; i < 3; i++)
	{
		res(i) = s.vel[i];                  // 位置导数 = 速度
		res(i + 3) = omega[i];              // 姿态导数 = 角速度（SO(3)上的指数映射部分）
		res(i + 12) = a_inertial[i] + s.grav[i];  // 速度导数 = 世界系加速度 + 重力
	}
	// 其余状态（外参、bias、重力）导数为0（假设为恒定值或缓变量）

	return res;
}

// === 状态转移矩阵 df/dx（公式(7)中的 F_x） ===
// 输入: s(状态量), in(IMU输入量)
// 输出: 24x24 雅可比矩阵
// 非零块对应关系:
//   (0:3, 12:15) = I          -> dp/dv
//   (12:15, 3:6) = -R * [a]_× -> dv/dθ  (a = acc - ba)
//   (12:15, 18:21) = -R       -> dv/d(ba)
//   (12:15, 21:24) = I        -> dv/dg
//   (3:6, 15:18) = -I         -> dω/d(bg)
Eigen::Matrix<double, 24, 24> df_dx(state_ikfom s, input_ikfom in)
{
	Eigen::Matrix<double, 24, 24> cov = Eigen::Matrix<double, 24, 24>::Zero();
	cov.block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();         // dp/dv = I
	Eigen::Vector3d acc_ = in.acc - s.ba;                          // 去偏置后的加速度测量值

	cov.block<3, 3>(12, 3) = -s.rot.matrix() * Sophus::SO3d::hat(acc_);  // dv/dθ = -R*[a]× (公式(7)第3行第1列)
	cov.block<3, 3>(12, 18) = -s.rot.matrix();                             // dv/d(ba) = -R (公式(7)第3行第5列)

	cov.template block<3, 3>(12, 21) = Eigen::Matrix3d::Identity();        // dv/dg = I (公式(7)第3行第6列)
	cov.template block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();        // dω/d(bg) = -I (公式(7)第1行第4列)
	return cov;
}

// === 噪声驱动矩阵 df/dw（公式(7)中的 F_w） ===
// 输入: s(状态量), in(IMU输入量)
// 输出: 24x12 矩阵（24个状态关于12个过程噪声的雅可比）
// 非零块对应关系:
//   (12:15, 3:6) = -R       -> dv/d(n_a)
//   (3:6, 0:3) = -I         -> dω/d(n_ω)  (简化为 -I)
//   (15:18, 6:9) = I        -> d(bg)/d(n_bg)
//   (18:21, 9:12) = I       -> d(ba)/d(n_ba)
Eigen::Matrix<double, 24, 12> df_dw(state_ikfom s, input_ikfom in)
{
	Eigen::Matrix<double, 24, 12> cov = Eigen::Matrix<double, 24, 12>::Zero();
	cov.block<3, 3>(12, 3) = -s.rot.matrix();                  // dv/d(n_a) = -R (公式(7)第3行第2列)
	cov.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();      // dω/d(n_ω) = -A(w*dt) ≈ -I (公式(7)第1行第1列)
	cov.block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();      // d(bg)/d(n_bg) = I (公式(7)第4行第3列)
	cov.block<3, 3>(18, 9) = Eigen::Matrix3d::Identity();      // d(ba)/d(n_ba) = I (公式(7)第5行第4列)
	return cov;
}
