// esekfom.hpp - ESKF (Error-State Kalman Filter on Manifold) 实现
// 功能：
//   1. 定义基于SO(3)流形的误差状态卡尔曼滤波器
//   2. 提供状态预测 predict()：IMU运动模型前向传播
//   3. 提供状态更新 update_iterated_dyn_share_modified()：IEKF迭代更新
//   4. 观测模型 h_share_model()：基于点到平面距离残差
// 理论基础：FAST-LIO2 论文中的迭代卡尔曼滤波器

#ifndef ESEKFOM_EKF_HPP1
#define ESEKFOM_EKF_HPP1

#include <vector>
#include <cstdlib>
#include <boost/bind.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "use-ikfom.hpp"
#include <ikd-Tree/ikd_Tree.h>

// 收敛判定阈值：状态增量小于此值时认为收敛
const double epsi = 0.001;

namespace esekfom
{
	using namespace Eigen;

	// === 全局变量：观测模型辅助数据结构 ===
	// normvec: 存储特征点对应的平面参数 (法向量nx,ny,nz, 点到平面距离d)
	PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
	// laserCloudOri: 有效的激光特征点（经过筛选的点）
	PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
	// corr_normvect: 有效特征点对应的平面法向量
	PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
	// point_selected_surf: 标记每个点是否为有效特征点
	bool point_selected_surf[100000] = {1};

	// 动态共享数据结构：存储每次观测模型计算的中间结果
	struct dyn_share_datastruct
	{
		bool valid;                                         // 观测是否有效
		bool converge;                                     // 迭代是否收敛
		Eigen::Matrix<double, Eigen::Dynamic, 1> h;         // 观测残差向量
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> h_x; // 观测雅可比矩阵 H (对误差状态的导数)
	};

	class esekf
	{
	public:
		typedef Matrix<double, 24, 24> cov;             // 24x24 协方差矩阵类型
		typedef Matrix<double, 24, 1> vectorized_state;  // 24维误差状态向量类型

		esekf(){};
		~esekf(){};

		// 获取当前状态量
		state_ikfom get_x()
		{
			return x_;
		}

		// 获取当前协方差矩阵
		cov get_P()
		{
			return P_;
		}

		// 替换整个状态量（用于初始化）
		void change_x(state_ikfom &input_state)
		{
			x_ = input_state;
		}

		// 替换协方差矩阵
		void change_P(cov &input_cov)
		{
			P_ = input_cov;
		}

		// === 广义加法 boxplus(x, δx) ===
		// 在流形上的加法操作：将误差状态 δx (24维向量) 加到名义状态 x 上
		// 非旋转量（pos, T, vel, bg, ba, g）使用普通加法：x + δx
		// 旋转量（rot, offset_R_L_I）使用SO(3)指数映射：R * exp(δθ^)
		state_ikfom boxplus(state_ikfom x, Eigen::Matrix<double, 24, 1> f_)
		{
			state_ikfom x_r;
			// 位置：普通加法
			x_r.pos = x.pos + f_.block<3, 1>(0, 0);

			// 姿态：SO(3)乘法，右乘指数映射  R' = R * exp(δθ^)
			x_r.rot = x.rot * Sophus::SO3d::exp(f_.block<3, 1>(3, 0));
			// 外参旋转：同上
			x_r.offset_R_L_I = x.offset_R_L_I * Sophus::SO3d::exp(f_.block<3, 1>(6, 0));

			// 外参平移：普通加法
			x_r.offset_T_L_I = x.offset_T_L_I + f_.block<3, 1>(9, 0);
			// 速度：普通加法
			x_r.vel = x.vel + f_.block<3, 1>(12, 0);
			// 陀螺仪bias：普通加法
			x_r.bg = x.bg + f_.block<3, 1>(15, 0);
			// 加速度计bias：普通加法
			x_r.ba = x.ba + f_.block<3, 1>(18, 0);
			// 重力向量：普通加法
			x_r.grav = x.grav + f_.block<3, 1>(21, 0);

			return x_r;
		}

		// === IMU预测步骤 ===
		// 使用IMU测量进行状态前向传播和协方差传播
		// 参数: dt (时间间隔), Q (过程噪声协方差), i_in (IMU输入值)
		// 公式参考: x = boxplus(x, dt * f(x, u))
		//           F_x = I + dt * df_dx
		//           P = F_x * P * F_x^T + (dt * F_w) * Q * (dt * F_w)^T
		void predict(double &dt, Eigen::Matrix<double, 12, 12> &Q, const input_ikfom &i_in)
		{
			Eigen::Matrix<double, 24, 1> f_ = get_f(x_, i_in);      // 状态导数 f(x,u)  公式(3)
			Eigen::Matrix<double, 24, 24> f_x_ = df_dx(x_, i_in);   // 状态雅可比 df/dx  公式(7)
			Eigen::Matrix<double, 24, 12> f_w_ = df_dw(x_, i_in);   // 噪声雅可比 df/dw  公式(7)

			// 状态前向传播: x_k = x_{k-1} + dt * f(x, u)（流形上的欧拉积分）
			x_ = boxplus(x_, f_ * dt);

			// 状态转移矩阵: F = I + dt * F_x （补充之前缺少的单位阵和dt）
			f_x_ = Matrix<double, 24, 24>::Identity() + f_x_ * dt;

			// 协方差传播: P = F * P * F^T + G * Q * G^T
			// 其中 G = dt * F_w (离散化后的噪声驱动矩阵)
			P_ = (f_x_)*P_ * (f_x_).transpose() + (dt * f_w_) * Q * (dt * f_w_).transpose(); // 公式(8)
		}

		// === 观测模型 h_share_model ===
		// 计算观测残差和观测雅可比矩阵
		// 观测方程: 点到平面的距离 d = n^T * p_w + d0
		// 对每个有效特征点，找到地图中5个最近邻点，拟合成平面，计算残差和雅可比
		//
		// 输出: ekfom_data.h (残差向量，每个特征点一个残差)
		//       ekfom_data.h_x (H矩阵，每行对应一个特征点的12维雅可比)
		//
		// 雅可比推导 (12维 = [dp, dR, dR_ext, dT_ext]):
		//   h = n^T * (R * (R_ext * p + T_ext) + p_world)
		//   dh/dp = n^T
		//   dh/dR = n^T * [R_ext * p + T_ext]×^T * R^T
		//   dh/d(R_ext) = n^T * R * [p]× * R_ext^T
		//   dh/d(T_ext) = n^T * R
		void h_share_model(dyn_share_datastruct &ekfom_data, PointCloudXYZI::Ptr &feats_down_body,
						   KD_TREE<PointType> &ikdtree, vector<PointVector> &Nearest_Points, bool extrinsic_est)
		{
			int feats_down_size = feats_down_body->points.size();
			laserCloudOri->clear();   // 清空有效特征点
			corr_normvect->clear();   // 清空有效特征点对应法向量

#ifdef MP_EN
			omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for  // OpenMP并行遍历特征点
#endif

			for (int i = 0; i < feats_down_size; i++)  // 遍历所有特征点
			{
				PointType &point_body = feats_down_body->points[i];  // LiDAR系坐标（body系）
				PointType point_world;  // 世界系坐标

				V3D p_body(point_body.x, point_body.y, point_body.z);

				// 转换到世界坐标系：p_w = R * (R_ext * p_b + T_ext) + pos
				V3D p_global(x_.rot * (x_.offset_R_L_I * p_body + x_.offset_T_L_I) + x_.pos);
				point_world.x = p_global(0);
				point_world.y = p_global(1);
				point_world.z = p_global(2);
				point_world.intensity = point_body.intensity;

				vector<float> pointSearchSqDis(NUM_MATCH_POINTS);  // 5个最近邻点的距离平方
				auto &points_near = Nearest_Points[i];              // 按距离由近到远排序的最近邻点

				double ta = omp_get_wtime();

				if (ekfom_data.converge)
				{
					// 在ikd-Tree中搜索最近邻点
					ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);

					// 筛选有效点：需要找到5个最近邻，且最远点距离不超过5m
					point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
				}
				if (!point_selected_surf[i])
					continue;  // 无效点跳过

				Matrix<float, 4, 1> pabcd;         // 平面参数 [A, B, C, D]（归一化后）
				point_selected_surf[i] = false;

				// 用5个最近邻点估计平面
				if (esti_plane(pabcd, points_near, 0.1f))
				{
					// 计算点到平面距离 pd2 = A*x + B*y + C*z + D
					float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);

					// 权重系数：距离越大权重越小（s = 1 - 0.9 * |d| / |p_body|）
					// 如果权重 > 0.9，认为是有效平面点
					float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());
					if (s > 0.9)
					{
						point_selected_surf[i] = true;
						normvec->points[i].x = pabcd(0);          // 法向量x分量
						normvec->points[i].y = pabcd(1);          // 法向量y分量
						normvec->points[i].z = pabcd(2);          // 法向量z分量
						normvec->points[i].intensity = pd2;       // 存储点到平面距离（残差）
					}
				}
			}

			// 统计有效特征点数量
			int effct_feat_num = 0;
			for (int i = 0; i < feats_down_size; i++)
			{
				if (point_selected_surf[i])
				{
					laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];    // 存储有效点
					corr_normvect->points[effct_feat_num] = normvec->points[i];            // 存储对应的平面参数
					effct_feat_num++;
				}
			}

			if (effct_feat_num < 1)
			{
				ekfom_data.valid = false;
				ROS_WARN("No Effective Points! \n");
				return;
			}

			// === 构建雅可比矩阵 H 和残差向量 h ===
			ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12);  // 雅可比矩阵
			ekfom_data.h.resize(effct_feat_num);                   // 残差向量

			for (int i = 0; i < effct_feat_num; i++)
			{
				V3D point_(laserCloudOri->points[i].x, laserCloudOri->points[i].y, laserCloudOri->points[i].z);
				M3D point_crossmat;
				point_crossmat << SKEW_SYM_MATRX(point_);         // [p]× 反对称矩阵
				V3D point_I_ = x_.offset_R_L_I * point_ + x_.offset_T_L_I;  // LiDAR系->IMU系
				M3D point_I_crossmat;
				point_I_crossmat << SKEW_SYM_MATRX(point_I_);     // [R_ext*p + T_ext]× 反对称矩阵

				const PointType &norm_p = corr_normvect->points[i];
				V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);        // 平面法向量 n

				// 中间变量 C = R^T * n（将法向量转到IMU系）
				V3D C(x_.rot.matrix().transpose() * norm_vec);
				// 姿态部分的雅可比 A = [p_I]× * C
				V3D A(point_I_crossmat * C);

				if (extrinsic_est)  // 如果需要在线估计外参
				{
					// 外参旋转雅可比 B = [p]× * R_ext^T * C
					V3D B(point_crossmat * x_.offset_R_L_I.matrix().transpose() * C);
					// 完整的12维雅可比：[n^T, A^T, B^T, C^T]
					ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
				}
				else  // 不估计外参，后6维置零
				{
					ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
				}

				// 残差 = -点到平面距离（plane_distance = n·p + D，我们希望距离为0）
				ekfom_data.h(i) = -norm_p.intensity;
			}
		}

		// === 广义减法 boxminus(x1, x2) ===
		// 在流形上的减法操作：计算 x1 "减" x2 的误差状态（24维）
		// 非旋转量使用普通减法
		// 旋转量使用SO(3)对数映射：log(R2^T * R1)
		vectorized_state boxminus(state_ikfom x1, state_ikfom x2)
		{
			vectorized_state x_r = vectorized_state::Zero();

			x_r.block<3, 1>(0, 0) = x1.pos - x2.pos;

			// 姿态误差: R_err = R2^T * R1, δθ = log(R_err)
			x_r.block<3, 1>(3, 0) = Sophus::SO3d(x2.rot.matrix().transpose() * x1.rot.matrix()).log();
			x_r.block<3, 1>(6, 0) = Sophus::SO3d(x2.offset_R_L_I.matrix().transpose() * x1.offset_R_L_I.matrix()).log();

			x_r.block<3, 1>(9, 0) = x1.offset_T_L_I - x2.offset_T_L_I;
			x_r.block<3, 1>(12, 0) = x1.vel - x2.vel;
			x_r.block<3, 1>(15, 0) = x1.bg - x2.bg;
			x_r.block<3, 1>(18, 0) = x1.ba - x2.ba;
			x_r.block<3, 1>(21, 0) = x1.grav - x2.grav;

			return x_r;
		}

		// === 迭代更新函数 update_iterated_dyn_share_modified ===
		// 基于IEKF的LiDAR观测更新主流程
		// 参数:
		//   R: 观测噪声协方差（常数标量，简化）
		//   feats_down_body: 降采样后的特征点（LiDAR系）
		//   ikdtree: 增量KD树（存储全局地图）
		//   Nearest_Points: 最近邻点容器
		//   maximum_iter: 最大迭代次数
		//   extrinsic_est: 是否在线估计外参
		//
		// 迭代流程（论文公式(18)-(19)）:
		//   1. 计算 h(x), H = dh/dx
		//   2. 计算卡尔曼增益 K = (J^T * (H^T H / R + P^{-1})^{-1} * H^T / R
		//   3. 状态更新 dx = K * h + (K * H - I) * (x ⊞ x_prop)^{-1}
		//   4. x = x ⊞ dx
		//   5. 检查收敛：|dx| < epsi
		//   6. 协方差更新 P = (I - K * H) * P
		void update_iterated_dyn_share_modified(double R, PointCloudXYZI::Ptr &feats_down_body,
												KD_TREE<PointType> &ikdtree, vector<PointVector> &Nearest_Points, int maximum_iter, bool extrinsic_est)
		{
			normvec->resize(int(feats_down_body->points.size()));

			dyn_share_datastruct dyn_share;
			dyn_share.valid = true;
			dyn_share.converge = true;
			int t = 0;  // 连续收敛计数

			// 保存预测后的状态和协方差（用于计算状态增量）
			state_ikfom x_propagated = x_;
			cov P_propagated = P_;
			vectorized_state dx_new = vectorized_state::Zero();  // 迭代状态增量

			for (int i = -1; i < maximum_iter; i++)  // maximum_iter次迭代
			{
				dyn_share.valid = true;
				// 计算观测雅可比 H 和残差 h
				h_share_model(dyn_share, feats_down_body, ikdtree, Nearest_Points, extrinsic_est);

				if (!dyn_share.valid)
				{
					continue;
				}

				vectorized_state dx;
				// 计算当前状态与传播后状态之间的差异 dx_new = x ⊞ x_prop
				dx_new = boxminus(x_, x_propagated);

				auto H = dyn_share.h_x;                                              // m x 12 雅可比矩阵
				Eigen::Matrix<double, 24, 24> HTH = Matrix<double, 24, 24>::Zero();  // H^T * H，只有左上角12x12非零
				HTH.block<12, 12>(0, 0) = H.transpose() * H;

				// 卡尔曼增益 K = (H^T H / R + P^{-1})^{-1} * H^T / R
				auto K_front = (HTH / R + P_.inverse()).inverse();
				Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> K;
				K = K_front.block<24, 12>(0, 0) * H.transpose() / R;

				// 矩阵 K * H
				Eigen::Matrix<double, 24, 24> KH = Matrix<double, 24, 24>::Zero();
				KH.block<24, 12>(0, 0) = K * H;

				// 状态更新量 dx = K * h + (K * H - I) * dx_new  公式(18)
				Matrix<double, 24, 1> dx_ = K * dyn_share.h + (KH - Matrix<double, 24, 24>::Identity()) * dx_new;

				// 更新状态 x = x ⊞ dx    公式(18)
				x_ = boxplus(x_, dx_);

				// 检查是否收敛：所有状态增量都小于 epsi
				dyn_share.converge = true;
				for (int j = 0; j < 24; j++)
				{
					if (std::fabs(dx_[j]) > epsi)
					{
						dyn_share.converge = false;
						break;
					}
				}

				if (dyn_share.converge)
					t++;  // 连续收敛计数+1

				// 如果第3次迭代还没收敛，强制收敛（触发重新搜索最近邻）
				if (!t && i == maximum_iter - 2)
				{
					dyn_share.converge = true;
				}

				// 收敛条件：连续收敛2次 或 达到最大迭代次数
				if (t > 1 || i == maximum_iter - 1)
				{
					P_ = (Matrix<double, 24, 24>::Identity() - KH) * P_;  // 公式(19) 协方差更新
					return;
				}
			}
		}

	private:
		state_ikfom x_;                // 24维系统状态量
		cov P_ = cov::Identity();     // 24x24协方差矩阵（初始化为单位阵）
	};

} // namespace esekfom

#endif //  ESEKFOM_EKF_HPP1
