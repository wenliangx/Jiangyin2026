/* INCLUDE */
#include <iostream>
#include <vector>
#include <fstream>

/* ROS头文件 */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseArray.h>
#include <traj_utils/Flag.h>
#include <geometry_msgs/PoseStamped.h>

/* EIGEN头文件 */
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>

/*************************************************
 * 范数约束下卡尔曼滤波
 *  输入：
 *      先验估计；
 *      先验估计协方差矩阵；
 *      当前测量；
 *      观测矩阵；
 *      观测噪声协方差矩阵；
 *      范数约束等式值；
 *  输出：
 *      后验估计；
 *      后验估计协方差矩阵；
 *************************************************/
std::pair<Eigen::MatrixXd, Eigen::MatrixXd> NCKF(Eigen::MatrixXd x, Eigen::MatrixXd P, 
                                                 Eigen::MatrixXd z, Eigen::MatrixXd H, 
                                                 Eigen::MatrixXd R, double l)
{
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> post_result;
    Eigen::MatrixXd inv_W = (H * P * H.transpose() + R).inverse();
    Eigen::MatrixXd e = z - H * x;
    Eigen::MatrixXd e_w = e.transpose() * inv_W * e;
    double lam = -1.0 / e_w(0, 0) 
                 + (x + P * H.transpose() * inv_W * e).norm() / (e_w(0, 0) * sqrt(l));
    Eigen::MatrixXd K = P * H.transpose() * inv_W - lam * x * e.transpose() * inv_W
                        - (lam / (1.0 + (lam * e.transpose() * inv_W * e)(0, 0))) * P 
                          * H.transpose() * inv_W * e * e.transpose() * inv_W
                        + (lam * lam / (1.0 + (lam * e.transpose() * inv_W * e)(0, 0))) * x
                          * e.transpose() * inv_W * e * e.transpose() * inv_W;
    Eigen::MatrixXd x_post = x + K * e;
    Eigen::MatrixXd I = P;
    I.setIdentity();
    Eigen::MatrixXd P_post = (I - K * H) * P * (I - K * H).transpose() 
                             + K * R * K.transpose();
    post_result.first = x_post;
    post_result.second = P_post;
    return post_result;
}

/*************************************************
 * 计算方差递推关系
 *  输入：
 *      先验方差；
 *      观测矩阵；
 *      观测协方差；
 *  输出：
 *      后验方差；
 *************************************************/
Eigen::MatrixXd calculate_next_P(Eigen::MatrixXd P, Eigen::MatrixXd H, Eigen::MatrixXd R)
{
    Eigen::MatrixXd P_result;
    Eigen::MatrixXd HT = H.transpose();
    P_result = P - P * HT * (H * P * HT + R).inverse() * H * P;
    return P_result;
}

/*************************************************
 * 目标分布可视化函数
 *  输入：
 *      目标分布值；
 *      空间步长；
 *      高度缩放系数；
 *  输出：
 *      密度值点云；
 *************************************************/
sensor_msgs::PointCloud visualize_density(std::vector<std::vector<double>> density, 
                                          double step, double factor)
{
    sensor_msgs::PointCloud cloud_result;
    for (int i = 0; i < density.size(); i++)
    {
        for (int j = 0; j < density[0].size(); j++)
        {
            geometry_msgs::Point32 pt_temp;
            pt_temp.x = i * step;
            pt_temp.y = j * step;
            pt_temp.z = density[i][j] * factor;
            cloud_result.points.push_back(pt_temp);
        } 
    }
    return cloud_result;
}

/*************************************************
 * 计算二次型状态价值函数及其偏导数
 *  输入：
 *      状态向量；
 *      权重矩阵；
 *  输出：
 *      价值函数；
 *      状态价值一阶偏导数；
 *      状态价值二阶偏导数；
 *************************************************/
std::vector<Eigen::MatrixXd> get_Vs_Q(Eigen::MatrixXd x, Eigen::MatrixXd Q)
{
    std::vector<Eigen::MatrixXd> Vs_result;
    Eigen::MatrixXd V = x.transpose() * Q * x;
    Vs_result.push_back(V);
    Eigen::MatrixXd V_x = 2.0 * Q * x;
    Vs_result.push_back(V_x);
    Eigen::MatrixXd V_xx = 2.0 * Q;
    Vs_result.push_back(V_xx);
    return Vs_result;
}

/*************************************************
 * 计算线性状态价值函数及其偏导数
 *  输入：
 *      状态向量；
 *      权重矩阵；
 *  输出：
 *      价值函数；
 *      状态价值一阶偏导数；
 *      状态价值二阶偏导数；
 *************************************************/
std::vector<Eigen::MatrixXd> get_Vs_L(Eigen::MatrixXd x, Eigen::MatrixXd alpha)
{
    std::vector<Eigen::MatrixXd> Vs_result;
    Eigen::MatrixXd V = alpha.transpose() * x;
    Vs_result.push_back(V);
    Eigen::MatrixXd V_x = alpha;
    Vs_result.push_back(V_x);
    Eigen::MatrixXd V_xx = Eigen::MatrixXd::Zero(x.rows(), x.rows());
    Vs_result.push_back(V_xx);
    return Vs_result;
}

/*************************************************
 * 计算方差矩阵中元素对x的导数
 *  输入：
 *      先验协方差；
 *      观测矩阵；
 *      观测协方差；
 *      观测矩阵对位置x的导数(按x拆分行向量)；
 *  输出：
 *      协方差矩阵对x的导数(按x拆分列向量)；
 *************************************************/
std::vector<Eigen::MatrixXd> calculate_nabla_P(Eigen::MatrixXd Pk, Eigen::MatrixXd Hk, 
                                               Eigen::MatrixXd Rk,
                                               std::vector<Eigen::MatrixXd> nabla_H)
{
    std::vector<Eigen::MatrixXd> nabla_P_result;
    int Pk_dim = Pk.rows();
    Eigen::MatrixXd Hk_T = Hk.transpose();
    Eigen::MatrixXd inv_W = (Hk * Pk * Hk_T + Rk).inverse();
    Eigen::MatrixXd WHP = inv_W * Hk * Pk;
    Eigen::MatrixXd WHP_T = WHP.transpose();
    Eigen::MatrixXd first_term_0 = -Pk * nabla_H[0].transpose() * WHP;
    Eigen::MatrixXd first_term_0_T = first_term_0.transpose();
    first_term_0 = first_term_0 + first_term_0_T;
    Eigen::MatrixXd first_term_1 = -Pk * nabla_H[1].transpose() * WHP;
    Eigen::MatrixXd first_term_1_T = first_term_1.transpose();
    first_term_1 = first_term_1 + first_term_1_T;
    Eigen::MatrixXd nabla_W_0 = nabla_H[0] * Pk * Hk_T;
    Eigen::MatrixXd nabla_W_1 = nabla_H[1] * Pk * Hk_T;
    Eigen::MatrixXd second_term_0 = WHP_T * nabla_W_0 * WHP;
    Eigen::MatrixXd second_term_0_T = second_term_0.transpose();
    second_term_0 = second_term_0 + second_term_0_T;
    Eigen::MatrixXd second_term_1 = WHP_T * nabla_W_1 * WHP;
    Eigen::MatrixXd second_term_1_T = second_term_1.transpose();
    second_term_1 = second_term_1 + second_term_1_T;
    int idx_count;
    Eigen::MatrixXd matrix_temp, nabla_P_vec;
    nabla_P_vec = Eigen::MatrixXd::Zero(Pk_dim*(Pk_dim+1)/2, 1);
    matrix_temp = first_term_0 + second_term_0;
    idx_count = 0;
    for (int i = 0; i < Pk_dim; i++)
    {
        for (int j = i; j < Pk_dim; j++)
        {
            nabla_P_vec(idx_count, 0) = matrix_temp(i, j);
            idx_count = idx_count + 1;
        }
    }
    nabla_P_result.push_back(nabla_P_vec);
    matrix_temp = first_term_1 + second_term_1;
    idx_count = 0;
    for (int i = 0; i < Pk_dim; i++)
    {
        for (int j = i; j < Pk_dim; j++)
        {
            nabla_P_vec(idx_count, 0) = matrix_temp(i, j);
            idx_count = idx_count + 1;
        }
    }
    nabla_P_result.push_back(nabla_P_vec);
    return nabla_P_result;
}

/*************************************************
 * 计算P递推关系中F部分对p_ij的偏导数
 *  输入：
 *      协方差矩阵；
 *      矩阵W = H^T(HPH^T+R)^{-1}H；
 *      p_ij的索引；
 *  输出：
 *      F部分对p_ij的偏导数（矩阵）；
 *************************************************/
Eigen::MatrixXd calculate_partial_F(Eigen::MatrixXd P, Eigen::MatrixXd W, int i, int j)
{
    Eigen::MatrixXd partial_F_result;
    Eigen::MatrixXd partial_P = Eigen::MatrixXd::Zero(P.rows(), P.cols());
    partial_P(i, j) = 1.0;
    partial_F_result = partial_P * W * P + P * W * partial_P - P * W * partial_P * W * P;
    return partial_F_result;
}

/*************************************************
 * 计算方差F部分递推A矩阵
 *  输入：
 *      协方差矩阵；
 *      测量矩阵；
 *      测量协方差； 
 *  输出：
 *      F部分递推关系矩阵；
 *************************************************/
Eigen::MatrixXd calculate_A_F(Eigen::MatrixXd P, Eigen::MatrixXd H, Eigen::MatrixXd R)
{
    Eigen::MatrixXd A_F_result;
    int dim = P.rows();
    int P_states_dim = (dim + 1) * dim / 2;
    A_F_result = Eigen::MatrixXd::Zero(P_states_dim, P_states_dim);
    Eigen::MatrixXd W = H.transpose() * (H * P * H.transpose() + R).inverse() * H;
    Eigen::MatrixXd first_term, second_term;
    first_term = W * P;
    second_term = P * W;
    Eigen::MatrixXd Pij_temp_0, Pij_temp_1, Pij_temp_2;
    Eigen::MatrixXd partial_F_pij;
    for (int i = 0; i < dim; i++)
    {
        for (int j = 0; j < dim; j++)
        {
            Pij_temp_0 = Eigen::MatrixXd::Zero(dim, dim);
            Pij_temp_1 = Eigen::MatrixXd::Zero(dim, dim);
            Pij_temp_2 = Eigen::MatrixXd::Zero(dim, dim);
            Pij_temp_0.block(i, 0, 1, dim) = first_term.block(j, 0, 1, dim);
            Pij_temp_1.block(0, j, dim, 1) = second_term.block(0, i, dim, 1);
            Pij_temp_2 = second_term.block(0, i, dim, 1) * first_term.block(j, 0, 1, dim);
            partial_F_pij = Pij_temp_0 + Pij_temp_1 + Pij_temp_2;

            int element_idx = 0;
            for (int m = 0; m < dim; m++)
            {
                for (int n = m; n < dim; n++)
                {
                    if (i <= j)
                    {
                        A_F_result(element_idx, (dim+(dim-(i-1)))*i/2 +j-i) = partial_F_pij(m, n);
                    }
                    else
                    {
                        A_F_result(element_idx, (dim+(dim-(j-1)))*j/2 +i-j) = partial_F_pij(m, n);
                    }
                    element_idx = element_idx + 1;
                }
            }
        }
    }       
    return A_F_result;
}

/*************************************************
 * 计算标称轨迹下的Q
 *  输入：
 *      本时刻L矩阵及其偏导数；
 *      下时刻V矩阵及其偏导数；
 *      系统矩阵A_t、B_t；
 *  输出：
 *      本时刻Q矩阵及其偏导数；
 *************************************************/
std::vector<Eigen::MatrixXd> calculate_Qs(Eigen::MatrixXd L, Eigen::MatrixXd Lu, 
                                    Eigen::MatrixXd Luu, Eigen::MatrixXd Lx, Eigen::MatrixXd Lxx,
                                    Eigen::MatrixXd V, Eigen::MatrixXd Vx,
                                    Eigen::MatrixXd Vxx, Eigen::MatrixXd A, Eigen::MatrixXd B)
{
    std::vector<Eigen::MatrixXd> Qs_result;
    Eigen::MatrixXd Q = V + L;
    Qs_result.push_back(Q);
    Eigen::MatrixXd Qx = Lx + A.transpose() * Vx;
    Qs_result.push_back(Qx);
    Eigen::MatrixXd Qu = Lu + B.transpose() * Vx;
    Qs_result.push_back(Qu);
    Eigen::MatrixXd Qxx = Lxx + A.transpose() * Vxx * A;
    Qs_result.push_back(Qxx);
    Eigen::MatrixXd Qxu = A.transpose() * Vxx * B;
    Qs_result.push_back(Qxu);
    Eigen::MatrixXd Quu = Luu + B.transpose() * Vxx * B;
    Qs_result.push_back(Quu);
    return Qs_result;
}

/*************************************************
 * 前向计算价值函数V 
 *  输入：
 *      本时刻的Q矩阵及其偏导数；
 *  输出：
 *      本时刻的状态价值V及其偏导数；
 *************************************************/
std::vector<Eigen::MatrixXd> calculate_Vs_before(Eigen::MatrixXd Q, Eigen::MatrixXd Qx,
                                                Eigen::MatrixXd Qu, Eigen::MatrixXd Qxx,
                                                Eigen::MatrixXd Qxu, Eigen::MatrixXd Quu)
{
    std::vector<Eigen::MatrixXd> V_before_result;
    Eigen::MatrixXd V = Q - 0.5 * Qu.transpose() * Quu.inverse() * Qu;
    V_before_result.push_back(V);
    Eigen::MatrixXd Vx = Qx - Qxu * Quu.inverse() * Qu;
    V_before_result.push_back(Vx);
    Eigen::MatrixXd Vxx = Qxx - Qxu * Quu.inverse() * Qxu.transpose();
    V_before_result.push_back(Vxx);
    return V_before_result;
}

/*************************************************
 * 计算对偶DDP下标称状态及输入的变化量
 *  输入：
 *      位置变量标称轨迹；
 *      遍历性误差状态轨迹；
 *      估计方差向量轨迹；
 *      估计方差矩阵轨迹；
 *      标称控制输入轨迹；
 *      时间步长；
 *      权重diag矩阵lambda；
 *      傅立叶基函数对x求导轨迹（横x竖基）；
 *      观测矩阵对x求导轨迹（按x拆分列向量）；
 *      控制输入权重系数；
 *      P_mu对P_c的雅可比矩阵；
 *      轨迹观测矩阵；
 *      观测协方差矩阵；
 *  输出：
 *      状态轨迹变化量；
 *      控制输入变化量；
 *************************************************/
std::vector<std::vector<Eigen::MatrixXd>> calculate_dualDDP_delta(std::vector<Eigen::MatrixXd> x_t, 
                                            std::vector<Eigen::MatrixXd> S_k,
                                            std::vector<Eigen::MatrixXd> P_v,
                                            std::vector<Eigen::MatrixXd> P_v_cube,
                                            std::vector<Eigen::MatrixXd> u_t,
                                            double delta_t, Eigen::MatrixXd lam_diag, 
                                            std::vector<Eigen::MatrixXd> nabla_fk,
                                            std::vector<std::vector<Eigen::MatrixXd>> nabla_H,
                                            double w, Eigen::MatrixXd mu_v_Jacc, 
                                            std::vector<Eigen::MatrixXd> H, Eigen::MatrixXd R)
{
    std::vector<std::vector<Eigen::MatrixXd>> delta_result;
    
    /* 相关参数 */
    int traj_length = x_t.size();
    int S_k_dim = S_k[0].rows();
    int x_t_dim = x_t[0].rows();
    int ergodic_state_dim = x_t_dim + S_k_dim;
    int variance_state_dim = P_v[0].rows();
    int state_dim = ergodic_state_dim + variance_state_dim;

    /* 两部分最后时刻状态价值 */
    /* ---1、遍历性指标 */
    Eigen::MatrixXd ergodic_total_x = Eigen::MatrixXd::Zero(ergodic_state_dim, 1);
    ergodic_total_x.block(0, 0, x_t_dim, 1) = x_t[traj_length-1];
    ergodic_total_x.block(x_t_dim, 0, S_k_dim, 1) = S_k[traj_length-1];
    Eigen::MatrixXd ergodic_V, ergodic_V_x, ergodic_V_xx;
    Eigen::MatrixXd param_Q = Eigen::MatrixXd::Zero(ergodic_state_dim, ergodic_state_dim);
    param_Q.block(x_t_dim, x_t_dim, S_k_dim, S_k_dim) = lam_diag;
    std::vector<Eigen::MatrixXd> ergodic_Vs = get_Vs_Q(ergodic_total_x, param_Q);
    ergodic_V = ergodic_Vs[0];
    ergodic_V_x = ergodic_Vs[1];
    ergodic_V_xx = ergodic_Vs[2];
    /* ---2、估计方差指标 */
    Eigen::MatrixXd variance_V, variance_V_x, variance_V_xx;
    Eigen::MatrixXd mu_v_param = mu_v_Jacc.transpose() * lam_diag * mu_v_Jacc;
    Eigen::MatrixXd mu_v_param_vec = Eigen::MatrixXd::Zero(variance_state_dim, 1);
    int count_temp = 0;
    for (int i = 0; i < mu_v_param.rows(); i++)
    {
        for (int j = i; j < mu_v_param.cols(); j++)
        {
            mu_v_param_vec(count_temp, 0) = mu_v_param(i, j);
            if (i != j)
            {
                mu_v_param_vec(count_temp, 0) = mu_v_param_vec(count_temp, 0) + mu_v_param(j, i);
            }
            count_temp = count_temp + 1;
        }
    }
    std::vector<Eigen::MatrixXd> variance_Vs = get_Vs_L(P_v[traj_length-1], mu_v_param_vec);
    variance_V = variance_Vs[0];
    variance_V_x = variance_Vs[1];
    variance_V_xx = variance_Vs[2];

    /* 价值函数及其导数计算 */
    std::vector<Eigen::MatrixXd> V(traj_length), V_x(traj_length), V_xx(traj_length);
    std::vector<Eigen::MatrixXd> Q(traj_length), Q_x(traj_length), Q_u(traj_length), 
                                 Q_xx(traj_length), Q_xu(traj_length), Q_uu(traj_length);
    std::vector<Eigen::MatrixXd> As(traj_length, Eigen::MatrixXd::Identity(state_dim, state_dim)), 
                                 Bs(traj_length, Eigen::MatrixXd::Zero(state_dim, x_t_dim));
    /* ---1.最后时刻的价值 */
    V[traj_length-1] = Eigen::MatrixXd::Zero(1, 1);
    V[traj_length-1] = ergodic_V + variance_V;
    V_x[traj_length-1] = Eigen::MatrixXd::Zero(state_dim, 1);
    V_x[traj_length-1].block(0, 0, ergodic_state_dim, 1) = ergodic_V_x;
    V_x[traj_length-1].block(ergodic_state_dim, 0, variance_state_dim, 1) = variance_V_x;
    V_xx[traj_length-1] = Eigen::MatrixXd::Zero(state_dim, state_dim);
    V_xx[traj_length-1].block(0, 0, ergodic_state_dim, ergodic_state_dim) = ergodic_V_xx;
    V_xx[traj_length-1].block(ergodic_state_dim, ergodic_state_dim, 
                              variance_state_dim, variance_state_dim) = variance_V_xx;
    /* ---2.前向递推计算价值函数 */
    for (int i = 0; i < traj_length - 1; i++)   // 当前索引：traj_length-2-i
    {
        Eigen::MatrixXd A_t = Eigen::MatrixXd::Identity(state_dim, state_dim);
        A_t.block(x_t_dim, 0, S_k_dim, x_t_dim) = delta_t * nabla_fk[traj_length-2-i];
        std::vector<Eigen::MatrixXd> F_x_temp = calculate_nabla_P(P_v_cube[traj_length-2-i], 
                                                                  H[traj_length-1-i], R, 
                                                                  nabla_H[traj_length-2-i]);
        A_t.block(ergodic_state_dim, 0, variance_state_dim, 1) = F_x_temp[0];
        A_t.block(ergodic_state_dim, 1, variance_state_dim, 1) = F_x_temp[1];
        Eigen::MatrixXd A_F = calculate_A_F(P_v_cube[traj_length-2-i], H[traj_length-1-i], R);
        A_t.block(ergodic_state_dim, ergodic_state_dim, variance_state_dim, variance_state_dim)
            = Eigen::MatrixXd::Identity(variance_state_dim, variance_state_dim) - A_F;
        As[traj_length-2-i] = A_t;
        Eigen::MatrixXd B_t = Eigen::MatrixXd::Zero(state_dim, x_t_dim);
        B_t.block(0, 0, x_t_dim, x_t_dim) = Eigen::MatrixXd::Identity(x_t_dim, x_t_dim) 
                                            * delta_t;
        B_t.block(ergodic_state_dim, 0, variance_state_dim, 1) = F_x_temp[0] * delta_t;
        B_t.block(ergodic_state_dim, 1, variance_state_dim, 1) = F_x_temp[1] * delta_t;
        Bs[traj_length-2-i] = B_t;
        Eigen::MatrixXd L(1, 1), L_u(x_t_dim, 1), L_uu(x_t_dim, x_t_dim);
        L = w * u_t[traj_length-2-i].transpose() * u_t[traj_length-2-i];
        L_u = 2.0 * w * u_t[traj_length-2-i];
        L_uu = 2.0 * w * Eigen::MatrixXd::Identity(x_t_dim, x_t_dim);
        
        /* 改：持续性指标 */
        ergodic_total_x.block(0, 0, x_t_dim, 1) = x_t[traj_length-2-i];
        ergodic_total_x.block(x_t_dim, 0, S_k_dim, 1) = S_k[traj_length-2-i];
        std::vector<Eigen::MatrixXd> Ergodic_Ls = get_Vs_Q(ergodic_total_x, param_Q);
        std::vector<Eigen::MatrixXd> Variance_Ls = get_Vs_L(P_v[traj_length-2-i], mu_v_param_vec);
        L = L + Ergodic_Ls[0] + Variance_Ls[0];
        Eigen::MatrixXd L_x = Eigen::MatrixXd::Zero(state_dim, 1);
        L_x.block(0, 0, ergodic_state_dim, 1) = Ergodic_Ls[1];
        L_x.block(ergodic_state_dim, 0, variance_state_dim, 1) = Variance_Ls[1];
        Eigen::MatrixXd L_xx = Eigen::MatrixXd::Zero(state_dim, state_dim);
        L_xx.block(0, 0, ergodic_state_dim, ergodic_state_dim) = Ergodic_Ls[2];
        L_xx.block(ergodic_state_dim, ergodic_state_dim, 
                              variance_state_dim, variance_state_dim) = Variance_Ls[2];
        

        std::vector<Eigen::MatrixXd> Qs = calculate_Qs(L, L_u, L_uu, L_x, L_xx, 
                    V[traj_length-1-i], V_x[traj_length-1-i], V_xx[traj_length-1-i], A_t, B_t);
        Q[traj_length-2-i] = Qs[0];
        Q_x[traj_length-2-i] = Qs[1];
        Q_u[traj_length-2-i] = Qs[2];
        Q_xx[traj_length-2-i] = Qs[3];
        Q_xu[traj_length-2-i] = Qs[4];
        Q_uu[traj_length-2-i] = Qs[5];
        std::vector<Eigen::MatrixXd> Vs_before = calculate_Vs_before(Q[traj_length-2-i],
                                                                     Q_x[traj_length-2-i],
                                                                     Q_u[traj_length-2-i],
                                                                     Q_xx[traj_length-2-i],
                                                                     Q_xu[traj_length-2-i],
                                                                     Q_uu[traj_length-2-i]);
        V[traj_length-2-i] = Vs_before[0];
        V_x[traj_length-2-i] = Vs_before[1];
        V_xx[traj_length-2-i] = Vs_before[2];
    }
    /* ---3.后向递推计算改变量 */
    std::vector<Eigen::MatrixXd> delta_xs(traj_length), delta_us(traj_length);
    delta_xs[0] = Eigen::MatrixXd::Zero(state_dim, 1);
    for (int i = 0; i < traj_length - 1; i++)
    {
        delta_us[i] = -Q_uu[i].inverse() * (Q_u[i] + Q_xu[i].transpose() * delta_xs[i]);
        delta_xs[i+1] = As[i] * delta_xs[i] + Bs[i] * delta_us[i];
    }
    delta_result.push_back(delta_xs);
    delta_result.push_back(delta_us);
    return delta_result;
}

/*************************************************
 * 计算目标分布的傅立叶分解系数真值
 *  输入：
 *      单轴基函数数量；
 *      基函数值分布(先位置后索引)；
 *      基函数归一化倒系数矩阵；
 *      目标分布；
 *      空间步长；
 *  输出：
 *      傅立叶系数矩阵；
 *************************************************/
Eigen::MatrixXd calculate_mu_k(int K, std::vector<std::vector<Eigen::MatrixXd>> basis, 
                               Eigen::MatrixXd h_k_cinv, std::vector<std::vector<double>> mu,
                               double delta_x)
{
    Eigen::MatrixXd mu_k_result(K, K);
    for (int i = 0; i < basis.size(); i++)
    {
        for (int j = 0; j < basis[0].size(); j++)
        {
            mu_k_result = mu_k_result + basis[i][j] * mu[i][j] * delta_x * delta_x;
        }
    }
    mu_k_result = mu_k_result.cwiseProduct(h_k_cinv);
    return mu_k_result;
}

/*************************************************
 * 计算基函数值矩阵分布
 *  输入：
 *      分布场中点的个数；
 *      基函数单轴个数；
 *      空间步长；
 *      区域范围；
 *  输出：
 *      基函数分布(先位置后索引)；
 *************************************************/
std::vector<std::vector<Eigen::MatrixXd>> calculate_basis(int x_size, int K, double delta_x,
                                                          double L)
{
    std::vector<std::vector<Eigen::MatrixXd>> basis_result;
    for (int i = 0; i < x_size; i++)
    {
        std::vector<Eigen::MatrixXd> line_temp;
        for (int j = 0; j < x_size; j++)
        {
            Eigen::MatrixXd value_temp(K, K);
            for (int k1 = 0; k1 < K; k1++)
            {
                for (int k2 = 0; k2 < K; k2++)
                {
                    value_temp(k1, k2) = cos(k1 * M_PI / L * i * delta_x) 
                                         * cos(k2 * M_PI / L * j * delta_x);
                }
            }
            line_temp.push_back(value_temp);
        }
        basis_result.push_back(line_temp);
    }
    return basis_result;
}

/*************************************************
 * 计算傅立叶基函数归一化系数矩阵
 *  输入：
 *      基函数分布(先位置后索引)；
 *      基函数单轴个数；
 *      空间步长；
 *  输出：
 *      归一化系数矩阵；
 *************************************************/
Eigen::MatrixXd calculate_h_k(std::vector<std::vector<Eigen::MatrixXd>> basis, int K, 
                              double delta_x)
{
    Eigen::MatrixXd h_k = Eigen::MatrixXd::Zero(K, K);
    for (int i = 0; i < basis.size(); i++)
    {
        for (int j = 0; j < basis[0].size(); j++)
        {
            h_k = h_k + basis[i][j].cwiseProduct(basis[i][j]) * delta_x * delta_x;
        }
    }
    h_k = h_k.cwiseSqrt();
    return h_k;
}

/*************************************************
 * 计算x处傅立叶基函数值矩阵
 *  输入：
 *      矩阵的维数；
 *      当前位置；
 *      归一化系数倒数矩阵；
 *      区域尺寸；
 *  输出：
 *      x处傅立叶基函数值矩阵；
 *************************************************/
Eigen::MatrixXd calculate_fk_at_x(int K, Eigen::MatrixXd x, Eigen::MatrixXd h_k_cinv, double L)
{
    Eigen::MatrixXd fk_result(K, K);
    for (int i = 0; i < K; i++)
    {
        for (int j = 0; j < K; j++)
        {
            fk_result(i, j) = h_k_cinv(i, j) * cos(i * M_PI / L * x(0, 0))
                              * cos(j * M_PI / L * x(1, 0));
        }
    }
    return fk_result;
}

/*************************************************
 * 计算x处傅立叶基函梯度
 *  输入：
 *      矩阵的维数；
 *      当前位置；
 *      归一化系数倒数矩阵；
 *      区域尺寸；
 *  输出：
 *      x处傅立叶基函数梯度矩阵（按x拆分）；
 *************************************************/
std::vector<Eigen::MatrixXd> calculate_nabla_fk_at_x(int K, Eigen::MatrixXd x, 
                                                     Eigen::MatrixXd h_k_cinv, double L)
{
    std::vector<Eigen::MatrixXd> nabla_fk_result(x.rows());
    for (int i = 0; i < nabla_fk_result.size(); i++)
    {
        nabla_fk_result[i] = Eigen::MatrixXd::Zero(K, K);
    }
    for (int i = 0; i < K; i++)
    {
        for (int j = 0; j < K; j++)
        {
            nabla_fk_result[0](i, j) = -i * M_PI / L * h_k_cinv(i, j) 
                                        * sin(i * M_PI / L * x(0, 0)) 
                                        * cos(j * M_PI / L * x(1, 0));
            nabla_fk_result[1](i, j) = -j * M_PI / L * h_k_cinv(i, j) 
                                        * cos(i * M_PI / L * x(0, 0)) 
                                        * sin(j * M_PI / L * x(1, 0));
        }
    }
    return nabla_fk_result;
}

/*************************************************
 * 轨迹分布系数递推计算
 *  输入：
 *      前一时刻轨迹分布系数矩阵；
 *      系数矩阵的维数；
 *      当前位置；
 *      时间步长；
 *      归一化系数倒数矩阵；
 *      区域尺寸
 *  输出：
 *      轨迹分布系数矩阵；
 *************************************************/
Eigen::MatrixXd calculate_c_k(Eigen::MatrixXd ck_before, int K, Eigen::MatrixXd x_t, 
                              double delta_t, Eigen::MatrixXd hk_cinv, double L)
{
    Eigen::MatrixXd ck_result;
    Eigen::MatrixXd delta_ck = calculate_fk_at_x(K, x_t, hk_cinv, L);
    ck_result = ck_before + delta_ck * delta_t;
    return ck_result;
}

/*************************************************
 * 计算索伯维尔空间范数权重矩阵
 *  输入：
 *      基函数维数；
 *  输出：
 *      权重矩阵；
 *************************************************/
Eigen::MatrixXd calculate_Lambda(int K)
{
    Eigen::MatrixXd lambda_result(K, K);
    for (int i = 0; i < K; i++)
    {
        for (int j = 0; j < K; j++)
        {
            lambda_result(i, j) = 1.0 / pow(1.0 + i*i + j*j, 1.5);
        }
    }
    return lambda_result;
}

/*************************************************
 * 对偶微分动态规划循环一次
 *  输入：
 *      DDP优化计算次数；
 *      一段轨迹的长度；
 *      机器人当前位置；
 *      轨迹分布系数矩阵；
 *      目标分布系数矩阵；
 *      当前时间；
 *      时间步长；
 *      遍历性误差基函数个数；
 *      归一化系数倒数矩阵；
 *      区域范围；
 *      索伯维尔权重diag矩阵；
 *      当前估计方差矩阵；
 *      直接估计值基函数个数；
 *      测量噪声R；
 *      分布值mu对估计值v的雅可比矩阵；
 *  输出：
 *      控制输入序列；
 *************************************************/
std::vector<Eigen::MatrixXd> Dual_DDP_looponce(int opt_times, int traj_length, 
                                               Eigen::MatrixXd x_t, Eigen::MatrixXd c_k_t, 
                                               Eigen::MatrixXd mu_k, double now_t, 
                                               double delta_t, int K, Eigen::MatrixXd hk_cinv,
                                               double L, Eigen::MatrixXd lam_diag, 
                                               Eigen::MatrixXd Pv_t, int K_v, Eigen::MatrixXd R,
                                               Eigen::MatrixXd mu_v_Jacc)
{
    std::vector<Eigen::MatrixXd> input_result;

    /* 初始化标称轨迹 */
    std::vector<Eigen::MatrixXd> x_traj(traj_length), u_traj(traj_length), Sk_traj(traj_length), 
                                 Pv_cube_traj(traj_length), Pv_vec_traj(traj_length),
                                 nabla_fk_traj(traj_length), H_traj(traj_length);
    std::vector<std::vector<Eigen::MatrixXd>> nabla_H_traj(traj_length);
    x_traj[0] = x_t;
    Sk_traj[0] = c_k_t - mu_k * now_t;
    Sk_traj[0].resize(K*K, 1);
    Pv_cube_traj[0] = Pv_t;
    Pv_vec_traj[0] = Eigen::MatrixXd::Zero((K_v*K_v+1)*K_v*K_v/2, 1);
    int element_idx = 0;
    for (int i = 0; i < K_v*K_v; i++)
    {
        for (int j = i; j < K_v * K_v; j++)
        {
            Pv_vec_traj[0](element_idx, 0) = Pv_cube_traj[0](i, j);
            element_idx = element_idx + 1;
        }
    }
    int x_dim = x_t.rows();
    u_traj[0] = Eigen::MatrixXd::Zero(x_dim, 1);
    Eigen::MatrixXd c_k_temp = c_k_t;
    double time_temp = now_t;
    std::vector<Eigen::MatrixXd> nabla_fk_temp = calculate_nabla_fk_at_x(K, x_t, hk_cinv, L);
    Eigen::MatrixXd vec1_temp, vec2_temp;
    nabla_fk_traj[0] = Eigen::MatrixXd::Zero(K*K, 2);
    vec1_temp = nabla_fk_temp[0];
    vec1_temp.resize(K*K, 1);
    nabla_fk_traj[0].block(0, 0, K*K, 1) = vec1_temp;
    vec2_temp = nabla_fk_temp[1];
    vec2_temp.resize(K*K, 1);
    nabla_fk_traj[0].block(0, 1, K*K, 1) = vec2_temp;
    nabla_H_traj[0].push_back(nabla_fk_temp[0].block(0, 0, K_v, K_v));
    nabla_H_traj[0][0].resize(1, K_v*K_v);
    nabla_H_traj[0].push_back(nabla_fk_temp[1].block(0, 0, K_v, K_v));
    nabla_H_traj[0][1].resize(1, K_v*K_v);
    H_traj[0] = calculate_fk_at_x(K_v, x_t, hk_cinv, L);
    H_traj[0].resize(1, K_v*K_v);
    for (int i = 1; i < traj_length; i++)
    {
        x_traj[i] = x_t;
        c_k_temp = calculate_c_k(c_k_temp, K, x_traj[i], delta_t, hk_cinv, L);
        time_temp = time_temp + delta_t;
        Sk_traj[i] = c_k_temp - mu_k * time_temp;
        Sk_traj[i].resize(K*K, 1);
        H_traj[i] = calculate_fk_at_x(K_v, x_traj[i], hk_cinv, L);
        H_traj[i].resize(1, K_v*K_v);
        Pv_cube_traj[i] = calculate_next_P(Pv_cube_traj[i-1], H_traj[i], R);
        Pv_vec_traj[i] = Eigen::MatrixXd::Zero((K_v*K_v+1)*K_v*K_v/2, 1);
        element_idx = 0;
        for (int m = 0; m < K_v*K_v; m++)
        {
            for (int n = m; n < K_v * K_v; n++)
            {
                Pv_vec_traj[i](element_idx, 0) = Pv_cube_traj[i](m, n);
                element_idx = element_idx + 1;
            }
        }
        u_traj[i] = Eigen::MatrixXd::Zero(x_dim, 1);
        nabla_fk_temp = calculate_nabla_fk_at_x(K, x_traj[i], hk_cinv, L);
        nabla_fk_traj[i] = Eigen::MatrixXd::Zero(K*K, 2);
        vec1_temp = nabla_fk_temp[0];
        vec1_temp.resize(K*K, 1);
        nabla_fk_traj[i].block(0, 0, K*K, 1) = vec1_temp;
        vec2_temp = nabla_fk_temp[1];
        vec2_temp.resize(K*K, 1);
        nabla_fk_traj[i].block(0, 1, K*K, 1) = vec2_temp;
        nabla_H_traj[i].push_back(nabla_fk_temp[0].block(0, 0, K_v, K_v));
        nabla_H_traj[i][0].resize(1, K_v*K_v);
        nabla_H_traj[i].push_back(nabla_fk_temp[1].block(0, 0, K_v, K_v));
        nabla_H_traj[i][1].resize(1, K_v*K_v);
    }

    /* 迭代优化 */
    for (int count = 0; count < opt_times; count++)
    {
        /* 计算标称轨迹变化量 */
        std::vector<std::vector<Eigen::MatrixXd>> DDP_delta 
            = calculate_dualDDP_delta(x_traj, Sk_traj, Pv_vec_traj, Pv_cube_traj, 
                                      u_traj, delta_t, lam_diag, nabla_fk_traj, nabla_H_traj, 4.0, 
                                      mu_v_Jacc, H_traj, R);

        /* 向后递推新的标称状态 */
        std::vector<Eigen::MatrixXd> delta_u_traj = DDP_delta[1];
        time_temp = now_t;
        c_k_temp = c_k_t;
        for (int i = 0; i < traj_length - 1; i++)
        {
            u_traj[i] = u_traj[i] + delta_u_traj[i];
            x_traj[i+1] = x_traj[i] + u_traj[i] * delta_t;
            c_k_temp = calculate_c_k(c_k_temp, K, x_traj[i+1], delta_t, hk_cinv, L);
            time_temp = time_temp + delta_t;
            Sk_traj[i+1] = c_k_temp - mu_k * time_temp; 
            Sk_traj[i+1].resize(K*K, 1);
            H_traj[i+1] = calculate_fk_at_x(K_v, x_traj[i+1], hk_cinv, L);
            H_traj[i+1].resize(1, K_v*K_v);
            Pv_cube_traj[i+1] = calculate_next_P(Pv_cube_traj[i], H_traj[i+1], R);
            Pv_vec_traj[i+1] = Eigen::MatrixXd::Zero((K_v*K_v+1)*K_v*K_v/2, 1);
            element_idx = 0;
            for (int m = 0; m < K_v*K_v; m++)
            {
                for (int n = m; n < K_v * K_v; n++)
                {
                    Pv_vec_traj[i+1](element_idx, 0) = Pv_cube_traj[i+1](m, n);
                    element_idx = element_idx + 1;
                }
            }
            nabla_fk_temp = calculate_nabla_fk_at_x(K, x_traj[i+1], hk_cinv, L);
            vec1_temp = nabla_fk_temp[0];
            vec1_temp.resize(K*K, 1);
            nabla_fk_traj[i+1].block(0, 0, K*K, 1) = vec1_temp;
            vec2_temp = nabla_fk_temp[1];
            vec2_temp.resize(K*K, 1);
            nabla_fk_traj[i+1].block(0, 1, K*K, 1) = vec2_temp;
            nabla_H_traj[i+1].push_back(nabla_fk_temp[0].block(0, 0, K_v, K_v));
            nabla_H_traj[i+1][0].resize(1, K_v*K_v);
            nabla_H_traj[i+1].push_back(nabla_fk_temp[1].block(0, 0, K_v, K_v));
            nabla_H_traj[i+1][1].resize(1, K_v*K_v);
        }
    }
    input_result = u_traj;
    return input_result;
}

/*************************************************
 * 计算由v_k得mu_k
 *  输入：
 *      v_k矩阵；
 *      v_k的维数；
 *      mu_k的归一化系数矩阵；
 *  输出：
 *      mu_k矩阵；
 *************************************************/
Eigen::MatrixXd calculate_mu_from_v(Eigen::MatrixXd vk, int Kv, Eigen::MatrixXd hk)
{
    Eigen::MatrixXd mu_result = Eigen::MatrixXd::Zero(2*Kv-1, 2*Kv-1);
    for (int i = 0; i < Kv; i++)
    {
        for (int j = 0; j < Kv; j++)
        {
            for (int k = 0; k < Kv; k++)
            {
                for (int l = 0; l < Kv; l++)
                {
                    double vv = vk(i, j) * vk(k, l) * 0.25 / hk(i, j) / hk(k, l);
                    mu_result(i+k, j+l) = mu_result(i+k, j+l) + vv * hk(i+k, j+l);
                    mu_result(abs(i-k), j+l) = mu_result(abs(i-k), j+l) + vv * hk(abs(i-k), j+l);
                    mu_result(i+k, abs(j-l)) = mu_result(i+k, abs(j-l)) + vv * hk(i+k, abs(j-l));
                    mu_result(abs(i-k), abs(j-l)) = mu_result(abs(i-k), abs(j-l))
                                                    + vv * hk(abs(i-k), abs(j-l));
                }
            }
        }
    }
    return mu_result;
}

/*************************************************
 * 计算mu对v的雅可比矩阵
 *  输入：
 *      v矩阵；
 *      v矩阵的维数；
 *      归一化系数矩阵；
 *  输出：
 *      mu对v的雅可比矩阵；
 *************************************************/
Eigen::MatrixXd calculate_mu_v_Jacc(Eigen::MatrixXd v_k, int K_v, Eigen::MatrixXd h_k)
{
    Eigen::MatrixXd Jacc_result = Eigen::MatrixXd::Zero((2*K_v-1)*(2*K_v-1), K_v*K_v);
    for (int i = 0; i < 2*K_v-1; i++)
    {
        for (int j = 0; j < 2*K_v-1; j++)
        {
            for (int k = 0; k < K_v; k++)
            {
                for (int l = 0; l < K_v; l++)
                {
                    if (i-k >=0 && i-k < K_v)
                    {
                        if (j-l >= 0 && j-l < K_v)
                        {
                            Jacc_result(i+j*(2*K_v-1), k+l*K_v) +=
                                h_k(i, j) * v_k(i-k, j-l) / h_k(k, l) / h_k(i-k, j-l);
                        }
                        if (j+l >= 0 && j+l < K_v)
                        {
                            Jacc_result(i+j*(2*K_v-1), k+l*K_v) +=
                                h_k(i, j) * v_k(i-k, j+l) / h_k(k, l) / h_k(i-k, j+l);
                        }
                        if (l-j >= 0 && l-j < K_v)
                        {
                            Jacc_result(i+j*(2*K_v-1), k+l*K_v) +=
                                h_k(i, j) * v_k(i-k, l-j) / h_k(k, l) / h_k(i-k, l-j);
                        }
                    }
                    if (i+k >=0 && i+k < K_v)
                    {
                        if (j-l >= 0 && j-l < K_v)
                        {
                            Jacc_result(i+j*(2*K_v-1), k+l*K_v) +=
                                h_k(i, j) * v_k(i+k, j-l) / h_k(k, l) / h_k(i+k, j-l);
                        }
                        if (j+l >= 0 && j+l < K_v)
                        {
                            Jacc_result(i+j*(2*K_v-1), k+l*K_v) +=
                                h_k(i, j) * v_k(i+k, j+l) / h_k(k, l) / h_k(i+k, j+l);
                        }
                        if (l-j >= 0 && l-j < K_v)
                        {
                            Jacc_result(i+j*(2*K_v-1), k+l*K_v) +=
                                h_k(i, j) * v_k(i+k, l-j) / h_k(k, l) / h_k(i+k, l-j);
                        }
                    }
                    if (k-i >=0 && k-i < K_v)
                    {
                        if (j-l >= 0 && j-l < K_v)
                        {
                            Jacc_result(i+j*(2*K_v-1), k+l*K_v) +=
                                h_k(i, j) * v_k(k-i, j-l) / h_k(k, l) / h_k(k-i, j-l);
                        }
                        if (j+l >= 0 && j+l < K_v)
                        {
                            Jacc_result(i+j*(2*K_v-1), k+l*K_v) +=
                                h_k(i, j) * v_k(k-i, j+l) / h_k(k, l) / h_k(k-i, j+l);
                        }
                        if (l-j >= 0 && l-j < K_v)
                        {
                            Jacc_result(i+j*(2*K_v-1), k+l*K_v) +=
                                h_k(i, j) * v_k(k-i, l-j) / h_k(k, l) / h_k(k-i, l-j);
                        }
                    }
                }
            }
        }
    } 
    return Jacc_result;
}

/*************************************************
 * Trajectory Insertion
 *  Input:
 *      original trajectory;
 *      number of points to be inserted;
 *  Output:
 *      inserted trajectory;
 *************************************************/
std::vector<Eigen::MatrixXd> insert_traj_points(std::vector<Eigen::MatrixXd> traj, int num)
{
    std::vector<Eigen::MatrixXd> traj_result;
    for (int i = 0; i < traj.size()-1; i++)
    {
        traj_result.push_back(traj[i]);
        double dist_x = traj[i+1](0, 0) - traj[i](0, 0);
        double dist_y = traj[i+1](1, 0) - traj[i](1, 0);
        double step_x = dist_x / (num + 1);
        double step_y = dist_y / (num + 1);
        for (int j = 0; j < num; j++)
        {
            Eigen::MatrixXd pt_temp = Eigen::MatrixXd::Zero(2, 1);
            pt_temp(0, 0) = traj[i](0, 0) + (j+1) * step_x;
            pt_temp(1, 0) = traj[i](1, 0) + (j+1) * step_y;
            traj_result.push_back(pt_temp);
        }
    }
    return traj_result;
}

/* 参数与变量 */
double length_g;
double delta_x_g;
int K_v_g;
int K_g;
int x_size_g;
std::vector<std::vector<Eigen::MatrixXd>> basis_g;
Eigen::MatrixXd h_k_g;
Eigen::MatrixXd h_k_cinv_g;
Eigen::MatrixXd c_k_g;
Eigen::MatrixXd x_t_g;
double delta_t_g;
double now_t_g;
std::vector<std::vector<double>> mu_g;
Eigen::MatrixXd mu_k_g;
Eigen::MatrixXd mu_k_real_g;
Eigen::MatrixXd lambda_g;
Eigen::MatrixXd lam_diag_g;
Eigen::MatrixXd v_k_g;
Eigen::MatrixXd P_v_g;
Eigen::MatrixXd R_g;
std::vector<std::vector<double>> mu_rebuilt_g;
double x_offset_g;
Eigen::MatrixXd mu_v_Jacc_g;
std::vector<Eigen::MatrixXd> ref_traj_g;
int traj_pt_idx_g;

/* ROS相关 */
ros::Publisher mu_g_vis_puber, mu_rebuilt_vis_puber;
sensor_msgs::PointCloud mu_g_vis_msg;
ros::Timer publish_timer, traj_update_timer;
ros::Publisher traj_pts_puber;
geometry_msgs::PoseArray traj_pts_msg;
ros::Publisher cmd_puber;
ros::Subscriber pos_suber;
void pos_subCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    // x_t_g(0, 0) = msg->pose.position.x + x_offset_g;
    // x_t_g(1, 0) = msg->pose.position.y + x_offset_g;
    // if (x_t_g(0, 0) > length_g - 0.1) x_t_g(0, 0) = length_g - 0.1;
    // if (x_t_g(0, 0) < 0.1) x_t_g(0, 0) = 0.1;
    // if (x_t_g(1, 0) > length_g - 0.1) x_t_g(1, 0) = length_g - 0.1;
    // if (x_t_g(1, 0) < 0.1) x_t_g(1, 0) = 0.1; 
}
void publish_timeCallback(const ros::TimerEvent&)
{
    if (traj_pt_idx_g == -1)
    {
        return;
    }
    else
    {
        traj_utils::Flag cmd_msg;
        for (int i = 0; i < 9; i++)
        {
            if (traj_pt_idx_g+i+1 < ref_traj_g.size())
            {
                cmd_msg.cmd[i].position.x = ref_traj_g[traj_pt_idx_g+i](0, 0) - x_offset_g;
                cmd_msg.cmd[i].position.y = ref_traj_g[traj_pt_idx_g+i](1, 0) - x_offset_g;
                cmd_msg.cmd[i].velocity.x = (ref_traj_g[traj_pt_idx_g+i+1](0, 0) 
                                            - ref_traj_g[traj_pt_idx_g+i](0, 0)) / 0.05;
                cmd_msg.cmd[i].velocity.y = (ref_traj_g[traj_pt_idx_g+i+1](1, 0) 
                                            - ref_traj_g[traj_pt_idx_g+i](1, 0)) / 0.05;
            }
            else
            {
                cmd_msg.cmd[i].position.x = ref_traj_g[ref_traj_g.size()-1](0, 0) - x_offset_g;
                cmd_msg.cmd[i].position.y = ref_traj_g[ref_traj_g.size()-1](1, 0) - x_offset_g;
                cmd_msg.cmd[i].velocity.x = 0.0;
                cmd_msg.cmd[i].velocity.y = 0.0;
            }
        }
        traj_pt_idx_g = traj_pt_idx_g + 1;
        cmd_puber.publish(cmd_msg);
    }
}
void update_timeCallback(const ros::TimerEvent&)
{
    now_t_g = now_t_g + delta_t_g;

    /* 目标分布的估计 */
    Eigen::MatrixXd z_t(1, 1);
    z_t(0, 0) = sqrt(mu_g[floor(x_t_g(0, 0)/delta_x_g)][floor(x_t_g(1, 0)/delta_x_g)]);
    Eigen::MatrixXd v_vec = v_k_g;
    v_vec.resize(K_v_g*K_v_g, 1);
    Eigen::MatrixXd H_t = calculate_fk_at_x(K_v_g, x_t_g, h_k_cinv_g, length_g);
    H_t.resize(1, K_v_g*K_v_g);
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> post_result = NCKF(v_vec, P_v_g, z_t, H_t, R_g, 1.0);
    v_vec = post_result.first;
    v_vec.resize(K_v_g, K_v_g);
    v_k_g = v_vec;
    P_v_g = post_result.second;
    mu_k_g = calculate_mu_from_v(v_k_g, K_v_g, h_k_g);
    mu_v_Jacc_g = calculate_mu_v_Jacc(v_k_g, K_v_g, h_k_g);

    /* 目标分布可视化 */
    mu_g_vis_puber.publish(mu_g_vis_msg);
    for (int i = 0; i < x_size_g; i++)
    {
        for (int j = 0; j < x_size_g; j++)
        {
            mu_rebuilt_g[i][j] = 
                mu_k_g.cwiseProduct(h_k_cinv_g.cwiseProduct(basis_g[i][j])).sum();
        }
    }
    sensor_msgs::PointCloud mu_rebuilt_vis_msg;
    mu_rebuilt_vis_msg = visualize_density(mu_rebuilt_g, delta_x_g, 3.0);
    mu_rebuilt_vis_msg.header.frame_id = "world";
    mu_rebuilt_vis_puber.publish(mu_rebuilt_vis_msg);   

    /* 估计误差输出 */
    // double estimation_error = ( (mu_k_g - mu_k_real_g).cwiseProduct((mu_k_g - mu_k_real_g)) ).sum();
    // std::ofstream est_ofs;
    // est_ofs.open("est_error_1.txt", std::ios::out|std::ios::app);
    // est_ofs << estimation_error << std::endl;
    // est_ofs.close(); 

    /* 计算当前轨迹分布系数 */
    c_k_g = calculate_c_k(c_k_g, K_g, x_t_g, delta_t_g, h_k_cinv_g, length_g);
    // double ergodic_error = lambda_g.cwiseProduct(
    //                         (c_k_g - mu_k_real_g * now_t_g).cwiseProduct(c_k_g - mu_k_real_g * now_t_g)
    //                         ).sum();
    // std::ofstream erg_ofs;
    // erg_ofs.open("erg_error_1.txt", std::ios::out|std::ios::app);
    // erg_ofs << ergodic_error << std::endl;
    // erg_ofs.close(); 

    /* 微分动态规划 */
    ros::Time t1, t2;
    t1 = ros::Time::now();
    int traj_seg_length = 5;
    std::vector<Eigen::MatrixXd> us_t = Dual_DDP_looponce(3, traj_seg_length, x_t_g, c_k_g, mu_k_g, now_t_g,
                                                          delta_t_g, K_g, h_k_cinv_g, length_g, 
                                                          lam_diag_g, P_v_g, K_v_g, R_g, mu_v_Jacc_g);
    t2 = ros::Time::now();
    std::cout << "time: " << t2 - t1 << "s" << std::endl;

    /* 计算参考轨迹 */
    Eigen::MatrixXd pos_temp = x_t_g;
    std::vector<Eigen::MatrixXd> traj_pts_temp;
    for (int i = 0; i < traj_seg_length; i++)
    {
        pos_temp = pos_temp + delta_t_g * us_t[i];
        traj_pts_temp.push_back(pos_temp);
    }
    ref_traj_g = insert_traj_points(traj_pts_temp, 5);
    traj_pt_idx_g = 0;

    /* 位置可视化 */
    x_t_g = x_t_g + us_t[0] * delta_t_g;    // 【注意】：实物实验需注掉此行
    if (x_t_g(0, 0) > length_g - 0.1) x_t_g(0, 0) = length_g - 0.1;
    if (x_t_g(0, 0) < 0.1) x_t_g(0, 0) = 0.1;
    if (x_t_g(1, 0) > length_g - 0.1) x_t_g(1, 0) = length_g - 0.1;
    if (x_t_g(1, 0) < 0.1) x_t_g(1, 0) = 0.1; 
    geometry_msgs::Pose pt_temp;
    pt_temp.position.x = x_t_g(0, 0);
    pt_temp.position.y = x_t_g(1, 0);
    pt_temp.position.z = 3.0;
    traj_pts_msg.poses.push_back(pt_temp);
    traj_pts_puber.publish(traj_pts_msg);
}

double calculate_single_gaussian(int i, int j, double x, double y, double sigma)
{
    double value_result;
    value_result = 1.0 / 2.0 / sigma / sigma / M_PI *
                   exp( 0.5 * ( -pow(i*0.1 - x, 2) 
                   - pow(j*0.1 - y, 2) ) / (sigma * sigma));
    return value_result;
}

int main(int argc, char **argv)
{
    Eigen::initParallel();
    Eigen::setNbThreads(2);

    /* 参数与变量初始化 */
    length_g = 4.0;
    delta_x_g = 0.1;
    K_v_g = 5;
    K_g = K_v_g * 2 - 1;
    x_size_g = floor(length_g / delta_x_g);
    basis_g = calculate_basis(x_size_g, K_g, delta_x_g, length_g);
    h_k_g = calculate_h_k(basis_g, K_g, delta_x_g);
    h_k_cinv_g = h_k_g.cwiseInverse();
    c_k_g = Eigen::MatrixXd::Zero(K_g, K_g);
    x_offset_g = 1.0;
    x_t_g = Eigen::MatrixXd::Zero(2, 1);
    x_t_g(0, 0) = x_offset_g;
    x_t_g(1, 0) = x_offset_g;
    delta_t_g = 0.5;
    now_t_g = 0.0;
    double mu_sum = 0.0;
    for (int i = 0; i < x_size_g; i++)
    {
        std::vector<double> line_temp;
        for (int j = 0; j < x_size_g; j++)
        { 
            double test_value = 0.0;
            test_value = test_value + calculate_single_gaussian(i, j, 1.10, 2.24, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 0.80, 2.21, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 0.67, 1.97, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 0.83, 1.71, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 1.22, 1.73, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 1.33, 2.02, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 1.55, 2.31, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 1.33, 2.54, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 1.23, 2.53, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 0.76, 2.45, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 0.49, 2.32, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 0.45, 2.07, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 0.44, 1.77, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 0.71, 1.42, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 1.06, 1.39, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 1.31, 1.41, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 1.54, 1.67, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 1.60, 1.98, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 2.85, 2.28, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 2.76, 2.00, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 2.90, 1.70, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 3.22, 1.68, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 3.32, 2.03, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 3.13, 2.25, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 3.56, 2.30, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 3.37, 2.44, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 2.92, 2.63, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 2.68, 2.45, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 2.45, 2.28, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 2.37, 2.06, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 2.49, 1.77, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 2.63, 1.53, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 2.98, 1.46, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 3.31, 1.41, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 3.44, 1.68, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 3.57, 1.96, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 1.00, 2.00, 0.3) / 38.0;
            test_value = test_value + calculate_single_gaussian(i, j, 3.00, 2.00, 0.3) / 38.0;
                                                       
            mu_sum = mu_sum + test_value * delta_x_g * delta_x_g;
            line_temp.push_back(test_value);
        }
        mu_g.push_back(line_temp);
    }
    for (int i = 0; i < x_size_g; i++)
    {
        for (int j = 0; j < x_size_g; j++)
        {
            mu_g[i][j] = mu_g[i][j] / mu_sum;
        }
    }
    mu_k_real_g = calculate_mu_k(K_g, basis_g, h_k_cinv_g, mu_g, delta_x_g);
    mu_rebuilt_g = mu_g;
    v_k_g = Eigen::MatrixXd::Zero(K_v_g, K_v_g);
    v_k_g(0, 0) = 1.0;
    P_v_g = 0.4 * Eigen::MatrixXd::Identity(K_v_g*K_v_g, K_v_g*K_v_g);
    R_g = 0.05 * Eigen::MatrixXd::Identity(1, 1);
    mu_k_g = calculate_mu_from_v(v_k_g, K_v_g, h_k_g);
    lam_diag_g = Eigen::MatrixXd::Zero(K_g*K_g, K_g*K_g);
    Eigen::MatrixXd lambda = calculate_Lambda(K_g);
    lambda_g = lambda;
    for (int i = 0; i < K_g; i++)
    {
        for (int j = 0; j < K_g; j++)
        {
            lam_diag_g(j*K_g+i, j*K_g+i) = lambda(i, j);
        }
    }
    traj_pt_idx_g = -1;
    
    /* ROS部分 */
    ros::init(argc, argv, "traj_test_node");
    ros::NodeHandle nh("~");    
    /* ---位姿订阅 */
    pos_suber = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, pos_subCallback);
    /* ---目标分布可视化 */
    mu_g_vis_puber = nh.advertise<sensor_msgs::PointCloud>("/mu_g", 1);
    mu_g_vis_msg = visualize_density(mu_g, delta_x_g, 3.0);
    mu_g_vis_msg.header.frame_id = "world";
    mu_rebuilt_vis_puber = nh.advertise<sensor_msgs::PointCloud>("/mu_rebuilt", 1);
    /* ---轨迹可视化 */
    traj_pts_puber = nh.advertise<geometry_msgs::PoseArray>("/traj_pts", 1);
    traj_pts_msg.header.frame_id = "world";
    /* ---轨迹更新计算 */
    traj_update_timer = nh.createTimer(ros::Duration(0.3), update_timeCallback);
    /* ---信息发布 */
    cmd_puber = nh.advertise<traj_utils::Flag>("/ergodic_cmd", 1);
    publish_timer = nh.createTimer(ros::Duration(0.1), publish_timeCallback);
    ros::spin();

    return 0;
}
