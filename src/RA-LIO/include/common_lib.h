// common_lib.h - RA-LIO公共类型定义和工具函数
// 功能：
//   1. 定义全局宏常量（PI、重力加速度、最近邻搜索点数等）
//   2. 定义全局类型别名（PointType, PointVector, V3D, M3D 等）
//   3. 定义 MeasureGroup 结构体（Lidar+IMU数据组）
//   4. 提供工具函数：坐标变换、平面估计、旋转矩阵转换等

#pragma once

#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <ra_lio/Pose6D.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>

using namespace std;
using namespace Eigen;

// === 常量定义 ===
#define PI_M (3.14159265358)
// #define G_m_s2 (9.81)               // 广东地区重力加速度
#define G_m_s2 (9.801)                 // 北京地区重力加速度
#define NUM_MATCH_POINTS    (5)        // 最近邻搜索点数，用于平面拟合法向量估计

// === 宏定义 ===
// 将 vector<double> 转换为 Eigen 初始化列表
#define VEC_FROM_ARRAY(v)        v[0],v[1],v[2]
// 将 vector<double> 转换为 3x3 矩阵初始化列表（行优先）
#define MAT_FROM_ARRAY(v)        v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8]
// 生成3x3反对称矩阵（skew-symmetric）的宏
#define SKEW_SYM_MATRX(v)        0.0,-v[2],v[1],v[2],0.0,-v[0],-v[1],v[0],0.0
// 拼接调试日志文件路径
#define DEBUG_FILE_DIR(name)     (string(string(ROOT_DIR) + "Log/"+ name))

// === 类型别名定义 ===
typedef ra_lio::Pose6D Pose6D;                        // 自定义位姿消息类型
typedef pcl::PointXYZINormal PointType;               // PCL点类型（含法向量、曲率）
typedef pcl::PointCloud<PointType> PointCloudXYZI;    // PCL点云类型
typedef vector<PointType, Eigen::aligned_allocator<PointType>>  PointVector; // 对齐内存的点容器
typedef Vector3d V3D;    // 3维双精度向量
typedef Matrix3d M3D;    // 3x3双精度矩阵
typedef Vector3f V3F;    // 3维单精度向量
typedef Matrix3f M3F;    // 3x3单精度矩阵

// 常用矩阵/向量常量（单位阵和零向量）
M3D Eye3d(M3D::Identity());
M3F Eye3f(M3F::Identity());
V3D Zero3d(0, 0, 0);
V3F Zero3f(0, 0, 0);

// 一次处理的测量数据组，包含一帧LiDAR点云和对应的IMU数据
struct MeasureGroup
{
    MeasureGroup()
    {
        lidar_beg_time = 0.0;
        this->lidar.reset(new PointCloudXYZI());
    };
    double lidar_beg_time;                     // LiDAR帧起始时间
    double lidar_end_time;                     // LiDAR帧结束时间
    PointCloudXYZI::Ptr lidar;                 // LiDAR点云指针
    deque<sensor_msgs::Imu::ConstPtr> imu;     // 该帧时间内的IMU测量序列
};

// 构造 Pose6D 消息（用于调试/日志输出）
template<typename T>
auto set_pose6d(const double t, const Matrix<T, 3, 1> &a, const Matrix<T, 3, 1> &g, \
                const Matrix<T, 3, 1> &v, const Matrix<T, 3, 1> &p, const Matrix<T, 3, 3> &R)
{
    Pose6D rot_kp;
    rot_kp.offset_time = t;
    for (int i = 0; i < 3; i++)
    {
        rot_kp.acc[i] = a(i);   // IMU加速度
        rot_kp.gyr[i] = g(i);   // IMU角速度
        rot_kp.vel[i] = v(i);   // 估计速度
        rot_kp.pos[i] = p(i);   // 估计位置
        for (int j = 0; j < 3; j++)  rot_kp.rot[i*3+j] = R(i,j); // 估计旋转矩阵
    }
    return move(rot_kp);
}

// 计算两点之间的欧氏距离平方
float calc_dist(PointType p1, PointType p2){
    float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    return d;
}

// 从5个最近邻点估计平面参数 A*x + B*y + C*z + D = 0
// pca_result: [A/norm, B/norm, C/norm, 1/norm] 即归一化平面系数
// point: 5个最近邻点
// threshold: 平面拟合残差阈值
template<typename T>
bool esti_plane(Matrix<T, 4, 1> &pca_result, const PointVector &point, const T &threshold)
{
    Matrix<T, NUM_MATCH_POINTS, 3> A;   // 系数矩阵
    Matrix<T, NUM_MATCH_POINTS, 1> b;    // 常数列向量
    A.setZero();
    b.setOnes();
    b *= -1.0f;   // b = [-1, -1, -1, -1, -1]^T

    // 构造线性方程组 A*target + b = 0，求解 A/D*x + B/D*y + C/D*z + 1 = 0 的参数
    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        A(j,0) = point[j].x;
        A(j,1) = point[j].y;
        A(j,2) = point[j].z;
    }

    // 使用列主元Householder QR分解求解超定方程组
    Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

    T n = normvec.norm();

    pca_result(0) = normvec(0) / n;  // 单位法向量分量
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;
    pca_result(3) = 1.0 / n;         // 平面偏移量 (D/norm)

    // 验证所有5个点是否都在拟合平面上（残差 < threshold）
    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) > threshold)
        {
            return false;
        }
    }
    return true;
}

// 旋转矩阵 R -> Yaw/Pitch/Roll（欧拉角，ZYX旋转顺序）返回值单位：度
static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
{
    Eigen::Vector3d n = R.col(0);  // x轴方向（前向）
    Eigen::Vector3d o = R.col(1);  // y轴方向（左向）
    Eigen::Vector3d a = R.col(2);  // z轴方向（上向）

    Eigen::Vector3d ypr(3);
    double y = atan2(n(1), n(0));                                          // Yaw: 绕z轴
    double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));               // Pitch: 绕y轴
    double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y)); // Roll: 绕x轴
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    return ypr / M_PI * 180.0;  // 转换为度
}

// Yaw/Pitch/Roll（度） -> 旋转矩阵（ZYX旋转顺序）
template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived> &ypr)
{
    typedef typename Derived::Scalar Scalar_t;

    Scalar_t y = ypr(0) / 180.0 * M_PI;
    Scalar_t p = ypr(1) / 180.0 * M_PI;
    Scalar_t r = ypr(2) / 180.0 * M_PI;

    // 绕z轴旋转 (Yaw)
    Eigen::Matrix<Scalar_t, 3, 3> Rz;
    Rz << cos(y), -sin(y), 0,
            sin(y), cos(y), 0,
            0, 0, 1;

    // 绕y轴旋转 (Pitch)
    Eigen::Matrix<Scalar_t, 3, 3> Ry;
    Ry << cos(p), 0., sin(p),
            0., 1., 0.,
            -sin(p), 0., cos(p);

    // 绕x轴旋转 (Roll)
    Eigen::Matrix<Scalar_t, 3, 3> Rx;
    Rx << 1., 0., 0.,
            0., cos(r), -sin(r),
            0., sin(r), cos(r);

    return Rz * Ry * Rx;
}

// 重力方向向量 -> 将重力对齐到世界z轴的旋转矩阵
// g: 当前估计的重力方向向量（IMU系下）
// 返回: 能够将重力方向旋转到世界系z轴方向的旋转矩阵
Eigen::Matrix3d g2R(const Eigen::Vector3d &g)
{
    Eigen::Matrix3d R0;
    Eigen::Vector3d ng1 = g.normalized();
    Eigen::Vector3d ng2{0, 0, 1.0};
    // 计算从重力方向到世界z轴的最短旋转
    R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
    double yaw = R2ypr(R0).x();
    R0 = ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;  // 消除旋转后的偏航分量
    return R0;
}
