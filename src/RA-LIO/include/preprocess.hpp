// preprocess.hpp - RA-LIO点云预处理模块头文件
// 功能：原始点云数据预处理，包括不同型号激光雷达的数据格式转换、时间戳处理、采样滤波
// 支持激光雷达类型：AVIA (Livox), VELO16, OUST64, RS32 (速腾), VANJEE16 (万集)

#pragma once

#include <livox_ros_driver2/CustomMsg.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <array>
#include <cstdint>
#include <vector>

#include "common_lib.hpp"

namespace ra_lio {

[[nodiscard]] inline bool isInvalidCoordinate(double value) noexcept {
  return std::abs(value) > 1e8;
}

// 激光雷达类型枚举
enum LID_TYPE {
  AVIA = 1,  // Livox AVIA 固态激光雷达（非重复扫描）
  VELO16,    // Velodyne VLP-16 机械旋转式雷达
  OUST64,    // Ouster OS1-64
  RS32,      // 速腾聚创 RS-32
  VANJEE16   // 万集 16线激光雷达
};  //{1, 2, 3, 4, 5}

// 时间戳单位枚举
enum TIME_UNIT {
  SEC = 0,  // 秒
  MS = 1,   // 毫秒
  US = 2,   // 微秒
  NS = 3    // 纳秒
};

// 特征点类型枚举（用于特征提取模式）
enum Feature {
  Nor,         // 普通点，未分类
  Poss_Plane,  // 可能是平面点
  Real_Plane,  // 确认为平面点
  Edge_Jump,   // 边缘跳变点（深度不连续处）
  Edge_Plane,  // 两个平面交界处的点
  Wire,        // 线状特征点（细杆、电线等）
  ZeroPoint    // 零点
};

// 相邻点方向枚举
enum Surround {
  Prev,  // 前一个点
  Next   // 后一个点
};

// 边缘跳变类型枚举
enum E_jump {
  Nr_nor,   // 正常邻接
  Nr_zero,  // 邻接点距离为零（重合点）
  Nr_180,   // 邻接点与当前点夹角接近180度
  Nr_inf,   // 邻接点距离无穷远（盲区外）
  Nr_blind  // 邻接点在盲区内
};

// 点的组织信息结构体，存储点的几何特征
struct orgtype {
  double range;      // 点到原点的距离（水平投影距离）
  double dista;      // 当前点与下一个点之间的欧氏距离
  double angle[2];   // 与前后邻接点的夹角余弦值 angle[Prev]/angle[Next]
  double intersect;  // 前后邻接向量的夹角余弦值
  E_jump edj[2];     // 前后邻接边缘跳变类型 edj[Prev]/edj[Next]
  Feature ftype;     // 特征点类型
  orgtype() {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;  // 初始化为2（远大于1，表示未计算）
  }
};

}  // namespace ra_lio

// === 各品牌激光雷达的自定义PCL点结构体注册 ===
// PCL要求对不同厂商雷达的点云格式进行注册，以便使用fromROSMsg/toROSMsg转换

namespace velodyne_ros {
struct EIGEN_ALIGN16 Point {
  PCL_ADD_POINT4D;  // 添加 x, y, z 字段
  float intensity;  // 反射强度
  float time;       // 相对时间偏移（秒）
  uint16_t ring;    // 线束编号
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                      float, intensity, intensity)(float, time, time)(std::uint16_t,
                                                                                      ring, ring))

namespace rslidar_ros {
struct EIGEN_ALIGN16 Point {
  PCL_ADD_POINT4D;
  std::uint8_t intensity;  // 反射强度
  std::uint16_t ring = 0;  // 线束编号
  double timestamp = 0;    // 绝对时间戳
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace rslidar_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(rslidar_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity,
                                                                          intensity)(
                                      std::uint16_t, ring, ring)(double, timestamp, timestamp))

// 万集雷达点类型定义
namespace vanjee_ros {
struct EIGEN_ALIGN16 Point {
  PCL_ADD_POINT4D;
  float intensity;
  std::uint16_t ring;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace vanjee_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(vanjee_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                          intensity)(
                                      std::uint16_t, ring, ring)(double, timestamp, timestamp))

// Ouster雷达点类型定义
namespace ouster_ros {
struct EIGEN_ALIGN16 Point {
  PCL_ADD_POINT4D;
  float intensity;
  std::uint32_t t;             // 纳秒级时间戳
  std::uint16_t reflectivity;  // 反射率
  std::uint8_t ring;           // 线束编号
  std::uint16_t ambient;       // 环境光强度
  std::uint32_t range;         // 距离（毫米）
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)
// clang-format on

namespace ra_lio {

struct PreprocessorConfig {
  bool feature_enabled{false};
  int lidar_type{AVIA};
  int scan_lines{6};
  int scan_rate{10};
  int timestamp_unit{US};
  int point_filter_num{1};
  double blind{0.01};
};

// 点云预处理类
class Preprocessor {
 public:
  Preprocessor();

  // process函数重载：支持Livox自定义消息格式和标准PointCloud2格式
  void process(const livox_ros_driver2::CustomMsg::ConstPtr& msg, PointCloudXYZI::Ptr& pcl_out);
  void process(const sensor_msgs::PointCloud2::ConstPtr& msg, PointCloudXYZI::Ptr& pcl_out);

  // 通过参数设置预处理参数
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);
  void configure(const PreprocessorConfig& config);
  [[nodiscard]] int lidarType() const noexcept { return lidar_type; }

 private:
  // 点云存储：pl_full全部点、pl_corn角点/边缘点、pl_surf平面点/输出点
  PointCloudXYZI pl_full, pl_corn, pl_surf;
  std::array<PointCloudXYZI, 128> pl_buff;       // 按线号缓存点云，最大支持128线
  std::array<std::vector<orgtype>, 128> typess;  // 按线号缓存点的特征类型信息
  float time_unit_scale;                         // 时间单位缩放因子
  int lidar_type, point_filter_num, N_SCANS, SCAN_RATE, time_unit;
  double blind;            // 盲区距离（m），小于此距离的点被过滤
  bool feature_enabled;    // 是否启用特征提取（AVIA雷达的特定功能）
  bool given_offset_time;  // 雷达数据是否自带时间偏移
  ros::Publisher pub_full, pub_surf, pub_corn;

  // 各雷达型号的处理函数
  void vanjee_handler(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void rs_handler(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void avia_handler(const livox_ros_driver2::CustomMsg::ConstPtr& msg);
  void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr& msg);

  // 特征提取辅助函数
  void give_feature(PointCloudXYZI& pl, std::vector<orgtype>& types);  // 给点云中每个点赋予特征类型
  void pub_func(PointCloudXYZI& pl, const ros::Time& ct);              // 发布点云
  int plane_judge(const PointCloudXYZI& pl, std::vector<orgtype>& types, uint i, uint& i_nex,
                  Eigen::Vector3d& curr_direct);  // 平面判定
  bool small_plane(const PointCloudXYZI& pl, std::vector<orgtype>& types, uint i_cur, uint& i_nex,
                   Eigen::Vector3d& curr_direct);
  bool edge_jump_judge(const PointCloudXYZI& pl, std::vector<orgtype>& types, uint i,
                       Surround nor_dir);  // 边缘跳变判定

  // 特征提取相关参数
  int group_size;                // 每组点的数量
  double disA, disB, inf_bound;  // 距离参数：disA*range+disB=组距离阈值, inf_bound=无穷远阈值
  double limit_maxmid, limit_midmin, limit_maxmin;  // 距离比值限制（用于平面判定）
  double p2l_ratio;                                 // 点到线距离比率阈值（用于平面判定）
  double jump_up_limit, jump_down_limit;            // 跳变角度余弦的上下限
  double cos160;                                    // cos(160°)值
  double edgea, edgeb;                              // 边缘判定参数
  double smallp_intersect, smallp_ratio;            // 小平面判定参数
  double vx, vy, vz;                                // 差分向量分量（当前点与下一点的xyz差）
};

}  // namespace ra_lio
