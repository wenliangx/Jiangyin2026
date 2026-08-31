// preprocess.cpp - RA-LIO点云预处理实现
// 功能：
//   1. 各型号激光雷达数据格式转换为统一的PCL PointXYZINormal格式
//   2. 点云滤波（盲区过滤、降采样、重合点去除）
//   3. 可选的特征提取（平面点、角点/边缘点分类）
// 支持雷达：Livox AVIA, Velodyne VLP-16, Ouster OS1-64, RS-32, Vanjee-16

#include "preprocess.hpp"

namespace ra_lio {

Preprocessor::Preprocessor() {
  feature_enabled = false;
  lidar_type = AVIA;
  blind = 0.01;
  point_filter_num = 1;
  inf_bound = 10;             // 无穷远边界：10m
  N_SCANS = 6;                // 默认扫描线数（Livox AVIA）
  SCAN_RATE = 10;             // 默认扫描频率 10Hz
  group_size = 8;             // 平面判定时的组大小
  disA = 0.01;                // 距离系数A：group_dis的线性系数
  disA = 0.1;                 // 距离系数A（覆盖了上面的值）
  p2l_ratio = 225;            // 点到线距离比率阈值
  limit_maxmid = 6.25;        // 最大距离/中位距离比值阈值
  limit_midmin = 6.25;        // 中位距离/最小距离比值阈值
  limit_maxmin = 3.24;        // 最大距离/最小距离比值阈值（非AVIA雷达使用）
  jump_up_limit = 170.0;      // 跳变角度上限（度）-> 转换为 cos(170°)
  jump_down_limit = 8.0;      // 跳变角度下限（度）-> 转换为 cos(8°)
  cos160 = 160.0;             // cos(160°) 用于边缘判定
  edgea = 2;                  // 边缘跳变判定参数a
  edgeb = 0.1;                // 边缘跳变判定参数b
  smallp_intersect = 172.5;   // 小平面相交角度下限（度）
  smallp_ratio = 1.2;         // 小平面距离比率阈值
  given_offset_time = false;  // 雷达是否自带点的时间戳偏移

  // 将角度值预计算为cos值（加速后续运算）
  jump_up_limit = cos(jump_up_limit / 180 * M_PI);        // cos(170°) ≈ -0.9848
  jump_down_limit = cos(jump_down_limit / 180 * M_PI);    // cos(8°) ≈ 0.9903
  cos160 = cos(cos160 / 180 * M_PI);                      // cos(160°) ≈ -0.9397
  smallp_intersect = cos(smallp_intersect / 180 * M_PI);  // cos(172.5°) ≈ -0.9914
}

// 设置预处理参数
void Preprocessor::set(bool feat_en, int lid_type, double bld, int pfilt_num) {
  feature_enabled = feat_en;     // 是否启用特征提取
  lidar_type = lid_type;         // 激光雷达型号
  blind = bld;                   // 盲区距离（m）
  point_filter_num = pfilt_num;  // 降采样间隔
}

void Preprocessor::configure(const PreprocessorConfig& config) {
  feature_enabled = config.feature_enabled;
  lidar_type = config.lidar_type;
  blind = config.blind;
  point_filter_num = config.point_filter_num;
  N_SCANS = config.scan_lines;
  SCAN_RATE = config.scan_rate;
  time_unit = config.timestamp_unit;
}

// === process函数重载 ===
// 处理Livox自定义消息格式（AVIA雷达）
void Preprocessor::process(const livox_ros_driver2::CustomMsg::ConstPtr& msg,
                           PointCloudXYZI::Ptr& pcl_out) {
  avia_handler(msg);   // 调用AVIA专用处理函数
  *pcl_out = pl_surf;  // 输出处理后的点云
}

// 处理标准PointCloud2格式（其他雷达）
void Preprocessor::process(const sensor_msgs::PointCloud2::ConstPtr& msg,
                           PointCloudXYZI::Ptr& pcl_out) {
  // 根据配置的时间戳单位设置缩放因子
  switch (time_unit) {
    case SEC:
      time_unit_scale = 1.e3f;
      break;  // 秒 -> 毫秒
    case MS:
      time_unit_scale = 1.f;
      break;  // 毫秒 -> 毫秒（不变）
    case US:
      time_unit_scale = 1.e-3f;
      break;  // 微秒 -> 毫秒
    case NS:
      time_unit_scale = 1.e-6f;
      break;  // 纳秒 -> 毫秒
    default:
      time_unit_scale = 1.f;
      break;  // 默认毫秒
  }

  // 根据雷达型号分发到不同的处理函数
  switch (lidar_type) {
    case OUST64:
      oust64_handler(msg);
      break;
    case VELO16:
      velodyne_handler(msg);
      break;
    case RS32:
      rs_handler(msg);
      break;
    case VANJEE16:
      vanjee_handler(msg);
      break;
    default:
      printf("Error LiDAR Type");
      break;
  }
  // 输出预处理后的点云
  *pcl_out = pl_surf;
}

// === avia_handler：处理Livox AVIA雷达数据 ===
// 特点：非重复扫描模式，点分布不规则
// 支持两种模式：特征提取模式、简单降采样模式
void Preprocessor::avia_handler(const livox_ros_driver2::CustomMsg::ConstPtr& msg) {
  // 清空所有缓存
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  int plsize = msg->point_num;

  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  pl_full.resize(plsize);

  // 初始化每线的点缓存
  for (int i = 0; i < N_SCANS; i++) {
    pl_buff[i].clear();
    pl_buff[i].reserve(plsize);
  }
  uint valid_num = 0;

  // --- 模式1：特征提取模式（feature_enabled = true）---
  if (feature_enabled) {
    for (uint i = 1; i < plsize; i++) {
      // 检查线号范围（0~N_SCANS-1）和点类型标记 tag bits 5-6
      // tag & 0x30 == 0x10: 单回波点 或 tag & 0x30 == 0x00: 正常点
      if ((msg->points[i].line < N_SCANS) &&
          ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)) {
        pl_full[i].x = msg->points[i].x;
        pl_full[i].y = msg->points[i].y;
        pl_full[i].z = msg->points[i].z;
        pl_full[i].intensity = msg->points[i].reflectivity;
        // 将偏移时间（微秒）转为毫秒，存储到curvature字段
        // curvature在此处作为每点相对于帧起始的时间偏移（ms）
        pl_full[i].curvature = msg->points[i].offset_time / float(1000000);

        // 过滤重合点（严格比较两点坐标差异）
        if ((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) ||
            (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) ||
            (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7)) {
          pl_buff[msg->points[i].line].push_back(pl_full[i]);  // 按线号缓存
        }
      }
    }
    static int count = 0;
    static double time = 0.0;
    count++;
    double t0 = omp_get_wtime();

    // 对每条扫描线分别提取特征
    for (int j = 0; j < N_SCANS; j++) {
      if (pl_buff[j].size() <= 5) continue;  // 点数太少跳过
      pcl::PointCloud<PointType>& pl = pl_buff[j];
      plsize = pl.size();
      std::vector<orgtype>& types = typess[j];
      types.clear();
      types.resize(plsize);
      plsize--;
      // 计算每个点到原点的range（水平投影距离）和相邻点间距dista
      for (uint i = 0; i < plsize; i++) {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = sqrt(vx * vx + vy * vy + vz * vz);  // 相邻点之间的欧氏距离
      }
      types[plsize].range = sqrt(pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y);
      give_feature(pl, types);  // 特征提取核心函数
    }
    time += omp_get_wtime() - t0;
    printf("Feature extraction time: %lf \n", time / count);
  }
  // --- 模式2：简单降采样模式（feature_enabled = false, 默认模式）---
  else {
    for (uint i = 1; i < plsize; i++) {
      if ((msg->points[i].line < N_SCANS) &&
          ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)) {
        valid_num++;
        // 降采样：每隔point_filter_num个有效点取一个
        if (valid_num % point_filter_num == 0) {
          pl_full[i].x = msg->points[i].x;
          pl_full[i].y = msg->points[i].y;
          pl_full[i].z = msg->points[i].z;
          pl_full[i].intensity = msg->points[i].reflectivity;
          // 将offset_time（微秒）转换为毫秒，存储到curvature
          pl_full[i].curvature = msg->points[i].offset_time / float(1000000);

          // 过滤重合点和盲区内的点
          if (((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) ||
               (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) ||
               (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7)) &&
              (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y +
                   pl_full[i].z * pl_full[i].z >
               (blind * blind))) {
            pl_surf.push_back(pl_full[i]);  // 添加到输出点云
          }
        }
      }
    }
  }
}

// === oust64_handler：处理Ouster OS1-64雷达数据 ===
void Preprocessor::oust64_handler(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);  // ROS消息转PCL格式
  int plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);

  // --- 特征提取模式 ---
  if (feature_enabled) {
    for (int i = 0; i < N_SCANS; i++) {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    for (uint i = 0; i < plsize; i++) {
      double range = pl_orig.points[i].x * pl_orig.points[i].x +
                     pl_orig.points[i].y * pl_orig.points[i].y +
                     pl_orig.points[i].z * pl_orig.points[i].z;
      if (range < (blind * blind)) continue;  // 盲区过滤

      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;
      if (yaw_angle >= 180.0) yaw_angle -= 360.0;
      if (yaw_angle <= -180.0) yaw_angle += 360.0;

      added_pt.curvature = pl_orig.points[i].t * time_unit_scale;  // 时间偏移转换为ms
      if (pl_orig.points[i].ring < N_SCANS) {
        pl_buff[pl_orig.points[i].ring].push_back(added_pt);  // 按线号分组
      }
    }

    // 每条线单独提取特征
    for (int j = 0; j < N_SCANS; j++) {
      PointCloudXYZI& pl = pl_buff[j];
      int linesize = pl.size();
      std::vector<orgtype>& types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;
      for (uint i = 0; i < linesize; i++) {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;  // 注意：Ouster使用平方距离
      }
      types[linesize].range =
          sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);
    }
  }
  // --- 简化模式 ---
  else {
    for (int i = 0; i < pl_orig.points.size(); i++) {
      if (i % point_filter_num != 0) continue;  // 降采样

      double range = pl_orig.points[i].x * pl_orig.points[i].x +
                     pl_orig.points[i].y * pl_orig.points[i].y +
                     pl_orig.points[i].z * pl_orig.points[i].z;
      if (range < (blind * blind)) continue;  // 盲区过滤

      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.curvature = pl_orig.points[i].t * time_unit_scale;  // 时间偏移 ms

      pl_surf.points.push_back(added_pt);
    }
  }
}

// === velodyne_handler：处理Velodyne VLP-16雷达数据 ===
void Preprocessor::velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  pcl::PointCloud<velodyne_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.points.size();
  if (plsize == 0) return;
  pl_surf.reserve(plsize);

  // 当雷达点没有自带时间戳时，根据扫描角度推算时间偏移
  double omega_l = 0.361 * SCAN_RATE;          // 扫描角速度 (度/ms)
  std::vector<bool> is_first(N_SCANS, true);   // 记录每条线的第一个点
  std::vector<double> yaw_fp(N_SCANS, 0.0);    // 每条线第一个点的偏航角
  std::vector<float> yaw_last(N_SCANS, 0.0);   // 每条线上一个点的偏航角
  std::vector<float> time_last(N_SCANS, 0.0);  // 每条线上一个点的时间偏移

  // 检查最后一个点是否有时间戳
  if (pl_orig.points[plsize - 1].time > 0) {
    given_offset_time = true;
  } else {
    given_offset_time = false;
  }

  // --- 特征提取模式 ---
  if (feature_enabled) {
    for (int i = 0; i < N_SCANS; i++) {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    for (int i = 0; i < plsize; i++) {
      PointType added_pt;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      int layer = pl_orig.points[i].ring;
      if (layer >= N_SCANS) continue;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].time * time_unit_scale;  // 时间偏移 ms

      // 推算时间偏移（无时间戳时）
      if (!given_offset_time) {
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
        if (is_first[layer]) {
          yaw_fp[layer] = yaw_angle;
          is_first[layer] = false;
          added_pt.curvature = 0.0;  // 同线第一个点时间设为0
          yaw_last[layer] = yaw_angle;
          time_last[layer] = added_pt.curvature;
          continue;
        }

        // 根据角度差计算时间偏移
        if (yaw_angle <= yaw_fp[layer]) {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
        } else {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
        }

        if (added_pt.curvature < time_last[layer])
          added_pt.curvature += 360.0 / omega_l;  // 处理环绕

        yaw_last[layer] = yaw_angle;
        time_last[layer] = added_pt.curvature;
      }

      pl_buff[layer].points.push_back(added_pt);
    }

    // 每条线分别特征提取
    for (int j = 0; j < N_SCANS; j++) {
      PointCloudXYZI& pl = pl_buff[j];
      int linesize = pl.size();
      if (linesize < 2) continue;
      std::vector<orgtype>& types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;
      for (uint i = 0; i < linesize; i++) {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range =
          sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);
    }
  }
  // --- 简化模式 ---
  else {
    for (int i = 0; i < plsize; i++) {
      PointType added_pt;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].time * time_unit_scale;

      // 推算时间偏移
      if (!given_offset_time) {
        int layer = pl_orig.points[i].ring;
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

        if (is_first[layer]) {
          yaw_fp[layer] = yaw_angle;
          is_first[layer] = false;
          added_pt.curvature = 0.0;
          yaw_last[layer] = yaw_angle;
          time_last[layer] = added_pt.curvature;
          continue;
        }

        if (yaw_angle <= yaw_fp[layer]) {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
        } else {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
        }

        if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;

        yaw_last[layer] = yaw_angle;
        time_last[layer] = added_pt.curvature;
      }

      // 降采样 + 盲区过滤
      if (i % point_filter_num == 0) {
        if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z >
            (blind * blind)) {
          pl_surf.points.push_back(added_pt);
        }
      }
    }
  }
}

// === give_feature：特征提取核心函数 ===
// 对一条扫描线上的点进行分类：识别平面点(Planar)、边缘点(Edge)、角点(Corner)
// 算法流程：
//   1. plane_judge: 判定一组连续点是否构成平面
//   2. 边缘跳变检测：检测深度不连续处（遮挡边界）
//   3. 小平面检测：检测不稳定的平面点
//   4. 输出筛选：按point_filter_num间隔输出平面点，同时单独输出角点/边缘点
void Preprocessor::give_feature(pcl::PointCloud<PointType>& pl, std::vector<orgtype>& types) {
  int plsize = pl.size();
  int plsize2;
  if (plsize == 0) {
    printf("something wrong\n");
    return;
  }
  uint head = 0;

  // 跳过盲区内的起始点
  while (types[head].range < blind) {
    head++;
  }

  // --- 第一阶段：平面点检测 ---
  plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;

  Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());  // 当前组的方向向量
  Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());  // 上一组的方向向量

  uint i_nex = 0;
  int last_state = 0;
  int plane_type;

  for (uint i = head; i < plsize2; i++) {
    if (types[i].range < blind) continue;

    // 判定从i开始的一组点是否构成平面
    plane_type = plane_judge(pl, types, i, i_nex, curr_direct);

    if (plane_type == 1)  // 找到平面
    {
      // 标记组内点为平面点
      for (uint j = i; j <= i_nex; j++) {
        if (j != i && j != i_nex)
          types[j].ftype = Real_Plane;  // 内部点：确认平面点
        else
          types[j].ftype = Poss_Plane;  // 边界点：可能平面点
      }

      // 检查与上一平面的夹角：如果接近90度，标记为边缘点
      if (last_state == 1 && last_direct.norm() > 0.1) {
        double mod = last_direct.transpose() * curr_direct;  // cos(夹角)
        if (mod > -0.707 && mod < 0.707)                     // 夹角约在45°~135°
        {
          types[i].ftype = Edge_Plane;  // 平面交界处即为边缘
        } else {
          types[i].ftype = Real_Plane;
        }
      }

      i = i_nex - 1;
      last_state = 1;
    } else  // plane_type == 2 或 0，非平面
    {
      i = i_nex;
      last_state = 0;
    }

    last_direct = curr_direct;
  }

  // --- 第二阶段：边缘跳变检测 ---
  // 检测因遮挡产生的深度不连续（如物体的边界）
  plsize2 = plsize > 3 ? plsize - 3 : 0;
  for (uint i = head + 3; i < plsize2; i++) {
    if (types[i].range < blind || types[i].ftype >= Real_Plane) continue;
    if (types[i - 1].dista < 1e-16 || types[i].dista < 1e-16) continue;

    Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);
    Eigen::Vector3d vecs[2]{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

    // 计算与前后邻接点的夹角信息
    for (int j = 0; j < 2; j++) {
      int m = -1;
      if (j == 1) m = 1;  // Prev(-1) / Next(+1)

      if (types[i + m].range < blind) {
        if (types[i].range > inf_bound)
          types[i].edj[j] = Nr_inf;  // 邻接点在盲区且当前点远
        else
          types[i].edj[j] = Nr_blind;  // 邻接点在盲区且当前点近
        continue;
      }

      vecs[j] = Eigen::Vector3d(pl[i + m].x, pl[i + m].y, pl[i + m].z);
      vecs[j] = vecs[j] - vec_a;  // 邻接向量

      types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();  // cos(夹角)
      if (types[i].angle[j] < jump_up_limit)         // cos < cos(170°) ≈ -0.9848
        types[i].edj[j] = Nr_180;                    // 夹角接近180°
      else if (types[i].angle[j] > jump_down_limit)  // cos > cos(8°) ≈ 0.9903
        types[i].edj[j] = Nr_zero;                   // 夹角接近0°
    }

    types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();

    // 跳变检测条件：
    // edj[Prev]==Nr_nor && edj[Next]==Nr_zero: 前面正常、后面几乎重合 -> 前方深度跳变
    // edj[Prev]==Nr_zero && edj[Next]==Nr_nor: 前面重合、后面正常 -> 后方深度跳变
    if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_zero && types[i].dista > 0.0225 &&
        types[i].dista > 4 * types[i - 1].dista) {
      if (types[i].intersect > cos160) {
        if (edge_jump_judge(pl, types, i, Prev)) types[i].ftype = Edge_Jump;
      }
    } else if (types[i].edj[Prev] == Nr_zero && types[i].edj[Next] == Nr_nor &&
               types[i - 1].dista > 0.0225 && types[i - 1].dista > 4 * types[i].dista) {
      if (types[i].intersect > cos160) {
        if (edge_jump_judge(pl, types, i, Next)) types[i].ftype = Edge_Jump;
      }
    } else if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_inf) {
      if (edge_jump_judge(pl, types, i, Prev)) types[i].ftype = Edge_Jump;
    } else if (types[i].edj[Prev] == Nr_inf && types[i].edj[Next] == Nr_nor) {
      if (edge_jump_judge(pl, types, i, Next)) types[i].ftype = Edge_Jump;
    }
    // 两个方向都跳变 -> 线状特征（电线、细杆）
    else if (types[i].edj[Prev] > Nr_nor && types[i].edj[Next] > Nr_nor) {
      if (types[i].ftype == Nor) types[i].ftype = Wire;
    }
  }

  // --- 第三阶段：小平面检测 ---
  // 检测距离变化剧烈的不稳定平面点（可能是测量误差）
  plsize2 = plsize - 1;
  double ratio;
  for (uint i = head + 1; i < plsize2; i++) {
    if (types[i].range < blind || types[i - 1].range < blind || types[i + 1].range < blind)
      continue;
    if (types[i - 1].dista < 1e-8 || types[i].dista < 1e-8) continue;

    if (types[i].ftype == Nor) {
      if (types[i - 1].dista > types[i].dista)
        ratio = types[i - 1].dista / types[i].dista;
      else
        ratio = types[i].dista / types[i - 1].dista;

      if (types[i].intersect < smallp_intersect && ratio < smallp_ratio) {
        if (types[i - 1].ftype == Nor) types[i - 1].ftype = Real_Plane;
        if (types[i + 1].ftype == Nor) types[i + 1].ftype = Real_Plane;
        types[i].ftype = Real_Plane;
      }
    }
  }

  // --- 第四阶段：输出筛选 ---
  int last_surface = -1;
  for (uint j = head; j < plsize; j++) {
    if (types[j].ftype == Poss_Plane || types[j].ftype == Real_Plane) {
      if (last_surface == -1) {
        last_surface = j;
      }

      // 按point_filter_num间隔输出平面点
      if (j == uint(last_surface + point_filter_num - 1)) {
        PointType ap;
        ap.x = pl[j].x;
        ap.y = pl[j].y;
        ap.z = pl[j].z;
        ap.intensity = pl[j].intensity;
        ap.curvature = pl[j].curvature;
        pl_surf.push_back(ap);
        last_surface = -1;
      }
    } else {
      // 输出角点和边缘跳变点
      if (types[j].ftype == Edge_Jump || types[j].ftype == Edge_Plane) {
        pl_corn.push_back(pl[j]);
      }
      // 输出连续平面段的平均点
      if (last_surface != -1) {
        PointType ap;
        for (uint k = last_surface; k < j; k++) {
          ap.x += pl[k].x;
          ap.y += pl[k].y;
          ap.z += pl[k].z;
          ap.intensity += pl[k].intensity;
          ap.curvature += pl[k].curvature;
        }
        ap.x /= (j - last_surface);
        ap.y /= (j - last_surface);
        ap.z /= (j - last_surface);
        ap.intensity /= (j - last_surface);
        ap.curvature /= (j - last_surface);
        pl_surf.push_back(ap);
      }
      last_surface = -1;
    }
  }
}

// 发布点云辅助函数（未使用，保留作为调试接口）
void Preprocessor::pub_func(PointCloudXYZI& pl, const ros::Time& ct) {
  pl.height = 1;
  pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "livox";
  output.header.stamp = ct;
}

// === plane_judge：平面判定函数 ===
// 判定从 i_cur 开始的一组连续点是否构成平面
// 原理：
//   1. 找出一组空间上相邻的点（距离小于 disA*range + disB）
//   2. 计算该组点到首末点连线的最大距离（点到线距离）
//   3. 根据距离比值判断是否为平面：
//      - (two_dis^2 / max_width) < p2l_ratio  -> 非平面(0)
//      - 距离比值满足限制条件                    -> 平面(1)
//   4. 对AVIA雷达额外检查距离分布均匀性（dismax_mid, dismid_min）
int Preprocessor::plane_judge(const PointCloudXYZI& pl, std::vector<orgtype>& types, uint i_cur,
                              uint& i_nex, Eigen::Vector3d& curr_direct) {
  double group_dis = disA * types[i_cur].range + disB;  // 组判定距离阈值
  group_dis = group_dis * group_dis;

  double two_dis = 0.0;
  std::vector<double> disarr;
  disarr.reserve(20);

  // 收集至少group_size个相邻点
  for (i_nex = i_cur; i_nex < i_cur + group_size; i_nex++) {
    if (types[i_nex].range < blind) {
      curr_direct.setZero();
      return 2;
    }  // 盲区点 -> 无效
    disarr.push_back(types[i_nex].dista);
  }

  // 继续收集相邻点直到与起始点距离超过group_dis
  for (;;) {
    if ((i_cur >= pl.size()) || (i_nex >= pl.size())) break;

    if (types[i_nex].range < blind) {
      curr_direct.setZero();
      return 2;
    }
    vx = pl[i_nex].x - pl[i_cur].x;
    vy = pl[i_nex].y - pl[i_cur].y;
    vz = pl[i_nex].z - pl[i_cur].z;
    two_dis = vx * vx + vy * vy + vz * vz;  // 首末点距离的平方
    if (two_dis >= group_dis) break;        // 距离超过阈值则停止
    disarr.push_back(types[i_nex].dista);
    i_nex++;
  }

  // 计算中间点到首末点连线的最大距离
  double leng_wid = 0;
  double v1[3], v2[3];
  for (uint j = i_cur + 1; j < i_nex; j++) {
    if ((j >= pl.size()) || (i_cur >= pl.size())) break;
    v1[0] = pl[j].x - pl[i_cur].x;
    v1[1] = pl[j].y - pl[i_cur].y;
    v1[2] = pl[j].z - pl[i_cur].z;

    // 叉积 v2 = v1 × (pl[i_nex] - pl[i_cur])，得到与首末连线垂直的向量
    v2[0] = v1[1] * vz - vy * v1[2];
    v2[1] = v1[2] * vx - v1[0] * vz;
    v2[2] = v1[0] * vy - vx * v1[1];

    double lw = v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2];  // 垂直距离平方
    if (lw > leng_wid) leng_wid = lw;                           // 更新最大垂直距离
  }

  // 条件1：点到线距离比率检查
  if ((two_dis * two_dis / leng_wid) < p2l_ratio) {
    curr_direct.setZero();
    return 0;  // 点到线距离太大，非平面
  }

  // 将相邻点距离从大到小排序
  uint disarrsize = disarr.size();
  for (uint j = 0; j < disarrsize - 1; j++) {
    for (uint k = j + 1; k < disarrsize; k++) {
      if (disarr[j] < disarr[k]) {
        leng_wid = disarr[j];
        disarr[j] = disarr[k];
        disarr[k] = leng_wid;
      }
    }
  }

  if (disarr[disarr.size() - 2] < 1e-16)  // 次小距离为0，避免除零
  {
    curr_direct.setZero();
    return 0;
  }

  // 条件2：距离比值限制
  if (lidar_type == AVIA) {
    // AVIA特殊判定：最大/中位距离比值 和 中位/最小距离比值
    double dismax_mid = disarr[0] / disarr[disarrsize / 2];
    double dismid_min = disarr[disarrsize / 2] / disarr[disarrsize - 2];

    if (dismax_mid >= limit_maxmid || dismid_min >= limit_midmin) {
      curr_direct.setZero();
      return 0;  // 点间距分布不均匀 -> 非平面
    }
  } else {
    // 其他雷达判定：最大/最小距离比值
    double dismax_min = disarr[0] / disarr[disarrsize - 2];
    if (dismax_min >= limit_maxmin) {
      curr_direct.setZero();
      return 0;
    }
  }

  // 通过了所有检查 -> 认定为平面，记录平面的方向向量
  curr_direct << vx, vy, vz;
  curr_direct.normalize();
  return 1;
}

// === edge_jump_judge：边缘跳变判定 ===
// 检查距离跳变是否为真正的边缘还是噪声
// 原理：比较相邻点的间距变化，如果一侧间距急剧增大则可能为边缘
bool Preprocessor::edge_jump_judge(const PointCloudXYZI&, std::vector<orgtype>& types, uint i,
                                   Surround nor_dir) {
  // 检查所需的临近点是否在盲区
  if (nor_dir == 0)  // Prev方向
  {
    if (types[i - 1].range < blind || types[i - 2].range < blind) return false;
  } else if (nor_dir == 1)  // Next方向
  {
    if (types[i + 1].range < blind || types[i + 2].range < blind) return false;
  }

  double d1 = types[i + nor_dir - 1].dista;      // 较近邻接点的距离
  double d2 = types[i + 3 * nor_dir - 2].dista;  // 较远邻接点的距离
  double d;

  if (d1 < d2) {
    d = d1;
    d1 = d2;
    d2 = d;
  }  // 确保 d1 >= d2

  d1 = sqrt(d1);
  d2 = sqrt(d2);

  // 条件判断：d1 <= edgea * d2 且 d1 - d2 <= edgeb
  // edgea=2, edgeb=0.1 -> 间距变化不能太大
  if (d1 > edgea * d2 || (d1 - d2) > edgeb) return false;

  return true;
}

// === rs_handler：处理速腾RS-32雷达数据 ===
void Preprocessor::rs_handler(
    const sensor_msgs::PointCloud2_<std::allocator<void>>::ConstPtr& msg) {
  pl_surf.clear();

  pcl::PointCloud<rslidar_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.points.size();
  pl_surf.reserve(plsize);

  // 当雷达点没有自带时间戳时，根据扫描角度推算时间偏移
  double omega_l = 0.361 * SCAN_RATE;
  std::vector<bool> is_first(N_SCANS, true);
  std::vector<double> yaw_fp(N_SCANS, 0.0);
  std::vector<float> yaw_last(N_SCANS, 0.0);
  std::vector<float> time_last(N_SCANS, 0.0);

  // 检查是否自带时间戳（RS数据使用timestamp字段）
  if (pl_orig.points[plsize - 1].timestamp > 0) {
    given_offset_time = true;
  } else {
    given_offset_time = false;
  }

  for (int i = 0; i < plsize; i++) {
    PointType added_pt;

    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    // RS雷达的时间使用绝对时间戳，转换为相对偏移（ms）
    added_pt.curvature = (pl_orig.points[i].timestamp - pl_orig.points[0].timestamp) * 1000.0;

    // 推算时间偏移
    if (!given_offset_time) {
      int layer = pl_orig.points[i].ring;
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

      if (is_first[layer]) {
        yaw_fp[layer] = yaw_angle;
        is_first[layer] = false;
        added_pt.curvature = 0.0;
        yaw_last[layer] = yaw_angle;
        time_last[layer] = added_pt.curvature;
        continue;
      }

      if (yaw_angle <= yaw_fp[layer])
        added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
      else
        added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;

      if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;

      yaw_last[layer] = yaw_angle;
      time_last[layer] = added_pt.curvature;
    }

    // 降采样 + 盲区过滤
    if (i % point_filter_num == 0) {
      if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z >
          (blind * blind)) {
        pl_surf.points.push_back(added_pt);
      }
    }
  }
}

// === vanjee_handler：处理万集Vanjee-16雷达数据 ===
void Preprocessor::vanjee_handler(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  pl_surf.clear();

  pcl::PointCloud<vanjee_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.points.size();
  pl_surf.reserve(plsize);

  // 时间戳推算参数
  double omega_l = 0.361 * SCAN_RATE;
  std::vector<bool> is_first(N_SCANS, true);
  std::vector<double> yaw_fp(N_SCANS, 0.0);
  std::vector<float> yaw_last(N_SCANS, 0.0);
  std::vector<float> time_last(N_SCANS, 0.0);

  if (pl_orig.points[plsize - 1].timestamp > 0) {
    given_offset_time = true;
  } else {
    given_offset_time = false;
  }

  for (int i = 0; i < plsize; i++) {
    PointType added_pt;

    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    added_pt.curvature = pl_orig.points[i].timestamp * time_unit_scale;

    // 推算时间偏移
    if (!given_offset_time) {
      int layer = pl_orig.points[i].ring;
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

      if (is_first[layer]) {
        yaw_fp[layer] = yaw_angle;
        is_first[layer] = false;
        added_pt.curvature = 0.0;
        yaw_last[layer] = yaw_angle;
        time_last[layer] = added_pt.curvature;
        continue;
      }

      if (yaw_angle <= yaw_fp[layer])
        added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
      else
        added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;

      if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;

      yaw_last[layer] = yaw_angle;
      time_last[layer] = added_pt.curvature;
    }

    // 降采样 + 盲区过滤
    if (i % point_filter_num == 0) {
      if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z >
          (blind * blind)) {
        pl_surf.points.push_back(added_pt);
      }
    }
  }
}

}  // namespace ra_lio
