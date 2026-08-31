// laser_mapping.cpp - RA-LIO主节点：激光雷达-惯性里程计实时建图
// 基于 FAST-LIO2 框架的简化版本，适配Livox MID360固态激光雷达
// 核心流程：
//   1. 数据同步：将LiDAR帧与对应时间段内的IMU数据打包成MeasureGroup
//   2. IMU处理：初始化（重力对齐、bias估计）和点云去畸变/坐标变换
//   3. 局部地图管理：基于当前位姿动态更新局部地图范围（FOV）
//   4. ESKF更新：基于点到平面距离残差的迭代卡尔曼滤波
//   5. 地图增量：将当前帧点云加入全局ikd-Tree地图
//   6. 发布：里程计Odometry、点云Cloud、路径Path
//
// 坐标系说明：
//   body系 = IMU系
//   LiDAR系 -> 外参变换 -> IMU系(body系) -> 状态量(姿态/位置) -> 世界系
//   发布时额外支持姿态补偿变换（用于倾斜安装的无人机平台）


#include <mutex>
#include <omp.h>
#include <condition_variable>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>

#include <livox_ros_driver2/CustomMsg.h>
#include "preprocess.hpp"
#include <ikd-Tree/ikd_Tree.h>

#include "imu_processing.hpp"

#include <cmath>
#include <limits>


// === 常量和宏定义 ===
#define INIT_TIME (0.1)            // EKF初始化时间（s），系统启动后0.1秒内不进行EKF更新
#define LASER_POINT_COV (0.001)    // 激光点观测噪声协方差（约3.16cm标准差）
#define PUBFRAME_PERIOD (20)       // 每隔20帧发布一次点云（降低发布频率）


// === 时间统计变量 ===
int add_point_size = 0, kdtree_delete_counter = 0;      // 添加点数、被删除点数统计
bool pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true, speed_vector_en= true;

// === 函数声明（部分在文件中定义的外部函数）===

float res_last[100000] = {0.0};          // 残差缓存
float DET_RANGE = 300.0f;                 // 局部地图检测范围（m），决定局部地图的中心移动阈值
const float MOV_THRESHOLD = 1.5f;         // 地图滑动阈值系数
double time_diff_lidar_to_imu = 0.0;     // LiDAR到IMU的时间偏移（用于时间对齐）

// === 线程同步 ===
mutex mtx_buffer;                         // 数据buffer互斥锁
condition_variable sig_buffer;            // 条件变量：用于通知主线程有新的数据到达

string root_dir = ROOT_DIR;              // 程序根目录
string map_file_path, lid_topic, imu_topic; // 地图文件路径、LiDAR和IMU话题名

// === 时间戳和噪声参数 ===
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001; // IMU噪声协方差默认值
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int scan_count = 0, publish_count = 0;
int feats_down_size = 0, feats_undistort_size = 0, NUM_MAX_ITERATIONS = 0, pcd_save_interval = -1, pcd_index = 0;

// === 状态标志 ===
bool lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
bool indoor_env = false;                  // 室内环境标志

// === 数据和地图容器 ===
vector<BoxPointType> cub_needrm;          // 需要从ikd-Tree中删除的局部地图区域
vector<PointVector> Nearest_Points;       // 每个特征点的最近邻点集合
vector<double> extrinT(3, 0.0);           // LiDAR到IMU的外参平移（从配置读取）
vector<double> extrinR(9, 0.0);           // LiDAR到IMU的外参旋转矩阵（行优先，从配置读取）
deque<double> time_buffer;                // LiDAR时间戳队列
deque<PointCloudXYZI::Ptr> lidar_buffer;  // LiDAR点云队列
deque<sensor_msgs::Imu::ConstPtr> imu_buffer; // IMU数据队列

// === 点云指针 ===
PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());       // 从地图中提取的点云（用于发布）
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());    // 去畸变后（坐标变换后）的点云
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());    // 降采样后的单帧点云（LiDAR/body系）
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());  // 降采样后的单帧点云（世界系）

// === 体素滤波器（降采样） ===
pcl::VoxelGrid<PointType> downSizeFilterSurf;  // 对当前帧点云降采样
pcl::VoxelGrid<PointType> downSizeFilterMap;   // 对地图点降采样

// === ikd-Tree：增量KD树，存储全局地图 ===
KD_TREE<PointType> ikdtree;

// === LiDAR到IMU的外参 ===
V3D Lidar_T_wrt_IMU(Zero3d);   // 平移外参
M3D Lidar_R_wrt_IMU(Eye3d);    // 旋转外参

// === 速度向量（用于可视化） ===
V3D speed(Zero3d);

// === EKF输入输出 ===
MeasureGroup Measures;           // 当前处理的LiDAR+IMU数据包

esekfom::esekf kf;              // ESKF滤波器实例（存储待优化的24维状态量）

state_ikfom state_point;        // 当前时刻的状态量
Eigen::Vector3d pos_lid;        // 估计的LiDAR在世界系下的位置

// === ROS消息对象 ===
nav_msgs::Path path;                    // 路径消息
nav_msgs::Odometry odomAftMapped;      // 里程计消息
geometry_msgs::PoseStamped msg_body_pose; // 身体位姿消息（用于路径）
visualization_msgs::Marker marker;      // 速度可视化箭头Marker
visualization_msgs::Marker line;        // 轨迹线Marker（未使用）
geometry_msgs::Point p1,p2;             // 线段的起点和终点（未使用）

// === 预处理和IMU处理模块 ===
shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu1(new ImuProcess());

// === 信号处理函数 ===
// 捕获Ctrl+C等终止信号，安全退出程序
void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

// === 标准PointCloud2回调（用于非Livox雷达） ===
void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    mtx_buffer.lock();
    scan_count++;
    double preprocess_start_time = omp_get_wtime();

    // 检查时间戳回退（可能是rosbag循环播放）
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);                                  // 调用预处理函数转换格式
    lidar_buffer.push_back(ptr);                               // 加入LiDAR buffer
    time_buffer.push_back(msg->header.stamp.toSec());          // 加入时间戳buffer
    last_timestamp_lidar = msg->header.stamp.toSec();          // 更新上一帧时间戳
    mtx_buffer.unlock();
    sig_buffer.notify_all();                                   // 通知主线程
}

// === LiDAR-IMU时间偏移（自动同步） ===
double timediff_lidar_wrt_imu = 0.0;
bool timediff_set_flg = false;

// === Livox自定义消息回调（AVIA雷达专用） ===
void livox_pcl_cbk(const livox_ros_driver2::CustomMsg::ConstPtr &msg)
{
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count++;

    // 时间戳回退检查
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp.toSec();

    // 检查IMU-LiDAR时间同步状态
    // 如果未启用时间同步且时间差 > 10秒：报警
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty())
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n", last_timestamp_imu, last_timestamp_lidar);
    }

    // 自动时间同步：计算IMU与LiDAR之间的时间偏移
    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;  // LiDAR时间 = IMU时间 + 偏移
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);               // 调用AVIA预处理
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);

    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

// === IMU数据回调 ===
void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    publish_count++;

    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    // 时间同步补偿：如果自动同步模式，调整IMU时间戳
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp =
            ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
    }

    // 应用固定的LiDAR-IMU时间偏移
    msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec() - time_diff_lidar_to_imu);

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();

    // 时间戳回退检查
    if (timestamp < last_timestamp_imu)
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;
    imu_buffer.push_back(msg);  // 加入IMU buffer

    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

// === LiDAR扫描时间统计 ===
double lidar_mean_scantime = 0.0;  // 平均扫描时间（ms）
int scan_num = 0;                   // 扫描帧计数

// === sync_packages：数据同步 ===
// 功能：从buffer中取出一帧LiDAR数据和对应的IMU数据，打包成MeasureGroup
// 返回false表示数据不足，返回true表示打包成功、可以进行下一步处理
bool sync_packages(MeasureGroup &meas)
{
    // 没有LiDAR或IMU数据则等待
    if (lidar_buffer.empty() || imu_buffer.empty())
    {
        return false;
    }

    // --- 步骤1：取出一帧LiDAR ---
    if (!lidar_pushed)  // 如果还没取LiDAR帧
    {
        meas.lidar = lidar_buffer.front();          // 取队列最前面的LiDAR帧
        meas.lidar_beg_time = time_buffer.front();  // 记录帧起始时间

        // 估算帧结束时间
        if (meas.lidar->points.size() <= 5)
        {
            // 点太少，用平均帧时间估算结束时间
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            ROS_WARN("Too few input point cloud!\n");
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            // 最后一个点的时间（秒）太短，用平均时间
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            // 正常情况：帧结束时间 = 起始时间 + 最后一个点的偏移时间
            scan_num++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            // 更新平均扫描时间（指数滑动平均）
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;
        lidar_pushed = true;  // 标记已取帧
    }

    // --- 步骤2：等待并取出该帧时间内的IMU数据 ---
    if (last_timestamp_imu < lidar_end_time)
    {
        return false;  // IMU数据还没覆盖到LiDAR帧结束时间，等待
    }

    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();

    // 取出所有时间戳 < LiDAR帧结束时间 的IMU数据
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if (imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    // 清理已处理的LiDAR数据
    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;  // 重置标记，准备处理下一帧
    return true;
}

// === pointBodyToWorld：Body系(IMU/LiDAR系)点到世界系的坐标变换 ===
// 变换公式: p_world = R * (R_ext * p_body + T_ext) + pos
void pointBodyToWorld(PointType const *const pi, PointType *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot.matrix() * (state_point.offset_R_L_I.matrix() * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

// === 模板版本的pointBodyToWorld（用于Eigen T类型） ===
template <typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    Eigen::Matrix3d R_1;
    R_1<<1,0,0,
                 0,-1,0,
                 0,0,-1;
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot.matrix() * (state_point.offset_R_L_I.matrix() * p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

// === lasermap_fov_segment：局部地图FOV管理 ===
// 功能：根据当前LiDAR位置动态调整局部地图范围
// 原理：
//   1. 初始时以当前位置为中心建立 cube_len × cube_len 的局部地图
//   2. 当LiDAR接近局部地图边界时（距离 < MOV_THRESHOLD * DET_RANGE），
//      沿对应方向移动地图，并删除移动后超出范围的旧区域
// 这样保证了ikd-Tree中只保留当前感兴区域内的地图点，提高效率
BoxPointType LocalMap_Points;       // 局部地图范围
bool Localmap_Initialized = false;  // 局部地图是否已初始化

void lasermap_fov_segment()
{
    cub_needrm.clear();         // 清空需要删除的区域列表
    kdtree_delete_counter = 0;

    V3D pos_LiD = pos_lid;     // LiDAR在世界系下的位置

    // --- 初始化局部地图 ---
    if (!Localmap_Initialized)
    {
        for (int i = 0; i < 3; i++)
        {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }

    // --- 检查是否需要移动局部地图 ---
    float dist_to_map_edge[3][2];  // LiDAR到地图6个面的距离
    bool need_move = false;
    for (int i = 0; i < 3; i++)
    {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);  // 到最小面的距离
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);  // 到最大面的距离

        // 如果任一方向距离 < MOV_THRESHOLD * DET_RANGE，需要移动地图
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            need_move = true;
    }
    if (!need_move) return;  // 不需要移动

    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;

    // 计算移动距离
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD - 1)));
    for (int i = 0; i < 3; i++)
    {
        tmp_boxpoints = LocalMap_Points;

        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE)  // 接近最小面
        {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;        // 地图向负方向移动
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;  // 计算被移出的区域
            cub_needrm.push_back(tmp_boxpoints);                  // 记录需要删除的区域
        }
        else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)  // 接近最大面
        {
            New_LocalMap_Points.vertex_max[i] += mov_dist;        // 地图向正方向移动
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;  // 更新局部地图范围

    // 获取之前删除的点（用于后续可能的回环检测等）
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);

    // 从ikd-Tree中删除移出局部地图范围的点
    if (cub_needrm.size() > 0)
        kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
}

// === RGBpointBodyLidarToIMU：LiDAR系 -> IMU系坐标变换（用于发布body系点云） ===
void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I.matrix() * p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

// === map_incremental：增量地将当前帧点云加入ikd-Tree全局地图 ===
// 策略：对于每个特征点
//   1. 转换到世界坐标系
//   2. 在ikd-Tree体素内已有点时，选择距离体素中心最近的点保留
//   3. 降采样后再添加到地图
void map_incremental()
{
    PointVector PointToAdd;               // 需要降采样后添加的点
    PointVector PointNoNeedDownsample;   // 不需要降采样的点（最近邻在体素外）
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);

    for (int i = 0; i < feats_down_size; i++)
    {
        // 步骤1：转换到世界坐标系
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));

        // 步骤2：降采样策略判断
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];  // 该点在地图中的最近邻
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType mid_point;  // 当前点所在体素的中心

            // 计算当前点所在体素的中心坐标
            mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;

            float dist = calc_dist(feats_down_world->points[i], mid_point);  // 当前点到体素中心的距离

            // 如果最近邻与当前点不在同一体素 -> 直接添加（不降采样）
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min)
            {
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }

            // 如果体素内已有更近的点 -> 不添加（保持降采样）
            for (int j = 0; j < NUM_MATCH_POINTS; j++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;

                if (calc_dist(points_near[j], mid_point) < dist)  // 已有更近的点
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);  // EKF未初始化或没有近邻点时直接添加
        }
    }

    // 添加到ikd-Tree
    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);           // 需要降采样的点
    ikdtree.Add_Points(PointNoNeedDownsample, false);                 // 不需要降采样的点
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
}

// === publish_frame_world：发布世界坐标系下的当前帧点云 ===
PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));  // 点云发布缓存
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());           // PCD保存缓存

void publish_frame_world(const ros::Publisher &pubLaserCloudFull_)
{
    if (scan_pub_en)
    {
        // dense_pub_en为true时发布原始去畸变点云，否则发布降采样点云
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        // 使用与建图和里程计相同的世界坐标系发布点云。
        for (int i = 0; i < size; i++)
        {
            pointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
        }

        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "world";
        pubLaserCloudFull_.publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    // --- PCD保存模式 ---
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            pointBodyToWorld(&feats_undistort->points[i], &laserCloudWorld->points[i]);
        }

        static int scan_wait_num = 0;
        scan_wait_num++;

        // 每4帧合并一次点云
        if (scan_wait_num % 4 == 0)
            *pcl_wait_save += *laserCloudWorld;

        // 达到保存间隔时写入PCD文件
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
        {
            pcd_index++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

// === publish_frame_body：发布IMU坐标系(body系)下的当前帧点云 ===
void publish_frame_body(const ros::Publisher &pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

// === publish_frame_lidar：发布LiDAR坐标系下的原始点云（去畸变后） ===
void publish_frame_lidar(const ros::Publisher &pubLaserCloudFull_lidar)
{
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*feats_undistort, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "lidar";
    pubLaserCloudFull_lidar.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

// === publish_map：发布当前全局地图（ikd-Tree展平后的点云） ===
void publish_map(const ros::Publisher &pubLaserCloudMap)
{
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudMap.header.frame_id = "world";
    pubLaserCloudMap.publish(laserCloudMap);
}

// === set_posestamp：设置里程计消息中的位置和姿态 ===
// 包含姿态补偿：R_3 * R_roll * R_pitch 坐标系调整

static bool set_quaternion_msg(const Eigen::Matrix3d &rot, geometry_msgs::Quaternion &orientation)
{
    Eigen::Quaterniond q(rot);
    const double norm = q.norm();
    if (!std::isfinite(norm) || norm < 1e-12)
    {
        ROS_WARN_THROTTLE(1.0, "Invalid orientation quaternion, use identity instead");
        orientation.x = 0.0;
        orientation.y = 0.0;
        orientation.z = 0.0;
        orientation.w = 1.0;
        return false;
    }

    q.normalize();
    orientation.x = q.x();
    orientation.y = q.y();
    orientation.z = q.z();
    orientation.w = q.w();
    return true;
}

// 输出 RA-LIO 的世界坐标状态。雷达安装外参已在滤波器内部处理，
// 此处不能再次施加固定安装角补偿。
template <typename T>
void set_posestamp(T &out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    set_quaternion_msg(state_point.rot.matrix(), out.pose.orientation);
}

// === set_twiststamp：设置里程计消息中的线速度 ===
template <typename T>
void set_twiststamp(T &twi)
{
    twi.twist.linear.x = state_point.vel(0);
    twi.twist.linear.y = state_point.vel(1);
    twi.twist.linear.z = state_point.vel(2);
}

// === set_pathstamp：设置路径消息中的位姿（不带姿态补偿，直接使用原始状态） ===
template <typename T>
void set_pathstamp(T &out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);

    set_quaternion_msg(state_point.rot.matrix(), out.pose.orientation);
}

// === publish_odometry：发布里程计Odometry消息 ===
// 功能：
//   1. 发布Odometry消息（位置、姿态、线速度、协方差）
//   2. 发布TF变换（world -> camera_init -> body 的坐标变换链）
void publish_odometry(const ros::Publisher &pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "world";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose);      // 设置位置和姿态（带补偿）
    set_twiststamp(odomAftMapped.twist);    // 设置线速度（带补偿）

    pubOdomAftMapped.publish(odomAftMapped);

    // --- 填充协方差矩阵 ---
    // 从EKF协方差P中提取位置和姿态的交叉协方差（6x6块）
    auto P = kf.get_P();
    for (int i = 0; i < 6; i++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
    }

    // --- 发布 body -> world 的TF变换 ---
    // TF树: world -> camera_init -> body
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x,
                                    odomAftMapped.pose.pose.position.y,
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "body"));

    // --- 发布 world -> camera_init 的静态TF变换（identity，即同一个坐标系） ---
    static tf::TransformBroadcaster br_world;
    transform.setOrigin(tf::Vector3(0, 0, 0));
    q.setW(1); q.setX(0); q.setY(0); q.setZ(0);
    transform.setRotation(q);
    br_world.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "world", "camera_init"));
}

// === publish_path：发布运动轨迹Path ===
// 每5帧发布一次（降低发布频率，避免rviz崩溃）
void publish_path(const ros::Publisher pubPath)
{
    set_pathstamp(msg_body_pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "world";

    static int jjj = 0;
    jjj++;

    if (jjj % 5 == 0)  // 每5帧发布一次路径
    {
        path.header.stamp = msg_body_pose.header.stamp;
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
    }
}

// === Visualization_speed：发布速度向量可视化Marker ===
// 在世界坐标系中显示一个箭头，表示当前运动方向
static void visualization_speed(const ros::Publisher &marker_pub) {
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.lifetime = ros::Duration();

  marker.ns = "speed";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW; // 箭头类型
  marker.action = visualization_msgs::Marker::ADD; // 添加操作

  marker.scale.x = 2.0; // 箭头大小
  marker.scale.y = 2.0;
  marker.scale.z = 3;

  marker.color.r = 1.0f; // 颜色：品红
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  // 设置marker的位置和姿态
  marker.pose.position = odomAftMapped.pose.pose.position;
  set_quaternion_msg(kf.get_x().rot.matrix(), marker.pose.orientation);

  marker_pub.publish(marker);
}

// =================================================================
// === main：主函数 ===
// =================================================================
int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserMapping");  // 初始化ROS节点
    ros::NodeHandle nh;

    // === 加载配置参数 ===
    nh.param<bool>("publish/path_en", path_en, true);
    nh.param<bool>("publish/speed_vector_en", speed_vector_en, true);
    nh.param<bool>("publish/scan_publish_en", scan_pub_en, true);
    nh.param<bool>("publish/dense_publish_en", dense_pub_en, true);
    nh.param<bool>("publish/scan_bodyframe_pub_en", scan_body_pub_en, true);
    nh.param<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
    nh.param<string>("map_file_path", map_file_path, "");
    nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");
    nh.param<string>("common/imu_topic", imu_topic, "/livox/imu");
    nh.param<bool>("common/time_sync_en", time_sync_en, false);
    nh.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
    nh.param<double>("filter_size_corner", filter_size_corner_min, 0.5);
    nh.param<double>("filter_size_surf", filter_size_surf_min, 0.5);
    nh.param<double>("filter_size_map", filter_size_map_min, 0.5);
    nh.param<double>("cube_side_length", cube_len, 200);
    nh.param<float>("mapping/det_range", DET_RANGE, 300.f);
    nh.param<double>("mapping/fov_degree", fov_deg, 180);
    nh.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
    nh.param<double>("mapping/acc_cov", acc_cov, 0.1);
    nh.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
    nh.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
    nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);
    nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, false);
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());

    cout << "Lidar_type: " << p_pre->lidar_type << endl;

    // 初始化路径header
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "camera_init";

    // === 订阅和发布 ===
    // 根据雷达类型选择不同的回调函数
    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);

    ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);          // 世界系去畸变点云
    ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000); // Body(IMU)系点云
    ros::Publisher pubLaserCloudFull_lidar = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_registered_lidar", 100000); // LiDAR系点云
    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100000);           // 有效特征点云
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100000);                  // 全局地图
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);                         // 里程计
    ros::Publisher pubPath = nh.advertise<nav_msgs::Path>("/path", 100000);                                          // 轨迹
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/speed_vector", 10);                       // 速度可视化

    // === 初始化外参和IMU处理参数 ===
    Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);    // 平移外参
    Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);    // 旋转外参

    // 设置IMU处理模块参数：外参、噪声协方差
    p_imu1->set_param(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU, V3D(gyr_cov, gyr_cov, gyr_cov), V3D(acc_cov, acc_cov, acc_cov),
                      V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov), V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    // 注册信号处理器（Ctrl+C安全退出）
    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);  // 5kHz主循环频率

    // ============================================================
    // === 主循环 ===
    // ============================================================
    while (ros::ok())
    {
        if (flg_exit) break;    // 收到退出信号
        ros::spinOnce();         // 处理一次ROS回调（收集数据）

        // --- 数据同步和打包 ---
        if (sync_packages(Measures))  // 成功打包一帧LiDAR+对应的IMU数据
        {
            double t00 = omp_get_wtime();  // 计时开始

            // 设置体素滤波器参数
            downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
            downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);

            // 跳过第一帧（仅用于初始化IMU）
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu1->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                continue;
            }

            // --- 步骤1：IMU处理（初始化和点云"去畸变"） ---
            p_imu1->Process(Measures, kf, feats_undistort);

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            // --- 步骤2：获取当前EKF状态 ---
            state_point = kf.get_x();
            // 计算LiDAR在世界系的位置：P_lidar_w = pos + R * T_ext
            pos_lid = state_point.pos + state_point.rot.matrix() * state_point.offset_T_L_I;

            // EKF初始化检查：启动后INIT_TIME(0.1s)内不进行EKF更新
            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;

            // --- 步骤3：局部地图FOV管理 ---
            lasermap_fov_segment();

            // --- 步骤4：降采样当前帧点云 ---
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            feats_undistort_size = feats_undistort->points.size();
            feats_down_size = feats_down_body->points.size();

            if (feats_down_size < 5)
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            // --- 步骤5：首次构建ikd-Tree ---
            // ikd-Tree为空时，直接用第一帧构建（跳过ESKF更新）
            if (ikdtree.Root_Node == nullptr)
            {
                ikdtree.set_downsample_param(filter_size_map_min);
                feats_down_world->resize(feats_down_size);
                for (int i = 0; i < feats_down_size; i++)
                {
                    pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                }
                ikdtree.Build(feats_down_world->points);  // 根据世界系点云构建ikd-Tree
                continue;
            }

            // --- 步骤6（可选）：查看全局地图 ---
            // 将ikd-Tree展平为点云，用于发布全局地图
            if (0) // 改为 if(1) 来启用地图发布
            {
                PointVector().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }

            // --- 步骤7：ESKF迭代更新 ---
            // 核心步骤：利用点到平面残差进行迭代卡尔曼滤波
            Nearest_Points.resize(feats_down_size);
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, feats_down_body, ikdtree, Nearest_Points, NUM_MAX_ITERATIONS, extrinsic_est_en);

            // --- 步骤8：更新状态 ---
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot.matrix() * state_point.offset_T_L_I;

            std::printf("6\n");

            // --- 步骤9：发布里程计 ---
            publish_odometry(pubOdomAftMapped);

            // --- 步骤10：地图增量 ---
            // 将当前帧点云加入全局ikd-Tree地图
            feats_down_world->resize(feats_down_size);
            map_incremental();

            // --- 步骤11：发布路径 ---
            if (path_en) publish_path(pubPath);

            // --- 步骤12：发布速度向量可视化 ---
            if(speed_vector_en)
            {
                speed(0) = kf.get_x().vel(0);
                speed(1) = kf.get_x().vel(1);
                speed(2) = kf.get_x().vel(2);
                visualization_speed(marker_pub);
            }

            // --- 步骤13：发布点云 ---
            if (scan_pub_en || pcd_save_en) publish_frame_world(pubLaserCloudFull);
            if (scan_pub_en && scan_body_pub_en)
            {
                publish_frame_body(pubLaserCloudFull_body);
                publish_frame_lidar(pubLaserCloudFull_lidar);
            }

            // 计时和日志输出
            double t11 = omp_get_wtime();
            std::cout << "feats_down_size: " << feats_down_size << "  Whole mapping time(ms):  " << (t11 - t00) * 1000 << std::endl << std::endl;
            std::cout << "feats_undistort_size: " << feats_undistort_size  << std::endl;
        }

        rate.sleep();
    }

    // === 程序退出时保存最后的PCD地图 ===
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("GlobalMap.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name << endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    return 0;
}
