#include <omp.h>
#include <mutex>
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

#include <livox_ros_driver/CustomMsg.h>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>

#include "IMU_Processing.hpp"

#include <cmath>
#define INIT_TIME (0.1)
#define LASER_POINT_COV (0.001)
#define PUBFRAME_PERIOD (20)
#define PI 3.1415926
/*** Time Log Variables ***/
int add_point_size = 0, kdtree_delete_counter = 0;
bool pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true, speed_vector_en= true;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;
double theta = 0.0;
double alpha = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int scan_count = 0, publish_count = 0;
int feats_down_size = 0, feats_undistort_size = 0, NUM_MAX_ITERATIONS = 0, pcd_save_interval = -1, pcd_index = 0;

bool lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
bool indoor_env = false;

vector<BoxPointType> cub_needrm;
vector<PointVector> Nearest_Points;
vector<double> extrinT(3, 0.0);
vector<double> extrinR(9, 0.0);
deque<double> time_buffer;
deque<PointCloudXYZI::Ptr> lidar_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());  //畸变纠正后降采样的单帧点云，lidar系
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI()); //畸变纠正后降采样的单帧点云，W系

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;

V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

V3D speed(Zero3d);
/*** EKF inputs and output ***/
MeasureGroup Measures;

esekfom::esekf kf; //kf会存储待优化的状态量

state_ikfom state_point;
Eigen::Vector3d pos_lid; //估计的W系下的位置

nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::PoseStamped msg_body_pose;
visualization_msgs::Marker marker;
visualization_msgs::Marker line;
geometry_msgs::Point p1,p2;  //p1起点，p2终点

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu1(new ImuProcess());




//signal信号处理函数，终止ROS
void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    mtx_buffer.lock(); //互斥锁锁上
    scan_count++; //处理帧计数
    double preprocess_start_time = omp_get_wtime(); 
    if (msg->header.stamp.toSec() < last_timestamp_lidar) //判断当前时间戳是否小于上一帧时间戳，如果小于则报错
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());//定义新的点云指针Ptr
    p_pre->process(msg, ptr);//通过process函数将msg的格式转换为pcl格式，在preprocess.cpp中
    lidar_buffer.push_back(ptr);  //将pcl格式点云push进入buffer队列中
    time_buffer.push_back(msg->header.stamp.toSec());//把时间戳放入time buffer中
    last_timestamp_lidar = msg->header.stamp.toSec();//把当前帧时间戳置为上一帧时间戳
    mtx_buffer.unlock();//互斥锁解锁
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool timediff_set_flg = false;
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count++;
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp.toSec();

    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty())
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n", last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);

    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
	//用于计数
    publish_count++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
	//定义了一个指向IMU消息msg_in的指针msg
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
	
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp =
            ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
    }

    msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec() - time_diff_lidar_to_imu);


    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;
	//将该帧数据msg push进入buffer队列
    imu_buffer.push_back(msg);

    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int scan_num = 0;


bool sync_packages(MeasureGroup &meas)
{

    if (lidar_buffer.empty() || imu_buffer.empty())
    {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed) //如果Lidar还没有被push
    {
		
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();

		//如果点太少，直接用平均帧时间计算结束时间
        if (meas.lidar->points.size() <= 5) 
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            ROS_WARN("Too few input point cloud!\n");
        }
		//如果time太短，直接用平均帧时间计算结束时间
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }

        else
        {
            scan_num++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;  //注意curvature中存储的是相对第一个点的时间
        }

        meas.lidar_end_time = lidar_end_time;
		//处理过后置为true
        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)  
    {
        return false;
    }


    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear(); //清空imu

	while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if (imu_time > lidar_end_time)
            break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }
	
    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

void pointBodyToWorld(PointType const *const pi, PointType *const po)
{


    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot.matrix() * (state_point.offset_R_L_I.matrix() * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void pointComp(PointType const *const pi1, PointType *const po1)
{

    Eigen::Matrix3d R_3;
    R_3<<1,0,0,
                 0,-1,0,
                 0,0,-1;


    Eigen::Matrix3d R_compensate_roll = Eigen::AngleAxisd(alpha*PI/180.0, Eigen::Vector3d(1,0,0)).toRotationMatrix();
    Eigen::Matrix3d R_compensate3 = Eigen::AngleAxisd(theta*PI/180.0, Eigen::Vector3d(0,1,0)).toRotationMatrix();
    Eigen::Vector3d Pos_w(state_point.pos(0),  state_point.pos(1), state_point.pos(2));
    Eigen::Vector3d Pos_compensate3= R_3*R_compensate_roll*R_compensate3* Pos_w;
    Eigen::Matrix3d rot_comp = R_3*R_compensate_roll*R_compensate3*state_point.rot.matrix();



    V3D p_body1(pi1->x, pi1->y, pi1->z);
    V3D p_global1(rot_comp * (state_point.offset_R_L_I.matrix() * p_body1 + state_point.offset_T_L_I) + Pos_compensate3);

    po1->x = p_global1(0);
    po1->y = p_global1(1);
    po1->z = p_global1(2);
    po1->intensity = pi1->intensity;
}









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

BoxPointType LocalMap_Points;      
bool Localmap_Initialized = false; 

void lasermap_fov_segment()
{
    cub_needrm.clear(); // 清空需要移除的区域
    kdtree_delete_counter = 0; 

    V3D pos_LiD = pos_lid; 

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


    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++)
    {

        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);

        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            need_move = true;
    }
    if (!need_move)
        return; 


    BoxPointType New_LocalMap_Points, tmp_boxpoints;

    New_LocalMap_Points = LocalMap_Points;

    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD - 1)));
    for (int i = 0; i < 3; i++)
    {
		
        tmp_boxpoints = LocalMap_Points;

        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }

        else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);


    if (cub_needrm.size() > 0)
        kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm); 
}

void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I.matrix() * p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}


void map_incremental()
{
    PointVector PointToAdd;  
    PointVector PointNoNeedDownsample; //不需要降采样的点
    PointToAdd.reserve(feats_down_size); //resize成当前点的数量
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {

        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));

        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType mid_point; //点所在体素的中心

            mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
			
            float dist = calc_dist(feats_down_world->points[i], mid_point);

            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min)
            {
                PointNoNeedDownsample.push_back(feats_down_world->points[i]); 
                continue;
            }

            for (int j = 0; j < NUM_MATCH_POINTS; j++)
            {

                if (points_near.size() < NUM_MATCH_POINTS)
                    break;

                if (calc_dist(points_near[j], mid_point) < dist) 
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add)
                PointToAdd.push_back(feats_down_world->points[i]);
        }

        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(const ros::Publisher &pubLaserCloudFull_)
{
    if (scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(
            new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            pointComp(&laserCloudFullRes->points[i],
                             &laserCloudWorld->points[i]);
        }

        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        //laserCloudmsg.header.frame_id = "camera_init";
        laserCloudmsg.header.frame_id = "world";
        pubLaserCloudFull_.publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }


    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(
            new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            pointBodyToWorld(&feats_undistort->points[i],
                             &laserCloudWorld->points[i]);
        }

        static int scan_wait_num = 0;
        scan_wait_num++;

        if (scan_wait_num % 4 == 0)
            *pcl_wait_save += *laserCloudWorld;

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

void publish_frame_body(const ros::Publisher &pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i],
                               &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_frame_lidar(const ros::Publisher &pubLaserCloudFull_lidar)
{
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*feats_undistort, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "lidar";
    pubLaserCloudFull_lidar.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}



void publish_map(const ros::Publisher &pubLaserCloudMap)
{
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudMap.header.frame_id = "world";
    pubLaserCloudMap.publish(laserCloudMap);
}

template <typename T>
void set_posestamp(T &out)
{
    Eigen::Matrix3d R_0;
    R_0<<1,0,0,
                 0,-1,0,
                 0,0,-1;

   
    Eigen::Matrix3d R_compensate_roll1 = Eigen::AngleAxisd(alpha*PI/180.0, Eigen::Vector3d(1,0,0)).toRotationMatrix();

    Eigen::Matrix3d R_compensate = Eigen::AngleAxisd(theta*PI/180.0, Eigen::Vector3d(0,1,0)).toRotationMatrix();

    Eigen::Vector3d Pos_(state_point.pos(0),  state_point.pos(1), state_point.pos(2));
    Eigen::Vector3d Pos_compensate= R_0*R_compensate_roll1*R_compensate* Pos_;

 
    out.pose.position.x =Pos_compensate(0);
    out.pose.position.y =Pos_compensate(1);
    out.pose.position.z =Pos_compensate(2); 




    auto q_ = Eigen::Quaterniond(R_0*R_compensate_roll1*R_compensate*state_point.rot.matrix()*(R_0*R_compensate_roll1*R_compensate).inverse());
    //auto q_ = Eigen::Quaterniond(state_point.rot.matrix());
    out.pose.orientation.x = q_.coeffs()[0];
    out.pose.orientation.y = q_.coeffs()[1];
    out.pose.orientation.z = q_.coeffs()[2];
    out.pose.orientation.w = q_.coeffs()[3];



    
    
}

template <typename T>
void set_twiststamp(T &twi)
{
    Eigen::Matrix3d R_1;
    R_1<<1,0,0,
                 0,-1,0,
                 0,0,-1;

    Eigen::Matrix3d R_compensate_roll2 = Eigen::AngleAxisd(alpha*PI/180.0, Eigen::Vector3d(1,0,0)).toRotationMatrix();
    Eigen::Matrix3d R_compensate1 = Eigen::AngleAxisd(theta*PI/180.0, Eigen::Vector3d(0,1,0)).toRotationMatrix();

    Eigen::Vector3d Vel_(state_point.vel(0),  state_point.vel(1), state_point.vel(2));
    Eigen::Vector3d Vel_compensate= R_1*R_compensate_roll2*R_compensate1* Vel_;


    twi.twist.linear.x = Vel_compensate(0);
    twi.twist.linear.y = Vel_compensate(1);
    twi.twist.linear.z = Vel_compensate(2);

}

template <typename T>
void set_pathstamp(T &out)
{

    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);

    //auto q_ = Eigen::Quaterniond(R_0*R_compensate*state_point.rot.matrix());
    auto q_ = Eigen::Quaterniond(state_point.rot.matrix());
    out.pose.orientation.x = q_.coeffs()[0];
    out.pose.orientation.y = q_.coeffs()[1];
    out.pose.orientation.z = q_.coeffs()[2];
    out.pose.orientation.w = q_.coeffs()[3];
    
}


void publish_odometry(const ros::Publisher &pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "world";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);//将Lidar帧结束时间作为时间戳
    set_posestamp(odomAftMapped.pose);  
    set_twiststamp(odomAftMapped.twist);

    pubOdomAftMapped.publish(odomAftMapped);
    //std::cout << "z axis: " << odomAftMapped.pose.pose.position.z  << std::endl;
                      

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
	//下面为发布tf，把位姿变换关系发布出去
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

    
    /*Eigen::Matrix3d R_2;
    R_2<<1,0,0,
                 0,-1,0,
                 0,0,-1;

   
    Eigen::Matrix3d R_compensate2 = Eigen::AngleAxisd(16.7637*PI/180.0, Eigen::Vector3d(0,1,0)).toRotationMatrix();
    auto q_1 = Eigen::Quaterniond(R_2*R_compensate2);*/


    static tf::TransformBroadcaster br_world;
    transform.setOrigin(tf::Vector3(0, 0, 0));
    //q.setValue(0, 0, 0, 1);
    q.setW(1);
    q.setX(0);
    q.setY(0);
    q.setZ(0);

    //q.setValue(p_imu1->wq(0), p_imu1->wq(1), p_imu1->wq(2), p_imu1->wq(3));
    //q.setValue(q_1.coeffs()[0], q_1.coeffs()[1], q_1.coeffs()[2], q_1.coeffs()[3]);
    
    transform.setRotation(q);
    br_world.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "world", "camera_init"));

    // //WAB
    // transform.setRotation(q);
    // br_world.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "world", "camera_init_temp"));

    // transform.setRotation(q);
    // br_world.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "camera_init_temp"));

}

void publish_path(const ros::Publisher pubPath)
{
    set_pathstamp(msg_body_pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    //msg_body_pose.header.frame_id = "camera_init";
    msg_body_pose.header.frame_id = "world";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;

    if (jjj % 5 == 0)
    {

        path.header.stamp = msg_body_pose.header.stamp;
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
    }
}


void Visualization_speed(const ros::Publisher &marker_pub)
{

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.lifetime = ros::Duration();   

    marker.ns = "speed";
    marker.id = 0;
    marker.type =  visualization_msgs::Marker::ARROW;  //箭头
    marker.action = visualization_msgs::Marker::ADD; //添加

    marker.scale.x = 2.0;  
    marker.scale.y = 2.0;
    marker.scale.z = 3;

    marker.color.r = 1.0f;   //颜色
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;


    Eigen::Matrix3d R_compensate = Eigen::AngleAxisd(13.5647*PI/180.0, Eigen::Vector3d(0,1,0)).toRotationMatrix();
    Eigen::Vector3d Pos_(odomAftMapped.pose.pose.position.x, odomAftMapped.pose.pose.position.y, odomAftMapped.pose.pose.position.z);
    Eigen::Vector3d Pos_compensate= R_compensate* Pos_;
    Eigen::Matrix3d Atti_compensate= R_compensate* kf.get_x().rot.matrix();
/*     std::cout << "a: " << Pos_compensate(0) << std::endl;
    std::cout << "b: " << Pos_compensate(1) << std::endl;
    std::cout << "c: " << Pos_compensate(2) << std::endl; */


    //下面为速度矢量marker的位姿
    marker.pose.position.x=Pos_compensate(0);
    marker.pose.position.y=Pos_compensate(1);
    marker.pose.position.z=Pos_compensate(2);

    Atti_compensate=Atti_compensate.transpose();
    auto sq = Eigen::Quaterniond(Atti_compensate);

    marker.pose.orientation.w = sq.coeffs()[0];
    marker.pose.orientation.x = sq.coeffs()[1];
    marker.pose.orientation.y = sq.coeffs()[2];
    marker.pose.orientation.z = sq.coeffs()[3]; 



/*     double sx=speed(0)*cos(13.5647*PI/180.0)+speed(2)*sin(13.5647*PI/180.0);
    double sy=speed(1);
    double sz=speed(2)*cos(13.5647*PI/180.0)+speed(0)*sin(13.5647*PI/180.0);
    Eigen::Vector3d S;
    S << sx, sy, sz;
    S.normalize();
    Eigen::Matrix3d Rs;
    Rs << S(0), S(1), S(2), 0, 0, 0, 0, 0, 0;
-
    auto sq=Eigen::Quaterniond(Rs);

    marker.pose.orientation.w = sq.coeffs()[0];
    marker.pose.orientation.x = sq.coeffs()[1];
    marker.pose.orientation.y = sq.coeffs()[2];
    marker.pose.orientation.z = sq.coeffs()[3]; */






    
    marker_pub.publish(marker) ;



}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserMapping");  //初始化节点
    ros::NodeHandle nh;
	//下面是来自launch和config里面的参数
    nh.param<bool>("publish/path_en", path_en, true);
    nh.param<bool>("publish/speed_vector_en", speed_vector_en, true);
    nh.param<bool>("publish/scan_publish_en", scan_pub_en, true);            // 是否发布当前正在扫描的点云的topic
    nh.param<bool>("publish/dense_publish_en", dense_pub_en, true);          // 是否发布经过运动畸变校正注册到IMU坐标系的点云的topic
    nh.param<bool>("publish/scan_bodyframe_pub_en", scan_body_pub_en, true); // 是否发布经过运动畸变校正注册到IMU坐标系的点云的topic，需要该变量和上一个变量同时为true才发布
    nh.param<int>("max_iteration", NUM_MAX_ITERATIONS, 4);                   // 卡尔曼滤波的最大迭代次数
    nh.param<string>("map_file_path", map_file_path, "");                    // 地图保存路径
    nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");         // 雷达点云topic名称
    nh.param<string>("common/imu_topic", imu_topic, "/livox/imu");           // IMU的topic名称
    nh.param<bool>("common/time_sync_en", time_sync_en, false);              // 是否需要时间同步，只有当外部未进行时间同步时设为true
    nh.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
    nh.param<double>("filter_size_corner", filter_size_corner_min, 0.5); // VoxelGrid降采样时的体素大小
    nh.param<double>("filter_size_surf", filter_size_surf_min, 0.5);
    nh.param<double>("filter_size_map", filter_size_map_min, 0.5);
    nh.param<double>("cube_side_length", cube_len, 200);    // 
    nh.param<float>("mapping/det_range", DET_RANGE, 300.f); // 激光雷达的最大探测范围，与局部地图更新的范围有关系
    nh.param<double>("mapping/fov_degree", fov_deg, 180);
    nh.param<double>("mapping/gyr_cov", gyr_cov, 0.1);               // IMU陀螺仪的协方差
    nh.param<double>("mapping/acc_cov", acc_cov, 0.1);               // IMU加速度计的协方差
    nh.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);        // IMU陀螺仪偏置的协方差
    nh.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);        // IMU加速度计偏置的协方差
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);        // 最小距离阈值，即过滤掉0～blind范围内的点云
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA); // 激光雷达的类型
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);       // 激光雷达扫描的线数（livox avia为6线）
    nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
    nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);           // 采样间隔，即每隔point_filter_num个点取1个点
    nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, false); // 是否提取特征点（FAST_LIO2默认不进行特征点提取）
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false); // 是否将点云地图保存到PCD文件
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>()); // 雷达相对于IMU的外参T（即雷达在IMU坐标系中的坐标）
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>()); // 雷达相对于IMU的外参R

    cout << "Lidar_type: " << p_pre->lidar_type << endl;

    path.header.stamp = ros::Time::now();
    path.header.frame_id = "camera_init";


    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);

    ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
    ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);
    ros::Publisher pubLaserCloudFull_lidar = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_registered_lidar", 100000);

    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100000);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100000);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
    ros::Publisher pubPath = nh.advertise<nav_msgs::Path>("/path", 100000);

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/speed_vector", 10);

    Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);

    p_imu1->set_param(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU, V3D(gyr_cov, gyr_cov, gyr_cov), V3D(acc_cov, acc_cov, acc_cov),
                      V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov), V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    signal(SIGINT, SigHandle); //当程序检测到signal信号（例如ctrl+c） 时  执行 SigHandle 函数，离开ros，停止程序。
    ros::Rate rate(5000);

    while (ros::ok())
    {
        if (flg_exit)  //收到signal信号后会被置为true
            break;
        ros::spinOnce();  //每次收到消息都要去执行回调函数


        if (sync_packages(Measures))  //打包完成后
        {
            double t00 = omp_get_wtime();
            downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
            downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);


            if (flg_first_scan)  //是否是第一帧
            {
                first_lidar_time = Measures.lidar_beg_time;  
                p_imu1->first_lidar_time = first_lidar_time;
                flg_first_scan = false;  //因为仅有一帧是无法进行运算的
                continue;
            }


			
            p_imu1->Process(Measures, kf, feats_undistort);


            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            state_point = kf.get_x();

            pos_lid = state_point.pos + state_point.rot.matrix() * state_point.offset_T_L_I;

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;


            lasermap_fov_segment(); 


            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            feats_undistort_size = feats_undistort->points.size();
            feats_down_size = feats_down_body->points.size();


            if (feats_down_size < 5)
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }


            if (ikdtree.Root_Node == nullptr)
            {
                ikdtree.set_downsample_param(filter_size_map_min);
                feats_down_world->resize(feats_down_size);  
                for (int i = 0; i < feats_down_size; i++)
                {
                    pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i])); // lidar坐标系的点转到世界坐标系
                }
                ikdtree.Build(feats_down_world->points); //根据世界坐标系下的点Build构建ikdtree
                continue;
            }

            if (0) // If you need to see map point, change to "if(1)"，是否要看全局地图
            {
                PointVector().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD); //PCL_Storage为存储的全局地图
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
                // std::cout << "ikdtree size: " << featsFromMap->points.size() << std::endl;
            }

            Nearest_Points.resize(feats_down_size); 
			//kf下的成员函数：ESKF迭代更新主函数
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, feats_down_body, ikdtree, Nearest_Points, NUM_MAX_ITERATIONS, extrinsic_est_en);
			 
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot.matrix() * state_point.offset_T_L_I;


            //std::printf(" vel: %.4f %.4f %.4f\n", kf.get_x().vel(0), kf.get_x().vel(1), kf.get_x().vel(2));
            //std::printf(" bg: %.4f %.4f %.4f\n", kf.get_x().bg(0), kf.get_x().bg(1), kf.get_x().bg(2));
           // std::printf(" ba: %.4f %.4f %.4f\n", kf.get_x().ba(0), kf.get_x().ba(1), kf.get_x().ba(2));
            //std::printf(" g: %.4f %.4f %.4f\n", kf.get_x().grav(0), kf.get_x().grav(1), kf.get_x().grav(2));

            //drone mid360 equip      -16.7637度
            // theta = -atan(kf.get_x().grav(0)/ kf.get_x().grav(2))*180.0/PI;
            theta = 20.00;  //+-?
            std::printf(" theta: %.4f \n", theta);

            alpha = 0.0000;
            // alpha = -atan(kf.get_x().grav(1)/ kf.get_x().grav(2))*180.0/PI;
            std::printf(" alpha: %.4f \n", alpha);
            
            
        
            
            publish_odometry(pubOdomAftMapped); 

           
            feats_down_world->resize(feats_down_size);
            map_incremental();

            
            if (path_en)
                publish_path(pubPath);
            /******* Visualize speed vector*******/
            if(speed_vector_en)
            {
                speed(0)=kf.get_x().vel(0);
                speed(1)=kf.get_x().vel(1);
                speed(2)=kf.get_x().vel(2);
                Visualization_speed(marker_pub);
            }


            /******* Publish points *******/
            if (scan_pub_en || pcd_save_en)
                publish_frame_world(pubLaserCloudFull);
            if (scan_pub_en && scan_body_pub_en)
            {
                publish_frame_body(pubLaserCloudFull_body);
                publish_frame_lidar(pubLaserCloudFull_lidar);
            }

            // publish_map(pubLaserCloudMap);

            double t11 = omp_get_wtime();
            std::cout << "feats_down_size: " << feats_down_size << "  Whole mapping time(ms):  " << (t11 - t00) * 1000 << std::endl
                      << std::endl;
            std::cout << "feats_undistort_size: " << feats_undistort_size  << std::endl;
                      
           //filter_size_surf_min=filter_size_surf_min*sqrt(feats_down_size/2500);
           //filter_size_map_min=filter_size_map_min*sqrt(feats_down_size/2500);


            //std::cout << "filter_size_surf_min: " << filter_size_surf_min << "  filter_size_map_min:  " << filter_size_map_min << std::endl
                      //<< std::endl;

        }

        rate.sleep();
    }


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
