#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "std_msgs/Int8.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <thread>
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/pca.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <math.h>
#include <time.h>
#include <sophus/se3.hpp>
#include <boost/format.hpp> 

#include <DBSCAN.h>

ros::Publisher pointCloud1pub,pointCloud2pub,pointCloud3pub,pointCloud4pub;
ros::Publisher tagPospub;
Eigen::Vector3d current_pos;

struct Point {
    double x, y, z;
};
// 计算点到平面的距离
double distancePointToPlane(pcl::PointXYZ point, const Eigen::Vector4d& plane) {
    return fabs((plane[0] * point.x + plane[1] * point.y + plane[2] * point.z + plane[3]) / sqrt(plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]));
}

// 使用三个点计算平面方程
Eigen::Vector4d computePlaneEquation(pcl::PointXYZ p1, pcl::PointXYZ p2, pcl::PointXYZ p3) {

    Eigen::Vector3f P0 = Eigen::Vector3f(p1.x, p1.y, p1.z);
    Eigen::Vector3f P1 = Eigen::Vector3f(p2.x, p2.y, p2.z);
    Eigen::Vector3f P2 = Eigen::Vector3f(p3.x, p3.y, p3.z);

    Eigen::Vector3f L01 = P1 - P0;
    Eigen::Vector3f L02 = P2 - P0;

    Eigen::Vector3f Normal_Vector = L01.cross(L02);
    //Normal.norm() = 1   and    up
    Normal_Vector = Normal_Vector / Normal_Vector.norm();
    if (Normal_Vector(2) < 0) {Normal_Vector = -Normal_Vector;}

    // Ax + By + Cz + D = 0
    float A = Normal_Vector[0];
    float B = Normal_Vector[1];
    float C = Normal_Vector[2];
    float D = -(A * P0(0) + B * P0(1) + C * P0(2));

    return Eigen::Vector4d(A, B, C, D);
}

void pointPublish(pcl::PointCloud<pcl::PointXYZ> points,ros::Publisher pub){
    sensor_msgs::PointCloud2 cloud_msg;

    // 设置PointCloud2消息的头部
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "world";

    // 设置PointCloud2消息的宽度、高度和点云数据
    cloud_msg.width = points.size();
    cloud_msg.height = 1;
    cloud_msg.is_dense = true;
    cloud_msg.is_bigendian = false;

    // 添加字段信息，这里假设我们只发布XYZ坐标
    sensor_msgs::PointField field;
    field.name = "x";
    field.offset = 0;
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.count = 1;
    cloud_msg.fields.push_back(field);
    field.name = "y";
    field.offset = 4;
    cloud_msg.fields.push_back(field);
    field.name = "z";
    field.offset = 8;
    cloud_msg.fields.push_back(field);

    // 计算点云数据的总大小
    cloud_msg.point_step = 12; // sizeof(float) * 3
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;

    // 分配足够的空间来存储点云数据
    cloud_msg.data.resize(cloud_msg.row_step * cloud_msg.height);

    // 填充点云数据
    for (size_t i = 0; i < points.size(); ++i) {
        float* data_ptr = reinterpret_cast<float*>(&cloud_msg.data[i * cloud_msg.point_step]);
        data_ptr[0] = points[i].x;
        data_ptr[1] = points[i].y;
        data_ptr[2] = points[i].z;
    }

    // 发布PointCloud2消息
    pub.publish(cloud_msg);
}
pcl::PointCloud<pcl::PointXYZ> planeRemoval(pcl::PointCloud<pcl::PointXYZ> cloud){
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>());
     *cloud_src=cloud;
    // 创建环境
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // 创建滤波器
    outrem.setInputCloud(cloud_src);//设置输入点云
    outrem.setRadiusSearch(0.2);//设置在0.8的半径内找临近点
    outrem.setMinNeighborsInRadius(10);//设置查询点的邻近点集数小于2的删除
    // 应用滤波器
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
    outrem.filter (*filteredCloud);
    return *filteredCloud;
    
}
bool planeValidation(pcl::PointCloud<pcl::PointXYZ> cloud){
    
    std::cout<<"ValCloudPointsNum: "<<cloud.size()<<std::endl;
    if(cloud.size()<3){
        return false;
    }
    pcl::PointCloud<pcl::PointXYZ> cloud_out;
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloud.makeShared());
    Eigen::Vector3f eigen_values;
    eigen_values=pca.getEigenValues();
    Eigen::Matrix3f eigen_vector;
    eigen_vector=pca.getEigenVectors();
    // std::cout << eigen_values << std::endl;
    // std::cout << eigen_vector << std::endl;

    pca.project(cloud, cloud_out);

    double cloud_xmin = INFINITY;
    double cloud_xmax = -INFINITY;
    double cloud_ymin = INFINITY; 
    double cloud_ymax = -INFINITY;
    for(int i=0;i<cloud_out.size();i++){
        double x = cloud_out.points[i].x;
        double y = cloud_out.points[i].y;
        if(x<cloud_xmin) cloud_xmin=x;
        if(x>cloud_xmax) cloud_xmax=x;
        if(y<cloud_ymin) cloud_ymin=y;
        if(y>cloud_ymax) cloud_ymax=y;
    }
    //以上是找该部分点云的范围
    double cloud_scaleX[6];
    double cloud_scaleY[6];
    int cloud_ExistFlag[5][5];
    double scaleX = (cloud_xmax-cloud_xmin)/5;
    double scaleY = (cloud_ymax-cloud_ymin)/5;
    for(int i=0;i<6;i++){
        cloud_scaleX[i] = cloud_xmin + scaleX*i;
        cloud_scaleY[i] = cloud_ymin + scaleY*i;
        if(i<5){
            for(int j=0;j<5;j++){
                cloud_ExistFlag[i][j] = 0;
            }
        }
    }
    //以上是构建5*5的点云分度格子
    for(int i=0;i<cloud_out.size();i++){
        double x = cloud_out.points[i].x;
        double y = cloud_out.points[i].y;
        int x_index = (x-cloud_xmin)/scaleX;
        int y_index = (y-cloud_ymin)/scaleY;
        cloud_ExistFlag[x_index][y_index]++;
    }
    //以上是统计每个格子内的点云数量
    int cloudValidThreshold = 5;
    for(int i=0;i<5;i++){
        for(int j=0;j<5;j++){
            if(cloud_ExistFlag[i][j]>cloudValidThreshold){
                cloud_ExistFlag[i][j] = 1;
            }
            else{
                cloud_ExistFlag[i][j] = 0;
            }
        }
    }
    //以上是判断每个格子内的点云数量是否大于阈值
    int cloudValidNum = 0;
    for(int i=0;i<5;i++){
        for(int j=0;j<5;j++){
            if(cloud_ExistFlag[i][j]>0){
                cloudValidNum++;
            }
        }
    }
    bool cloudValidFlag = false;
    std::cout<<"cloudValidNum: "<<cloudValidNum<<std::endl;
    if(cloudValidNum>0){
        std::cout<<"cloud is Valid !!!!"<<std::endl;
        cloudValidFlag = true;
    }
    return cloudValidFlag;

    
    // std::cout<<"cloud_outSize: "<<cloud_out.size()<<std::endl;
    
    // for(int i=0;i<cloud_out.size();i++){
    //     std::cout<<"cloud_out["<<i<<"]: "<<cloud_out.points[i].x<<" "<<cloud_out.points[i].y<<" "<<cloud_out.points[i].z<<std::endl;
    //     std::cout<<"cloud_in["<<i<<"]: "<<cloud.points[i].x<<" "<<cloud.points[i].y<<" "<<cloud.points[i].z<<std::endl;
    // }

}

bool cmp(const pcl::PointIndices& a, const pcl::PointIndices& b) {
    return a.indices.size() > b.indices.size();
}

void estDoorPos(pcl::PointCloud<pcl::PointXYZ> cloud_in){

    pcl::PointCloud<pcl::PointXYZ> cloud = cloud_in;
    // pcl::PointCloud<pcl::PointXYZ> cloud_out;
    // pcl::PCA<pcl::PointXYZ> pca;
    // pca.setInputCloud(cloud.makeShared());
    // Eigen::Vector3f eigen_values;
    // eigen_values=pca.getEigenValues();
    // Eigen::Matrix3f eigen_vector;
    // eigen_vector=pca.getEigenVectors();
    // std::cout << eigen_values << std::endl;
    // std::cout << eigen_vector << std::endl;

    // pca.project(cloud, cloud_out);

    // for(int i=0;i<cloud_out.size();i++){
    //     std::cout<<"cloud_out["<<i<<"]: "<<cloud_out.points[i].x<<" "<<cloud_out.points[i].y<<" "<<cloud_out.points[i].z<<std::endl;
    // }
    double cloud_xmin = INFINITY;
    double cloud_xmax = -INFINITY;
    double cloud_ymin = INFINITY; 
    double cloud_ymax = -INFINITY;
    for(int i =0;i<cloud.size();i++){
        double x = cloud.points[i].x;
        double y = cloud.points[i].y;
        if(x<cloud_xmin) cloud_xmin=x;
        if(x>cloud_xmax) cloud_xmax=x;
        if(y<cloud_ymin) cloud_ymin=y;
        if(y>cloud_ymax) cloud_ymax=y;
    }
    std::cout<<"cloud_xmin: "<<cloud_xmin<<std::endl;
    std::cout<<"cloud_xmax: "<<cloud_xmax<<std::endl;
    std::cout<<"cloud_ymin: "<<cloud_ymin<<std::endl;
    std::cout<<"cloud_ymax: "<<cloud_ymax<<std::endl;
    std::cout<<"plane_width: "<<cloud_ymax-cloud_ymin<<std::endl;
    std::cout<<"plane_angle: "<<cloud_xmax-cloud_xmin<<std::endl;

    // double scaleX = (cloud_xmax-cloud_xmin)/20;
    // double scaleY = (cloud_ymax-cloud_ymin)/20;
    // int point_Xcount[20] = {0};
    // int point_Ycount[20] = {0};
    // for(int i=0;i<cloud.size();i++){
    //     double x = cloud.points[i].x;
    //     double y = cloud.points[i].y;
    //     int x_index = (x-cloud_xmin)/scaleX;
    //     int y_index = (y-cloud_ymin)/scaleY;
    //     point_Xcount[x_index]++;
    //     point_Ycount[y_index]++;
    // }
    
    // int left_index =0;
    // int right_index = 0;
    // double YchangeMaxZ = -INFINITY;
    // double YchangeMaxF = INFINITY;
    // for(int i=2;i<15;i++){
    //     // std::cout<<"point_Xcount: "<<point_Xcount[i]<<std::endl;
    //     // std::cout<<"point_Ycount: "<<point_Ycount[i]<<std::endl;
    //     // if(point_Ycount[i+1]-point_Ycount[i]<-20){
    //     //     left = i*scaleY+cloud_ymin;
    //     //     continue;
    //     // }
    //     // if(point_Ycount[i+1]-point_Ycount[i]>20){
    //     //     right = i*scaleY+cloud_ymin;
    //     //     continue;
    //     // }
    //     if(point_Ycount[i+1]-point_Ycount[i]>YchangeMaxZ){
    //         YchangeMaxZ = point_Ycount[i+1]-point_Ycount[i];
    //         right_index = i;
    //     }
    //     else if(point_Ycount[i+1]-point_Ycount[i]<YchangeMaxF){
    //         YchangeMaxF = point_Ycount[i+1]-point_Ycount[i];
    //         left_index = i;
    //     }
    // }
    //     // std::cout<<"point_Ycount: "<<point_Ycount[i]<<std::endl;
    // double left = left_index*scaleY+cloud_ymin;
    // double right = right_index*scaleY+cloud_ymin;

    // std::cout<<"left: "<<left_index<<std::endl;
    // std::cout<<"right: "<<right_index<<std::endl;

    double wallWidthTrue = 2.45;
    double wallHightTrue = 1.8;

    if((cloud_ymax-cloud_ymin)>0.95*wallWidthTrue && (cloud_xmax-cloud_xmin)<1.05*wallWidthTrue){
        std::cout<<"DoorPosEstLeft: "<<cloud_ymax-0.3<<std::endl;
        std::cout<<"DoorPosEstRight: "<<cloud_ymax-0.9<<std::endl;
    }


}

// RANSAC平面拟合
std::vector<Eigen::Vector4d> ransacFitPlane(std::vector<Point>& points_input,pcl::PointCloud<pcl::PointXYZ> points_pcl_input, int iterations, double threshold,int GoalInliners) {
    bool hasFindedOkPlane = false;
    std::vector<Point> points = points_input;
    pcl::PointCloud<pcl::PointXYZ> points_pcl = points_pcl_input;
    std::vector<std::vector<Point>> planePoints;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, points_pcl.size() - 1);
    
    std::vector<Eigen::Vector4d> bestPlane;
    int goalPlaneNum = 1;
    int findedPlaneNum = 0;
    // bestPlane.resize(goalPlaneNum);//最多去找四个平面
    // planePoints.resize(goalPlaneNum);

    float points_num_initial = points_pcl.size();
    float points_num_now = points_num_initial;
    int plane_num = 0;

    pcl::PointCloud<pcl::PointXYZ> pclCloud[goalPlaneNum];
    // pcl::PointCloud<pcl::PointXYZ> cloud1_out;
    // cloud1.resize(2000);
    

    while((points_num_now/points_num_initial>0.01) && plane_num < goalPlaneNum){

        int bestInliers = 0;
        Eigen::Vector4d bestPlaneNow;
        std::cout<<"points_num_nowin: "<<points_pcl.size()<<std::endl;
        for (int i = 0; i < iterations; ++i) {
            // 随机选择三个点

            // auto frame_start_time = std::chrono::high_resolution_clock::now();
            std::set<int> random_numbers;
            while (random_numbers.size() < 3) {
                random_numbers.insert(dis(gen));
            }
            pcl::PointCloud<pcl::PointXYZ> samplePoints;
            for (int num : random_numbers) {
                int index = num;
                samplePoints.push_back(points_pcl[index]);
                // std::cout<<"i: "<<i<<" "<<index<<std::endl;
            }
            // auto frame_end_time = std::chrono::high_resolution_clock::now();
            // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(frame_end_time - frame_start_time);
            // std::cout << "time:" << duration.count() << std::endl;

            // 计算平面方程
            Eigen::Vector4d plane = computePlaneEquation(samplePoints[0], samplePoints[1], samplePoints[2]);
            // std::cout<<plane<<std::endl;

            //根据先验位置进行修改
            if(-plane[0]/plane[3]>1.0/2*1.05 ||-plane[0]/plane[3]<1.0/2*0.95 || abs(plane[1]/plane[3])>0.1 || abs(plane[2]/plane[3])>0.1){
                // std::cout<<"A: "<<-plane[0]/plane[3]<<std::endl;
                continue;
            }
            
            int inliers = 0;
            for (const auto& point : points_pcl) {
                // 计算每个点到平面的距离
                if (distancePointToPlane(point, plane) < threshold) {
                    ++inliers;
                }
                // std::cout<<distancePointToPlane(point, plane)<<std::endl;
            }
            // std::cout<<inliers<<std::endl;
            // 如果当前模型的内点数量多于之前找到的最好模型，则更新最好模型
            if (inliers > bestInliers) {
                bestInliers = inliers;
                bestPlaneNow = plane;
                // bestPlane[plane_num] = plane;
            }
            if(bestInliers>0.4*points_num_now){
                break;
            }
            // std::cout<<i<<std::endl;
        }
        bestPlane.push_back(bestPlaneNow);
        
        auto it = points_pcl.begin();
        int i = 0;
        std::vector<Point> planePointsNow;
        while (it != points_pcl.end()) {
            // 计算每个点到平面的距离
            if (distancePointToPlane(*it, bestPlane[plane_num]) < threshold) {
                // 使用erase方法删除当前元素，并自动更新迭代器
                // planePoints[plane_num].push_back(*it);
                Point point;
                point.x = it->x;
                point.y = it->y;
                point.z = it->z;
                planePointsNow.push_back(point);
                // pcl::PointXYZ point;
                // point.x = it->x;
                // point.y = it->y;
                // point.z = it->z;
                pclCloud[plane_num].push_back(*it);
                it = points_pcl.erase(it);
            } else {
                // 如果不删除元素，则前进到下一个元素
                ++it;
            }
            i++;
        }
        planePoints.push_back(planePointsNow);
        points_num_now = points_pcl.size();
        std::cout<<"bestInliers: "<<bestInliers<<std::endl;
        std::cout<<"points_num_now: "<<points_num_now<<std::endl;
        std::cout<<"planePoints: "<<planePoints[plane_num].size()<<std::endl;
        std::cout<<"plane_num: "<<plane_num<<std::endl;
        plane_num++;
    }

    bool cloudValidFlag[goalPlaneNum];
    pcl::PointCloud<pcl::PointXYZ> pclCloudFilted[goalPlaneNum];
    //检验平面拟合准确性
    for(int i =0;i<planePoints.size();i++){
        cloudValidFlag[i] = false;
        // pclCloudFilted[i] = planeRemoval(pclCloud[i]);
        // std::cout<<"Filted VAL: "<<std::endl;
        // cloudValidFlag[i] = planeValidation(pclCloudFilted[i]);
        std::cout<<"ALL Points VAL: "<<std::endl;
        cloudValidFlag[i] = planeValidation(pclCloud[i]);
        if(!cloudValidFlag[i]){
            planePoints[i].clear();
        }
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud_input = pclCloud[0];
    
    //点云聚类
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_input);
    std::vector<pcl::PointIndices> cluster_indices;

    DBSCANKdtreeCluster<pcl::PointXYZ> ec;
    ec.setCorePointMinPts(5);
    ec.setClusterTolerance(0.2);//容忍距离
    ec.setMinClusterSize(10);//点云聚类的最小点数量
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_input);
    ec.extract(cluster_indices);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clustered(new pcl::PointCloud<pcl::PointXYZ>);

    // sort(cluster_indices.begin(), cluster_indices.end(), cmp);
    // for(int i =0;i<cluster_indices.size();i++){

    //     std::cout<<"cluster_indices: "<<cluster_indices[i].indices.size()<<std::endl;
    // }

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            pcl::PointXYZ tmp;
            tmp.x = cloud_input->points[*pit].x;
            tmp.y = cloud_input->points[*pit].y;
            tmp.z = cloud_input->points[*pit].z;
            cloud_clustered->points.push_back(tmp);
            // std::cout<<"!!!!!!!"<<std::endl;
        }
    }
    std::cout<<"cloud_clustered: "<<cloud_clustered->size()<<std::endl;

    estDoorPos(*cloud_clustered);
    if(cloudValidFlag[0]) pointPublish(*cloud_clustered,pointCloud1pub);
    // if(cloudValidFlag[0]) pointPublish(pclCloud[0],pointCloud1pub);
    // if(cloudValidFlag[1]) pointPublish(pclCloudFilted[1],pointCloud2pub);

    // if(cloudValidFlag[0]) pointPublish(planePoints[0],pointCloud1pub);
    // if(cloudValidFlag[1]) pointPublish(planePoints[1],pointCloud2pub);
    // if(cloudValidFlag[2]) pointPublish(planePoints[2],pointCloud3pub);
    // if(cloudValidFlag[3]) pointPublish(planePoints[3],pointCloud4pub);
    
    // std::cout<<bestPlane.size()<<std::endl;
    return bestPlane;
}
void ransacFitCircle(pcl::PointCloud<pcl::PointXYZ>::Ptr points){
        ///存放拟合圆
    // std::vector<int> inliersCircle3D;
    
    // ///oriCloud_inliers：待求解拟合圆的原始点云
    // pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>::Ptr socCircle3D(new pcl::SampleConsensusModelCircle3D<pcl::PointXYZ> (points));
    
    // ///求解拟合圆
    // pcl::RandomSampleConsensus<pcl::PointXYZ>::Ptr  ransac( new pcl::RandomSampleConsensus<pcl::PointXYZ> (socCircle3D) );
    // ransac->setDistanceThreshold(0.1f);
    // ransac->computeModel();
    // ransac->getInliers(inliersCircle3D);

    // ///获取拟合圆的参数
    // Eigen::VectorXf circle_coeff;
    // ransac->getModelCoefficients(circle_coeff);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr finalCircle3D(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::copyPointCloud (*points, inliersCircle3D, *finalCircle3D);

    // pointPublish(*finalCircle3D,pointCloud3pub);
    
    if(points->size()<10){
        return;
    }
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);//创建索引对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cir(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    //	估计法向量
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(points);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.01);
    ne.compute(*normals);
    //RANSAC
    pcl::SACSegmentationFromNormals<pcl::PointXYZ,pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CIRCLE3D);//拟合3D圆
    seg.setMethodType(pcl::SAC_RANSAC);//RANSAC
    seg.setNormalDistanceWeight(0.9);//法向量权重
    seg.setMaxIterations(1000);//设置最大迭代次数
    seg.setDistanceThreshold(0.1);//eps
    seg.setRadiusLimits(0.5,0.6);//添加拟合圆的半径限制，防止拟合过大或过小的圆
    seg.setInputNormals(normals);//输入法向量
    seg.setInputCloud(points);//传入点集
    seg.segment(*inliers,*coefficients);//分割

    if(inliers->indices.size()==0){
        std::cout<<"Could not estimate a circle model for the given dataset."<<std::endl;
        return;
    }
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(points);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_cir);

    
    
    if(abs(coefficients->values[4])>0.95 && abs(coefficients->values[5])<0.2 && abs(coefficients->values[6])<0.2){
        
        std::cout<< "circle pos:"<<coefficients->values[0]<<" "<<coefficients->values[1]<<" "<<coefficients->values[2]<<std::endl;
        std::cout<< "circle size:"<<coefficients->values[3]<<" "<<coefficients->values[4]<<" "<<coefficients->values[5]<<" "<<coefficients->values[6]<<std::endl;
        
        double A = coefficients->values[4];
        double B = coefficients->values[5];
        double C = coefficients->values[6];
        double D = -A*coefficients->values[0]-B*coefficients->values[1]-C*coefficients->values[2];

        //std::cout<<A<<" "<<B<<" "<<C<<" "<<D<<std::endl;
        
        Eigen::Vector4d plane(A,B,C,D);
        int plane_inliers = 0;
        for (const auto& point : *points) {
            // 计算每个点到平面的距离
            if (distancePointToPlane(point, plane) < 0.1) {
                ++plane_inliers;
            }
        }
        //std::cout<<"plane_inliers: "<<plane_inliers<<" cirlce_inliers: "<<inliers->indices.size()<<std::endl;
        if(plane_inliers/inliers->indices.size()>1.5){
            return;
        }

        pointPublish(*cloud_cir,pointCloud3pub);

        geometry_msgs::PoseStamped circle_pose;
        circle_pose.header.stamp = ros::Time::now();
        circle_pose.header.frame_id = "world";
        circle_pose.pose.position.x = coefficients->values[0];
        circle_pose.pose.position.y = coefficients->values[1];
        circle_pose.pose.position.z = coefficients->values[2];
        circle_pose.pose.orientation.x = 0;
        circle_pose.pose.orientation.y = 0;
        circle_pose.pose.orientation.z = 0;
        circle_pose.pose.orientation.w = 1;
        tagPospub.publish(circle_pose);


    }



}
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPreProcess(pcl::PointCloud<pcl::PointXYZ> cloud_in,int task){
    
    double x_min,x_max,y_min,y_max,z_min,z_max;
    if(task == 2){
        x_min = current_pos.x() + 1;
        x_max = current_pos.x() + 5;
        y_min = current_pos.y() - 2;
        y_max = current_pos.y() + 2; 
        z_min = 0.2;
        z_max = 3;
    }else if(task == 4){
        x_min = current_pos.x() - 5;
        x_max = current_pos.x() + 0;
        y_min = current_pos.y() - 2;
        y_max = current_pos.y() + 2; 
        z_min = 0.2;
        z_max = 3;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud_input = cloud_in;
    *cloud_filtered = cloud_in;
    if(task == 2){
        //以下为对输入点云进行简单的离群点去除+地面点删除
        // std::cout<<"points_num: "<<points_pcl.size()<<std::endl;
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ>::Ptr oulier_removal(new pcl::StatisticalOutlierRemoval<pcl::PointXYZ>);
        oulier_removal->setInputCloud(cloud_input);
        oulier_removal->setMeanK(10); // 设置邻域大小为50
        oulier_removal->setStddevMulThresh(2.0); // 设置去除阈值为1.0
        oulier_removal->filter(*cloud_filtered);
        // std::cout<<"filted1Points_num: "<<cloud_filtered->size()<<std::endl;
    }

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_filtered);	
    pass.setFilterFieldName("z");           //设置过滤时所需要点云类型为Z字段
    pass.setFilterLimits(z_min, z_max);         //设置在过滤字段的范围
    pass.setFilterLimitsNegative(false);     //设置保留还是过滤掉字段范围内的点，设置为true表示过滤掉字段范围内的点
    pass.filter(*cloud_filtered);
    pass.setFilterFieldName("y");           //设置过滤时所需要点云类型为Z字段
    pass.setFilterLimits(y_min, y_max);         //设置在过滤字段的范围
    pass.filter(*cloud_filtered);
    pass.setFilterFieldName("x");           //设置过滤时所需要点云类型为Z字段
    pass.setFilterLimits(x_min, x_max);         //设置在过滤字段的范围
    pass.filter(*cloud_filtered);
    // std::cout<<"filted2Points_num: "<<cloud_filtered->size()<<std::endl;
    //以上为对输入点云进行简单的离群点去除+地面点删除
    pointPublish(*cloud_filtered,pointCloud2pub);

    return cloud_filtered;

}
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    
    auto frame_start_time = std::chrono::high_resolution_clock::now();
    std::vector<Point> points;
    pcl::PointCloud<pcl::PointXYZ> points_pcl;
    // 计算点的总数
    size_t num_points = cloud_msg->width * cloud_msg->height;
    // 提取点云数据
    for (size_t i = 0; i < num_points; ++i) {
        Point p;
        p.x = *(reinterpret_cast<const float*>(&cloud_msg->data[i * cloud_msg->point_step + cloud_msg->fields[0].offset]));
        p.y = *(reinterpret_cast<const float*>(&cloud_msg->data[i * cloud_msg->point_step + cloud_msg->fields[1].offset]));
        p.z = *(reinterpret_cast<const float*>(&cloud_msg->data[i * cloud_msg->point_step + cloud_msg->fields[2].offset]));
        points.push_back(p);
        pcl::PointXYZ point;
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;
        points_pcl.push_back(point);
    }
    // // 打印点的数量
    ROS_INFO("Stored %zu points in the vector.", points.size());
    int task = 4;

    if(task == 2){

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = cloudPreProcess(points_pcl,task);

        int NumOfPoints = cloud_filtered->size();
        float MinProbInliers = 0.3;
        int MaxIteration = ceil(log(1 - 0.99) / log(1 - pow(MinProbInliers, 3)));//最大迭代轮数
        // MaxIteration = 200000;
        std::cout<<"MaxIteration: "<<MaxIteration<<std::endl;
        int GoalInliners = std::max(int(MinProbInliers * NumOfPoints), 4);

        double threshold = 0.05;
        
        // 拟合平面
        std::vector<Eigen::Vector4d> plane = ransacFitPlane(points,*cloud_filtered, MaxIteration, threshold,GoalInliners);
        // 输出结果
        for(int i =0;i<plane.size();i++){
            std::cout << "Best fit plane "<<i<<" : " << plane[i][0] << "x + " << plane[i][1] << "y + " << plane[i][2] << "z + " << plane[i][3] << " = 0" << std::endl;
        }

    }else if(task == 4){

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = cloudPreProcess(points_pcl,task);
        ransacFitCircle(cloud_filtered);

    }
    auto frame_end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> frame_duration = frame_end_time - frame_start_time ;
    float fps = 1.0 / frame_duration.count();
    std::cout << "fps:" << fps << std::endl;

}

// 定义一个回调函数，用于处理接收到的odom消息
void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    // 从odom消息中提取位置信息并转换为Eigen::Vector3d类型
    Eigen::Vector3d current_position(msg->pose.pose.position.x,
                                     msg->pose.pose.position.y,
                                     msg->pose.pose.position.z);

    // 打印出存储的位置信息
    // ROS_INFO_STREAM("Stored position: \n" << current_position);

    current_pos = current_position;
    
}

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "object_detection_node");
    ros::NodeHandle n;
    ros::Subscriber odom_subscriber = n.subscribe("/Odometry", 10, odom_callback);
    ros::Subscriber pointCloud_subscriber = n.subscribe("/cloud_registered", 10, pointCloudCallback);
    

    ///cx
    ros::Rate rate(100);

    pointCloud1pub = n.advertise<sensor_msgs::PointCloud2>("/point_cloud1", 10);
    pointCloud2pub = n.advertise<sensor_msgs::PointCloud2>("/point_cloud2", 10);
    pointCloud3pub = n.advertise<sensor_msgs::PointCloud2>("/point_cloud3", 10);
    pointCloud4pub = n.advertise<sensor_msgs::PointCloud2>("/point_cloud4", 10);
    tagPospub = n.advertise<geometry_msgs::PoseStamped>("/target_pose", 10);
    
    while (ros::ok()){

        ros::spinOnce();
        rate.sleep();

    }

    return 0;
}
