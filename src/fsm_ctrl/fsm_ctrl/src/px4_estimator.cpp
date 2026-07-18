/**
 * @file    px4_estimator
 * @brief   subscribe odometry from sensors & publish pose to mavros for odometry - imu fusion
 * @author  FLAG Lab, BIT
 * @version 1.1 remove some shit
 * @date    2024-05-26
 */

#include <fsm_ctrl/px4_estimator.hpp>

using namespace std;



/**
 * @brief  motion capture pose subscriber callback
 * @param  msg: from MoCap
 * @return NULL
 */
void Mocap_Callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    int flag_mocap_frame = 0;    // frame in Motive, 0: Z-up; 1: Y-up

    if(flag_mocap_frame == 0)
    {
        if(!is_mocap_init)
        {
            pos_mocap_init = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
            quat_mocap_init = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
            is_mocap_init = true;
        }
        
        Eigen::Vector3d pos_raw = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        Eigen::Quaterniond quat_raw = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
        pos_mocap = quat_mocap_init.inverse()*(pos_raw - pos_mocap_init);
        quat_mocap = quat_mocap_init.inverse()*quat_raw;
        euler_mocap = QuatToEuler(quat_mocap);
    }

    else if(flag_mocap_frame == 1)
    {
        if(!is_mocap_init)
        {
            pos_mocap_init = Eigen::Vector3d(msg->pose.position.x, -msg->pose.position.z, msg->pose.position.y);
            quat_mocap_init = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.z, msg->pose.orientation.y);
            is_mocap_init = true;
        }

        Eigen::Vector3d pos_raw = Eigen::Vector3d(msg->pose.position.x, -msg->pose.position.z, msg->pose.position.y);
        Eigen::Quaterniond quat_raw = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.z, msg->pose.orientation.y);
        pos_mocap = quat_mocap_init.inverse()*(pos_raw - pos_mocap_init);
        quat_mocap = quat_mocap_init.inverse()*quat_raw;
        euler_mocap = QuatToEuler(quat_mocap);
    }

    else
    {
        ROS_ERROR("Invalid Motive Frame!!!");
        return;
    }

    if(flag_vision_source == 0)
    {
        vision_pose.header.stamp = msg->header.stamp;
        vision_pose.header.frame_id = "map";
        vision_pose.pose.position.x = pos_mocap(0) ;
        vision_pose.pose.position.y = pos_mocap(1) ;
        vision_pose.pose.position.z = pos_mocap(2) ;
        vision_pose.pose.orientation.w = quat_mocap.w();
        vision_pose.pose.orientation.x = quat_mocap.x();
        vision_pose.pose.orientation.y = quat_mocap.y();
        vision_pose.pose.orientation.z = quat_mocap.z();
        is_source_new = true;
    }
}


/**
 * @brief  lidar odometry subscriber callback
 * @param  msg: from LIO
 * @return NULL
 */
void Lidar_Callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    pos_lidar = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    vel_lidar = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    quat_lidar = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    euler_lidar = QuatToEuler(quat_lidar);

    if(flag_vision_source == 1)
    {
        vision_pose.header.stamp = msg->header.stamp;
        vision_pose.header.frame_id = "map";
        vision_pose.pose.position.x = pos_lidar(0) ;
        vision_pose.pose.position.y = pos_lidar(1) ;
        vision_pose.pose.position.z = pos_lidar(2) ;
        vision_pose.pose.orientation.w = quat_lidar.w();
        vision_pose.pose.orientation.x = quat_lidar.x();
        vision_pose.pose.orientation.y = quat_lidar.y();
        vision_pose.pose.orientation.z = quat_lidar.z();
        is_source_new = true;
    }
}


/**
 * @brief  camera odometry subscriber callback
 * @param  msg: from VIO
 * @return NULL
 */
void Camera_Callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    pos_camera = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    vel_camera = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    quat_camera = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    euler_camera = QuatToEuler(quat_camera);

    if(flag_vision_source == 2)
    {
        vision_pose.header.stamp = msg->header.stamp;
        vision_pose.header.frame_id = "map";
        vision_pose.pose.position.x = pos_camera(0) ;
        vision_pose.pose.position.y = pos_camera(1) ;
        vision_pose.pose.position.z = pos_camera(2) ;
        vision_pose.pose.orientation.w = quat_camera.w();
        vision_pose.pose.orientation.x = quat_camera.x();
        vision_pose.pose.orientation.y = quat_camera.y();
        vision_pose.pose.orientation.z = quat_camera.z();
        is_source_new = true;
    }
}


/**
 * @brief  fusion pose subscriber callback
 * @param  msg: from FCU EKF
 * @return NULL
 */
void Pose_Callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_fcu = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    quat_fcu = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    euler_fcu = QuatToEuler(quat_fcu);
}


/**
 * @brief  fusion velocity subscriber callback
 * @param  msg: from FCU EKF
 * @return NULL
 */
void Vel_Callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    vel_fcu = Eigen::Vector3d(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
}


/**
 * @brief  map ranger data to height 
 * @param  _range: raw data from ranger
 * @param  _roll, _pitch: roll & pitch angle
 * @return NULL
 */
double MapHeight(double _range, double _roll, double _pitch)
{
    return _range*cos(atan(sqrt(tan(_roll)*tan(_roll) + tan(_pitch)*tan(_pitch))));
}

/**
 * @brief  ranger data subscriber callback
 * @param  msg: sensor_msgs::Range from TFmini
 * @return NULL
 */
void Ranger_Callback(const sensor_msgs::Range::ConstPtr &msg)
{
    tfmini_raw = msg->range;
    tfmini_map = MapHeight(tfmini_raw, euler_fcu(0), euler_fcu(1));
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_estimator");
    ros::NodeHandle nh;
    
    /* parameter */
    nh.param("/px4_estimator/vision_source", flag_vision_source, 0);

    /* publisher */
    ros::Publisher ready_pub = nh.advertise<std_msgs::Bool>("/fsm_ctrl/ekf_ready", 1);
    ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1);
    
    /* subscriber */
    ros::Subscriber mocap_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/jy0/pose", 1, Mocap_Callback);
    ros::Subscriber lidar_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 1, Lidar_Callback);    
    ros::Subscriber camera_sub = nh.subscribe<nav_msgs::Odometry>("camera_odom", 1, Camera_Callback);
    ros::Subscriber ranger_sub = nh.subscribe<sensor_msgs::Range>("/tfmini", 1, Ranger_Callback);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, Pose_Callback);
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 10, Vel_Callback);

    ros::Rate rate(100.0);
    while(ros::ok())
    {   
        ros::spinOnce();
        
        if(is_source_new)
        {
            vision_pub.publish(vision_pose);
            is_source_new = false;
        }

        Eigen::Vector3d euler_vision = QuatToEuler(vision_pose.pose.orientation.w, vision_pose.pose.orientation.x, vision_pose.pose.orientation.y, vision_pose.pose.orientation.z);
        if(abs(vision_pose.pose.position.x - pos_fcu(0)) < 0.05 &&
           abs(vision_pose.pose.position.y - pos_fcu(1)) < 0.05 &&
           abs(vision_pose.pose.position.z - pos_fcu(2)) < 0.05 &&
           abs(euler_vision(2) - euler_fcu(2)) < M_PI/20.0)
        {ekf_ready.data = true;}                                                                                                                                                                                                                                                                                                                                                                                                                                                                           // else {ekf_ready.data = false;}
        ready_pub.publish(ekf_ready);

        rate.sleep();
    }

    return 0;
}