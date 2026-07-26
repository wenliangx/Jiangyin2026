#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

class PoseToOdom
{
public:
  PoseToOdom()
    : first_pose_(false)
  {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // --- /Odometry publisher ---
    pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
        "/mavros/local_position/pose", 10,
        &PoseToOdom::poseCallback, this);
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("/Odometry", 10);

    // --- /cloud_registered publisher (empty cloud for no-LiDAR testing) ---
    double cloud_rate;
    pnh.param<double>("cloud_rate", cloud_rate, 10.0);  // default 10 Hz
    pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 10);
    pc_timer_ = nh.createTimer(ros::Duration(1.0 / cloud_rate),
                               &PoseToOdom::publishEmptyCloud, this);

    // Pre-build the empty PointCloud2 fields (PointXYZI format, same as RA-LIO)
    empty_cloud_.header.frame_id = "world";
    empty_cloud_.height = 1;
    empty_cloud_.width  = 0;
    empty_cloud_.is_bigendian = false;
    empty_cloud_.is_dense     = true;

    sensor_msgs::PointCloud2Modifier modifier(empty_cloud_);
    modifier.setPointCloud2Fields(4,
        "x",         1, sensor_msgs::PointField::FLOAT32,
        "y",         1, sensor_msgs::PointField::FLOAT32,
        "z",         1, sensor_msgs::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::PointField::FLOAT32);

    ROS_INFO("pose_to_odom_node: /mavros/local_position/pose -> /Odometry");
    ROS_INFO("pose_to_odom_node: publishing empty /cloud_registered at %.1f Hz", cloud_rate);
  }

private:
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
  {
    nav_msgs::Odometry odom_msg;

    // --- Header ---
    odom_msg.header.stamp    = pose_msg->header.stamp;
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id  = "base_link";

    // --- Position ---
    odom_msg.pose.pose.position.x = pose_msg->pose.position.x;
    odom_msg.pose.pose.position.y = pose_msg->pose.position.y;
    odom_msg.pose.pose.position.z = pose_msg->pose.position.z;

    // --- Orientation ---
    odom_msg.pose.pose.orientation.w = pose_msg->pose.orientation.w;
    odom_msg.pose.pose.orientation.x = pose_msg->pose.orientation.x;
    odom_msg.pose.pose.orientation.y = pose_msg->pose.orientation.y;
    odom_msg.pose.pose.orientation.z = pose_msg->pose.orientation.z;

    // --- Linear velocity (from position differentiation) ---
    if (first_pose_)
    {
      double dt = (pose_msg->header.stamp - last_time_).toSec();
      if (dt > 1e-6)
      {
        odom_msg.twist.twist.linear.x =
            (pose_msg->pose.position.x - last_pos_.x) / dt;
        odom_msg.twist.twist.linear.y =
            (pose_msg->pose.position.y - last_pos_.y) / dt;
        odom_msg.twist.twist.linear.z =
            (pose_msg->pose.position.z - last_pos_.z) / dt;
      }
    }

    // Store for next callback
    last_pos_.x = pose_msg->pose.position.x;
    last_pos_.y = pose_msg->pose.position.y;
    last_pos_.z = pose_msg->pose.position.z;
    last_time_  = pose_msg->header.stamp;
    first_pose_ = true;

    odom_pub_.publish(odom_msg);
  }

  void publishEmptyCloud(const ros::TimerEvent&)
  {
    empty_cloud_.header.stamp = ros::Time::now();
    pc_pub_.publish(empty_cloud_);
  }

  // --- /Odometry ---
  ros::Subscriber pose_sub_;
  ros::Publisher  odom_pub_;

  bool              first_pose_;
  geometry_msgs::Point last_pos_;
  ros::Time         last_time_;

  // --- /cloud_registered ---
  ros::Publisher            pc_pub_;
  ros::Timer                pc_timer_;
  sensor_msgs::PointCloud2  empty_cloud_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_to_odom_node");
  PoseToOdom converter;
  ros::spin();
  return 0;
}
