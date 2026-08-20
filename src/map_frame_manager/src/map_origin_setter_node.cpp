/**
 * @file map_origin_setter_node.cpp
 * @brief Turn a relocalized, unanchored PCD into a permanent yaw-level map.
 */

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>

#include <boost/filesystem.hpp>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

namespace {

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

double yawFromPose(const geometry_msgs::Pose& pose) {
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                       pose.orientation.y, pose.orientation.z);
  if (q.norm() < 1e-9) return 0.0;
  const Eigen::Matrix3d r = q.normalized().toRotationMatrix();
  return std::atan2(r(1, 0), r(0, 0));
}

class MapOriginSetter {
 public:
  MapOriginSetter() : nh_(), pnh_("~") {
    pnh_.param<std::string>("input_pcd", input_pcd_, "");
    pnh_.param<std::string>("output_directory", output_directory_, "/tmp/jiangyin_map");
    pnh_.param<std::string>("map_name", map_name_, "competition_map");
    pnh_.param<std::string>("odom_topic", odom_topic_, "/Odometry_lio_global");
    pnh_.param<std::string>("initialized_topic", initialized_topic_,
                            "/relocalization/initialized");
    pnh_.param<std::string>("state_topic", state_topic_, "/map_origin_setter/state");
    pnh_.param<std::string>("input_frame", input_frame_, "unanchored_lio_local");
    pnh_.param<std::string>("output_frame", output_frame_, "lio_global");
    pnh_.param("output_leaf_size", output_leaf_size_, 0.05);
    pnh_.param("require_stationary", require_stationary_, true);
    pnh_.param("max_linear_speed", max_linear_speed_, 0.05);
    pnh_.param("max_angular_speed", max_angular_speed_, 0.03);

    state_pub_ = nh_.advertise<std_msgs::String>(state_topic_, 1, true);
    ready_pub_ = pnh_.advertise<std_msgs::Bool>("ready", 1, true);
    odom_sub_ = nh_.subscribe(odom_topic_, 20, &MapOriginSetter::odomCallback, this);
    initialized_sub_ = nh_.subscribe(initialized_topic_, 2,
                                     &MapOriginSetter::initializedCallback, this);
    set_srv_ = pnh_.advertiseService("set_origin", &MapOriginSetter::setService, this);
    publishState("WAITING_FOR_RELOCALIZATION");
  }

 private:
  void initializedCallback(const std_msgs::Bool::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    relocalized_ = msg->data;
    publishStateLocked(relocalized_ && have_odom_ ? "READY_TO_SET_ORIGIN"
                                                  : "WAITING_FOR_RELOCALIZATION");
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_odom_ = *msg;
    have_odom_ = true;
    if (relocalized_) publishStateLocked("READY_TO_SET_ORIGIN");
  }

  bool setService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    nav_msgs::Odometry odom;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!relocalized_ || !have_odom_) {
        response.success = false;
        response.message = "Relocalization is not ready";
        return true;
      }
      const auto& linear = latest_odom_.twist.twist.linear;
      const auto& angular = latest_odom_.twist.twist.angular;
      const double linear_speed = std::sqrt(linear.x * linear.x + linear.y * linear.y +
                                            linear.z * linear.z);
      const double angular_speed = std::sqrt(angular.x * angular.x + angular.y * angular.y +
                                              angular.z * angular.z);
      if (require_stationary_ &&
          (linear_speed > max_linear_speed_ || angular_speed > max_angular_speed_)) {
        response.success = false;
        response.message = "Vehicle is moving; hold it still and retry";
        return true;
      }
      odom = latest_odom_;
      publishStateLocked("SAVING");
    }
    response.success = writeMap(odom, &response.message);
    publishState(response.success ? "SAVED" : "FAILED_SAVE");
    return true;
  }

  bool writeMap(const nav_msgs::Odometry& odom, std::string* message) {
    Cloud input;
    if (input_pcd_.empty() || pcl::io::loadPCDFile<Point>(input_pcd_, input) != 0 ||
        input.empty()) {
      *message = "Cannot load input PCD: " + input_pcd_;
      return false;
    }

    const float yaw = static_cast<float>(yawFromPose(odom.pose.pose));
    const Eigen::Vector3f position(odom.pose.pose.position.x, odom.pose.pose.position.y,
                                   odom.pose.pose.position.z);
    Eigen::Matrix4f output_from_input = Eigen::Matrix4f::Identity();
    output_from_input.block<3, 3>(0, 0) =
        Eigen::AngleAxisf(-yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    output_from_input.block<3, 1>(0, 3) =
        -output_from_input.block<3, 3>(0, 0) * position;

    Cloud transformed;
    pcl::transformPointCloud(input, transformed, output_from_input);
    Cloud output;
    if (output_leaf_size_ > 0.0) {
      pcl::VoxelGrid<Point> voxel;
      voxel.setLeafSize(output_leaf_size_, output_leaf_size_, output_leaf_size_);
      voxel.setInputCloud(transformed.makeShared());
      voxel.filter(output);
    } else {
      output.swap(transformed);
    }

    boost::system::error_code ec;
    boost::filesystem::create_directories(output_directory_, ec);
    if (ec) {
      *message = "Cannot create output directory: " + ec.message();
      return false;
    }
    const std::string base = output_directory_ + "/" + map_name_;
    const std::string pcd_tmp = base + ".pcd.tmp";
    const std::string pcd_path = base + ".pcd";
    const std::string yaml_tmp = base + ".yaml.tmp";
    const std::string yaml_path = base + ".yaml";
    if (pcl::io::savePCDFileBinary(pcd_tmp, output) != 0 ||
        std::rename(pcd_tmp.c_str(), pcd_path.c_str()) != 0) {
      *message = "Failed to save transformed PCD";
      return false;
    }

    Eigen::Quaternionf q(output_from_input.block<3, 3>(0, 0));
    q.normalize();
    std::ofstream yaml(yaml_tmp.c_str(), std::ios::trunc);
    yaml << std::setprecision(10)
         << "format_version: 2\n"
         << "map_id: " << map_name_ << "\n"
         << "pcd_file: " << map_name_ << ".pcd\n"
         << "source_pcd: " << input_pcd_ << "\n"
         << "point_count: " << output.size() << "\n"
         << "frame_convention: ENU\n"
         << "origin_definition: relocalized_vehicle_position_and_heading\n"
         << "map_frame: " << output_frame_ << "\n"
         << "source_frame: " << input_frame_ << "\n"
         << "transform_output_from_source:\n"
         << "  translation: [" << output_from_input(0, 3) << ", "
         << output_from_input(1, 3) << ", " << output_from_input(2, 3) << "]\n"
         << "  quaternion: [" << q.x() << ", " << q.y() << ", " << q.z() << ", "
         << q.w() << "]\n"
         << "origin_source_position: [" << position.x() << ", " << position.y() << ", "
         << position.z() << "]\n"
         << "origin_source_yaw: " << yaw << "\n";
    yaml.close();
    if (!yaml || std::rename(yaml_tmp.c_str(), yaml_path.c_str()) != 0) {
      *message = "Failed to save origin metadata";
      return false;
    }
    std::ostringstream result;
    result << "Saved permanent-origin map to " << pcd_path << " (" << output.size()
           << " points)";
    *message = result.str();
    return true;
  }

  void publishState(const std::string& state) {
    std::lock_guard<std::mutex> lock(mutex_);
    publishStateLocked(state);
  }

  void publishStateLocked(const std::string& state) {
    if (state == state_) return;
    state_ = state;
    std_msgs::String msg;
    msg.data = state;
    state_pub_.publish(msg);
    std_msgs::Bool ready;
    ready.data = state == "READY_TO_SET_ORIGIN" || state == "SAVED";
    ready_pub_.publish(ready);
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber odom_sub_, initialized_sub_;
  ros::Publisher state_pub_, ready_pub_;
  ros::ServiceServer set_srv_;
  std::mutex mutex_;
  nav_msgs::Odometry latest_odom_;
  std::string input_pcd_, output_directory_, map_name_, odom_topic_;
  std::string initialized_topic_, state_topic_, input_frame_, output_frame_, state_;
  double output_leaf_size_ = 0.05, max_linear_speed_ = 0.05, max_angular_speed_ = 0.03;
  bool require_stationary_ = true, relocalized_ = false, have_odom_ = false;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_origin_setter");
  MapOriginSetter node;
  ros::spin();
  return 0;
}
