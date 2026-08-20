/**
 * @file map_origin_recorder_node.cpp
 * @brief Automatically freeze a permanent yaw-level map frame and save PCD+metadata.
 */

#include <cmath>
#include <cstdio>
#include <deque>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <pcl/common/point_tests.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_broadcaster.h>

namespace {

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

struct PoseSample {
  ros::Time stamp;
  Eigen::Vector3d position;
  double yaw;
  double linear_speed;
  double angular_speed;
};

double yawFromPose(const geometry_msgs::Pose& pose) {
  const Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                             pose.orientation.y, pose.orientation.z);
  const Eigen::Matrix3d r = q.normalized().toRotationMatrix();
  return std::atan2(r(1, 0), r(0, 0));
}

Eigen::Matrix4f poseMatrix(const geometry_msgs::Pose& pose) {
  Eigen::Quaternionf q(pose.orientation.w, pose.orientation.x,
                       pose.orientation.y, pose.orientation.z);
  if (q.norm() < 1e-6f) q = Eigen::Quaternionf::Identity();
  q.normalize();
  Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
  result.block<3, 3>(0, 0) = q.toRotationMatrix();
  result.block<3, 1>(0, 3) << pose.position.x, pose.position.y, pose.position.z;
  return result;
}

geometry_msgs::Pose matrixPose(const Eigen::Matrix4f& transform) {
  geometry_msgs::Pose pose;
  pose.position.x = transform(0, 3);
  pose.position.y = transform(1, 3);
  pose.position.z = transform(2, 3);
  Eigen::Quaternionf q(transform.block<3, 3>(0, 0));
  q.normalize();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

class MapOriginRecorder {
 public:
  MapOriginRecorder()
      : nh_(), pnh_("~"), accumulated_(new Cloud), raw_accumulated_(new Cloud) {
    loadParameters();
    state_pub_ = nh_.advertise<std_msgs::String>(state_topic_, 1, true);
    ready_pub_ = nh_.advertise<std_msgs::Bool>(ready_topic_, 1, true);
    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(map_cloud_topic_, 1, true);
    global_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(global_odom_topic_, 20);
    odom_sub_ = nh_.subscribe(odom_topic_, 100, &MapOriginRecorder::odomCallback, this);
    cloud_sub_ = nh_.subscribe(cloud_topic_, 20, &MapOriginRecorder::cloudCallback, this);
    lock_srv_ = pnh_.advertiseService("lock_origin", &MapOriginRecorder::lockService, this);
    save_srv_ = pnh_.advertiseService("save_map", &MapOriginRecorder::saveService, this);
    reset_srv_ = pnh_.advertiseService("reset", &MapOriginRecorder::resetService, this);
    start_time_ = ros::Time::now();
    setState("WAITING_FOR_LIO", false);
  }

  ~MapOriginRecorder() {
    if (save_on_shutdown_ && origin_locked_ && !saved_ &&
        accumulated_->size() >= static_cast<std::size_t>(minimum_save_points_)) {
      std::string message;
      if (!saveMap(&message)) ROS_ERROR_STREAM("Shutdown map save failed: " << message);
    }
  }

 private:
  void loadParameters() {
    pnh_.param<std::string>("odom_topic", odom_topic_, "/Odometry");
    pnh_.param<std::string>("cloud_topic", cloud_topic_, "/cloud_registered");
    pnh_.param<std::string>("global_odom_topic", global_odom_topic_,
                            "/Odometry_lio_global");
    pnh_.param<std::string>("map_cloud_topic", map_cloud_topic_, "/map_building_cloud");
    pnh_.param<std::string>("state_topic", state_topic_, "/map_frame_manager/state");
    pnh_.param<std::string>("ready_topic", ready_topic_, "/map_frame_manager/ready");
    pnh_.param<std::string>("map_frame", map_frame_, "lio_global");
    pnh_.param<std::string>("local_frame", local_frame_, "world");
    pnh_.param<std::string>("body_frame", body_frame_, "body");
    pnh_.param<std::string>("map_output_directory", output_directory_, "/tmp/jiangyin_map");
    pnh_.param<std::string>("map_name", map_name_, "competition_map");
    pnh_.param("auto_lock_origin", auto_lock_origin_, true);
    pnh_.param("save_on_shutdown", save_on_shutdown_, true);
    pnh_.param("stability_duration", stability_duration_, 3.0);
    pnh_.param("stability_timeout", stability_timeout_, 30.0);
    pnh_.param("max_linear_speed", max_linear_speed_, 0.05);
    pnh_.param("max_angular_speed", max_angular_speed_, 0.03);
    pnh_.param("max_position_stddev", max_position_stddev_, 0.03);
    pnh_.param("max_yaw_stddev", max_yaw_stddev_, 0.0174533);
    pnh_.param("minimum_cloud_frames", minimum_cloud_frames_, 20);
    pnh_.param("minimum_save_points", minimum_save_points_, 1000);
    pnh_.param("compact_every_frames", compact_every_frames_, 50);
    pnh_.param("map_leaf_size", map_leaf_size_, 0.10);
    pnh_.param("handheld_mapping", handheld_mapping_, false);
    pnh_.param("operator_exclusion/enabled", operator_exclusion_enabled_, true);
    pnh_.param("operator_exclusion/max_odom_age", operator_max_odom_age_, 0.20);
    std::vector<double> exclusion_min{-0.8, -0.8, -2.0};
    std::vector<double> exclusion_max{0.8, 0.8, -0.15};
    pnh_.getParam("operator_exclusion/min", exclusion_min);
    pnh_.getParam("operator_exclusion/max", exclusion_max);
    if (exclusion_min.size() == 3 && exclusion_max.size() == 3) {
      operator_min_ = Eigen::Vector3f(exclusion_min[0], exclusion_min[1], exclusion_min[2]);
      operator_max_ = Eigen::Vector3f(exclusion_max[0], exclusion_max[1], exclusion_max[2]);
    } else {
      ROS_WARN("Invalid operator exclusion bounds; using defaults");
    }
  }

  bool validFrame(const std::string& actual, const std::string& expected,
                  const char* input_name) const {
    if (actual.empty() || actual == expected) return true;
    ROS_WARN_THROTTLE(2.0, "Ignoring %s in frame '%s'; expected '%s'", input_name,
                      actual.c_str(), expected.c_str());
    return false;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (!validFrame(msg->header.frame_id, local_frame_, "odometry")) return;
    if (!msg->child_frame_id.empty() && msg->child_frame_id != body_frame_) {
      ROS_WARN_THROTTLE(2.0, "Odometry child frame is '%s'; expected '%s'",
                        msg->child_frame_id.c_str(), body_frame_.c_str());
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!samples_.empty() && msg->header.stamp < samples_.back().stamp) resetLocked("timestamp rollback");
    latest_odom_ = *msg;
    have_odom_ = true;

    const auto& linear = msg->twist.twist.linear;
    const auto& angular = msg->twist.twist.angular;
    PoseSample sample;
    sample.stamp = msg->header.stamp;
    sample.position = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                      msg->pose.pose.position.z);
    sample.yaw = yawFromPose(msg->pose.pose);
    sample.linear_speed = std::sqrt(linear.x * linear.x + linear.y * linear.y + linear.z * linear.z);
    sample.angular_speed = std::sqrt(angular.x * angular.x + angular.y * angular.y + angular.z * angular.z);
    samples_.push_back(sample);
    while (!samples_.empty() && (sample.stamp - samples_.front().stamp).toSec() > stability_duration_)
      samples_.pop_front();

    if (!origin_locked_) {
      setStateLocked("WAITING_FOR_STABILITY", false);
      if (auto_lock_origin_ && stableLocked()) lockOriginLocked();
      else if (stability_timeout_ > 0.0 && (ros::Time::now() - start_time_).toSec() > stability_timeout_)
        setStateLocked("FAILED_STABILITY_TIMEOUT", false);
    } else {
      publishGlobalOdomLocked(*msg);
    }
  }

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (!validFrame(msg->header.frame_id, local_frame_, "registered cloud")) return;
    Cloud cloud;
    pcl::fromROSMsg(*msg, cloud);
    if (cloud.empty()) return;
    std::lock_guard<std::mutex> lock(mutex_);
    ++cloud_frames_;
    if (!origin_locked_) return;
    Cloud raw_transformed;
    pcl::transformPointCloud(cloud, raw_transformed, map_from_local_);
    *raw_accumulated_ += raw_transformed;

    Cloud filtered_local;
    const double odom_age = std::abs((msg->header.stamp - latest_odom_.header.stamp).toSec());
    if (handheld_mapping_ && operator_exclusion_enabled_ &&
        (!have_odom_ || odom_age > operator_max_odom_age_)) {
      ROS_WARN_THROTTLE(2.0,
                        "Skipping handheld working-map frame: odometry age %.3f s",
                        odom_age);
    } else if (handheld_mapping_ && operator_exclusion_enabled_) {
      const Eigen::Matrix4f body_from_local = poseMatrix(latest_odom_.pose.pose).inverse();
      filtered_local.reserve(cloud.size());
      for (const Point& point : cloud.points) {
        if (!pcl::isFinite(point)) continue;
        const Eigen::Vector4f local(point.x, point.y, point.z, 1.0f);
        const Eigen::Vector3f body = (body_from_local * local).head<3>();
        const bool is_operator =
            (body.array() >= operator_min_.array()).all() &&
            (body.array() <= operator_max_.array()).all();
        if (!is_operator) filtered_local.push_back(point);
      }
    } else {
      filtered_local = cloud;
    }
    Cloud transformed;
    pcl::transformPointCloud(filtered_local, transformed, map_from_local_);
    *accumulated_ += transformed;
    if (compact_every_frames_ > 0 && cloud_frames_ % compact_every_frames_ == 0) {
      Cloud::Ptr compacted(new Cloud);
      pcl::VoxelGrid<Point> voxel;
      voxel.setLeafSize(map_leaf_size_, map_leaf_size_, map_leaf_size_);
      voxel.setInputCloud(accumulated_);
      voxel.filter(*compacted);
      accumulated_.swap(compacted);
      Cloud::Ptr raw_compacted(new Cloud);
      voxel.setInputCloud(raw_accumulated_);
      voxel.filter(*raw_compacted);
      raw_accumulated_.swap(raw_compacted);
    }
    saved_ = false;
    setStateLocked("MAPPING", true);
    publishMapLocked(msg->header.stamp);
  }

  bool stableLocked() const {
    if (!have_odom_ || cloud_frames_ < minimum_cloud_frames_ || samples_.size() < 2) return false;
    const double duration = (samples_.back().stamp - samples_.front().stamp).toSec();
    if (duration < stability_duration_ * 0.95) return false;
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    double mean_sin = 0.0, mean_cos = 0.0;
    for (const PoseSample& sample : samples_) {
      if (sample.linear_speed > max_linear_speed_ || sample.angular_speed > max_angular_speed_) return false;
      mean += sample.position;
      mean_sin += std::sin(sample.yaw);
      mean_cos += std::cos(sample.yaw);
    }
    mean /= samples_.size();
    const double mean_yaw = std::atan2(mean_sin, mean_cos);
    double position_variance = 0.0, yaw_variance = 0.0;
    for (const PoseSample& sample : samples_) {
      position_variance += (sample.position - mean).squaredNorm();
      const double dyaw = std::atan2(std::sin(sample.yaw - mean_yaw),
                                     std::cos(sample.yaw - mean_yaw));
      yaw_variance += dyaw * dyaw;
    }
    const double position_stddev = std::sqrt(position_variance / samples_.size());
    const double yaw_stddev = std::sqrt(yaw_variance / samples_.size());
    return position_stddev <= max_position_stddev_ && yaw_stddev <= max_yaw_stddev_;
  }

  void lockOriginLocked() {
    if (!have_odom_) return;
    const Eigen::Vector3f p(latest_odom_.pose.pose.position.x,
                            latest_odom_.pose.pose.position.y,
                            latest_odom_.pose.pose.position.z);
    const float yaw = static_cast<float>(yawFromPose(latest_odom_.pose.pose));
    map_from_local_ = Eigen::Matrix4f::Identity();
    map_from_local_.block<3, 3>(0, 0) =
        Eigen::AngleAxisf(-yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    map_from_local_.block<3, 1>(0, 3) = -map_from_local_.block<3, 3>(0, 0) * p;
    origin_locked_ = true;
    saved_ = false;
    setStateLocked("ORIGIN_LOCKED", true);
    publishGlobalOdomLocked(latest_odom_);
    ROS_INFO_STREAM("Permanent map origin locked at local position [" << p.transpose()
                    << "], initial yaw=" << yaw);
  }

  void publishGlobalOdomLocked(const nav_msgs::Odometry& local) {
    nav_msgs::Odometry global = local;
    global.header.frame_id = map_frame_;
    global.child_frame_id = body_frame_;
    global.pose.pose = matrixPose(map_from_local_ * poseMatrix(local.pose.pose));
    global_odom_pub_.publish(global);

    const geometry_msgs::Pose pose = matrixPose(map_from_local_);
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
    transform.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y,
                                         pose.orientation.z, pose.orientation.w));
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform, local.header.stamp,
                                                        map_frame_, local_frame_));
  }

  void publishMapLocked(const ros::Time& stamp) {
    if (map_pub_.getNumSubscribers() == 0) return;
    Cloud::Ptr preview(new Cloud);
    pcl::VoxelGrid<Point> voxel;
    voxel.setLeafSize(map_leaf_size_, map_leaf_size_, map_leaf_size_);
    voxel.setInputCloud(accumulated_);
    voxel.filter(*preview);
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*preview, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    map_pub_.publish(msg);
  }

  bool lockService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_odom_) {
      response.success = false;
      response.message = "No valid odometry received";
      return true;
    }
    lockOriginLocked();
    response.success = true;
    response.message = "Permanent origin locked";
    return true;
  }

  bool saveService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    response.success = saveMap(&response.message);
    return true;
  }

  bool resetService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    std::lock_guard<std::mutex> lock(mutex_);
    resetLocked("service request");
    response.success = true;
    response.message = "Origin and accumulated map cleared";
    return true;
  }

  bool saveMap(std::string* message) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!origin_locked_) {
      *message = "Origin is not locked";
      return false;
    }
    if (accumulated_->size() < static_cast<std::size_t>(minimum_save_points_)) {
      *message = "Not enough accumulated points";
      return false;
    }
    setStateLocked("SAVING", false);
    boost::system::error_code ec;
    boost::filesystem::create_directories(output_directory_, ec);
    if (ec) {
      *message = "Cannot create output directory: " + ec.message();
      setStateLocked("FAILED_SAVE", false);
      return false;
    }

    Cloud filtered;
    pcl::VoxelGrid<Point> voxel;
    voxel.setLeafSize(map_leaf_size_, map_leaf_size_, map_leaf_size_);
    voxel.setInputCloud(accumulated_);
    voxel.filter(filtered);
    const std::string base = output_directory_ + "/" + map_name_;
    const std::string pcd_tmp = base + ".pcd.tmp";
    const std::string pcd_path = base + ".pcd";
    const std::string raw_pcd_tmp = base + "_raw.pcd.tmp";
    const std::string raw_pcd_path = base + "_raw.pcd";
    const std::string yaml_tmp = base + ".yaml.tmp";
    const std::string yaml_path = base + ".yaml";
    if (pcl::io::savePCDFileBinary(pcd_tmp, filtered) != 0) {
      *message = "PCD write failed";
      setStateLocked("FAILED_SAVE", false);
      return false;
    }
    if (std::rename(pcd_tmp.c_str(), pcd_path.c_str()) != 0) {
      *message = "PCD atomic rename failed";
      setStateLocked("FAILED_SAVE", false);
      return false;
    }
    Cloud raw_filtered;
    voxel.setInputCloud(raw_accumulated_);
    voxel.filter(raw_filtered);
    if (pcl::io::savePCDFileBinary(raw_pcd_tmp, raw_filtered) != 0 ||
        std::rename(raw_pcd_tmp.c_str(), raw_pcd_path.c_str()) != 0) {
      *message = "Raw PCD write failed";
      setStateLocked("FAILED_SAVE", false);
      return false;
    }

    std::ofstream yaml(yaml_tmp.c_str(), std::ios::trunc);
    if (!yaml) {
      *message = "Metadata write failed";
      setStateLocked("FAILED_SAVE", false);
      return false;
    }
    const geometry_msgs::Pose transform = matrixPose(map_from_local_);
    yaml << std::setprecision(10)
         << "format_version: 1\n"
         << "map_id: " << map_name_ << "\n"
         << "pcd_file: " << map_name_ << ".pcd\n"
         << "raw_pcd_file: " << map_name_ << "_raw.pcd\n"
         << "point_count: " << filtered.size() << "\n"
         << "raw_point_count: " << raw_filtered.size() << "\n"
         << "frame_convention: ENU\n"
         << "origin_definition: initial_vehicle_position_and_heading\n"
         << "map_frame: " << map_frame_ << "\n"
         << "creation_local_frame: " << local_frame_ << "\n"
         << "creation_body_frame: " << body_frame_ << "\n"
         << "transform_map_from_creation_local:\n"
         << "  translation: [" << transform.position.x << ", " << transform.position.y
         << ", " << transform.position.z << "]\n"
         << "  quaternion: [" << transform.orientation.x << ", " << transform.orientation.y
         << ", " << transform.orientation.z << ", " << transform.orientation.w << "]\n"
         << "map_leaf_size: " << map_leaf_size_ << "\n"
         << "handheld_mapping: " << (handheld_mapping_ ? "true" : "false") << "\n"
         << "operator_exclusion_enabled: "
         << ((handheld_mapping_ && operator_exclusion_enabled_) ? "true" : "false") << "\n"
         << "operator_exclusion_min: [" << operator_min_.x() << ", "
         << operator_min_.y() << ", " << operator_min_.z() << "]\n"
         << "operator_exclusion_max: [" << operator_max_.x() << ", "
         << operator_max_.y() << ", " << operator_max_.z() << "]\n";
    yaml.close();
    if (!yaml || std::rename(yaml_tmp.c_str(), yaml_path.c_str()) != 0) {
      *message = "Metadata finalize failed";
      setStateLocked("FAILED_SAVE", false);
      return false;
    }
    saved_ = true;
    setStateLocked("READY", true);
    std::ostringstream result;
    result << "Saved " << filtered.size() << " points to " << pcd_path;
    *message = result.str();
    ROS_INFO_STREAM(*message);
    return true;
  }

  void resetLocked(const std::string& reason) {
    origin_locked_ = false;
    saved_ = false;
    have_odom_ = false;
    cloud_frames_ = 0;
    samples_.clear();
    accumulated_->clear();
    raw_accumulated_->clear();
    start_time_ = ros::Time::now();
    setStateLocked("WAITING_FOR_LIO", false);
    ROS_WARN_STREAM("Map origin reset: " << reason);
  }

  void setState(const std::string& state, bool ready) {
    std::lock_guard<std::mutex> lock(mutex_);
    setStateLocked(state, ready);
  }

  void setStateLocked(const std::string& state, bool ready) {
    if (state == state_ && ready == ready_) return;
    state_ = state;
    ready_ = ready;
    std_msgs::String state_msg;
    state_msg.data = state_;
    state_pub_.publish(state_msg);
    std_msgs::Bool ready_msg;
    ready_msg.data = ready_;
    ready_pub_.publish(ready_msg);
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber odom_sub_, cloud_sub_;
  ros::Publisher state_pub_, ready_pub_, map_pub_, global_odom_pub_;
  ros::ServiceServer lock_srv_, save_srv_, reset_srv_;
  tf::TransformBroadcaster tf_broadcaster_;
  std::mutex mutex_;
  std::deque<PoseSample> samples_;
  Cloud::Ptr accumulated_;
  Cloud::Ptr raw_accumulated_;
  nav_msgs::Odometry latest_odom_;
  Eigen::Matrix4f map_from_local_ = Eigen::Matrix4f::Identity();
  ros::Time start_time_;
  std::string odom_topic_, cloud_topic_, global_odom_topic_, map_cloud_topic_;
  std::string state_topic_, ready_topic_, map_frame_, local_frame_, body_frame_;
  std::string output_directory_, map_name_, state_;
  bool auto_lock_origin_ = true, save_on_shutdown_ = true;
  bool handheld_mapping_ = false, operator_exclusion_enabled_ = true;
  bool have_odom_ = false, origin_locked_ = false, saved_ = false, ready_ = false;
  double stability_duration_ = 3.0, stability_timeout_ = 30.0;
  double max_linear_speed_ = 0.05, max_angular_speed_ = 0.03;
  double max_position_stddev_ = 0.03, max_yaw_stddev_ = 0.0174533;
  double map_leaf_size_ = 0.10;
  double operator_max_odom_age_ = 0.20;
  Eigen::Vector3f operator_min_{-0.8f, -0.8f, -2.0f};
  Eigen::Vector3f operator_max_{0.8f, 0.8f, -0.15f};
  int minimum_cloud_frames_ = 20, minimum_save_points_ = 1000, cloud_frames_ = 0;
  int compact_every_frames_ = 50;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_origin_recorder");
  MapOriginRecorder node;
  ros::spin();
  return 0;
}
