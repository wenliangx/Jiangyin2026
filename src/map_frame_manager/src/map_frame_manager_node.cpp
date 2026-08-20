/**
 * @file map_frame_manager_node.cpp
 * @brief Align any LIO local frame with a prior PCD map.
 *
 * Frame convention:
 *   T_map_pcd   configurable transform applied once while loading the PCD
 *   T_map_world estimated by NDT followed by ICP
 *   T_map_body  = T_map_world * T_world_body
 *
 * /initialpose is interpreted as T_map_body and is converted to an initial
 * registration guess using the latest RA-LIO odometry.
 */

#include <cmath>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

namespace {

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

Eigen::Matrix4f poseMatrix(const geometry_msgs::Pose& pose) {
  Eigen::Quaternionf q(static_cast<float>(pose.orientation.w),
                       static_cast<float>(pose.orientation.x),
                       static_cast<float>(pose.orientation.y),
                       static_cast<float>(pose.orientation.z));
  if (q.norm() < 1e-6f) q = Eigen::Quaternionf::Identity();
  q.normalize();
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = q.toRotationMatrix();
  transform(0, 3) = static_cast<float>(pose.position.x);
  transform(1, 3) = static_cast<float>(pose.position.y);
  transform(2, 3) = static_cast<float>(pose.position.z);
  return transform;
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

Eigen::Matrix4f xyzRpy(double x, double y, double z, double roll, double pitch,
                       double yaw) {
  const Eigen::AngleAxisf rx(static_cast<float>(roll), Eigen::Vector3f::UnitX());
  const Eigen::AngleAxisf ry(static_cast<float>(pitch), Eigen::Vector3f::UnitY());
  const Eigen::AngleAxisf rz(static_cast<float>(yaw), Eigen::Vector3f::UnitZ());
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = (rz * ry * rx).toRotationMatrix();
  transform(0, 3) = static_cast<float>(x);
  transform(1, 3) = static_cast<float>(y);
  transform(2, 3) = static_cast<float>(z);
  return transform;
}

class MapRelocalization {
 public:
  MapRelocalization() : nh_(), pnh_("~"), prior_map_(new Cloud), initialized_(false),
                        have_odom_(false), have_guess_(false), running_(false) {
    loadParameters();
    const std::string resolved_input = nh_.resolveName(odom_topic_);
    const std::string resolved_global = nh_.resolveName(global_odom_topic_);
    const std::string resolved_original = nh_.resolveName(original_odom_topic_);
    if (resolved_input == resolved_global ||
        (publish_to_original_odom_topic_ &&
         (resolved_input == resolved_original || resolved_global == resolved_original))) {
      ROS_FATAL_STREAM("Conflicting resolved odometry topics: input='" << resolved_input
                       << "', global='" << resolved_global << "', compatibility='"
                       << resolved_original << "'. Every active input/output must use a "
                       << "different ROS topic.");
      ros::shutdown();
      return;
    }
    if (!loadMap()) {
      ROS_FATAL_STREAM("Cannot start map relocalization with PCD: " << map_file_);
      ros::shutdown();
      return;
    }

    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(map_topic_, 1, true);
    aligned_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(aligned_topic_, 1);
    global_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(global_odom_topic_, 20);
    if (publish_to_original_odom_topic_) {
      original_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(original_odom_topic_, 20);
    }
    status_pub_ = nh_.advertise<std_msgs::Bool>(status_topic_, 1, true);
    state_pub_ = nh_.advertise<std_msgs::String>(state_topic_, 1, true);
    fitness_pub_ = nh_.advertise<std_msgs::Float64>(fitness_topic_, 1, true);
    odom_sub_ = nh_.subscribe(odom_topic_, 50, &MapRelocalization::odomCallback, this);
    cloud_sub_ = nh_.subscribe(cloud_topic_, 10, &MapRelocalization::cloudCallback, this);
    initial_pose_sub_ = nh_.subscribe(initial_pose_topic_, 1,
                                      &MapRelocalization::initialPoseCallback, this);
    relocalize_srv_ = pnh_.advertiseService("relocalize", &MapRelocalization::relocalizeService,
                                            this);
    reset_srv_ = pnh_.advertiseService("reset", &MapRelocalization::resetService, this);

    publishMap();
    publishStatus(false);
    publishState("WAITING_FOR_LIO");
    ROS_INFO_STREAM("Prior map ready: " << prior_map_->size() << " points in frame '"
                    << map_frame_ << "'. Waiting for configured automatic seed or /initialpose.");
  }

 private:
  void loadParameters() {
    pnh_.param<std::string>("map_file", map_file_, "");
    pnh_.param<std::string>("map_frame", map_frame_, "lio_global");
    pnh_.param<std::string>("local_frame", local_frame_, "world");
    pnh_.param<std::string>("body_frame", body_frame_, "body");
    pnh_.param<std::string>("odom_topic", odom_topic_, "/Odometry");
    pnh_.param<std::string>("cloud_topic", cloud_topic_, "/cloud_registered");
    pnh_.param<std::string>("initial_pose_topic", initial_pose_topic_, "/initialpose");
    pnh_.param<std::string>("global_odom_topic", global_odom_topic_,
                            "/Odometry_lio_global");
    pnh_.param<std::string>("original_odom_topic", original_odom_topic_, "/Odometry");
    pnh_.param("publish_to_original_odom_topic", publish_to_original_odom_topic_, false);
    pnh_.param<std::string>("map_topic", map_topic_, "/prior_map");
    pnh_.param<std::string>("aligned_topic", aligned_topic_, "/relocalization/aligned_cloud");
    pnh_.param<std::string>("status_topic", status_topic_, "/relocalization/initialized");
    pnh_.param<std::string>("state_topic", state_topic_, "/map_frame_manager/state");
    pnh_.param<std::string>("fitness_topic", fitness_topic_, "/map_frame_manager/fitness");
    pnh_.param("map_leaf_size", map_leaf_size_, 0.20);
    pnh_.param("scan_leaf_size", scan_leaf_size_, 0.15);
    pnh_.param("target_crop_radius", target_crop_radius_, 30.0);
    pnh_.param("accumulate_frames", accumulate_frames_, 8);
    pnh_.param("ndt_resolution", ndt_resolution_, 1.0);
    pnh_.param("ndt_step_size", ndt_step_size_, 0.1);
    pnh_.param("ndt_max_iterations", ndt_max_iterations_, 40);
    pnh_.param("icp_max_correspondence", icp_max_correspondence_, 0.8);
    pnh_.param("icp_max_iterations", icp_max_iterations_, 40);
    pnh_.param("max_fitness_score", max_fitness_score_, 0.30);
    pnh_.param("max_translation_correction", max_translation_correction_, 5.0);
    pnh_.param("max_yaw_correction", max_yaw_correction_, 0.785398);
    pnh_.param("auto_initialize", auto_initialize_, false);

    std::vector<double> origin;
    pnh_.param<std::vector<double>>("map_transform", origin,
                                   std::vector<double>{0, 0, 0, 0, 0, 0});
    if (origin.size() != 6) {
      ROS_WARN("~map_transform must be [x,y,z,roll,pitch,yaw]; using identity");
      origin.assign(6, 0.0);
    }
    map_from_pcd_ = xyzRpy(origin[0], origin[1], origin[2], origin[3], origin[4],
                           origin[5]);

    std::vector<double> initial;
    pnh_.param<std::vector<double>>("initial_pose", initial,
                                   std::vector<double>{0, 0, 0, 0, 0, 0});
    if (initial.size() == 6) {
      configured_map_from_body_ = xyzRpy(initial[0], initial[1], initial[2], initial[3],
                                         initial[4], initial[5]);
    } else {
      configured_map_from_body_ = Eigen::Matrix4f::Identity();
    }
  }

  bool loadMap() {
    if (map_file_.empty()) {
      ROS_ERROR("~map_file is empty");
      return false;
    }
    Cloud::Ptr raw(new Cloud);
    if (pcl::io::loadPCDFile<Point>(map_file_, *raw) != 0 || raw->empty()) return false;
    Cloud::Ptr transformed(new Cloud);
    pcl::transformPointCloud(*raw, *transformed, map_from_pcd_);
    pcl::VoxelGrid<Point> voxel;
    voxel.setLeafSize(map_leaf_size_, map_leaf_size_, map_leaf_size_);
    voxel.setInputCloud(transformed);
    voxel.filter(*prior_map_);
    return !prior_map_->empty();
  }

  void publishMap() {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*prior_map_, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = map_frame_;
    map_pub_.publish(msg);
  }

  void publishStatus(bool value) {
    std_msgs::Bool status;
    status.data = value;
    status_pub_.publish(status);
  }

  void publishState(const std::string& value) {
    std_msgs::String state;
    state.data = value;
    state_pub_.publish(state);
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (!msg->header.frame_id.empty() && msg->header.frame_id != local_frame_) {
      ROS_WARN_THROTTLE(2.0, "Ignoring odometry in frame '%s'; configured local frame is '%s'",
                        msg->header.frame_id.c_str(), local_frame_.c_str());
      return;
    }
    if (!msg->child_frame_id.empty() && msg->child_frame_id != body_frame_) {
      ROS_WARN_THROTTLE(2.0, "Odometry child frame is '%s'; configured body frame is '%s'",
                        msg->child_frame_id.c_str(), body_frame_.c_str());
    }
    // Compatibility output deliberately preserves the raw local pose and its
    // world axes. Relocalization remains available through the global output
    // and the lio_global -> world TF.
    if (publish_to_original_odom_topic_) original_odom_pub_.publish(*msg);
    std::lock_guard<std::mutex> lock(mutex_);
    latest_odom_ = *msg;
    world_from_body_ = poseMatrix(msg->pose.pose);
    have_odom_ = true;
    if (auto_initialize_ && !have_guess_) {
      initial_map_from_body_ = configured_map_from_body_;
      initial_map_from_world_guess_ = initial_map_from_body_ * world_from_body_.inverse();
      have_guess_ = true;
      publishState("COLLECTING_SCAN");
    }
    if (initialized_) publishGlobalOdomLocked(*msg);
  }

  void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
      ROS_WARN_STREAM("Ignoring /initialpose in frame '" << msg->header.frame_id
                      << "'; expected '" << map_frame_ << "'");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    initial_map_from_body_ = poseMatrix(msg->pose.pose);
    if (!have_odom_) {
      ROS_WARN("Received /initialpose before local odometry; send it again after odometry starts");
      return;
    }
    initial_map_from_world_guess_ = initial_map_from_body_ * world_from_body_.inverse();
    have_guess_ = true;
    initialized_ = false;
    accumulated_.clear();
    publishStatus(false);
    publishState("COLLECTING_SCAN");
    ROS_INFO("Received relocalization seed; collecting scans");
  }

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (!msg->header.frame_id.empty() && msg->header.frame_id != local_frame_) {
      ROS_WARN_THROTTLE(2.0, "Ignoring registered cloud in frame '%s'; expected '%s'",
                        msg->header.frame_id.c_str(), local_frame_.c_str());
      return;
    }
    Cloud::Ptr cloud(new Cloud);
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->empty()) return;

    Eigen::Matrix4f guess;
    Cloud::Ptr source(new Cloud);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!have_odom_ || !have_guess_ || initialized_ || running_) return;
      accumulated_.push_back(cloud);
      while (static_cast<int>(accumulated_.size()) > accumulate_frames_) accumulated_.pop_front();
      if (static_cast<int>(accumulated_.size()) < accumulate_frames_) return;
      for (const Cloud::Ptr& frame : accumulated_) *source += *frame;
      // This guess was captured at the instant /initialpose arrived.  The
      // accumulated source scans already remain fixed in RA-LIO's world frame.
      guess = initial_map_from_world_guess_;
      running_ = true;
      publishState("MATCHING");
    }

    const bool success = registerCloud(source, guess, msg->header.stamp);
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
    if (!success) {
      accumulated_.clear();
      publishState("FAILED_RETRYING");
    }
  }

  bool registerCloud(const Cloud::Ptr& input, const Eigen::Matrix4f& guess,
                     const ros::Time& stamp) {
    Cloud::Ptr source(new Cloud);
    pcl::VoxelGrid<Point> voxel;
    voxel.setLeafSize(scan_leaf_size_, scan_leaf_size_, scan_leaf_size_);
    voxel.setInputCloud(input);
    voxel.filter(*source);
    if (source->size() < 100) {
      ROS_WARN("Relocalization source cloud has too few points");
      return false;
    }

    Cloud::Ptr guessed_source(new Cloud);
    pcl::transformPointCloud(*source, *guessed_source, guess);
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    for (const Point& point : guessed_source->points)
      center += Eigen::Vector3f(point.x, point.y, point.z);
    center /= static_cast<float>(guessed_source->size());
    Cloud::Ptr target(new Cloud);
    pcl::CropBox<Point> crop;
    crop.setInputCloud(prior_map_);
    const float radius = static_cast<float>(target_crop_radius_);
    crop.setMin(Eigen::Vector4f(center.x() - radius, center.y() - radius,
                               center.z() - radius, 1.0f));
    crop.setMax(Eigen::Vector4f(center.x() + radius, center.y() + radius,
                               center.z() + radius, 1.0f));
    crop.filter(*target);
    if (target->size() < 100) {
      ROS_WARN("Relocalization target crop has too few points; check initial position");
      return false;
    }

    pcl::NormalDistributionsTransform<Point, Point> ndt;
    ndt.setResolution(ndt_resolution_);
    ndt.setStepSize(ndt_step_size_);
    ndt.setMaximumIterations(ndt_max_iterations_);
    ndt.setTransformationEpsilon(0.01);
    ndt.setInputSource(source);
    ndt.setInputTarget(target);
    Cloud ndt_output;
    ndt.align(ndt_output, guess);
    if (!ndt.hasConverged()) {
      ROS_WARN("NDT did not converge; adjust /initialpose or NDT resolution");
      return false;
    }

    pcl::IterativeClosestPoint<Point, Point> icp;
    icp.setMaxCorrespondenceDistance(icp_max_correspondence_);
    icp.setMaximumIterations(icp_max_iterations_);
    icp.setTransformationEpsilon(1e-7);
    icp.setEuclideanFitnessEpsilon(1e-5);
    icp.setInputSource(source);
    icp.setInputTarget(target);
    Cloud aligned;
    icp.align(aligned, ndt.getFinalTransformation());
    const double score = icp.getFitnessScore(icp_max_correspondence_);
    const Eigen::Matrix4f result = icp.getFinalTransformation();
    const Eigen::Matrix4f correction = result * guess.inverse();
    const double translation_correction = correction.block<3, 1>(0, 3).norm();
    const double yaw_correction = std::fabs(std::atan2(correction(1, 0), correction(0, 0)));
    std_msgs::Float64 fitness_msg;
    fitness_msg.data = score;
    fitness_pub_.publish(fitness_msg);
    if (!icp.hasConverged() || !std::isfinite(score) || score > max_fitness_score_ ||
        translation_correction > max_translation_correction_ ||
        yaw_correction > max_yaw_correction_) {
      ROS_WARN_STREAM("ICP rejected: converged=" << icp.hasConverged()
                      << ", fitness=" << score << ", translation correction="
                      << translation_correction << ", yaw correction=" << yaw_correction);
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      map_from_world_ = result;
      initialized_ = true;
      accumulated_.clear();
      publishGlobalOdomLocked(latest_odom_);
    }
    sensor_msgs::PointCloud2 aligned_msg;
    pcl::toROSMsg(aligned, aligned_msg);
    aligned_msg.header.stamp = stamp;
    aligned_msg.header.frame_id = map_frame_;
    aligned_pub_.publish(aligned_msg);
    publishStatus(true);
    publishState("READY");
    ROS_INFO_STREAM("Relocalization succeeded, ICP fitness=" << score);
    return true;
  }

  bool relocalizeService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_odom_) {
      response.success = false;
      response.message = "No valid local odometry received";
      return true;
    }
    initial_map_from_body_ = configured_map_from_body_;
    initial_map_from_world_guess_ = initial_map_from_body_ * world_from_body_.inverse();
    have_guess_ = true;
    initialized_ = false;
    running_ = false;
    accumulated_.clear();
    publishStatus(false);
    publishState("COLLECTING_SCAN");
    response.success = true;
    response.message = "Relocalization requested";
    return true;
  }

  bool resetService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    std::lock_guard<std::mutex> lock(mutex_);
    initialized_ = false;
    have_guess_ = false;
    running_ = false;
    accumulated_.clear();
    publishStatus(false);
    publishState("WAITING_FOR_SEED");
    response.success = true;
    response.message = "Global initialization cleared";
    return true;
  }

  void publishGlobalOdomLocked(const nav_msgs::Odometry& local) {
    nav_msgs::Odometry global = local;
    global.header.frame_id = map_frame_;
    global.child_frame_id = body_frame_;
    const Eigen::Matrix4f map_from_body = map_from_world_ * poseMatrix(local.pose.pose);
    global.pose.pose = matrixPose(map_from_body);

    // nav_msgs/Odometry twist is conventionally expressed in child_frame_id.
    // Preserve it unchanged; rotate only if a producer explicitly uses world-frame twist.
    global_odom_pub_.publish(global);

    tf::Transform tf_map_world;
    const geometry_msgs::Pose pose = matrixPose(map_from_world_);
    tf_map_world.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
    tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z,
                     pose.orientation.w);
    tf_map_world.setRotation(q);
    tf_broadcaster_.sendTransform(tf::StampedTransform(
        tf_map_world, local.header.stamp, map_frame_, local_frame_));
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber odom_sub_, cloud_sub_, initial_pose_sub_;
  ros::Publisher map_pub_, aligned_pub_, global_odom_pub_, original_odom_pub_;
  ros::Publisher status_pub_, state_pub_, fitness_pub_;
  ros::ServiceServer relocalize_srv_, reset_srv_;
  tf::TransformBroadcaster tf_broadcaster_;
  std::mutex mutex_;
  Cloud::Ptr prior_map_;
  std::deque<Cloud::Ptr> accumulated_;
  nav_msgs::Odometry latest_odom_;
  Eigen::Matrix4f map_from_pcd_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f configured_map_from_body_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f initial_map_from_body_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f initial_map_from_world_guess_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f world_from_body_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f map_from_world_ = Eigen::Matrix4f::Identity();
  std::string map_file_, map_frame_, local_frame_, body_frame_;
  std::string odom_topic_, cloud_topic_, initial_pose_topic_, global_odom_topic_;
  std::string original_odom_topic_;
  std::string map_topic_, aligned_topic_, status_topic_, state_topic_, fitness_topic_;
  double map_leaf_size_, scan_leaf_size_, target_crop_radius_, ndt_resolution_, ndt_step_size_;
  double icp_max_correspondence_, max_fitness_score_;
  double max_translation_correction_, max_yaw_correction_;
  int accumulate_frames_, ndt_max_iterations_, icp_max_iterations_;
  bool auto_initialize_, initialized_, have_odom_, have_guess_, running_;
  bool publish_to_original_odom_topic_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_frame_manager");
  MapRelocalization node;
  ros::spin();
  return 0;
}
