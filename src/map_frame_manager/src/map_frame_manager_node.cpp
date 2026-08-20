/**
 * @file map_frame_manager_node.cpp
 * @brief Fixed-origin, yaw-only prior-map initialization for LIO.
 */

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_broadcaster.h>

namespace {
using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;
constexpr double PI = 3.14159265358979323846;

double rad(double deg) { return deg * PI / 180.0; }
double wrap(double yaw) { return std::atan2(std::sin(yaw), std::cos(yaw)); }

Eigen::Matrix4f poseMatrix(const geometry_msgs::Pose& pose) {
  Eigen::Quaternionf q(pose.orientation.w, pose.orientation.x,
                       pose.orientation.y, pose.orientation.z);
  if (q.norm() < 1e-6f) q = Eigen::Quaternionf::Identity();
  q.normalize();
  Eigen::Matrix4f t = Eigen::Matrix4f::Identity();
  t.block<3, 3>(0, 0) = q.toRotationMatrix();
  t.block<3, 1>(0, 3) << pose.position.x, pose.position.y, pose.position.z;
  return t;
}

geometry_msgs::Pose matrixPose(const Eigen::Matrix4f& t) {
  geometry_msgs::Pose pose;
  pose.position.x = t(0, 3); pose.position.y = t(1, 3); pose.position.z = t(2, 3);
  Eigen::Quaternionf q(t.block<3, 3>(0, 0));
  q.normalize();
  pose.orientation.x = q.x(); pose.orientation.y = q.y();
  pose.orientation.z = q.z(); pose.orientation.w = q.w();
  return pose;
}

struct YawScore {
  double yaw = 0.0;
  double cost = std::numeric_limits<double>::infinity();
  double rmse = std::numeric_limits<double>::infinity();
  double overlap = 0.0;
};

class YawOnlyRelocalization {
 public:
  YawOnlyRelocalization() : nh_(), pnh_("~"), map_(new Cloud) {
    loadParameters();
    if (publish_original_ && nh_.resolveName(odom_topic_) == nh_.resolveName(original_topic_)) {
      ROS_FATAL_STREAM("Yaw-only input and original output resolve to the same topic: "
                       << nh_.resolveName(odom_topic_));
      ros::shutdown();
      return;
    }
    if (!loadMap()) {
      ROS_FATAL_STREAM("Cannot load fixed-origin map: " << map_file_);
      ros::shutdown();
      return;
    }
    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(map_topic_, 1, true);
    aligned_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(aligned_topic_, 1);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(global_odom_topic_, 20);
    if (publish_original_)
      original_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(original_topic_, 20);
    status_pub_ = nh_.advertise<std_msgs::Bool>(status_topic_, 1, true);
    state_pub_ = nh_.advertise<std_msgs::String>(state_topic_, 1, true);
    fitness_pub_ = nh_.advertise<std_msgs::Float64>(fitness_topic_, 1, true);
    transform_pub_ = nh_.advertise<geometry_msgs::TransformStamped>(transform_topic_, 1, true);
    odom_sub_ = nh_.subscribe(odom_topic_, 50, &YawOnlyRelocalization::odomCallback, this);
    cloud_sub_ = nh_.subscribe(cloud_topic_, 10, &YawOnlyRelocalization::cloudCallback, this);
    relocalize_srv_ = pnh_.advertiseService("relocalize",
                                            &YawOnlyRelocalization::relocalizeService, this);
    reset_srv_ = pnh_.advertiseService("reset", &YawOnlyRelocalization::resetService, this);
    publishMap(); publishStatus(false); publishState("WAITING_FOR_LIO");
    ROS_INFO_STREAM("Fixed-origin yaw map ready: " << map_->size() << " points in frame '"
                    << map_frame_ << "'. Manual /initialpose is disabled.");
  }

 private:
  void loadParameters() {
    pnh_.param<std::string>("map_file", map_file_, "");
    pnh_.param<std::string>("map_frame", map_frame_, "lio_global");
    pnh_.param<std::string>("local_frame", local_frame_, "world");
    pnh_.param<std::string>("body_frame", body_frame_, "body");
    pnh_.param<std::string>("odom_topic", odom_topic_, "/Odometry");
    pnh_.param<std::string>("cloud_topic", cloud_topic_, "/cloud_registered");
    pnh_.param<std::string>("global_odom_topic", global_odom_topic_, "/Odometry_lio_global");
    pnh_.param<std::string>("original_odom_topic", original_topic_, "/Odometry");
    pnh_.param("publish_to_original_odom_topic", publish_original_, false);
    pnh_.param<std::string>("map_topic", map_topic_, "/prior_map");
    pnh_.param<std::string>("aligned_topic", aligned_topic_, "/relocalization/aligned_cloud");
    pnh_.param<std::string>("status_topic", status_topic_, "/relocalization/initialized");
    pnh_.param<std::string>("state_topic", state_topic_, "/map_frame_manager/state");
    pnh_.param<std::string>("fitness_topic", fitness_topic_, "/map_frame_manager/fitness");
    pnh_.param<std::string>("transform_topic", transform_topic_,
                            "/relocalization/world_from_lio_local");
    pnh_.param("map_leaf_size", map_leaf_, 0.08);
    pnh_.param("scan_leaf_size", scan_leaf_, 0.08);
    pnh_.param("origin_map_radius", origin_radius_, 15.0);
    pnh_.param("accumulate_frames", frames_, 20);
    pnh_.param("coarse_yaw_step_deg", coarse_step_, 2.0);
    pnh_.param("fine_yaw_step_deg", fine_step_, 0.05);
    pnh_.param("fine_yaw_window_deg", fine_window_, 2.0);
    pnh_.param("max_correspondence_distance", max_corr_, 0.40);
    pnh_.param("minimum_overlap_ratio", min_overlap_, 0.45);
    pnh_.param("maximum_rmse", max_rmse_, 0.20);
    pnh_.param("ambiguity_separation_deg", ambiguity_sep_, 15.0);
    pnh_.param("minimum_score_margin", min_margin_, 0.02);
    pnh_.param("overlap_penalty_weight", overlap_weight_, 0.50);
    pnh_.param("required_consistent_results", required_results_, 3);
    pnh_.param("maximum_yaw_consistency_deg", max_consistency_, 0.30);
    pnh_.param("max_linear_speed", max_linear_speed_, 0.05);
    pnh_.param("max_angular_speed", max_angular_speed_, 0.03);
  }

  bool loadMap() {
    if (map_file_.empty() || map_leaf_ <= 0 || scan_leaf_ <= 0 || frames_ < 1 ||
        coarse_step_ <= 0 || fine_step_ <= 0 || fine_window_ <= 0 || max_corr_ <= 0)
      return false;
    Cloud::Ptr raw(new Cloud), cropped(new Cloud);
    if (pcl::io::loadPCDFile<Point>(map_file_, *raw) != 0 || raw->empty()) return false;
    pcl::CropBox<Point> crop;
    crop.setInputCloud(raw);
    const float r = origin_radius_;
    crop.setMin(Eigen::Vector4f(-r, -r, -r, 1));
    crop.setMax(Eigen::Vector4f(r, r, r, 1));
    crop.filter(*cropped);
    pcl::VoxelGrid<Point> voxel;
    voxel.setLeafSize(map_leaf_, map_leaf_, map_leaf_);
    voxel.setInputCloud(cropped); voxel.filter(*map_);
    if (map_->size() < 100) return false;
    tree_.setInputCloud(map_);
    return true;
  }

  void publishMap() {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*map_, msg); msg.header.stamp = ros::Time::now();
    msg.header.frame_id = map_frame_; map_pub_.publish(msg);
  }
  void publishState(const std::string& value) {
    std_msgs::String msg; msg.data = value; state_pub_.publish(msg);
  }
  void publishStatus(bool value) {
    std_msgs::Bool msg; msg.data = value; status_pub_.publish(msg);
  }
  bool validFrame(const std::string& actual, const std::string& expected,
                  const char* name) const {
    if (actual.empty() || actual == expected) return true;
    ROS_WARN_THROTTLE(2.0, "Ignoring %s in frame '%s'; expected '%s'", name,
                      actual.c_str(), expected.c_str());
    return false;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (!validFrame(msg->header.frame_id, local_frame_, "odometry")) return;
    std::lock_guard<std::mutex> lock(mutex_);
    latest_odom_ = *msg; local_from_body_ = poseMatrix(msg->pose.pose); have_odom_ = true;
    if (initialized_) publishOdomLocked(*msg);
    else if (!running_) publishState("COLLECTING_SCAN");
  }

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (!validFrame(msg->header.frame_id, local_frame_, "registered cloud")) return;
    Cloud::Ptr cloud(new Cloud), source(new Cloud);
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->empty()) return;
    Eigen::Vector3f anchor;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!have_odom_ || initialized_ || running_) return;
      const auto& v = latest_odom_.twist.twist.linear;
      const auto& w = latest_odom_.twist.twist.angular;
      if (std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z) > max_linear_speed_ ||
          std::sqrt(w.x*w.x + w.y*w.y + w.z*w.z) > max_angular_speed_) {
        scans_.clear(); yaws_.clear(); publishState("WAITING_FOR_STABILITY"); return;
      }
      scans_.push_back(cloud);
      while (static_cast<int>(scans_.size()) > frames_) scans_.pop_front();
      if (static_cast<int>(scans_.size()) < frames_) return;
      for (const Cloud::Ptr& scan : scans_) *source += *scan;
      scans_.clear(); anchor = local_from_body_.block<3, 1>(0, 3);
      running_ = true; publishState("MATCHING_YAW");
    }
    const YawScore result = estimateYaw(source, anchor);
    finishEstimate(result, source, msg->header.stamp);
  }

  YawScore score(const Cloud::ConstPtr& source, const Eigen::Vector3f& anchor,
                 double yaw) const {
    const float c = std::cos(yaw), s = std::sin(yaw), max_sq = max_corr_ * max_corr_;
    double sum = 0; std::size_t inliers = 0;
    std::vector<int> index(1); std::vector<float> distance(1);
    for (const Point& p : source->points) {
      const float x = p.x - anchor.x(), y = p.y - anchor.y();
      Point q = p; q.x = c*x - s*y; q.y = s*x + c*y; q.z = p.z - anchor.z();
      if (tree_.nearestKSearch(q, 1, index, distance) > 0 && distance[0] <= max_sq) {
        sum += distance[0]; ++inliers;
      }
    }
    YawScore out; out.yaw = wrap(yaw);
    out.overlap = source->empty() ? 0.0 : static_cast<double>(inliers) / source->size();
    if (inliers) out.rmse = std::sqrt(sum / inliers);
    out.cost = out.rmse + overlap_weight_ * (1.0 - out.overlap);
    return out;
  }

  YawScore estimateYaw(const Cloud::Ptr& raw, const Eigen::Vector3f& anchor) {
    Cloud::Ptr source(new Cloud);
    pcl::VoxelGrid<Point> voxel; voxel.setLeafSize(scan_leaf_, scan_leaf_, scan_leaf_);
    voxel.setInputCloud(raw); voxel.filter(*source);
    if (source->size() < 100) return YawScore();
    std::vector<YawScore> coarse;
    for (double deg = -180; deg < 180; deg += coarse_step_)
      coarse.push_back(score(source, anchor, rad(deg)));
    std::sort(coarse.begin(), coarse.end(),
              [](const YawScore& a, const YawScore& b) { return a.cost < b.cost; });
    YawScore best = coarse.front(), second;
    for (const YawScore& candidate : coarse) {
      if (std::fabs(wrap(candidate.yaw - best.yaw)) >= rad(ambiguity_sep_)) {
        second = candidate; break;
      }
    }
    const double coarse_yaw = best.yaw;
    for (double offset = -fine_window_; offset <= fine_window_ + fine_step_/2;
         offset += fine_step_) {
      const YawScore candidate = score(source, anchor, coarse_yaw + rad(offset));
      if (candidate.cost < best.cost) best = candidate;
    }
    const double margin = second.cost - best.cost;
    if (!std::isfinite(best.cost) || best.overlap < min_overlap_ ||
        best.rmse > max_rmse_ || !std::isfinite(margin) || margin < min_margin_) {
      ROS_WARN_STREAM("Yaw rejected: yaw=" << best.yaw*180/PI << " deg, rmse="
                      << best.rmse << ", overlap=" << best.overlap << ", margin=" << margin);
      best.cost = std::numeric_limits<double>::infinity();
    }
    return best;
  }

  void finishEstimate(const YawScore& result, const Cloud::Ptr& source,
                      const ros::Time& stamp) {
    std::lock_guard<std::mutex> lock(mutex_); running_ = false;
    std_msgs::Float64 fitness; fitness.data = result.rmse; fitness_pub_.publish(fitness);
    if (!std::isfinite(result.cost)) {
      yaws_.clear(); publishState("FAILED_RETRYING"); return;
    }
    if (!yaws_.empty() && std::fabs(wrap(result.yaw - yaws_.back())) > rad(max_consistency_))
      yaws_.clear();
    yaws_.push_back(result.yaw);
    if (static_cast<int>(yaws_.size()) < required_results_) {
      publishState("VERIFYING_YAW"); return;
    }
    double ss = 0, cc = 0;
    for (double yaw : yaws_) { ss += std::sin(yaw); cc += std::cos(yaw); }
    const double yaw = std::atan2(ss, cc);
    map_from_local_ = Eigen::Matrix4f::Identity();
    map_from_local_.block<3, 3>(0, 0) =
        Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    // Translation is determined, never optimized: current body becomes (0,0,0).
    const Eigen::Vector3f current_body = local_from_body_.block<3, 1>(0, 3);
    map_from_local_.block<3, 1>(0, 3) =
        -map_from_local_.block<3, 3>(0, 0) * current_body;
    initialized_ = true; publishTransformLocked(stamp); publishOdomLocked(latest_odom_);
    publishStatus(true); publishState("READY");
    Cloud aligned; pcl::transformPointCloud(*source, aligned, map_from_local_);
    sensor_msgs::PointCloud2 msg; pcl::toROSMsg(aligned, msg);
    msg.header.stamp = stamp; msg.header.frame_id = map_frame_; aligned_pub_.publish(msg);
    ROS_INFO_STREAM("Yaw-only initialization ready: yaw=" << yaw*180/PI
                    << " deg, rmse=" << result.rmse << ", overlap=" << result.overlap);
  }

  bool relocalizeService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    std::lock_guard<std::mutex> lock(mutex_); clearLocked();
    publishState(have_odom_ ? "COLLECTING_SCAN" : "WAITING_FOR_LIO");
    res.success = true; res.message = "Automatic fixed-origin yaw initialization requested";
    return true;
  }
  bool resetService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    std::lock_guard<std::mutex> lock(mutex_); clearLocked(); publishState("WAITING_FOR_LIO");
    res.success = true; res.message = "Yaw initialization cleared"; return true;
  }
  void clearLocked() {
    initialized_ = false; running_ = false; scans_.clear(); yaws_.clear(); publishStatus(false);
  }

  void publishOdomLocked(const nav_msgs::Odometry& local) {
    nav_msgs::Odometry global = local; global.header.frame_id = map_frame_;
    global.child_frame_id = body_frame_;
    global.pose.pose = matrixPose(map_from_local_ * poseMatrix(local.pose.pose));
    odom_pub_.publish(global);
    if (publish_original_) original_odom_pub_.publish(global);
    publishTransformLocked(local.header.stamp);
  }
  void publishTransformLocked(const ros::Time& stamp) {
    const geometry_msgs::Pose pose = matrixPose(map_from_local_);
    geometry_msgs::TransformStamped msg; msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_; msg.child_frame_id = local_frame_;
    msg.transform.translation.x = pose.position.x;
    msg.transform.translation.y = pose.position.y;
    msg.transform.translation.z = pose.position.z; msg.transform.rotation = pose.orientation;
    transform_pub_.publish(msg);
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
    transform.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y,
                                         pose.orientation.z, pose.orientation.w));
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform, stamp, map_frame_, local_frame_));
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber odom_sub_, cloud_sub_;
  ros::Publisher map_pub_, aligned_pub_, odom_pub_, original_odom_pub_;
  ros::Publisher status_pub_, state_pub_, fitness_pub_, transform_pub_;
  ros::ServiceServer relocalize_srv_, reset_srv_;
  tf::TransformBroadcaster tf_broadcaster_;
  mutable pcl::KdTreeFLANN<Point> tree_;
  Cloud::Ptr map_; std::deque<Cloud::Ptr> scans_; std::vector<double> yaws_;
  nav_msgs::Odometry latest_odom_;
  Eigen::Matrix4f local_from_body_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f map_from_local_ = Eigen::Matrix4f::Identity();
  std::mutex mutex_;
  std::string map_file_, map_frame_, local_frame_, body_frame_, odom_topic_, cloud_topic_;
  std::string global_odom_topic_, original_topic_, map_topic_, aligned_topic_, status_topic_, state_topic_;
  std::string fitness_topic_, transform_topic_;
  double map_leaf_=0.08, scan_leaf_=0.08, origin_radius_=15, coarse_step_=2;
  double fine_step_=0.05, fine_window_=2, max_corr_=0.4, min_overlap_=0.45;
  double max_rmse_=0.2, ambiguity_sep_=15, min_margin_=0.02, overlap_weight_=0.5;
  double max_consistency_=0.3, max_linear_speed_=0.05, max_angular_speed_=0.03;
  int frames_=20, required_results_=3;
  bool publish_original_=false, have_odom_=false, initialized_=false, running_=false;
};
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_frame_manager");
  YawOnlyRelocalization node;
  ros::spin();
  return 0;
}
