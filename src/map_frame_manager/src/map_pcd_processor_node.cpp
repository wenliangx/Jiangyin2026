/**
 * @file map_pcd_processor_node.cpp
 * @brief Reproducible offline cleanup and derivation of relocalization/planning PCDs.
 */

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <pcl/common/point_tests.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

namespace {

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

bool loadVec3(ros::NodeHandle& nh, const std::string& name,
              const std::vector<double>& fallback, Eigen::Vector4f* value) {
  std::vector<double> raw = fallback;
  nh.getParam(name, raw);
  if (raw.size() != 3) {
    ROS_ERROR_STREAM("Parameter " << name << " must contain exactly three numbers");
    return false;
  }
  *value = Eigen::Vector4f(raw[0], raw[1], raw[2], 1.0f);
  return true;
}

bool saveAtomic(const std::string& path, const Cloud& cloud) {
  const std::string temporary = path + ".tmp";
  return pcl::io::savePCDFileBinary(temporary, cloud) == 0 &&
         std::rename(temporary.c_str(), path.c_str()) == 0;
}

Cloud::Ptr voxelized(const Cloud::ConstPtr& input, double leaf) {
  Cloud::Ptr output(new Cloud);
  pcl::VoxelGrid<Point> voxel;
  voxel.setLeafSize(leaf, leaf, leaf);
  voxel.setInputCloud(input);
  voxel.filter(*output);
  return output;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_pcd_processor");
  ros::NodeHandle pnh("~");

  std::string input_pcd, output_directory, map_name, map_frame;
  pnh.param<std::string>("input_pcd", input_pcd, "");
  pnh.param<std::string>("output_directory", output_directory, "/tmp/jiangyin_map");
  pnh.param<std::string>("map_name", map_name, "competition_map");
  pnh.param<std::string>("map_frame", map_frame, "world");
  double clean_leaf = 0.05, relocal_leaf = 0.15, planner_leaf = 0.08;
  double radius = 0.15, stddev = 1.5;
  int radius_min_neighbors = 2, sor_mean_k = 30;
  pnh.param("clean_leaf_size", clean_leaf, clean_leaf);
  pnh.param("relocal_leaf_size", relocal_leaf, relocal_leaf);
  pnh.param("planner_leaf_size", planner_leaf, planner_leaf);
  pnh.param("radius", radius, radius);
  pnh.param("radius_min_neighbors", radius_min_neighbors, radius_min_neighbors);
  pnh.param("sor_mean_k", sor_mean_k, sor_mean_k);
  pnh.param("sor_stddev", stddev, stddev);

  Eigen::Vector4f crop_min, crop_max;
  if (!loadVec3(pnh, "venue_bounds/min", {-100.0, -100.0, -10.0}, &crop_min) ||
      !loadVec3(pnh, "venue_bounds/max", {100.0, 100.0, 20.0}, &crop_max)) {
    return 2;
  }
  if (input_pcd.empty() || clean_leaf <= 0.0 || relocal_leaf <= 0.0 ||
      planner_leaf <= 0.0 || radius <= 0.0 || sor_mean_k < 2 ||
      radius_min_neighbors < 1 ||
      (crop_min.head<3>().array() >= crop_max.head<3>().array()).any()) {
    ROS_ERROR("Invalid map processing configuration");
    return 2;
  }

  Cloud::Ptr raw(new Cloud);
  if (pcl::io::loadPCDFile<Point>(input_pcd, *raw) != 0 || raw->empty()) {
    ROS_ERROR_STREAM("Cannot load input PCD: " << input_pcd);
    return 3;
  }
  Cloud::Ptr finite(new Cloud);
  finite->reserve(raw->size());
  for (const Point& point : raw->points) {
    if (pcl::isFinite(point)) finite->push_back(point);
  }

  Cloud::Ptr cropped(new Cloud);
  pcl::CropBox<Point> crop;
  crop.setMin(crop_min);
  crop.setMax(crop_max);
  crop.setInputCloud(finite);
  crop.filter(*cropped);

  Cloud::Ptr coarse = voxelized(cropped, clean_leaf);
  Cloud::Ptr statistical(new Cloud);
  pcl::StatisticalOutlierRemoval<Point> sor;
  sor.setInputCloud(coarse);
  sor.setMeanK(sor_mean_k);
  sor.setStddevMulThresh(stddev);
  sor.filter(*statistical);

  Cloud::Ptr clean(new Cloud);
  pcl::RadiusOutlierRemoval<Point> radius_filter;
  radius_filter.setInputCloud(statistical);
  radius_filter.setRadiusSearch(radius);
  radius_filter.setMinNeighborsInRadius(radius_min_neighbors);
  radius_filter.filter(*clean);
  if (clean->empty()) {
    ROS_ERROR("All points were removed; check bounds and filter parameters");
    return 4;
  }

  Cloud::Ptr relocal = voxelized(clean, relocal_leaf);
  Cloud::Ptr planner = voxelized(clean, planner_leaf);
  boost::system::error_code ec;
  boost::filesystem::create_directories(output_directory, ec);
  if (ec) {
    ROS_ERROR_STREAM("Cannot create output directory: " << ec.message());
    return 5;
  }
  const std::string base = output_directory + "/" + map_name;
  const std::string clean_path = base + "_clean.pcd";
  const std::string relocal_path = base + "_relocal.pcd";
  const std::string planner_path = base + "_planner.pcd";
  if (!saveAtomic(clean_path, *clean) || !saveAtomic(relocal_path, *relocal) ||
      !saveAtomic(planner_path, *planner)) {
    ROS_ERROR("Failed to write one or more derived PCD files");
    return 6;
  }

  std::ofstream metadata((base + ".yaml").c_str(), std::ios::trunc);
  metadata << std::setprecision(10)
           << "format_version: 2\n"
           << "map_id: " << map_name << "\n"
           << "map_frame: " << map_frame << "\n"
           << "source_pcd: " << input_pcd << "\n"
           << "clean_pcd: " << map_name << "_clean.pcd\n"
           << "relocal_pcd: " << map_name << "_relocal.pcd\n"
           << "planner_pcd: " << map_name << "_planner.pcd\n"
           << "source_point_count: " << raw->size() << "\n"
           << "clean_point_count: " << clean->size() << "\n"
           << "relocal_point_count: " << relocal->size() << "\n"
           << "planner_point_count: " << planner->size() << "\n"
           << "venue_bounds_min: [" << crop_min.x() << ", " << crop_min.y() << ", "
           << crop_min.z() << "]\n"
           << "venue_bounds_max: [" << crop_max.x() << ", " << crop_max.y() << ", "
           << crop_max.z() << "]\n"
           << "clean_leaf_size: " << clean_leaf << "\n"
           << "relocal_leaf_size: " << relocal_leaf << "\n"
           << "planner_leaf_size: " << planner_leaf << "\n"
           << "sor_mean_k: " << sor_mean_k << "\n"
           << "sor_stddev: " << stddev << "\n"
           << "radius: " << radius << "\n"
           << "radius_min_neighbors: " << radius_min_neighbors << "\n";
  if (!metadata) {
    ROS_ERROR("Failed to write map bundle metadata");
    return 7;
  }
  ROS_INFO_STREAM("Map bundle generated: clean=" << clean->size()
                  << ", relocal=" << relocal->size()
                  << ", planner=" << planner->size());
  return 0;
}
