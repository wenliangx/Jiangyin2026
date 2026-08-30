#include <livox_ros_driver2/CustomMsg.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <string>

namespace {

constexpr double kPi = 3.14159265358979323846;

class PointCloudToLivox {
public:
  PointCloudToLivox() : private_node_("~") {
    private_node_.param("input_topic", input_topic_,
                        std::string("/mid360/points"));
    private_node_.param("output_topic", output_topic_,
                        std::string("/livox/lidar"));
    private_node_.param("scan_rate", scan_rate_, 10.0);
    private_node_.param("lidar_id", lidar_id_, 0);
    scan_period_ns_ = static_cast<std::uint32_t>(1.0e9 / scan_rate_);

    publisher_ =
        node_.advertise<livox_ros_driver2::CustomMsg>(output_topic_, 2);
    subscriber_ = node_.subscribe(input_topic_, 2,
                                  &PointCloudToLivox::cloudCallback, this);
    ROS_INFO("MID360 simulation bridge: %s -> %s at %.1f Hz",
             input_topic_.c_str(), output_topic_.c_str(), scan_rate_);
  }

private:
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud) {
    livox_ros_driver2::CustomMsg output;
    output.header = cloud->header;
    output.timebase = cloud->header.stamp.toNSec();
    output.lidar_id =
        static_cast<std::uint8_t>(std::max(0, std::min(255, lidar_id_)));
    output.rsvd = {{0, 0, 0}};
    output.points.reserve(static_cast<std::size_t>(cloud->width) *
                          cloud->height);

    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*cloud, "z");
      sensor_msgs::PointCloud2ConstIterator<float> intensity(*cloud,
                                                             "intensity");
      sensor_msgs::PointCloud2ConstIterator<std::uint16_t> ring(*cloud, "ring");

      for (; x != x.end(); ++x, ++y, ++z, ++intensity, ++ring) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
          continue;
        }
        if (*x == 0.0F && *y == 0.0F && *z == 0.0F) {
          continue;
        }

        const double normalized_angle =
            (std::atan2(*y, *x) + kPi) / (2.0 * kPi);
        livox_ros_driver2::CustomPoint point;
        point.offset_time =
            static_cast<std::uint32_t>(normalized_angle * scan_period_ns_);
        point.x = *x;
        point.y = *y;
        point.z = *z;
        point.reflectivity = static_cast<std::uint8_t>(
            std::max(0.0F, std::min(255.0F, *intensity)));
        point.tag = 0;
        point.line =
            static_cast<std::uint8_t>(std::min<std::uint16_t>(63, *ring));
        output.points.push_back(point);
      }
    } catch (const std::exception &error) {
      ROS_ERROR_THROTTLE(2.0, "Invalid MID360 PointCloud2 layout: %s",
                         error.what());
      return;
    }

    output.point_num = static_cast<std::uint32_t>(output.points.size());
    publisher_.publish(output);
    ROS_INFO_THROTTLE(5.0, "MID360 bridge published %u points",
                      output.point_num);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  std::string input_topic_;
  std::string output_topic_;
  double scan_rate_{10.0};
  int lidar_id_{0};
  std::uint32_t scan_period_ns_{100000000};
};

} // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "mid360_bridge");
  PointCloudToLivox bridge;
  ros::spin();
  return 0;
}
