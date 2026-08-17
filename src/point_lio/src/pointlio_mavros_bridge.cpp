#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>

namespace
{
bool finite(double value)
{
  return std::isfinite(value);
}

bool validOdometry(const nav_msgs::Odometry &odom)
{
  const auto &p = odom.pose.pose.position;
  const auto &q = odom.pose.pose.orientation;
  const auto &linear = odom.twist.twist.linear;
  const auto &angular = odom.twist.twist.angular;
  const double quaternion_norm_squared =
      q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return !odom.header.stamp.isZero() && finite(p.x) && finite(p.y) && finite(p.z) &&
         finite(q.x) && finite(q.y) && finite(q.z) && finite(q.w) &&
         quaternion_norm_squared > 1e-12 && finite(linear.x) && finite(linear.y) &&
         finite(linear.z) && finite(angular.x) && finite(angular.y) && finite(angular.z);
}
}  // namespace

class PointlioMavrosBridge
{
public:
  PointlioMavrosBridge() : private_nh_("~")
  {
    private_nh_.param<std::string>("input_topic", input_topic_, "/point_lio/odometry");
    private_nh_.param<std::string>("planner_topic", planner_topic_, "/Odometry");
    private_nh_.param<std::string>("mavros_topic", mavros_topic_, "/mavros/odometry/out");
    private_nh_.param<std::string>("mavros_frame_id", mavros_frame_id_, "odom");
    private_nh_.param<std::string>("mavros_child_frame_id", mavros_child_frame_id_,
                                   "base_link");
    private_nh_.param<double>("mavros_rate_hz", mavros_rate_hz_, 20.0);
    private_nh_.param<double>("stale_timeout_sec", stale_timeout_sec_, 0.25);
    private_nh_.param<bool>("publish_to_px4", publish_to_px4_, true);

    if (mavros_rate_hz_ <= 0.0 || stale_timeout_sec_ <= 0.0)
    {
      ROS_FATAL("mavros_rate_hz and stale_timeout_sec must be positive");
      ros::shutdown();
      return;
    }

    planner_pub_ = nh_.advertise<nav_msgs::Odometry>(planner_topic_, 20);
    if (publish_to_px4_)
      mavros_pub_ = nh_.advertise<nav_msgs::Odometry>(mavros_topic_, 10);
    health_pub_ = private_nh_.advertise<std_msgs::Bool>("healthy", 1, true);
    odometry_sub_ = nh_.subscribe(input_topic_, 50,
                                  &PointlioMavrosBridge::odometryCallback, this,
                                  ros::TransportHints().tcpNoDelay());
    timer_ = nh_.createTimer(ros::Duration(1.0 / mavros_rate_hz_),
                             &PointlioMavrosBridge::timerCallback, this);

    ROS_INFO_STREAM("Point-LIO bridge: " << input_topic_ << " -> " << planner_topic_
                    << " (source rate), " << mavros_topic_ << " (latest corrected state, max "
                    << mavros_rate_hz_ << " Hz)");
  }

private:
  void odometryCallback(const nav_msgs::Odometry::ConstPtr &message)
  {
    if (!validOdometry(*message))
    {
      ROS_WARN_THROTTLE(1.0, "Dropping invalid Point-LIO odometry");
      return;
    }

    nav_msgs::Odometry normalized = *message;
    auto &q = normalized.pose.pose.orientation;
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;
    q.w /= norm;

    // SUPER consumes this topic directly and benefits from every new Point-LIO state.
    planner_pub_.publish(normalized);

    std::lock_guard<std::mutex> lock(mutex_);
    if (have_odometry_ && normalized.header.stamp < latest_odometry_.header.stamp)
    {
      // Accept rosbag loops or a sensor clock reset, but allow the new epoch to be sent.
      last_sent_sequence_ = sequence_;
    }
    latest_odometry_ = normalized;
    have_odometry_ = true;
    ++sequence_;
  }

  void timerCallback(const ros::TimerEvent &)
  {
    nav_msgs::Odometry output;
    bool publish = false;
    bool healthy = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (have_odometry_)
      {
        const double age = (ros::Time::now() - latest_odometry_.header.stamp).toSec();
        healthy = age >= -0.1 && age <= stale_timeout_sec_;
        if (publish_to_px4_ && healthy && sequence_ != last_sent_sequence_)
        {
          output = latest_odometry_;
          last_sent_sequence_ = sequence_;
          publish = true;
        }
      }
    }

    std_msgs::Bool health_message;
    health_message.data = healthy;
    health_pub_.publish(health_message);

    if (!healthy)
    {
      if (publish_to_px4_)
        ROS_WARN_THROTTLE(2.0, "Point-LIO odometry is missing or stale; PX4 output is paused");
      return;
    }
    if (!publish) return;

    // MAVROS odometry plugin requires these frame names for its ENU/FLU conversion.
    output.header.frame_id = mavros_frame_id_;
    output.child_frame_id = mavros_child_frame_id_;
    mavros_pub_.publish(output);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber odometry_sub_;
  ros::Publisher planner_pub_;
  ros::Publisher mavros_pub_;
  ros::Publisher health_pub_;
  ros::Timer timer_;

  std::string input_topic_;
  std::string planner_topic_;
  std::string mavros_topic_;
  std::string mavros_frame_id_;
  std::string mavros_child_frame_id_;
  double mavros_rate_hz_ = 20.0;
  double stale_timeout_sec_ = 0.25;
  bool publish_to_px4_ = true;

  std::mutex mutex_;
  nav_msgs::Odometry latest_odometry_;
  bool have_odometry_ = false;
  std::uint64_t sequence_ = 0;
  std::uint64_t last_sent_sequence_ = 0;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointlio_mavros_bridge");
  PointlioMavrosBridge bridge;
  ros::spin();
  return 0;
}
