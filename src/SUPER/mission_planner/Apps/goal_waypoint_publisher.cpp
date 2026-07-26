#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <super_msgs/Flag.h>
#include <yaml-cpp/yaml.h>

namespace {

struct Waypoint {
    int id;
    int mode;
    int is_map;
    double x;
    double y;
    double z;
    double yaw;
    double desired_speed;
};

template <typename T>
T required(const YAML::Node &node, const std::string &key, std::size_t index) {
    if (!node[key]) {
        throw std::runtime_error("waypoints[" + std::to_string(index) +
                                 "] missing required field '" + key + "'");
    }
    return node[key].as<T>();
}

std::vector<Waypoint> loadWaypoints(const std::string &config_path, double &wait_time) {
    const YAML::Node config = YAML::LoadFile(config_path);
    if (!config["wait_time"]) {
        throw std::runtime_error("missing required top-level field 'wait_time'");
    }
    wait_time = config["wait_time"].as<double>();
    if (!std::isfinite(wait_time) || wait_time < 0.0) {
        throw std::runtime_error("'wait_time' must be a finite non-negative number");
    }

    const YAML::Node entries = config["waypoints"];
    if (!entries || !entries.IsSequence() || entries.size() == 0) {
        throw std::runtime_error("'waypoints' must be a non-empty YAML sequence");
    }

    std::vector<Waypoint> waypoints;
    waypoints.reserve(entries.size());
    for (std::size_t i = 0; i < entries.size(); ++i) {
        const YAML::Node &entry = entries[i];
        if (!entry.IsMap()) {
            throw std::runtime_error("waypoints[" + std::to_string(i) + "] must be a map");
        }

        Waypoint waypoint{
            required<int>(entry, "id", i),
            required<int>(entry, "mode", i),
            required<int>(entry, "is_map", i),
            required<double>(entry, "x", i),
            required<double>(entry, "y", i),
            required<double>(entry, "z", i),
            required<double>(entry, "yaw", i),
            required<double>(entry, "desired_speed", i),
        };

        if (waypoint.id < 0) {
            throw std::runtime_error("waypoints[" + std::to_string(i) + "].id must be non-negative");
        }
        if (waypoint.id != static_cast<int>(i)) {
            throw std::runtime_error("waypoints[" + std::to_string(i) +
                                     "].id must equal its zero-based sequence index");
        }
        if (waypoint.is_map != 0 && waypoint.is_map != 1) {
            throw std::runtime_error("waypoints[" + std::to_string(i) + "].is_map must be 0 or 1");
        }
        if (!std::isfinite(waypoint.x) || !std::isfinite(waypoint.y) ||
            !std::isfinite(waypoint.z)) {
            throw std::runtime_error("waypoints[" + std::to_string(i) +
                                     "] position must be finite");
        }
        waypoints.push_back(waypoint);
    }
    return waypoints;
}

}  // namespace

int main(int argc, char **argv) {
    ros::init(argc, argv, "goal_waypoint_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string config_path = std::string(ROOT_DIR) + "config/goal.yaml";
    std::string topic = "/flag/waypoint";
    double publish_interval = 0.2;
    double start_delay = 1.0;
    double subscriber_wait_timeout = 5.0;
    private_nh.param("config_path", config_path, config_path);
    private_nh.param("topic", topic, topic);
    private_nh.param("publish_interval", publish_interval, publish_interval);
    private_nh.param("start_delay", start_delay, start_delay);
    private_nh.param("subscriber_wait_timeout", subscriber_wait_timeout, subscriber_wait_timeout);

    double wait_time = 0.0;
    std::vector<Waypoint> waypoints;
    try {
        waypoints = loadWaypoints(config_path, wait_time);
    } catch (const std::exception &error) {
        ROS_FATAL_STREAM("Failed to load waypoint config '" << config_path << "': " << error.what());
        return 1;
    }

    ros::Publisher publisher = nh.advertise<super_msgs::Flag>(topic, 10, true);
    const ros::WallTime wait_start = ros::WallTime::now();
    while (ros::ok() && publisher.getNumSubscribers() == 0 &&
           (ros::WallTime::now() - wait_start).toSec() < subscriber_wait_timeout) {
        ros::WallDuration(0.05).sleep();
    }
    if (publisher.getNumSubscribers() == 0) {
        ROS_WARN_STREAM("No subscriber connected to " << topic
                        << " after " << subscriber_wait_timeout
                        << " s; publishing anyway.");
    }
    ros::WallDuration(std::max(0.0, start_delay)).sleep();

    ROS_INFO_STREAM("Publishing " << waypoints.size() << " waypoints from " << config_path
                                  << " to " << topic);
    for (std::size_t i = 0; i < waypoints.size(); ++i) {
        if (!ros::ok()) {
            break;
        }
        const Waypoint &waypoint = waypoints[i];

        super_msgs::Flag message;
        message.header.stamp = ros::Time::now();
        message.header.frame_id = "world";
        message.id = waypoint.id;
        message.mode = waypoint.mode;
        message.is_map = waypoint.is_map;
        message.position.x = waypoint.x;
        message.position.y = waypoint.y;
        message.position.z = waypoint.z;
        message.yaw = static_cast<float>(waypoint.yaw);
        message.desired_speed =
            static_cast<float>(std::max(0.0, waypoint.desired_speed));
        publisher.publish(message);

        ROS_INFO_STREAM("Waypoint " << message.id << ": p=("
                                    << message.position.x << ", "
                                    << message.position.y << ", "
                                    << message.position.z << "), mode=" << message.mode
                                    << ", is_map=" << message.is_map
                                    << ", yaw=" << message.yaw
                                    << ", desired_speed=" << message.desired_speed);
        if (i == 2 && waypoints.size() > 3) {
            ROS_INFO_STREAM("First three waypoints published; waiting "
                            << wait_time << " s before publishing the remaining "
                            << waypoints.size() - 3 << " waypoints.");
            ros::WallDuration(wait_time).sleep();
        } else if (i + 1 < waypoints.size()) {
            ros::WallDuration(std::max(0.0, publish_interval)).sleep();
        }
    }

    ROS_INFO("All waypoints published.");
    ros::spin();
    return 0;
}
