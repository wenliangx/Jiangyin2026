#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <super_msgs/Flag.h>
#include <yaml-cpp/yaml.h>

namespace {

constexpr std::size_t kFirstBatchSize = 3;

struct Waypoint {
    int id{0};
    int mode{0};
    int is_map{1};
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double yaw{std::numeric_limits<double>::quiet_NaN()};
    double desired_speed{0.0};
};

double optionalYaw(const YAML::Node &node) {
    if (!node || node.IsNull()) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    try {
        return node.as<double>();
    } catch (const YAML::Exception &) {
        std::string value = node.as<std::string>();
        std::transform(value.begin(), value.end(), value.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (value == "nan" || value == ".nan" || value == "free" || value == "auto") {
            return std::numeric_limits<double>::quiet_NaN();
        }
        throw;
    }
}

class GoalWaypointPublisher {
public:
    explicit GoalWaypointPublisher(const ros::NodeHandle &private_nh)
        : private_nh_(private_nh), start_time_(ros::Time::now()) {
#define CONFIG_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "config/" + name))
        std::string config_path;
        std::string config_name;
        if (private_nh_.getParam("config_path", config_path)) {
            ROS_INFO_STREAM("[goal_waypoint_publisher] Load config from " << config_path);
        } else {
            private_nh_.param<std::string>("config_name", config_name, "goal.yaml");
            config_path = CONFIG_FILE_DIR(config_name);
            ROS_INFO_STREAM("[goal_waypoint_publisher] Load config from " << config_path);
        }

        loadConfig(config_path);
        waypoint_pub_ = nh_.advertise<super_msgs::Flag>(waypoint_topic_, waypoints_.size(), false);
        timer_ = nh_.createTimer(ros::Duration(0.01), &GoalWaypointPublisher::timerCallback, this);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher waypoint_pub_;
    ros::Timer timer_;

    std::vector<Waypoint> waypoints_;
    std::string waypoint_topic_{"/super/flag_waypoint"};
    std::string frame_id_{"world"};
    double start_delay_{1.0};
    double publish_interval_{0.05};
    double wait_time_{0.0};
    ros::Time start_time_;
    ros::Time last_publish_time_;
    std::size_t next_waypoint_{0};
    bool wait_logged_{false};

    void loadConfig(const std::string &config_path) {
        const YAML::Node root = YAML::LoadFile(config_path);

        if (root["waypoint_topic"]) {
            waypoint_topic_ = root["waypoint_topic"].as<std::string>();
        }
        if (root["frame_id"]) {
            frame_id_ = root["frame_id"].as<std::string>();
        }
        if (root["start_delay"]) {
            start_delay_ = std::max(0.0, root["start_delay"].as<double>());
        }
        if (root["publish_interval"]) {
            publish_interval_ = root["publish_interval"].as<double>();
        }
        if (root["wait_time"]) {
            wait_time_ = root["wait_time"].as<double>();
        }
        if (!std::isfinite(publish_interval_) || publish_interval_ <= 0.0) {
            throw std::runtime_error("publish_interval must be a finite positive number");
        }
        if (!std::isfinite(wait_time_) || wait_time_ < 0.0) {
            throw std::runtime_error("wait_time must be a finite non-negative number");
        }

        const YAML::Node waypoint_nodes = root["waypoints"];
        if (!waypoint_nodes || !waypoint_nodes.IsSequence() || waypoint_nodes.size() == 0) {
            throw std::runtime_error("goal.yaml must contain a non-empty 'waypoints' sequence");
        }

        waypoints_.reserve(waypoint_nodes.size());
        for (std::size_t i = 0; i < waypoint_nodes.size(); ++i) {
            const YAML::Node node = waypoint_nodes[i];
            Waypoint waypoint;
            waypoint.id = node["id"].as<int>();
            waypoint.mode = node["mode"].as<int>();
            waypoint.is_map = node["is_map"].as<int>();
            waypoint.x = node["x"].as<double>();
            waypoint.y = node["y"].as<double>();
            waypoint.z = node["z"].as<double>();
            waypoint.yaw = optionalYaw(node["yaw"]);
            waypoint.desired_speed = node["desired_speed"]
                                         ? node["desired_speed"].as<double>()
                                         : 0.0;

            if (waypoint.id != static_cast<int>(i)) {
                throw std::runtime_error("waypoint ids must be contiguous and start at 0");
            }
            if (waypoint.mode < 0 || waypoint.mode > 2) {
                throw std::runtime_error("waypoint mode must be 0, 1, or 2");
            }
            if (waypoint.is_map != 0 && waypoint.is_map != 1) {
                throw std::runtime_error("waypoint is_map must be 0 or 1");
            }
            if (!std::isfinite(waypoint.x) || !std::isfinite(waypoint.y) ||
                !std::isfinite(waypoint.z)) {
                throw std::runtime_error("waypoint x/y/z must be finite");
            }
            if (!std::isfinite(waypoint.desired_speed) || waypoint.desired_speed <= 0.0) {
                waypoint.desired_speed = 0.0;
            }
            waypoints_.push_back(waypoint);
        }

        if (waypoints_.back().desired_speed > 0.0) {
            ROS_WARN("[goal_waypoint_publisher] Last waypoint has positive desired_speed; "
                     "set it to 0 if the vehicle must stop at the end of the mission.");
        }
        ROS_INFO_STREAM("[goal_waypoint_publisher] Loaded " << waypoints_.size()
                        << " waypoints for " << waypoint_topic_
                        << "; wait " << wait_time_
                        << " s after the first " << kFirstBatchSize << " waypoints");
    }

    void publishWaypoint(const Waypoint &waypoint) {
        super_msgs::Flag msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = frame_id_;
        msg.id = waypoint.id;
        msg.total_waypoint = static_cast<int16_t>(waypoints_.size());
        msg.mode = waypoint.mode;
        msg.is_map = waypoint.is_map;
        msg.position.x = waypoint.x;
        msg.position.y = waypoint.y;
        msg.position.z = waypoint.z;
        msg.yaw = static_cast<float>(waypoint.yaw);
        msg.desired_speed = static_cast<float>(waypoint.desired_speed);
        waypoint_pub_.publish(msg);

        if (std::isfinite(waypoint.yaw)) {
            ROS_INFO("[goal_waypoint_publisher] Send id=%d p=(%.3f, %.3f, %.3f), "
                     "mode=%d, is_map=%d, yaw=%.3f rad, desired_speed=%.3f m/s",
                     waypoint.id, waypoint.x, waypoint.y, waypoint.z, waypoint.mode,
                     waypoint.is_map, waypoint.yaw, waypoint.desired_speed);
        } else {
            ROS_INFO("[goal_waypoint_publisher] Send id=%d p=(%.3f, %.3f, %.3f), "
                     "mode=%d, is_map=%d, yaw=free, desired_speed=%.3f m/s",
                     waypoint.id, waypoint.x, waypoint.y, waypoint.z, waypoint.mode,
                     waypoint.is_map, waypoint.desired_speed);
        }
    }

    void timerCallback(const ros::TimerEvent &) {
        if (next_waypoint_ >= waypoints_.size()) {
            timer_.stop();
            return;
        }

        const ros::Time now = ros::Time::now();
        if ((now - start_time_).toSec() < start_delay_) {
            return;
        }
        if (waypoint_pub_.getNumSubscribers() == 0) {
            return;
        }
        if (next_waypoint_ == kFirstBatchSize && next_waypoint_ < waypoints_.size() &&
            !last_publish_time_.isZero()) {
            const double waited_time = (now - last_publish_time_).toSec();
            if (waited_time < wait_time_) {
                if (!wait_logged_) {
                    ROS_INFO("[goal_waypoint_publisher] First %zu waypoints sent; "
                             "waiting %.3f s before sending the remaining waypoints.",
                             kFirstBatchSize, wait_time_);
                    wait_logged_ = true;
                }
                return;
            }
            if (wait_logged_) {
                ROS_INFO("[goal_waypoint_publisher] Wait finished; sending remaining waypoints.");
            }
        }
        if (!last_publish_time_.isZero() &&
            (now - last_publish_time_).toSec() < publish_interval_) {
            return;
        }

        publishWaypoint(waypoints_[next_waypoint_]);
        ++next_waypoint_;
        last_publish_time_ = now;
        if (next_waypoint_ == waypoints_.size()) {
            ROS_INFO("[goal_waypoint_publisher] All waypoints have been sent to the mission state machine.");
        }
    }
};

}  // namespace

int main(int argc, char **argv) {
    ros::init(argc, argv, "goal_waypoint_publisher");
    ros::NodeHandle private_nh("~");

    try {
        GoalWaypointPublisher publisher(private_nh);
        ros::spin();
    } catch (const std::exception &e) {
        ROS_FATAL_STREAM("[goal_waypoint_publisher] " << e.what());
        return 1;
    }
    return 0;
}
