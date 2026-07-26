//
// Created by yunfan on 2022/3/26.
//

#ifndef PERFECT_DRONE_CONFIG_HPP
#define PERFECT_DRONE_CONFIG_HPP


#include "cstring"
#include "cmath"
#include "stdexcept"
#include "vector"
#include "iostream"
#include <Eigen/Core>
#include "utils/yaml_loader.hpp"

namespace perfect_drone {
    using std::cout;
    using std::endl;
    using std::string;
    using std::vector;

    class Config {
    public:
        std::string mesh_resource;
        Eigen::Vector3d init_pos;
        double init_yaw;
        double sensing_rate;
        bool conical_fov_en;
        double lidar_axis_tilt_deg;
        double lidar_fov_min_angle_deg;
        double lidar_fov_max_angle_deg;
        Eigen::Vector3d lidar_axis_body{Eigen::Vector3d::UnitZ()};
        double lidar_fov_cos_min_angle{1.0};
        double lidar_fov_cos_max_angle{-1.0};

        Config() = default;

        Config(const std::string &cfg_path) {
            yaml_loader::YamlLoader loader(cfg_path);
            loader.LoadParam("mesh_resource", mesh_resource, std::string("package://perfect_drone_sim/meshes/f250.dae"),
                             false);
            loader.LoadParam("init_position/x", init_pos.x(), 0.0);
            loader.LoadParam("init_position/y", init_pos.y(), 0.0);
            loader.LoadParam("init_position/z", init_pos.z(), 1.5);
            loader.LoadParam("init_yaw", init_yaw, 0.0);
            loader.LoadParam("sensing_rate", sensing_rate, 10.0);//loader.LoadParam("参数名", 目标变量,yaml里没有时的默认值);
            loader.LoadParam("conical_fov_en", conical_fov_en, false);
            loader.LoadParam("lidar_axis_tilt_deg", lidar_axis_tilt_deg, 60.0);
            loader.LoadParam("lidar_fov_min_angle_deg", lidar_fov_min_angle_deg, 38.0);
            loader.LoadParam("lidar_fov_max_angle_deg", lidar_fov_max_angle_deg, 97.0);

            if (!std::isfinite(lidar_axis_tilt_deg) ||
                !std::isfinite(lidar_fov_min_angle_deg) ||
                !std::isfinite(lidar_fov_max_angle_deg) ||
                lidar_axis_tilt_deg < 0.0 || lidar_axis_tilt_deg > 180.0 ||
                lidar_fov_min_angle_deg < 0.0 ||
                lidar_fov_max_angle_deg > 180.0 ||
                lidar_fov_min_angle_deg > lidar_fov_max_angle_deg) {
                throw std::runtime_error("Invalid conical LiDAR FOV parameters");
            }

            constexpr double DEG_TO_RAD = M_PI / 180.0;
            const double tilt_rad = lidar_axis_tilt_deg * DEG_TO_RAD;
            // Body frame convention: +X forward, +Z upward. The LiDAR
            // symmetry axis tilts from +Z toward +X.
            lidar_axis_body =
                    Eigen::Vector3d(std::sin(tilt_rad), 0.0, std::cos(tilt_rad)).normalized();
            lidar_fov_cos_min_angle = std::cos(lidar_fov_min_angle_deg * DEG_TO_RAD);
            lidar_fov_cos_max_angle = std::cos(lidar_fov_max_angle_deg * DEG_TO_RAD);
        }

        bool pointInLidarFov(const Eigen::Vector3d &ray_body) const {
            if (!conical_fov_en) {
                return true;
            }
            const double ray_norm = ray_body.norm();
            if (ray_norm <= 1e-9) {
                return false;
            }
            const double cos_angle = lidar_axis_body.dot(ray_body) / ray_norm;
            return cos_angle <= lidar_fov_cos_min_angle &&
                   cos_angle >= lidar_fov_cos_max_angle;
        }
    };
}

#endif
