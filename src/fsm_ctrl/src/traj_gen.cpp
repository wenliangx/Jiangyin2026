/**
 * @file    traj_gen.cpp
 * @brief   Trajectory generator implementation
 * @author  FLAG Lab, BIT
 * @date    2026-06-28
 */

#include "fsm_ctrl/traj_gen.hpp"
#include <cmath>
#include <stdexcept>
#include <algorithm>

namespace traj_gen
{

/* =================================================================
 *  CircleTrajectory
 * ================================================================= */

CircleTrajectory::CircleTrajectory(double cx, double cy, double altitude,
                                   double radius, double speed, bool clockwise)
    : cx_(cx), cy_(cy), alt_(altitude), radius_(radius), speed_(speed)
{
    dir_ = clockwise ? -1 : 1;
    omega_ = speed / radius;           // angular frequency [rad/s]
}

TrajPoint CircleTrajectory::sample(double t) const
{
    double theta = dir_ * omega_ * t;  // current angle on circle

    double px = cx_ + radius_ * std::cos(theta);
    double py = cy_ + radius_ * std::sin(theta);
    double vx = -dir_ * radius_ * omega_ * std::sin(theta);  // tangent velocity
    double vy =  dir_ * radius_ * omega_ * std::cos(theta);

    // yaw points along the tangent direction
    double yaw = std::atan2(vy, vx);

    return TrajPoint(Eigen::Vector3d(px, py, alt_),
                     Eigen::Vector3d(vx, vy, 0.0),
                     yaw);
}


/* =================================================================
 *  SquareTrajectory
 *
 *  Path:  corner1 → corner2 → corner3 → corner4 → corner1 (repeat)
 *  Corners are at (±half_side, ±half_side) around center.
 *  Direction: +X → +Y → -X → -Y (counter-clockwise from bottom-left)
 * ================================================================= */

SquareTrajectory::SquareTrajectory(double cx, double cy, double altitude,
                                   double side_length, double speed)
    : cx_(cx), cy_(cy), alt_(altitude), side_(side_length), speed_(speed)
{
    half_side_ = side_length / 2.0;
    seg_time_  = side_length / speed;       // time for one edge [s]
    total_time_ = 4.0 * seg_time_;           // time for full square [s]
}

TrajPoint SquareTrajectory::sample(double t) const
{
    // wrap into one cycle
    double phase = std::fmod(t, total_time_);
    if (phase < 0) phase += total_time_;

    int seg = static_cast<int>(phase / seg_time_);           // 0..3
    double s = (phase - seg * seg_time_) * speed_;           // distance along current edge [0, side_)

    double px, py, vx, vy;

    // Corner positions: bottom-left, bottom-right, top-right, top-left
    const double x0 = cx_ - half_side_, y0 = cy_ - half_side_;  // BL
    const double x1 = cx_ + half_side_, y1 = cy_ - half_side_;  // BR
    const double x2 = cx_ + half_side_, y2 = cy_ + half_side_;  // TR
    const double x3 = cx_ - half_side_, y3 = cy_ + half_side_;  // TL

    switch (seg)
    {
    case 0:  // BL → BR (+X)
        px = x0 + s;  py = y0;
        vx = speed_;  vy = 0.0;
        break;
    case 1:  // BR → TR (+Y)
        px = x1;  py = y0 + s;
        vx = 0.0;  vy = speed_;
        break;
    case 2:  // TR → TL (-X)
        px = x2 - s;  py = y2;
        vx = -speed_; vy = 0.0;
        break;
    case 3:  // TL → BL (-Y)
    default:
        px = x3;  py = y2 - s;
        vx = 0.0;  vy = -speed_;
        break;
    }

    double yaw = std::atan2(vy, vx);

    return TrajPoint(Eigen::Vector3d(px, py, alt_),
                     Eigen::Vector3d(vx, vy, 0.0),
                     yaw);
}


/* =================================================================
 *  WaypointTrajectory
 * ================================================================= */

WaypointTrajectory::WaypointTrajectory(double altitude, double speed, bool loop)
    : alt_(altitude), speed_(speed), loop_(loop), total_len_(0.0)
{}

void WaypointTrajectory::addWaypoint(double x, double y, double yaw)
{
    wpts_.push_back({Eigen::Vector3d(x, y, alt_), yaw});
    rebuild();
}

void WaypointTrajectory::addWaypoint(const Eigen::Vector3d& pos, double yaw)
{
    wpts_.push_back({Eigen::Vector3d(pos.x(), pos.y(), alt_), yaw});
    rebuild();
}

void WaypointTrajectory::clearWaypoints()
{
    wpts_.clear();
    seg_lens_.clear();
    cum_lens_.clear();
    total_len_ = 0.0;
}

void WaypointTrajectory::setWaypoints(const std::vector<Eigen::Vector3d>& pts)
{
    wpts_.clear();
    for (const auto& p : pts)
        wpts_.push_back({Eigen::Vector3d(p.x(), p.y(), alt_), 0.0});
    rebuild();
}

void WaypointTrajectory::rebuild()
{
    const size_t n = wpts_.size();
    seg_lens_.clear();
    cum_lens_.clear();
    total_len_ = 0.0;

    if (n < 2) return;

    size_t n_seg = loop_ ? n : n - 1;
    seg_lens_.resize(n_seg);
    cum_lens_.resize(n_seg + 1);
    cum_lens_[0] = 0.0;

    for (size_t i = 0; i < n_seg; i++)
    {
        size_t j = (i + 1) % n;
        seg_lens_[i] = (wpts_[j].pos - wpts_[i].pos).norm();
        total_len_ += seg_lens_[i];
        cum_lens_[i + 1] = total_len_;
    }
}

TrajPoint WaypointTrajectory::sample(double t) const
{
    if (wpts_.size() < 2)
    {
        // not enough points — just hover at first point or origin
        if (!wpts_.empty())
            return TrajPoint(wpts_[0].pos, Eigen::Vector3d::Zero(), wpts_[0].yaw);
        return TrajPoint(Eigen::Vector3d(0, 0, alt_));
    }

    double dist = t * speed_;

    if (!loop_ && dist >= total_len_)
    {
        // past end — hold at last waypoint
        const auto& last = wpts_.back();
        return TrajPoint(last.pos, Eigen::Vector3d::Zero(), last.yaw);
    }

    // wrap for looping
    if (loop_ && total_len_ > 0.0)
        dist = std::fmod(dist, total_len_);
    if (dist < 0) dist += total_len_;

    // find which segment we're on
    size_t n_seg = loop_ ? wpts_.size() : wpts_.size() - 1;
    size_t seg = 0;
    for (size_t i = 1; i <= n_seg; i++)
    {
        if (dist <= cum_lens_[i]) { seg = i - 1; break; }
        seg = i - 1;  // fallback: last segment
    }

    double seg_dist = dist - cum_lens_[seg];
    double alpha = (seg_lens_[seg] > 0.0) ? seg_dist / seg_lens_[seg] : 0.0;
    alpha = std::max(0.0, std::min(1.0, alpha));

    size_t i0 = seg;
    size_t i1 = (seg + 1) % wpts_.size();

    const auto& p0 = wpts_[i0];
    const auto& p1 = wpts_[i1];

    Eigen::Vector3d pos = p0.pos + alpha * (p1.pos - p0.pos);
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    if (seg_lens_[seg] > 0.0)
        vel = speed_ * (p1.pos - p0.pos).normalized();
    double yaw = p0.yaw + alpha * (p1.yaw - p0.yaw);

    return TrajPoint(pos, vel, yaw);
}


/* =================================================================
 *  Factory
 * ================================================================= */

std::shared_ptr<Trajectory> createTrajectory(
    const std::string& type,
    double cx, double cy, double alt,
    double param1, double param2)
{
    if (type == "circle")
    {
        return std::make_shared<CircleTrajectory>(cx, cy, alt, param1, param2);
    }
    else if (type == "square")
    {
        return std::make_shared<SquareTrajectory>(cx, cy, alt, param1, param2);
    }
    else if (type == "waypoint")
    {
        auto traj = std::make_shared<WaypointTrajectory>(alt, param2, true);
        // For waypoint type, the caller should add waypoints after creation
        return traj;
    }
    else
    {
        throw std::runtime_error("Unknown trajectory type: " + type);
    }
}

}  // namespace traj_gen
