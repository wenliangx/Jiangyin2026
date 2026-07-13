/**
 * @file    traj_gen
 * @brief   Trajectory generator for NMPC test — circle, square, waypoint
 * @author  FLAG Lab, BIT
 * @date    2026-06-28
 */

#ifndef __TRAJ_GEN_H__
#define __TRAJ_GEN_H__

#include <eigen3/Eigen/Dense>
#include <vector>
#include <string>
#include <memory>

namespace traj_gen
{

struct TrajPoint
{
    Eigen::Vector3d pos;   // position [m]
    Eigen::Vector3d vel;   // velocity [m/s]
    double yaw;            // yaw angle [rad]

    TrajPoint() : pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), yaw(0.0) {}
    TrajPoint(const Eigen::Vector3d& p, const Eigen::Vector3d& v = Eigen::Vector3d::Zero(), double y = 0.0)
        : pos(p), vel(v), yaw(y) {}
};


/* ======================== Abstract Base ======================== */

class Trajectory
{
public:
    virtual ~Trajectory() = default;

    /// Sample trajectory at time t [seconds from start]
    virtual TrajPoint sample(double t) const = 0;

    /// Total duration [s]. < 0 means infinite loop.
    virtual double duration() const = 0;

    /// Reset internal state
    virtual void reset() = 0;

    /// Human-readable name
    virtual std::string name() const = 0;
};


/* ======================== Circle ======================== */

class CircleTrajectory : public Trajectory
{
public:
    /**
     * @param cx, cy      Circle center [m]
     * @param altitude    Flight altitude [m]
     * @param radius      Circle radius [m]
     * @param speed       Tangential speed [m/s]
     * @param clockwise   true = clockwise, false = counter-clockwise
     */
    CircleTrajectory(double cx, double cy, double altitude,
                     double radius, double speed, bool clockwise = false);

    TrajPoint sample(double t) const override;
    double duration() const override { return -1.0; }
    void reset() override {}
    std::string name() const override { return "circle"; }

private:
    double cx_, cy_, alt_, radius_, speed_, omega_;
    int dir_;  // +1 CCW, -1 CW
};


/* ======================== Square ======================== */

class SquareTrajectory : public Trajectory
{
public:
    /**
     * @param cx, cy      Square center [m]
     * @param altitude    Flight altitude [m]
     * @param side_length Side length [m]
     * @param speed       Speed along edges [m/s]
     */
    SquareTrajectory(double cx, double cy, double altitude,
                     double side_length, double speed);

    TrajPoint sample(double t) const override;
    double duration() const override { return -1.0; }
    void reset() override {}
    std::string name() const override { return "square"; }

private:
    double cx_, cy_, alt_, side_, speed_;
    double half_side_;
    double seg_time_;   // time to traverse one side [s]
    double total_time_; // time to traverse all 4 sides [s]
};


/* ======================== Waypoint ======================== */

class WaypointTrajectory : public Trajectory
{
public:
    /**
     * @param altitude    Flight altitude [m]
     * @param speed       Speed between waypoints [m/s]
     * @param loop        true = loop back to first, false = stop at last
     */
    WaypointTrajectory(double altitude, double speed, bool loop = true);

    /// Add a single waypoint (x, y, yaw)
    void addWaypoint(double x, double y, double yaw = 0.0);
    void addWaypoint(const Eigen::Vector3d& pos, double yaw = 0.0);

    /// Replace all waypoints at once
    void setWaypoints(const std::vector<Eigen::Vector3d>& pts);

    /// Remove all waypoints
    void clearWaypoints();

    TrajPoint sample(double t) const override;
    double duration() const override { return loop_ ? -1.0 : total_len_ / speed_; }
    void reset() override {}
    std::string name() const override { return "waypoint"; }

private:
    double alt_, speed_;
    bool loop_;

    struct Wpt { Eigen::Vector3d pos; double yaw; };
    std::vector<Wpt> wpts_;
    std::vector<double> seg_lens_;    // length of each segment [m]
    std::vector<double> cum_lens_;    // cumulative length [m]
    double total_len_;                // total path length [m]

    void rebuild();
};


/* ======================== Factory ======================== */

/**
 * Create a trajectory by name.
 *
 * Valid types and their param1 / param2 meanings:
 *   "circle"   → param1 = radius,         param2 = speed
 *   "square"   → param1 = side_length,    param2 = speed
 *   "waypoint" → param1 = (unused),       param2 = speed
 */
std::shared_ptr<Trajectory> createTrajectory(
    const std::string& type,
    double cx, double cy, double alt,
    double param1, double param2);

}  // namespace traj_gen

#endif  // __TRAJ_GEN_H__
