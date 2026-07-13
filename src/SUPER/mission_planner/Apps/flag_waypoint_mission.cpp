//
// Created by yunfan on 2021/8/29.
//

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
/*
 * Test code:
 *      roslaunch simulator test_env.launch
 * */
#define BACKWARD_HAS_DW 1
#include "utils/backward.hpp"
namespace backward{
    backward::SignalHandling sh;
}

#include "waypoint_mission/flag_waypoint_planner.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "flag_waypoint_mission");
    ros::NodeHandle nh("~");
    /* Publisher and subcriber */

    for(int i = 0; i < 10; i++) {
        ros::Duration(0.1).sleep();
    }

	mission_planner::WaypointPlanner wpl(nh);

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Duration(1.0).sleep();
    ros::waitForShutdown();
    return 0;
}

