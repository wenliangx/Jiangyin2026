#include <iostream>
#include <algorithm>
#include <ros/ros.h>
#include <string>
#include <unistd.h>
#include <traj_utils/Flag.h>

ros::Publisher _flag_pub;
int id = 0;
int mode = 1;
int waypoint_num_;
double waypoints_[300][3];

void flag_msg_pub(int id, int mode, double x, double y, double z)
{
	traj_utils::Flag flag;
	flag.header.frame_id = "world";
	flag.header.stamp = ros::Time::now();
	flag.header.seq = 0;
	flag.id = id;
	flag.mode = mode;
	flag.position.x = x;
	flag.position.y = y;
	flag.position.z = z;

	_flag_pub.publish(flag);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "flag_pubtest");
	ros::NodeHandle nh("~");
	_flag_pub = nh.advertise<traj_utils::Flag>("/ego_planner/flag_msg", 1);

	ROS_INFO("flag_pubtest node started");

	nh.param("point_num", waypoint_num_, -1);
	for (int i = 0; i < waypoint_num_; i++)
	{
		nh.param("point" + std::to_string(i) + "_x", waypoints_[i][0], -1.0);
		nh.param("point" + std::to_string(i) + "_y", waypoints_[i][1], -1.0);
		nh.param("point" + std::to_string(i) + "_z", waypoints_[i][2], -1.0);
		ROS_INFO("Flag %d: x: %f, y: %f, z: %f", i, waypoints_[i][0], waypoints_[i][1], waypoints_[i][2]);
	}
	ros::Rate rate(5);
	sleep(4.0);

	while (ros::ok())
	{
		rate.sleep();
		flag_msg_pub(id, mode, waypoints_[id][0], waypoints_[id][1], waypoints_[id][2]);
		ROS_INFO("Publishing flag %d", id);
		ROS_INFO("Flag %d: x: %f, y: %f, z: %f", id, waypoints_[id][0], waypoints_[id][1], waypoints_[id][2]);
		id++;

		if (id >= waypoint_num_)
			break;
	}

	return 0;
}