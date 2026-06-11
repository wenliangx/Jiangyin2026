#include <iostream>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
// #include "quadrotor_msgs/PositionCommand.h"
#include "super_msgs/PositionCommand.h"

ros::Subscriber _cmd_sub,_super_cmd_sub;
ros::Publisher  _odom_pub;

// quadrotor_msgs::PositionCommand _cmd;
super_msgs::PositionCommand _super_cmd;
double _init_x, _init_y, _init_z;
bool ego_or_super;

bool rcv_cmd = false;
// void egoPosCmdCallBack(const quadrotor_msgs::PositionCommand cmd)
// {	
// 	rcv_cmd = true;
// 	_cmd    = cmd;
// }
void superPosCmdCallBack(const super_msgs::PositionCommand cmd)
{	
	rcv_cmd = true;
	_super_cmd    = cmd;
}

void pubOdom()
{	
	nav_msgs::Odometry odom;
	odom.header.stamp    = ros::Time::now();
	odom.header.frame_id = "world";

	if(rcv_cmd)
	{
		Eigen::Vector3d alpha;
	    if(true){
			odom.pose.pose.position.x = _super_cmd.position.x;
			odom.pose.pose.position.y = _super_cmd.position.y;
			odom.pose.pose.position.z = _super_cmd.position.z;
			alpha = Eigen::Vector3d(_super_cmd.acceleration.x, _super_cmd.acceleration.y, _super_cmd.acceleration.z) + 9.8*Eigen::Vector3d(0,0,1);
			Eigen::Vector3d xC(cos(_super_cmd.yaw), sin(_super_cmd.yaw), 0);
			Eigen::Vector3d yC(-sin(_super_cmd.yaw), cos(_super_cmd.yaw), 0);
			Eigen::Vector3d xB = (yC.cross(alpha)).normalized();
			Eigen::Vector3d yB = (alpha.cross(xB)).normalized();
			Eigen::Vector3d zB = xB.cross(yB);
			if (std::abs(xB.dot(yB)) > 1e-6 || std::abs(xB.dot(zB)) > 1e-6 || std::abs(yB.dot(zB)) > 1e-6) {
				std::cerr << "[error] Rotation matrix vectors are not orthogonal!" << std::endl;
				return;
    		}
			Eigen::Matrix3d R;
			R.col(0) = xB;
			R.col(1) = yB;
			R.col(2) = zB;
			Eigen::Quaterniond q(R);
			q.normalize();
			if(q.norm()!=1)
			{
				// std::cout<<"[debug] q is not norm:"<<q.vec()<<std::endl;
				odom.pose.pose.orientation.w = 1;
				odom.pose.pose.orientation.x = 0;
				odom.pose.pose.orientation.y = 0;
				odom.pose.pose.orientation.z = 0;
			}else{
				// std::cout<<"[debug] q:"<<q.vec()<<std::endl;
				odom.pose.pose.orientation.w = q.w();
				odom.pose.pose.orientation.x = q.x();
				odom.pose.pose.orientation.y = q.y();
				odom.pose.pose.orientation.z = q.z();
			}
			
			odom.twist.twist.linear.x = _super_cmd.velocity.x;
			odom.twist.twist.linear.y = _super_cmd.velocity.y;
			odom.twist.twist.linear.z = _super_cmd.velocity.z;

			odom.twist.twist.angular.x = _super_cmd.acceleration.x;
			odom.twist.twist.angular.y = _super_cmd.acceleration.y;
			odom.twist.twist.angular.z = _super_cmd.acceleration.z;
		}else{
			// odom.pose.pose.position.x = _cmd.position.x;
			// odom.pose.pose.position.y = _cmd.position.y;
			// odom.pose.pose.position.z = _cmd.position.z;
			// Eigen::Vector3d alpha = Eigen::Vector3d(_cmd.acceleration.x, _cmd.acceleration.y, _cmd.acceleration.z) + 9.8*Eigen::Vector3d(0,0,1);
			// odom.pose.pose.position.x = _cmd.position.x;
			// odom.pose.pose.position.y = _cmd.position.y;
			// odom.pose.pose.position.z = _cmd.position.z;
			// alpha = Eigen::Vector3d(_cmd.acceleration.x, _cmd.acceleration.y, _cmd.acceleration.z) + 9.8*Eigen::Vector3d(0,0,1);
			// Eigen::Vector3d xC(cos(_cmd.yaw), sin(_cmd.yaw), 0);
			// Eigen::Vector3d yC(-sin(_cmd.yaw), cos(_cmd.yaw), 0);
			// Eigen::Vector3d xB = (yC.cross(alpha)).normalized();
			// Eigen::Vector3d yB = (alpha.cross(xB)).normalized();
			// Eigen::Vector3d zB = xB.cross(yB);
			// Eigen::Matrix3d R;
			// R.col(0) = xB;
			// R.col(1) = yB;
			// R.col(2) = zB;
			// Eigen::Quaterniond q(R);
			// odom.pose.pose.orientation.w = q.w();
			// odom.pose.pose.orientation.x = q.x();
			// odom.pose.pose.orientation.y = q.y();
			// odom.pose.pose.orientation.z = q.z();
			
			// odom.twist.twist.linear.x = _cmd.velocity.x;
			// odom.twist.twist.linear.y = _cmd.velocity.y;
			// odom.twist.twist.linear.z = _cmd.velocity.z;

			// odom.twist.twist.angular.x = _cmd.acceleration.x;
			// odom.twist.twist.angular.y = _cmd.acceleration.y;
			// odom.twist.twist.angular.z = _cmd.acceleration.z;
		}
		
		
	}
	else
	{
		odom.pose.pose.position.x = _init_x;
	    odom.pose.pose.position.y = _init_y;
	    odom.pose.pose.position.z = _init_z;

	    odom.pose.pose.orientation.w = 1;
	    odom.pose.pose.orientation.x = 0;
	    odom.pose.pose.orientation.y = 0;
	    odom.pose.pose.orientation.z = 0;

	    odom.twist.twist.linear.x = 0.0;
	    odom.twist.twist.linear.y = 0.0;
	    odom.twist.twist.linear.z = 0.0;

	    odom.twist.twist.angular.x = 0.0;
	    odom.twist.twist.angular.y = 0.0;
	    odom.twist.twist.angular.z = 0.0;
	}

    _odom_pub.publish(odom);
}

int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "odom_generator");
    ros::NodeHandle nh( "~" );

    nh.param("init_x", _init_x,  0.0);
    nh.param("init_y", _init_y,  0.0);
    nh.param("init_z", _init_z,  0.0);


	// if(ego_or_super){
		_super_cmd_sub = nh.subscribe( "command", 1, superPosCmdCallBack);
	// }else{
		// _cmd_sub  = nh.subscribe( "command", 1, egoPosCmdCallBack );
	// }
    
    _odom_pub = nh.advertise<nav_msgs::Odometry>("odometry", 1);                      

    ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
		pubOdom();                   
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }

    return 0;
}