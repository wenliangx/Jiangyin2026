#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_echo_message/apriltag_echo_message.h"
#include "geometry_msgs/PoseStamped.h"
#include "iostream"
#include <vector>

using namespace std;

ros::Subscriber ar_sub_;
geometry_msgs::PoseStamped msg_pos;


class Localizer{
private:
    vector<float> pos;  // position: x, y, z
    vector<float> ori;  // orientation represented by Quaternions: x, y, z, w
    vector<float> ori_rpy;  // orientation represented by Euler Angles: roll, pitch, yall

public:
    Localizer(ros::NodeHandle &nh);
    void number_callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);
};


void Localizer::number_callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    if (msg->detections.size() > 0)
    {
        for (int i = 0; i < msg->detections.size(); i++)
        {

            int tag_id = msg->detections[i].id[0];
            cout << "tag id: " << tag_id << endl;

            float tag_size = msg->detections[i].size[0];
            cout << "tag size: " << tag_size << endl;

            pos[0] = msg->detections[i].pose.pose.pose.position.x;  // get position message from /tag_detections
            pos[1] = msg->detections[i].pose.pose.pose.position.y;
            pos[2] = msg->detections[i].pose.pose.pose.position.z;

            ori[0] = msg->detections[i].pose.pose.pose.orientation.x;  // get orientation message from /tag_detections
            ori[1] = msg->detections[i].pose.pose.pose.orientation.y;
            ori[2] = msg->detections[i].pose.pose.pose.orientation.z;
            ori[3] = msg->detections[i].pose.pose.pose.orientation.w;

            // cout message
            // ori_rpy = toEulerAngle(ori[0], ori[1], ori[2], ori[3]);  // convert from quaternions to euler angles

            // ori_rpy[0] = RadToAngle(ori_rpy[0]);  // convert from rad to angle
            // ori_rpy[1] = RadToAngle(ori_rpy[1]);
            // ori_rpy[2] = RadToAngle(ori_rpy[2]);

            // cout << "position:" << endl;
            // cout << "  x: " << pos[0] << endl;
            // cout << "  y: " << pos[1] << endl;
            // cout << "  z: " << pos[2] << endl;
            // // ROS_INFO_STREAM(msg->detections[0].pose.pose.pose);

            // cout << "orientation" << endl;
            // cout << "  x: " << ori[0] << endl;
            // cout << "  y: " << ori[1] << endl;
            // cout << "  z: " << ori[2] << endl;
            // cout << "  w: " << ori[3] << endl;

            // cout << "orientation RPY" << endl;
            // cout << "  roll:  " << ori_rpy[0] << "°" << endl;
            // cout << "  pitch: " << ori_rpy[1] << "°" << endl; // -90 ~ 90
            // cout << "  yaw:   " << ori_rpy[2] << "°" << endl; // -180 ~ 180

            // cout << endl;
        }
    }
    else{
        cout << "No tag target found!" << endl;
    }

    ros::Time currentTime = ros::Time::now();
    msg_pos.header.stamp = currentTime;

    msg_pos.pose.position.x = pos[0];
    msg_pos.pose.position.y = pos[1];
    msg_pos.pose.position.z = pos[2];

    msg_pos.pose.orientation.x = ori[0];
    msg_pos.pose.orientation.y = ori[1];
    msg_pos.pose.orientation.z = ori[2];
    msg_pos.pose.orientation.w = ori[3];

    // ROS_INFO("publishing the position and orientaion");
    // ROS_INFO("position(x,y,z): %f , %f, %f", msg_pos.pose.position.x,
    //         msg_pos.pose.position.y, msg_pos.pose.position.z);
    // ROS_INFO("orientation(x,y,z,w): %f , %f, %f, %f", msg_pos.pose.orientation.x,
    //         msg_pos.pose.orientation.y, msg_pos.pose.orientation.z, msg_pos.pose.orientation.w);
    // ROS_INFO("the time we get the pose is %f", msg_pos.header.stamp.sec + 1e-9 * msg_pos.header.stamp.nsec);

    cout << "---echo message finished---" << endl;
}

Localizer::Localizer(ros::NodeHandle &nh){
    ar_sub_ = nh.subscribe<apriltag_ros::AprilTagDetectionArray>("/tag_detections", 1, &Localizer::number_callback, this);
    for (int i = 0; i < 3; i++)
    {
        pos.push_back(0);
        ori.push_back(0);
        ori_rpy.push_back(0);
    }
        ori.push_back(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "apriltag_detector_subscriber");
    ros::NodeHandle node_obj;
    Localizer localizer(node_obj);

    ros::Publisher pub_raw_pose = node_obj.advertise<geometry_msgs::PoseStamped>("uav_raw_pose", 10); // initialize chatter
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        pub_raw_pose.publish(msg_pos);  // publish raw data from /tag_detection as rostopic PoseStamped
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}

