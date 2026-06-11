/* the logic of echo message:
    if we get the value of rostopic named: /tag_detection 
    take the average of x (y/z) as the postion output
    take the oritation of the first detected tag as the oritation output
*/

#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_echo_message/apriltag_echo_message.h"
#include "geometry_msgs/PoseStamped.h"
#include "iostream"
#include <vector>

using namespace std;

ros::Subscriber ar_sub_;
geometry_msgs::PoseStamped msg_pos;

// float bias_x[5] = {0.32, -0.32, 0, 0.32, -0.32};  // 0 1 2 3 4
// float bias_y[5] = {0.31, 0.31, 0, -0.31, -0.31};
// float bias_x[5] = {0, 0, 0, 0, 0};
// float bias_y[5] = {0, 0, 0, 0, 0};
float bias_x[5] = {-0.40, 0.40, 0, -0.40 , 0.40};  // 0 1 2 3 4
float bias_y[5] = {-0.40, -0.40, 0, 0.40, 0.40};
// float bias_x[5] = {0.345, -0.345, 0, 0.345 , -0.345};  // 0 1 2 3 4
// float bias_y[5] = {0.34, 0.34, 0, -0.34, -0.34};
float bias_z[5] = {0, 0, 0, 0, 0};

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
    const int detection_num = msg->detections.size();

    vector<float> pos_arr[detection_num], ori_arr[detection_num];
    cout << "detected tag number: " << detection_num << endl;

    if (detection_num > 0)
    {
        for (int i = 0; i < detection_num; i++)
        {
            int tag_id = msg->detections[i].id[0];
            cout << "tag id: " << tag_id << endl;

            float tag_size = msg->detections[i].size[0];
            cout << "tag size: " << tag_size << endl;

            vector<float> pos_temp;
            for (int j = 0; j < 3; j++)
                pos_temp.push_back(0);

            pos_temp[0] = msg->detections[i].pose.pose.pose.position.x + bias_x[tag_id]; // get position message from /tag_detections
            pos_temp[1] = msg->detections[i].pose.pose.pose.position.y + bias_y[tag_id];
            pos_temp[2] = msg->detections[i].pose.pose.pose.position.z + bias_z[tag_id];
            cout << "tag position: " << pos_temp[0] << " " << pos_temp[1] << " " << pos_temp[2] << endl;
            ori[0] = msg->detections[i].pose.pose.pose.orientation.x;  // get orientation message from /tag_detections
            ori[1] = msg->detections[i].pose.pose.pose.orientation.y;
            ori[2] = msg->detections[i].pose.pose.pose.orientation.z;
            ori[3] = msg->detections[i].pose.pose.pose.orientation.w;

            pos_arr[i] = pos_temp;
            ori_arr[i] = ori;
        }
    }
    else if (detection_num == 0)
    {
        cout << "No tag target found!" << endl;
    }


    for (int i = 0; i < 3; i++)
    {
        float pos_value = 0;

        for (int j = 0; j < detection_num; j++)
        {
            pos_value += pos_arr[j][i];
        }
        pos[i] = pos_value / detection_num; // average
    }

    ros::Time currentTime = ros::Time::now();
    // msg_pos.header.stamp = currentTime;
    msg_pos.header.stamp = msg->header.stamp;

    msg_pos.pose.position.x = pos[0];
    msg_pos.pose.position.y = pos[1];
    msg_pos.pose.position.z = pos[2];

    msg_pos.pose.orientation.x = ori[0];
    msg_pos.pose.orientation.y = ori[1];
    msg_pos.pose.orientation.z = ori[2];
    msg_pos.pose.orientation.w = ori[3];

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

