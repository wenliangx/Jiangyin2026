#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"

#include "apriltag_echo_message/apriltag_echo_message.h"

using namespace std;

geometry_msgs::PoseStamped intput_pose;  // receive PoseStamped message from rostopic, waiting to be transformed
geometry_msgs::PoseStamped output_pose;  // tf convertion output, published as a rostopic

void PoseListenerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // PoseStamped
    intput_pose.header.frame_id = "usb_cam";
    // intput_pose.header.stamp = ros::Time::now();
    intput_pose.header.stamp = msg->header.stamp;

    intput_pose.pose.position.x = msg->pose.position.x;
    intput_pose.pose.position.y = msg->pose.position.y;
    intput_pose.pose.position.z = msg->pose.position.z;

    intput_pose.pose.orientation.x = msg->pose.orientation.x;
    intput_pose.pose.orientation.y = msg->pose.orientation.y;
    intput_pose.pose.orientation.z = msg->pose.orientation.z;
    intput_pose.pose.orientation.w = msg->pose.orientation.w;
}


void InitMsgPos()
{
    intput_pose.header.frame_id = "usb_cam";
    // intput_pose.header.stamp = ros::Time::now();

    intput_pose.pose.position.x = 0;
    intput_pose.pose.position.y = 0;
    intput_pose.pose.position.z = 0;

    intput_pose.pose.orientation.x = 0;
    intput_pose.pose.orientation.y = 0;
    intput_pose.pose.orientation.z = 0;
    intput_pose.pose.orientation.w = 1;
}


int main(int argc, char *argv[])
{
    InitMsgPos();

    setlocale(LC_ALL, "");

    ros::init(argc, argv, "tf_pub");
    ros::NodeHandle nh;

    // 创建 TF 订阅对象
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    ros::Rate r(10);

    ros::Subscriber sub = nh.subscribe("uav_raw_pose", 10, PoseListenerCallback);
    ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>("tf_output", 10);

    while (ros::ok())
    {
        try
        {
            // 解析 usb_cam 中的点在 tf_trans 中的坐标
            geometry_msgs::TransformStamped tfs = buffer.lookupTransform("tf_trans", "usb_cam", ros::Time(0));
            string exi_sys = tfs.child_frame_id;
            string new_sys = tfs.header.frame_id;

            cout << "raw data: " << endl
                 << "  pos: x=" << intput_pose.pose.position.x << " y=" << intput_pose.pose.position.y
                 << " z=" << intput_pose.pose.position.z << endl;
            vector<float> input_pose_rpy = toEulerAngle(intput_pose.pose.orientation.x, intput_pose.pose.orientation.y,
                                                        intput_pose.pose.orientation.z, intput_pose.pose.orientation.w);
            cout << "  ori_rpy: x=" << RadToAngle(input_pose_rpy[0])
                 << " y=" << RadToAngle(input_pose_rpy[1])
                 << " z=" << RadToAngle(input_pose_rpy[2]) << endl;

            // tf transform
            output_pose = buffer.transform(intput_pose, "tf_trans");
            output_pose.header.stamp = intput_pose.header.stamp;
            // show the output of tf transform
            cout << "PoseStamped in tf_trans: Position" << endl
                 << "  x=" << output_pose.pose.position.x
                 << ", y=" << output_pose.pose.position.y
                 << ", z=" << output_pose.pose.position.z << endl;
            // cout << "PoseStamped in tf_trans: Orientation" << endl
            //      << "  x=" << output_pose.pose.orientation.x
            //      << ", y=" << output_pose.pose.orientation.y
            //      << ", z=" << output_pose.pose.orientation.z
            //      << ", w=" << output_pose.pose.orientation.w << endl;

            vector<float> output_pose_rpy = toEulerAngle(output_pose.pose.orientation.x, output_pose.pose.orientation.y,
                                                         output_pose.pose.orientation.z, output_pose.pose.orientation.w);

            cout << "PoseStamped in tf_trans: Orientation_rpy" << endl
                 << "  x=" << RadToAngle(output_pose_rpy[0])
                 << ", y=" << RadToAngle(output_pose_rpy[1])
                 << ", z=" << RadToAngle(output_pose_rpy[2]) << endl;

            if(abs(intput_pose.pose.position.x) < 10000 && abs(intput_pose.pose.position.z) > 0.03)
            {
                // publish the output of tf transform
                pub_pose.publish(output_pose);
            }
            
            cout << "---------------------one loop--------------------" << endl;
        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            ROS_ERROR("异常信息:%s", e.what());
        }

        r.sleep();
        ros::spinOnce();

        
    }
    return 0;
}