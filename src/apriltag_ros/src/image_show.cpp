#include<ros/ros.h> //ros标准库头文件
#include<iostream> //C++标准输入输出库
//OpenCV2标准头文件
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{  
    try  
    {   
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        Mat colorImg = cv_ptr->image;    

        // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
        // cv::Mat colorImg = cv_ptr->image;

        imshow("view",colorImg);    
        cv::waitKey(1);
        // cout << "i get" << endl;
    }
    catch (cv_bridge::Exception& e)
    {    
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());  
    }
}

//主函数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "SUB_RGB");
    // ros::NodeHandle nh;  
    // image_transport::ImageTransport it(nh);  
    // image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);

    ros::NodeHandle *nh = new ros::NodeHandle;
    // image_transport::ImageTransport it(nh[0]);
    image_transport::ImageTransport it(*nh);
    image_transport::Subscriber sub = it.subscribe("/tag_detections_image", 1, imageCallback);

    ros::spin();    
}