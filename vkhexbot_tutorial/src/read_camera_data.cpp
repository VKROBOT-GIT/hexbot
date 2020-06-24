#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<ros/ros.h>
static bool bSaveImageToFile = true;

void Cam_RGB_Callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    if(bSaveImageToFile == true)
    {
        imwrite("/home/vkrobot/camera_rgb.jpg",cv_ptr->image);
        printf("保存一帧摄像头图像到 camera_rgb.jpg 文件\n");
        bSaveImageToFile = false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "color_image_node");

    ros::NodeHandle nh;
    ros::Subscriber rgb_sub = nh.subscribe("/usb_cam/image_raw", 1 ,Cam_RGB_Callback);

    ros::Rate loop_rate(30);
    while( ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
