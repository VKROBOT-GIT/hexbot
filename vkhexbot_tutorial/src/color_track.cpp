#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

using namespace cv;
using namespace std;
static int iLowH = 0;
static int iHighH = 43;

static int iLowS = 0;
static int iHighS = 116;

static int iLowV = 0;
static int iHighV = 122;

geometry_msgs::Twist vel_cmd; //速度消息包
ros::Publisher vel_pub;       //速度发送
int nCountToStop = 0;         //计时然后停止
double globalArea = 2000;
void Cam_RGB_Callback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    Mat imgOriginal = cv_ptr->image;

    //将RGB图片转换成HSV
    Mat imgHSV;
    vector<Mat> hsvSplit;
    cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

    //在HSV空间做直方图均衡化
    split(imgHSV, hsvSplit);
    equalizeHist(hsvSplit[2], hsvSplit[2]);
    merge(hsvSplit, imgHSV);
    Mat imgThresholded;

    //使用上面的Hue,Saturation和Value的阈值范围对图像进行二值化
    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);

    //开操作 (去除一些噪点)
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);

    //闭操作 (连接一些连通域)
    morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);

    //获取边界
    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hireachy;
    Rect rect;
    Point2f center;
    float radius = 0;
    double maxArea = 0;
    findContours(imgThresholded, contours, hireachy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
    //框选面积最大的边界
    if (contours.size() > 0)
    {
        for (int i = 0; i < contours.size(); i++)
        {
            double area = contourArea(contours[static_cast<int>(i)]);
            if (area > maxArea)
            {
                maxArea = area;
                rect = boundingRect(contours[static_cast<int>(i)]);
                minEnclosingCircle(contours[static_cast<int>(i)], center, radius);
            }
        }
    }
    if (contours.size() > 0)
    {
        //矩形框
        rectangle(imgThresholded, rect, Scalar(0, 255, 0), 2);
        printf("x    y ( %d , %d ) ", center.x, center.y);
        //画坐标
        Point line_begin = Point(center.x - 10, center.y);
        Point line_end = Point(center.x + 10, center.y);
        line(imgOriginal, line_begin, line_end, Scalar(255, 0, 0), 3);
        line_begin.x = center.x;
        line_begin.y = center.y - 10;
        line_end.x = center.x;
        line_end.y = center.y + 10;
        line(imgOriginal, line_begin, line_end, Scalar(255, 0, 0), 3);
        //计算机器人运动速度
        float fVelFoward = (maxArea - globalArea) * 0.002 * -1; //面积差值*比例*方向
        float fVelTurn = (center.x - 320) * 0.003 * -1;         //横坐标差值*比例*方向
        vel_cmd.linear.x = fVelFoward;
        vel_cmd.linear.y = 0;
        vel_cmd.linear.z = 0;
        vel_cmd.angular.x = 0;
        vel_cmd.angular.y = 0;
        vel_cmd.angular.z = fVelTurn;
    }
    else
    {
        printf("目标颜色消失...\n");
        vel_cmd.linear.x = 0;
        vel_cmd.linear.y = 0;
        vel_cmd.linear.z = 0;
        vel_cmd.angular.x = 0;
        vel_cmd.angular.y = 0;
        vel_cmd.angular.z = 0;
    }
    //运行10秒(300帧图像)后自动停止
    nCountToStop++;
    if (nCountToStop > 300)
    {
        vel_cmd.linear.x = 0;
        vel_cmd.angular.z = 0;
        ROS_WARN("Stop!");
    }

    vel_pub.publish(vel_cmd);
    printf("速度( 前后= %.2f , 旋转= %.2f )\n", vel_cmd.linear.x, vel_cmd.angular.z);

    //显示处理结果
    imshow("RGB", imgOriginal);
    imshow("HSV", imgHSV);
    imshow("Result", imgThresholded);
    cv::waitKey(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "color_track");

    ros::NodeHandle nh;
    ros::Subscriber rgb_sub = nh.subscribe("/usb_cam/image_raw", 1, Cam_RGB_Callback);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 30);

    ros::Rate loop_rate(30);

    namedWindow("RGB");
    namedWindow("HSV");
    namedWindow("Result");

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
