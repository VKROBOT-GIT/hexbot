#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace cv;
using namespace std;

static int iLowH = 20;
static int iHighH = 80;
static int iLowS = 90;
static int iHighS = 255;
static int iLowV = 50;
static int iHighV = 255;

void Cam_RGB_Callback(const sensor_msgs::ImageConstPtr& msg)
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
    equalizeHist(hsvSplit[2],hsvSplit[2]);
    merge(hsvSplit,imgHSV);
    Mat imgThresholded; 

    //使用上面的Hue,Saturation和Value的阈值范围对图像进行二值化 
    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);

    //开操作 (去除一些噪点)
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);  

    //闭操作 (连接一些连通域)
    morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);

    //遍历二值化后的图像数据
    int nTargetX = 0;
    int nTargetY = 0;
    int nPixCount = 0;
    int nImgWidth = imgThresholded.cols;
    int nImgHeight = imgThresholded.rows;
    int nImgChannels = imgThresholded.channels();
    printf("w= %d   h= %d   size = %d\n",nImgWidth,nImgHeight,nImgChannels);
    for (int y = 0; y < nImgHeight; y++)    
    {
    for(int x = 0; x < nImgWidth; x++)
        {
            if(imgThresholded.data[y*nImgWidth + x] == 255)
            {
                nTargetX += x;
                nTargetY += y;
                nPixCount ++;
            }
        }
    }   
    if(nPixCount > 0)
    {
        nTargetX /= nPixCount;
        nTargetY /= nPixCount;
        printf("颜色质心坐标( %d , %d )  点数 = %d\n",nTargetX,nTargetY,nPixCount); 
        //画坐标
        Point line_begin = Point(nTargetX-10,nTargetY);
        Point line_end = Point(nTargetX+10,nTargetY);
        line(imgOriginal,line_begin,line_end,Scalar(255,0,0));  
        line_begin.x = nTargetX; line_begin.y = nTargetY-10;
        line_end.x = nTargetX; line_end.y = nTargetY+10;
        line(imgOriginal,line_begin,line_end,Scalar(255,0,0));
    }
    else
    {
        printf("目标颜色消失...\n");
    }

    //显示处理结果
    imshow("RGB", imgOriginal);
    imshow("HSV", imgHSV);
    imshow("Result", imgThresholded);
    cv::waitKey(5);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hsv_test");

    ros::NodeHandle nh;
    ros::Subscriber rgb_sub = nh.subscribe("/usb_cam/image_raw", 1 , Cam_RGB_Callback);

    ros::Rate loop_rate(30);

    //生成图像显示和参数调节的窗口空见
    namedWindow("Control", CV_WINDOW_AUTOSIZE);

    cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);

    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);

    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);     

    namedWindow("RGB");
    namedWindow("HSV");
    namedWindow("Result");
    while( ros::ok())
    {   
        ros::spinOnce();
        loop_rate.sleep();
    }
}
