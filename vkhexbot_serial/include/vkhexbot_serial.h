#ifndef VKHEXBOT_SERIAL_H_
#define VKHEXBOT_SERIAL_H_

#include <ros/ros.h>
#include <serial/serial.h>
#include <string.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <vkhexbot_msgs/Buffer.h>
#include <vkhexbot_msgs/Battery.h>
#include "hexbotProtocol.h"
#include "hexbotProtocolPack.h"

class VKHexbotSerial{
	public:
		VKHexbotSerial(void);
		//读取数据
		size_t readData(unsigned char* buffer);
		//发送数据
		void writeData(unsigned char* buffer, int len);
		//获取串口状态
		bool getSerialStatus();
		//打开串口
		void openSerial();
                //解析接收数据
		void recvDataProcess(HexbotProtocol *dataFrame);
		//
		void publishOdometry();
        ros::WallTime getLastCommandTime();
		bool getMovingState();
		void setMovingState(bool);
		void cmd_velCallback(const geometry_msgs::TwistConstPtr &cmd_vel_msg);
	private:
		bool initControlMode;
		bool movingState;
		ros::WallTime last_command_time_;
		//订阅话题
		ros::Subscriber cmd_vel_sub_;
		
        ros::Subscriber app_control_msg_sub_;
		void app_control_msg_Callback(const vkhexbot_msgs::Buffer::ConstPtr& msg);
		//发布话题
		ros::Publisher odom_pub_;	
        tf::TransformBroadcaster odom_broadcaster;	
		ros::Publisher battery_pub_;
		//节点句柄
		ros::NodeHandle nh_;
		//接收到的目标速度
		geometry_msgs::Twist cmd_vel_imcoming;
		//串口
		serial::Serial ser;
		//串口设备名，波特率
		int BAUDRATE;
		std::string SERIAL_NAME;
		ros::Time last_vel_time_;
		ros::Subscriber state_sub_;
        void stateCallback( const std_msgs::BoolConstPtr &state_msg );
		bool hex_state_;      // Current loop state
	    float xPos,yPos,zPos;
		float linear_x_scale_;
		float linear_y_scale_;
        float angle_z_scale_;
		float linear_velocity_x_;
		float linear_velocity_y_;
		float angular_velocity_z_;
		float vel_dt_;
        unsigned char elecPercentage;
        
};

#endif  //HEXAPOD_SERIAL_H_
