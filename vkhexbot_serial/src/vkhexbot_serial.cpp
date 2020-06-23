#include "vkhexbot_serial.h"


VKHexbotSerial::VKHexbotSerial():
	BAUDRATE(115200),
	SERIAL_NAME("/dev/vk_hexbot"),
	linear_velocity_x_(0.0),
    linear_velocity_y_(0.0),
    angular_velocity_z_(0.0),
	linear_x_scale_(0.0),
	linear_y_scale_(0.0),
	angle_z_scale_(0.0),
    last_vel_time_(0),   
    xPos(0.0),
    yPos(0.0),
	zPos(0.0),
	vel_dt_(0),
	elecPercentage(0),
	initControlMode(false),
	movingState(false)
{
	ros::param::get("BAUDRATE",BAUDRATE);
	ros::param::get("SERIAL_NAME",SERIAL_NAME);
	ros::param::get("vkhexbot_serial/linear_x_scale",linear_x_scale_);
	ros::param::get("vkhexbot_serial/linear_y_scale",linear_y_scale_);
    ros::param::get("vkhexbot_serial/angle_z_scale",angle_z_scale_);
   	hex_state_ = false;

	cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>( "/cmd_vel", 1, &VKHexbotSerial::cmd_velCallback, this );
	app_control_msg_sub_ = nh_.subscribe<vkhexbot_msgs::Buffer>("chatter",10,&VKHexbotSerial::app_control_msg_Callback, this);
	state_sub_ = nh_.subscribe<std_msgs::Bool>( "/state", 1, &VKHexbotSerial::stateCallback, this );
	odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom",50);
	battery_pub_ = nh_.advertise<vkhexbot_msgs::Battery>("battery",10);
	//打开串口
	openSerial();
}

//读取数据
size_t VKHexbotSerial::readData(unsigned char* buffer){
	size_t bytes_read = 0;
	if (ser.available()) {
        bytes_read = ser.read(buffer, ser.available());
	}
	return bytes_read;
}

//发送数据
void VKHexbotSerial::writeData(unsigned char* buffer, int len){
	ser.write(buffer,len);
}

//获取串口状态
bool VKHexbotSerial::getSerialStatus(){
	return ser.isOpen();
}


//打开串口
void VKHexbotSerial::openSerial(){
	//打开串口
	try {
		//ser.setPort("/dev/vkhexbot_serial");
		//ser.setBaudrate(115200);
		ser.setPort(SERIAL_NAME);
		ser.setBaudrate(BAUDRATE);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		ser.setTimeout(to);
		ser.open();
	} catch (serial::IOException &e) {
		ROS_ERROR_STREAM("Unable to open port,Please, connect to device. ");   
    }
    if (ser.isOpen()) {
		ROS_INFO_STREAM("Serial Port initialized");
	} 
}

void VKHexbotSerial::cmd_velCallback(const geometry_msgs::TwistConstPtr &cmd_vel_msg){
	if(!initControlMode){
		HexbotProtocol data; 	
		data.resetDataFrame();
		set_control_mode_pack(1,&data);
		int len = data.getMessageTotalLen();      
		byte *tempBuf = new byte[len];                    
		memset(tempBuf,0,len);                   
		data.dataFrameToByteArray(tempBuf,len);   
		writeData(tempBuf,len);
		initControlMode = true;
		delete tempBuf;
	}
	last_command_time_ = ros::WallTime::now();
	
	cmd_vel_imcoming.linear.x = cmd_vel_msg->linear.x;
	cmd_vel_imcoming.linear.y = cmd_vel_msg->linear.y;
	cmd_vel_imcoming.angular.z = cmd_vel_msg->angular.z;
	//串口发送设置速度命令到底层机器人
	HexbotProtocol dataFrame; 
	set_gait_navigate_vel_pack(cmd_vel_imcoming.linear.x,cmd_vel_imcoming.linear.y,cmd_vel_imcoming.angular.z, &dataFrame);
	int len = dataFrame.getMessageTotalLen();      
	byte *buf = new byte[len];                    
	memset(buf,0,len);                   
	dataFrame.dataFrameToByteArray(buf,len);   
	writeData(buf,len);
	/*if(cmd_vel_msg->linear.x == 0.0 && cmd_vel_msg->linear.y == 0.0 && cmd_vel_msg->angular.z == 0.0)
		movingState = false;
	else 
		movingState = true;*/
	movingState = true;
	delete buf;
}

bool VKHexbotSerial::getMovingState(){
	return movingState;
}

void VKHexbotSerial::setMovingState(bool state){
	movingState = state;
}
ros::WallTime VKHexbotSerial::getLastCommandTime(){
	return last_command_time_;
}
void VKHexbotSerial::app_control_msg_Callback(const vkhexbot_msgs::Buffer::ConstPtr& msg){
	vkhexbot_msgs::Buffer data;
	data = *msg;
	ser.write(data.buff);
}

void VKHexbotSerial::publishOdometry(){
 	ros::Time current_time = ros::Time::now();
	
	vel_dt_ = (current_time - last_vel_time_).toSec();
   	last_vel_time_ = current_time;
	

	double delta_th = angular_velocity_z_ * vel_dt_;
	zPos += delta_th;

    	double delta_x = (linear_velocity_x_ * cos(zPos) - linear_velocity_y_ * sin(zPos)) * vel_dt_ ; //m
    	double delta_y = (linear_velocity_x_ * sin(zPos) + linear_velocity_y_ * cos(zPos)) * vel_dt_ ; //m
    	xPos += delta_x;
    	yPos += delta_y;
   
	// since all odometry is 6DOF we'll need a quaternion created from yaw
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(zPos);

	// first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = xPos;
	odom_trans.transform.translation.y = yPos;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	//odom_broadcaster.sendTransform( odom_trans );

	// next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";

	// set the position
	odom.pose.pose.position.x = xPos;
	odom.pose.pose.position.y = yPos;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	odom.pose.covariance[0] = 0.0001;  // x
	odom.pose.covariance[7] = 0.0001;  // y
	odom.pose.covariance[14] = 0.00001; // z
	odom.pose.covariance[21] = 1000000000000.0; // rot x
	odom.pose.covariance[28] = 1000000000000.0; // rot y
	odom.pose.covariance[35] = 0.1; // rot z

	// set the velocity
	odom.twist.twist.linear.x = linear_velocity_x_;
	odom.twist.twist.linear.y = linear_velocity_y_;
	odom.twist.twist.linear.z = 0.0;
	odom.twist.twist.angular.x = 0.0;
	odom.twist.twist.angular.y = 0.0;
	odom.twist.twist.angular.z = angular_velocity_z_;
	odom.twist.covariance = odom.pose.covariance; // needed?

	odom_pub_.publish( odom );
 	//std::printf("odom.pose.pose.position.x:%02f \n ",xPos );
        //std::printf("odom.pose.pose.position.y:%02f \n ",yPos);
        //std::printf("odom.pose.pose.orientation:%02f \n ",zPos );
        //std::printf("odom.twist.twist.linear.x:%02f \n ",odom.twist.twist.linear.x );
        //std::printf("odom.twist.twist.linear.y:%02f \n ",odom.twist.twist.linear.y );
        //std::printf("odom.twist.twist.angular.z:%02f \n ",odom.twist.twist.angular.z );
}

void VKHexbotSerial::stateCallback( const std_msgs::BoolConstPtr &state_msg )
{
	HexbotProtocol dataFrame; 
        byte flag = 0;
	if((state_msg->data == true && hex_state_ == true) || (state_msg->data == false && hex_state_ == false))
	    return;
   	if(state_msg->data == true )
    	{
             flag = 1;       
	     hex_state_ = true;
    	}else {
 	     flag = 0;       
	     hex_state_ = false;
	}
	set_gait_navigate_mode_pack(flag, &dataFrame);
	int len = dataFrame.getMessageTotalLen();      
	byte *buf = new byte[len];                    
	memset(buf,0,len);                   
	dataFrame.dataFrameToByteArray(buf,len);   
	writeData(buf,len);
	delete buf;
}

void VKHexbotSerial::recvDataProcess(HexbotProtocol *dataFrame){
  /*byte elecPercentag=0;
  byte controlMode=0;
  byte gaitMode=0;
  byte versionF=0;
  byte versionS=0;
  byte versionT=0;
  byte servoNums=0;
  byte servoIndex[32];
  short servoPos[32];
  short servoOffset[32];
  byte result=3;
  byte actionGroupId=0;
  short actionNums=0;
  short actionCount=0;
  short speedFactor=0;
  byte gpio_num=0;
  byte gpio_value=0;
  short iic_addr=0;*/
  

  switch(dataFrame->getMessageId()){
    /*case RECV_RETURN_SYSTEM_STATUS:     
        //返回系统状态
        recv_return_system_status_decode(dataFrame,elecPercentag,controlMode,gaitMode);
       break;
  
    case RECV_RETURN_CONTROL_MODE:
        //返回系统状态
        recv_return_control_mode_decode(dataFrame,controlMode);
        break;  
	*/
    case RECV_RETURN_BATTERY_INFO:
	{
        //返回电池电量
		vkhexbot_msgs::Battery battery;
		unsigned char elecPercentage = 0;
        recv_return_battery_info_decode(dataFrame,elecPercentage);
		battery.elecPercentage = elecPercentage;
	    battery_pub_.publish(battery);
		
        break;
	}
 
    /*case RECV_RETURN_FIRMWARE_VERSION:
        //返回机器人固件版本号
        recv_return_firmware_version_decode(dataFrame,versionF,versionS,versionT);     
        break;
  
    case RECV_RETURN_SERVOS_POSITION:
        //返回舵机位置
        recv_return_servos_position_decode(dataFrame,servoNums,servoIndex,servoPos);
        break;

    case RECV_RETURN_SERVOS_OFFSET:
        //返回舵机偏差
        recv_return_servos_offset_decode(dataFrame,servoNums,servoIndex,servoOffset);                      
        break;

    case RECV_RETURN_FINISH_DOWNLOAD_SERVO_OFFSET:
        //舵机偏差下载是否成功的返回消息
        recv_return_finish_download_servo_offset_decode(dataFrame,servoNums,result);       
        break;
 
    case RECV_RETURN_RUN_ACTION_GROUP:
        //运行动作组命令的返回信息
        recv_return_run_action_group_decode(dataFrame,actionGroupId,actionNums,result);           
        break;
	case RECV_RETURN_STOP_ACTION_GROUP:
        //停止运行动作组命令的返回信息
        recv_return_stop_action_group_decode(dataFrame,actionGroupId,actionNums,result);           
        break;
    case RECV_RETURN_COMPLETE_ACTION_GROUP: 
        //动作组执行完成的返回信息
        recv_return_complete_action_group_decode(dataFrame,actionGroupId,actionNums);         
        break;
  
    case RECV_RETURN_FINSH_DOWNLOAD_ACTION_GROUP:
        //下载动作组完成的返回消息
        recv_return_finish_download_action_group_decode(dataFrame,actionGroupId,actionCount,result);         
        break;
   
    case RECV_RETURN_ERASE_COMPLETE:
        //擦除完成返回消息
        recv_return_erase_complete_decode(dataFrame,actionGroupId,result);           
        break;
  
    case RECV_RETURN_SET_NOT_GAIT_SPEED:
        //设置速度成功返回消息
        recv_return_set_not_gait_speed_decode(dataFrame,speedFactor,result);           
        break;
  
    case RECV_RETURN_READ_GPIO_VALUE:
        //读取GPIO值返回消息
        recv_return_read_gpio_value_decode(dataFrame,gpio_num,gpio_value);           
        break;

    case RECV_RETURN_SET_IIC_REGISTER_VALUE:
        //设置IIC设备寄存器返回消息
        recv_return_set_iic_register_value_decode(dataFrame,iic_addr,result);          
        break;
  
    case RECV_RETURN_SET_READ_REGISTER_VALUE:  
        //读取IIC设备寄存器值返回消息
        recv_return_read_iic_register_value_decode(dataFrame,iic_addr,result,data);           
        break;
  
    case RECV_RETURN_READ_SENSOR_DATA:
        //读取传感器数据返回消息
        recv_return_read_sensor_data_decode(dataFrame,data);            
        break;*/
	case RECV_RETURN_NAVIGATE_POS_AND_VEL:
	{
		//导航模式下，机器人的位置和速度返回消息
		float x,y,z;
		recv_return_navigate_pos_and_vel_decode(dataFrame,x,y,z,linear_velocity_x_,linear_velocity_y_,angular_velocity_z_);
		linear_velocity_x_ = linear_velocity_x_ * linear_x_scale_;
    		linear_velocity_y_ = linear_velocity_y_ * linear_y_scale_;
    		angular_velocity_z_ = angular_velocity_z_*angle_z_scale_;
 		//std::printf("linear_velocity_x_1:%02f \n ",linear_velocity_x_ );
        	//std::printf("linear_velocity_y_1:%02f \n ",linear_velocity_y_ );
        	//std::printf("angular_velocity_z_1:%02f \n ",angular_velocity_z_ );
		break;
	}
	default:
		break;
	
    
    }
    dataFrame->resetDataFrame();
  }

#define sBUFFERSIZE 100 // send buffer size串口发送缓存长度
#define rBUFFERSIZE 256 // receive buffer size 串口接收缓存长度
unsigned char s_buffer[sBUFFERSIZE]; //发送缓存
unsigned char r_buffer[rBUFFERSIZE]; //接收缓存

int main(int argc, char** argv){
	ros::init(argc,argv,"vkhexbot_serial");
	VKHexbotSerial vkhexbotSerial;
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::Rate loop_rate(100);
	//接收数据长度
	int recLen = 0;
	while(ros::ok()){
		if(!vkhexbotSerial.getSerialStatus()){
			vkhexbotSerial.openSerial();
			continue;
		}
		memset(r_buffer,0,rBUFFERSIZE);
		recLen = 0;
		if ((recLen=vkhexbotSerial.readData(r_buffer))>0) {
			//处理接收数据
			//ROS_INFO_STREAM("read serial joy data: ");	
			//for(int i=0;i<recLen;i++){
	    		//	std::printf("%02x ",r_buffer[i] );
			//}
       			//std::printf("\n");
			unsigned char checksum = 0;
			byte messageType =0;
			byte messageId = 0;
			byte payloadLen = 0;
			HexbotProtocol recv_dataFrame;
			for(int i = 0;i < recLen; i++){       
				if(r_buffer[i] == VK_PROTOCOL_STX1 && r_buffer[i+1] == VK_PROTOCOL_STX2){       //检测数据头
					int payloadLen = r_buffer[i+4];
					for(int j = 0;j < (payloadLen+3); j++)
						checksum += r_buffer[i+2+j];
					checksum = ~checksum;                //计算校验值
					if(checksum == r_buffer[i+5+payloadLen]){     //判断校验位
						messageType = r_buffer[i+2];          
						recv_dataFrame.setMessageType((MESSAGE_TYPE)messageType);
						messageId = r_buffer[i+3];
						recv_dataFrame.setMessageId(messageId);
						recv_dataFrame.addPayload(&r_buffer[i+5],payloadLen);
						//解析返回数据
						vkhexbotSerial.recvDataProcess(&recv_dataFrame);     
					}
				}
			}
        	}
		vkhexbotSerial.publishOdometry();
		if (vkhexbotSerial.getMovingState() && (ros::WallTime::now() - vkhexbotSerial.getLastCommandTime() > ros::WallDuration(0.5)))
  		{
			ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);;
			geometry_msgs::Twist twist;
    			twist.linear.x = 0.0;
			twist.linear.y = 0.0;
			twist.angular.z = 0.0;
			twist_pub.publish(twist);  
			vkhexbotSerial.setMovingState(false);
  		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	
}
