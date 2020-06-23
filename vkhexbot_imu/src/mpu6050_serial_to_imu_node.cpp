#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#define sBUFFERSIZE 100 // send buffer size串口发送缓存长度
#define rBUFFERSIZE 2048 // receive buffer size 串口接收缓存长度

#define MPU_PROTOCOL_HEAD 0X55  //mpu6050协议头
#define ACC_MSA 0X51 //加速度包消息ID
#define ANLGE_VEL_MSA 0X52 //角速度包消息ID
#define ANGLE_MSA 0X53 //角度包消息ID

#define MPU_PACKET_LENGTH 11
#define MPU_FRAME_LENGTH 33

#define pi 3.1415926
#define angleToRaduis pi/180


uint8_t CalcSum(uint8_t* buf, uint8_t len) {
    uint8_t sum = 0;
    uint8_t i = 0;
    for(; i < len; i++) {

        sum += buf[i];
    }
    return sum;
}


int main(int argc, char** argv)
{
  serial::Serial ser;
  std::string port;
  int baudrate;
  std::string frame_id;
  double linear_acceleration_stddev;
  double angular_velocity_stddev;
  double orientation_stddev;
  int remainLen = 0;
  unsigned char r_buffer[rBUFFERSIZE]; //接收缓存

  ros::init(argc, argv, "mpu6050_serial_to_imu_node");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle private_node_handle("~");
  private_node_handle.param<std::string>("port", port, "/dev/vk_imu");
  private_node_handle.param<int>("baudrate", baudrate, 115200);
  private_node_handle.param<std::string>("frame_id", frame_id, "imu_link");
  private_node_handle.param<double>("linear_acceleration_stddev", linear_acceleration_stddev, 0.0);
  private_node_handle.param<double>("angular_velocity_stddev", angular_velocity_stddev, 0.0);
  private_node_handle.param<double>("orientation_stddev", orientation_stddev, 0.0);

  ros::NodeHandle nh("");
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 50);

  ros::Rate r(200); // 100 hz

  sensor_msgs::Imu imu;

  imu.linear_acceleration_covariance[0] = linear_acceleration_stddev;
  imu.linear_acceleration_covariance[4] = linear_acceleration_stddev;
  imu.linear_acceleration_covariance[8] = linear_acceleration_stddev;

  imu.angular_velocity_covariance[0] = angular_velocity_stddev;
  imu.angular_velocity_covariance[4] = angular_velocity_stddev;
  imu.angular_velocity_covariance[8] = angular_velocity_stddev;

  imu.orientation_covariance[0] = orientation_stddev;
  imu.orientation_covariance[4] = orientation_stddev;
  imu.orientation_covariance[8] = orientation_stddev;


  bool receiveAcc = false;
  bool receiveAngelVel = false;
  bool receiveAngle = false;

  // try and open the serial port
  try
  {
    ser.setPort(port);
    ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    ser.open();
  }
  catch (serial::IOException& e)
  {
     ROS_ERROR_STREAM("Unable to open serial port " << ser.getPort() << ". Trying again in 5 seconds.");
     return 0;
  }

  if(ser.isOpen())
  {
       ROS_INFO_STREAM("Serial port " << ser.getPort() << " initialized and opened.");
  }else{

  }

  unsigned char s_buffer_zZero[3] = {0xFF,0XAA,0X52};
  unsigned char s_buffer_accCalib[3] = {0xFF,0XAA,0X67};

  ser.write(s_buffer_zZero,3);
  ROS_INFO_STREAM("wait 5s for imu z axis initialized...");
  ros::Duration(5).sleep();

  ser.write(s_buffer_accCalib,3);
  ROS_INFO_STREAM("wait 5s for imu acc calibrate...");
  ros::Duration(5).sleep();
		
  ROS_INFO_STREAM("IMU initialized finish, publish imu data...");

  ser.flushInput();
  while(ros::ok())
  {
    try
    {
        // read string from serial device
        if(ser.available())
        {
			
			size_t bytes_read = ser.read(r_buffer+remainLen, ser.available());
			
			/*for(int j=0;j<bytes_read;j++){
				printf("%02X ",r_buffer[j]);
			}*/
			bytes_read = bytes_read + remainLen;
            remainLen = 0;
			double xLinearAcc = 0.0, yLinearAcc = 0.0, zLinearAcc = 0.0;
			double xAngleVel = 0.0, yAngleVel = 0.0, zAngleVel = 0.0;
			double roll = 0.0, pitch = 0.0, yaw = 0.0;

			/*if(bytes_read < MPU_FRAME_LENGTH){
				//消息包太短，等待下次数据一起处理
				remainLen = bytes_read;
			    continue;
			}*/

			for(int i = 0; i < bytes_read; ) {
				if(r_buffer[i] == MPU_PROTOCOL_HEAD) {
					if((bytes_read-i)<MPU_PACKET_LENGTH){
						remainLen = bytes_read-i;
						for(int k=0;k<remainLen;k++){
							r_buffer[k] = r_buffer[bytes_read-remainLen+k];
						}
						break;
					}
						
					if(!(r_buffer[i+1] == ACC_MSA || r_buffer[i+1] == ANLGE_VEL_MSA || r_buffer[i+1] == ANGLE_MSA)){
						i++;
						continue;
					}
           	 		switch(r_buffer[i + 1]) {
            		case ACC_MSA:
				        if(r_buffer[i + MPU_PACKET_LENGTH - 1] == CalcSum(&r_buffer[i], MPU_PACKET_LENGTH - 1)) {
				            xLinearAcc = ((short)(r_buffer[i + 3] << 8 | r_buffer[i + 2])) / 32768.0 * 16; //X轴加速度（m/s2）
				            yLinearAcc = ((short)(r_buffer[i + 5] << 8 | r_buffer[i + 4])) / 32768.0 * 16; //Y轴加速度（m/s2）
				            zLinearAcc = ((short)(r_buffer[i + 7] << 8 | r_buffer[i + 6])) / 32768.0 * 16; //Z轴加速度（m/s2）
				            i += MPU_PACKET_LENGTH;
							receiveAcc = true;

				        }else{
							i++;
							ROS_INFO_STREAM("data error 111...");
						}
				        break;
				    case ANLGE_VEL_MSA:
				        if(r_buffer[i + MPU_PACKET_LENGTH - 1] == CalcSum(&r_buffer[i], MPU_PACKET_LENGTH - 1)) {
				            xAngleVel = ((short)(r_buffer[i + 3] << 8 | r_buffer[i + 2])) / 32768.0 * 2000*angleToRaduis; //X轴角速度(弧度/s)
				            yAngleVel = ((short)(r_buffer[i + 5] << 8 | r_buffer[i + 4])) / 32768.0 * 2000*angleToRaduis; //Y轴角速度(弧度/s)
				            zAngleVel = ((short)(r_buffer[i + 7] << 8 | r_buffer[i + 6])) / 32768.0 * 2000*angleToRaduis; //Z轴角速度(弧度/s)
				            i += MPU_PACKET_LENGTH;
							receiveAngelVel = true;
				        }else{
							i++;
							ROS_INFO_STREAM("data error 222...");
						}
				        break;
				    case ANGLE_MSA:
				        if(r_buffer[i + MPU_PACKET_LENGTH - 1] == CalcSum(&r_buffer[i], MPU_PACKET_LENGTH - 1)) {
				            roll = ((short)(r_buffer[i + 3] << 8 | r_buffer[i + 2])) / 32768.0*180*angleToRaduis; //X轴滚转角（x 轴）(弧度)
				            pitch = ((short)(r_buffer[i + 5] << 8 | r_buffer[i + 4])) / 32768.0*180*angleToRaduis ; //Y轴俯仰角（y 轴）(弧度)
				            yaw = ((short)(r_buffer[i + 7] << 8 | r_buffer[i + 6])) / 32768.0*180*angleToRaduis ; //Z轴偏航角（z 轴）(弧度)
				            i += MPU_PACKET_LENGTH;
							receiveAngle = true;
				        }
						else{
							i++;
							ROS_INFO_STREAM("data error 333...");
						}
				        break;
				    default:
				        i++;
				        break;
				    }

				} else {
					int tt = i;
					int value = r_buffer[i];
					ROS_INFO("data error...,i=%d, data= %02x",tt,value);
				    i++;
					//ROS_INFO_STREAM("data error...");
					
				}				
			}
			// calculate measurement time
			if(receiveAcc && receiveAngelVel && receiveAngle){
				//printf("RPY:%02f, %02f, %02f \n",roll,pitch,yaw);
				ros::Time measurement_time = ros::Time::now();	
				geometry_msgs::Quaternion imu_quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
				// publish imu message
				imu.header.stamp = measurement_time;
				imu.header.frame_id = frame_id;
				imu.orientation = imu_quat;
			
				imu.linear_acceleration.x = xLinearAcc;
				imu.linear_acceleration.y = yLinearAcc;
				imu.linear_acceleration.z = zLinearAcc;
			
				imu.angular_velocity.x = xAngleVel;
				imu.angular_velocity.y = yAngleVel;
				imu.angular_velocity.z = zAngleVel;

				imu_pub.publish(imu);
				receiveAcc = false;
				receiveAngelVel = false;
				receiveAngle = false;
				//ROS_INFO_STREAM(" publish imu data...");
			}
		}
    }
    catch (serial::IOException& e)
    {
      ROS_ERROR_STREAM("Error reading from the serial port " << ser.getPort() << ". Closing connection.");
      ser.close();
    }
    catch(std::exception e){
		ROS_ERROR_STREAM("error..");
	}
	ros::spinOnce();
	r.sleep();
  }
}
