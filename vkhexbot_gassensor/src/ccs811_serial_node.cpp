#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <vkhexbot_msgs/Gassensor.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <string>

#define sBUFFERSIZE 100 // send buffer size串口发送缓存长度
#define rBUFFERSIZE 2048 // receive buffer size 串口接收缓存长度

#define GAS_PROTOCOL_COO_HEAD "CO2 = "  //CCS811 CO2协议头
#define GAS_PROTOCOL_TVOS_HEAD "TVOS = "  //CCS811 TVOS协议头

#define MPU_PACKET_LENGTH 11
#define MPU_FRAME_LENGTH 33


int main(int argc, char** argv)
{
  serial::Serial ser;
  std::string port;
  int baudrate;
  unsigned char r_buffer[rBUFFERSIZE]; //接收缓存

  ros::init(argc, argv, "ccs811_serial_node");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle private_node_handle("~");
  private_node_handle.param<std::string>("port", port, "/dev/vk_gassensor");
  private_node_handle.param<int>("baudrate", baudrate, 115200);

  ros::NodeHandle nh("");
  ros::Publisher gassensor_pub = nh.advertise<vkhexbot_msgs::Gassensor>("gassensor_data", 50);

  ros::Rate r(200); // 100 hz

  vkhexbot_msgs::Gassensor gassensor;

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

  while(ros::ok())
  {
    try
    {
        // read string from serial device
        if(ser.available())
        {
			
            unsigned char *p;
            int co2 = 0 ,tvos = 0;			
            size_t bytes_read = ser.read(r_buffer, ser.available());
            p=(unsigned char*)strstr((char*)r_buffer,"CO2 = ")+6;
            while(*p!='p')
            {
                if(*p>='0'&&*p<='9'){
                    co2=co2*10+(*p-'0');
                }
                p++;
        
            }

            p=(unsigned char*)strstr((char*)r_buffer,"TVOS = ")+7;
            while(*p!='p')
            {
                if(*p>='0'&&*p<='9') {
                    tvos=tvos*10+(*p-'0');              
                }
                p++;        
            }
            if(strstr((char*)r_buffer,"CO2 = ")!=NULL && strstr((char*)r_buffer,"TVOS = ")!=NULL)
            {
                gassensor.co2 = co2;
                gassensor.tvos = tvos;
                gassensor_pub.publish(gassensor);  
                //ROS_INFO_STREAM("CO2:%d " << co2 << " ppm  TVOS:%d "<< tvos << " ppb\r\n ");
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
