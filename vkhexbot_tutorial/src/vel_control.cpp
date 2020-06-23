#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main (int argc, char **argv) 
{
  ros::init(argc, argv, "vel_control") ;
  ros::NodeHandle nh;
  ros::Publisher twist_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  ros::Rate loop_rate(10);
  int count = 0;
  int ucount = 0;
  ROS_INFO_STREAM("hexbot is moving...!") ;
  while(ros::ok())
    {
	geometry_msgs::Twist twist;
	if(count<300)
	{
    	    twist.linear.x = 0.05;
	    twist.linear.y = 0;
	    twist.linear.z = 0;
	    twist.angular.x = 0;
	    twist.angular.y = 0;
	    twist.angular.z = 0;
	}else{
	    twist.linear.x = 0;
	    twist.linear.y = 0;
	    twist.linear.z = 0;
	    twist.angular.x = 0;
	    twist.angular.y = 0;
	    twist.angular.z = 0;
	    ucount++;
            if(ucount>3){ 
 	        ROS_INFO_STREAM("hexbot is stop...!") ;
		break;
	    }
	   
	}
	twist_pub_.publish(twist);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
      
    }
    return 0;
}
