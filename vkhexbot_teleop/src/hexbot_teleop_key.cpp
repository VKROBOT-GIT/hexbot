#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <sys/poll.h>
#include <boost/thread/thread.hpp>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

class HexbotTeleopKey
{
public:
  HexbotTeleopKey();
  void keyLoop();
  void stopRobot();
       
private:

  ros::NodeHandle nh_;
  double linear_, angular_, l_scale_, a_scale_;
  ros::Publisher twist_pub_;
  
};

HexbotTeleopKey::HexbotTeleopKey():
  linear_(0),
  angular_(0),
  l_scale_(2.0),
  a_scale_(2.0)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  HexbotTeleopKey hexbot_teleop_key;
  /* 创建一个新的线程 */
  boost::thread t = boost::thread(boost::bind(&HexbotTeleopKey::keyLoop, &hexbot_teleop_key));
  ros::spin(); 
  t.interrupt();
  t.join();
  hexbot_teleop_key.stopRobot();
  /* 设置终端参数 */
  tcsetattr(kfd, TCSANOW, &cooked);
    
  return(0);

}

void HexbotTeleopKey::stopRobot()
{
	geometry_msgs::Twist twist;
    twist.angular.z = 0;
    twist.linear.x = 0;
    twist_pub_.publish(twist);  
}

void HexbotTeleopKey::keyLoop()
{
  char c;
  bool dirty=false;

  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("-----------------------------");
  puts("Use arrow keys to move the hexbot robot. 'q' to quit");

  
  struct pollfd ufd;
  ufd.fd = kfd;
  ufd.events = POLLIN;

  for(;;)
  {
  
    boost::this_thread::interruption_point();
    int num;
    // get the next event from the keyboard 
    if ((num = poll(&ufd, 1, 250)) < 0)
    {
        perror("poll():");
        return;
    }
    else if(num > 0)
    {
	if(read(kfd, &c, 1) < 0)
	{
	    perror("read():");
	    exit(-1);
	}
    }
    else
    {
	/* 每按下一次动一下 */
        if (dirty == true)
        {
            stopRobot();
            dirty = false;
        }          
        continue;
    }
    
    geometry_msgs::Twist twist;
    twist.angular.z = 0;
    twist.linear.x = 0;
    ROS_DEBUG("value: 0x%02X\n", c);
    switch(c)
    {
      case KEYCODE_L:
        angular_ = 1.0;
        linear_ = 0.0;
        twist.angular.z = a_scale_*angular_;
    	twist.linear.x = l_scale_*linear_;
    	twist_pub_.publish(twist);  
        dirty = true;
        break;
      case KEYCODE_R:
        angular_ = -1.0;
        linear_ = 0.0;
        twist.angular.z = a_scale_*angular_;
    	twist.linear.x = l_scale_*linear_;
    	twist_pub_.publish(twist);  
        dirty = true;
        break;
      case KEYCODE_U:
 	angular_ = 0.0;
        linear_ = 1.0;
        twist.angular.z = a_scale_*angular_;
    	twist.linear.x = l_scale_*linear_;
    	twist_pub_.publish(twist);  
        dirty = true;
        break;
      case KEYCODE_D:
 	angular_ = 0.0;
        linear_ = -1.0;
        twist.angular.z = a_scale_*angular_;
    	twist.linear.x = l_scale_*linear_;
    	twist_pub_.publish(twist);  
        dirty = true;
        break;
      case KEYCODE_Q:
	stopRobot();
	quit(0);
        break;
      default:
   
        break;
    }  
   
  }
}




