#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "std_msgs/UInt8.h" // Publiser voor movement_controller
#include "std_msgs/Int8.h"	// Publiser voor movement_controller

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

class TeleopTurtle
{
public:
  TeleopTurtle();
  void keyLoop();
  
private:

  
  ros::NodeHandle nh_;
  double linear_, angular_, l_scale_, a_scale_;
  ros::Publisher twist_pub_;
  ros::Publisher chatter_linear_offset;
  ros::Publisher chatter_angular_offset; 
};




TeleopTurtle::TeleopTurtle():
  linear_(0),
  angular_(0),
  l_scale_(2.0),
  a_scale_(2.0)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

// publisher for movement_controller
  chatter_linear_offset = nh_.advertise<std_msgs::UInt8>("key_speed_LF", 1000);
  chatter_angular_offset = nh_.advertise<std_msgs::UInt8>("key_speed_RF", 1000);

  ros::Rate loop_rate(10);
// end 
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;



  signal(SIGINT,quit);

  teleop_turtle.keyLoop();
  
  return(0);
}


void TeleopTurtle::keyLoop()
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
  puts("---------------------------");
  puts("Use arrow keys to move the turtle.");

  int linear_offset = 0;  // variable om int te maken
  int angular_offset = 0; // variable om int te maken

  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    linear_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        angular_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        angular_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        linear_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        linear_ = -1.0;
        dirty = true;
        break;
    }
	int linear_offset = static_cast<int>(linear_); 			// van double naar int
  	int angular_offset = static_cast<int>(angular_);	 	// van double naar int




/////////////////////////////// CALCULATIONS //////////////////////////////////////////
	int calc_speed_LF = 0;
	int calc_speed_RF = 0;

	if (angular_ == 1.0) {
		calc_speed_LF = 80;
		calc_speed_RF = 255;
	} else if (angular_ == -1.0) {
		calc_speed_LF = 255;
		calc_speed_RF = 80;
	} else if (linear_ == 1.0) {
		calc_speed_LF = 255;
		calc_speed_RF = 255;
	} else {
		calc_speed_LF = 0;
		calc_speed_RF = 0;
	}


/////////////////////////////// END CALCULATIONS //////////////////////////////////////////
	
	
    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_*angular_;
    twist.linear.x = l_scale_*linear_;
    
    if(dirty ==true)
    {
      twist_pub_.publish(twist);

	  std_msgs::UInt8 msg_speed_LF;
	  std_msgs::UInt8 msg_speed_RF;

	  msg_speed_LF.data = calc_speed_LF;
	  msg_speed_RF.data = calc_speed_RF;

	  chatter_linear_offset.publish(msg_speed_LF);
	  chatter_angular_offset.publish(msg_speed_RF);
      dirty=false;
    }
  }

 
  return;
}



