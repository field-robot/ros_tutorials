#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "std_msgs/UInt8.h" // Publiser voor movement_controller
#include "std_msgs/Int8.h"	// Publiser voor movement_controller
#include "std_msgs/Bool.h"

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
//#define KEYCODE_Q 0x71
#define KEYCODE_S 0x20

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
  ros::Publisher chatter_dir_L;
  ros::Publisher chatter_dir_R; 
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
  chatter_dir_L = nh_.advertise<std_msgs::Bool>("dir_L", 1000);
  chatter_dir_R = nh_.advertise<std_msgs::Bool>("dir_R", 1000);

  ros::Rate loop_rate(100);
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
  int calc_speed_LF = 0;
  int calc_speed_RF = 0;
  int set_speed_LF = 0;
  int set_speed_RF = 0;
  int send_speed_LF = 0;
  int send_speed_RF = 0;
  bool dir_L = false;
  bool dir_R = false;
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
    
	//angular_ = 0.0;
	//linear_ = 0.0;

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
      case KEYCODE_S:
        ROS_DEBUG("STOP");
        linear_ = 5;
        dirty = true;
        break;
	  //default:
		//ROS_DEBUG("STOP");
		//angular_ = 0.0;
		//linear_ = 0.0;
		//break;
			
    }
	int linear_offset = static_cast<int>(linear_); 			// van double naar int
  	int angular_offset = static_cast<int>(angular_);	 	// van double naar int




/////////////////////////////// CALCULATIONS //////////////////////////////////////////


	if (angular_ == 1.0) {
		set_speed_LF = 50;
		set_speed_RF = 250;
	} else if (angular_ == -1.0) {
		set_speed_LF = 250;
		set_speed_RF = 50;
	} else if (linear_ == 1.0) {
		set_speed_LF = 250;
		set_speed_RF = 250;
	} else if (linear_ == -1.0) {
        set_speed_LF = -250;
		set_speed_RF = -250;
    } else if (linear_ == 5.0) {
        set_speed_LF = 0;
        set_speed_RF = 0;
       // calc_speed_LF = 0;
       // calc_speed_RF = 0;
       // dir_L = true;
       // dir_R = true;
    }

    if (set_speed_LF != 0){
        if (calc_speed_LF > set_speed_LF){
            calc_speed_LF--;
        }

        if (calc_speed_LF < set_speed_LF){
            calc_speed_LF++;
        }

        if (calc_speed_RF > set_speed_RF){
            calc_speed_RF--;
        }

        if (calc_speed_RF < set_speed_RF){
            calc_speed_RF++;
        }
    } else {
        if (calc_speed_LF > set_speed_LF){
            calc_speed_LF = calc_speed_LF-15;
        }

        if (calc_speed_LF < set_speed_LF){
            calc_speed_LF = calc_speed_LF+15;
        }

        if (calc_speed_RF > set_speed_RF){
            calc_speed_RF = calc_speed_RF-15;
        }

        if (calc_speed_RF < set_speed_RF){
            calc_speed_RF = calc_speed_RF+15;
        }
    }

	if (calc_speed_LF >= 50){
		send_speed_LF = calc_speed_LF;
		dir_L = true;
	} else if (calc_speed_LF <= -50){
		send_speed_LF = abs(calc_speed_LF);
		dir_L = false;
	} else {
		send_speed_LF = 0;
	}
	
	if (calc_speed_RF >= 50){
		send_speed_RF = calc_speed_RF;
		dir_R = true;
	} else if (calc_speed_LF <= -50){
		send_speed_RF = abs(calc_speed_RF);
		dir_R = false;
	} else {
		send_speed_RF = 0;
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
	  std_msgs::Bool msg_dir_L;
	  std_msgs::Bool msg_dir_R;

	  msg_speed_LF.data = send_speed_LF;
	  msg_speed_RF.data = send_speed_RF;
	  msg_dir_L.data = dir_L;
	  msg_dir_R.data = dir_R;

	  chatter_linear_offset.publish(msg_speed_LF);
	  chatter_angular_offset.publish(msg_speed_RF);
	  chatter_dir_L.publish(msg_dir_L);
	  chatter_dir_R.publish(msg_dir_R);

      dirty=false;
    }
  }

 
  return;
}



