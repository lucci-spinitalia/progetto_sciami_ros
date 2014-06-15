#include <ros/ros.h>
#include <saetta_msgs/cmd_vel.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "joystick.h"
#include <geometry_msgs/Twist.h>

    
#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
    


class TeleopSaetta
{
public:
  TeleopSaetta();
  void keyLoop();

private:

  
  ros::NodeHandle nh_;
  double linear_, angular_, l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  
};

TeleopSaetta::TeleopSaetta():
  linear_(0),
  angular_(0),
  l_scale_(2.0),
  a_scale_(2.0)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  //vel_pub_ = nh_.advertise<saetta_msgs::cmd_vel>("/saetta/velocity", 1);
   vel_pub_ =  nh_.advertise<geometry_msgs::Twist>("/saetta/velocity",1);  
}



void quit(int sig)
{

  ros::shutdown();
  exit(0);
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_saetta");
  TeleopSaetta teleop_turtle;
  initJoy();
  ros::NodeHandle n;

  int rate = 4;
  ros::Rate r(rate);
  while (n.ok())
    {
        ros::spinOnce();
        teleop_turtle.keyLoop();
        r.sleep();

    }
  
  
  return(0);
}


void TeleopSaetta::keyLoop()
{
        //saetta_msgs::cmd_vel vel;
	geometry_msgs::Twist vel;
        JoystickLoop();
	command_turn = copysign(pow(command_turn,2),command_turn);
	command_vel = copysign(pow(command_vel,2),command_vel);
        /*vel.angular=2*command_turn;
        vel.linear=10*command_vel;
        if (vel.linear >= -0.3 && vel.linear <= 0.3)
            vel.linear = 0.0;
        if (vel.angular >= -0.3 && vel.angular <= 0.3)
            vel.angular = 0.0;
        
        vel_pub_.publish(vel);
        ROS_INFO("JoyValue: %3.2f  %3.2f",vel.linear,vel.angular);*/
	vel.angular.x=0;    	
	vel.angular.y=0;
	vel.angular.z=2*command_turn;
        vel.linear.x=10*command_vel;
	vel.linear.y=0;
	vel.linear.z=0;
        if (vel.linear.x >= -0.3 && vel.linear.x <= 0.3)
            vel.linear.x = 0.0;
        if (vel.angular.z >= -0.3 && vel.angular.z <= 0.3)
            vel.angular.z = 0.0;
        
        vel_pub_.publish(vel);
        ROS_INFO("JoyValue: %3.2f  %3.2f",vel.linear.x,vel.angular.z);
 }





