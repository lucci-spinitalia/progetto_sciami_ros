#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

   
    


class TeleopSaetta
{
public:
  TeleopSaetta();


private:

  
  ros::NodeHandle nh_;
  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
};

TeleopSaetta::TeleopSaetta():
  linear_(1),
  angular_(2),
  l_scale_(2.0),
  a_scale_(2.0)
{
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

   joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopSaetta::joyCallback, this);
   vel_pub_ =  nh_.advertise<geometry_msgs::Twist>("/saetta/velocity",1);  
}


void TeleopSaetta::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;
  vel.angular.z = a_scale_*joy->axes[angular_];
  vel.linear.x = l_scale_*joy->axes[linear_];
  vel_pub_.publish(vel);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_saetta");
  TeleopSaetta teleop_saetta;
  ros::NodeHandle n;

  int rate = 4;
  ros::Rate r(rate);
  while (n.ok())
    {
        ros::spinOnce();
        r.sleep();

    }
  
  
  return(0);
}






