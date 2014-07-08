#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

   
    


class TeleopSaetta
{
public:
  TeleopSaetta();
  int linear_, angular_;
  double l_scale_, a_scale_;

private:
  ros::NodeHandle nh_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
};

TeleopSaetta::TeleopSaetta()
{
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
  TeleopSaetta* teleop_saetta = new TeleopSaetta;
  ros::NodeHandle n("~");
  n.param<int>("axis_linear", teleop_saetta->linear_, 1);
  n.param<int>("axis_angular", teleop_saetta->angular_, 0);
  n.param<double>("scale_angular", teleop_saetta->a_scale_, 0.5);
  n.param<double>("scale_linear", teleop_saetta->l_scale_, 0.1);
  int rate = 4;
  ros::Rate r(rate);
  while (n.ok())
    {
        ros::spinOnce();
        r.sleep();

    }
  
  
  return(0);
}






