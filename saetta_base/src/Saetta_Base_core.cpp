/*------------------------------------------------------------------------------
 * 
 */

#include "Saetta_Base.h"

/*------------------------------------------------------------------------------
 * Constructor.
 *----------------------------------------------------------------------------*/

Saetta_Base::Saetta_Base()
{
    this->_angular = 0;
    this->_linear = 0;
    this->node_handler=new ros::NodeHandle();
    this->pos_publisher = this->node_handler->advertise<nav_msgs::Odometry>("saetta/odom",1000);
} 

/*------------------------------------------------------------------------------
 * Destructor.
 *----------------------------------------------------------------------------*/

Saetta_Base::~Saetta_Base()
{
} 
        
/*------------------------------------------------------------------------------
 * listenerCallback()
 * Receive twist message.
 *----------------------------------------------------------------------------*/

void Saetta_Base::set_position(float x, float y, float th)
{
    this->x = x;
    this->y = y;
    this->th = th;
}


void Saetta_Base::listenerCallback(_local_msgtype & msg)
{
   // ROS_INFO("Received linear: [%f], angular: [%f]\n", msg->linear, msg->angular);
    this->processData(msg);
}


void Saetta_Base::processData(_local_msgtype & msg)
{	
    this->_linear = 100.0 * msg->linear.x;
    this->_angular = msg->angular.z;
}

/*------------------------------------------------------------------------------
 * publish_odom()
 * Publish the message.
 *----------------------------------------------------------------------------*/

void Saetta_Base::publish_odom()
{
    tf::TransformBroadcaster odom_broadcaster;
    ros::Time current_time = ros::Time::now();
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(this->th);

    //publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = this->x;
    odom_trans.transform.translation.y = this->y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = this->x;
    odom.pose.pose.position.y = this->y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = (this->_linear)/100;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = this->_angular;

    //publish the odometry
    this->pos_publisher.publish(odom);
} 