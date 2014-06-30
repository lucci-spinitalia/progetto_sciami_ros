/**
 * 
 *
 */

#ifndef SAETTA_BASE_H
#define SAETTA_BASE_H

#include "ros/ros.h"
#include "ros/time.h"
#include <tf/transform_broadcaster.h> 
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "robot_core.h"


typedef const geometry_msgs::Twist::ConstPtr _local_msgtype;

class Saetta_Base
{
    public:
        //! Constructor.
        Saetta_Base();

        //! Destructor.
        ~Saetta_Base();
        
        // callback
        void listenerCallback (_local_msgtype & msg);
        
        void set_position(float x, float y, float th);
        
        void publish_odom();
        
        float x;
        
        float y;
        
        float th;  //verificare se serve
        
        // linear velocity
        float _linear;
        
        // angular velocity
        float _angular;

    private:
        
        ros::Publisher pos_publisher;
        
        ros::NodeHandle *node_handler;
        
        void processData(_local_msgtype & msg);
   
};


#endif // SR_NODE_EXAMPLE_CORE_H
