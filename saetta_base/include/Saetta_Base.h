/**
<<<<<<< HEAD
 * 
=======
 * controllare gli include
>>>>>>> fdf1fb0d09c03c40bc715d7bf1694521c49c606f
 *
 */

#ifndef SAETTA_BASE_H
#define SAETTA_BASE_H
<<<<<<< HEAD

#include "ros/ros.h"
#include "ros/time.h"
//#include <tf/transform_broadcaster.h> //inserire nel caso di trasformazione odom
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "robot_core.h"


typedef const geometry_msgs::Twist::ConstPtr _local_msgtype;
=======
#include <cstdlib>
#include <iostream>
#include <utility>
#include <sstream>
#include <thread>
#include <mutex>
#include <list>

#include "ros/ros.h"
#include "ros/time.h"
#include <tf/transform_broadcaster.h>
#include <termios.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"


#include <signal.h>
#include "robot_core.h"
#include "pic2netus.h"
#include "pic_rel.h"
#include "robot_sensors.h"
#include "robot_comm.h"
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

typedef const geometry_msgs::Twist::ConstPtr _local_msgtype; //inutile
>>>>>>> fdf1fb0d09c03c40bc715d7bf1694521c49c606f

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
