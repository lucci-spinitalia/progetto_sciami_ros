/*------------------------------------------------------------------------------
 *  Title:        node_example_core.cpp
 *  Description:  Common class functions for example talker and listener nodes.
 *----------------------------------------------------------------------------*/

/*
 *
 *      Copyright (c) 2010 <iBotics -- www.sdibotics.org>
 *      All rights reserved.
 *
 *      Redistribution and use in source and binary forms, with or without
 *      modification, are permitted provided that the following conditions are
 *      met:
 *
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following disclaimer
 *        in the documentation and/or other materials provided with the
 *        distribution.
 *      * Neither the name of the Stingray, iBotics nor the names of its
 *        contributors may be used to endorse or promote products derived from
 *        this software without specific prior written permission.
 *
 *      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *      A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "Saetta_Base.h"

/*------------------------------------------------------------------------------
 * NodeExample()
 * Constructor.
 *----------------------------------------------------------------------------*/

Saetta_Base::Saetta_Base()
{
    this->_angular = 0;
    this->_linear = 0;
    this->node_handler=new ros::NodeHandle();
    this->pos_publisher = this->node_handler->advertise<nav_msgs::Odometry>("saetta/odom",1000);
} // end NodeExample()

/*------------------------------------------------------------------------------
 * ~NodeExample()
 * Destructor.
 *----------------------------------------------------------------------------*/

Saetta_Base::~Saetta_Base()
{
} // end ~NodeExample()
        
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
int counterr=0;
void Saetta_Base::processData(_local_msgtype & msg){
	
	this->_linear = 100.0 * msg->linear.x;
  	this->_angular = msg->angular.z;
   	//this->_uptodate = true;
}

/*------------------------------------------------------------------------------
 * publish_odom()
 * Publish the message.
 *----------------------------------------------------------------------------*/

void Saetta_Base::publish_odom()
{
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "odom";
  odom.pose.pose.position.x = this->x;
  odom.pose.pose.position.y = this->y;
  odom.pose.pose.position.z = 0.0;
  this->pos_publisher.publish(odom);
} 