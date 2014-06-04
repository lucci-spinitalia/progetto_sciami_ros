/**
 *  \file node_example_core.h
 *  \brief Common class functions for example talker and listener nodes.
 */

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

#ifndef SAETTA_BASE_H
#define SAETTA_BASE_H
#include <cstdlib>
#include <iostream>
#include <utility>
#include <sstream>
#include <thread>
#include <mutex>
#include <list>

//#include "serverWIFI.h"
// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include <tf/transform_broadcaster.h>
#include <saetta_msgs/cmd_vel.h>
#include <termios.h>
#include <errno.h>
#include <unistd.h>
#include <ncurses.h>
#include <string.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>

volatile sig_atomic_t s_interrupted = 0;
volatile sig_atomic_t alarm_expired = 0;


static void s_signal_handler (int signal_value)
{
	if (signal_value == SIGTERM || signal_value == SIGSTOP || signal_value == SIGINT)
		s_interrupted = 1;
	if (signal_value == SIGALRM)
		//signal(SIGALRM,alarm_wakeup);
		alarm_expired = 1;
}

static void s_catch_signals (void)
{
	struct sigaction action;
	action.sa_handler = s_signal_handler;
	action.sa_flags = 0;
	sigemptyset (&action.sa_mask);
	sigaction (SIGINT, &action, NULL);
	sigaction (SIGTERM, &action, NULL);
	sigaction (SIGALRM, &action, NULL);
}

// Custom message includes. Auto-generated from msg/ directory.
//#include <Saetta_Base/Saetta_Base_data.h>

// Dynamic reconfigure includes.
//#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
//#include <Saetta_Base/Saetta_Base_paramsConfig.h>
#include <signal.h>
#include "robot_core.h"
#include "pic2netus.h"
#include "pic_rel.h"
#include "robot_sensors.h"
#include "robot_comm.h"
#include <ncurses.h>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

//typedef const saetta_msgs::cmd_vel::ConstPtr _local_msgtype;
typedef const geometry_msgs::Twist::ConstPtr _local_msgtype;

using std::string;

class Saetta_Base
{
    public:
        //! Constructor.
        Saetta_Base();

        //! Destructor.
        ~Saetta_Base();

        //! Callback function for dynamic reconfigure server.
        //void configCallback(Saetta_Base::Saetta_Base_paramsConfig &config, uint32_t level);

        //! Listener callback function.
        //void listenerCallback (const turtlesim::Velocity::ConstPtr & msg);
        
        void listenerCallback (_local_msgtype & msg);
        
        //! Uptodate property
        //bool IsUpToDate(void);
        
        //void InitWifi(char *ip);
        
        //void SendWifiSpeed(void);
        
        //! Publish the message.
        //void publishMessage(ros::Publisher *pub_message);

        //! Callback function for subscriber.
        //void messageCallback(const Saetta_Base::Saetta_Base_data::ConstPtr &msg);

        //! The actual message.
        string message;

        //! The first integer to use in addition.
        int a;

        //! The second integer to use in addition.
        int b;
        
         float _linear;
        float _angular;

    private:
        void processData(_local_msgtype & msg);
        //SaettaServerWifi::WifiTx _localwifihndl;
        int _socket;
   
        bool _uptodate;
        bool _init;
};

/*#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <pthread.h>
#include <errno.h>
#include <sys/select.h>
#include <sys/time.h>		// for the gettimeofday
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
*/



unsigned char 	c;

char 		file_log[100]="/home/panda/.ros/saetta_base/acc.txt";
float		*robot_state=NULL;
struct timeval  *tempi=NULL;
//===================
float w_xbee=0,v_xbee=0;

#ifdef DEBUG_TIMING
	int debug_fd;
#endif
pthread_t	thread_main;
pthread_t	thread_pic;
pthread_cond_t  cond;
pthread_mutex_t  tmutex;
int log_fd;
int pic_log_fd;

int
set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        /*tty.c_iflag &= ~IGNBRK;         // ignore break signal
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;*/

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}
pthread_t	thread_wifi;
//struct payload_pkg_trajectory pay;
int file_exists(const char * filename);
void setup_termination();
void termination_handler(int signum);
void main_init();
void* tf_pic2netus(void *args);

#endif // SR_NODE_EXAMPLE_CORE_H
