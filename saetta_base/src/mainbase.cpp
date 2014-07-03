
/* 
 * File:   mainbase.cpp
 * Author: Massimo Cardellicchio
 *
 * Created on June 9, 2014, 10:45 PM
 */

#include <signal.h>
#include "Saetta_Base.h"
#include "Rs232/rs232.h"

using namespace std;

float* robot_state;
  
void setup_termination();
void termination_handler(int signum);


void setup_termination() 
{
    if (signal(SIGINT, termination_handler) == SIG_IGN) signal(SIGINT, SIG_IGN);
    if (signal(SIGHUP, termination_handler) == SIG_IGN) signal(SIGHUP, SIG_IGN);
    if (signal(SIGTERM, termination_handler) == SIG_IGN) signal(SIGTERM, SIG_IGN);
}

void termination_handler(int signum) 
{
  close_robot();

  if(robot_state > 0)
    free(robot_state);
    
  exit(0);
}

/*------------------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *----------------------------------------------------------------------------*/
int main(int argc, char** argv)
{
  robot_state = (float *) malloc(sizeof (float) *3);

  // Set up ROS.
  ros::init(argc, argv, "Saetta_Base");
  ros::NodeHandle n("~");
  Saetta_Base* localbase = new Saetta_Base();

  // set velocity subscriber
  ros::Subscriber sub = n.subscribe("/saetta/velocity", 10, &Saetta_Base::listenerCallback, localbase);

  int period_us;
  n.param<int>("period", period_us, "20000");
  ROS_INFO_STREAM("ROS publish refresh: " << period_us << " us");

  std::string nPort; //default port name
  n.param<std::string>("port", nPort, "/dev/ttyO3");
  ROS_INFO_STREAM("ROS parameter 'port' setted as: " << nPort);

  // convert string to char*
  char* nPort_char = new char[nPort.size() + 1];
  std::copy(nPort.begin(), nPort.end(), nPort_char);
  nPort_char[nPort.size()] = '\0'; // terminating char* with 0

  // print the serial port
  printf("Starting serial communication on serial port %s . . . ", nPort_char);

  if(init_robot(nPort_char) < 0)
  {
    printf("communication failed\n");
    return -1;
  }
  else
    printf("communication started!\n");

  delete[] nPort_char; //free memory

  setup_termination();

  while(n.ok())
  {
    if(robot_loop(period_us))
    {
      ros::spinOnce();
      set_robot_speed(&(localbase->_linear),&(localbase->_angular));
    
      get_robot_state(&robot_state);
      
      //the robot states x and y are in [cm], we publish them in [m]
      localbase->set_position(robot_state[0]/100, robot_state[1]/100, robot_state[2]); 
      localbase->publish_odom();
    }
  }
  
  return (EXIT_SUCCESS);
}
