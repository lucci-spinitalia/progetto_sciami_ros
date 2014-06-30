
/* 
 * File:   mainbase.cpp
 * Author: Massimo Cardellicchio
 *
 * Created on June 9, 2014, 10:45 PM
 */

/* LISTA VARIABILI GLOBALI PRESE DALLA LIBRERIA C
 * 
 * da netus2pic/serial_comm.h :
 * flag
 * PACKET_TIMING_LENGTH
 * 
 * da netus2pic/robot_comm.h :
 * LEN_PIC_BUFFER
 * pic_message_reset_steps_acc
 * 
 * da core/pic2netus.h
 * LOAD_PACKET_ANALYZED = 2
 
 */
#include "Saetta_Base.h"

/* Macro */
#undef max 
#define max(x,y) ((x) > (y) ? (x) : (y))

using namespace std;


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
  pthread_mutex_destroy(&tmutex);  
  pthread_cancel(thread_pic);

  close_robot_comm();
  exit(0);
}

/*------------------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *----------------------------------------------------------------------------*/
int main(int argc, char** argv)
{
  int select_result = -1; // value returned frome select()
  int nfds = 0; // fd to pass to select()
  fd_set rd, wr, er; // structure for select()
  struct timeval select_timeout;
  
  select_timeout.tv_sec = 0;
  select_timeout.tv_usec = 20000;

  // Set up ROS.
  ros::init(argc, argv, "Saetta_Base");
  ros::NodeHandle n("~");
  Saetta_Base* localbase = new Saetta_Base();
  
  // Declare variables that can be modified by launch file or command line.
  int rate = 20;
  
  // set velocity subscriber
  ros::Subscriber sub = n.subscribe("/saetta/velocity", 10, &Saetta_Base::listenerCallback, localbase);

  // Tell ROS how fast to run this node.
  ros::Rate r(rate);
  
  std::string nPort; //default port name
  n.param<std::string>("port", nPort, "/dev/ttyO3");
  ROS_INFO_STREAM("ROS parameter 'port' setted as: " << nPort);
    
  // convert string to char*
  char* nPort_char = new char[nPort.size() + 1];
  std::copy(nPort.begin(), nPort.end(), nPort_char);
  nPort_char[nPort.size()] = '\0'; // terminating char* with 0

  // print the serial port
  printf("- Starting serial communication with the serial port: %s\n", nPort_char);

  if(init_robot(nPort_char) < 0)
    return -1;
  else
    printf("- Communication started!\n");

  delete[] nPort_char; //free memory
  flag = 0;  //verificare se serve
  float* robot_state = (float *) malloc(sizeof (float) *3);

  // set starting speed
  set_vel_2_array(0.0, 0.0);

  setup_termination();
    
  // Create PIC thread
  if(pthread_create(&thread_pic, NULL, &tf_pic2netus, NULL)!= 0) 
  {
    perror("Error creating PIC thread.\n");
    return 1;
  }
  
  pthread_cond_wait(&cond, &tmutex);
  pthread_mutex_unlock(&tmutex);

  while (n.ok())
  {
/*    FD_ZERO(&rd);
    FD_ZERO(&wr);
    FD_ZERO(&er);
    
    select_result = select(nfds + 1, &rd, &wr, NULL, &select_timeout);

    if(select_result == -1 && errno == EAGAIN)
    {
      perror("select");
      continue;
    }

    if(select_result == -1)
    {
      perror("main:");

      return 1;
    }

    if(select_result > 0)
    {
    }
    
    if((select_timeout.tv_sec == 0) && (select_timeout.tv_usec == 0))
    {*/
      ros::spinOnce();
      set_robot_speed(&(localbase->_linear),&(localbase->_angular));
 
      //pthread_cond_wait(&cond, &tmutex);
      //pthread_mutex_unlock(&tmutex);
    
      get_robot_state(&robot_state);
      
      //the robot states x and y are in [cm], we publish them in [m]
      localbase->set_position(robot_state[0]/100,robot_state[1]/100,robot_state[2]); 
      localbase->publish_odom();
    
      select_timeout.tv_sec = 0;
      select_timeout.tv_usec = 20000;
      r.sleep();
    //}
  }

  return (EXIT_SUCCESS);
}