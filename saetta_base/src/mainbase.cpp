
/* 
 * File:   mainbase.cpp
 * Author: Massimo Cardellicchio
 *
 * Created on June 9, 2014, 10:45 PM
 */

/* LISTA VARIABILI GLOBALI PRESE DALLA LIBRERIA C
 * 
 * da robot_comm.h :
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
#include <signal.h>
#include <string.h>
#include "Saetta_Base.h"
#include "Rs232/rs232.h"

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
  //pthread_mutex_destroy(&tmutex);  
  //pthread_cancel(thread_pic);

  close_robot_comm();
  exit(0);
}

/*------------------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *----------------------------------------------------------------------------*/
int main(int argc, char** argv)
{
  char message_buffer[256];
  unsigned char message_state = 0;
  unsigned char bytes_read = 0;
  
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
  //int rate = 20;

  // set velocity subscriber
  ros::Subscriber sub = n.subscribe("/saetta/velocity", 10, &Saetta_Base::listenerCallback, localbase);

  // Tell ROS how fast to run this node.
  //ros::Rate r(rate);

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
  flag = 0;  //verificare se serve
  float* robot_state = (float *) malloc(sizeof (float) *3);

  // set starting speed
  set_vel_2_array(0.0, 0.0);
  printf("Velocity set to 0\n");

  setup_termination();

  // Create PIC thread
  /*if(pthread_create(&thread_pic, NULL, &tf_pic2netus, NULL)!= 0) 
  {
    perror("Error creating PIC thread.\n");
    return 1;
  }

  printf("Thread created. Waiting for mutex. . .");
  fflush(stdout);
  pthread_cond_wait(&cond, &tmutex);
  pthread_mutex_unlock(&tmutex);
  printf("[OK]\n");*/

  while(n.ok())
  {
    FD_ZERO(&rd);
    FD_ZERO(&wr);
    FD_ZERO(&er);
    
    if(pic_fd > 0)
    {
      FD_SET(pic_fd, &rd);
      nfds = max(nfds, pic_fd);
      
      if(rs232_buffer_tx_empty == 0)
      {
        FD_SET(pic_fd, &wr);
        nfds = max(nfds, pic_fd);
      }
    }
    
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
      if(pic_fd > 0)
      {
        if(FD_ISSET(pic_fd, &rd))
        {
          bytes_read = rs232_read(pic_fd);
     
          if((bytes_read > 0) || ((bytes_read < 0) && rs232_buffer_rx_full))
          {
            bytes_read = rs232_unload_rx_filtered(message_buffer, 0x0a);

            if(bytes_read > 0)
            {
              message_buffer[bytes_read] = '\0';

              switch(message_state)
              {
                case 0:
                  if(strncmp(message_buffer, "Start", strlen("Start")) == 0)
                  {
                    printf("Analizza pacchetto\n");
                    message_state++;
                  }
                  break;
                  
                case 1:
                  int i = 0;
                  for(i = 0; i < bytes_read; i++)
                    printf("[%x]", message_buffer[i]);
                    
                  printf("\n");
                  
                  analizza_pacchetto(pic_buffer, message_buffer, bytes_read);

                  message_state = 0;
                  break;
              }
            }
          }
        }

        if(FD_ISSET(pic_fd, &wr))
        {
          printf("Write operation!\n");
          bytes_sent = rs232_write(pic_fd);
        }
      }
    }
    
    if((select_timeout.tv_sec == 0) && (select_timeout.tv_usec == 0))
    {
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
      //r.sleep();
    }
  }

  return (EXIT_SUCCESS);
}
