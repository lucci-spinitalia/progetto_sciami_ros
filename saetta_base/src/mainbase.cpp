
/* 
 * File:   mainbase.cpp
 * Author: Massimo Cardellicchio
 *
 * Created on June 9, 2014, 10:45 PM
 */

/* LISTA VARIABILI GLOBALI PRESE DALLA LIBRERIA C
 * 
 * da netus2pic/serial_comm.h :
 * pic_fd
 * pic_buffer
 * flag
 * PACKET_TIMING_LENGTH
 * 
 * da netus2pic/robot_comm.h :
 * LEN_PIC_BUFFER
 * pic_last_vel_2_send
 * pic_last_vel_2_write
 * pic_message_reset_steps_acc
 * 
 * da core/pic2netus.h
 * LOAD_PACKET_ANALYZED = 2
 
 */
#include "Saetta_Base.h"

using namespace std;

pthread_t thread_pic;
pthread_mutex_t tmutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;


void setup_termination();
void termination_handler(int signum);
void* tf_pic2netus(void *args);
int set_interface_attribs (int fd, int speed, int parity);



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

    // Clean the buffer	
    tcflush(pic_fd, TCIFLUSH);
    tcflush(pic_fd, TCOFLUSH);	
    //printf("lastvel2send = %i\n", pic_last_vel_2_send);
    //printf("lastvel2write = %i\n", pic_last_vel_2_write);

    set_vel_2_array(pic_buffer[pic_last_vel_2_send], 0, 0);
    write(pic_fd, pic_buffer[pic_last_vel_2_write], PACKET_SPEED_LENGTH + 1);
    sync();
    close_serial_comm(); // da serial_comm.c
    exit(0);
}

/*------------------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *----------------------------------------------------------------------------*/
int main(int argc, char** argv)
{    
    // Set up ROS.
    ros::init(argc, argv, "Saetta_Base");
    ros::NodeHandle n("~");
    Saetta_Base* localbase = new Saetta_Base();
    // Declare variables that can be modified by launch file or command line.
    int rate = 20;
    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    //ros::NodeHandle base_handle_("~"); //verificare se è davvero essenziale (non credo)
    ros::Subscriber sub = n.subscribe("/saetta/velocity", 10, &Saetta_Base::listenerCallback, localbase);

    // Tell ROS how fast to run this node.
    ros::Rate r(rate);
    std::string nPort; //default port name
    n.param<std::string>("port", nPort, "/dev/ttyO3");
    ROS_INFO_STREAM("Port: " << nPort);
    float starting_speed = 0.0;
    //secondo libreria il log va aperto per forza. 
    //O si leva dalla lib o si trova un path adeguato
    //char file_log[100]="/home/panda/.ros/saetta_base/acc.txt";
    char file_log[100]="acc.txt";  
    fp_log = fopen(file_log, "w");
    
    // convert string to char*
    char* nPort_char = new char[nPort.size() + 1];
    std::copy(nPort.begin(), nPort.end(), nPort_char);
    nPort_char[nPort.size()] = '\0'; // terminating char* with 0
    
    // print the serial port
    printf("- Starting serial communication with the serial port: ");
    
    int i = 0;
    while(nPort_char[i]!='\0')
    {
        printf("%c",nPort_char[i]);
        i++;
    }
    printf("\n");
    
    int myfd = open (nPort_char, O_RDWR | O_NOCTTY | O_SYNC);
    if (myfd < 0)
    {
        printf ("error %d opening %s: %s", errno, nPort_char, strerror (errno));
        return -1;
    }
    set_interface_attribs (myfd, B115200, 0);
    close(myfd);
    
    init_robot();
    
    // start serial communication
    init_modulo_comm(nPort_char); //da robot_comm.c
    printf("- Communication started!\n");
    delete[] nPort_char; //free memory
    flag = 0;  //verificare se serve
    float* robot_state = (float *) malloc(sizeof (float) *3);

    for (i = 0; i < LEN_PIC_BUFFER; i++) 
    {
        set_vel_2_array(pic_buffer[i], starting_speed, starting_speed);
    }

    write(pic_fd, pic_message_reset_steps_acc, PACKET_TIMING_LENGTH + 1);
    tcflush(pic_fd, TCOFLUSH);
    sync();
    //steps_anomaly = 0; 

    setup_termination();  //???
    // Create PIC thread
    if(pthread_create(&thread_pic,NULL,&tf_pic2netus,NULL)!= 0) 
    {
        perror("Error creating PIC thread.\n");
        return 1;
    }
    pthread_cond_wait(&cond, &tmutex);
    pthread_mutex_unlock(&tmutex);
    setup_termination(); //???

    while (n.ok())
    {
        ros::spinOnce();
        set_robot_speed(&(localbase->_linear),&(localbase->_angular));
	pthread_cond_wait(&cond, &tmutex);
	pthread_mutex_unlock(&tmutex);
	get_robot_state(&robot_state);
        localbase->set_position(robot_state[0],robot_state[1],robot_state[2]);
        localbase->publish_odom();
        r.sleep();
    }

    return (EXIT_SUCCESS);
}


/* thread per il PIC */
void* tf_pic2netus(void *args) 
{
    unsigned char buf[256];
    int byte_read;
    int tot_byte_read;
    int an_ret;
    tcflush(pic_fd, TCIFLUSH);
    tcflush(pic_fd, TCOFLUSH);
    int rr;
    // Hook cycle
    do 
    {	
        rr=read(pic_fd, buf, 1);
    }   while(buf[0]!=0x0A);
    
    while(1) 
    {
        memset(buf,'\0',128);
        byte_read=0;
        // Get the start
        do 
        {
            read(pic_fd, buf, 1);
            
        }   while(buf[0]!='S');
        
        byte_read++;
        // Get the whole pkg    
        do 
        {
            read(pic_fd,buf+byte_read,1);
            byte_read++;
            
        }   while(*(buf+byte_read-1)!='\n');
        
        tot_byte_read = byte_read;
        analizza_pacchetto(buf,byte_read);
        byte_read = 0;
        // Get the whole pkg
        memset(buf,'\0',128);
        byte_read = 0;
        do 
        {
            read(pic_fd,buf+byte_read,1);
            byte_read++;
        }
        while(*(buf+byte_read-1)!='\n');
        an_ret = analizza_pacchetto(buf,byte_read);
        if (an_ret == LOAD_PACKET_ANALYZED )
        {
            pthread_cond_signal(&cond); //riparte parte il ciclo ROS
        }
    }	
}



int set_interface_attribs (int fd, int speed, int parity)
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
        
        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}