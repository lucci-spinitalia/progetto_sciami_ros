/* 
 * File:   newmain.cpp
 * Author: erupter
 *
 * Created on December 3, 2012, 4:53 PM
 */
//#define PROC_SEPARATION
#include "Saetta_Base.h"
#define MAX_LEN 99
#define NUM_SENS 5
char buffer[MAX_LEN+1] = {0};
int out_pipe[2];
int saved_stdout;
using namespace std;
char *portname = "/dev/ttyO3";
const char mesg[]="Testing stdout redirection";
int main_proc(int argc, char **argv);
void pipe_eater(void);
std::list<std::string> local_messages;
std::mutex msg_mutex, struct_mutex;
void destroy_win(WINDOW *local_win);
void curses_windows_setup();

#ifdef PROC_SEPARATION
#define WIN_SPACE 	2
#define IR_X_POS 	0
#define IR_Y_POS 	0
#define IR_X_LENGTH 	col-ODOM_X_LENGTH
#define IR_Y_LENGTH 	7
#define SHELL_X_POS 	0
#define SHELL_Y_POS	IR_Y_POS+IR_Y_LENGTH+WIN_SPACE
#define SHELL_X_LENGTH	col
#define SHELL_Y_LENGTH 	10
#define ODOM_X_POS 	col-ODOM_X_LENGTH
#define ODOM_Y_POS 	0
#define ODOM_X_LENGTH	12
#define ODOM_Y_LENGTH	5
WINDOW *win_shell, *win_shell_borders, *win_ir_borders, *win_ir, *win_odom_borders, *win_odom;
#endif

/*------------------------------------------------------------------------------
 * listenerCallback()
 * Receive twist message.
 *----------------------------------------------------------------------------*/

void listenerCallback (const geometry_msgs::Twist::ConstPtr & msg)
{
	 
 //ROS_INFO("Received linear: [%f], angular: [%f]\n", msg->linear.x, msg->angular.z);
}

struct shared_mem {        /* Defines "structure" of shared memory */
    unsigned int IR[5];
    float odom[2];
    int uptodate;
    int dowork;
};


/*void listenerCallback (const turtlesim::Velocity::ConstPtr & msg)
{
    ROS_INFO("Received linear: [%f], angular: [%f]\n", msg->linear, msg->angular);
}*/

/*------------------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *----------------------------------------------------------------------------*/
int main(int argc, char** argv)
{
    int cycle = 0, pid, p[2], row, col;

#ifdef PROC_SEPARATION
if(pipe(p) == -1)
	{
		perror("pipe call error");
		return(1);
	}
	
	switch (pid=fork())
	{
		case -1: perror("error: fork call");
			return(2);

		case 0:  /* if child then write down pipe */
			//close(p[0]);  /* first close the read end of the pipe */
			if(dup2(p[1], 1) == -1 ) /* stdout == write end of the pipe */
			{
				perror( "dup2 failed" );
				return(1);
			}
			
			setvbuf(stdout, NULL, _IOLBF, 1000);
			main_proc(argc,argv);
			printf("\n\tChild quitting cleanly\n\n");
			break;
		default:
			s_catch_signals();
			std::string mystr;
			//close(p[1]);  /* first close the write end of the pipe */
			if(dup2(p[0], 0 ) == -1 ) /* stdin == read end of the pipe */
			{
				perror( "dup2 failed" );
				return(1);
			}
			/* Create shared memory object and set its size */

			struct shared_mem *rptr;
			int shared_mem_fd;
			shared_mem_fd = shm_open("/saetta_shared_mem", O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
			if (shared_mem_fd == -1)
			    /* Handle error */;


			if (ftruncate(shared_mem_fd, sizeof(struct shared_mem)) == -1)
			    /* Handle error */;


			/* Map shared memory object */


			rptr = (struct shared_mem*) mmap(NULL, sizeof(struct shared_mem),
			       PROT_READ | PROT_WRITE, MAP_SHARED, shared_mem_fd, 0);
			if (rptr == MAP_FAILED)
			    /* Handle error */;


			/* Now we can refer to mapped region using fields of rptr;
			   for example, rptr->len */
			rptr->dowork = 1;
			initscr();				/* start the curses mode */
			getmaxyx(stdscr,row,col);		/* get the number of rows and columns */
			//mvprintw(row/2,(col-strlen(mesg))/2,"%s",mesg);
							/* print the message at the center of the screen */

			curses_windows_setup();
			//refresh();
			
			//wprintw(win_shell,"This screen has %d rows and %d columns\n",row,col);
			//wprintw(win_shell,"Try resizing your window(if possible) and then run this program again\n");
			scrollok(win_shell,TRUE);
			int myy,myx;
			std::thread t(pipe_eater);
			std::stringstream ss;
			const char fill_char[] = "|";
			while(!s_interrupted)
			{
				msg_mutex.lock();
				{
					for (int i=0; i<5; i++)
					{
						mvwprintw(win_ir,i,0,"S%1d: %3u",i,rptr->IR[i]);
						int maxwidth = IR_X_LENGTH-2, width;
						float fwidth;
						fwidth = (float)rptr->IR[i] / 255.0;
						width = maxwidth-(int)(float(maxwidth)*fwidth);
						for (int j=9; j<maxwidth; j++)
						{
							if(j>maxwidth-width)
								mvwprintw(win_ir,i,j,fill_char);
							else
								mvwprintw(win_ir,i,j," ");
						}
						wrefresh(win_ir);						
						/*ss.clear();
						ss << "S" << i << ": " << rptr->IR[i] << std::endl;
						mystr= ss.str();
						waddstr(win_ir,mystr.c_str());
						wmove(win_ir,i,0);*/
					}
					wrefresh(win_ir);
					mvwprintw(win_odom,0,0,"X: %07.2f",rptr->odom[0]);
					mvwprintw(win_odom,1,0,"Y: %07.2f",rptr->odom[1]);
					wrefresh(win_odom);
					while(local_messages.size()>0)
					{

						//waddstr(win_shell,local_messages.front().c_str());

						wprintw(win_shell, "%s\n",local_messages.front().c_str());
						local_messages.pop_front();
					}
					msg_mutex.unlock();
					wrefresh(win_shell);
					usleep(100000);
				}

			}
			msg_mutex.lock();
			rptr->dowork=0;
			msg_mutex.unlock();
			t.join();
			endwin();

			/*while(!s_interrupted)
			{
				wmove(win_ir,0,0);
				for (int i=0; i<5; i++)
				{
					mvwprintw(win_ir,i,0,"S%1d: %3u",i,rptr->IR[i]);
					wrefresh(win_ir);						
					/*ss.clear();
					ss << "S" << i << ": " << rptr->IR[i] << std::endl;
					mystr= ss.str();
					waddstr(win_ir,mystr.c_str());
				}
				wrefresh(win_ir);
				while( std::getline(std::cin, mystr) )
				{
					ss.clear();
					ss << mystr << std::endl;
					mystr = ss.str();
					//std::cout<< mystr<<std::endl;
					waddstr(win_shell,mystr.c_str());
					wrefresh(win_shell);
					//getyx(win_shell,myy,myx);
					//wmove(win_shell,myy+1,0);

				}

				usleep(100000);

			}
			endwin();*/
			
			
			break;
	}



	//dup2(saved_stdout, STDOUT_FILENO);  /* reconnect stdout for testing */
	//setvbuf(stdout, NULL, _IOLBF, 0);
	endwin();
	printf("\n\tProgram quitting cleanly\n\n");
	return (EXIT_SUCCESS);
}

int main_proc (int argc, char **argv)
{
    int cycle=0;
/* Create shared memory object and set its size */

struct shared_mem *rptr;
int shared_mem_fd;
shared_mem_fd = shm_open("/saetta_shared_mem", O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
if (shared_mem_fd == -1)
    /* Handle error */;


if (ftruncate(shared_mem_fd, sizeof(struct shared_mem)) == -1)
    /* Handle error */;


/* Map shared memory object */


rptr = (struct shared_mem*)mmap(NULL, sizeof(struct shared_mem),
       PROT_READ | PROT_WRITE, MAP_SHARED, shared_mem_fd, 0);
if (rptr == MAP_FAILED)
    /* Handle error */;


/* Now we can refer to mapped region using fields of rptr;
   for example, rptr->len */
#endif
    int myfd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (myfd < 0)
    {
        printf ("error %d opening %s: %s", errno, portname, strerror (errno));
        return -1;
    }
    set_interface_attribs (myfd, B115200, 0);
    close(myfd);
    std::string mypath;
    std::stringstream ss;
    ss << argv[0];
    mypath = ss.str();
    int last = mypath.find_last_of ('/',mypath.length());
    mypath = mypath.substr (0,last);
    printf ("The current working directory is %s\n", mypath.c_str());
    // Setup termination handler
    chdir(mypath.c_str());
    int res = file_exists("/home/panda/.ros/saetta_base/acc.txt");
    //res ? printf("%s exists\n","acc.txt") : printf("File doesn't exist\n");
    //return 0;

    main_init();

    fp_log = fopen(file_log, "w");
    setup_termination();

    pthread_mutex_init(&tmutex, NULL);

    pthread_t thread_output;
    pthread_attr_t attr_main;
    pthread_attr_init(&attr_main);
    pthread_create(&thread_pic, &attr_main, &tf_pic2netus, NULL);
   // printf("FD Pic: %d\n", pic_fd);
    setup_termination();
    struct timeval tvb, tva;

    char debug_buf[24];
    log_fd=open("/home/panda/.ros/saetta_base/timing_main.txt",O_CREAT | O_WRONLY | O_TRUNC , S_IWUSR | S_IRUSR | S_IRGRP | S_IROTH);

    // The timing of the first cycle is not allined due to the 
    // fact that the two threads are not setup exactly at the
    // same time
    pthread_cond_wait(&cond, &tmutex);
    pthread_mutex_unlock(&tmutex);		



    /*WifiTx mywifi;
    char myaddr[]= "192.168.1.11";
    mywifi.init_wifi(myaddr);
    mywifi.send_wifi_vel(1.0,0.0);
    cout << "Sending speed command for 2 seconds.\n";
    sleep (2);
    cout << "Done.\n";
    mywifi.send_wifi_vel(0,0);*/
    Saetta_Base* localbase = new Saetta_Base();
    
    

    // Set up ROS.
    ros::init(argc, argv, "Saetta_Base");
    ros::NodeHandle n;

    // Declare variables that can be modified by launch file or command line.
    int rate = 20;

    
//    localbase->InitWifi(ip);
    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle base_handle_("~");
    
    //base_handle_.param("rate", rate, int(40));
    ros::Subscriber sub = n.subscribe("/saetta/velocity", 10, &Saetta_Base::listenerCallback, localbase);

    // Create a new NodeExample object.
    //Saetta_Base *base_node = new Saetta_Base();

    // Create a subscriber.
    // Name the topic, message queue, callback function with class name, and object containing callback function.
    //ros::Subscriber sub_message = n.subscribe("example", 1000, &base_node.messageCallback, Saetta_Base);

    // Tell ROS how fast to run this node.
    ros::Rate r(rate);
    long counter = 0;
    //tf::TransformBroadcaster br;
    //tf::Transform transform;    

    while (n.ok())
    {

        ros::spinOnce();

 set_robot_speed(&(localbase->_linear),&(localbase->_angular));
       // ROS_INFO("localbase linear: [%f], angular: [%f]\n", localbase->_linear, localbase->_angular);
	counter++;
	pthread_cond_wait(&cond, &tmutex);
//	printf("Alive! [%ld]\n",counter);
	//printf("Steps\tX:%04.2f\tY:%04.2f\n",robot_state[0],robot_state[1]);
	pthread_mutex_unlock(&tmutex);
	gettimeofday(&tvb,NULL);

	//
	get_robot_state(&robot_state);
	//printf("%f\t%f\t%f\n",state[0],state[1],RAD2DEG(state[2]));
	//printf("lu: %9.7f   \tlw: %9.7f\n",last_v_ref,last_w_ref);


	gettimeofday(&tva,NULL);
	//printf("IR Ranger:\n");
	int i;
#ifdef PROC_SEPARATION
	struct_mutex.lock();
	{
		for (i=0; i<NUM_SENS; i++)
			rptr->IR[i]=*(ir->range+i);
		//printf("S%02d  %04u\n", i, *(ir->range+i));

		rptr->odom[0]=robot_state[0];
		rptr->odom[1]=robot_state[1];
		//transform.setOrigin( tf::Vector3(robot_state[0]/100, robot_state[1]/100, 0.0) );
		//transform.setRotation( tf::Quaternion(0, 0, robot_state[2]) );
		struct_mutex.unlock();
		//br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
		//printf("Transform sent!\n");
	}
#else
	//printf("\n\n");
		for (i=0; i<NUM_SENS; i++)
	//	printf("S%02d  %04u\n", i, *(ir->range+i));
        //transform.setOrigin( tf::Vector3(robot_state[0]/100, robot_state[1]/100, 0.0) );
        //transform.setRotation( tf::Quaternion(0, 0, robot_state[2]) );
        //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
//	printf("Odom X: %6.2f\nOdom Y: %6.2f\nOdom a: %6.2f\n",robot_state[0]/100, robot_state[1]/100, robot_state[3]);
        //printf("Transform sent!\n");

#endif
	if (tva.tv_sec==tvb.tv_sec)
	{
#ifdef LOG_FROM_MAIN_THREAD 
	  //  printf("%ld\n", tva.tv_usec-tvb.tv_usec);

   	    cycle=tva.tv_usec-tvb.tv_usec;
#endif
	}
	else
	{
            int delta;
            delta = 1000000-tvb.tv_usec;
#ifdef LOG_FROM_MAIN_THREAD
        //    printf("%ld\n",tva.tv_usec+delta);

	    cycle=tva.tv_usec+delta;
#endif
	}
#ifdef LOG_FROM_MAIN_THREAD
	//sprintf(debug_buf,"%d\n",cycle);
	write(log_fd, debug_buf,strlen(debug_buf));
#endif	
        r.sleep();

    }

//    dup2(saved_stdout, STDOUT_FILENO);  /* reconnect stdout for testing */
    return (EXIT_SUCCESS);
}

void main_init() {
    int i;
    float starting_speed = 0.0;
    init_robot();
    flag = 0;
    robot_state = (float *) malloc(sizeof (float) *3);

    for (i = 0; i < LEN_PIC_BUFFER; i++) {
        set_vel_2_array(pic_buffer[i], starting_speed, starting_speed);
    }

    write(pic_fd, pic_message_reset_steps_acc, PACKET_TIMING_LENGTH + 1);
    tcflush(pic_fd, TCOFLUSH);
    sync();
    steps_anomaly = 0;

}

void setup_termination() {
    if (signal(SIGINT, termination_handler) == SIG_IGN) signal(SIGINT, SIG_IGN);
    if (signal(SIGHUP, termination_handler) == SIG_IGN) signal(SIGHUP, SIG_IGN);
    if (signal(SIGTERM, termination_handler) == SIG_IGN) signal(SIGTERM, SIG_IGN);
}

void termination_handler(int signum) {

    int i;


    pthread_mutex_destroy(&tmutex);  
    pthread_cancel(thread_pic);

    // Clean the bufffer	
    tcflush(pic_fd, TCIFLUSH);
    tcflush(pic_fd, TCOFLUSH);	
 
   fprintf(fp_log, "\nNum packet data: correct  %d wrong  %d sent_wrong: %d \n", num_packet_data_ok, num_packet_data_wrong, num_packet_sent_wrong);
    fflush(fp_log);
    fclose(fp_log);
#ifdef LOG_FROM_PIC_THREAD
    printf("scritto\n");
#endif
    set_vel_2_array(pic_buffer[pic_last_vel_2_send], 0, 0);
    write(pic_fd, pic_buffer[pic_last_vel_2_write], PACKET_SPEED_LENGTH + 1);
    sync();
 
    exit(0);
}

void* tf_pic2netus(void *args) {
	
	
	int i;
	unsigned char buf[256];
	struct timeval tvb, tva;	
	int counter;
	int byte_read;
	int tot_byte_read;
	int an_ret;

#ifdef LOG_FROM_PIC_THREAD
	int cycle;
	unsigned char debug_buf[128];
	pic_log_fd=open("/home/panda/.ros/saetta_base/timing_pic.txt",O_CREAT | O_WRONLY | O_TRUNC , S_IWUSR | S_IRUSR | S_IRGRP | S_IROTH);
#endif	


#ifdef LOG_FROM_PIC_THREAD	
	cycle=0;
#endif
	
	counter=0;

	tcflush(pic_fd, TCIFLUSH);
	tcflush(pic_fd, TCOFLUSH);

	int rr;
	// Hook cycle
	do {	
		rr=read(pic_fd, buf, 1);
#ifdef LOG_FROM_PIC_THREAD
		printf("Read %d\n",rr);
#endif
	}
	while(buf[0]!=0x0A);
	
	while(1) {
		memset(buf,'\0',128);
		counter++;
		byte_read=0;
#ifdef LOG_FROM_PIC_THREAD
		printf("Cycle: %d\n",counter);
#endif
             // Get the start
                do {
                        read(pic_fd, buf, 1);
                }
                while(buf[0]!='S');
                byte_read++;
 		gettimeofday(&tva,NULL);

 	        if (tva.tv_sec==tvb.tv_sec){
#ifdef LOG_FROM_PIC_THREAD
        //	        printf("%ld  \t%d\n", tva.tv_usec-tvb.tv_usec, byte_read);

                	cycle=tva.tv_usec-tvb.tv_usec;
#endif
        	}
        	else {
                	int delta;
                	delta = 1000000-tvb.tv_usec;
#ifdef LOG_FROM_PIC_THREAD
            //    	printf("%ld  \t%d\n",tva.tv_usec+delta, byte_read);

                	cycle=tva.tv_usec+delta;
#endif
        	}

#ifdef LOG_FROM_PIC_THREAD
            //    sprintf(debug_buf,"%ld  \t%d\n",cycle,byte_read);
                write(pic_log_fd, debug_buf,strlen(debug_buf));
#endif


              
		// Get the whole pkg    
                do {
                        read(pic_fd,buf+byte_read,1);
                        byte_read++;
                }
                while(*(buf+byte_read-1)!='\n');
/*                for (i=0; i< byte_read; i++) {
                        printf("%02x\t",buf[i]);
                //      printf("%c\t",buf[i]);
                }       
            //    printf("\n");
*/                tot_byte_read=byte_read;
//		write(pic_fd, pic_message_timing, 4);
		analizza_pacchetto(buf,byte_read);
		byte_read=0;
                // Get the whole pkg    
                memset(buf,'\0',128);
                byte_read=0;
                do {
                        read(pic_fd,buf+byte_read,1);
                        byte_read++;
                }
                while(*(buf+byte_read-1)!='\n');
/*                for (i=0; i< byte_read; i++) {
                        printf("%02x\t",buf[i]);
                //      printf("%c\t",buf[i]);
                }
             //   printf("\n");
*/                tot_byte_read+=byte_read;
		an_ret=analizza_pacchetto(buf,byte_read);
		gettimeofday(&tvb,NULL);
	
		if (an_ret == LOAD_PACKET_ANALYZED )
			pthread_cond_signal(&cond);
		//else
		//	printf("problem!\n");

	}
	
}




WINDOW *create_newwin(int height, int width, int starty, int startx, int vert, int horz)
{	WINDOW *local_win;

	local_win = newwin(height, width, starty, startx);
	box(local_win, vert , horz);		/* 0, 0 gives default characters 
					 * for the vertical and horizontal
					 * lines			*/
	wrefresh(local_win);		/* Show that box 		*/

	return local_win;
}

void destroy_win(WINDOW *local_win)
{	
	/* box(local_win, ' ', ' '); : This won't produce the desired
	 * result of erasing the window. It will leave it's four corners 
	 * and so an ugly remnant of window. 
	 */
	wborder(local_win, ' ', ' ', ' ',' ',' ',' ',' ',' ');
	/* The parameters taken are 
	 * 1. win: the window on which to operate
	 * 2. ls: character to be used for the left side of the window 
	 * 3. rs: character to be used for the right side of the window 
	 * 4. ts: character to be used for the top side of the window 
	 * 5. bs: character to be used for the bottom side of the window 
	 * 6. tl: character to be used for the top left corner of the window 
	 * 7. tr: character to be used for the top right corner of the window 
	 * 8. bl: character to be used for the bottom left corner of the window 
	 * 9. br: character to be used for the bottom right corner of the window
	 */
	wrefresh(local_win);
	delwin(local_win);
}	


int file_exists(const char * filename)
{
    FILE * file;
    if (file = fopen(filename, "r"))
    {
        fclose(file);
        return 1;
    }
    return 0;
}

void curses_windows_setup()
{
#ifdef PROC_SEPARATION
	int row,col;
	getmaxyx(stdscr,row,col);
	
	win_odom_borders = create_newwin( ODOM_Y_LENGTH, ODOM_X_LENGTH, ODOM_Y_POS, ODOM_X_POS, 0, 0);
	wrefresh(win_odom_borders);

	win_odom = create_newwin( ODOM_Y_LENGTH-2, ODOM_X_LENGTH-2, ODOM_Y_POS+1, ODOM_X_POS+1, ' ', ' ');
	wrefresh(win_odom);

	win_ir_borders = create_newwin( IR_Y_LENGTH, IR_X_LENGTH, IR_Y_POS, IR_X_POS, 0, 0);
	wrefresh(win_ir_borders);

	win_ir = create_newwin( IR_Y_LENGTH-2, IR_X_LENGTH-2, IR_Y_POS+1, IR_X_POS+1, ' ', ' ');
	wrefresh(win_ir);

	int shell_y_length = SHELL_Y_LENGTH;
	if (SHELL_Y_LENGTH > row-IR_Y_LENGTH-WIN_SPACE)
		shell_y_length = row-IR_Y_LENGTH-WIN_SPACE;
	else
		shell_y_length = SHELL_Y_LENGTH;
	
	win_shell_borders = create_newwin(shell_y_length,SHELL_X_LENGTH,SHELL_Y_POS, SHELL_X_POS, 0, 0);	
	wrefresh(win_shell_borders);

	win_shell = create_newwin(shell_y_length-2,SHELL_X_LENGTH-2,SHELL_Y_POS+1, SHELL_X_POS+1, ' ', ' ');
	wrefresh(win_shell);	

	const char * win_shell_title = "Shell Output";
	const char * win_ir_title = "IRs";
	const char * win_odom_title = "Odom";

	mvwprintw(win_ir_borders,0,(int)((IR_X_LENGTH)/2) - (int)strlen(win_ir_title)/2,"%s",win_ir_title);
	wrefresh(win_ir_borders);

	mvwprintw(win_shell_borders,0,(int)(SHELL_X_LENGTH)/2 - (int)strlen(win_shell_title)/2,"%s",win_shell_title);
	wrefresh(win_shell_borders);

	mvwprintw(win_odom_borders,0,(int)(ODOM_X_LENGTH)/2 - (int)strlen(win_odom_title)/2,"%s",win_odom_title);
	wrefresh(win_odom_borders);

	
#endif
}

void pipe_eater (void)
{
	std::string mystr;
	std::stringstream ss;

	struct shared_mem *rptr;
	int shared_mem_fd;
	shared_mem_fd = shm_open("/saetta_shared_mem", O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
	if (shared_mem_fd == -1)
	    /* Handle error */;


	if (ftruncate(shared_mem_fd, sizeof(struct shared_mem)) == -1)
	    /* Handle error */;


	/* Map shared memory object */


	rptr = (struct shared_mem*) mmap(NULL, sizeof(struct shared_mem),
	       PROT_READ | PROT_WRITE, MAP_SHARED, shared_mem_fd, 0);
	if (rptr == MAP_FAILED)
	    /* Handle error */;

	int dowork = 1;
	while(dowork)
	{
		while( std::getline(std::cin, mystr) )
		{
			msg_mutex.lock();
			local_messages.push_back(mystr);
			msg_mutex.unlock();
			struct_mutex.lock();
			if (rptr->dowork==0)
			{
				struct_mutex.unlock();
				dowork = 0;
				break;
			}
			struct_mutex.unlock();


		}
	}
}
