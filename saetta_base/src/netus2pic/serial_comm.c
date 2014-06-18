#include "serial_comm.h"

// Pic
void init_serial_comm(char* portname){
	pthread_mutex_init(&m_analizza_pacchetto, NULL);
    //Variabile contatore
	int i;
	//open_port(c, 0, 1, 0);
	//tty_open(PIC_DEVICE);
        tty_open(portname);
	//tty_open_rfid();
	//apri_seriale();
	for(i=0;i<N_PIC_MES_BUFF;i++){*(pic_message_buffer+i)=malloc(MAX_PIC_MES_LENGHT*sizeof(unsigned char));}
	current_pic_packet_slot=0;
}
//______________________________________________________________________________



//______________________________________________________________________________
void signal_handler_IO (int status) {
	
pthread_mutex_lock (&m_analizza_pacchetto);
flag=1;
pthread_cond_signal (&cv_analizza_pacchetto);
/* Unlock the mutex. */
pthread_mutex_unlock (&m_analizza_pacchetto);
}
//------------------------------------------------------------------------------

//______________________________________________________________________________
// alternativa ad apri seriale
int tty_open(char* tty_dev) {
	struct termios new_attributes;

	pic_fd = open(tty_dev,O_RDWR| O_NOCTTY | O_NONBLOCK);
//	pic_fd = open(tty_dev,O_RDWR| O_NOCTTY  |  O_NDELAY);
	
	if (pic_fd<0) {
		return -1;
	}
	else {
		tcgetattr(pic_fd,&oldtio);
		tcgetattr(pic_fd,&new_attributes);
		
		// Set the new attributes for the serial port
		// http://linux.about.com/library/cmd/blcmdl3_termios.htm
		// http://www.gnu.org/software/libc/manual/html_node/Low_002dLevel-I_002fO.html#Low_002dLevel-I_002fO
		fcntl(pic_fd, F_SETFL, 0);	// Blocking	
		// c_cflag
		new_attributes.c_cflag |= CREAD;		 	// Enable receiver
		new_attributes.c_cflag |= B115200;		 	// Set baud rate
		//new_attributes.c_cflag |= 38400;		 	// Set baud rate
		new_attributes.c_cflag |= CS8;			 	// 8 data bit
	
		// c_iflag
		new_attributes.c_iflag |= IGNPAR;		 	// Ignore framing errors and parity errors. 
	
		// c_lflag
		new_attributes.c_lflag &= ~(ICANON); 	// DISABLE canonical mode. 
																				// Disables the special characters EOF, EOL, EOL2, 
																				// ERASE, KILL, LNEXT, REPRINT, STATUS, and WERASE, and buffers by lines.
		new_attributes.c_lflag &= ~(ECHO);		// DISABLE this: Echo input characters.
		new_attributes.c_lflag &= ~(ECHOE);		// DISABLE this: If ICANON is also set, the ERASE character erases the preceding input 
																				// character, and WERASE erases the preceding word.
		new_attributes.c_lflag &= ~(ISIG);		// DISABLE this: When any of the characters INTR, QUIT, SUSP, 
																				// or DSUSP are received, generate the corresponding signal.
	  
		new_attributes.c_cc[VMIN]=5;					// Minimum number of characters for non-canonical read.
		new_attributes.c_cc[VTIME]=10;					// Timeout in deciseconds for non-canonical read.
		new_attributes.c_oflag = 0;
		new_attributes.c_oflag &= ~OPOST;

		tcsetattr(pic_fd, TCSANOW, &new_attributes);
                
	}
  return pic_fd;
}
//______________________________________________________________________________

void close_serial_comm()
{
int i;
	for(i=0;i<N_PIC_MES_BUFF;i++)
	{
		free(pic_message_buffer[i]);
	}
	pthread_mutex_destroy(&m_analizza_pacchetto);
        close(pic_fd);
}
