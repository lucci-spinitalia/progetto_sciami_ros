#include "serial_comm.h"

// Pic
int init_serial_comm(char* portname)
{
	pthread_mutex_init(&m_analizza_pacchetto, NULL);
    //Variabile contatore
	int i;
	//open_port(c, 0, 1, 0);
	//tty_open(PIC_DEVICE);
  if(tty_open(portname) == -1)
    return -1;
    
	//tty_open_rfid();
	//apri_seriale();
	for(i=0; i < N_PIC_MES_BUFF; i++)
  {
    *(pic_message_buffer + i) = malloc(MAX_PIC_MES_LENGHT * sizeof(unsigned char));
  }
  
	current_pic_packet_slot = 0;
  
  return 1;
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

    pic_fd = open (tty_dev, O_RDWR | O_NOCTTY | O_SYNC);
    if (pic_fd < 0) {
            printf ("error %d opening %s: %s", errno, tty_dev, strerror (errno));
            return -1;
    }
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (pic_fd, &tty) != 0)
    {
        printf ("error %d from tcgetattr", errno);
        return -1;
    }

    cfsetospeed (&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_cflag = B115200 | CS8 | CSTOPB | PARODD | CREAD | CLOCAL;
    tty.c_cflag &= ~(PARODD | PARENB | CRTSCTS);
    tty.c_iflag = ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON | IXOFF | IUCLC | IXANY | IMAXBEL | IUTF8);            
    tty.c_oflag = 0;
    tty.c_lflag = ~(ECHO | ECHOE | ECHOK | ECHONL | ECHOPRT | ECHOCTL | ECHOKE | ICANON | ISIG | IEXTEN | NOFLSH | XCASE | TOSTOP);
    tty.c_cc[VTIME] = 10; //inter-character timer unused
    tty.c_cc[VMIN] = 4; //blocking read

    if (tcsetattr (pic_fd, TCSANOW, &tty) != 0)
    {
        printf ("error %d from tcsetattr", errno);
        return -1;
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
