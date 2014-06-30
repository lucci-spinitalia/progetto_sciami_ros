#include "serial_comm.h"

int tty_open(char* tty_dev) 
{ 
  tty_fd = open (tty_dev, O_RDWR | O_NOCTTY | O_SYNC);
    
  if (tty_fd < 0) 
  {
    printf ("error %d opening %s: %s", errno, tty_dev, strerror (errno));
    return -1;
  }
  
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (tty_fd, &tty) != 0)
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

  if (tcsetattr (tty_fd, TCSANOW, &tty) != 0)
  {
    printf ("error %d from tcsetattr", errno);
    return -1;
  }

  return tty_fd;
}
//______________________________________________________________________________

void close_serial_comm(int tty_fd)
{
  close(tty_fd);
}
