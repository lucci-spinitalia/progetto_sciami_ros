
#include "robot_comm.h"

///\brief	pacchetto di ricezione per comunicazione da PIC
unsigned char* pic_buffer;

int init_modulo_comm(char* portname)
{
  pic_buffer = malloc(MAX_LEN_PIC_PACKET * sizeof(unsigned char));
  
  if(pic_buffer == NULL)
    return 0;
  
  set_vel_2_array(10,10);
  
  int i;
  for(i = 0; i < MAX_LEN_PIC_PACKET; i++)
  {
    pic_buffer_raw[i] = 0;
  }

	num_packet_data_ok=0;
	num_packet_data_wrong=0;
	num_packet_sent_wrong=0;
  
  pthread_mutex_init(&m_analizza_pacchetto, NULL);
  //pic_fd = tty_open(portname);
  pic_fd = com_open(portname, 115200, 'N', 8, 1);
  
	if(pic_fd <= 0)
    return -1;
    
	#ifdef USA_XBEE
		inizializza_xbee();	
	#endif
	
  
  return 1;
}
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
void send_vel_packet_2_pic(packet_vel v)
{
	write(pic_fd, v, PACKET_SPEED_LENGTH+1);
}
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
void set_vel_packet_2_pic(packet_vel payload, int vel1, int vel2){
	

	unsigned char *paux;
	int i=0;
	payload->crc=0;
	payload->dir1=(vel1<0);
	payload->dir2=(vel2<0);
	payload->vel1=MAX_VEL-abs(vel1);
	payload->vel2=MAX_VEL-abs(vel2);
	payload->power=0;
	
	paux=(char*)payload;
	for(i=0;i<PACKET_SPEED_LENGTH-LEN_CRC;i++){
			//printf("%02x->", *(paux+i));
			payload->crc+=*(paux+i);
			//printf("%02x\n", payload->crc);
	}
	payload->crc=(~payload->crc)+1;
	//printf("CRC %02x", payload->crc);

}
//------------------------------------------------------------------------------





//----------------------------------------------------------------------
int pulisci_pacchetto(unsigned char *pacchetto_sporco, unsigned char *pacchetto_pulito, int len_sporco){

	int i,j;
	i=0;j=0;

	for(i=0;i<len_sporco;i++){
		
		if(*(pacchetto_sporco+i)==PACKET_SOGLIA){*(pacchetto_sporco+(++i))-=PACKET_SOGLIA;}
		*(pacchetto_pulito+j)=*(pacchetto_sporco+i);
		j++;
	
	}
	
	return j;
}
//----------------------------------------------------------------------



//----------------------------------------------------------------------
int controlla_crc(unsigned char* p, int len){

	int i;
	unsigned short int contatore=0;
	unsigned short int crc=0;
	for(i=0;i<len-(LEN_CRC+1);i++){
		contatore+=*(p+i);
	}
	contatore = ~contatore;
	contatore++;
	crc=(*(p+i)+*(p+i+1)*256);
	return contatore==crc;
}
//----------------------------------------------------------------------

//----------------------------------------------------------------------
int check_soglia(unsigned char* pac){

		if(*pac<=PACKET_SOGLIA){
			*(pac+1)=*(pac)+PACKET_SOGLIA;
			*pac=PACKET_SOGLIA;
			return 1;
		}
		return 0;
}

//----------------------------------------------------------------------


//______________________________________________________________________________


//------------------------------------------------------------------------------
int get_fine_pacchetto(unsigned char* p){

	int i=0;
	while(*(p+i)!='\n' && i<MAX_LEN_PIC_PACKET){i++;}
	if(i==MAX_LEN_PIC_PACKET){return -1;}
	return i;
}
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

void set_vel_2_array(int vel1, int vel2)
{
	unsigned int crc;
	int i;
	crc=0;	
	
  *(pic_buffer) = 0x7F;
	
	*(pic_buffer+1)=(vel1<0);
	*(pic_buffer+2)=(vel2<0);
	
	*(pic_buffer+3)=(MAX_VEL-abs(vel1))&255;
	*(pic_buffer+4)=(MAX_VEL-abs(vel1))>>8;

	
	*(pic_buffer+5)=(MAX_VEL-abs(vel2))&255;
	*(pic_buffer+6)=(MAX_VEL-abs(vel2))>>8;
	
	*(pic_buffer+7)=!(vel1==0 & vel2==0);
	for(i=0;i<8;i++){
		crc+=*(pic_buffer+i);
	}
	crc=(~crc);
	crc++;
	*(pic_buffer+8)=crc&255;	
	*(pic_buffer+9)=crc>>8;
	*(pic_buffer+10)=0xa;
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------

void set_pos_2_array(unsigned char *p, int vel1, int vel2, int p1, int p2){


	unsigned int crc;
	int i;
	crc=0;
	*p=128;
	
	*(p+1)=(vel1<0);
	*(p+2)=(vel2<0);
	
	*(p+3)=(MAX_VEL-abs(vel1))&255;
	*(p+4)=(MAX_VEL-abs(vel1))>>8;

	
	*(p+5)=(MAX_VEL-abs(vel2))&255;
	*(p+6)=(MAX_VEL-abs(vel2))>>8;
	
	*(p+7)=0;
	*(p+8)=p1&255;
	*(p+9)=p1>>8;
	*(p+10)=p2&255;
	*(p+11)=p2>>8;

	for(i=0;i<12;i++){
		crc+=*(p+i);
	}
	crc=(~crc);
	crc++;
	*(p+12)=crc&255;	
	*(p+13)=crc>>8;
	*(p+14)=0xa;	

}

/* thread per il PIC */
void* tf_pic2netus(void *args) 
{
  unsigned char buf[256];
  int byte_read;
  int an_ret;
  
  tcflush(pic_fd, TCIFLUSH);
  tcflush(pic_fd, TCOFLUSH);

  int rr;

  // Hook cycle
  do 
  {	
    rr = read(pic_fd, buf, 1);        
  } while(buf[0] != 0x0A);
    
  while(1) 
  {
    memset(buf,'\0',128);
    byte_read = 0;
  
    // Get the start
    do 
    {
      read(pic_fd, buf, 1);        
    } while(buf[0]!='S');
        
    byte_read++;
    
    // Get the whole pkg    
    do 
    {
      read(pic_fd,buf+byte_read,1);
      byte_read++;      
    } while(*(buf+byte_read-1)!='\n');
        
    analizza_pacchetto(pic_buffer, buf, byte_read);
    
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
  
    an_ret = analizza_pacchetto(pic_buffer, buf, byte_read);
  
    if(an_ret == LOAD_PACKET_ANALYZED)
      pthread_cond_signal(&cond); //riparte il ciclo ROS
  }
  
  return 0;
}

void close_robot_comm()
{
  // Clean the buffer	
  tcflush(pic_fd, TCIFLUSH);
  tcflush(pic_fd, TCOFLUSH);	

  set_vel_2_array(0, 0);
  write(pic_fd, pic_buffer, PACKET_SPEED_LENGTH + 1);
  sync();

  close_serial_comm(pic_fd); // da serial_comm.c
  free(pic_buffer);
}
