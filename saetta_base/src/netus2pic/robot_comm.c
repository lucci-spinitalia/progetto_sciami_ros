#include "robot_comm.h"

//----------------------------------------------------------------------
int check_soglia(unsigned char* pac)
{
  if(*pac<=PACKET_SOGLIA)
  {
	  *(pac+1)=*(pac)+PACKET_SOGLIA;
	  *pac=PACKET_SOGLIA;
	  return 1;
  }
  
  return 0;
}

void set_vel_2_array(char *buffer_tx, int vel1, int vel2)
{
	unsigned int crc;
	int i;
	crc=0;	
	
  *(buffer_tx) = 0x7F;
	
	*(buffer_tx + 1)=(vel1<0);
	*(buffer_tx + 2)=(vel2<0);
	
	*(buffer_tx + 3)=(MAX_VEL-abs(vel1))&255;
	*(buffer_tx + 4)=(MAX_VEL-abs(vel1))>>8;

	
	*(buffer_tx + 5)=(MAX_VEL-abs(vel2))&255;
	*(buffer_tx + 6)=(MAX_VEL-abs(vel2))>>8;
	
	*(buffer_tx + 7) =!((vel1 == 0) & (vel2 == 0));
	for(i=0;i<8;i++){
		crc+=*(buffer_tx + i);
	}
	crc=(~crc);
	crc++;
	*(buffer_tx + 8)=crc&255;	
	*(buffer_tx + 9)=crc>>8;
	*(buffer_tx + 10)=0xa;
}

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