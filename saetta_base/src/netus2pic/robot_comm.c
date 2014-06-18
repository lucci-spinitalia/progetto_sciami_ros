
#include "robot_comm.h"

//______________________________________________________________________________
//______________________________________________________________________________
//______________________________________________________________________________

//------------------------------------------------------------------------------
void 	init_modulo_comm(char* portname){
	
	
	int i;
	pic_last_vel_2_write=0;
	for(i=0;i<LEN_PIC_BUFFER;i++){
		pic_buffer[i]=malloc(MAX_LEN_PIC_PACKET*sizeof(unsigned char));
		set_vel_2_array(pic_buffer[i],10,10);
	}
	for(i=0;i<MAX_LEN_PIC_PACKET;i++){pic_buffer_raw[i]=0;}

	num_packet_data_ok=0;
	num_packet_data_wrong=0;
	num_packet_sent_wrong=0;
	init_serial_comm(portname);
	#ifdef USA_XBEE
		inizializza_xbee();	
	#endif
	
}
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
void send_vel_packet_2_pic(packet_vel v){
	write(pic_fd, v, PACKET_SPEED_LENGTH+1);
	//stampa_pacchetto(v, PACKET_SPEED_LENGTH);
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
	//printf("----------------------\n");
	//stampa_pacchetto(pacchetto_sporco, len_sporco);
	//printf("----------------------\n");
	for(i=0;i<len_sporco;i++){
		
		if(*(pacchetto_sporco+i)==PACKET_SOGLIA){*(pacchetto_sporco+(++i))-=PACKET_SOGLIA;}
		*(pacchetto_pulito+j)=*(pacchetto_sporco+i);
		j++;
	
	}
//	printf("llllllllllllllllllllll\n");
//	stampa_pacchetto(pacchetto_pulito, j);
//	printf("llllllllllllllllllllll\n");
//	printf("Valore di ritorno J: %d\n", j);
	
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




//------------------------------------------------------------------------------
void stampa_pacchetto(unsigned char* slot, int l){
	
	int i;	
	//printf("Length: %d\n", l);
	for(i=0;i<l;i++)	{
		printf("%02x ", *(slot+i));	
	}
	printf("\n");
}
//------------------------------------------------------------------------------


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
int inserisci_in_pic_buffer(int len){

	int start, end, l, len_packet=0;

	start=0;
	end=0;

	while(end<len){
		start=end;
		l=get_fine_pacchetto(&(pic_buffer_raw[start]));
		if(l!=-1){
			len_packet=pulisci_pacchetto(&(pic_buffer_raw[start]),&(pic_packet_good[0]), l+1);
			if(controlla_crc(pic_packet_good, len_packet)){
				//pic_last_vel_2_write=(++pic_last_vel_2_write)%LEN_PIC_BUFFER;
				++pic_last_vel_2_write;
				pic_last_vel_2_write%=LEN_PIC_BUFFER;
				memcpy(pic_buffer[pic_last_vel_2_write], pic_packet_good, len_packet);
				//stampa_pacchetto(pic_buffer[pic_last_vel_2_write], len_packet);
			}

			end+=l;
			++end;
		}
		else{return -1;}
	}
	return 1;
}
//------------------------------------------------------------------------------

	
//------------------------------------------------------------------------------

void show_pic_buffer(){

	int i;
	for(i=0;i<LEN_PIC_BUFFER;i++){
		stampa_pacchetto(pic_buffer[i], MAX_LEN_PIC_PACKET);
	}
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------

void set_vel_2_array(unsigned char *p, int vel1, int vel2){


	unsigned int crc;
	int i;
	crc=0;
	*p=0x7F;
	
	*(p+1)=(vel1<0);
	*(p+2)=(vel2<0);
	
	*(p+3)=(MAX_VEL-abs(vel1))&255;
	*(p+4)=(MAX_VEL-abs(vel1))>>8;

	
	*(p+5)=(MAX_VEL-abs(vel2))&255;
	*(p+6)=(MAX_VEL-abs(vel2))>>8;
	
	*(p+7)=!(vel1==0 & vel2==0);
	for(i=0;i<8;i++){
		crc+=*(p+i);
	}
	crc=(~crc);
	crc++;
	*(p+8)=crc&255;	
	*(p+9)=crc>>8;
	*(p+10)=0xa;	

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

void    close_robot_comm()
{
	int i;
	
	for(i=0;i<LEN_PIC_BUFFER;i++)
	{
		free(pic_buffer[i]);
	}
}


//------------------------------------------------------------------------------


//Codice da utilizzare in fase di Debugging
//Unuseful!




		/*
		*(pic_buffer[i])=0x7F;
		*(pic_buffer[i]+1)=0;
		*(pic_buffer[i]+2)=0;
		*(pic_buffer[i]+3)=0;
		*(pic_buffer[i]+4)=4;
		*(pic_buffer[i]+5)=0;	
		*(pic_buffer[i]+6)=4;
		*(pic_buffer[i]+7)=0;
		//*(pic_buffer[i]+8)=0x7D;
		//*(pic_buffer[i]+9)=0xFF;

		*(pic_buffer[i]+8)=0x79;
		*(pic_buffer[i]+9)=0xFF;
		*(pic_buffer[i]+10)=0x0A;*/
		
		/**(pic_buffer[i])=0x7F;
		*(pic_buffer[i]+1)=0;
		*(pic_buffer[i]+2)=0;
		*(pic_buffer[i]+3)=0;
		*(pic_buffer[i]+4)=2;
		*(pic_buffer[i]+5)=0;
		*(pic_buffer[i]+6)=2;
		*(pic_buffer[i]+7)=1;
		*(pic_buffer[i]+8)=0x7C;
		*(pic_buffer[i]+9)=0xFF;
		*(pic_buffer[i]+10)=0x0A;*/

