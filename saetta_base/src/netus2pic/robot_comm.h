#ifndef _ROBOT_COMM_
#define _ROBOT_COMM_
///\file	File per la gestione della comunicazione
//		MODULO DI COMUNICAZIONE
#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>     
#include <string.h>    
#include <unistd.h>    
#include "serial_comm.h"



//							COSTANTI


///\brief	massima velocità MOTORI (rispetto al driver)
#define 	MAX_VEL				1024	//	max speed
///\brief	stato motori ON
#define 	FLAG_MOTOR_POWER_ON		1		//	flag motor power on
///\brief	lunghezza - in byte - del CRC dei pacchetti
#define 	LEN_CRC				2		//  crc packet len
///\brief	massima lunghezza del pacchetto 
#define		MAX_LEN_PIC_PACKET		30		//	max len of pic-clean packet
///\brief	lunghezza del buffer di ricezione radio 
#define 	LEN_PIC_BUFFER 			5		// 	len of pic buffer
///\brief	massima lunghezza del pacchetto radio 
#define 	MAX_LEN_RAW_PIC_PACKET 		100		//	max len of raw-pic packet
///\brief	FLAG per l'uso dell'xbee
//#define 	USA_XBEE


//							STRUCTURES


//			speed packet
/**\brief	paccheto di velocità
* \struct
*/
struct packet_vel_t{				///\brief	header del pacchetto
	unsigned char header;			///\brief	verso di rotazione del motore 1
	unsigned char dir1;			///\brief	verso di rotazione del motore 2
	unsigned char dir2;			///\brief	velocità del motore 1
	unsigned short int vel1;		///\brief	velocità del motore 2
	unsigned short int vel2;		///\brief	accensione motore
	unsigned char power;			///\brief	crc del pacchetto
	unsigned short int crc;			///\brief	carattere finale del pacchetto
	unsigned char end_char;			

};


//_____________________________________________________________________________
//			servoing packet
/**\brief	paccheto di servoing (asservimento in posizione)
*\struct
*/
struct packet_servoing_t{				///\brief	header del pacchetto
	unsigned char 	header;		///\brief	direzione motore 1
	unsigned char 	dir1;		///\brief	direzione motore 2
	unsigned char 	dir2;		///\brief	velocità motore 1
	unsigned short	int 	vel1;	///\brief	velocità motore 2
	unsigned short	int 	vel2;	///\brief	posizione (in passi motore) motore 1 da asservire
	unsigned short	int 	pos1;	///\brief	posizione (in passi motore) motore 2 da asservire
	unsigned short  int 	pos2;	///\brief	accensione motore
	unsigned char 	power;		///\brief	crc del pacchetto			
	unsigned short	int crc;	///\brief	carattere finale del pacchetto
	unsigned char 	end_char;

};
//_____________________________________________________________________________


	
/**\brief	puntatore ad un pacchetto di velocità
*\struct
*/
typedef struct 	packet_vel_t *packet_vel;	//	pointer to speed packet



//------------------------------------------------------------------------------
///\brief	pacchetto di velocità
packet_vel			pacchetto_vel;
//int 				moltiplicatore[2]={1,256};		//	aux
//int 				segno[2]={1,-1};				//	aux
///\brief	numero di pacchetti ricevuti correttamente
int 					num_packet_data_ok;				//	number of correct received packets
///\brief	numero di pacchetti ricevuti in modo errato
int					num_packet_data_wrong;			//	nomber of total received packets
///\brief	number of corrupted sent packets
int					num_packet_sent_wrong;			//	nomber of total received packets

//------------------------------------------------------------------------------
///\brief	last command written to the pic buffer
unsigned short int		pic_last_vel_2_write;					//	last correct vel received
///\brief	last speed to send to the pic
unsigned short int		pic_last_vel_2_send;					//	last correct vel received

///\brief	dimensione del pacchetto di ricezione per comunicazione da PIC
int 				pic_buffer_size;				//	size of joy buffer
///\brief	pacchetto di ricezione per comunicazione da PIC
unsigned char*			pic_buffer[LEN_PIC_BUFFER];		//	joy buffer
///\brief	pacchetto di ricezione per comunicazione da PIC con mascheramento
unsigned char 			pic_buffer_raw[MAX_LEN_RAW_PIC_PACKET];	//	raw joy packet
///\brief	pacchetto di ricezione per comunicazione da pic "smascherato" (cioè filtrato)
unsigned char 			pic_packet_good[MAX_LEN_PIC_PACKET];	//	clean joy packet



///\brief	messaggio timing
static unsigned char			pic_message_timing[4]={130,0x7E, 0xFF, 0x0A};	//	comm buffer 

///\brief	messaggio timing
static unsigned char			pic_message_reset_steps_acc[4]={131,0x7D, 0xFF, 0x0A};	//	comm buffer




//		INITIALIZATION
/**
Funzione di inizializzazione dei vari moduli di comunicazione (pic e radio)
@brief Inizializzazione  della porta seriale-pic e dei buffer di comunicazione
*/
void 	init_modulo_comm(char* portname);				//	comm module initialization

//		operazioni sul pacchetto vel


//______________________________________________________________________________

/**
Creazione diretta dell'array da inviare al PIC per un comando in velocità
@param[in] p 		puntatore all'array
@param[in] vel1  	velocità motore 1
@param[in] vel2  	velocità motore 2
*/

void	set_vel_2_array(unsigned char *p, int vel1, int vel2);

/**
Creazione diretta dell'array da inviare al PIC per un comando in posizione
@param[in] p 		puntatore all'array
@param[in] vel1  	velocità motore 1
@param[in] vel2  	velocità motore 2
@param[in] p1  		passi da far effettuare al motore 1
@param[in] p2  		passi da far effettuare al motore 2

*/

void 	set_pos_2_array(unsigned char *p, int vel1, int vel2, int p1, int p2);

/**
Preparazione pacchetto di velocità
@param[in] payload puntatore a pacchetto di velocità
@param[in] vel1 velocità motore 1
@param[in] vel2 velocità motore 2
@brief Invio pacchetto vel
*/

void 	set_vel_packet_2_pic(packet_vel payload, int vel1, int vel2);	// vel packet setting

/**
Funzione che invia un pacchetto di velocità al pic
@param[in] v puntatore a pacchetto di velocità
@brief Invio pacchetto vel
*/
void	send_vel_packet_2_pic(packet_vel v);	//	send vel packet to pic

/**
* Function able to close (and clean up the memory) the netus2pic communication module 
*/
void	close_robot_comm();
//______________________________________________________________________________
//		operazioni ausiliarie
//______________________________________________________________________________

//		remove thresholds from packet
/**
Funzione che "smaschera" un pacchetto, togliendo le soglie per l'invio su seriale
@param[in] pacchetto_sporco puntatore al pacchetto di velocità con soglie
@param[in] pacchetto_pulito puntatore a pacchetto di velocità senza soglie
@param[in] len_sporco	lunghezza del pacchetto sporco
@return	lunghezza - in byte - del pacchetto pulito
@brief Invio pacchetto vel
*/
int 	pulisci_pacchetto(unsigned char *pacchetto_sporco, unsigned char *pacchetto_pulito, int len_sporco);
//		crc control
/**
Funzione per il check del crc
@param[in] p	puntatore al pacchetto di velocità con soglie
@param[in] len	lunghezza del pacchetto da controllare
@return 	1 se è andata bene
*/
int 	controlla_crc(unsigned char* p, int len);
//		packet printing - debug function
/**
Funzione per la stama a video di un pacchetto
@param[in] slot puntatore al pacchetto
@param[in] l lunghezza del pacchettoo
*/
void 	stampa_pacchetto(unsigned char* slot, int l);		//debug function

//		auxiliar internal function
//-------------------------------------------
/**
@brief	Funzione ausiliaria: se il carattere è al di sotto della soglia della seriale, tale carattere viene sostituito dalla combinazione
CARATTERE_SOGLIA e CARATTERE+CARATTERE_SOGLIA
@return 0
*/
int 	check_soglia(unsigned char* pac);
/**
@brief	Funzione ausiliaria: ottiene la fine di un pacchetto (identificata dal carattere invio)
@return: lunghezza del pacchetto
*/
int 	get_fine_pacchetto(unsigned char* p);	

/**
@brief pic_bufferto nel buffer dell'ultimo pacchetto ricevuto
@param[in] len lunghezza in byte del pacchetto ricevuto
*/
int 	inserisci_in_pic_buffer(int len);	// insert a correct vel into the buffer
/**
@brief	Stampa a video del buffer
*/
void 	show_pic_buffer();					// buffer printing - debug structure
//-------------------------------------------

//char 		ack_packet[]={0x7f, 00, 00 ,00, 04, 00, 04, 01, 0x78 ,0xff,0x0a};
#ifdef __cplusplus
}
#endif

#endif

