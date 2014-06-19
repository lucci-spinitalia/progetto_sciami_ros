#ifndef _LIB_SERIAL_COMM
#define _LIB_SERIAL_COMM
#ifdef __cplusplus
extern "C" {
#endif

/**
@author Maurizio Di Rocco
*/


#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>


pthread_mutex_t m_analizza_pacchetto;
pthread_cond_t cv_analizza_pacchetto;

pthread_mutex_t m_io;	

#define 		FALSE 				0
#define 		TRUE 				1
///\brief		porta seriale del PIC
//#define 		PIC_DEVICE			"/dev/ttyO3"	//	pic physical port
///\brief		velocità della porta seriale del PIC
#define 		BAUDRATE 			B115200			//	port baudrate



//							CONSTANTS
///\brief		header pacchetto di errore ricezione da parte del pic
#define 		PACKET_ERRPIC_HEADER			0x45
///\brief		header pacchetto di velocità
#define 		PACKET_SPEED_HEADER			127
///\brief		header pacchetto IR
#define 		PACKET_IR_HEADER			0x22		///	header pacchetto dati IR
///\brief		header pacchetto ACC
#define 		PACKET_ACC_HEADER			0x23
///\brief		header pacchetto GYRO
#define 		PACKET_GYRO_HEADER			0x24
///\brief		header pacchetto MAGNETO
#define 		PACKET_MAGNETO_HEADER		0x25
///\brief		header pacchetto distance
#define 		PACKET_DISTANCE_HEADER		0x26		///	header pacchetto dati IR
///\brief		header pacchetto ACK
#define 		PACKET_ACK_HEADER			0x41
///\brief		header pacchetto START
#define 		PACKET_START_HEADER			0x53



///\brief		header pacchetto IR
#define 		PACKET_MAGACC_HEADER			0x30		///	header pacchetto dati IR
#define 		PACKET_MAGACC_LENGTH			9		///	header pacchetto dati IR





///\brief		lunghezza pacchetto velocità
#define 		PACKET_ERRPIC_LENGTH		8

///\brief		lunghezza pacchetto velocità
#define 		PACKET_SPEED_LENGTH			10
///\brief		lunghezza pacchetto IR
#define 		PACKET_IR_LENGTH   			13
///\brief		lunghezza pacchetto IR
#define 		PACKET_DISTANCE_LENGTH   	8
///\brief		lunghezza pacchetto ACC
#define 		PACKET_ACC_LENGTH			9
///\brief		lunghezza pacchetto GYRO
#define 		PACKET_GYRO_LENGTH			7
///\brief		lunghezza pacchetto MAGNETO
#define 		PACKET_MAGNETO_LENGTH		7
///\brief		lunghezza pacchetto ACK
#define 		PACKET_ACK_LENGTH					5
///\brief		lunghezza pacchetto START
#define 		PACKET_START_LENGTH			7
///\brief		lunghezza pacchetto DATA
#define 		PACKET_DATA_LENGTH			34
///\brief		soglia sotto la quale occorre mascheramento dati
#define 		PACKET_SOGLIA				0x2E
///\brief		offset di mascheramento
#define			PACKET_OFFSET_MASCHERAMENTO 0x01
///\brief		profondità - in pacchetti - del buffer di ricezione della seriale relativa alla comunicazione col pic
#define 		N_PIC_MES_BUFF				2
///\brief		max lunghezza - in byte - del messaggio su seriale del pic
#define 		MAX_PIC_MES_LENGHT 			300
///\brief		header del pacchetto di servoing
#define 		PACKET_SERVOING_HEADER			128
///\brief		lunghezza pacchetto SERVOING
#define 		PACKET_SERVOING_LENGTH		14
///\brief		header del pacchetto di offset magnetometro
#define 		PACKET_MAG_OFFSET_HEADER			0x42
///\brief		lunghezza pacchetto di offset magnetometro
#define 		PACKET_MAG_OFFSET_LENGTH		7

///\brief		header del pacchetto di offset gyro
#define 		PACKET_GYRO_OFFSET_HEADER			0x42
///\brief		lunghezza pacchetto di offset gyro
#define 		PACKET_GYRO_OFFSET_LENGTH			5

///\brief		header del pacchetto di offset gyro
#define 		PACKET_GYRO_INTEGRAL_HEADER			0x43
///\brief		lunghezza pacchetto di offset gyro
#define 		PACKET_GYRO_INTEGRAL_LENGTH			5



///\brief		header pacchetto starting period
#define 		PACKET_START_PERIOD_HEADER		0x26
///\brief		lunghezza pacchetto starting period
#define 		PACKET_START_PERIOD_LENGTH		3
///\brief		header pacchetto starting period
#define 		PACKET_SERVOING_FEEDBACK_HEADER		0x29
///\brief		lunghezza pacchetto starting period
#define 		PACKET_SERVOING_FEEDBACK_LENGTH		7




///\brief		header pacchetto odometria
#define 		PACKET_ODOM_HEADER				0x28
///\brief		lunghezza pacchetto odometria
#define 		PACKET_ODOM_LENGTH				8


///\brief		comando per l'invio di un pacchetto tale per cui non si inviano comandi ma solo la ricezione dei dati sensoriali
#define 		PACKET_TIMING_HEADER		0x30

///\brief		lunghezza del pacchetto timing
#define 		PACKET_TIMING_LENGTH		3


///\brief	pic file descriptor
int 					pic_fd;
///\brief	pic file descriptor
int 					rfid_fd;

///\brief	flag che viene messo in off alla ricezione dell'interrupt su seriale-pic
int 					flag;			//	flag for communication irq
///\brief	struttura per l'interrupt su seriale-pic
struct 					sigaction saio;     //	aux pic port struct
///\brief	struttura contenenti i vecchi settaggi della porta seriale all'apertura del programma
struct 					termios oldtio;		//	aux pic port struct
///\brief	struttura contenente i nuovi settaggi della porta utilizzati nell'esecuzione del programma
struct 					termios newtio;		//	aux pic port struct


///\brief	buffer per la ricezione dei messaggi da parte del PIC
unsigned char*			pic_message_buffer[N_PIC_MES_BUFF];	//	comm buffer

///\brief	messaggio timing
//unsigned char			pic_message_timing[4]={130,0x7E, 0xFF, 0x0A};	//	comm buffer

///\brief	messaggio timing
//unsigned char			pic_message_reset_steps_acc[4]={131,0x7D, 0xFF, 0x0A};	//	comm buffer


///\brief	indice del pacchetto corrente da analizzare all'interno del buffer di ricezione dei messaggi da parte del PIC
int  					current_pic_packet_slot;			//	firs slot available in com buffer

///\brief	mutex per il flag su seriale
pthread_mutex_t mutex_wait_flag;


/**Funzione chiamata quando si preme ctrl+C
@brief Interrupt handler
@param[out] signum segnale di ricezione
*/
extern void	termination_handler (int signum) ;
/**
Gestore dell'interrupt - sollevato dalla ricezione del carattere '\n' (INVIO) - sulla porta seriale-pic
@brief Gestore dell'interrupt
@brief Interrupt handler
@param[out] signum segnale di ricezione
*/
void	signal_handler_IO (int status);				//	isr  due to 0x0A character reception on serial port
/**
Apertura della porta seriale-pic
@brief Apertura della serialae
*\param[in] tty_dev Descrittore del device seriale
*\return pic_fd Pic file descriptor id
*/
int tty_open(char* tty_dev);								//	opening serial port with interrupt on 0x0A char reception

/**
Apertura della porta seriale-pic: viene settato un interrupt alla ricezione del carattere '\n' (INVIO)
@brief Inizializzazione  della porta seriale-pic
*/
int init_serial_comm();		//	initialization of both physical and data structures serial port
void close_serial_comm();
#ifdef __cplusplus
}
#endif

#endif
