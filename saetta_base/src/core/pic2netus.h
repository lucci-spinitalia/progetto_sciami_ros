#ifndef _LOW_LEVEL_COMMUNICATION_H
#define _LOW_LEVEL_COMMUNICATION_H
#ifdef __cplusplus
extern "C" {
#endif
#define		PIC_LOG_TO_SCREEN	0

#include "../Rs232/rs232.h"
#include "pic_rel.h"
#include "robot_core.h"
#include "sensor_struct.h"

#define		PIC_PACKET_SCREEN_LOG	0
///\brief	flag for a corrupted packet just analyzed
#define 	ERROR_PACKET_ANALYZED	0
///\brief	flag for a start packet just analyzed
#define 	START_PACKET_ANALYZED	1
///\brief	flag for a load packet just analyzed
#define 	LOAD_PACKET_ANALYZED	2

///\brief	code relative to the last parsed packet
unsigned int 	packet_type;

//					OPERAZIONI SU SENSORI
//______________________________________________________________________________
//		get sensors data from a pic packet
///\brief	approximation of the state builded by gyro info
float		gyro_integral=0.0;
///\brief	gyro integral given by the pic
short		gyro_integral_pic=0.0;
///\brief	approximation of the state builded by gyro info
float		gyro_sup_integral=0.0;
///\brief	accumulator for noise misunderstanding gyro
int		accumulatore_gyro=0;
///\brief	riferimento di passi passato nell'operazione di servoing
int		riferimenti_servoing[2]={0,0};
///\brief	riferimento di direzione nell'operazione di servoing
int		direction_servoing[2]={0,0};

/**
Funzione che memorizza da un pacchetto dati le letture dei sensori: tali letture possono costituire un sottopacchetto dell'intero pacchetto
\brief	storage delle letture dei sensori
@param[in] payload puntatore all'inizio del sottopacchetto
@param[in] l  lunghezza del sottopacchetto
@param[in] s puntatore al sensore
@param[out] verifica se è 1 vuol dire che il parsing  è andato a buon fine
@return 	lunghezza del sottopacchetto parsato
*/
int 	get_sensore(unsigned char* payload, int l,sensore s, int *verifica);

/**
Function retrieving gyro measured (integrated over the sampling period) from the GYRO_INTEGRAL_PACKET
\brief	storage delle letture dei sensori
@param[in] payload puntatore all'inizio del sottopacchetto
@param[in] l  lunghezza del sottopacchetto
@param[in] s puntatore al sensore
@param[out] verifica se è 1 vuol dire che il parsing  è andato a buon fine
@return 	lunghezza del sottopacchetto parsato
*/
int 	get_gyro_integral(unsigned char* payload, int l, short *integral, int *verifica);

/**
Funzione che memorizza da un pacchetto dati le letture dei sensori: tali letture possono costituire un sottopacchetto dell'intero pacchetto
\brief	storage delle letture dei sensori
@param[in] payload puntatore all'inizio del sottopacchetto
@param[in] l  lunghezza del sottopacchetto
@param[in] s puntatore al sensore
@param[out] verifica se è 1 vuol dire che il parsing  è andato a buon fine
@return 	lunghezza del sottopacchetto parsato
*/

int	get_magacc(unsigned char* payload, int l, sensore s, int *verifica);
//	get distance data from a distance_packet: measures achieved are in mm
/**
Funzione che memorizza da un pacchetto dati le letture dei sensori: tali letture possono costituire un sottopacchetto dell'intero pacchetto
\brief	storage delle letture dei sensori
@param[in] payload puntatore all'inizio del sottopacchetto
@param[in] l  lunghezza del sottopacchetto
@param[in] s puntatore al sensore
@param[out] verifica se è 1 vuol dire che il parsing  è andato a buon fine
@return 	lunghezza del sottopacchetto parsato
*/

int get_distanza(unsigned char* payload, int l, sensore s, int *verifica);



/**
Funzione che memorizza da un pacchetto dati gli offset dei sensori: tali letture possono costituire un sottopacchetto dell'intero pacchetto
\brief	storage delle letture dei sensori
@param[in] payload puntatore all'inizio del sottopacchetto
@param[in] l  lunghezza del sottopacchetto
@param[in] s puntatore al sensore
@param[out] verifica se è 1 vuol dire che il parsing  è andato a buon fine
@return 	lunghezza del sottopacchetto parsato
*/
int get_offset_sensore(unsigned char* payload, int l, sensore s, int *verifica);


/**
Acquisizione del numero dei passi fatti dagli stepper
\brief	acquisizione numero passi
@param[in] *payload	puntatore al pacchetto dati
@param[in] l			lunghezza del pacchetto
@param[in] *p_m1			passi motore1
@param[in] *p_m2			passi motore2
@param[in] *s1				segno passi motore 1
@param[in] *s2				segno passi motore 2
@param[in] verifica		verifica correttezza pacchetto
@param[out] passi effettuati dal motore sinistro e motore destro
*/
int	get_odometria(unsigned char* payload, int l,  int *p_m1, int *p_m2, int *s1, int *s2, int *verifica);

/**
Log odometria
\brief	Log odometria
@param[in] *fp		puntatore a file
@param[in] *stato	puntatore allo stato
@param[in] *passi	passi effettuati
*/
int	print_odometria(FILE *paux, float *stato, int *passi );

/**
Feedback sul comando di servoing
\brief	Feedback servoing
@param[in] *payload	puntatore al pacchetto dati
@param[in] l			lunghezza del pacchetto
@param[in] *p_m1			passi motore1
@param[in] *p_m2			passi motore2
@param[in] verifica		verifica correttezza pacchetto
@param[out] passi effettuati dal motore sinistro e motore destro
*/
int	get_servoing_feedback(unsigned char* payload, int l,  int *ref_m1, int *ref_m2, int *verifica);

/**
computing of state increment in a T period
\brief	acquisizione numero passi
@param[in]  *p_m1			steps motor 1
@param[in]  *p_m2			steps motor 2
@param[in]  theta			actual bearing
@param[out] stato			robot state
*/
int	get_state(int *p_m1, int *p_m2,float theta, float *delta);

/**
Funzione genera il sottopacchetto relativo alle letture memorizzate nelle strutture dati sensori
\brief	generazione sottopacchetti sensoriali
@param[in] pacchetto puntatore all'inizio del sottopacchetto
@param[in] s puntatore alla struttura sensore
@param[in] header header  relativo al tipo di sensore
@return 	lunghezza del sottopacchetto creato
*/
int 	genera_pacchetto_sensore_standard(unsigned char *pacchetto, struct sensore_t *s, unsigned char header);


/**
Processing di uno stream di dati in input: STA PER SOSTITUIRE processa_pacchetto();
\brief	processing del pacchetto: IN PROCESSING
*/
int	analizza_pacchetto_init(unsigned char* buffer);
int analizza_pacchetto(unsigned char *pic_buffer, unsigned char *buf, int len);
#ifdef __cplusplus
}
#endif

#endif
