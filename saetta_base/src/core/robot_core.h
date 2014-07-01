#ifndef _ROBOT_H_
#define _ROBOT_H_
#ifdef __cplusplus
extern "C" {
#endif

//====================================
//				Include
//#include 	<tgmath.h>
#include	<math.h>
#include 	"funzioni_ausiliarie.h"
#include	"robot_sensors.h"
#include	"robot_hardware.h"
#include 	"robot_comm.h"
#include	"pic_rel.h"
#include	"pic2netus.h"
//====================================


//====================================
//				Define

///\brief	posizione della variabile X dello stato del robot
#define 		STATE_X			0
///\brief	posizione della variabile X dello stato del robot
#define 		STATE_Y			1
///\brief	posizione della variabile X dello stato del robot
#define 		STATE_THETA		2


///\brief	max linear vel (cm/s)
#define                 MAX_LIN_VEL             10 // 16.0
#define                 MAX_LIN_VEL_EVO         20.0
#define                 MAX_LIN_VEL_EVO_GIO     25.0//16.0//32.0 //50.0

///\brief	max turn rate (rad/s)
#define                 MAX_TURN_RATE           0.8// 1.0
#define                 MAX_TURN_RATE_EVO	0.2
#define                 MAX_TURN_RATE_EVO_GIO	0.70

///\brief	Se settato a 1 abilita il log su file: ricorda di NON COMMENTARE
#define                 LOG_SU_FILE		0

/*******************************************/

///\brief	TO USE DATA FROM THE GYRO LOOKUPTABLE
//#define                 USE_GYRO_TABLE		1

//====================================

//====================================
//		Global variables declaration
///\brief	puntatore al file per il log dei dati da sensore
FILE* fp_log;
///\brief	mutex del puntatore a file
pthread_mutex_t mutex_fp;

///\brief	numero di passi fatti dai due motori
int steps_done[2];
///\brief	incremento stato del robot
float delta_state[3] = {0, 0, 0};
///\brief	stato del robot
float state[3] = {0, 0, 0};

///\brief	passi fatti dai motori ad ogni T
int delta_passi[2] = {0, 0};
///\brief	verso dei passi fatti dai motori ad ogni T
int sign_motors[2] = {0, 0};
///\brief	anomaly steps counter
int steps_anomaly = 0;
///\brief	linear vel m1 (cm/s)
float v_m1 = 0;
///\brief	linear vel m2 (cm/s)
float v_m2 = 0;
///\brief	linear vel m1 (steps)
int pulse_m1 = 0;
///\brief	linear vel m2 (steps)
int pulse_m2 = 0;
///\brief	VARIABILR AD MINCHIAM
int ad_minc = 0;
///\brief       Last set linear speed
float last_v_ref=0.0;
///\brief       Last set angular speed
float last_w_ref=0.0;
///\brief       Maximum amount of linear acceleration
const float max_lin_acc=18;//24;
///\brief       Maximum amount of angular acceleration
const float max_ang_acc=2;//1.3114;
///\brief	mutex for locking robot state buffer
pthread_mutex_t mutex_state;

///\brief	 variabile ausiliaria per la moltiplicazione dei byte + e - significativi
int moltiplicatore[2] = {1, 256}; //	aux
///\brief	variabile ausiliaria per la moltiplicazione con segno
int segno[2] = {1, -1}; //	aux


//====================================

//====================================
//		Functions declaration
///\brief	inizializzazione del modulo robot
int init_robot(char* portname);

///\brief	loop principale del robot
int robot_loop(int time_us);

///\brief	chiusura del modulo robot
void close_robot();
//					OPERAZIONI DI CONTROLLO
//______________________________________________________________________________

/**
Dato un vettore in forma polare calcolo le velocità dei due motori
\brief	calcolo delle velocità dei motori da un vettore in forma polare
@param[in] V_D modulo di velocità in m/s
@param[in] W_D fase della velocità rad/s
@param[out] *M1 puntatore alla vel del motore 1: ritorna la vel in cm/s !
@param[out] *M2 puntatore alla vel del motore 2: ritorna la vel in cm/s !
*/
//void get_vel_motori_evolution(float *V_D, float *W_D, float *M1, float* M2);

//void get_vel_motori_evolution_gio(float *V_D, float *W_D, float *M1, float* M2);
/**
Dato un vettore in forma polare calcolo le velocità dei due motori
\brief	calcolo delle velocità dei motori da un vettore in forma polare
@param[in] V_D modulo di velocità in m/s
@param[in] W_D fase della velocità rad/s
@param[out] *M1 puntatore alla vel del motore 1: ritorna la vel in cm/s !
@param[out] *M2 puntatore alla vel del motore 2: ritorna la vel in cm/s !
*/
void 	get_vel_motori_constant_ratio(float *V_D, float *W_D, float *M1, float* M2);



/**
Calcolo del numero di passi per il compimento di un rettilineo
\brief	generazione riferimenti per moto rettilineo
@param[in] distanza distanza da compiere in mm
@param[in] velocità di asservimento in cm/s
@param[in] modalita_stepping modalita di movimentazione (deve essere in accordo con quella del PIC)
@param[out] num_passi numero di passi da far effettuare ai motori
@param[out] vel_pic velocità di asservimento
*/
void genera_rettilineo(float distanza, float vel, int modalita_stepping, unsigned int* num_passi, int *vel_pic);


/**
Calcolo del riferimento in velocità da passare al PIC
\brief	generazione riferimenti per moto rettilineo
@param[in] velocità di asservimento in cm/s
@param[in] modalita_stepping modalita di movimentazione (deve essere in accordo con quella del PIC)
@param[out] vel_pic velocità di asservimento
*/
//void calcola_velocita_evolution(float velocita, int modalita_stepping, int *vel_pic);
void calcola_velocita(float velocita, int modalita_stepping, int *vel_pic);

/**
Calcolo del riferimento in velocità da passare al PIC
\brief	generazione riferimenti per moto rettilineo
@param[in] distanza distanza da compiere in mm
@param[in] modalita_stepping modalita di movimentazione (deve essere in accordo con quella del PIC)
@param[out] num_passi numero di passi da far effettuare ai motori
*/
void calcola_passi(float distanza, int modalita_stepping, int *num_passi);

/**
Comandi per la generazione di una circonferenza
\brief	generazione riferimenti per moto circolare uniforme
@param[in] velocita	velocita' lineare del moto circolare uniforme
@param[in] raggio raggio della circonferenza
@param[out] v1 velocità del motore 1
@param[out] v2 velocità del motore 2
*/
void calcola_circonferenza(float velocita, float raggio, int *v1, int *v2);

/**
Comandi per la rotazione su se stesso
\brief	rotazione su se stesso
@param[in] velocita	velocita' lineare del moto circolare uniforme
@param[in] raggio raggio della circonferenza
@param[out] v1 velocità del motore 1
@param[out] v2 velocità del motore 2
@param[out] num_passi passi da far compiere alla due ruote (egual numero in contro fase)
*/
void calcola_angolo(float angolo, float velocita, int *v1, int *v2, int *num_passi);

/**
Controllore di posizione proporzionale anolonomo a pseudo inversa
\brief	controllore proporzionale con parte lineare/angolare accoppiata  (pseudoinversa)
@param[in]  *state			stato del robot nel frame rispetto al quale si vuole fare il controllo
@param[in]  *goal			posizione da raggiungere nel frame rispetto al quale si vuole fare il controllo
@param[in] 	k_v				guadagno parte lineare
@param[in] 	k_w				guadagno parte angolare
@param[out] v_return		controllo lineare
@param[out] w_return		controllo angolare
@param[out]	int		return a code: -1 smthg wrong , 0 ok, 1 the error norm constraint has been satisfied
*/
int cartesian_controller(float *state_r, float *goal_r, float k_v, float k_w, float *v_return, float *w_return);

/**
Controllore di posizione proporzionale anolonomo a pseudo inversa
\brief	controllore proporzionale con parte lineare/angolare accoppiata  (pseudoinversa)
@param[in]  *state			stato del robot nel frame rispetto al quale si vuole fare il controllo
@param[in]  *goal			posizione da raggiungere nel frame rispetto al quale si vuole fare il controllo
@param[in] 	k_v				guadagno parte lineare
@param[in] 	k_w				guadagno parte angolare
@param[out] v_return		controllo lineare
@param[out] w_return		controllo angolare
@param[out]	int		return a code: -1 smthg wrong , 0 ok, 1 the error norm constraint has been satisfied
*/
int pose_controller(float *error, float k_v, float k_w, float *v_return, float *w_return);

/**
Controllore di posizione proporzionale anolonomo a pseudo inversa
\brief	controllore proporzionale con parte lineare/angolare accoppiata  (pseudoinversa)
@param[in]  *state			stato del robot nel frame rispetto al quale si vuole fare il controllo
@param[in]  *goal			posizione da raggiungere nel frame rispetto al quale si vuole fare il controllo
@param[in] 	k_v				guadagno parte lineare
@param[in] 	k_w				guadagno parte angolare
@param[in] 	k_3				guadagno
@param[out] v_return		controllo lineare
@param[out] w_return		controllo angolare
*/
int posture_controller(float *state_r, float *goal_r, float k_v, float k_w, float k_3, float *v_return, float *w_return);

/**
Position controller: position refs are achieved to perform decoupled set points for the
linear and angular dynamics
\brief	controllore proporzionale con parte lineare/angolare accoppiata  (pseudoinversa)
@param[in]  *state_r			stato del robot nel frame rispetto al quale si vuole fare il controllo
@param[in]  *goal_r			posizione da raggiungere nel frame rispetto al quale si vuole fare il controllo
@param[out] set_lin_return		linear setpoint ref
@param[out] set_w_return		angular setpoint ref
*/
int decoupled_controller(float *state_r, float *goal_r, float *set_lin_return, float *set_w_return);

/**
*\brief Function to get the robot odometry.
*This function returns the robot odometry reading in a thread-safe fashion.
*\param[out] robot_state Robot odometry (n.b. the float vector must be pre-allocated (dimension=sizeof(float)*3))
*/
void get_robot_state(float **robot_state);

/**
*\brief Function to set the robot speed (either angular or linear).
* This function has to execute a linear and angular setting command. The parameters are passed as pointers because,
* due to saturation effects, this function may modify their values. 
* \param[in] linear_speed the linear speed to be set
* \param[in] angular_speed the angular speed to be set 
*/
void set_robot_speed(float *linear_speed,float *angular_speed);


void step2vel(int step1, int step2, float *vref, float *wref);
#ifdef __cplusplus
}
#endif

#endif
