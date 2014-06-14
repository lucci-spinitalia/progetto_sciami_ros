#ifndef _ROBOT_STRUCTURE_H
#define _ROBOT_STRUCTURE_H

///\brief	mechanical steps/turn (# of mechanical steps in each complete wheel revolution) 
#define 	NUM_STEPS	200
///\brief	PIC elementary period ([us] (microseconds))
#define 	MICRO_PERIOD	50
///\brief	wheel circumference [cm]
#define 	CIRCONFERENZA_RUOTA		21.5503
///\brief	inter-axis [cm]
#define 	INTERASSE				18.3
///\brief	inter-axis [m]
#define 	INTERASSE_M				0.179
///\brief	wheel radius [cm]
#define 	RAGGIO_RUOTA			3.429836770113272
///\brief	wheel radius [mm]
#define 	RAGGIO	33

/*Equipped sensors section*/
///\brief	numero di canali IR
#define 		NUM_IR			5	//	number of IR sensors
///\brief	numero di canali ACCELEROMETRICI
#define 		NUM_ACC			3	//	number of ACC sensors
///\brief	numero di canali GIROSCOPICI
#define 		NUM_GYRO		2	//	number of GYRO sensors
///\brief	numero di canali BUSSOLA
#define 		NUM_MAGNETO		2	//	number of MAGNETO sensors
/*------------------------*/


/**
*\enum step Enumeration of stepping cycle
*/
enum
{
//Giro elettrico del motore effettuato in 8 passi
    BISTEPPING=2,
//Giro elettrico del motore effettuato in 12 passi
    TRISTEPPING=3
};
#endif