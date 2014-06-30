#ifndef _SENSORI_H_
#define _SENSORI_H_
//	SENSORI

/** 
File per la gestione dei sensori
@author Maurizio Di Rocco
*/

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../Rs232/rs232.h"
#include "pic2netus.h"
#include "sensor_struct.h"

//relativo a creaTabellaMagneto
///\brief file risultato della fase di calibrazione magnetometro (tabella)
//#define OUTPUT         "/home/panda/.ros/saetta_base/output.txt"
//#define OUTPUT         "output.txt"

//relativo a creaTabellaMagneto
///\brief file risultato della fase di calibrazione magnetometro (max min)
//#define DATIMAG        "/home/panda/.ros/saetta_base/datimag.txt"
//#define DATIMAG        "datimag.txt"

float 		distanza, velocita;
int   		angle_degrees;
float 		angle_rad;
unsigned int 	num_passi;

///\brief minimum index of the gyro lookup table
int	gyro_table_min_index;
///\brief maximum index of the gyro lookup table
int	gyro_table_max_index;
///\brief maximum index of the gyro lookup table
int	gyro_table_nominal_zero;
///\brief maximum index of the gyro lookup table
int	gyro_table_effective_zero;
///\brief 	puntatore al file per il log dei dati del solo sensore magneto relativo ad una rotazione completa
FILE* fpm_log;
///\brief 	puntatore al file per la lookup table del magneto
FILE* fpm_table;

///\brief filename of the magneto self-rotation
//#define MAGNETO_LOG "/home/panda/.ros/saetta_base/magneto_rot.txt"
#define MAGNETO_LOG "log_files/magneto_rot.txt"

///\brief filename of the magneto self-rotation
//#define MAGNETO_TABLE "/home/panda/.ros/saetta_base/magneto_table.txt"
#define MAGNETO_TABLE "tables/magneto_table.txt"

///\brief filename of the gyro lookup table
//#define GYRO_FILE "/home/panda/.ros/saetta_base/gyro_table.txt"
#define GYRO_FILE "tables/gyro_table.txt"

///\brief file pointer of the gyro lookup table
FILE *fp_gyro;
///\brief pointer to the gyro lookup table values
float *gyro_table_value;


///\brief	ir distance look-up table
static int	ir_distance[1024]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0												, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 151, 152, 153, 154, 156, 157, 158, 159, 161, 161, 163, 164, 164, 166, 167, 168, 169, 169, 170, 171, 172, 173, 174, 174, 176, 176, 177, 178, 179, 179, 180, 181, 182, 183, 183, 184, 185, 186, 186, 187, 188, 189, 189, 190, 191, 191, 192, 193, 193, 194, 195, 196, 196, 197, 198, 198, 199, 199, 200, 201, 201, 202, 203, 203, 204, 204, 205, 206, 206, 207, 208, 208, 209, 209, 210, 211, 211, 212, 213, 213, 214, 214, 215, 216, 216, 217, 218, 218, 219, 219, 220, 221, 221, 222, 223, 223, 224, 224, 224, 225, 226, 226, 227, 228, 228, 229, 229, 230, 231, 231, 232, 233, 233, 233, 234, 234, 235, 236, 236, 237, 238, 238, 239, 239, 240, 240, 241, 241, 242, 243, 243, 244, 244, 245, 246, 246, 246, 247, 248, 248, 249, 249, 250, 251, 251, 252, 252, 253, 253, 254, 254, 255, 256, 256, 257, 258, 258, 258, 259, 259, 260, 261, 261, 262, 263, 263, 264, 264, 264, 265, 266, 266, 267, 268, 268, 269, 269, 270, 270, 271, 271, 272, 273, 273, 274, 274, 275, 276, 276, 277, 277, 278, 278, 279, 279, 280, 281, 281, 282, 283, 283, 284, 284, 284, 285, 286, 286, 287, 288, 288, 289, 289, 290, 291, 291, 292, 293, 293, 293, 294, 294, 295, 296, 296, 297, 298, 298, 299, 299, 300, 301, 301, 302, 303, 303, 304, 304, 305, 306, 306, 307, 307, 308, 308, 309, 309, 310, 311, 311, 312, 313, 313, 314, 314, 315, 316, 316, 317, 318, 318, 319, 319, 320, 321, 321, 322, 323, 323, 324, 324, 325, 326, 326, 327, 328, 328, 329, 330, 331, 331, 332, 333, 333, 334, 334, 335, 336, 336, 337, 338, 338, 339, 339, 340, 341, 341, 343, 343, 344, 344, 345, 346, 346, 347, 348, 348, 349, 350, 351, 351, 352, 353, 353, 354, 354, 355, 356, 357, 358, 358, 359, 359, 360, 361, 362, 363, 363, 364, 364, 365, 366, 367, 368, 368, 369, 369, 371, 371, 372, 373, 373, 374, 375, 376, 376, 377, 378, 379, 379, 380, 381, 382, 383, 383, 384, 385, 386, 386, 387, 388, 389, 389, 391, 391, 392, 393, 394, 394, 395, 396, 397, 398, 398, 399, 400, 401, 402, 403, 403, 405, 405, 406, 407, 408, 408, 410, 410, 411, 412, 413, 414, 415, 415, 416, 417, 418, 419, 420, 421, 421, 423, 423, 424, 425, 426, 427, 428, 429, 430, 431, 431, 432, 433, 434, 435, 436, 437, 438, 439, 440, 441, 441, 443, 443, 445, 445, 446, 448, 448, 450, 450, 451, 452, 453, 454, 455, 456, 457, 458, 459, 460, 461, 462, 463, 465, 465, 466, 467, 468, 470, 470, 471, 473, 473, 475, 476, 476, 478, 479, 480, 481, 482, 483, 484, 485, 486, 488, 488, 490, 491, 492, 493, 494, 495, 496, 498, 498, 500, 501, 502, 503, 504, 505, 506, 508, 509, 510, 511, 512, 513, 515, 516, 517, 518, 520, 521, 522, 523, 524, 525, 526, 528, 529, 530, 531, 533, 534, 535, 536, 538, 539, 540, 541, 543, 544, 545, 546, 548, 549, 550, 552, 553, 555, 556, 557, 558, 560, 561, 562, 564, 565, 566, 568, 569, 570, 572, 573, 575, 576, 577, 579, 580, 581, 583, 585, 586, 587, 589, 590, 591, 593, 595, 596, 597, 599, 600, 602, 603, 605, 606, 608, 610, 611, 613, 614, 615, 617, 618, 620, 622, 623, 625, 626, 628, 630, 631, 633, 635, 636, 638, 640, 641, 643, 645, 646, 648, 650, 652, 653, 655, 657, 659, 661, 663, 665, 666, 668, 670, 672, 673, 676, 678, 680, 682, 683, 685, 687, 690, 692, 693, 695, 698, 700, 702, 704, 706, 708, 710, 713, 715, 717, 720, 722, 724, 727, 729, 732, 734, 737, 739, 742, 744, 747, 749, 752, 755, 757, 760, 763, 766, 768, 772, 775, 778, 781, 784, 787, 790, 793, 797, 800, 803, 807, 810, 813, 817, 821, 825, 828, 832, 836, 840, 844, 848, 852, 856, 860, 864, 868, 873, 877, 882, 886, 890, 895, 899, 904, 909, 913, 917, 922, 927, 931, 936, 940, 945, 950, 954, 959, 964, 969, 973, 978, 983, 987, 992, 997, 1002, 1007, 1012, 1017, 1022, 1027, 1032, 1037, 1042, 1048, 1054, 1059, 1065, 1070, 1077, 1084, 1090, 1097, 1104, 1112, 1120, 1129, 1139, 1149, 1161, 1173, 1186, 1200, 1212, 1224, 1234, 1243, 1252, 1259, 1266, 1272, 1278, 1284, 1398, 1294, 1299, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000};

///\brief puntatore alla struttura di tipo structMag che contiene i dati post-calibrazione del magnetometro
structMag *pMag;

//flag utile per la calibrazione magnetometro
int flag_servoing_completed = 0;

///\brief	puntatore a sensore gyro
sensore 	gyro;			//	gyro sensor
///\brief	mutex sensore gyro
pthread_mutex_t mutex_gyro;		//	gyro sensor
///\brief	puntatore a sensore ir
sensore	  		ir;			//	ir sensor
///\brief	mutex sensore ir
pthread_mutex_t mutex_ir;		//	gyro sensor
///\brief	puntatore a sensore acc
sensore  		acc;		//	acc sensor
///\brief	mutex sensore acc
pthread_mutex_t mutex_acc;		//	gyro sensor
///\brief	puntatore a sensore magneto
sensore 		magneto;	//	magneto sensor
///\brief	puntatore a sensore magneto
pthread_mutex_t mutex_magneto;	//	magneto sensor

///\brief	inizializzazione della parte sensoriale
void init_sensors();

/**Inizializzazione del sensore
@param[out] s puntatore alla struttura sensore
@param[in] numero_sensori numero di canali analogici da campionare
*/
void 	init_sensore(sensore* s, int numero_sensori); // sensor initialization
/**Stampa a video del sensore
@param[in] s puntatore alla struttura
*/
void 	print_sensore(sensore s);				//	debug function
/**Stampa su file del sensore
@param[in] s puntatore al sensore
@param[in] paux puntatore al file di log
*/
void 	print_sensore_on_file(sensore s, FILE* paux);				//	debug function

/**
Funzione che genera i risultati di atan2 in forma tabulare a partire dai dati della calibrazione
\brief generazione tabella atan2 (structMag tabMag)
@param[in] im puntatore che punta all'indirizzo di memoria della struct contenete i dati del magneto
*/
int magneto_table_2_struct(structMag **im);

/**
Funzione che libera lo spazio precedentemente riservato per i dati relativi alla calibrazione del magnetometro
\brief eliminazione della struct StructMag tabMag
@param[in] im puntatore alla struct StructMag
*/
void distruggiTabellaMagneto( structMag *im );



/**
Function that creates a lookup table for the magneto sensor
\brief magneto lookup table
*/
void	create_magneto_table();

/**
Funzione accede alla tabella atan2 structMag e fornisce i radianti corrispondenti (comprende saturazione)
\brief fornisce i radianti corrispondenti ai valori del magnetometro
@param[in] im puntatore alla struct StructMag
@param[in] magX valore su x del magnetometro
@param[in] magY valore su y del magnetometro
@return     radianti corrispondenti
*/
void radMagneto(structMag *im,int magX,int magY, float *angle);


/**
Retrieving function for the gyro lookup table
\brief gyro lookup table data
@param[in] file_name pointer to the file name of data
*/
int load_gyro_table(char *file_name);
/**
Function that creates a struct for the magneto table considering offsets
\brief gyro lookup table struct
@param[in] file_name pointer to the file name of data
*/
int creaTabellaMagneto(structMag **im);

/**
*TODO
*/
inline void gyro_zero_setting(unsigned char *pic_buffer);
/**
*TODO
*/
inline void calibrazione_magneto(unsigned char *pic_buffer);
/**
*TODO
*/
inline void magneto_zero_setting(unsigned char *pic_buffer);

void close_sensors();
#ifdef __cplusplus
}
#endif

#endif
