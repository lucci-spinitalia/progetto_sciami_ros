#ifndef _SENSOR_STRUCT_
#define _SENSOR_STRUCT_
//								STRUCTURES
#ifdef __cplusplus
extern "C" {
#endif

/**
\brief   general fox sensor representation
*/
struct sensore_t{
	unsigned int 		num_canali;		//!\brief numero di canali analogici
	unsigned int*		range;			//!\misura rozza
	float*				converted;		//! misura convertita
	unsigned int*				bias;			//! bias di conversione
	float 				d2c;			//!\brief fattore di conversione D/A
	unsigned int 		is_valid;		//!\brief flag di convalida circa il contenuto della struttura
	
};
/// puntatore alla struttura
typedef struct sensore_t* sensore;


/**
\brief struttura che contiene dati post-calibrazione magnetometro
*/
typedef struct {
  int      righeTabMag;        ///< numero di righe della tabella tabMag (numero di valori su x)
  int      colonneTabMag;      ///< numero di righe della tabella tabMag (numero di valori su x)
  int      tozerox;		///< scalatura valore corrente-> array per la x
  int      tozeroy;		///< scalatura valore corrente-> array per la y
  float*   tabMag;		///< tabella contenente la corrispondenza atan2(x/y) inclusa normalizzazione
  float    nordRad;		///< radianti corrispondenti al nord

} structMag;
#ifdef __cplusplus
}
#endif

#endif
