

#include "robot_sensors.h"

//------------------------------------------------------------------------------

void init_sensors()
{
    init_sensore(&gyro, NUM_GYRO);
    //init_sensore(&acc, NUM_ACC);
    init_sensore(&acc, 1);
    init_sensore(&magneto, NUM_MAGNETO);
    init_sensore(&ir, NUM_IR);
    // creazione tabella atan2 post-calibrazione
    //creaTabellaMagneto(&pMag);
	#ifdef	USE_GYRO_TABLE
		load_gyro_table(GYRO_FILE);
	#endif


}
//------------------------------------------------------------------------------


//----------------------------------------------------------------------
void init_sensore(sensore* s, int numero_sensori)
{
	*s = malloc(sizeof(struct sensore_t));
	(*s)->num_canali = numero_sensori;
	(*s)->range = malloc(numero_sensori*sizeof(unsigned int));
	(*s)->bias = malloc(numero_sensori*sizeof(unsigned int));
	(*s)->converted = malloc(numero_sensori*sizeof(int));
}
//----------------------------------------------------------------------

//=========================================================================================================

inline void gyro_zero_setting(unsigned char *pic_buffer, int buffer_size) 
{
  int local_counter = 0;
  int local_value = 0;
  
  while (local_counter < 3000) 
  {   
	  packet_type = analizza_pacchetto_init(pic_buffer, buffer_size);
    
	  if (gyro->is_valid == TRUE) 
    {
	    if (local_value == *(gyro->range + 1)) 
      {
		    local_counter++;
		    printf("lv: %d\n", local_value);
	    } 
      else 
      {
		    local_counter = 0;
		    local_value = *(gyro->range + 1);
	    }
      
	    if(local_counter > 10) 
      {
		    gyro_table_effective_zero = local_value;
		    local_counter = 3001;
	    }
	  }
  }
  
  printf("zero_trovato= %d su %d iterazioni\n", gyro_table_effective_zero, local_counter);
}
//=========================================================================================================


//=========================================================================================================

inline void calibrazione_magneto(unsigned char *pic_buffer, int buffer_size) 
{
  int magneto_flag = 0;
  int v1,v2;
  
  velocita = 2.0;
  angle_degrees = 360.0;
  angle_rad = M_PI * angle_degrees / 180.0;
  calcola_angolo(angle_rad, velocita, &v1, &v2, &num_passi);
  v1 = v1 / modulo(v1)*(1024 - modulo(v1));
  v2 = v2 / modulo(v2)*(1024 - modulo(v2));
  set_pos_2_array(pic_buffer[0], v1, v2, num_passi, num_passi);

  while (flag_servoing_completed != 1) 
  {
	  packet_type = analizza_pacchetto_init(pic_buffer, buffer_size);
  }
}
//=========================================================================================================


//=========================================================================================================

inline void magneto_zero_setting(unsigned char *pic_buffer, int buffer_size) 
{
  int local_counter = 0;
  int local_value = 0;
  int x = 0;
  int y = 0;

  while (local_counter < 16) 
  {
	  packet_type = analizza_pacchetto_init(pic_buffer, buffer_size);
    
	  if(magneto->is_valid == TRUE) 
    {
	    x += *(magneto->range);
	    y += *(magneto->range + 1);
	    ++local_counter;
	  }
  }
  
  x = x >> 4;
  y = y >> 4;

  pMag->nordRad = pMag->tabMag[(x - pMag->tozerox)*(pMag->colonneTabMag)+(y - pMag->tozeroy)];
  printf("Gradi inziali:%d %d %f\n", x, y, pMag->nordRad);
}
//=========================================================================================================

//------------------------------------------------------------------------------
void print_sensore(sensore s)
{
	int i;
	for(i=0;i<s->num_canali;i++)
  {
		//printf("%4u ", ir_distance[1023-*(s->range+i)]);
		printf("%4u ", *(s->range+i));
		//printf("b %4u ", *(s->bias+i));
	}
	printf("\n");
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
void 	print_sensore_on_file(sensore s, FILE* paux){

	int i;
	//	carattere tab
  if (paux != NULL) 
  {
    fprintf(paux, "%c", 0x09);	
     
    for(i=0;i<s->num_canali;i++)
      fprintf(paux, "%4u ", *(s->range+i));
  }
}
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
/*int creaTabellaMagneto(structMag **im) {

    *im = (structMag *)malloc( sizeof( structMag ) );
	if ( *im == NULL)
	{
		printf("creaTabellaMagneto: Non è possibile creare strucMag (out of memory?)...\n");
		return -1;
	}


    FILE        *uscita;
    FILE        *datiMag;

    int a,posizione,minx,miny,maxx,maxy;
    float rad;

    // apertura file datimag
    datiMag=fopen(DATIMAG,"r");
    if(datiMag==NULL) {
	printf("errore apertura file datimag\n");
	return -1;
    }

    // acquisisco i dati max min sul magneto
    fscanf(datiMag,"%d %d %d %d",&minx,&maxx,&miny,&maxy);
    fclose(datiMag);


    // apertura file output
    uscita=fopen(OUTPUT,"r");
    if(uscita==NULL) {
	printf("errore apertura file output\n");
	return -1;
    }

    // riempo i primi 4 campi di structMag e predispongo l'array tabMag
    (*im)->righeTabMag  = maxx-minx+1;
    (*im)->colonneTabMag = maxy-miny+1;
    (*im)->tozerox = minx;
    (*im)->tozeroy = miny;
    (*im)->tabMag = (float*)malloc(sizeof(float)* (*im)->righeTabMag * (*im)->colonneTabMag);

    if ( (*im)->tabMag == NULL)
	{
		printf("creaTabellaMagneto: Non è possibile creare il campo tabMag (out of memory?)...\n");
		return -1;
	}

    //printf("%d %d %d\n",(*im)->righeTabMag,(*im)->colonneTabMag,(*im)->righeTabMag*(*im)->colonneTabMag);

    posizione=0;
    while(fscanf(uscita,"%d %d %f\n",&a,&a,&rad)!=EOF) {
	(*im)->tabMag[posizione]=rad;
	posizione++;
    }


    fclose(uscita);




    return 0;
}*/
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
void distruggiTabellaMagneto( structMag *im ) {

	free( im->tabMag );
	free( im );
    //printf("FATTO");
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
void radMagneto(structMag *im,int magX,int magY, float *angle) {

    float radianti;
	//printf("valori raw: %d %d\n", magX, magY);
    // saturazione valori magnetomero
    if(magX<(im->tozerox))
	magX = im->tozerox;
    if(magX>=(im->tozerox + im->righeTabMag))
	magX = im->tozerox + im->righeTabMag -1;
    if(magY<(im->tozeroy))
	magY = im->tozeroy;
    if(magY>=(im->tozeroy + im->colonneTabMag))
	magY = im->tozeroy + im->colonneTabMag -1;

	//printf("valori raw: %d %d\n", magX, magY);
	//printf("offsetrad: %d %d\n", magX-im->tozerox,magY-im->tozeroy);
	//printf("rad: %d %d\n", magX-im->tozerox,magY-im->tozeroy);
    radianti = 	im->tabMag[(magX-im->tozerox)*(im->colonneTabMag)+(magY-im->tozeroy)]-im->nordRad;
	
    //riporto i valori tra -PI e +PI
    if (radianti > M_PI) {
	  radianti = -2*M_PI + radianti;
    } 
    if (radianti < -M_PI) {
	  radianti = 2*M_PI + radianti;
    }

    //radianti = im->tabMag[(magX-im->tozerox)*(im->colonneTabMag)+(magY-im->tozeroy)];

    *angle=-radianti;
    
}
//------------------------------------------------------------------------------




//=========================================================================================================
int	load_gyro_table(char *file_name){

	int	num_campi;
	int i;
	fp_gyro=fopen(GYRO_FILE, "r");
        if (fp_gyro != NULL) {
            fscanf(fp_gyro, "%d", &gyro_table_min_index);
            fscanf(fp_gyro, "%d", &gyro_table_max_index);
            fscanf(fp_gyro, "%d", &gyro_table_nominal_zero);
            num_campi = gyro_table_max_index-gyro_table_min_index;
            gyro_table_value = malloc(num_campi*sizeof(float));
            for(i=1; i<num_campi; i++){fscanf(fp_gyro, "%f\n",  (gyro_table_value+i));}
        } else {
            printf("Error opening gyro: gyro_table.txt");
        }

}
//=========================================================================================================


//=========================================================================================================
void create_magneto_table(){

	int x,y,maxx,maxy,minx,miny;
	float offsetx,offsety,fattnormx,fattnormy,i,j,rad;
	maxx 		= 	0;
	maxy 		= 	0;
	minx 		= 	2000;
	miny 		= 	2000;
        
        fpm_log = fopen(MAGNETO_LOG,"r");
        if (fpm_log == NULL) {
            printf("Error opening magneto_rot.txt\n");
        }
        else {
            fpm_table = fopen(MAGNETO_TABLE, "w");
            if (fpm_table == NULL) {
                printf("Error opening magneto_table.txt\n");
            } else {
                    while(fscanf(fpm_log,"%d %d\n",&x,&y)!=EOF){
                            if (x>-1 && y>-1) {
                                    if (x>maxx){maxx = x;}
                                    if (x<minx){minx = x;}
                                    if (y>maxy){maxy = y;}
                                    if (y<miny){miny = y;}
                            }
                    }
                    offsetx		=	(maxx+minx)/2.0;
                    offsety		=	(maxy+miny)/2.0;
                    fattnormx	=	maxx-offsetx;
                    fattnormy	=	maxy-offsety;

                    fprintf(fpm_table,"%d %d %d %d\n",minx,maxx,miny,maxy);
                    for(i=-fattnormx;i<=fattnormx;i++) {
                            for(j=-fattnormy;j<=fattnormy;j++) {
                                    rad = atan2(j/fattnormy,i/fattnormx);
                                    fprintf(fpm_table,"%d %d %f\n",(int)(i+offsetx),(int)(j+offsety),rad);
                            }
                    }
                    fclose(fpm_table);
            }
            fclose(fpm_log);
        }
        
}
//=========================================================================================================




//=========================================================================================================
int magneto_table_2_struct(structMag **im) {
    
    fpm_table =	fopen(MAGNETO_TABLE, "r");
    if (fpm_table != NULL) {
        *im = (structMag *)malloc( sizeof( structMag ) );
            if ( *im == NULL)
            {
                    printf("creaTabellaMagneto: Non è possibile creare strucMag (out of memory?)...\n");
                    return -1;
            }
        int a,posizione,minx,miny,maxx,maxy;
        float rad;
        
        fscanf(fpm_table,"%d %d %d %d",&minx,&maxx,&miny,&maxy);

        (*im)->righeTabMag  	=	maxx-minx+1;
        (*im)->colonneTabMag 	=	maxy-miny+1;
        (*im)->tozerox 			=	minx;
        (*im)->tozeroy 			=	miny;
        (*im)->tabMag 			=	(float*)malloc(sizeof(float)* (*im)->righeTabMag * (*im)->colonneTabMag);
            //printf("OFFSET  RILEVATI: %d %d\n",(*im)->tozerox,(*im)->tozeroy);
        if ( (*im)->tabMag == NULL)
            {
                    printf("creaTabellaMagneto: Non è possibile creare il campo tabMag (out of memory?)...\n");
                    return -1;
            }

        //printf("%d %d %d\n",(*im)->righeTabMag,(*im)->colonneTabMag,(*im)->righeTabMag*(*im)->colonneTabMag);

        posizione=0;
        while(fscanf(fpm_table,"%d %d %f\n",&a,&a,&rad)!=EOF) {
            (*im)->tabMag[posizione]=rad;
            posizione++;
        }


        fclose(fpm_table);

        //imposto la posizione iniziale a -100 (posizione iniziale non ancora salvata)
        //(*im)->nordRad = -100;
    } else {
        printf("Error opening magneto_table.txt\n");
    }

    return 0;
}
//=========================================================================================================

void close_sensors()
{
    
}












