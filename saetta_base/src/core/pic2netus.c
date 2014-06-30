#include "pic2netus.h"


// To check the content DEBUG ONLY
void debug_char(unsigned char *p, int len) {
	int i;
	
	for( i=0; i<len; i++)
		printf("%02x ",p[i]);
	
	printf("\n");
}


//------------------------------------------------------------------------------
int get_sensore(unsigned char* payload, int l, sensore s, int *verifica)
{
    unsigned short int contatore = 0;
    contatore += *payload;
    int i, j;
    j = 1;
    for (i = 0; i < s->num_canali; i++)
    {
		s->range[i] = 0;
    }
    for (i = 1; i < l - 2; i++)
    {
		
		if (*(payload + j) == PACKET_SOGLIA)
		{
			j++;
			*(payload + j) -= (PACKET_SOGLIA + PACKET_OFFSET_MASCHERAMENTO);
		}
		
		contatore += *(payload + j);
		//printf("C: %02x CONTATORE: %02x\n",*(payload+j), contatore);
		//s->range[i/2]+=256*(i%2)*(*(payload+j));
		*(s->range + ((i - 1) / 2)) = *(s->range + ((i - 1) / 2))+(*(payload + j)) * moltiplicatore[(i - 1) % 2];
		
		j++;
    }
    //printf("CONTATORE1: %02x\n", contatore);
    contatore = ~contatore;
	
    contatore++;
	
    //printf("CONTATORE2: %02x\n", contatore);
    //for (i = 0; i < 2; i++)
	for (i = 0; i < 1; i++)
    {
		if (*(payload + j) == PACKET_SOGLIA)
		{
			j++;
			*(payload + j) -= (PACKET_SOGLIA + PACKET_OFFSET_MASCHERAMENTO);
		}
		//printf("CONTATORE: %d\n", contatore);
		//contatore -= (*(payload + j) * moltiplicatore[i]);
		contatore =   (contatore & 255)-*(payload + j);
		j++;
    }
    //printf("CONTATORE FINALE: %02x\n", contatore);
    s->is_valid = (contatore == 0);

    *verifica &= (contatore == 0);
	
    return j;
}
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
int 	get_gyro_integral(unsigned char* payload, int l, short *integral, int *verifica){
	
	
    unsigned short int contatore = 0;
    contatore += *payload;
    int i, j;
    j = 1;

	*integral=0;
    for (i = 1; i < l - 2; i++)
    {
		
		if (*(payload + j) == PACKET_SOGLIA){
			j++;
			*(payload + j) -= (PACKET_SOGLIA + PACKET_OFFSET_MASCHERAMENTO);
		}

		contatore += *(payload + j);

		*integral= (*integral) +(*(payload + j)) * moltiplicatore[(i - 1) % 2];
		
		j++;
    }

    contatore = ~contatore;
	
    contatore++;

    for (i = 0; i < 1; i++)
    {
		if (*(payload + j) == PACKET_SOGLIA)
		{
			j++;
			*(payload + j) -= (PACKET_SOGLIA + PACKET_OFFSET_MASCHERAMENTO);
		}

		contatore =   (contatore & 255)-*(payload + j);
		j++;
    }
    *verifica &= (contatore == 0);
    return j;
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
int get_magacc(unsigned char* payload, int l, sensore s, int *verifica)
{
    unsigned short int contatore = 0;
    contatore += *payload;
    int i, j;
    j = 1;
	// acquisition from magneto
    for (i = 0; i < 2; i++)
    {
		magneto->range[i] = 0;
    }
	acc->range[0]=0;
    for (i = 1; i < l - 4; i++)
    {
		
		if (*(payload + j) == PACKET_SOGLIA)
		{
			j++;
			*(payload + j) -= (PACKET_SOGLIA + PACKET_OFFSET_MASCHERAMENTO);
		}
		
		contatore += *(payload + j);
		//printf("C: %02x CONTATORE: %02x\n",*(payload+j), contatore);
		//s->range[i/2]+=256*(i%2)*(*(payload+j));
		*(magneto->range + ((i - 1) / 2)) = *(magneto->range + ((i - 1) / 2))+(*(payload + j)) * moltiplicatore[(i - 1) % 2];
		
		j++;
    }
	// acquisition acc
	
    
    for (i = 1; i < 3; i++)
    {
		
		if (*(payload + j) == PACKET_SOGLIA)
		{
			j++;
			*(payload + j) -= (PACKET_SOGLIA + PACKET_OFFSET_MASCHERAMENTO);
		}
		
		contatore += *(payload + j);
		*(acc->range + ((i - 1) / 2)) = *(acc->range + ((i - 1) / 2))+(*(payload + j)) * moltiplicatore[(i - 1) % 2];
		
		j++;
    }
	
    contatore = ~contatore;
	
    contatore++;
	
	for (i = 0; i < 1; i++)
    {
		if (*(payload + j) == PACKET_SOGLIA)
		{
			j++;
			*(payload + j) -= (PACKET_SOGLIA + PACKET_OFFSET_MASCHERAMENTO);
		}

		contatore =   (contatore & 255)-*(payload + j);
		j++;
    }

    magneto->is_valid = (contatore == 0);
	acc->is_valid = (contatore == 0);

    *verifica &= (contatore == 0);
	
    return j;
}
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
int get_distanza(unsigned char* payload, int l, sensore s, int *verifica)
{
    unsigned short int contatore = 0;
    contatore += *payload;
    int i, j;
    j = 1;

    for (i = 0; i < s->num_canali; i++)
    {
		s->range[i] = 0;
    }
    for (i = 0; i < l - 3; i++)
    {
		
		if (*(payload + j) == PACKET_SOGLIA)
		{
			j++;
			*(payload + j) -= (PACKET_SOGLIA + PACKET_OFFSET_MASCHERAMENTO);
		}
		
		contatore += *(payload + j);
		*(s->range + i) = *(payload+j);
		
		j++;
    }
    contatore = ~contatore;
    contatore++;

	for (i = 0; i < 1; i++)
    {
		if (*(payload + j) == PACKET_SOGLIA)
		{
			j++;
			*(payload + j) -= (PACKET_SOGLIA + PACKET_OFFSET_MASCHERAMENTO);
		}

		contatore =   (contatore & 255)-*(payload + j);
		j++;
    }

    s->is_valid = (contatore == 0);
    *verifica &= (contatore == 0);
	
    return j;
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------

int get_odometria(unsigned char* payload, int l, int *p_m1, int *p_m2, int *s1, int *s2, int *verifica)
{
	
    unsigned short int contatore = 0;
    contatore += *payload;
    int i, j;
    j = 1;
    *p_m1 = 0;
    *p_m2 = 0;
	
    for (i = 1; i < l - 3; i++)
    {
		
		if (*(payload + j) == PACKET_SOGLIA)
		{
			j++;
			*(payload + j) -= (PACKET_SOGLIA + PACKET_OFFSET_MASCHERAMENTO);
		}
		contatore += *(payload + j);
		if (i < 3)
			*p_m1 += (*(payload + j) << (((i - 1) % 2)*8));
		else
			*p_m2 += (*(payload + j) << (((i - 1) % 2)*8));
		
		j++;
    }
	
    if (*(payload + j) == PACKET_SOGLIA)
    {
		  j++;
		  *(payload + j) -= (PACKET_SOGLIA + PACKET_OFFSET_MASCHERAMENTO);
    }
    
    contatore += *(payload + j);
	
    *s1 = segno[((*(payload + j))&2) >> 1];
    *s2 = segno[(*(payload + j))&1];

    j++;

#if(PIC_LOG_TO_SCREEN == 1)
    printf("Pack: %d %d\n", *p_m1,*p_m2);
#endif
    contatore = ~contatore;
	
    contatore++;

    for (i = 0; i < 1; i++)
    {
		if (*(payload + j) == PACKET_SOGLIA)
		{
			j++;
			*(payload + j) -= (PACKET_SOGLIA + PACKET_OFFSET_MASCHERAMENTO);
		}

		contatore&=255;
		contatore =   contatore-(int)(*(payload + j));
		j++;
    }

    *verifica = (contatore == 0);

    if(*verifica!=1)
    {
      printf("FUCK ---------------------------\n");
    }

    return j;
	
	
}

int print_odometria(FILE *paux, float *stato, int *passi)
{
    //	carattere tab
    //fprintf(paux, "\n");
    fprintf(paux, "%c", 0x09);
    fprintf(paux, "%f %f %f", *(stato + STATE_X), *(stato + STATE_Y), *(stato + STATE_THETA));
    fprintf(paux, "%c", 0x09);
    fprintf(paux, "%d %d %c", *(passi), *(passi + 1), 0x09);
}

int get_state(int *p_m1, int *p_m2, float theta, float *delta)
{
  float linDis = RAGGIO_RUOTA * M_PI * ((*p_m1 + *p_m2) / 2.0) / 300.0;

  *(delta + STATE_THETA) = RAGGIO_RUOTA / INTERASSE * M_PI * (*p_m2 - *p_m1) / 300.0;
  *(delta + STATE_X) = linDis * cos(*(delta + STATE_THETA) + theta);
  *(delta + STATE_Y) = linDis * sin(*(delta + STATE_THETA) + theta);
}

int get_servoing_feedback(unsigned char* payload, int l, int *ref_m1, int *ref_m2, int *verifica)
{
	
    unsigned short int contatore = 0;
    contatore += *payload;
    int i, j;
    j = 1;
	
    for (i = 1; i < l - 2; i++)
    {
		
		if (*(payload + j) == PACKET_SOGLIA)
		{
			j++;
			*(payload + j) -= (PACKET_SOGLIA + PACKET_OFFSET_MASCHERAMENTO);
		}
		contatore += *(payload + j);
		if (i < 3)
			*ref_m1 -= (*(payload + j) << (((i - 1) % 2)*8));
		else
			*ref_m2 -= (*(payload + j) << (((i - 1) % 2)*8));
		
		j++;
    }

    contatore = ~contatore;
    contatore++;
	
    for (i = 0; i < 2; i++)
    {
		if (*(payload + j) == PACKET_SOGLIA)
		{
			j++;
			*(payload + j) -= (PACKET_SOGLIA + PACKET_OFFSET_MASCHERAMENTO);
		}

		contatore -= (*(payload + j) * moltiplicatore[i]);
		j++;
    }
	
    *verifica = (contatore == 0);
	
    return j;
}

//----------------------------------------------------------------------

int genera_pacchetto_sensore_standard(unsigned char *pacchetto, struct sensore_t *s, unsigned char header)
{
    int i, j;
    *pacchetto = header;
    unsigned short int crc = *pacchetto;
    j = 1;
    for (i = 0; i < s->num_canali; i++)
    {
		
		*(pacchetto + j) = *(s->range + i)&255;
		crc += *(pacchetto + j);
		j += check_soglia((pacchetto + j));
		j++;
		
		*(pacchetto + j) = *(s->range + i) >> 8;
		crc += *(pacchetto + j);
		j += check_soglia((pacchetto + j));
		j++;
		
    }
    crc = ~crc;
    crc++;
    *(pacchetto + j) = crc & 255;
    j += check_soglia((pacchetto + j));
    j++;
    *(pacchetto + j) = crc >> 8;
    j += check_soglia((pacchetto + j));
    j++;
	
    return j;
}
//----------------------------------------------------------------------


//------------------------------------------------------------------------------
int analizza_pacchetto_init(unsigned char* buffer)
{
  int message_size;
  int message_index = 0;
  int controllo_dati = 1;
  unsigned int code;
  unsigned int i;
	
  unsigned char pic_message_buffer[256];
  
	printf("-----analizza pacchetto----\n");	

	message_size = 0;
	read(pic_fd, pic_message_buffer + message_size, 1);
  
	while((pic_message_buffer + message_size) != 0xa)
  {
		++message_size;
		read(pic_fd, pic_message_buffer + message_size, 1);
	}

	printf("buffer size: %d\n", message_size);
  
	while (message_index < message_size - 1)
  {
#if (PIC_LOG_TO_SCREEN == 1)
		printf("message_index: %d\n",message_index);
#endif
		
		code = (unsigned int) (pic_message_buffer + message_index);
		
		switch (code)
		{
      //______________________ START ___________________________________________________
			case PACKET_START_HEADER:
#if (PIC_LOG_TO_SCREEN == 1)
				printf("PACKET_START_HEADER\n");
#endif
				if (steps_anomaly == 1)
				{
					write(pic_fd, pic_message_reset_steps_acc, PACKET_TIMING_LENGTH + 1);
#ifdef VERBOSE
					printf("R\n");
#endif
					steps_anomaly = 0;
				}
				else
				{
					switch(*buffer)
					{
							
						case PACKET_SPEED_HEADER:
							printf("--PACKET_SPEED_HEADER\n");
							write(pic_fd, buffer, PACKET_SPEED_LENGTH + 1);
							int vel1, vel2;

							vel1 = (*(buffer + 4) << 8) | *(buffer + 3);
							vel1 = MAX_VEL - vel1;
							vel1 = *(buffer + 1)?-vel1:vel1;
							
							vel2 = (*(buffer + 6) << 8) | *(buffer + 5);
							vel2 = MAX_VEL - vel2;
							vel2 = *(buffer + 2)?-vel2:vel2;
							
							step2vel(vel1,vel2,&last_v_ref,&last_w_ref);
							
							*buffer = 0;
							break;
              
						case PACKET_SERVOING_HEADER:
							printf("--PACKET_SERVOING_HEADER\n");
							write(pic_fd, buffer, PACKET_SERVOING_LENGTH + 1);
							*buffer = 0;
							break;
              
						default:
							printf("--DEFAULT CASE\n");
							write(pic_fd, pic_message_timing, PACKET_TIMING_LENGTH + 1);
							// a scanso di equivoci...
							*buffer = 0;
							break;
					}
				}
				message_index = message_size;
				return START_PACKET_ANALYZED;
				break;
				//_____________________________________________________________________________________________
				
				//______________________ DISTANCE ___________________________________________________
			case PACKET_DISTANCE_HEADER:
				printf("PACKET_DISTANCE_HEADER\n");
				++ad_minc;
				pthread_mutex_lock(&mutex_ir);
				ir->is_valid =	FALSE;
				message_index += get_distanza(pic_message_buffer + message_index, PACKET_DISTANCE_LENGTH, ir, &controllo_dati);
#ifdef LOG_SU_FILE
				pthread_mutex_lock(&mutex_fp);
				if (ir->is_valid == TRUE){print_sensore_on_file(ir, fp_log);}//print_sensore(ir);
				else{if (fp_log!=NULL){fprintf(fp_log, "%c %d %d %d %d %d ", 0x09, -1, -1, -1, -1, -1);}}
				//ir->is_valid=FALSE;
				pthread_mutex_unlock(&mutex_fp);
#else
				if (ir->is_valid == TRUE){print_sensore(ir);}
#endif
				pthread_mutex_unlock(&mutex_ir);
				break;
				//_____________________________________________________________________________________________
				
				//______________________ IR ___________________________________________________
			case PACKET_IR_HEADER:
				printf("PACKET_IR_HEADER\n");
				++ad_minc;
				pthread_mutex_lock(&mutex_ir);
				ir->is_valid =	FALSE;
				message_index += get_sensore(pic_message_buffer + message_index, PACKET_IR_LENGTH, ir, &controllo_dati);
#ifdef LOG_SU_FILE
				pthread_mutex_lock(&mutex_fp);
				if (ir->is_valid == TRUE){print_sensore_on_file(ir, fp_log);}//print_sensore(ir);
				else{if (fp_log!=NULL){fprintf(fp_log, "%c %d %d %d %d %d ", 0x09, -1, -1, -1, -1, -1);}}
				//ir->is_valid=FALSE;
				pthread_mutex_unlock(&mutex_fp);
#else
				if (ir->is_valid == TRUE){print_sensore(ir);}
#endif
				pthread_mutex_unlock(&mutex_ir);
				break;
				//_____________________________________________________________________________________________
				
				//______________________ ACC ___________________________________________________
			case PACKET_ACC_HEADER:
				printf("PACKET_ACC_HEADER\n");
				//printf("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n");
				pthread_mutex_lock(&mutex_acc);
				acc->is_valid =FALSE;
				message_index += get_sensore(pic_message_buffer + message_index, PACKET_ACC_LENGTH, acc, &controllo_dati);
#ifdef LOG_SU_FILE
				pthread_mutex_lock(&mutex_fp);
				if (acc->is_valid == TRUE){print_sensore_on_file(acc, fp_log);print_sensore(acc);}
				else{if (fp_log!=NULL){fprintf(fp_log, "%c %d %d %d ", 0x09, -1, -1, -1);}}
				pthread_mutex_unlock(&mutex_fp);
#else
				if (acc->is_valid == TRUE){print_sensore(acc);}
#endif
				//if (acc->is_valid == TRUE){print_sensore(acc);}
				pthread_mutex_unlock(&mutex_acc);
				break;
				//_____________________________________________________________________________________________

				//______________________ MAGACC ___________________________________________________
			case PACKET_MAGACC_HEADER:
				printf("PACKET_MAGACC_HEADER\n");
				//printf("**************************************\n");
				//printf("**************************************\n");
				//printf("**************************************\n");
				pthread_mutex_lock(&mutex_acc);
				pthread_mutex_lock(&mutex_magneto);
				acc->is_valid =FALSE;
				message_index += get_magacc(pic_message_buffer + message_index, PACKET_ACC_LENGTH, acc, &controllo_dati);
#ifdef LOG_SU_FILE
				pthread_mutex_lock(&mutex_fp);
				if (acc->is_valid == TRUE){print_sensore_on_file(acc, fp_log);}//print_sensore(acc);
				if (magneto->is_valid == TRUE){print_sensore_on_file(magneto, fp_log);}//print_sensore(magneto);
				else{if (fp_log!=NULL){fprintf(fp_log, "%c %d %d %d ", 0x09, -1, -1, -1);}}
				pthread_mutex_unlock(&mutex_fp);
#else
				if (acc->is_valid == TRUE){print_sensore(acc);}
#endif
				//if (acc->is_valid == TRUE){print_sensore(acc);}
				pthread_mutex_unlock(&mutex_acc);
				pthread_mutex_unlock(&mutex_magneto);
				break;
				//_____________________________________________________________________________________________
				
				
				
				//______________________ MAGNETO ___________________________________________________
			case PACKET_MAGNETO_HEADER:
				printf("PACKET_MAGNETO_HEADER\n");
				pthread_mutex_lock(&mutex_magneto);
				magneto->is_valid =FALSE;
				message_index += get_sensore(pic_message_buffer + message_index, PACKET_MAGNETO_LENGTH-1, magneto, &controllo_dati);
#ifdef LOG_SU_FILE
				pthread_mutex_lock(&mutex_fp);
				if (magneto->is_valid == TRUE){print_sensore_on_file(magneto, fp_log);}
				else{if (fp_log!=NULL){fprintf(fp_log, "%c %d %d ", 0x09, -1, -1);}}
				pthread_mutex_unlock(&mutex_fp);
#else
				//if (magneto->is_valid == TRUE){print_sensore(magneto);}
#endif
				//if (magneto->is_valid == TRUE){print_sensore(magneto);}
				pthread_mutex_unlock(&mutex_magneto);
				break;
				//_____________________________________________________________________________________________
				
				
				//______________________ GYRO INTEGRAL ___________________________________________________
			case PACKET_GYRO_INTEGRAL_HEADER:
				printf("PACKET_GYRO_INTEGRAL_HEADER\n");
				message_index += get_gyro_integral(pic_message_buffer + message_index, PACKET_GYRO_INTEGRAL_LENGTH, &gyro_integral_pic, &controllo_dati);
				if(controllo_dati==1){
					
					if(abs(gyro_integral_pic)>16){gyro_sup_integral+=((float)gyro_integral_pic)/1024;}
					if (fp_log!=NULL){fprintf(fp_log, "%c %d %f",0x09, gyro_integral_pic,gyro_sup_integral);}
				}
				else{printf("INTEGRALE SBAGLIATO\n");}
				//			printf("ANGLE: %f\n", gyro_sup_integral);
				break;
				//_____________________________________________________________________________________________
				
				
				//______________________ GYRO ___________________________________________________
			case PACKET_GYRO_HEADER:
				printf("PACKET_GYRO_HEADER\n");
				gyro->is_valid =FALSE;
				pthread_mutex_lock(&mutex_gyro);
				message_index += get_sensore(pic_message_buffer + message_index, PACKET_GYRO_LENGTH, gyro, &controllo_dati);
#ifdef USE_GYRO_TABLE
				if(gyro->is_valid==TRUE){
					for(i=0;i<gyro->num_canali;i++)	{
						if(*(gyro->range+i)>=gyro_table_min_index && *(gyro->range+i)<=gyro_table_max_index){				
							*(gyro->converted+i)=*(gyro_table_value+(*(gyro->range+i)-gyro_table_min_index+(gyro_table_nominal_zero-	gyro_table_effective_zero))); 
						}
						else{*(gyro->converted+i)=-10.0;}
					}
					
					// it is worth to note that in case of bad measure the  last usefull element is integrated:
					// this portion of code runs only when the gyro sensor measure is vald (gyro->is_valid==TRUE )
					//printf("W_GYRO: %d %f, %f\n",*(gyro->range+1),*(gyro->converted+1), gyro_integral);
				}
				gyro_integral=gyro_integral+*(gyro->converted+1)*T;
				//printf("lettura: %d\n",*(gyro->range+1));
#endif
#ifdef LOG_SU_FILE
				pthread_mutex_lock(&mutex_fp);
				
				if (gyro->is_valid == TRUE){print_sensore_on_file(gyro, fp_log);}
				else{if (fp_log!=NULL){fprintf(fp_log, "%c %d %d ", 0x09, -1, -1);}}
				pthread_mutex_unlock(&mutex_fp);
#else
				//if (gyro->is_valid == TRUE){print_sensore(gyro);}
#endif
				pthread_mutex_unlock(&mutex_gyro);
				break;
				//_____________________________________________________________________________________________
				
				//______________________ ACK ___________________________________________________
				
			case PACKET_ACK_HEADER:
				printf("PACKET_ACK_HEADER\n");
				message_index += PACKET_ACK_LENGTH;
				pthread_mutex_lock(&mutex_fp);
				pthread_mutex_unlock(&mutex_fp);
				break;
				//_____________________________________________________________________________________________
			case PACKET_SERVOING_FEEDBACK_HEADER:
				message_index += get_servoing_feedback(pic_message_buffer + message_index, PACKET_SERVOING_FEEDBACK_LENGTH, &(riferimenti_servoing[0]), &(riferimenti_servoing[1]), &controllo_dati);
				//flag utile per calibrazione
				flag_servoing_completed = 1;
				//if (controllo_dati != 1) {printf("ERR_SERV_FEED\n");}
				
				break;
				
			case PACKET_GYRO_OFFSET_HEADER:
#if(PIC_LOG_TO_SCREEN == 1)
				printf("PACKET_GYRO_OFFSET_HEADER\n");
#endif
				message_index += get_offset_sensore(pic_message_buffer + message_index, PACKET_GYRO_LENGTH, gyro, &controllo_dati);
				if (gyro->is_valid == TRUE)
				{
					print_sensore(gyro);
				}
				printf("------------------------------------------------------------\n");
				printf("------------------------------------------------------------\n");
				printf("------------------------------------------------------------\n");
				printf("------------------------------------------------------------\n");
				printf("------------------------------------------------------------\n");
				
				//if (controllo_dati != 1) {printf("ERR_MAGNETO\n");}
				break;
				
			case PACKET_ODOM_HEADER:
#if(PIC_LOG_TO_SCREEN == 1)
				printf("PACKET_ODOM_HEADER\n");
#endif
				steps_done[0] = 0;
				steps_done[1] = 0;
				message_index += get_odometria(pic_message_buffer + message_index, PACKET_ODOM_LENGTH, &(steps_done[0]), &(steps_done[1]), &(sign_motors[0]), &(sign_motors[1]), &controllo_dati);
				
				//if (controllo_dati != 1) {
				//    //printf("ERR_ODOM\n");
				//    break;
				//}
				if (controllo_dati == 1)
				{
					if (steps_done[0] >= delta_passi[0])
					{
						delta_passi[0] = steps_done[0] - delta_passi[0];
					}
					else
					{
						delta_passi[0] = MAX_STEPS_COUNTER - delta_passi[0] + steps_done[0];
					}
					
					if (steps_done[1] >= delta_passi[1])
					{
						delta_passi[1] = steps_done[1] - delta_passi[1];
					}
					else
					{
						delta_passi[1] = MAX_STEPS_COUNTER - delta_passi[1] + steps_done[1];
					}
					
					
					if (delta_passi[0] > THRESHOLD_STEPS_COUNTER || delta_passi[1] > THRESHOLD_STEPS_COUNTER)
					{
						
						steps_done[0]=0;
						steps_done[1]=0;
						delta_passi[0]=0;
						delta_passi[1]=0;
						steps_anomaly = 1;
#ifdef VERBOSE
						
						printf("ANOMALIA\n");
#endif
					}
					else
					{
						delta_passi[0] *= sign_motors[0];
						delta_passi[1] *= sign_motors[1];
						
						pthread_mutex_lock(&mutex_state);
						get_state(&(delta_passi[0]), &(delta_passi[1]), state[STATE_THETA], &(delta_state[0]));
						
						state[STATE_X] += delta_state[STATE_X];
						state[STATE_Y] += delta_state[STATE_Y];
						state[STATE_THETA] += delta_state[STATE_THETA];
						if (state[STATE_THETA] > M_PI)
						{
							state[STATE_THETA] -= (2 * M_PI);
						}
						if (state[STATE_THETA]<-M_PI)
						{
							state[STATE_THETA] += (2 * M_PI);
						}
						
						pthread_mutex_unlock(&mutex_state);
#ifdef VERBOSE
						//printf("STATO: %f %f %f\n", state[STATE_X], state[STATE_Y], state[STATE_THETA]);
#endif
						//printf("A: %d %d\n", steps_done[0],steps_done[1]);
#ifdef LOG_SU_FILE
						
						pthread_mutex_lock(&mutex_fp);
						if (controllo_dati == TRUE)
						{
							print_odometria(fp_log, &state[0], &(steps_done[0]));
						}
						else
						{
							if (fp_log!=NULL){fprintf(fp_log, "%c %f %f %f %c %d %d %c", 0x09, -1.0,-1.0,-1.0, 0x09, -1,-1,0x09);}
						}
						pthread_mutex_unlock(&mutex_fp);
						
#else
						printf("%f %f %f\n", state[STATE_X], state[STATE_Y], state[STATE_THETA]);
#endif
						delta_passi[0] = steps_done[0];
						delta_passi[1] = steps_done[1];
					}
					
					
				}
				else
				{
#ifdef VERBOSE
					printf("NOLOG\n");
#endif
					if (fp_log!=NULL){fprintf(fp_log, "%c %f %f %f %c %d %d %c", 0x09, -1.0,-1.0,-1.0, 0x09, -1,-1,0x09);}
				}
				break;
			case PACKET_ERRPIC_HEADER:
#if(PIC_LOG_TO_SCREEN == 1)
				printf("PACKET_ERRPIC_HEADER\n");
#endif
				steps_done[0]=0;
				steps_done[1]=0;
				++num_packet_sent_wrong;
				message_index += PACKET_ERRPIC_LENGTH;
				//printf("e\n");
				break;
			default:
				++num_packet_data_wrong;
				controllo_dati = 0;
				pthread_mutex_lock(&mutex_fp);
				pthread_mutex_unlock(&mutex_fp);
				return ERROR_PACKET_ANALYZED;
				
		}
  }

  if (controllo_dati == 1 && message_size > 0)
  {
		pthread_mutex_lock(&mutex_fp);
    
		if (fp_log!=NULL)
      fprintf(fp_log, "%d %d\n", ++num_packet_data_ok, message_size);

		pthread_mutex_unlock(&mutex_fp);
		clock_counter++;
		return LOAD_PACKET_ANALYZED;
  }
  else
  {
		pthread_mutex_lock(&mutex_fp);
		
    if (fp_log!=NULL){fprintf(fp_log, "\nN%d\n", ++num_packet_data_wrong);}
	  	printf("-----------------------------------------------------------------\n");
		
    pthread_mutex_unlock(&mutex_fp);
		clock_counter++;
		return ERROR_PACKET_ANALYZED;
		
  }
  return 1;
}


int analizza_pacchetto(char *pic_buffer, char *buf, int buffer_size)
{
  int message_index = 0;
  int controllo_dati = 1;
  unsigned int code;

  unsigned char pic_message_buffer[256];
  
  if(sizeof(pic_message_buffer) >= buffer_size)
    memcpy(pic_message_buffer, buf, buffer_size);
  else
  {
    // cannot use temp buffer
    return -1;
  }

  while (message_index < buffer_size - 1)
  {
#if (PIC_LOG_TO_SCREEN == 1)
		printf("message_index: %d\n",message_index);
#endif
		
		code = (unsigned int) *(pic_message_buffer + message_index);
		
		switch (code)
		{
		  //______________________ START ___________________________________________________
		  case PACKET_START_HEADER:
#if(PIC_LOG_TO_SCREEN == 1)
				printf("PACKET_START_HEADER\n");
#endif
				if (steps_anomaly == 1)
				{
					//write(pic_fd, pic_message_reset_steps_acc, PACKET_TIMING_LENGTH + 1);
          rs232_load_tx(pic_message_reset_steps_acc, PACKET_TIMING_LENGTH + 1);

#ifdef VERBOSE
					printf("R\n");
#endif
					steps_anomaly = 0;
				}
				else
				{
					switch(*pic_buffer)
					{
						case PACKET_SPEED_HEADER:
#if(PIC_LOG_TO_SCREEN == 1)
							printf("--PACKET_SPEED_HEADER\n");
#endif
							//write(pic_fd, pic_buffer, PACKET_SPEED_LENGTH + 1);
              rs232_load_tx(pic_buffer, PACKET_SPEED_LENGTH + 1);
              
							int vel1, vel2;

							vel1 = (*(pic_buffer + 4) << 8) | *(pic_buffer + 3);
							vel1 = MAX_VEL - vel1;
							vel1 = *(pic_buffer + 1)?-vel1:vel1;
							
							vel2 = (*(pic_buffer + 6) << 8) | *(pic_buffer + 5);
							vel2 = MAX_VEL - vel2;
							vel2 = *(pic_buffer + 2)?-vel2:vel2;
							
							step2vel(vel1,vel2,&last_v_ref,&last_w_ref);
							
							*pic_buffer = 0;
							break;
						case PACKET_SERVOING_HEADER:
#if(PIC_LOG_TO_SCREEN == 1)
							printf("--PACKET_SERVOING_HEADER\n");
#endif
							//write(pic_fd, pic_buffer, PACKET_SERVOING_LENGTH + 1);
              rs232_load_tx(pic_buffer, PACKET_SERVOING_LENGTH + 1);
              
							*pic_buffer = 0;
							break;
						default:
#if(PIC_LOG_TO_SCREEN == 1)
							printf("--DEFAULT CASE\n");
#endif
							//write(pic_fd, pic_message_timing, PACKET_TIMING_LENGTH + 1);
              rs232_load_tx(pic_message_timing, PACKET_TIMING_LENGTH + 1);
              
							// a scanso di equivoci...
							*pic_buffer = 0;
							break;
					}
				}
				message_index = buffer_size;
				return START_PACKET_ANALYZED;
				break;
				//_____________________________________________________________________________________________
				
				//______________________ DISTANCE ___________________________________________________
			case PACKET_DISTANCE_HEADER:
#if(PIC_LOG_TO_SCREEN == 1)
				printf("PACKET_DISTANCE_HEADER\n");
#endif
				++ad_minc;
				pthread_mutex_lock(&mutex_ir);

				message_index += get_distanza(pic_message_buffer + message_index, PACKET_DISTANCE_LENGTH, ir, &controllo_dati);
#ifdef LOG_SU_FILE
				pthread_mutex_lock(&mutex_fp);
				if (ir->is_valid == TRUE)
          print_sensore_on_file(ir, fp_log);
				else
        {
          if (fp_log != NULL)
            fprintf(fp_log, "%c %d %d %d %d %d ", 0x09, -1, -1, -1, -1, -1);
        }

				pthread_mutex_unlock(&mutex_fp);
#else
				if (ir->is_valid == TRUE){print_sensore(ir);}
#endif
				pthread_mutex_unlock(&mutex_ir);
				break;
				//_____________________________________________________________________________________________
				
				//______________________ IR ___________________________________________________
			case PACKET_IR_HEADER:
#if(PIC_LOG_TO_SCREEN == 1)
				printf("PACKET_IR_HEADER\n");
#endif
				++ad_minc;
				pthread_mutex_lock(&mutex_ir);
				ir->is_valid =	FALSE;
				message_index += get_sensore(pic_message_buffer + message_index, PACKET_IR_LENGTH, ir, &controllo_dati);
#ifdef LOG_SU_FILE
				pthread_mutex_lock(&mutex_fp);
				if (ir->is_valid == TRUE){print_sensore_on_file(ir, fp_log);}//print_sensore(ir);
				else{if (fp_log!=NULL){fprintf(fp_log, "%c %d %d %d %d %d ", 0x09, -1, -1, -1, -1, -1);}}
				//ir->is_valid=FALSE;
				pthread_mutex_unlock(&mutex_fp);
#else
				if (ir->is_valid == TRUE){print_sensore(ir);}
#endif
				pthread_mutex_unlock(&mutex_ir);
				break;
				//_____________________________________________________________________________________________
				
				//______________________ ACC ___________________________________________________
			case PACKET_ACC_HEADER:
#if(PIC_LOG_TO_SCREEN == 1)
				printf("PACKET_ACC_HEADER\n");
#endif
				//printf("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n");
				pthread_mutex_lock(&mutex_acc);
				acc->is_valid =FALSE;
				message_index += get_sensore(pic_message_buffer + message_index, PACKET_ACC_LENGTH, acc, &controllo_dati);
#ifdef LOG_SU_FILE
				pthread_mutex_lock(&mutex_fp);
				if (acc->is_valid == TRUE){print_sensore_on_file(acc, fp_log);print_sensore(acc);}
				else{if (fp_log!=NULL){fprintf(fp_log, "%c %d %d %d ", 0x09, -1, -1, -1);}}
				pthread_mutex_unlock(&mutex_fp);
#else
				if (acc->is_valid == TRUE){print_sensore(acc);}
#endif
				//if (acc->is_valid == TRUE){print_sensore(acc);}
				pthread_mutex_unlock(&mutex_acc);
				break;
				//_____________________________________________________________________________________________
				
				
				
				
				//______________________ MAGACC ___________________________________________________
			case PACKET_MAGACC_HEADER:
#if(PIC_LOG_TO_SCREEN == 1)
				printf("PACKET_MAGACC_HEADER\n");
#endif
				//printf("**************************************\n");
				//printf("**************************************\n");
				//printf("**************************************\n");
				pthread_mutex_lock(&mutex_acc);
				pthread_mutex_lock(&mutex_magneto);
				acc->is_valid =FALSE;
				message_index += get_magacc(pic_message_buffer + message_index, PACKET_ACC_LENGTH, acc, &controllo_dati);
#ifdef LOG_SU_FILE
				pthread_mutex_lock(&mutex_fp);
				if (acc->is_valid == TRUE){print_sensore_on_file(acc, fp_log);}//print_sensore(acc);
				if (magneto->is_valid == TRUE){print_sensore_on_file(magneto, fp_log);}//print_sensore(magneto);
				else{if (fp_log!=NULL){fprintf(fp_log, "%c %d %d %d ", 0x09, -1, -1, -1);}}
				pthread_mutex_unlock(&mutex_fp);
#else
				if (acc->is_valid == TRUE){print_sensore(acc);}
#endif
				//if (acc->is_valid == TRUE){print_sensore(acc);}
				pthread_mutex_unlock(&mutex_acc);
				pthread_mutex_unlock(&mutex_magneto);
				break;
				//_____________________________________________________________________________________________
				
				
				
				//______________________ MAGNETO ___________________________________________________
			case PACKET_MAGNETO_HEADER:
#if(PIC_LOG_TO_SCREEN == 1)
				printf("PACKET_MAGNETO_HEADER\n");
#endif
				pthread_mutex_lock(&mutex_magneto);
				magneto->is_valid =FALSE;
				message_index += get_sensore(pic_message_buffer + message_index, PACKET_MAGNETO_LENGTH-1, magneto, &controllo_dati);
#ifdef LOG_SU_FILE
				pthread_mutex_lock(&mutex_fp);
				if (magneto->is_valid == TRUE){print_sensore_on_file(magneto, fp_log);}
				else{if (fp_log!=NULL){fprintf(fp_log, "%c %d %d ", 0x09, -1, -1);}}
				pthread_mutex_unlock(&mutex_fp);
#else
				//if (magneto->is_valid == TRUE){print_sensore(magneto);}
#endif
				//if (magneto->is_valid == TRUE){print_sensore(magneto);}
				pthread_mutex_unlock(&mutex_magneto);
				break;
				//_____________________________________________________________________________________________
				
				
				//______________________ GYRO INTEGRAL ___________________________________________________
			case PACKET_GYRO_INTEGRAL_HEADER:
#if(PIC_LOG_TO_SCREEN == 1)
				printf("PACKET_GYRO_INTEGRAL_HEADER\n");
#endif
				message_index += get_gyro_integral(pic_message_buffer + message_index, PACKET_GYRO_INTEGRAL_LENGTH, &gyro_integral_pic, &controllo_dati);
				if(controllo_dati==1){
					
					if(abs(gyro_integral_pic)>16){gyro_sup_integral+=((float)gyro_integral_pic)/1024;}
					if (fp_log!=NULL){fprintf(fp_log, "%c %d %f",0x09, gyro_integral_pic,gyro_sup_integral);}
				}
				else{printf("INTEGRALE SBAGLIATO\n");}
				//			printf("ANGLE: %f\n", gyro_sup_integral);
				break;
				//_____________________________________________________________________________________________
				
				
				//______________________ GYRO ___________________________________________________
			case PACKET_GYRO_HEADER:
#if(PIC_LOG_TO_SCREEN == 1)
				printf("PACKET_GYRO_HEADER\n");
#endif
				gyro->is_valid =FALSE;
				pthread_mutex_lock(&mutex_gyro);
				message_index += get_sensore(pic_message_buffer + message_index, PACKET_GYRO_LENGTH, gyro, &controllo_dati);
#ifdef USE_GYRO_TABLE
				if(gyro->is_valid==TRUE){
					for(i=0;i<gyro->num_canali;i++)	{
						if(*(gyro->range+i)>=gyro_table_min_index && *(gyro->range+i)<=gyro_table_max_index){				
							*(gyro->converted+i)=*(gyro_table_value+(*(gyro->range+i)-gyro_table_min_index+(gyro_table_nominal_zero-	gyro_table_effective_zero))); 
						}
						else{*(gyro->converted+i)=-10.0;}
					}
					
					// it is worth to note that in case of bad measure the  last usefull element is integrated:
					// this portion of code runs only when the gyro sensor measure is vald (gyro->is_valid==TRUE )
					//printf("W_GYRO: %d %f, %f\n",*(gyro->range+1),*(gyro->converted+1), gyro_integral);
				}
				gyro_integral=gyro_integral+*(gyro->converted+1)*T;
				//printf("lettura: %d\n",*(gyro->range+1));
#endif
#ifdef LOG_SU_FILE
				pthread_mutex_lock(&mutex_fp);
				
				if (gyro->is_valid == TRUE){print_sensore_on_file(gyro, fp_log);}
				else{if (fp_log!=NULL){fprintf(fp_log, "%c %d %d ", 0x09, -1, -1);}}
				pthread_mutex_unlock(&mutex_fp);
#else
				//if (gyro->is_valid == TRUE){print_sensore(gyro);}
#endif
				pthread_mutex_unlock(&mutex_gyro);
				break;
				//_____________________________________________________________________________________________
				
				//______________________ ACK ___________________________________________________
				
			case PACKET_ACK_HEADER:
#if(PIC_LOG_TO_SCREEN == 1)
				printf("PACKET_ACK_HEADER\n");
#endif
				message_index += PACKET_ACK_LENGTH;
				pthread_mutex_lock(&mutex_fp);
				pthread_mutex_unlock(&mutex_fp);
				break;
				//_____________________________________________________________________________________________
			case PACKET_SERVOING_FEEDBACK_HEADER:
				message_index += get_servoing_feedback(pic_message_buffer + message_index, PACKET_SERVOING_FEEDBACK_LENGTH, &(riferimenti_servoing[0]), &(riferimenti_servoing[1]), &controllo_dati);
				//flag utile per calibrazione
				flag_servoing_completed = 1;
				//if (controllo_dati != 1) {printf("ERR_SERV_FEED\n");}
				
				break;
				
			case PACKET_GYRO_OFFSET_HEADER:
#if(PIC_LOG_TO_SCREEN == 1)
				printf("PACKET_GYRO_OFFSET_HEADER\n");
#endif
				message_index += get_offset_sensore(pic_message_buffer + message_index, PACKET_GYRO_LENGTH, gyro, &controllo_dati);
				if (gyro->is_valid == TRUE)
				{
					print_sensore(gyro);
				}
				printf("------------------------------------------------------------\n");
				printf("------------------------------------------------------------\n");
				printf("------------------------------------------------------------\n");
				printf("------------------------------------------------------------\n");
				printf("------------------------------------------------------------\n");
				
				//if (controllo_dati != 1) {printf("ERR_MAGNETO\n");}
				break;
				
			case PACKET_ODOM_HEADER:
#if(PIC_LOG_TO_SCREEN == 1)
				printf("PACKET_ODOM_HEADER\n");
#endif
				steps_done[0] = 0;
				steps_done[1] = 0;
				message_index += get_odometria(pic_message_buffer + message_index, PACKET_ODOM_LENGTH, &(steps_done[0]), &(steps_done[1]), &(sign_motors[0]), &(sign_motors[1]), &controllo_dati);
				
				//if (controllo_dati != 1) {
				//    //printf("ERR_ODOM\n");
				//    break;
				//}
				if (controllo_dati == 1)
				{
					if (steps_done[0] >= delta_passi[0])
					{
						delta_passi[0] = steps_done[0] - delta_passi[0];
					}
					else
					{
						delta_passi[0] = MAX_STEPS_COUNTER - delta_passi[0] + steps_done[0];
					}
					
					if (steps_done[1] >= delta_passi[1])
					{
						delta_passi[1] = steps_done[1] - delta_passi[1];
					}
					else
					{
						delta_passi[1] = MAX_STEPS_COUNTER - delta_passi[1] + steps_done[1];
					}
					
					
					if (delta_passi[0] > THRESHOLD_STEPS_COUNTER || delta_passi[1] > THRESHOLD_STEPS_COUNTER)
					{
						
						steps_done[0]=0;
						steps_done[1]=0;
						delta_passi[0]=0;
						delta_passi[1]=0;
						steps_anomaly = 1;
#ifdef VERBOSE
						
						printf("ANOMALIA\n");
#endif
					}
					else
					{
						delta_passi[0] *= sign_motors[0];
						delta_passi[1] *= sign_motors[1];
						
						pthread_mutex_lock(&mutex_state);
						get_state(&(delta_passi[0]), &(delta_passi[1]), state[STATE_THETA], &(delta_state[0]));
						
						state[STATE_X] += delta_state[STATE_X];
						state[STATE_Y] += delta_state[STATE_Y];
						state[STATE_THETA] += delta_state[STATE_THETA];
            
						if (state[STATE_THETA] > M_PI)
						{
							state[STATE_THETA] -= (2 * M_PI);
						}
						if (state[STATE_THETA]<-M_PI)
						{
							state[STATE_THETA] += (2 * M_PI);
						}
						
						pthread_mutex_unlock(&mutex_state);
#ifdef VERBOSE
						//printf("STATO: %f %f %f\n", state[STATE_X], state[STATE_Y], state[STATE_THETA]);
#endif
						//printf("A: %d %d\n", steps_done[0],steps_done[1]);
#ifdef LOG_SU_FILE
						
						pthread_mutex_lock(&mutex_fp);
                                                if (fp_log != NULL) {
                                                    if (controllo_dati == TRUE)
                                                    {
                                                            print_odometria(fp_log, &state[0], &(steps_done[0]));
                                                    }
                                                    else
                                                    {
                                                            fprintf(fp_log, "%c %f %f %f %c %d %d %c", 0x09, -1.0,-1.0,-1.0, 0x09, -1,-1,0x09);
                                                    }
                                                }
						pthread_mutex_unlock(&mutex_fp);
						
#else
#if(PIC_LOG_TO_SCREEN == 1)
						printf("%f %f %f\n", state[STATE_X], state[STATE_Y], state[STATE_THETA]);
#endif

#endif
						delta_passi[0] = steps_done[0];
						delta_passi[1] = steps_done[1];
					}
					
					
				}
				else
				{
#ifdef VERBOSE
					printf("NOLOG\n");
#endif
                                        if (fp_log!=NULL){ fprintf(fp_log, "%c %f %f %f %c %d %d %c", 0x09, -1.0,-1.0,-1.0, 0x09, -1,-1,0x09);}
				}
				break;
			case PACKET_ERRPIC_HEADER:
#if(PIC_LOG_TO_SCREEN == 1)
				printf("PACKET_ERRPIC_HEADER\n");
#endif
				steps_done[0]=0;
				steps_done[1]=0;
				++num_packet_sent_wrong;
				message_index += PACKET_ERRPIC_LENGTH;
				//printf("e\n");
				break;
			default:
#ifdef VERBOSE
				printf("NC: message_index %d    buffer_size %d code: %02x\n", message_index, buffer_size, code);
				
#endif
				++num_packet_data_wrong;
				controllo_dati = 0;
				pthread_mutex_lock(&mutex_fp);
				//fprintf(fp_log, "\nN%d\n", ++num_packet_data_wrong);
				//fprintf(fp_log, "\nN%d\n", ++num_packet_data_ok);
				pthread_mutex_unlock(&mutex_fp);
				return ERROR_PACKET_ANALYZED;
				
		}
  }
	
    //printf("RIMASTO: %d\n", buffer_size-message_index);
  if(controllo_dati == 1 && buffer_size > 0)
  {
		/*pthread_mutex_lock(&mutex_fp);
		if (fp_log!=NULL) {fprintf(fp_log, "%d %d\n", ++num_packet_data_ok, buffer_size);}

		pthread_mutex_unlock(&mutex_fp);*/
		clock_counter++;
		return LOAD_PACKET_ANALYZED;
  }
  else
  {
		/*pthread_mutex_lock(&mutex_fp);
		if (fp_log!=NULL){fprintf(fp_log, "\nN%d\n", ++num_packet_data_wrong);}
		printf("-----------------------------------------------------------------\n");
		pthread_mutex_unlock(&mutex_fp);*/
		clock_counter++;
		return ERROR_PACKET_ANALYZED;
  }
  return 1;
}








//----------------------------------------------------------------------
