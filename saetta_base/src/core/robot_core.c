#include "robot_core.h"

//______________________________________________________________________________

//------------------------------------------------------------------------------

int init_robot(char* portname) 
{
  //=====================================
  //			Init
  clock_counter = 0;
  //Packet Type
  packet_type = ERROR_PACKET_ANALYZED;
  //=====================================
  //
  pthread_mutex_init(&mutex_fp, NULL);
  pthread_mutex_init(&mutex_state, NULL);
  pthread_mutex_init(&mutex_ir, NULL);
  pthread_mutex_init(&mutex_gyro, NULL);
  pthread_mutex_init(&mutex_acc, NULL);
  pthread_mutex_init(&mutex_magneto, NULL);
  init_sensors();
    
  // start serial communication
  if(init_modulo_comm(portname) < 0) //da robot_comm.c
    return -1;

  write(pic_fd, pic_message_reset_steps_acc, PACKET_TIMING_LENGTH + 1);
  tcflush(pic_fd, TCOFLUSH);
  sync();      
}

//------------------------------------------------------------------------------
// Given the translational and rotational velocity the velocities for the two
// motors are computed by taking into account the saturation. The translational
// and rotational velocities are modified accordingly.
void get_vel_motori_constant_ratio(float *V_D, float *W_D, float *M1, float* M2) {
    float ratio;
    float biggest;
    float aux;
    int indexM = 0;

    ratio = 0.0;

    aux = INTERASSE * (*W_D);
    *M1 = *V_D - aux / 2.0;
    *M2 = aux + *M1;

    ratio = fabs(*M1) / fabs(*M2);

    if (fabs(*M1) > fabs(*M2)) {
        biggest = fabs(*M1);
        indexM = 1;
    } else {
        biggest = fabs(*M2);
        indexM = 2;
    }

    if (biggest > MAX_LIN_VEL_EVO_GIO) {
        if (indexM == 1) {
            *M1 = (*M1 / fabs(*M1)) * MAX_LIN_VEL_EVO_GIO;
            *M2 = (*M2 / fabs(*M2))*(MAX_LIN_VEL_EVO_GIO / ratio);
        } else {
            *M2 = (*M2 / fabs(*M2)) * MAX_LIN_VEL_EVO_GIO;
            *M1 = (*M1 / fabs(*M1))*(MAX_LIN_VEL_EVO_GIO * ratio);
        }
    }
    *V_D = (*M1 + *M2) / 2.0;
    *W_D = (*M2 - *M1) / INTERASSE;

}

//------------------------------------------------------------------------------

//----------------------------------------------------------------------

void genera_rettilineo(float distanza, float vel, int modalita_stepping, unsigned int* num_passi, int *vel_pic) {

    calcola_velocita(vel, modalita_stepping, vel_pic);
    calcola_passi(distanza, modalita_stepping, num_passi);
}
//----------------------------------------------------------------------



//----------------------------------------------------------------------

void calcola_velocita(float velocita, int modalita_stepping, int *vel_pic) {
    float periodo;
    if (velocita == 0.0) {
        *vel_pic = 0;
    } else {
        periodo = CIRCONFERENZA_RUOTA / (velocita * NUM_STEPS * modalita_stepping);
        *vel_pic = periodo*FREQUENZA_PIC;
    }
}
//----------------------------------------------------------------------

//----------------------------------------------------------------------

void calcola_passi(float distanza, int modalita_stepping, int *num_passi) {
    *num_passi = (unsigned int) (distanza / (CIRCONFERENZA_RUOTA)*(NUM_STEPS * modalita_stepping));
    //	*num_passi=(unsigned int)(distanza/(2*M_PI*RAGGIO_RUOTA	)*(NUM_STEPS*modalita_stepping));
}
//----------------------------------------------------------------------

//----------------------------------------------------------------------

void calcola_circonferenza(float velocita, float raggio, int *v1, int *v2) {

    float omega;
    float vaux1, vaux2;
    omega = velocita / raggio;
    printf("INPUTCIRC: V %f R %f W %f\n", velocita, raggio, omega);


    get_vel_motori_constant_ratio(&velocita, &omega, &vaux1, &vaux2);

    printf("VEL_MOTORI: %f %f cm/s\n", vaux1, vaux2);


    calcola_velocita(vaux1, TRISTEPPING, v1);
    calcola_velocita(vaux2, TRISTEPPING, v2);
    printf("VEL_ELETTRICA: %d %d ", *v1, *v2);

}

//----------------------------------------------------------------------

//----------------------------------------------------------------------

void calcola_angolo(float angolo, float velocita, int *v1, int *v2, int *num_passi) {

    float s;

    s = INTERASSE / 2.0 * angolo;
    calcola_passi(s, TRISTEPPING, num_passi);
    calcola_velocita(velocita, TRISTEPPING, v1);
    *v2 = -(*v1);
}
//----------------------------------------------------------------------



//------------------------------------------------------------------------------

int get_offset_sensore(unsigned char* payload, int l, sensore s, int *verifica) {


    unsigned short int contatore = 0;
    contatore += *payload;
    int i, j;
    j = 1;
    for (i = 0; i < s->num_canali; i++) {
        s->bias[i] = 0;
    }
    for (i = 1; i < l - 2; i++) {

        if (*(payload + j) == PACKET_SOGLIA) {
            j++;
            *(payload + j) -= (PACKET_SOGLIA + PACKET_OFFSET_MASCHERAMENTO);
        }

        contatore += *(payload + j);
        //printf("C: %02x CONTATORE: %02x\n",*(payload+j), contatore);
        //s->range[i/2]+=256*(i%2)*(*(payload+j));
        *(s->bias + ((i - 1) / 2)) = *(s->bias + ((i - 1) / 2))+(*(payload + j)) * moltiplicatore[(i - 1) % 2];

        j++;
    }
    //printf("CONTATORE1: %02x\n", contatore);
    contatore = ~contatore;

    contatore++;

    //printf("CONTATORE2: %02x\n", contatore);
    for (i = 0; i < 2; i++) {
        if (*(payload + j) == PACKET_SOGLIA) {
            j++;
            *(payload + j) -= (PACKET_SOGLIA + PACKET_OFFSET_MASCHERAMENTO);
        }
        //printf("CONTATORE: %d\n", contatore);
        contatore -= (*(payload + j) * moltiplicatore[i]);
        j++;
    }
    //printf("CONTATORE FINALE: %02x\n", contatore);
    s->is_valid = (contatore == 0);

    printf("RICEVUTO OFFSET: %d\n", *s->bias);
    printf("*********************************************************************************************************************\n");
    *verifica &= (contatore == 0);

    return j;
}



//------------------------------------------------------------------------------

int cartesian_controller(float *state_r, float *goal_r, float k_v, float k_w, float *v_return, float *w_return) {

    float delta[3];
    float ratio;
    float err;
    
    pthread_mutex_lock(&mutex_state);
    delta[STATE_X] = *(goal_r + STATE_X)-*(state_r + STATE_X);
    delta[STATE_Y] = *(goal_r + STATE_Y)-*(state_r + STATE_Y);
    delta[STATE_THETA] = atan2(delta[STATE_Y], delta[STATE_X])-*(state_r + STATE_THETA);
    pthread_mutex_unlock(&mutex_state);
    
    if (delta[STATE_THETA] > M_PI) {
        delta[STATE_THETA] -= (2 * M_PI);
    }
    if (delta[STATE_THETA]<-M_PI) {
        delta[STATE_THETA] += (2 * M_PI);
    }

	err=sqrt( pow(delta[STATE_X],2)+pow(delta[STATE_Y],2));

	if (err > 1) {	// cm

    		*w_return = k_w * delta[STATE_THETA];

    		//Alternativa
    		*v_return=k_v*(delta[STATE_X]*cos(*(state_r+STATE_THETA))+delta[STATE_Y]*sin(*(state_r+STATE_THETA)));
    

		if (*w_return!=0 && *v_return!=0) {
			ratio = fmin(MAX_TURN_RATE / fabs(*w_return), MAX_LIN_VEL / fabs(*v_return));
		}	
	
		if (*w_return==0 && *v_return!=0) {
			ratio =  MAX_LIN_VEL / fabs(*v_return);
		}
	
		if (*w_return!=0 && *v_return==0) {
			ratio = MAX_TURN_RATE / fabs(*w_return);
		}

    		if (ratio < 1.0) {
			printf("ratio: %f\n",ratio);  
			*w_return *= ratio;
        		*v_return *= ratio;
    		}
		return 0;
	}
	else {
		*w_return =0;
		*v_return =0;
		return 1;
	}



}
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------

int pose_controller(float *error, float k_v, float k_w, float *v_return, float *w_return) {

    float ratio;
    float aux;

    if (*(error + STATE_THETA) > M_PI) {
        *(error + STATE_THETA) -= (2 * M_PI);
    }
    if (*(error + STATE_THETA)<-M_PI) {
        *(error + STATE_THETA) += (2 * M_PI);
    }
    aux = *(error + STATE_THETA);



    *w_return = k_w *aux; // (*(error+STATE_THETA));
    *v_return = k_v * (sqrt(*(error + STATE_X)*(*(error + STATE_X)) + *(error + STATE_Y)*(*(error + STATE_Y))));

    //Alternativa
    //*v_return=k_v*(delta[STATE_X]*cos(*(state_r+STATE_THETA))+delta[STATE_Y]*sin(*(state_r+STATE_THETA)));
    ratio = fmin(MAX_TURN_RATE / (abs(*w_return)), MAX_LIN_VEL / abs(*v_return));

    //printf("INGRESSI: %f %f RATIO %f\n", *v_return, *w_return, ratio);
    if (ratio < 1.0) {

        *w_return *= ratio;
        *v_return *= ratio;
    }

    return 0;


}
//------------------------------------------------------------------------------




//------------------------------------------------------------------------------

int posture_controller(float *state_r, float *goal_r, float k_v, float k_w, float k_3, float *v_return, float *w_return) {
    float delta[3];
    float ratio;
    float rho;
    float gamma;
    float deltaAng;

    pthread_mutex_lock(&mutex_state);
    delta[STATE_X] = *(goal_r + STATE_X)-*(state_r + STATE_X);
    delta[STATE_Y] = *(goal_r + STATE_Y)-*(state_r + STATE_Y);
    delta[STATE_THETA] = atan2(delta[STATE_Y], delta[STATE_X])-*(state_r + STATE_THETA); //*(goal_r+STATE_THETA)-*(state_r+STATE_THETA);
    gamma = delta[STATE_THETA];
    rho = (sqrt(delta[STATE_X] * delta[STATE_X] + delta[STATE_Y] * delta[STATE_Y]));
    deltaAng = gamma + *(state_r + STATE_THETA);
    pthread_mutex_unlock(&mutex_state);
    //    if(delta[STATE_THETA]>M_PI){delta[STATE_THETA]-=(2*M_PI);}
    //  if(delta[STATE_THETA]<-M_PI){delta[STATE_THETA]+=(2*M_PI);}


    printf("delta: %f %f %f\n", delta[0], delta[1], delta[2]);
    *v_return = k_v * rho * cos(gamma);
    *w_return = k_w * gamma + k_v * ((sin(gamma) * cos(gamma)) / gamma)*(gamma + k_3 * deltaAng);

    ratio = fmin(MAX_TURN_RATE / ((*w_return)), MAX_LIN_VEL / (*v_return));

    if (ratio < 1.0) {

        *w_return *= ratio;
        *v_return *= ratio;
    }

}

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

int decoupled_controller(float *state_r, float *goal_r, float *set_lin_return, float *set_w_return) {


    float delta[2];
    delta[STATE_X] = *(state_r + STATE_X)-*(goal_r + STATE_X);
    delta[STATE_Y] = *(state_r + STATE_Y)-*(goal_r + STATE_Y);
    *set_w_return = *(state_r + STATE_THETA)-*(goal_r + STATE_THETA);
    *set_lin_return = (sqrt(delta[STATE_X] * delta[STATE_X] + delta[STATE_Y] * delta[STATE_Y]));

}

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Given the translational and rotational velocities first the two wheels velocities
// are computed, then the velocities for the pic are derived. The wheels velocities 
// are represented as cm/sec while the pic velocities are represented as pulse/sec. 
void set_robot_speed(float *linear_speed, float *angular_speed) {

float linear_speed_limited=*linear_speed;
 float angular_speed_limited=*angular_speed;

//printf("last_ref: v:[%f], w:[%f]\n", last_v_ref, last_w_ref);
    /*Backward differentiation*/

    float back_diff_v_ref = (*linear_speed - last_v_ref) / Hz_4;
    float back_diff_w_ref = (*angular_speed - last_w_ref) / Hz_4;

    /**************************/
//printf("speed BEFORE SATURATION - linear: %f  \t angular: %f\n",*linear_speed, *angular_speed);
    /*Saturation*/
    if (fabs(back_diff_v_ref) > max_lin_acc && fabs(*linear_speed)>fabs(last_v_ref))
        linear_speed_limited = last_v_ref + (max_lin_acc * Hz_4)*(*linear_speed / fabs(*linear_speed));
    if (fabs(back_diff_w_ref) > max_ang_acc && fabs(*angular_speed)>fabs(last_w_ref))
       angular_speed_limited= last_w_ref + (max_ang_acc * Hz_4)*(*angular_speed / fabs(*angular_speed));

    /************/

    /*From linear and angular speed to each engine velocity*/
   //printf("saturated speed -> linear: %f  \t angular: %f\n", linear_speed_limited , angular_speed_limited);
	get_vel_motori_constant_ratio(&linear_speed_limited,&angular_speed_limited, &v_m1, &v_m2);
    /*Getting the number of pulses*/
    calcola_velocita(v_m1, TRISTEPPING, &pulse_m1);
    calcola_velocita(v_m2, TRISTEPPING, &pulse_m2);
	//printf("vm1: %f ---- vm2: %f\n",v_m1, v_m2);

    /*Pulses Saturation*/
    if(pulse_m1 != 0)
    {
    	pulse_m1 = (int)fmin(fmax(-900.0, (double)pulse_m1), 900.0);
        pulse_m1 = pulse_m1 / abs(pulse_m1)*(1024 - abs(pulse_m1));

    }
    if(pulse_m2 != 0)
    {
	pulse_m2 = (int)fmin(fmax(-900.0, (double)pulse_m2), 900.0);
        pulse_m2 = pulse_m2 / abs(pulse_m2)*(1024 - abs(pulse_m2));
    }
    /*Writing onto the pic serial buffer*/
    set_vel_2_array(pulse_m1, pulse_m2);
	// Save the latest translational and rotational velocities
    last_v_ref = linear_speed_limited;
    last_w_ref =  angular_speed_limited;
}

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// To get the state of the robot
void get_robot_state(float **robot_state) {
    pthread_mutex_lock(&mutex_state);
    (*robot_state)[STATE_X] = state[STATE_X];
    (*robot_state)[STATE_Y] = state[STATE_Y];
    (*robot_state)[STATE_THETA] = state[STATE_THETA];
	//printf("[ROBOT CORE] internal Theta: %6.2f\n", state[STATE_THETA]);
	//printf("[ROBOT CORE] external theta: %6.2f\n", (*robot_state)[STATE_THETA]);
    pthread_mutex_unlock(&mutex_state);
}

//------------------------------------------------------------------------------

void close_robot() {
    pthread_mutex_destroy(&mutex_fp);
    pthread_mutex_destroy(&mutex_state);
    close_sensors();
}


// modalita_stepping = TRISTEPPING
void step2vel(int step1, int step2, float *vref, float *wref) {

        // First step is to convert pulse to wheels velocities
        float v_m1, v_m2;
        float modalita_stepping = TRISTEPPING;
        int s1, s2;

	// If the step1 and step2 are equal to zero it means no command
	// has been set to the robot yet (home condition)
	if (step1 & step2) {
        	//s1= signum(step1)*1024-step1;
        	//s2= sgn(step2)*1024-step2;
		s1=(int)copysign((double)1024,(double)step1)-(double)step1;
		s2=(int)copysign((double)1024,(double)step2)-(double)step2;

        	if (s1)
        	        v_m1= (float )( CIRCONFERENZA_RUOTA / (s1 * NUM_STEPS * modalita_stepping)   )*FREQUENZA_PIC;
        	else
        	        v_m1=0;
	
        	if (s2)
        	        v_m2= (float )( CIRCONFERENZA_RUOTA / (s2 * NUM_STEPS * modalita_stepping)   )*FREQUENZA_PIC;
        	else
                	v_m2=0;


        	// Second step is to conver the wheels velocity to vref and wref

        	*wref = (v_m2 - v_m1)/INTERASSE;
        	*vref = (v_m1 + v_m2)/2.0;
#ifdef DEBUG
        	printf("s1: %d   s2: %d  step1: %d    step2: %d\n",s1, s2, step1, step2);
        	printf("v_m1: %f v_m2: %f\tvref: %f    wref: %f\n",v_m1, v_m2, *vref, *wref);
#endif
	}
	else {
#ifdef DEBUG
		printf("step1=0 step2=0\n");
#endif
		*vref=0;
		*wref=0;
	}

}




