#ifndef _PIC_RELATED_H
#define _PIC_RELATED_H

///\brief	PIC clock frequency
#define 	FREQUENZA_PIC	20000
///\brief	constant period for 4 Hz
#define 	Hz_4	0.25
///\brief	constant period for 5 Hz
#define 	Hz_5	0.2
///\brief	main systime period
#define 	T	Hz_4
///\brief	saturation value for the pic steps counter
#define 	MAX_STEPS_COUNTER	65535
///\brief	threshold over which an anomaly steps occurs
#define 	THRESHOLD_STEPS_COUNTER	200

///\brief	counter for the number of 200[ms] steps 
unsigned int 	clock_counter;
#endif