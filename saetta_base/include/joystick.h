/* 
 * File:   joystick.h
 * Author: erupter
 *
 * Created on December 5, 2012, 4:11 PM
 */

#ifndef JOYSTICK_H
#define	JOYSTICK_H
#define COEFF	0.5
#ifdef	__cplusplus
extern "C" {
#endif
#include <SDL/SDL.h>

extern float vel,turn,command_vel,command_turn;
extern int NoJoy,
    global_event,
    button[4],
    button_status[4],
    button_counter[4],
    button_debouncingup[4],
    button_debouncingdown[4];
extern long button_interdiction[4];
extern SDL_Joystick *mystick;    

void initJoy (void);
void JoystickLoop(void);


#ifdef	__cplusplus
}
#endif

#endif	/* JOYSTICK_H */

