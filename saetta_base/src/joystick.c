#include "joystick.h"

float vel,turn,command_vel,command_turn;
int NoJoy = 0,
    global_event = 0,
    button[4],
    button_status[4],
    button_counter[4],
    button_debouncingup[4],
    button_debouncingdown[4];
long button_interdiction[4];

SDL_Joystick *mystick;

void initJoy (void)
{
    if (!(SDL_Init(SDL_INIT_EVENTTHREAD | SDL_INIT_JOYSTICK) < 0))
    {
        if (SDL_NumJoysticks() > 1)
        {
            mystick = SDL_JoystickOpen(1);
            if(mystick != NULL)
                NoJoy = 0;
            else
                NoJoy = 1;
        }
        else
        {
            if (SDL_NumJoysticks() > 0)
            {
                mystick = SDL_JoystickOpen(0);
                if(mystick != NULL)
                    NoJoy = 0;
                else
                    NoJoy = 1;
            }
            else
                NoJoy = 1;
        }

        
    }
    SDL_JoystickEventState(SDL_QUERY);
    const char *name;
	int i;
    for ( i=0; i<SDL_NumJoysticks(); ++i ) {
		name = SDL_JoystickName(i);
		printf("Joystick %d: %s\n",i,name ? name : "Unknown Joystick");
		mystick = SDL_JoystickOpen(i);
		if (mystick == NULL) {
			fprintf(stderr, "SDL_JoystickOpen(%d) failed: %s\n", i, SDL_GetError());
		} else {
			printf("       axes: %d\n", SDL_JoystickNumAxes(mystick));
			printf("      balls: %d\n", SDL_JoystickNumBalls(mystick));
			printf("       hats: %d\n", SDL_JoystickNumHats(mystick));
			printf("    buttons: %d\n", SDL_JoystickNumButtons(mystick));
			SDL_JoystickClose(mystick);
		}
	}
}

void JoystickLoop(void)
{
    int i;
    for (i=0;i<4;i++)
        {
            if (button_interdiction[i]>0)
                button_interdiction[i]--;
        }
        if (button_status[0]==1 && button_interdiction[0]==0)
        {
            button_status[0]=0;
            button_interdiction[0]=2000000;
            //pause();
        }
        
        if (button_status[1]==1 && button_interdiction[1]==0)
        {
            button_status[1]=0;
            button_interdiction[1]=2000000;

            //pause();
        }
        
        if (button_status[2]==1 && button_interdiction[2]==0)
        {
            button_status[2]=0;
            button_interdiction[2]=2000000;
            //pause();
        }
        
        if (!NoJoy)
        {
            SDL_JoystickUpdate();
            vel = SDL_JoystickGetAxis(mystick,1);
            turn = SDL_JoystickGetAxis(mystick,2);
            for (i=0; i<4;i++)
            {
                button[i]=SDL_JoystickGetButton(mystick, i);
                if (button[i] != button_status[i])
                {
                    if (button_debouncingup[i] == 0 && button_debouncingdown[i] == 0)
                    {
                        if (button[i] < button_status[i])
                        {
                            button_debouncingup[i] = 1;
                        }
                        else
                        {
                            button_debouncingdown[i]=1;
                        }
                    }
                    if( button_debouncingup[i]==1 && button[i] < button_status[i] )                    
                        button_counter[i]++;
                    
                    if (button_debouncingdown[i]==1 && button[i] > button_status[i])
                        button_counter[i]++;
                    
                    if ((button_debouncingup[i]==1 && button[i] > button_status[i]) || (button_debouncingdown[i]==1 && button[i] < button_status[i]))
                    {
                        button_counter[i]=0;
                        button_debouncingup[i]=0;
                        button_debouncingdown[i]=0;
                    }
                    
                    if (button_counter[i]>500 && button_debouncingdown[i])
                    {
                        button_counter[i]=0;
                        button_debouncingdown[i]=0;
                        button_status[i]=1;
                    }
                    if (button_counter[i]>200 && button_debouncingup[i])
                    {
                        button_counter[i]=0;
                        button_debouncingup[i]=0;
                        button_status[i]=0;
                    }
                }
                else
                {
                    button_counter[i]=0;
                    button_debouncingup[i]=0;
                    button_debouncingdown[i]=0;
                }
            }
            
            command_vel = COEFF*command_vel + (1-COEFF) * (-vel/32768);
            command_turn = COEFF*command_turn + (1-COEFF) * (-turn/32768);
//		command_vel=-vel;
//		command_turn=-turn;
            
            
        }
}
