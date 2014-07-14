/*
 * Handles the events coming from the keyboard 
 * or from other sources via the SDL framework
 *
 * author: Nicolas Van der Noot
 */
#if defined(SDL) & defined(REAL_TIME)

#include "plot_sdl.h"
#include "visu_past.h"
#include "controller_def.h"

// personnal joystick values
#define MIN_JOYSTICK -32768
#define MAX_JOYSTICK 32767
#define MEAN_JOYSTICK (0.5 * (MIN_JOYSTICK + MAX_JOYSTICK))
#define DIFF_JOYSTICK (MAX_JOYSTICK - MIN_JOYSTICK) 

#define MAX_KEYBOARD_COMMAND 10.0

// shoulder sagittal (pitch)
#define MIN_SH_SAG  ( -60.0 * DEG_TO_RAD )
#define MAX_SH_SAG  (  10.0 * DEG_TO_RAD )
#define DIFF_SH_SAG (MAX_SH_SAG - MIN_SH_SAG)

// shoulder lateral (roll)
#define MIN_SH_LAT  ( -100.0 * DEG_TO_RAD )
#define MAX_SH_LAT  (  20.0  * DEG_TO_RAD )
#define DIFF_SH_LAT (MAX_SH_LAT - MIN_SH_LAT)

// shoulder yaw
#define MIN_SH_YAW  ( -10.0 * DEG_TO_RAD )
#define MAX_SH_YAW  (  40.0 * DEG_TO_RAD )
#define DIFF_SH_YAW (MAX_SH_YAW - MIN_SH_YAW)

// elbow
#define MIN_ELB  ( -60.0 * DEG_TO_RAD )
#define MAX_ELB  (   0.0 * DEG_TO_RAD )
#define DIFF_ELB (MAX_ELB - MIN_ELB)


/*
 * Handles the events (SDL)
 */
void events_sdl(Screen_sdl *screen_sdl, Simu_real_time *real_time, MBSdataStruct *MBSdata, int cur_t_usec)
{
	// -- Vraiables declaration -- //

	SDL_Event event;

	const Uint8 *keystates;

	UserIOStruct     *uvs;
    ControllerStruct *cvs;

    double comman_val_r, comman_val_l;

    // -- Variables initialization -- //

    uvs = MBSdata->user_IO;
    cvs = uvs->cvs;

    
    // -- Handles the events -- //

    // SDL events refresh
	SDL_PumpEvents();


	keystates = SDL_GetKeyboardState(NULL);

	if (cur_t_usec >= real_time->next_user_keyboard_event_usec) 
	{

		// ----- MODIFICATIONS HERE (KEYBOARD INPUTS) ----- //

	    /*
	     * Use keystates['key code'] (see https://wiki.libsdl.org/SDL_Scancode)
	     * to detect when the corresponding key is pressed and add your own functions
	     * according to the key pressed.
	     *
	     * Call 'wait_key(real_time, cur_t_usec, time);' in the statement where you replace 'time'
	     * by a time in seconds if you want to wait before detecting a new user command.
	     *
	     * example:
		 *      if (keystates[SDL_SCANCODE_UP]) 
		 *      {
		 *          MBSdata->user_IO->my_command++;
		 *          wait_key(real_time, cur_t_usec, 0.1);
		 *      }
		 *      else if (keystates[SDL_SCANCODE_DOWN]) 
		 *      {
		 *          MBSdata->user_IO->my_command++;
		 *          wait_key(real_time, cur_t_usec, 0.1);
		 *      }
		 */

		// Up arrow: keyboard command 1 increase
		if (keystates[SDL_SCANCODE_UP]) 
		{
		    uvs->keyboard_command_1++;

		    wait_key(real_time, cur_t_usec, 0.1);
		}

		// Down arrow: keyboard command 1 decrease
		else if (keystates[SDL_SCANCODE_DOWN]) 
		{
		    uvs->keyboard_command_1--;

		    wait_key(real_time, cur_t_usec, 0.1);
		}

		// Right arrow: keyboard command 2 increase
		else if (keystates[SDL_SCANCODE_RIGHT]) 
		{
		    uvs->keyboard_command_2++;

		    wait_key(real_time, cur_t_usec, 0.1);
		}

		// Left arrow: keyboard command 2 decrease
		else if (keystates[SDL_SCANCODE_LEFT]) 
		{
		    uvs->keyboard_command_2--;

		    wait_key(real_time, cur_t_usec, 0.1);
		}

		// --------------- AVOID MODIFYING --------------- //
	}



    // ----- MODIFICATIONS HERE (JOYSTICK AXES) ----- //

    /*
     * Use get_Joystick_axis(int joystickID, int axisID, Screen_sdl *screen_sdl)
     * to return the value associated with the joystick number joystickID
     * returns 0.0 if this joystickID is not available (joystickID starts at 0)
     *
     * example:
     *     joystick_val = get_Joystick_axis(0, 2, screen_sdl);
     *     MBSdata->user_IO->my_variable = joystick_val * scaling_factor;
     */

     if (screen_sdl->nb_joysticks) // at least one joytick detected
     {
     	// value bounded in the [0 ; 1] interval
     	comman_val_l = ( (-get_Joystick_axis(0, 0, screen_sdl) - MEAN_JOYSTICK) / DIFF_JOYSTICK) + 0.5; // left
     	comman_val_r = ( (-get_Joystick_axis(0, 2, screen_sdl) - MEAN_JOYSTICK) / DIFF_JOYSTICK) + 0.5; // right

     	
     }
     else // otherwise, keyboard keys are used
     {
     	uvs->keyboard_command_1 = limit_value(uvs->keyboard_command_1, -MAX_KEYBOARD_COMMAND, MAX_KEYBOARD_COMMAND);
     	uvs->keyboard_command_2 = limit_value(uvs->keyboard_command_2, -MAX_KEYBOARD_COMMAND, MAX_KEYBOARD_COMMAND);

     	// value bounded in the [0 ; 1] interval
     	comman_val_l = ( uvs->keyboard_command_1 / ( 2.0 * MAX_KEYBOARD_COMMAND ) ) + 0.5; 
     	comman_val_r = ( uvs->keyboard_command_2 / ( 2.0 * MAX_KEYBOARD_COMMAND ) ) + 0.5; 
     }	 

    #ifdef STANDALONE
	// left arm position reference [rad]
	cvs->q_ref_l_sh_sag =  MIN_SH_SAG + comman_val_l * DIFF_SH_SAG;
	cvs->q_ref_l_sh_lat = -MAX_SH_LAT + comman_val_l * DIFF_SH_LAT;
	cvs->q_ref_l_sh_yaw = -MAX_SH_YAW + comman_val_l * DIFF_SH_YAW;
	cvs->q_ref_l_elb    =  MIN_ELB    + comman_val_l * DIFF_ELB; 

	// right arm position reference [rad]
	cvs->q_ref_r_sh_sag = MIN_SH_SAG + comman_val_r * DIFF_SH_SAG;
	cvs->q_ref_r_sh_lat = MAX_SH_LAT - comman_val_r * DIFF_SH_LAT;
	cvs->q_ref_r_sh_yaw = MAX_SH_YAW - comman_val_r * DIFF_SH_YAW;
	cvs->q_ref_r_elb    = MIN_ELB    + comman_val_r * DIFF_ELB; 

	// correction for the long arms case
	#ifdef LONG_ARMS

	// frame change
	cvs->q_ref_l_sh_lat -= PI_2;
	cvs->q_ref_r_sh_lat += PI_2;

	// adapt to arms morphology
	cvs->q_ref_l_sh_yaw = 0.0;
	cvs->q_ref_l_elb    = 0.0;
	cvs->q_ref_r_sh_yaw = 0.0;
	cvs->q_ref_r_elb    = 0.0;
	#endif

	#endif  

    // --------------- AVOID MODIFYING --------------- //
 

	// get event (if any)
	if (SDL_PollEvent(&event)) 
	{
		// analyze the type of event
		switch (event.type)
		{
			// joystick button pressed
			case SDL_JOYBUTTONDOWN:

				// ID of the joystick button -> uncomment the following line to know the corresponding joystick ID button
				// printf("Joystick button pressed: %d\n",event.jbutton.button);

				switch(event.jbutton.button)
				{
					// ----- MODIFICATIONS HERE (JOYSTICK BUTTONS) ----- //

					/*
					 * To configure the actions related to the pression of a button on the joystick, check that the line 
					 * 'printf("Joystick button pressed: %d\n",event.jbutton.button);' (above) is uncommented.
					 * Then, when you press a button on the joystick, the corresponding ID should be printed 
					 * in the Terminal (Unix) or in the Console (Windows).
					 * 
					 * Use this switch case to define the corresponding code executed when this button is pressed.
					 * You have access to MBSdata, so you should be able to impact nearly any part of the simulation.
					 *
					 * example:
					 *     case 0:
					 *         MBSdata->user_IO->my_action = 1;
					 *         break;
					 */
					case 0:
						break;

					// ---------------- AVOID MODIFYING ---------------- //
				
					default:
						break;
				}

				break;

			// top corner button pressed 
			case SDL_QUIT:
				real_time->simu_quit  = 1;
				real_time->simu_break = 0;
				real_time->last_break = 0;
				break;

			// mouse wheel
			case SDL_MOUSEWHEEL:

				if (cur_t_usec - real_time->last_mouse_event_usec > TIME_MOUSE_DELAY)
				{
					real_time->last_mouse_event_usec = cur_t_usec;
				
					if ( (PLOT_X_START < real_time->mouse_cur_x) && (real_time->mouse_cur_x < PLOT_X_END) && 
						 (PLOT_Y_START < real_time->mouse_cur_y) && (real_time->mouse_cur_y < PLOT_Y_END) )
					{
						// mouse wheel up
						if (event.wheel.y > 0)
						{
							mouse_sdl(screen_sdl, real_time, MBSdata->tsim, 1);
							real_time->last_action_break_usec = cur_t_usec;
						}
						// mouse wheel down
						else if (event.wheel.y < 0)
						{
							mouse_sdl(screen_sdl, real_time, MBSdata->tsim, -1);
							real_time->last_action_break_usec = cur_t_usec;
						}
					}
				}
		    	break;

		    default:
		    	break;
		}
	}


	// generic keyboard inputs
	if (cur_t_usec >= real_time->next_generic_keyboard_event_usec) 
	{
		// key: P (pause)
		if (keystates[SDL_SCANCODE_P])
		{
			if (!real_time->simu_break)
        	{
        		real_time->simu_break = 1; // break flag
        		real_time->last_action_break_usec = cur_t_usec;
        	}
        	else
        	{
        		real_time->simu_break = 0;
        	}

        	wait_key_generic(real_time, cur_t_usec, 0.2);
		} 

		// key M: quit the simulation
		else if (keystates[SDL_SCANCODE_M])
		{
			real_time->simu_quit  = 1; // flag to quit the simulation
			real_time->simu_break = 0;
			real_time->last_break = 0;
		}

		// key L: decrease the simulation speed
		else if (keystates[SDL_SCANCODE_L])
		{
			real_time->simu_speed_flag--;	
			screen_sdl->break_plot_flag = 1;       

			wait_key_generic(real_time, cur_t_usec, 0.2);
		}

		// key O: increase the simulation speed
		else if (keystates[SDL_SCANCODE_O])
		{
			real_time->simu_speed_flag++;	
			screen_sdl->break_plot_flag = 1;			
			
			wait_key_generic(real_time, cur_t_usec, 0.2);
		}

		// key K: activate / deactivate the signals auto-scaling
		else if (keystates[SDL_SCANCODE_K])
		{
			if (screen_sdl->signal_auto_scaling)
			{
				screen_sdl->signal_auto_scaling = 0;
			}
			else
			{
				screen_sdl->signal_auto_scaling = 1;
			}
			screen_sdl->break_plot_flag = 1;			
			
			wait_key_generic(real_time, cur_t_usec, 0.2);
		}

		// key I: activate / deactivate the y axis auto-scaling
		else if (keystates[SDL_SCANCODE_I])
		{
			if (screen_sdl->plot_auto_scaling)
			{
				screen_sdl->plot_auto_scaling = 0;
			}
			else
			{
				screen_sdl->plot_auto_scaling = 1;
			}
			screen_sdl->break_plot_flag = 1;			
			
			wait_key_generic(real_time, cur_t_usec, 0.2);
		}

		// key U: broaden x or y axis
		else if (keystates[SDL_SCANCODE_U])
		{
			if (screen_sdl->hor_plot_scaling)
			{
				screen_sdl->increase_plot_x_diff_flag = 1;		        			
			}
			else
			{
				screen_sdl->increase_plot_y_diff_flag = 1;
				screen_sdl->plot_auto_scaling         = 0;
			}	
			screen_sdl->break_plot_flag = 1;
				
			
			wait_key_generic(real_time, cur_t_usec, 0.2);
		}

		// key J: shrink x or y axis
		else if (keystates[SDL_SCANCODE_J])
		{
			if (screen_sdl->hor_plot_scaling)
			{
				screen_sdl->decrease_plot_x_diff_flag = 1;		        			
			}
			else
			{
				screen_sdl->decrease_plot_y_diff_flag = 1;
				screen_sdl->plot_auto_scaling         = 0;
			}	
			screen_sdl->break_plot_flag = 1;			
			
			wait_key_generic(real_time, cur_t_usec, 0.2);
		}

		// key Y: shift x or y axis: up or right
		else if (keystates[SDL_SCANCODE_Y])
		{
			if (screen_sdl->hor_plot_scaling)
			{
				screen_sdl->right_plot_y_flag = 1;		        			
			}
			else
			{
				screen_sdl->up_plot_y_flag    = 1;
				screen_sdl->plot_auto_scaling = 0;
			}	
			screen_sdl->break_plot_flag = 1;			
			
			wait_key_generic(real_time, cur_t_usec, 0.2);
		}

		// key H: shift x or y axis: bottom or left
		else if (keystates[SDL_SCANCODE_H])
		{
			if (screen_sdl->hor_plot_scaling)
			{
				screen_sdl->left_plot_y_flag = 1;		        			
			}
			else
			{
				screen_sdl->bottom_plot_y_flag = 1;
				screen_sdl->plot_auto_scaling  = 0;
			}	
			screen_sdl->break_plot_flag = 1;			
			
			wait_key_generic(real_time, cur_t_usec, 0.2);
		}

		// key N: switch from y axis to x axis auto-scaling and vice-versa
		else if (keystates[SDL_SCANCODE_N])
		{
			if (real_time->simu_break)
			{
				if (screen_sdl->hor_plot_scaling)
		    	{
		    		screen_sdl->hor_plot_scaling = 0;
		    	}
		    	else
		    	{
		    		screen_sdl->hor_plot_scaling = 1;
		    	}
			}
			else
			{
				screen_sdl->hor_plot_scaling = 0;
			}
			screen_sdl->break_plot_flag = 1;			
			
			wait_key_generic(real_time, cur_t_usec, 0.2);
		}

		// key V: change viewpoint for JNI
		else if (keystates[SDL_SCANCODE_V])
		{
			real_time->change_viewpoint = 1;			
			
			wait_key_generic(real_time, cur_t_usec, 0.2);
		}

		// key T: go forward in simulation visualization
		else if (keystates[SDL_SCANCODE_T])
		{
			#ifdef JNI
			real_time->visu_past_flag = 1;
			update_t_visu_past(MBSdata, real_time, 1);
			#endif			
			
			wait_key_generic(real_time, cur_t_usec, PERIOD_BACK_IN_VISU);
		}

		// key G: go backward in simulation visualization
		else if (keystates[SDL_SCANCODE_G])
		{
			#ifdef JNI
			real_time->visu_past_flag = 1;
			update_t_visu_past(MBSdata, real_time, 0);
			#endif			
			
			wait_key_generic(real_time, cur_t_usec, PERIOD_BACK_IN_VISU);
		}
	}

	// mouse position
    SDL_GetMouseState(&(real_time->mouse_cur_x), &(real_time->mouse_cur_y));

    // left mouse button
    if (SDL_GetMouseState(NULL, NULL) & SDL_BUTTON(SDL_BUTTON_LEFT)) 
    {
    	// press left button
    	if (!real_time->mouse_left_pressed)
    	{
    		real_time->mouse_left_pressed = 1;

    		real_time->mouse_init_x = real_time->mouse_cur_x;
	        real_time->mouse_init_y = real_time->mouse_cur_y;	

	        real_time->start_mouse_usec = cur_t_usec;	        
    	}	 
    	real_time->last_action_break_usec = cur_t_usec;   
	}
	else
	{
		// release left button
		if (real_time->mouse_left_pressed)
		{
			real_time->mouse_left_pressed = 0;

			real_time->mouse_delta_x = real_time->mouse_cur_x - real_time->mouse_init_x;
	        real_time->mouse_delta_y = real_time->mouse_cur_y - real_time->mouse_init_y;
	        real_time->mouse_init_x  = real_time->mouse_cur_x;
	        real_time->mouse_init_y  = real_time->mouse_cur_y;	

	        mouse_sdl(screen_sdl, real_time, MBSdata->tsim, 0);
		}		
	}

	// right mouse button
	if (SDL_GetMouseState(NULL, NULL) & SDL_BUTTON(SDL_BUTTON_RIGHT))
	{
		// press right button
		if (!real_time->mouse_right_pressed)
		{
			real_time->mouse_right_pressed = 1;

			real_time->mouse_wheel_flag++;	

	    	if (real_time->mouse_wheel_flag > 2)
	        {
	        	real_time->mouse_wheel_flag = 0;
	        }	

	        screen_sdl->change_mouse_wheel_flag = 1;
	        plot_screen_sdl(screen_sdl, real_time, MBSdata->tsim, 1);
		}	
		real_time->last_action_break_usec = cur_t_usec;    
	}
	else
	{
		// release right button
		if (real_time->mouse_right_pressed)
		{
			real_time->mouse_right_pressed = 0;
		}		
	}

	/*
     * According to SDL 2.0 specifications, the lines with SDL_BUTTON(4) and SDL_BUTTON(5)
     * do not correspond to the mouse wheel motions anymore.
     * Only the lines with SDL_MOUSEWHEEL should be used.
     * Nevertheless, this is not always working with all mouses.
     */
	if (SDL_GetMouseState(NULL, NULL) & SDL_BUTTON(4))
	{
		if (cur_t_usec - real_time->last_mouse_event_usec > TIME_MOUSE_DELAY)
		{
			real_time->last_mouse_event_usec = cur_t_usec;
		
			if ( (PLOT_X_START < real_time->mouse_cur_x) && (real_time->mouse_cur_x < PLOT_X_END) && 
				 (PLOT_Y_START < real_time->mouse_cur_y) && (real_time->mouse_cur_y < PLOT_Y_END) )
			{
				// mouse wheel up
				mouse_sdl(screen_sdl, real_time, MBSdata->tsim, 1);
			}
			real_time->last_action_break_usec = cur_t_usec;
		}
	} 
	else if (SDL_GetMouseState(NULL, NULL) & SDL_BUTTON(5))
	{
		if (cur_t_usec - real_time->last_mouse_event_usec > TIME_MOUSE_DELAY)
		{
			real_time->last_mouse_event_usec = cur_t_usec;
		
			if ( (PLOT_X_START < real_time->mouse_cur_x) && (real_time->mouse_cur_x < PLOT_X_END) && 
				 (PLOT_Y_START < real_time->mouse_cur_y) && (real_time->mouse_cur_y < PLOT_Y_END) )
			{
				// mouse wheel down
				mouse_sdl(screen_sdl, real_time, MBSdata->tsim, -1);
			}
			real_time->last_action_break_usec = cur_t_usec;
		}
	} 


	// regulary (TIME_RELEASE_MOUSE_PLOT micro-seconds) release the plot when the left mouse button is pressed
	if ( real_time->mouse_left_pressed && (cur_t_usec - real_time->start_mouse_usec > TIME_RELEASE_MOUSE_PLOT) )
	{
		real_time->mouse_delta_x = real_time->mouse_cur_x - real_time->mouse_init_x;
        real_time->mouse_delta_y = real_time->mouse_cur_y - real_time->mouse_init_y;
        real_time->mouse_init_x  = real_time->mouse_cur_x;
        real_time->mouse_init_y  = real_time->mouse_cur_y;

        mouse_sdl(screen_sdl, real_time, MBSdata->tsim, 0);

        real_time->start_mouse_usec = cur_t_usec;

        real_time->last_action_break_usec = cur_t_usec;
	}

}

double get_Joystick_axis(int joystickID, int axisID, Screen_sdl *screen_sdl)
{
	if (joystickID >= screen_sdl->nb_joysticks)
	{
		//printf("Error: Joystick ID too high !\n");
		return 0.0;
	}

	return SDL_JoystickGetAxis(screen_sdl->joysticks[joystickID],axisID);
}

#endif
