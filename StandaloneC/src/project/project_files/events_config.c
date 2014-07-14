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

	UserIOStruct     *uvs;
    ControllerStruct *cvs;

    double comman_val_r, comman_val_l;

    // -- Variables initialization -- //

    uvs = MBSdata->user_IO;
    cvs = uvs->cvs;

    // -- Handles the events -- //


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


			// keyboard button pressed
			case SDL_KEYDOWN:

				// ID of the key -> uncomment the following line to know the corresponding keyboard ID key
				// printf("Key pressed: %d\n",event.key.keysym.sym);


				// handles keyboard events
				if (cur_t_usec - real_time->last_keyboard_event_usec > TIME_PRESS_KEY_DELAY)
				{
					real_time->last_keyboard_event_usec = cur_t_usec;

			        switch (event.key.keysym.sym)
			        {
			        	// ----- MODIFICATIONS HERE (KEYBOARD KEYS) ----- //

			        	/*
						 * To configure the actions related to the pression of a key on the keyboard, check that the line 
						 * 'printf("Key pressed: %d\n",event.key.keysym.sym);' (above) is uncommented.
						 * Then, when you press a key on the keyboard, the corresponding ID should be printed 
						 * in the Terminal (Unix) or in the Console (Windows).
						 * 
						 * Use this switch case to define the corresponding code executed when this button is pressed.
						 * You have access to MBSdata, so you should be able to impact nearly any part of the simulation.
						 *
						 * example:
						 *     case 0:
						 *         MBSdata->user_IO->my_action = 1;
						 *         break;
						 *
						 * Beware: some key IDs are already sued by the curves plotted in real-time (see below).
						 */


			        	// interact (key: q)
			        	case 113:
			        		uvs->keyboard_command_1++;
			        		break;

			        	// interact (key: a)
			        	case 97:
			        		uvs->keyboard_command_1--;
			        		break;

			        	// interact (key: w)
			        	case 119:
			        		uvs->keyboard_command_2++;
			        		break;

			        	// interact (key: s)
			        	case 115:
			        		uvs->keyboard_command_2--;
			        		break;

			        	// --------------- AVOID MODIFYING --------------- //

			        	// break (key : p)
			        	case 112: 
				        	if (!real_time->simu_break)
				        	{
				        		real_time->simu_break = 1; // break flag
				        		real_time->start_break_usec = cur_t_usec;
				        	}
				        	else
				        	{
				        		real_time->simu_break = 0;
				        	}		        		
			        		break;

			        	// quit the simulation (key: m)
			        	case 109:
			        		real_time->simu_quit  = 1; // flag to quit the simulation
			        		real_time->simu_break = 0;
			        		real_time->last_break = 0;
			        		break;

			        	// decrease the simulation speed (key: l)
			        	case 108:
			        		real_time->simu_speed_flag--;	
			        		screen_sdl->break_plot_flag = 1;        		
			        		break;

			        	// increase the simulation speed (key: o)
			        	case 111: 
			        		real_time->simu_speed_flag++;	
			        		screen_sdl->break_plot_flag = 1;        		
			        		break;

			        	// activate / deactivate the signals auto-scaling (key: k)
			        	case 107:
				        	if (screen_sdl->signal_auto_scaling)
				        	{
				        		screen_sdl->signal_auto_scaling = 0;
				        	}
				        	else
				        	{
				        		screen_sdl->signal_auto_scaling = 1;
				        	}
				        	screen_sdl->break_plot_flag = 1;
			        		break;

			        	// activate / deactivate the y axis auto-scaling (key: i)
			        	case 105:
				        	if (screen_sdl->plot_auto_scaling)
				        	{
				        		screen_sdl->plot_auto_scaling = 0;
				        	}
				        	else
				        	{
				        		screen_sdl->plot_auto_scaling = 1;
				        	}
				        	screen_sdl->break_plot_flag = 1;
			        		break;
			        	
			        	// broaden x or y axis (key: u)
			        	case 117: 
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
			        		break;

			        	// shrink x or y axis (key: j)
			        	case 106:        		
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
			        		break;

			        	// shift x or y axis: up or right (key: y)
			        	case 121:      		
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
			        		break;

			        	// shift x or y axis: bottom or left (key: h)
			        	case 104:
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
			        		break;

			        	// switch from y axis to x axis auto-scaling and vice-versa (key: n)
			        	case 110:		        		
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
				        	break;

				        // change viewpoint for JNI (key: v)
				        case 118:
				        	real_time->change_viewpoint = 1;
				        	break;

				        // go forward in simulation visualization (key: t)
			        	case 116:
			        		#ifdef JNI
			        		real_time->visu_past_flag = 1;
			        		update_t_visu_past(MBSdata, real_time, 1);
			        		#endif
			        		break;

			        	// go backward in simulation visualization (key: g)
			        	case 103:
			        		#ifdef JNI
			        		real_time->visu_past_flag = 1;
			        		update_t_visu_past(MBSdata, real_time, 0);
			        		#endif
			        		break;
				        	
			        	default:
			        		break;
			        }					
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
						}
						// mouse wheel down
						else if (event.wheel.y < 0)
						{
							mouse_sdl(screen_sdl, real_time, MBSdata->tsim, -1);
						}
					}
				}
		    	break;

		    default:
		    	break;
		}
	}

	// mouse events
	SDL_PumpEvents();

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
