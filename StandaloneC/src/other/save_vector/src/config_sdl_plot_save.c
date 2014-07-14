/*
 * Handles the events coming from the keyboard 
 * or from other sources via the SDL framework
 *
 * author: Nicolas Van der Noot
 */

#include "plot_save_vectors.h"

/*
 * Handles the events (SDL)
 */
void events_sdl_plot_save(Screen_sdl *screen_sdl, Simu_real_time *real_time, int cur_t_usec, double tsim)
{
	SDL_Event event;

	const Uint8 *keystates;
 
	// -- Handles the events -- //

    // SDL events refresh
	SDL_PumpEvents();


	keystates = SDL_GetKeyboardState(NULL);


	// get event (if any)
	if (SDL_PollEvent(&event)) 
	{
		// analyze the type of event
		switch (event.type)
		{
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
							mouse_sdl(screen_sdl, real_time, tsim, 1);
							real_time->last_action_break_usec = cur_t_usec;
						}
						// mouse wheel down
						else if (event.wheel.y < 0)
						{
							mouse_sdl(screen_sdl, real_time, tsim, -1);
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

	        mouse_sdl(screen_sdl, real_time, tsim, 0);
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
	        plot_screen_sdl(screen_sdl, real_time, tsim, 1);
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
				mouse_sdl(screen_sdl, real_time, tsim, 1);
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
				mouse_sdl(screen_sdl, real_time, tsim, -1);
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

        mouse_sdl(screen_sdl, real_time, tsim, 0);

        real_time->start_mouse_usec = cur_t_usec;

        real_time->last_action_break_usec = cur_t_usec;
	}
}
