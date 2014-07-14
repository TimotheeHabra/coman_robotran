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
 
	// get event (if any)
	if (SDL_PollEvent(&event)) 
	{
		if (real_time->simu_break)
		{
			real_time->start_break_usec = cur_t_usec;
		}

		// analyze the type of event
		switch (event.type)
		{
			// keyboard button pressed
			case SDL_KEYDOWN:

				// id of the key -> uncomment the following line to know the corresponding keyboard id key
				// printf("Key pressed: %d\n",event.key.keysym.sym);

				// handles keyboard events
		        switch (event.key.keysym.sym)
		        {
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
		        		break;

		        	// go backward in simulation visualization (key: g)
		        	case 103:
		        		break;
			        	
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
							mouse_sdl(screen_sdl, real_time, tsim, 1);
						}
						// mouse wheel down
						else if (event.wheel.y < 0)
						{
							mouse_sdl(screen_sdl, real_time, tsim, -1);
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
	}
}
