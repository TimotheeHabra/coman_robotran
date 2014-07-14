/*
 * Gestion of the break (whole execution in this case) used to display the plot
 *
 * author: Nicolas Van der Noot
 */

#include "plot_save_vectors.h"

/*
 * Break gestion: handles user inputs and shows the corresponding SDL graph
 */
void break_gestion_plot_save(Screen_sdl *screen_sdl, Simu_real_time *real_time, int init_t_sec, int init_t_usec, double tsim)
{
	// variables declaration
	int cur_t_usec;

	// first break plot
	update_x_min_max(screen_sdl, real_time);
	update_scale_signals(screen_sdl, real_time, 1);
	plot_screen_sdl(screen_sdl, real_time, tsim, 1);

	while (real_time->simu_break == 1)
	{
		// handle events
		cur_t_usec = t_usec(init_t_sec, init_t_usec);
		events_sdl_plot_save(screen_sdl, real_time, cur_t_usec, tsim);

		// break plot
		if (screen_sdl->break_plot_flag)
		{
			screen_sdl->break_plot_flag = 0;

			update_scale_signals(screen_sdl, real_time, 1);
			plot_screen_sdl(screen_sdl, real_time, tsim, 1);
		}

		// program is sleeping during 25 ms
		// decrease CPU usage during break if user is not interacting
		if (t_usec(init_t_sec, init_t_usec) - real_time->last_action_break_usec > TIME_NO_INTERACTION_BREAK)
		{
			SDL_Delay(TIME_SDL_DELAY);
		}	
	}
}
