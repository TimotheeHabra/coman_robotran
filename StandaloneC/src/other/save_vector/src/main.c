/*
 * Main file used to plot the saved vectors (post-process)
 *
 * author: Nicolas Van der Noot
 */

#include "plot_save_vectors.h"

/*
 * Main function used to plot save vectors
 */
int main(int argc, char const *argv[])
{
	// variables declaration
	int i, j;
	int nb_curves;
	int size_vector, cur_size_vector;
	int init_t_sec, init_t_usec;

	double last_tsim;
	double cur_value, cur_min, cur_max;
	double y_min_init, y_max_init;

	char *generic_vec_file;
	char **vec_names;
	char t_file[PATH_MAX_LENGTH];
	char cur_vec_file[PATH_MAX_LENGTH];

	double *vec_t;
	double **vec_out;
	double *y_tab_min, *y_tab_max;

	Screen_sdl *screen_sdl;
	Simu_real_time *real_time;

	// vectors to plot
	vec_names = get_vec_names(&nb_curves);

	// vectors initialization
	generic_vec_file = CUR_PROJECT_ABS_PATH"/vectors";

	sprintf (t_file, "%s/vectors/%s", CUR_PROJECT_ABS_PATH, vec_names[0]);	

	vec_t = read_vector(t_file, &size_vector);

	vec_out = (double**) malloc (nb_curves*sizeof(double*));

	for (i=0; i<nb_curves; i++)
	{
		sprintf (cur_vec_file, "%s/%s", generic_vec_file, vec_names[i+1]);

		vec_out[i] = read_vector(cur_vec_file, &cur_size_vector);

		if (cur_size_vector != size_vector)
		{
			printf("Problem: all vectors do not have the same size !\n");
			printf("Time vector size: %d\n", size_vector);
			printf("Vector %s size: %d\n", cur_vec_file, cur_size_vector);
			exit(1);
		}
	}

	// max and min tabulars
	y_tab_min = (double*) malloc(nb_curves*sizeof(double));
	y_tab_max = (double*) malloc(nb_curves*sizeof(double));

	// compute minimal and maximal values
	cur_value  = vec_out[0][0];
	y_min_init = cur_value;
	y_max_init = cur_value;

	for (i=0; i<nb_curves; i++)
	{
		cur_value = vec_out[i][0];
		cur_min   = cur_value;
		cur_max   = cur_value;

		for (j=1; j<size_vector; j++)
		{
			cur_value = vec_out[i][j];

			if (cur_value < cur_min)
			{
				cur_min = cur_value;
			}

			if (cur_value > cur_max)
			{
				cur_max = cur_value;
			}
		}

		y_tab_min[i] = cur_min;
		y_tab_max[i] = cur_max;

		if (cur_min < y_min_init)
		{
			y_min_init = cur_min;
		}

		if (cur_max > y_max_init)
		{
			y_max_init = cur_max;
		}
	}

	// absolute time before starting loop
	time_get(&init_t_sec, &init_t_usec);

	// real time structure initialization
	real_time = init_real_time(init_t_sec, init_t_usec);

	// init screen SDL
	screen_sdl = configure_screen_sdl_plot_save(init_t_sec, init_t_usec, y_min_init, y_max_init, size_vector, nb_curves);

	// fill screen SDL
	for (i=0; i<nb_curves; i++)
	{
		for (j=1; j<size_vector; j++)
		{
			screen_sdl->y_vectors[i][j] = vec_out[i][j];
		}
	}

	for (i=0; i<size_vector; i++)
	{
		screen_sdl->tsim_vec[i] = vec_t[i];
	}

	for (i=0; i<nb_curves; i++)
	{
		screen_sdl->y_tab_min[i] = y_tab_min[i];
		screen_sdl->y_tab_max[i] = y_tab_max[i];
	}

	// special values
	real_time->simu_break  = 1;
	screen_sdl->index_simu = size_vector-1;
	last_tsim              = vec_t[size_vector-1];

	// plot main loop
	break_gestion_plot_save(screen_sdl, real_time, init_t_sec, init_t_usec, last_tsim);

	// release memory
	for (i=0; i<nb_curves; i++)
	{
		free(vec_out[i]);
	}

	free(vec_out);
	free(vec_t);
	free(y_tab_min);
	free(y_tab_max);
	
	free_screen_sdl(screen_sdl);
	free_simu_real_time(real_time);
	free_char_tab(vec_names);

	return 0;
}
