/*
 * Initialize the Screen_sdl structure 
 *
 * author: Nicolas Van der Noot
 */

#include "plot_save_vectors.h"

/*
 * Modify this function according to your choice for:
 *    -> the colors of the curves
 *    -> the number of legends (maximum 6)
 *    -> the legends names (maximum 'nb_legend_curves')
 */
Screen_sdl* configure_screen_sdl_plot_save(int init_t_sec, int init_t_usec, double y_min_init, double y_max_init, int size_vector, int nb_curves)
{
	// -- Variables declaration -- //

	int nb_legend_curves;
	char **label_curves;

	// ------ CURVES DEFINITION ------ //

	// colors of the vectors -> modify this order according to your will but keep the same number (NB_CURVES_MAX) of colors

	int color_vec[NB_CURVES_MAX] = {BLUE_SDL, RED_SDL, DARK_GREEN_SDL, PURPLE_SDL, ORANGE_SDL,
									LIGHT_BLUE_SDL, PINK_SDL, TURQUOISE_SDL, LIGHT_GREEN_SDL, 
									DARK_YELLOW_SDL, YELLOW_SDL, GREEN_SDL};

	// do not modify
	label_curves = get_char_tab(NB_CURVES_MAX, 20); 

	// legends (with scaling) to show -> modify (maximum 6)
	nb_legend_curves = 6;

	// legends of the curves -> to modify (maximum nb_legend_curves)
	label_curves[0] = "vec_1";
	label_curves[1] = "vec_2";
	label_curves[2] = "vec_3";
	label_curves[3] = "vec_4";
	label_curves[4] = "vec_5";
	label_curves[5] = "vec_6";

	// ------------------------------ //

	return init_screen_sdl(init_t_sec, init_t_usec, size_vector, FQC_SDL, color_vec, y_min_init, y_max_init, nb_curves, nb_legend_curves, label_curves);
}

/*
 * Modify the content of this function according to the vectors to plot 
 *    (in the StandaloneC/src/other/save_vectors/vectors folder)
 */
char** get_vec_names(int *nb_curves)
{
	char **vec_names;

	// vectors creation
	vec_names = get_char_tab(NB_CURVES_MAX+1, PATH_MAX_LENGTH);

	// number of curves -> to modify (maximum 12)
	*nb_curves = 6;

	// name of the time vector -> to modify
	vec_names[0] = "vec_t.txt";

	// names of the vectors to plot -> to modify
	vec_names[1] = "vec_1.txt";
	vec_names[2] = "vec_2.txt";
	vec_names[3] = "vec_3.txt";
	vec_names[4] = "vec_4.txt";
	vec_names[5] = "vec_5.txt";
	vec_names[6] = "vec_6.txt";

	return vec_names;
}
