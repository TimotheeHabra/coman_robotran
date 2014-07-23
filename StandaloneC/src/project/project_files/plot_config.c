/*
 * Configure the SDL screen to plot simulations values
 *
 * author: Nicolas Van der Noot
 */
#if defined(SDL) & defined(REAL_TIME)

#include "plot_sdl.h"
#include "main_simulation.h"
#include "simu_def.h"
#include "controller_def.h"
#include "info_project.h"

/*
 * Assign values for the SDL functions
 */
void get_screen_sdl_functions(double *y_vec, MBSdataStruct *MBSdata)
{
	// -- Variables declaration -- //

	UserIOStruct *uvs;
    ControllerStruct *cvs;


    // -- Variables initialization -- //

    uvs = MBSdata->user_IO;
    cvs = uvs->cvs;

    // ------ ASSIGN VALUES FOR THE CURVES ------ //

   /*
    * Starts from y_vec[0], up to y_vec[nb_curves-1]
    * -> Assigns the variable to plot
    * (nb_curves is defined below in the 'configure_screen_sdl' function)
    *
    * example:
    *   y_vec[0] = MBSdata->q[1];
    *   y_vec[1] = MBSdata->q[2];
    *   y_vec[2] = MBSdata->q[3];
    */

    
    // right arm: reference and actual position for the elbow [rad]
    y_vec[0] = cvs->q_ref_r_elb;  // reference
    y_vec[1] = MBSdata->q[R_ELB]; // actual position

    // left arm: reference and actual position for the elbow [rad]
    y_vec[2] = cvs->q_ref_l_elb;  // reference
    y_vec[3] = MBSdata->q[L_ELB]; // actual position
}

/*
 * Configuration of the SDL screen to plot curves
 */
Screen_sdl* configure_screen_sdl(int init_t_sec, int init_t_usec)
{
	// -- Variables declaration -- //

	int nb_curves, nb_legend_curves;

	double y_min_init, y_max_init;

	char **label_curves;


	// ------ CURVES DEFINITION ------ //

	// colors of the vectors -> modify this order according to your will but keep the same number (NB_CURVES_MAX = 12) of colors

	int color_vec[NB_CURVES_MAX] = {BLUE_SDL, RED_SDL, DARK_GREEN_SDL, PURPLE_SDL, ORANGE_SDL,
									LIGHT_BLUE_SDL, PINK_SDL, TURQUOISE_SDL, LIGHT_GREEN_SDL, 
									DARK_YELLOW_SDL, YELLOW_SDL, GREEN_SDL};

	label_curves = get_char_tab(NB_CURVES_MAX, 20); // do not modify

	// number of curves

	nb_curves        = 4; // curves to plot (maximum: NB_CURVES_MAX = 12)
	nb_legend_curves = 4; // legends (with scaling) to show (maximum: 6)

	// initial y bounds

	y_min_init = -1.3;  // y min value
	y_max_init =  0.25; // y max value

	
    /*
     * Labels of the curves
     *
     * Starts from label_curves[0], up to label_curves[nb_legend_curves-1]
     * -> Assigns the names of the variable to plot
     *
     * example:
     *   label_curves[0] = "q1";
     *   label_curves[1] = "q2";
     *   label_curves[2] = "q3";
     *
     * Beware: " " is accpeted, "" is prohibited !
     */
	label_curves[0] = "right ref [rad]";
    label_curves[1] = "right pos [rad]";
    label_curves[2] = "left ref [rad]";
    label_curves[3] = "left pos [rad]";

	// ------------------------------ //

	return init_screen_sdl(init_t_sec, init_t_usec, NB_SIMU_STEPS, FQC_SCREEN, color_vec, y_min_init, y_max_init, nb_curves, nb_legend_curves, label_curves);
}

#endif


