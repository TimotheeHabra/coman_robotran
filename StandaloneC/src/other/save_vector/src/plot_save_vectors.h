/*
 * Main header used to plot the saved vectors (post-process)
 *
 * author: Nicolas Van der Noot
 */

#ifndef _PLOT_SAVE_VECTORS_H_
#define _PLOT_SAVE_VECTORS_H_

#include <stdlib.h>
#include <stdio.h>

#include "cmake_config.h"
#include "plot_sdl.h"

// frequency of the SDL plot in Real-time (no impact here)
#define FQC_SDL 50

// char maximal sizes
#define FGETS_MAX_SIZE 50
#define PATH_MAX_LENGTH 200

// maximal number of curves
#define NB_CURVES_MAX 12

// functions prototype
double* read_vector(char *fileName, int *size_vector);
Screen_sdl* configure_screen_sdl_plot_save(int init_t_sec, int init_t_usec, double y_min_init, double y_max_init, int size_vector, int nb_curves);
void break_gestion_plot_save(Screen_sdl *screen_sdl, Simu_real_time *real_time, int init_t_sec, int init_t_usec, double tsim);
void events_sdl_plot_save(Screen_sdl *screen_sdl, Simu_real_time *real_time, int cur_t_usec, double tsim);
char** get_vec_names(int *nb_curves);

#endif
