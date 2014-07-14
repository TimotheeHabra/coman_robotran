/*
 * Header of the main files
 *
 * author: Nicolas Van der Noot
 */

#ifndef __MAIN_SIMULATION_H_INCLUDED__  // guard against multiple/recursive includes
#define __MAIN_SIMULATION_H_INCLUDED__

#include "MBSdataStruct.h"
#include "info_project.h"
#include "simu_in_out.h"
#include "nrutil.h"

#if defined(JNI) & defined (REAL_TIME)
#include "jni_functions.h"
#endif

#include "save_vectors.h"

#if defined(SDL) & defined (REAL_TIME)
#include "plot_sdl.h"
#endif

#ifdef REAL_TIME
#include "real_time.h"
#endif

// -- Macros -- //

#define PATH_MAX_LENGTH 200

#ifdef ADAPTIVE_TIME_STEP
#define NB_SIMU_STEPS ( (int) ((TSIM_END - TSIM_INIT) / DELTA_TSIM) * SAFETY_FACTOR_ADAPTIVE_STEP )
#else
#define NB_SIMU_STEPS ( (int) ((TSIM_END - TSIM_INIT) / DELTA_TSIM) )
#endif


// arguments of the main loop
typedef struct Loop_arguments
{
    int nvar;
    int simu_go;
    int init_t_sec, init_t_usec;
    int final_nb_steps;
    
    double x, h;
    double *v,*vout,*dv;

    LocalDataStruct *lds;
    MBSdataStruct   *MBSdata;

    #if defined(WRITE_FILES) || defined(REAL_TIME)
    Save_vectors *save_vectors;
    #endif

    // not update real-time and storage at each time step
    #ifdef ADAPTIVE_TIME_STEP
    double next_update_adaptive;
    #endif

    #ifdef REAL_TIME
    // real-time (in us) vs simulation (in s) variables
    int    speed_last_t_usec;
    double speed_last_tsim;

    // Real-time constraints
    Real_time_constraint **constraints;

    // Real-time constraints main structure
    Simu_real_time *real_time;
    #endif

    // SDL
    #if defined(SDL) & defined(REAL_TIME)
    Screen_sdl *screen_sdl;
    #endif

    // JNI
    #if defined(JNI) & defined (REAL_TIME)
    JNI_struct* jni_struct;
    #endif

} Loop_arguments;

// -- Prototypes -- //

OutputSimu* model_eval(InputSimu *inputSimu);
Loop_arguments* init_simulation(void);
OutputSimu* loop_simulation(Loop_arguments *loop_arguments);
void loop_iteration(Loop_arguments *loop_arguments);
void finish_simulation(Loop_arguments *loop_arguments);

// last break gestion
#if defined (SDL) & defined (REAL_TIME)
void last_break_gestion(Loop_arguments *loop_arguments);
#endif

// definig the differential equation for dopri5
#ifdef ADAPTIVE_TIME_STEP
void f_prim_dopri5 (unsigned n, double x, double *y, double *f, Loop_arguments *loop_arguments);
#endif

#endif
