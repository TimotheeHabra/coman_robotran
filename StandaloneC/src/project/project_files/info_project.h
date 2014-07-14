/*
 * Macros used to define the simulation
 *
 * author: Nicolas Van der Noot
 */
#ifndef __PROJECT_INFO_H_INCLUDED__  // guard against multiple/recursive includes
#define __PROJECT_INFO_H_INCLUDED__

#include "cmake_config.h"

// -- Model -- //

// MBS file name, located in dataR (project name with .mbs)
#define PROJECT_NAME_MBS ROBOTRAN_ROJECT_NAME".mbs"

// total number of joints of the model (independent, driven and dependent)
#ifdef COMP_FEET
#define NB_JOINTS 31
#else 
#ifdef LONG_ARMS
#define NB_JOINTS 35
#else
#define NB_JOINTS 29
#endif
#endif

// -- Simulation -- //

// simulation time
#define TSIM_INIT  0.0    // inital time of the simulation  [s]
#define TSIM_END   60.0   // end time of the simulation     [s]


// -- Fixed time step integrator -- //

#define DELTA_TSIM 5.0e-4 // fixed time-step of the integrator [s]


// -- Adaptive time step integrator -- //

#define INIT_STEP_SIZE 1.0e-5 // initial time-step of the integrator [s]
#define MAX_STEP_SIZE 1.0e-3  // maximal time-step of the integrator [s]
#define R_TOLER 1.0e-3        // relative error tolerances [-]
#define A_TOLER 1.0e-3        // absolute error tolerances [-]
/* 
 * Some vectors must be allocated with their size corresponding 
 * to the number of time steps during the simulation.
 * As the final number of steps is unknown, an initial guess is multiplied 
 * by the SAFETY_FACTOR_ADAPTIVE_STEP number (see below).
 * Increase this number if a segmentation fault happens during the simulation 
 * due to the vectors lack of space.
 *
 * vector length =  ((TSIM_END - TSIM_INIT) / DELTA_TSIM) * SAFETY_FACTOR_ADAPTIVE_STEP
 */
#define SAFETY_FACTOR_ADAPTIVE_STEP 10 // [-]
/*
 * The real-time features and storage values are not updated at each time step
 * as this time step can be modified during the simulation. 
 * Consequently, these features and the storage of values are only
 * updated after a fixed minimal amount of time 
 * corresponding to the TSIM_UPDATE_STORAGE_REAL_TIME value.
 */
#define TSIM_UPDATE_STORAGE_REAL_TIME 1.0e-3 // [s]


// -- Real-time refresh -- //

/*
 * You can modify the refresh frequency of both the curves plotted in real-time (SDL window)
 * and the Java visulaization (with JNI: Java Native Interface).
 * Decrease these values if your computer is not able to handle this fast enough.
 * On top of that, FQC_SCREEN also affects the speed of the cursor in the SDL window.
 */
#define FQC_SCREEN 30.0              // frequence of the SDL window refresh in normal situation [Hz]
#define FQC_MOUSE  30.0              // frequence of the SDL window refresh when using the mouse for translations [Hz]
#define FQC_JNI    30.0              // frequence of the Java visualization window refresh [Hz]
#define NB_REAL_TIME_CONSTRAINTS 2   // number of real-time constraints -> keep '2' (do not modify)


// -- screen plot dimensions -- //

/*
 * Modify these values if you want to modify the size of the window for the curves 
 * plotted in real-time with SDL.
 * Beware: the layout was optimized for a width of 660 and a height of 520.
 *         Consequently, the display might not be optimal if you modify these values, 
 *         especially if you decrease them (increasing them should not be a problem).
 */
#define SCREEN_WIDTH  660 // screen width  : number of pixels [-]
#define SCREEN_HEIGHT 520 // screen height : number of pixels [-]


// -- Real-time -- //

// put 0 to directly start the simulation, 1 if you want to start with a pause 
#define INIT_BREAK 0

// put 0 to directly stop the simulation at the end, 1 if you want to get a last pause
#define FINAL_BREAK 0


// -- JNI visualization info -- //

// number of viewpoints defined in the dataR/*.mbs file
#define NB_VIEWPOINTS 3     

// initial viewpoint when the simulation starts                             
#define START_VIEWPOINT 0

#endif
