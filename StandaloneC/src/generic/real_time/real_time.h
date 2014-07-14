/*
 * Real-time constraints main header
 *
 * author: Nicolas Van der Noot
 */
#ifndef __REAL_TIME_H_INCLUDED__  // guard against multiple/recursive includes
#define __REAL_TIME_H_INCLUDED__

// -- Includes -- //

#ifdef UNIX
#include <sys/time.h>
#endif

#ifdef WIN32
#include <time.h>
#include <sys/timeb.h>
#endif

// -- Macros -- //

 // real-time
#define DEFAULT_DELAT_T_USEC 1e5
#define REAL_TIME_SPEED_PERIOD_USEC 25e4
#define TIME_NO_INTERACTION_BREAK 5e6
#define TIME_SDL_DELAY 25
#define TIME_PRESS_KEY_DELAY 1e5
#define TIME_MOUSE_DELAY 5e4

// -- Structures -- //

#ifdef WIN32
typedef struct timeval {
     long tv_sec;
     long tv_usec;
} timeval;
#endif

#ifdef REAL_TIME
// One constraint strcuture
typedef struct Real_time_constraint
{
    int delta_t_usec;
    int next_t_usec;
    double next_tsim;
    
} Real_time_constraint;

// Real-time structure
typedef struct Simu_real_time
{
    // flags
    int simu_quit;
    int simu_break; 
    int last_break;
    int simu_speed_flag;
    int no_additional_constraint;

    #if defined (SDL) & defined (REAL_TIME)

    int mouse_init_x;
    int mouse_init_y;
    int mouse_delta_x;
    int mouse_delta_y;
    int mouse_cur_x;
    int mouse_cur_y;
    int mouse_wheel_flag;
    int start_mouse_usec;
    int last_action_break_usec;
    int next_user_keyboard_event_usec;
    int next_generic_keyboard_event_usec;
    int last_mouse_event_usec;
    int mouse_left_pressed;
    int mouse_right_pressed;

    #endif

    // speed factors
    double real_simu_speed_factor;
    double last_real_simu_speed_factor;

    // time
    double tsim_end;
    double last_tsim;

    // constraints
    int nb_constraints;

    // gates
    int next_t_usec_gate;  // real time [us]
    double next_tsim_gate; // simulation time [s] 

    int change_viewpoint;  // flag 
    int viewpoint_nb;

    int visu_past_flag;
    double t_visu_past;

    // structures of multiple real-time constraints
    Real_time_constraint **constraints;

} Simu_real_time;

// -- Functions prototypes -- //

// real-time features
Simu_real_time* init_real_time(int init_t_sec, int init_t_usec);
Simu_real_time* init_simu_real_time(int nb_constraints, double *fqc_tab, int cur_t_usec, double tsim, double tsim_end);
Real_time_constraint* init_real_time_constraint(int delta_t_usec, int cur_t_usec, double tsim);
void free_real_time_constraint(Real_time_constraint* constraint);
void free_simu_real_time(Simu_real_time *real_time);
void update_simu_real_time(Simu_real_time *real_time);
void update_real_time_constraint(Real_time_constraint *constraint, int simu_speed_flag);
void update_real_time_constraints_break(Simu_real_time *real_time, int delta_break_u_sec);
double get_simu_speed_factor(int simu_speed_flag);
#endif

// time functions
void time_get(int *t_sec, int *t_usec);
int t_usec(int init_t_sec, int init_t_usec);

#ifdef WIN32
int gettimeofday (struct timeval *tp, void *tz);
#endif

#endif
