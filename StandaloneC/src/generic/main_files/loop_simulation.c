/*
 * Main loop of the simulation
 *
 * author: Nicolas Van der Noot
 */

#include "main_simulation.h"
#include "integrator.h"
#include "save_vectors.h"

#ifdef ADAPTIVE_TIME_STEP
#include "dopri5.h"
#endif


/*
 * Top level main loop
 */
OutputSimu* loop_simulation(Loop_arguments *loop_arguments)
{
    // -- Variables decalration -- //

    OutputSimu *outputSimu;

	#ifdef ADAPTIVE_TIME_STEP
	double atoler, rtoler;
	#else
    int i;
    #endif

   
    // -- Main loop -- //

    #ifdef ADAPTIVE_TIME_STEP

	rtoler = R_TOLER;
	atoler = A_TOLER;

	dopri5 (loop_arguments->nvar, f_prim_dopri5, TSIM_INIT, loop_arguments->v, TSIM_END , &rtoler, &atoler, 0, NULL, 0,
		stdout, 0.0, 0.0, 0.0, 0.0, 0.0, MAX_STEP_SIZE, INIT_STEP_SIZE, NB_SIMU_STEPS, 1, 0, 0, NULL, 0, loop_arguments);

	loop_arguments->final_nb_steps = get_nstep_dopri5 ();

    #else

    // Simulation over nstep (= NB_SIMU_STEPS) steps: loop
    for (i = 1; (i <= NB_SIMU_STEPS) && loop_arguments->simu_go; i++)
    {
    	loop_iteration(loop_arguments);
    }

    loop_arguments->final_nb_steps = i-1;

    #endif


    // -- Simulation end -- //

    // last break at the end of the simulation (if asked)
    #if defined (SDL) & defined (REAL_TIME)
    last_break_gestion(loop_arguments);
    #endif
    

    // -- Output simulation -- //

    outputSimu = create_OutputSimu(loop_arguments->MBSdata);

    return outputSimu;
}


/*
 * This function corresponds to one call (one iteration) of the mail loop
 */
void loop_iteration(Loop_arguments *loop_arguments)
{
	// -- Variables declaration -- //

	double x, h;
    double *v,*vout,*dv;

    LocalDataStruct *lds;
    MBSdataStruct   *MBSdata;
	
	#ifdef REAL_TIME
	int    speed_new_t_usec;
    double speed_new_tsim;
    double tsim;
    #endif

    #if defined(SDL) & defined(REAL_TIME)
    double y_vec[NB_CURVES_MAX];
    #endif

    #if defined(WRITE_FILES) || defined(REAL_TIME)
    Save_vectors *save_vectors;
    #endif

    #ifdef REAL_TIME
    int cur_t_usec; 
    int init_t_sec, init_t_usec;

    int    speed_last_t_usec;
    double speed_last_tsim;

    Real_time_constraint **constraints;
    Simu_real_time        *real_time;
    #endif

    #if defined(SDL) & defined(REAL_TIME)
    Screen_sdl *screen_sdl;
    #endif

    #if defined(JNI) & defined (REAL_TIME)
    JNI_struct* jni_struct;
    #endif

    #if !defined(ADAPTIVE_TIME_STEP) || defined(REAL_TIME)
	int i;
	#endif


    // -- Variables initialization -- //
    
    x = loop_arguments->x;
    h = loop_arguments->h;

    v    = loop_arguments->v;
    vout = loop_arguments->vout;
    dv   = loop_arguments->dv;

    lds     = loop_arguments->lds;
    MBSdata = loop_arguments->MBSdata;

    #if defined(WRITE_FILES) || defined(REAL_TIME)
    save_vectors = loop_arguments->save_vectors;
    #endif

    #ifdef REAL_TIME
    init_t_sec  = loop_arguments->init_t_sec;
    init_t_usec = loop_arguments->init_t_usec;

    speed_last_t_usec = loop_arguments->speed_last_t_usec;
    speed_last_tsim   = loop_arguments->speed_last_tsim;

    constraints = loop_arguments->real_time->constraints;
    real_time   = loop_arguments->real_time;
    #endif

    #if defined(SDL) & defined(REAL_TIME)
    screen_sdl = loop_arguments->screen_sdl;
    #endif

    #if defined(JNI) & defined (REAL_TIME)
    jni_struct = loop_arguments->jni_struct;
    #endif


    // -- Additional simulation computations -- //

    // user own functions (controller_files, simulation_files and user files)
    user_compute_output(MBSdata, lds);
    

    /*
     * Main routine of the integrator.
     *
     * Starting from initial values ystart[1..nvar] known at x1 (= TSIM_INIT) use fourth-order Runge-Kutta
     * to advance nstep (=NB_SIMU_STEPS) equal increments h to x2 (= TSIM_END). The user-supplied routine derivs(x,v,dvdx)
     * evaluates derivatives. Results are stored in the global variables y[1..nvar][1..nstep+1]
     * and xx[1..nstep+1].
     */
    #ifndef ADAPTIVE_TIME_STEP
    // derivatives of the system to integrate
    (*derivs)(x,v,dv,lds,MBSdata);

    // Runge-Kutta 4
    rk4(v,dv,loop_arguments->nvar,x,h,vout,derivs,lds,MBSdata);

    // x (simulation time) update
    if ((double)(x+h) == x)
    {
        nrerror("Step size too small in routine odeint");
    } 

    loop_arguments->x += h;

    // state update
    for (i=1;i<=loop_arguments->nvar;i++)
    {
        v[i] = vout[i];
    }
    #endif

    // only updates real-time and storage features after a fixed simulation time
    #ifdef ADAPTIVE_TIME_STEP
    if (MBSdata->tsim < loop_arguments->next_update_adaptive)
    {
    	return;
    }
    else
    {
    	loop_arguments->next_update_adaptive = MBSdata->tsim + TSIM_UPDATE_STORAGE_REAL_TIME;
    }
    #endif

    // -- Real time (time values update) -- //

    #ifdef REAL_TIME
    // simulation time
    tsim = MBSdata->tsim;

    // real time
    cur_t_usec = t_usec(init_t_sec, init_t_usec);
    #endif


    // -- Store vectors for post-processing and past visualization -- //

    // save vectors
    #if defined(WRITE_FILES) || defined(REAL_TIME)
    update_save_vectors(save_vectors, MBSdata);
    #endif

    #ifdef REAL_TIME
    real_time->t_visu_past = MBSdata->tsim;
    #endif


    // -- Update SDL vectors to plot -- //

    #if defined(SDL) & defined(REAL_TIME)
    // assign values for the plot
    get_screen_sdl_functions(y_vec, MBSdata);

    // update simulation vectors for the plot
    update_full_vectors(screen_sdl, tsim, y_vec);
    #endif


    // -- Real time gestion -- //
    
    // check actions related to the Real-time constraints
    #ifdef REAL_TIME

    // plot vectors and update visualization
    for (i=0; i<NB_REAL_TIME_CONSTRAINTS; i++)
    {
    	
        if (tsim >= constraints[i]->next_tsim)
        {
            // plot screen sdl
            #if defined(SDL) & defined (REAL_TIME)                
            if (!i)
            {
                update_plot_vectors(screen_sdl, real_time, tsim, y_vec);
                plot_screen_sdl(screen_sdl, real_time, tsim, 0);
            }
            #endif

            // Java visualization
            #if defined(JNI) & defined (REAL_TIME)
            if (i == 1)
            {
                update_jni(jni_struct, MBSdata, real_time, MBSdata->q+1);
            }                
            #endif
            
            // new real-time constraints
            update_real_time_constraint(constraints[i], real_time->simu_speed_flag);
        }
    }

    // handle events (coming from the keyboard...)
    #if defined (SDL) & defined(JNI) & defined (REAL_TIME)
    events_simu(screen_sdl, real_time, MBSdata, save_vectors, &(loop_arguments->simu_go), &speed_last_t_usec, init_t_sec, init_t_usec, tsim, jni_struct);
    #elif defined (SDL) & defined (REAL_TIME)
    events_simu(screen_sdl, real_time, MBSdata, save_vectors, &(loop_arguments->simu_go), &speed_last_t_usec, init_t_sec, init_t_usec, tsim);
    #endif

    // gate locked: waiting time
    if (tsim >= real_time->next_tsim_gate)
    {
        // loop in order to wait to respect the real-time constraints
        while (real_time->next_t_usec_gate > cur_t_usec)
        {
        	// update real time
    		cur_t_usec = t_usec(init_t_sec, init_t_usec);

        	// handle events (coming from the keyboard...)
            #if defined (SDL) & defined(JNI) & defined (REAL_TIME)
            events_simu(screen_sdl, real_time, MBSdata, save_vectors, &(loop_arguments->simu_go), &speed_last_t_usec, init_t_sec, init_t_usec, tsim, jni_struct);
            #elif defined (SDL) & defined (REAL_TIME)
            events_simu(screen_sdl, real_time, MBSdata, save_vectors, &(loop_arguments->simu_go), &speed_last_t_usec, init_t_sec, init_t_usec, tsim);
            #endif
        }

        // in case there is no additional constraint
        if (real_time->no_additional_constraint)
        {
            update_real_time_constraint(real_time->constraints[0], real_time->simu_speed_flag);
        }

        // update real-time strcuture and related variables
        update_simu_real_time(real_time);
    }

    // computes the real time speed factor of the simulation (every REAL_TIME_SPEED_PERIOD_USEC s)
    if (cur_t_usec - speed_last_t_usec > REAL_TIME_SPEED_PERIOD_USEC)
    {
        speed_new_t_usec = cur_t_usec;
        speed_new_tsim   = tsim;

        real_time->real_simu_speed_factor = (speed_new_tsim - speed_last_tsim) / (1.0e-6 * (speed_new_t_usec - speed_last_t_usec));

        // change the plot
        #if defined(SDL) & defined (REAL_TIME)
        screen_sdl->bottom_flag = 1;
        #endif
        
        loop_arguments->speed_last_t_usec = speed_new_t_usec;
        loop_arguments->speed_last_tsim   = speed_new_tsim;
    }
    #endif

    
    // -- Simulation stop -- //

    // stop the simulation if 'stop_simu == 1'
    #ifdef STOP_SIMU
    if (MBSdata->user_IO->stop_simu)
    {
        loop_arguments->simu_go = 0;
    }
    #endif
}

/* 
 * Handle a last break at the end of the simulation if asked
 */
#if defined (SDL) & defined (REAL_TIME)
void last_break_gestion(Loop_arguments *loop_arguments)
{
	if (loop_arguments->real_time->last_break)
    {
        loop_arguments->real_time->simu_break = 1;

        #if defined (SDL) & defined(JNI) & defined (REAL_TIME)
        events_simu(loop_arguments->screen_sdl, loop_arguments->real_time, loop_arguments->MBSdata, loop_arguments->save_vectors, &(loop_arguments->simu_go), &(loop_arguments->speed_last_t_usec), loop_arguments->init_t_sec, loop_arguments->init_t_usec, loop_arguments->MBSdata->tsim, loop_arguments->jni_struct);
        #elif defined (SDL) & defined (REAL_TIME)
        events_simu(loop_arguments->screen_sdl, loop_arguments->real_time, loop_arguments->MBSdata, loop_arguments->save_vectors, &(loop_arguments->simu_go), &(loop_arguments->speed_last_t_usec), loop_arguments->init_t_sec, loop_arguments->init_t_usec, loop_arguments->MBSdata->tsim);
        #endif
    }  
}
#endif

/*
 * Function definig the differential equation for dopri5
 */
#ifdef ADAPTIVE_TIME_STEP
void f_prim_dopri5 (unsigned n, double x, double *y, double *f, Loop_arguments *loop_arguments) 
{
	loop_iteration(loop_arguments);

	derivs(x,y,f, loop_arguments->lds, loop_arguments->MBSdata); 
}
#endif
