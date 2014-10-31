/*
 * Initialization of the simulation
 *
 * authors: Nicolas Van der Noot and Allan Barrea
 */

#include "main_simulation.h"
#include "nrutil.h"
#include "loadMBSheader_xml.h"
#include "real_time.h"
#include "cmake_config.h"
#include "integrator.h"

#ifdef UNIX
#include <time.h>
#include <sys/time.h>
#endif

#ifdef WIN32
#include <time.h>
#include <sys/timeb.h>
#endif

#define MBS_FILE PROJECT_ABS_PATH"/../dataR/"PROJECT_NAME_MBS

/*
 * Simulation initialization
 */
Loop_arguments* init_simulation(void)
{
	// -- Variables declaration -- //
	
	int i;
	int nvar;

    double *ystart;

	MBSdataStruct   *MBSdata        = NULL;
	LocalDataStruct *lds            = NULL;
	Loop_arguments  *loop_arguments = NULL;

	#if defined(JNI) & defined (REAL_TIME)
	JNI_struct* jni_struct;
	#endif

	#if defined(WRITE_FILES) || defined(REAL_TIME)
	Save_vectors *save_vectors;
	#endif

	struct timeval seed_time;


	// -- Seed for random -- //

	gettimeofday(&seed_time, NULL);
	srand(seed_time.tv_usec * seed_time.tv_sec);
    

    // -- Variables initialization -- //

	// MBSDataStruct initialization
	MBSdata = loadMBSdata_xml(MBS_FILE);

    printMbsData(MBSdata);

	if(MBSdata == NULL)
	{
        printf("error while loading MBSdata \n");
	}
	#ifdef PRINT_REPORT
	else
	{
        printf("MBSdata successfully loaded \n");
	}
	#endif

    // LocalDataStruct initialization

	#if !defined(ACCELRED) && !defined(ODN)
	lds = initLocalDataStruct(MBSdata); 
	#else
	lds = NULL;
	#endif

	if(lds == NULL)
	{
	    printf("error while initializing LocalDataStruct\n");
	}

	// Model initialization
	if(user_initialization(MBSdata, lds))
	{
	    printf("error in user_initialization\n");
	}
	
	// Integrator parameters initialization
	nvar = 2*MBSdata->nqu + MBSdata->Nux;

	ystart = dvector(1,nvar);

	#if defined(WRITE_FILES) || defined(REAL_TIME)
	save_vectors = init_save_vectors(NB_SIMU_STEPS, MBSdata->njoint);
	#endif

    // Simulation state initialization
	for(i=1; i<=MBSdata->nqu; i++)
	{
		ystart[i]              = MBSdata->q[MBSdata->qu[i]];
		ystart[i+MBSdata->nqu] = MBSdata->qd[MBSdata->qu[i]];
	}

	for(i=1; i<=MBSdata->Nux; i++)
	{
	    ystart[i+2*MBSdata->nqu] = MBSdata->ux[i];
	}
	
	#ifdef PRINT_REPORT
	printf("Model successfully initialized\n\n");
	#endif

	// JNI visualization
	#if defined(JNI) & defined (REAL_TIME)
	jni_struct = init_jni(MBSdata);
	#endif

    // initialize the inputs of the simulation loop
    loop_arguments = (Loop_arguments*) malloc(sizeof(Loop_arguments));

    // absolute initial time of the simulation
    time_get(&(loop_arguments->init_t_sec), &(loop_arguments->init_t_usec));

    loop_arguments->nvar    = nvar;
    loop_arguments->lds     = lds;
    loop_arguments->MBSdata = MBSdata;

    #if defined(WRITE_FILES) || defined(REAL_TIME)
    loop_arguments->save_vectors = save_vectors;
    #endif

    // integrator vectors
    loop_arguments->v    = dvector(1,nvar-INDEX_0_FLAG_INTEG);
    loop_arguments->vout = dvector(1,nvar);
    loop_arguments->dv   = dvector(1,nvar);

    // Load starting values
    for (i=1; i <= nvar; i++)
    {
        loop_arguments->v[i-INDEX_0_FLAG_INTEG] = ystart[i];
    }

    free_dvector(ystart, 1);

    // integrator time values
    loop_arguments->x = TSIM_INIT;
    loop_arguments->h = (TSIM_END - TSIM_INIT) / NB_SIMU_STEPS; // [s]

    // Real-time constraiints
    #ifdef REAL_TIME
    loop_arguments->real_time = init_real_time(loop_arguments->init_t_sec, loop_arguments->init_t_usec);
    #endif

    // not update real-time and storage at each time step
    #ifdef ADAPTIVE_TIME_STEP
    loop_arguments->next_update_adaptive = TSIM_INIT;
    #endif

    // SDL window
    #if defined(SDL) & defined (REAL_TIME)	
	loop_arguments->screen_sdl = configure_screen_sdl(loop_arguments->init_t_sec, loop_arguments->init_t_usec);
	#endif

	// Jave visualization
	#if defined(JNI) & defined (REAL_TIME)
    loop_arguments->jni_struct = jni_struct;
    update_jni(loop_arguments->jni_struct, MBSdata, loop_arguments->real_time, MBSdata->q+1);
    #endif

    // real-time variables
    #ifdef REAL_TIME
    loop_arguments->speed_last_t_usec = t_usec(loop_arguments->init_t_sec, loop_arguments->init_t_usec);
    loop_arguments->speed_last_tsim   = TSIM_INIT;
    #endif

    #if defined(SDL) & defined(REAL_TIME)
    // first plot
    plot_screen_sdl(loop_arguments->screen_sdl, loop_arguments->real_time, TSIM_INIT, 2);
    #endif

    // flag : 0 if the simulation must be stopped (1 otherwise) -> starts the simulation with 1
    loop_arguments->simu_go = 1;

    // Running model integration
    #ifdef PRINT_REPORT
    printf("\nRunning integration of the model...\n\n");
    #endif

    return loop_arguments;
} 
