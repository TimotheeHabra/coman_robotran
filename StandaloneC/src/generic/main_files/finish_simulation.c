/*
 * End of the simulation (print reports, free memory...)
 * 
 * author: Nicolas Van der Noot
 */

#include "main_simulation.h"
#include "loadMBSheader_xml.h"
#include "real_time.h"
#include "cmake_config.h"
#include "simu_def.h"

/*
 * End of the simulation
 */
void finish_simulation(Loop_arguments *loop_arguments)
{
    // -- Variables declaration -- //

    #ifdef PRINT_REPORT
	double total_t_sec;
    #endif

    MBSdataStruct *MBSdata;

    // -- Simulation end -- //

    // for the anim file
    #ifdef WRITE_FILES
	#ifdef REAL_TIME
	const char *fileout_anim = PROJECT_ABS_PATH"/../animationR/Model_real_time.anim";
	#else
	const char *fileout_anim = PROJECT_ABS_PATH"/../animationR/Model_standalone.anim";
	#endif
    const char generic_fileout[PATH_MAX_LENGTH] = PROJECT_ABS_PATH"/src/other/save_vector/vectors/vec";
	#endif

	// MBSdata
	MBSdata = loop_arguments->MBSdata;

    
    #ifdef PRINT_REPORT
    // total real-time needed for the simulation
    total_t_sec = t_usec(loop_arguments->init_t_sec, loop_arguments->init_t_usec) / 1.0e6;

    // Printing simulation report
    printf("... Done\n\n");
    printf("-------------------------------\n");
    printf("nb steps:\t%d\n", loop_arguments->final_nb_steps);
    #ifndef ADAPTIVE_TIME_STEP
    printf("time step:\t%.3f\t[ms]\n", DELTA_TSIM * 1000);
    #endif
    printf("-------------------------------\n");
    printf("t_start:\t%.3f\t[s]\n", TSIM_INIT);
    printf("t_end:\t\t%.3f\t[s]\n", TSIM_END);
    printf("-------------------------------\n");
    printf("Final time:\t%.3f\t[s]\n", MBSdata->tsim);
	printf("Executed in:\t%.3f\t[s]\n", total_t_sec);
    printf("-------------------------------\n\n");
	#endif

    // writing the .anim file
    #ifdef WRITE_FILES

    if(write_anim_file(loop_arguments->save_vectors, MBSdata->njoint, fileout_anim))
    {
        printf("error: cannot write the anim file\n");
    }
    #ifdef PRINT_REPORT
    else
    {
        printf("File '%s' successfully written\n", fileout_anim);
    }
    #endif

    // writing the output vectors
    if (write_out_files(loop_arguments->save_vectors, generic_fileout))
    {
        printf("error: cannot write the output vectors files\n");
    }
    #ifdef PRINT_REPORT
    else
    {
        printf("Output Files '%s' successfully written\n", generic_fileout);
    }
    #endif

    #endif

    // User function called at the end of the simulation
    #ifdef STANDALONE
    user_finalization(MBSdata);
    #endif


	// -- Closing operations -- //

    // .anim files
    #if defined(WRITE_FILES) || defined(REAL_TIME)
	free_save_vectors(loop_arguments->save_vectors, MBSdata->njoint);
    #endif

    // JNI visualization
    #if defined(JNI) & defined (REAL_TIME)
	free_jni(loop_arguments->jni_struct);
    #endif

    // Real-time constraints
    #ifdef REAL_TIME
	free_simu_real_time(loop_arguments->real_time);
    #endif

    // SDL window
    #if defined(SDL) & defined(REAL_TIME)
	free_screen_sdl(loop_arguments->screen_sdl);
    #endif

    // LocalDataStruct
    #if !defined(ACCELRED) && !defined(ODN)
    freeLocalDataStruct(loop_arguments->lds,MBSdata);
    #endif

    // MBSdata_xml
    freeMBSdata_xml(MBSdata);

    // integrator vectors
    free_dvector(loop_arguments->v   , 1);
    free_dvector(loop_arguments->vout, 1);
    free_dvector(loop_arguments->dv  , 1);

    // simulation loop arguments
	free(loop_arguments);
}
