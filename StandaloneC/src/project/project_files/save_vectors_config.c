/*
 * Header of the output vetors configuration
 *
 * author: Nicolas Van der Noot
 */



#include "save_vectors.h"
#include "main_simulation.h"
#include "useful_functions.h"

#include "controller_def.h"

/*
 * Update the 'Save_vectors' structure'
 */
void update_save_vectors(Save_vectors *save_vectors, MBSdataStruct *MBSdata)
{
    int i;
    int kount;
    double *t;
    double **qq;

    #ifdef WRITE_FILES
    double **out_vec;
    #endif

    //UserIOStruct *uvs;
    //ControllerStruct *cvs;

    t     = save_vectors->t;
    qq    = save_vectors->qq;
    kount = save_vectors->kount;

    #ifdef WRITE_FILES
    out_vec = save_vectors->out_vec;
    #endif

    //uvs = MBSdata->user_IO;
    //cvs = uvs->cvs;

    kount++;

    t[kount] = MBSdata->tsim;

    for(i=0; i<MBSdata->njoint; i++)
    {
        qq[i][kount] = MBSdata->q[i+1];
    }


    #ifdef WRITE_FILES

    // -- TO MODIFY -- //
	/*
	 * You can assign values for the debugging vectors located in StandaloneC/src/other/save_vector/vectors.
     * To do this, fill out_vec with the variable (or something else...) to save in a .txt files 
     * to be analyzed in post-process by this simulator, Matlab or any other program.
	 * 
	 * example:
	 *    out_vec[0][kount] = MBSdata->q[1];
	 *
	 * The fisrt index must be in the [0 ; 5] interval (0 corresponding to output_vec_1.txt)
	 * The second index is always 'kount'
	 */

    for(i=0; i<NB_OUTPUT_VEC; i++)
    {
        out_vec[i][kount] = 0.0;
    }

    #endif

    // --------------- //

    save_vectors->kount = kount;
}
