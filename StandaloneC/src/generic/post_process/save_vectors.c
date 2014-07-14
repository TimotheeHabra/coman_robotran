/*
 * Write files functions
 *
 * authors: Nicolas Van der Noot and Allan Barrea
 */

#include "main_simulation.h"
#include "useful_functions.h"

/*
 * Initialize a 'Save_vectors' structure
 */
Save_vectors* init_save_vectors(int nstep, int njoint)
{
    Save_vectors* save_vectors;

    save_vectors = (Save_vectors*) malloc(sizeof(Save_vectors));

    save_vectors->t = get_double_vec(nstep);

    save_vectors->qq = get_double_tab(njoint, nstep);

    save_vectors->out_vec = get_double_tab(NB_OUTPUT_VEC, nstep);

    save_vectors->kount = -1;

    return save_vectors;
} 

/*
 * Free a 'Save_vectors' structure
 */
void free_save_vectors(Save_vectors *save_vectors, int njoint)
{
	#ifndef WIN32 // strange bug with Windows -> to investigate
		free_double_vec(save_vectors->t);
		free_double_tab(save_vectors->qq, njoint);
		free_double_tab(save_vectors->out_vec, NB_OUTPUT_VEC);
	#endif

    free(save_vectors);
}

#ifdef WRITE_FILES

/*
 * Writes the .anim file.
 * Returns 0 if no problem and 1 if an error occured.
 */
int write_anim_file(Save_vectors* save_vectors, int njoint, const char *fileout)
{
    int kount;
    double *t;
    double**qq;
    
	int i = 0;
	int j = 0;
	FILE* fid = NULL; // internal filename

    t     = save_vectors->t;
    qq    = save_vectors->qq;
    kount = save_vectors->kount;

    // Opening file
    fid = fopen(fileout, "w"); // external filename
    if(fid == NULL)
    {
        printf("error: cannot open file '%s'\n", fileout);
        return 1;
    }

    // Dumping values
    for (j=0; j<=kount; j++)
    {
        fprintf(fid, "%12.8f ", t[j]);
        
        for (i=0; i<njoint; i++)
        {
            fprintf(fid, "%12.8f ", qq[i][j]);
        }
        
        fprintf(fid, "\n");        
    }

    // Closing file
    fclose(fid);

    return 0;
}

int write_out_files(Save_vectors* save_vectors, const char generic_fileout[PATH_MAX_LENGTH])
{
    int kount;
    double **out_vec;
    double *t;
    
    int i;
    int j;

    FILE* fid = NULL; // internal filename
    char cur_fileout[PATH_MAX_LENGTH];

    out_vec = save_vectors->out_vec;
    kount   = save_vectors->kount;
    t       = save_vectors->t;


    // -- Time vector -- //

    sprintf (cur_fileout, "%s_t.txt", generic_fileout);
    fid = fopen(cur_fileout, "w"); // external filename

    if(fid == NULL)
    {
        printf("error: cannot open file '%s'\n", cur_fileout);
        return 1;
    }

    // Dumping values
    for (j=0; j<=kount; j++)
    {
        fprintf(fid, "%12.8f\n", t[j]);     
    }

    // Closing file
    fclose(fid);

    // -- Output vectors -- //
    
    for (i=0; i<NB_OUTPUT_VEC; i++)
    {
        sprintf (cur_fileout, "%s_%d.txt", generic_fileout, i+1);
        fid = fopen(cur_fileout, "w"); // external filename

        if(fid == NULL)
        {
            printf("error: cannot open file '%s'\n", cur_fileout);
            return 1;
        }

        // Dumping values
        for (j=0; j<=kount; j++)
        {
            fprintf(fid, "%12.8f\n", out_vec[i][j]);     
        }

        // Closing file
        fclose(fid);
    }

    return 0;
}

#endif


