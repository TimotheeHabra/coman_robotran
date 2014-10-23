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

/*
 * Print values of MBSdata (used for debug)
 */
void printMbsData(MBSdataStruct *MBSdata)
{
    int i = 0;

    // dpt[]
    printf("dpt = [\n");
    for(i = 1;  i <= MBSdata->npt; i++){
        printf("%f \t", MBSdata->dpt[1][i]);
        printf("%f \t", MBSdata->dpt[2][i]);
        printf("%f \n", MBSdata->dpt[3][i]);
    }
    printf("] \n\n");

    //npt
    printf("npt = %d \n\n",MBSdata->npt);

    //l
    printf("l = [\n");
    for(i = 1;  i <= MBSdata->nbody; i++){
        printf("%f \t", MBSdata->l[1][i]);
        printf("%f \t", MBSdata->l[2][i]);
        printf("%f \n", MBSdata->l[3][i]);
    }
    printf("] \n\n");

    //m
    printf("m = [\n");
    for(i = 1;  i <= MBSdata->nbody; i++){
        printf("%f \t", MBSdata->m[i]);
        if((i%5)==0) printf("\n");
    }
    printf("] \n\n");

    //In
    printf("In = [\n");
    for(i = 1;  i <= MBSdata->nbody; i++){
        printf("%.10f \t", MBSdata->In[1][i]);
        printf("%.10f \t", MBSdata->In[2][i]);
        printf("%.10f \t", MBSdata->In[3][i]);
        printf("%.10f \t", MBSdata->In[4][i]);
        printf("%.10f \t", MBSdata->In[5][i]);
        printf("%.10f \t", MBSdata->In[6][i]);
        printf("%.10f \t", MBSdata->In[7][i]);
        printf("%.10f \t", MBSdata->In[8][i]);
        printf("%.10f \n", MBSdata->In[9][i]);
    }
    printf("] \n\n");

    //g
    printf("g = [%f %f %f] \n\n",MBSdata->g[1],MBSdata->g[2],MBSdata->g[3]);

    //number of ...
    printf("nbody = %d\n",MBSdata->nbody);
    printf("njoint = %d\n",MBSdata->njoint);
    printf("nqu = %d\n",MBSdata->nqu);
    printf("nqc = %d\n",MBSdata->nqc);
    printf("nqlocked = %d\n",MBSdata->nqlocked);
    printf("ndriven = %d\n",MBSdata->nqdriven);
    printf("nqa = %d\n",MBSdata->nqa);
    printf("nqv = %d\n",MBSdata->nqv);
    printf("nhu = %d\n",MBSdata->nhu);
    printf("nxfrc = %d\n",MBSdata->Nxfrc);
    printf("nloopc = %d\n",MBSdata->Nloopc);
    printf("ncons = %d\n",MBSdata->Ncons);
    printf("nuserc = %d\n",MBSdata->Nuserc);
    printf("Nlink = %d\n",MBSdata->Nlink);
    printf("Nlink3D = %d\n",MBSdata->Nlink3D);
    printf("Nsensor = %d\n",MBSdata->Nsensor);

    //qu
    printf("qu = [\n");
    for(i = 1;  i <= MBSdata->nqu; i++){
        printf("%d \t", MBSdata->qu[i]);
        if((i%5)==0) printf("\n");
    }
    printf("] \n\n");

    //qc
    printf("qc = [\n");
    for(i = 1;  i <= MBSdata->nqc; i++){
        printf("%d \t", MBSdata->qc[i]);
        if((i%5)==0) printf("\n");
    }
    printf("] \n\n");

    //qlocked
    printf("qlocked = [\n");
    for(i = 1;  i <= MBSdata->nqlocked; i++){
        printf("%d \t", MBSdata->qlocked[i]);
        if((i%5)==0) printf("\n");
    }
    printf("] \n\n");

    //qdriven
    printf("qdriven = [\n");
    for(i = 1;  i <= MBSdata->nqdriven; i++){
        printf("%d \t", MBSdata->qdriven[i]);
        if((i%5)==0) printf("\n");
    }
    printf("] \n\n");

    //qa
    printf("qa = [\n");
    for(i = 1;  i <= MBSdata->nqa; i++){
        printf("%d \t", MBSdata->qa[i]);
        if((i%5)==0) printf("\n");
    }
    printf("] \n\n");

    //qv
    printf("qv = [\n");
    for(i = 1;  i <= MBSdata->nqv; i++){
        printf("%d \t", MBSdata->qv[i]);
        if((i%5)==0) printf("\n");
    }
    printf("] \n\n");

    //hu
    printf("hu = [\n");
    for(i = 1;  i <= MBSdata->nhu; i++){
        printf("%d \t", MBSdata->hu[i]);
        if((i%5)==0) printf("\n");
    }
    printf("] \n\n");

    // q[]
    printf("q = [\n");
    for(i = 1;  i <= MBSdata->njoint; i++){
        printf("%f \t", MBSdata->q[i]);
        if((i%5)==0) printf("\n");
    }
    printf("] \n\n");

    // qd[]
    printf("qd = [\n\n");
    for(i = 1;  i <= MBSdata->njoint; i++){
        printf("%f \t", MBSdata->qd[i]);
        if((i%5)==0) printf("\n");
    }
    printf("] \n\n");

    // qdd[]
    printf("qdd = [\n");
    for(i = 1;  i <= MBSdata->njoint; i++){
        printf("%f \t", MBSdata->qdd[i]);
        if((i%5)==0) printf("\n");
    }
    printf("] \n\n");

    // qmin[] and qmax not yet implemented in xml reading

    // frc
    printf("frc = [\n");
    for(i = 1;  i <= MBSdata->nbody; i++){
        printf("%f \t", MBSdata->frc[1][i]);
        printf("%f \t", MBSdata->frc[2][i]);
        printf("%f \n", MBSdata->frc[3][i]);
    }
    printf("] \n\n");

    // trq
    printf("trq = [\n");
    for(i = 1;  i <= MBSdata->nbody; i++){
        printf("%f \t", MBSdata->trq[1][i]);
        printf("%f \t", MBSdata->trq[2][i]);
        printf("%f \n", MBSdata->trq[3][i]);
    }
    printf("] \n\n");

    // Qq[]
    printf("Qq = [\n");
    for(i = 1;  i <= MBSdata->njoint; i++){
        printf("%f \t", MBSdata->Qq[i]);
        if((i%5)==0) printf("\n");
    }
    printf("] \n\n");

    // tsim
    printf("tsim = %f\n\n",MBSdata->tsim);

    //lrod not yet implemented in xml reading

    //lambda not yet implemented in xml reading

    //user_constraints not yet implemented in xml reading

    // Z, Zd, Fl not implemented yet

    //SWr
    printf("SWr = [\n");
    for(i = 1;  i <= MBSdata->Nxfrc; i++){
        printf("%f \t", MBSdata->SWr[i][1]);
        printf("%f \t", MBSdata->SWr[i][2]);
        printf("%f \t", MBSdata->SWr[i][3]);
        printf("%f \t", MBSdata->SWr[i][4]);
        printf("%f \t", MBSdata->SWr[i][5]);
        printf("%f \t", MBSdata->SWr[i][6]);
        printf("%f \t", MBSdata->SWr[i][7]);
        printf("%f \t", MBSdata->SWr[i][8]);
        printf("%f \n", MBSdata->SWr[i][9]);
    }
    printf("] \n\n");

    //xfidpt
    printf("xfidpt = [\n");
    for(i = 1;  i <= MBSdata->Nxfrc; i++){
        printf("%d \t", MBSdata->xfidpt[i]);
        if((i%5)==0) printf("\n");
    }
    printf("] \n\n");

}


