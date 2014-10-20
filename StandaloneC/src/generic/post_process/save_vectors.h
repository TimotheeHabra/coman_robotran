/*
 * Write the output vectors and tabs (.anim,...)
 *
 * author: Nicolas Van der Noot and Allan Barrea
 */ 
#ifndef __WRITE_FILES_H_INCLUDED__  // guard against multiple/recursive includes
#define __WRITE_FILES_H_INCLUDED__

#include "MBSdataStruct.h"
#include "info_project.h"

// -- Structures -- //

// .anim structure
typedef struct Save_vectors
{
    double *t;
    double **qq;
    int kount;
    double **out_vec;

} Save_vectors;

// -- Functions prototypes -- //

Save_vectors* init_save_vectors(int nstep, int njoint);
void free_save_vectors(Save_vectors *save_vectors, int njoint);

// configure
void update_save_vectors(Save_vectors *save_vectors, MBSdataStruct *MBSdata);

#ifdef WRITE_FILES

int write_anim_file(Save_vectors* save_vectors, int njoint, const char *fileout);
int write_out_files(Save_vectors* save_vectors, const char generic_fileout[50]);

#endif

void printMbsData(MBSdataStruct *MBSdata);

#endif
