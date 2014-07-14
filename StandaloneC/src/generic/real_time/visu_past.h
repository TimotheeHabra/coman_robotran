
/*
 * Header for the fucntions used to go backward or forward in the simulation time (only for visualization)
 *
 * author: Nicolas Van der Noot
 */
#if defined(JNI) & defined (REAL_TIME)

#ifndef _VISU_PAST_H_
#define _VISU_PAST_H_

#include "save_vectors.h"
#include "real_time.h"

#define PERIOD_T_VISU_PAST 0.05

double* get_q_past_vector(MBSdataStruct *MBSdata, Save_vectors *save_vectors, double t);
void update_t_visu_past(MBSdataStruct *MBSdata, Simu_real_time *real_time, int flag_forward);

#endif

#endif
