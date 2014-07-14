/*
 * Functions used to go backward or forward in the simulation time (only for visualization)
 *
 * author: Nicolas Van der Noot
 */

#if defined(JNI) & defined (REAL_TIME)

#include "visu_past.h"
#include "info_project.h"
#include "math.h"

/*
 * Returns a vector with the q position at time 't'
 */ 
double* get_q_past_vector(MBSdataStruct *MBSdata, Save_vectors *save_vectors, double t)
{
	// variables declaration
	int i;
	int nb_joints;
	int kount, index_request;

	double *vec_q_past;
	double **qq;

	// variables initialization
	kount = save_vectors->kount;

	nb_joints = MBSdata->njoint;

	vec_q_past = (double*) calloc(nb_joints, sizeof(double));

	qq = save_vectors->qq;

	// index for this request
	index_request = (int) floor( ((t - TSIM_INIT) / (MBSdata->tsim - TSIM_INIT)) * kount);
	
	// bounds
	if (index_request < 0)
	{
		index_request = 0;
	}
	else if (index_request > kount)
	{
		index_request = kount;
	}

	// get the requested values
	for(i=0; i<nb_joints; i++)
	{
		vec_q_past[i] = qq[i][index_request];
	}

	// returns the vector
	return vec_q_past;
}


/*
 * Updates 't_visu_part' according to the simulation speed factor
 */
void update_t_visu_past(MBSdataStruct *MBSdata, Simu_real_time *real_time, int flag_forward)
{
	double simu_speed_factor;

	// simulation speed
	simu_speed_factor = get_simu_speed_factor(real_time->simu_speed_flag);

	// new 't_visu_part'
	if (flag_forward)
	{
		real_time->t_visu_past += simu_speed_factor * PERIOD_BACK_IN_VISU;
	}	
	else
	{
		real_time->t_visu_past -= simu_speed_factor * PERIOD_BACK_IN_VISU;
	}

	// bounds
	if (real_time->t_visu_past < TSIM_INIT)
	{
		real_time->t_visu_past = TSIM_INIT;
	}

	if (real_time->t_visu_past > MBSdata->tsim)
	{
		real_time->t_visu_past = MBSdata->tsim;
	}
}

#endif
