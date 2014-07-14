//---------------------------
//
// Nicolas Van der Noot
//
// Creation : 10/07/2014
// Last update : 10/07/2014
//
// Get the position references for the arms
//
//---------------------------

#ifndef STANDALONE

#include "controller_def.h"

// shoulder sagittal (pitch)
#define MIN_SH_SAG  ( -60.0 * DEG_TO_RAD )
#define MAX_SH_SAG  (  10.0 * DEG_TO_RAD )

// shoulder lateral (roll)
#define MIN_SH_LAT  ( -100.0 * DEG_TO_RAD )
#define MAX_SH_LAT  (  20.0  * DEG_TO_RAD )

// shoulder yaw
#define MIN_SH_YAW  ( -10.0 * DEG_TO_RAD )
#define MAX_SH_YAW  (  40.0 * DEG_TO_RAD )

// elbow
#define MIN_ELB  ( -60.0 * DEG_TO_RAD )
#define MAX_ELB  (   0.0 * DEG_TO_RAD )

// time
#define TIME_1 2.0
#define TIME_2 10.0
#define TIME_3 20.0
#define TIME_4 30.0
#define TIME_5 40.0
#define TIME_6 50.0

/*
 * Arms position references for the Simulink version
 */
void arm_pos_ref(ControllerStruct *cvs)
{
	double tsim;

	tsim = cvs->Inputs->t;

	if (tsim < TIME_1)
	{
		// left arm position reference [rad]
		cvs->q_ref_r_sh_sag = 0.0;
		cvs->q_ref_r_sh_lat = 0.0;
		cvs->q_ref_r_sh_yaw = 0.0;
		cvs->q_ref_r_elb    = 0.0; 

		// right arm position reference [rad]
		cvs->q_ref_l_sh_sag = 0.0;
		cvs->q_ref_l_sh_lat = 0.0;
		cvs->q_ref_l_sh_yaw = 0.0;
		cvs->q_ref_l_elb    = 0.0;   
	}
	else if (tsim < TIME_2)
	{
		// left arm position reference [rad]
		cvs->q_ref_r_sh_sag = linear_interpolation(tsim, TIME_1, TIME_2, 0.0, MIN_SH_SAG);
		cvs->q_ref_r_sh_lat = linear_interpolation(tsim, TIME_1, TIME_2, 0.0, MAX_SH_LAT);
		cvs->q_ref_r_sh_yaw = linear_interpolation(tsim, TIME_1, TIME_2, 0.0, MAX_SH_YAW);
		cvs->q_ref_r_elb    = linear_interpolation(tsim, TIME_1, TIME_2, 0.0, MIN_ELB); 

		// right arm position reference [rad]
		cvs->q_ref_l_sh_sag = 0.0;
		cvs->q_ref_l_sh_lat = 0.0;
		cvs->q_ref_l_sh_yaw = 0.0;
		cvs->q_ref_l_elb    = 0.0;   
	}
	else if (tsim < TIME_3)
	{
		// left arm position reference [rad]
		cvs->q_ref_r_sh_sag = MIN_SH_SAG;
		cvs->q_ref_r_sh_lat = MAX_SH_LAT;
		cvs->q_ref_r_sh_yaw = MAX_SH_YAW;
		cvs->q_ref_r_elb    = MIN_ELB; 

		// right arm position reference [rad]
		cvs->q_ref_l_sh_sag = linear_interpolation(tsim, TIME_2, TIME_3, 0.0,  MAX_SH_SAG);
		cvs->q_ref_l_sh_lat = linear_interpolation(tsim, TIME_2, TIME_3, 0.0, -MIN_SH_LAT);
		cvs->q_ref_l_sh_yaw = linear_interpolation(tsim, TIME_2, TIME_3, 0.0, -MIN_SH_YAW);
		cvs->q_ref_l_elb    = linear_interpolation(tsim, TIME_2, TIME_3, 0.0,  MAX_ELB);
	}
	else if (tsim < TIME_4)
	{
		// left arm position reference [rad]
		cvs->q_ref_r_sh_sag = linear_interpolation(tsim, TIME_3, TIME_4, MIN_SH_SAG, MAX_SH_SAG);
		cvs->q_ref_r_sh_lat = linear_interpolation(tsim, TIME_3, TIME_4, MAX_SH_LAT, MIN_SH_LAT);
		cvs->q_ref_r_sh_yaw = linear_interpolation(tsim, TIME_3, TIME_4, MAX_SH_YAW, MIN_SH_YAW);
		cvs->q_ref_r_elb    = linear_interpolation(tsim, TIME_3, TIME_4, MIN_ELB,    MAX_ELB);

		// right arm position reference [rad]
		cvs->q_ref_l_sh_sag = linear_interpolation(tsim, TIME_3, TIME_4,  MAX_SH_SAG,  MIN_SH_SAG);
		cvs->q_ref_l_sh_lat = linear_interpolation(tsim, TIME_3, TIME_4, -MIN_SH_LAT, -MAX_SH_LAT);
		cvs->q_ref_l_sh_yaw = linear_interpolation(tsim, TIME_3, TIME_4, -MIN_SH_YAW, -MAX_SH_YAW);
		cvs->q_ref_l_elb    = linear_interpolation(tsim, TIME_3, TIME_4,  MAX_ELB,     MIN_ELB);  
	}
	else if (tsim < TIME_5)
	{
		// left arm position reference [rad]
		cvs->q_ref_r_sh_sag = MAX_SH_SAG;
		cvs->q_ref_r_sh_lat = MIN_SH_LAT;
		cvs->q_ref_r_sh_yaw = MIN_SH_YAW;
		cvs->q_ref_r_elb    = MAX_ELB; 

		// right arm position reference [rad]
		cvs->q_ref_l_sh_sag =  MIN_SH_SAG;
		cvs->q_ref_l_sh_lat = -MAX_SH_LAT;
		cvs->q_ref_l_sh_yaw = -MAX_SH_YAW;
		cvs->q_ref_l_elb    =  MIN_ELB;   
	}
	else if (tsim < TIME_6)
	{
		// left arm position reference [rad]
		cvs->q_ref_r_sh_sag = linear_interpolation(tsim, TIME_5, TIME_6,  MAX_SH_SAG,  0.0);
		cvs->q_ref_r_sh_lat = linear_interpolation(tsim, TIME_5, TIME_6,  MIN_SH_LAT,  0.0);
		cvs->q_ref_r_sh_yaw = linear_interpolation(tsim, TIME_5, TIME_6,  MIN_SH_YAW,  0.0);
		cvs->q_ref_r_elb    = linear_interpolation(tsim, TIME_5, TIME_6,  MAX_ELB,  0.0); 

		// right arm position reference [rad]
		cvs->q_ref_l_sh_sag =  MIN_SH_SAG;
		cvs->q_ref_l_sh_lat = -MAX_SH_LAT;
		cvs->q_ref_l_sh_yaw = -MAX_SH_YAW;
		cvs->q_ref_l_elb    =  MIN_ELB;   
	}
	else
	{
		// left arm position reference [rad]
		cvs->q_ref_r_sh_sag = 0.0;
		cvs->q_ref_r_sh_lat = 0.0;
		cvs->q_ref_r_sh_yaw = 0.0;
		cvs->q_ref_r_elb    = 0.0; 

		// right arm position reference [rad]
		cvs->q_ref_l_sh_sag = 0.0;
		cvs->q_ref_l_sh_lat = 0.0;
		cvs->q_ref_l_sh_yaw = 0.0;
		cvs->q_ref_l_elb    = 0.0;   
	}   

	// correction for the long arms case
	#ifdef LONG_ARMS

	// frame change
	cvs->q_ref_l_sh_lat -= PI_2;
	cvs->q_ref_r_sh_lat += PI_2;

	// adapt to arms morphology
	cvs->q_ref_l_sh_yaw = 0.0;
	cvs->q_ref_l_elb    = 0.0;
	cvs->q_ref_r_sh_yaw = 0.0;
	cvs->q_ref_r_elb    = 0.0;
	#endif
}

#endif
