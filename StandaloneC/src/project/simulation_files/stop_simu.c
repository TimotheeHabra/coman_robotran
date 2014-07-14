//---------------------------
// Nicolas Van der Noot & Allan Barrea
//
// Creation : 29/10/2013
// Last update : 08/07/2014
//
// Stop the simulation if needed
//
//---------------------------

#include "simu_def.h"

/*
 * Stop simulation if uvs->stop_simu == 1
 */
void stop_simu(MBSdataStruct *MBSdata)
{
	// --- Variables declaration --- //
    
    // user variables
    UserIOStruct *uvs;
    
    int i;
	int fall_detect;

    #ifdef PRINT_REPORT
	int exploded_joint;
    #endif
    
	double fall_measure;
    
    double midWaistHeight;
    double RFootsHeight;
    double LFootsHeight;
    double footsHeight;

	MBSsensorStruct S_MidWaist;
    MBSsensorStruct S_RFoots;
    MBSsensorStruct S_LFoots;
    
	// --- Variables initialization and memory allocation --- //
    
    uvs = MBSdata->user_IO;
    
    allocate_sensor(&S_MidWaist,COMAN_NB_JOINT_TOTAL);
    init_sensor(&S_MidWaist,COMAN_NB_JOINT_TOTAL);
        
    allocate_sensor(&S_RFoots,COMAN_NB_JOINT_TOTAL);
    init_sensor(&S_RFoots,COMAN_NB_JOINT_TOTAL);
        
    allocate_sensor(&S_LFoots,COMAN_NB_JOINT_TOTAL);
    init_sensor(&S_LFoots,COMAN_NB_JOINT_TOTAL);
    
	// --- Event detection fall --- //

    sensor(&S_MidWaist, MBSdata, S_MIDWAIST);
    sensor(&S_RFoots, MBSdata, S_RFOOTS);
    sensor(&S_LFoots, MBSdata, S_LFOOTS);
        
    midWaistHeight = S_MidWaist.P[3];
    RFootsHeight = S_RFoots.P[3];
    LFootsHeight = S_LFoots.P[3];

    footsHeight = (RFootsHeight < LFootsHeight) ? RFootsHeight : LFootsHeight;
    
    uvs->waist_relative_ground = midWaistHeight - footsHeight;
    
    fall_measure = midWaistHeight - footsHeight - FALL_THRESHOLD;
    fall_detect = (fall_measure <= 0.0) ? 1 : 0;

	// --- Writing output --- //
    uvs->stop_simu = fall_detect;
	
	// --- Memory free --- //
    free_sensor(&S_MidWaist);
    free_sensor(&S_RFoots);
    free_sensor(&S_LFoots);
    
    // ---- Stopping simulation if ground forces too high ---- //
    
    if((fabs(uvs->GRF_r[3]) > GROUND_FORCES_THRESHOLD) ||
	   (fabs(uvs->GRF_l[3]) > GROUND_FORCES_THRESHOLD))
	{
		uvs->stop_simu = 1;
    }
	
	// --- Stopping simulation if joints are out of bounds --- //
    #ifdef PRINT_REPORT
    exploded_joint = 0;
    #endif
    
	for(i=COMAN_NB_JOINT_BASE+1; i<=COMAN_NB_JOINT_TOTAL; i++)
	{
		if((MBSdata->q[i] < uvs->joint_limits_min[i]) ||
		   (MBSdata->q[i] > uvs->joint_limits_max[i]))
		{
			uvs->stop_simu = 1;
            #ifdef PRINT_REPORT
            exploded_joint = i;
            #endif
			break;
		}
	}

	#ifdef PRINT_REPORT
	// Print report
	if(exploded_joint)
	{
		printf("Error: joint %d is out of bounds [%.3f;%.3f]; joint position = %.3f degrees (t = %.3f).\n", 
            exploded_joint, uvs->joint_limits_min[exploded_joint]*180.0/PI, uvs->joint_limits_max[exploded_joint]*180.0/PI, MBSdata->q[exploded_joint]*180.0/PI, MBSdata->tsim);
	}
	#endif
}
