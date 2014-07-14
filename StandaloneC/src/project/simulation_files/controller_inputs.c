//---------------------------
//
// Nicolas Van der Noot
//
// Creation : 29/10/2013
// Last update : 08/07/2014
//---------------------------

#include "simu_def.h"

/*
 * Assigns the inputs of the controller
 */
void controller_inputs(MBSdataStruct *MBSdata)
{
    UserIOStruct     *uvs;
    ControllerStruct *cvs;
    ControllerInputs *ivs;

	MBSsensorStruct S_MidWaist;

	int i;

	double R_11, R_21;
    
    int robotran_id;
    int robotran_id_table[] = {
        WAIST_YAW, //  1. TORSO_YAW
        WAIST_SAG, //  2. TORSO_PITCH
        WAIST_LAT, //  3. TORSO_ROLL
        
        R_HIP_SAG, //  4. RIGHT_HIP_PITCH
        L_HIP_SAG, //  5. LEFT_HIP_PITCH
        
        R_HIP_LAT, //  6. RIGHT_HIP_ROLL
        R_HIP_YAW, //  7. RIGHT_HIP_YAW
        R_KNEE_SAG, //  8. RIGHT_KNEE_PITCH
        R_ANK_SAG, //  9. RIGHT_FOOT_PITCH
        R_ANK_LAT, // 10. RIGHT_FOOT_ROLL
        
        L_HIP_LAT, // 11. LEFT_HIP_ROLL
        L_HIP_YAW, // 12. LEFT_HIP_YAW
        L_KNEE_SAG, // 13. LEFT_KNEE_PITCH
        L_ANK_SAG, // 14. LEFT_FOOT_PITCH
        L_ANK_LAT, // 15. LEFT_FOOT_ROLL
        
        R_SH_SAG, // 16. RIGHT_SHOULDER_PITCH
        R_SH_LAT, // 17. RIGHT_SHOULDER_ROLL
        R_SH_YAW, // 18. RIGHT_SHOULDER_YAW
        R_ELB, // 19. RIGHT_ELBOW_PITCH
        
        L_SH_SAG, // 20. LEFT_SHOULDER_PITCH
        L_SH_LAT, // 21. LEFT_SHOULDER_ROLL
        L_SH_YAW, // 22. LEFT_SHOULDER_YAW
        L_ELB, // 23. LEFT_ELBOW_PITCH

        #ifdef LONG_ARMS
        R_FORE_ARM_PLATE, // 24- RIGHT_FORE_ARM_PLATE
        R_WRJ1,           // 25. RIGHT_WRJ1
        R_WRJ2,           // 26. RIGHT_WRJ2

        L_FORE_ARM_PLATE, // 24- LEFT_FORE_ARM_PLATE
        L_WRJ1,           // 25. LEFT_WRJ1
        L_WRJ2,           // 26. LEFT_WRJ2
        #endif
    };


    uvs = MBSdata->user_IO;
    cvs = uvs->cvs;
    ivs = cvs->Inputs;
    

    // -- Time -- //
    
    ivs->t = MBSdata->tsim; // time [s]


    // ---- Forces under the feet ---- //
    
	// Vertical forces [N]
    for (i=0; i<3; i++)
    {
        ivs->F_Rfoot[i] = uvs->GRF_r[i+1];
        ivs->F_Lfoot[i] = uvs->GRF_l[i+1];
        
        ivs->T_Rfoot[i] = uvs->GRM_r[i+1];
        ivs->T_Lfoot[i] = uvs->GRM_l[i+1];

        #ifdef COMP_FEET
        ivs->F_Rfoot[i] += uvs->GRF_r_dist[i+1];
        ivs->F_Lfoot[i] += uvs->GRF_l_dist[i+1];
        
        ivs->T_Rfoot[i] += uvs->GRM_r_dist[i+1];
        ivs->T_Lfoot[i] += uvs->GRM_l_dist[i+1];
        #endif
    }
    
    // -- Joint positions, velocities and torques -- //
    
    for(i=0; i<COMAN_NB_JOINT_ACTUATED; i++)
	{
        robotran_id = robotran_id_table[i];
        
		ivs->q[i]  = MBSdata->q[robotran_id];   // position [rad]
		ivs->qd[i] = MBSdata->qd[robotran_id];  // velocity [rad/s]
		ivs->Qq[i] = MBSdata->Qq[robotran_id];  // torque   [Nm]
	}
    
    // -- IMU -- //
    
    // allocation
    allocate_sensor(&S_MidWaist,COMAN_NB_JOINT_TOTAL);
    init_sensor(&S_MidWaist,COMAN_NB_JOINT_TOTAL);
    sensor(&S_MidWaist, MBSdata, S_MIDWAIST); // IMU located in the MidWaist body
    
    ivs->IMU_Orientation[0] = S_MidWaist.R[1][1];
    ivs->IMU_Orientation[1] = S_MidWaist.R[1][2];
    ivs->IMU_Orientation[2] = S_MidWaist.R[1][3];
    ivs->IMU_Orientation[3] = S_MidWaist.R[2][1];
    ivs->IMU_Orientation[4] = S_MidWaist.R[2][2];
    ivs->IMU_Orientation[5] = S_MidWaist.R[2][3];
    ivs->IMU_Orientation[6] = S_MidWaist.R[3][1];
    ivs->IMU_Orientation[7] = S_MidWaist.R[3][2];
    ivs->IMU_Orientation[8] = S_MidWaist.R[3][3];
    
    ivs->IMU_Angular_Rate[0] = S_MidWaist.OM[1];
    ivs->IMU_Angular_Rate[1] = S_MidWaist.OM[2];
    ivs->IMU_Angular_Rate[2] = S_MidWaist.OM[3];
    
    free_sensor(&S_MidWaist);
    
    
	// -- IMU yaw info: not available on the real CoMan -- //
    
    // rotation matrix
    R_11 = ivs->IMU_Orientation[0];
    R_21 = ivs->IMU_Orientation[3];
    
    // absolute orientation [rad]
    uvs->real_theta_3_waist = atan2(-R_21,R_11);
    
    // derivative of absolute orientation [rad/s]
    uvs->real_omega_3_waist = ivs->IMU_Angular_Rate[2];	
}
