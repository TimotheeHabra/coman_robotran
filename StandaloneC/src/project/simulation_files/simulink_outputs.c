//---------------------------
// Nicolas Van der Noot
//
// Creation : 29/10/2013
// Last update : 08/07/2014
//
// simulation outputs for graphs and debug via simulink interface (useless for the Standalone version)
//
//---------------------------

#include "simu_def.h"

/*
 * Defines outputs for the Simulink environment.
 * These outputs are defined in the 'simu_vars_out' section of the 'simu_variables.m' file.
 */
void simulink_outputs(MBSdataStruct *MBSdata)
{
    // variables declaration
    UserIOStruct      *uvs;
    ControllerStruct  *cvs;
    ControllerOutputs *ovs;

	int i;

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

    // variables initialization
    uvs = MBSdata->user_IO;
    cvs = uvs->cvs;
    ovs = cvs->Outputs;

	// time [s]
	uvs->tsim_out = MBSdata->tsim;

	// torques [Nm]
	for(i = 0; i < COMAN_NB_JOINT_ACTUATED ; i++)
	{
		robotran_id = robotran_id_table[i];

		uvs->Qq_out[i+1] = MBSdata->Qq[robotran_id];
        uvs->q_ref[i+1]  = ovs->q_ref[i];
        uvs->qd_ref[i+1] = ovs->qd_ref[i];
        uvs->Qq_ref[i+1] = ovs->Qq_ref[i];
        uvs->imp_ctrl_index[i+1] = ovs->imp_ctrl_index[i];
	}
	
    
    /*
     * The user can define its own outputs via this field
     */

    // right arm: reference and actual position for the elbow [rad]
    uvs->out[1] = cvs->q_ref_r_elb;
    uvs->out[2] = MBSdata->q[R_ELB];

    // left arm: reference and actual position for the elbow [rad]
    uvs->out[3] = cvs->q_ref_l_elb;
    uvs->out[4] = MBSdata->q[L_ELB];
}
