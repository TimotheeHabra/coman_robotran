//--------------------------- 
// UCL-CEREM-MBS 
// 
// @version MBsysLab_s 1.7.a 
// 
// Creation : 2006 
// Last update : 01/10/2008 
//--------------------------- 
 
#include "simu_def.h"
 
double* user_JointForces(MBSdataStruct *MBSdata, double tsim)
{ 
	int i;
	int i_real;
    
    // user variables
    UserIOStruct *uvs;
    
    uvs = MBSdata->user_IO;
 
    // adding the coupling torques between the motors and the joints (i: motor reference)
    for (i=1; i<=COMAN_NB_JOINT_ACTUATED; i++) 
    {
		i_real = uvs->actuated2real[i]; // real joint number (Robotran numbering)
        MBSdata->Qq[i_real]  = (uvs->Actuator_KKs[i_real]*(MBSdata->ux[i]-MBSdata->q[i_real]));
        MBSdata->Qq[i_real] += (uvs->Actuator_DDs[i_real]*(MBSdata->ux[COMAN_NB_JOINT_ACTUATED+i]-MBSdata->qd[i_real]));
    }

    // --- Torque produced by the springs in the compliant foot --- //

    #ifdef COMP_FEET

    // Right foot
    MBSdata->Qq[R_TOE]  = uvs->Actuator_KKs[R_TOE]*(-MBSdata->q[R_TOE]);  // stiffness
    MBSdata->Qq[R_TOE] += uvs->Actuator_DDs[R_TOE]*(-MBSdata->qd[R_TOE]); // damping
    
    // Left foot
    MBSdata->Qq[L_TOE]  = uvs->Actuator_KKs[L_TOE]*(-MBSdata->q[L_TOE]);  // stiffness
    MBSdata->Qq[L_TOE] += uvs->Actuator_DDs[L_TOE]*(-MBSdata->qd[L_TOE]); // damping

    #endif
 
    return MBSdata->Qq;
} 
