//--------------------------- 
// UCL-CEREM-MBS 
// 
// @version MBsysLab_s 1.7.a 
// 
// Creation : 2006 
// Last update : 01/10/2008 
//--------------------------- 
 
#include "simu_def.h"

void user_Derivative(MBSdataStruct *MBSdata)
{
	int i;
	int i_real;
    
    // user variables
    UserIOStruct *uvs;
    
    uvs = MBSdata->user_IO;

	for (i=1; i<=COMAN_NB_JOINT_ACTUATED; i++)
	{ 
		// NOTE: ux[1 -> COMAN_NB_JOINT_ACTUATED] = joints positions; ux[COMAN_NB_JOINT_ACTUATED+1 -> 2*COMAN_NB_JOINT_ACTUATED] = joints velocities
		// returning motor velocity to the uxd 
        MBSdata->uxd[i] = MBSdata->ux[COMAN_NB_JOINT_ACTUATED+i];
	}

	// Computing accelerations of motors (i: motor reference)
	for (i=1; i<=COMAN_NB_JOINT_ACTUATED; i++)
	{
		i_real = uvs->actuated2real[i]; // real joint number

		// uvs->Control[i] contains the voltage applied to the motors converted in torque form.
		MBSdata->uxd[COMAN_NB_JOINT_ACTUATED+i] = (uvs->Control[i]-(uvs->Actuator_Ddrives)*(MBSdata->ux[COMAN_NB_JOINT_ACTUATED+i])) / (uvs->Actuator_Jdrives); 
		MBSdata->uxd[COMAN_NB_JOINT_ACTUATED+i]+= (-uvs->Actuator_KKs[i_real]*(MBSdata->ux[i]-MBSdata->q[i_real])) / (uvs->Actuator_Jdrives);     
		MBSdata->uxd[COMAN_NB_JOINT_ACTUATED+i]+= (-uvs->Actuator_DDs[i_real]*(MBSdata->ux[COMAN_NB_JOINT_ACTUATED+i]-MBSdata->qd[i_real])) / (uvs->Actuator_Jdrives);
	}
}

