//---------------------------
// Nicolas Van der Noot
//
// Creation : 29/10/2013
// Last update : 08/07/2014
//
// Outputs of the controller
//
//---------------------------

#include "simu_def.h"

/*
 * Interface of the controller with the robot for the outputs
 */ 
void controller_outputs(MBSdataStruct *MBSdata)
{
    int i;

    UserIOStruct *uvs;

    uvs = MBSdata->user_IO;

    // impedance controller (not the real one implemented on the CoMan)
    impedance_controller(MBSdata);

    // convert voltage in torque form 
    //      uvs->Control[i] is the torque form (in [Nm])
    //      uvs->Voltage[i] is the voltage (in [V])
    // i is the index in the motor indexes list
    for(i=1; i<=COMAN_NB_JOINT_ACTUATED; i++)
    {
        uvs->Control[i] = uvs->Voltage[i]*(uvs->Actuator_VTgain);
    }
}
