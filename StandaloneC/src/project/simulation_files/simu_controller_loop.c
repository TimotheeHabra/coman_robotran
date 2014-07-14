//---------------------------
// Nicolas Van der Noot
//
// Creation : 29/10/2013
// Last update : 08/07/2014
//
// Main loop of the controller and user simulation files
//
//---------------------------

#include "simu_def.h"

void simu_controller_loop(MBSdataStruct *MBSdata)
{
    // variables declaration
    double tsim;

    UserIOStruct     *uvs;
    ControllerStruct *cvs;
    
    // variables initialization
    tsim = MBSdata->tsim;

    uvs = MBSdata->user_IO;
	cvs = uvs->cvs;

    
    // controller called every milli-second (PERIOD_CTRL)
    if (tsim >= uvs->last_t_ctrl + PERIOD_CTRL - TIME_EPSILON)
    {
        uvs->last_t_ctrl = tsim;
        
        controller_inputs(MBSdata);   // interface: inputs
        controller_loop(cvs);         // controller
        controller_outputs(MBSdata);  // interface: outputs
    }

    // simulation outputs for simulink (useless for the Standalone version)
    simulink_outputs(MBSdata);

	// stopping the simulation if needed
	stop_simu(MBSdata);
}
