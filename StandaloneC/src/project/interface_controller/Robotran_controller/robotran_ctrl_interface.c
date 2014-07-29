//---------------------------
//
// Nicolas Van der Noot
//
// Creation : 28/07/2014
// Last update : 28/07/2014
//
// Controller interface
//
//---------------------------

#include "robotran_ctrl_interface.h"

/*
 * Initialization function
 */
void robotran_ctrl_init_interface(MBSdataStruct *MBSdata, Ctrl_Inputs  *ivs)
{
	ControllerStruct *cvs;

	cvs = MBSdata->user_IO->cvs;

	// interface: inputs (generic)
	controller_inputs(MBSdata, ivs);

	// interface: inputs (robotran_ctrl)
	robotran_ctrl_inputs(ivs, cvs);
	
	// controller initialization
	controller_init(cvs);
}
	
/*
 * loop function
 */
void robotran_ctrl_loop_interface(MBSdataStruct *MBSdata, Ctrl_Inputs  *ivs, Ctrl_Outputs *ovs)
{
	ControllerStruct *cvs;

	cvs = MBSdata->user_IO->cvs;

	// interface: inputs (generic)
	controller_inputs(MBSdata, ivs);   

	// interface: inputs (robotran_ctrl)
	robotran_ctrl_inputs(ivs, cvs);
	
	// controller loop
    controller_loop(cvs);  

    // interface: outputs (robotran_ctrl)
    robotran_ctrl_outputs(ovs, cvs);    

    // interface: outputs (generic)
    controller_outputs(MBSdata, ovs);  
}
