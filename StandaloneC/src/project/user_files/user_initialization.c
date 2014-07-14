//--------------------------- 
// UCL-CEREM-MBS 
// 
// @version MBsysLab_s 1.7.a 
// 
// Creation : 2008 
// Last update : 24/10/2008 
//--------------------------- 
 
#include "simu_def.h"

#define FORCE_VEC_LENGTH 50

// User parameters initialization
#ifndef STANDALONE
void user_initialization(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds)
#else
// Returns 0 if no problem
int user_initialization(MBSdataStruct *MBSdata, LocalDataStruct *lds)
#endif
{    
	ControllerStruct *cvs;

	cvs = MBSdata->user_IO->cvs;

	// init GCM (Ground Contact Model)
	init_GCM(MBSdata);

	// init actuator models
	init_actuator_model(MBSdata);

	// inputs of the controller
	controller_inputs(MBSdata);
	
	// controller initialization
    controller_init(cvs);

    #ifdef STANDALONE
    return 0;
    #endif
}
