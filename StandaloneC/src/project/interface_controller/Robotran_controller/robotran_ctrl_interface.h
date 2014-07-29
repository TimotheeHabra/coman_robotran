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

#ifndef _ROBOTRAN_CTRL_INTERFACE_H_
#define _ROBOTRAN_CTRL_INTERFACE_H_

#include "controller_interface.h"

// controller
#ifdef __cplusplus
extern "C" {
#endif
	void controller_init(ControllerStruct *cvs);
	void controller_loop(ControllerStruct *cvs);

	void robotran_ctrl_inputs(Ctrl_Inputs *ivs, ControllerStruct *cvs);
	void robotran_ctrl_outputs(Ctrl_Outputs *ovs, ControllerStruct *cvs);
#ifdef __cplusplus
}
#endif

#endif
