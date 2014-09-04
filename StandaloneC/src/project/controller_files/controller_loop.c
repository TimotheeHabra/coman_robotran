//---------------------------
//
// Nicolas Van der Noot
//
// Creation : 01/11/2013
// Last update : 08/07/2014
//
// Main loop of the controller file
//
//---------------------------

#include "controller_def.h"

/*
 * Get references to sent to the impedance controller
 */
void controller_loop(ControllerStruct *cvs)
{
	// hardcoded arm references only for the Simulink version 
	// (real-time interactions for the Standalone version)
	#if !defined(STANDALONE) || !defined(SDL)
	arm_pos_ref(cvs);
	#endif

    // get position, velocity and torque references
    get_ref(cvs);
}
