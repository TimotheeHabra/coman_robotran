//---------------------------
//
// Nicolas Van der Noot
//
// Creation : 27/07/2014
// Last update : 27/07/2014
//
// Inputs and outputs gestion
//
//---------------------------

#include "controller_interface.h"
#include "controller_io.h"

/*
 * Controller inputs
 */
void robotran_ctrl_inputs(Ctrl_Inputs *ivs, ControllerStruct *cvs)
{
	int i;

	ControllerInputs  *ctrl_ivs;

	ctrl_ivs = cvs->Inputs;

	// time
	ctrl_ivs->t = ivs->t;

	// positions - velocities - torques
	for (i=0; i<COMAN_NB_JOINT_ACTUATED; i++)
	{
		ctrl_ivs->q[i]      = ivs->q[i];
		ctrl_ivs->qd[i]     = ivs->qd[i];
		ctrl_ivs->Qq[i]     = ivs->Qq[i];
		ctrl_ivs->q_mot[i]  = ivs->q_mot[i];
		ctrl_ivs->qd_mot[i] = ivs->qd_mot[i];
	}

	// feet (forces - torques) and IMU
	for (i=0; i<3; i++)
	{
		ctrl_ivs->F_Rfoot[i]          = ivs->F_Rfoot[i];
		ctrl_ivs->F_Lfoot[i]          = ivs->F_Lfoot[i];
		ctrl_ivs->T_Rfoot[i]          = ivs->T_Rfoot[i];
		ctrl_ivs->T_Lfoot[i]          = ivs->T_Lfoot[i];
		ctrl_ivs->IMU_Angular_Rate[i] = ivs->IMU_Angular_Rate[i];
		ctrl_ivs->IMU_Acceleration[i] = ivs->IMU_Acceleration[i];
	}

	// IMU orientation
	for(i=0; i<9; i++)
    {
        ctrl_ivs->IMU_Orientation[i] = ivs->IMU_Orientation[i];
    }
}

/*
 * Controller outputs
 */
void robotran_ctrl_outputs(Ctrl_Outputs *ovs, ControllerStruct *cvs)
{
	int i;

	ControllerOutputs *ctrl_ovs;

	ctrl_ovs = cvs->Outputs;

	// tracking (position - velocity - torque)
	for (i=0; i<COMAN_NB_JOINT_ACTUATED; i++)
	{
		ovs->q_ref[i]          = ctrl_ovs->q_ref[i];
		ovs->qd_ref[i]         = ctrl_ovs->qd_ref[i];
		ovs->Qq_ref[i]         = ctrl_ovs->Qq_ref[i];
		ovs->imp_ctrl_index[i] = ctrl_ovs->imp_ctrl_index[i];
	}   
}
