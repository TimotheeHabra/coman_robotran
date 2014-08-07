//---------------------------
// Nicolas Van der Noot & Allan Barrea
//
// Creation : 03/03/2013
// Last update : 03/03/2013
//
// Initialize the actuators models
//
//---------------------------

#include "simu_def.h"

// Initializes actuators model (stiffness and damping coefficients)
void init_actuator_model(MBSdataStruct *MBSdata)
{

    int i;
    int i_real;

    // user variables
    UserIOStruct *uvs;

	// Control
	// (full of zeroes... skipped)

    uvs = MBSdata->user_IO;

	// --- KKs ---

	// Floating base
	uvs->Actuator_KKs[FJ_T1]  = 0.0;
	uvs->Actuator_KKs[FJ_T2]  = 0.0;
	uvs->Actuator_KKs[FJ_T3]  = 0.0;
	uvs->Actuator_KKs[FJ_R1]  = 0.0;
	uvs->Actuator_KKs[FJ_R2]  = 0.0;
	uvs->Actuator_KKs[FJ_R3]  = 0.0;

	// Right leg
	uvs->Actuator_KKs[R_HIP_SAG]   = 395.46;
	uvs->Actuator_KKs[R_HIP_LAT]   = 8400;
	uvs->Actuator_KKs[R_HIP_TRANS] = 8400;
	uvs->Actuator_KKs[R_KNEE_SAG]  = 395.46;
	uvs->Actuator_KKs[R_ANK_LAT]   = 8400;
	uvs->Actuator_KKs[R_ANK_SAG]   = 395.46;

	#ifdef COMP_FEET
	uvs->Actuator_KKs[R_TOE] = 5.0; // compliant foot
	#endif

	// Left leg
	uvs->Actuator_KKs[L_HIP_SAG]   = 395.46;
	uvs->Actuator_KKs[L_HIP_LAT]   = 8400;
	uvs->Actuator_KKs[L_HIP_TRANS] = 8400;
	uvs->Actuator_KKs[L_KNEE_SAG]  = 395.46;
	uvs->Actuator_KKs[L_ANK_LAT]   = 8400;
	uvs->Actuator_KKs[L_ANK_SAG]   = 395.46;

	#ifdef COMP_FEET
	uvs->Actuator_KKs[L_TOE] = 5.0; // compliant foot
	#endif

	// Torso
	uvs->Actuator_KKs[WAIST_LAT]   = 8400;
	uvs->Actuator_KKs[WAIST_SAG]   = 395.46;
	uvs->Actuator_KKs[WAIST_TRANS] = 395.46;

	// Right arm
	uvs->Actuator_KKs[R_SH_SAG]   = 120;
	uvs->Actuator_KKs[R_SH_LAT]   = 120;
	uvs->Actuator_KKs[R_SH_TRANS] = 8400;
	uvs->Actuator_KKs[R_ELB]      = 120;

	#ifdef LONG_ARMS
	uvs->Actuator_KKs[R_FORE_ARM_PLATE] = 120;
	uvs->Actuator_KKs[R_WRJ1] = 120;
	uvs->Actuator_KKs[R_WRJ2] = 120;
	#endif

	// Left arm
	uvs->Actuator_KKs[L_SH_SAG]   = 120;
	uvs->Actuator_KKs[L_SH_LAT]   = 120;
	uvs->Actuator_KKs[L_SH_TRANS] = 8400;
	uvs->Actuator_KKs[L_ELB]      = 120;

	#ifdef LONG_ARMS
	uvs->Actuator_KKs[L_FORE_ARM_PLATE] = 120;
	uvs->Actuator_KKs[L_WRJ1] = 120;
	uvs->Actuator_KKs[L_WRJ2] = 120;
	#endif

	// --- DDs ---

	// Floating base
	uvs->Actuator_DDs[FJ_T1] = 0.0;
	uvs->Actuator_DDs[FJ_T2] = 0.0;
	uvs->Actuator_DDs[FJ_T3] = 0.0;
	uvs->Actuator_DDs[FJ_R1] = 0.0;
	uvs->Actuator_DDs[FJ_R2] = 0.0;
	uvs->Actuator_DDs[FJ_R3] = 0.0;

	// Right leg
	uvs->Actuator_DDs[R_HIP_SAG]   = 0.198;
	uvs->Actuator_DDs[R_HIP_LAT]   = 4.2;
	uvs->Actuator_DDs[R_HIP_TRANS] = 4.2;
	uvs->Actuator_DDs[R_KNEE_SAG]  = 0.198;
	uvs->Actuator_DDs[R_ANK_LAT]   = 4.2;
	uvs->Actuator_DDs[R_ANK_SAG]   = 0.198;

	#ifdef COMP_FEET
	uvs->Actuator_DDs[R_TOE] = 0.1;
	#endif

	// Left leg
	uvs->Actuator_DDs[L_HIP_SAG]   = 0.198;
	uvs->Actuator_DDs[L_HIP_LAT]   = 4.2;
	uvs->Actuator_DDs[L_HIP_TRANS] = 4.2;
	uvs->Actuator_DDs[L_KNEE_SAG]  = 0.198;
	uvs->Actuator_DDs[L_ANK_LAT]   = 4.2;
	uvs->Actuator_DDs[L_ANK_SAG]   = 0.198;

	#ifdef COMP_FEET
	uvs->Actuator_DDs[L_TOE] = 0.1;
	#endif

	// Torso
	uvs->Actuator_DDs[WAIST_LAT]   = 4.2;
	uvs->Actuator_DDs[WAIST_SAG]   = 0.198;
	uvs->Actuator_DDs[WAIST_TRANS] = 0.198;

	// Right arm
	uvs->Actuator_DDs[R_SH_SAG]   = 0.198;
	uvs->Actuator_DDs[R_SH_LAT]   = 0.198;
	uvs->Actuator_DDs[R_SH_TRANS] = 4.2;
	uvs->Actuator_DDs[R_ELB]      = 0.198;

	#ifdef LONG_ARMS
	uvs->Actuator_DDs[R_FORE_ARM_PLATE] = 0.198;
	uvs->Actuator_DDs[R_WRJ1]           = 0.198;
	uvs->Actuator_DDs[R_WRJ2]           = 0.198;
	#endif

	// Left arm
	uvs->Actuator_DDs[L_SH_SAG]   = 0.198;
	uvs->Actuator_DDs[L_SH_LAT]   = 0.198;
	uvs->Actuator_DDs[L_SH_TRANS] = 4.2;
	uvs->Actuator_DDs[L_ELB]      = 0.198;

	#ifdef LONG_ARMS
	uvs->Actuator_DDs[L_FORE_ARM_PLATE] = 0.198;
	uvs->Actuator_DDs[L_WRJ1]           = 0.198;
	uvs->Actuator_DDs[L_WRJ2]           = 0.198;
	#endif

	// --- Jdrives ---
	uvs->Actuator_Jdrives = 0.1387;

	// --- Ddrives ---
	uvs->Actuator_Ddrives = 25.52;

	// --- VTgain ---
	uvs->Actuator_VTgain = 6.175;

	// --- Conversion between joint number actuated [1-23] and real [1-29] or [1-31]
	uvs->actuated2real[R_HIP_SAG_CTRL_MOT]   = R_HIP_SAG;
	uvs->actuated2real[R_HIP_LAT_CTRL_MOT]   = R_HIP_LAT;
	uvs->actuated2real[R_HIP_TRANS_CTRL_MOT] = R_HIP_TRANS;
	uvs->actuated2real[R_KNEE_SAG_CTRL_MOT]  = R_KNEE_SAG;
	uvs->actuated2real[R_ANK_LAT_CTRL_MOT]   = R_ANK_LAT;
	uvs->actuated2real[R_ANK_SAG_CTRL_MOT]   = R_ANK_SAG;
	uvs->actuated2real[L_HIP_SAG_CTRL_MOT]   = L_HIP_SAG;
	uvs->actuated2real[L_HIP_LAT_CTRL_MOT]   = L_HIP_LAT;
	uvs->actuated2real[L_HIP_TRANS_CTRL_MOT] = L_HIP_TRANS;
	uvs->actuated2real[L_KNEE_SAG_CTRL_MOT]  = L_KNEE_SAG;
	uvs->actuated2real[L_ANK_LAT_CTRL_MOT]   = L_ANK_LAT;
	uvs->actuated2real[L_ANK_SAG_CTRL_MOT]   = L_ANK_SAG;
	uvs->actuated2real[WAIST_LAT_CTRL_MOT]   = WAIST_LAT;
	uvs->actuated2real[WAIST_SAG_CTRL_MOT]   = WAIST_SAG;
	uvs->actuated2real[WAIST_TRANS_CTRL_MOT] = WAIST_TRANS;
	uvs->actuated2real[R_SH_SAG_CTRL_MOT]    = R_SH_SAG;
	uvs->actuated2real[R_SH_LAT_CTRL_MOT]    = R_SH_LAT;
	uvs->actuated2real[R_SH_TRANS_CTRL_MOT]  = R_SH_TRANS;
	uvs->actuated2real[R_ELB_CTRL_MOT]       = R_ELB;
	uvs->actuated2real[L_SH_SAG_CTRL_MOT]    = L_SH_SAG;
	uvs->actuated2real[L_SH_LAT_CTRL_MOT]    = L_SH_LAT;
	uvs->actuated2real[L_SH_TRANS_CTRL_MOT]  = L_SH_TRANS;
	uvs->actuated2real[L_ELB_CTRL_MOT]       = L_ELB;

	uvs->real2actuated[R_HIP_SAG]   = R_HIP_SAG_CTRL_MOT;
	uvs->real2actuated[R_HIP_LAT]   = R_HIP_LAT_CTRL_MOT;
	uvs->real2actuated[R_HIP_TRANS] = R_HIP_TRANS_CTRL_MOT;
	uvs->real2actuated[R_KNEE_SAG]  = R_KNEE_SAG_CTRL_MOT;
	uvs->real2actuated[R_ANK_LAT]   = R_ANK_LAT_CTRL_MOT;
	uvs->real2actuated[R_ANK_SAG]   = R_ANK_SAG_CTRL_MOT;
	uvs->real2actuated[L_HIP_SAG]   = L_HIP_SAG_CTRL_MOT;
	uvs->real2actuated[L_HIP_LAT]   = L_HIP_LAT_CTRL_MOT;
	uvs->real2actuated[L_HIP_TRANS] = L_HIP_TRANS_CTRL_MOT;
	uvs->real2actuated[L_KNEE_SAG]  = L_KNEE_SAG_CTRL_MOT;
	uvs->real2actuated[L_ANK_LAT]   = L_ANK_LAT_CTRL_MOT;
	uvs->real2actuated[L_ANK_SAG]   = L_ANK_SAG_CTRL_MOT;
	uvs->real2actuated[WAIST_LAT]   = WAIST_LAT_CTRL_MOT;
	uvs->real2actuated[WAIST_SAG]   = WAIST_SAG_CTRL_MOT;
	uvs->real2actuated[WAIST_TRANS] = WAIST_TRANS_CTRL_MOT;
	uvs->real2actuated[R_SH_SAG]    = R_SH_SAG_CTRL_MOT;
	uvs->real2actuated[R_SH_LAT]    = R_SH_LAT_CTRL_MOT;
	uvs->real2actuated[R_SH_TRANS]  = R_SH_TRANS_CTRL_MOT;
	uvs->real2actuated[R_ELB]       = R_ELB_CTRL_MOT;
	uvs->real2actuated[L_SH_SAG]    = L_SH_SAG_CTRL_MOT;
	uvs->real2actuated[L_SH_LAT]    = L_SH_LAT_CTRL_MOT;
	uvs->real2actuated[L_SH_TRANS]  = L_SH_TRANS_CTRL_MOT;
	uvs->real2actuated[L_ELB]       = L_ELB_CTRL_MOT;


	#ifdef LONG_ARMS
	uvs->actuated2real[R_FORE_ARM_PLATE_CTRL_MOT] = R_FORE_ARM_PLATE;
	uvs->actuated2real[R_WRJ1_CTRL_MOT]           = R_WRJ1;
	uvs->actuated2real[R_WRJ2_CTRL_MOT]           = R_WRJ2;
	uvs->actuated2real[L_FORE_ARM_PLATE_CTRL_MOT] = L_FORE_ARM_PLATE;
	uvs->actuated2real[L_WRJ1_CTRL_MOT]           = L_WRJ1;
	uvs->actuated2real[L_WRJ2_CTRL_MOT]           = L_WRJ2;

	uvs->real2actuated[R_FORE_ARM_PLATE] = R_FORE_ARM_PLATE_CTRL_MOT;
	uvs->real2actuated[R_WRJ1]           = R_WRJ1_CTRL_MOT;
	uvs->real2actuated[R_WRJ2]           = R_WRJ2_CTRL_MOT;
	uvs->real2actuated[L_FORE_ARM_PLATE] = L_FORE_ARM_PLATE_CTRL_MOT;
	uvs->real2actuated[L_WRJ1]           = L_WRJ1_CTRL_MOT;
	uvs->real2actuated[L_WRJ2]           = L_WRJ2_CTRL_MOT;
	#endif


    // --- Joint limits ---

	// Entered here in degrees
	// (source: coman_Hardware_slides.pdf)
	// for joint numbers, cf. Robotran model.
	// coman_Hardware_slides.pdf is not complete: it only gives information
	// about the legs and nothing about the upper body (including torso).
	// Common sense and observation on the simulations helps a lot to define the bounds.
	// Sometimes, I extend the bounds a little for simulation safety.

	// To understand coman_Hardware_slides.pdf:
	// flex/ext = flexion/extension = rotation about the Y axis (= sag = R2)
	// abd/add = abduction/adduction = rotation about the X axis (= lat = R1)
	// rotation = rotation about the Z axis (= yaw = R3)

	// Warning ! For the X axis, the joints limits are inverted between both legs.
	// --> take symmetric bounds to ease the thing.

	// ---- Min joint limits ---- //

	// Floating base
	uvs->joint_limits_min[FJ_T1]  = 0.0;
	uvs->joint_limits_min[FJ_T2]  = 0.0;
	uvs->joint_limits_min[FJ_T3]  = 0.0;
	uvs->joint_limits_min[FJ_R1]  = 0.0;
	uvs->joint_limits_min[FJ_R2]  = 0.0;
	uvs->joint_limits_min[FJ_R3]  = 0.0;

	// Right leg
	uvs->joint_limits_min[R_HIP_SAG]   = -120.0;
	uvs->joint_limits_min[R_HIP_LAT]   = -80.0;
	uvs->joint_limits_min[R_HIP_TRANS] = -50.0;
	uvs->joint_limits_min[R_KNEE_SAG]  = -15.0;
	uvs->joint_limits_min[R_ANK_LAT]   = -35.0;
	uvs->joint_limits_min[R_ANK_SAG]   = -90.0;
	#ifdef COMP_FEET
	uvs->joint_limits_min[R_TOE] = -90.0;
	#endif

	// Left leg
	uvs->joint_limits_min[L_HIP_SAG]   = -120.0;
	uvs->joint_limits_min[L_HIP_LAT]   = -35.0;
	uvs->joint_limits_min[L_HIP_TRANS] = -50.0;
	uvs->joint_limits_min[L_KNEE_SAG]  = -15.0;
	uvs->joint_limits_min[L_ANK_LAT]   = -35.0;
	uvs->joint_limits_min[L_ANK_SAG]   = -90.0;
	#ifdef COMP_FEET
	uvs->joint_limits_min[L_TOE] = -90.0;
	#endif

	// Torso
	uvs->joint_limits_min[WAIST_LAT]   = -70.0;
	uvs->joint_limits_min[WAIST_SAG]   = -60.0;
	uvs->joint_limits_min[WAIST_TRANS] = -90.0;

	// Right arm
	uvs->joint_limits_min[R_SH_SAG] = -90.0;

	#ifdef LONG_ARMS
	uvs->joint_limits_min[R_SH_LAT] = -30.0;
	#else
	uvs->joint_limits_min[R_SH_LAT] = -120.0;
	#endif

	uvs->joint_limits_min[R_SH_TRANS] = -50.0;
	uvs->joint_limits_min[R_ELB]      = -120.0;

	#ifdef LONG_ARMS
	uvs->joint_limits_min[R_FORE_ARM_PLATE] = -90.0;
	uvs->joint_limits_min[R_WRJ1]           = -90.0;
	uvs->joint_limits_min[R_WRJ2]           = -90.0;
	#endif

	// Left arm
	uvs->joint_limits_min[L_SH_SAG] = -90.0;

	#ifdef LONG_ARMS
	uvs->joint_limits_min[L_SH_LAT] = -125.0;
	#else
	uvs->joint_limits_min[L_SH_LAT] = -35.0;
	#endif

	uvs->joint_limits_min[L_SH_TRANS] = -50.0;
	uvs->joint_limits_min[L_ELB]      = -120.0;

	#ifdef LONG_ARMS
	uvs->joint_limits_min[L_FORE_ARM_PLATE] = -90.0;
	uvs->joint_limits_min[L_WRJ1]           = -90.0;
	uvs->joint_limits_min[L_WRJ2]           = -90.0;
	#endif


	// ---- Max joint limits ---- //

	// Floating base
	uvs->joint_limits_max[FJ_T1] = 0.0;
	uvs->joint_limits_max[FJ_T2] = 0.0;
	uvs->joint_limits_max[FJ_T3] = 0.0;
	uvs->joint_limits_max[FJ_R1] = 0.0;
	uvs->joint_limits_max[FJ_R2] = 0.0;
	uvs->joint_limits_max[FJ_R3] = 0.0;

	// Right leg
	uvs->joint_limits_max[R_HIP_SAG]   = 70.0;
	uvs->joint_limits_max[R_HIP_LAT]   = 35.0;
	uvs->joint_limits_max[R_HIP_TRANS] = 50.0;
	uvs->joint_limits_max[R_KNEE_SAG]  = 120.0;
	uvs->joint_limits_max[R_ANK_LAT]   = 35.0;
	uvs->joint_limits_max[R_ANK_SAG]   = 90.0;
	#ifdef COMP_FEET
	uvs->joint_limits_max[R_TOE] = 45.0;
	#endif

	// Left leg
	uvs->joint_limits_max[L_HIP_SAG]   = 70.0;
	uvs->joint_limits_max[L_HIP_LAT]   = 80.0;
	uvs->joint_limits_max[L_HIP_TRANS] = 50.0;
	uvs->joint_limits_max[L_KNEE_SAG]  = 120.0;
	uvs->joint_limits_max[L_ANK_LAT]   = 35.0;
	uvs->joint_limits_max[L_ANK_SAG]   = 90.0;
	#ifdef COMP_FEET
	uvs->joint_limits_max[L_TOE] = 45.0;
	#endif

	// Torso
	uvs->joint_limits_max[WAIST_LAT]   = 70.0;
	uvs->joint_limits_max[WAIST_SAG]   = 90.0;
	uvs->joint_limits_max[WAIST_TRANS] = 90.0;

	// Right arm
	uvs->joint_limits_max[R_SH_SAG] = 90.0;

	#ifdef LONG_ARMS
	uvs->joint_limits_max[R_SH_LAT] = 125.0;
	#else
	uvs->joint_limits_max[R_SH_LAT] = 35.0;
	#endif

	uvs->joint_limits_max[R_SH_TRANS] = 50.0;
	uvs->joint_limits_max[R_ELB]      = 45.0;

	#ifdef LONG_ARMS
	uvs->joint_limits_max[R_FORE_ARM_PLATE] = 90.0;
	uvs->joint_limits_max[R_WRJ1]           = 90.0;
	uvs->joint_limits_max[R_WRJ2]           = 90.0;
	#endif

	// Left arm
	uvs->joint_limits_max[L_SH_SAG] = 90.0;

	#ifdef LONG_ARMS
	uvs->joint_limits_max[L_SH_LAT] = 30.0;
	#else
	uvs->joint_limits_max[L_SH_LAT] = 120.0;
	#endif

	uvs->joint_limits_max[L_SH_TRANS] = 50.0;
	uvs->joint_limits_max[L_ELB]      = 45.0;

	#ifdef LONG_ARMS
	uvs->joint_limits_max[L_FORE_ARM_PLATE] = 90.0;
	uvs->joint_limits_max[L_WRJ1]           = 90.0;
	uvs->joint_limits_max[L_WRJ2]           = 90.0;
	#endif

	// Conversion from degrees to radians
	for(i=1; i<=COMAN_NB_JOINT_TOTAL; i++)
	{
		uvs->joint_limits_min[i] = uvs->joint_limits_min[i]*PI/180.0;
		uvs->joint_limits_max[i] = uvs->joint_limits_max[i]*PI/180.0;
	}

	// initial motors position
	for(i=1; i<=COMAN_NB_JOINT_ACTUATED; i++)
	{
		i_real = uvs->actuated2real[i];

		MBSdata->ux[i] = MBSdata->q[i_real];
		MBSdata->ux[COMAN_NB_JOINT_ACTUATED+i] = MBSdata->qd[i_real];
	}
}
