//---------------------------
//
// Nicolas Van der Noot
//
// Creation : 28/07/2014
// Last update : 28/07/2014
//
// Controller IO structures
//
//---------------------------

#ifndef _CONTROLLER_IO_H_
#define _CONTROLLER_IO_H_

// total number of actuated joints (= nb of motors)
#ifdef LONG_ARMS
#define COMAN_NB_JOINT_ACTUATED	 29
#else
#define COMAN_NB_JOINT_ACTUATED	 23
#endif

// inputs structure
typedef struct Ctrl_Inputs
{
    double t;
    double q[COMAN_NB_JOINT_ACTUATED];
    double qd[COMAN_NB_JOINT_ACTUATED];
    double Qq[COMAN_NB_JOINT_ACTUATED];
    double q_mot[COMAN_NB_JOINT_ACTUATED];
    double qd_mot[COMAN_NB_JOINT_ACTUATED];
    double F_Rfoot[3];
    double F_Lfoot[3];
    double T_Rfoot[3];
    double T_Lfoot[3];
    double IMU_Orientation[9];
    double IMU_Angular_Rate[3];
    double IMU_Acceleration[3];

} Ctrl_Inputs;

// outputs structure
typedef struct Ctrl_Outputs
{
    double q_ref[COMAN_NB_JOINT_ACTUATED];
    double qd_ref[COMAN_NB_JOINT_ACTUATED];
    double Qq_ref[COMAN_NB_JOINT_ACTUATED];
    int imp_ctrl_index[COMAN_NB_JOINT_ACTUATED];

} Ctrl_Outputs;

#endif
