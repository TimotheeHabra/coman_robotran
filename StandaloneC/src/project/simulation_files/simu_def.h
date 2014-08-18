//---------------------------
// Nicolas Van der Noot
//
// Creation : 29/10/2013
// Last update : 08/07/2014
//
// Simulation files main header
//
//---------------------------

#ifndef simulation_def_h
#define simulation_def_h
//--------------------*/

#include <math.h>
#include <stdlib.h>

#include "user_sf_IO.h"
#include "MBSdef.h"
#include "MBSfun.h"
#include "MBSdataStruct.h"
#include "ControllersStruct.h"
#include "nrutil.h"

#include "controller_io.h"


// ---- Constants & Macros ---- //

// controller calls
#define PERIOD_CTRL 1.0e-3
#define TIME_EPSILON 1.0e-5


// -- ALL DoFs numbering (Robotran) -- //


#ifdef COMP_FEET

// floating base
#define FJ_T1 1
#define FJ_T2 2
#define FJ_T3 3
#define FJ_R1 4
#define FJ_R2 5
#define FJ_R3 6

// right leg
#define R_HIP_SAG 7
#define R_HIP_LAT 8
#define R_HIP_TRANS 9
#define R_KNEE_SAG 10
#define R_ANK_LAT 11
#define R_ANK_SAG 12
#define R_TOE 13

// left leg
#define L_HIP_SAG 14
#define L_HIP_LAT 15
#define L_HIP_TRANS 16
#define L_KNEE_SAG 17
#define L_ANK_LAT 18
#define L_ANK_SAG 19
#define L_TOE 20

// waist
#define WAIST_LAT 21
#define WAIST_SAG 22
#define WAIST_TRANS 23

// right arm
#define R_SH_SAG 24
#define R_SH_LAT 25
#define R_SH_TRANS 26
#define R_ELB 27

// left arm
#define L_SH_SAG 28
#define L_SH_LAT 29
#define L_SH_TRANS 30
#define L_ELB 31


#else

#ifdef LONG_ARMS

// floating base
#define FJ_T1 1
#define FJ_T2 2
#define FJ_T3 3
#define FJ_R1 4
#define FJ_R2 5
#define FJ_R3 6

// right leg
#define R_HIP_SAG 7
#define R_HIP_LAT 8
#define R_HIP_TRANS 9
#define R_KNEE_SAG 10
#define R_ANK_LAT 11
#define R_ANK_SAG 12

// right leg
#define L_HIP_SAG 13
#define L_HIP_LAT 14
#define L_HIP_TRANS 15
#define L_KNEE_SAG 16
#define L_ANK_LAT 17
#define L_ANK_SAG 18

// waist
#define WAIST_LAT 19
#define WAIST_SAG 20
#define WAIST_TRANS 21

// right arm
#define R_SH_SAG 22
#define R_SH_LAT 23
#define R_SH_TRANS 24
#define R_ELB 25
#define R_FORE_ARM_PLATE 26
#define R_WRJ1 27
#define R_WRJ2 28

// left arm
#define L_SH_SAG 29
#define L_SH_LAT 30
#define L_SH_TRANS 31
#define L_ELB 32
#define L_FORE_ARM_PLATE 33
#define L_WRJ1 34
#define L_WRJ2 35

#else

// floating base
#define FJ_T1 1
#define FJ_T2 2
#define FJ_T3 3
#define FJ_R1 4
#define FJ_R2 5
#define FJ_R3 6

// right leg
#define R_HIP_SAG 7
#define R_HIP_LAT 8
#define R_HIP_TRANS 9
#define R_KNEE_SAG 10
#define R_ANK_LAT 11
#define R_ANK_SAG 12

// left leg
#define L_HIP_SAG 13
#define L_HIP_LAT 14
#define L_HIP_TRANS 15
#define L_KNEE_SAG 16
#define L_ANK_LAT 17
#define L_ANK_SAG 18

// waist
#define WAIST_LAT 19
#define WAIST_SAG 20
#define WAIST_TRANS 21

// right arm
#define R_SH_SAG 22
#define R_SH_LAT 23
#define R_SH_TRANS 24
#define R_ELB 25

// left arm
#define L_SH_SAG 26
#define L_SH_LAT 27
#define L_SH_TRANS 28
#define L_ELB 29

#endif

#endif



// -- Motors numbering (according to the real CoMan) -- //

// right leg
#define R_HIP_SAG_CTRL_MOT 4
#define R_HIP_LAT_CTRL_MOT 6
#define R_HIP_TRANS_CTRL_MOT 7
#define R_KNEE_SAG_CTRL_MOT 8
#define R_ANK_LAT_CTRL_MOT 10
#define R_ANK_SAG_CTRL_MOT 9

// left leg
#define L_HIP_SAG_CTRL_MOT 5
#define L_HIP_LAT_CTRL_MOT 11
#define L_HIP_TRANS_CTRL_MOT 12
#define L_KNEE_SAG_CTRL_MOT 13
#define L_ANK_LAT_CTRL_MOT 15
#define L_ANK_SAG_CTRL_MOT 14

// waist
#define WAIST_LAT_CTRL_MOT 3
#define WAIST_SAG_CTRL_MOT 2
#define WAIST_TRANS_CTRL_MOT 1

// right arm
#define R_SH_SAG_CTRL_MOT 16
#define R_SH_LAT_CTRL_MOT 17
#define R_SH_TRANS_CTRL_MOT 18
#define R_ELB_CTRL_MOT 19

// left arm
#define L_SH_SAG_CTRL_MOT 20
#define L_SH_LAT_CTRL_MOT 21
#define L_SH_TRANS_CTRL_MOT 22
#define L_ELB_CTRL_MOT 23


#ifdef LONG_ARMS
#define R_FORE_ARM_PLATE_CTRL_MOT 24
#define R_WRJ1_CTRL_MOT 25
#define R_WRJ2_CTRL_MOT 26

#define L_FORE_ARM_PLATE_CTRL_MOT 27
#define L_WRJ1_CTRL_MOT 28
#define L_WRJ2_CTRL_MOT 29
#endif


// -- Sensors -- //

#ifdef COMP_FEET

// S sensors
#define S_MIDWAIST 5
#define S_RFOOTS 15
#define S_LFOOTS 28
#define S_RFOOTS_DIST 17
#define S_LFOOTS_DIST 29

// F sensors
#define RFOOT_FSENS_ID 1
#define LFOOT_FSENS_ID 3
#define RFOOT_DIST_FSENS_ID 2
#define LFOOT_DIST_FSENS_ID 4
#define TORSO_PERT_FSENS_ID 5

#else

#ifdef LONG_ARMS

// S sensors
#define S_MIDWAIST 5
#define S_RFOOTS 15
#define S_LFOOTS 26

// F sensors
#define RFOOT_FSENS_ID 1
#define LFOOT_FSENS_ID 2
#define TORSO_PERT_FSENS_ID 3
#define RHAND_FSENS_ID 5
#define LHAND_FSENS_ID 6

#else

// S sensors
#define S_MIDWAIST 5
#define S_RFOOTS 15
#define S_LFOOTS 27

// F sensors
#define RFOOT_FSENS_ID 1
#define LFOOT_FSENS_ID 2
#define TORSO_PERT_FSENS_ID 3

#endif

#endif


// -- Other -- //

// limiting external forces
#define MAX_EXT_FORCES	5000.0
#define MAX_EXT_MOMENTS	5000.0

// number of joints
#define COMAN_NB_JOINT_BASE	 6       // number of joints of the floating base


// total number of actuated joints (= nb of motors)
#ifdef LONG_ARMS
#define COMAN_NB_JOINT_ACTUATED	 29
#else
#define COMAN_NB_JOINT_ACTUATED	 23
#endif

// number of joints of the model
#ifdef COMP_FEET
#define COMAN_NB_JOINT_TOTAL 31
#else

#ifdef LONG_ARMS
#define COMAN_NB_JOINT_TOTAL 35
#else
#define COMAN_NB_JOINT_TOTAL 29
#endif

#endif


// pi
#define PI 3.14159265359
#define PI_2 (PI/2.0)

// thresholds used to stop the simulation
#define FALL_THRESHOLD 0.15
#define GROUND_FORCES_THRESHOLD 10000.0


// limiting actuator voltages
#define MAX_ACT_VOLTAGE 100.0

// GCM (Ground Contact Model)
#define K_GCM 10000.0    // stiffness coeff
#define D_GCM 10.0       // damping coeff
#define MU_GCM 0.9       // friction coeff (mu) [-]
#define K_GZ_GCM 27818.0 // k_gz           (HG: 27818)
#define V_GZ_MAX_GCM 0.3 // v_gz_max [m/s] (HG: 0.03)


// ---- Custom Functions ---- //

// simulation loop
void simulink_outputs(MBSdataStruct *MBSdata);
void simu_controller_loop(MBSdataStruct *MBSdata);
void stop_simu(MBSdataStruct *MBSdata);

#ifdef STANDALONE
void user_finalization(MBSdataStruct *MBSdata);
#endif

// controller interface
void controller_init_interface(MBSdataStruct *MBSdata);
void controller_loop_interface(MBSdataStruct *MBSdata);
void controller_close_interface(MBSdataStruct *MBSdata);

// useful functions
double limit_function(double value, double this_min, double this_max);

// actuators
void init_actuator_model(MBSdataStruct *MBSdata);

// GCM (Ground Contact Model)
double get_ground_height(double x, double y, double tsim, MBSdataStruct *MBSdata);
void ground_mesh_model(double PxF[4], double RxF[4][4],
					   double VxF[4], double OMxF[4],
					   MBSdataStruct *MBSdata, double tsim,
					   int ixF, double *dxF, double *SWr);
void init_GCM(MBSdataStruct *MBSdata);
double z_left_foot(double x, double y);
double z_right_foot(double x, double y);

// impedance controller
void impedance_controller(MBSdataStruct *MBSdata, Ctrl_Outputs *ovs);
void get_IC_gains(int id_coman, double position_PID[3], double torque_PID[3]);

// initialization
void opti_parameters_init_simu(MBSdataStruct *MBSdata);

// random number
double rnd_simu(void);

/*--------------------*/
#endif
