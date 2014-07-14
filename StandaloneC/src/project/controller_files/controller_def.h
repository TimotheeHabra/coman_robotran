//---------------------------
// Creation : 29/10/2013
// Last update : 08/07/2014
//
// Controller main header file
//
//---------------------------

#ifndef controller_def_h
#define controller_def_h
//--------------------*/

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "ControllersStruct.h"

/*
 * Uncomment the ' #include "simstruc.h" ' line to have access
 * to the ' printf ' function in the controller files
 * (the results of the printf will be available in the 'Command Window' of Matlab)
 *
 * This line must be commented before transfering the controller to the real robot
 */
//#include "simstruc.h"



// ---- Constants & Macros ---- //

// right and left legs
enum{R_LEG, L_LEG};

// -- Motors numbering -- //


// right leg
#define R_HIP_SAG_DOF 3
#define R_HIP_LAT_DOF 5
#define R_HIP_YAW_DOF 6
#define R_KNEE_SAG_DOF 7
#define R_ANK_LAT_DOF 9
#define R_ANK_SAG_DOF 8

// left leg
#define L_HIP_SAG_DOF 4
#define L_HIP_LAT_DOF 10
#define L_HIP_YAW_DOF 11
#define L_KNEE_SAG_DOF 12
#define L_ANK_LAT_DOF 14
#define L_ANK_SAG_DOF 13

// waist
#define WAIST_LAT_DOF 2
#define WAIST_SAG_DOF 1
#define WAIST_YAW_DOF 0

// right arm
#define R_SH_SAG_DOF 15
#define R_SH_LAT_DOF 16
#define R_SH_YAW_DOF 17
#define R_ELB_DOF 18

// left arm
#define L_SH_SAG_DOF 19
#define L_SH_LAT_DOF 20
#define L_SH_YAW_DOF 21
#define L_ELB_DOF 22

#ifdef LONG_ARMS
#define R_FORE_ARM_PLATE_DOF 23
#define R_WRJ1_DOF 24
#define R_WRJ2_DOF 25

#define L_FORE_ARM_PLATE_DOF 26
#define L_WRJ1_DOF 27
#define L_WRJ2_DOF 28
#endif

// number of joints
#ifdef LONG_ARMS
#define COMAN_NB_JOINT_ACTUATED	 29
#else
#define COMAN_NB_JOINT_ACTUATED	 23
#endif


// tracking: position, velocity or torque
#define Q_REF_TRACKING  1
#define QD_REF_TRACKING 2
#define QQ_REF_TRACKING 3

// pi
#define PI 3.14159265359
#define PI_2 (PI/2.0)
#define DEG_TO_RAD (PI/180.0)

// CoMan weight
#define GRAVITY_ACC 9.81
#define COMAN_MASS 28.3574
#define COMAN_WEIGHT (GRAVITY_ACC * COMAN_MASS)

#define TORQUE_LIMIT 50.0

// Controller period
#define CONTROLLER_PERIOD 0.001


// home position
#define ELBOW_HOME (3.0*DEG_TO_RAD)


#ifdef LONG_ARMS
#define LAT_SH_HOME (-85.0*DEG_TO_RAD)
#else
#define LAT_SH_HOME (5.0*DEG_TO_RAD)
#endif

// ---- Custom Functions ---- //

// initialization and main loop
#ifdef __cplusplus
extern "C" {
#endif
	void controller_init(ControllerStruct *cvs);
	void controller_loop(ControllerStruct *cvs);
#ifdef __cplusplus
}
#endif

// useful functions
double limit_angle(double angle);
double limit_value(double value, double this_min, double this_max);

// get references to sent to the impedance controller
void get_ref(ControllerStruct *cvs);

// get the position references for the arms
#ifndef STANDALONE
void arm_pos_ref(ControllerStruct *cvs);
#endif

// return a linear interpolation
double linear_interpolation(double x, double x1, double x2, double y1, double y2);

/*--------------------*/
#endif
