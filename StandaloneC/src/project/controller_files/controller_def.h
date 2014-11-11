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


// waist
#define TRUNK_YAW   0
#define TRUNK_PITCH 1
#define TRUNK_ROLL  2

// hips pitch
#define R_HIP_PITCH 3
#define L_HIP_PITCH  4

// right leg
#define R_HIP_ROLL   5
#define R_HIP_YAW    6
#define R_KNEE_PITCH 7
#define R_FOOT_PITCH 8
#define R_FOOT_ROLL  9

// left leg
#define L_HIP_ROLL   10
#define L_HIP_YAW    11
#define L_KNEE_PITCH 12
#define L_FOOT_PITCH 13
#define L_FOOT_ROLL  14

// right arm
#define R_SHOULDER_PITCH 15
#define R_SHOULDER_ROLL  16
#define R_SHOULDER_YAW   17
#define R_ELBOW_PITCH    18

// left arm
#define L_SHOULDER_PITCH 19
#define L_SHOULDER_ROLL  20
#define L_SHOULDER_YAW   21
#define L_ELBOW_PITCH    22

#ifdef LONG_ARMS
#define R_FORE_ARM_PLATE_MOT 23
#define R_WRJ1_MOT 24
#define R_WRJ2_MOT 25

#define L_FORE_ARM_PLATE_MOT 26
#define L_WRJ1_MOT 27
#define L_WRJ2_MOT 28
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
#if !defined(STANDALONE) || !defined(SDL)
void arm_pos_ref(ControllerStruct *cvs);
#endif

// return a linear interpolation
double linear_interpolation(double x, double x1, double x2, double y1, double y2);

/*--------------------*/
#endif
