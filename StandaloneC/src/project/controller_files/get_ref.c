//---------------------------
//
// Nicolas Van der Noot
//
// Creation : 01/11/2013
// Last update : 08/07/2014
//
// Get the position, velocity ant torque references
//
//---------------------------

#include "controller_def.h"

void get_ref(ControllerStruct *cvs)
{
    // ---- Variables ---- //

    ControllerOutputs *ovs;
    
    int i;

    double q_ref[COMAN_NB_JOINT_ACTUATED];
    double qd_ref[COMAN_NB_JOINT_ACTUATED];
    double Qq_ref[COMAN_NB_JOINT_ACTUATED];


    ovs = cvs->Outputs;

    // ---- Position reference ---- //
    
    for(i=0; i<COMAN_NB_JOINT_ACTUATED; i++)
    {
        q_ref[i]  = 0.0;
    }

    // right arm
    q_ref[RIGHT_SHOULDER_PITCH] = cvs->q_ref_r_sh_sag;
    q_ref[RIGHT_SHOULDER_ROLL]  = cvs->q_ref_r_sh_lat;
    q_ref[RIGHT_SHOULDER_YAW]   = cvs->q_ref_r_sh_yaw;
    q_ref[RIGHT_ELBOW_PITCH]    = cvs->q_ref_r_elb;

    // left arm
    q_ref[LEFT_SHOULDER_PITCH] = cvs->q_ref_l_sh_sag;
    q_ref[LEFT_SHOULDER_ROLL]  = cvs->q_ref_l_sh_lat;
    q_ref[LEFT_SHOULDER_YAW]   = cvs->q_ref_l_sh_yaw;
    q_ref[LEFT_ELBOW_PITCH]    = cvs->q_ref_l_elb;


    // ---- Velocity reference ---- //
    
    for(i=0; i<COMAN_NB_JOINT_ACTUATED; i++)
    {
        qd_ref[i] = 0.0;
    }

    
    // ---- Torque reference ---- //
    
    for(i=0; i<COMAN_NB_JOINT_ACTUATED; i++)
    {
        Qq_ref[i] = 0.0;
    }
    
    
    // ---- References sent to the controller with limitations ---- //
    
    for(i=0; i<COMAN_NB_JOINT_ACTUATED; i++)
    {
        ovs->q_ref[i]  = limit_angle(q_ref[i]);
        ovs->qd_ref[i] = qd_ref[i];
        ovs->Qq_ref[i] = limit_value(Qq_ref[i], -TORQUE_LIMIT, TORQUE_LIMIT);

        /*
         * Q_REF_TRACKING  : positions tracking
         * QD_REF_TRACKING : velocity tracking
         * QQ_REF_TRACKING : torque tracking
         */
        ovs->imp_ctrl_index[i] = Q_REF_TRACKING;
    }
}
