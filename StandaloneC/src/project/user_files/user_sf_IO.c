/*===========================================================================*
 *
 *  user_sf_IO.c
 * 
 *  Generation date: Mon Jul 21 17:32:31 2014

 * 
 *  (c) Universite catholique de Louvain
 *      Departement de Mecanique 
 *      Unite de Production Mecanique et Machines 
 *      2, Place du Levant 
 *      1348 Louvain-la-Neuve 
 *  http://www.robotran.be// 
 *  
/*===========================================================================*/

#include "MBSfun.h" 
#include "user_sf_IO.h" 
#include "sfdef.h" 
#include "userDef.h"
#include "ControllersStruct.h"


UserIOStruct * initUserIO(MBSdataStruct *s)
{
    UserIOStruct *uvs;

    int i;

    uvs = (UserIOStruct*) malloc(sizeof(UserIOStruct));

    // Voltage //
    for (i=1;i<=29;i++)
    {
        uvs->Voltage[i] = 0.0;
    }

    // Control //
    for (i=1;i<=29;i++)
    {
        uvs->Control[i] = 0.0;
    }

    // Actuator_KKs //
    for (i=1;i<=35;i++)
    {
        uvs->Actuator_KKs[i] = 0.0;
    }

    // Actuator_DDs //
    for (i=1;i<=35;i++)
    {
        uvs->Actuator_DDs[i] = 0.0;
    }

    // Actuator_Jdrives //
    uvs->Actuator_Jdrives = 0.0;

    // Actuator_Ddrives //
    uvs->Actuator_Ddrives = 0.0;

    // Actuator_VTgain //
    uvs->Actuator_VTgain = 0.0;

    // actuated2real //
    for (i=1;i<=29;i++)
    {
        uvs->actuated2real[i] = 0;
    }

    // real2actuated //
    for (i=1;i<=29;i++)
    {
        uvs->real2actuated[i] = 0;
    }

    // joint_limits_min //
    for (i=1;i<=35;i++)
    {
        uvs->joint_limits_min[i] = 0.0;
    }

    // joint_limits_max //
    for (i=1;i<=35;i++)
    {
        uvs->joint_limits_max[i] = 0.0;
    }

    // waist_relative_ground //
    uvs->waist_relative_ground = 0.0;

    // GRF_r //
    for (i=1;i<=3;i++)
    {
        uvs->GRF_r[i] = 0.0;
    }

    // GRF_l //
    for (i=1;i<=3;i++)
    {
        uvs->GRF_l[i] = 0.0;
    }

    // GRM_r //
    for (i=1;i<=3;i++)
    {
        uvs->GRM_r[i] = 0.0;
    }

    // GRM_l //
    for (i=1;i<=3;i++)
    {
        uvs->GRM_l[i] = 0.0;
    }

    // GRF_r_dist //
    for (i=1;i<=3;i++)
    {
        uvs->GRF_r_dist[i] = 0.0;
    }

    // GRF_l_dist //
    for (i=1;i<=3;i++)
    {
        uvs->GRF_l_dist[i] = 0.0;
    }

    // GRM_r_dist //
    for (i=1;i<=3;i++)
    {
        uvs->GRM_r_dist[i] = 0.0;
    }

    // GRM_l_dist //
    for (i=1;i<=3;i++)
    {
        uvs->GRM_l_dist[i] = 0.0;
    }

    // mu_grf //
    uvs->mu_grf = 0.0;

    // F_left_leg //
    uvs->F_left_leg = 0.0;

    // F_right_leg //
    uvs->F_right_leg = 0.0;

    // Msize_GCM //
    uvs->Msize_GCM = 0;

    // Msize_GCM_prox //
    uvs->Msize_GCM_prox = 0;

    // Msize_GCM_dist //
    uvs->Msize_GCM_dist = 0;

    // rn_left_x //
    for (i=1;i<=200;i++)
    {
        uvs->rn_left_x[i] = 0.0;
    }

    // rn_left_y //
    for (i=1;i<=200;i++)
    {
        uvs->rn_left_y[i] = 0.0;
    }

    // rn_left_z //
    for (i=1;i<=200;i++)
    {
        uvs->rn_left_z[i] = 0.0;
    }

    // rn_right_x //
    for (i=1;i<=200;i++)
    {
        uvs->rn_right_x[i] = 0.0;
    }

    // rn_right_y //
    for (i=1;i<=200;i++)
    {
        uvs->rn_right_y[i] = 0.0;
    }

    // rn_right_z //
    for (i=1;i<=200;i++)
    {
        uvs->rn_right_z[i] = 0.0;
    }

    // temp_grfx_left //
    for (i=1;i<=200;i++)
    {
        uvs->temp_grfx_left[i] = 0.0;
    }

    // temp_grfy_left //
    for (i=1;i<=200;i++)
    {
        uvs->temp_grfy_left[i] = 0.0;
    }

    // temp_grfx_right //
    for (i=1;i<=200;i++)
    {
        uvs->temp_grfx_right[i] = 0.0;
    }

    // temp_grfy_right //
    for (i=1;i<=200;i++)
    {
        uvs->temp_grfy_right[i] = 0.0;
    }

    // flag_grfx_left //
    for (i=1;i<=200;i++)
    {
        uvs->flag_grfx_left[i] = 0;
    }

    // flag_grfy_left //
    for (i=1;i<=200;i++)
    {
        uvs->flag_grfy_left[i] = 0;
    }

    // flag_grfx_right //
    for (i=1;i<=200;i++)
    {
        uvs->flag_grfx_right[i] = 0;
    }

    // flag_grfy_right //
    for (i=1;i<=200;i++)
    {
        uvs->flag_grfy_right[i] = 0;
    }

    // rn_left_prox_x //
    for (i=1;i<=150;i++)
    {
        uvs->rn_left_prox_x[i] = 0.0;
    }

    // rn_left_prox_y //
    for (i=1;i<=150;i++)
    {
        uvs->rn_left_prox_y[i] = 0.0;
    }

    // rn_left_prox_z //
    for (i=1;i<=150;i++)
    {
        uvs->rn_left_prox_z[i] = 0.0;
    }

    // rn_right_prox_x //
    for (i=1;i<=150;i++)
    {
        uvs->rn_right_prox_x[i] = 0.0;
    }

    // rn_right_prox_y //
    for (i=1;i<=150;i++)
    {
        uvs->rn_right_prox_y[i] = 0.0;
    }

    // rn_right_prox_z //
    for (i=1;i<=150;i++)
    {
        uvs->rn_right_prox_z[i] = 0.0;
    }

    // temp_grfx_left_prox //
    for (i=1;i<=150;i++)
    {
        uvs->temp_grfx_left_prox[i] = 0.0;
    }

    // temp_grfy_left_prox //
    for (i=1;i<=150;i++)
    {
        uvs->temp_grfy_left_prox[i] = 0.0;
    }

    // temp_grfx_right_prox //
    for (i=1;i<=150;i++)
    {
        uvs->temp_grfx_right_prox[i] = 0.0;
    }

    // temp_grfy_right_prox //
    for (i=1;i<=150;i++)
    {
        uvs->temp_grfy_right_prox[i] = 0.0;
    }

    // flag_grfx_left_prox //
    for (i=1;i<=150;i++)
    {
        uvs->flag_grfx_left_prox[i] = 0;
    }

    // flag_grfy_left_prox //
    for (i=1;i<=150;i++)
    {
        uvs->flag_grfy_left_prox[i] = 0;
    }

    // flag_grfx_right_prox //
    for (i=1;i<=150;i++)
    {
        uvs->flag_grfx_right_prox[i] = 0;
    }

    // flag_grfy_right_prox //
    for (i=1;i<=150;i++)
    {
        uvs->flag_grfy_right_prox[i] = 0;
    }

    // rn_left_dist_x //
    for (i=1;i<=60;i++)
    {
        uvs->rn_left_dist_x[i] = 0.0;
    }

    // rn_left_dist_y //
    for (i=1;i<=60;i++)
    {
        uvs->rn_left_dist_y[i] = 0.0;
    }

    // rn_left_dist_z //
    for (i=1;i<=60;i++)
    {
        uvs->rn_left_dist_z[i] = 0.0;
    }

    // rn_right_dist_x //
    for (i=1;i<=60;i++)
    {
        uvs->rn_right_dist_x[i] = 0.0;
    }

    // rn_right_dist_y //
    for (i=1;i<=60;i++)
    {
        uvs->rn_right_dist_y[i] = 0.0;
    }

    // rn_right_dist_z //
    for (i=1;i<=60;i++)
    {
        uvs->rn_right_dist_z[i] = 0.0;
    }

    // temp_grfx_left_dist //
    for (i=1;i<=60;i++)
    {
        uvs->temp_grfx_left_dist[i] = 0.0;
    }

    // temp_grfy_left_dist //
    for (i=1;i<=60;i++)
    {
        uvs->temp_grfy_left_dist[i] = 0.0;
    }

    // temp_grfx_right_dist //
    for (i=1;i<=60;i++)
    {
        uvs->temp_grfx_right_dist[i] = 0.0;
    }

    // temp_grfy_right_dist //
    for (i=1;i<=60;i++)
    {
        uvs->temp_grfy_right_dist[i] = 0.0;
    }

    // flag_grfx_left_dist //
    for (i=1;i<=60;i++)
    {
        uvs->flag_grfx_left_dist[i] = 0;
    }

    // flag_grfy_left_dist //
    for (i=1;i<=60;i++)
    {
        uvs->flag_grfy_left_dist[i] = 0;
    }

    // flag_grfx_right_dist //
    for (i=1;i<=60;i++)
    {
        uvs->flag_grfx_right_dist[i] = 0;
    }

    // flag_grfy_right_dist //
    for (i=1;i<=60;i++)
    {
        uvs->flag_grfy_right_dist[i] = 0;
    }

    // last_tsim_int_tau //
    uvs->last_tsim_int_tau = 0.0;

    // last_err_tau //
    for (i=1;i<=29;i++)
    {
        uvs->last_err_tau[i] = 0.0;
    }

    // last_int_err_tau //
    for (i=1;i<=29;i++)
    {
        uvs->last_int_err_tau[i] = 0.0;
    }

    // real_theta_3_waist //
    uvs->real_theta_3_waist = 0.0;

    // real_omega_3_waist //
    uvs->real_omega_3_waist = 0.0;

    // last_t_ctrl //
    uvs->last_t_ctrl = 0.0;

    // keyboard_command_1 //
    uvs->keyboard_command_1 = 0;

    // keyboard_command_2 //
    uvs->keyboard_command_2 = 0;

    // Qq_out //
    for (i=1;i<=29;i++)
    {
        uvs->Qq_out[i] = 0.0;
    }

    // tsim_out //
    uvs->tsim_out = 0.0;

    // q_ref //
    for (i=1;i<=29;i++)
    {
        uvs->q_ref[i] = 0.0;
    }

    // qd_ref //
    for (i=1;i<=29;i++)
    {
        uvs->qd_ref[i] = 0.0;
    }

    // Qq_ref //
    for (i=1;i<=29;i++)
    {
        uvs->Qq_ref[i] = 0.0;
    }

    // imp_ctrl_index //
    for (i=1;i<=29;i++)
    {
        uvs->imp_ctrl_index[i] = 0;
    }

    // out //
    for (i=1;i<=10;i++)
    {
        uvs->out[i] = 0.0;
    }

    // cvs //
    uvs->cvs = init_ControllerStruct();
    // stop simulation //
    uvs->stop_simu = 0;

    // simbodyStruct //
    #ifdef SIMBODY
    uvs->simbodyStruct = init_SimbodyStruct();
    #endif

    return uvs;
}


void freeUserIO(UserIOStruct *uvs, MBSdataStruct *s)
{

    // ControllerStruct: cvs //
    free_ControllerStruct(uvs->cvs);

    // SimbodyStruct: simbodyStruct //
    #ifdef SIMBODY
    free_SimbodyStruct(uvs->simbodyStruct);
    #endif

    free(uvs);
}

