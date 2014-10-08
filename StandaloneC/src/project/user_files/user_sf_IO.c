/*===========================================================================*
  *
  *  user_sf_IO.c
  *	
  *  Project:	coman_robotran
  * 
  *  Generation date: 08-Oct-2014 15:51:03
  * 
  *  (c) Universite catholique de Louvain
  *      D�partement de M�canique 
  *      Unit� de Production M�canique et Machines 
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

    for (i=1;i<=29;i++)
    {
        uvs->Voltage[i] = 0.0;
    }

    for (i=1;i<=29;i++)
    {
        uvs->Control[i] = 0.0;
    }

    for (i=1;i<=35;i++)
    {
        uvs->Actuator_KKs[i] = 0.0;
    }

    for (i=1;i<=35;i++)
    {
        uvs->Actuator_DDs[i] = 0.0;
    }

    uvs->Actuator_Jdrives = 0.0;

    uvs->Actuator_Ddrives = 0.0;

    uvs->Actuator_VTgain = 0.0;

    for (i=1;i<=29;i++)
    {
        uvs->actuated2real[i] = 0;
    }

    for (i=1;i<=35;i++)
    {
        uvs->real2actuated[i] = 0;
    }

    for (i=1;i<=35;i++)
    {
        uvs->joint_limits_min[i] = 0.0;
    }

    for (i=1;i<=35;i++)
    {
        uvs->joint_limits_max[i] = 0.0;
    }

    uvs->waist_relative_ground = 0.0;

    for (i=1;i<=3;i++)
    {
        uvs->GRF_r[i] = 0.0;
    }

    for (i=1;i<=3;i++)
    {
        uvs->GRF_l[i] = 0.0;
    }

    for (i=1;i<=3;i++)
    {
        uvs->GRM_r[i] = 0.0;
    }

    for (i=1;i<=3;i++)
    {
        uvs->GRM_l[i] = 0.0;
    }

    for (i=1;i<=3;i++)
    {
        uvs->GRF_r_dist[i] = 0.0;
    }

    for (i=1;i<=3;i++)
    {
        uvs->GRF_l_dist[i] = 0.0;
    }

    for (i=1;i<=3;i++)
    {
        uvs->GRM_r_dist[i] = 0.0;
    }

    for (i=1;i<=3;i++)
    {
        uvs->GRM_l_dist[i] = 0.0;
    }

    uvs->mu_grf = 0.0;

    uvs->F_left_leg = 0.0;

    uvs->F_right_leg = 0.0;

    uvs->Msize_GCM = 0;

    uvs->Msize_GCM_prox = 0;

    uvs->Msize_GCM_dist = 0;

    for (i=1;i<=200;i++)
    {
        uvs->rn_left_x[i] = 0.0;
    }

    for (i=1;i<=200;i++)
    {
        uvs->rn_left_y[i] = 0.0;
    }

    for (i=1;i<=200;i++)
    {
        uvs->rn_left_z[i] = 0.0;
    }

    for (i=1;i<=200;i++)
    {
        uvs->rn_right_x[i] = 0.0;
    }

    for (i=1;i<=200;i++)
    {
        uvs->rn_right_y[i] = 0.0;
    }

    for (i=1;i<=200;i++)
    {
        uvs->rn_right_z[i] = 0.0;
    }

    for (i=1;i<=200;i++)
    {
        uvs->temp_grfx_left[i] = 0.0;
    }

    for (i=1;i<=200;i++)
    {
        uvs->temp_grfy_left[i] = 0.0;
    }

    for (i=1;i<=200;i++)
    {
        uvs->temp_grfx_right[i] = 0.0;
    }

    for (i=1;i<=200;i++)
    {
        uvs->temp_grfy_right[i] = 0.0;
    }

    for (i=1;i<=200;i++)
    {
        uvs->flag_grfx_left[i] = 0;
    }

    for (i=1;i<=200;i++)
    {
        uvs->flag_grfy_left[i] = 0;
    }

    for (i=1;i<=200;i++)
    {
        uvs->flag_grfx_right[i] = 0;
    }

    for (i=1;i<=200;i++)
    {
        uvs->flag_grfy_right[i] = 0;
    }

    for (i=1;i<=150;i++)
    {
        uvs->rn_left_prox_x[i] = 0.0;
    }

    for (i=1;i<=150;i++)
    {
        uvs->rn_left_prox_y[i] = 0.0;
    }

    for (i=1;i<=150;i++)
    {
        uvs->rn_left_prox_z[i] = 0.0;
    }

    for (i=1;i<=150;i++)
    {
        uvs->rn_right_prox_x[i] = 0.0;
    }

    for (i=1;i<=150;i++)
    {
        uvs->rn_right_prox_y[i] = 0.0;
    }

    for (i=1;i<=150;i++)
    {
        uvs->rn_right_prox_z[i] = 0.0;
    }

    for (i=1;i<=150;i++)
    {
        uvs->temp_grfx_left_prox[i] = 0.0;
    }

    for (i=1;i<=150;i++)
    {
        uvs->temp_grfy_left_prox[i] = 0.0;
    }

    for (i=1;i<=150;i++)
    {
        uvs->temp_grfx_right_prox[i] = 0.0;
    }

    for (i=1;i<=150;i++)
    {
        uvs->temp_grfy_right_prox[i] = 0.0;
    }

    for (i=1;i<=150;i++)
    {
        uvs->flag_grfx_left_prox[i] = 0;
    }

    for (i=1;i<=150;i++)
    {
        uvs->flag_grfy_left_prox[i] = 0;
    }

    for (i=1;i<=150;i++)
    {
        uvs->flag_grfx_right_prox[i] = 0;
    }

    for (i=1;i<=150;i++)
    {
        uvs->flag_grfy_right_prox[i] = 0;
    }

    for (i=1;i<=60;i++)
    {
        uvs->rn_left_dist_x[i] = 0.0;
    }

    for (i=1;i<=60;i++)
    {
        uvs->rn_left_dist_y[i] = 0.0;
    }

    for (i=1;i<=60;i++)
    {
        uvs->rn_left_dist_z[i] = 0.0;
    }

    for (i=1;i<=60;i++)
    {
        uvs->rn_right_dist_x[i] = 0.0;
    }

    for (i=1;i<=60;i++)
    {
        uvs->rn_right_dist_y[i] = 0.0;
    }

    for (i=1;i<=60;i++)
    {
        uvs->rn_right_dist_z[i] = 0.0;
    }

    for (i=1;i<=60;i++)
    {
        uvs->temp_grfx_left_dist[i] = 0.0;
    }

    for (i=1;i<=60;i++)
    {
        uvs->temp_grfy_left_dist[i] = 0.0;
    }

    for (i=1;i<=60;i++)
    {
        uvs->temp_grfx_right_dist[i] = 0.0;
    }

    for (i=1;i<=60;i++)
    {
        uvs->temp_grfy_right_dist[i] = 0.0;
    }

    for (i=1;i<=60;i++)
    {
        uvs->flag_grfx_left_dist[i] = 0;
    }

    for (i=1;i<=60;i++)
    {
        uvs->flag_grfy_left_dist[i] = 0;
    }

    for (i=1;i<=60;i++)
    {
        uvs->flag_grfx_right_dist[i] = 0;
    }

    for (i=1;i<=60;i++)
    {
        uvs->flag_grfy_right_dist[i] = 0;
    }

    uvs->last_tsim_int_tau = 0.0;

    for (i=1;i<=29;i++)
    {
        uvs->last_err_tau[i] = 0.0;
    }

    for (i=1;i<=29;i++)
    {
        uvs->last_int_err_tau[i] = 0.0;
    }

    uvs->real_theta_3_waist = 0.0;

    uvs->real_omega_3_waist = 0.0;

    uvs->last_t_ctrl = 0.0;

    uvs->keyboard_command_1 = 0;

    uvs->keyboard_command_2 = 0;

    for (i=1;i<=29;i++)
    {
        uvs->Qq_out[i] = 0.0;
    }

    uvs->tsim_out = 0.0;

    for (i=1;i<=29;i++)
    {
        uvs->q_ref[i] = 0.0;
    }

    for (i=1;i<=29;i++)
    {
        uvs->qd_ref[i] = 0.0;
    }

    for (i=1;i<=29;i++)
    {
        uvs->Qq_ref[i] = 0.0;
    }

    for (i=1;i<=29;i++)
    {
        uvs->imp_ctrl_index[i] = 0;
    }

    for (i=1;i<=10;i++)
    {
        uvs->out[i] = 0.0;
    }

    uvs->stop_simu = 0;

    uvs->cvs = init_ControllerStruct();

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

#ifndef CMEX 
 
void sf_set_user_input_sizes(SimStruct *S, MBSdataStruct *MBSdata, int sf_ninput) 
{ 
   if (SF_N_USER_INPUT > 0) { // warning: index starts at sf_ninput 
        // example: ssSetInputPortWidth(S,sf_ninput,10); 
   } 
} 

void sf_set_user_output_sizes(SimStruct *S, MBSdataStruct *MBSdata) 
        // example: ssSetOutputPortWidth(S, SF_NOUTPUT, 10); 
{ 
   if (SF_N_USER_OUTPUT > 0) { // warning: index starts at SF_NOUTPUT 

       /* User output port0 : Qq_out */ 
       ssSetOutputPortWidth(S, SF_NOUTPUT, 29); 

       /* User output port1 : tsim_out */ 
       ssSetOutputPortWidth(S, SF_NOUTPUT+1, 1); 

       /* User output port2 : q_ref */ 
       ssSetOutputPortWidth(S, SF_NOUTPUT+2, 29); 

       /* User output port3 : qd_ref */ 
       ssSetOutputPortWidth(S, SF_NOUTPUT+3, 29); 

       /* User output port4 : Qq_ref */ 
       ssSetOutputPortWidth(S, SF_NOUTPUT+4, 29); 

       /* User output port5 : imp_ctrl_index */ 
       ssSetOutputPortWidth(S, SF_NOUTPUT+5, 29); 

       /* User output port6 : out */ 
       ssSetOutputPortWidth(S, SF_NOUTPUT+6, 10); 

       /* User output port7 : stop_simu */ 
       ssSetOutputPortWidth(S, SF_NOUTPUT+7, 1); 
   } 
} 

void sf_get_user_input(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds, int sf_ninput) 
{ 
    // warning: index starts at sf_ninput
    // example: InputRealPtrsType uPtrs0 = ssGetInputPortRealSignalPtrs(S,sf_ninput);
    //          MBSdata->user_IO->var1 = *uPtrs0[0];
} 

void sf_set_user_output(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds) 
{ 
    // warning: index starts at SF_NOUTPUT  
    // example: real_T *y0 = ssGetOutputPortRealSignal(S,SF_NOUTPUT); 
    //          *y0 = MBSdata->user_IO->var1;  
    int i;
  real_T *y0 = ssGetOutputPortRealSignal(S,SF_NOUTPUT); 
  real_T *y1 = ssGetOutputPortRealSignal(S,SF_NOUTPUT+1); 
  real_T *y2 = ssGetOutputPortRealSignal(S,SF_NOUTPUT+2); 
  real_T *y3 = ssGetOutputPortRealSignal(S,SF_NOUTPUT+3); 
  real_T *y4 = ssGetOutputPortRealSignal(S,SF_NOUTPUT+4); 
  real_T *y5 = ssGetOutputPortRealSignal(S,SF_NOUTPUT+5); 
  real_T *y6 = ssGetOutputPortRealSignal(S,SF_NOUTPUT+6); 
  real_T *y7 = ssGetOutputPortRealSignal(S,SF_NOUTPUT+7); 

   /* User output port0 : Qq_out */ 
   if (ssGetOutputPortConnected(S,SF_NOUTPUT)) 
      for (i=1;i<=29;i++)
          y0[i-1] = MBSdata->user_IO->Qq_out[i]; 

   /* User output port1 : tsim_out */ 
   if (ssGetOutputPortConnected(S,SF_NOUTPUT+1)) 
      *y1 = MBSdata->user_IO->tsim_out; 

   /* User output port2 : q_ref */ 
   if (ssGetOutputPortConnected(S,SF_NOUTPUT+2)) 
      for (i=1;i<=29;i++)
          y2[i-1] = MBSdata->user_IO->q_ref[i]; 

   /* User output port3 : qd_ref */ 
   if (ssGetOutputPortConnected(S,SF_NOUTPUT+3)) 
      for (i=1;i<=29;i++)
          y3[i-1] = MBSdata->user_IO->qd_ref[i]; 

   /* User output port4 : Qq_ref */ 
   if (ssGetOutputPortConnected(S,SF_NOUTPUT+4)) 
      for (i=1;i<=29;i++)
          y4[i-1] = MBSdata->user_IO->Qq_ref[i]; 

   /* User output port5 : imp_ctrl_index */ 
   if (ssGetOutputPortConnected(S,SF_NOUTPUT+5)) 
      for (i=1;i<=29;i++)
          y5[i-1] = MBSdata->user_IO->imp_ctrl_index[i]; 

   /* User output port6 : out */ 
   if (ssGetOutputPortConnected(S,SF_NOUTPUT+6)) 
      for (i=1;i<=10;i++)
          y6[i-1] = MBSdata->user_IO->out[i]; 

   /* User output port7 : stop_simu */ 
   if (ssGetOutputPortConnected(S,SF_NOUTPUT+7)) 
      *y7 = MBSdata->user_IO->stop_simu; 
} 

#endif 
