//---------------------------
// Nicolas Van der Noot & Allan Barrea
//
// Creation : 29/10/2013
// Last update : 08/07/2014
//
// Impedance controller
//
//---------------------------

#include "simu_def.h"

/*
 * Impedance controller is used to get the voltage sent to the motors.
 * Currently, this impedance controller is different from the one implemented on the real CoMan.
 */
void impedance_controller(MBSdataStruct *MBSdata, Ctrl_Outputs *ovs) 
{
    // ---- Varibales declaration ---- //
    
    // user variables
    UserIOStruct *uvs;
    ControllerStruct *cvs;

    int robotran_id;
    
    double u_pos, ud_pos;
    double Qq;
    
    // gains
    double position_PID[3];
    double torque_PID[3];
    double K_p, D_p, P_t, I_t;
    
    double position_error;
    double last_err_tau;
    double last_int_err_tau;
    double last_tsim_int_tau;
    double delta_tsim;
    double new_err_tau;
    double new_int_err_tau;
    double tsim;
    
    int i;

    int robotran_id_table[] = {
        WAIST_YAW, //  1. TORSO_YAW
        WAIST_SAG, //  2. TORSO_PITCH
        WAIST_LAT, //  3. TORSO_ROLL
        
        R_HIP_SAG, //  4. RIGHT_HIP_PITCH
        L_HIP_SAG, //  5. LEFT_HIP_PITCH
        
        R_HIP_LAT,  //  6. RIGHT_HIP_ROLL
        R_HIP_YAW,  //  7. RIGHT_HIP_YAW
        R_KNEE_SAG, //  8. RIGHT_KNEE_PITCH
        R_ANK_SAG,  //  9. RIGHT_FOOT_PITCH
        R_ANK_LAT,  // 10. RIGHT_FOOT_ROLL
        
        L_HIP_LAT,  // 11. LEFT_HIP_ROLL
        L_HIP_YAW,  // 12. LEFT_HIP_YAW
        L_KNEE_SAG, // 13. LEFT_KNEE_PITCH
        L_ANK_SAG,  // 14. LEFT_FOOT_PITCH
        L_ANK_LAT,  // 15. LEFT_FOOT_ROLL
        
        R_SH_SAG, // 16. RIGHT_SHOULDER_PITCH
        R_SH_LAT, // 17. RIGHT_SHOULDER_ROLL
        R_SH_YAW, // 18. RIGHT_SHOULDER_YAW
        R_ELB,    // 19. RIGHT_ELBOW_PITCH
        
        L_SH_SAG, // 20. LEFT_SHOULDER_PITCH
        L_SH_LAT, // 21. LEFT_SHOULDER_ROLL
        L_SH_YAW, // 22. LEFT_SHOULDER_YAW
        L_ELB,    // 23. LEFT_ELBOW_PITCH

        #ifdef LONG_ARMS
        R_FORE_ARM_PLATE, // 24- RIGHT_FORE_ARM_PLATE
        R_WRJ1,           // 25. RIGHT_WRJ1
        R_WRJ2,           // 26. RIGHT_WRJ2

        L_FORE_ARM_PLATE, // 24- LEFT_FORE_ARM_PLATE
        L_WRJ1,           // 25. LEFT_WRJ1
        L_WRJ2,           // 26. LEFT_WRJ2
        #endif
    };

    // ---- Controller ---- //
    
    uvs = MBSdata->user_IO;
    cvs = uvs->cvs;
    
    tsim = MBSdata->tsim;
    last_tsim_int_tau = uvs->last_tsim_int_tau;
    delta_tsim = tsim - last_tsim_int_tau;
    
    for(i=0; i<COMAN_NB_JOINT_ACTUATED; i++) {

        robotran_id = robotran_id_table[i];
        
        // measured position, position velocity and torque
        u_pos  = MBSdata->ux[i+1];
        ud_pos = MBSdata->ux[COMAN_NB_JOINT_ACTUATED+i+1];
        Qq     = MBSdata->Qq[robotran_id];
        
        get_IC_gains(i+1, position_PID, torque_PID);

        K_p = (position_PID[0] / 1000.0);
        D_p = (position_PID[2] / 1000.0);
        P_t = (torque_PID[0] / 1000.0);
        I_t = (torque_PID[1] / 1000.0);

        /* 
        * Multiplying the gains by some 'magic factors' to make it work in simulation.
        * The results got on the real CoMan were better (very good position and reference tracking
        * on the real CoMan).
        */
        switch(ovs->imp_ctrl_index[i])
        {
            case 1: // position tracking
                K_p *= 50.0;
                D_p *= 15.0;
                P_t *= 1.0;
                I_t  = 0.0;
                break;

            case 2: // velocity tracking -> not implemented
                K_p = 0.0;
                D_p = 0.0;
                P_t = 0.0;
                I_t = 0.0;
                break;

            case 3: // torque tracking

                if (robotran_id == R_HIP_LAT || robotran_id == R_ANK_LAT ||
                 robotran_id == L_HIP_LAT || robotran_id == L_ANK_LAT ||
                 robotran_id == R_HIP_YAW || robotran_id == L_HIP_YAW)
                {
                    K_p = 0.0;
                    D_p = 0.0;
                    P_t *= 7.0;
                    I_t *= 8.0;
                }
                else
                {
                    K_p = 0.0;
                    D_p = 0.0;
                    P_t *= 18.0;
                    I_t *= 18.0;
                }
                break;

            default:
                break;
        }

        
        
        // ---- position error (with stiffness and damping) ---- //
        
        position_error = K_p * (ovs->q_ref[i] - u_pos) - D_p * ud_pos;
        
        // ---- PI controller (with position and torque control) ---- //
        
        // Proportional error
        new_err_tau = (position_error + ovs->Qq_ref[i]) - Qq;  

        
        // Integral error
        last_err_tau      = uvs->last_err_tau[i+1];
        last_int_err_tau  = uvs->last_int_err_tau[i+1];
        
        new_int_err_tau                    = last_int_err_tau + ((new_err_tau + last_err_tau)/2.0)*delta_tsim;
        uvs->last_err_tau[i+1]     = new_err_tau;
        uvs->last_int_err_tau[i+1] = new_int_err_tau;
        
        // PI controller
        uvs->Voltage[i+1] =  P_t * new_err_tau + I_t * new_int_err_tau;
    }
    
    uvs->last_tsim_int_tau = tsim;
    
    // limiting voltage
    for(i=1; i<=COMAN_NB_JOINT_ACTUATED; i++)
    {
        uvs->Voltage[i] = limit_function(uvs->Voltage[i], -MAX_ACT_VOLTAGE, MAX_ACT_VOLTAGE);
    }
}

// Impedance controller gains
void get_IC_gains(int id_coman, double position_PID[3], double torque_PID[3]) 
{    
    switch (id_coman) 
    {    
            /* ---- Sagittal Lower Body ---- */
            
        case 8: // R knee
            position_PID[0] = 200000.0;
            position_PID[1] = 0.0;
            position_PID[2] = 5000.0;
            torque_PID[0]   = 445.0;
            torque_PID[1]   = 22.0;
            torque_PID[2]   = 0.0;
            break;
            
            
        case 9: // R ankle pitch
            position_PID[0] = 400000.0;
            position_PID[1] = 0.0;
            position_PID[2] = 5000.0;
            torque_PID[0]   = 445.0;
            torque_PID[1]   = 22.0;
            torque_PID[2]   = 0.0;
            break;
            
        case 4: // R hip pitch
            position_PID[0] = 200000.0;
            position_PID[1] = 0.0;
            position_PID[2] = 5000.0;
            torque_PID[0]   = 445.0;
            torque_PID[1]   = 22.0;
            torque_PID[2]   = 0.0;
            break;
            
        case 5: // L hip pitch
            position_PID[0] = 200000.0;
            position_PID[1] = 0.0;
            position_PID[2] = 5000.0;
            torque_PID[0]   = 445.0;
            torque_PID[1]   = 22.0;
            torque_PID[2]   = 0.0;
            break;
            
        case 13: // L knee
            position_PID[0] = 200000.0;
            position_PID[1] = 0.0;
            position_PID[2] = 5000.0;
            torque_PID[0]   = 445.0;
            torque_PID[1]   = 22.0;
            torque_PID[2]   = 0.0;
            break;
            
        case 14: // L ankle pitch
            position_PID[0] = 200000.0;
            position_PID[1] = 0.0;
            position_PID[2] = 5000.0;
            torque_PID[0]   = 445.0;
            torque_PID[1]   = 22.0;
            torque_PID[2]   = 0.0;
            break;
            
            /* ---- Non Sagittal Lower Body ---- */
            
        case 10: // R ankle roll
            position_PID[0] = 200000.0;
            position_PID[1] = 0.0;
            position_PID[2] = 5000.0;
            torque_PID[0]   = 445.0;
            torque_PID[1]   = 22.0;
            torque_PID[2]   = 0.0;
            break;
            
        case 15: // L ankle roll
            position_PID[0] = 200000.0;
            position_PID[1] = 0.0;
            position_PID[2] = 5000.0;
            torque_PID[0]   = 445.0;
            torque_PID[1]   = 22.0;
            torque_PID[2]   = 0.0;
            break;
            
        case 6: // R hip roll
            position_PID[0] = 150000.0;
            position_PID[1] = 0.0;
            position_PID[2] = 5000.0;
            torque_PID[0]   = 445.0;
            torque_PID[1]   = 22.0;
            torque_PID[2]   = 0.0;
            break;
            
        case 11: // L hip roll
            position_PID[0] = 150000.0;
            position_PID[1] = 0.0;
            position_PID[2] = 5000.0;
            torque_PID[0]   = 445.0;
            torque_PID[1]   = 22.0;
            torque_PID[2]   = 0.0;
            break;
            
        case 7: // R hip yaw
            position_PID[0] = 60000.0;
            position_PID[1] = 0.0;
            position_PID[2] = 5000.0;
            torque_PID[0]   = 445.0;
            torque_PID[1]   = 22.0;
            torque_PID[2]   = 0.0;
            break;
            
        case 12: // L hip yaw
            position_PID[0] = 60000.0;
            position_PID[1] = 0.0;
            position_PID[2] = 5000.0;
            torque_PID[0]   = 445.0;
            torque_PID[1]   = 22.0;
            torque_PID[2]   = 0.0;
            break;
            
            /* ---- Waist ---- */
            
        case 1: // Waist yaw
            position_PID[0] = 150000.0;
            position_PID[1] = 0.0;
            position_PID[2] = 3000.0;
            torque_PID[0]   = 445.0;
            torque_PID[1]   = 22.0;
            torque_PID[2]   = 0.0;
            break;
            
        case 2: // Waist pitch -> not indicated !!!
            position_PID[0] = 150000.0;
            position_PID[1] = 0.0;
            position_PID[2] = 3000.0;
            torque_PID[0]   = 445.0;
            torque_PID[1]   = 22.0;
            torque_PID[2]   = 0.0;
            break;
            
        case 3: // Waist roll
            position_PID[0] = 150000.0;
            position_PID[1] = 0.0;
            position_PID[2] = 3000.0;
            torque_PID[0]   = 445.0;
            torque_PID[1]   = 22.0;
            torque_PID[2]   = 0.0;
            break;
            
            /* ---- Upper Body ---- */
            
        case 16: // R Shoulder pitch
            position_PID[0] = 3000.0;
            position_PID[1] = 0.0;
            position_PID[2] = 1000.0;
            torque_PID[0]   = 445.0;
            torque_PID[1]   = 22.0;
            torque_PID[2]   = 0.0;
            break;
            
        case 17: // R Shoulder roll
            position_PID[0] = 3000.0;
            position_PID[1] = 0.0;
            position_PID[2] = 1000.0;
            torque_PID[0]   = 445.0;
            torque_PID[1]   = 22.0;
            torque_PID[2]   = 0.0;
            break;
            
        case 18: // R Shoulder yaw
            position_PID[0] = 3000.0;
            position_PID[1] = 0.0;
            position_PID[2] = 1000.0;
            torque_PID[0]   = 445.0;
            torque_PID[1]   = 22.0;
            torque_PID[2]   = 0.0;
            break;
            
        case 19: // R Elbow pitch
            position_PID[0] = 3000.0;
            position_PID[1] = 0.0;
            position_PID[2] = 1000.0;
            torque_PID[0]   = 445.0;
            torque_PID[1]   = 22.0;
            torque_PID[2]   = 0.0;
            break;
            
        case 20: // L Shoulder pitch
            position_PID[0] = 3000.0;
            position_PID[1] = 0.0;
            position_PID[2] = 1000.0;
            torque_PID[0]   = 445.0;
            torque_PID[1]   = 22.0;
            torque_PID[2]   = 0.0;
            break;
            
        case 21: // L Shoulder roll
            position_PID[0] = 3000.0;
            position_PID[1] = 0.0;
            position_PID[2] = 1000.0;
            torque_PID[0]   = 445.0;
            torque_PID[1]   = 22.0;
            torque_PID[2]   = 0.0;
            break;
            
        case 22: // L Shoulder yaw
            position_PID[0] = 3000.0;
            position_PID[1] = 0.0;
            position_PID[2] = 1000.0;
            torque_PID[0]   = 445.0;
            torque_PID[1]   = 22.0;
            torque_PID[2]   = 0.0;
            break;
            
        case 23: // L Elbow
            position_PID[0] = 3000.0;
            position_PID[1] = 0.0;
            position_PID[2] = 1000.0;
            torque_PID[0]   = 445.0;
            torque_PID[1]   = 22.0;
            torque_PID[2]   = 0.0;
            break;
            
        default:
            position_PID[0] = 3000.0;
            position_PID[1] = 0.0;
            position_PID[2] = 1000.0;
            torque_PID[0]   = 445.0;
            torque_PID[1]   = 22.0;
            torque_PID[2]   = 0.0;
            break;
    }
}
