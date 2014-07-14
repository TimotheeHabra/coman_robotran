//---------------------------
// Nicolas Van der Noot
//
// Creation : 19-Sep-2013
// Last update : 14-Jul-2014
//---------------------------

#ifndef ControllerStruct_h
#define ControllerStruct_h


// ---- Structures definitions (typedef) ---- //

// ControllerInputsStruc
typedef struct ControllerInputs
{
    double t;
    double q[29];
    double qd[29];
    double Qq[29];
    double F_Rfoot[3];
    double F_Lfoot[3];
    double T_Rfoot[3];
    double T_Lfoot[3];
    double IMU_Orientation[9];
    double IMU_Angular_Rate[3];

} ControllerInputs;


// ControllerOutputsStruc
typedef struct ControllerOutputs
{
    double q_ref[29];
    double qd_ref[29];
    double Qq_ref[29];
    int imp_ctrl_index[29];

} ControllerOutputs;


// ControllerStructStruc
typedef struct ControllerStruct
{
    struct ControllerInputs *Inputs;
    struct ControllerOutputs *Outputs;
    double out[20];
    double q_ref_r_sh_sag;
    double q_ref_r_sh_lat;
    double q_ref_r_sh_yaw;
    double q_ref_r_elb;
    double q_ref_l_sh_sag;
    double q_ref_l_sh_lat;
    double q_ref_l_sh_yaw;
    double q_ref_l_elb;

} ControllerStruct;


// ---- Init and free functions: declarations ---- //

ControllerInputs * init_ControllerInputs(void);
void free_ControllerInputs(ControllerInputs *cvs);

ControllerOutputs * init_ControllerOutputs(void);
void free_ControllerOutputs(ControllerOutputs *cvs);

ControllerStruct * init_ControllerStruct(void);
void free_ControllerStruct(ControllerStruct *cvs);

/*--------------------*/
#endif

