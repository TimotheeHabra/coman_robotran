function [simu_vars_none, simu_vars_in, simu_vars_out, simu_vars_struct] = simu_variables()

% Definition of the simulation variables and I/O ports (available in MBSdata->user_IO)
% For each line: varname , type , size
%
% 4 types of simulation variables:
%       . simu_vars_none   : normal variables
%       . simu_vars_in     : user inputs coming from the Matlab environment
%       . simu_vars_out    : user outputs to analyse results in Matlab
%       . simu_vars_struct : structures
%
% their corresponding type fields:
%       . none   = internal variable (type: 'int'/'double')
%       . in     = input (type: 'int'/'double')
%       . out    = output (type: 'int'/'double')
%       . struct = structure variable (type: structure name with '')
% 
%   - varname = name of the port or of the variable (with '')
%
%   - size: number of elements in the vector
%       1    :  simple variable
%       n    :  vector of n (n>1) elements
%       [m n]:  tabular of 2 entries with a size m*n 
%       0    :  forbidden
%       <1   :  pointer -> fabs(x) = number of stars
%          indexes start at 1 -> different from 'control_variables' 
%

simu_vars_none = {
    'Voltage','double',29
    'Control','double',29
    'Actuator_KKs','double',35
    'Actuator_DDs','double',35
    'Actuator_Jdrives','double',1
    'Actuator_Ddrives','double',1
    'Actuator_VTgain','double',1
    'actuated2real','int',29
    'joint_limits_min','double',35
    'joint_limits_max','double',35
    'waist_relative_ground','double',1
    'GRF_r','double',3
    'GRF_l','double',3
    'GRM_r','double',3
    'GRM_l','double',3
    'GRF_r_dist','double',3
    'GRF_l_dist','double',3
    'GRM_r_dist','double',3
    'GRM_l_dist','double',3
    'mu_grf','double',1
    'F_left_leg','double',1
    'F_right_leg','double',1
    'Msize_GCM','int',1
    'Msize_GCM_prox','int',1
    'Msize_GCM_dist','int',1
    'rn_left_x','double',200
    'rn_left_y','double',200
    'rn_left_z','double',200
    'rn_right_x','double',200
    'rn_right_y','double',200
    'rn_right_z','double',200
    'temp_grfx_left','double',200
    'temp_grfy_left','double',200
    'temp_grfx_right','double',200
    'temp_grfy_right','double',200
    'flag_grfx_left','int',200
    'flag_grfy_left','int',200
    'flag_grfx_right','int',200
    'flag_grfy_right','int',200
    'rn_left_prox_x','double',150
    'rn_left_prox_y','double',150
    'rn_left_prox_z','double',150
    'rn_right_prox_x','double',150
    'rn_right_prox_y','double',150
    'rn_right_prox_z','double',150
    'temp_grfx_left_prox','double',150
    'temp_grfy_left_prox','double',150
    'temp_grfx_right_prox','double',150
    'temp_grfy_right_prox','double',150
    'flag_grfx_left_prox','int',150
    'flag_grfy_left_prox','int',150
    'flag_grfx_right_prox','int',150
    'flag_grfy_right_prox','int',150
    'rn_left_dist_x','double',60
    'rn_left_dist_y','double',60
    'rn_left_dist_z','double',60
    'rn_right_dist_x','double',60
    'rn_right_dist_y','double',60
    'rn_right_dist_z','double',60
    'temp_grfx_left_dist','double',60
    'temp_grfy_left_dist','double',60
    'temp_grfx_right_dist','double',60
    'temp_grfy_right_dist','double',60
    'flag_grfx_left_dist','int',60
    'flag_grfy_left_dist','int',60
    'flag_grfx_right_dist','int',60
    'flag_grfy_right_dist','int',60
    'last_tsim_int_tau','double',1
    'last_err_tau','double',29
    'last_int_err_tau','double',29
    'real_theta_3_waist','double',1
    'real_omega_3_waist','double',1
    'last_t_ctrl','double',1
    'keyboard_command_1','int',1
    'keyboard_command_2','int',1
};

simu_vars_in = {
};

simu_vars_out = {
    'Qq_out','double',29
    'tsim_out','double',1
    'q_ref','double',29
    'qd_ref','double',29
    'Qq_ref','double',29
    'imp_ctrl_index','int',29
    'out','double',10
};

simu_vars_struct = {
    'cvs','ControllerStruct',1
};

end
