function [controllers_struct] = control_variables()

% Definition of the control state variables 
%      (available in MBSdata->user_IO->cvs if you add the line ''cvs','ControllerStruct',1' in simu_variables.m)
%
% For each line: varname , type , size
%
%  - varname: name of the variable (with '')
%
%  - type: int / double / name of the structure
%
%  - size: number of elements in the vector
%       1    :  simple variable
%       n    :  vector of n (n>1) elements
%       [m n]:  tabular of 2 entries with a size m*n 
%       0    :  forbidden
%       <1   :  pointer -> fabs(x) = number of stars
% indexes start at 0 -> different from 'simu_variables'
%
% You can use recursive structures. 
% In this case, you must define sub-structures before their parent structure.
%

%% controller_1

controller_1_name = 'ControllerInputs';
controller_1_vars = {
    't','double',1
    'q','double',29
    'qd','double',29
    'Qq','double',29
    'q_mot','double',29
    'qd_mot','double',29
    'F_Rfoot','double',3
    'F_Lfoot','double',3
    'T_Rfoot','double',3
    'T_Lfoot','double',3
    'IMU_Orientation','double',9
    'IMU_Angular_Rate','double',3
    'IMU_Acceleration','double',3
};


%% controller_2

controller_2_name = 'ControllerOutputs';
controller_2_vars = {
    'q_ref','double',29
    'qd_ref','double',29
    'Qq_ref','double',29
    'imp_ctrl_index','int',29
};


%% controller_3

controller_3_name = 'ControllerStruct';
controller_3_vars = {
    'Inputs','ControllerInputs',1
    'Outputs','ControllerOutputs',1
    'out','double',20
    'q_ref_r_sh_sag','double',1
    'q_ref_r_sh_lat','double',1
    'q_ref_r_sh_yaw','double',1
    'q_ref_r_elb','double',1
    'q_ref_l_sh_sag','double',1
    'q_ref_l_sh_lat','double',1
    'q_ref_l_sh_yaw','double',1
    'q_ref_l_elb','double',1
};


%% controllers_vars

controllers_struct = {
    controller_1_name, controller_1_vars;
    controller_2_name, controller_2_vars;
    controller_3_name, controller_3_vars;
};

end
