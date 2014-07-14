function [ mbs_data ] = generate_mbs_data()

% Generates the mbs_data structure 
% used in 'call_simulink' and in the xml file generation

%% Project loading

global MBS_user;       % Declaration of the global user structure
MBS_user.process = ''; 

prjname = 'coman_robotran';      % project name
[mbs_data, mbs_info] = mbs_load(prjname,'default');

% constraints_2D = 1: T2, R1 and R3 joints of the floating base constraints
%                     -> 2D walking gaits
% constraints_2D = 0: no constraints
constraints_2D = 0;

%% 2. Coordinate partitioning [mbs_exe_part]             % For constrained MBS only
%--------------------------------------------------------------------------
MBS_user.process = 'part';

% driven joints
id_FJ_T2 = mbs_get_joint_id(mbs_info,'FJ_T2');
id_FJ_R1 = mbs_get_joint_id(mbs_info,'FJ_R1');
id_FJ_R3 = mbs_get_joint_id(mbs_info,'FJ_R3');

if constraints_2D
    mbs_data = mbs_set_qdriven(mbs_data,[id_FJ_T2 id_FJ_R1 id_FJ_R3]);
else
    mbs_data = mbs_set_qdriven(mbs_data,[]);
end

opt.part = {'rowperm','yes','threshold',1e-9,'verbose','yes'};
[mbs_part,mbs_data] = mbs_exe_part(mbs_data,opt.part);

% Coordinate partitioning results
disp('Coordinate partitioning results');
disp(['Sorted independent variables = ', mat2str(mbs_part.ind_u)]);
disp(['Permutated dependent variables = ', mat2str(mbs_part.ind_v)]);
disp(['Permutated independent constraints = ', mat2str(mbs_part.hu)]);
disp(['Redundant constraints = ', mat2str(mbs_part.hv)]);
disp(' ');

end

