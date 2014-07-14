%--------------------------------------------------------------------------
%   Universite catholique de Louvain
%   CEREM : Centre for research in mechatronics
%   http://www.robotran.be  
%   Contact : robotran@prm.ucl.ac.be
%   Version : ROBOTRAN $Version$
%
%   MBsysLab main script template:
%      - featuring default options
%      - to be adapted by the user
%
%   Project : coman_robotran
%   Author : Team Robotran
%   Date : 04/06/2014 
%--------------------------------------------------------------------------

%% 1. Initialization and Project Loading [mbs_load]
%--------------------------------------------------------------------------
close all; clear variables; clc;                                            % Cleaning of the Matlab workspace
global MBS_user;                                                            % Declaration of the global user structure
MBS_user.process = '';                                                      % Initialisation of the user field "process"

% Project loading
prjname = 'coman_robotran';
[mbs_data, mbs_info] = mbs_load(prjname,'default');                         % Option 'default': automatic loading of "$project_name$.mbs" 
mbs_data_ini = mbs_data;                                                    % Backup of the initial multibody data structure
                                                                            % Have a look at the content of the mbs_data structure on www.robotran.be

%% 2. Coordinate partitioning [mbs_exe_part]             % For constrained MBS only
%--------------------------------------------------------------------------
MBS_user.process = 'part';

% driven joints
id_FJ_T2 = mbs_get_joint_id(mbs_info,'FJ_T2');
id_FJ_R1 = mbs_get_joint_id(mbs_info,'FJ_R1');
id_FJ_R3 = mbs_get_joint_id(mbs_info,'FJ_R3');

constraints_2D = 0;

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


%% 5. Direct dynamics [mbs_exe_dirdyn]
%--------------------------------------------------------------------------
MBS_user.process = 'dirdyn';
mbs_data = mbs_set_qu(mbs_data,mbs_data_ini.qu);                            % Retrieving of the initial set of independent variables

opt.dirdyn = {'time',0:0.01:5,'motion','simulation',...
    'odemethod','ode45','save2file','yes','framerate',1000,...
    'renamefile','no','verbose','yes'};
% other options : 'visualize', 'save2file', 'depinteg', 'dtmax', 'dtinit',
%                 'reltol', 'abstol', 'clearmbsglobal'                      % Help about options on www.robotran.be

[mbs_dirdyn,mbs_data] = mbs_exe_dirdyn(mbs_data,opt.dirdyn);                % Direct dynamics process (time simulation)

% Graphical Results
figure(1);
plot(mbs_dirdyn.tsim,mbs_dirdyn.q(:,3));                                    % Joint motion time history : joint n� 1 motion (example)
xlabel('time [s]');
ylabel('height [m]');
title('CoMan fall');


%% 8. Closing operations (optional)
%--------------------------------------------------------------------------
mbs_rm_allprjpath;                                                          % Cleaning of the Matlab project paths
mbs_del_glob('MBS_user','MBS_info','MBS_data');                             % Cleaning of the global MBS variables                                                                      % Cleaning of the Matlab command window
