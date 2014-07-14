%% Initialization
clc;
close all;
clear all;

%% Generate mbs_data and mbs_info

prjname = 'coman_robotran';      % project name
[mbs_data, mbs_info] = mbs_load(prjname,'default');

% put this flag to 1 if it corresponds to the compliant version
compliant_version = 1;

fprintf('\n');

%% Joints

% floating base
id_FJ_T1 = mbs_get_joint_id(mbs_info,'FJ_T1');
id_FJ_T2 = mbs_get_joint_id(mbs_info,'FJ_T2');
id_FJ_T3 = mbs_get_joint_id(mbs_info,'FJ_T3');
id_FJ_R1 = mbs_get_joint_id(mbs_info,'FJ_R1');
id_FJ_R2 = mbs_get_joint_id(mbs_info,'FJ_R2');
id_FJ_R3 = mbs_get_joint_id(mbs_info,'FJ_R3');

fprintf('// floating base\n');
fprintf('#define FJ_T1 %d\n', id_FJ_T1);
fprintf('#define FJ_T2 %d\n', id_FJ_T2);
fprintf('#define FJ_T3 %d\n', id_FJ_T3);
fprintf('#define FJ_R1 %d\n', id_FJ_R1);
fprintf('#define FJ_R2 %d\n', id_FJ_R2);
fprintf('#define FJ_R3 %d\n', id_FJ_R3);
fprintf('\n');

% right leg
id_RHipSag = mbs_get_joint_id(mbs_info,'RHipSag');
id_RHipLat = mbs_get_joint_id(mbs_info,'RHipLat');
id_RHipYaw = mbs_get_joint_id(mbs_info,'RHipYaw');
id_RKneeSag = mbs_get_joint_id(mbs_info,'RKneeSag');
id_RAnkLat = mbs_get_joint_id(mbs_info,'RAnkLat');
id_RAnkSag = mbs_get_joint_id(mbs_info,'RAnkSag');
if compliant_version
    id_RToe = mbs_get_joint_id(mbs_info,'RToe');
end

fprintf('// right leg\n');
fprintf('#define R_HIP_SAG %d\n', id_RHipSag);
fprintf('#define R_HIP_LAT %d\n', id_RHipLat);
fprintf('#define R_HIP_YAW %d\n', id_RHipYaw);
fprintf('#define R_KNEE_SAG %d\n', id_RKneeSag);
fprintf('#define R_ANK_LAT %d\n', id_RAnkLat);
fprintf('#define R_ANK_SAG %d\n', id_RAnkSag);
if compliant_version
    fprintf('#define R_TOE %d\n', id_RToe);
end
fprintf('\n');

% left leg
id_LHipSag = mbs_get_joint_id(mbs_info,'LHipSag');
id_LHipLat = mbs_get_joint_id(mbs_info,'LHipLat');
id_LHipYaw = mbs_get_joint_id(mbs_info,'LHipYaw');
id_LKneeSag = mbs_get_joint_id(mbs_info,'LKneeSag');
id_LAnkLat = mbs_get_joint_id(mbs_info,'LAnkLat');
id_LAnkSag = mbs_get_joint_id(mbs_info,'LAnkSag');
if compliant_version
    id_LToe = mbs_get_joint_id(mbs_info,'LToe');
end

fprintf('// right leg\n');
fprintf('#define L_HIP_SAG %d\n', id_LHipSag);
fprintf('#define L_HIP_LAT %d\n', id_LHipLat);
fprintf('#define L_HIP_YAW %d\n', id_LHipYaw);
fprintf('#define L_KNEE_SAG %d\n', id_LKneeSag);
fprintf('#define L_ANK_LAT %d\n', id_LAnkLat);
fprintf('#define L_ANK_SAG %d\n', id_LAnkSag);
if compliant_version
    fprintf('#define L_TOE %d\n', id_LToe);
end
fprintf('\n');

% waist
id_WaistLat = mbs_get_joint_id(mbs_info,'WaistLat');
id_WaistSag = mbs_get_joint_id(mbs_info,'WaistSag');
id_WaistYaw = mbs_get_joint_id(mbs_info,'WaistYaw');

fprintf('// waist\n');
fprintf('#define WAIST_LAT %d\n', id_WaistLat);
fprintf('#define WAIST_SAG %d\n', id_WaistSag);
fprintf('#define WAIST_YAW %d\n', id_WaistYaw);
fprintf('\n');

% right arm
id_RShSag = mbs_get_joint_id(mbs_info,'RShSag');
id_RShLat = mbs_get_joint_id(mbs_info,'RShLat');
id_RShYaw = mbs_get_joint_id(mbs_info,'RShYaw');
id_RElb = mbs_get_joint_id(mbs_info,'RElb');

fprintf('// right arm\n');
fprintf('#define R_SH_SAG %d\n', id_RShSag);
fprintf('#define R_SH_LAT %d\n', id_RShLat);
fprintf('#define R_SH_YAW %d\n', id_RShYaw);
fprintf('#define R_ELB %d\n', id_RElb);
fprintf('\n');

% left arm
id_LShSag = mbs_get_joint_id(mbs_info,'LShSag');
id_LShLat = mbs_get_joint_id(mbs_info,'LShLat');
id_LShYaw = mbs_get_joint_id(mbs_info,'LShYaw');
id_LElb = mbs_get_joint_id(mbs_info,'LElb');

fprintf('// left arm\n');
fprintf('#define L_SH_SAG %d\n', id_LShSag);
fprintf('#define L_SH_LAT %d\n', id_LShLat);
fprintf('#define L_SH_YAW %d\n', id_LShYaw);
fprintf('#define L_ELB %d\n', id_LElb);
fprintf('\n');

%% S sensors

id_S_MidWaist = mbs_get_S_sensor_id(mbs_info,'MidWaist');
id_S_RFoots = mbs_get_S_sensor_id(mbs_info,'RFoots');
id_S_LFoots = mbs_get_S_sensor_id(mbs_info,'LFoots');
if compliant_version
    id_S_RFoots_dist = mbs_get_S_sensor_id(mbs_info,'RFoots_dist');
    id_S_LFoots_dist = mbs_get_S_sensor_id(mbs_info,'LFoots_dist');
end

fprintf('// S sensors\n');
fprintf('#define S_MIDWAIST %d\n', id_S_MidWaist);
fprintf('#define S_RFOOTS %d\n', id_S_RFoots);
fprintf('#define S_LFOOTS %d\n', id_S_LFoots);
if compliant_version
    fprintf('#define S_RFOOTS_DIST %d\n', id_S_RFoots_dist);
    fprintf('#define S_LFOOTS_DIST %d\n', id_S_LFoots_dist);
end
fprintf('\n');

%% F sensors

id_F_RFoot_force = mbs_get_F_sensor_id(mbs_info,'RFoot_force');
id_F_LFoot_force = mbs_get_F_sensor_id(mbs_info,'LFoot_force');
if compliant_version
    id_F_RFoot_dist_force = mbs_get_F_sensor_id(mbs_info,'RFoot_dist_force');
    id_F_LFoot_dist_force = mbs_get_F_sensor_id(mbs_info,'LFoot_dist_force');
end
id_F_Force_Pert = mbs_get_F_sensor_id(mbs_info,'Force_Pert');

fprintf('// F sensors\n');
fprintf('#define RFOOT_FSENS_ID %d\n', id_F_RFoot_force);
fprintf('#define LFOOT_FSENS_ID %d\n', id_F_LFoot_force);
if compliant_version
    fprintf('#define RFOOT_DIST_FSENS_ID %d\n', id_F_RFoot_dist_force);
    fprintf('#define LFOOT_DIST_FSENS_ID %d\n', id_F_LFoot_dist_force);
end
fprintf('#define TORSO_PERT_FSENS_ID %d\n', id_F_Force_Pert);
fprintf('\n');



