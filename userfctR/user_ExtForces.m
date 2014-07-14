function Swr = user_ExtForces(PxF,RxF,VxF,OMxF,AxF,OMPxF,mbs_data,tsim,ixF)
% --------------------------
% UCL-CEREM-MBS
%
% @version MBsysLab_m 1.7.a
%
% Creation : 2006
% Last update : 30/09/2008
% -------------------------
%
%Swr = user_ExtForces(PxF,RxF,VxF,OMxF,AxF,OMPxF,mbs_data,tsim,ixF)
%
% PxF(3,1) : absolute position vector of the external force application point 
% RxF(3,3) : absolute rotation matrix of the body
% VxF(3,1) : absolute velocity vector of the external force application point 
% OMxF(3,1) : absolute angular velocity vector of the body
% AxF(3,1) : absolute acceleration vector of the external force application point 
% OMPxF(3,1) : absolute angular acceleration vector of the body
%
% => All above vectors are expressed in the inertial reference frame !
%
% mbs_data : multibody data structure
% tsim : current time
% ixF : index of the external force sensor ('F' type in MBsysPad)
%        (can be obtained via the 'mbs_get_F_sensor_id' function)
%
% Swr(9,1) = [Fx; Fy; Fz; Mx; My; Mz; dxF];
%   - Force components (expressed in the inertial frame) : Fx, Fy, Fz
%   - Pure torque components (expressed in the inertial frame) : Mx, My, Mz
%   - Application point local coordinates vector (expressed in the body-fixed frame) : dxF(1:3,1);
%
% this function may use a global structure called MBS_user

global MBS_user MBS_info

Fx=0.0; Fy=0.0; Fz=0.0;
Mx=0.0; My=0.0; Mz=0.0;
idpt = mbs_data.xfidpt(ixF);
dxF = mbs_data.dpt(:,idpt);

%/*-- Begin of user code --*/
% 
% Use the 'mbs_get_F_sensor_id' function to get easily the force sensor
% indices, e.g. :
% F1 = mbs_get_F_sensor_id(MBS_info,'myFsensor_1');
% [F2,F3] = mbs_get_F_sensor_id(MBS_info,'myFsensor_2','myFsensor_3');
%

% switch(ixF)
%     case F1
%         ; % instructions for case 1
%     case F2
%         ; % instructions for case 2
%     case F3
%         ; % ...
% end

%/*-- End of user code --*/

Swr = [Fx; Fy; Fz; Mx; My; Mz; dxF];

return