function [Qq] = user_JointForces(mbs_data,tsim);
% --------------------------
% UCL-CEREM-MBS
%
% @version MBsysLab_m 1.7.a
%
% Creation : 2006
% Last update : 30/09/2008
% -------------------------
%
%[Qq] = user_JointForces(mbs_data,tsim);
%
% mbs_data : multibody data structure
% tsim : current time
%
% Qq : joint generalized force/torque (for all joints)
% Qq(i) : joint force/torque in joint (i) along its joint axis
%
% this function may use a global structure called MBS_user

global MBS_user MBS_info

Qq = mbs_data.Qq;

%/*-- Begin of user code --*/

% Qq = ;

%/*-- End of user code --*/

return