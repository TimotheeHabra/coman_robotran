function [q,qd,qdd] = user_DrivenJoints(mbs_data,tsim)
% --------------------------
% UCL-CEREM-MBS
%
% @version MBsysLab_m 1.7.a
%
% Creation : 2005
% Last update : 30/09/2008
% -------------------------
%
%[q,qd,qdd] = user_DrivenJoints(mbs_data,tsim)
%
% mbs_data : multibody data structure
% tsim : current time
%
% q, qd, qdd : updated column vectors of generalized coordinates
%
%
% mbs_data.q : generalized coordinates [column vector]
% mbs_data.qd : generalized velocities [column vector]
% mbs_data.qdd : generalized accelerations [column vector]
% mbs_data.nqc : number of driven variables
% mbs_data.qc : indices of driven variables [column vector]

global MBS_user MBS_info

q   = mbs_data.q;
qd  = mbs_data.qd;
qdd = mbs_data.qdd;

%/*-- Begin of user code --*/

% i = ...
%     q(i) = f(tsim) ...
%     qd(i) = f'(tsim) ...
%     qdd(i) = f''(tsim) ...

%/*-- End of user code --*/

return