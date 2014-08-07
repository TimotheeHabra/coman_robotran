%
%-------------------------------------------------------------
%
%	ROBOTRAN - Version 6.6 (build : february 22, 2008)
%
%	Copyright 
%	Universite catholique de Louvain 
%	Departement de Mecanique 
%	Unite de Production Mecanique et Machines 
%	2, Place du Levant 
%	1348 Louvain-la-Neuve 
%	http://www.robotran.be// 
%
%	==> Generation Date : Wed Aug  6 17:27:58 2014
%
%	==> Project name : coman_robotran
%	==> using XML input file 
%
%	==> Number of joints : 35
%
%	==> Function : F 6 : Sensors Kinematical Informations (sens) 
%	==> Flops complexity : 16660
%
%	==> All Parameter Symbols included
%	==> Generation Time :  0.340 seconds
%	==> Post-Processing :  0.280 seconds
%
%-------------------------------------------------------------
%
function [sens] = gensensor(s,tsim,usrfun,isens)

 sens.P = zeros(3,1);
 sens.R = zeros(3,3);
 sens.V = zeros(3,1);
 sens.OM = zeros(3,1);
 sens.A = zeros(3,1);
 sens.OMP = zeros(3,1);
 sens.J = zeros(6,35);

q = s.q; 
qd = s.qd; 
qdd = s.qdd; 
frc = s.frc; 
trq = s.trq; 

% === begin imp_aux === 

% === end imp_aux === 

% ===== BEGIN task 0 ===== 
 
% Sensor Kinematics 



% = = Block_0_0_0_0_0_1 = = 
 
% Trigonometric Variables  

  C4 = cos(q(4));
  S4 = sin(q(4));
  C5 = cos(q(5));
  S5 = sin(q(5));
  C6 = cos(q(6));
  S6 = sin(q(6));

% = = Block_0_0_0_0_0_2 = = 
 
% Trigonometric Variables  

  C7 = cos(q(7));
  S7 = sin(q(7));
  C8 = cos(q(8));
  S8 = sin(q(8));
  C9 = cos(q(9));
  S9 = sin(q(9));
  C10 = cos(q(10));
  S10 = sin(q(10));
  C11 = cos(q(11));
  S11 = sin(q(11));
  C12 = cos(q(12));
  S12 = sin(q(12));

% = = Block_0_0_0_0_0_3 = = 
 
% Trigonometric Variables  

  C13 = cos(q(13));
  S13 = sin(q(13));
  C14 = cos(q(14));
  S14 = sin(q(14));
  C15 = cos(q(15));
  S15 = sin(q(15));
  C16 = cos(q(16));
  S16 = sin(q(16));
  C17 = cos(q(17));
  S17 = sin(q(17));
  C18 = cos(q(18));
  S18 = sin(q(18));

% = = Block_0_0_0_0_0_4 = = 
 
% Trigonometric Variables  

  C19 = cos(q(19));
  S19 = sin(q(19));
  C20 = cos(q(20));
  S20 = sin(q(20));
  C21 = cos(q(21));
  S21 = sin(q(21));

% = = Block_0_0_0_0_0_5 = = 
 
% Trigonometric Variables  

  C22 = cos(q(22));
  S22 = sin(q(22));
  C23 = cos(q(23));
  S23 = sin(q(23));
  C24 = cos(q(24));
  S24 = sin(q(24));
  C25 = cos(q(25));
  S25 = sin(q(25));
  C26 = cos(q(26));
  S26 = sin(q(26));
  C27 = cos(q(27));
  S27 = sin(q(27));
  C28 = cos(q(28));
  S28 = sin(q(28));

% = = Block_0_0_0_0_0_6 = = 
 
% Trigonometric Variables  

  C29 = cos(q(29));
  S29 = sin(q(29));
  C30 = cos(q(30));
  S30 = sin(q(30));
  C31 = cos(q(31));
  S31 = sin(q(31));
  C32 = cos(q(32));
  S32 = sin(q(32));
  C33 = cos(q(33));
  S33 = sin(q(33));
  C34 = cos(q(34));
  S34 = sin(q(34));
  C35 = cos(q(35));
  S35 = sin(q(35));

% ====== END Task 0 ====== 

% ===== BEGIN task 1 ===== 
 
switch isens

 
% 
case 1, 


% = = Block_1_0_0_1_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = q(1);
    sens.R(1,1) = (1.0);
    sens.R(2,2) = (1.0);
    sens.R(3,3) = (1.0);
    sens.V(1) = qd(1);
    sens.A(1) = qdd(1);
 
% 
case 2, 


% = = Block_1_0_0_2_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = q(1);
    sens.P(2) = q(2);
    sens.R(1,1) = (1.0);
    sens.R(2,2) = (1.0);
    sens.R(3,3) = (1.0);
    sens.V(1) = qd(1);
    sens.V(2) = qd(2);
    sens.A(1) = qdd(1);
    sens.A(2) = qdd(2);
 
% 
case 3, 


% = = Block_1_0_0_3_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = q(1);
    sens.P(2) = q(2);
    sens.P(3) = q(3);
    sens.R(1,1) = (1.0);
    sens.R(2,2) = (1.0);
    sens.R(3,3) = (1.0);
    sens.V(1) = qd(1);
    sens.V(2) = qd(2);
    sens.V(3) = qd(3);
    sens.A(1) = qdd(1);
    sens.A(2) = qdd(2);
    sens.A(3) = qdd(3);
 
% 
case 4, 


% = = Block_1_0_0_4_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = q(1);
    sens.P(2) = q(2);
    sens.P(3) = q(3);
    sens.R(1,1) = (1.0);
    sens.R(2,2) = C4;
    sens.R(2,3) = S4;
    sens.R(3,2) = -S4;
    sens.R(3,3) = C4;
    sens.V(1) = qd(1);
    sens.V(2) = qd(2);
    sens.V(3) = qd(3);
    sens.OM(1) = qd(4);
    sens.A(1) = qdd(1);
    sens.A(2) = qdd(2);
    sens.A(3) = qdd(3);
    sens.OMP(1) = qdd(4);
 
% 
case 5, 


% = = Block_1_0_0_5_0_1 = = 
 
% Sensor Kinematics 


    ROcp4_25 = S4*S5;
    ROcp4_35 = -C4*S5;
    ROcp4_85 = -S4*C5;
    ROcp4_95 = C4*C5;
    OMcp4_25 = qd(5)*C4;
    OMcp4_35 = qd(5)*S4;
    OPcp4_25 = qdd(5)*C4-qd(4)*qd(5)*S4;
    OPcp4_35 = qdd(5)*S4+qd(4)*qd(5)*C4;

% = = Block_1_0_0_5_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = q(1);
    sens.P(2) = q(2);
    sens.P(3) = q(3);
    sens.R(1,1) = C5;
    sens.R(1,2) = ROcp4_25;
    sens.R(1,3) = ROcp4_35;
    sens.R(2,2) = C4;
    sens.R(2,3) = S4;
    sens.R(3,1) = S5;
    sens.R(3,2) = ROcp4_85;
    sens.R(3,3) = ROcp4_95;
    sens.V(1) = qd(1);
    sens.V(2) = qd(2);
    sens.V(3) = qd(3);
    sens.OM(1) = qd(4);
    sens.OM(2) = OMcp4_25;
    sens.OM(3) = OMcp4_35;
    sens.A(1) = qdd(1);
    sens.A(2) = qdd(2);
    sens.A(3) = qdd(3);
    sens.OMP(1) = qdd(4);
    sens.OMP(2) = OPcp4_25;
    sens.OMP(3) = OPcp4_35;
 
% 
case 6, 


% = = Block_1_0_0_6_0_1 = = 
 
% Sensor Kinematics 


    ROcp5_25 = S4*S5;
    ROcp5_35 = -C4*S5;
    ROcp5_85 = -S4*C5;
    ROcp5_95 = C4*C5;
    ROcp5_16 = C5*C6;
    ROcp5_26 = ROcp5_25*C6+C4*S6;
    ROcp5_36 = ROcp5_35*C6+S4*S6;
    ROcp5_46 = -C5*S6;
    ROcp5_56 = -(ROcp5_25*S6-C4*C6);
    ROcp5_66 = -(ROcp5_35*S6-S4*C6);
    OMcp5_25 = qd(5)*C4;
    OMcp5_35 = qd(5)*S4;
    OMcp5_16 = qd(4)+qd(6)*S5;
    OMcp5_26 = OMcp5_25+ROcp5_85*qd(6);
    OMcp5_36 = OMcp5_35+ROcp5_95*qd(6);
    OPcp5_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp5_26 = ROcp5_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp5_35*S5-ROcp5_95*qd(4));
    OPcp5_36 = ROcp5_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp5_25*S5-ROcp5_85*qd(4));

% = = Block_1_0_0_6_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = q(1);
    sens.P(2) = q(2);
    sens.P(3) = q(3);
    sens.R(1,1) = ROcp5_16;
    sens.R(1,2) = ROcp5_26;
    sens.R(1,3) = ROcp5_36;
    sens.R(2,1) = ROcp5_46;
    sens.R(2,2) = ROcp5_56;
    sens.R(2,3) = ROcp5_66;
    sens.R(3,1) = S5;
    sens.R(3,2) = ROcp5_85;
    sens.R(3,3) = ROcp5_95;
    sens.V(1) = qd(1);
    sens.V(2) = qd(2);
    sens.V(3) = qd(3);
    sens.OM(1) = OMcp5_16;
    sens.OM(2) = OMcp5_26;
    sens.OM(3) = OMcp5_36;
    sens.A(1) = qdd(1);
    sens.A(2) = qdd(2);
    sens.A(3) = qdd(3);
    sens.OMP(1) = OPcp5_16;
    sens.OMP(2) = OPcp5_26;
    sens.OMP(3) = OPcp5_36;
 
% 
case 7, 


% = = Block_1_0_0_7_0_1 = = 
 
% Sensor Kinematics 


    ROcp6_25 = S4*S5;
    ROcp6_35 = -C4*S5;
    ROcp6_85 = -S4*C5;
    ROcp6_95 = C4*C5;
    ROcp6_16 = C5*C6;
    ROcp6_26 = ROcp6_25*C6+C4*S6;
    ROcp6_36 = ROcp6_35*C6+S4*S6;
    ROcp6_46 = -C5*S6;
    ROcp6_56 = -(ROcp6_25*S6-C4*C6);
    ROcp6_66 = -(ROcp6_35*S6-S4*C6);
    OMcp6_25 = qd(5)*C4;
    OMcp6_35 = qd(5)*S4;
    OMcp6_16 = qd(4)+qd(6)*S5;
    OMcp6_26 = OMcp6_25+ROcp6_85*qd(6);
    OMcp6_36 = OMcp6_35+ROcp6_95*qd(6);
    OPcp6_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp6_26 = ROcp6_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp6_35*S5-ROcp6_95*qd(4));
    OPcp6_36 = ROcp6_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp6_25*S5-ROcp6_85*qd(4));

% = = Block_1_0_0_7_0_2 = = 
 
% Sensor Kinematics 


    ROcp6_17 = ROcp6_16*C7-S5*S7;
    ROcp6_27 = ROcp6_26*C7-ROcp6_85*S7;
    ROcp6_37 = ROcp6_36*C7-ROcp6_95*S7;
    ROcp6_77 = ROcp6_16*S7+S5*C7;
    ROcp6_87 = ROcp6_26*S7+ROcp6_85*C7;
    ROcp6_97 = ROcp6_36*S7+ROcp6_95*C7;
    RLcp6_17 = s.dpt(1,1)*ROcp6_16+s.dpt(3,1)*S5+ROcp6_46*s.dpt(2,1);
    RLcp6_27 = s.dpt(1,1)*ROcp6_26+s.dpt(3,1)*ROcp6_85+ROcp6_56*s.dpt(2,1);
    RLcp6_37 = s.dpt(1,1)*ROcp6_36+s.dpt(3,1)*ROcp6_95+ROcp6_66*s.dpt(2,1);
    POcp6_17 = RLcp6_17+q(1);
    POcp6_27 = RLcp6_27+q(2);
    POcp6_37 = RLcp6_37+q(3);
    OMcp6_17 = OMcp6_16+ROcp6_46*qd(7);
    OMcp6_27 = OMcp6_26+ROcp6_56*qd(7);
    OMcp6_37 = OMcp6_36+ROcp6_66*qd(7);
    ORcp6_17 = OMcp6_26*RLcp6_37-OMcp6_36*RLcp6_27;
    ORcp6_27 = -(OMcp6_16*RLcp6_37-OMcp6_36*RLcp6_17);
    ORcp6_37 = OMcp6_16*RLcp6_27-OMcp6_26*RLcp6_17;
    VIcp6_17 = ORcp6_17+qd(1);
    VIcp6_27 = ORcp6_27+qd(2);
    VIcp6_37 = ORcp6_37+qd(3);
    OPcp6_17 = OPcp6_16+ROcp6_46*qdd(7)+qd(7)*(OMcp6_26*ROcp6_66-OMcp6_36*ROcp6_56);
    OPcp6_27 = OPcp6_26+ROcp6_56*qdd(7)-qd(7)*(OMcp6_16*ROcp6_66-OMcp6_36*ROcp6_46);
    OPcp6_37 = OPcp6_36+ROcp6_66*qdd(7)+qd(7)*(OMcp6_16*ROcp6_56-OMcp6_26*ROcp6_46);
    ACcp6_17 = qdd(1)+OMcp6_26*ORcp6_37-OMcp6_36*ORcp6_27+OPcp6_26*RLcp6_37-OPcp6_36*RLcp6_27;
    ACcp6_27 = qdd(2)-OMcp6_16*ORcp6_37+OMcp6_36*ORcp6_17-OPcp6_16*RLcp6_37+OPcp6_36*RLcp6_17;
    ACcp6_37 = qdd(3)+OMcp6_16*ORcp6_27-OMcp6_26*ORcp6_17+OPcp6_16*RLcp6_27-OPcp6_26*RLcp6_17;

% = = Block_1_0_0_7_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp6_17;
    sens.P(2) = POcp6_27;
    sens.P(3) = POcp6_37;
    sens.R(1,1) = ROcp6_17;
    sens.R(1,2) = ROcp6_27;
    sens.R(1,3) = ROcp6_37;
    sens.R(2,1) = ROcp6_46;
    sens.R(2,2) = ROcp6_56;
    sens.R(2,3) = ROcp6_66;
    sens.R(3,1) = ROcp6_77;
    sens.R(3,2) = ROcp6_87;
    sens.R(3,3) = ROcp6_97;
    sens.V(1) = VIcp6_17;
    sens.V(2) = VIcp6_27;
    sens.V(3) = VIcp6_37;
    sens.OM(1) = OMcp6_17;
    sens.OM(2) = OMcp6_27;
    sens.OM(3) = OMcp6_37;
    sens.A(1) = ACcp6_17;
    sens.A(2) = ACcp6_27;
    sens.A(3) = ACcp6_37;
    sens.OMP(1) = OPcp6_17;
    sens.OMP(2) = OPcp6_27;
    sens.OMP(3) = OPcp6_37;
 
% 
case 8, 


% = = Block_1_0_0_8_0_1 = = 
 
% Sensor Kinematics 


    ROcp7_25 = S4*S5;
    ROcp7_35 = -C4*S5;
    ROcp7_85 = -S4*C5;
    ROcp7_95 = C4*C5;
    ROcp7_16 = C5*C6;
    ROcp7_26 = ROcp7_25*C6+C4*S6;
    ROcp7_36 = ROcp7_35*C6+S4*S6;
    ROcp7_46 = -C5*S6;
    ROcp7_56 = -(ROcp7_25*S6-C4*C6);
    ROcp7_66 = -(ROcp7_35*S6-S4*C6);
    OMcp7_25 = qd(5)*C4;
    OMcp7_35 = qd(5)*S4;
    OMcp7_16 = qd(4)+qd(6)*S5;
    OMcp7_26 = OMcp7_25+ROcp7_85*qd(6);
    OMcp7_36 = OMcp7_35+ROcp7_95*qd(6);
    OPcp7_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp7_26 = ROcp7_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp7_35*S5-ROcp7_95*qd(4));
    OPcp7_36 = ROcp7_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp7_25*S5-ROcp7_85*qd(4));

% = = Block_1_0_0_8_0_2 = = 
 
% Sensor Kinematics 


    ROcp7_17 = ROcp7_16*C7-S5*S7;
    ROcp7_27 = ROcp7_26*C7-ROcp7_85*S7;
    ROcp7_37 = ROcp7_36*C7-ROcp7_95*S7;
    ROcp7_77 = ROcp7_16*S7+S5*C7;
    ROcp7_87 = ROcp7_26*S7+ROcp7_85*C7;
    ROcp7_97 = ROcp7_36*S7+ROcp7_95*C7;
    ROcp7_48 = ROcp7_46*C8+ROcp7_77*S8;
    ROcp7_58 = ROcp7_56*C8+ROcp7_87*S8;
    ROcp7_68 = ROcp7_66*C8+ROcp7_97*S8;
    ROcp7_78 = -(ROcp7_46*S8-ROcp7_77*C8);
    ROcp7_88 = -(ROcp7_56*S8-ROcp7_87*C8);
    ROcp7_98 = -(ROcp7_66*S8-ROcp7_97*C8);
    RLcp7_17 = s.dpt(1,1)*ROcp7_16+s.dpt(3,1)*S5+ROcp7_46*s.dpt(2,1);
    RLcp7_27 = s.dpt(1,1)*ROcp7_26+s.dpt(3,1)*ROcp7_85+ROcp7_56*s.dpt(2,1);
    RLcp7_37 = s.dpt(1,1)*ROcp7_36+s.dpt(3,1)*ROcp7_95+ROcp7_66*s.dpt(2,1);
    OMcp7_17 = OMcp7_16+ROcp7_46*qd(7);
    OMcp7_27 = OMcp7_26+ROcp7_56*qd(7);
    OMcp7_37 = OMcp7_36+ROcp7_66*qd(7);
    ORcp7_17 = OMcp7_26*RLcp7_37-OMcp7_36*RLcp7_27;
    ORcp7_27 = -(OMcp7_16*RLcp7_37-OMcp7_36*RLcp7_17);
    ORcp7_37 = OMcp7_16*RLcp7_27-OMcp7_26*RLcp7_17;
    OPcp7_17 = OPcp7_16+ROcp7_46*qdd(7)+qd(7)*(OMcp7_26*ROcp7_66-OMcp7_36*ROcp7_56);
    OPcp7_27 = OPcp7_26+ROcp7_56*qdd(7)-qd(7)*(OMcp7_16*ROcp7_66-OMcp7_36*ROcp7_46);
    OPcp7_37 = OPcp7_36+ROcp7_66*qdd(7)+qd(7)*(OMcp7_16*ROcp7_56-OMcp7_26*ROcp7_46);
    RLcp7_18 = s.dpt(1,6)*ROcp7_17+s.dpt(3,6)*ROcp7_77+ROcp7_46*s.dpt(2,6);
    RLcp7_28 = s.dpt(1,6)*ROcp7_27+s.dpt(3,6)*ROcp7_87+ROcp7_56*s.dpt(2,6);
    RLcp7_38 = s.dpt(1,6)*ROcp7_37+s.dpt(3,6)*ROcp7_97+ROcp7_66*s.dpt(2,6);
    POcp7_18 = RLcp7_17+RLcp7_18+q(1);
    POcp7_28 = RLcp7_27+RLcp7_28+q(2);
    POcp7_38 = RLcp7_37+RLcp7_38+q(3);
    OMcp7_18 = OMcp7_17+ROcp7_17*qd(8);
    OMcp7_28 = OMcp7_27+ROcp7_27*qd(8);
    OMcp7_38 = OMcp7_37+ROcp7_37*qd(8);
    ORcp7_18 = OMcp7_27*RLcp7_38-OMcp7_37*RLcp7_28;
    ORcp7_28 = -(OMcp7_17*RLcp7_38-OMcp7_37*RLcp7_18);
    ORcp7_38 = OMcp7_17*RLcp7_28-OMcp7_27*RLcp7_18;
    VIcp7_18 = ORcp7_17+ORcp7_18+qd(1);
    VIcp7_28 = ORcp7_27+ORcp7_28+qd(2);
    VIcp7_38 = ORcp7_37+ORcp7_38+qd(3);
    OPcp7_18 = OPcp7_17+ROcp7_17*qdd(8)+qd(8)*(OMcp7_27*ROcp7_37-OMcp7_37*ROcp7_27);
    OPcp7_28 = OPcp7_27+ROcp7_27*qdd(8)-qd(8)*(OMcp7_17*ROcp7_37-OMcp7_37*ROcp7_17);
    OPcp7_38 = OPcp7_37+ROcp7_37*qdd(8)+qd(8)*(OMcp7_17*ROcp7_27-OMcp7_27*ROcp7_17);
    ACcp7_18 = qdd(1)+OMcp7_26*ORcp7_37+OMcp7_27*ORcp7_38-OMcp7_36*ORcp7_27-OMcp7_37*ORcp7_28+OPcp7_26*RLcp7_37+OPcp7_27*RLcp7_38-OPcp7_36*RLcp7_27...
 -OPcp7_37*RLcp7_28;
    ACcp7_28 = qdd(2)-OMcp7_16*ORcp7_37-OMcp7_17*ORcp7_38+OMcp7_36*ORcp7_17+OMcp7_37*ORcp7_18-OPcp7_16*RLcp7_37-OPcp7_17*RLcp7_38+OPcp7_36*RLcp7_17...
 +OPcp7_37*RLcp7_18;
    ACcp7_38 = qdd(3)+OMcp7_16*ORcp7_27+OMcp7_17*ORcp7_28-OMcp7_26*ORcp7_17-OMcp7_27*ORcp7_18+OPcp7_16*RLcp7_27+OPcp7_17*RLcp7_28-OPcp7_26*RLcp7_17...
 -OPcp7_27*RLcp7_18;

% = = Block_1_0_0_8_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp7_18;
    sens.P(2) = POcp7_28;
    sens.P(3) = POcp7_38;
    sens.R(1,1) = ROcp7_17;
    sens.R(1,2) = ROcp7_27;
    sens.R(1,3) = ROcp7_37;
    sens.R(2,1) = ROcp7_48;
    sens.R(2,2) = ROcp7_58;
    sens.R(2,3) = ROcp7_68;
    sens.R(3,1) = ROcp7_78;
    sens.R(3,2) = ROcp7_88;
    sens.R(3,3) = ROcp7_98;
    sens.V(1) = VIcp7_18;
    sens.V(2) = VIcp7_28;
    sens.V(3) = VIcp7_38;
    sens.OM(1) = OMcp7_18;
    sens.OM(2) = OMcp7_28;
    sens.OM(3) = OMcp7_38;
    sens.A(1) = ACcp7_18;
    sens.A(2) = ACcp7_28;
    sens.A(3) = ACcp7_38;
    sens.OMP(1) = OPcp7_18;
    sens.OMP(2) = OPcp7_28;
    sens.OMP(3) = OPcp7_38;
 
% 
case 9, 


% = = Block_1_0_0_9_0_1 = = 
 
% Sensor Kinematics 


    ROcp8_25 = S4*S5;
    ROcp8_35 = -C4*S5;
    ROcp8_85 = -S4*C5;
    ROcp8_95 = C4*C5;
    ROcp8_16 = C5*C6;
    ROcp8_26 = ROcp8_25*C6+C4*S6;
    ROcp8_36 = ROcp8_35*C6+S4*S6;
    ROcp8_46 = -C5*S6;
    ROcp8_56 = -(ROcp8_25*S6-C4*C6);
    ROcp8_66 = -(ROcp8_35*S6-S4*C6);
    OMcp8_25 = qd(5)*C4;
    OMcp8_35 = qd(5)*S4;
    OMcp8_16 = qd(4)+qd(6)*S5;
    OMcp8_26 = OMcp8_25+ROcp8_85*qd(6);
    OMcp8_36 = OMcp8_35+ROcp8_95*qd(6);
    OPcp8_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp8_26 = ROcp8_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp8_35*S5-ROcp8_95*qd(4));
    OPcp8_36 = ROcp8_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp8_25*S5-ROcp8_85*qd(4));

% = = Block_1_0_0_9_0_2 = = 
 
% Sensor Kinematics 


    ROcp8_17 = ROcp8_16*C7-S5*S7;
    ROcp8_27 = ROcp8_26*C7-ROcp8_85*S7;
    ROcp8_37 = ROcp8_36*C7-ROcp8_95*S7;
    ROcp8_77 = ROcp8_16*S7+S5*C7;
    ROcp8_87 = ROcp8_26*S7+ROcp8_85*C7;
    ROcp8_97 = ROcp8_36*S7+ROcp8_95*C7;
    ROcp8_48 = ROcp8_46*C8+ROcp8_77*S8;
    ROcp8_58 = ROcp8_56*C8+ROcp8_87*S8;
    ROcp8_68 = ROcp8_66*C8+ROcp8_97*S8;
    ROcp8_78 = -(ROcp8_46*S8-ROcp8_77*C8);
    ROcp8_88 = -(ROcp8_56*S8-ROcp8_87*C8);
    ROcp8_98 = -(ROcp8_66*S8-ROcp8_97*C8);
    ROcp8_19 = ROcp8_17*C9+ROcp8_48*S9;
    ROcp8_29 = ROcp8_27*C9+ROcp8_58*S9;
    ROcp8_39 = ROcp8_37*C9+ROcp8_68*S9;
    ROcp8_49 = -(ROcp8_17*S9-ROcp8_48*C9);
    ROcp8_59 = -(ROcp8_27*S9-ROcp8_58*C9);
    ROcp8_69 = -(ROcp8_37*S9-ROcp8_68*C9);
    RLcp8_17 = s.dpt(1,1)*ROcp8_16+s.dpt(3,1)*S5+ROcp8_46*s.dpt(2,1);
    RLcp8_27 = s.dpt(1,1)*ROcp8_26+s.dpt(3,1)*ROcp8_85+ROcp8_56*s.dpt(2,1);
    RLcp8_37 = s.dpt(1,1)*ROcp8_36+s.dpt(3,1)*ROcp8_95+ROcp8_66*s.dpt(2,1);
    OMcp8_17 = OMcp8_16+ROcp8_46*qd(7);
    OMcp8_27 = OMcp8_26+ROcp8_56*qd(7);
    OMcp8_37 = OMcp8_36+ROcp8_66*qd(7);
    ORcp8_17 = OMcp8_26*RLcp8_37-OMcp8_36*RLcp8_27;
    ORcp8_27 = -(OMcp8_16*RLcp8_37-OMcp8_36*RLcp8_17);
    ORcp8_37 = OMcp8_16*RLcp8_27-OMcp8_26*RLcp8_17;
    OPcp8_17 = OPcp8_16+ROcp8_46*qdd(7)+qd(7)*(OMcp8_26*ROcp8_66-OMcp8_36*ROcp8_56);
    OPcp8_27 = OPcp8_26+ROcp8_56*qdd(7)-qd(7)*(OMcp8_16*ROcp8_66-OMcp8_36*ROcp8_46);
    OPcp8_37 = OPcp8_36+ROcp8_66*qdd(7)+qd(7)*(OMcp8_16*ROcp8_56-OMcp8_26*ROcp8_46);
    RLcp8_18 = s.dpt(1,6)*ROcp8_17+s.dpt(3,6)*ROcp8_77+ROcp8_46*s.dpt(2,6);
    RLcp8_28 = s.dpt(1,6)*ROcp8_27+s.dpt(3,6)*ROcp8_87+ROcp8_56*s.dpt(2,6);
    RLcp8_38 = s.dpt(1,6)*ROcp8_37+s.dpt(3,6)*ROcp8_97+ROcp8_66*s.dpt(2,6);
    OMcp8_18 = OMcp8_17+ROcp8_17*qd(8);
    OMcp8_28 = OMcp8_27+ROcp8_27*qd(8);
    OMcp8_38 = OMcp8_37+ROcp8_37*qd(8);
    ORcp8_18 = OMcp8_27*RLcp8_38-OMcp8_37*RLcp8_28;
    ORcp8_28 = -(OMcp8_17*RLcp8_38-OMcp8_37*RLcp8_18);
    ORcp8_38 = OMcp8_17*RLcp8_28-OMcp8_27*RLcp8_18;
    OPcp8_18 = OPcp8_17+ROcp8_17*qdd(8)+qd(8)*(OMcp8_27*ROcp8_37-OMcp8_37*ROcp8_27);
    OPcp8_28 = OPcp8_27+ROcp8_27*qdd(8)-qd(8)*(OMcp8_17*ROcp8_37-OMcp8_37*ROcp8_17);
    OPcp8_38 = OPcp8_37+ROcp8_37*qdd(8)+qd(8)*(OMcp8_17*ROcp8_27-OMcp8_27*ROcp8_17);
    RLcp8_19 = s.dpt(1,8)*ROcp8_17+s.dpt(2,8)*ROcp8_48+ROcp8_78*s.dpt(3,8);
    RLcp8_29 = s.dpt(1,8)*ROcp8_27+s.dpt(2,8)*ROcp8_58+ROcp8_88*s.dpt(3,8);
    RLcp8_39 = s.dpt(1,8)*ROcp8_37+s.dpt(2,8)*ROcp8_68+ROcp8_98*s.dpt(3,8);
    POcp8_19 = RLcp8_17+RLcp8_18+RLcp8_19+q(1);
    POcp8_29 = RLcp8_27+RLcp8_28+RLcp8_29+q(2);
    POcp8_39 = RLcp8_37+RLcp8_38+RLcp8_39+q(3);
    OMcp8_19 = OMcp8_18+ROcp8_78*qd(9);
    OMcp8_29 = OMcp8_28+ROcp8_88*qd(9);
    OMcp8_39 = OMcp8_38+ROcp8_98*qd(9);
    ORcp8_19 = OMcp8_28*RLcp8_39-OMcp8_38*RLcp8_29;
    ORcp8_29 = -(OMcp8_18*RLcp8_39-OMcp8_38*RLcp8_19);
    ORcp8_39 = OMcp8_18*RLcp8_29-OMcp8_28*RLcp8_19;
    VIcp8_19 = ORcp8_17+ORcp8_18+ORcp8_19+qd(1);
    VIcp8_29 = ORcp8_27+ORcp8_28+ORcp8_29+qd(2);
    VIcp8_39 = ORcp8_37+ORcp8_38+ORcp8_39+qd(3);
    OPcp8_19 = OPcp8_18+ROcp8_78*qdd(9)+qd(9)*(OMcp8_28*ROcp8_98-OMcp8_38*ROcp8_88);
    OPcp8_29 = OPcp8_28+ROcp8_88*qdd(9)-qd(9)*(OMcp8_18*ROcp8_98-OMcp8_38*ROcp8_78);
    OPcp8_39 = OPcp8_38+ROcp8_98*qdd(9)+qd(9)*(OMcp8_18*ROcp8_88-OMcp8_28*ROcp8_78);
    ACcp8_19 = qdd(1)+OMcp8_26*ORcp8_37+OMcp8_27*ORcp8_38+OMcp8_28*ORcp8_39-OMcp8_36*ORcp8_27-OMcp8_37*ORcp8_28-OMcp8_38*ORcp8_29+OPcp8_26*RLcp8_37...
 +OPcp8_27*RLcp8_38+OPcp8_28*RLcp8_39-OPcp8_36*RLcp8_27-OPcp8_37*RLcp8_28-OPcp8_38*RLcp8_29;
    ACcp8_29 = qdd(2)-OMcp8_16*ORcp8_37-OMcp8_17*ORcp8_38-OMcp8_18*ORcp8_39+OMcp8_36*ORcp8_17+OMcp8_37*ORcp8_18+OMcp8_38*ORcp8_19-OPcp8_16*RLcp8_37...
 -OPcp8_17*RLcp8_38-OPcp8_18*RLcp8_39+OPcp8_36*RLcp8_17+OPcp8_37*RLcp8_18+OPcp8_38*RLcp8_19;
    ACcp8_39 = qdd(3)+OMcp8_16*ORcp8_27+OMcp8_17*ORcp8_28+OMcp8_18*ORcp8_29-OMcp8_26*ORcp8_17-OMcp8_27*ORcp8_18-OMcp8_28*ORcp8_19+OPcp8_16*RLcp8_27...
 +OPcp8_17*RLcp8_28+OPcp8_18*RLcp8_29-OPcp8_26*RLcp8_17-OPcp8_27*RLcp8_18-OPcp8_28*RLcp8_19;

% = = Block_1_0_0_9_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp8_19;
    sens.P(2) = POcp8_29;
    sens.P(3) = POcp8_39;
    sens.R(1,1) = ROcp8_19;
    sens.R(1,2) = ROcp8_29;
    sens.R(1,3) = ROcp8_39;
    sens.R(2,1) = ROcp8_49;
    sens.R(2,2) = ROcp8_59;
    sens.R(2,3) = ROcp8_69;
    sens.R(3,1) = ROcp8_78;
    sens.R(3,2) = ROcp8_88;
    sens.R(3,3) = ROcp8_98;
    sens.V(1) = VIcp8_19;
    sens.V(2) = VIcp8_29;
    sens.V(3) = VIcp8_39;
    sens.OM(1) = OMcp8_19;
    sens.OM(2) = OMcp8_29;
    sens.OM(3) = OMcp8_39;
    sens.A(1) = ACcp8_19;
    sens.A(2) = ACcp8_29;
    sens.A(3) = ACcp8_39;
    sens.OMP(1) = OPcp8_19;
    sens.OMP(2) = OPcp8_29;
    sens.OMP(3) = OPcp8_39;
 
% 
case 10, 


% = = Block_1_0_0_10_0_1 = = 
 
% Sensor Kinematics 


    ROcp9_25 = S4*S5;
    ROcp9_35 = -C4*S5;
    ROcp9_85 = -S4*C5;
    ROcp9_95 = C4*C5;
    ROcp9_16 = C5*C6;
    ROcp9_26 = ROcp9_25*C6+C4*S6;
    ROcp9_36 = ROcp9_35*C6+S4*S6;
    ROcp9_46 = -C5*S6;
    ROcp9_56 = -(ROcp9_25*S6-C4*C6);
    ROcp9_66 = -(ROcp9_35*S6-S4*C6);
    OMcp9_25 = qd(5)*C4;
    OMcp9_35 = qd(5)*S4;
    OMcp9_16 = qd(4)+qd(6)*S5;
    OMcp9_26 = OMcp9_25+ROcp9_85*qd(6);
    OMcp9_36 = OMcp9_35+ROcp9_95*qd(6);
    OPcp9_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp9_26 = ROcp9_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp9_35*S5-ROcp9_95*qd(4));
    OPcp9_36 = ROcp9_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp9_25*S5-ROcp9_85*qd(4));

% = = Block_1_0_0_10_0_2 = = 
 
% Sensor Kinematics 


    ROcp9_17 = ROcp9_16*C7-S5*S7;
    ROcp9_27 = ROcp9_26*C7-ROcp9_85*S7;
    ROcp9_37 = ROcp9_36*C7-ROcp9_95*S7;
    ROcp9_77 = ROcp9_16*S7+S5*C7;
    ROcp9_87 = ROcp9_26*S7+ROcp9_85*C7;
    ROcp9_97 = ROcp9_36*S7+ROcp9_95*C7;
    ROcp9_48 = ROcp9_46*C8+ROcp9_77*S8;
    ROcp9_58 = ROcp9_56*C8+ROcp9_87*S8;
    ROcp9_68 = ROcp9_66*C8+ROcp9_97*S8;
    ROcp9_78 = -(ROcp9_46*S8-ROcp9_77*C8);
    ROcp9_88 = -(ROcp9_56*S8-ROcp9_87*C8);
    ROcp9_98 = -(ROcp9_66*S8-ROcp9_97*C8);
    ROcp9_19 = ROcp9_17*C9+ROcp9_48*S9;
    ROcp9_29 = ROcp9_27*C9+ROcp9_58*S9;
    ROcp9_39 = ROcp9_37*C9+ROcp9_68*S9;
    ROcp9_49 = -(ROcp9_17*S9-ROcp9_48*C9);
    ROcp9_59 = -(ROcp9_27*S9-ROcp9_58*C9);
    ROcp9_69 = -(ROcp9_37*S9-ROcp9_68*C9);
    ROcp9_110 = ROcp9_19*C10-ROcp9_78*S10;
    ROcp9_210 = ROcp9_29*C10-ROcp9_88*S10;
    ROcp9_310 = ROcp9_39*C10-ROcp9_98*S10;
    ROcp9_710 = ROcp9_19*S10+ROcp9_78*C10;
    ROcp9_810 = ROcp9_29*S10+ROcp9_88*C10;
    ROcp9_910 = ROcp9_39*S10+ROcp9_98*C10;
    RLcp9_17 = s.dpt(1,1)*ROcp9_16+s.dpt(3,1)*S5+ROcp9_46*s.dpt(2,1);
    RLcp9_27 = s.dpt(1,1)*ROcp9_26+s.dpt(3,1)*ROcp9_85+ROcp9_56*s.dpt(2,1);
    RLcp9_37 = s.dpt(1,1)*ROcp9_36+s.dpt(3,1)*ROcp9_95+ROcp9_66*s.dpt(2,1);
    OMcp9_17 = OMcp9_16+ROcp9_46*qd(7);
    OMcp9_27 = OMcp9_26+ROcp9_56*qd(7);
    OMcp9_37 = OMcp9_36+ROcp9_66*qd(7);
    ORcp9_17 = OMcp9_26*RLcp9_37-OMcp9_36*RLcp9_27;
    ORcp9_27 = -(OMcp9_16*RLcp9_37-OMcp9_36*RLcp9_17);
    ORcp9_37 = OMcp9_16*RLcp9_27-OMcp9_26*RLcp9_17;
    OPcp9_17 = OPcp9_16+ROcp9_46*qdd(7)+qd(7)*(OMcp9_26*ROcp9_66-OMcp9_36*ROcp9_56);
    OPcp9_27 = OPcp9_26+ROcp9_56*qdd(7)-qd(7)*(OMcp9_16*ROcp9_66-OMcp9_36*ROcp9_46);
    OPcp9_37 = OPcp9_36+ROcp9_66*qdd(7)+qd(7)*(OMcp9_16*ROcp9_56-OMcp9_26*ROcp9_46);
    RLcp9_18 = s.dpt(1,6)*ROcp9_17+s.dpt(3,6)*ROcp9_77+ROcp9_46*s.dpt(2,6);
    RLcp9_28 = s.dpt(1,6)*ROcp9_27+s.dpt(3,6)*ROcp9_87+ROcp9_56*s.dpt(2,6);
    RLcp9_38 = s.dpt(1,6)*ROcp9_37+s.dpt(3,6)*ROcp9_97+ROcp9_66*s.dpt(2,6);
    OMcp9_18 = OMcp9_17+ROcp9_17*qd(8);
    OMcp9_28 = OMcp9_27+ROcp9_27*qd(8);
    OMcp9_38 = OMcp9_37+ROcp9_37*qd(8);
    ORcp9_18 = OMcp9_27*RLcp9_38-OMcp9_37*RLcp9_28;
    ORcp9_28 = -(OMcp9_17*RLcp9_38-OMcp9_37*RLcp9_18);
    ORcp9_38 = OMcp9_17*RLcp9_28-OMcp9_27*RLcp9_18;
    OPcp9_18 = OPcp9_17+ROcp9_17*qdd(8)+qd(8)*(OMcp9_27*ROcp9_37-OMcp9_37*ROcp9_27);
    OPcp9_28 = OPcp9_27+ROcp9_27*qdd(8)-qd(8)*(OMcp9_17*ROcp9_37-OMcp9_37*ROcp9_17);
    OPcp9_38 = OPcp9_37+ROcp9_37*qdd(8)+qd(8)*(OMcp9_17*ROcp9_27-OMcp9_27*ROcp9_17);
    RLcp9_19 = s.dpt(1,8)*ROcp9_17+s.dpt(2,8)*ROcp9_48+ROcp9_78*s.dpt(3,8);
    RLcp9_29 = s.dpt(1,8)*ROcp9_27+s.dpt(2,8)*ROcp9_58+ROcp9_88*s.dpt(3,8);
    RLcp9_39 = s.dpt(1,8)*ROcp9_37+s.dpt(2,8)*ROcp9_68+ROcp9_98*s.dpt(3,8);
    OMcp9_19 = OMcp9_18+ROcp9_78*qd(9);
    OMcp9_29 = OMcp9_28+ROcp9_88*qd(9);
    OMcp9_39 = OMcp9_38+ROcp9_98*qd(9);
    ORcp9_19 = OMcp9_28*RLcp9_39-OMcp9_38*RLcp9_29;
    ORcp9_29 = -(OMcp9_18*RLcp9_39-OMcp9_38*RLcp9_19);
    ORcp9_39 = OMcp9_18*RLcp9_29-OMcp9_28*RLcp9_19;
    OPcp9_19 = OPcp9_18+ROcp9_78*qdd(9)+qd(9)*(OMcp9_28*ROcp9_98-OMcp9_38*ROcp9_88);
    OPcp9_29 = OPcp9_28+ROcp9_88*qdd(9)-qd(9)*(OMcp9_18*ROcp9_98-OMcp9_38*ROcp9_78);
    OPcp9_39 = OPcp9_38+ROcp9_98*qdd(9)+qd(9)*(OMcp9_18*ROcp9_88-OMcp9_28*ROcp9_78);
    RLcp9_110 = s.dpt(1,10)*ROcp9_19+s.dpt(2,10)*ROcp9_49+ROcp9_78*s.dpt(3,10);
    RLcp9_210 = s.dpt(1,10)*ROcp9_29+s.dpt(2,10)*ROcp9_59+ROcp9_88*s.dpt(3,10);
    RLcp9_310 = s.dpt(1,10)*ROcp9_39+s.dpt(2,10)*ROcp9_69+ROcp9_98*s.dpt(3,10);
    POcp9_110 = RLcp9_110+RLcp9_17+RLcp9_18+RLcp9_19+q(1);
    POcp9_210 = RLcp9_210+RLcp9_27+RLcp9_28+RLcp9_29+q(2);
    POcp9_310 = RLcp9_310+RLcp9_37+RLcp9_38+RLcp9_39+q(3);
    OMcp9_110 = OMcp9_19+ROcp9_49*qd(10);
    OMcp9_210 = OMcp9_29+ROcp9_59*qd(10);
    OMcp9_310 = OMcp9_39+ROcp9_69*qd(10);
    ORcp9_110 = OMcp9_29*RLcp9_310-OMcp9_39*RLcp9_210;
    ORcp9_210 = -(OMcp9_19*RLcp9_310-OMcp9_39*RLcp9_110);
    ORcp9_310 = OMcp9_19*RLcp9_210-OMcp9_29*RLcp9_110;
    VIcp9_110 = ORcp9_110+ORcp9_17+ORcp9_18+ORcp9_19+qd(1);
    VIcp9_210 = ORcp9_210+ORcp9_27+ORcp9_28+ORcp9_29+qd(2);
    VIcp9_310 = ORcp9_310+ORcp9_37+ORcp9_38+ORcp9_39+qd(3);
    OPcp9_110 = OPcp9_19+ROcp9_49*qdd(10)+qd(10)*(OMcp9_29*ROcp9_69-OMcp9_39*ROcp9_59);
    OPcp9_210 = OPcp9_29+ROcp9_59*qdd(10)-qd(10)*(OMcp9_19*ROcp9_69-OMcp9_39*ROcp9_49);
    OPcp9_310 = OPcp9_39+ROcp9_69*qdd(10)+qd(10)*(OMcp9_19*ROcp9_59-OMcp9_29*ROcp9_49);
    ACcp9_110 = qdd(1)+OMcp9_26*ORcp9_37+OMcp9_27*ORcp9_38+OMcp9_28*ORcp9_39+OMcp9_29*ORcp9_310-OMcp9_36*ORcp9_27-OMcp9_37*ORcp9_28-OMcp9_38*...
 ORcp9_29-OMcp9_39*ORcp9_210+OPcp9_26*RLcp9_37+OPcp9_27*RLcp9_38+OPcp9_28*RLcp9_39+OPcp9_29*RLcp9_310-OPcp9_36*RLcp9_27-OPcp9_37*RLcp9_28-OPcp9_38*...
 RLcp9_29-OPcp9_39*RLcp9_210;
    ACcp9_210 = qdd(2)-OMcp9_16*ORcp9_37-OMcp9_17*ORcp9_38-OMcp9_18*ORcp9_39-OMcp9_19*ORcp9_310+OMcp9_36*ORcp9_17+OMcp9_37*ORcp9_18+OMcp9_38*...
 ORcp9_19+OMcp9_39*ORcp9_110-OPcp9_16*RLcp9_37-OPcp9_17*RLcp9_38-OPcp9_18*RLcp9_39-OPcp9_19*RLcp9_310+OPcp9_36*RLcp9_17+OPcp9_37*RLcp9_18+OPcp9_38*...
 RLcp9_19+OPcp9_39*RLcp9_110;
    ACcp9_310 = qdd(3)+OMcp9_16*ORcp9_27+OMcp9_17*ORcp9_28+OMcp9_18*ORcp9_29+OMcp9_19*ORcp9_210-OMcp9_26*ORcp9_17-OMcp9_27*ORcp9_18-OMcp9_28*...
 ORcp9_19-OMcp9_29*ORcp9_110+OPcp9_16*RLcp9_27+OPcp9_17*RLcp9_28+OPcp9_18*RLcp9_29+OPcp9_19*RLcp9_210-OPcp9_26*RLcp9_17-OPcp9_27*RLcp9_18-OPcp9_28*...
 RLcp9_19-OPcp9_29*RLcp9_110;

% = = Block_1_0_0_10_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp9_110;
    sens.P(2) = POcp9_210;
    sens.P(3) = POcp9_310;
    sens.R(1,1) = ROcp9_110;
    sens.R(1,2) = ROcp9_210;
    sens.R(1,3) = ROcp9_310;
    sens.R(2,1) = ROcp9_49;
    sens.R(2,2) = ROcp9_59;
    sens.R(2,3) = ROcp9_69;
    sens.R(3,1) = ROcp9_710;
    sens.R(3,2) = ROcp9_810;
    sens.R(3,3) = ROcp9_910;
    sens.V(1) = VIcp9_110;
    sens.V(2) = VIcp9_210;
    sens.V(3) = VIcp9_310;
    sens.OM(1) = OMcp9_110;
    sens.OM(2) = OMcp9_210;
    sens.OM(3) = OMcp9_310;
    sens.A(1) = ACcp9_110;
    sens.A(2) = ACcp9_210;
    sens.A(3) = ACcp9_310;
    sens.OMP(1) = OPcp9_110;
    sens.OMP(2) = OPcp9_210;
    sens.OMP(3) = OPcp9_310;
 
% 
case 11, 


% = = Block_1_0_0_11_0_1 = = 
 
% Sensor Kinematics 


    ROcp10_25 = S4*S5;
    ROcp10_35 = -C4*S5;
    ROcp10_85 = -S4*C5;
    ROcp10_95 = C4*C5;
    ROcp10_16 = C5*C6;
    ROcp10_26 = ROcp10_25*C6+C4*S6;
    ROcp10_36 = ROcp10_35*C6+S4*S6;
    ROcp10_46 = -C5*S6;
    ROcp10_56 = -(ROcp10_25*S6-C4*C6);
    ROcp10_66 = -(ROcp10_35*S6-S4*C6);
    OMcp10_25 = qd(5)*C4;
    OMcp10_35 = qd(5)*S4;
    OMcp10_16 = qd(4)+qd(6)*S5;
    OMcp10_26 = OMcp10_25+ROcp10_85*qd(6);
    OMcp10_36 = OMcp10_35+ROcp10_95*qd(6);
    OPcp10_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp10_26 = ROcp10_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp10_35*S5-ROcp10_95*qd(4));
    OPcp10_36 = ROcp10_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp10_25*S5-ROcp10_85*qd(4));

% = = Block_1_0_0_11_0_2 = = 
 
% Sensor Kinematics 


    ROcp10_17 = ROcp10_16*C7-S5*S7;
    ROcp10_27 = ROcp10_26*C7-ROcp10_85*S7;
    ROcp10_37 = ROcp10_36*C7-ROcp10_95*S7;
    ROcp10_77 = ROcp10_16*S7+S5*C7;
    ROcp10_87 = ROcp10_26*S7+ROcp10_85*C7;
    ROcp10_97 = ROcp10_36*S7+ROcp10_95*C7;
    ROcp10_48 = ROcp10_46*C8+ROcp10_77*S8;
    ROcp10_58 = ROcp10_56*C8+ROcp10_87*S8;
    ROcp10_68 = ROcp10_66*C8+ROcp10_97*S8;
    ROcp10_78 = -(ROcp10_46*S8-ROcp10_77*C8);
    ROcp10_88 = -(ROcp10_56*S8-ROcp10_87*C8);
    ROcp10_98 = -(ROcp10_66*S8-ROcp10_97*C8);
    ROcp10_19 = ROcp10_17*C9+ROcp10_48*S9;
    ROcp10_29 = ROcp10_27*C9+ROcp10_58*S9;
    ROcp10_39 = ROcp10_37*C9+ROcp10_68*S9;
    ROcp10_49 = -(ROcp10_17*S9-ROcp10_48*C9);
    ROcp10_59 = -(ROcp10_27*S9-ROcp10_58*C9);
    ROcp10_69 = -(ROcp10_37*S9-ROcp10_68*C9);
    ROcp10_110 = ROcp10_19*C10-ROcp10_78*S10;
    ROcp10_210 = ROcp10_29*C10-ROcp10_88*S10;
    ROcp10_310 = ROcp10_39*C10-ROcp10_98*S10;
    ROcp10_710 = ROcp10_19*S10+ROcp10_78*C10;
    ROcp10_810 = ROcp10_29*S10+ROcp10_88*C10;
    ROcp10_910 = ROcp10_39*S10+ROcp10_98*C10;
    ROcp10_411 = ROcp10_49*C11+ROcp10_710*S11;
    ROcp10_511 = ROcp10_59*C11+ROcp10_810*S11;
    ROcp10_611 = ROcp10_69*C11+ROcp10_910*S11;
    ROcp10_711 = -(ROcp10_49*S11-ROcp10_710*C11);
    ROcp10_811 = -(ROcp10_59*S11-ROcp10_810*C11);
    ROcp10_911 = -(ROcp10_69*S11-ROcp10_910*C11);
    RLcp10_17 = s.dpt(1,1)*ROcp10_16+s.dpt(3,1)*S5+ROcp10_46*s.dpt(2,1);
    RLcp10_27 = s.dpt(1,1)*ROcp10_26+s.dpt(3,1)*ROcp10_85+ROcp10_56*s.dpt(2,1);
    RLcp10_37 = s.dpt(1,1)*ROcp10_36+s.dpt(3,1)*ROcp10_95+ROcp10_66*s.dpt(2,1);
    OMcp10_17 = OMcp10_16+ROcp10_46*qd(7);
    OMcp10_27 = OMcp10_26+ROcp10_56*qd(7);
    OMcp10_37 = OMcp10_36+ROcp10_66*qd(7);
    ORcp10_17 = OMcp10_26*RLcp10_37-OMcp10_36*RLcp10_27;
    ORcp10_27 = -(OMcp10_16*RLcp10_37-OMcp10_36*RLcp10_17);
    ORcp10_37 = OMcp10_16*RLcp10_27-OMcp10_26*RLcp10_17;
    OPcp10_17 = OPcp10_16+ROcp10_46*qdd(7)+qd(7)*(OMcp10_26*ROcp10_66-OMcp10_36*ROcp10_56);
    OPcp10_27 = OPcp10_26+ROcp10_56*qdd(7)-qd(7)*(OMcp10_16*ROcp10_66-OMcp10_36*ROcp10_46);
    OPcp10_37 = OPcp10_36+ROcp10_66*qdd(7)+qd(7)*(OMcp10_16*ROcp10_56-OMcp10_26*ROcp10_46);
    RLcp10_18 = s.dpt(1,6)*ROcp10_17+s.dpt(3,6)*ROcp10_77+ROcp10_46*s.dpt(2,6);
    RLcp10_28 = s.dpt(1,6)*ROcp10_27+s.dpt(3,6)*ROcp10_87+ROcp10_56*s.dpt(2,6);
    RLcp10_38 = s.dpt(1,6)*ROcp10_37+s.dpt(3,6)*ROcp10_97+ROcp10_66*s.dpt(2,6);
    OMcp10_18 = OMcp10_17+ROcp10_17*qd(8);
    OMcp10_28 = OMcp10_27+ROcp10_27*qd(8);
    OMcp10_38 = OMcp10_37+ROcp10_37*qd(8);
    ORcp10_18 = OMcp10_27*RLcp10_38-OMcp10_37*RLcp10_28;
    ORcp10_28 = -(OMcp10_17*RLcp10_38-OMcp10_37*RLcp10_18);
    ORcp10_38 = OMcp10_17*RLcp10_28-OMcp10_27*RLcp10_18;
    OPcp10_18 = OPcp10_17+ROcp10_17*qdd(8)+qd(8)*(OMcp10_27*ROcp10_37-OMcp10_37*ROcp10_27);
    OPcp10_28 = OPcp10_27+ROcp10_27*qdd(8)-qd(8)*(OMcp10_17*ROcp10_37-OMcp10_37*ROcp10_17);
    OPcp10_38 = OPcp10_37+ROcp10_37*qdd(8)+qd(8)*(OMcp10_17*ROcp10_27-OMcp10_27*ROcp10_17);
    RLcp10_19 = s.dpt(1,8)*ROcp10_17+s.dpt(2,8)*ROcp10_48+ROcp10_78*s.dpt(3,8);
    RLcp10_29 = s.dpt(1,8)*ROcp10_27+s.dpt(2,8)*ROcp10_58+ROcp10_88*s.dpt(3,8);
    RLcp10_39 = s.dpt(1,8)*ROcp10_37+s.dpt(2,8)*ROcp10_68+ROcp10_98*s.dpt(3,8);
    OMcp10_19 = OMcp10_18+ROcp10_78*qd(9);
    OMcp10_29 = OMcp10_28+ROcp10_88*qd(9);
    OMcp10_39 = OMcp10_38+ROcp10_98*qd(9);
    ORcp10_19 = OMcp10_28*RLcp10_39-OMcp10_38*RLcp10_29;
    ORcp10_29 = -(OMcp10_18*RLcp10_39-OMcp10_38*RLcp10_19);
    ORcp10_39 = OMcp10_18*RLcp10_29-OMcp10_28*RLcp10_19;
    OPcp10_19 = OPcp10_18+ROcp10_78*qdd(9)+qd(9)*(OMcp10_28*ROcp10_98-OMcp10_38*ROcp10_88);
    OPcp10_29 = OPcp10_28+ROcp10_88*qdd(9)-qd(9)*(OMcp10_18*ROcp10_98-OMcp10_38*ROcp10_78);
    OPcp10_39 = OPcp10_38+ROcp10_98*qdd(9)+qd(9)*(OMcp10_18*ROcp10_88-OMcp10_28*ROcp10_78);
    RLcp10_110 = s.dpt(1,10)*ROcp10_19+s.dpt(2,10)*ROcp10_49+ROcp10_78*s.dpt(3,10);
    RLcp10_210 = s.dpt(1,10)*ROcp10_29+s.dpt(2,10)*ROcp10_59+ROcp10_88*s.dpt(3,10);
    RLcp10_310 = s.dpt(1,10)*ROcp10_39+s.dpt(2,10)*ROcp10_69+ROcp10_98*s.dpt(3,10);
    OMcp10_110 = OMcp10_19+ROcp10_49*qd(10);
    OMcp10_210 = OMcp10_29+ROcp10_59*qd(10);
    OMcp10_310 = OMcp10_39+ROcp10_69*qd(10);
    ORcp10_110 = OMcp10_29*RLcp10_310-OMcp10_39*RLcp10_210;
    ORcp10_210 = -(OMcp10_19*RLcp10_310-OMcp10_39*RLcp10_110);
    ORcp10_310 = OMcp10_19*RLcp10_210-OMcp10_29*RLcp10_110;
    OPcp10_110 = OPcp10_19+ROcp10_49*qdd(10)+qd(10)*(OMcp10_29*ROcp10_69-OMcp10_39*ROcp10_59);
    OPcp10_210 = OPcp10_29+ROcp10_59*qdd(10)-qd(10)*(OMcp10_19*ROcp10_69-OMcp10_39*ROcp10_49);
    OPcp10_310 = OPcp10_39+ROcp10_69*qdd(10)+qd(10)*(OMcp10_19*ROcp10_59-OMcp10_29*ROcp10_49);
    RLcp10_111 = s.dpt(1,12)*ROcp10_110+s.dpt(2,12)*ROcp10_49+ROcp10_710*s.dpt(3,12);
    RLcp10_211 = s.dpt(1,12)*ROcp10_210+s.dpt(2,12)*ROcp10_59+ROcp10_810*s.dpt(3,12);
    RLcp10_311 = s.dpt(1,12)*ROcp10_310+s.dpt(2,12)*ROcp10_69+ROcp10_910*s.dpt(3,12);
    POcp10_111 = RLcp10_110+RLcp10_111+RLcp10_17+RLcp10_18+RLcp10_19+q(1);
    POcp10_211 = RLcp10_210+RLcp10_211+RLcp10_27+RLcp10_28+RLcp10_29+q(2);
    POcp10_311 = RLcp10_310+RLcp10_311+RLcp10_37+RLcp10_38+RLcp10_39+q(3);
    OMcp10_111 = OMcp10_110+ROcp10_110*qd(11);
    OMcp10_211 = OMcp10_210+ROcp10_210*qd(11);
    OMcp10_311 = OMcp10_310+ROcp10_310*qd(11);
    ORcp10_111 = OMcp10_210*RLcp10_311-OMcp10_310*RLcp10_211;
    ORcp10_211 = -(OMcp10_110*RLcp10_311-OMcp10_310*RLcp10_111);
    ORcp10_311 = OMcp10_110*RLcp10_211-OMcp10_210*RLcp10_111;
    VIcp10_111 = ORcp10_110+ORcp10_111+ORcp10_17+ORcp10_18+ORcp10_19+qd(1);
    VIcp10_211 = ORcp10_210+ORcp10_211+ORcp10_27+ORcp10_28+ORcp10_29+qd(2);
    VIcp10_311 = ORcp10_310+ORcp10_311+ORcp10_37+ORcp10_38+ORcp10_39+qd(3);
    OPcp10_111 = OPcp10_110+ROcp10_110*qdd(11)+qd(11)*(OMcp10_210*ROcp10_310-OMcp10_310*ROcp10_210);
    OPcp10_211 = OPcp10_210+ROcp10_210*qdd(11)-qd(11)*(OMcp10_110*ROcp10_310-OMcp10_310*ROcp10_110);
    OPcp10_311 = OPcp10_310+ROcp10_310*qdd(11)+qd(11)*(OMcp10_110*ROcp10_210-OMcp10_210*ROcp10_110);
    ACcp10_111 = qdd(1)+OMcp10_210*ORcp10_311+OMcp10_26*ORcp10_37+OMcp10_27*ORcp10_38+OMcp10_28*ORcp10_39+OMcp10_29*ORcp10_310-OMcp10_310*...
 ORcp10_211-OMcp10_36*ORcp10_27-OMcp10_37*ORcp10_28-OMcp10_38*ORcp10_29-OMcp10_39*ORcp10_210+OPcp10_210*RLcp10_311+OPcp10_26*RLcp10_37+OPcp10_27*...
 RLcp10_38+OPcp10_28*RLcp10_39+OPcp10_29*RLcp10_310-OPcp10_310*RLcp10_211-OPcp10_36*RLcp10_27-OPcp10_37*RLcp10_28-OPcp10_38*RLcp10_29-OPcp10_39*...
 RLcp10_210;
    ACcp10_211 = qdd(2)-OMcp10_110*ORcp10_311-OMcp10_16*ORcp10_37-OMcp10_17*ORcp10_38-OMcp10_18*ORcp10_39-OMcp10_19*ORcp10_310+OMcp10_310*...
 ORcp10_111+OMcp10_36*ORcp10_17+OMcp10_37*ORcp10_18+OMcp10_38*ORcp10_19+OMcp10_39*ORcp10_110-OPcp10_110*RLcp10_311-OPcp10_16*RLcp10_37-OPcp10_17*...
 RLcp10_38-OPcp10_18*RLcp10_39-OPcp10_19*RLcp10_310+OPcp10_310*RLcp10_111+OPcp10_36*RLcp10_17+OPcp10_37*RLcp10_18+OPcp10_38*RLcp10_19+OPcp10_39*...
 RLcp10_110;
    ACcp10_311 = qdd(3)+OMcp10_110*ORcp10_211+OMcp10_16*ORcp10_27+OMcp10_17*ORcp10_28+OMcp10_18*ORcp10_29+OMcp10_19*ORcp10_210-OMcp10_210*...
 ORcp10_111-OMcp10_26*ORcp10_17-OMcp10_27*ORcp10_18-OMcp10_28*ORcp10_19-OMcp10_29*ORcp10_110+OPcp10_110*RLcp10_211+OPcp10_16*RLcp10_27+OPcp10_17*...
 RLcp10_28+OPcp10_18*RLcp10_29+OPcp10_19*RLcp10_210-OPcp10_210*RLcp10_111-OPcp10_26*RLcp10_17-OPcp10_27*RLcp10_18-OPcp10_28*RLcp10_19-OPcp10_29*...
 RLcp10_110;

% = = Block_1_0_0_11_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp10_111;
    sens.P(2) = POcp10_211;
    sens.P(3) = POcp10_311;
    sens.R(1,1) = ROcp10_110;
    sens.R(1,2) = ROcp10_210;
    sens.R(1,3) = ROcp10_310;
    sens.R(2,1) = ROcp10_411;
    sens.R(2,2) = ROcp10_511;
    sens.R(2,3) = ROcp10_611;
    sens.R(3,1) = ROcp10_711;
    sens.R(3,2) = ROcp10_811;
    sens.R(3,3) = ROcp10_911;
    sens.V(1) = VIcp10_111;
    sens.V(2) = VIcp10_211;
    sens.V(3) = VIcp10_311;
    sens.OM(1) = OMcp10_111;
    sens.OM(2) = OMcp10_211;
    sens.OM(3) = OMcp10_311;
    sens.A(1) = ACcp10_111;
    sens.A(2) = ACcp10_211;
    sens.A(3) = ACcp10_311;
    sens.OMP(1) = OPcp10_111;
    sens.OMP(2) = OPcp10_211;
    sens.OMP(3) = OPcp10_311;
 
% 
case 12, 


% = = Block_1_0_0_12_0_1 = = 
 
% Sensor Kinematics 


    ROcp11_25 = S4*S5;
    ROcp11_35 = -C4*S5;
    ROcp11_85 = -S4*C5;
    ROcp11_95 = C4*C5;
    ROcp11_16 = C5*C6;
    ROcp11_26 = ROcp11_25*C6+C4*S6;
    ROcp11_36 = ROcp11_35*C6+S4*S6;
    ROcp11_46 = -C5*S6;
    ROcp11_56 = -(ROcp11_25*S6-C4*C6);
    ROcp11_66 = -(ROcp11_35*S6-S4*C6);
    OMcp11_25 = qd(5)*C4;
    OMcp11_35 = qd(5)*S4;
    OMcp11_16 = qd(4)+qd(6)*S5;
    OMcp11_26 = OMcp11_25+ROcp11_85*qd(6);
    OMcp11_36 = OMcp11_35+ROcp11_95*qd(6);
    OPcp11_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp11_26 = ROcp11_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp11_35*S5-ROcp11_95*qd(4));
    OPcp11_36 = ROcp11_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp11_25*S5-ROcp11_85*qd(4));

% = = Block_1_0_0_12_0_2 = = 
 
% Sensor Kinematics 


    ROcp11_17 = ROcp11_16*C7-S5*S7;
    ROcp11_27 = ROcp11_26*C7-ROcp11_85*S7;
    ROcp11_37 = ROcp11_36*C7-ROcp11_95*S7;
    ROcp11_77 = ROcp11_16*S7+S5*C7;
    ROcp11_87 = ROcp11_26*S7+ROcp11_85*C7;
    ROcp11_97 = ROcp11_36*S7+ROcp11_95*C7;
    ROcp11_48 = ROcp11_46*C8+ROcp11_77*S8;
    ROcp11_58 = ROcp11_56*C8+ROcp11_87*S8;
    ROcp11_68 = ROcp11_66*C8+ROcp11_97*S8;
    ROcp11_78 = -(ROcp11_46*S8-ROcp11_77*C8);
    ROcp11_88 = -(ROcp11_56*S8-ROcp11_87*C8);
    ROcp11_98 = -(ROcp11_66*S8-ROcp11_97*C8);
    ROcp11_19 = ROcp11_17*C9+ROcp11_48*S9;
    ROcp11_29 = ROcp11_27*C9+ROcp11_58*S9;
    ROcp11_39 = ROcp11_37*C9+ROcp11_68*S9;
    ROcp11_49 = -(ROcp11_17*S9-ROcp11_48*C9);
    ROcp11_59 = -(ROcp11_27*S9-ROcp11_58*C9);
    ROcp11_69 = -(ROcp11_37*S9-ROcp11_68*C9);
    ROcp11_110 = ROcp11_19*C10-ROcp11_78*S10;
    ROcp11_210 = ROcp11_29*C10-ROcp11_88*S10;
    ROcp11_310 = ROcp11_39*C10-ROcp11_98*S10;
    ROcp11_710 = ROcp11_19*S10+ROcp11_78*C10;
    ROcp11_810 = ROcp11_29*S10+ROcp11_88*C10;
    ROcp11_910 = ROcp11_39*S10+ROcp11_98*C10;
    ROcp11_411 = ROcp11_49*C11+ROcp11_710*S11;
    ROcp11_511 = ROcp11_59*C11+ROcp11_810*S11;
    ROcp11_611 = ROcp11_69*C11+ROcp11_910*S11;
    ROcp11_711 = -(ROcp11_49*S11-ROcp11_710*C11);
    ROcp11_811 = -(ROcp11_59*S11-ROcp11_810*C11);
    ROcp11_911 = -(ROcp11_69*S11-ROcp11_910*C11);
    ROcp11_112 = ROcp11_110*C12-ROcp11_711*S12;
    ROcp11_212 = ROcp11_210*C12-ROcp11_811*S12;
    ROcp11_312 = ROcp11_310*C12-ROcp11_911*S12;
    ROcp11_712 = ROcp11_110*S12+ROcp11_711*C12;
    ROcp11_812 = ROcp11_210*S12+ROcp11_811*C12;
    ROcp11_912 = ROcp11_310*S12+ROcp11_911*C12;
    RLcp11_17 = s.dpt(1,1)*ROcp11_16+s.dpt(3,1)*S5+ROcp11_46*s.dpt(2,1);
    RLcp11_27 = s.dpt(1,1)*ROcp11_26+s.dpt(3,1)*ROcp11_85+ROcp11_56*s.dpt(2,1);
    RLcp11_37 = s.dpt(1,1)*ROcp11_36+s.dpt(3,1)*ROcp11_95+ROcp11_66*s.dpt(2,1);
    OMcp11_17 = OMcp11_16+ROcp11_46*qd(7);
    OMcp11_27 = OMcp11_26+ROcp11_56*qd(7);
    OMcp11_37 = OMcp11_36+ROcp11_66*qd(7);
    ORcp11_17 = OMcp11_26*RLcp11_37-OMcp11_36*RLcp11_27;
    ORcp11_27 = -(OMcp11_16*RLcp11_37-OMcp11_36*RLcp11_17);
    ORcp11_37 = OMcp11_16*RLcp11_27-OMcp11_26*RLcp11_17;
    OPcp11_17 = OPcp11_16+ROcp11_46*qdd(7)+qd(7)*(OMcp11_26*ROcp11_66-OMcp11_36*ROcp11_56);
    OPcp11_27 = OPcp11_26+ROcp11_56*qdd(7)-qd(7)*(OMcp11_16*ROcp11_66-OMcp11_36*ROcp11_46);
    OPcp11_37 = OPcp11_36+ROcp11_66*qdd(7)+qd(7)*(OMcp11_16*ROcp11_56-OMcp11_26*ROcp11_46);
    RLcp11_18 = s.dpt(1,6)*ROcp11_17+s.dpt(3,6)*ROcp11_77+ROcp11_46*s.dpt(2,6);
    RLcp11_28 = s.dpt(1,6)*ROcp11_27+s.dpt(3,6)*ROcp11_87+ROcp11_56*s.dpt(2,6);
    RLcp11_38 = s.dpt(1,6)*ROcp11_37+s.dpt(3,6)*ROcp11_97+ROcp11_66*s.dpt(2,6);
    OMcp11_18 = OMcp11_17+ROcp11_17*qd(8);
    OMcp11_28 = OMcp11_27+ROcp11_27*qd(8);
    OMcp11_38 = OMcp11_37+ROcp11_37*qd(8);
    ORcp11_18 = OMcp11_27*RLcp11_38-OMcp11_37*RLcp11_28;
    ORcp11_28 = -(OMcp11_17*RLcp11_38-OMcp11_37*RLcp11_18);
    ORcp11_38 = OMcp11_17*RLcp11_28-OMcp11_27*RLcp11_18;
    OPcp11_18 = OPcp11_17+ROcp11_17*qdd(8)+qd(8)*(OMcp11_27*ROcp11_37-OMcp11_37*ROcp11_27);
    OPcp11_28 = OPcp11_27+ROcp11_27*qdd(8)-qd(8)*(OMcp11_17*ROcp11_37-OMcp11_37*ROcp11_17);
    OPcp11_38 = OPcp11_37+ROcp11_37*qdd(8)+qd(8)*(OMcp11_17*ROcp11_27-OMcp11_27*ROcp11_17);
    RLcp11_19 = s.dpt(1,8)*ROcp11_17+s.dpt(2,8)*ROcp11_48+ROcp11_78*s.dpt(3,8);
    RLcp11_29 = s.dpt(1,8)*ROcp11_27+s.dpt(2,8)*ROcp11_58+ROcp11_88*s.dpt(3,8);
    RLcp11_39 = s.dpt(1,8)*ROcp11_37+s.dpt(2,8)*ROcp11_68+ROcp11_98*s.dpt(3,8);
    OMcp11_19 = OMcp11_18+ROcp11_78*qd(9);
    OMcp11_29 = OMcp11_28+ROcp11_88*qd(9);
    OMcp11_39 = OMcp11_38+ROcp11_98*qd(9);
    ORcp11_19 = OMcp11_28*RLcp11_39-OMcp11_38*RLcp11_29;
    ORcp11_29 = -(OMcp11_18*RLcp11_39-OMcp11_38*RLcp11_19);
    ORcp11_39 = OMcp11_18*RLcp11_29-OMcp11_28*RLcp11_19;
    OPcp11_19 = OPcp11_18+ROcp11_78*qdd(9)+qd(9)*(OMcp11_28*ROcp11_98-OMcp11_38*ROcp11_88);
    OPcp11_29 = OPcp11_28+ROcp11_88*qdd(9)-qd(9)*(OMcp11_18*ROcp11_98-OMcp11_38*ROcp11_78);
    OPcp11_39 = OPcp11_38+ROcp11_98*qdd(9)+qd(9)*(OMcp11_18*ROcp11_88-OMcp11_28*ROcp11_78);
    RLcp11_110 = s.dpt(1,10)*ROcp11_19+s.dpt(2,10)*ROcp11_49+ROcp11_78*s.dpt(3,10);
    RLcp11_210 = s.dpt(1,10)*ROcp11_29+s.dpt(2,10)*ROcp11_59+ROcp11_88*s.dpt(3,10);
    RLcp11_310 = s.dpt(1,10)*ROcp11_39+s.dpt(2,10)*ROcp11_69+ROcp11_98*s.dpt(3,10);
    OMcp11_110 = OMcp11_19+ROcp11_49*qd(10);
    OMcp11_210 = OMcp11_29+ROcp11_59*qd(10);
    OMcp11_310 = OMcp11_39+ROcp11_69*qd(10);
    ORcp11_110 = OMcp11_29*RLcp11_310-OMcp11_39*RLcp11_210;
    ORcp11_210 = -(OMcp11_19*RLcp11_310-OMcp11_39*RLcp11_110);
    ORcp11_310 = OMcp11_19*RLcp11_210-OMcp11_29*RLcp11_110;
    OPcp11_110 = OPcp11_19+ROcp11_49*qdd(10)+qd(10)*(OMcp11_29*ROcp11_69-OMcp11_39*ROcp11_59);
    OPcp11_210 = OPcp11_29+ROcp11_59*qdd(10)-qd(10)*(OMcp11_19*ROcp11_69-OMcp11_39*ROcp11_49);
    OPcp11_310 = OPcp11_39+ROcp11_69*qdd(10)+qd(10)*(OMcp11_19*ROcp11_59-OMcp11_29*ROcp11_49);
    RLcp11_111 = s.dpt(1,12)*ROcp11_110+s.dpt(2,12)*ROcp11_49+ROcp11_710*s.dpt(3,12);
    RLcp11_211 = s.dpt(1,12)*ROcp11_210+s.dpt(2,12)*ROcp11_59+ROcp11_810*s.dpt(3,12);
    RLcp11_311 = s.dpt(1,12)*ROcp11_310+s.dpt(2,12)*ROcp11_69+ROcp11_910*s.dpt(3,12);
    OMcp11_111 = OMcp11_110+ROcp11_110*qd(11);
    OMcp11_211 = OMcp11_210+ROcp11_210*qd(11);
    OMcp11_311 = OMcp11_310+ROcp11_310*qd(11);
    ORcp11_111 = OMcp11_210*RLcp11_311-OMcp11_310*RLcp11_211;
    ORcp11_211 = -(OMcp11_110*RLcp11_311-OMcp11_310*RLcp11_111);
    ORcp11_311 = OMcp11_110*RLcp11_211-OMcp11_210*RLcp11_111;
    OPcp11_111 = OPcp11_110+ROcp11_110*qdd(11)+qd(11)*(OMcp11_210*ROcp11_310-OMcp11_310*ROcp11_210);
    OPcp11_211 = OPcp11_210+ROcp11_210*qdd(11)-qd(11)*(OMcp11_110*ROcp11_310-OMcp11_310*ROcp11_110);
    OPcp11_311 = OPcp11_310+ROcp11_310*qdd(11)+qd(11)*(OMcp11_110*ROcp11_210-OMcp11_210*ROcp11_110);
    RLcp11_112 = s.dpt(1,14)*ROcp11_110+s.dpt(2,14)*ROcp11_411+s.dpt(3,14)*ROcp11_711;
    RLcp11_212 = s.dpt(1,14)*ROcp11_210+s.dpt(2,14)*ROcp11_511+s.dpt(3,14)*ROcp11_811;
    RLcp11_312 = s.dpt(1,14)*ROcp11_310+s.dpt(2,14)*ROcp11_611+s.dpt(3,14)*ROcp11_911;
    POcp11_112 = RLcp11_110+RLcp11_111+RLcp11_112+RLcp11_17+RLcp11_18+RLcp11_19+q(1);
    POcp11_212 = RLcp11_210+RLcp11_211+RLcp11_212+RLcp11_27+RLcp11_28+RLcp11_29+q(2);
    POcp11_312 = RLcp11_310+RLcp11_311+RLcp11_312+RLcp11_37+RLcp11_38+RLcp11_39+q(3);
    OMcp11_112 = OMcp11_111+ROcp11_411*qd(12);
    OMcp11_212 = OMcp11_211+ROcp11_511*qd(12);
    OMcp11_312 = OMcp11_311+ROcp11_611*qd(12);
    ORcp11_112 = OMcp11_211*RLcp11_312-OMcp11_311*RLcp11_212;
    ORcp11_212 = -(OMcp11_111*RLcp11_312-OMcp11_311*RLcp11_112);
    ORcp11_312 = OMcp11_111*RLcp11_212-OMcp11_211*RLcp11_112;
    VIcp11_112 = ORcp11_110+ORcp11_111+ORcp11_112+ORcp11_17+ORcp11_18+ORcp11_19+qd(1);
    VIcp11_212 = ORcp11_210+ORcp11_211+ORcp11_212+ORcp11_27+ORcp11_28+ORcp11_29+qd(2);
    VIcp11_312 = ORcp11_310+ORcp11_311+ORcp11_312+ORcp11_37+ORcp11_38+ORcp11_39+qd(3);
    OPcp11_112 = OPcp11_111+ROcp11_411*qdd(12)+qd(12)*(OMcp11_211*ROcp11_611-OMcp11_311*ROcp11_511);
    OPcp11_212 = OPcp11_211+ROcp11_511*qdd(12)-qd(12)*(OMcp11_111*ROcp11_611-OMcp11_311*ROcp11_411);
    OPcp11_312 = OPcp11_311+ROcp11_611*qdd(12)+qd(12)*(OMcp11_111*ROcp11_511-OMcp11_211*ROcp11_411);
    ACcp11_112 = qdd(1)+OMcp11_210*ORcp11_311+OMcp11_211*ORcp11_312+OMcp11_26*ORcp11_37+OMcp11_27*ORcp11_38+OMcp11_28*ORcp11_39+OMcp11_29*...
 ORcp11_310-OMcp11_310*ORcp11_211-OMcp11_311*ORcp11_212-OMcp11_36*ORcp11_27-OMcp11_37*ORcp11_28-OMcp11_38*ORcp11_29-OMcp11_39*ORcp11_210+OPcp11_210*...
 RLcp11_311+OPcp11_211*RLcp11_312+OPcp11_26*RLcp11_37+OPcp11_27*RLcp11_38+OPcp11_28*RLcp11_39+OPcp11_29*RLcp11_310-OPcp11_310*RLcp11_211-OPcp11_311*...
 RLcp11_212-OPcp11_36*RLcp11_27-OPcp11_37*RLcp11_28-OPcp11_38*RLcp11_29-OPcp11_39*RLcp11_210;
    ACcp11_212 = qdd(2)-OMcp11_110*ORcp11_311-OMcp11_111*ORcp11_312-OMcp11_16*ORcp11_37-OMcp11_17*ORcp11_38-OMcp11_18*ORcp11_39-OMcp11_19*...
 ORcp11_310+OMcp11_310*ORcp11_111+OMcp11_311*ORcp11_112+OMcp11_36*ORcp11_17+OMcp11_37*ORcp11_18+OMcp11_38*ORcp11_19+OMcp11_39*ORcp11_110-OPcp11_110*...
 RLcp11_311-OPcp11_111*RLcp11_312-OPcp11_16*RLcp11_37-OPcp11_17*RLcp11_38-OPcp11_18*RLcp11_39-OPcp11_19*RLcp11_310+OPcp11_310*RLcp11_111+OPcp11_311*...
 RLcp11_112+OPcp11_36*RLcp11_17+OPcp11_37*RLcp11_18+OPcp11_38*RLcp11_19+OPcp11_39*RLcp11_110;
    ACcp11_312 = qdd(3)+OMcp11_110*ORcp11_211+OMcp11_111*ORcp11_212+OMcp11_16*ORcp11_27+OMcp11_17*ORcp11_28+OMcp11_18*ORcp11_29+OMcp11_19*...
 ORcp11_210-OMcp11_210*ORcp11_111-OMcp11_211*ORcp11_112-OMcp11_26*ORcp11_17-OMcp11_27*ORcp11_18-OMcp11_28*ORcp11_19-OMcp11_29*ORcp11_110+OPcp11_110*...
 RLcp11_211+OPcp11_111*RLcp11_212+OPcp11_16*RLcp11_27+OPcp11_17*RLcp11_28+OPcp11_18*RLcp11_29+OPcp11_19*RLcp11_210-OPcp11_210*RLcp11_111-OPcp11_211*...
 RLcp11_112-OPcp11_26*RLcp11_17-OPcp11_27*RLcp11_18-OPcp11_28*RLcp11_19-OPcp11_29*RLcp11_110;

% = = Block_1_0_0_12_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp11_112;
    sens.P(2) = POcp11_212;
    sens.P(3) = POcp11_312;
    sens.R(1,1) = ROcp11_112;
    sens.R(1,2) = ROcp11_212;
    sens.R(1,3) = ROcp11_312;
    sens.R(2,1) = ROcp11_411;
    sens.R(2,2) = ROcp11_511;
    sens.R(2,3) = ROcp11_611;
    sens.R(3,1) = ROcp11_712;
    sens.R(3,2) = ROcp11_812;
    sens.R(3,3) = ROcp11_912;
    sens.V(1) = VIcp11_112;
    sens.V(2) = VIcp11_212;
    sens.V(3) = VIcp11_312;
    sens.OM(1) = OMcp11_112;
    sens.OM(2) = OMcp11_212;
    sens.OM(3) = OMcp11_312;
    sens.A(1) = ACcp11_112;
    sens.A(2) = ACcp11_212;
    sens.A(3) = ACcp11_312;
    sens.OMP(1) = OPcp11_112;
    sens.OMP(2) = OPcp11_212;
    sens.OMP(3) = OPcp11_312;
 
% 
case 13, 


% = = Block_1_0_0_13_0_1 = = 
 
% Sensor Kinematics 


    ROcp12_25 = S4*S5;
    ROcp12_35 = -C4*S5;
    ROcp12_85 = -S4*C5;
    ROcp12_95 = C4*C5;
    ROcp12_16 = C5*C6;
    ROcp12_26 = ROcp12_25*C6+C4*S6;
    ROcp12_36 = ROcp12_35*C6+S4*S6;
    ROcp12_46 = -C5*S6;
    ROcp12_56 = -(ROcp12_25*S6-C4*C6);
    ROcp12_66 = -(ROcp12_35*S6-S4*C6);
    OMcp12_25 = qd(5)*C4;
    OMcp12_35 = qd(5)*S4;
    OMcp12_16 = qd(4)+qd(6)*S5;
    OMcp12_26 = OMcp12_25+ROcp12_85*qd(6);
    OMcp12_36 = OMcp12_35+ROcp12_95*qd(6);
    OPcp12_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp12_26 = ROcp12_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp12_35*S5-ROcp12_95*qd(4));
    OPcp12_36 = ROcp12_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp12_25*S5-ROcp12_85*qd(4));

% = = Block_1_0_0_13_0_3 = = 
 
% Sensor Kinematics 


    ROcp12_113 = ROcp12_16*C13-S13*S5;
    ROcp12_213 = ROcp12_26*C13-ROcp12_85*S13;
    ROcp12_313 = ROcp12_36*C13-ROcp12_95*S13;
    ROcp12_713 = ROcp12_16*S13+C13*S5;
    ROcp12_813 = ROcp12_26*S13+ROcp12_85*C13;
    ROcp12_913 = ROcp12_36*S13+ROcp12_95*C13;
    RLcp12_113 = s.dpt(1,2)*ROcp12_16+s.dpt(3,2)*S5+ROcp12_46*s.dpt(2,2);
    RLcp12_213 = s.dpt(1,2)*ROcp12_26+s.dpt(3,2)*ROcp12_85+ROcp12_56*s.dpt(2,2);
    RLcp12_313 = s.dpt(1,2)*ROcp12_36+s.dpt(3,2)*ROcp12_95+ROcp12_66*s.dpt(2,2);
    POcp12_113 = RLcp12_113+q(1);
    POcp12_213 = RLcp12_213+q(2);
    POcp12_313 = RLcp12_313+q(3);
    OMcp12_113 = OMcp12_16+ROcp12_46*qd(13);
    OMcp12_213 = OMcp12_26+ROcp12_56*qd(13);
    OMcp12_313 = OMcp12_36+ROcp12_66*qd(13);
    ORcp12_113 = OMcp12_26*RLcp12_313-OMcp12_36*RLcp12_213;
    ORcp12_213 = -(OMcp12_16*RLcp12_313-OMcp12_36*RLcp12_113);
    ORcp12_313 = OMcp12_16*RLcp12_213-OMcp12_26*RLcp12_113;
    VIcp12_113 = ORcp12_113+qd(1);
    VIcp12_213 = ORcp12_213+qd(2);
    VIcp12_313 = ORcp12_313+qd(3);
    OPcp12_113 = OPcp12_16+ROcp12_46*qdd(13)+qd(13)*(OMcp12_26*ROcp12_66-OMcp12_36*ROcp12_56);
    OPcp12_213 = OPcp12_26+ROcp12_56*qdd(13)-qd(13)*(OMcp12_16*ROcp12_66-OMcp12_36*ROcp12_46);
    OPcp12_313 = OPcp12_36+ROcp12_66*qdd(13)+qd(13)*(OMcp12_16*ROcp12_56-OMcp12_26*ROcp12_46);
    ACcp12_113 = qdd(1)+OMcp12_26*ORcp12_313-OMcp12_36*ORcp12_213+OPcp12_26*RLcp12_313-OPcp12_36*RLcp12_213;
    ACcp12_213 = qdd(2)-OMcp12_16*ORcp12_313+OMcp12_36*ORcp12_113-OPcp12_16*RLcp12_313+OPcp12_36*RLcp12_113;
    ACcp12_313 = qdd(3)+OMcp12_16*ORcp12_213-OMcp12_26*ORcp12_113+OPcp12_16*RLcp12_213-OPcp12_26*RLcp12_113;

% = = Block_1_0_0_13_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp12_113;
    sens.P(2) = POcp12_213;
    sens.P(3) = POcp12_313;
    sens.R(1,1) = ROcp12_113;
    sens.R(1,2) = ROcp12_213;
    sens.R(1,3) = ROcp12_313;
    sens.R(2,1) = ROcp12_46;
    sens.R(2,2) = ROcp12_56;
    sens.R(2,3) = ROcp12_66;
    sens.R(3,1) = ROcp12_713;
    sens.R(3,2) = ROcp12_813;
    sens.R(3,3) = ROcp12_913;
    sens.V(1) = VIcp12_113;
    sens.V(2) = VIcp12_213;
    sens.V(3) = VIcp12_313;
    sens.OM(1) = OMcp12_113;
    sens.OM(2) = OMcp12_213;
    sens.OM(3) = OMcp12_313;
    sens.A(1) = ACcp12_113;
    sens.A(2) = ACcp12_213;
    sens.A(3) = ACcp12_313;
    sens.OMP(1) = OPcp12_113;
    sens.OMP(2) = OPcp12_213;
    sens.OMP(3) = OPcp12_313;
 
% 
case 14, 


% = = Block_1_0_0_14_0_1 = = 
 
% Sensor Kinematics 


    ROcp13_25 = S4*S5;
    ROcp13_35 = -C4*S5;
    ROcp13_85 = -S4*C5;
    ROcp13_95 = C4*C5;
    ROcp13_16 = C5*C6;
    ROcp13_26 = ROcp13_25*C6+C4*S6;
    ROcp13_36 = ROcp13_35*C6+S4*S6;
    ROcp13_46 = -C5*S6;
    ROcp13_56 = -(ROcp13_25*S6-C4*C6);
    ROcp13_66 = -(ROcp13_35*S6-S4*C6);
    OMcp13_25 = qd(5)*C4;
    OMcp13_35 = qd(5)*S4;
    OMcp13_16 = qd(4)+qd(6)*S5;
    OMcp13_26 = OMcp13_25+ROcp13_85*qd(6);
    OMcp13_36 = OMcp13_35+ROcp13_95*qd(6);
    OPcp13_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp13_26 = ROcp13_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp13_35*S5-ROcp13_95*qd(4));
    OPcp13_36 = ROcp13_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp13_25*S5-ROcp13_85*qd(4));

% = = Block_1_0_0_14_0_3 = = 
 
% Sensor Kinematics 


    ROcp13_113 = ROcp13_16*C13-S13*S5;
    ROcp13_213 = ROcp13_26*C13-ROcp13_85*S13;
    ROcp13_313 = ROcp13_36*C13-ROcp13_95*S13;
    ROcp13_713 = ROcp13_16*S13+C13*S5;
    ROcp13_813 = ROcp13_26*S13+ROcp13_85*C13;
    ROcp13_913 = ROcp13_36*S13+ROcp13_95*C13;
    ROcp13_414 = ROcp13_46*C14+ROcp13_713*S14;
    ROcp13_514 = ROcp13_56*C14+ROcp13_813*S14;
    ROcp13_614 = ROcp13_66*C14+ROcp13_913*S14;
    ROcp13_714 = -(ROcp13_46*S14-ROcp13_713*C14);
    ROcp13_814 = -(ROcp13_56*S14-ROcp13_813*C14);
    ROcp13_914 = -(ROcp13_66*S14-ROcp13_913*C14);
    RLcp13_113 = s.dpt(1,2)*ROcp13_16+s.dpt(3,2)*S5+ROcp13_46*s.dpt(2,2);
    RLcp13_213 = s.dpt(1,2)*ROcp13_26+s.dpt(3,2)*ROcp13_85+ROcp13_56*s.dpt(2,2);
    RLcp13_313 = s.dpt(1,2)*ROcp13_36+s.dpt(3,2)*ROcp13_95+ROcp13_66*s.dpt(2,2);
    OMcp13_113 = OMcp13_16+ROcp13_46*qd(13);
    OMcp13_213 = OMcp13_26+ROcp13_56*qd(13);
    OMcp13_313 = OMcp13_36+ROcp13_66*qd(13);
    ORcp13_113 = OMcp13_26*RLcp13_313-OMcp13_36*RLcp13_213;
    ORcp13_213 = -(OMcp13_16*RLcp13_313-OMcp13_36*RLcp13_113);
    ORcp13_313 = OMcp13_16*RLcp13_213-OMcp13_26*RLcp13_113;
    OPcp13_113 = OPcp13_16+ROcp13_46*qdd(13)+qd(13)*(OMcp13_26*ROcp13_66-OMcp13_36*ROcp13_56);
    OPcp13_213 = OPcp13_26+ROcp13_56*qdd(13)-qd(13)*(OMcp13_16*ROcp13_66-OMcp13_36*ROcp13_46);
    OPcp13_313 = OPcp13_36+ROcp13_66*qdd(13)+qd(13)*(OMcp13_16*ROcp13_56-OMcp13_26*ROcp13_46);
    RLcp13_114 = s.dpt(1,18)*ROcp13_113+s.dpt(3,18)*ROcp13_713+ROcp13_46*s.dpt(2,18);
    RLcp13_214 = s.dpt(1,18)*ROcp13_213+s.dpt(3,18)*ROcp13_813+ROcp13_56*s.dpt(2,18);
    RLcp13_314 = s.dpt(1,18)*ROcp13_313+s.dpt(3,18)*ROcp13_913+ROcp13_66*s.dpt(2,18);
    POcp13_114 = RLcp13_113+RLcp13_114+q(1);
    POcp13_214 = RLcp13_213+RLcp13_214+q(2);
    POcp13_314 = RLcp13_313+RLcp13_314+q(3);
    OMcp13_114 = OMcp13_113+ROcp13_113*qd(14);
    OMcp13_214 = OMcp13_213+ROcp13_213*qd(14);
    OMcp13_314 = OMcp13_313+ROcp13_313*qd(14);
    ORcp13_114 = OMcp13_213*RLcp13_314-OMcp13_313*RLcp13_214;
    ORcp13_214 = -(OMcp13_113*RLcp13_314-OMcp13_313*RLcp13_114);
    ORcp13_314 = OMcp13_113*RLcp13_214-OMcp13_213*RLcp13_114;
    VIcp13_114 = ORcp13_113+ORcp13_114+qd(1);
    VIcp13_214 = ORcp13_213+ORcp13_214+qd(2);
    VIcp13_314 = ORcp13_313+ORcp13_314+qd(3);
    OPcp13_114 = OPcp13_113+ROcp13_113*qdd(14)+qd(14)*(OMcp13_213*ROcp13_313-OMcp13_313*ROcp13_213);
    OPcp13_214 = OPcp13_213+ROcp13_213*qdd(14)-qd(14)*(OMcp13_113*ROcp13_313-OMcp13_313*ROcp13_113);
    OPcp13_314 = OPcp13_313+ROcp13_313*qdd(14)+qd(14)*(OMcp13_113*ROcp13_213-OMcp13_213*ROcp13_113);
    ACcp13_114 = qdd(1)+OMcp13_213*ORcp13_314+OMcp13_26*ORcp13_313-OMcp13_313*ORcp13_214-OMcp13_36*ORcp13_213+OPcp13_213*RLcp13_314+OPcp13_26*...
 RLcp13_313-OPcp13_313*RLcp13_214-OPcp13_36*RLcp13_213;
    ACcp13_214 = qdd(2)-OMcp13_113*ORcp13_314-OMcp13_16*ORcp13_313+OMcp13_313*ORcp13_114+OMcp13_36*ORcp13_113-OPcp13_113*RLcp13_314-OPcp13_16*...
 RLcp13_313+OPcp13_313*RLcp13_114+OPcp13_36*RLcp13_113;
    ACcp13_314 = qdd(3)+OMcp13_113*ORcp13_214+OMcp13_16*ORcp13_213-OMcp13_213*ORcp13_114-OMcp13_26*ORcp13_113+OPcp13_113*RLcp13_214+OPcp13_16*...
 RLcp13_213-OPcp13_213*RLcp13_114-OPcp13_26*RLcp13_113;

% = = Block_1_0_0_14_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp13_114;
    sens.P(2) = POcp13_214;
    sens.P(3) = POcp13_314;
    sens.R(1,1) = ROcp13_113;
    sens.R(1,2) = ROcp13_213;
    sens.R(1,3) = ROcp13_313;
    sens.R(2,1) = ROcp13_414;
    sens.R(2,2) = ROcp13_514;
    sens.R(2,3) = ROcp13_614;
    sens.R(3,1) = ROcp13_714;
    sens.R(3,2) = ROcp13_814;
    sens.R(3,3) = ROcp13_914;
    sens.V(1) = VIcp13_114;
    sens.V(2) = VIcp13_214;
    sens.V(3) = VIcp13_314;
    sens.OM(1) = OMcp13_114;
    sens.OM(2) = OMcp13_214;
    sens.OM(3) = OMcp13_314;
    sens.A(1) = ACcp13_114;
    sens.A(2) = ACcp13_214;
    sens.A(3) = ACcp13_314;
    sens.OMP(1) = OPcp13_114;
    sens.OMP(2) = OPcp13_214;
    sens.OMP(3) = OPcp13_314;
 
% 
case 15, 


% = = Block_1_0_0_15_0_1 = = 
 
% Sensor Kinematics 


    ROcp14_25 = S4*S5;
    ROcp14_35 = -C4*S5;
    ROcp14_85 = -S4*C5;
    ROcp14_95 = C4*C5;
    ROcp14_16 = C5*C6;
    ROcp14_26 = ROcp14_25*C6+C4*S6;
    ROcp14_36 = ROcp14_35*C6+S4*S6;
    ROcp14_46 = -C5*S6;
    ROcp14_56 = -(ROcp14_25*S6-C4*C6);
    ROcp14_66 = -(ROcp14_35*S6-S4*C6);
    OMcp14_25 = qd(5)*C4;
    OMcp14_35 = qd(5)*S4;
    OMcp14_16 = qd(4)+qd(6)*S5;
    OMcp14_26 = OMcp14_25+ROcp14_85*qd(6);
    OMcp14_36 = OMcp14_35+ROcp14_95*qd(6);
    OPcp14_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp14_26 = ROcp14_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp14_35*S5-ROcp14_95*qd(4));
    OPcp14_36 = ROcp14_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp14_25*S5-ROcp14_85*qd(4));

% = = Block_1_0_0_15_0_3 = = 
 
% Sensor Kinematics 


    ROcp14_113 = ROcp14_16*C13-S13*S5;
    ROcp14_213 = ROcp14_26*C13-ROcp14_85*S13;
    ROcp14_313 = ROcp14_36*C13-ROcp14_95*S13;
    ROcp14_713 = ROcp14_16*S13+C13*S5;
    ROcp14_813 = ROcp14_26*S13+ROcp14_85*C13;
    ROcp14_913 = ROcp14_36*S13+ROcp14_95*C13;
    ROcp14_414 = ROcp14_46*C14+ROcp14_713*S14;
    ROcp14_514 = ROcp14_56*C14+ROcp14_813*S14;
    ROcp14_614 = ROcp14_66*C14+ROcp14_913*S14;
    ROcp14_714 = -(ROcp14_46*S14-ROcp14_713*C14);
    ROcp14_814 = -(ROcp14_56*S14-ROcp14_813*C14);
    ROcp14_914 = -(ROcp14_66*S14-ROcp14_913*C14);
    ROcp14_115 = ROcp14_113*C15+ROcp14_414*S15;
    ROcp14_215 = ROcp14_213*C15+ROcp14_514*S15;
    ROcp14_315 = ROcp14_313*C15+ROcp14_614*S15;
    ROcp14_415 = -(ROcp14_113*S15-ROcp14_414*C15);
    ROcp14_515 = -(ROcp14_213*S15-ROcp14_514*C15);
    ROcp14_615 = -(ROcp14_313*S15-ROcp14_614*C15);
    RLcp14_113 = s.dpt(1,2)*ROcp14_16+s.dpt(3,2)*S5+ROcp14_46*s.dpt(2,2);
    RLcp14_213 = s.dpt(1,2)*ROcp14_26+s.dpt(3,2)*ROcp14_85+ROcp14_56*s.dpt(2,2);
    RLcp14_313 = s.dpt(1,2)*ROcp14_36+s.dpt(3,2)*ROcp14_95+ROcp14_66*s.dpt(2,2);
    OMcp14_113 = OMcp14_16+ROcp14_46*qd(13);
    OMcp14_213 = OMcp14_26+ROcp14_56*qd(13);
    OMcp14_313 = OMcp14_36+ROcp14_66*qd(13);
    ORcp14_113 = OMcp14_26*RLcp14_313-OMcp14_36*RLcp14_213;
    ORcp14_213 = -(OMcp14_16*RLcp14_313-OMcp14_36*RLcp14_113);
    ORcp14_313 = OMcp14_16*RLcp14_213-OMcp14_26*RLcp14_113;
    OPcp14_113 = OPcp14_16+ROcp14_46*qdd(13)+qd(13)*(OMcp14_26*ROcp14_66-OMcp14_36*ROcp14_56);
    OPcp14_213 = OPcp14_26+ROcp14_56*qdd(13)-qd(13)*(OMcp14_16*ROcp14_66-OMcp14_36*ROcp14_46);
    OPcp14_313 = OPcp14_36+ROcp14_66*qdd(13)+qd(13)*(OMcp14_16*ROcp14_56-OMcp14_26*ROcp14_46);
    RLcp14_114 = s.dpt(1,18)*ROcp14_113+s.dpt(3,18)*ROcp14_713+ROcp14_46*s.dpt(2,18);
    RLcp14_214 = s.dpt(1,18)*ROcp14_213+s.dpt(3,18)*ROcp14_813+ROcp14_56*s.dpt(2,18);
    RLcp14_314 = s.dpt(1,18)*ROcp14_313+s.dpt(3,18)*ROcp14_913+ROcp14_66*s.dpt(2,18);
    OMcp14_114 = OMcp14_113+ROcp14_113*qd(14);
    OMcp14_214 = OMcp14_213+ROcp14_213*qd(14);
    OMcp14_314 = OMcp14_313+ROcp14_313*qd(14);
    ORcp14_114 = OMcp14_213*RLcp14_314-OMcp14_313*RLcp14_214;
    ORcp14_214 = -(OMcp14_113*RLcp14_314-OMcp14_313*RLcp14_114);
    ORcp14_314 = OMcp14_113*RLcp14_214-OMcp14_213*RLcp14_114;
    OPcp14_114 = OPcp14_113+ROcp14_113*qdd(14)+qd(14)*(OMcp14_213*ROcp14_313-OMcp14_313*ROcp14_213);
    OPcp14_214 = OPcp14_213+ROcp14_213*qdd(14)-qd(14)*(OMcp14_113*ROcp14_313-OMcp14_313*ROcp14_113);
    OPcp14_314 = OPcp14_313+ROcp14_313*qdd(14)+qd(14)*(OMcp14_113*ROcp14_213-OMcp14_213*ROcp14_113);
    RLcp14_115 = s.dpt(1,20)*ROcp14_113+s.dpt(2,20)*ROcp14_414+ROcp14_714*s.dpt(3,20);
    RLcp14_215 = s.dpt(1,20)*ROcp14_213+s.dpt(2,20)*ROcp14_514+ROcp14_814*s.dpt(3,20);
    RLcp14_315 = s.dpt(1,20)*ROcp14_313+s.dpt(2,20)*ROcp14_614+ROcp14_914*s.dpt(3,20);
    POcp14_115 = RLcp14_113+RLcp14_114+RLcp14_115+q(1);
    POcp14_215 = RLcp14_213+RLcp14_214+RLcp14_215+q(2);
    POcp14_315 = RLcp14_313+RLcp14_314+RLcp14_315+q(3);
    OMcp14_115 = OMcp14_114+ROcp14_714*qd(15);
    OMcp14_215 = OMcp14_214+ROcp14_814*qd(15);
    OMcp14_315 = OMcp14_314+ROcp14_914*qd(15);
    ORcp14_115 = OMcp14_214*RLcp14_315-OMcp14_314*RLcp14_215;
    ORcp14_215 = -(OMcp14_114*RLcp14_315-OMcp14_314*RLcp14_115);
    ORcp14_315 = OMcp14_114*RLcp14_215-OMcp14_214*RLcp14_115;
    VIcp14_115 = ORcp14_113+ORcp14_114+ORcp14_115+qd(1);
    VIcp14_215 = ORcp14_213+ORcp14_214+ORcp14_215+qd(2);
    VIcp14_315 = ORcp14_313+ORcp14_314+ORcp14_315+qd(3);
    OPcp14_115 = OPcp14_114+ROcp14_714*qdd(15)+qd(15)*(OMcp14_214*ROcp14_914-OMcp14_314*ROcp14_814);
    OPcp14_215 = OPcp14_214+ROcp14_814*qdd(15)-qd(15)*(OMcp14_114*ROcp14_914-OMcp14_314*ROcp14_714);
    OPcp14_315 = OPcp14_314+ROcp14_914*qdd(15)+qd(15)*(OMcp14_114*ROcp14_814-OMcp14_214*ROcp14_714);
    ACcp14_115 = qdd(1)+OMcp14_213*ORcp14_314+OMcp14_214*ORcp14_315+OMcp14_26*ORcp14_313-OMcp14_313*ORcp14_214-OMcp14_314*ORcp14_215-OMcp14_36*...
 ORcp14_213+OPcp14_213*RLcp14_314+OPcp14_214*RLcp14_315+OPcp14_26*RLcp14_313-OPcp14_313*RLcp14_214-OPcp14_314*RLcp14_215-OPcp14_36*RLcp14_213;
    ACcp14_215 = qdd(2)-OMcp14_113*ORcp14_314-OMcp14_114*ORcp14_315-OMcp14_16*ORcp14_313+OMcp14_313*ORcp14_114+OMcp14_314*ORcp14_115+OMcp14_36*...
 ORcp14_113-OPcp14_113*RLcp14_314-OPcp14_114*RLcp14_315-OPcp14_16*RLcp14_313+OPcp14_313*RLcp14_114+OPcp14_314*RLcp14_115+OPcp14_36*RLcp14_113;
    ACcp14_315 = qdd(3)+OMcp14_113*ORcp14_214+OMcp14_114*ORcp14_215+OMcp14_16*ORcp14_213-OMcp14_213*ORcp14_114-OMcp14_214*ORcp14_115-OMcp14_26*...
 ORcp14_113+OPcp14_113*RLcp14_214+OPcp14_114*RLcp14_215+OPcp14_16*RLcp14_213-OPcp14_213*RLcp14_114-OPcp14_214*RLcp14_115-OPcp14_26*RLcp14_113;

% = = Block_1_0_0_15_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp14_115;
    sens.P(2) = POcp14_215;
    sens.P(3) = POcp14_315;
    sens.R(1,1) = ROcp14_115;
    sens.R(1,2) = ROcp14_215;
    sens.R(1,3) = ROcp14_315;
    sens.R(2,1) = ROcp14_415;
    sens.R(2,2) = ROcp14_515;
    sens.R(2,3) = ROcp14_615;
    sens.R(3,1) = ROcp14_714;
    sens.R(3,2) = ROcp14_814;
    sens.R(3,3) = ROcp14_914;
    sens.V(1) = VIcp14_115;
    sens.V(2) = VIcp14_215;
    sens.V(3) = VIcp14_315;
    sens.OM(1) = OMcp14_115;
    sens.OM(2) = OMcp14_215;
    sens.OM(3) = OMcp14_315;
    sens.A(1) = ACcp14_115;
    sens.A(2) = ACcp14_215;
    sens.A(3) = ACcp14_315;
    sens.OMP(1) = OPcp14_115;
    sens.OMP(2) = OPcp14_215;
    sens.OMP(3) = OPcp14_315;
 
% 
case 16, 


% = = Block_1_0_0_16_0_1 = = 
 
% Sensor Kinematics 


    ROcp15_25 = S4*S5;
    ROcp15_35 = -C4*S5;
    ROcp15_85 = -S4*C5;
    ROcp15_95 = C4*C5;
    ROcp15_16 = C5*C6;
    ROcp15_26 = ROcp15_25*C6+C4*S6;
    ROcp15_36 = ROcp15_35*C6+S4*S6;
    ROcp15_46 = -C5*S6;
    ROcp15_56 = -(ROcp15_25*S6-C4*C6);
    ROcp15_66 = -(ROcp15_35*S6-S4*C6);
    OMcp15_25 = qd(5)*C4;
    OMcp15_35 = qd(5)*S4;
    OMcp15_16 = qd(4)+qd(6)*S5;
    OMcp15_26 = OMcp15_25+ROcp15_85*qd(6);
    OMcp15_36 = OMcp15_35+ROcp15_95*qd(6);
    OPcp15_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp15_26 = ROcp15_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp15_35*S5-ROcp15_95*qd(4));
    OPcp15_36 = ROcp15_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp15_25*S5-ROcp15_85*qd(4));

% = = Block_1_0_0_16_0_3 = = 
 
% Sensor Kinematics 


    ROcp15_113 = ROcp15_16*C13-S13*S5;
    ROcp15_213 = ROcp15_26*C13-ROcp15_85*S13;
    ROcp15_313 = ROcp15_36*C13-ROcp15_95*S13;
    ROcp15_713 = ROcp15_16*S13+C13*S5;
    ROcp15_813 = ROcp15_26*S13+ROcp15_85*C13;
    ROcp15_913 = ROcp15_36*S13+ROcp15_95*C13;
    ROcp15_414 = ROcp15_46*C14+ROcp15_713*S14;
    ROcp15_514 = ROcp15_56*C14+ROcp15_813*S14;
    ROcp15_614 = ROcp15_66*C14+ROcp15_913*S14;
    ROcp15_714 = -(ROcp15_46*S14-ROcp15_713*C14);
    ROcp15_814 = -(ROcp15_56*S14-ROcp15_813*C14);
    ROcp15_914 = -(ROcp15_66*S14-ROcp15_913*C14);
    ROcp15_115 = ROcp15_113*C15+ROcp15_414*S15;
    ROcp15_215 = ROcp15_213*C15+ROcp15_514*S15;
    ROcp15_315 = ROcp15_313*C15+ROcp15_614*S15;
    ROcp15_415 = -(ROcp15_113*S15-ROcp15_414*C15);
    ROcp15_515 = -(ROcp15_213*S15-ROcp15_514*C15);
    ROcp15_615 = -(ROcp15_313*S15-ROcp15_614*C15);
    ROcp15_116 = ROcp15_115*C16-ROcp15_714*S16;
    ROcp15_216 = ROcp15_215*C16-ROcp15_814*S16;
    ROcp15_316 = ROcp15_315*C16-ROcp15_914*S16;
    ROcp15_716 = ROcp15_115*S16+ROcp15_714*C16;
    ROcp15_816 = ROcp15_215*S16+ROcp15_814*C16;
    ROcp15_916 = ROcp15_315*S16+ROcp15_914*C16;
    RLcp15_113 = s.dpt(1,2)*ROcp15_16+s.dpt(3,2)*S5+ROcp15_46*s.dpt(2,2);
    RLcp15_213 = s.dpt(1,2)*ROcp15_26+s.dpt(3,2)*ROcp15_85+ROcp15_56*s.dpt(2,2);
    RLcp15_313 = s.dpt(1,2)*ROcp15_36+s.dpt(3,2)*ROcp15_95+ROcp15_66*s.dpt(2,2);
    OMcp15_113 = OMcp15_16+ROcp15_46*qd(13);
    OMcp15_213 = OMcp15_26+ROcp15_56*qd(13);
    OMcp15_313 = OMcp15_36+ROcp15_66*qd(13);
    ORcp15_113 = OMcp15_26*RLcp15_313-OMcp15_36*RLcp15_213;
    ORcp15_213 = -(OMcp15_16*RLcp15_313-OMcp15_36*RLcp15_113);
    ORcp15_313 = OMcp15_16*RLcp15_213-OMcp15_26*RLcp15_113;
    OPcp15_113 = OPcp15_16+ROcp15_46*qdd(13)+qd(13)*(OMcp15_26*ROcp15_66-OMcp15_36*ROcp15_56);
    OPcp15_213 = OPcp15_26+ROcp15_56*qdd(13)-qd(13)*(OMcp15_16*ROcp15_66-OMcp15_36*ROcp15_46);
    OPcp15_313 = OPcp15_36+ROcp15_66*qdd(13)+qd(13)*(OMcp15_16*ROcp15_56-OMcp15_26*ROcp15_46);
    RLcp15_114 = s.dpt(1,18)*ROcp15_113+s.dpt(3,18)*ROcp15_713+ROcp15_46*s.dpt(2,18);
    RLcp15_214 = s.dpt(1,18)*ROcp15_213+s.dpt(3,18)*ROcp15_813+ROcp15_56*s.dpt(2,18);
    RLcp15_314 = s.dpt(1,18)*ROcp15_313+s.dpt(3,18)*ROcp15_913+ROcp15_66*s.dpt(2,18);
    OMcp15_114 = OMcp15_113+ROcp15_113*qd(14);
    OMcp15_214 = OMcp15_213+ROcp15_213*qd(14);
    OMcp15_314 = OMcp15_313+ROcp15_313*qd(14);
    ORcp15_114 = OMcp15_213*RLcp15_314-OMcp15_313*RLcp15_214;
    ORcp15_214 = -(OMcp15_113*RLcp15_314-OMcp15_313*RLcp15_114);
    ORcp15_314 = OMcp15_113*RLcp15_214-OMcp15_213*RLcp15_114;
    OPcp15_114 = OPcp15_113+ROcp15_113*qdd(14)+qd(14)*(OMcp15_213*ROcp15_313-OMcp15_313*ROcp15_213);
    OPcp15_214 = OPcp15_213+ROcp15_213*qdd(14)-qd(14)*(OMcp15_113*ROcp15_313-OMcp15_313*ROcp15_113);
    OPcp15_314 = OPcp15_313+ROcp15_313*qdd(14)+qd(14)*(OMcp15_113*ROcp15_213-OMcp15_213*ROcp15_113);
    RLcp15_115 = s.dpt(1,20)*ROcp15_113+s.dpt(2,20)*ROcp15_414+ROcp15_714*s.dpt(3,20);
    RLcp15_215 = s.dpt(1,20)*ROcp15_213+s.dpt(2,20)*ROcp15_514+ROcp15_814*s.dpt(3,20);
    RLcp15_315 = s.dpt(1,20)*ROcp15_313+s.dpt(2,20)*ROcp15_614+ROcp15_914*s.dpt(3,20);
    OMcp15_115 = OMcp15_114+ROcp15_714*qd(15);
    OMcp15_215 = OMcp15_214+ROcp15_814*qd(15);
    OMcp15_315 = OMcp15_314+ROcp15_914*qd(15);
    ORcp15_115 = OMcp15_214*RLcp15_315-OMcp15_314*RLcp15_215;
    ORcp15_215 = -(OMcp15_114*RLcp15_315-OMcp15_314*RLcp15_115);
    ORcp15_315 = OMcp15_114*RLcp15_215-OMcp15_214*RLcp15_115;
    OPcp15_115 = OPcp15_114+ROcp15_714*qdd(15)+qd(15)*(OMcp15_214*ROcp15_914-OMcp15_314*ROcp15_814);
    OPcp15_215 = OPcp15_214+ROcp15_814*qdd(15)-qd(15)*(OMcp15_114*ROcp15_914-OMcp15_314*ROcp15_714);
    OPcp15_315 = OPcp15_314+ROcp15_914*qdd(15)+qd(15)*(OMcp15_114*ROcp15_814-OMcp15_214*ROcp15_714);
    RLcp15_116 = s.dpt(1,22)*ROcp15_115+s.dpt(2,22)*ROcp15_415+ROcp15_714*s.dpt(3,22);
    RLcp15_216 = s.dpt(1,22)*ROcp15_215+s.dpt(2,22)*ROcp15_515+ROcp15_814*s.dpt(3,22);
    RLcp15_316 = s.dpt(1,22)*ROcp15_315+s.dpt(2,22)*ROcp15_615+ROcp15_914*s.dpt(3,22);
    POcp15_116 = RLcp15_113+RLcp15_114+RLcp15_115+RLcp15_116+q(1);
    POcp15_216 = RLcp15_213+RLcp15_214+RLcp15_215+RLcp15_216+q(2);
    POcp15_316 = RLcp15_313+RLcp15_314+RLcp15_315+RLcp15_316+q(3);
    OMcp15_116 = OMcp15_115+ROcp15_415*qd(16);
    OMcp15_216 = OMcp15_215+ROcp15_515*qd(16);
    OMcp15_316 = OMcp15_315+ROcp15_615*qd(16);
    ORcp15_116 = OMcp15_215*RLcp15_316-OMcp15_315*RLcp15_216;
    ORcp15_216 = -(OMcp15_115*RLcp15_316-OMcp15_315*RLcp15_116);
    ORcp15_316 = OMcp15_115*RLcp15_216-OMcp15_215*RLcp15_116;
    VIcp15_116 = ORcp15_113+ORcp15_114+ORcp15_115+ORcp15_116+qd(1);
    VIcp15_216 = ORcp15_213+ORcp15_214+ORcp15_215+ORcp15_216+qd(2);
    VIcp15_316 = ORcp15_313+ORcp15_314+ORcp15_315+ORcp15_316+qd(3);
    OPcp15_116 = OPcp15_115+ROcp15_415*qdd(16)+qd(16)*(OMcp15_215*ROcp15_615-OMcp15_315*ROcp15_515);
    OPcp15_216 = OPcp15_215+ROcp15_515*qdd(16)-qd(16)*(OMcp15_115*ROcp15_615-OMcp15_315*ROcp15_415);
    OPcp15_316 = OPcp15_315+ROcp15_615*qdd(16)+qd(16)*(OMcp15_115*ROcp15_515-OMcp15_215*ROcp15_415);
    ACcp15_116 = qdd(1)+OMcp15_213*ORcp15_314+OMcp15_214*ORcp15_315+OMcp15_215*ORcp15_316+OMcp15_26*ORcp15_313-OMcp15_313*ORcp15_214-OMcp15_314*...
 ORcp15_215-OMcp15_315*ORcp15_216-OMcp15_36*ORcp15_213+OPcp15_213*RLcp15_314+OPcp15_214*RLcp15_315+OPcp15_215*RLcp15_316+OPcp15_26*RLcp15_313-...
 OPcp15_313*RLcp15_214-OPcp15_314*RLcp15_215-OPcp15_315*RLcp15_216-OPcp15_36*RLcp15_213;
    ACcp15_216 = qdd(2)-OMcp15_113*ORcp15_314-OMcp15_114*ORcp15_315-OMcp15_115*ORcp15_316-OMcp15_16*ORcp15_313+OMcp15_313*ORcp15_114+OMcp15_314*...
 ORcp15_115+OMcp15_315*ORcp15_116+OMcp15_36*ORcp15_113-OPcp15_113*RLcp15_314-OPcp15_114*RLcp15_315-OPcp15_115*RLcp15_316-OPcp15_16*RLcp15_313+...
 OPcp15_313*RLcp15_114+OPcp15_314*RLcp15_115+OPcp15_315*RLcp15_116+OPcp15_36*RLcp15_113;
    ACcp15_316 = qdd(3)+OMcp15_113*ORcp15_214+OMcp15_114*ORcp15_215+OMcp15_115*ORcp15_216+OMcp15_16*ORcp15_213-OMcp15_213*ORcp15_114-OMcp15_214*...
 ORcp15_115-OMcp15_215*ORcp15_116-OMcp15_26*ORcp15_113+OPcp15_113*RLcp15_214+OPcp15_114*RLcp15_215+OPcp15_115*RLcp15_216+OPcp15_16*RLcp15_213-...
 OPcp15_213*RLcp15_114-OPcp15_214*RLcp15_115-OPcp15_215*RLcp15_116-OPcp15_26*RLcp15_113;

% = = Block_1_0_0_16_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp15_116;
    sens.P(2) = POcp15_216;
    sens.P(3) = POcp15_316;
    sens.R(1,1) = ROcp15_116;
    sens.R(1,2) = ROcp15_216;
    sens.R(1,3) = ROcp15_316;
    sens.R(2,1) = ROcp15_415;
    sens.R(2,2) = ROcp15_515;
    sens.R(2,3) = ROcp15_615;
    sens.R(3,1) = ROcp15_716;
    sens.R(3,2) = ROcp15_816;
    sens.R(3,3) = ROcp15_916;
    sens.V(1) = VIcp15_116;
    sens.V(2) = VIcp15_216;
    sens.V(3) = VIcp15_316;
    sens.OM(1) = OMcp15_116;
    sens.OM(2) = OMcp15_216;
    sens.OM(3) = OMcp15_316;
    sens.A(1) = ACcp15_116;
    sens.A(2) = ACcp15_216;
    sens.A(3) = ACcp15_316;
    sens.OMP(1) = OPcp15_116;
    sens.OMP(2) = OPcp15_216;
    sens.OMP(3) = OPcp15_316;
 
% 
case 17, 


% = = Block_1_0_0_17_0_1 = = 
 
% Sensor Kinematics 


    ROcp16_25 = S4*S5;
    ROcp16_35 = -C4*S5;
    ROcp16_85 = -S4*C5;
    ROcp16_95 = C4*C5;
    ROcp16_16 = C5*C6;
    ROcp16_26 = ROcp16_25*C6+C4*S6;
    ROcp16_36 = ROcp16_35*C6+S4*S6;
    ROcp16_46 = -C5*S6;
    ROcp16_56 = -(ROcp16_25*S6-C4*C6);
    ROcp16_66 = -(ROcp16_35*S6-S4*C6);
    OMcp16_25 = qd(5)*C4;
    OMcp16_35 = qd(5)*S4;
    OMcp16_16 = qd(4)+qd(6)*S5;
    OMcp16_26 = OMcp16_25+ROcp16_85*qd(6);
    OMcp16_36 = OMcp16_35+ROcp16_95*qd(6);
    OPcp16_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp16_26 = ROcp16_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp16_35*S5-ROcp16_95*qd(4));
    OPcp16_36 = ROcp16_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp16_25*S5-ROcp16_85*qd(4));

% = = Block_1_0_0_17_0_3 = = 
 
% Sensor Kinematics 


    ROcp16_113 = ROcp16_16*C13-S13*S5;
    ROcp16_213 = ROcp16_26*C13-ROcp16_85*S13;
    ROcp16_313 = ROcp16_36*C13-ROcp16_95*S13;
    ROcp16_713 = ROcp16_16*S13+C13*S5;
    ROcp16_813 = ROcp16_26*S13+ROcp16_85*C13;
    ROcp16_913 = ROcp16_36*S13+ROcp16_95*C13;
    ROcp16_414 = ROcp16_46*C14+ROcp16_713*S14;
    ROcp16_514 = ROcp16_56*C14+ROcp16_813*S14;
    ROcp16_614 = ROcp16_66*C14+ROcp16_913*S14;
    ROcp16_714 = -(ROcp16_46*S14-ROcp16_713*C14);
    ROcp16_814 = -(ROcp16_56*S14-ROcp16_813*C14);
    ROcp16_914 = -(ROcp16_66*S14-ROcp16_913*C14);
    ROcp16_115 = ROcp16_113*C15+ROcp16_414*S15;
    ROcp16_215 = ROcp16_213*C15+ROcp16_514*S15;
    ROcp16_315 = ROcp16_313*C15+ROcp16_614*S15;
    ROcp16_415 = -(ROcp16_113*S15-ROcp16_414*C15);
    ROcp16_515 = -(ROcp16_213*S15-ROcp16_514*C15);
    ROcp16_615 = -(ROcp16_313*S15-ROcp16_614*C15);
    ROcp16_116 = ROcp16_115*C16-ROcp16_714*S16;
    ROcp16_216 = ROcp16_215*C16-ROcp16_814*S16;
    ROcp16_316 = ROcp16_315*C16-ROcp16_914*S16;
    ROcp16_716 = ROcp16_115*S16+ROcp16_714*C16;
    ROcp16_816 = ROcp16_215*S16+ROcp16_814*C16;
    ROcp16_916 = ROcp16_315*S16+ROcp16_914*C16;
    ROcp16_417 = ROcp16_415*C17+ROcp16_716*S17;
    ROcp16_517 = ROcp16_515*C17+ROcp16_816*S17;
    ROcp16_617 = ROcp16_615*C17+ROcp16_916*S17;
    ROcp16_717 = -(ROcp16_415*S17-ROcp16_716*C17);
    ROcp16_817 = -(ROcp16_515*S17-ROcp16_816*C17);
    ROcp16_917 = -(ROcp16_615*S17-ROcp16_916*C17);
    RLcp16_113 = s.dpt(1,2)*ROcp16_16+s.dpt(3,2)*S5+ROcp16_46*s.dpt(2,2);
    RLcp16_213 = s.dpt(1,2)*ROcp16_26+s.dpt(3,2)*ROcp16_85+ROcp16_56*s.dpt(2,2);
    RLcp16_313 = s.dpt(1,2)*ROcp16_36+s.dpt(3,2)*ROcp16_95+ROcp16_66*s.dpt(2,2);
    OMcp16_113 = OMcp16_16+ROcp16_46*qd(13);
    OMcp16_213 = OMcp16_26+ROcp16_56*qd(13);
    OMcp16_313 = OMcp16_36+ROcp16_66*qd(13);
    ORcp16_113 = OMcp16_26*RLcp16_313-OMcp16_36*RLcp16_213;
    ORcp16_213 = -(OMcp16_16*RLcp16_313-OMcp16_36*RLcp16_113);
    ORcp16_313 = OMcp16_16*RLcp16_213-OMcp16_26*RLcp16_113;
    OPcp16_113 = OPcp16_16+ROcp16_46*qdd(13)+qd(13)*(OMcp16_26*ROcp16_66-OMcp16_36*ROcp16_56);
    OPcp16_213 = OPcp16_26+ROcp16_56*qdd(13)-qd(13)*(OMcp16_16*ROcp16_66-OMcp16_36*ROcp16_46);
    OPcp16_313 = OPcp16_36+ROcp16_66*qdd(13)+qd(13)*(OMcp16_16*ROcp16_56-OMcp16_26*ROcp16_46);
    RLcp16_114 = s.dpt(1,18)*ROcp16_113+s.dpt(3,18)*ROcp16_713+ROcp16_46*s.dpt(2,18);
    RLcp16_214 = s.dpt(1,18)*ROcp16_213+s.dpt(3,18)*ROcp16_813+ROcp16_56*s.dpt(2,18);
    RLcp16_314 = s.dpt(1,18)*ROcp16_313+s.dpt(3,18)*ROcp16_913+ROcp16_66*s.dpt(2,18);
    OMcp16_114 = OMcp16_113+ROcp16_113*qd(14);
    OMcp16_214 = OMcp16_213+ROcp16_213*qd(14);
    OMcp16_314 = OMcp16_313+ROcp16_313*qd(14);
    ORcp16_114 = OMcp16_213*RLcp16_314-OMcp16_313*RLcp16_214;
    ORcp16_214 = -(OMcp16_113*RLcp16_314-OMcp16_313*RLcp16_114);
    ORcp16_314 = OMcp16_113*RLcp16_214-OMcp16_213*RLcp16_114;
    OPcp16_114 = OPcp16_113+ROcp16_113*qdd(14)+qd(14)*(OMcp16_213*ROcp16_313-OMcp16_313*ROcp16_213);
    OPcp16_214 = OPcp16_213+ROcp16_213*qdd(14)-qd(14)*(OMcp16_113*ROcp16_313-OMcp16_313*ROcp16_113);
    OPcp16_314 = OPcp16_313+ROcp16_313*qdd(14)+qd(14)*(OMcp16_113*ROcp16_213-OMcp16_213*ROcp16_113);
    RLcp16_115 = s.dpt(1,20)*ROcp16_113+s.dpt(2,20)*ROcp16_414+ROcp16_714*s.dpt(3,20);
    RLcp16_215 = s.dpt(1,20)*ROcp16_213+s.dpt(2,20)*ROcp16_514+ROcp16_814*s.dpt(3,20);
    RLcp16_315 = s.dpt(1,20)*ROcp16_313+s.dpt(2,20)*ROcp16_614+ROcp16_914*s.dpt(3,20);
    OMcp16_115 = OMcp16_114+ROcp16_714*qd(15);
    OMcp16_215 = OMcp16_214+ROcp16_814*qd(15);
    OMcp16_315 = OMcp16_314+ROcp16_914*qd(15);
    ORcp16_115 = OMcp16_214*RLcp16_315-OMcp16_314*RLcp16_215;
    ORcp16_215 = -(OMcp16_114*RLcp16_315-OMcp16_314*RLcp16_115);
    ORcp16_315 = OMcp16_114*RLcp16_215-OMcp16_214*RLcp16_115;
    OPcp16_115 = OPcp16_114+ROcp16_714*qdd(15)+qd(15)*(OMcp16_214*ROcp16_914-OMcp16_314*ROcp16_814);
    OPcp16_215 = OPcp16_214+ROcp16_814*qdd(15)-qd(15)*(OMcp16_114*ROcp16_914-OMcp16_314*ROcp16_714);
    OPcp16_315 = OPcp16_314+ROcp16_914*qdd(15)+qd(15)*(OMcp16_114*ROcp16_814-OMcp16_214*ROcp16_714);
    RLcp16_116 = s.dpt(1,22)*ROcp16_115+s.dpt(2,22)*ROcp16_415+ROcp16_714*s.dpt(3,22);
    RLcp16_216 = s.dpt(1,22)*ROcp16_215+s.dpt(2,22)*ROcp16_515+ROcp16_814*s.dpt(3,22);
    RLcp16_316 = s.dpt(1,22)*ROcp16_315+s.dpt(2,22)*ROcp16_615+ROcp16_914*s.dpt(3,22);
    OMcp16_116 = OMcp16_115+ROcp16_415*qd(16);
    OMcp16_216 = OMcp16_215+ROcp16_515*qd(16);
    OMcp16_316 = OMcp16_315+ROcp16_615*qd(16);
    ORcp16_116 = OMcp16_215*RLcp16_316-OMcp16_315*RLcp16_216;
    ORcp16_216 = -(OMcp16_115*RLcp16_316-OMcp16_315*RLcp16_116);
    ORcp16_316 = OMcp16_115*RLcp16_216-OMcp16_215*RLcp16_116;
    OPcp16_116 = OPcp16_115+ROcp16_415*qdd(16)+qd(16)*(OMcp16_215*ROcp16_615-OMcp16_315*ROcp16_515);
    OPcp16_216 = OPcp16_215+ROcp16_515*qdd(16)-qd(16)*(OMcp16_115*ROcp16_615-OMcp16_315*ROcp16_415);
    OPcp16_316 = OPcp16_315+ROcp16_615*qdd(16)+qd(16)*(OMcp16_115*ROcp16_515-OMcp16_215*ROcp16_415);
    RLcp16_117 = s.dpt(1,24)*ROcp16_116+s.dpt(2,24)*ROcp16_415+ROcp16_716*s.dpt(3,24);
    RLcp16_217 = s.dpt(1,24)*ROcp16_216+s.dpt(2,24)*ROcp16_515+ROcp16_816*s.dpt(3,24);
    RLcp16_317 = s.dpt(1,24)*ROcp16_316+s.dpt(2,24)*ROcp16_615+ROcp16_916*s.dpt(3,24);
    POcp16_117 = RLcp16_113+RLcp16_114+RLcp16_115+RLcp16_116+RLcp16_117+q(1);
    POcp16_217 = RLcp16_213+RLcp16_214+RLcp16_215+RLcp16_216+RLcp16_217+q(2);
    POcp16_317 = RLcp16_313+RLcp16_314+RLcp16_315+RLcp16_316+RLcp16_317+q(3);
    OMcp16_117 = OMcp16_116+ROcp16_116*qd(17);
    OMcp16_217 = OMcp16_216+ROcp16_216*qd(17);
    OMcp16_317 = OMcp16_316+ROcp16_316*qd(17);
    ORcp16_117 = OMcp16_216*RLcp16_317-OMcp16_316*RLcp16_217;
    ORcp16_217 = -(OMcp16_116*RLcp16_317-OMcp16_316*RLcp16_117);
    ORcp16_317 = OMcp16_116*RLcp16_217-OMcp16_216*RLcp16_117;
    VIcp16_117 = ORcp16_113+ORcp16_114+ORcp16_115+ORcp16_116+ORcp16_117+qd(1);
    VIcp16_217 = ORcp16_213+ORcp16_214+ORcp16_215+ORcp16_216+ORcp16_217+qd(2);
    VIcp16_317 = ORcp16_313+ORcp16_314+ORcp16_315+ORcp16_316+ORcp16_317+qd(3);
    OPcp16_117 = OPcp16_116+ROcp16_116*qdd(17)+qd(17)*(OMcp16_216*ROcp16_316-OMcp16_316*ROcp16_216);
    OPcp16_217 = OPcp16_216+ROcp16_216*qdd(17)-qd(17)*(OMcp16_116*ROcp16_316-OMcp16_316*ROcp16_116);
    OPcp16_317 = OPcp16_316+ROcp16_316*qdd(17)+qd(17)*(OMcp16_116*ROcp16_216-OMcp16_216*ROcp16_116);
    ACcp16_117 = qdd(1)+OMcp16_213*ORcp16_314+OMcp16_214*ORcp16_315+OMcp16_215*ORcp16_316+OMcp16_216*ORcp16_317+OMcp16_26*ORcp16_313-OMcp16_313*...
 ORcp16_214-OMcp16_314*ORcp16_215-OMcp16_315*ORcp16_216-OMcp16_316*ORcp16_217-OMcp16_36*ORcp16_213+OPcp16_213*RLcp16_314+OPcp16_214*RLcp16_315+...
 OPcp16_215*RLcp16_316+OPcp16_216*RLcp16_317+OPcp16_26*RLcp16_313-OPcp16_313*RLcp16_214-OPcp16_314*RLcp16_215-OPcp16_315*RLcp16_216-OPcp16_316*...
 RLcp16_217-OPcp16_36*RLcp16_213;
    ACcp16_217 = qdd(2)-OMcp16_113*ORcp16_314-OMcp16_114*ORcp16_315-OMcp16_115*ORcp16_316-OMcp16_116*ORcp16_317-OMcp16_16*ORcp16_313+OMcp16_313*...
 ORcp16_114+OMcp16_314*ORcp16_115+OMcp16_315*ORcp16_116+OMcp16_316*ORcp16_117+OMcp16_36*ORcp16_113-OPcp16_113*RLcp16_314-OPcp16_114*RLcp16_315-...
 OPcp16_115*RLcp16_316-OPcp16_116*RLcp16_317-OPcp16_16*RLcp16_313+OPcp16_313*RLcp16_114+OPcp16_314*RLcp16_115+OPcp16_315*RLcp16_116+OPcp16_316*...
 RLcp16_117+OPcp16_36*RLcp16_113;
    ACcp16_317 = qdd(3)+OMcp16_113*ORcp16_214+OMcp16_114*ORcp16_215+OMcp16_115*ORcp16_216+OMcp16_116*ORcp16_217+OMcp16_16*ORcp16_213-OMcp16_213*...
 ORcp16_114-OMcp16_214*ORcp16_115-OMcp16_215*ORcp16_116-OMcp16_216*ORcp16_117-OMcp16_26*ORcp16_113+OPcp16_113*RLcp16_214+OPcp16_114*RLcp16_215+...
 OPcp16_115*RLcp16_216+OPcp16_116*RLcp16_217+OPcp16_16*RLcp16_213-OPcp16_213*RLcp16_114-OPcp16_214*RLcp16_115-OPcp16_215*RLcp16_116-OPcp16_216*...
 RLcp16_117-OPcp16_26*RLcp16_113;

% = = Block_1_0_0_17_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp16_117;
    sens.P(2) = POcp16_217;
    sens.P(3) = POcp16_317;
    sens.R(1,1) = ROcp16_116;
    sens.R(1,2) = ROcp16_216;
    sens.R(1,3) = ROcp16_316;
    sens.R(2,1) = ROcp16_417;
    sens.R(2,2) = ROcp16_517;
    sens.R(2,3) = ROcp16_617;
    sens.R(3,1) = ROcp16_717;
    sens.R(3,2) = ROcp16_817;
    sens.R(3,3) = ROcp16_917;
    sens.V(1) = VIcp16_117;
    sens.V(2) = VIcp16_217;
    sens.V(3) = VIcp16_317;
    sens.OM(1) = OMcp16_117;
    sens.OM(2) = OMcp16_217;
    sens.OM(3) = OMcp16_317;
    sens.A(1) = ACcp16_117;
    sens.A(2) = ACcp16_217;
    sens.A(3) = ACcp16_317;
    sens.OMP(1) = OPcp16_117;
    sens.OMP(2) = OPcp16_217;
    sens.OMP(3) = OPcp16_317;
 
% 
case 18, 


% = = Block_1_0_0_18_0_1 = = 
 
% Sensor Kinematics 


    ROcp17_25 = S4*S5;
    ROcp17_35 = -C4*S5;
    ROcp17_85 = -S4*C5;
    ROcp17_95 = C4*C5;
    ROcp17_16 = C5*C6;
    ROcp17_26 = ROcp17_25*C6+C4*S6;
    ROcp17_36 = ROcp17_35*C6+S4*S6;
    ROcp17_46 = -C5*S6;
    ROcp17_56 = -(ROcp17_25*S6-C4*C6);
    ROcp17_66 = -(ROcp17_35*S6-S4*C6);
    OMcp17_25 = qd(5)*C4;
    OMcp17_35 = qd(5)*S4;
    OMcp17_16 = qd(4)+qd(6)*S5;
    OMcp17_26 = OMcp17_25+ROcp17_85*qd(6);
    OMcp17_36 = OMcp17_35+ROcp17_95*qd(6);
    OPcp17_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp17_26 = ROcp17_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp17_35*S5-ROcp17_95*qd(4));
    OPcp17_36 = ROcp17_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp17_25*S5-ROcp17_85*qd(4));

% = = Block_1_0_0_18_0_3 = = 
 
% Sensor Kinematics 


    ROcp17_113 = ROcp17_16*C13-S13*S5;
    ROcp17_213 = ROcp17_26*C13-ROcp17_85*S13;
    ROcp17_313 = ROcp17_36*C13-ROcp17_95*S13;
    ROcp17_713 = ROcp17_16*S13+C13*S5;
    ROcp17_813 = ROcp17_26*S13+ROcp17_85*C13;
    ROcp17_913 = ROcp17_36*S13+ROcp17_95*C13;
    ROcp17_414 = ROcp17_46*C14+ROcp17_713*S14;
    ROcp17_514 = ROcp17_56*C14+ROcp17_813*S14;
    ROcp17_614 = ROcp17_66*C14+ROcp17_913*S14;
    ROcp17_714 = -(ROcp17_46*S14-ROcp17_713*C14);
    ROcp17_814 = -(ROcp17_56*S14-ROcp17_813*C14);
    ROcp17_914 = -(ROcp17_66*S14-ROcp17_913*C14);
    ROcp17_115 = ROcp17_113*C15+ROcp17_414*S15;
    ROcp17_215 = ROcp17_213*C15+ROcp17_514*S15;
    ROcp17_315 = ROcp17_313*C15+ROcp17_614*S15;
    ROcp17_415 = -(ROcp17_113*S15-ROcp17_414*C15);
    ROcp17_515 = -(ROcp17_213*S15-ROcp17_514*C15);
    ROcp17_615 = -(ROcp17_313*S15-ROcp17_614*C15);
    ROcp17_116 = ROcp17_115*C16-ROcp17_714*S16;
    ROcp17_216 = ROcp17_215*C16-ROcp17_814*S16;
    ROcp17_316 = ROcp17_315*C16-ROcp17_914*S16;
    ROcp17_716 = ROcp17_115*S16+ROcp17_714*C16;
    ROcp17_816 = ROcp17_215*S16+ROcp17_814*C16;
    ROcp17_916 = ROcp17_315*S16+ROcp17_914*C16;
    ROcp17_417 = ROcp17_415*C17+ROcp17_716*S17;
    ROcp17_517 = ROcp17_515*C17+ROcp17_816*S17;
    ROcp17_617 = ROcp17_615*C17+ROcp17_916*S17;
    ROcp17_717 = -(ROcp17_415*S17-ROcp17_716*C17);
    ROcp17_817 = -(ROcp17_515*S17-ROcp17_816*C17);
    ROcp17_917 = -(ROcp17_615*S17-ROcp17_916*C17);
    ROcp17_118 = ROcp17_116*C18-ROcp17_717*S18;
    ROcp17_218 = ROcp17_216*C18-ROcp17_817*S18;
    ROcp17_318 = ROcp17_316*C18-ROcp17_917*S18;
    ROcp17_718 = ROcp17_116*S18+ROcp17_717*C18;
    ROcp17_818 = ROcp17_216*S18+ROcp17_817*C18;
    ROcp17_918 = ROcp17_316*S18+ROcp17_917*C18;
    RLcp17_113 = s.dpt(1,2)*ROcp17_16+s.dpt(3,2)*S5+ROcp17_46*s.dpt(2,2);
    RLcp17_213 = s.dpt(1,2)*ROcp17_26+s.dpt(3,2)*ROcp17_85+ROcp17_56*s.dpt(2,2);
    RLcp17_313 = s.dpt(1,2)*ROcp17_36+s.dpt(3,2)*ROcp17_95+ROcp17_66*s.dpt(2,2);
    OMcp17_113 = OMcp17_16+ROcp17_46*qd(13);
    OMcp17_213 = OMcp17_26+ROcp17_56*qd(13);
    OMcp17_313 = OMcp17_36+ROcp17_66*qd(13);
    ORcp17_113 = OMcp17_26*RLcp17_313-OMcp17_36*RLcp17_213;
    ORcp17_213 = -(OMcp17_16*RLcp17_313-OMcp17_36*RLcp17_113);
    ORcp17_313 = OMcp17_16*RLcp17_213-OMcp17_26*RLcp17_113;
    OPcp17_113 = OPcp17_16+ROcp17_46*qdd(13)+qd(13)*(OMcp17_26*ROcp17_66-OMcp17_36*ROcp17_56);
    OPcp17_213 = OPcp17_26+ROcp17_56*qdd(13)-qd(13)*(OMcp17_16*ROcp17_66-OMcp17_36*ROcp17_46);
    OPcp17_313 = OPcp17_36+ROcp17_66*qdd(13)+qd(13)*(OMcp17_16*ROcp17_56-OMcp17_26*ROcp17_46);
    RLcp17_114 = s.dpt(1,18)*ROcp17_113+s.dpt(3,18)*ROcp17_713+ROcp17_46*s.dpt(2,18);
    RLcp17_214 = s.dpt(1,18)*ROcp17_213+s.dpt(3,18)*ROcp17_813+ROcp17_56*s.dpt(2,18);
    RLcp17_314 = s.dpt(1,18)*ROcp17_313+s.dpt(3,18)*ROcp17_913+ROcp17_66*s.dpt(2,18);
    OMcp17_114 = OMcp17_113+ROcp17_113*qd(14);
    OMcp17_214 = OMcp17_213+ROcp17_213*qd(14);
    OMcp17_314 = OMcp17_313+ROcp17_313*qd(14);
    ORcp17_114 = OMcp17_213*RLcp17_314-OMcp17_313*RLcp17_214;
    ORcp17_214 = -(OMcp17_113*RLcp17_314-OMcp17_313*RLcp17_114);
    ORcp17_314 = OMcp17_113*RLcp17_214-OMcp17_213*RLcp17_114;
    OPcp17_114 = OPcp17_113+ROcp17_113*qdd(14)+qd(14)*(OMcp17_213*ROcp17_313-OMcp17_313*ROcp17_213);
    OPcp17_214 = OPcp17_213+ROcp17_213*qdd(14)-qd(14)*(OMcp17_113*ROcp17_313-OMcp17_313*ROcp17_113);
    OPcp17_314 = OPcp17_313+ROcp17_313*qdd(14)+qd(14)*(OMcp17_113*ROcp17_213-OMcp17_213*ROcp17_113);
    RLcp17_115 = s.dpt(1,20)*ROcp17_113+s.dpt(2,20)*ROcp17_414+ROcp17_714*s.dpt(3,20);
    RLcp17_215 = s.dpt(1,20)*ROcp17_213+s.dpt(2,20)*ROcp17_514+ROcp17_814*s.dpt(3,20);
    RLcp17_315 = s.dpt(1,20)*ROcp17_313+s.dpt(2,20)*ROcp17_614+ROcp17_914*s.dpt(3,20);
    OMcp17_115 = OMcp17_114+ROcp17_714*qd(15);
    OMcp17_215 = OMcp17_214+ROcp17_814*qd(15);
    OMcp17_315 = OMcp17_314+ROcp17_914*qd(15);
    ORcp17_115 = OMcp17_214*RLcp17_315-OMcp17_314*RLcp17_215;
    ORcp17_215 = -(OMcp17_114*RLcp17_315-OMcp17_314*RLcp17_115);
    ORcp17_315 = OMcp17_114*RLcp17_215-OMcp17_214*RLcp17_115;
    OPcp17_115 = OPcp17_114+ROcp17_714*qdd(15)+qd(15)*(OMcp17_214*ROcp17_914-OMcp17_314*ROcp17_814);
    OPcp17_215 = OPcp17_214+ROcp17_814*qdd(15)-qd(15)*(OMcp17_114*ROcp17_914-OMcp17_314*ROcp17_714);
    OPcp17_315 = OPcp17_314+ROcp17_914*qdd(15)+qd(15)*(OMcp17_114*ROcp17_814-OMcp17_214*ROcp17_714);
    RLcp17_116 = s.dpt(1,22)*ROcp17_115+s.dpt(2,22)*ROcp17_415+ROcp17_714*s.dpt(3,22);
    RLcp17_216 = s.dpt(1,22)*ROcp17_215+s.dpt(2,22)*ROcp17_515+ROcp17_814*s.dpt(3,22);
    RLcp17_316 = s.dpt(1,22)*ROcp17_315+s.dpt(2,22)*ROcp17_615+ROcp17_914*s.dpt(3,22);
    OMcp17_116 = OMcp17_115+ROcp17_415*qd(16);
    OMcp17_216 = OMcp17_215+ROcp17_515*qd(16);
    OMcp17_316 = OMcp17_315+ROcp17_615*qd(16);
    ORcp17_116 = OMcp17_215*RLcp17_316-OMcp17_315*RLcp17_216;
    ORcp17_216 = -(OMcp17_115*RLcp17_316-OMcp17_315*RLcp17_116);
    ORcp17_316 = OMcp17_115*RLcp17_216-OMcp17_215*RLcp17_116;
    OPcp17_116 = OPcp17_115+ROcp17_415*qdd(16)+qd(16)*(OMcp17_215*ROcp17_615-OMcp17_315*ROcp17_515);
    OPcp17_216 = OPcp17_215+ROcp17_515*qdd(16)-qd(16)*(OMcp17_115*ROcp17_615-OMcp17_315*ROcp17_415);
    OPcp17_316 = OPcp17_315+ROcp17_615*qdd(16)+qd(16)*(OMcp17_115*ROcp17_515-OMcp17_215*ROcp17_415);
    RLcp17_117 = s.dpt(1,24)*ROcp17_116+s.dpt(2,24)*ROcp17_415+ROcp17_716*s.dpt(3,24);
    RLcp17_217 = s.dpt(1,24)*ROcp17_216+s.dpt(2,24)*ROcp17_515+ROcp17_816*s.dpt(3,24);
    RLcp17_317 = s.dpt(1,24)*ROcp17_316+s.dpt(2,24)*ROcp17_615+ROcp17_916*s.dpt(3,24);
    OMcp17_117 = OMcp17_116+ROcp17_116*qd(17);
    OMcp17_217 = OMcp17_216+ROcp17_216*qd(17);
    OMcp17_317 = OMcp17_316+ROcp17_316*qd(17);
    ORcp17_117 = OMcp17_216*RLcp17_317-OMcp17_316*RLcp17_217;
    ORcp17_217 = -(OMcp17_116*RLcp17_317-OMcp17_316*RLcp17_117);
    ORcp17_317 = OMcp17_116*RLcp17_217-OMcp17_216*RLcp17_117;
    OPcp17_117 = OPcp17_116+ROcp17_116*qdd(17)+qd(17)*(OMcp17_216*ROcp17_316-OMcp17_316*ROcp17_216);
    OPcp17_217 = OPcp17_216+ROcp17_216*qdd(17)-qd(17)*(OMcp17_116*ROcp17_316-OMcp17_316*ROcp17_116);
    OPcp17_317 = OPcp17_316+ROcp17_316*qdd(17)+qd(17)*(OMcp17_116*ROcp17_216-OMcp17_216*ROcp17_116);
    RLcp17_118 = s.dpt(1,26)*ROcp17_116+s.dpt(2,26)*ROcp17_417+s.dpt(3,26)*ROcp17_717;
    RLcp17_218 = s.dpt(1,26)*ROcp17_216+s.dpt(2,26)*ROcp17_517+s.dpt(3,26)*ROcp17_817;
    RLcp17_318 = s.dpt(1,26)*ROcp17_316+s.dpt(2,26)*ROcp17_617+s.dpt(3,26)*ROcp17_917;
    POcp17_118 = RLcp17_113+RLcp17_114+RLcp17_115+RLcp17_116+RLcp17_117+RLcp17_118+q(1);
    POcp17_218 = RLcp17_213+RLcp17_214+RLcp17_215+RLcp17_216+RLcp17_217+RLcp17_218+q(2);
    POcp17_318 = RLcp17_313+RLcp17_314+RLcp17_315+RLcp17_316+RLcp17_317+RLcp17_318+q(3);
    OMcp17_118 = OMcp17_117+ROcp17_417*qd(18);
    OMcp17_218 = OMcp17_217+ROcp17_517*qd(18);
    OMcp17_318 = OMcp17_317+ROcp17_617*qd(18);
    ORcp17_118 = OMcp17_217*RLcp17_318-OMcp17_317*RLcp17_218;
    ORcp17_218 = -(OMcp17_117*RLcp17_318-OMcp17_317*RLcp17_118);
    ORcp17_318 = OMcp17_117*RLcp17_218-OMcp17_217*RLcp17_118;
    VIcp17_118 = ORcp17_113+ORcp17_114+ORcp17_115+ORcp17_116+ORcp17_117+ORcp17_118+qd(1);
    VIcp17_218 = ORcp17_213+ORcp17_214+ORcp17_215+ORcp17_216+ORcp17_217+ORcp17_218+qd(2);
    VIcp17_318 = ORcp17_313+ORcp17_314+ORcp17_315+ORcp17_316+ORcp17_317+ORcp17_318+qd(3);
    OPcp17_118 = OPcp17_117+ROcp17_417*qdd(18)+qd(18)*(OMcp17_217*ROcp17_617-OMcp17_317*ROcp17_517);
    OPcp17_218 = OPcp17_217+ROcp17_517*qdd(18)-qd(18)*(OMcp17_117*ROcp17_617-OMcp17_317*ROcp17_417);
    OPcp17_318 = OPcp17_317+ROcp17_617*qdd(18)+qd(18)*(OMcp17_117*ROcp17_517-OMcp17_217*ROcp17_417);
    ACcp17_118 = qdd(1)+OMcp17_213*ORcp17_314+OMcp17_214*ORcp17_315+OMcp17_215*ORcp17_316+OMcp17_216*ORcp17_317+OMcp17_217*ORcp17_318+OMcp17_26*...
 ORcp17_313-OMcp17_313*ORcp17_214-OMcp17_314*ORcp17_215-OMcp17_315*ORcp17_216-OMcp17_316*ORcp17_217-OMcp17_317*ORcp17_218-OMcp17_36*ORcp17_213+...
 OPcp17_213*RLcp17_314+OPcp17_214*RLcp17_315+OPcp17_215*RLcp17_316+OPcp17_216*RLcp17_317+OPcp17_217*RLcp17_318+OPcp17_26*RLcp17_313-OPcp17_313*...
 RLcp17_214-OPcp17_314*RLcp17_215-OPcp17_315*RLcp17_216-OPcp17_316*RLcp17_217-OPcp17_317*RLcp17_218-OPcp17_36*RLcp17_213;
    ACcp17_218 = qdd(2)-OMcp17_113*ORcp17_314-OMcp17_114*ORcp17_315-OMcp17_115*ORcp17_316-OMcp17_116*ORcp17_317-OMcp17_117*ORcp17_318-OMcp17_16*...
 ORcp17_313+OMcp17_313*ORcp17_114+OMcp17_314*ORcp17_115+OMcp17_315*ORcp17_116+OMcp17_316*ORcp17_117+OMcp17_317*ORcp17_118+OMcp17_36*ORcp17_113-...
 OPcp17_113*RLcp17_314-OPcp17_114*RLcp17_315-OPcp17_115*RLcp17_316-OPcp17_116*RLcp17_317-OPcp17_117*RLcp17_318-OPcp17_16*RLcp17_313+OPcp17_313*...
 RLcp17_114+OPcp17_314*RLcp17_115+OPcp17_315*RLcp17_116+OPcp17_316*RLcp17_117+OPcp17_317*RLcp17_118+OPcp17_36*RLcp17_113;
    ACcp17_318 = qdd(3)+OMcp17_113*ORcp17_214+OMcp17_114*ORcp17_215+OMcp17_115*ORcp17_216+OMcp17_116*ORcp17_217+OMcp17_117*ORcp17_218+OMcp17_16*...
 ORcp17_213-OMcp17_213*ORcp17_114-OMcp17_214*ORcp17_115-OMcp17_215*ORcp17_116-OMcp17_216*ORcp17_117-OMcp17_217*ORcp17_118-OMcp17_26*ORcp17_113+...
 OPcp17_113*RLcp17_214+OPcp17_114*RLcp17_215+OPcp17_115*RLcp17_216+OPcp17_116*RLcp17_217+OPcp17_117*RLcp17_218+OPcp17_16*RLcp17_213-OPcp17_213*...
 RLcp17_114-OPcp17_214*RLcp17_115-OPcp17_215*RLcp17_116-OPcp17_216*RLcp17_117-OPcp17_217*RLcp17_118-OPcp17_26*RLcp17_113;

% = = Block_1_0_0_18_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp17_118;
    sens.P(2) = POcp17_218;
    sens.P(3) = POcp17_318;
    sens.R(1,1) = ROcp17_118;
    sens.R(1,2) = ROcp17_218;
    sens.R(1,3) = ROcp17_318;
    sens.R(2,1) = ROcp17_417;
    sens.R(2,2) = ROcp17_517;
    sens.R(2,3) = ROcp17_617;
    sens.R(3,1) = ROcp17_718;
    sens.R(3,2) = ROcp17_818;
    sens.R(3,3) = ROcp17_918;
    sens.V(1) = VIcp17_118;
    sens.V(2) = VIcp17_218;
    sens.V(3) = VIcp17_318;
    sens.OM(1) = OMcp17_118;
    sens.OM(2) = OMcp17_218;
    sens.OM(3) = OMcp17_318;
    sens.A(1) = ACcp17_118;
    sens.A(2) = ACcp17_218;
    sens.A(3) = ACcp17_318;
    sens.OMP(1) = OPcp17_118;
    sens.OMP(2) = OPcp17_218;
    sens.OMP(3) = OPcp17_318;
 
% 
case 19, 


% = = Block_1_0_0_19_0_1 = = 
 
% Sensor Kinematics 


    ROcp18_25 = S4*S5;
    ROcp18_35 = -C4*S5;
    ROcp18_85 = -S4*C5;
    ROcp18_95 = C4*C5;
    ROcp18_16 = C5*C6;
    ROcp18_26 = ROcp18_25*C6+C4*S6;
    ROcp18_36 = ROcp18_35*C6+S4*S6;
    ROcp18_46 = -C5*S6;
    ROcp18_56 = -(ROcp18_25*S6-C4*C6);
    ROcp18_66 = -(ROcp18_35*S6-S4*C6);
    OMcp18_25 = qd(5)*C4;
    OMcp18_35 = qd(5)*S4;
    OMcp18_16 = qd(4)+qd(6)*S5;
    OMcp18_26 = OMcp18_25+ROcp18_85*qd(6);
    OMcp18_36 = OMcp18_35+ROcp18_95*qd(6);
    OPcp18_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp18_26 = ROcp18_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp18_35*S5-ROcp18_95*qd(4));
    OPcp18_36 = ROcp18_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp18_25*S5-ROcp18_85*qd(4));

% = = Block_1_0_0_19_0_4 = = 
 
% Sensor Kinematics 


    ROcp18_419 = ROcp18_46*C19+S19*S5;
    ROcp18_519 = ROcp18_56*C19+ROcp18_85*S19;
    ROcp18_619 = ROcp18_66*C19+ROcp18_95*S19;
    ROcp18_719 = -(ROcp18_46*S19-C19*S5);
    ROcp18_819 = -(ROcp18_56*S19-ROcp18_85*C19);
    ROcp18_919 = -(ROcp18_66*S19-ROcp18_95*C19);
    RLcp18_119 = s.dpt(2,3)*ROcp18_46+ROcp18_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp18_219 = s.dpt(2,3)*ROcp18_56+ROcp18_26*s.dpt(1,3)+ROcp18_85*s.dpt(3,3);
    RLcp18_319 = s.dpt(2,3)*ROcp18_66+ROcp18_36*s.dpt(1,3)+ROcp18_95*s.dpt(3,3);
    POcp18_119 = RLcp18_119+q(1);
    POcp18_219 = RLcp18_219+q(2);
    POcp18_319 = RLcp18_319+q(3);
    OMcp18_119 = OMcp18_16+ROcp18_16*qd(19);
    OMcp18_219 = OMcp18_26+ROcp18_26*qd(19);
    OMcp18_319 = OMcp18_36+ROcp18_36*qd(19);
    ORcp18_119 = OMcp18_26*RLcp18_319-OMcp18_36*RLcp18_219;
    ORcp18_219 = -(OMcp18_16*RLcp18_319-OMcp18_36*RLcp18_119);
    ORcp18_319 = OMcp18_16*RLcp18_219-OMcp18_26*RLcp18_119;
    VIcp18_119 = ORcp18_119+qd(1);
    VIcp18_219 = ORcp18_219+qd(2);
    VIcp18_319 = ORcp18_319+qd(3);
    OPcp18_119 = OPcp18_16+ROcp18_16*qdd(19)+qd(19)*(OMcp18_26*ROcp18_36-OMcp18_36*ROcp18_26);
    OPcp18_219 = OPcp18_26+ROcp18_26*qdd(19)-qd(19)*(OMcp18_16*ROcp18_36-OMcp18_36*ROcp18_16);
    OPcp18_319 = OPcp18_36+ROcp18_36*qdd(19)+qd(19)*(OMcp18_16*ROcp18_26-OMcp18_26*ROcp18_16);
    ACcp18_119 = qdd(1)+OMcp18_26*ORcp18_319-OMcp18_36*ORcp18_219+OPcp18_26*RLcp18_319-OPcp18_36*RLcp18_219;
    ACcp18_219 = qdd(2)-OMcp18_16*ORcp18_319+OMcp18_36*ORcp18_119-OPcp18_16*RLcp18_319+OPcp18_36*RLcp18_119;
    ACcp18_319 = qdd(3)+OMcp18_16*ORcp18_219-OMcp18_26*ORcp18_119+OPcp18_16*RLcp18_219-OPcp18_26*RLcp18_119;

% = = Block_1_0_0_19_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp18_119;
    sens.P(2) = POcp18_219;
    sens.P(3) = POcp18_319;
    sens.R(1,1) = ROcp18_16;
    sens.R(1,2) = ROcp18_26;
    sens.R(1,3) = ROcp18_36;
    sens.R(2,1) = ROcp18_419;
    sens.R(2,2) = ROcp18_519;
    sens.R(2,3) = ROcp18_619;
    sens.R(3,1) = ROcp18_719;
    sens.R(3,2) = ROcp18_819;
    sens.R(3,3) = ROcp18_919;
    sens.V(1) = VIcp18_119;
    sens.V(2) = VIcp18_219;
    sens.V(3) = VIcp18_319;
    sens.OM(1) = OMcp18_119;
    sens.OM(2) = OMcp18_219;
    sens.OM(3) = OMcp18_319;
    sens.A(1) = ACcp18_119;
    sens.A(2) = ACcp18_219;
    sens.A(3) = ACcp18_319;
    sens.OMP(1) = OPcp18_119;
    sens.OMP(2) = OPcp18_219;
    sens.OMP(3) = OPcp18_319;
 
% 
case 20, 


% = = Block_1_0_0_20_0_1 = = 
 
% Sensor Kinematics 


    ROcp19_25 = S4*S5;
    ROcp19_35 = -C4*S5;
    ROcp19_85 = -S4*C5;
    ROcp19_95 = C4*C5;
    ROcp19_16 = C5*C6;
    ROcp19_26 = ROcp19_25*C6+C4*S6;
    ROcp19_36 = ROcp19_35*C6+S4*S6;
    ROcp19_46 = -C5*S6;
    ROcp19_56 = -(ROcp19_25*S6-C4*C6);
    ROcp19_66 = -(ROcp19_35*S6-S4*C6);
    OMcp19_25 = qd(5)*C4;
    OMcp19_35 = qd(5)*S4;
    OMcp19_16 = qd(4)+qd(6)*S5;
    OMcp19_26 = OMcp19_25+ROcp19_85*qd(6);
    OMcp19_36 = OMcp19_35+ROcp19_95*qd(6);
    OPcp19_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp19_26 = ROcp19_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp19_35*S5-ROcp19_95*qd(4));
    OPcp19_36 = ROcp19_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp19_25*S5-ROcp19_85*qd(4));

% = = Block_1_0_0_20_0_4 = = 
 
% Sensor Kinematics 


    ROcp19_419 = ROcp19_46*C19+S19*S5;
    ROcp19_519 = ROcp19_56*C19+ROcp19_85*S19;
    ROcp19_619 = ROcp19_66*C19+ROcp19_95*S19;
    ROcp19_719 = -(ROcp19_46*S19-C19*S5);
    ROcp19_819 = -(ROcp19_56*S19-ROcp19_85*C19);
    ROcp19_919 = -(ROcp19_66*S19-ROcp19_95*C19);
    ROcp19_120 = ROcp19_16*C20-ROcp19_719*S20;
    ROcp19_220 = ROcp19_26*C20-ROcp19_819*S20;
    ROcp19_320 = ROcp19_36*C20-ROcp19_919*S20;
    ROcp19_720 = ROcp19_16*S20+ROcp19_719*C20;
    ROcp19_820 = ROcp19_26*S20+ROcp19_819*C20;
    ROcp19_920 = ROcp19_36*S20+ROcp19_919*C20;
    RLcp19_119 = s.dpt(2,3)*ROcp19_46+ROcp19_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp19_219 = s.dpt(2,3)*ROcp19_56+ROcp19_26*s.dpt(1,3)+ROcp19_85*s.dpt(3,3);
    RLcp19_319 = s.dpt(2,3)*ROcp19_66+ROcp19_36*s.dpt(1,3)+ROcp19_95*s.dpt(3,3);
    OMcp19_119 = OMcp19_16+ROcp19_16*qd(19);
    OMcp19_219 = OMcp19_26+ROcp19_26*qd(19);
    OMcp19_319 = OMcp19_36+ROcp19_36*qd(19);
    ORcp19_119 = OMcp19_26*RLcp19_319-OMcp19_36*RLcp19_219;
    ORcp19_219 = -(OMcp19_16*RLcp19_319-OMcp19_36*RLcp19_119);
    ORcp19_319 = OMcp19_16*RLcp19_219-OMcp19_26*RLcp19_119;
    OPcp19_119 = OPcp19_16+ROcp19_16*qdd(19)+qd(19)*(OMcp19_26*ROcp19_36-OMcp19_36*ROcp19_26);
    OPcp19_219 = OPcp19_26+ROcp19_26*qdd(19)-qd(19)*(OMcp19_16*ROcp19_36-OMcp19_36*ROcp19_16);
    OPcp19_319 = OPcp19_36+ROcp19_36*qdd(19)+qd(19)*(OMcp19_16*ROcp19_26-OMcp19_26*ROcp19_16);
    RLcp19_120 = s.dpt(1,30)*ROcp19_16+s.dpt(2,30)*ROcp19_419+s.dpt(3,30)*ROcp19_719;
    RLcp19_220 = s.dpt(1,30)*ROcp19_26+s.dpt(2,30)*ROcp19_519+s.dpt(3,30)*ROcp19_819;
    RLcp19_320 = s.dpt(1,30)*ROcp19_36+s.dpt(2,30)*ROcp19_619+s.dpt(3,30)*ROcp19_919;
    POcp19_120 = RLcp19_119+RLcp19_120+q(1);
    POcp19_220 = RLcp19_219+RLcp19_220+q(2);
    POcp19_320 = RLcp19_319+RLcp19_320+q(3);
    OMcp19_120 = OMcp19_119+ROcp19_419*qd(20);
    OMcp19_220 = OMcp19_219+ROcp19_519*qd(20);
    OMcp19_320 = OMcp19_319+ROcp19_619*qd(20);
    ORcp19_120 = OMcp19_219*RLcp19_320-OMcp19_319*RLcp19_220;
    ORcp19_220 = -(OMcp19_119*RLcp19_320-OMcp19_319*RLcp19_120);
    ORcp19_320 = OMcp19_119*RLcp19_220-OMcp19_219*RLcp19_120;
    VIcp19_120 = ORcp19_119+ORcp19_120+qd(1);
    VIcp19_220 = ORcp19_219+ORcp19_220+qd(2);
    VIcp19_320 = ORcp19_319+ORcp19_320+qd(3);
    OPcp19_120 = OPcp19_119+ROcp19_419*qdd(20)+qd(20)*(OMcp19_219*ROcp19_619-OMcp19_319*ROcp19_519);
    OPcp19_220 = OPcp19_219+ROcp19_519*qdd(20)-qd(20)*(OMcp19_119*ROcp19_619-OMcp19_319*ROcp19_419);
    OPcp19_320 = OPcp19_319+ROcp19_619*qdd(20)+qd(20)*(OMcp19_119*ROcp19_519-OMcp19_219*ROcp19_419);
    ACcp19_120 = qdd(1)+OMcp19_219*ORcp19_320+OMcp19_26*ORcp19_319-OMcp19_319*ORcp19_220-OMcp19_36*ORcp19_219+OPcp19_219*RLcp19_320+OPcp19_26*...
 RLcp19_319-OPcp19_319*RLcp19_220-OPcp19_36*RLcp19_219;
    ACcp19_220 = qdd(2)-OMcp19_119*ORcp19_320-OMcp19_16*ORcp19_319+OMcp19_319*ORcp19_120+OMcp19_36*ORcp19_119-OPcp19_119*RLcp19_320-OPcp19_16*...
 RLcp19_319+OPcp19_319*RLcp19_120+OPcp19_36*RLcp19_119;
    ACcp19_320 = qdd(3)+OMcp19_119*ORcp19_220+OMcp19_16*ORcp19_219-OMcp19_219*ORcp19_120-OMcp19_26*ORcp19_119+OPcp19_119*RLcp19_220+OPcp19_16*...
 RLcp19_219-OPcp19_219*RLcp19_120-OPcp19_26*RLcp19_119;

% = = Block_1_0_0_20_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp19_120;
    sens.P(2) = POcp19_220;
    sens.P(3) = POcp19_320;
    sens.R(1,1) = ROcp19_120;
    sens.R(1,2) = ROcp19_220;
    sens.R(1,3) = ROcp19_320;
    sens.R(2,1) = ROcp19_419;
    sens.R(2,2) = ROcp19_519;
    sens.R(2,3) = ROcp19_619;
    sens.R(3,1) = ROcp19_720;
    sens.R(3,2) = ROcp19_820;
    sens.R(3,3) = ROcp19_920;
    sens.V(1) = VIcp19_120;
    sens.V(2) = VIcp19_220;
    sens.V(3) = VIcp19_320;
    sens.OM(1) = OMcp19_120;
    sens.OM(2) = OMcp19_220;
    sens.OM(3) = OMcp19_320;
    sens.A(1) = ACcp19_120;
    sens.A(2) = ACcp19_220;
    sens.A(3) = ACcp19_320;
    sens.OMP(1) = OPcp19_120;
    sens.OMP(2) = OPcp19_220;
    sens.OMP(3) = OPcp19_320;
 
% 
case 21, 


% = = Block_1_0_0_21_0_1 = = 
 
% Sensor Kinematics 


    ROcp20_25 = S4*S5;
    ROcp20_35 = -C4*S5;
    ROcp20_85 = -S4*C5;
    ROcp20_95 = C4*C5;
    ROcp20_16 = C5*C6;
    ROcp20_26 = ROcp20_25*C6+C4*S6;
    ROcp20_36 = ROcp20_35*C6+S4*S6;
    ROcp20_46 = -C5*S6;
    ROcp20_56 = -(ROcp20_25*S6-C4*C6);
    ROcp20_66 = -(ROcp20_35*S6-S4*C6);
    OMcp20_25 = qd(5)*C4;
    OMcp20_35 = qd(5)*S4;
    OMcp20_16 = qd(4)+qd(6)*S5;
    OMcp20_26 = OMcp20_25+ROcp20_85*qd(6);
    OMcp20_36 = OMcp20_35+ROcp20_95*qd(6);
    OPcp20_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp20_26 = ROcp20_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp20_35*S5-ROcp20_95*qd(4));
    OPcp20_36 = ROcp20_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp20_25*S5-ROcp20_85*qd(4));

% = = Block_1_0_0_21_0_4 = = 
 
% Sensor Kinematics 


    ROcp20_419 = ROcp20_46*C19+S19*S5;
    ROcp20_519 = ROcp20_56*C19+ROcp20_85*S19;
    ROcp20_619 = ROcp20_66*C19+ROcp20_95*S19;
    ROcp20_719 = -(ROcp20_46*S19-C19*S5);
    ROcp20_819 = -(ROcp20_56*S19-ROcp20_85*C19);
    ROcp20_919 = -(ROcp20_66*S19-ROcp20_95*C19);
    ROcp20_120 = ROcp20_16*C20-ROcp20_719*S20;
    ROcp20_220 = ROcp20_26*C20-ROcp20_819*S20;
    ROcp20_320 = ROcp20_36*C20-ROcp20_919*S20;
    ROcp20_720 = ROcp20_16*S20+ROcp20_719*C20;
    ROcp20_820 = ROcp20_26*S20+ROcp20_819*C20;
    ROcp20_920 = ROcp20_36*S20+ROcp20_919*C20;
    ROcp20_121 = ROcp20_120*C21+ROcp20_419*S21;
    ROcp20_221 = ROcp20_220*C21+ROcp20_519*S21;
    ROcp20_321 = ROcp20_320*C21+ROcp20_619*S21;
    ROcp20_421 = -(ROcp20_120*S21-ROcp20_419*C21);
    ROcp20_521 = -(ROcp20_220*S21-ROcp20_519*C21);
    ROcp20_621 = -(ROcp20_320*S21-ROcp20_619*C21);
    RLcp20_119 = s.dpt(2,3)*ROcp20_46+ROcp20_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp20_219 = s.dpt(2,3)*ROcp20_56+ROcp20_26*s.dpt(1,3)+ROcp20_85*s.dpt(3,3);
    RLcp20_319 = s.dpt(2,3)*ROcp20_66+ROcp20_36*s.dpt(1,3)+ROcp20_95*s.dpt(3,3);
    OMcp20_119 = OMcp20_16+ROcp20_16*qd(19);
    OMcp20_219 = OMcp20_26+ROcp20_26*qd(19);
    OMcp20_319 = OMcp20_36+ROcp20_36*qd(19);
    ORcp20_119 = OMcp20_26*RLcp20_319-OMcp20_36*RLcp20_219;
    ORcp20_219 = -(OMcp20_16*RLcp20_319-OMcp20_36*RLcp20_119);
    ORcp20_319 = OMcp20_16*RLcp20_219-OMcp20_26*RLcp20_119;
    OPcp20_119 = OPcp20_16+ROcp20_16*qdd(19)+qd(19)*(OMcp20_26*ROcp20_36-OMcp20_36*ROcp20_26);
    OPcp20_219 = OPcp20_26+ROcp20_26*qdd(19)-qd(19)*(OMcp20_16*ROcp20_36-OMcp20_36*ROcp20_16);
    OPcp20_319 = OPcp20_36+ROcp20_36*qdd(19)+qd(19)*(OMcp20_16*ROcp20_26-OMcp20_26*ROcp20_16);
    RLcp20_120 = s.dpt(1,30)*ROcp20_16+s.dpt(2,30)*ROcp20_419+s.dpt(3,30)*ROcp20_719;
    RLcp20_220 = s.dpt(1,30)*ROcp20_26+s.dpt(2,30)*ROcp20_519+s.dpt(3,30)*ROcp20_819;
    RLcp20_320 = s.dpt(1,30)*ROcp20_36+s.dpt(2,30)*ROcp20_619+s.dpt(3,30)*ROcp20_919;
    OMcp20_120 = OMcp20_119+ROcp20_419*qd(20);
    OMcp20_220 = OMcp20_219+ROcp20_519*qd(20);
    OMcp20_320 = OMcp20_319+ROcp20_619*qd(20);
    ORcp20_120 = OMcp20_219*RLcp20_320-OMcp20_319*RLcp20_220;
    ORcp20_220 = -(OMcp20_119*RLcp20_320-OMcp20_319*RLcp20_120);
    ORcp20_320 = OMcp20_119*RLcp20_220-OMcp20_219*RLcp20_120;
    OPcp20_120 = OPcp20_119+ROcp20_419*qdd(20)+qd(20)*(OMcp20_219*ROcp20_619-OMcp20_319*ROcp20_519);
    OPcp20_220 = OPcp20_219+ROcp20_519*qdd(20)-qd(20)*(OMcp20_119*ROcp20_619-OMcp20_319*ROcp20_419);
    OPcp20_320 = OPcp20_319+ROcp20_619*qdd(20)+qd(20)*(OMcp20_119*ROcp20_519-OMcp20_219*ROcp20_419);
    RLcp20_121 = s.dpt(1,32)*ROcp20_120+s.dpt(2,32)*ROcp20_419+ROcp20_720*s.dpt(3,32);
    RLcp20_221 = s.dpt(1,32)*ROcp20_220+s.dpt(2,32)*ROcp20_519+ROcp20_820*s.dpt(3,32);
    RLcp20_321 = s.dpt(1,32)*ROcp20_320+s.dpt(2,32)*ROcp20_619+ROcp20_920*s.dpt(3,32);
    POcp20_121 = RLcp20_119+RLcp20_120+RLcp20_121+q(1);
    POcp20_221 = RLcp20_219+RLcp20_220+RLcp20_221+q(2);
    POcp20_321 = RLcp20_319+RLcp20_320+RLcp20_321+q(3);
    OMcp20_121 = OMcp20_120+ROcp20_720*qd(21);
    OMcp20_221 = OMcp20_220+ROcp20_820*qd(21);
    OMcp20_321 = OMcp20_320+ROcp20_920*qd(21);
    ORcp20_121 = OMcp20_220*RLcp20_321-OMcp20_320*RLcp20_221;
    ORcp20_221 = -(OMcp20_120*RLcp20_321-OMcp20_320*RLcp20_121);
    ORcp20_321 = OMcp20_120*RLcp20_221-OMcp20_220*RLcp20_121;
    VIcp20_121 = ORcp20_119+ORcp20_120+ORcp20_121+qd(1);
    VIcp20_221 = ORcp20_219+ORcp20_220+ORcp20_221+qd(2);
    VIcp20_321 = ORcp20_319+ORcp20_320+ORcp20_321+qd(3);
    OPcp20_121 = OPcp20_120+ROcp20_720*qdd(21)+qd(21)*(OMcp20_220*ROcp20_920-OMcp20_320*ROcp20_820);
    OPcp20_221 = OPcp20_220+ROcp20_820*qdd(21)-qd(21)*(OMcp20_120*ROcp20_920-OMcp20_320*ROcp20_720);
    OPcp20_321 = OPcp20_320+ROcp20_920*qdd(21)+qd(21)*(OMcp20_120*ROcp20_820-OMcp20_220*ROcp20_720);
    ACcp20_121 = qdd(1)+OMcp20_219*ORcp20_320+OMcp20_220*ORcp20_321+OMcp20_26*ORcp20_319-OMcp20_319*ORcp20_220-OMcp20_320*ORcp20_221-OMcp20_36*...
 ORcp20_219+OPcp20_219*RLcp20_320+OPcp20_220*RLcp20_321+OPcp20_26*RLcp20_319-OPcp20_319*RLcp20_220-OPcp20_320*RLcp20_221-OPcp20_36*RLcp20_219;
    ACcp20_221 = qdd(2)-OMcp20_119*ORcp20_320-OMcp20_120*ORcp20_321-OMcp20_16*ORcp20_319+OMcp20_319*ORcp20_120+OMcp20_320*ORcp20_121+OMcp20_36*...
 ORcp20_119-OPcp20_119*RLcp20_320-OPcp20_120*RLcp20_321-OPcp20_16*RLcp20_319+OPcp20_319*RLcp20_120+OPcp20_320*RLcp20_121+OPcp20_36*RLcp20_119;
    ACcp20_321 = qdd(3)+OMcp20_119*ORcp20_220+OMcp20_120*ORcp20_221+OMcp20_16*ORcp20_219-OMcp20_219*ORcp20_120-OMcp20_220*ORcp20_121-OMcp20_26*...
 ORcp20_119+OPcp20_119*RLcp20_220+OPcp20_120*RLcp20_221+OPcp20_16*RLcp20_219-OPcp20_219*RLcp20_120-OPcp20_220*RLcp20_121-OPcp20_26*RLcp20_119;

% = = Block_1_0_0_21_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp20_121;
    sens.P(2) = POcp20_221;
    sens.P(3) = POcp20_321;
    sens.R(1,1) = ROcp20_121;
    sens.R(1,2) = ROcp20_221;
    sens.R(1,3) = ROcp20_321;
    sens.R(2,1) = ROcp20_421;
    sens.R(2,2) = ROcp20_521;
    sens.R(2,3) = ROcp20_621;
    sens.R(3,1) = ROcp20_720;
    sens.R(3,2) = ROcp20_820;
    sens.R(3,3) = ROcp20_920;
    sens.V(1) = VIcp20_121;
    sens.V(2) = VIcp20_221;
    sens.V(3) = VIcp20_321;
    sens.OM(1) = OMcp20_121;
    sens.OM(2) = OMcp20_221;
    sens.OM(3) = OMcp20_321;
    sens.A(1) = ACcp20_121;
    sens.A(2) = ACcp20_221;
    sens.A(3) = ACcp20_321;
    sens.OMP(1) = OPcp20_121;
    sens.OMP(2) = OPcp20_221;
    sens.OMP(3) = OPcp20_321;
 
% 
case 22, 


% = = Block_1_0_0_22_0_1 = = 
 
% Sensor Kinematics 


    ROcp21_25 = S4*S5;
    ROcp21_35 = -C4*S5;
    ROcp21_85 = -S4*C5;
    ROcp21_95 = C4*C5;
    ROcp21_16 = C5*C6;
    ROcp21_26 = ROcp21_25*C6+C4*S6;
    ROcp21_36 = ROcp21_35*C6+S4*S6;
    ROcp21_46 = -C5*S6;
    ROcp21_56 = -(ROcp21_25*S6-C4*C6);
    ROcp21_66 = -(ROcp21_35*S6-S4*C6);
    OMcp21_25 = qd(5)*C4;
    OMcp21_35 = qd(5)*S4;
    OMcp21_16 = qd(4)+qd(6)*S5;
    OMcp21_26 = OMcp21_25+ROcp21_85*qd(6);
    OMcp21_36 = OMcp21_35+ROcp21_95*qd(6);
    OPcp21_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp21_26 = ROcp21_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp21_35*S5-ROcp21_95*qd(4));
    OPcp21_36 = ROcp21_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp21_25*S5-ROcp21_85*qd(4));

% = = Block_1_0_0_22_0_4 = = 
 
% Sensor Kinematics 


    ROcp21_419 = ROcp21_46*C19+S19*S5;
    ROcp21_519 = ROcp21_56*C19+ROcp21_85*S19;
    ROcp21_619 = ROcp21_66*C19+ROcp21_95*S19;
    ROcp21_719 = -(ROcp21_46*S19-C19*S5);
    ROcp21_819 = -(ROcp21_56*S19-ROcp21_85*C19);
    ROcp21_919 = -(ROcp21_66*S19-ROcp21_95*C19);
    ROcp21_120 = ROcp21_16*C20-ROcp21_719*S20;
    ROcp21_220 = ROcp21_26*C20-ROcp21_819*S20;
    ROcp21_320 = ROcp21_36*C20-ROcp21_919*S20;
    ROcp21_720 = ROcp21_16*S20+ROcp21_719*C20;
    ROcp21_820 = ROcp21_26*S20+ROcp21_819*C20;
    ROcp21_920 = ROcp21_36*S20+ROcp21_919*C20;
    ROcp21_121 = ROcp21_120*C21+ROcp21_419*S21;
    ROcp21_221 = ROcp21_220*C21+ROcp21_519*S21;
    ROcp21_321 = ROcp21_320*C21+ROcp21_619*S21;
    ROcp21_421 = -(ROcp21_120*S21-ROcp21_419*C21);
    ROcp21_521 = -(ROcp21_220*S21-ROcp21_519*C21);
    ROcp21_621 = -(ROcp21_320*S21-ROcp21_619*C21);
    RLcp21_119 = s.dpt(2,3)*ROcp21_46+ROcp21_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp21_219 = s.dpt(2,3)*ROcp21_56+ROcp21_26*s.dpt(1,3)+ROcp21_85*s.dpt(3,3);
    RLcp21_319 = s.dpt(2,3)*ROcp21_66+ROcp21_36*s.dpt(1,3)+ROcp21_95*s.dpt(3,3);
    OMcp21_119 = OMcp21_16+ROcp21_16*qd(19);
    OMcp21_219 = OMcp21_26+ROcp21_26*qd(19);
    OMcp21_319 = OMcp21_36+ROcp21_36*qd(19);
    ORcp21_119 = OMcp21_26*RLcp21_319-OMcp21_36*RLcp21_219;
    ORcp21_219 = -(OMcp21_16*RLcp21_319-OMcp21_36*RLcp21_119);
    ORcp21_319 = OMcp21_16*RLcp21_219-OMcp21_26*RLcp21_119;
    OPcp21_119 = OPcp21_16+ROcp21_16*qdd(19)+qd(19)*(OMcp21_26*ROcp21_36-OMcp21_36*ROcp21_26);
    OPcp21_219 = OPcp21_26+ROcp21_26*qdd(19)-qd(19)*(OMcp21_16*ROcp21_36-OMcp21_36*ROcp21_16);
    OPcp21_319 = OPcp21_36+ROcp21_36*qdd(19)+qd(19)*(OMcp21_16*ROcp21_26-OMcp21_26*ROcp21_16);
    RLcp21_120 = s.dpt(1,30)*ROcp21_16+s.dpt(2,30)*ROcp21_419+s.dpt(3,30)*ROcp21_719;
    RLcp21_220 = s.dpt(1,30)*ROcp21_26+s.dpt(2,30)*ROcp21_519+s.dpt(3,30)*ROcp21_819;
    RLcp21_320 = s.dpt(1,30)*ROcp21_36+s.dpt(2,30)*ROcp21_619+s.dpt(3,30)*ROcp21_919;
    OMcp21_120 = OMcp21_119+ROcp21_419*qd(20);
    OMcp21_220 = OMcp21_219+ROcp21_519*qd(20);
    OMcp21_320 = OMcp21_319+ROcp21_619*qd(20);
    ORcp21_120 = OMcp21_219*RLcp21_320-OMcp21_319*RLcp21_220;
    ORcp21_220 = -(OMcp21_119*RLcp21_320-OMcp21_319*RLcp21_120);
    ORcp21_320 = OMcp21_119*RLcp21_220-OMcp21_219*RLcp21_120;
    OPcp21_120 = OPcp21_119+ROcp21_419*qdd(20)+qd(20)*(OMcp21_219*ROcp21_619-OMcp21_319*ROcp21_519);
    OPcp21_220 = OPcp21_219+ROcp21_519*qdd(20)-qd(20)*(OMcp21_119*ROcp21_619-OMcp21_319*ROcp21_419);
    OPcp21_320 = OPcp21_319+ROcp21_619*qdd(20)+qd(20)*(OMcp21_119*ROcp21_519-OMcp21_219*ROcp21_419);
    RLcp21_121 = s.dpt(1,32)*ROcp21_120+s.dpt(2,32)*ROcp21_419+ROcp21_720*s.dpt(3,32);
    RLcp21_221 = s.dpt(1,32)*ROcp21_220+s.dpt(2,32)*ROcp21_519+ROcp21_820*s.dpt(3,32);
    RLcp21_321 = s.dpt(1,32)*ROcp21_320+s.dpt(2,32)*ROcp21_619+ROcp21_920*s.dpt(3,32);
    OMcp21_121 = OMcp21_120+ROcp21_720*qd(21);
    OMcp21_221 = OMcp21_220+ROcp21_820*qd(21);
    OMcp21_321 = OMcp21_320+ROcp21_920*qd(21);
    ORcp21_121 = OMcp21_220*RLcp21_321-OMcp21_320*RLcp21_221;
    ORcp21_221 = -(OMcp21_120*RLcp21_321-OMcp21_320*RLcp21_121);
    ORcp21_321 = OMcp21_120*RLcp21_221-OMcp21_220*RLcp21_121;
    OPcp21_121 = OPcp21_120+ROcp21_720*qdd(21)+qd(21)*(OMcp21_220*ROcp21_920-OMcp21_320*ROcp21_820);
    OPcp21_221 = OPcp21_220+ROcp21_820*qdd(21)-qd(21)*(OMcp21_120*ROcp21_920-OMcp21_320*ROcp21_720);
    OPcp21_321 = OPcp21_320+ROcp21_920*qdd(21)+qd(21)*(OMcp21_120*ROcp21_820-OMcp21_220*ROcp21_720);

% = = Block_1_0_0_22_0_5 = = 
 
% Sensor Kinematics 


    ROcp21_122 = ROcp21_121*C22-ROcp21_720*S22;
    ROcp21_222 = ROcp21_221*C22-ROcp21_820*S22;
    ROcp21_322 = ROcp21_321*C22-ROcp21_920*S22;
    ROcp21_722 = ROcp21_121*S22+ROcp21_720*C22;
    ROcp21_822 = ROcp21_221*S22+ROcp21_820*C22;
    ROcp21_922 = ROcp21_321*S22+ROcp21_920*C22;
    RLcp21_122 = ROcp21_121*s.dpt(1,36)+ROcp21_421*s.dpt(2,36)+ROcp21_720*s.dpt(3,36);
    RLcp21_222 = ROcp21_221*s.dpt(1,36)+ROcp21_521*s.dpt(2,36)+ROcp21_820*s.dpt(3,36);
    RLcp21_322 = ROcp21_321*s.dpt(1,36)+ROcp21_621*s.dpt(2,36)+ROcp21_920*s.dpt(3,36);
    POcp21_122 = RLcp21_119+RLcp21_120+RLcp21_121+RLcp21_122+q(1);
    POcp21_222 = RLcp21_219+RLcp21_220+RLcp21_221+RLcp21_222+q(2);
    POcp21_322 = RLcp21_319+RLcp21_320+RLcp21_321+RLcp21_322+q(3);
    OMcp21_122 = OMcp21_121+ROcp21_421*qd(22);
    OMcp21_222 = OMcp21_221+ROcp21_521*qd(22);
    OMcp21_322 = OMcp21_321+ROcp21_621*qd(22);
    ORcp21_122 = OMcp21_221*RLcp21_322-OMcp21_321*RLcp21_222;
    ORcp21_222 = -(OMcp21_121*RLcp21_322-OMcp21_321*RLcp21_122);
    ORcp21_322 = OMcp21_121*RLcp21_222-OMcp21_221*RLcp21_122;
    VIcp21_122 = ORcp21_119+ORcp21_120+ORcp21_121+ORcp21_122+qd(1);
    VIcp21_222 = ORcp21_219+ORcp21_220+ORcp21_221+ORcp21_222+qd(2);
    VIcp21_322 = ORcp21_319+ORcp21_320+ORcp21_321+ORcp21_322+qd(3);
    OPcp21_122 = OPcp21_121+ROcp21_421*qdd(22)+qd(22)*(OMcp21_221*ROcp21_621-OMcp21_321*ROcp21_521);
    OPcp21_222 = OPcp21_221+ROcp21_521*qdd(22)-qd(22)*(OMcp21_121*ROcp21_621-OMcp21_321*ROcp21_421);
    OPcp21_322 = OPcp21_321+ROcp21_621*qdd(22)+qd(22)*(OMcp21_121*ROcp21_521-OMcp21_221*ROcp21_421);
    ACcp21_122 = qdd(1)+OMcp21_219*ORcp21_320+OMcp21_220*ORcp21_321+OMcp21_221*ORcp21_322+OMcp21_26*ORcp21_319-OMcp21_319*ORcp21_220-OMcp21_320*...
 ORcp21_221-OMcp21_321*ORcp21_222-OMcp21_36*ORcp21_219+OPcp21_219*RLcp21_320+OPcp21_220*RLcp21_321+OPcp21_221*RLcp21_322+OPcp21_26*RLcp21_319-...
 OPcp21_319*RLcp21_220-OPcp21_320*RLcp21_221-OPcp21_321*RLcp21_222-OPcp21_36*RLcp21_219;
    ACcp21_222 = qdd(2)-OMcp21_119*ORcp21_320-OMcp21_120*ORcp21_321-OMcp21_121*ORcp21_322-OMcp21_16*ORcp21_319+OMcp21_319*ORcp21_120+OMcp21_320*...
 ORcp21_121+OMcp21_321*ORcp21_122+OMcp21_36*ORcp21_119-OPcp21_119*RLcp21_320-OPcp21_120*RLcp21_321-OPcp21_121*RLcp21_322-OPcp21_16*RLcp21_319+...
 OPcp21_319*RLcp21_120+OPcp21_320*RLcp21_121+OPcp21_321*RLcp21_122+OPcp21_36*RLcp21_119;
    ACcp21_322 = qdd(3)+OMcp21_119*ORcp21_220+OMcp21_120*ORcp21_221+OMcp21_121*ORcp21_222+OMcp21_16*ORcp21_219-OMcp21_219*ORcp21_120-OMcp21_220*...
 ORcp21_121-OMcp21_221*ORcp21_122-OMcp21_26*ORcp21_119+OPcp21_119*RLcp21_220+OPcp21_120*RLcp21_221+OPcp21_121*RLcp21_222+OPcp21_16*RLcp21_219-...
 OPcp21_219*RLcp21_120-OPcp21_220*RLcp21_121-OPcp21_221*RLcp21_122-OPcp21_26*RLcp21_119;

% = = Block_1_0_0_22_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp21_122;
    sens.P(2) = POcp21_222;
    sens.P(3) = POcp21_322;
    sens.R(1,1) = ROcp21_122;
    sens.R(1,2) = ROcp21_222;
    sens.R(1,3) = ROcp21_322;
    sens.R(2,1) = ROcp21_421;
    sens.R(2,2) = ROcp21_521;
    sens.R(2,3) = ROcp21_621;
    sens.R(3,1) = ROcp21_722;
    sens.R(3,2) = ROcp21_822;
    sens.R(3,3) = ROcp21_922;
    sens.V(1) = VIcp21_122;
    sens.V(2) = VIcp21_222;
    sens.V(3) = VIcp21_322;
    sens.OM(1) = OMcp21_122;
    sens.OM(2) = OMcp21_222;
    sens.OM(3) = OMcp21_322;
    sens.A(1) = ACcp21_122;
    sens.A(2) = ACcp21_222;
    sens.A(3) = ACcp21_322;
    sens.OMP(1) = OPcp21_122;
    sens.OMP(2) = OPcp21_222;
    sens.OMP(3) = OPcp21_322;
 
% 
case 23, 


% = = Block_1_0_0_23_0_1 = = 
 
% Sensor Kinematics 


    ROcp22_25 = S4*S5;
    ROcp22_35 = -C4*S5;
    ROcp22_85 = -S4*C5;
    ROcp22_95 = C4*C5;
    ROcp22_16 = C5*C6;
    ROcp22_26 = ROcp22_25*C6+C4*S6;
    ROcp22_36 = ROcp22_35*C6+S4*S6;
    ROcp22_46 = -C5*S6;
    ROcp22_56 = -(ROcp22_25*S6-C4*C6);
    ROcp22_66 = -(ROcp22_35*S6-S4*C6);
    OMcp22_25 = qd(5)*C4;
    OMcp22_35 = qd(5)*S4;
    OMcp22_16 = qd(4)+qd(6)*S5;
    OMcp22_26 = OMcp22_25+ROcp22_85*qd(6);
    OMcp22_36 = OMcp22_35+ROcp22_95*qd(6);
    OPcp22_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp22_26 = ROcp22_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp22_35*S5-ROcp22_95*qd(4));
    OPcp22_36 = ROcp22_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp22_25*S5-ROcp22_85*qd(4));

% = = Block_1_0_0_23_0_4 = = 
 
% Sensor Kinematics 


    ROcp22_419 = ROcp22_46*C19+S19*S5;
    ROcp22_519 = ROcp22_56*C19+ROcp22_85*S19;
    ROcp22_619 = ROcp22_66*C19+ROcp22_95*S19;
    ROcp22_719 = -(ROcp22_46*S19-C19*S5);
    ROcp22_819 = -(ROcp22_56*S19-ROcp22_85*C19);
    ROcp22_919 = -(ROcp22_66*S19-ROcp22_95*C19);
    ROcp22_120 = ROcp22_16*C20-ROcp22_719*S20;
    ROcp22_220 = ROcp22_26*C20-ROcp22_819*S20;
    ROcp22_320 = ROcp22_36*C20-ROcp22_919*S20;
    ROcp22_720 = ROcp22_16*S20+ROcp22_719*C20;
    ROcp22_820 = ROcp22_26*S20+ROcp22_819*C20;
    ROcp22_920 = ROcp22_36*S20+ROcp22_919*C20;
    ROcp22_121 = ROcp22_120*C21+ROcp22_419*S21;
    ROcp22_221 = ROcp22_220*C21+ROcp22_519*S21;
    ROcp22_321 = ROcp22_320*C21+ROcp22_619*S21;
    ROcp22_421 = -(ROcp22_120*S21-ROcp22_419*C21);
    ROcp22_521 = -(ROcp22_220*S21-ROcp22_519*C21);
    ROcp22_621 = -(ROcp22_320*S21-ROcp22_619*C21);
    RLcp22_119 = s.dpt(2,3)*ROcp22_46+ROcp22_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp22_219 = s.dpt(2,3)*ROcp22_56+ROcp22_26*s.dpt(1,3)+ROcp22_85*s.dpt(3,3);
    RLcp22_319 = s.dpt(2,3)*ROcp22_66+ROcp22_36*s.dpt(1,3)+ROcp22_95*s.dpt(3,3);
    OMcp22_119 = OMcp22_16+ROcp22_16*qd(19);
    OMcp22_219 = OMcp22_26+ROcp22_26*qd(19);
    OMcp22_319 = OMcp22_36+ROcp22_36*qd(19);
    ORcp22_119 = OMcp22_26*RLcp22_319-OMcp22_36*RLcp22_219;
    ORcp22_219 = -(OMcp22_16*RLcp22_319-OMcp22_36*RLcp22_119);
    ORcp22_319 = OMcp22_16*RLcp22_219-OMcp22_26*RLcp22_119;
    OPcp22_119 = OPcp22_16+ROcp22_16*qdd(19)+qd(19)*(OMcp22_26*ROcp22_36-OMcp22_36*ROcp22_26);
    OPcp22_219 = OPcp22_26+ROcp22_26*qdd(19)-qd(19)*(OMcp22_16*ROcp22_36-OMcp22_36*ROcp22_16);
    OPcp22_319 = OPcp22_36+ROcp22_36*qdd(19)+qd(19)*(OMcp22_16*ROcp22_26-OMcp22_26*ROcp22_16);
    RLcp22_120 = s.dpt(1,30)*ROcp22_16+s.dpt(2,30)*ROcp22_419+s.dpt(3,30)*ROcp22_719;
    RLcp22_220 = s.dpt(1,30)*ROcp22_26+s.dpt(2,30)*ROcp22_519+s.dpt(3,30)*ROcp22_819;
    RLcp22_320 = s.dpt(1,30)*ROcp22_36+s.dpt(2,30)*ROcp22_619+s.dpt(3,30)*ROcp22_919;
    OMcp22_120 = OMcp22_119+ROcp22_419*qd(20);
    OMcp22_220 = OMcp22_219+ROcp22_519*qd(20);
    OMcp22_320 = OMcp22_319+ROcp22_619*qd(20);
    ORcp22_120 = OMcp22_219*RLcp22_320-OMcp22_319*RLcp22_220;
    ORcp22_220 = -(OMcp22_119*RLcp22_320-OMcp22_319*RLcp22_120);
    ORcp22_320 = OMcp22_119*RLcp22_220-OMcp22_219*RLcp22_120;
    OPcp22_120 = OPcp22_119+ROcp22_419*qdd(20)+qd(20)*(OMcp22_219*ROcp22_619-OMcp22_319*ROcp22_519);
    OPcp22_220 = OPcp22_219+ROcp22_519*qdd(20)-qd(20)*(OMcp22_119*ROcp22_619-OMcp22_319*ROcp22_419);
    OPcp22_320 = OPcp22_319+ROcp22_619*qdd(20)+qd(20)*(OMcp22_119*ROcp22_519-OMcp22_219*ROcp22_419);
    RLcp22_121 = s.dpt(1,32)*ROcp22_120+s.dpt(2,32)*ROcp22_419+ROcp22_720*s.dpt(3,32);
    RLcp22_221 = s.dpt(1,32)*ROcp22_220+s.dpt(2,32)*ROcp22_519+ROcp22_820*s.dpt(3,32);
    RLcp22_321 = s.dpt(1,32)*ROcp22_320+s.dpt(2,32)*ROcp22_619+ROcp22_920*s.dpt(3,32);
    OMcp22_121 = OMcp22_120+ROcp22_720*qd(21);
    OMcp22_221 = OMcp22_220+ROcp22_820*qd(21);
    OMcp22_321 = OMcp22_320+ROcp22_920*qd(21);
    ORcp22_121 = OMcp22_220*RLcp22_321-OMcp22_320*RLcp22_221;
    ORcp22_221 = -(OMcp22_120*RLcp22_321-OMcp22_320*RLcp22_121);
    ORcp22_321 = OMcp22_120*RLcp22_221-OMcp22_220*RLcp22_121;
    OPcp22_121 = OPcp22_120+ROcp22_720*qdd(21)+qd(21)*(OMcp22_220*ROcp22_920-OMcp22_320*ROcp22_820);
    OPcp22_221 = OPcp22_220+ROcp22_820*qdd(21)-qd(21)*(OMcp22_120*ROcp22_920-OMcp22_320*ROcp22_720);
    OPcp22_321 = OPcp22_320+ROcp22_920*qdd(21)+qd(21)*(OMcp22_120*ROcp22_820-OMcp22_220*ROcp22_720);

% = = Block_1_0_0_23_0_5 = = 
 
% Sensor Kinematics 


    ROcp22_122 = ROcp22_121*C22-ROcp22_720*S22;
    ROcp22_222 = ROcp22_221*C22-ROcp22_820*S22;
    ROcp22_322 = ROcp22_321*C22-ROcp22_920*S22;
    ROcp22_722 = ROcp22_121*S22+ROcp22_720*C22;
    ROcp22_822 = ROcp22_221*S22+ROcp22_820*C22;
    ROcp22_922 = ROcp22_321*S22+ROcp22_920*C22;
    ROcp22_423 = ROcp22_421*C23+ROcp22_722*S23;
    ROcp22_523 = ROcp22_521*C23+ROcp22_822*S23;
    ROcp22_623 = ROcp22_621*C23+ROcp22_922*S23;
    ROcp22_723 = -(ROcp22_421*S23-ROcp22_722*C23);
    ROcp22_823 = -(ROcp22_521*S23-ROcp22_822*C23);
    ROcp22_923 = -(ROcp22_621*S23-ROcp22_922*C23);
    RLcp22_122 = ROcp22_121*s.dpt(1,36)+ROcp22_421*s.dpt(2,36)+ROcp22_720*s.dpt(3,36);
    RLcp22_222 = ROcp22_221*s.dpt(1,36)+ROcp22_521*s.dpt(2,36)+ROcp22_820*s.dpt(3,36);
    RLcp22_322 = ROcp22_321*s.dpt(1,36)+ROcp22_621*s.dpt(2,36)+ROcp22_920*s.dpt(3,36);
    OMcp22_122 = OMcp22_121+ROcp22_421*qd(22);
    OMcp22_222 = OMcp22_221+ROcp22_521*qd(22);
    OMcp22_322 = OMcp22_321+ROcp22_621*qd(22);
    ORcp22_122 = OMcp22_221*RLcp22_322-OMcp22_321*RLcp22_222;
    ORcp22_222 = -(OMcp22_121*RLcp22_322-OMcp22_321*RLcp22_122);
    ORcp22_322 = OMcp22_121*RLcp22_222-OMcp22_221*RLcp22_122;
    OPcp22_122 = OPcp22_121+ROcp22_421*qdd(22)+qd(22)*(OMcp22_221*ROcp22_621-OMcp22_321*ROcp22_521);
    OPcp22_222 = OPcp22_221+ROcp22_521*qdd(22)-qd(22)*(OMcp22_121*ROcp22_621-OMcp22_321*ROcp22_421);
    OPcp22_322 = OPcp22_321+ROcp22_621*qdd(22)+qd(22)*(OMcp22_121*ROcp22_521-OMcp22_221*ROcp22_421);
    RLcp22_123 = s.dpt(1,40)*ROcp22_122+s.dpt(3,40)*ROcp22_722+ROcp22_421*s.dpt(2,40);
    RLcp22_223 = s.dpt(1,40)*ROcp22_222+s.dpt(3,40)*ROcp22_822+ROcp22_521*s.dpt(2,40);
    RLcp22_323 = s.dpt(1,40)*ROcp22_322+s.dpt(3,40)*ROcp22_922+ROcp22_621*s.dpt(2,40);
    POcp22_123 = RLcp22_119+RLcp22_120+RLcp22_121+RLcp22_122+RLcp22_123+q(1);
    POcp22_223 = RLcp22_219+RLcp22_220+RLcp22_221+RLcp22_222+RLcp22_223+q(2);
    POcp22_323 = RLcp22_319+RLcp22_320+RLcp22_321+RLcp22_322+RLcp22_323+q(3);
    OMcp22_123 = OMcp22_122+ROcp22_122*qd(23);
    OMcp22_223 = OMcp22_222+ROcp22_222*qd(23);
    OMcp22_323 = OMcp22_322+ROcp22_322*qd(23);
    ORcp22_123 = OMcp22_222*RLcp22_323-OMcp22_322*RLcp22_223;
    ORcp22_223 = -(OMcp22_122*RLcp22_323-OMcp22_322*RLcp22_123);
    ORcp22_323 = OMcp22_122*RLcp22_223-OMcp22_222*RLcp22_123;
    VIcp22_123 = ORcp22_119+ORcp22_120+ORcp22_121+ORcp22_122+ORcp22_123+qd(1);
    VIcp22_223 = ORcp22_219+ORcp22_220+ORcp22_221+ORcp22_222+ORcp22_223+qd(2);
    VIcp22_323 = ORcp22_319+ORcp22_320+ORcp22_321+ORcp22_322+ORcp22_323+qd(3);
    OPcp22_123 = OPcp22_122+ROcp22_122*qdd(23)+qd(23)*(OMcp22_222*ROcp22_322-OMcp22_322*ROcp22_222);
    OPcp22_223 = OPcp22_222+ROcp22_222*qdd(23)-qd(23)*(OMcp22_122*ROcp22_322-OMcp22_322*ROcp22_122);
    OPcp22_323 = OPcp22_322+ROcp22_322*qdd(23)+qd(23)*(OMcp22_122*ROcp22_222-OMcp22_222*ROcp22_122);
    ACcp22_123 = qdd(1)+OMcp22_219*ORcp22_320+OMcp22_220*ORcp22_321+OMcp22_221*ORcp22_322+OMcp22_222*ORcp22_323+OMcp22_26*ORcp22_319-OMcp22_319*...
 ORcp22_220-OMcp22_320*ORcp22_221-OMcp22_321*ORcp22_222-OMcp22_322*ORcp22_223-OMcp22_36*ORcp22_219+OPcp22_219*RLcp22_320+OPcp22_220*RLcp22_321+...
 OPcp22_221*RLcp22_322+OPcp22_222*RLcp22_323+OPcp22_26*RLcp22_319-OPcp22_319*RLcp22_220-OPcp22_320*RLcp22_221-OPcp22_321*RLcp22_222-OPcp22_322*...
 RLcp22_223-OPcp22_36*RLcp22_219;
    ACcp22_223 = qdd(2)-OMcp22_119*ORcp22_320-OMcp22_120*ORcp22_321-OMcp22_121*ORcp22_322-OMcp22_122*ORcp22_323-OMcp22_16*ORcp22_319+OMcp22_319*...
 ORcp22_120+OMcp22_320*ORcp22_121+OMcp22_321*ORcp22_122+OMcp22_322*ORcp22_123+OMcp22_36*ORcp22_119-OPcp22_119*RLcp22_320-OPcp22_120*RLcp22_321-...
 OPcp22_121*RLcp22_322-OPcp22_122*RLcp22_323-OPcp22_16*RLcp22_319+OPcp22_319*RLcp22_120+OPcp22_320*RLcp22_121+OPcp22_321*RLcp22_122+OPcp22_322*...
 RLcp22_123+OPcp22_36*RLcp22_119;
    ACcp22_323 = qdd(3)+OMcp22_119*ORcp22_220+OMcp22_120*ORcp22_221+OMcp22_121*ORcp22_222+OMcp22_122*ORcp22_223+OMcp22_16*ORcp22_219-OMcp22_219*...
 ORcp22_120-OMcp22_220*ORcp22_121-OMcp22_221*ORcp22_122-OMcp22_222*ORcp22_123-OMcp22_26*ORcp22_119+OPcp22_119*RLcp22_220+OPcp22_120*RLcp22_221+...
 OPcp22_121*RLcp22_222+OPcp22_122*RLcp22_223+OPcp22_16*RLcp22_219-OPcp22_219*RLcp22_120-OPcp22_220*RLcp22_121-OPcp22_221*RLcp22_122-OPcp22_222*...
 RLcp22_123-OPcp22_26*RLcp22_119;

% = = Block_1_0_0_23_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp22_123;
    sens.P(2) = POcp22_223;
    sens.P(3) = POcp22_323;
    sens.R(1,1) = ROcp22_122;
    sens.R(1,2) = ROcp22_222;
    sens.R(1,3) = ROcp22_322;
    sens.R(2,1) = ROcp22_423;
    sens.R(2,2) = ROcp22_523;
    sens.R(2,3) = ROcp22_623;
    sens.R(3,1) = ROcp22_723;
    sens.R(3,2) = ROcp22_823;
    sens.R(3,3) = ROcp22_923;
    sens.V(1) = VIcp22_123;
    sens.V(2) = VIcp22_223;
    sens.V(3) = VIcp22_323;
    sens.OM(1) = OMcp22_123;
    sens.OM(2) = OMcp22_223;
    sens.OM(3) = OMcp22_323;
    sens.A(1) = ACcp22_123;
    sens.A(2) = ACcp22_223;
    sens.A(3) = ACcp22_323;
    sens.OMP(1) = OPcp22_123;
    sens.OMP(2) = OPcp22_223;
    sens.OMP(3) = OPcp22_323;
 
% 
case 24, 


% = = Block_1_0_0_24_0_1 = = 
 
% Sensor Kinematics 


    ROcp23_25 = S4*S5;
    ROcp23_35 = -C4*S5;
    ROcp23_85 = -S4*C5;
    ROcp23_95 = C4*C5;
    ROcp23_16 = C5*C6;
    ROcp23_26 = ROcp23_25*C6+C4*S6;
    ROcp23_36 = ROcp23_35*C6+S4*S6;
    ROcp23_46 = -C5*S6;
    ROcp23_56 = -(ROcp23_25*S6-C4*C6);
    ROcp23_66 = -(ROcp23_35*S6-S4*C6);
    OMcp23_25 = qd(5)*C4;
    OMcp23_35 = qd(5)*S4;
    OMcp23_16 = qd(4)+qd(6)*S5;
    OMcp23_26 = OMcp23_25+ROcp23_85*qd(6);
    OMcp23_36 = OMcp23_35+ROcp23_95*qd(6);
    OPcp23_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp23_26 = ROcp23_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp23_35*S5-ROcp23_95*qd(4));
    OPcp23_36 = ROcp23_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp23_25*S5-ROcp23_85*qd(4));

% = = Block_1_0_0_24_0_4 = = 
 
% Sensor Kinematics 


    ROcp23_419 = ROcp23_46*C19+S19*S5;
    ROcp23_519 = ROcp23_56*C19+ROcp23_85*S19;
    ROcp23_619 = ROcp23_66*C19+ROcp23_95*S19;
    ROcp23_719 = -(ROcp23_46*S19-C19*S5);
    ROcp23_819 = -(ROcp23_56*S19-ROcp23_85*C19);
    ROcp23_919 = -(ROcp23_66*S19-ROcp23_95*C19);
    ROcp23_120 = ROcp23_16*C20-ROcp23_719*S20;
    ROcp23_220 = ROcp23_26*C20-ROcp23_819*S20;
    ROcp23_320 = ROcp23_36*C20-ROcp23_919*S20;
    ROcp23_720 = ROcp23_16*S20+ROcp23_719*C20;
    ROcp23_820 = ROcp23_26*S20+ROcp23_819*C20;
    ROcp23_920 = ROcp23_36*S20+ROcp23_919*C20;
    ROcp23_121 = ROcp23_120*C21+ROcp23_419*S21;
    ROcp23_221 = ROcp23_220*C21+ROcp23_519*S21;
    ROcp23_321 = ROcp23_320*C21+ROcp23_619*S21;
    ROcp23_421 = -(ROcp23_120*S21-ROcp23_419*C21);
    ROcp23_521 = -(ROcp23_220*S21-ROcp23_519*C21);
    ROcp23_621 = -(ROcp23_320*S21-ROcp23_619*C21);
    RLcp23_119 = s.dpt(2,3)*ROcp23_46+ROcp23_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp23_219 = s.dpt(2,3)*ROcp23_56+ROcp23_26*s.dpt(1,3)+ROcp23_85*s.dpt(3,3);
    RLcp23_319 = s.dpt(2,3)*ROcp23_66+ROcp23_36*s.dpt(1,3)+ROcp23_95*s.dpt(3,3);
    OMcp23_119 = OMcp23_16+ROcp23_16*qd(19);
    OMcp23_219 = OMcp23_26+ROcp23_26*qd(19);
    OMcp23_319 = OMcp23_36+ROcp23_36*qd(19);
    ORcp23_119 = OMcp23_26*RLcp23_319-OMcp23_36*RLcp23_219;
    ORcp23_219 = -(OMcp23_16*RLcp23_319-OMcp23_36*RLcp23_119);
    ORcp23_319 = OMcp23_16*RLcp23_219-OMcp23_26*RLcp23_119;
    OPcp23_119 = OPcp23_16+ROcp23_16*qdd(19)+qd(19)*(OMcp23_26*ROcp23_36-OMcp23_36*ROcp23_26);
    OPcp23_219 = OPcp23_26+ROcp23_26*qdd(19)-qd(19)*(OMcp23_16*ROcp23_36-OMcp23_36*ROcp23_16);
    OPcp23_319 = OPcp23_36+ROcp23_36*qdd(19)+qd(19)*(OMcp23_16*ROcp23_26-OMcp23_26*ROcp23_16);
    RLcp23_120 = s.dpt(1,30)*ROcp23_16+s.dpt(2,30)*ROcp23_419+s.dpt(3,30)*ROcp23_719;
    RLcp23_220 = s.dpt(1,30)*ROcp23_26+s.dpt(2,30)*ROcp23_519+s.dpt(3,30)*ROcp23_819;
    RLcp23_320 = s.dpt(1,30)*ROcp23_36+s.dpt(2,30)*ROcp23_619+s.dpt(3,30)*ROcp23_919;
    OMcp23_120 = OMcp23_119+ROcp23_419*qd(20);
    OMcp23_220 = OMcp23_219+ROcp23_519*qd(20);
    OMcp23_320 = OMcp23_319+ROcp23_619*qd(20);
    ORcp23_120 = OMcp23_219*RLcp23_320-OMcp23_319*RLcp23_220;
    ORcp23_220 = -(OMcp23_119*RLcp23_320-OMcp23_319*RLcp23_120);
    ORcp23_320 = OMcp23_119*RLcp23_220-OMcp23_219*RLcp23_120;
    OPcp23_120 = OPcp23_119+ROcp23_419*qdd(20)+qd(20)*(OMcp23_219*ROcp23_619-OMcp23_319*ROcp23_519);
    OPcp23_220 = OPcp23_219+ROcp23_519*qdd(20)-qd(20)*(OMcp23_119*ROcp23_619-OMcp23_319*ROcp23_419);
    OPcp23_320 = OPcp23_319+ROcp23_619*qdd(20)+qd(20)*(OMcp23_119*ROcp23_519-OMcp23_219*ROcp23_419);
    RLcp23_121 = s.dpt(1,32)*ROcp23_120+s.dpt(2,32)*ROcp23_419+ROcp23_720*s.dpt(3,32);
    RLcp23_221 = s.dpt(1,32)*ROcp23_220+s.dpt(2,32)*ROcp23_519+ROcp23_820*s.dpt(3,32);
    RLcp23_321 = s.dpt(1,32)*ROcp23_320+s.dpt(2,32)*ROcp23_619+ROcp23_920*s.dpt(3,32);
    OMcp23_121 = OMcp23_120+ROcp23_720*qd(21);
    OMcp23_221 = OMcp23_220+ROcp23_820*qd(21);
    OMcp23_321 = OMcp23_320+ROcp23_920*qd(21);
    ORcp23_121 = OMcp23_220*RLcp23_321-OMcp23_320*RLcp23_221;
    ORcp23_221 = -(OMcp23_120*RLcp23_321-OMcp23_320*RLcp23_121);
    ORcp23_321 = OMcp23_120*RLcp23_221-OMcp23_220*RLcp23_121;
    OPcp23_121 = OPcp23_120+ROcp23_720*qdd(21)+qd(21)*(OMcp23_220*ROcp23_920-OMcp23_320*ROcp23_820);
    OPcp23_221 = OPcp23_220+ROcp23_820*qdd(21)-qd(21)*(OMcp23_120*ROcp23_920-OMcp23_320*ROcp23_720);
    OPcp23_321 = OPcp23_320+ROcp23_920*qdd(21)+qd(21)*(OMcp23_120*ROcp23_820-OMcp23_220*ROcp23_720);

% = = Block_1_0_0_24_0_5 = = 
 
% Sensor Kinematics 


    ROcp23_122 = ROcp23_121*C22-ROcp23_720*S22;
    ROcp23_222 = ROcp23_221*C22-ROcp23_820*S22;
    ROcp23_322 = ROcp23_321*C22-ROcp23_920*S22;
    ROcp23_722 = ROcp23_121*S22+ROcp23_720*C22;
    ROcp23_822 = ROcp23_221*S22+ROcp23_820*C22;
    ROcp23_922 = ROcp23_321*S22+ROcp23_920*C22;
    ROcp23_423 = ROcp23_421*C23+ROcp23_722*S23;
    ROcp23_523 = ROcp23_521*C23+ROcp23_822*S23;
    ROcp23_623 = ROcp23_621*C23+ROcp23_922*S23;
    ROcp23_723 = -(ROcp23_421*S23-ROcp23_722*C23);
    ROcp23_823 = -(ROcp23_521*S23-ROcp23_822*C23);
    ROcp23_923 = -(ROcp23_621*S23-ROcp23_922*C23);
    ROcp23_124 = ROcp23_122*C24-ROcp23_723*S24;
    ROcp23_224 = ROcp23_222*C24-ROcp23_823*S24;
    ROcp23_324 = ROcp23_322*C24-ROcp23_923*S24;
    ROcp23_724 = ROcp23_122*S24+ROcp23_723*C24;
    ROcp23_824 = ROcp23_222*S24+ROcp23_823*C24;
    ROcp23_924 = ROcp23_322*S24+ROcp23_923*C24;
    RLcp23_122 = ROcp23_121*s.dpt(1,36)+ROcp23_421*s.dpt(2,36)+ROcp23_720*s.dpt(3,36);
    RLcp23_222 = ROcp23_221*s.dpt(1,36)+ROcp23_521*s.dpt(2,36)+ROcp23_820*s.dpt(3,36);
    RLcp23_322 = ROcp23_321*s.dpt(1,36)+ROcp23_621*s.dpt(2,36)+ROcp23_920*s.dpt(3,36);
    OMcp23_122 = OMcp23_121+ROcp23_421*qd(22);
    OMcp23_222 = OMcp23_221+ROcp23_521*qd(22);
    OMcp23_322 = OMcp23_321+ROcp23_621*qd(22);
    ORcp23_122 = OMcp23_221*RLcp23_322-OMcp23_321*RLcp23_222;
    ORcp23_222 = -(OMcp23_121*RLcp23_322-OMcp23_321*RLcp23_122);
    ORcp23_322 = OMcp23_121*RLcp23_222-OMcp23_221*RLcp23_122;
    OPcp23_122 = OPcp23_121+ROcp23_421*qdd(22)+qd(22)*(OMcp23_221*ROcp23_621-OMcp23_321*ROcp23_521);
    OPcp23_222 = OPcp23_221+ROcp23_521*qdd(22)-qd(22)*(OMcp23_121*ROcp23_621-OMcp23_321*ROcp23_421);
    OPcp23_322 = OPcp23_321+ROcp23_621*qdd(22)+qd(22)*(OMcp23_121*ROcp23_521-OMcp23_221*ROcp23_421);
    RLcp23_123 = s.dpt(1,40)*ROcp23_122+s.dpt(3,40)*ROcp23_722+ROcp23_421*s.dpt(2,40);
    RLcp23_223 = s.dpt(1,40)*ROcp23_222+s.dpt(3,40)*ROcp23_822+ROcp23_521*s.dpt(2,40);
    RLcp23_323 = s.dpt(1,40)*ROcp23_322+s.dpt(3,40)*ROcp23_922+ROcp23_621*s.dpt(2,40);
    OMcp23_123 = OMcp23_122+ROcp23_122*qd(23);
    OMcp23_223 = OMcp23_222+ROcp23_222*qd(23);
    OMcp23_323 = OMcp23_322+ROcp23_322*qd(23);
    ORcp23_123 = OMcp23_222*RLcp23_323-OMcp23_322*RLcp23_223;
    ORcp23_223 = -(OMcp23_122*RLcp23_323-OMcp23_322*RLcp23_123);
    ORcp23_323 = OMcp23_122*RLcp23_223-OMcp23_222*RLcp23_123;
    OPcp23_123 = OPcp23_122+ROcp23_122*qdd(23)+qd(23)*(OMcp23_222*ROcp23_322-OMcp23_322*ROcp23_222);
    OPcp23_223 = OPcp23_222+ROcp23_222*qdd(23)-qd(23)*(OMcp23_122*ROcp23_322-OMcp23_322*ROcp23_122);
    OPcp23_323 = OPcp23_322+ROcp23_322*qdd(23)+qd(23)*(OMcp23_122*ROcp23_222-OMcp23_222*ROcp23_122);
    RLcp23_124 = s.dpt(1,42)*ROcp23_122+s.dpt(3,42)*ROcp23_723+ROcp23_423*s.dpt(2,42);
    RLcp23_224 = s.dpt(1,42)*ROcp23_222+s.dpt(3,42)*ROcp23_823+ROcp23_523*s.dpt(2,42);
    RLcp23_324 = s.dpt(1,42)*ROcp23_322+s.dpt(3,42)*ROcp23_923+ROcp23_623*s.dpt(2,42);
    POcp23_124 = RLcp23_119+RLcp23_120+RLcp23_121+RLcp23_122+RLcp23_123+RLcp23_124+q(1);
    POcp23_224 = RLcp23_219+RLcp23_220+RLcp23_221+RLcp23_222+RLcp23_223+RLcp23_224+q(2);
    POcp23_324 = RLcp23_319+RLcp23_320+RLcp23_321+RLcp23_322+RLcp23_323+RLcp23_324+q(3);
    OMcp23_124 = OMcp23_123+ROcp23_423*qd(24);
    OMcp23_224 = OMcp23_223+ROcp23_523*qd(24);
    OMcp23_324 = OMcp23_323+ROcp23_623*qd(24);
    ORcp23_124 = OMcp23_223*RLcp23_324-OMcp23_323*RLcp23_224;
    ORcp23_224 = -(OMcp23_123*RLcp23_324-OMcp23_323*RLcp23_124);
    ORcp23_324 = OMcp23_123*RLcp23_224-OMcp23_223*RLcp23_124;
    VIcp23_124 = ORcp23_119+ORcp23_120+ORcp23_121+ORcp23_122+ORcp23_123+ORcp23_124+qd(1);
    VIcp23_224 = ORcp23_219+ORcp23_220+ORcp23_221+ORcp23_222+ORcp23_223+ORcp23_224+qd(2);
    VIcp23_324 = ORcp23_319+ORcp23_320+ORcp23_321+ORcp23_322+ORcp23_323+ORcp23_324+qd(3);
    OPcp23_124 = OPcp23_123+ROcp23_423*qdd(24)+qd(24)*(OMcp23_223*ROcp23_623-OMcp23_323*ROcp23_523);
    OPcp23_224 = OPcp23_223+ROcp23_523*qdd(24)-qd(24)*(OMcp23_123*ROcp23_623-OMcp23_323*ROcp23_423);
    OPcp23_324 = OPcp23_323+ROcp23_623*qdd(24)+qd(24)*(OMcp23_123*ROcp23_523-OMcp23_223*ROcp23_423);
    ACcp23_124 = qdd(1)+OMcp23_219*ORcp23_320+OMcp23_220*ORcp23_321+OMcp23_221*ORcp23_322+OMcp23_222*ORcp23_323+OMcp23_223*ORcp23_324+OMcp23_26*...
 ORcp23_319-OMcp23_319*ORcp23_220-OMcp23_320*ORcp23_221-OMcp23_321*ORcp23_222-OMcp23_322*ORcp23_223-OMcp23_323*ORcp23_224-OMcp23_36*ORcp23_219+...
 OPcp23_219*RLcp23_320+OPcp23_220*RLcp23_321+OPcp23_221*RLcp23_322+OPcp23_222*RLcp23_323+OPcp23_223*RLcp23_324+OPcp23_26*RLcp23_319-OPcp23_319*...
 RLcp23_220-OPcp23_320*RLcp23_221-OPcp23_321*RLcp23_222-OPcp23_322*RLcp23_223-OPcp23_323*RLcp23_224-OPcp23_36*RLcp23_219;
    ACcp23_224 = qdd(2)-OMcp23_119*ORcp23_320-OMcp23_120*ORcp23_321-OMcp23_121*ORcp23_322-OMcp23_122*ORcp23_323-OMcp23_123*ORcp23_324-OMcp23_16*...
 ORcp23_319+OMcp23_319*ORcp23_120+OMcp23_320*ORcp23_121+OMcp23_321*ORcp23_122+OMcp23_322*ORcp23_123+OMcp23_323*ORcp23_124+OMcp23_36*ORcp23_119-...
 OPcp23_119*RLcp23_320-OPcp23_120*RLcp23_321-OPcp23_121*RLcp23_322-OPcp23_122*RLcp23_323-OPcp23_123*RLcp23_324-OPcp23_16*RLcp23_319+OPcp23_319*...
 RLcp23_120+OPcp23_320*RLcp23_121+OPcp23_321*RLcp23_122+OPcp23_322*RLcp23_123+OPcp23_323*RLcp23_124+OPcp23_36*RLcp23_119;
    ACcp23_324 = qdd(3)+OMcp23_119*ORcp23_220+OMcp23_120*ORcp23_221+OMcp23_121*ORcp23_222+OMcp23_122*ORcp23_223+OMcp23_123*ORcp23_224+OMcp23_16*...
 ORcp23_219-OMcp23_219*ORcp23_120-OMcp23_220*ORcp23_121-OMcp23_221*ORcp23_122-OMcp23_222*ORcp23_123-OMcp23_223*ORcp23_124-OMcp23_26*ORcp23_119+...
 OPcp23_119*RLcp23_220+OPcp23_120*RLcp23_221+OPcp23_121*RLcp23_222+OPcp23_122*RLcp23_223+OPcp23_123*RLcp23_224+OPcp23_16*RLcp23_219-OPcp23_219*...
 RLcp23_120-OPcp23_220*RLcp23_121-OPcp23_221*RLcp23_122-OPcp23_222*RLcp23_123-OPcp23_223*RLcp23_124-OPcp23_26*RLcp23_119;

% = = Block_1_0_0_24_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp23_124;
    sens.P(2) = POcp23_224;
    sens.P(3) = POcp23_324;
    sens.R(1,1) = ROcp23_124;
    sens.R(1,2) = ROcp23_224;
    sens.R(1,3) = ROcp23_324;
    sens.R(2,1) = ROcp23_423;
    sens.R(2,2) = ROcp23_523;
    sens.R(2,3) = ROcp23_623;
    sens.R(3,1) = ROcp23_724;
    sens.R(3,2) = ROcp23_824;
    sens.R(3,3) = ROcp23_924;
    sens.V(1) = VIcp23_124;
    sens.V(2) = VIcp23_224;
    sens.V(3) = VIcp23_324;
    sens.OM(1) = OMcp23_124;
    sens.OM(2) = OMcp23_224;
    sens.OM(3) = OMcp23_324;
    sens.A(1) = ACcp23_124;
    sens.A(2) = ACcp23_224;
    sens.A(3) = ACcp23_324;
    sens.OMP(1) = OPcp23_124;
    sens.OMP(2) = OPcp23_224;
    sens.OMP(3) = OPcp23_324;
 
% 
case 25, 


% = = Block_1_0_0_25_0_1 = = 
 
% Sensor Kinematics 


    ROcp24_25 = S4*S5;
    ROcp24_35 = -C4*S5;
    ROcp24_85 = -S4*C5;
    ROcp24_95 = C4*C5;
    ROcp24_16 = C5*C6;
    ROcp24_26 = ROcp24_25*C6+C4*S6;
    ROcp24_36 = ROcp24_35*C6+S4*S6;
    ROcp24_46 = -C5*S6;
    ROcp24_56 = -(ROcp24_25*S6-C4*C6);
    ROcp24_66 = -(ROcp24_35*S6-S4*C6);
    OMcp24_25 = qd(5)*C4;
    OMcp24_35 = qd(5)*S4;
    OMcp24_16 = qd(4)+qd(6)*S5;
    OMcp24_26 = OMcp24_25+ROcp24_85*qd(6);
    OMcp24_36 = OMcp24_35+ROcp24_95*qd(6);
    OPcp24_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp24_26 = ROcp24_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp24_35*S5-ROcp24_95*qd(4));
    OPcp24_36 = ROcp24_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp24_25*S5-ROcp24_85*qd(4));

% = = Block_1_0_0_25_0_4 = = 
 
% Sensor Kinematics 


    ROcp24_419 = ROcp24_46*C19+S19*S5;
    ROcp24_519 = ROcp24_56*C19+ROcp24_85*S19;
    ROcp24_619 = ROcp24_66*C19+ROcp24_95*S19;
    ROcp24_719 = -(ROcp24_46*S19-C19*S5);
    ROcp24_819 = -(ROcp24_56*S19-ROcp24_85*C19);
    ROcp24_919 = -(ROcp24_66*S19-ROcp24_95*C19);
    ROcp24_120 = ROcp24_16*C20-ROcp24_719*S20;
    ROcp24_220 = ROcp24_26*C20-ROcp24_819*S20;
    ROcp24_320 = ROcp24_36*C20-ROcp24_919*S20;
    ROcp24_720 = ROcp24_16*S20+ROcp24_719*C20;
    ROcp24_820 = ROcp24_26*S20+ROcp24_819*C20;
    ROcp24_920 = ROcp24_36*S20+ROcp24_919*C20;
    ROcp24_121 = ROcp24_120*C21+ROcp24_419*S21;
    ROcp24_221 = ROcp24_220*C21+ROcp24_519*S21;
    ROcp24_321 = ROcp24_320*C21+ROcp24_619*S21;
    ROcp24_421 = -(ROcp24_120*S21-ROcp24_419*C21);
    ROcp24_521 = -(ROcp24_220*S21-ROcp24_519*C21);
    ROcp24_621 = -(ROcp24_320*S21-ROcp24_619*C21);
    RLcp24_119 = s.dpt(2,3)*ROcp24_46+ROcp24_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp24_219 = s.dpt(2,3)*ROcp24_56+ROcp24_26*s.dpt(1,3)+ROcp24_85*s.dpt(3,3);
    RLcp24_319 = s.dpt(2,3)*ROcp24_66+ROcp24_36*s.dpt(1,3)+ROcp24_95*s.dpt(3,3);
    OMcp24_119 = OMcp24_16+ROcp24_16*qd(19);
    OMcp24_219 = OMcp24_26+ROcp24_26*qd(19);
    OMcp24_319 = OMcp24_36+ROcp24_36*qd(19);
    ORcp24_119 = OMcp24_26*RLcp24_319-OMcp24_36*RLcp24_219;
    ORcp24_219 = -(OMcp24_16*RLcp24_319-OMcp24_36*RLcp24_119);
    ORcp24_319 = OMcp24_16*RLcp24_219-OMcp24_26*RLcp24_119;
    OPcp24_119 = OPcp24_16+ROcp24_16*qdd(19)+qd(19)*(OMcp24_26*ROcp24_36-OMcp24_36*ROcp24_26);
    OPcp24_219 = OPcp24_26+ROcp24_26*qdd(19)-qd(19)*(OMcp24_16*ROcp24_36-OMcp24_36*ROcp24_16);
    OPcp24_319 = OPcp24_36+ROcp24_36*qdd(19)+qd(19)*(OMcp24_16*ROcp24_26-OMcp24_26*ROcp24_16);
    RLcp24_120 = s.dpt(1,30)*ROcp24_16+s.dpt(2,30)*ROcp24_419+s.dpt(3,30)*ROcp24_719;
    RLcp24_220 = s.dpt(1,30)*ROcp24_26+s.dpt(2,30)*ROcp24_519+s.dpt(3,30)*ROcp24_819;
    RLcp24_320 = s.dpt(1,30)*ROcp24_36+s.dpt(2,30)*ROcp24_619+s.dpt(3,30)*ROcp24_919;
    OMcp24_120 = OMcp24_119+ROcp24_419*qd(20);
    OMcp24_220 = OMcp24_219+ROcp24_519*qd(20);
    OMcp24_320 = OMcp24_319+ROcp24_619*qd(20);
    ORcp24_120 = OMcp24_219*RLcp24_320-OMcp24_319*RLcp24_220;
    ORcp24_220 = -(OMcp24_119*RLcp24_320-OMcp24_319*RLcp24_120);
    ORcp24_320 = OMcp24_119*RLcp24_220-OMcp24_219*RLcp24_120;
    OPcp24_120 = OPcp24_119+ROcp24_419*qdd(20)+qd(20)*(OMcp24_219*ROcp24_619-OMcp24_319*ROcp24_519);
    OPcp24_220 = OPcp24_219+ROcp24_519*qdd(20)-qd(20)*(OMcp24_119*ROcp24_619-OMcp24_319*ROcp24_419);
    OPcp24_320 = OPcp24_319+ROcp24_619*qdd(20)+qd(20)*(OMcp24_119*ROcp24_519-OMcp24_219*ROcp24_419);
    RLcp24_121 = s.dpt(1,32)*ROcp24_120+s.dpt(2,32)*ROcp24_419+ROcp24_720*s.dpt(3,32);
    RLcp24_221 = s.dpt(1,32)*ROcp24_220+s.dpt(2,32)*ROcp24_519+ROcp24_820*s.dpt(3,32);
    RLcp24_321 = s.dpt(1,32)*ROcp24_320+s.dpt(2,32)*ROcp24_619+ROcp24_920*s.dpt(3,32);
    OMcp24_121 = OMcp24_120+ROcp24_720*qd(21);
    OMcp24_221 = OMcp24_220+ROcp24_820*qd(21);
    OMcp24_321 = OMcp24_320+ROcp24_920*qd(21);
    ORcp24_121 = OMcp24_220*RLcp24_321-OMcp24_320*RLcp24_221;
    ORcp24_221 = -(OMcp24_120*RLcp24_321-OMcp24_320*RLcp24_121);
    ORcp24_321 = OMcp24_120*RLcp24_221-OMcp24_220*RLcp24_121;
    OPcp24_121 = OPcp24_120+ROcp24_720*qdd(21)+qd(21)*(OMcp24_220*ROcp24_920-OMcp24_320*ROcp24_820);
    OPcp24_221 = OPcp24_220+ROcp24_820*qdd(21)-qd(21)*(OMcp24_120*ROcp24_920-OMcp24_320*ROcp24_720);
    OPcp24_321 = OPcp24_320+ROcp24_920*qdd(21)+qd(21)*(OMcp24_120*ROcp24_820-OMcp24_220*ROcp24_720);

% = = Block_1_0_0_25_0_5 = = 
 
% Sensor Kinematics 


    ROcp24_122 = ROcp24_121*C22-ROcp24_720*S22;
    ROcp24_222 = ROcp24_221*C22-ROcp24_820*S22;
    ROcp24_322 = ROcp24_321*C22-ROcp24_920*S22;
    ROcp24_722 = ROcp24_121*S22+ROcp24_720*C22;
    ROcp24_822 = ROcp24_221*S22+ROcp24_820*C22;
    ROcp24_922 = ROcp24_321*S22+ROcp24_920*C22;
    ROcp24_423 = ROcp24_421*C23+ROcp24_722*S23;
    ROcp24_523 = ROcp24_521*C23+ROcp24_822*S23;
    ROcp24_623 = ROcp24_621*C23+ROcp24_922*S23;
    ROcp24_723 = -(ROcp24_421*S23-ROcp24_722*C23);
    ROcp24_823 = -(ROcp24_521*S23-ROcp24_822*C23);
    ROcp24_923 = -(ROcp24_621*S23-ROcp24_922*C23);
    ROcp24_124 = ROcp24_122*C24-ROcp24_723*S24;
    ROcp24_224 = ROcp24_222*C24-ROcp24_823*S24;
    ROcp24_324 = ROcp24_322*C24-ROcp24_923*S24;
    ROcp24_724 = ROcp24_122*S24+ROcp24_723*C24;
    ROcp24_824 = ROcp24_222*S24+ROcp24_823*C24;
    ROcp24_924 = ROcp24_322*S24+ROcp24_923*C24;
    ROcp24_125 = ROcp24_124*C25+ROcp24_423*S25;
    ROcp24_225 = ROcp24_224*C25+ROcp24_523*S25;
    ROcp24_325 = ROcp24_324*C25+ROcp24_623*S25;
    ROcp24_425 = -(ROcp24_124*S25-ROcp24_423*C25);
    ROcp24_525 = -(ROcp24_224*S25-ROcp24_523*C25);
    ROcp24_625 = -(ROcp24_324*S25-ROcp24_623*C25);
    RLcp24_122 = ROcp24_121*s.dpt(1,36)+ROcp24_421*s.dpt(2,36)+ROcp24_720*s.dpt(3,36);
    RLcp24_222 = ROcp24_221*s.dpt(1,36)+ROcp24_521*s.dpt(2,36)+ROcp24_820*s.dpt(3,36);
    RLcp24_322 = ROcp24_321*s.dpt(1,36)+ROcp24_621*s.dpt(2,36)+ROcp24_920*s.dpt(3,36);
    OMcp24_122 = OMcp24_121+ROcp24_421*qd(22);
    OMcp24_222 = OMcp24_221+ROcp24_521*qd(22);
    OMcp24_322 = OMcp24_321+ROcp24_621*qd(22);
    ORcp24_122 = OMcp24_221*RLcp24_322-OMcp24_321*RLcp24_222;
    ORcp24_222 = -(OMcp24_121*RLcp24_322-OMcp24_321*RLcp24_122);
    ORcp24_322 = OMcp24_121*RLcp24_222-OMcp24_221*RLcp24_122;
    OPcp24_122 = OPcp24_121+ROcp24_421*qdd(22)+qd(22)*(OMcp24_221*ROcp24_621-OMcp24_321*ROcp24_521);
    OPcp24_222 = OPcp24_221+ROcp24_521*qdd(22)-qd(22)*(OMcp24_121*ROcp24_621-OMcp24_321*ROcp24_421);
    OPcp24_322 = OPcp24_321+ROcp24_621*qdd(22)+qd(22)*(OMcp24_121*ROcp24_521-OMcp24_221*ROcp24_421);
    RLcp24_123 = s.dpt(1,40)*ROcp24_122+s.dpt(3,40)*ROcp24_722+ROcp24_421*s.dpt(2,40);
    RLcp24_223 = s.dpt(1,40)*ROcp24_222+s.dpt(3,40)*ROcp24_822+ROcp24_521*s.dpt(2,40);
    RLcp24_323 = s.dpt(1,40)*ROcp24_322+s.dpt(3,40)*ROcp24_922+ROcp24_621*s.dpt(2,40);
    OMcp24_123 = OMcp24_122+ROcp24_122*qd(23);
    OMcp24_223 = OMcp24_222+ROcp24_222*qd(23);
    OMcp24_323 = OMcp24_322+ROcp24_322*qd(23);
    ORcp24_123 = OMcp24_222*RLcp24_323-OMcp24_322*RLcp24_223;
    ORcp24_223 = -(OMcp24_122*RLcp24_323-OMcp24_322*RLcp24_123);
    ORcp24_323 = OMcp24_122*RLcp24_223-OMcp24_222*RLcp24_123;
    OPcp24_123 = OPcp24_122+ROcp24_122*qdd(23)+qd(23)*(OMcp24_222*ROcp24_322-OMcp24_322*ROcp24_222);
    OPcp24_223 = OPcp24_222+ROcp24_222*qdd(23)-qd(23)*(OMcp24_122*ROcp24_322-OMcp24_322*ROcp24_122);
    OPcp24_323 = OPcp24_322+ROcp24_322*qdd(23)+qd(23)*(OMcp24_122*ROcp24_222-OMcp24_222*ROcp24_122);
    RLcp24_124 = s.dpt(1,42)*ROcp24_122+s.dpt(3,42)*ROcp24_723+ROcp24_423*s.dpt(2,42);
    RLcp24_224 = s.dpt(1,42)*ROcp24_222+s.dpt(3,42)*ROcp24_823+ROcp24_523*s.dpt(2,42);
    RLcp24_324 = s.dpt(1,42)*ROcp24_322+s.dpt(3,42)*ROcp24_923+ROcp24_623*s.dpt(2,42);
    OMcp24_124 = OMcp24_123+ROcp24_423*qd(24);
    OMcp24_224 = OMcp24_223+ROcp24_523*qd(24);
    OMcp24_324 = OMcp24_323+ROcp24_623*qd(24);
    ORcp24_124 = OMcp24_223*RLcp24_324-OMcp24_323*RLcp24_224;
    ORcp24_224 = -(OMcp24_123*RLcp24_324-OMcp24_323*RLcp24_124);
    ORcp24_324 = OMcp24_123*RLcp24_224-OMcp24_223*RLcp24_124;
    OPcp24_124 = OPcp24_123+ROcp24_423*qdd(24)+qd(24)*(OMcp24_223*ROcp24_623-OMcp24_323*ROcp24_523);
    OPcp24_224 = OPcp24_223+ROcp24_523*qdd(24)-qd(24)*(OMcp24_123*ROcp24_623-OMcp24_323*ROcp24_423);
    OPcp24_324 = OPcp24_323+ROcp24_623*qdd(24)+qd(24)*(OMcp24_123*ROcp24_523-OMcp24_223*ROcp24_423);
    RLcp24_125 = s.dpt(3,44)*ROcp24_724+ROcp24_124*s.dpt(1,44)+ROcp24_423*s.dpt(2,44);
    RLcp24_225 = s.dpt(3,44)*ROcp24_824+ROcp24_224*s.dpt(1,44)+ROcp24_523*s.dpt(2,44);
    RLcp24_325 = s.dpt(3,44)*ROcp24_924+ROcp24_324*s.dpt(1,44)+ROcp24_623*s.dpt(2,44);
    POcp24_125 = RLcp24_119+RLcp24_120+RLcp24_121+RLcp24_122+RLcp24_123+RLcp24_124+RLcp24_125+q(1);
    POcp24_225 = RLcp24_219+RLcp24_220+RLcp24_221+RLcp24_222+RLcp24_223+RLcp24_224+RLcp24_225+q(2);
    POcp24_325 = RLcp24_319+RLcp24_320+RLcp24_321+RLcp24_322+RLcp24_323+RLcp24_324+RLcp24_325+q(3);
    OMcp24_125 = OMcp24_124+ROcp24_724*qd(25);
    OMcp24_225 = OMcp24_224+ROcp24_824*qd(25);
    OMcp24_325 = OMcp24_324+ROcp24_924*qd(25);
    ORcp24_125 = OMcp24_224*RLcp24_325-OMcp24_324*RLcp24_225;
    ORcp24_225 = -(OMcp24_124*RLcp24_325-OMcp24_324*RLcp24_125);
    ORcp24_325 = OMcp24_124*RLcp24_225-OMcp24_224*RLcp24_125;
    VIcp24_125 = ORcp24_119+ORcp24_120+ORcp24_121+ORcp24_122+ORcp24_123+ORcp24_124+ORcp24_125+qd(1);
    VIcp24_225 = ORcp24_219+ORcp24_220+ORcp24_221+ORcp24_222+ORcp24_223+ORcp24_224+ORcp24_225+qd(2);
    VIcp24_325 = ORcp24_319+ORcp24_320+ORcp24_321+ORcp24_322+ORcp24_323+ORcp24_324+ORcp24_325+qd(3);
    OPcp24_125 = OPcp24_124+ROcp24_724*qdd(25)+qd(25)*(OMcp24_224*ROcp24_924-OMcp24_324*ROcp24_824);
    OPcp24_225 = OPcp24_224+ROcp24_824*qdd(25)-qd(25)*(OMcp24_124*ROcp24_924-OMcp24_324*ROcp24_724);
    OPcp24_325 = OPcp24_324+ROcp24_924*qdd(25)+qd(25)*(OMcp24_124*ROcp24_824-OMcp24_224*ROcp24_724);
    ACcp24_125 = qdd(1)+OMcp24_219*ORcp24_320+OMcp24_220*ORcp24_321+OMcp24_221*ORcp24_322+OMcp24_222*ORcp24_323+OMcp24_223*ORcp24_324+OMcp24_224*...
 ORcp24_325+OMcp24_26*ORcp24_319-OMcp24_319*ORcp24_220-OMcp24_320*ORcp24_221-OMcp24_321*ORcp24_222-OMcp24_322*ORcp24_223-OMcp24_323*ORcp24_224-...
 OMcp24_324*ORcp24_225-OMcp24_36*ORcp24_219+OPcp24_219*RLcp24_320+OPcp24_220*RLcp24_321+OPcp24_221*RLcp24_322+OPcp24_222*RLcp24_323+OPcp24_223*...
 RLcp24_324+OPcp24_224*RLcp24_325+OPcp24_26*RLcp24_319-OPcp24_319*RLcp24_220-OPcp24_320*RLcp24_221-OPcp24_321*RLcp24_222-OPcp24_322*RLcp24_223-...
 OPcp24_323*RLcp24_224-OPcp24_324*RLcp24_225-OPcp24_36*RLcp24_219;
    ACcp24_225 = qdd(2)-OMcp24_119*ORcp24_320-OMcp24_120*ORcp24_321-OMcp24_121*ORcp24_322-OMcp24_122*ORcp24_323-OMcp24_123*ORcp24_324-OMcp24_124*...
 ORcp24_325-OMcp24_16*ORcp24_319+OMcp24_319*ORcp24_120+OMcp24_320*ORcp24_121+OMcp24_321*ORcp24_122+OMcp24_322*ORcp24_123+OMcp24_323*ORcp24_124+...
 OMcp24_324*ORcp24_125+OMcp24_36*ORcp24_119-OPcp24_119*RLcp24_320-OPcp24_120*RLcp24_321-OPcp24_121*RLcp24_322-OPcp24_122*RLcp24_323-OPcp24_123*...
 RLcp24_324-OPcp24_124*RLcp24_325-OPcp24_16*RLcp24_319+OPcp24_319*RLcp24_120+OPcp24_320*RLcp24_121+OPcp24_321*RLcp24_122+OPcp24_322*RLcp24_123+...
 OPcp24_323*RLcp24_124+OPcp24_324*RLcp24_125+OPcp24_36*RLcp24_119;
    ACcp24_325 = qdd(3)+OMcp24_119*ORcp24_220+OMcp24_120*ORcp24_221+OMcp24_121*ORcp24_222+OMcp24_122*ORcp24_223+OMcp24_123*ORcp24_224+OMcp24_124*...
 ORcp24_225+OMcp24_16*ORcp24_219-OMcp24_219*ORcp24_120-OMcp24_220*ORcp24_121-OMcp24_221*ORcp24_122-OMcp24_222*ORcp24_123-OMcp24_223*ORcp24_124-...
 OMcp24_224*ORcp24_125-OMcp24_26*ORcp24_119+OPcp24_119*RLcp24_220+OPcp24_120*RLcp24_221+OPcp24_121*RLcp24_222+OPcp24_122*RLcp24_223+OPcp24_123*...
 RLcp24_224+OPcp24_124*RLcp24_225+OPcp24_16*RLcp24_219-OPcp24_219*RLcp24_120-OPcp24_220*RLcp24_121-OPcp24_221*RLcp24_122-OPcp24_222*RLcp24_123-...
 OPcp24_223*RLcp24_124-OPcp24_224*RLcp24_125-OPcp24_26*RLcp24_119;

% = = Block_1_0_0_25_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp24_125;
    sens.P(2) = POcp24_225;
    sens.P(3) = POcp24_325;
    sens.R(1,1) = ROcp24_125;
    sens.R(1,2) = ROcp24_225;
    sens.R(1,3) = ROcp24_325;
    sens.R(2,1) = ROcp24_425;
    sens.R(2,2) = ROcp24_525;
    sens.R(2,3) = ROcp24_625;
    sens.R(3,1) = ROcp24_724;
    sens.R(3,2) = ROcp24_824;
    sens.R(3,3) = ROcp24_924;
    sens.V(1) = VIcp24_125;
    sens.V(2) = VIcp24_225;
    sens.V(3) = VIcp24_325;
    sens.OM(1) = OMcp24_125;
    sens.OM(2) = OMcp24_225;
    sens.OM(3) = OMcp24_325;
    sens.A(1) = ACcp24_125;
    sens.A(2) = ACcp24_225;
    sens.A(3) = ACcp24_325;
    sens.OMP(1) = OPcp24_125;
    sens.OMP(2) = OPcp24_225;
    sens.OMP(3) = OPcp24_325;
 
% 
case 26, 


% = = Block_1_0_0_26_0_1 = = 
 
% Sensor Kinematics 


    ROcp25_25 = S4*S5;
    ROcp25_35 = -C4*S5;
    ROcp25_85 = -S4*C5;
    ROcp25_95 = C4*C5;
    ROcp25_16 = C5*C6;
    ROcp25_26 = ROcp25_25*C6+C4*S6;
    ROcp25_36 = ROcp25_35*C6+S4*S6;
    ROcp25_46 = -C5*S6;
    ROcp25_56 = -(ROcp25_25*S6-C4*C6);
    ROcp25_66 = -(ROcp25_35*S6-S4*C6);
    OMcp25_25 = qd(5)*C4;
    OMcp25_35 = qd(5)*S4;
    OMcp25_16 = qd(4)+qd(6)*S5;
    OMcp25_26 = OMcp25_25+ROcp25_85*qd(6);
    OMcp25_36 = OMcp25_35+ROcp25_95*qd(6);
    OPcp25_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp25_26 = ROcp25_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp25_35*S5-ROcp25_95*qd(4));
    OPcp25_36 = ROcp25_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp25_25*S5-ROcp25_85*qd(4));

% = = Block_1_0_0_26_0_4 = = 
 
% Sensor Kinematics 


    ROcp25_419 = ROcp25_46*C19+S19*S5;
    ROcp25_519 = ROcp25_56*C19+ROcp25_85*S19;
    ROcp25_619 = ROcp25_66*C19+ROcp25_95*S19;
    ROcp25_719 = -(ROcp25_46*S19-C19*S5);
    ROcp25_819 = -(ROcp25_56*S19-ROcp25_85*C19);
    ROcp25_919 = -(ROcp25_66*S19-ROcp25_95*C19);
    ROcp25_120 = ROcp25_16*C20-ROcp25_719*S20;
    ROcp25_220 = ROcp25_26*C20-ROcp25_819*S20;
    ROcp25_320 = ROcp25_36*C20-ROcp25_919*S20;
    ROcp25_720 = ROcp25_16*S20+ROcp25_719*C20;
    ROcp25_820 = ROcp25_26*S20+ROcp25_819*C20;
    ROcp25_920 = ROcp25_36*S20+ROcp25_919*C20;
    ROcp25_121 = ROcp25_120*C21+ROcp25_419*S21;
    ROcp25_221 = ROcp25_220*C21+ROcp25_519*S21;
    ROcp25_321 = ROcp25_320*C21+ROcp25_619*S21;
    ROcp25_421 = -(ROcp25_120*S21-ROcp25_419*C21);
    ROcp25_521 = -(ROcp25_220*S21-ROcp25_519*C21);
    ROcp25_621 = -(ROcp25_320*S21-ROcp25_619*C21);
    RLcp25_119 = s.dpt(2,3)*ROcp25_46+ROcp25_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp25_219 = s.dpt(2,3)*ROcp25_56+ROcp25_26*s.dpt(1,3)+ROcp25_85*s.dpt(3,3);
    RLcp25_319 = s.dpt(2,3)*ROcp25_66+ROcp25_36*s.dpt(1,3)+ROcp25_95*s.dpt(3,3);
    OMcp25_119 = OMcp25_16+ROcp25_16*qd(19);
    OMcp25_219 = OMcp25_26+ROcp25_26*qd(19);
    OMcp25_319 = OMcp25_36+ROcp25_36*qd(19);
    ORcp25_119 = OMcp25_26*RLcp25_319-OMcp25_36*RLcp25_219;
    ORcp25_219 = -(OMcp25_16*RLcp25_319-OMcp25_36*RLcp25_119);
    ORcp25_319 = OMcp25_16*RLcp25_219-OMcp25_26*RLcp25_119;
    OPcp25_119 = OPcp25_16+ROcp25_16*qdd(19)+qd(19)*(OMcp25_26*ROcp25_36-OMcp25_36*ROcp25_26);
    OPcp25_219 = OPcp25_26+ROcp25_26*qdd(19)-qd(19)*(OMcp25_16*ROcp25_36-OMcp25_36*ROcp25_16);
    OPcp25_319 = OPcp25_36+ROcp25_36*qdd(19)+qd(19)*(OMcp25_16*ROcp25_26-OMcp25_26*ROcp25_16);
    RLcp25_120 = s.dpt(1,30)*ROcp25_16+s.dpt(2,30)*ROcp25_419+s.dpt(3,30)*ROcp25_719;
    RLcp25_220 = s.dpt(1,30)*ROcp25_26+s.dpt(2,30)*ROcp25_519+s.dpt(3,30)*ROcp25_819;
    RLcp25_320 = s.dpt(1,30)*ROcp25_36+s.dpt(2,30)*ROcp25_619+s.dpt(3,30)*ROcp25_919;
    OMcp25_120 = OMcp25_119+ROcp25_419*qd(20);
    OMcp25_220 = OMcp25_219+ROcp25_519*qd(20);
    OMcp25_320 = OMcp25_319+ROcp25_619*qd(20);
    ORcp25_120 = OMcp25_219*RLcp25_320-OMcp25_319*RLcp25_220;
    ORcp25_220 = -(OMcp25_119*RLcp25_320-OMcp25_319*RLcp25_120);
    ORcp25_320 = OMcp25_119*RLcp25_220-OMcp25_219*RLcp25_120;
    OPcp25_120 = OPcp25_119+ROcp25_419*qdd(20)+qd(20)*(OMcp25_219*ROcp25_619-OMcp25_319*ROcp25_519);
    OPcp25_220 = OPcp25_219+ROcp25_519*qdd(20)-qd(20)*(OMcp25_119*ROcp25_619-OMcp25_319*ROcp25_419);
    OPcp25_320 = OPcp25_319+ROcp25_619*qdd(20)+qd(20)*(OMcp25_119*ROcp25_519-OMcp25_219*ROcp25_419);
    RLcp25_121 = s.dpt(1,32)*ROcp25_120+s.dpt(2,32)*ROcp25_419+ROcp25_720*s.dpt(3,32);
    RLcp25_221 = s.dpt(1,32)*ROcp25_220+s.dpt(2,32)*ROcp25_519+ROcp25_820*s.dpt(3,32);
    RLcp25_321 = s.dpt(1,32)*ROcp25_320+s.dpt(2,32)*ROcp25_619+ROcp25_920*s.dpt(3,32);
    OMcp25_121 = OMcp25_120+ROcp25_720*qd(21);
    OMcp25_221 = OMcp25_220+ROcp25_820*qd(21);
    OMcp25_321 = OMcp25_320+ROcp25_920*qd(21);
    ORcp25_121 = OMcp25_220*RLcp25_321-OMcp25_320*RLcp25_221;
    ORcp25_221 = -(OMcp25_120*RLcp25_321-OMcp25_320*RLcp25_121);
    ORcp25_321 = OMcp25_120*RLcp25_221-OMcp25_220*RLcp25_121;
    OPcp25_121 = OPcp25_120+ROcp25_720*qdd(21)+qd(21)*(OMcp25_220*ROcp25_920-OMcp25_320*ROcp25_820);
    OPcp25_221 = OPcp25_220+ROcp25_820*qdd(21)-qd(21)*(OMcp25_120*ROcp25_920-OMcp25_320*ROcp25_720);
    OPcp25_321 = OPcp25_320+ROcp25_920*qdd(21)+qd(21)*(OMcp25_120*ROcp25_820-OMcp25_220*ROcp25_720);

% = = Block_1_0_0_26_0_5 = = 
 
% Sensor Kinematics 


    ROcp25_122 = ROcp25_121*C22-ROcp25_720*S22;
    ROcp25_222 = ROcp25_221*C22-ROcp25_820*S22;
    ROcp25_322 = ROcp25_321*C22-ROcp25_920*S22;
    ROcp25_722 = ROcp25_121*S22+ROcp25_720*C22;
    ROcp25_822 = ROcp25_221*S22+ROcp25_820*C22;
    ROcp25_922 = ROcp25_321*S22+ROcp25_920*C22;
    ROcp25_423 = ROcp25_421*C23+ROcp25_722*S23;
    ROcp25_523 = ROcp25_521*C23+ROcp25_822*S23;
    ROcp25_623 = ROcp25_621*C23+ROcp25_922*S23;
    ROcp25_723 = -(ROcp25_421*S23-ROcp25_722*C23);
    ROcp25_823 = -(ROcp25_521*S23-ROcp25_822*C23);
    ROcp25_923 = -(ROcp25_621*S23-ROcp25_922*C23);
    ROcp25_124 = ROcp25_122*C24-ROcp25_723*S24;
    ROcp25_224 = ROcp25_222*C24-ROcp25_823*S24;
    ROcp25_324 = ROcp25_322*C24-ROcp25_923*S24;
    ROcp25_724 = ROcp25_122*S24+ROcp25_723*C24;
    ROcp25_824 = ROcp25_222*S24+ROcp25_823*C24;
    ROcp25_924 = ROcp25_322*S24+ROcp25_923*C24;
    ROcp25_125 = ROcp25_124*C25+ROcp25_423*S25;
    ROcp25_225 = ROcp25_224*C25+ROcp25_523*S25;
    ROcp25_325 = ROcp25_324*C25+ROcp25_623*S25;
    ROcp25_425 = -(ROcp25_124*S25-ROcp25_423*C25);
    ROcp25_525 = -(ROcp25_224*S25-ROcp25_523*C25);
    ROcp25_625 = -(ROcp25_324*S25-ROcp25_623*C25);
    ROcp25_126 = ROcp25_125*C26-ROcp25_724*S26;
    ROcp25_226 = ROcp25_225*C26-ROcp25_824*S26;
    ROcp25_326 = ROcp25_325*C26-ROcp25_924*S26;
    ROcp25_726 = ROcp25_125*S26+ROcp25_724*C26;
    ROcp25_826 = ROcp25_225*S26+ROcp25_824*C26;
    ROcp25_926 = ROcp25_325*S26+ROcp25_924*C26;
    RLcp25_122 = ROcp25_121*s.dpt(1,36)+ROcp25_421*s.dpt(2,36)+ROcp25_720*s.dpt(3,36);
    RLcp25_222 = ROcp25_221*s.dpt(1,36)+ROcp25_521*s.dpt(2,36)+ROcp25_820*s.dpt(3,36);
    RLcp25_322 = ROcp25_321*s.dpt(1,36)+ROcp25_621*s.dpt(2,36)+ROcp25_920*s.dpt(3,36);
    OMcp25_122 = OMcp25_121+ROcp25_421*qd(22);
    OMcp25_222 = OMcp25_221+ROcp25_521*qd(22);
    OMcp25_322 = OMcp25_321+ROcp25_621*qd(22);
    ORcp25_122 = OMcp25_221*RLcp25_322-OMcp25_321*RLcp25_222;
    ORcp25_222 = -(OMcp25_121*RLcp25_322-OMcp25_321*RLcp25_122);
    ORcp25_322 = OMcp25_121*RLcp25_222-OMcp25_221*RLcp25_122;
    OPcp25_122 = OPcp25_121+ROcp25_421*qdd(22)+qd(22)*(OMcp25_221*ROcp25_621-OMcp25_321*ROcp25_521);
    OPcp25_222 = OPcp25_221+ROcp25_521*qdd(22)-qd(22)*(OMcp25_121*ROcp25_621-OMcp25_321*ROcp25_421);
    OPcp25_322 = OPcp25_321+ROcp25_621*qdd(22)+qd(22)*(OMcp25_121*ROcp25_521-OMcp25_221*ROcp25_421);
    RLcp25_123 = s.dpt(1,40)*ROcp25_122+s.dpt(3,40)*ROcp25_722+ROcp25_421*s.dpt(2,40);
    RLcp25_223 = s.dpt(1,40)*ROcp25_222+s.dpt(3,40)*ROcp25_822+ROcp25_521*s.dpt(2,40);
    RLcp25_323 = s.dpt(1,40)*ROcp25_322+s.dpt(3,40)*ROcp25_922+ROcp25_621*s.dpt(2,40);
    OMcp25_123 = OMcp25_122+ROcp25_122*qd(23);
    OMcp25_223 = OMcp25_222+ROcp25_222*qd(23);
    OMcp25_323 = OMcp25_322+ROcp25_322*qd(23);
    ORcp25_123 = OMcp25_222*RLcp25_323-OMcp25_322*RLcp25_223;
    ORcp25_223 = -(OMcp25_122*RLcp25_323-OMcp25_322*RLcp25_123);
    ORcp25_323 = OMcp25_122*RLcp25_223-OMcp25_222*RLcp25_123;
    OPcp25_123 = OPcp25_122+ROcp25_122*qdd(23)+qd(23)*(OMcp25_222*ROcp25_322-OMcp25_322*ROcp25_222);
    OPcp25_223 = OPcp25_222+ROcp25_222*qdd(23)-qd(23)*(OMcp25_122*ROcp25_322-OMcp25_322*ROcp25_122);
    OPcp25_323 = OPcp25_322+ROcp25_322*qdd(23)+qd(23)*(OMcp25_122*ROcp25_222-OMcp25_222*ROcp25_122);
    RLcp25_124 = s.dpt(1,42)*ROcp25_122+s.dpt(3,42)*ROcp25_723+ROcp25_423*s.dpt(2,42);
    RLcp25_224 = s.dpt(1,42)*ROcp25_222+s.dpt(3,42)*ROcp25_823+ROcp25_523*s.dpt(2,42);
    RLcp25_324 = s.dpt(1,42)*ROcp25_322+s.dpt(3,42)*ROcp25_923+ROcp25_623*s.dpt(2,42);
    OMcp25_124 = OMcp25_123+ROcp25_423*qd(24);
    OMcp25_224 = OMcp25_223+ROcp25_523*qd(24);
    OMcp25_324 = OMcp25_323+ROcp25_623*qd(24);
    ORcp25_124 = OMcp25_223*RLcp25_324-OMcp25_323*RLcp25_224;
    ORcp25_224 = -(OMcp25_123*RLcp25_324-OMcp25_323*RLcp25_124);
    ORcp25_324 = OMcp25_123*RLcp25_224-OMcp25_223*RLcp25_124;
    OPcp25_124 = OPcp25_123+ROcp25_423*qdd(24)+qd(24)*(OMcp25_223*ROcp25_623-OMcp25_323*ROcp25_523);
    OPcp25_224 = OPcp25_223+ROcp25_523*qdd(24)-qd(24)*(OMcp25_123*ROcp25_623-OMcp25_323*ROcp25_423);
    OPcp25_324 = OPcp25_323+ROcp25_623*qdd(24)+qd(24)*(OMcp25_123*ROcp25_523-OMcp25_223*ROcp25_423);
    RLcp25_125 = s.dpt(3,44)*ROcp25_724+ROcp25_124*s.dpt(1,44)+ROcp25_423*s.dpt(2,44);
    RLcp25_225 = s.dpt(3,44)*ROcp25_824+ROcp25_224*s.dpt(1,44)+ROcp25_523*s.dpt(2,44);
    RLcp25_325 = s.dpt(3,44)*ROcp25_924+ROcp25_324*s.dpt(1,44)+ROcp25_623*s.dpt(2,44);
    OMcp25_125 = OMcp25_124+ROcp25_724*qd(25);
    OMcp25_225 = OMcp25_224+ROcp25_824*qd(25);
    OMcp25_325 = OMcp25_324+ROcp25_924*qd(25);
    ORcp25_125 = OMcp25_224*RLcp25_325-OMcp25_324*RLcp25_225;
    ORcp25_225 = -(OMcp25_124*RLcp25_325-OMcp25_324*RLcp25_125);
    ORcp25_325 = OMcp25_124*RLcp25_225-OMcp25_224*RLcp25_125;
    OPcp25_125 = OPcp25_124+ROcp25_724*qdd(25)+qd(25)*(OMcp25_224*ROcp25_924-OMcp25_324*ROcp25_824);
    OPcp25_225 = OPcp25_224+ROcp25_824*qdd(25)-qd(25)*(OMcp25_124*ROcp25_924-OMcp25_324*ROcp25_724);
    OPcp25_325 = OPcp25_324+ROcp25_924*qdd(25)+qd(25)*(OMcp25_124*ROcp25_824-OMcp25_224*ROcp25_724);
    RLcp25_126 = s.dpt(2,46)*ROcp25_425+s.dpt(3,46)*ROcp25_724+ROcp25_125*s.dpt(1,46);
    RLcp25_226 = s.dpt(2,46)*ROcp25_525+s.dpt(3,46)*ROcp25_824+ROcp25_225*s.dpt(1,46);
    RLcp25_326 = s.dpt(2,46)*ROcp25_625+s.dpt(3,46)*ROcp25_924+ROcp25_325*s.dpt(1,46);
    POcp25_126 = RLcp25_119+RLcp25_120+RLcp25_121+RLcp25_122+RLcp25_123+RLcp25_124+RLcp25_125+RLcp25_126+q(1);
    POcp25_226 = RLcp25_219+RLcp25_220+RLcp25_221+RLcp25_222+RLcp25_223+RLcp25_224+RLcp25_225+RLcp25_226+q(2);
    POcp25_326 = RLcp25_319+RLcp25_320+RLcp25_321+RLcp25_322+RLcp25_323+RLcp25_324+RLcp25_325+RLcp25_326+q(3);
    OMcp25_126 = OMcp25_125+ROcp25_425*qd(26);
    OMcp25_226 = OMcp25_225+ROcp25_525*qd(26);
    OMcp25_326 = OMcp25_325+ROcp25_625*qd(26);
    ORcp25_126 = OMcp25_225*RLcp25_326-OMcp25_325*RLcp25_226;
    ORcp25_226 = -(OMcp25_125*RLcp25_326-OMcp25_325*RLcp25_126);
    ORcp25_326 = OMcp25_125*RLcp25_226-OMcp25_225*RLcp25_126;
    VIcp25_126 = ORcp25_119+ORcp25_120+ORcp25_121+ORcp25_122+ORcp25_123+ORcp25_124+ORcp25_125+ORcp25_126+qd(1);
    VIcp25_226 = ORcp25_219+ORcp25_220+ORcp25_221+ORcp25_222+ORcp25_223+ORcp25_224+ORcp25_225+ORcp25_226+qd(2);
    VIcp25_326 = ORcp25_319+ORcp25_320+ORcp25_321+ORcp25_322+ORcp25_323+ORcp25_324+ORcp25_325+ORcp25_326+qd(3);
    OPcp25_126 = OPcp25_125+ROcp25_425*qdd(26)+qd(26)*(OMcp25_225*ROcp25_625-OMcp25_325*ROcp25_525);
    OPcp25_226 = OPcp25_225+ROcp25_525*qdd(26)-qd(26)*(OMcp25_125*ROcp25_625-OMcp25_325*ROcp25_425);
    OPcp25_326 = OPcp25_325+ROcp25_625*qdd(26)+qd(26)*(OMcp25_125*ROcp25_525-OMcp25_225*ROcp25_425);
    ACcp25_126 = qdd(1)+OMcp25_219*ORcp25_320+OMcp25_220*ORcp25_321+OMcp25_221*ORcp25_322+OMcp25_222*ORcp25_323+OMcp25_223*ORcp25_324+OMcp25_224*...
 ORcp25_325+OMcp25_225*ORcp25_326+OMcp25_26*ORcp25_319-OMcp25_319*ORcp25_220-OMcp25_320*ORcp25_221-OMcp25_321*ORcp25_222-OMcp25_322*ORcp25_223-...
 OMcp25_323*ORcp25_224-OMcp25_324*ORcp25_225-OMcp25_325*ORcp25_226-OMcp25_36*ORcp25_219+OPcp25_219*RLcp25_320+OPcp25_220*RLcp25_321+OPcp25_221*...
 RLcp25_322+OPcp25_222*RLcp25_323+OPcp25_223*RLcp25_324+OPcp25_224*RLcp25_325+OPcp25_225*RLcp25_326+OPcp25_26*RLcp25_319-OPcp25_319*RLcp25_220-...
 OPcp25_320*RLcp25_221-OPcp25_321*RLcp25_222-OPcp25_322*RLcp25_223-OPcp25_323*RLcp25_224-OPcp25_324*RLcp25_225-OPcp25_325*RLcp25_226-OPcp25_36*...
 RLcp25_219;
    ACcp25_226 = qdd(2)-OMcp25_119*ORcp25_320-OMcp25_120*ORcp25_321-OMcp25_121*ORcp25_322-OMcp25_122*ORcp25_323-OMcp25_123*ORcp25_324-OMcp25_124*...
 ORcp25_325-OMcp25_125*ORcp25_326-OMcp25_16*ORcp25_319+OMcp25_319*ORcp25_120+OMcp25_320*ORcp25_121+OMcp25_321*ORcp25_122+OMcp25_322*ORcp25_123+...
 OMcp25_323*ORcp25_124+OMcp25_324*ORcp25_125+OMcp25_325*ORcp25_126+OMcp25_36*ORcp25_119-OPcp25_119*RLcp25_320-OPcp25_120*RLcp25_321-OPcp25_121*...
 RLcp25_322-OPcp25_122*RLcp25_323-OPcp25_123*RLcp25_324-OPcp25_124*RLcp25_325-OPcp25_125*RLcp25_326-OPcp25_16*RLcp25_319+OPcp25_319*RLcp25_120+...
 OPcp25_320*RLcp25_121+OPcp25_321*RLcp25_122+OPcp25_322*RLcp25_123+OPcp25_323*RLcp25_124+OPcp25_324*RLcp25_125+OPcp25_325*RLcp25_126+OPcp25_36*...
 RLcp25_119;
    ACcp25_326 = qdd(3)+OMcp25_119*ORcp25_220+OMcp25_120*ORcp25_221+OMcp25_121*ORcp25_222+OMcp25_122*ORcp25_223+OMcp25_123*ORcp25_224+OMcp25_124*...
 ORcp25_225+OMcp25_125*ORcp25_226+OMcp25_16*ORcp25_219-OMcp25_219*ORcp25_120-OMcp25_220*ORcp25_121-OMcp25_221*ORcp25_122-OMcp25_222*ORcp25_123-...
 OMcp25_223*ORcp25_124-OMcp25_224*ORcp25_125-OMcp25_225*ORcp25_126-OMcp25_26*ORcp25_119+OPcp25_119*RLcp25_220+OPcp25_120*RLcp25_221+OPcp25_121*...
 RLcp25_222+OPcp25_122*RLcp25_223+OPcp25_123*RLcp25_224+OPcp25_124*RLcp25_225+OPcp25_125*RLcp25_226+OPcp25_16*RLcp25_219-OPcp25_219*RLcp25_120-...
 OPcp25_220*RLcp25_121-OPcp25_221*RLcp25_122-OPcp25_222*RLcp25_123-OPcp25_223*RLcp25_124-OPcp25_224*RLcp25_125-OPcp25_225*RLcp25_126-OPcp25_26*...
 RLcp25_119;

% = = Block_1_0_0_26_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp25_126;
    sens.P(2) = POcp25_226;
    sens.P(3) = POcp25_326;
    sens.R(1,1) = ROcp25_126;
    sens.R(1,2) = ROcp25_226;
    sens.R(1,3) = ROcp25_326;
    sens.R(2,1) = ROcp25_425;
    sens.R(2,2) = ROcp25_525;
    sens.R(2,3) = ROcp25_625;
    sens.R(3,1) = ROcp25_726;
    sens.R(3,2) = ROcp25_826;
    sens.R(3,3) = ROcp25_926;
    sens.V(1) = VIcp25_126;
    sens.V(2) = VIcp25_226;
    sens.V(3) = VIcp25_326;
    sens.OM(1) = OMcp25_126;
    sens.OM(2) = OMcp25_226;
    sens.OM(3) = OMcp25_326;
    sens.A(1) = ACcp25_126;
    sens.A(2) = ACcp25_226;
    sens.A(3) = ACcp25_326;
    sens.OMP(1) = OPcp25_126;
    sens.OMP(2) = OPcp25_226;
    sens.OMP(3) = OPcp25_326;
 
% 
case 27, 


% = = Block_1_0_0_27_0_1 = = 
 
% Sensor Kinematics 


    ROcp26_25 = S4*S5;
    ROcp26_35 = -C4*S5;
    ROcp26_85 = -S4*C5;
    ROcp26_95 = C4*C5;
    ROcp26_16 = C5*C6;
    ROcp26_26 = ROcp26_25*C6+C4*S6;
    ROcp26_36 = ROcp26_35*C6+S4*S6;
    ROcp26_46 = -C5*S6;
    ROcp26_56 = -(ROcp26_25*S6-C4*C6);
    ROcp26_66 = -(ROcp26_35*S6-S4*C6);
    OMcp26_25 = qd(5)*C4;
    OMcp26_35 = qd(5)*S4;
    OMcp26_16 = qd(4)+qd(6)*S5;
    OMcp26_26 = OMcp26_25+ROcp26_85*qd(6);
    OMcp26_36 = OMcp26_35+ROcp26_95*qd(6);
    OPcp26_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp26_26 = ROcp26_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp26_35*S5-ROcp26_95*qd(4));
    OPcp26_36 = ROcp26_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp26_25*S5-ROcp26_85*qd(4));

% = = Block_1_0_0_27_0_4 = = 
 
% Sensor Kinematics 


    ROcp26_419 = ROcp26_46*C19+S19*S5;
    ROcp26_519 = ROcp26_56*C19+ROcp26_85*S19;
    ROcp26_619 = ROcp26_66*C19+ROcp26_95*S19;
    ROcp26_719 = -(ROcp26_46*S19-C19*S5);
    ROcp26_819 = -(ROcp26_56*S19-ROcp26_85*C19);
    ROcp26_919 = -(ROcp26_66*S19-ROcp26_95*C19);
    ROcp26_120 = ROcp26_16*C20-ROcp26_719*S20;
    ROcp26_220 = ROcp26_26*C20-ROcp26_819*S20;
    ROcp26_320 = ROcp26_36*C20-ROcp26_919*S20;
    ROcp26_720 = ROcp26_16*S20+ROcp26_719*C20;
    ROcp26_820 = ROcp26_26*S20+ROcp26_819*C20;
    ROcp26_920 = ROcp26_36*S20+ROcp26_919*C20;
    ROcp26_121 = ROcp26_120*C21+ROcp26_419*S21;
    ROcp26_221 = ROcp26_220*C21+ROcp26_519*S21;
    ROcp26_321 = ROcp26_320*C21+ROcp26_619*S21;
    ROcp26_421 = -(ROcp26_120*S21-ROcp26_419*C21);
    ROcp26_521 = -(ROcp26_220*S21-ROcp26_519*C21);
    ROcp26_621 = -(ROcp26_320*S21-ROcp26_619*C21);
    RLcp26_119 = s.dpt(2,3)*ROcp26_46+ROcp26_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp26_219 = s.dpt(2,3)*ROcp26_56+ROcp26_26*s.dpt(1,3)+ROcp26_85*s.dpt(3,3);
    RLcp26_319 = s.dpt(2,3)*ROcp26_66+ROcp26_36*s.dpt(1,3)+ROcp26_95*s.dpt(3,3);
    OMcp26_119 = OMcp26_16+ROcp26_16*qd(19);
    OMcp26_219 = OMcp26_26+ROcp26_26*qd(19);
    OMcp26_319 = OMcp26_36+ROcp26_36*qd(19);
    ORcp26_119 = OMcp26_26*RLcp26_319-OMcp26_36*RLcp26_219;
    ORcp26_219 = -(OMcp26_16*RLcp26_319-OMcp26_36*RLcp26_119);
    ORcp26_319 = OMcp26_16*RLcp26_219-OMcp26_26*RLcp26_119;
    OPcp26_119 = OPcp26_16+ROcp26_16*qdd(19)+qd(19)*(OMcp26_26*ROcp26_36-OMcp26_36*ROcp26_26);
    OPcp26_219 = OPcp26_26+ROcp26_26*qdd(19)-qd(19)*(OMcp26_16*ROcp26_36-OMcp26_36*ROcp26_16);
    OPcp26_319 = OPcp26_36+ROcp26_36*qdd(19)+qd(19)*(OMcp26_16*ROcp26_26-OMcp26_26*ROcp26_16);
    RLcp26_120 = s.dpt(1,30)*ROcp26_16+s.dpt(2,30)*ROcp26_419+s.dpt(3,30)*ROcp26_719;
    RLcp26_220 = s.dpt(1,30)*ROcp26_26+s.dpt(2,30)*ROcp26_519+s.dpt(3,30)*ROcp26_819;
    RLcp26_320 = s.dpt(1,30)*ROcp26_36+s.dpt(2,30)*ROcp26_619+s.dpt(3,30)*ROcp26_919;
    OMcp26_120 = OMcp26_119+ROcp26_419*qd(20);
    OMcp26_220 = OMcp26_219+ROcp26_519*qd(20);
    OMcp26_320 = OMcp26_319+ROcp26_619*qd(20);
    ORcp26_120 = OMcp26_219*RLcp26_320-OMcp26_319*RLcp26_220;
    ORcp26_220 = -(OMcp26_119*RLcp26_320-OMcp26_319*RLcp26_120);
    ORcp26_320 = OMcp26_119*RLcp26_220-OMcp26_219*RLcp26_120;
    OPcp26_120 = OPcp26_119+ROcp26_419*qdd(20)+qd(20)*(OMcp26_219*ROcp26_619-OMcp26_319*ROcp26_519);
    OPcp26_220 = OPcp26_219+ROcp26_519*qdd(20)-qd(20)*(OMcp26_119*ROcp26_619-OMcp26_319*ROcp26_419);
    OPcp26_320 = OPcp26_319+ROcp26_619*qdd(20)+qd(20)*(OMcp26_119*ROcp26_519-OMcp26_219*ROcp26_419);
    RLcp26_121 = s.dpt(1,32)*ROcp26_120+s.dpt(2,32)*ROcp26_419+ROcp26_720*s.dpt(3,32);
    RLcp26_221 = s.dpt(1,32)*ROcp26_220+s.dpt(2,32)*ROcp26_519+ROcp26_820*s.dpt(3,32);
    RLcp26_321 = s.dpt(1,32)*ROcp26_320+s.dpt(2,32)*ROcp26_619+ROcp26_920*s.dpt(3,32);
    OMcp26_121 = OMcp26_120+ROcp26_720*qd(21);
    OMcp26_221 = OMcp26_220+ROcp26_820*qd(21);
    OMcp26_321 = OMcp26_320+ROcp26_920*qd(21);
    ORcp26_121 = OMcp26_220*RLcp26_321-OMcp26_320*RLcp26_221;
    ORcp26_221 = -(OMcp26_120*RLcp26_321-OMcp26_320*RLcp26_121);
    ORcp26_321 = OMcp26_120*RLcp26_221-OMcp26_220*RLcp26_121;
    OPcp26_121 = OPcp26_120+ROcp26_720*qdd(21)+qd(21)*(OMcp26_220*ROcp26_920-OMcp26_320*ROcp26_820);
    OPcp26_221 = OPcp26_220+ROcp26_820*qdd(21)-qd(21)*(OMcp26_120*ROcp26_920-OMcp26_320*ROcp26_720);
    OPcp26_321 = OPcp26_320+ROcp26_920*qdd(21)+qd(21)*(OMcp26_120*ROcp26_820-OMcp26_220*ROcp26_720);

% = = Block_1_0_0_27_0_5 = = 
 
% Sensor Kinematics 


    ROcp26_122 = ROcp26_121*C22-ROcp26_720*S22;
    ROcp26_222 = ROcp26_221*C22-ROcp26_820*S22;
    ROcp26_322 = ROcp26_321*C22-ROcp26_920*S22;
    ROcp26_722 = ROcp26_121*S22+ROcp26_720*C22;
    ROcp26_822 = ROcp26_221*S22+ROcp26_820*C22;
    ROcp26_922 = ROcp26_321*S22+ROcp26_920*C22;
    ROcp26_423 = ROcp26_421*C23+ROcp26_722*S23;
    ROcp26_523 = ROcp26_521*C23+ROcp26_822*S23;
    ROcp26_623 = ROcp26_621*C23+ROcp26_922*S23;
    ROcp26_723 = -(ROcp26_421*S23-ROcp26_722*C23);
    ROcp26_823 = -(ROcp26_521*S23-ROcp26_822*C23);
    ROcp26_923 = -(ROcp26_621*S23-ROcp26_922*C23);
    ROcp26_124 = ROcp26_122*C24-ROcp26_723*S24;
    ROcp26_224 = ROcp26_222*C24-ROcp26_823*S24;
    ROcp26_324 = ROcp26_322*C24-ROcp26_923*S24;
    ROcp26_724 = ROcp26_122*S24+ROcp26_723*C24;
    ROcp26_824 = ROcp26_222*S24+ROcp26_823*C24;
    ROcp26_924 = ROcp26_322*S24+ROcp26_923*C24;
    ROcp26_125 = ROcp26_124*C25+ROcp26_423*S25;
    ROcp26_225 = ROcp26_224*C25+ROcp26_523*S25;
    ROcp26_325 = ROcp26_324*C25+ROcp26_623*S25;
    ROcp26_425 = -(ROcp26_124*S25-ROcp26_423*C25);
    ROcp26_525 = -(ROcp26_224*S25-ROcp26_523*C25);
    ROcp26_625 = -(ROcp26_324*S25-ROcp26_623*C25);
    ROcp26_126 = ROcp26_125*C26-ROcp26_724*S26;
    ROcp26_226 = ROcp26_225*C26-ROcp26_824*S26;
    ROcp26_326 = ROcp26_325*C26-ROcp26_924*S26;
    ROcp26_726 = ROcp26_125*S26+ROcp26_724*C26;
    ROcp26_826 = ROcp26_225*S26+ROcp26_824*C26;
    ROcp26_926 = ROcp26_325*S26+ROcp26_924*C26;
    ROcp26_127 = ROcp26_126*C27+ROcp26_425*S27;
    ROcp26_227 = ROcp26_226*C27+ROcp26_525*S27;
    ROcp26_327 = ROcp26_326*C27+ROcp26_625*S27;
    ROcp26_427 = -(ROcp26_126*S27-ROcp26_425*C27);
    ROcp26_527 = -(ROcp26_226*S27-ROcp26_525*C27);
    ROcp26_627 = -(ROcp26_326*S27-ROcp26_625*C27);
    RLcp26_122 = ROcp26_121*s.dpt(1,36)+ROcp26_421*s.dpt(2,36)+ROcp26_720*s.dpt(3,36);
    RLcp26_222 = ROcp26_221*s.dpt(1,36)+ROcp26_521*s.dpt(2,36)+ROcp26_820*s.dpt(3,36);
    RLcp26_322 = ROcp26_321*s.dpt(1,36)+ROcp26_621*s.dpt(2,36)+ROcp26_920*s.dpt(3,36);
    OMcp26_122 = OMcp26_121+ROcp26_421*qd(22);
    OMcp26_222 = OMcp26_221+ROcp26_521*qd(22);
    OMcp26_322 = OMcp26_321+ROcp26_621*qd(22);
    ORcp26_122 = OMcp26_221*RLcp26_322-OMcp26_321*RLcp26_222;
    ORcp26_222 = -(OMcp26_121*RLcp26_322-OMcp26_321*RLcp26_122);
    ORcp26_322 = OMcp26_121*RLcp26_222-OMcp26_221*RLcp26_122;
    OPcp26_122 = OPcp26_121+ROcp26_421*qdd(22)+qd(22)*(OMcp26_221*ROcp26_621-OMcp26_321*ROcp26_521);
    OPcp26_222 = OPcp26_221+ROcp26_521*qdd(22)-qd(22)*(OMcp26_121*ROcp26_621-OMcp26_321*ROcp26_421);
    OPcp26_322 = OPcp26_321+ROcp26_621*qdd(22)+qd(22)*(OMcp26_121*ROcp26_521-OMcp26_221*ROcp26_421);
    RLcp26_123 = s.dpt(1,40)*ROcp26_122+s.dpt(3,40)*ROcp26_722+ROcp26_421*s.dpt(2,40);
    RLcp26_223 = s.dpt(1,40)*ROcp26_222+s.dpt(3,40)*ROcp26_822+ROcp26_521*s.dpt(2,40);
    RLcp26_323 = s.dpt(1,40)*ROcp26_322+s.dpt(3,40)*ROcp26_922+ROcp26_621*s.dpt(2,40);
    OMcp26_123 = OMcp26_122+ROcp26_122*qd(23);
    OMcp26_223 = OMcp26_222+ROcp26_222*qd(23);
    OMcp26_323 = OMcp26_322+ROcp26_322*qd(23);
    ORcp26_123 = OMcp26_222*RLcp26_323-OMcp26_322*RLcp26_223;
    ORcp26_223 = -(OMcp26_122*RLcp26_323-OMcp26_322*RLcp26_123);
    ORcp26_323 = OMcp26_122*RLcp26_223-OMcp26_222*RLcp26_123;
    OPcp26_123 = OPcp26_122+ROcp26_122*qdd(23)+qd(23)*(OMcp26_222*ROcp26_322-OMcp26_322*ROcp26_222);
    OPcp26_223 = OPcp26_222+ROcp26_222*qdd(23)-qd(23)*(OMcp26_122*ROcp26_322-OMcp26_322*ROcp26_122);
    OPcp26_323 = OPcp26_322+ROcp26_322*qdd(23)+qd(23)*(OMcp26_122*ROcp26_222-OMcp26_222*ROcp26_122);
    RLcp26_124 = s.dpt(1,42)*ROcp26_122+s.dpt(3,42)*ROcp26_723+ROcp26_423*s.dpt(2,42);
    RLcp26_224 = s.dpt(1,42)*ROcp26_222+s.dpt(3,42)*ROcp26_823+ROcp26_523*s.dpt(2,42);
    RLcp26_324 = s.dpt(1,42)*ROcp26_322+s.dpt(3,42)*ROcp26_923+ROcp26_623*s.dpt(2,42);
    OMcp26_124 = OMcp26_123+ROcp26_423*qd(24);
    OMcp26_224 = OMcp26_223+ROcp26_523*qd(24);
    OMcp26_324 = OMcp26_323+ROcp26_623*qd(24);
    ORcp26_124 = OMcp26_223*RLcp26_324-OMcp26_323*RLcp26_224;
    ORcp26_224 = -(OMcp26_123*RLcp26_324-OMcp26_323*RLcp26_124);
    ORcp26_324 = OMcp26_123*RLcp26_224-OMcp26_223*RLcp26_124;
    OPcp26_124 = OPcp26_123+ROcp26_423*qdd(24)+qd(24)*(OMcp26_223*ROcp26_623-OMcp26_323*ROcp26_523);
    OPcp26_224 = OPcp26_223+ROcp26_523*qdd(24)-qd(24)*(OMcp26_123*ROcp26_623-OMcp26_323*ROcp26_423);
    OPcp26_324 = OPcp26_323+ROcp26_623*qdd(24)+qd(24)*(OMcp26_123*ROcp26_523-OMcp26_223*ROcp26_423);
    RLcp26_125 = s.dpt(3,44)*ROcp26_724+ROcp26_124*s.dpt(1,44)+ROcp26_423*s.dpt(2,44);
    RLcp26_225 = s.dpt(3,44)*ROcp26_824+ROcp26_224*s.dpt(1,44)+ROcp26_523*s.dpt(2,44);
    RLcp26_325 = s.dpt(3,44)*ROcp26_924+ROcp26_324*s.dpt(1,44)+ROcp26_623*s.dpt(2,44);
    OMcp26_125 = OMcp26_124+ROcp26_724*qd(25);
    OMcp26_225 = OMcp26_224+ROcp26_824*qd(25);
    OMcp26_325 = OMcp26_324+ROcp26_924*qd(25);
    ORcp26_125 = OMcp26_224*RLcp26_325-OMcp26_324*RLcp26_225;
    ORcp26_225 = -(OMcp26_124*RLcp26_325-OMcp26_324*RLcp26_125);
    ORcp26_325 = OMcp26_124*RLcp26_225-OMcp26_224*RLcp26_125;
    OPcp26_125 = OPcp26_124+ROcp26_724*qdd(25)+qd(25)*(OMcp26_224*ROcp26_924-OMcp26_324*ROcp26_824);
    OPcp26_225 = OPcp26_224+ROcp26_824*qdd(25)-qd(25)*(OMcp26_124*ROcp26_924-OMcp26_324*ROcp26_724);
    OPcp26_325 = OPcp26_324+ROcp26_924*qdd(25)+qd(25)*(OMcp26_124*ROcp26_824-OMcp26_224*ROcp26_724);
    RLcp26_126 = s.dpt(2,46)*ROcp26_425+s.dpt(3,46)*ROcp26_724+ROcp26_125*s.dpt(1,46);
    RLcp26_226 = s.dpt(2,46)*ROcp26_525+s.dpt(3,46)*ROcp26_824+ROcp26_225*s.dpt(1,46);
    RLcp26_326 = s.dpt(2,46)*ROcp26_625+s.dpt(3,46)*ROcp26_924+ROcp26_325*s.dpt(1,46);
    OMcp26_126 = OMcp26_125+ROcp26_425*qd(26);
    OMcp26_226 = OMcp26_225+ROcp26_525*qd(26);
    OMcp26_326 = OMcp26_325+ROcp26_625*qd(26);
    ORcp26_126 = OMcp26_225*RLcp26_326-OMcp26_325*RLcp26_226;
    ORcp26_226 = -(OMcp26_125*RLcp26_326-OMcp26_325*RLcp26_126);
    ORcp26_326 = OMcp26_125*RLcp26_226-OMcp26_225*RLcp26_126;
    OPcp26_126 = OPcp26_125+ROcp26_425*qdd(26)+qd(26)*(OMcp26_225*ROcp26_625-OMcp26_325*ROcp26_525);
    OPcp26_226 = OPcp26_225+ROcp26_525*qdd(26)-qd(26)*(OMcp26_125*ROcp26_625-OMcp26_325*ROcp26_425);
    OPcp26_326 = OPcp26_325+ROcp26_625*qdd(26)+qd(26)*(OMcp26_125*ROcp26_525-OMcp26_225*ROcp26_425);
    RLcp26_127 = s.dpt(1,47)*ROcp26_126+s.dpt(3,47)*ROcp26_726+ROcp26_425*s.dpt(2,47);
    RLcp26_227 = s.dpt(1,47)*ROcp26_226+s.dpt(3,47)*ROcp26_826+ROcp26_525*s.dpt(2,47);
    RLcp26_327 = s.dpt(1,47)*ROcp26_326+s.dpt(3,47)*ROcp26_926+ROcp26_625*s.dpt(2,47);
    POcp26_127 = RLcp26_119+RLcp26_120+RLcp26_121+RLcp26_122+RLcp26_123+RLcp26_124+RLcp26_125+RLcp26_126+RLcp26_127+q(1);
    POcp26_227 = RLcp26_219+RLcp26_220+RLcp26_221+RLcp26_222+RLcp26_223+RLcp26_224+RLcp26_225+RLcp26_226+RLcp26_227+q(2);
    POcp26_327 = RLcp26_319+RLcp26_320+RLcp26_321+RLcp26_322+RLcp26_323+RLcp26_324+RLcp26_325+RLcp26_326+RLcp26_327+q(3);
    OMcp26_127 = OMcp26_126+ROcp26_726*qd(27);
    OMcp26_227 = OMcp26_226+ROcp26_826*qd(27);
    OMcp26_327 = OMcp26_326+ROcp26_926*qd(27);
    ORcp26_127 = OMcp26_226*RLcp26_327-OMcp26_326*RLcp26_227;
    ORcp26_227 = -(OMcp26_126*RLcp26_327-OMcp26_326*RLcp26_127);
    ORcp26_327 = OMcp26_126*RLcp26_227-OMcp26_226*RLcp26_127;
    VIcp26_127 = ORcp26_119+ORcp26_120+ORcp26_121+ORcp26_122+ORcp26_123+ORcp26_124+ORcp26_125+ORcp26_126+ORcp26_127+qd(1);
    VIcp26_227 = ORcp26_219+ORcp26_220+ORcp26_221+ORcp26_222+ORcp26_223+ORcp26_224+ORcp26_225+ORcp26_226+ORcp26_227+qd(2);
    VIcp26_327 = ORcp26_319+ORcp26_320+ORcp26_321+ORcp26_322+ORcp26_323+ORcp26_324+ORcp26_325+ORcp26_326+ORcp26_327+qd(3);
    OPcp26_127 = OPcp26_126+ROcp26_726*qdd(27)+qd(27)*(OMcp26_226*ROcp26_926-OMcp26_326*ROcp26_826);
    OPcp26_227 = OPcp26_226+ROcp26_826*qdd(27)-qd(27)*(OMcp26_126*ROcp26_926-OMcp26_326*ROcp26_726);
    OPcp26_327 = OPcp26_326+ROcp26_926*qdd(27)+qd(27)*(OMcp26_126*ROcp26_826-OMcp26_226*ROcp26_726);
    ACcp26_127 = qdd(1)+OMcp26_219*ORcp26_320+OMcp26_220*ORcp26_321+OMcp26_221*ORcp26_322+OMcp26_222*ORcp26_323+OMcp26_223*ORcp26_324+OMcp26_224*...
 ORcp26_325+OMcp26_225*ORcp26_326+OMcp26_226*ORcp26_327+OMcp26_26*ORcp26_319-OMcp26_319*ORcp26_220-OMcp26_320*ORcp26_221-OMcp26_321*ORcp26_222-...
 OMcp26_322*ORcp26_223-OMcp26_323*ORcp26_224-OMcp26_324*ORcp26_225-OMcp26_325*ORcp26_226-OMcp26_326*ORcp26_227-OMcp26_36*ORcp26_219+OPcp26_219*...
 RLcp26_320+OPcp26_220*RLcp26_321+OPcp26_221*RLcp26_322+OPcp26_222*RLcp26_323+OPcp26_223*RLcp26_324+OPcp26_224*RLcp26_325+OPcp26_225*RLcp26_326+...
 OPcp26_226*RLcp26_327+OPcp26_26*RLcp26_319-OPcp26_319*RLcp26_220-OPcp26_320*RLcp26_221-OPcp26_321*RLcp26_222-OPcp26_322*RLcp26_223-OPcp26_323*...
 RLcp26_224-OPcp26_324*RLcp26_225-OPcp26_325*RLcp26_226-OPcp26_326*RLcp26_227-OPcp26_36*RLcp26_219;
    ACcp26_227 = qdd(2)-OMcp26_119*ORcp26_320-OMcp26_120*ORcp26_321-OMcp26_121*ORcp26_322-OMcp26_122*ORcp26_323-OMcp26_123*ORcp26_324-OMcp26_124*...
 ORcp26_325-OMcp26_125*ORcp26_326-OMcp26_126*ORcp26_327-OMcp26_16*ORcp26_319+OMcp26_319*ORcp26_120+OMcp26_320*ORcp26_121+OMcp26_321*ORcp26_122+...
 OMcp26_322*ORcp26_123+OMcp26_323*ORcp26_124+OMcp26_324*ORcp26_125+OMcp26_325*ORcp26_126+OMcp26_326*ORcp26_127+OMcp26_36*ORcp26_119-OPcp26_119*...
 RLcp26_320-OPcp26_120*RLcp26_321-OPcp26_121*RLcp26_322-OPcp26_122*RLcp26_323-OPcp26_123*RLcp26_324-OPcp26_124*RLcp26_325-OPcp26_125*RLcp26_326-...
 OPcp26_126*RLcp26_327-OPcp26_16*RLcp26_319+OPcp26_319*RLcp26_120+OPcp26_320*RLcp26_121+OPcp26_321*RLcp26_122+OPcp26_322*RLcp26_123+OPcp26_323*...
 RLcp26_124+OPcp26_324*RLcp26_125+OPcp26_325*RLcp26_126+OPcp26_326*RLcp26_127+OPcp26_36*RLcp26_119;
    ACcp26_327 = qdd(3)+OMcp26_119*ORcp26_220+OMcp26_120*ORcp26_221+OMcp26_121*ORcp26_222+OMcp26_122*ORcp26_223+OMcp26_123*ORcp26_224+OMcp26_124*...
 ORcp26_225+OMcp26_125*ORcp26_226+OMcp26_126*ORcp26_227+OMcp26_16*ORcp26_219-OMcp26_219*ORcp26_120-OMcp26_220*ORcp26_121-OMcp26_221*ORcp26_122-...
 OMcp26_222*ORcp26_123-OMcp26_223*ORcp26_124-OMcp26_224*ORcp26_125-OMcp26_225*ORcp26_126-OMcp26_226*ORcp26_127-OMcp26_26*ORcp26_119+OPcp26_119*...
 RLcp26_220+OPcp26_120*RLcp26_221+OPcp26_121*RLcp26_222+OPcp26_122*RLcp26_223+OPcp26_123*RLcp26_224+OPcp26_124*RLcp26_225+OPcp26_125*RLcp26_226+...
 OPcp26_126*RLcp26_227+OPcp26_16*RLcp26_219-OPcp26_219*RLcp26_120-OPcp26_220*RLcp26_121-OPcp26_221*RLcp26_122-OPcp26_222*RLcp26_123-OPcp26_223*...
 RLcp26_124-OPcp26_224*RLcp26_125-OPcp26_225*RLcp26_126-OPcp26_226*RLcp26_127-OPcp26_26*RLcp26_119;

% = = Block_1_0_0_27_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp26_127;
    sens.P(2) = POcp26_227;
    sens.P(3) = POcp26_327;
    sens.R(1,1) = ROcp26_127;
    sens.R(1,2) = ROcp26_227;
    sens.R(1,3) = ROcp26_327;
    sens.R(2,1) = ROcp26_427;
    sens.R(2,2) = ROcp26_527;
    sens.R(2,3) = ROcp26_627;
    sens.R(3,1) = ROcp26_726;
    sens.R(3,2) = ROcp26_826;
    sens.R(3,3) = ROcp26_926;
    sens.V(1) = VIcp26_127;
    sens.V(2) = VIcp26_227;
    sens.V(3) = VIcp26_327;
    sens.OM(1) = OMcp26_127;
    sens.OM(2) = OMcp26_227;
    sens.OM(3) = OMcp26_327;
    sens.A(1) = ACcp26_127;
    sens.A(2) = ACcp26_227;
    sens.A(3) = ACcp26_327;
    sens.OMP(1) = OPcp26_127;
    sens.OMP(2) = OPcp26_227;
    sens.OMP(3) = OPcp26_327;
 
% 
case 28, 


% = = Block_1_0_0_28_0_1 = = 
 
% Sensor Kinematics 


    ROcp27_25 = S4*S5;
    ROcp27_35 = -C4*S5;
    ROcp27_85 = -S4*C5;
    ROcp27_95 = C4*C5;
    ROcp27_16 = C5*C6;
    ROcp27_26 = ROcp27_25*C6+C4*S6;
    ROcp27_36 = ROcp27_35*C6+S4*S6;
    ROcp27_46 = -C5*S6;
    ROcp27_56 = -(ROcp27_25*S6-C4*C6);
    ROcp27_66 = -(ROcp27_35*S6-S4*C6);
    OMcp27_25 = qd(5)*C4;
    OMcp27_35 = qd(5)*S4;
    OMcp27_16 = qd(4)+qd(6)*S5;
    OMcp27_26 = OMcp27_25+ROcp27_85*qd(6);
    OMcp27_36 = OMcp27_35+ROcp27_95*qd(6);
    OPcp27_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp27_26 = ROcp27_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp27_35*S5-ROcp27_95*qd(4));
    OPcp27_36 = ROcp27_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp27_25*S5-ROcp27_85*qd(4));

% = = Block_1_0_0_28_0_4 = = 
 
% Sensor Kinematics 


    ROcp27_419 = ROcp27_46*C19+S19*S5;
    ROcp27_519 = ROcp27_56*C19+ROcp27_85*S19;
    ROcp27_619 = ROcp27_66*C19+ROcp27_95*S19;
    ROcp27_719 = -(ROcp27_46*S19-C19*S5);
    ROcp27_819 = -(ROcp27_56*S19-ROcp27_85*C19);
    ROcp27_919 = -(ROcp27_66*S19-ROcp27_95*C19);
    ROcp27_120 = ROcp27_16*C20-ROcp27_719*S20;
    ROcp27_220 = ROcp27_26*C20-ROcp27_819*S20;
    ROcp27_320 = ROcp27_36*C20-ROcp27_919*S20;
    ROcp27_720 = ROcp27_16*S20+ROcp27_719*C20;
    ROcp27_820 = ROcp27_26*S20+ROcp27_819*C20;
    ROcp27_920 = ROcp27_36*S20+ROcp27_919*C20;
    ROcp27_121 = ROcp27_120*C21+ROcp27_419*S21;
    ROcp27_221 = ROcp27_220*C21+ROcp27_519*S21;
    ROcp27_321 = ROcp27_320*C21+ROcp27_619*S21;
    ROcp27_421 = -(ROcp27_120*S21-ROcp27_419*C21);
    ROcp27_521 = -(ROcp27_220*S21-ROcp27_519*C21);
    ROcp27_621 = -(ROcp27_320*S21-ROcp27_619*C21);
    RLcp27_119 = s.dpt(2,3)*ROcp27_46+ROcp27_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp27_219 = s.dpt(2,3)*ROcp27_56+ROcp27_26*s.dpt(1,3)+ROcp27_85*s.dpt(3,3);
    RLcp27_319 = s.dpt(2,3)*ROcp27_66+ROcp27_36*s.dpt(1,3)+ROcp27_95*s.dpt(3,3);
    OMcp27_119 = OMcp27_16+ROcp27_16*qd(19);
    OMcp27_219 = OMcp27_26+ROcp27_26*qd(19);
    OMcp27_319 = OMcp27_36+ROcp27_36*qd(19);
    ORcp27_119 = OMcp27_26*RLcp27_319-OMcp27_36*RLcp27_219;
    ORcp27_219 = -(OMcp27_16*RLcp27_319-OMcp27_36*RLcp27_119);
    ORcp27_319 = OMcp27_16*RLcp27_219-OMcp27_26*RLcp27_119;
    OPcp27_119 = OPcp27_16+ROcp27_16*qdd(19)+qd(19)*(OMcp27_26*ROcp27_36-OMcp27_36*ROcp27_26);
    OPcp27_219 = OPcp27_26+ROcp27_26*qdd(19)-qd(19)*(OMcp27_16*ROcp27_36-OMcp27_36*ROcp27_16);
    OPcp27_319 = OPcp27_36+ROcp27_36*qdd(19)+qd(19)*(OMcp27_16*ROcp27_26-OMcp27_26*ROcp27_16);
    RLcp27_120 = s.dpt(1,30)*ROcp27_16+s.dpt(2,30)*ROcp27_419+s.dpt(3,30)*ROcp27_719;
    RLcp27_220 = s.dpt(1,30)*ROcp27_26+s.dpt(2,30)*ROcp27_519+s.dpt(3,30)*ROcp27_819;
    RLcp27_320 = s.dpt(1,30)*ROcp27_36+s.dpt(2,30)*ROcp27_619+s.dpt(3,30)*ROcp27_919;
    OMcp27_120 = OMcp27_119+ROcp27_419*qd(20);
    OMcp27_220 = OMcp27_219+ROcp27_519*qd(20);
    OMcp27_320 = OMcp27_319+ROcp27_619*qd(20);
    ORcp27_120 = OMcp27_219*RLcp27_320-OMcp27_319*RLcp27_220;
    ORcp27_220 = -(OMcp27_119*RLcp27_320-OMcp27_319*RLcp27_120);
    ORcp27_320 = OMcp27_119*RLcp27_220-OMcp27_219*RLcp27_120;
    OPcp27_120 = OPcp27_119+ROcp27_419*qdd(20)+qd(20)*(OMcp27_219*ROcp27_619-OMcp27_319*ROcp27_519);
    OPcp27_220 = OPcp27_219+ROcp27_519*qdd(20)-qd(20)*(OMcp27_119*ROcp27_619-OMcp27_319*ROcp27_419);
    OPcp27_320 = OPcp27_319+ROcp27_619*qdd(20)+qd(20)*(OMcp27_119*ROcp27_519-OMcp27_219*ROcp27_419);
    RLcp27_121 = s.dpt(1,32)*ROcp27_120+s.dpt(2,32)*ROcp27_419+ROcp27_720*s.dpt(3,32);
    RLcp27_221 = s.dpt(1,32)*ROcp27_220+s.dpt(2,32)*ROcp27_519+ROcp27_820*s.dpt(3,32);
    RLcp27_321 = s.dpt(1,32)*ROcp27_320+s.dpt(2,32)*ROcp27_619+ROcp27_920*s.dpt(3,32);
    OMcp27_121 = OMcp27_120+ROcp27_720*qd(21);
    OMcp27_221 = OMcp27_220+ROcp27_820*qd(21);
    OMcp27_321 = OMcp27_320+ROcp27_920*qd(21);
    ORcp27_121 = OMcp27_220*RLcp27_321-OMcp27_320*RLcp27_221;
    ORcp27_221 = -(OMcp27_120*RLcp27_321-OMcp27_320*RLcp27_121);
    ORcp27_321 = OMcp27_120*RLcp27_221-OMcp27_220*RLcp27_121;
    OPcp27_121 = OPcp27_120+ROcp27_720*qdd(21)+qd(21)*(OMcp27_220*ROcp27_920-OMcp27_320*ROcp27_820);
    OPcp27_221 = OPcp27_220+ROcp27_820*qdd(21)-qd(21)*(OMcp27_120*ROcp27_920-OMcp27_320*ROcp27_720);
    OPcp27_321 = OPcp27_320+ROcp27_920*qdd(21)+qd(21)*(OMcp27_120*ROcp27_820-OMcp27_220*ROcp27_720);

% = = Block_1_0_0_28_0_5 = = 
 
% Sensor Kinematics 


    ROcp27_122 = ROcp27_121*C22-ROcp27_720*S22;
    ROcp27_222 = ROcp27_221*C22-ROcp27_820*S22;
    ROcp27_322 = ROcp27_321*C22-ROcp27_920*S22;
    ROcp27_722 = ROcp27_121*S22+ROcp27_720*C22;
    ROcp27_822 = ROcp27_221*S22+ROcp27_820*C22;
    ROcp27_922 = ROcp27_321*S22+ROcp27_920*C22;
    ROcp27_423 = ROcp27_421*C23+ROcp27_722*S23;
    ROcp27_523 = ROcp27_521*C23+ROcp27_822*S23;
    ROcp27_623 = ROcp27_621*C23+ROcp27_922*S23;
    ROcp27_723 = -(ROcp27_421*S23-ROcp27_722*C23);
    ROcp27_823 = -(ROcp27_521*S23-ROcp27_822*C23);
    ROcp27_923 = -(ROcp27_621*S23-ROcp27_922*C23);
    ROcp27_124 = ROcp27_122*C24-ROcp27_723*S24;
    ROcp27_224 = ROcp27_222*C24-ROcp27_823*S24;
    ROcp27_324 = ROcp27_322*C24-ROcp27_923*S24;
    ROcp27_724 = ROcp27_122*S24+ROcp27_723*C24;
    ROcp27_824 = ROcp27_222*S24+ROcp27_823*C24;
    ROcp27_924 = ROcp27_322*S24+ROcp27_923*C24;
    ROcp27_125 = ROcp27_124*C25+ROcp27_423*S25;
    ROcp27_225 = ROcp27_224*C25+ROcp27_523*S25;
    ROcp27_325 = ROcp27_324*C25+ROcp27_623*S25;
    ROcp27_425 = -(ROcp27_124*S25-ROcp27_423*C25);
    ROcp27_525 = -(ROcp27_224*S25-ROcp27_523*C25);
    ROcp27_625 = -(ROcp27_324*S25-ROcp27_623*C25);
    ROcp27_126 = ROcp27_125*C26-ROcp27_724*S26;
    ROcp27_226 = ROcp27_225*C26-ROcp27_824*S26;
    ROcp27_326 = ROcp27_325*C26-ROcp27_924*S26;
    ROcp27_726 = ROcp27_125*S26+ROcp27_724*C26;
    ROcp27_826 = ROcp27_225*S26+ROcp27_824*C26;
    ROcp27_926 = ROcp27_325*S26+ROcp27_924*C26;
    ROcp27_127 = ROcp27_126*C27+ROcp27_425*S27;
    ROcp27_227 = ROcp27_226*C27+ROcp27_525*S27;
    ROcp27_327 = ROcp27_326*C27+ROcp27_625*S27;
    ROcp27_427 = -(ROcp27_126*S27-ROcp27_425*C27);
    ROcp27_527 = -(ROcp27_226*S27-ROcp27_525*C27);
    ROcp27_627 = -(ROcp27_326*S27-ROcp27_625*C27);
    ROcp27_428 = ROcp27_427*C28+ROcp27_726*S28;
    ROcp27_528 = ROcp27_527*C28+ROcp27_826*S28;
    ROcp27_628 = ROcp27_627*C28+ROcp27_926*S28;
    ROcp27_728 = -(ROcp27_427*S28-ROcp27_726*C28);
    ROcp27_828 = -(ROcp27_527*S28-ROcp27_826*C28);
    ROcp27_928 = -(ROcp27_627*S28-ROcp27_926*C28);
    RLcp27_122 = ROcp27_121*s.dpt(1,36)+ROcp27_421*s.dpt(2,36)+ROcp27_720*s.dpt(3,36);
    RLcp27_222 = ROcp27_221*s.dpt(1,36)+ROcp27_521*s.dpt(2,36)+ROcp27_820*s.dpt(3,36);
    RLcp27_322 = ROcp27_321*s.dpt(1,36)+ROcp27_621*s.dpt(2,36)+ROcp27_920*s.dpt(3,36);
    OMcp27_122 = OMcp27_121+ROcp27_421*qd(22);
    OMcp27_222 = OMcp27_221+ROcp27_521*qd(22);
    OMcp27_322 = OMcp27_321+ROcp27_621*qd(22);
    ORcp27_122 = OMcp27_221*RLcp27_322-OMcp27_321*RLcp27_222;
    ORcp27_222 = -(OMcp27_121*RLcp27_322-OMcp27_321*RLcp27_122);
    ORcp27_322 = OMcp27_121*RLcp27_222-OMcp27_221*RLcp27_122;
    OPcp27_122 = OPcp27_121+ROcp27_421*qdd(22)+qd(22)*(OMcp27_221*ROcp27_621-OMcp27_321*ROcp27_521);
    OPcp27_222 = OPcp27_221+ROcp27_521*qdd(22)-qd(22)*(OMcp27_121*ROcp27_621-OMcp27_321*ROcp27_421);
    OPcp27_322 = OPcp27_321+ROcp27_621*qdd(22)+qd(22)*(OMcp27_121*ROcp27_521-OMcp27_221*ROcp27_421);
    RLcp27_123 = s.dpt(1,40)*ROcp27_122+s.dpt(3,40)*ROcp27_722+ROcp27_421*s.dpt(2,40);
    RLcp27_223 = s.dpt(1,40)*ROcp27_222+s.dpt(3,40)*ROcp27_822+ROcp27_521*s.dpt(2,40);
    RLcp27_323 = s.dpt(1,40)*ROcp27_322+s.dpt(3,40)*ROcp27_922+ROcp27_621*s.dpt(2,40);
    OMcp27_123 = OMcp27_122+ROcp27_122*qd(23);
    OMcp27_223 = OMcp27_222+ROcp27_222*qd(23);
    OMcp27_323 = OMcp27_322+ROcp27_322*qd(23);
    ORcp27_123 = OMcp27_222*RLcp27_323-OMcp27_322*RLcp27_223;
    ORcp27_223 = -(OMcp27_122*RLcp27_323-OMcp27_322*RLcp27_123);
    ORcp27_323 = OMcp27_122*RLcp27_223-OMcp27_222*RLcp27_123;
    OPcp27_123 = OPcp27_122+ROcp27_122*qdd(23)+qd(23)*(OMcp27_222*ROcp27_322-OMcp27_322*ROcp27_222);
    OPcp27_223 = OPcp27_222+ROcp27_222*qdd(23)-qd(23)*(OMcp27_122*ROcp27_322-OMcp27_322*ROcp27_122);
    OPcp27_323 = OPcp27_322+ROcp27_322*qdd(23)+qd(23)*(OMcp27_122*ROcp27_222-OMcp27_222*ROcp27_122);
    RLcp27_124 = s.dpt(1,42)*ROcp27_122+s.dpt(3,42)*ROcp27_723+ROcp27_423*s.dpt(2,42);
    RLcp27_224 = s.dpt(1,42)*ROcp27_222+s.dpt(3,42)*ROcp27_823+ROcp27_523*s.dpt(2,42);
    RLcp27_324 = s.dpt(1,42)*ROcp27_322+s.dpt(3,42)*ROcp27_923+ROcp27_623*s.dpt(2,42);
    OMcp27_124 = OMcp27_123+ROcp27_423*qd(24);
    OMcp27_224 = OMcp27_223+ROcp27_523*qd(24);
    OMcp27_324 = OMcp27_323+ROcp27_623*qd(24);
    ORcp27_124 = OMcp27_223*RLcp27_324-OMcp27_323*RLcp27_224;
    ORcp27_224 = -(OMcp27_123*RLcp27_324-OMcp27_323*RLcp27_124);
    ORcp27_324 = OMcp27_123*RLcp27_224-OMcp27_223*RLcp27_124;
    OPcp27_124 = OPcp27_123+ROcp27_423*qdd(24)+qd(24)*(OMcp27_223*ROcp27_623-OMcp27_323*ROcp27_523);
    OPcp27_224 = OPcp27_223+ROcp27_523*qdd(24)-qd(24)*(OMcp27_123*ROcp27_623-OMcp27_323*ROcp27_423);
    OPcp27_324 = OPcp27_323+ROcp27_623*qdd(24)+qd(24)*(OMcp27_123*ROcp27_523-OMcp27_223*ROcp27_423);
    RLcp27_125 = s.dpt(3,44)*ROcp27_724+ROcp27_124*s.dpt(1,44)+ROcp27_423*s.dpt(2,44);
    RLcp27_225 = s.dpt(3,44)*ROcp27_824+ROcp27_224*s.dpt(1,44)+ROcp27_523*s.dpt(2,44);
    RLcp27_325 = s.dpt(3,44)*ROcp27_924+ROcp27_324*s.dpt(1,44)+ROcp27_623*s.dpt(2,44);
    OMcp27_125 = OMcp27_124+ROcp27_724*qd(25);
    OMcp27_225 = OMcp27_224+ROcp27_824*qd(25);
    OMcp27_325 = OMcp27_324+ROcp27_924*qd(25);
    ORcp27_125 = OMcp27_224*RLcp27_325-OMcp27_324*RLcp27_225;
    ORcp27_225 = -(OMcp27_124*RLcp27_325-OMcp27_324*RLcp27_125);
    ORcp27_325 = OMcp27_124*RLcp27_225-OMcp27_224*RLcp27_125;
    OPcp27_125 = OPcp27_124+ROcp27_724*qdd(25)+qd(25)*(OMcp27_224*ROcp27_924-OMcp27_324*ROcp27_824);
    OPcp27_225 = OPcp27_224+ROcp27_824*qdd(25)-qd(25)*(OMcp27_124*ROcp27_924-OMcp27_324*ROcp27_724);
    OPcp27_325 = OPcp27_324+ROcp27_924*qdd(25)+qd(25)*(OMcp27_124*ROcp27_824-OMcp27_224*ROcp27_724);
    RLcp27_126 = s.dpt(2,46)*ROcp27_425+s.dpt(3,46)*ROcp27_724+ROcp27_125*s.dpt(1,46);
    RLcp27_226 = s.dpt(2,46)*ROcp27_525+s.dpt(3,46)*ROcp27_824+ROcp27_225*s.dpt(1,46);
    RLcp27_326 = s.dpt(2,46)*ROcp27_625+s.dpt(3,46)*ROcp27_924+ROcp27_325*s.dpt(1,46);
    OMcp27_126 = OMcp27_125+ROcp27_425*qd(26);
    OMcp27_226 = OMcp27_225+ROcp27_525*qd(26);
    OMcp27_326 = OMcp27_325+ROcp27_625*qd(26);
    ORcp27_126 = OMcp27_225*RLcp27_326-OMcp27_325*RLcp27_226;
    ORcp27_226 = -(OMcp27_125*RLcp27_326-OMcp27_325*RLcp27_126);
    ORcp27_326 = OMcp27_125*RLcp27_226-OMcp27_225*RLcp27_126;
    OPcp27_126 = OPcp27_125+ROcp27_425*qdd(26)+qd(26)*(OMcp27_225*ROcp27_625-OMcp27_325*ROcp27_525);
    OPcp27_226 = OPcp27_225+ROcp27_525*qdd(26)-qd(26)*(OMcp27_125*ROcp27_625-OMcp27_325*ROcp27_425);
    OPcp27_326 = OPcp27_325+ROcp27_625*qdd(26)+qd(26)*(OMcp27_125*ROcp27_525-OMcp27_225*ROcp27_425);
    RLcp27_127 = s.dpt(1,47)*ROcp27_126+s.dpt(3,47)*ROcp27_726+ROcp27_425*s.dpt(2,47);
    RLcp27_227 = s.dpt(1,47)*ROcp27_226+s.dpt(3,47)*ROcp27_826+ROcp27_525*s.dpt(2,47);
    RLcp27_327 = s.dpt(1,47)*ROcp27_326+s.dpt(3,47)*ROcp27_926+ROcp27_625*s.dpt(2,47);
    OMcp27_127 = OMcp27_126+ROcp27_726*qd(27);
    OMcp27_227 = OMcp27_226+ROcp27_826*qd(27);
    OMcp27_327 = OMcp27_326+ROcp27_926*qd(27);
    ORcp27_127 = OMcp27_226*RLcp27_327-OMcp27_326*RLcp27_227;
    ORcp27_227 = -(OMcp27_126*RLcp27_327-OMcp27_326*RLcp27_127);
    ORcp27_327 = OMcp27_126*RLcp27_227-OMcp27_226*RLcp27_127;
    OPcp27_127 = OPcp27_126+ROcp27_726*qdd(27)+qd(27)*(OMcp27_226*ROcp27_926-OMcp27_326*ROcp27_826);
    OPcp27_227 = OPcp27_226+ROcp27_826*qdd(27)-qd(27)*(OMcp27_126*ROcp27_926-OMcp27_326*ROcp27_726);
    OPcp27_327 = OPcp27_326+ROcp27_926*qdd(27)+qd(27)*(OMcp27_126*ROcp27_826-OMcp27_226*ROcp27_726);
    RLcp27_128 = s.dpt(1,48)*ROcp27_127+s.dpt(2,48)*ROcp27_427+s.dpt(3,48)*ROcp27_726;
    RLcp27_228 = s.dpt(1,48)*ROcp27_227+s.dpt(2,48)*ROcp27_527+s.dpt(3,48)*ROcp27_826;
    RLcp27_328 = s.dpt(1,48)*ROcp27_327+s.dpt(2,48)*ROcp27_627+s.dpt(3,48)*ROcp27_926;
    POcp27_128 = RLcp27_119+RLcp27_120+RLcp27_121+RLcp27_122+RLcp27_123+RLcp27_124+RLcp27_125+RLcp27_126+RLcp27_127+RLcp27_128+q(1);
    POcp27_228 = RLcp27_219+RLcp27_220+RLcp27_221+RLcp27_222+RLcp27_223+RLcp27_224+RLcp27_225+RLcp27_226+RLcp27_227+RLcp27_228+q(2);
    POcp27_328 = RLcp27_319+RLcp27_320+RLcp27_321+RLcp27_322+RLcp27_323+RLcp27_324+RLcp27_325+RLcp27_326+RLcp27_327+RLcp27_328+q(3);
    OMcp27_128 = OMcp27_127+ROcp27_127*qd(28);
    OMcp27_228 = OMcp27_227+ROcp27_227*qd(28);
    OMcp27_328 = OMcp27_327+ROcp27_327*qd(28);
    ORcp27_128 = OMcp27_227*RLcp27_328-OMcp27_327*RLcp27_228;
    ORcp27_228 = -(OMcp27_127*RLcp27_328-OMcp27_327*RLcp27_128);
    ORcp27_328 = OMcp27_127*RLcp27_228-OMcp27_227*RLcp27_128;
    VIcp27_128 = ORcp27_119+ORcp27_120+ORcp27_121+ORcp27_122+ORcp27_123+ORcp27_124+ORcp27_125+ORcp27_126+ORcp27_127+ORcp27_128+qd(1);
    VIcp27_228 = ORcp27_219+ORcp27_220+ORcp27_221+ORcp27_222+ORcp27_223+ORcp27_224+ORcp27_225+ORcp27_226+ORcp27_227+ORcp27_228+qd(2);
    VIcp27_328 = ORcp27_319+ORcp27_320+ORcp27_321+ORcp27_322+ORcp27_323+ORcp27_324+ORcp27_325+ORcp27_326+ORcp27_327+ORcp27_328+qd(3);
    OPcp27_128 = OPcp27_127+ROcp27_127*qdd(28)+qd(28)*(OMcp27_227*ROcp27_327-OMcp27_327*ROcp27_227);
    OPcp27_228 = OPcp27_227+ROcp27_227*qdd(28)-qd(28)*(OMcp27_127*ROcp27_327-OMcp27_327*ROcp27_127);
    OPcp27_328 = OPcp27_327+ROcp27_327*qdd(28)+qd(28)*(OMcp27_127*ROcp27_227-OMcp27_227*ROcp27_127);
    ACcp27_128 = qdd(1)+OMcp27_219*ORcp27_320+OMcp27_220*ORcp27_321+OMcp27_221*ORcp27_322+OMcp27_222*ORcp27_323+OMcp27_223*ORcp27_324+OMcp27_224*...
 ORcp27_325+OMcp27_225*ORcp27_326+OMcp27_226*ORcp27_327+OMcp27_227*ORcp27_328+OMcp27_26*ORcp27_319-OMcp27_319*ORcp27_220-OMcp27_320*ORcp27_221-...
 OMcp27_321*ORcp27_222-OMcp27_322*ORcp27_223-OMcp27_323*ORcp27_224-OMcp27_324*ORcp27_225-OMcp27_325*ORcp27_226-OMcp27_326*ORcp27_227-OMcp27_327*...
 ORcp27_228-OMcp27_36*ORcp27_219+OPcp27_219*RLcp27_320+OPcp27_220*RLcp27_321+OPcp27_221*RLcp27_322+OPcp27_222*RLcp27_323+OPcp27_223*RLcp27_324+...
 OPcp27_224*RLcp27_325+OPcp27_225*RLcp27_326+OPcp27_226*RLcp27_327+OPcp27_227*RLcp27_328+OPcp27_26*RLcp27_319-OPcp27_319*RLcp27_220-OPcp27_320*...
 RLcp27_221-OPcp27_321*RLcp27_222-OPcp27_322*RLcp27_223-OPcp27_323*RLcp27_224-OPcp27_324*RLcp27_225-OPcp27_325*RLcp27_226-OPcp27_326*RLcp27_227-...
 OPcp27_327*RLcp27_228-OPcp27_36*RLcp27_219;
    ACcp27_228 = qdd(2)-OMcp27_119*ORcp27_320-OMcp27_120*ORcp27_321-OMcp27_121*ORcp27_322-OMcp27_122*ORcp27_323-OMcp27_123*ORcp27_324-OMcp27_124*...
 ORcp27_325-OMcp27_125*ORcp27_326-OMcp27_126*ORcp27_327-OMcp27_127*ORcp27_328-OMcp27_16*ORcp27_319+OMcp27_319*ORcp27_120+OMcp27_320*ORcp27_121+...
 OMcp27_321*ORcp27_122+OMcp27_322*ORcp27_123+OMcp27_323*ORcp27_124+OMcp27_324*ORcp27_125+OMcp27_325*ORcp27_126+OMcp27_326*ORcp27_127+OMcp27_327*...
 ORcp27_128+OMcp27_36*ORcp27_119-OPcp27_119*RLcp27_320-OPcp27_120*RLcp27_321-OPcp27_121*RLcp27_322-OPcp27_122*RLcp27_323-OPcp27_123*RLcp27_324-...
 OPcp27_124*RLcp27_325-OPcp27_125*RLcp27_326-OPcp27_126*RLcp27_327-OPcp27_127*RLcp27_328-OPcp27_16*RLcp27_319+OPcp27_319*RLcp27_120+OPcp27_320*...
 RLcp27_121+OPcp27_321*RLcp27_122+OPcp27_322*RLcp27_123+OPcp27_323*RLcp27_124+OPcp27_324*RLcp27_125+OPcp27_325*RLcp27_126+OPcp27_326*RLcp27_127+...
 OPcp27_327*RLcp27_128+OPcp27_36*RLcp27_119;
    ACcp27_328 = qdd(3)+OMcp27_119*ORcp27_220+OMcp27_120*ORcp27_221+OMcp27_121*ORcp27_222+OMcp27_122*ORcp27_223+OMcp27_123*ORcp27_224+OMcp27_124*...
 ORcp27_225+OMcp27_125*ORcp27_226+OMcp27_126*ORcp27_227+OMcp27_127*ORcp27_228+OMcp27_16*ORcp27_219-OMcp27_219*ORcp27_120-OMcp27_220*ORcp27_121-...
 OMcp27_221*ORcp27_122-OMcp27_222*ORcp27_123-OMcp27_223*ORcp27_124-OMcp27_224*ORcp27_125-OMcp27_225*ORcp27_126-OMcp27_226*ORcp27_127-OMcp27_227*...
 ORcp27_128-OMcp27_26*ORcp27_119+OPcp27_119*RLcp27_220+OPcp27_120*RLcp27_221+OPcp27_121*RLcp27_222+OPcp27_122*RLcp27_223+OPcp27_123*RLcp27_224+...
 OPcp27_124*RLcp27_225+OPcp27_125*RLcp27_226+OPcp27_126*RLcp27_227+OPcp27_127*RLcp27_228+OPcp27_16*RLcp27_219-OPcp27_219*RLcp27_120-OPcp27_220*...
 RLcp27_121-OPcp27_221*RLcp27_122-OPcp27_222*RLcp27_123-OPcp27_223*RLcp27_124-OPcp27_224*RLcp27_125-OPcp27_225*RLcp27_126-OPcp27_226*RLcp27_127-...
 OPcp27_227*RLcp27_128-OPcp27_26*RLcp27_119;

% = = Block_1_0_0_28_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp27_128;
    sens.P(2) = POcp27_228;
    sens.P(3) = POcp27_328;
    sens.R(1,1) = ROcp27_127;
    sens.R(1,2) = ROcp27_227;
    sens.R(1,3) = ROcp27_327;
    sens.R(2,1) = ROcp27_428;
    sens.R(2,2) = ROcp27_528;
    sens.R(2,3) = ROcp27_628;
    sens.R(3,1) = ROcp27_728;
    sens.R(3,2) = ROcp27_828;
    sens.R(3,3) = ROcp27_928;
    sens.V(1) = VIcp27_128;
    sens.V(2) = VIcp27_228;
    sens.V(3) = VIcp27_328;
    sens.OM(1) = OMcp27_128;
    sens.OM(2) = OMcp27_228;
    sens.OM(3) = OMcp27_328;
    sens.A(1) = ACcp27_128;
    sens.A(2) = ACcp27_228;
    sens.A(3) = ACcp27_328;
    sens.OMP(1) = OPcp27_128;
    sens.OMP(2) = OPcp27_228;
    sens.OMP(3) = OPcp27_328;
 
% 
case 29, 


% = = Block_1_0_0_29_0_1 = = 
 
% Sensor Kinematics 


    ROcp28_25 = S4*S5;
    ROcp28_35 = -C4*S5;
    ROcp28_85 = -S4*C5;
    ROcp28_95 = C4*C5;
    ROcp28_16 = C5*C6;
    ROcp28_26 = ROcp28_25*C6+C4*S6;
    ROcp28_36 = ROcp28_35*C6+S4*S6;
    ROcp28_46 = -C5*S6;
    ROcp28_56 = -(ROcp28_25*S6-C4*C6);
    ROcp28_66 = -(ROcp28_35*S6-S4*C6);
    OMcp28_25 = qd(5)*C4;
    OMcp28_35 = qd(5)*S4;
    OMcp28_16 = qd(4)+qd(6)*S5;
    OMcp28_26 = OMcp28_25+ROcp28_85*qd(6);
    OMcp28_36 = OMcp28_35+ROcp28_95*qd(6);
    OPcp28_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp28_26 = ROcp28_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp28_35*S5-ROcp28_95*qd(4));
    OPcp28_36 = ROcp28_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp28_25*S5-ROcp28_85*qd(4));

% = = Block_1_0_0_29_0_4 = = 
 
% Sensor Kinematics 


    ROcp28_419 = ROcp28_46*C19+S19*S5;
    ROcp28_519 = ROcp28_56*C19+ROcp28_85*S19;
    ROcp28_619 = ROcp28_66*C19+ROcp28_95*S19;
    ROcp28_719 = -(ROcp28_46*S19-C19*S5);
    ROcp28_819 = -(ROcp28_56*S19-ROcp28_85*C19);
    ROcp28_919 = -(ROcp28_66*S19-ROcp28_95*C19);
    ROcp28_120 = ROcp28_16*C20-ROcp28_719*S20;
    ROcp28_220 = ROcp28_26*C20-ROcp28_819*S20;
    ROcp28_320 = ROcp28_36*C20-ROcp28_919*S20;
    ROcp28_720 = ROcp28_16*S20+ROcp28_719*C20;
    ROcp28_820 = ROcp28_26*S20+ROcp28_819*C20;
    ROcp28_920 = ROcp28_36*S20+ROcp28_919*C20;
    ROcp28_121 = ROcp28_120*C21+ROcp28_419*S21;
    ROcp28_221 = ROcp28_220*C21+ROcp28_519*S21;
    ROcp28_321 = ROcp28_320*C21+ROcp28_619*S21;
    ROcp28_421 = -(ROcp28_120*S21-ROcp28_419*C21);
    ROcp28_521 = -(ROcp28_220*S21-ROcp28_519*C21);
    ROcp28_621 = -(ROcp28_320*S21-ROcp28_619*C21);
    RLcp28_119 = s.dpt(2,3)*ROcp28_46+ROcp28_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp28_219 = s.dpt(2,3)*ROcp28_56+ROcp28_26*s.dpt(1,3)+ROcp28_85*s.dpt(3,3);
    RLcp28_319 = s.dpt(2,3)*ROcp28_66+ROcp28_36*s.dpt(1,3)+ROcp28_95*s.dpt(3,3);
    OMcp28_119 = OMcp28_16+ROcp28_16*qd(19);
    OMcp28_219 = OMcp28_26+ROcp28_26*qd(19);
    OMcp28_319 = OMcp28_36+ROcp28_36*qd(19);
    ORcp28_119 = OMcp28_26*RLcp28_319-OMcp28_36*RLcp28_219;
    ORcp28_219 = -(OMcp28_16*RLcp28_319-OMcp28_36*RLcp28_119);
    ORcp28_319 = OMcp28_16*RLcp28_219-OMcp28_26*RLcp28_119;
    OPcp28_119 = OPcp28_16+ROcp28_16*qdd(19)+qd(19)*(OMcp28_26*ROcp28_36-OMcp28_36*ROcp28_26);
    OPcp28_219 = OPcp28_26+ROcp28_26*qdd(19)-qd(19)*(OMcp28_16*ROcp28_36-OMcp28_36*ROcp28_16);
    OPcp28_319 = OPcp28_36+ROcp28_36*qdd(19)+qd(19)*(OMcp28_16*ROcp28_26-OMcp28_26*ROcp28_16);
    RLcp28_120 = s.dpt(1,30)*ROcp28_16+s.dpt(2,30)*ROcp28_419+s.dpt(3,30)*ROcp28_719;
    RLcp28_220 = s.dpt(1,30)*ROcp28_26+s.dpt(2,30)*ROcp28_519+s.dpt(3,30)*ROcp28_819;
    RLcp28_320 = s.dpt(1,30)*ROcp28_36+s.dpt(2,30)*ROcp28_619+s.dpt(3,30)*ROcp28_919;
    OMcp28_120 = OMcp28_119+ROcp28_419*qd(20);
    OMcp28_220 = OMcp28_219+ROcp28_519*qd(20);
    OMcp28_320 = OMcp28_319+ROcp28_619*qd(20);
    ORcp28_120 = OMcp28_219*RLcp28_320-OMcp28_319*RLcp28_220;
    ORcp28_220 = -(OMcp28_119*RLcp28_320-OMcp28_319*RLcp28_120);
    ORcp28_320 = OMcp28_119*RLcp28_220-OMcp28_219*RLcp28_120;
    OPcp28_120 = OPcp28_119+ROcp28_419*qdd(20)+qd(20)*(OMcp28_219*ROcp28_619-OMcp28_319*ROcp28_519);
    OPcp28_220 = OPcp28_219+ROcp28_519*qdd(20)-qd(20)*(OMcp28_119*ROcp28_619-OMcp28_319*ROcp28_419);
    OPcp28_320 = OPcp28_319+ROcp28_619*qdd(20)+qd(20)*(OMcp28_119*ROcp28_519-OMcp28_219*ROcp28_419);
    RLcp28_121 = s.dpt(1,32)*ROcp28_120+s.dpt(2,32)*ROcp28_419+ROcp28_720*s.dpt(3,32);
    RLcp28_221 = s.dpt(1,32)*ROcp28_220+s.dpt(2,32)*ROcp28_519+ROcp28_820*s.dpt(3,32);
    RLcp28_321 = s.dpt(1,32)*ROcp28_320+s.dpt(2,32)*ROcp28_619+ROcp28_920*s.dpt(3,32);
    OMcp28_121 = OMcp28_120+ROcp28_720*qd(21);
    OMcp28_221 = OMcp28_220+ROcp28_820*qd(21);
    OMcp28_321 = OMcp28_320+ROcp28_920*qd(21);
    ORcp28_121 = OMcp28_220*RLcp28_321-OMcp28_320*RLcp28_221;
    ORcp28_221 = -(OMcp28_120*RLcp28_321-OMcp28_320*RLcp28_121);
    ORcp28_321 = OMcp28_120*RLcp28_221-OMcp28_220*RLcp28_121;
    OPcp28_121 = OPcp28_120+ROcp28_720*qdd(21)+qd(21)*(OMcp28_220*ROcp28_920-OMcp28_320*ROcp28_820);
    OPcp28_221 = OPcp28_220+ROcp28_820*qdd(21)-qd(21)*(OMcp28_120*ROcp28_920-OMcp28_320*ROcp28_720);
    OPcp28_321 = OPcp28_320+ROcp28_920*qdd(21)+qd(21)*(OMcp28_120*ROcp28_820-OMcp28_220*ROcp28_720);

% = = Block_1_0_0_29_0_6 = = 
 
% Sensor Kinematics 


    ROcp28_129 = ROcp28_121*C29-ROcp28_720*S29;
    ROcp28_229 = ROcp28_221*C29-ROcp28_820*S29;
    ROcp28_329 = ROcp28_321*C29-ROcp28_920*S29;
    ROcp28_729 = ROcp28_121*S29+ROcp28_720*C29;
    ROcp28_829 = ROcp28_221*S29+ROcp28_820*C29;
    ROcp28_929 = ROcp28_321*S29+ROcp28_920*C29;
    RLcp28_129 = ROcp28_121*s.dpt(1,37)+ROcp28_421*s.dpt(2,37)+ROcp28_720*s.dpt(3,37);
    RLcp28_229 = ROcp28_221*s.dpt(1,37)+ROcp28_521*s.dpt(2,37)+ROcp28_820*s.dpt(3,37);
    RLcp28_329 = ROcp28_321*s.dpt(1,37)+ROcp28_621*s.dpt(2,37)+ROcp28_920*s.dpt(3,37);
    POcp28_129 = RLcp28_119+RLcp28_120+RLcp28_121+RLcp28_129+q(1);
    POcp28_229 = RLcp28_219+RLcp28_220+RLcp28_221+RLcp28_229+q(2);
    POcp28_329 = RLcp28_319+RLcp28_320+RLcp28_321+RLcp28_329+q(3);
    OMcp28_129 = OMcp28_121+ROcp28_421*qd(29);
    OMcp28_229 = OMcp28_221+ROcp28_521*qd(29);
    OMcp28_329 = OMcp28_321+ROcp28_621*qd(29);
    ORcp28_129 = OMcp28_221*RLcp28_329-OMcp28_321*RLcp28_229;
    ORcp28_229 = -(OMcp28_121*RLcp28_329-OMcp28_321*RLcp28_129);
    ORcp28_329 = OMcp28_121*RLcp28_229-OMcp28_221*RLcp28_129;
    VIcp28_129 = ORcp28_119+ORcp28_120+ORcp28_121+ORcp28_129+qd(1);
    VIcp28_229 = ORcp28_219+ORcp28_220+ORcp28_221+ORcp28_229+qd(2);
    VIcp28_329 = ORcp28_319+ORcp28_320+ORcp28_321+ORcp28_329+qd(3);
    OPcp28_129 = OPcp28_121+ROcp28_421*qdd(29)+qd(29)*(OMcp28_221*ROcp28_621-OMcp28_321*ROcp28_521);
    OPcp28_229 = OPcp28_221+ROcp28_521*qdd(29)-qd(29)*(OMcp28_121*ROcp28_621-OMcp28_321*ROcp28_421);
    OPcp28_329 = OPcp28_321+ROcp28_621*qdd(29)+qd(29)*(OMcp28_121*ROcp28_521-OMcp28_221*ROcp28_421);
    ACcp28_129 = qdd(1)+OMcp28_219*ORcp28_320+OMcp28_220*ORcp28_321+OMcp28_221*ORcp28_329+OMcp28_26*ORcp28_319-OMcp28_319*ORcp28_220-OMcp28_320*...
 ORcp28_221-OMcp28_321*ORcp28_229-OMcp28_36*ORcp28_219+OPcp28_219*RLcp28_320+OPcp28_220*RLcp28_321+OPcp28_221*RLcp28_329+OPcp28_26*RLcp28_319-...
 OPcp28_319*RLcp28_220-OPcp28_320*RLcp28_221-OPcp28_321*RLcp28_229-OPcp28_36*RLcp28_219;
    ACcp28_229 = qdd(2)-OMcp28_119*ORcp28_320-OMcp28_120*ORcp28_321-OMcp28_121*ORcp28_329-OMcp28_16*ORcp28_319+OMcp28_319*ORcp28_120+OMcp28_320*...
 ORcp28_121+OMcp28_321*ORcp28_129+OMcp28_36*ORcp28_119-OPcp28_119*RLcp28_320-OPcp28_120*RLcp28_321-OPcp28_121*RLcp28_329-OPcp28_16*RLcp28_319+...
 OPcp28_319*RLcp28_120+OPcp28_320*RLcp28_121+OPcp28_321*RLcp28_129+OPcp28_36*RLcp28_119;
    ACcp28_329 = qdd(3)+OMcp28_119*ORcp28_220+OMcp28_120*ORcp28_221+OMcp28_121*ORcp28_229+OMcp28_16*ORcp28_219-OMcp28_219*ORcp28_120-OMcp28_220*...
 ORcp28_121-OMcp28_221*ORcp28_129-OMcp28_26*ORcp28_119+OPcp28_119*RLcp28_220+OPcp28_120*RLcp28_221+OPcp28_121*RLcp28_229+OPcp28_16*RLcp28_219-...
 OPcp28_219*RLcp28_120-OPcp28_220*RLcp28_121-OPcp28_221*RLcp28_129-OPcp28_26*RLcp28_119;

% = = Block_1_0_0_29_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp28_129;
    sens.P(2) = POcp28_229;
    sens.P(3) = POcp28_329;
    sens.R(1,1) = ROcp28_129;
    sens.R(1,2) = ROcp28_229;
    sens.R(1,3) = ROcp28_329;
    sens.R(2,1) = ROcp28_421;
    sens.R(2,2) = ROcp28_521;
    sens.R(2,3) = ROcp28_621;
    sens.R(3,1) = ROcp28_729;
    sens.R(3,2) = ROcp28_829;
    sens.R(3,3) = ROcp28_929;
    sens.V(1) = VIcp28_129;
    sens.V(2) = VIcp28_229;
    sens.V(3) = VIcp28_329;
    sens.OM(1) = OMcp28_129;
    sens.OM(2) = OMcp28_229;
    sens.OM(3) = OMcp28_329;
    sens.A(1) = ACcp28_129;
    sens.A(2) = ACcp28_229;
    sens.A(3) = ACcp28_329;
    sens.OMP(1) = OPcp28_129;
    sens.OMP(2) = OPcp28_229;
    sens.OMP(3) = OPcp28_329;
 
% 
case 30, 


% = = Block_1_0_0_30_0_1 = = 
 
% Sensor Kinematics 


    ROcp29_25 = S4*S5;
    ROcp29_35 = -C4*S5;
    ROcp29_85 = -S4*C5;
    ROcp29_95 = C4*C5;
    ROcp29_16 = C5*C6;
    ROcp29_26 = ROcp29_25*C6+C4*S6;
    ROcp29_36 = ROcp29_35*C6+S4*S6;
    ROcp29_46 = -C5*S6;
    ROcp29_56 = -(ROcp29_25*S6-C4*C6);
    ROcp29_66 = -(ROcp29_35*S6-S4*C6);
    OMcp29_25 = qd(5)*C4;
    OMcp29_35 = qd(5)*S4;
    OMcp29_16 = qd(4)+qd(6)*S5;
    OMcp29_26 = OMcp29_25+ROcp29_85*qd(6);
    OMcp29_36 = OMcp29_35+ROcp29_95*qd(6);
    OPcp29_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp29_26 = ROcp29_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp29_35*S5-ROcp29_95*qd(4));
    OPcp29_36 = ROcp29_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp29_25*S5-ROcp29_85*qd(4));

% = = Block_1_0_0_30_0_4 = = 
 
% Sensor Kinematics 


    ROcp29_419 = ROcp29_46*C19+S19*S5;
    ROcp29_519 = ROcp29_56*C19+ROcp29_85*S19;
    ROcp29_619 = ROcp29_66*C19+ROcp29_95*S19;
    ROcp29_719 = -(ROcp29_46*S19-C19*S5);
    ROcp29_819 = -(ROcp29_56*S19-ROcp29_85*C19);
    ROcp29_919 = -(ROcp29_66*S19-ROcp29_95*C19);
    ROcp29_120 = ROcp29_16*C20-ROcp29_719*S20;
    ROcp29_220 = ROcp29_26*C20-ROcp29_819*S20;
    ROcp29_320 = ROcp29_36*C20-ROcp29_919*S20;
    ROcp29_720 = ROcp29_16*S20+ROcp29_719*C20;
    ROcp29_820 = ROcp29_26*S20+ROcp29_819*C20;
    ROcp29_920 = ROcp29_36*S20+ROcp29_919*C20;
    ROcp29_121 = ROcp29_120*C21+ROcp29_419*S21;
    ROcp29_221 = ROcp29_220*C21+ROcp29_519*S21;
    ROcp29_321 = ROcp29_320*C21+ROcp29_619*S21;
    ROcp29_421 = -(ROcp29_120*S21-ROcp29_419*C21);
    ROcp29_521 = -(ROcp29_220*S21-ROcp29_519*C21);
    ROcp29_621 = -(ROcp29_320*S21-ROcp29_619*C21);
    RLcp29_119 = s.dpt(2,3)*ROcp29_46+ROcp29_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp29_219 = s.dpt(2,3)*ROcp29_56+ROcp29_26*s.dpt(1,3)+ROcp29_85*s.dpt(3,3);
    RLcp29_319 = s.dpt(2,3)*ROcp29_66+ROcp29_36*s.dpt(1,3)+ROcp29_95*s.dpt(3,3);
    OMcp29_119 = OMcp29_16+ROcp29_16*qd(19);
    OMcp29_219 = OMcp29_26+ROcp29_26*qd(19);
    OMcp29_319 = OMcp29_36+ROcp29_36*qd(19);
    ORcp29_119 = OMcp29_26*RLcp29_319-OMcp29_36*RLcp29_219;
    ORcp29_219 = -(OMcp29_16*RLcp29_319-OMcp29_36*RLcp29_119);
    ORcp29_319 = OMcp29_16*RLcp29_219-OMcp29_26*RLcp29_119;
    OPcp29_119 = OPcp29_16+ROcp29_16*qdd(19)+qd(19)*(OMcp29_26*ROcp29_36-OMcp29_36*ROcp29_26);
    OPcp29_219 = OPcp29_26+ROcp29_26*qdd(19)-qd(19)*(OMcp29_16*ROcp29_36-OMcp29_36*ROcp29_16);
    OPcp29_319 = OPcp29_36+ROcp29_36*qdd(19)+qd(19)*(OMcp29_16*ROcp29_26-OMcp29_26*ROcp29_16);
    RLcp29_120 = s.dpt(1,30)*ROcp29_16+s.dpt(2,30)*ROcp29_419+s.dpt(3,30)*ROcp29_719;
    RLcp29_220 = s.dpt(1,30)*ROcp29_26+s.dpt(2,30)*ROcp29_519+s.dpt(3,30)*ROcp29_819;
    RLcp29_320 = s.dpt(1,30)*ROcp29_36+s.dpt(2,30)*ROcp29_619+s.dpt(3,30)*ROcp29_919;
    OMcp29_120 = OMcp29_119+ROcp29_419*qd(20);
    OMcp29_220 = OMcp29_219+ROcp29_519*qd(20);
    OMcp29_320 = OMcp29_319+ROcp29_619*qd(20);
    ORcp29_120 = OMcp29_219*RLcp29_320-OMcp29_319*RLcp29_220;
    ORcp29_220 = -(OMcp29_119*RLcp29_320-OMcp29_319*RLcp29_120);
    ORcp29_320 = OMcp29_119*RLcp29_220-OMcp29_219*RLcp29_120;
    OPcp29_120 = OPcp29_119+ROcp29_419*qdd(20)+qd(20)*(OMcp29_219*ROcp29_619-OMcp29_319*ROcp29_519);
    OPcp29_220 = OPcp29_219+ROcp29_519*qdd(20)-qd(20)*(OMcp29_119*ROcp29_619-OMcp29_319*ROcp29_419);
    OPcp29_320 = OPcp29_319+ROcp29_619*qdd(20)+qd(20)*(OMcp29_119*ROcp29_519-OMcp29_219*ROcp29_419);
    RLcp29_121 = s.dpt(1,32)*ROcp29_120+s.dpt(2,32)*ROcp29_419+ROcp29_720*s.dpt(3,32);
    RLcp29_221 = s.dpt(1,32)*ROcp29_220+s.dpt(2,32)*ROcp29_519+ROcp29_820*s.dpt(3,32);
    RLcp29_321 = s.dpt(1,32)*ROcp29_320+s.dpt(2,32)*ROcp29_619+ROcp29_920*s.dpt(3,32);
    OMcp29_121 = OMcp29_120+ROcp29_720*qd(21);
    OMcp29_221 = OMcp29_220+ROcp29_820*qd(21);
    OMcp29_321 = OMcp29_320+ROcp29_920*qd(21);
    ORcp29_121 = OMcp29_220*RLcp29_321-OMcp29_320*RLcp29_221;
    ORcp29_221 = -(OMcp29_120*RLcp29_321-OMcp29_320*RLcp29_121);
    ORcp29_321 = OMcp29_120*RLcp29_221-OMcp29_220*RLcp29_121;
    OPcp29_121 = OPcp29_120+ROcp29_720*qdd(21)+qd(21)*(OMcp29_220*ROcp29_920-OMcp29_320*ROcp29_820);
    OPcp29_221 = OPcp29_220+ROcp29_820*qdd(21)-qd(21)*(OMcp29_120*ROcp29_920-OMcp29_320*ROcp29_720);
    OPcp29_321 = OPcp29_320+ROcp29_920*qdd(21)+qd(21)*(OMcp29_120*ROcp29_820-OMcp29_220*ROcp29_720);

% = = Block_1_0_0_30_0_6 = = 
 
% Sensor Kinematics 


    ROcp29_129 = ROcp29_121*C29-ROcp29_720*S29;
    ROcp29_229 = ROcp29_221*C29-ROcp29_820*S29;
    ROcp29_329 = ROcp29_321*C29-ROcp29_920*S29;
    ROcp29_729 = ROcp29_121*S29+ROcp29_720*C29;
    ROcp29_829 = ROcp29_221*S29+ROcp29_820*C29;
    ROcp29_929 = ROcp29_321*S29+ROcp29_920*C29;
    ROcp29_430 = ROcp29_421*C30+ROcp29_729*S30;
    ROcp29_530 = ROcp29_521*C30+ROcp29_829*S30;
    ROcp29_630 = ROcp29_621*C30+ROcp29_929*S30;
    ROcp29_730 = -(ROcp29_421*S30-ROcp29_729*C30);
    ROcp29_830 = -(ROcp29_521*S30-ROcp29_829*C30);
    ROcp29_930 = -(ROcp29_621*S30-ROcp29_929*C30);
    RLcp29_129 = ROcp29_121*s.dpt(1,37)+ROcp29_421*s.dpt(2,37)+ROcp29_720*s.dpt(3,37);
    RLcp29_229 = ROcp29_221*s.dpt(1,37)+ROcp29_521*s.dpt(2,37)+ROcp29_820*s.dpt(3,37);
    RLcp29_329 = ROcp29_321*s.dpt(1,37)+ROcp29_621*s.dpt(2,37)+ROcp29_920*s.dpt(3,37);
    OMcp29_129 = OMcp29_121+ROcp29_421*qd(29);
    OMcp29_229 = OMcp29_221+ROcp29_521*qd(29);
    OMcp29_329 = OMcp29_321+ROcp29_621*qd(29);
    ORcp29_129 = OMcp29_221*RLcp29_329-OMcp29_321*RLcp29_229;
    ORcp29_229 = -(OMcp29_121*RLcp29_329-OMcp29_321*RLcp29_129);
    ORcp29_329 = OMcp29_121*RLcp29_229-OMcp29_221*RLcp29_129;
    OPcp29_129 = OPcp29_121+ROcp29_421*qdd(29)+qd(29)*(OMcp29_221*ROcp29_621-OMcp29_321*ROcp29_521);
    OPcp29_229 = OPcp29_221+ROcp29_521*qdd(29)-qd(29)*(OMcp29_121*ROcp29_621-OMcp29_321*ROcp29_421);
    OPcp29_329 = OPcp29_321+ROcp29_621*qdd(29)+qd(29)*(OMcp29_121*ROcp29_521-OMcp29_221*ROcp29_421);
    RLcp29_130 = s.dpt(1,51)*ROcp29_129+s.dpt(3,51)*ROcp29_729+ROcp29_421*s.dpt(2,51);
    RLcp29_230 = s.dpt(1,51)*ROcp29_229+s.dpt(3,51)*ROcp29_829+ROcp29_521*s.dpt(2,51);
    RLcp29_330 = s.dpt(1,51)*ROcp29_329+s.dpt(3,51)*ROcp29_929+ROcp29_621*s.dpt(2,51);
    POcp29_130 = RLcp29_119+RLcp29_120+RLcp29_121+RLcp29_129+RLcp29_130+q(1);
    POcp29_230 = RLcp29_219+RLcp29_220+RLcp29_221+RLcp29_229+RLcp29_230+q(2);
    POcp29_330 = RLcp29_319+RLcp29_320+RLcp29_321+RLcp29_329+RLcp29_330+q(3);
    OMcp29_130 = OMcp29_129+ROcp29_129*qd(30);
    OMcp29_230 = OMcp29_229+ROcp29_229*qd(30);
    OMcp29_330 = OMcp29_329+ROcp29_329*qd(30);
    ORcp29_130 = OMcp29_229*RLcp29_330-OMcp29_329*RLcp29_230;
    ORcp29_230 = -(OMcp29_129*RLcp29_330-OMcp29_329*RLcp29_130);
    ORcp29_330 = OMcp29_129*RLcp29_230-OMcp29_229*RLcp29_130;
    VIcp29_130 = ORcp29_119+ORcp29_120+ORcp29_121+ORcp29_129+ORcp29_130+qd(1);
    VIcp29_230 = ORcp29_219+ORcp29_220+ORcp29_221+ORcp29_229+ORcp29_230+qd(2);
    VIcp29_330 = ORcp29_319+ORcp29_320+ORcp29_321+ORcp29_329+ORcp29_330+qd(3);
    OPcp29_130 = OPcp29_129+ROcp29_129*qdd(30)+qd(30)*(OMcp29_229*ROcp29_329-OMcp29_329*ROcp29_229);
    OPcp29_230 = OPcp29_229+ROcp29_229*qdd(30)-qd(30)*(OMcp29_129*ROcp29_329-OMcp29_329*ROcp29_129);
    OPcp29_330 = OPcp29_329+ROcp29_329*qdd(30)+qd(30)*(OMcp29_129*ROcp29_229-OMcp29_229*ROcp29_129);
    ACcp29_130 = qdd(1)+OMcp29_219*ORcp29_320+OMcp29_220*ORcp29_321+OMcp29_221*ORcp29_329+OMcp29_229*ORcp29_330+OMcp29_26*ORcp29_319-OMcp29_319*...
 ORcp29_220-OMcp29_320*ORcp29_221-OMcp29_321*ORcp29_229-OMcp29_329*ORcp29_230-OMcp29_36*ORcp29_219+OPcp29_219*RLcp29_320+OPcp29_220*RLcp29_321+...
 OPcp29_221*RLcp29_329+OPcp29_229*RLcp29_330+OPcp29_26*RLcp29_319-OPcp29_319*RLcp29_220-OPcp29_320*RLcp29_221-OPcp29_321*RLcp29_229-OPcp29_329*...
 RLcp29_230-OPcp29_36*RLcp29_219;
    ACcp29_230 = qdd(2)-OMcp29_119*ORcp29_320-OMcp29_120*ORcp29_321-OMcp29_121*ORcp29_329-OMcp29_129*ORcp29_330-OMcp29_16*ORcp29_319+OMcp29_319*...
 ORcp29_120+OMcp29_320*ORcp29_121+OMcp29_321*ORcp29_129+OMcp29_329*ORcp29_130+OMcp29_36*ORcp29_119-OPcp29_119*RLcp29_320-OPcp29_120*RLcp29_321-...
 OPcp29_121*RLcp29_329-OPcp29_129*RLcp29_330-OPcp29_16*RLcp29_319+OPcp29_319*RLcp29_120+OPcp29_320*RLcp29_121+OPcp29_321*RLcp29_129+OPcp29_329*...
 RLcp29_130+OPcp29_36*RLcp29_119;
    ACcp29_330 = qdd(3)+OMcp29_119*ORcp29_220+OMcp29_120*ORcp29_221+OMcp29_121*ORcp29_229+OMcp29_129*ORcp29_230+OMcp29_16*ORcp29_219-OMcp29_219*...
 ORcp29_120-OMcp29_220*ORcp29_121-OMcp29_221*ORcp29_129-OMcp29_229*ORcp29_130-OMcp29_26*ORcp29_119+OPcp29_119*RLcp29_220+OPcp29_120*RLcp29_221+...
 OPcp29_121*RLcp29_229+OPcp29_129*RLcp29_230+OPcp29_16*RLcp29_219-OPcp29_219*RLcp29_120-OPcp29_220*RLcp29_121-OPcp29_221*RLcp29_129-OPcp29_229*...
 RLcp29_130-OPcp29_26*RLcp29_119;

% = = Block_1_0_0_30_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp29_130;
    sens.P(2) = POcp29_230;
    sens.P(3) = POcp29_330;
    sens.R(1,1) = ROcp29_129;
    sens.R(1,2) = ROcp29_229;
    sens.R(1,3) = ROcp29_329;
    sens.R(2,1) = ROcp29_430;
    sens.R(2,2) = ROcp29_530;
    sens.R(2,3) = ROcp29_630;
    sens.R(3,1) = ROcp29_730;
    sens.R(3,2) = ROcp29_830;
    sens.R(3,3) = ROcp29_930;
    sens.V(1) = VIcp29_130;
    sens.V(2) = VIcp29_230;
    sens.V(3) = VIcp29_330;
    sens.OM(1) = OMcp29_130;
    sens.OM(2) = OMcp29_230;
    sens.OM(3) = OMcp29_330;
    sens.A(1) = ACcp29_130;
    sens.A(2) = ACcp29_230;
    sens.A(3) = ACcp29_330;
    sens.OMP(1) = OPcp29_130;
    sens.OMP(2) = OPcp29_230;
    sens.OMP(3) = OPcp29_330;
 
% 
case 31, 


% = = Block_1_0_0_31_0_1 = = 
 
% Sensor Kinematics 


    ROcp30_25 = S4*S5;
    ROcp30_35 = -C4*S5;
    ROcp30_85 = -S4*C5;
    ROcp30_95 = C4*C5;
    ROcp30_16 = C5*C6;
    ROcp30_26 = ROcp30_25*C6+C4*S6;
    ROcp30_36 = ROcp30_35*C6+S4*S6;
    ROcp30_46 = -C5*S6;
    ROcp30_56 = -(ROcp30_25*S6-C4*C6);
    ROcp30_66 = -(ROcp30_35*S6-S4*C6);
    OMcp30_25 = qd(5)*C4;
    OMcp30_35 = qd(5)*S4;
    OMcp30_16 = qd(4)+qd(6)*S5;
    OMcp30_26 = OMcp30_25+ROcp30_85*qd(6);
    OMcp30_36 = OMcp30_35+ROcp30_95*qd(6);
    OPcp30_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp30_26 = ROcp30_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp30_35*S5-ROcp30_95*qd(4));
    OPcp30_36 = ROcp30_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp30_25*S5-ROcp30_85*qd(4));

% = = Block_1_0_0_31_0_4 = = 
 
% Sensor Kinematics 


    ROcp30_419 = ROcp30_46*C19+S19*S5;
    ROcp30_519 = ROcp30_56*C19+ROcp30_85*S19;
    ROcp30_619 = ROcp30_66*C19+ROcp30_95*S19;
    ROcp30_719 = -(ROcp30_46*S19-C19*S5);
    ROcp30_819 = -(ROcp30_56*S19-ROcp30_85*C19);
    ROcp30_919 = -(ROcp30_66*S19-ROcp30_95*C19);
    ROcp30_120 = ROcp30_16*C20-ROcp30_719*S20;
    ROcp30_220 = ROcp30_26*C20-ROcp30_819*S20;
    ROcp30_320 = ROcp30_36*C20-ROcp30_919*S20;
    ROcp30_720 = ROcp30_16*S20+ROcp30_719*C20;
    ROcp30_820 = ROcp30_26*S20+ROcp30_819*C20;
    ROcp30_920 = ROcp30_36*S20+ROcp30_919*C20;
    ROcp30_121 = ROcp30_120*C21+ROcp30_419*S21;
    ROcp30_221 = ROcp30_220*C21+ROcp30_519*S21;
    ROcp30_321 = ROcp30_320*C21+ROcp30_619*S21;
    ROcp30_421 = -(ROcp30_120*S21-ROcp30_419*C21);
    ROcp30_521 = -(ROcp30_220*S21-ROcp30_519*C21);
    ROcp30_621 = -(ROcp30_320*S21-ROcp30_619*C21);
    RLcp30_119 = s.dpt(2,3)*ROcp30_46+ROcp30_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp30_219 = s.dpt(2,3)*ROcp30_56+ROcp30_26*s.dpt(1,3)+ROcp30_85*s.dpt(3,3);
    RLcp30_319 = s.dpt(2,3)*ROcp30_66+ROcp30_36*s.dpt(1,3)+ROcp30_95*s.dpt(3,3);
    OMcp30_119 = OMcp30_16+ROcp30_16*qd(19);
    OMcp30_219 = OMcp30_26+ROcp30_26*qd(19);
    OMcp30_319 = OMcp30_36+ROcp30_36*qd(19);
    ORcp30_119 = OMcp30_26*RLcp30_319-OMcp30_36*RLcp30_219;
    ORcp30_219 = -(OMcp30_16*RLcp30_319-OMcp30_36*RLcp30_119);
    ORcp30_319 = OMcp30_16*RLcp30_219-OMcp30_26*RLcp30_119;
    OPcp30_119 = OPcp30_16+ROcp30_16*qdd(19)+qd(19)*(OMcp30_26*ROcp30_36-OMcp30_36*ROcp30_26);
    OPcp30_219 = OPcp30_26+ROcp30_26*qdd(19)-qd(19)*(OMcp30_16*ROcp30_36-OMcp30_36*ROcp30_16);
    OPcp30_319 = OPcp30_36+ROcp30_36*qdd(19)+qd(19)*(OMcp30_16*ROcp30_26-OMcp30_26*ROcp30_16);
    RLcp30_120 = s.dpt(1,30)*ROcp30_16+s.dpt(2,30)*ROcp30_419+s.dpt(3,30)*ROcp30_719;
    RLcp30_220 = s.dpt(1,30)*ROcp30_26+s.dpt(2,30)*ROcp30_519+s.dpt(3,30)*ROcp30_819;
    RLcp30_320 = s.dpt(1,30)*ROcp30_36+s.dpt(2,30)*ROcp30_619+s.dpt(3,30)*ROcp30_919;
    OMcp30_120 = OMcp30_119+ROcp30_419*qd(20);
    OMcp30_220 = OMcp30_219+ROcp30_519*qd(20);
    OMcp30_320 = OMcp30_319+ROcp30_619*qd(20);
    ORcp30_120 = OMcp30_219*RLcp30_320-OMcp30_319*RLcp30_220;
    ORcp30_220 = -(OMcp30_119*RLcp30_320-OMcp30_319*RLcp30_120);
    ORcp30_320 = OMcp30_119*RLcp30_220-OMcp30_219*RLcp30_120;
    OPcp30_120 = OPcp30_119+ROcp30_419*qdd(20)+qd(20)*(OMcp30_219*ROcp30_619-OMcp30_319*ROcp30_519);
    OPcp30_220 = OPcp30_219+ROcp30_519*qdd(20)-qd(20)*(OMcp30_119*ROcp30_619-OMcp30_319*ROcp30_419);
    OPcp30_320 = OPcp30_319+ROcp30_619*qdd(20)+qd(20)*(OMcp30_119*ROcp30_519-OMcp30_219*ROcp30_419);
    RLcp30_121 = s.dpt(1,32)*ROcp30_120+s.dpt(2,32)*ROcp30_419+ROcp30_720*s.dpt(3,32);
    RLcp30_221 = s.dpt(1,32)*ROcp30_220+s.dpt(2,32)*ROcp30_519+ROcp30_820*s.dpt(3,32);
    RLcp30_321 = s.dpt(1,32)*ROcp30_320+s.dpt(2,32)*ROcp30_619+ROcp30_920*s.dpt(3,32);
    OMcp30_121 = OMcp30_120+ROcp30_720*qd(21);
    OMcp30_221 = OMcp30_220+ROcp30_820*qd(21);
    OMcp30_321 = OMcp30_320+ROcp30_920*qd(21);
    ORcp30_121 = OMcp30_220*RLcp30_321-OMcp30_320*RLcp30_221;
    ORcp30_221 = -(OMcp30_120*RLcp30_321-OMcp30_320*RLcp30_121);
    ORcp30_321 = OMcp30_120*RLcp30_221-OMcp30_220*RLcp30_121;
    OPcp30_121 = OPcp30_120+ROcp30_720*qdd(21)+qd(21)*(OMcp30_220*ROcp30_920-OMcp30_320*ROcp30_820);
    OPcp30_221 = OPcp30_220+ROcp30_820*qdd(21)-qd(21)*(OMcp30_120*ROcp30_920-OMcp30_320*ROcp30_720);
    OPcp30_321 = OPcp30_320+ROcp30_920*qdd(21)+qd(21)*(OMcp30_120*ROcp30_820-OMcp30_220*ROcp30_720);

% = = Block_1_0_0_31_0_6 = = 
 
% Sensor Kinematics 


    ROcp30_129 = ROcp30_121*C29-ROcp30_720*S29;
    ROcp30_229 = ROcp30_221*C29-ROcp30_820*S29;
    ROcp30_329 = ROcp30_321*C29-ROcp30_920*S29;
    ROcp30_729 = ROcp30_121*S29+ROcp30_720*C29;
    ROcp30_829 = ROcp30_221*S29+ROcp30_820*C29;
    ROcp30_929 = ROcp30_321*S29+ROcp30_920*C29;
    ROcp30_430 = ROcp30_421*C30+ROcp30_729*S30;
    ROcp30_530 = ROcp30_521*C30+ROcp30_829*S30;
    ROcp30_630 = ROcp30_621*C30+ROcp30_929*S30;
    ROcp30_730 = -(ROcp30_421*S30-ROcp30_729*C30);
    ROcp30_830 = -(ROcp30_521*S30-ROcp30_829*C30);
    ROcp30_930 = -(ROcp30_621*S30-ROcp30_929*C30);
    ROcp30_131 = ROcp30_129*C31-ROcp30_730*S31;
    ROcp30_231 = ROcp30_229*C31-ROcp30_830*S31;
    ROcp30_331 = ROcp30_329*C31-ROcp30_930*S31;
    ROcp30_731 = ROcp30_129*S31+ROcp30_730*C31;
    ROcp30_831 = ROcp30_229*S31+ROcp30_830*C31;
    ROcp30_931 = ROcp30_329*S31+ROcp30_930*C31;
    RLcp30_129 = ROcp30_121*s.dpt(1,37)+ROcp30_421*s.dpt(2,37)+ROcp30_720*s.dpt(3,37);
    RLcp30_229 = ROcp30_221*s.dpt(1,37)+ROcp30_521*s.dpt(2,37)+ROcp30_820*s.dpt(3,37);
    RLcp30_329 = ROcp30_321*s.dpt(1,37)+ROcp30_621*s.dpt(2,37)+ROcp30_920*s.dpt(3,37);
    OMcp30_129 = OMcp30_121+ROcp30_421*qd(29);
    OMcp30_229 = OMcp30_221+ROcp30_521*qd(29);
    OMcp30_329 = OMcp30_321+ROcp30_621*qd(29);
    ORcp30_129 = OMcp30_221*RLcp30_329-OMcp30_321*RLcp30_229;
    ORcp30_229 = -(OMcp30_121*RLcp30_329-OMcp30_321*RLcp30_129);
    ORcp30_329 = OMcp30_121*RLcp30_229-OMcp30_221*RLcp30_129;
    OPcp30_129 = OPcp30_121+ROcp30_421*qdd(29)+qd(29)*(OMcp30_221*ROcp30_621-OMcp30_321*ROcp30_521);
    OPcp30_229 = OPcp30_221+ROcp30_521*qdd(29)-qd(29)*(OMcp30_121*ROcp30_621-OMcp30_321*ROcp30_421);
    OPcp30_329 = OPcp30_321+ROcp30_621*qdd(29)+qd(29)*(OMcp30_121*ROcp30_521-OMcp30_221*ROcp30_421);
    RLcp30_130 = s.dpt(1,51)*ROcp30_129+s.dpt(3,51)*ROcp30_729+ROcp30_421*s.dpt(2,51);
    RLcp30_230 = s.dpt(1,51)*ROcp30_229+s.dpt(3,51)*ROcp30_829+ROcp30_521*s.dpt(2,51);
    RLcp30_330 = s.dpt(1,51)*ROcp30_329+s.dpt(3,51)*ROcp30_929+ROcp30_621*s.dpt(2,51);
    OMcp30_130 = OMcp30_129+ROcp30_129*qd(30);
    OMcp30_230 = OMcp30_229+ROcp30_229*qd(30);
    OMcp30_330 = OMcp30_329+ROcp30_329*qd(30);
    ORcp30_130 = OMcp30_229*RLcp30_330-OMcp30_329*RLcp30_230;
    ORcp30_230 = -(OMcp30_129*RLcp30_330-OMcp30_329*RLcp30_130);
    ORcp30_330 = OMcp30_129*RLcp30_230-OMcp30_229*RLcp30_130;
    OPcp30_130 = OPcp30_129+ROcp30_129*qdd(30)+qd(30)*(OMcp30_229*ROcp30_329-OMcp30_329*ROcp30_229);
    OPcp30_230 = OPcp30_229+ROcp30_229*qdd(30)-qd(30)*(OMcp30_129*ROcp30_329-OMcp30_329*ROcp30_129);
    OPcp30_330 = OPcp30_329+ROcp30_329*qdd(30)+qd(30)*(OMcp30_129*ROcp30_229-OMcp30_229*ROcp30_129);
    RLcp30_131 = s.dpt(1,53)*ROcp30_129+s.dpt(3,53)*ROcp30_730+ROcp30_430*s.dpt(2,53);
    RLcp30_231 = s.dpt(1,53)*ROcp30_229+s.dpt(3,53)*ROcp30_830+ROcp30_530*s.dpt(2,53);
    RLcp30_331 = s.dpt(1,53)*ROcp30_329+s.dpt(3,53)*ROcp30_930+ROcp30_630*s.dpt(2,53);
    POcp30_131 = RLcp30_119+RLcp30_120+RLcp30_121+RLcp30_129+RLcp30_130+RLcp30_131+q(1);
    POcp30_231 = RLcp30_219+RLcp30_220+RLcp30_221+RLcp30_229+RLcp30_230+RLcp30_231+q(2);
    POcp30_331 = RLcp30_319+RLcp30_320+RLcp30_321+RLcp30_329+RLcp30_330+RLcp30_331+q(3);
    OMcp30_131 = OMcp30_130+ROcp30_430*qd(31);
    OMcp30_231 = OMcp30_230+ROcp30_530*qd(31);
    OMcp30_331 = OMcp30_330+ROcp30_630*qd(31);
    ORcp30_131 = OMcp30_230*RLcp30_331-OMcp30_330*RLcp30_231;
    ORcp30_231 = -(OMcp30_130*RLcp30_331-OMcp30_330*RLcp30_131);
    ORcp30_331 = OMcp30_130*RLcp30_231-OMcp30_230*RLcp30_131;
    VIcp30_131 = ORcp30_119+ORcp30_120+ORcp30_121+ORcp30_129+ORcp30_130+ORcp30_131+qd(1);
    VIcp30_231 = ORcp30_219+ORcp30_220+ORcp30_221+ORcp30_229+ORcp30_230+ORcp30_231+qd(2);
    VIcp30_331 = ORcp30_319+ORcp30_320+ORcp30_321+ORcp30_329+ORcp30_330+ORcp30_331+qd(3);
    OPcp30_131 = OPcp30_130+ROcp30_430*qdd(31)+qd(31)*(OMcp30_230*ROcp30_630-OMcp30_330*ROcp30_530);
    OPcp30_231 = OPcp30_230+ROcp30_530*qdd(31)-qd(31)*(OMcp30_130*ROcp30_630-OMcp30_330*ROcp30_430);
    OPcp30_331 = OPcp30_330+ROcp30_630*qdd(31)+qd(31)*(OMcp30_130*ROcp30_530-OMcp30_230*ROcp30_430);
    ACcp30_131 = qdd(1)+OMcp30_219*ORcp30_320+OMcp30_220*ORcp30_321+OMcp30_221*ORcp30_329+OMcp30_229*ORcp30_330+OMcp30_230*ORcp30_331+OMcp30_26*...
 ORcp30_319-OMcp30_319*ORcp30_220-OMcp30_320*ORcp30_221-OMcp30_321*ORcp30_229-OMcp30_329*ORcp30_230-OMcp30_330*ORcp30_231-OMcp30_36*ORcp30_219+...
 OPcp30_219*RLcp30_320+OPcp30_220*RLcp30_321+OPcp30_221*RLcp30_329+OPcp30_229*RLcp30_330+OPcp30_230*RLcp30_331+OPcp30_26*RLcp30_319-OPcp30_319*...
 RLcp30_220-OPcp30_320*RLcp30_221-OPcp30_321*RLcp30_229-OPcp30_329*RLcp30_230-OPcp30_330*RLcp30_231-OPcp30_36*RLcp30_219;
    ACcp30_231 = qdd(2)-OMcp30_119*ORcp30_320-OMcp30_120*ORcp30_321-OMcp30_121*ORcp30_329-OMcp30_129*ORcp30_330-OMcp30_130*ORcp30_331-OMcp30_16*...
 ORcp30_319+OMcp30_319*ORcp30_120+OMcp30_320*ORcp30_121+OMcp30_321*ORcp30_129+OMcp30_329*ORcp30_130+OMcp30_330*ORcp30_131+OMcp30_36*ORcp30_119-...
 OPcp30_119*RLcp30_320-OPcp30_120*RLcp30_321-OPcp30_121*RLcp30_329-OPcp30_129*RLcp30_330-OPcp30_130*RLcp30_331-OPcp30_16*RLcp30_319+OPcp30_319*...
 RLcp30_120+OPcp30_320*RLcp30_121+OPcp30_321*RLcp30_129+OPcp30_329*RLcp30_130+OPcp30_330*RLcp30_131+OPcp30_36*RLcp30_119;
    ACcp30_331 = qdd(3)+OMcp30_119*ORcp30_220+OMcp30_120*ORcp30_221+OMcp30_121*ORcp30_229+OMcp30_129*ORcp30_230+OMcp30_130*ORcp30_231+OMcp30_16*...
 ORcp30_219-OMcp30_219*ORcp30_120-OMcp30_220*ORcp30_121-OMcp30_221*ORcp30_129-OMcp30_229*ORcp30_130-OMcp30_230*ORcp30_131-OMcp30_26*ORcp30_119+...
 OPcp30_119*RLcp30_220+OPcp30_120*RLcp30_221+OPcp30_121*RLcp30_229+OPcp30_129*RLcp30_230+OPcp30_130*RLcp30_231+OPcp30_16*RLcp30_219-OPcp30_219*...
 RLcp30_120-OPcp30_220*RLcp30_121-OPcp30_221*RLcp30_129-OPcp30_229*RLcp30_130-OPcp30_230*RLcp30_131-OPcp30_26*RLcp30_119;

% = = Block_1_0_0_31_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp30_131;
    sens.P(2) = POcp30_231;
    sens.P(3) = POcp30_331;
    sens.R(1,1) = ROcp30_131;
    sens.R(1,2) = ROcp30_231;
    sens.R(1,3) = ROcp30_331;
    sens.R(2,1) = ROcp30_430;
    sens.R(2,2) = ROcp30_530;
    sens.R(2,3) = ROcp30_630;
    sens.R(3,1) = ROcp30_731;
    sens.R(3,2) = ROcp30_831;
    sens.R(3,3) = ROcp30_931;
    sens.V(1) = VIcp30_131;
    sens.V(2) = VIcp30_231;
    sens.V(3) = VIcp30_331;
    sens.OM(1) = OMcp30_131;
    sens.OM(2) = OMcp30_231;
    sens.OM(3) = OMcp30_331;
    sens.A(1) = ACcp30_131;
    sens.A(2) = ACcp30_231;
    sens.A(3) = ACcp30_331;
    sens.OMP(1) = OPcp30_131;
    sens.OMP(2) = OPcp30_231;
    sens.OMP(3) = OPcp30_331;
 
% 
case 32, 


% = = Block_1_0_0_32_0_1 = = 
 
% Sensor Kinematics 


    ROcp31_25 = S4*S5;
    ROcp31_35 = -C4*S5;
    ROcp31_85 = -S4*C5;
    ROcp31_95 = C4*C5;
    ROcp31_16 = C5*C6;
    ROcp31_26 = ROcp31_25*C6+C4*S6;
    ROcp31_36 = ROcp31_35*C6+S4*S6;
    ROcp31_46 = -C5*S6;
    ROcp31_56 = -(ROcp31_25*S6-C4*C6);
    ROcp31_66 = -(ROcp31_35*S6-S4*C6);
    OMcp31_25 = qd(5)*C4;
    OMcp31_35 = qd(5)*S4;
    OMcp31_16 = qd(4)+qd(6)*S5;
    OMcp31_26 = OMcp31_25+ROcp31_85*qd(6);
    OMcp31_36 = OMcp31_35+ROcp31_95*qd(6);
    OPcp31_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp31_26 = ROcp31_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp31_35*S5-ROcp31_95*qd(4));
    OPcp31_36 = ROcp31_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp31_25*S5-ROcp31_85*qd(4));

% = = Block_1_0_0_32_0_4 = = 
 
% Sensor Kinematics 


    ROcp31_419 = ROcp31_46*C19+S19*S5;
    ROcp31_519 = ROcp31_56*C19+ROcp31_85*S19;
    ROcp31_619 = ROcp31_66*C19+ROcp31_95*S19;
    ROcp31_719 = -(ROcp31_46*S19-C19*S5);
    ROcp31_819 = -(ROcp31_56*S19-ROcp31_85*C19);
    ROcp31_919 = -(ROcp31_66*S19-ROcp31_95*C19);
    ROcp31_120 = ROcp31_16*C20-ROcp31_719*S20;
    ROcp31_220 = ROcp31_26*C20-ROcp31_819*S20;
    ROcp31_320 = ROcp31_36*C20-ROcp31_919*S20;
    ROcp31_720 = ROcp31_16*S20+ROcp31_719*C20;
    ROcp31_820 = ROcp31_26*S20+ROcp31_819*C20;
    ROcp31_920 = ROcp31_36*S20+ROcp31_919*C20;
    ROcp31_121 = ROcp31_120*C21+ROcp31_419*S21;
    ROcp31_221 = ROcp31_220*C21+ROcp31_519*S21;
    ROcp31_321 = ROcp31_320*C21+ROcp31_619*S21;
    ROcp31_421 = -(ROcp31_120*S21-ROcp31_419*C21);
    ROcp31_521 = -(ROcp31_220*S21-ROcp31_519*C21);
    ROcp31_621 = -(ROcp31_320*S21-ROcp31_619*C21);
    RLcp31_119 = s.dpt(2,3)*ROcp31_46+ROcp31_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp31_219 = s.dpt(2,3)*ROcp31_56+ROcp31_26*s.dpt(1,3)+ROcp31_85*s.dpt(3,3);
    RLcp31_319 = s.dpt(2,3)*ROcp31_66+ROcp31_36*s.dpt(1,3)+ROcp31_95*s.dpt(3,3);
    OMcp31_119 = OMcp31_16+ROcp31_16*qd(19);
    OMcp31_219 = OMcp31_26+ROcp31_26*qd(19);
    OMcp31_319 = OMcp31_36+ROcp31_36*qd(19);
    ORcp31_119 = OMcp31_26*RLcp31_319-OMcp31_36*RLcp31_219;
    ORcp31_219 = -(OMcp31_16*RLcp31_319-OMcp31_36*RLcp31_119);
    ORcp31_319 = OMcp31_16*RLcp31_219-OMcp31_26*RLcp31_119;
    OPcp31_119 = OPcp31_16+ROcp31_16*qdd(19)+qd(19)*(OMcp31_26*ROcp31_36-OMcp31_36*ROcp31_26);
    OPcp31_219 = OPcp31_26+ROcp31_26*qdd(19)-qd(19)*(OMcp31_16*ROcp31_36-OMcp31_36*ROcp31_16);
    OPcp31_319 = OPcp31_36+ROcp31_36*qdd(19)+qd(19)*(OMcp31_16*ROcp31_26-OMcp31_26*ROcp31_16);
    RLcp31_120 = s.dpt(1,30)*ROcp31_16+s.dpt(2,30)*ROcp31_419+s.dpt(3,30)*ROcp31_719;
    RLcp31_220 = s.dpt(1,30)*ROcp31_26+s.dpt(2,30)*ROcp31_519+s.dpt(3,30)*ROcp31_819;
    RLcp31_320 = s.dpt(1,30)*ROcp31_36+s.dpt(2,30)*ROcp31_619+s.dpt(3,30)*ROcp31_919;
    OMcp31_120 = OMcp31_119+ROcp31_419*qd(20);
    OMcp31_220 = OMcp31_219+ROcp31_519*qd(20);
    OMcp31_320 = OMcp31_319+ROcp31_619*qd(20);
    ORcp31_120 = OMcp31_219*RLcp31_320-OMcp31_319*RLcp31_220;
    ORcp31_220 = -(OMcp31_119*RLcp31_320-OMcp31_319*RLcp31_120);
    ORcp31_320 = OMcp31_119*RLcp31_220-OMcp31_219*RLcp31_120;
    OPcp31_120 = OPcp31_119+ROcp31_419*qdd(20)+qd(20)*(OMcp31_219*ROcp31_619-OMcp31_319*ROcp31_519);
    OPcp31_220 = OPcp31_219+ROcp31_519*qdd(20)-qd(20)*(OMcp31_119*ROcp31_619-OMcp31_319*ROcp31_419);
    OPcp31_320 = OPcp31_319+ROcp31_619*qdd(20)+qd(20)*(OMcp31_119*ROcp31_519-OMcp31_219*ROcp31_419);
    RLcp31_121 = s.dpt(1,32)*ROcp31_120+s.dpt(2,32)*ROcp31_419+ROcp31_720*s.dpt(3,32);
    RLcp31_221 = s.dpt(1,32)*ROcp31_220+s.dpt(2,32)*ROcp31_519+ROcp31_820*s.dpt(3,32);
    RLcp31_321 = s.dpt(1,32)*ROcp31_320+s.dpt(2,32)*ROcp31_619+ROcp31_920*s.dpt(3,32);
    OMcp31_121 = OMcp31_120+ROcp31_720*qd(21);
    OMcp31_221 = OMcp31_220+ROcp31_820*qd(21);
    OMcp31_321 = OMcp31_320+ROcp31_920*qd(21);
    ORcp31_121 = OMcp31_220*RLcp31_321-OMcp31_320*RLcp31_221;
    ORcp31_221 = -(OMcp31_120*RLcp31_321-OMcp31_320*RLcp31_121);
    ORcp31_321 = OMcp31_120*RLcp31_221-OMcp31_220*RLcp31_121;
    OPcp31_121 = OPcp31_120+ROcp31_720*qdd(21)+qd(21)*(OMcp31_220*ROcp31_920-OMcp31_320*ROcp31_820);
    OPcp31_221 = OPcp31_220+ROcp31_820*qdd(21)-qd(21)*(OMcp31_120*ROcp31_920-OMcp31_320*ROcp31_720);
    OPcp31_321 = OPcp31_320+ROcp31_920*qdd(21)+qd(21)*(OMcp31_120*ROcp31_820-OMcp31_220*ROcp31_720);

% = = Block_1_0_0_32_0_6 = = 
 
% Sensor Kinematics 


    ROcp31_129 = ROcp31_121*C29-ROcp31_720*S29;
    ROcp31_229 = ROcp31_221*C29-ROcp31_820*S29;
    ROcp31_329 = ROcp31_321*C29-ROcp31_920*S29;
    ROcp31_729 = ROcp31_121*S29+ROcp31_720*C29;
    ROcp31_829 = ROcp31_221*S29+ROcp31_820*C29;
    ROcp31_929 = ROcp31_321*S29+ROcp31_920*C29;
    ROcp31_430 = ROcp31_421*C30+ROcp31_729*S30;
    ROcp31_530 = ROcp31_521*C30+ROcp31_829*S30;
    ROcp31_630 = ROcp31_621*C30+ROcp31_929*S30;
    ROcp31_730 = -(ROcp31_421*S30-ROcp31_729*C30);
    ROcp31_830 = -(ROcp31_521*S30-ROcp31_829*C30);
    ROcp31_930 = -(ROcp31_621*S30-ROcp31_929*C30);
    ROcp31_131 = ROcp31_129*C31-ROcp31_730*S31;
    ROcp31_231 = ROcp31_229*C31-ROcp31_830*S31;
    ROcp31_331 = ROcp31_329*C31-ROcp31_930*S31;
    ROcp31_731 = ROcp31_129*S31+ROcp31_730*C31;
    ROcp31_831 = ROcp31_229*S31+ROcp31_830*C31;
    ROcp31_931 = ROcp31_329*S31+ROcp31_930*C31;
    ROcp31_132 = ROcp31_131*C32+ROcp31_430*S32;
    ROcp31_232 = ROcp31_231*C32+ROcp31_530*S32;
    ROcp31_332 = ROcp31_331*C32+ROcp31_630*S32;
    ROcp31_432 = -(ROcp31_131*S32-ROcp31_430*C32);
    ROcp31_532 = -(ROcp31_231*S32-ROcp31_530*C32);
    ROcp31_632 = -(ROcp31_331*S32-ROcp31_630*C32);
    RLcp31_129 = ROcp31_121*s.dpt(1,37)+ROcp31_421*s.dpt(2,37)+ROcp31_720*s.dpt(3,37);
    RLcp31_229 = ROcp31_221*s.dpt(1,37)+ROcp31_521*s.dpt(2,37)+ROcp31_820*s.dpt(3,37);
    RLcp31_329 = ROcp31_321*s.dpt(1,37)+ROcp31_621*s.dpt(2,37)+ROcp31_920*s.dpt(3,37);
    OMcp31_129 = OMcp31_121+ROcp31_421*qd(29);
    OMcp31_229 = OMcp31_221+ROcp31_521*qd(29);
    OMcp31_329 = OMcp31_321+ROcp31_621*qd(29);
    ORcp31_129 = OMcp31_221*RLcp31_329-OMcp31_321*RLcp31_229;
    ORcp31_229 = -(OMcp31_121*RLcp31_329-OMcp31_321*RLcp31_129);
    ORcp31_329 = OMcp31_121*RLcp31_229-OMcp31_221*RLcp31_129;
    OPcp31_129 = OPcp31_121+ROcp31_421*qdd(29)+qd(29)*(OMcp31_221*ROcp31_621-OMcp31_321*ROcp31_521);
    OPcp31_229 = OPcp31_221+ROcp31_521*qdd(29)-qd(29)*(OMcp31_121*ROcp31_621-OMcp31_321*ROcp31_421);
    OPcp31_329 = OPcp31_321+ROcp31_621*qdd(29)+qd(29)*(OMcp31_121*ROcp31_521-OMcp31_221*ROcp31_421);
    RLcp31_130 = s.dpt(1,51)*ROcp31_129+s.dpt(3,51)*ROcp31_729+ROcp31_421*s.dpt(2,51);
    RLcp31_230 = s.dpt(1,51)*ROcp31_229+s.dpt(3,51)*ROcp31_829+ROcp31_521*s.dpt(2,51);
    RLcp31_330 = s.dpt(1,51)*ROcp31_329+s.dpt(3,51)*ROcp31_929+ROcp31_621*s.dpt(2,51);
    OMcp31_130 = OMcp31_129+ROcp31_129*qd(30);
    OMcp31_230 = OMcp31_229+ROcp31_229*qd(30);
    OMcp31_330 = OMcp31_329+ROcp31_329*qd(30);
    ORcp31_130 = OMcp31_229*RLcp31_330-OMcp31_329*RLcp31_230;
    ORcp31_230 = -(OMcp31_129*RLcp31_330-OMcp31_329*RLcp31_130);
    ORcp31_330 = OMcp31_129*RLcp31_230-OMcp31_229*RLcp31_130;
    OPcp31_130 = OPcp31_129+ROcp31_129*qdd(30)+qd(30)*(OMcp31_229*ROcp31_329-OMcp31_329*ROcp31_229);
    OPcp31_230 = OPcp31_229+ROcp31_229*qdd(30)-qd(30)*(OMcp31_129*ROcp31_329-OMcp31_329*ROcp31_129);
    OPcp31_330 = OPcp31_329+ROcp31_329*qdd(30)+qd(30)*(OMcp31_129*ROcp31_229-OMcp31_229*ROcp31_129);
    RLcp31_131 = s.dpt(1,53)*ROcp31_129+s.dpt(3,53)*ROcp31_730+ROcp31_430*s.dpt(2,53);
    RLcp31_231 = s.dpt(1,53)*ROcp31_229+s.dpt(3,53)*ROcp31_830+ROcp31_530*s.dpt(2,53);
    RLcp31_331 = s.dpt(1,53)*ROcp31_329+s.dpt(3,53)*ROcp31_930+ROcp31_630*s.dpt(2,53);
    OMcp31_131 = OMcp31_130+ROcp31_430*qd(31);
    OMcp31_231 = OMcp31_230+ROcp31_530*qd(31);
    OMcp31_331 = OMcp31_330+ROcp31_630*qd(31);
    ORcp31_131 = OMcp31_230*RLcp31_331-OMcp31_330*RLcp31_231;
    ORcp31_231 = -(OMcp31_130*RLcp31_331-OMcp31_330*RLcp31_131);
    ORcp31_331 = OMcp31_130*RLcp31_231-OMcp31_230*RLcp31_131;
    OPcp31_131 = OPcp31_130+ROcp31_430*qdd(31)+qd(31)*(OMcp31_230*ROcp31_630-OMcp31_330*ROcp31_530);
    OPcp31_231 = OPcp31_230+ROcp31_530*qdd(31)-qd(31)*(OMcp31_130*ROcp31_630-OMcp31_330*ROcp31_430);
    OPcp31_331 = OPcp31_330+ROcp31_630*qdd(31)+qd(31)*(OMcp31_130*ROcp31_530-OMcp31_230*ROcp31_430);
    RLcp31_132 = s.dpt(3,55)*ROcp31_731+ROcp31_131*s.dpt(1,55)+ROcp31_430*s.dpt(2,55);
    RLcp31_232 = s.dpt(3,55)*ROcp31_831+ROcp31_231*s.dpt(1,55)+ROcp31_530*s.dpt(2,55);
    RLcp31_332 = s.dpt(3,55)*ROcp31_931+ROcp31_331*s.dpt(1,55)+ROcp31_630*s.dpt(2,55);
    POcp31_132 = RLcp31_119+RLcp31_120+RLcp31_121+RLcp31_129+RLcp31_130+RLcp31_131+RLcp31_132+q(1);
    POcp31_232 = RLcp31_219+RLcp31_220+RLcp31_221+RLcp31_229+RLcp31_230+RLcp31_231+RLcp31_232+q(2);
    POcp31_332 = RLcp31_319+RLcp31_320+RLcp31_321+RLcp31_329+RLcp31_330+RLcp31_331+RLcp31_332+q(3);
    OMcp31_132 = OMcp31_131+ROcp31_731*qd(32);
    OMcp31_232 = OMcp31_231+ROcp31_831*qd(32);
    OMcp31_332 = OMcp31_331+ROcp31_931*qd(32);
    ORcp31_132 = OMcp31_231*RLcp31_332-OMcp31_331*RLcp31_232;
    ORcp31_232 = -(OMcp31_131*RLcp31_332-OMcp31_331*RLcp31_132);
    ORcp31_332 = OMcp31_131*RLcp31_232-OMcp31_231*RLcp31_132;
    VIcp31_132 = ORcp31_119+ORcp31_120+ORcp31_121+ORcp31_129+ORcp31_130+ORcp31_131+ORcp31_132+qd(1);
    VIcp31_232 = ORcp31_219+ORcp31_220+ORcp31_221+ORcp31_229+ORcp31_230+ORcp31_231+ORcp31_232+qd(2);
    VIcp31_332 = ORcp31_319+ORcp31_320+ORcp31_321+ORcp31_329+ORcp31_330+ORcp31_331+ORcp31_332+qd(3);
    OPcp31_132 = OPcp31_131+ROcp31_731*qdd(32)+qd(32)*(OMcp31_231*ROcp31_931-OMcp31_331*ROcp31_831);
    OPcp31_232 = OPcp31_231+ROcp31_831*qdd(32)-qd(32)*(OMcp31_131*ROcp31_931-OMcp31_331*ROcp31_731);
    OPcp31_332 = OPcp31_331+ROcp31_931*qdd(32)+qd(32)*(OMcp31_131*ROcp31_831-OMcp31_231*ROcp31_731);
    ACcp31_132 = qdd(1)+OMcp31_219*ORcp31_320+OMcp31_220*ORcp31_321+OMcp31_221*ORcp31_329+OMcp31_229*ORcp31_330+OMcp31_230*ORcp31_331+OMcp31_231*...
 ORcp31_332+OMcp31_26*ORcp31_319-OMcp31_319*ORcp31_220-OMcp31_320*ORcp31_221-OMcp31_321*ORcp31_229-OMcp31_329*ORcp31_230-OMcp31_330*ORcp31_231-...
 OMcp31_331*ORcp31_232-OMcp31_36*ORcp31_219+OPcp31_219*RLcp31_320+OPcp31_220*RLcp31_321+OPcp31_221*RLcp31_329+OPcp31_229*RLcp31_330+OPcp31_230*...
 RLcp31_331+OPcp31_231*RLcp31_332+OPcp31_26*RLcp31_319-OPcp31_319*RLcp31_220-OPcp31_320*RLcp31_221-OPcp31_321*RLcp31_229-OPcp31_329*RLcp31_230-...
 OPcp31_330*RLcp31_231-OPcp31_331*RLcp31_232-OPcp31_36*RLcp31_219;
    ACcp31_232 = qdd(2)-OMcp31_119*ORcp31_320-OMcp31_120*ORcp31_321-OMcp31_121*ORcp31_329-OMcp31_129*ORcp31_330-OMcp31_130*ORcp31_331-OMcp31_131*...
 ORcp31_332-OMcp31_16*ORcp31_319+OMcp31_319*ORcp31_120+OMcp31_320*ORcp31_121+OMcp31_321*ORcp31_129+OMcp31_329*ORcp31_130+OMcp31_330*ORcp31_131+...
 OMcp31_331*ORcp31_132+OMcp31_36*ORcp31_119-OPcp31_119*RLcp31_320-OPcp31_120*RLcp31_321-OPcp31_121*RLcp31_329-OPcp31_129*RLcp31_330-OPcp31_130*...
 RLcp31_331-OPcp31_131*RLcp31_332-OPcp31_16*RLcp31_319+OPcp31_319*RLcp31_120+OPcp31_320*RLcp31_121+OPcp31_321*RLcp31_129+OPcp31_329*RLcp31_130+...
 OPcp31_330*RLcp31_131+OPcp31_331*RLcp31_132+OPcp31_36*RLcp31_119;
    ACcp31_332 = qdd(3)+OMcp31_119*ORcp31_220+OMcp31_120*ORcp31_221+OMcp31_121*ORcp31_229+OMcp31_129*ORcp31_230+OMcp31_130*ORcp31_231+OMcp31_131*...
 ORcp31_232+OMcp31_16*ORcp31_219-OMcp31_219*ORcp31_120-OMcp31_220*ORcp31_121-OMcp31_221*ORcp31_129-OMcp31_229*ORcp31_130-OMcp31_230*ORcp31_131-...
 OMcp31_231*ORcp31_132-OMcp31_26*ORcp31_119+OPcp31_119*RLcp31_220+OPcp31_120*RLcp31_221+OPcp31_121*RLcp31_229+OPcp31_129*RLcp31_230+OPcp31_130*...
 RLcp31_231+OPcp31_131*RLcp31_232+OPcp31_16*RLcp31_219-OPcp31_219*RLcp31_120-OPcp31_220*RLcp31_121-OPcp31_221*RLcp31_129-OPcp31_229*RLcp31_130-...
 OPcp31_230*RLcp31_131-OPcp31_231*RLcp31_132-OPcp31_26*RLcp31_119;

% = = Block_1_0_0_32_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp31_132;
    sens.P(2) = POcp31_232;
    sens.P(3) = POcp31_332;
    sens.R(1,1) = ROcp31_132;
    sens.R(1,2) = ROcp31_232;
    sens.R(1,3) = ROcp31_332;
    sens.R(2,1) = ROcp31_432;
    sens.R(2,2) = ROcp31_532;
    sens.R(2,3) = ROcp31_632;
    sens.R(3,1) = ROcp31_731;
    sens.R(3,2) = ROcp31_831;
    sens.R(3,3) = ROcp31_931;
    sens.V(1) = VIcp31_132;
    sens.V(2) = VIcp31_232;
    sens.V(3) = VIcp31_332;
    sens.OM(1) = OMcp31_132;
    sens.OM(2) = OMcp31_232;
    sens.OM(3) = OMcp31_332;
    sens.A(1) = ACcp31_132;
    sens.A(2) = ACcp31_232;
    sens.A(3) = ACcp31_332;
    sens.OMP(1) = OPcp31_132;
    sens.OMP(2) = OPcp31_232;
    sens.OMP(3) = OPcp31_332;
 
% 
case 33, 


% = = Block_1_0_0_33_0_1 = = 
 
% Sensor Kinematics 


    ROcp32_25 = S4*S5;
    ROcp32_35 = -C4*S5;
    ROcp32_85 = -S4*C5;
    ROcp32_95 = C4*C5;
    ROcp32_16 = C5*C6;
    ROcp32_26 = ROcp32_25*C6+C4*S6;
    ROcp32_36 = ROcp32_35*C6+S4*S6;
    ROcp32_46 = -C5*S6;
    ROcp32_56 = -(ROcp32_25*S6-C4*C6);
    ROcp32_66 = -(ROcp32_35*S6-S4*C6);
    OMcp32_25 = qd(5)*C4;
    OMcp32_35 = qd(5)*S4;
    OMcp32_16 = qd(4)+qd(6)*S5;
    OMcp32_26 = OMcp32_25+ROcp32_85*qd(6);
    OMcp32_36 = OMcp32_35+ROcp32_95*qd(6);
    OPcp32_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp32_26 = ROcp32_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp32_35*S5-ROcp32_95*qd(4));
    OPcp32_36 = ROcp32_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp32_25*S5-ROcp32_85*qd(4));

% = = Block_1_0_0_33_0_4 = = 
 
% Sensor Kinematics 


    ROcp32_419 = ROcp32_46*C19+S19*S5;
    ROcp32_519 = ROcp32_56*C19+ROcp32_85*S19;
    ROcp32_619 = ROcp32_66*C19+ROcp32_95*S19;
    ROcp32_719 = -(ROcp32_46*S19-C19*S5);
    ROcp32_819 = -(ROcp32_56*S19-ROcp32_85*C19);
    ROcp32_919 = -(ROcp32_66*S19-ROcp32_95*C19);
    ROcp32_120 = ROcp32_16*C20-ROcp32_719*S20;
    ROcp32_220 = ROcp32_26*C20-ROcp32_819*S20;
    ROcp32_320 = ROcp32_36*C20-ROcp32_919*S20;
    ROcp32_720 = ROcp32_16*S20+ROcp32_719*C20;
    ROcp32_820 = ROcp32_26*S20+ROcp32_819*C20;
    ROcp32_920 = ROcp32_36*S20+ROcp32_919*C20;
    ROcp32_121 = ROcp32_120*C21+ROcp32_419*S21;
    ROcp32_221 = ROcp32_220*C21+ROcp32_519*S21;
    ROcp32_321 = ROcp32_320*C21+ROcp32_619*S21;
    ROcp32_421 = -(ROcp32_120*S21-ROcp32_419*C21);
    ROcp32_521 = -(ROcp32_220*S21-ROcp32_519*C21);
    ROcp32_621 = -(ROcp32_320*S21-ROcp32_619*C21);
    RLcp32_119 = s.dpt(2,3)*ROcp32_46+ROcp32_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp32_219 = s.dpt(2,3)*ROcp32_56+ROcp32_26*s.dpt(1,3)+ROcp32_85*s.dpt(3,3);
    RLcp32_319 = s.dpt(2,3)*ROcp32_66+ROcp32_36*s.dpt(1,3)+ROcp32_95*s.dpt(3,3);
    OMcp32_119 = OMcp32_16+ROcp32_16*qd(19);
    OMcp32_219 = OMcp32_26+ROcp32_26*qd(19);
    OMcp32_319 = OMcp32_36+ROcp32_36*qd(19);
    ORcp32_119 = OMcp32_26*RLcp32_319-OMcp32_36*RLcp32_219;
    ORcp32_219 = -(OMcp32_16*RLcp32_319-OMcp32_36*RLcp32_119);
    ORcp32_319 = OMcp32_16*RLcp32_219-OMcp32_26*RLcp32_119;
    OPcp32_119 = OPcp32_16+ROcp32_16*qdd(19)+qd(19)*(OMcp32_26*ROcp32_36-OMcp32_36*ROcp32_26);
    OPcp32_219 = OPcp32_26+ROcp32_26*qdd(19)-qd(19)*(OMcp32_16*ROcp32_36-OMcp32_36*ROcp32_16);
    OPcp32_319 = OPcp32_36+ROcp32_36*qdd(19)+qd(19)*(OMcp32_16*ROcp32_26-OMcp32_26*ROcp32_16);
    RLcp32_120 = s.dpt(1,30)*ROcp32_16+s.dpt(2,30)*ROcp32_419+s.dpt(3,30)*ROcp32_719;
    RLcp32_220 = s.dpt(1,30)*ROcp32_26+s.dpt(2,30)*ROcp32_519+s.dpt(3,30)*ROcp32_819;
    RLcp32_320 = s.dpt(1,30)*ROcp32_36+s.dpt(2,30)*ROcp32_619+s.dpt(3,30)*ROcp32_919;
    OMcp32_120 = OMcp32_119+ROcp32_419*qd(20);
    OMcp32_220 = OMcp32_219+ROcp32_519*qd(20);
    OMcp32_320 = OMcp32_319+ROcp32_619*qd(20);
    ORcp32_120 = OMcp32_219*RLcp32_320-OMcp32_319*RLcp32_220;
    ORcp32_220 = -(OMcp32_119*RLcp32_320-OMcp32_319*RLcp32_120);
    ORcp32_320 = OMcp32_119*RLcp32_220-OMcp32_219*RLcp32_120;
    OPcp32_120 = OPcp32_119+ROcp32_419*qdd(20)+qd(20)*(OMcp32_219*ROcp32_619-OMcp32_319*ROcp32_519);
    OPcp32_220 = OPcp32_219+ROcp32_519*qdd(20)-qd(20)*(OMcp32_119*ROcp32_619-OMcp32_319*ROcp32_419);
    OPcp32_320 = OPcp32_319+ROcp32_619*qdd(20)+qd(20)*(OMcp32_119*ROcp32_519-OMcp32_219*ROcp32_419);
    RLcp32_121 = s.dpt(1,32)*ROcp32_120+s.dpt(2,32)*ROcp32_419+ROcp32_720*s.dpt(3,32);
    RLcp32_221 = s.dpt(1,32)*ROcp32_220+s.dpt(2,32)*ROcp32_519+ROcp32_820*s.dpt(3,32);
    RLcp32_321 = s.dpt(1,32)*ROcp32_320+s.dpt(2,32)*ROcp32_619+ROcp32_920*s.dpt(3,32);
    OMcp32_121 = OMcp32_120+ROcp32_720*qd(21);
    OMcp32_221 = OMcp32_220+ROcp32_820*qd(21);
    OMcp32_321 = OMcp32_320+ROcp32_920*qd(21);
    ORcp32_121 = OMcp32_220*RLcp32_321-OMcp32_320*RLcp32_221;
    ORcp32_221 = -(OMcp32_120*RLcp32_321-OMcp32_320*RLcp32_121);
    ORcp32_321 = OMcp32_120*RLcp32_221-OMcp32_220*RLcp32_121;
    OPcp32_121 = OPcp32_120+ROcp32_720*qdd(21)+qd(21)*(OMcp32_220*ROcp32_920-OMcp32_320*ROcp32_820);
    OPcp32_221 = OPcp32_220+ROcp32_820*qdd(21)-qd(21)*(OMcp32_120*ROcp32_920-OMcp32_320*ROcp32_720);
    OPcp32_321 = OPcp32_320+ROcp32_920*qdd(21)+qd(21)*(OMcp32_120*ROcp32_820-OMcp32_220*ROcp32_720);

% = = Block_1_0_0_33_0_6 = = 
 
% Sensor Kinematics 


    ROcp32_129 = ROcp32_121*C29-ROcp32_720*S29;
    ROcp32_229 = ROcp32_221*C29-ROcp32_820*S29;
    ROcp32_329 = ROcp32_321*C29-ROcp32_920*S29;
    ROcp32_729 = ROcp32_121*S29+ROcp32_720*C29;
    ROcp32_829 = ROcp32_221*S29+ROcp32_820*C29;
    ROcp32_929 = ROcp32_321*S29+ROcp32_920*C29;
    ROcp32_430 = ROcp32_421*C30+ROcp32_729*S30;
    ROcp32_530 = ROcp32_521*C30+ROcp32_829*S30;
    ROcp32_630 = ROcp32_621*C30+ROcp32_929*S30;
    ROcp32_730 = -(ROcp32_421*S30-ROcp32_729*C30);
    ROcp32_830 = -(ROcp32_521*S30-ROcp32_829*C30);
    ROcp32_930 = -(ROcp32_621*S30-ROcp32_929*C30);
    ROcp32_131 = ROcp32_129*C31-ROcp32_730*S31;
    ROcp32_231 = ROcp32_229*C31-ROcp32_830*S31;
    ROcp32_331 = ROcp32_329*C31-ROcp32_930*S31;
    ROcp32_731 = ROcp32_129*S31+ROcp32_730*C31;
    ROcp32_831 = ROcp32_229*S31+ROcp32_830*C31;
    ROcp32_931 = ROcp32_329*S31+ROcp32_930*C31;
    ROcp32_132 = ROcp32_131*C32+ROcp32_430*S32;
    ROcp32_232 = ROcp32_231*C32+ROcp32_530*S32;
    ROcp32_332 = ROcp32_331*C32+ROcp32_630*S32;
    ROcp32_432 = -(ROcp32_131*S32-ROcp32_430*C32);
    ROcp32_532 = -(ROcp32_231*S32-ROcp32_530*C32);
    ROcp32_632 = -(ROcp32_331*S32-ROcp32_630*C32);
    ROcp32_133 = ROcp32_132*C33-ROcp32_731*S33;
    ROcp32_233 = ROcp32_232*C33-ROcp32_831*S33;
    ROcp32_333 = ROcp32_332*C33-ROcp32_931*S33;
    ROcp32_733 = ROcp32_132*S33+ROcp32_731*C33;
    ROcp32_833 = ROcp32_232*S33+ROcp32_831*C33;
    ROcp32_933 = ROcp32_332*S33+ROcp32_931*C33;
    RLcp32_129 = ROcp32_121*s.dpt(1,37)+ROcp32_421*s.dpt(2,37)+ROcp32_720*s.dpt(3,37);
    RLcp32_229 = ROcp32_221*s.dpt(1,37)+ROcp32_521*s.dpt(2,37)+ROcp32_820*s.dpt(3,37);
    RLcp32_329 = ROcp32_321*s.dpt(1,37)+ROcp32_621*s.dpt(2,37)+ROcp32_920*s.dpt(3,37);
    OMcp32_129 = OMcp32_121+ROcp32_421*qd(29);
    OMcp32_229 = OMcp32_221+ROcp32_521*qd(29);
    OMcp32_329 = OMcp32_321+ROcp32_621*qd(29);
    ORcp32_129 = OMcp32_221*RLcp32_329-OMcp32_321*RLcp32_229;
    ORcp32_229 = -(OMcp32_121*RLcp32_329-OMcp32_321*RLcp32_129);
    ORcp32_329 = OMcp32_121*RLcp32_229-OMcp32_221*RLcp32_129;
    OPcp32_129 = OPcp32_121+ROcp32_421*qdd(29)+qd(29)*(OMcp32_221*ROcp32_621-OMcp32_321*ROcp32_521);
    OPcp32_229 = OPcp32_221+ROcp32_521*qdd(29)-qd(29)*(OMcp32_121*ROcp32_621-OMcp32_321*ROcp32_421);
    OPcp32_329 = OPcp32_321+ROcp32_621*qdd(29)+qd(29)*(OMcp32_121*ROcp32_521-OMcp32_221*ROcp32_421);
    RLcp32_130 = s.dpt(1,51)*ROcp32_129+s.dpt(3,51)*ROcp32_729+ROcp32_421*s.dpt(2,51);
    RLcp32_230 = s.dpt(1,51)*ROcp32_229+s.dpt(3,51)*ROcp32_829+ROcp32_521*s.dpt(2,51);
    RLcp32_330 = s.dpt(1,51)*ROcp32_329+s.dpt(3,51)*ROcp32_929+ROcp32_621*s.dpt(2,51);
    OMcp32_130 = OMcp32_129+ROcp32_129*qd(30);
    OMcp32_230 = OMcp32_229+ROcp32_229*qd(30);
    OMcp32_330 = OMcp32_329+ROcp32_329*qd(30);
    ORcp32_130 = OMcp32_229*RLcp32_330-OMcp32_329*RLcp32_230;
    ORcp32_230 = -(OMcp32_129*RLcp32_330-OMcp32_329*RLcp32_130);
    ORcp32_330 = OMcp32_129*RLcp32_230-OMcp32_229*RLcp32_130;
    OPcp32_130 = OPcp32_129+ROcp32_129*qdd(30)+qd(30)*(OMcp32_229*ROcp32_329-OMcp32_329*ROcp32_229);
    OPcp32_230 = OPcp32_229+ROcp32_229*qdd(30)-qd(30)*(OMcp32_129*ROcp32_329-OMcp32_329*ROcp32_129);
    OPcp32_330 = OPcp32_329+ROcp32_329*qdd(30)+qd(30)*(OMcp32_129*ROcp32_229-OMcp32_229*ROcp32_129);
    RLcp32_131 = s.dpt(1,53)*ROcp32_129+s.dpt(3,53)*ROcp32_730+ROcp32_430*s.dpt(2,53);
    RLcp32_231 = s.dpt(1,53)*ROcp32_229+s.dpt(3,53)*ROcp32_830+ROcp32_530*s.dpt(2,53);
    RLcp32_331 = s.dpt(1,53)*ROcp32_329+s.dpt(3,53)*ROcp32_930+ROcp32_630*s.dpt(2,53);
    OMcp32_131 = OMcp32_130+ROcp32_430*qd(31);
    OMcp32_231 = OMcp32_230+ROcp32_530*qd(31);
    OMcp32_331 = OMcp32_330+ROcp32_630*qd(31);
    ORcp32_131 = OMcp32_230*RLcp32_331-OMcp32_330*RLcp32_231;
    ORcp32_231 = -(OMcp32_130*RLcp32_331-OMcp32_330*RLcp32_131);
    ORcp32_331 = OMcp32_130*RLcp32_231-OMcp32_230*RLcp32_131;
    OPcp32_131 = OPcp32_130+ROcp32_430*qdd(31)+qd(31)*(OMcp32_230*ROcp32_630-OMcp32_330*ROcp32_530);
    OPcp32_231 = OPcp32_230+ROcp32_530*qdd(31)-qd(31)*(OMcp32_130*ROcp32_630-OMcp32_330*ROcp32_430);
    OPcp32_331 = OPcp32_330+ROcp32_630*qdd(31)+qd(31)*(OMcp32_130*ROcp32_530-OMcp32_230*ROcp32_430);
    RLcp32_132 = s.dpt(3,55)*ROcp32_731+ROcp32_131*s.dpt(1,55)+ROcp32_430*s.dpt(2,55);
    RLcp32_232 = s.dpt(3,55)*ROcp32_831+ROcp32_231*s.dpt(1,55)+ROcp32_530*s.dpt(2,55);
    RLcp32_332 = s.dpt(3,55)*ROcp32_931+ROcp32_331*s.dpt(1,55)+ROcp32_630*s.dpt(2,55);
    OMcp32_132 = OMcp32_131+ROcp32_731*qd(32);
    OMcp32_232 = OMcp32_231+ROcp32_831*qd(32);
    OMcp32_332 = OMcp32_331+ROcp32_931*qd(32);
    ORcp32_132 = OMcp32_231*RLcp32_332-OMcp32_331*RLcp32_232;
    ORcp32_232 = -(OMcp32_131*RLcp32_332-OMcp32_331*RLcp32_132);
    ORcp32_332 = OMcp32_131*RLcp32_232-OMcp32_231*RLcp32_132;
    OPcp32_132 = OPcp32_131+ROcp32_731*qdd(32)+qd(32)*(OMcp32_231*ROcp32_931-OMcp32_331*ROcp32_831);
    OPcp32_232 = OPcp32_231+ROcp32_831*qdd(32)-qd(32)*(OMcp32_131*ROcp32_931-OMcp32_331*ROcp32_731);
    OPcp32_332 = OPcp32_331+ROcp32_931*qdd(32)+qd(32)*(OMcp32_131*ROcp32_831-OMcp32_231*ROcp32_731);
    RLcp32_133 = s.dpt(2,57)*ROcp32_432+s.dpt(3,57)*ROcp32_731+ROcp32_132*s.dpt(1,57);
    RLcp32_233 = s.dpt(2,57)*ROcp32_532+s.dpt(3,57)*ROcp32_831+ROcp32_232*s.dpt(1,57);
    RLcp32_333 = s.dpt(2,57)*ROcp32_632+s.dpt(3,57)*ROcp32_931+ROcp32_332*s.dpt(1,57);
    POcp32_133 = RLcp32_119+RLcp32_120+RLcp32_121+RLcp32_129+RLcp32_130+RLcp32_131+RLcp32_132+RLcp32_133+q(1);
    POcp32_233 = RLcp32_219+RLcp32_220+RLcp32_221+RLcp32_229+RLcp32_230+RLcp32_231+RLcp32_232+RLcp32_233+q(2);
    POcp32_333 = RLcp32_319+RLcp32_320+RLcp32_321+RLcp32_329+RLcp32_330+RLcp32_331+RLcp32_332+RLcp32_333+q(3);
    OMcp32_133 = OMcp32_132+ROcp32_432*qd(33);
    OMcp32_233 = OMcp32_232+ROcp32_532*qd(33);
    OMcp32_333 = OMcp32_332+ROcp32_632*qd(33);
    ORcp32_133 = OMcp32_232*RLcp32_333-OMcp32_332*RLcp32_233;
    ORcp32_233 = -(OMcp32_132*RLcp32_333-OMcp32_332*RLcp32_133);
    ORcp32_333 = OMcp32_132*RLcp32_233-OMcp32_232*RLcp32_133;
    VIcp32_133 = ORcp32_119+ORcp32_120+ORcp32_121+ORcp32_129+ORcp32_130+ORcp32_131+ORcp32_132+ORcp32_133+qd(1);
    VIcp32_233 = ORcp32_219+ORcp32_220+ORcp32_221+ORcp32_229+ORcp32_230+ORcp32_231+ORcp32_232+ORcp32_233+qd(2);
    VIcp32_333 = ORcp32_319+ORcp32_320+ORcp32_321+ORcp32_329+ORcp32_330+ORcp32_331+ORcp32_332+ORcp32_333+qd(3);
    OPcp32_133 = OPcp32_132+ROcp32_432*qdd(33)+qd(33)*(OMcp32_232*ROcp32_632-OMcp32_332*ROcp32_532);
    OPcp32_233 = OPcp32_232+ROcp32_532*qdd(33)-qd(33)*(OMcp32_132*ROcp32_632-OMcp32_332*ROcp32_432);
    OPcp32_333 = OPcp32_332+ROcp32_632*qdd(33)+qd(33)*(OMcp32_132*ROcp32_532-OMcp32_232*ROcp32_432);
    ACcp32_133 = qdd(1)+OMcp32_219*ORcp32_320+OMcp32_220*ORcp32_321+OMcp32_221*ORcp32_329+OMcp32_229*ORcp32_330+OMcp32_230*ORcp32_331+OMcp32_231*...
 ORcp32_332+OMcp32_232*ORcp32_333+OMcp32_26*ORcp32_319-OMcp32_319*ORcp32_220-OMcp32_320*ORcp32_221-OMcp32_321*ORcp32_229-OMcp32_329*ORcp32_230-...
 OMcp32_330*ORcp32_231-OMcp32_331*ORcp32_232-OMcp32_332*ORcp32_233-OMcp32_36*ORcp32_219+OPcp32_219*RLcp32_320+OPcp32_220*RLcp32_321+OPcp32_221*...
 RLcp32_329+OPcp32_229*RLcp32_330+OPcp32_230*RLcp32_331+OPcp32_231*RLcp32_332+OPcp32_232*RLcp32_333+OPcp32_26*RLcp32_319-OPcp32_319*RLcp32_220-...
 OPcp32_320*RLcp32_221-OPcp32_321*RLcp32_229-OPcp32_329*RLcp32_230-OPcp32_330*RLcp32_231-OPcp32_331*RLcp32_232-OPcp32_332*RLcp32_233-OPcp32_36*...
 RLcp32_219;
    ACcp32_233 = qdd(2)-OMcp32_119*ORcp32_320-OMcp32_120*ORcp32_321-OMcp32_121*ORcp32_329-OMcp32_129*ORcp32_330-OMcp32_130*ORcp32_331-OMcp32_131*...
 ORcp32_332-OMcp32_132*ORcp32_333-OMcp32_16*ORcp32_319+OMcp32_319*ORcp32_120+OMcp32_320*ORcp32_121+OMcp32_321*ORcp32_129+OMcp32_329*ORcp32_130+...
 OMcp32_330*ORcp32_131+OMcp32_331*ORcp32_132+OMcp32_332*ORcp32_133+OMcp32_36*ORcp32_119-OPcp32_119*RLcp32_320-OPcp32_120*RLcp32_321-OPcp32_121*...
 RLcp32_329-OPcp32_129*RLcp32_330-OPcp32_130*RLcp32_331-OPcp32_131*RLcp32_332-OPcp32_132*RLcp32_333-OPcp32_16*RLcp32_319+OPcp32_319*RLcp32_120+...
 OPcp32_320*RLcp32_121+OPcp32_321*RLcp32_129+OPcp32_329*RLcp32_130+OPcp32_330*RLcp32_131+OPcp32_331*RLcp32_132+OPcp32_332*RLcp32_133+OPcp32_36*...
 RLcp32_119;
    ACcp32_333 = qdd(3)+OMcp32_119*ORcp32_220+OMcp32_120*ORcp32_221+OMcp32_121*ORcp32_229+OMcp32_129*ORcp32_230+OMcp32_130*ORcp32_231+OMcp32_131*...
 ORcp32_232+OMcp32_132*ORcp32_233+OMcp32_16*ORcp32_219-OMcp32_219*ORcp32_120-OMcp32_220*ORcp32_121-OMcp32_221*ORcp32_129-OMcp32_229*ORcp32_130-...
 OMcp32_230*ORcp32_131-OMcp32_231*ORcp32_132-OMcp32_232*ORcp32_133-OMcp32_26*ORcp32_119+OPcp32_119*RLcp32_220+OPcp32_120*RLcp32_221+OPcp32_121*...
 RLcp32_229+OPcp32_129*RLcp32_230+OPcp32_130*RLcp32_231+OPcp32_131*RLcp32_232+OPcp32_132*RLcp32_233+OPcp32_16*RLcp32_219-OPcp32_219*RLcp32_120-...
 OPcp32_220*RLcp32_121-OPcp32_221*RLcp32_129-OPcp32_229*RLcp32_130-OPcp32_230*RLcp32_131-OPcp32_231*RLcp32_132-OPcp32_232*RLcp32_133-OPcp32_26*...
 RLcp32_119;

% = = Block_1_0_0_33_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp32_133;
    sens.P(2) = POcp32_233;
    sens.P(3) = POcp32_333;
    sens.R(1,1) = ROcp32_133;
    sens.R(1,2) = ROcp32_233;
    sens.R(1,3) = ROcp32_333;
    sens.R(2,1) = ROcp32_432;
    sens.R(2,2) = ROcp32_532;
    sens.R(2,3) = ROcp32_632;
    sens.R(3,1) = ROcp32_733;
    sens.R(3,2) = ROcp32_833;
    sens.R(3,3) = ROcp32_933;
    sens.V(1) = VIcp32_133;
    sens.V(2) = VIcp32_233;
    sens.V(3) = VIcp32_333;
    sens.OM(1) = OMcp32_133;
    sens.OM(2) = OMcp32_233;
    sens.OM(3) = OMcp32_333;
    sens.A(1) = ACcp32_133;
    sens.A(2) = ACcp32_233;
    sens.A(3) = ACcp32_333;
    sens.OMP(1) = OPcp32_133;
    sens.OMP(2) = OPcp32_233;
    sens.OMP(3) = OPcp32_333;
 
% 
case 34, 


% = = Block_1_0_0_34_0_1 = = 
 
% Sensor Kinematics 


    ROcp33_25 = S4*S5;
    ROcp33_35 = -C4*S5;
    ROcp33_85 = -S4*C5;
    ROcp33_95 = C4*C5;
    ROcp33_16 = C5*C6;
    ROcp33_26 = ROcp33_25*C6+C4*S6;
    ROcp33_36 = ROcp33_35*C6+S4*S6;
    ROcp33_46 = -C5*S6;
    ROcp33_56 = -(ROcp33_25*S6-C4*C6);
    ROcp33_66 = -(ROcp33_35*S6-S4*C6);
    OMcp33_25 = qd(5)*C4;
    OMcp33_35 = qd(5)*S4;
    OMcp33_16 = qd(4)+qd(6)*S5;
    OMcp33_26 = OMcp33_25+ROcp33_85*qd(6);
    OMcp33_36 = OMcp33_35+ROcp33_95*qd(6);
    OPcp33_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp33_26 = ROcp33_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp33_35*S5-ROcp33_95*qd(4));
    OPcp33_36 = ROcp33_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp33_25*S5-ROcp33_85*qd(4));

% = = Block_1_0_0_34_0_4 = = 
 
% Sensor Kinematics 


    ROcp33_419 = ROcp33_46*C19+S19*S5;
    ROcp33_519 = ROcp33_56*C19+ROcp33_85*S19;
    ROcp33_619 = ROcp33_66*C19+ROcp33_95*S19;
    ROcp33_719 = -(ROcp33_46*S19-C19*S5);
    ROcp33_819 = -(ROcp33_56*S19-ROcp33_85*C19);
    ROcp33_919 = -(ROcp33_66*S19-ROcp33_95*C19);
    ROcp33_120 = ROcp33_16*C20-ROcp33_719*S20;
    ROcp33_220 = ROcp33_26*C20-ROcp33_819*S20;
    ROcp33_320 = ROcp33_36*C20-ROcp33_919*S20;
    ROcp33_720 = ROcp33_16*S20+ROcp33_719*C20;
    ROcp33_820 = ROcp33_26*S20+ROcp33_819*C20;
    ROcp33_920 = ROcp33_36*S20+ROcp33_919*C20;
    ROcp33_121 = ROcp33_120*C21+ROcp33_419*S21;
    ROcp33_221 = ROcp33_220*C21+ROcp33_519*S21;
    ROcp33_321 = ROcp33_320*C21+ROcp33_619*S21;
    ROcp33_421 = -(ROcp33_120*S21-ROcp33_419*C21);
    ROcp33_521 = -(ROcp33_220*S21-ROcp33_519*C21);
    ROcp33_621 = -(ROcp33_320*S21-ROcp33_619*C21);
    RLcp33_119 = s.dpt(2,3)*ROcp33_46+ROcp33_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp33_219 = s.dpt(2,3)*ROcp33_56+ROcp33_26*s.dpt(1,3)+ROcp33_85*s.dpt(3,3);
    RLcp33_319 = s.dpt(2,3)*ROcp33_66+ROcp33_36*s.dpt(1,3)+ROcp33_95*s.dpt(3,3);
    OMcp33_119 = OMcp33_16+ROcp33_16*qd(19);
    OMcp33_219 = OMcp33_26+ROcp33_26*qd(19);
    OMcp33_319 = OMcp33_36+ROcp33_36*qd(19);
    ORcp33_119 = OMcp33_26*RLcp33_319-OMcp33_36*RLcp33_219;
    ORcp33_219 = -(OMcp33_16*RLcp33_319-OMcp33_36*RLcp33_119);
    ORcp33_319 = OMcp33_16*RLcp33_219-OMcp33_26*RLcp33_119;
    OPcp33_119 = OPcp33_16+ROcp33_16*qdd(19)+qd(19)*(OMcp33_26*ROcp33_36-OMcp33_36*ROcp33_26);
    OPcp33_219 = OPcp33_26+ROcp33_26*qdd(19)-qd(19)*(OMcp33_16*ROcp33_36-OMcp33_36*ROcp33_16);
    OPcp33_319 = OPcp33_36+ROcp33_36*qdd(19)+qd(19)*(OMcp33_16*ROcp33_26-OMcp33_26*ROcp33_16);
    RLcp33_120 = s.dpt(1,30)*ROcp33_16+s.dpt(2,30)*ROcp33_419+s.dpt(3,30)*ROcp33_719;
    RLcp33_220 = s.dpt(1,30)*ROcp33_26+s.dpt(2,30)*ROcp33_519+s.dpt(3,30)*ROcp33_819;
    RLcp33_320 = s.dpt(1,30)*ROcp33_36+s.dpt(2,30)*ROcp33_619+s.dpt(3,30)*ROcp33_919;
    OMcp33_120 = OMcp33_119+ROcp33_419*qd(20);
    OMcp33_220 = OMcp33_219+ROcp33_519*qd(20);
    OMcp33_320 = OMcp33_319+ROcp33_619*qd(20);
    ORcp33_120 = OMcp33_219*RLcp33_320-OMcp33_319*RLcp33_220;
    ORcp33_220 = -(OMcp33_119*RLcp33_320-OMcp33_319*RLcp33_120);
    ORcp33_320 = OMcp33_119*RLcp33_220-OMcp33_219*RLcp33_120;
    OPcp33_120 = OPcp33_119+ROcp33_419*qdd(20)+qd(20)*(OMcp33_219*ROcp33_619-OMcp33_319*ROcp33_519);
    OPcp33_220 = OPcp33_219+ROcp33_519*qdd(20)-qd(20)*(OMcp33_119*ROcp33_619-OMcp33_319*ROcp33_419);
    OPcp33_320 = OPcp33_319+ROcp33_619*qdd(20)+qd(20)*(OMcp33_119*ROcp33_519-OMcp33_219*ROcp33_419);
    RLcp33_121 = s.dpt(1,32)*ROcp33_120+s.dpt(2,32)*ROcp33_419+ROcp33_720*s.dpt(3,32);
    RLcp33_221 = s.dpt(1,32)*ROcp33_220+s.dpt(2,32)*ROcp33_519+ROcp33_820*s.dpt(3,32);
    RLcp33_321 = s.dpt(1,32)*ROcp33_320+s.dpt(2,32)*ROcp33_619+ROcp33_920*s.dpt(3,32);
    OMcp33_121 = OMcp33_120+ROcp33_720*qd(21);
    OMcp33_221 = OMcp33_220+ROcp33_820*qd(21);
    OMcp33_321 = OMcp33_320+ROcp33_920*qd(21);
    ORcp33_121 = OMcp33_220*RLcp33_321-OMcp33_320*RLcp33_221;
    ORcp33_221 = -(OMcp33_120*RLcp33_321-OMcp33_320*RLcp33_121);
    ORcp33_321 = OMcp33_120*RLcp33_221-OMcp33_220*RLcp33_121;
    OPcp33_121 = OPcp33_120+ROcp33_720*qdd(21)+qd(21)*(OMcp33_220*ROcp33_920-OMcp33_320*ROcp33_820);
    OPcp33_221 = OPcp33_220+ROcp33_820*qdd(21)-qd(21)*(OMcp33_120*ROcp33_920-OMcp33_320*ROcp33_720);
    OPcp33_321 = OPcp33_320+ROcp33_920*qdd(21)+qd(21)*(OMcp33_120*ROcp33_820-OMcp33_220*ROcp33_720);

% = = Block_1_0_0_34_0_6 = = 
 
% Sensor Kinematics 


    ROcp33_129 = ROcp33_121*C29-ROcp33_720*S29;
    ROcp33_229 = ROcp33_221*C29-ROcp33_820*S29;
    ROcp33_329 = ROcp33_321*C29-ROcp33_920*S29;
    ROcp33_729 = ROcp33_121*S29+ROcp33_720*C29;
    ROcp33_829 = ROcp33_221*S29+ROcp33_820*C29;
    ROcp33_929 = ROcp33_321*S29+ROcp33_920*C29;
    ROcp33_430 = ROcp33_421*C30+ROcp33_729*S30;
    ROcp33_530 = ROcp33_521*C30+ROcp33_829*S30;
    ROcp33_630 = ROcp33_621*C30+ROcp33_929*S30;
    ROcp33_730 = -(ROcp33_421*S30-ROcp33_729*C30);
    ROcp33_830 = -(ROcp33_521*S30-ROcp33_829*C30);
    ROcp33_930 = -(ROcp33_621*S30-ROcp33_929*C30);
    ROcp33_131 = ROcp33_129*C31-ROcp33_730*S31;
    ROcp33_231 = ROcp33_229*C31-ROcp33_830*S31;
    ROcp33_331 = ROcp33_329*C31-ROcp33_930*S31;
    ROcp33_731 = ROcp33_129*S31+ROcp33_730*C31;
    ROcp33_831 = ROcp33_229*S31+ROcp33_830*C31;
    ROcp33_931 = ROcp33_329*S31+ROcp33_930*C31;
    ROcp33_132 = ROcp33_131*C32+ROcp33_430*S32;
    ROcp33_232 = ROcp33_231*C32+ROcp33_530*S32;
    ROcp33_332 = ROcp33_331*C32+ROcp33_630*S32;
    ROcp33_432 = -(ROcp33_131*S32-ROcp33_430*C32);
    ROcp33_532 = -(ROcp33_231*S32-ROcp33_530*C32);
    ROcp33_632 = -(ROcp33_331*S32-ROcp33_630*C32);
    ROcp33_133 = ROcp33_132*C33-ROcp33_731*S33;
    ROcp33_233 = ROcp33_232*C33-ROcp33_831*S33;
    ROcp33_333 = ROcp33_332*C33-ROcp33_931*S33;
    ROcp33_733 = ROcp33_132*S33+ROcp33_731*C33;
    ROcp33_833 = ROcp33_232*S33+ROcp33_831*C33;
    ROcp33_933 = ROcp33_332*S33+ROcp33_931*C33;
    ROcp33_134 = ROcp33_133*C34+ROcp33_432*S34;
    ROcp33_234 = ROcp33_233*C34+ROcp33_532*S34;
    ROcp33_334 = ROcp33_333*C34+ROcp33_632*S34;
    ROcp33_434 = -(ROcp33_133*S34-ROcp33_432*C34);
    ROcp33_534 = -(ROcp33_233*S34-ROcp33_532*C34);
    ROcp33_634 = -(ROcp33_333*S34-ROcp33_632*C34);
    RLcp33_129 = ROcp33_121*s.dpt(1,37)+ROcp33_421*s.dpt(2,37)+ROcp33_720*s.dpt(3,37);
    RLcp33_229 = ROcp33_221*s.dpt(1,37)+ROcp33_521*s.dpt(2,37)+ROcp33_820*s.dpt(3,37);
    RLcp33_329 = ROcp33_321*s.dpt(1,37)+ROcp33_621*s.dpt(2,37)+ROcp33_920*s.dpt(3,37);
    OMcp33_129 = OMcp33_121+ROcp33_421*qd(29);
    OMcp33_229 = OMcp33_221+ROcp33_521*qd(29);
    OMcp33_329 = OMcp33_321+ROcp33_621*qd(29);
    ORcp33_129 = OMcp33_221*RLcp33_329-OMcp33_321*RLcp33_229;
    ORcp33_229 = -(OMcp33_121*RLcp33_329-OMcp33_321*RLcp33_129);
    ORcp33_329 = OMcp33_121*RLcp33_229-OMcp33_221*RLcp33_129;
    OPcp33_129 = OPcp33_121+ROcp33_421*qdd(29)+qd(29)*(OMcp33_221*ROcp33_621-OMcp33_321*ROcp33_521);
    OPcp33_229 = OPcp33_221+ROcp33_521*qdd(29)-qd(29)*(OMcp33_121*ROcp33_621-OMcp33_321*ROcp33_421);
    OPcp33_329 = OPcp33_321+ROcp33_621*qdd(29)+qd(29)*(OMcp33_121*ROcp33_521-OMcp33_221*ROcp33_421);
    RLcp33_130 = s.dpt(1,51)*ROcp33_129+s.dpt(3,51)*ROcp33_729+ROcp33_421*s.dpt(2,51);
    RLcp33_230 = s.dpt(1,51)*ROcp33_229+s.dpt(3,51)*ROcp33_829+ROcp33_521*s.dpt(2,51);
    RLcp33_330 = s.dpt(1,51)*ROcp33_329+s.dpt(3,51)*ROcp33_929+ROcp33_621*s.dpt(2,51);
    OMcp33_130 = OMcp33_129+ROcp33_129*qd(30);
    OMcp33_230 = OMcp33_229+ROcp33_229*qd(30);
    OMcp33_330 = OMcp33_329+ROcp33_329*qd(30);
    ORcp33_130 = OMcp33_229*RLcp33_330-OMcp33_329*RLcp33_230;
    ORcp33_230 = -(OMcp33_129*RLcp33_330-OMcp33_329*RLcp33_130);
    ORcp33_330 = OMcp33_129*RLcp33_230-OMcp33_229*RLcp33_130;
    OPcp33_130 = OPcp33_129+ROcp33_129*qdd(30)+qd(30)*(OMcp33_229*ROcp33_329-OMcp33_329*ROcp33_229);
    OPcp33_230 = OPcp33_229+ROcp33_229*qdd(30)-qd(30)*(OMcp33_129*ROcp33_329-OMcp33_329*ROcp33_129);
    OPcp33_330 = OPcp33_329+ROcp33_329*qdd(30)+qd(30)*(OMcp33_129*ROcp33_229-OMcp33_229*ROcp33_129);
    RLcp33_131 = s.dpt(1,53)*ROcp33_129+s.dpt(3,53)*ROcp33_730+ROcp33_430*s.dpt(2,53);
    RLcp33_231 = s.dpt(1,53)*ROcp33_229+s.dpt(3,53)*ROcp33_830+ROcp33_530*s.dpt(2,53);
    RLcp33_331 = s.dpt(1,53)*ROcp33_329+s.dpt(3,53)*ROcp33_930+ROcp33_630*s.dpt(2,53);
    OMcp33_131 = OMcp33_130+ROcp33_430*qd(31);
    OMcp33_231 = OMcp33_230+ROcp33_530*qd(31);
    OMcp33_331 = OMcp33_330+ROcp33_630*qd(31);
    ORcp33_131 = OMcp33_230*RLcp33_331-OMcp33_330*RLcp33_231;
    ORcp33_231 = -(OMcp33_130*RLcp33_331-OMcp33_330*RLcp33_131);
    ORcp33_331 = OMcp33_130*RLcp33_231-OMcp33_230*RLcp33_131;
    OPcp33_131 = OPcp33_130+ROcp33_430*qdd(31)+qd(31)*(OMcp33_230*ROcp33_630-OMcp33_330*ROcp33_530);
    OPcp33_231 = OPcp33_230+ROcp33_530*qdd(31)-qd(31)*(OMcp33_130*ROcp33_630-OMcp33_330*ROcp33_430);
    OPcp33_331 = OPcp33_330+ROcp33_630*qdd(31)+qd(31)*(OMcp33_130*ROcp33_530-OMcp33_230*ROcp33_430);
    RLcp33_132 = s.dpt(3,55)*ROcp33_731+ROcp33_131*s.dpt(1,55)+ROcp33_430*s.dpt(2,55);
    RLcp33_232 = s.dpt(3,55)*ROcp33_831+ROcp33_231*s.dpt(1,55)+ROcp33_530*s.dpt(2,55);
    RLcp33_332 = s.dpt(3,55)*ROcp33_931+ROcp33_331*s.dpt(1,55)+ROcp33_630*s.dpt(2,55);
    OMcp33_132 = OMcp33_131+ROcp33_731*qd(32);
    OMcp33_232 = OMcp33_231+ROcp33_831*qd(32);
    OMcp33_332 = OMcp33_331+ROcp33_931*qd(32);
    ORcp33_132 = OMcp33_231*RLcp33_332-OMcp33_331*RLcp33_232;
    ORcp33_232 = -(OMcp33_131*RLcp33_332-OMcp33_331*RLcp33_132);
    ORcp33_332 = OMcp33_131*RLcp33_232-OMcp33_231*RLcp33_132;
    OPcp33_132 = OPcp33_131+ROcp33_731*qdd(32)+qd(32)*(OMcp33_231*ROcp33_931-OMcp33_331*ROcp33_831);
    OPcp33_232 = OPcp33_231+ROcp33_831*qdd(32)-qd(32)*(OMcp33_131*ROcp33_931-OMcp33_331*ROcp33_731);
    OPcp33_332 = OPcp33_331+ROcp33_931*qdd(32)+qd(32)*(OMcp33_131*ROcp33_831-OMcp33_231*ROcp33_731);
    RLcp33_133 = s.dpt(2,57)*ROcp33_432+s.dpt(3,57)*ROcp33_731+ROcp33_132*s.dpt(1,57);
    RLcp33_233 = s.dpt(2,57)*ROcp33_532+s.dpt(3,57)*ROcp33_831+ROcp33_232*s.dpt(1,57);
    RLcp33_333 = s.dpt(2,57)*ROcp33_632+s.dpt(3,57)*ROcp33_931+ROcp33_332*s.dpt(1,57);
    OMcp33_133 = OMcp33_132+ROcp33_432*qd(33);
    OMcp33_233 = OMcp33_232+ROcp33_532*qd(33);
    OMcp33_333 = OMcp33_332+ROcp33_632*qd(33);
    ORcp33_133 = OMcp33_232*RLcp33_333-OMcp33_332*RLcp33_233;
    ORcp33_233 = -(OMcp33_132*RLcp33_333-OMcp33_332*RLcp33_133);
    ORcp33_333 = OMcp33_132*RLcp33_233-OMcp33_232*RLcp33_133;
    OPcp33_133 = OPcp33_132+ROcp33_432*qdd(33)+qd(33)*(OMcp33_232*ROcp33_632-OMcp33_332*ROcp33_532);
    OPcp33_233 = OPcp33_232+ROcp33_532*qdd(33)-qd(33)*(OMcp33_132*ROcp33_632-OMcp33_332*ROcp33_432);
    OPcp33_333 = OPcp33_332+ROcp33_632*qdd(33)+qd(33)*(OMcp33_132*ROcp33_532-OMcp33_232*ROcp33_432);
    RLcp33_134 = s.dpt(1,58)*ROcp33_133+s.dpt(3,58)*ROcp33_733+ROcp33_432*s.dpt(2,58);
    RLcp33_234 = s.dpt(1,58)*ROcp33_233+s.dpt(3,58)*ROcp33_833+ROcp33_532*s.dpt(2,58);
    RLcp33_334 = s.dpt(1,58)*ROcp33_333+s.dpt(3,58)*ROcp33_933+ROcp33_632*s.dpt(2,58);
    POcp33_134 = RLcp33_119+RLcp33_120+RLcp33_121+RLcp33_129+RLcp33_130+RLcp33_131+RLcp33_132+RLcp33_133+RLcp33_134+q(1);
    POcp33_234 = RLcp33_219+RLcp33_220+RLcp33_221+RLcp33_229+RLcp33_230+RLcp33_231+RLcp33_232+RLcp33_233+RLcp33_234+q(2);
    POcp33_334 = RLcp33_319+RLcp33_320+RLcp33_321+RLcp33_329+RLcp33_330+RLcp33_331+RLcp33_332+RLcp33_333+RLcp33_334+q(3);
    OMcp33_134 = OMcp33_133+ROcp33_733*qd(34);
    OMcp33_234 = OMcp33_233+ROcp33_833*qd(34);
    OMcp33_334 = OMcp33_333+ROcp33_933*qd(34);
    ORcp33_134 = OMcp33_233*RLcp33_334-OMcp33_333*RLcp33_234;
    ORcp33_234 = -(OMcp33_133*RLcp33_334-OMcp33_333*RLcp33_134);
    ORcp33_334 = OMcp33_133*RLcp33_234-OMcp33_233*RLcp33_134;
    VIcp33_134 = ORcp33_119+ORcp33_120+ORcp33_121+ORcp33_129+ORcp33_130+ORcp33_131+ORcp33_132+ORcp33_133+ORcp33_134+qd(1);
    VIcp33_234 = ORcp33_219+ORcp33_220+ORcp33_221+ORcp33_229+ORcp33_230+ORcp33_231+ORcp33_232+ORcp33_233+ORcp33_234+qd(2);
    VIcp33_334 = ORcp33_319+ORcp33_320+ORcp33_321+ORcp33_329+ORcp33_330+ORcp33_331+ORcp33_332+ORcp33_333+ORcp33_334+qd(3);
    OPcp33_134 = OPcp33_133+ROcp33_733*qdd(34)+qd(34)*(OMcp33_233*ROcp33_933-OMcp33_333*ROcp33_833);
    OPcp33_234 = OPcp33_233+ROcp33_833*qdd(34)-qd(34)*(OMcp33_133*ROcp33_933-OMcp33_333*ROcp33_733);
    OPcp33_334 = OPcp33_333+ROcp33_933*qdd(34)+qd(34)*(OMcp33_133*ROcp33_833-OMcp33_233*ROcp33_733);
    ACcp33_134 = qdd(1)+OMcp33_219*ORcp33_320+OMcp33_220*ORcp33_321+OMcp33_221*ORcp33_329+OMcp33_229*ORcp33_330+OMcp33_230*ORcp33_331+OMcp33_231*...
 ORcp33_332+OMcp33_232*ORcp33_333+OMcp33_233*ORcp33_334+OMcp33_26*ORcp33_319-OMcp33_319*ORcp33_220-OMcp33_320*ORcp33_221-OMcp33_321*ORcp33_229-...
 OMcp33_329*ORcp33_230-OMcp33_330*ORcp33_231-OMcp33_331*ORcp33_232-OMcp33_332*ORcp33_233-OMcp33_333*ORcp33_234-OMcp33_36*ORcp33_219+OPcp33_219*...
 RLcp33_320+OPcp33_220*RLcp33_321+OPcp33_221*RLcp33_329+OPcp33_229*RLcp33_330+OPcp33_230*RLcp33_331+OPcp33_231*RLcp33_332+OPcp33_232*RLcp33_333+...
 OPcp33_233*RLcp33_334+OPcp33_26*RLcp33_319-OPcp33_319*RLcp33_220-OPcp33_320*RLcp33_221-OPcp33_321*RLcp33_229-OPcp33_329*RLcp33_230-OPcp33_330*...
 RLcp33_231-OPcp33_331*RLcp33_232-OPcp33_332*RLcp33_233-OPcp33_333*RLcp33_234-OPcp33_36*RLcp33_219;
    ACcp33_234 = qdd(2)-OMcp33_119*ORcp33_320-OMcp33_120*ORcp33_321-OMcp33_121*ORcp33_329-OMcp33_129*ORcp33_330-OMcp33_130*ORcp33_331-OMcp33_131*...
 ORcp33_332-OMcp33_132*ORcp33_333-OMcp33_133*ORcp33_334-OMcp33_16*ORcp33_319+OMcp33_319*ORcp33_120+OMcp33_320*ORcp33_121+OMcp33_321*ORcp33_129+...
 OMcp33_329*ORcp33_130+OMcp33_330*ORcp33_131+OMcp33_331*ORcp33_132+OMcp33_332*ORcp33_133+OMcp33_333*ORcp33_134+OMcp33_36*ORcp33_119-OPcp33_119*...
 RLcp33_320-OPcp33_120*RLcp33_321-OPcp33_121*RLcp33_329-OPcp33_129*RLcp33_330-OPcp33_130*RLcp33_331-OPcp33_131*RLcp33_332-OPcp33_132*RLcp33_333-...
 OPcp33_133*RLcp33_334-OPcp33_16*RLcp33_319+OPcp33_319*RLcp33_120+OPcp33_320*RLcp33_121+OPcp33_321*RLcp33_129+OPcp33_329*RLcp33_130+OPcp33_330*...
 RLcp33_131+OPcp33_331*RLcp33_132+OPcp33_332*RLcp33_133+OPcp33_333*RLcp33_134+OPcp33_36*RLcp33_119;
    ACcp33_334 = qdd(3)+OMcp33_119*ORcp33_220+OMcp33_120*ORcp33_221+OMcp33_121*ORcp33_229+OMcp33_129*ORcp33_230+OMcp33_130*ORcp33_231+OMcp33_131*...
 ORcp33_232+OMcp33_132*ORcp33_233+OMcp33_133*ORcp33_234+OMcp33_16*ORcp33_219-OMcp33_219*ORcp33_120-OMcp33_220*ORcp33_121-OMcp33_221*ORcp33_129-...
 OMcp33_229*ORcp33_130-OMcp33_230*ORcp33_131-OMcp33_231*ORcp33_132-OMcp33_232*ORcp33_133-OMcp33_233*ORcp33_134-OMcp33_26*ORcp33_119+OPcp33_119*...
 RLcp33_220+OPcp33_120*RLcp33_221+OPcp33_121*RLcp33_229+OPcp33_129*RLcp33_230+OPcp33_130*RLcp33_231+OPcp33_131*RLcp33_232+OPcp33_132*RLcp33_233+...
 OPcp33_133*RLcp33_234+OPcp33_16*RLcp33_219-OPcp33_219*RLcp33_120-OPcp33_220*RLcp33_121-OPcp33_221*RLcp33_129-OPcp33_229*RLcp33_130-OPcp33_230*...
 RLcp33_131-OPcp33_231*RLcp33_132-OPcp33_232*RLcp33_133-OPcp33_233*RLcp33_134-OPcp33_26*RLcp33_119;

% = = Block_1_0_0_34_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp33_134;
    sens.P(2) = POcp33_234;
    sens.P(3) = POcp33_334;
    sens.R(1,1) = ROcp33_134;
    sens.R(1,2) = ROcp33_234;
    sens.R(1,3) = ROcp33_334;
    sens.R(2,1) = ROcp33_434;
    sens.R(2,2) = ROcp33_534;
    sens.R(2,3) = ROcp33_634;
    sens.R(3,1) = ROcp33_733;
    sens.R(3,2) = ROcp33_833;
    sens.R(3,3) = ROcp33_933;
    sens.V(1) = VIcp33_134;
    sens.V(2) = VIcp33_234;
    sens.V(3) = VIcp33_334;
    sens.OM(1) = OMcp33_134;
    sens.OM(2) = OMcp33_234;
    sens.OM(3) = OMcp33_334;
    sens.A(1) = ACcp33_134;
    sens.A(2) = ACcp33_234;
    sens.A(3) = ACcp33_334;
    sens.OMP(1) = OPcp33_134;
    sens.OMP(2) = OPcp33_234;
    sens.OMP(3) = OPcp33_334;
 
% 
case 35, 


% = = Block_1_0_0_35_0_1 = = 
 
% Sensor Kinematics 


    ROcp34_25 = S4*S5;
    ROcp34_35 = -C4*S5;
    ROcp34_85 = -S4*C5;
    ROcp34_95 = C4*C5;
    ROcp34_16 = C5*C6;
    ROcp34_26 = ROcp34_25*C6+C4*S6;
    ROcp34_36 = ROcp34_35*C6+S4*S6;
    ROcp34_46 = -C5*S6;
    ROcp34_56 = -(ROcp34_25*S6-C4*C6);
    ROcp34_66 = -(ROcp34_35*S6-S4*C6);
    OMcp34_25 = qd(5)*C4;
    OMcp34_35 = qd(5)*S4;
    OMcp34_16 = qd(4)+qd(6)*S5;
    OMcp34_26 = OMcp34_25+ROcp34_85*qd(6);
    OMcp34_36 = OMcp34_35+ROcp34_95*qd(6);
    OPcp34_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp34_26 = ROcp34_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp34_35*S5-ROcp34_95*qd(4));
    OPcp34_36 = ROcp34_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp34_25*S5-ROcp34_85*qd(4));

% = = Block_1_0_0_35_0_4 = = 
 
% Sensor Kinematics 


    ROcp34_419 = ROcp34_46*C19+S19*S5;
    ROcp34_519 = ROcp34_56*C19+ROcp34_85*S19;
    ROcp34_619 = ROcp34_66*C19+ROcp34_95*S19;
    ROcp34_719 = -(ROcp34_46*S19-C19*S5);
    ROcp34_819 = -(ROcp34_56*S19-ROcp34_85*C19);
    ROcp34_919 = -(ROcp34_66*S19-ROcp34_95*C19);
    ROcp34_120 = ROcp34_16*C20-ROcp34_719*S20;
    ROcp34_220 = ROcp34_26*C20-ROcp34_819*S20;
    ROcp34_320 = ROcp34_36*C20-ROcp34_919*S20;
    ROcp34_720 = ROcp34_16*S20+ROcp34_719*C20;
    ROcp34_820 = ROcp34_26*S20+ROcp34_819*C20;
    ROcp34_920 = ROcp34_36*S20+ROcp34_919*C20;
    ROcp34_121 = ROcp34_120*C21+ROcp34_419*S21;
    ROcp34_221 = ROcp34_220*C21+ROcp34_519*S21;
    ROcp34_321 = ROcp34_320*C21+ROcp34_619*S21;
    ROcp34_421 = -(ROcp34_120*S21-ROcp34_419*C21);
    ROcp34_521 = -(ROcp34_220*S21-ROcp34_519*C21);
    ROcp34_621 = -(ROcp34_320*S21-ROcp34_619*C21);
    RLcp34_119 = s.dpt(2,3)*ROcp34_46+ROcp34_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp34_219 = s.dpt(2,3)*ROcp34_56+ROcp34_26*s.dpt(1,3)+ROcp34_85*s.dpt(3,3);
    RLcp34_319 = s.dpt(2,3)*ROcp34_66+ROcp34_36*s.dpt(1,3)+ROcp34_95*s.dpt(3,3);
    OMcp34_119 = OMcp34_16+ROcp34_16*qd(19);
    OMcp34_219 = OMcp34_26+ROcp34_26*qd(19);
    OMcp34_319 = OMcp34_36+ROcp34_36*qd(19);
    ORcp34_119 = OMcp34_26*RLcp34_319-OMcp34_36*RLcp34_219;
    ORcp34_219 = -(OMcp34_16*RLcp34_319-OMcp34_36*RLcp34_119);
    ORcp34_319 = OMcp34_16*RLcp34_219-OMcp34_26*RLcp34_119;
    OPcp34_119 = OPcp34_16+ROcp34_16*qdd(19)+qd(19)*(OMcp34_26*ROcp34_36-OMcp34_36*ROcp34_26);
    OPcp34_219 = OPcp34_26+ROcp34_26*qdd(19)-qd(19)*(OMcp34_16*ROcp34_36-OMcp34_36*ROcp34_16);
    OPcp34_319 = OPcp34_36+ROcp34_36*qdd(19)+qd(19)*(OMcp34_16*ROcp34_26-OMcp34_26*ROcp34_16);
    RLcp34_120 = s.dpt(1,30)*ROcp34_16+s.dpt(2,30)*ROcp34_419+s.dpt(3,30)*ROcp34_719;
    RLcp34_220 = s.dpt(1,30)*ROcp34_26+s.dpt(2,30)*ROcp34_519+s.dpt(3,30)*ROcp34_819;
    RLcp34_320 = s.dpt(1,30)*ROcp34_36+s.dpt(2,30)*ROcp34_619+s.dpt(3,30)*ROcp34_919;
    OMcp34_120 = OMcp34_119+ROcp34_419*qd(20);
    OMcp34_220 = OMcp34_219+ROcp34_519*qd(20);
    OMcp34_320 = OMcp34_319+ROcp34_619*qd(20);
    ORcp34_120 = OMcp34_219*RLcp34_320-OMcp34_319*RLcp34_220;
    ORcp34_220 = -(OMcp34_119*RLcp34_320-OMcp34_319*RLcp34_120);
    ORcp34_320 = OMcp34_119*RLcp34_220-OMcp34_219*RLcp34_120;
    OPcp34_120 = OPcp34_119+ROcp34_419*qdd(20)+qd(20)*(OMcp34_219*ROcp34_619-OMcp34_319*ROcp34_519);
    OPcp34_220 = OPcp34_219+ROcp34_519*qdd(20)-qd(20)*(OMcp34_119*ROcp34_619-OMcp34_319*ROcp34_419);
    OPcp34_320 = OPcp34_319+ROcp34_619*qdd(20)+qd(20)*(OMcp34_119*ROcp34_519-OMcp34_219*ROcp34_419);
    RLcp34_121 = s.dpt(1,32)*ROcp34_120+s.dpt(2,32)*ROcp34_419+ROcp34_720*s.dpt(3,32);
    RLcp34_221 = s.dpt(1,32)*ROcp34_220+s.dpt(2,32)*ROcp34_519+ROcp34_820*s.dpt(3,32);
    RLcp34_321 = s.dpt(1,32)*ROcp34_320+s.dpt(2,32)*ROcp34_619+ROcp34_920*s.dpt(3,32);
    OMcp34_121 = OMcp34_120+ROcp34_720*qd(21);
    OMcp34_221 = OMcp34_220+ROcp34_820*qd(21);
    OMcp34_321 = OMcp34_320+ROcp34_920*qd(21);
    ORcp34_121 = OMcp34_220*RLcp34_321-OMcp34_320*RLcp34_221;
    ORcp34_221 = -(OMcp34_120*RLcp34_321-OMcp34_320*RLcp34_121);
    ORcp34_321 = OMcp34_120*RLcp34_221-OMcp34_220*RLcp34_121;
    OPcp34_121 = OPcp34_120+ROcp34_720*qdd(21)+qd(21)*(OMcp34_220*ROcp34_920-OMcp34_320*ROcp34_820);
    OPcp34_221 = OPcp34_220+ROcp34_820*qdd(21)-qd(21)*(OMcp34_120*ROcp34_920-OMcp34_320*ROcp34_720);
    OPcp34_321 = OPcp34_320+ROcp34_920*qdd(21)+qd(21)*(OMcp34_120*ROcp34_820-OMcp34_220*ROcp34_720);

% = = Block_1_0_0_35_0_6 = = 
 
% Sensor Kinematics 


    ROcp34_129 = ROcp34_121*C29-ROcp34_720*S29;
    ROcp34_229 = ROcp34_221*C29-ROcp34_820*S29;
    ROcp34_329 = ROcp34_321*C29-ROcp34_920*S29;
    ROcp34_729 = ROcp34_121*S29+ROcp34_720*C29;
    ROcp34_829 = ROcp34_221*S29+ROcp34_820*C29;
    ROcp34_929 = ROcp34_321*S29+ROcp34_920*C29;
    ROcp34_430 = ROcp34_421*C30+ROcp34_729*S30;
    ROcp34_530 = ROcp34_521*C30+ROcp34_829*S30;
    ROcp34_630 = ROcp34_621*C30+ROcp34_929*S30;
    ROcp34_730 = -(ROcp34_421*S30-ROcp34_729*C30);
    ROcp34_830 = -(ROcp34_521*S30-ROcp34_829*C30);
    ROcp34_930 = -(ROcp34_621*S30-ROcp34_929*C30);
    ROcp34_131 = ROcp34_129*C31-ROcp34_730*S31;
    ROcp34_231 = ROcp34_229*C31-ROcp34_830*S31;
    ROcp34_331 = ROcp34_329*C31-ROcp34_930*S31;
    ROcp34_731 = ROcp34_129*S31+ROcp34_730*C31;
    ROcp34_831 = ROcp34_229*S31+ROcp34_830*C31;
    ROcp34_931 = ROcp34_329*S31+ROcp34_930*C31;
    ROcp34_132 = ROcp34_131*C32+ROcp34_430*S32;
    ROcp34_232 = ROcp34_231*C32+ROcp34_530*S32;
    ROcp34_332 = ROcp34_331*C32+ROcp34_630*S32;
    ROcp34_432 = -(ROcp34_131*S32-ROcp34_430*C32);
    ROcp34_532 = -(ROcp34_231*S32-ROcp34_530*C32);
    ROcp34_632 = -(ROcp34_331*S32-ROcp34_630*C32);
    ROcp34_133 = ROcp34_132*C33-ROcp34_731*S33;
    ROcp34_233 = ROcp34_232*C33-ROcp34_831*S33;
    ROcp34_333 = ROcp34_332*C33-ROcp34_931*S33;
    ROcp34_733 = ROcp34_132*S33+ROcp34_731*C33;
    ROcp34_833 = ROcp34_232*S33+ROcp34_831*C33;
    ROcp34_933 = ROcp34_332*S33+ROcp34_931*C33;
    ROcp34_134 = ROcp34_133*C34+ROcp34_432*S34;
    ROcp34_234 = ROcp34_233*C34+ROcp34_532*S34;
    ROcp34_334 = ROcp34_333*C34+ROcp34_632*S34;
    ROcp34_434 = -(ROcp34_133*S34-ROcp34_432*C34);
    ROcp34_534 = -(ROcp34_233*S34-ROcp34_532*C34);
    ROcp34_634 = -(ROcp34_333*S34-ROcp34_632*C34);
    ROcp34_435 = ROcp34_434*C35+ROcp34_733*S35;
    ROcp34_535 = ROcp34_534*C35+ROcp34_833*S35;
    ROcp34_635 = ROcp34_634*C35+ROcp34_933*S35;
    ROcp34_735 = -(ROcp34_434*S35-ROcp34_733*C35);
    ROcp34_835 = -(ROcp34_534*S35-ROcp34_833*C35);
    ROcp34_935 = -(ROcp34_634*S35-ROcp34_933*C35);
    RLcp34_129 = ROcp34_121*s.dpt(1,37)+ROcp34_421*s.dpt(2,37)+ROcp34_720*s.dpt(3,37);
    RLcp34_229 = ROcp34_221*s.dpt(1,37)+ROcp34_521*s.dpt(2,37)+ROcp34_820*s.dpt(3,37);
    RLcp34_329 = ROcp34_321*s.dpt(1,37)+ROcp34_621*s.dpt(2,37)+ROcp34_920*s.dpt(3,37);
    OMcp34_129 = OMcp34_121+ROcp34_421*qd(29);
    OMcp34_229 = OMcp34_221+ROcp34_521*qd(29);
    OMcp34_329 = OMcp34_321+ROcp34_621*qd(29);
    ORcp34_129 = OMcp34_221*RLcp34_329-OMcp34_321*RLcp34_229;
    ORcp34_229 = -(OMcp34_121*RLcp34_329-OMcp34_321*RLcp34_129);
    ORcp34_329 = OMcp34_121*RLcp34_229-OMcp34_221*RLcp34_129;
    OPcp34_129 = OPcp34_121+ROcp34_421*qdd(29)+qd(29)*(OMcp34_221*ROcp34_621-OMcp34_321*ROcp34_521);
    OPcp34_229 = OPcp34_221+ROcp34_521*qdd(29)-qd(29)*(OMcp34_121*ROcp34_621-OMcp34_321*ROcp34_421);
    OPcp34_329 = OPcp34_321+ROcp34_621*qdd(29)+qd(29)*(OMcp34_121*ROcp34_521-OMcp34_221*ROcp34_421);
    RLcp34_130 = s.dpt(1,51)*ROcp34_129+s.dpt(3,51)*ROcp34_729+ROcp34_421*s.dpt(2,51);
    RLcp34_230 = s.dpt(1,51)*ROcp34_229+s.dpt(3,51)*ROcp34_829+ROcp34_521*s.dpt(2,51);
    RLcp34_330 = s.dpt(1,51)*ROcp34_329+s.dpt(3,51)*ROcp34_929+ROcp34_621*s.dpt(2,51);
    OMcp34_130 = OMcp34_129+ROcp34_129*qd(30);
    OMcp34_230 = OMcp34_229+ROcp34_229*qd(30);
    OMcp34_330 = OMcp34_329+ROcp34_329*qd(30);
    ORcp34_130 = OMcp34_229*RLcp34_330-OMcp34_329*RLcp34_230;
    ORcp34_230 = -(OMcp34_129*RLcp34_330-OMcp34_329*RLcp34_130);
    ORcp34_330 = OMcp34_129*RLcp34_230-OMcp34_229*RLcp34_130;
    OPcp34_130 = OPcp34_129+ROcp34_129*qdd(30)+qd(30)*(OMcp34_229*ROcp34_329-OMcp34_329*ROcp34_229);
    OPcp34_230 = OPcp34_229+ROcp34_229*qdd(30)-qd(30)*(OMcp34_129*ROcp34_329-OMcp34_329*ROcp34_129);
    OPcp34_330 = OPcp34_329+ROcp34_329*qdd(30)+qd(30)*(OMcp34_129*ROcp34_229-OMcp34_229*ROcp34_129);
    RLcp34_131 = s.dpt(1,53)*ROcp34_129+s.dpt(3,53)*ROcp34_730+ROcp34_430*s.dpt(2,53);
    RLcp34_231 = s.dpt(1,53)*ROcp34_229+s.dpt(3,53)*ROcp34_830+ROcp34_530*s.dpt(2,53);
    RLcp34_331 = s.dpt(1,53)*ROcp34_329+s.dpt(3,53)*ROcp34_930+ROcp34_630*s.dpt(2,53);
    OMcp34_131 = OMcp34_130+ROcp34_430*qd(31);
    OMcp34_231 = OMcp34_230+ROcp34_530*qd(31);
    OMcp34_331 = OMcp34_330+ROcp34_630*qd(31);
    ORcp34_131 = OMcp34_230*RLcp34_331-OMcp34_330*RLcp34_231;
    ORcp34_231 = -(OMcp34_130*RLcp34_331-OMcp34_330*RLcp34_131);
    ORcp34_331 = OMcp34_130*RLcp34_231-OMcp34_230*RLcp34_131;
    OPcp34_131 = OPcp34_130+ROcp34_430*qdd(31)+qd(31)*(OMcp34_230*ROcp34_630-OMcp34_330*ROcp34_530);
    OPcp34_231 = OPcp34_230+ROcp34_530*qdd(31)-qd(31)*(OMcp34_130*ROcp34_630-OMcp34_330*ROcp34_430);
    OPcp34_331 = OPcp34_330+ROcp34_630*qdd(31)+qd(31)*(OMcp34_130*ROcp34_530-OMcp34_230*ROcp34_430);
    RLcp34_132 = s.dpt(3,55)*ROcp34_731+ROcp34_131*s.dpt(1,55)+ROcp34_430*s.dpt(2,55);
    RLcp34_232 = s.dpt(3,55)*ROcp34_831+ROcp34_231*s.dpt(1,55)+ROcp34_530*s.dpt(2,55);
    RLcp34_332 = s.dpt(3,55)*ROcp34_931+ROcp34_331*s.dpt(1,55)+ROcp34_630*s.dpt(2,55);
    OMcp34_132 = OMcp34_131+ROcp34_731*qd(32);
    OMcp34_232 = OMcp34_231+ROcp34_831*qd(32);
    OMcp34_332 = OMcp34_331+ROcp34_931*qd(32);
    ORcp34_132 = OMcp34_231*RLcp34_332-OMcp34_331*RLcp34_232;
    ORcp34_232 = -(OMcp34_131*RLcp34_332-OMcp34_331*RLcp34_132);
    ORcp34_332 = OMcp34_131*RLcp34_232-OMcp34_231*RLcp34_132;
    OPcp34_132 = OPcp34_131+ROcp34_731*qdd(32)+qd(32)*(OMcp34_231*ROcp34_931-OMcp34_331*ROcp34_831);
    OPcp34_232 = OPcp34_231+ROcp34_831*qdd(32)-qd(32)*(OMcp34_131*ROcp34_931-OMcp34_331*ROcp34_731);
    OPcp34_332 = OPcp34_331+ROcp34_931*qdd(32)+qd(32)*(OMcp34_131*ROcp34_831-OMcp34_231*ROcp34_731);
    RLcp34_133 = s.dpt(2,57)*ROcp34_432+s.dpt(3,57)*ROcp34_731+ROcp34_132*s.dpt(1,57);
    RLcp34_233 = s.dpt(2,57)*ROcp34_532+s.dpt(3,57)*ROcp34_831+ROcp34_232*s.dpt(1,57);
    RLcp34_333 = s.dpt(2,57)*ROcp34_632+s.dpt(3,57)*ROcp34_931+ROcp34_332*s.dpt(1,57);
    OMcp34_133 = OMcp34_132+ROcp34_432*qd(33);
    OMcp34_233 = OMcp34_232+ROcp34_532*qd(33);
    OMcp34_333 = OMcp34_332+ROcp34_632*qd(33);
    ORcp34_133 = OMcp34_232*RLcp34_333-OMcp34_332*RLcp34_233;
    ORcp34_233 = -(OMcp34_132*RLcp34_333-OMcp34_332*RLcp34_133);
    ORcp34_333 = OMcp34_132*RLcp34_233-OMcp34_232*RLcp34_133;
    OPcp34_133 = OPcp34_132+ROcp34_432*qdd(33)+qd(33)*(OMcp34_232*ROcp34_632-OMcp34_332*ROcp34_532);
    OPcp34_233 = OPcp34_232+ROcp34_532*qdd(33)-qd(33)*(OMcp34_132*ROcp34_632-OMcp34_332*ROcp34_432);
    OPcp34_333 = OPcp34_332+ROcp34_632*qdd(33)+qd(33)*(OMcp34_132*ROcp34_532-OMcp34_232*ROcp34_432);
    RLcp34_134 = s.dpt(1,58)*ROcp34_133+s.dpt(3,58)*ROcp34_733+ROcp34_432*s.dpt(2,58);
    RLcp34_234 = s.dpt(1,58)*ROcp34_233+s.dpt(3,58)*ROcp34_833+ROcp34_532*s.dpt(2,58);
    RLcp34_334 = s.dpt(1,58)*ROcp34_333+s.dpt(3,58)*ROcp34_933+ROcp34_632*s.dpt(2,58);
    OMcp34_134 = OMcp34_133+ROcp34_733*qd(34);
    OMcp34_234 = OMcp34_233+ROcp34_833*qd(34);
    OMcp34_334 = OMcp34_333+ROcp34_933*qd(34);
    ORcp34_134 = OMcp34_233*RLcp34_334-OMcp34_333*RLcp34_234;
    ORcp34_234 = -(OMcp34_133*RLcp34_334-OMcp34_333*RLcp34_134);
    ORcp34_334 = OMcp34_133*RLcp34_234-OMcp34_233*RLcp34_134;
    OPcp34_134 = OPcp34_133+ROcp34_733*qdd(34)+qd(34)*(OMcp34_233*ROcp34_933-OMcp34_333*ROcp34_833);
    OPcp34_234 = OPcp34_233+ROcp34_833*qdd(34)-qd(34)*(OMcp34_133*ROcp34_933-OMcp34_333*ROcp34_733);
    OPcp34_334 = OPcp34_333+ROcp34_933*qdd(34)+qd(34)*(OMcp34_133*ROcp34_833-OMcp34_233*ROcp34_733);
    RLcp34_135 = s.dpt(1,59)*ROcp34_134+s.dpt(2,59)*ROcp34_434+s.dpt(3,59)*ROcp34_733;
    RLcp34_235 = s.dpt(1,59)*ROcp34_234+s.dpt(2,59)*ROcp34_534+s.dpt(3,59)*ROcp34_833;
    RLcp34_335 = s.dpt(1,59)*ROcp34_334+s.dpt(2,59)*ROcp34_634+s.dpt(3,59)*ROcp34_933;
    POcp34_135 = RLcp34_119+RLcp34_120+RLcp34_121+RLcp34_129+RLcp34_130+RLcp34_131+RLcp34_132+RLcp34_133+RLcp34_134+RLcp34_135+q(1);
    POcp34_235 = RLcp34_219+RLcp34_220+RLcp34_221+RLcp34_229+RLcp34_230+RLcp34_231+RLcp34_232+RLcp34_233+RLcp34_234+RLcp34_235+q(2);
    POcp34_335 = RLcp34_319+RLcp34_320+RLcp34_321+RLcp34_329+RLcp34_330+RLcp34_331+RLcp34_332+RLcp34_333+RLcp34_334+RLcp34_335+q(3);
    OMcp34_135 = OMcp34_134+ROcp34_134*qd(35);
    OMcp34_235 = OMcp34_234+ROcp34_234*qd(35);
    OMcp34_335 = OMcp34_334+ROcp34_334*qd(35);
    ORcp34_135 = OMcp34_234*RLcp34_335-OMcp34_334*RLcp34_235;
    ORcp34_235 = -(OMcp34_134*RLcp34_335-OMcp34_334*RLcp34_135);
    ORcp34_335 = OMcp34_134*RLcp34_235-OMcp34_234*RLcp34_135;
    VIcp34_135 = ORcp34_119+ORcp34_120+ORcp34_121+ORcp34_129+ORcp34_130+ORcp34_131+ORcp34_132+ORcp34_133+ORcp34_134+ORcp34_135+qd(1);
    VIcp34_235 = ORcp34_219+ORcp34_220+ORcp34_221+ORcp34_229+ORcp34_230+ORcp34_231+ORcp34_232+ORcp34_233+ORcp34_234+ORcp34_235+qd(2);
    VIcp34_335 = ORcp34_319+ORcp34_320+ORcp34_321+ORcp34_329+ORcp34_330+ORcp34_331+ORcp34_332+ORcp34_333+ORcp34_334+ORcp34_335+qd(3);
    OPcp34_135 = OPcp34_134+ROcp34_134*qdd(35)+qd(35)*(OMcp34_234*ROcp34_334-OMcp34_334*ROcp34_234);
    OPcp34_235 = OPcp34_234+ROcp34_234*qdd(35)-qd(35)*(OMcp34_134*ROcp34_334-OMcp34_334*ROcp34_134);
    OPcp34_335 = OPcp34_334+ROcp34_334*qdd(35)+qd(35)*(OMcp34_134*ROcp34_234-OMcp34_234*ROcp34_134);
    ACcp34_135 = qdd(1)+OMcp34_219*ORcp34_320+OMcp34_220*ORcp34_321+OMcp34_221*ORcp34_329+OMcp34_229*ORcp34_330+OMcp34_230*ORcp34_331+OMcp34_231*...
 ORcp34_332+OMcp34_232*ORcp34_333+OMcp34_233*ORcp34_334+OMcp34_234*ORcp34_335+OMcp34_26*ORcp34_319-OMcp34_319*ORcp34_220-OMcp34_320*ORcp34_221-...
 OMcp34_321*ORcp34_229-OMcp34_329*ORcp34_230-OMcp34_330*ORcp34_231-OMcp34_331*ORcp34_232-OMcp34_332*ORcp34_233-OMcp34_333*ORcp34_234-OMcp34_334*...
 ORcp34_235-OMcp34_36*ORcp34_219+OPcp34_219*RLcp34_320+OPcp34_220*RLcp34_321+OPcp34_221*RLcp34_329+OPcp34_229*RLcp34_330+OPcp34_230*RLcp34_331+...
 OPcp34_231*RLcp34_332+OPcp34_232*RLcp34_333+OPcp34_233*RLcp34_334+OPcp34_234*RLcp34_335+OPcp34_26*RLcp34_319-OPcp34_319*RLcp34_220-OPcp34_320*...
 RLcp34_221-OPcp34_321*RLcp34_229-OPcp34_329*RLcp34_230-OPcp34_330*RLcp34_231-OPcp34_331*RLcp34_232-OPcp34_332*RLcp34_233-OPcp34_333*RLcp34_234-...
 OPcp34_334*RLcp34_235-OPcp34_36*RLcp34_219;
    ACcp34_235 = qdd(2)-OMcp34_119*ORcp34_320-OMcp34_120*ORcp34_321-OMcp34_121*ORcp34_329-OMcp34_129*ORcp34_330-OMcp34_130*ORcp34_331-OMcp34_131*...
 ORcp34_332-OMcp34_132*ORcp34_333-OMcp34_133*ORcp34_334-OMcp34_134*ORcp34_335-OMcp34_16*ORcp34_319+OMcp34_319*ORcp34_120+OMcp34_320*ORcp34_121+...
 OMcp34_321*ORcp34_129+OMcp34_329*ORcp34_130+OMcp34_330*ORcp34_131+OMcp34_331*ORcp34_132+OMcp34_332*ORcp34_133+OMcp34_333*ORcp34_134+OMcp34_334*...
 ORcp34_135+OMcp34_36*ORcp34_119-OPcp34_119*RLcp34_320-OPcp34_120*RLcp34_321-OPcp34_121*RLcp34_329-OPcp34_129*RLcp34_330-OPcp34_130*RLcp34_331-...
 OPcp34_131*RLcp34_332-OPcp34_132*RLcp34_333-OPcp34_133*RLcp34_334-OPcp34_134*RLcp34_335-OPcp34_16*RLcp34_319+OPcp34_319*RLcp34_120+OPcp34_320*...
 RLcp34_121+OPcp34_321*RLcp34_129+OPcp34_329*RLcp34_130+OPcp34_330*RLcp34_131+OPcp34_331*RLcp34_132+OPcp34_332*RLcp34_133+OPcp34_333*RLcp34_134+...
 OPcp34_334*RLcp34_135+OPcp34_36*RLcp34_119;
    ACcp34_335 = qdd(3)+OMcp34_119*ORcp34_220+OMcp34_120*ORcp34_221+OMcp34_121*ORcp34_229+OMcp34_129*ORcp34_230+OMcp34_130*ORcp34_231+OMcp34_131*...
 ORcp34_232+OMcp34_132*ORcp34_233+OMcp34_133*ORcp34_234+OMcp34_134*ORcp34_235+OMcp34_16*ORcp34_219-OMcp34_219*ORcp34_120-OMcp34_220*ORcp34_121-...
 OMcp34_221*ORcp34_129-OMcp34_229*ORcp34_130-OMcp34_230*ORcp34_131-OMcp34_231*ORcp34_132-OMcp34_232*ORcp34_133-OMcp34_233*ORcp34_134-OMcp34_234*...
 ORcp34_135-OMcp34_26*ORcp34_119+OPcp34_119*RLcp34_220+OPcp34_120*RLcp34_221+OPcp34_121*RLcp34_229+OPcp34_129*RLcp34_230+OPcp34_130*RLcp34_231+...
 OPcp34_131*RLcp34_232+OPcp34_132*RLcp34_233+OPcp34_133*RLcp34_234+OPcp34_134*RLcp34_235+OPcp34_16*RLcp34_219-OPcp34_219*RLcp34_120-OPcp34_220*...
 RLcp34_121-OPcp34_221*RLcp34_129-OPcp34_229*RLcp34_130-OPcp34_230*RLcp34_131-OPcp34_231*RLcp34_132-OPcp34_232*RLcp34_133-OPcp34_233*RLcp34_134-...
 OPcp34_234*RLcp34_135-OPcp34_26*RLcp34_119;

% = = Block_1_0_0_35_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp34_135;
    sens.P(2) = POcp34_235;
    sens.P(3) = POcp34_335;
    sens.R(1,1) = ROcp34_134;
    sens.R(1,2) = ROcp34_234;
    sens.R(1,3) = ROcp34_334;
    sens.R(2,1) = ROcp34_435;
    sens.R(2,2) = ROcp34_535;
    sens.R(2,3) = ROcp34_635;
    sens.R(3,1) = ROcp34_735;
    sens.R(3,2) = ROcp34_835;
    sens.R(3,3) = ROcp34_935;
    sens.V(1) = VIcp34_135;
    sens.V(2) = VIcp34_235;
    sens.V(3) = VIcp34_335;
    sens.OM(1) = OMcp34_135;
    sens.OM(2) = OMcp34_235;
    sens.OM(3) = OMcp34_335;
    sens.A(1) = ACcp34_135;
    sens.A(2) = ACcp34_235;
    sens.A(3) = ACcp34_335;
    sens.OMP(1) = OPcp34_135;
    sens.OMP(2) = OPcp34_235;
    sens.OMP(3) = OPcp34_335;

end


% ====== END Task 1 ====== 

  

