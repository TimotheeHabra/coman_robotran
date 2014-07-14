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
%	==> Generation Date : Wed Mar  5 10:45:08 2014
%
%	==> Project name : coman_robotran
%	==> using XML input file 
%
%	==> Number of joints : 31
%
%	==> Function : F 6 : Sensors Kinematical Informations (sens) 
%	==> Flops complexity : 32640
%
%	==> Generation Time :  0.490 seconds
%	==> Post-Processing :  0.820 seconds
%
%-------------------------------------------------------------
%
function [sens] = sensor(s,tsim,usrfun,isens)

 sens.P = zeros(3,1);
 sens.R = zeros(3,3);
 sens.V = zeros(3,1);
 sens.OM = zeros(3,1);
 sens.A = zeros(3,1);
 sens.OMP = zeros(3,1);
 sens.J = zeros(6,31);

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
  C13 = cos(q(13));
  S13 = sin(q(13));

% = = Block_0_0_0_0_0_3 = = 
 
% Trigonometric Variables  

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
  C19 = cos(q(19));
  S19 = sin(q(19));
  C20 = cos(q(20));
  S20 = sin(q(20));

% = = Block_0_0_0_0_0_4 = = 
 
% Trigonometric Variables  

  C21 = cos(q(21));
  S21 = sin(q(21));
  C22 = cos(q(22));
  S22 = sin(q(22));
  C23 = cos(q(23));
  S23 = sin(q(23));

% = = Block_0_0_0_0_0_5 = = 
 
% Trigonometric Variables  

  C24 = cos(q(24));
  S24 = sin(q(24));
  C25 = cos(q(25));
  S25 = sin(q(25));
  C26 = cos(q(26));
  S26 = sin(q(26));
  C27 = cos(q(27));
  S27 = sin(q(27));

% = = Block_0_0_0_0_0_6 = = 
 
% Trigonometric Variables  

  C28 = cos(q(28));
  S28 = sin(q(28));
  C29 = cos(q(29));
  S29 = sin(q(29));
  C30 = cos(q(30));
  S30 = sin(q(30));
  C31 = cos(q(31));
  S31 = sin(q(31));

% ====== END Task 0 ====== 

% ===== BEGIN task 1 ===== 
 
switch isens

 
% 
case 1, 


% = = Block_1_0_0_1_0_1 = = 
 
% Sensor Kinematics 


    ROcp0_25 = S4*S5;
    ROcp0_35 = -C4*S5;
    ROcp0_85 = -S4*C5;
    ROcp0_95 = C4*C5;
    ROcp0_16 = C5*C6;
    ROcp0_26 = ROcp0_25*C6+C4*S6;
    ROcp0_36 = ROcp0_35*C6+S4*S6;
    ROcp0_46 = -C5*S6;
    ROcp0_56 = -(ROcp0_25*S6-C4*C6);
    ROcp0_66 = -(ROcp0_35*S6-S4*C6);
    OMcp0_25 = qd(5)*C4;
    OMcp0_35 = qd(5)*S4;
    OMcp0_16 = qd(4)+qd(6)*S5;
    OMcp0_26 = OMcp0_25+ROcp0_85*qd(6);
    OMcp0_36 = OMcp0_35+ROcp0_95*qd(6);
    OPcp0_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp0_26 = ROcp0_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp0_35*S5-ROcp0_95*qd(4));
    OPcp0_36 = ROcp0_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp0_25*S5-ROcp0_85*qd(4));
    RLcp0_132 = ROcp0_46*s.dpt(2,1);
    RLcp0_232 = ROcp0_56*s.dpt(2,1);
    RLcp0_332 = ROcp0_66*s.dpt(2,1);
    POcp0_132 = RLcp0_132+q(1);
    POcp0_232 = RLcp0_232+q(2);
    POcp0_332 = RLcp0_332+q(3);
    JTcp0_132_5 = -(RLcp0_232*S4-RLcp0_332*C4);
    JTcp0_232_5 = RLcp0_132*S4;
    JTcp0_332_5 = -RLcp0_132*C4;
    JTcp0_132_6 = -(RLcp0_232*ROcp0_95-RLcp0_332*ROcp0_85);
    JTcp0_232_6 = RLcp0_132*ROcp0_95-RLcp0_332*S5;
    JTcp0_332_6 = -(RLcp0_132*ROcp0_85-RLcp0_232*S5);
    ORcp0_132 = OMcp0_26*RLcp0_332-OMcp0_36*RLcp0_232;
    ORcp0_232 = -(OMcp0_16*RLcp0_332-OMcp0_36*RLcp0_132);
    ORcp0_332 = OMcp0_16*RLcp0_232-OMcp0_26*RLcp0_132;
    VIcp0_132 = ORcp0_132+qd(1);
    VIcp0_232 = ORcp0_232+qd(2);
    VIcp0_332 = ORcp0_332+qd(3);
    ACcp0_132 = qdd(1)+OMcp0_26*ORcp0_332-OMcp0_36*ORcp0_232+OPcp0_26*RLcp0_332-OPcp0_36*RLcp0_232;
    ACcp0_232 = qdd(2)-OMcp0_16*ORcp0_332+OMcp0_36*ORcp0_132-OPcp0_16*RLcp0_332+OPcp0_36*RLcp0_132;
    ACcp0_332 = qdd(3)+OMcp0_16*ORcp0_232-OMcp0_26*ORcp0_132+OPcp0_16*RLcp0_232-OPcp0_26*RLcp0_132;

% = = Block_1_0_0_1_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp0_132;
    sens.P(2) = POcp0_232;
    sens.P(3) = POcp0_332;
    sens.R(1,1) = ROcp0_16;
    sens.R(1,2) = ROcp0_26;
    sens.R(1,3) = ROcp0_36;
    sens.R(2,1) = ROcp0_46;
    sens.R(2,2) = ROcp0_56;
    sens.R(2,3) = ROcp0_66;
    sens.R(3,1) = S5;
    sens.R(3,2) = ROcp0_85;
    sens.R(3,3) = ROcp0_95;
    sens.V(1) = VIcp0_132;
    sens.V(2) = VIcp0_232;
    sens.V(3) = VIcp0_332;
    sens.OM(1) = OMcp0_16;
    sens.OM(2) = OMcp0_26;
    sens.OM(3) = OMcp0_36;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp0_132_5;
    sens.J(1,6) = JTcp0_132_6;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = -RLcp0_332;
    sens.J(2,5) = JTcp0_232_5;
    sens.J(2,6) = JTcp0_232_6;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = RLcp0_232;
    sens.J(3,5) = JTcp0_332_5;
    sens.J(3,6) = JTcp0_332_6;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp0_85;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp0_95;
    sens.A(1) = ACcp0_132;
    sens.A(2) = ACcp0_232;
    sens.A(3) = ACcp0_332;
    sens.OMP(1) = OPcp0_16;
    sens.OMP(2) = OPcp0_26;
    sens.OMP(3) = OPcp0_36;
 
% 
case 2, 


% = = Block_1_0_0_2_0_1 = = 
 
% Sensor Kinematics 


    ROcp1_25 = S4*S5;
    ROcp1_35 = -C4*S5;
    ROcp1_85 = -S4*C5;
    ROcp1_95 = C4*C5;
    ROcp1_16 = C5*C6;
    ROcp1_26 = ROcp1_25*C6+C4*S6;
    ROcp1_36 = ROcp1_35*C6+S4*S6;
    ROcp1_46 = -C5*S6;
    ROcp1_56 = -(ROcp1_25*S6-C4*C6);
    ROcp1_66 = -(ROcp1_35*S6-S4*C6);
    OMcp1_25 = qd(5)*C4;
    OMcp1_35 = qd(5)*S4;
    OMcp1_16 = qd(4)+qd(6)*S5;
    OMcp1_26 = OMcp1_25+ROcp1_85*qd(6);
    OMcp1_36 = OMcp1_35+ROcp1_95*qd(6);
    OPcp1_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp1_26 = ROcp1_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp1_35*S5-ROcp1_95*qd(4));
    OPcp1_36 = ROcp1_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp1_25*S5-ROcp1_85*qd(4));
    RLcp1_133 = ROcp1_46*s.dpt(2,2);
    RLcp1_233 = ROcp1_56*s.dpt(2,2);
    RLcp1_333 = ROcp1_66*s.dpt(2,2);
    POcp1_133 = RLcp1_133+q(1);
    POcp1_233 = RLcp1_233+q(2);
    POcp1_333 = RLcp1_333+q(3);
    JTcp1_133_5 = -(RLcp1_233*S4-RLcp1_333*C4);
    JTcp1_233_5 = RLcp1_133*S4;
    JTcp1_333_5 = -RLcp1_133*C4;
    JTcp1_133_6 = -(RLcp1_233*ROcp1_95-RLcp1_333*ROcp1_85);
    JTcp1_233_6 = RLcp1_133*ROcp1_95-RLcp1_333*S5;
    JTcp1_333_6 = -(RLcp1_133*ROcp1_85-RLcp1_233*S5);
    ORcp1_133 = OMcp1_26*RLcp1_333-OMcp1_36*RLcp1_233;
    ORcp1_233 = -(OMcp1_16*RLcp1_333-OMcp1_36*RLcp1_133);
    ORcp1_333 = OMcp1_16*RLcp1_233-OMcp1_26*RLcp1_133;
    VIcp1_133 = ORcp1_133+qd(1);
    VIcp1_233 = ORcp1_233+qd(2);
    VIcp1_333 = ORcp1_333+qd(3);
    ACcp1_133 = qdd(1)+OMcp1_26*ORcp1_333-OMcp1_36*ORcp1_233+OPcp1_26*RLcp1_333-OPcp1_36*RLcp1_233;
    ACcp1_233 = qdd(2)-OMcp1_16*ORcp1_333+OMcp1_36*ORcp1_133-OPcp1_16*RLcp1_333+OPcp1_36*RLcp1_133;
    ACcp1_333 = qdd(3)+OMcp1_16*ORcp1_233-OMcp1_26*ORcp1_133+OPcp1_16*RLcp1_233-OPcp1_26*RLcp1_133;

% = = Block_1_0_0_2_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp1_133;
    sens.P(2) = POcp1_233;
    sens.P(3) = POcp1_333;
    sens.R(1,1) = ROcp1_16;
    sens.R(1,2) = ROcp1_26;
    sens.R(1,3) = ROcp1_36;
    sens.R(2,1) = ROcp1_46;
    sens.R(2,2) = ROcp1_56;
    sens.R(2,3) = ROcp1_66;
    sens.R(3,1) = S5;
    sens.R(3,2) = ROcp1_85;
    sens.R(3,3) = ROcp1_95;
    sens.V(1) = VIcp1_133;
    sens.V(2) = VIcp1_233;
    sens.V(3) = VIcp1_333;
    sens.OM(1) = OMcp1_16;
    sens.OM(2) = OMcp1_26;
    sens.OM(3) = OMcp1_36;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp1_133_5;
    sens.J(1,6) = JTcp1_133_6;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = -RLcp1_333;
    sens.J(2,5) = JTcp1_233_5;
    sens.J(2,6) = JTcp1_233_6;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = RLcp1_233;
    sens.J(3,5) = JTcp1_333_5;
    sens.J(3,6) = JTcp1_333_6;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp1_85;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp1_95;
    sens.A(1) = ACcp1_133;
    sens.A(2) = ACcp1_233;
    sens.A(3) = ACcp1_333;
    sens.OMP(1) = OPcp1_16;
    sens.OMP(2) = OPcp1_26;
    sens.OMP(3) = OPcp1_36;
 
% 
case 3, 


% = = Block_1_0_0_3_0_1 = = 
 
% Sensor Kinematics 


    ROcp2_25 = S4*S5;
    ROcp2_35 = -C4*S5;
    ROcp2_85 = -S4*C5;
    ROcp2_95 = C4*C5;
    ROcp2_16 = C5*C6;
    ROcp2_26 = ROcp2_25*C6+C4*S6;
    ROcp2_36 = ROcp2_35*C6+S4*S6;
    ROcp2_46 = -C5*S6;
    ROcp2_56 = -(ROcp2_25*S6-C4*C6);
    ROcp2_66 = -(ROcp2_35*S6-S4*C6);
    OMcp2_25 = qd(5)*C4;
    OMcp2_35 = qd(5)*S4;
    OMcp2_16 = qd(4)+qd(6)*S5;
    OMcp2_26 = OMcp2_25+ROcp2_85*qd(6);
    OMcp2_36 = OMcp2_35+ROcp2_95*qd(6);
    OPcp2_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp2_26 = ROcp2_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp2_35*S5-ROcp2_95*qd(4));
    OPcp2_36 = ROcp2_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp2_25*S5-ROcp2_85*qd(4));
    RLcp2_134 = ROcp2_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp2_234 = ROcp2_26*s.dpt(1,3)+ROcp2_85*s.dpt(3,3);
    RLcp2_334 = ROcp2_36*s.dpt(1,3)+ROcp2_95*s.dpt(3,3);
    POcp2_134 = RLcp2_134+q(1);
    POcp2_234 = RLcp2_234+q(2);
    POcp2_334 = RLcp2_334+q(3);
    JTcp2_134_5 = -(RLcp2_234*S4-RLcp2_334*C4);
    JTcp2_234_5 = RLcp2_134*S4;
    JTcp2_334_5 = -RLcp2_134*C4;
    JTcp2_134_6 = -(RLcp2_234*ROcp2_95-RLcp2_334*ROcp2_85);
    JTcp2_234_6 = RLcp2_134*ROcp2_95-RLcp2_334*S5;
    JTcp2_334_6 = -(RLcp2_134*ROcp2_85-RLcp2_234*S5);
    ORcp2_134 = OMcp2_26*RLcp2_334-OMcp2_36*RLcp2_234;
    ORcp2_234 = -(OMcp2_16*RLcp2_334-OMcp2_36*RLcp2_134);
    ORcp2_334 = OMcp2_16*RLcp2_234-OMcp2_26*RLcp2_134;
    VIcp2_134 = ORcp2_134+qd(1);
    VIcp2_234 = ORcp2_234+qd(2);
    VIcp2_334 = ORcp2_334+qd(3);
    ACcp2_134 = qdd(1)+OMcp2_26*ORcp2_334-OMcp2_36*ORcp2_234+OPcp2_26*RLcp2_334-OPcp2_36*RLcp2_234;
    ACcp2_234 = qdd(2)-OMcp2_16*ORcp2_334+OMcp2_36*ORcp2_134-OPcp2_16*RLcp2_334+OPcp2_36*RLcp2_134;
    ACcp2_334 = qdd(3)+OMcp2_16*ORcp2_234-OMcp2_26*ORcp2_134+OPcp2_16*RLcp2_234-OPcp2_26*RLcp2_134;

% = = Block_1_0_0_3_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp2_134;
    sens.P(2) = POcp2_234;
    sens.P(3) = POcp2_334;
    sens.R(1,1) = ROcp2_16;
    sens.R(1,2) = ROcp2_26;
    sens.R(1,3) = ROcp2_36;
    sens.R(2,1) = ROcp2_46;
    sens.R(2,2) = ROcp2_56;
    sens.R(2,3) = ROcp2_66;
    sens.R(3,1) = S5;
    sens.R(3,2) = ROcp2_85;
    sens.R(3,3) = ROcp2_95;
    sens.V(1) = VIcp2_134;
    sens.V(2) = VIcp2_234;
    sens.V(3) = VIcp2_334;
    sens.OM(1) = OMcp2_16;
    sens.OM(2) = OMcp2_26;
    sens.OM(3) = OMcp2_36;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp2_134_5;
    sens.J(1,6) = JTcp2_134_6;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = -RLcp2_334;
    sens.J(2,5) = JTcp2_234_5;
    sens.J(2,6) = JTcp2_234_6;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = RLcp2_234;
    sens.J(3,5) = JTcp2_334_5;
    sens.J(3,6) = JTcp2_334_6;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp2_85;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp2_95;
    sens.A(1) = ACcp2_134;
    sens.A(2) = ACcp2_234;
    sens.A(3) = ACcp2_334;
    sens.OMP(1) = OPcp2_16;
    sens.OMP(2) = OPcp2_26;
    sens.OMP(3) = OPcp2_36;
 
% 
case 4, 


% = = Block_1_0_0_4_0_1 = = 
 
% Sensor Kinematics 


    ROcp3_25 = S4*S5;
    ROcp3_35 = -C4*S5;
    ROcp3_85 = -S4*C5;
    ROcp3_95 = C4*C5;
    ROcp3_16 = C5*C6;
    ROcp3_26 = ROcp3_25*C6+C4*S6;
    ROcp3_36 = ROcp3_35*C6+S4*S6;
    ROcp3_46 = -C5*S6;
    ROcp3_56 = -(ROcp3_25*S6-C4*C6);
    ROcp3_66 = -(ROcp3_35*S6-S4*C6);
    OMcp3_25 = qd(5)*C4;
    OMcp3_35 = qd(5)*S4;
    OMcp3_16 = qd(4)+qd(6)*S5;
    OMcp3_26 = OMcp3_25+ROcp3_85*qd(6);
    OMcp3_36 = OMcp3_35+ROcp3_95*qd(6);
    OPcp3_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp3_26 = ROcp3_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp3_35*S5-ROcp3_95*qd(4));
    OPcp3_36 = ROcp3_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp3_25*S5-ROcp3_85*qd(4));
    RLcp3_135 = ROcp3_16*s.dpt(1,4)+ROcp3_46*s.dpt(2,4)+s.dpt(3,4)*S5;
    RLcp3_235 = ROcp3_26*s.dpt(1,4)+ROcp3_56*s.dpt(2,4)+ROcp3_85*s.dpt(3,4);
    RLcp3_335 = ROcp3_36*s.dpt(1,4)+ROcp3_66*s.dpt(2,4)+ROcp3_95*s.dpt(3,4);
    POcp3_135 = RLcp3_135+q(1);
    POcp3_235 = RLcp3_235+q(2);
    POcp3_335 = RLcp3_335+q(3);
    JTcp3_135_5 = -(RLcp3_235*S4-RLcp3_335*C4);
    JTcp3_235_5 = RLcp3_135*S4;
    JTcp3_335_5 = -RLcp3_135*C4;
    JTcp3_135_6 = -(RLcp3_235*ROcp3_95-RLcp3_335*ROcp3_85);
    JTcp3_235_6 = RLcp3_135*ROcp3_95-RLcp3_335*S5;
    JTcp3_335_6 = -(RLcp3_135*ROcp3_85-RLcp3_235*S5);
    ORcp3_135 = OMcp3_26*RLcp3_335-OMcp3_36*RLcp3_235;
    ORcp3_235 = -(OMcp3_16*RLcp3_335-OMcp3_36*RLcp3_135);
    ORcp3_335 = OMcp3_16*RLcp3_235-OMcp3_26*RLcp3_135;
    VIcp3_135 = ORcp3_135+qd(1);
    VIcp3_235 = ORcp3_235+qd(2);
    VIcp3_335 = ORcp3_335+qd(3);
    ACcp3_135 = qdd(1)+OMcp3_26*ORcp3_335-OMcp3_36*ORcp3_235+OPcp3_26*RLcp3_335-OPcp3_36*RLcp3_235;
    ACcp3_235 = qdd(2)-OMcp3_16*ORcp3_335+OMcp3_36*ORcp3_135-OPcp3_16*RLcp3_335+OPcp3_36*RLcp3_135;
    ACcp3_335 = qdd(3)+OMcp3_16*ORcp3_235-OMcp3_26*ORcp3_135+OPcp3_16*RLcp3_235-OPcp3_26*RLcp3_135;

% = = Block_1_0_0_4_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp3_135;
    sens.P(2) = POcp3_235;
    sens.P(3) = POcp3_335;
    sens.R(1,1) = ROcp3_16;
    sens.R(1,2) = ROcp3_26;
    sens.R(1,3) = ROcp3_36;
    sens.R(2,1) = ROcp3_46;
    sens.R(2,2) = ROcp3_56;
    sens.R(2,3) = ROcp3_66;
    sens.R(3,1) = S5;
    sens.R(3,2) = ROcp3_85;
    sens.R(3,3) = ROcp3_95;
    sens.V(1) = VIcp3_135;
    sens.V(2) = VIcp3_235;
    sens.V(3) = VIcp3_335;
    sens.OM(1) = OMcp3_16;
    sens.OM(2) = OMcp3_26;
    sens.OM(3) = OMcp3_36;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp3_135_5;
    sens.J(1,6) = JTcp3_135_6;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = -RLcp3_335;
    sens.J(2,5) = JTcp3_235_5;
    sens.J(2,6) = JTcp3_235_6;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = RLcp3_235;
    sens.J(3,5) = JTcp3_335_5;
    sens.J(3,6) = JTcp3_335_6;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp3_85;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp3_95;
    sens.A(1) = ACcp3_135;
    sens.A(2) = ACcp3_235;
    sens.A(3) = ACcp3_335;
    sens.OMP(1) = OPcp3_16;
    sens.OMP(2) = OPcp3_26;
    sens.OMP(3) = OPcp3_36;
 
% 
case 5, 


% = = Block_1_0_0_5_0_1 = = 
 
% Sensor Kinematics 


    ROcp4_25 = S4*S5;
    ROcp4_35 = -C4*S5;
    ROcp4_85 = -S4*C5;
    ROcp4_95 = C4*C5;
    ROcp4_16 = C5*C6;
    ROcp4_26 = ROcp4_25*C6+C4*S6;
    ROcp4_36 = ROcp4_35*C6+S4*S6;
    ROcp4_46 = -C5*S6;
    ROcp4_56 = -(ROcp4_25*S6-C4*C6);
    ROcp4_66 = -(ROcp4_35*S6-S4*C6);
    OMcp4_25 = qd(5)*C4;
    OMcp4_35 = qd(5)*S4;
    OMcp4_16 = qd(4)+qd(6)*S5;
    OMcp4_26 = OMcp4_25+ROcp4_85*qd(6);
    OMcp4_36 = OMcp4_35+ROcp4_95*qd(6);
    OPcp4_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp4_26 = ROcp4_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp4_35*S5-ROcp4_95*qd(4));
    OPcp4_36 = ROcp4_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp4_25*S5-ROcp4_85*qd(4));

% = = Block_1_0_0_5_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = q(1);
    sens.P(2) = q(2);
    sens.P(3) = q(3);
    sens.R(1,1) = ROcp4_16;
    sens.R(1,2) = ROcp4_26;
    sens.R(1,3) = ROcp4_36;
    sens.R(2,1) = ROcp4_46;
    sens.R(2,2) = ROcp4_56;
    sens.R(2,3) = ROcp4_66;
    sens.R(3,1) = S5;
    sens.R(3,2) = ROcp4_85;
    sens.R(3,3) = ROcp4_95;
    sens.V(1) = qd(1);
    sens.V(2) = qd(2);
    sens.V(3) = qd(3);
    sens.OM(1) = OMcp4_16;
    sens.OM(2) = OMcp4_26;
    sens.OM(3) = OMcp4_36;
    sens.J(1,1) = (1.0);
    sens.J(2,2) = (1.0);
    sens.J(3,3) = (1.0);
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp4_85;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp4_95;
    sens.A(1) = qdd(1);
    sens.A(2) = qdd(2);
    sens.A(3) = qdd(3);
    sens.OMP(1) = OPcp4_16;
    sens.OMP(2) = OPcp4_26;
    sens.OMP(3) = OPcp4_36;
 
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

% = = Block_1_0_0_6_0_2 = = 
 
% Sensor Kinematics 


    ROcp5_17 = ROcp5_16*C7-S5*S7;
    ROcp5_27 = ROcp5_26*C7-ROcp5_85*S7;
    ROcp5_37 = ROcp5_36*C7-ROcp5_95*S7;
    ROcp5_77 = ROcp5_16*S7+S5*C7;
    ROcp5_87 = ROcp5_26*S7+ROcp5_85*C7;
    ROcp5_97 = ROcp5_36*S7+ROcp5_95*C7;
    RLcp5_17 = ROcp5_46*s.dpt(2,1);
    RLcp5_27 = ROcp5_56*s.dpt(2,1);
    RLcp5_37 = ROcp5_66*s.dpt(2,1);
    OMcp5_17 = OMcp5_16+ROcp5_46*qd(7);
    OMcp5_27 = OMcp5_26+ROcp5_56*qd(7);
    OMcp5_37 = OMcp5_36+ROcp5_66*qd(7);
    ORcp5_17 = OMcp5_26*RLcp5_37-OMcp5_36*RLcp5_27;
    ORcp5_27 = -(OMcp5_16*RLcp5_37-OMcp5_36*RLcp5_17);
    ORcp5_37 = OMcp5_16*RLcp5_27-OMcp5_26*RLcp5_17;
    OPcp5_17 = OPcp5_16+ROcp5_46*qdd(7)+qd(7)*(OMcp5_26*ROcp5_66-OMcp5_36*ROcp5_56);
    OPcp5_27 = OPcp5_26+ROcp5_56*qdd(7)-qd(7)*(OMcp5_16*ROcp5_66-OMcp5_36*ROcp5_46);
    OPcp5_37 = OPcp5_36+ROcp5_66*qdd(7)+qd(7)*(OMcp5_16*ROcp5_56-OMcp5_26*ROcp5_46);
    RLcp5_137 = ROcp5_46*s.dpt(2,6);
    RLcp5_237 = ROcp5_56*s.dpt(2,6);
    RLcp5_337 = ROcp5_66*s.dpt(2,6);
    POcp5_137 = RLcp5_137+RLcp5_17+q(1);
    POcp5_237 = RLcp5_237+RLcp5_27+q(2);
    POcp5_337 = RLcp5_337+RLcp5_37+q(3);
    JTcp5_237_4 = -(RLcp5_337+RLcp5_37);
    JTcp5_337_4 = RLcp5_237+RLcp5_27;
    JTcp5_137_5 = C4*(RLcp5_337+RLcp5_37)-S4*(RLcp5_237+RLcp5_27);
    JTcp5_237_5 = S4*(RLcp5_137+RLcp5_17);
    JTcp5_337_5 = -C4*(RLcp5_137+RLcp5_17);
    JTcp5_137_6 = ROcp5_85*(RLcp5_337+RLcp5_37)-ROcp5_95*(RLcp5_237+RLcp5_27);
    JTcp5_237_6 = ROcp5_95*(RLcp5_137+RLcp5_17)-S5*(RLcp5_337+RLcp5_37);
    JTcp5_337_6 = -(ROcp5_85*(RLcp5_137+RLcp5_17)-S5*(RLcp5_237+RLcp5_27));
    JTcp5_137_7 = -(RLcp5_237*ROcp5_66-RLcp5_337*ROcp5_56);
    JTcp5_237_7 = RLcp5_137*ROcp5_66-RLcp5_337*ROcp5_46;
    JTcp5_337_7 = -(RLcp5_137*ROcp5_56-RLcp5_237*ROcp5_46);
    ORcp5_137 = OMcp5_27*RLcp5_337-OMcp5_37*RLcp5_237;
    ORcp5_237 = -(OMcp5_17*RLcp5_337-OMcp5_37*RLcp5_137);
    ORcp5_337 = OMcp5_17*RLcp5_237-OMcp5_27*RLcp5_137;
    VIcp5_137 = ORcp5_137+ORcp5_17+qd(1);
    VIcp5_237 = ORcp5_237+ORcp5_27+qd(2);
    VIcp5_337 = ORcp5_337+ORcp5_37+qd(3);
    ACcp5_137 = qdd(1)+OMcp5_26*ORcp5_37+OMcp5_27*ORcp5_337-OMcp5_36*ORcp5_27-OMcp5_37*ORcp5_237+OPcp5_26*RLcp5_37+OPcp5_27*RLcp5_337-OPcp5_36*...
 RLcp5_27-OPcp5_37*RLcp5_237;
    ACcp5_237 = qdd(2)-OMcp5_16*ORcp5_37-OMcp5_17*ORcp5_337+OMcp5_36*ORcp5_17+OMcp5_37*ORcp5_137-OPcp5_16*RLcp5_37-OPcp5_17*RLcp5_337+OPcp5_36*...
 RLcp5_17+OPcp5_37*RLcp5_137;
    ACcp5_337 = qdd(3)+OMcp5_16*ORcp5_27+OMcp5_17*ORcp5_237-OMcp5_26*ORcp5_17-OMcp5_27*ORcp5_137+OPcp5_16*RLcp5_27+OPcp5_17*RLcp5_237-OPcp5_26*...
 RLcp5_17-OPcp5_27*RLcp5_137;

% = = Block_1_0_0_6_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp5_137;
    sens.P(2) = POcp5_237;
    sens.P(3) = POcp5_337;
    sens.R(1,1) = ROcp5_17;
    sens.R(1,2) = ROcp5_27;
    sens.R(1,3) = ROcp5_37;
    sens.R(2,1) = ROcp5_46;
    sens.R(2,2) = ROcp5_56;
    sens.R(2,3) = ROcp5_66;
    sens.R(3,1) = ROcp5_77;
    sens.R(3,2) = ROcp5_87;
    sens.R(3,3) = ROcp5_97;
    sens.V(1) = VIcp5_137;
    sens.V(2) = VIcp5_237;
    sens.V(3) = VIcp5_337;
    sens.OM(1) = OMcp5_17;
    sens.OM(2) = OMcp5_27;
    sens.OM(3) = OMcp5_37;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp5_137_5;
    sens.J(1,6) = JTcp5_137_6;
    sens.J(1,7) = JTcp5_137_7;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp5_237_4;
    sens.J(2,5) = JTcp5_237_5;
    sens.J(2,6) = JTcp5_237_6;
    sens.J(2,7) = JTcp5_237_7;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp5_337_4;
    sens.J(3,5) = JTcp5_337_5;
    sens.J(3,6) = JTcp5_337_6;
    sens.J(3,7) = JTcp5_337_7;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,7) = ROcp5_46;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp5_85;
    sens.J(5,7) = ROcp5_56;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp5_95;
    sens.J(6,7) = ROcp5_66;
    sens.A(1) = ACcp5_137;
    sens.A(2) = ACcp5_237;
    sens.A(3) = ACcp5_337;
    sens.OMP(1) = OPcp5_17;
    sens.OMP(2) = OPcp5_27;
    sens.OMP(3) = OPcp5_37;
 
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
    RLcp6_17 = ROcp6_46*s.dpt(2,1);
    RLcp6_27 = ROcp6_56*s.dpt(2,1);
    RLcp6_37 = ROcp6_66*s.dpt(2,1);
    OMcp6_17 = OMcp6_16+ROcp6_46*qd(7);
    OMcp6_27 = OMcp6_26+ROcp6_56*qd(7);
    OMcp6_37 = OMcp6_36+ROcp6_66*qd(7);
    ORcp6_17 = OMcp6_26*RLcp6_37-OMcp6_36*RLcp6_27;
    ORcp6_27 = -(OMcp6_16*RLcp6_37-OMcp6_36*RLcp6_17);
    ORcp6_37 = OMcp6_16*RLcp6_27-OMcp6_26*RLcp6_17;
    OPcp6_17 = OPcp6_16+ROcp6_46*qdd(7)+qd(7)*(OMcp6_26*ROcp6_66-OMcp6_36*ROcp6_56);
    OPcp6_27 = OPcp6_26+ROcp6_56*qdd(7)-qd(7)*(OMcp6_16*ROcp6_66-OMcp6_36*ROcp6_46);
    OPcp6_37 = OPcp6_36+ROcp6_66*qdd(7)+qd(7)*(OMcp6_16*ROcp6_56-OMcp6_26*ROcp6_46);
    RLcp6_138 = ROcp6_17*s.dpt(1,7)+ROcp6_46*s.dpt(2,7)+ROcp6_77*s.dpt(3,7);
    RLcp6_238 = ROcp6_27*s.dpt(1,7)+ROcp6_56*s.dpt(2,7)+ROcp6_87*s.dpt(3,7);
    RLcp6_338 = ROcp6_37*s.dpt(1,7)+ROcp6_66*s.dpt(2,7)+ROcp6_97*s.dpt(3,7);
    POcp6_138 = RLcp6_138+RLcp6_17+q(1);
    POcp6_238 = RLcp6_238+RLcp6_27+q(2);
    POcp6_338 = RLcp6_338+RLcp6_37+q(3);
    JTcp6_238_4 = -(RLcp6_338+RLcp6_37);
    JTcp6_338_4 = RLcp6_238+RLcp6_27;
    JTcp6_138_5 = C4*(RLcp6_338+RLcp6_37)-S4*(RLcp6_238+RLcp6_27);
    JTcp6_238_5 = S4*(RLcp6_138+RLcp6_17);
    JTcp6_338_5 = -C4*(RLcp6_138+RLcp6_17);
    JTcp6_138_6 = ROcp6_85*(RLcp6_338+RLcp6_37)-ROcp6_95*(RLcp6_238+RLcp6_27);
    JTcp6_238_6 = ROcp6_95*(RLcp6_138+RLcp6_17)-S5*(RLcp6_338+RLcp6_37);
    JTcp6_338_6 = -(ROcp6_85*(RLcp6_138+RLcp6_17)-S5*(RLcp6_238+RLcp6_27));
    JTcp6_138_7 = -(RLcp6_238*ROcp6_66-RLcp6_338*ROcp6_56);
    JTcp6_238_7 = RLcp6_138*ROcp6_66-RLcp6_338*ROcp6_46;
    JTcp6_338_7 = -(RLcp6_138*ROcp6_56-RLcp6_238*ROcp6_46);
    ORcp6_138 = OMcp6_27*RLcp6_338-OMcp6_37*RLcp6_238;
    ORcp6_238 = -(OMcp6_17*RLcp6_338-OMcp6_37*RLcp6_138);
    ORcp6_338 = OMcp6_17*RLcp6_238-OMcp6_27*RLcp6_138;
    VIcp6_138 = ORcp6_138+ORcp6_17+qd(1);
    VIcp6_238 = ORcp6_238+ORcp6_27+qd(2);
    VIcp6_338 = ORcp6_338+ORcp6_37+qd(3);
    ACcp6_138 = qdd(1)+OMcp6_26*ORcp6_37+OMcp6_27*ORcp6_338-OMcp6_36*ORcp6_27-OMcp6_37*ORcp6_238+OPcp6_26*RLcp6_37+OPcp6_27*RLcp6_338-OPcp6_36*...
 RLcp6_27-OPcp6_37*RLcp6_238;
    ACcp6_238 = qdd(2)-OMcp6_16*ORcp6_37-OMcp6_17*ORcp6_338+OMcp6_36*ORcp6_17+OMcp6_37*ORcp6_138-OPcp6_16*RLcp6_37-OPcp6_17*RLcp6_338+OPcp6_36*...
 RLcp6_17+OPcp6_37*RLcp6_138;
    ACcp6_338 = qdd(3)+OMcp6_16*ORcp6_27+OMcp6_17*ORcp6_238-OMcp6_26*ORcp6_17-OMcp6_27*ORcp6_138+OPcp6_16*RLcp6_27+OPcp6_17*RLcp6_238-OPcp6_26*...
 RLcp6_17-OPcp6_27*RLcp6_138;

% = = Block_1_0_0_7_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp6_138;
    sens.P(2) = POcp6_238;
    sens.P(3) = POcp6_338;
    sens.R(1,1) = ROcp6_17;
    sens.R(1,2) = ROcp6_27;
    sens.R(1,3) = ROcp6_37;
    sens.R(2,1) = ROcp6_46;
    sens.R(2,2) = ROcp6_56;
    sens.R(2,3) = ROcp6_66;
    sens.R(3,1) = ROcp6_77;
    sens.R(3,2) = ROcp6_87;
    sens.R(3,3) = ROcp6_97;
    sens.V(1) = VIcp6_138;
    sens.V(2) = VIcp6_238;
    sens.V(3) = VIcp6_338;
    sens.OM(1) = OMcp6_17;
    sens.OM(2) = OMcp6_27;
    sens.OM(3) = OMcp6_37;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp6_138_5;
    sens.J(1,6) = JTcp6_138_6;
    sens.J(1,7) = JTcp6_138_7;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp6_238_4;
    sens.J(2,5) = JTcp6_238_5;
    sens.J(2,6) = JTcp6_238_6;
    sens.J(2,7) = JTcp6_238_7;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp6_338_4;
    sens.J(3,5) = JTcp6_338_5;
    sens.J(3,6) = JTcp6_338_6;
    sens.J(3,7) = JTcp6_338_7;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,7) = ROcp6_46;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp6_85;
    sens.J(5,7) = ROcp6_56;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp6_95;
    sens.J(6,7) = ROcp6_66;
    sens.A(1) = ACcp6_138;
    sens.A(2) = ACcp6_238;
    sens.A(3) = ACcp6_338;
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
    RLcp7_17 = ROcp7_46*s.dpt(2,1);
    RLcp7_27 = ROcp7_56*s.dpt(2,1);
    RLcp7_37 = ROcp7_66*s.dpt(2,1);
    OMcp7_17 = OMcp7_16+ROcp7_46*qd(7);
    OMcp7_27 = OMcp7_26+ROcp7_56*qd(7);
    OMcp7_37 = OMcp7_36+ROcp7_66*qd(7);
    ORcp7_17 = OMcp7_26*RLcp7_37-OMcp7_36*RLcp7_27;
    ORcp7_27 = -(OMcp7_16*RLcp7_37-OMcp7_36*RLcp7_17);
    ORcp7_37 = OMcp7_16*RLcp7_27-OMcp7_26*RLcp7_17;
    OPcp7_17 = OPcp7_16+ROcp7_46*qdd(7)+qd(7)*(OMcp7_26*ROcp7_66-OMcp7_36*ROcp7_56);
    OPcp7_27 = OPcp7_26+ROcp7_56*qdd(7)-qd(7)*(OMcp7_16*ROcp7_66-OMcp7_36*ROcp7_46);
    OPcp7_37 = OPcp7_36+ROcp7_66*qdd(7)+qd(7)*(OMcp7_16*ROcp7_56-OMcp7_26*ROcp7_46);
    RLcp7_18 = ROcp7_46*s.dpt(2,6);
    RLcp7_28 = ROcp7_56*s.dpt(2,6);
    RLcp7_38 = ROcp7_66*s.dpt(2,6);
    OMcp7_18 = OMcp7_17+ROcp7_17*qd(8);
    OMcp7_28 = OMcp7_27+ROcp7_27*qd(8);
    OMcp7_38 = OMcp7_37+ROcp7_37*qd(8);
    ORcp7_18 = OMcp7_27*RLcp7_38-OMcp7_37*RLcp7_28;
    ORcp7_28 = -(OMcp7_17*RLcp7_38-OMcp7_37*RLcp7_18);
    ORcp7_38 = OMcp7_17*RLcp7_28-OMcp7_27*RLcp7_18;
    OPcp7_18 = OPcp7_17+ROcp7_17*qdd(8)+qd(8)*(OMcp7_27*ROcp7_37-OMcp7_37*ROcp7_27);
    OPcp7_28 = OPcp7_27+ROcp7_27*qdd(8)-qd(8)*(OMcp7_17*ROcp7_37-OMcp7_37*ROcp7_17);
    OPcp7_38 = OPcp7_37+ROcp7_37*qdd(8)+qd(8)*(OMcp7_17*ROcp7_27-OMcp7_27*ROcp7_17);
    RLcp7_139 = ROcp7_78*s.dpt(3,8);
    RLcp7_239 = ROcp7_88*s.dpt(3,8);
    RLcp7_339 = ROcp7_98*s.dpt(3,8);
    POcp7_139 = RLcp7_139+RLcp7_17+RLcp7_18+q(1);
    POcp7_239 = RLcp7_239+RLcp7_27+RLcp7_28+q(2);
    POcp7_339 = RLcp7_339+RLcp7_37+RLcp7_38+q(3);
    JTcp7_239_4 = -(RLcp7_339+RLcp7_37+RLcp7_38);
    JTcp7_339_4 = RLcp7_239+RLcp7_27+RLcp7_28;
    JTcp7_139_5 = C4*(RLcp7_37+RLcp7_38)-S4*(RLcp7_27+RLcp7_28)-RLcp7_239*S4+RLcp7_339*C4;
    JTcp7_239_5 = S4*(RLcp7_139+RLcp7_17+RLcp7_18);
    JTcp7_339_5 = -C4*(RLcp7_139+RLcp7_17+RLcp7_18);
    JTcp7_139_6 = ROcp7_85*(RLcp7_37+RLcp7_38)-ROcp7_95*(RLcp7_27+RLcp7_28)-RLcp7_239*ROcp7_95+RLcp7_339*ROcp7_85;
    JTcp7_239_6 = -(RLcp7_339*S5-ROcp7_95*(RLcp7_139+RLcp7_17+RLcp7_18)+S5*(RLcp7_37+RLcp7_38));
    JTcp7_339_6 = RLcp7_239*S5-ROcp7_85*(RLcp7_139+RLcp7_17+RLcp7_18)+S5*(RLcp7_27+RLcp7_28);
    JTcp7_139_7 = ROcp7_56*(RLcp7_339+RLcp7_38)-ROcp7_66*(RLcp7_239+RLcp7_28);
    JTcp7_239_7 = -(ROcp7_46*(RLcp7_339+RLcp7_38)-ROcp7_66*(RLcp7_139+RLcp7_18));
    JTcp7_339_7 = ROcp7_46*(RLcp7_239+RLcp7_28)-ROcp7_56*(RLcp7_139+RLcp7_18);
    JTcp7_139_8 = -(RLcp7_239*ROcp7_37-RLcp7_339*ROcp7_27);
    JTcp7_239_8 = RLcp7_139*ROcp7_37-RLcp7_339*ROcp7_17;
    JTcp7_339_8 = -(RLcp7_139*ROcp7_27-RLcp7_239*ROcp7_17);
    ORcp7_139 = OMcp7_28*RLcp7_339-OMcp7_38*RLcp7_239;
    ORcp7_239 = -(OMcp7_18*RLcp7_339-OMcp7_38*RLcp7_139);
    ORcp7_339 = OMcp7_18*RLcp7_239-OMcp7_28*RLcp7_139;
    VIcp7_139 = ORcp7_139+ORcp7_17+ORcp7_18+qd(1);
    VIcp7_239 = ORcp7_239+ORcp7_27+ORcp7_28+qd(2);
    VIcp7_339 = ORcp7_339+ORcp7_37+ORcp7_38+qd(3);
    ACcp7_139 = qdd(1)+OMcp7_26*ORcp7_37+OMcp7_27*ORcp7_38+OMcp7_28*ORcp7_339-OMcp7_36*ORcp7_27-OMcp7_37*ORcp7_28-OMcp7_38*ORcp7_239+OPcp7_26*...
 RLcp7_37+OPcp7_27*RLcp7_38+OPcp7_28*RLcp7_339-OPcp7_36*RLcp7_27-OPcp7_37*RLcp7_28-OPcp7_38*RLcp7_239;
    ACcp7_239 = qdd(2)-OMcp7_16*ORcp7_37-OMcp7_17*ORcp7_38-OMcp7_18*ORcp7_339+OMcp7_36*ORcp7_17+OMcp7_37*ORcp7_18+OMcp7_38*ORcp7_139-OPcp7_16*...
 RLcp7_37-OPcp7_17*RLcp7_38-OPcp7_18*RLcp7_339+OPcp7_36*RLcp7_17+OPcp7_37*RLcp7_18+OPcp7_38*RLcp7_139;
    ACcp7_339 = qdd(3)+OMcp7_16*ORcp7_27+OMcp7_17*ORcp7_28+OMcp7_18*ORcp7_239-OMcp7_26*ORcp7_17-OMcp7_27*ORcp7_18-OMcp7_28*ORcp7_139+OPcp7_16*...
 RLcp7_27+OPcp7_17*RLcp7_28+OPcp7_18*RLcp7_239-OPcp7_26*RLcp7_17-OPcp7_27*RLcp7_18-OPcp7_28*RLcp7_139;

% = = Block_1_0_0_8_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp7_139;
    sens.P(2) = POcp7_239;
    sens.P(3) = POcp7_339;
    sens.R(1,1) = ROcp7_17;
    sens.R(1,2) = ROcp7_27;
    sens.R(1,3) = ROcp7_37;
    sens.R(2,1) = ROcp7_48;
    sens.R(2,2) = ROcp7_58;
    sens.R(2,3) = ROcp7_68;
    sens.R(3,1) = ROcp7_78;
    sens.R(3,2) = ROcp7_88;
    sens.R(3,3) = ROcp7_98;
    sens.V(1) = VIcp7_139;
    sens.V(2) = VIcp7_239;
    sens.V(3) = VIcp7_339;
    sens.OM(1) = OMcp7_18;
    sens.OM(2) = OMcp7_28;
    sens.OM(3) = OMcp7_38;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp7_139_5;
    sens.J(1,6) = JTcp7_139_6;
    sens.J(1,7) = JTcp7_139_7;
    sens.J(1,8) = JTcp7_139_8;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp7_239_4;
    sens.J(2,5) = JTcp7_239_5;
    sens.J(2,6) = JTcp7_239_6;
    sens.J(2,7) = JTcp7_239_7;
    sens.J(2,8) = JTcp7_239_8;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp7_339_4;
    sens.J(3,5) = JTcp7_339_5;
    sens.J(3,6) = JTcp7_339_6;
    sens.J(3,7) = JTcp7_339_7;
    sens.J(3,8) = JTcp7_339_8;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,7) = ROcp7_46;
    sens.J(4,8) = ROcp7_17;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp7_85;
    sens.J(5,7) = ROcp7_56;
    sens.J(5,8) = ROcp7_27;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp7_95;
    sens.J(6,7) = ROcp7_66;
    sens.J(6,8) = ROcp7_37;
    sens.A(1) = ACcp7_139;
    sens.A(2) = ACcp7_239;
    sens.A(3) = ACcp7_339;
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
    RLcp8_17 = ROcp8_46*s.dpt(2,1);
    RLcp8_27 = ROcp8_56*s.dpt(2,1);
    RLcp8_37 = ROcp8_66*s.dpt(2,1);
    OMcp8_17 = OMcp8_16+ROcp8_46*qd(7);
    OMcp8_27 = OMcp8_26+ROcp8_56*qd(7);
    OMcp8_37 = OMcp8_36+ROcp8_66*qd(7);
    ORcp8_17 = OMcp8_26*RLcp8_37-OMcp8_36*RLcp8_27;
    ORcp8_27 = -(OMcp8_16*RLcp8_37-OMcp8_36*RLcp8_17);
    ORcp8_37 = OMcp8_16*RLcp8_27-OMcp8_26*RLcp8_17;
    OPcp8_17 = OPcp8_16+ROcp8_46*qdd(7)+qd(7)*(OMcp8_26*ROcp8_66-OMcp8_36*ROcp8_56);
    OPcp8_27 = OPcp8_26+ROcp8_56*qdd(7)-qd(7)*(OMcp8_16*ROcp8_66-OMcp8_36*ROcp8_46);
    OPcp8_37 = OPcp8_36+ROcp8_66*qdd(7)+qd(7)*(OMcp8_16*ROcp8_56-OMcp8_26*ROcp8_46);
    RLcp8_18 = ROcp8_46*s.dpt(2,6);
    RLcp8_28 = ROcp8_56*s.dpt(2,6);
    RLcp8_38 = ROcp8_66*s.dpt(2,6);
    OMcp8_18 = OMcp8_17+ROcp8_17*qd(8);
    OMcp8_28 = OMcp8_27+ROcp8_27*qd(8);
    OMcp8_38 = OMcp8_37+ROcp8_37*qd(8);
    ORcp8_18 = OMcp8_27*RLcp8_38-OMcp8_37*RLcp8_28;
    ORcp8_28 = -(OMcp8_17*RLcp8_38-OMcp8_37*RLcp8_18);
    ORcp8_38 = OMcp8_17*RLcp8_28-OMcp8_27*RLcp8_18;
    OPcp8_18 = OPcp8_17+ROcp8_17*qdd(8)+qd(8)*(OMcp8_27*ROcp8_37-OMcp8_37*ROcp8_27);
    OPcp8_28 = OPcp8_27+ROcp8_27*qdd(8)-qd(8)*(OMcp8_17*ROcp8_37-OMcp8_37*ROcp8_17);
    OPcp8_38 = OPcp8_37+ROcp8_37*qdd(8)+qd(8)*(OMcp8_17*ROcp8_27-OMcp8_27*ROcp8_17);
    RLcp8_140 = ROcp8_17*s.dpt(1,9)+ROcp8_48*s.dpt(2,9)+ROcp8_78*s.dpt(3,9);
    RLcp8_240 = ROcp8_27*s.dpt(1,9)+ROcp8_58*s.dpt(2,9)+ROcp8_88*s.dpt(3,9);
    RLcp8_340 = ROcp8_37*s.dpt(1,9)+ROcp8_68*s.dpt(2,9)+ROcp8_98*s.dpt(3,9);
    POcp8_140 = RLcp8_140+RLcp8_17+RLcp8_18+q(1);
    POcp8_240 = RLcp8_240+RLcp8_27+RLcp8_28+q(2);
    POcp8_340 = RLcp8_340+RLcp8_37+RLcp8_38+q(3);
    JTcp8_240_4 = -(RLcp8_340+RLcp8_37+RLcp8_38);
    JTcp8_340_4 = RLcp8_240+RLcp8_27+RLcp8_28;
    JTcp8_140_5 = C4*(RLcp8_37+RLcp8_38)-S4*(RLcp8_27+RLcp8_28)-RLcp8_240*S4+RLcp8_340*C4;
    JTcp8_240_5 = S4*(RLcp8_140+RLcp8_17+RLcp8_18);
    JTcp8_340_5 = -C4*(RLcp8_140+RLcp8_17+RLcp8_18);
    JTcp8_140_6 = ROcp8_85*(RLcp8_37+RLcp8_38)-ROcp8_95*(RLcp8_27+RLcp8_28)-RLcp8_240*ROcp8_95+RLcp8_340*ROcp8_85;
    JTcp8_240_6 = -(RLcp8_340*S5-ROcp8_95*(RLcp8_140+RLcp8_17+RLcp8_18)+S5*(RLcp8_37+RLcp8_38));
    JTcp8_340_6 = RLcp8_240*S5-ROcp8_85*(RLcp8_140+RLcp8_17+RLcp8_18)+S5*(RLcp8_27+RLcp8_28);
    JTcp8_140_7 = ROcp8_56*(RLcp8_340+RLcp8_38)-ROcp8_66*(RLcp8_240+RLcp8_28);
    JTcp8_240_7 = -(ROcp8_46*(RLcp8_340+RLcp8_38)-ROcp8_66*(RLcp8_140+RLcp8_18));
    JTcp8_340_7 = ROcp8_46*(RLcp8_240+RLcp8_28)-ROcp8_56*(RLcp8_140+RLcp8_18);
    JTcp8_140_8 = -(RLcp8_240*ROcp8_37-RLcp8_340*ROcp8_27);
    JTcp8_240_8 = RLcp8_140*ROcp8_37-RLcp8_340*ROcp8_17;
    JTcp8_340_8 = -(RLcp8_140*ROcp8_27-RLcp8_240*ROcp8_17);
    ORcp8_140 = OMcp8_28*RLcp8_340-OMcp8_38*RLcp8_240;
    ORcp8_240 = -(OMcp8_18*RLcp8_340-OMcp8_38*RLcp8_140);
    ORcp8_340 = OMcp8_18*RLcp8_240-OMcp8_28*RLcp8_140;
    VIcp8_140 = ORcp8_140+ORcp8_17+ORcp8_18+qd(1);
    VIcp8_240 = ORcp8_240+ORcp8_27+ORcp8_28+qd(2);
    VIcp8_340 = ORcp8_340+ORcp8_37+ORcp8_38+qd(3);
    ACcp8_140 = qdd(1)+OMcp8_26*ORcp8_37+OMcp8_27*ORcp8_38+OMcp8_28*ORcp8_340-OMcp8_36*ORcp8_27-OMcp8_37*ORcp8_28-OMcp8_38*ORcp8_240+OPcp8_26*...
 RLcp8_37+OPcp8_27*RLcp8_38+OPcp8_28*RLcp8_340-OPcp8_36*RLcp8_27-OPcp8_37*RLcp8_28-OPcp8_38*RLcp8_240;
    ACcp8_240 = qdd(2)-OMcp8_16*ORcp8_37-OMcp8_17*ORcp8_38-OMcp8_18*ORcp8_340+OMcp8_36*ORcp8_17+OMcp8_37*ORcp8_18+OMcp8_38*ORcp8_140-OPcp8_16*...
 RLcp8_37-OPcp8_17*RLcp8_38-OPcp8_18*RLcp8_340+OPcp8_36*RLcp8_17+OPcp8_37*RLcp8_18+OPcp8_38*RLcp8_140;
    ACcp8_340 = qdd(3)+OMcp8_16*ORcp8_27+OMcp8_17*ORcp8_28+OMcp8_18*ORcp8_240-OMcp8_26*ORcp8_17-OMcp8_27*ORcp8_18-OMcp8_28*ORcp8_140+OPcp8_16*...
 RLcp8_27+OPcp8_17*RLcp8_28+OPcp8_18*RLcp8_240-OPcp8_26*RLcp8_17-OPcp8_27*RLcp8_18-OPcp8_28*RLcp8_140;

% = = Block_1_0_0_9_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp8_140;
    sens.P(2) = POcp8_240;
    sens.P(3) = POcp8_340;
    sens.R(1,1) = ROcp8_17;
    sens.R(1,2) = ROcp8_27;
    sens.R(1,3) = ROcp8_37;
    sens.R(2,1) = ROcp8_48;
    sens.R(2,2) = ROcp8_58;
    sens.R(2,3) = ROcp8_68;
    sens.R(3,1) = ROcp8_78;
    sens.R(3,2) = ROcp8_88;
    sens.R(3,3) = ROcp8_98;
    sens.V(1) = VIcp8_140;
    sens.V(2) = VIcp8_240;
    sens.V(3) = VIcp8_340;
    sens.OM(1) = OMcp8_18;
    sens.OM(2) = OMcp8_28;
    sens.OM(3) = OMcp8_38;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp8_140_5;
    sens.J(1,6) = JTcp8_140_6;
    sens.J(1,7) = JTcp8_140_7;
    sens.J(1,8) = JTcp8_140_8;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp8_240_4;
    sens.J(2,5) = JTcp8_240_5;
    sens.J(2,6) = JTcp8_240_6;
    sens.J(2,7) = JTcp8_240_7;
    sens.J(2,8) = JTcp8_240_8;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp8_340_4;
    sens.J(3,5) = JTcp8_340_5;
    sens.J(3,6) = JTcp8_340_6;
    sens.J(3,7) = JTcp8_340_7;
    sens.J(3,8) = JTcp8_340_8;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,7) = ROcp8_46;
    sens.J(4,8) = ROcp8_17;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp8_85;
    sens.J(5,7) = ROcp8_56;
    sens.J(5,8) = ROcp8_27;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp8_95;
    sens.J(6,7) = ROcp8_66;
    sens.J(6,8) = ROcp8_37;
    sens.A(1) = ACcp8_140;
    sens.A(2) = ACcp8_240;
    sens.A(3) = ACcp8_340;
    sens.OMP(1) = OPcp8_18;
    sens.OMP(2) = OPcp8_28;
    sens.OMP(3) = OPcp8_38;
 
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
    RLcp9_17 = ROcp9_46*s.dpt(2,1);
    RLcp9_27 = ROcp9_56*s.dpt(2,1);
    RLcp9_37 = ROcp9_66*s.dpt(2,1);
    OMcp9_17 = OMcp9_16+ROcp9_46*qd(7);
    OMcp9_27 = OMcp9_26+ROcp9_56*qd(7);
    OMcp9_37 = OMcp9_36+ROcp9_66*qd(7);
    ORcp9_17 = OMcp9_26*RLcp9_37-OMcp9_36*RLcp9_27;
    ORcp9_27 = -(OMcp9_16*RLcp9_37-OMcp9_36*RLcp9_17);
    ORcp9_37 = OMcp9_16*RLcp9_27-OMcp9_26*RLcp9_17;
    OPcp9_17 = OPcp9_16+ROcp9_46*qdd(7)+qd(7)*(OMcp9_26*ROcp9_66-OMcp9_36*ROcp9_56);
    OPcp9_27 = OPcp9_26+ROcp9_56*qdd(7)-qd(7)*(OMcp9_16*ROcp9_66-OMcp9_36*ROcp9_46);
    OPcp9_37 = OPcp9_36+ROcp9_66*qdd(7)+qd(7)*(OMcp9_16*ROcp9_56-OMcp9_26*ROcp9_46);
    RLcp9_18 = ROcp9_46*s.dpt(2,6);
    RLcp9_28 = ROcp9_56*s.dpt(2,6);
    RLcp9_38 = ROcp9_66*s.dpt(2,6);
    OMcp9_18 = OMcp9_17+ROcp9_17*qd(8);
    OMcp9_28 = OMcp9_27+ROcp9_27*qd(8);
    OMcp9_38 = OMcp9_37+ROcp9_37*qd(8);
    ORcp9_18 = OMcp9_27*RLcp9_38-OMcp9_37*RLcp9_28;
    ORcp9_28 = -(OMcp9_17*RLcp9_38-OMcp9_37*RLcp9_18);
    ORcp9_38 = OMcp9_17*RLcp9_28-OMcp9_27*RLcp9_18;
    OPcp9_18 = OPcp9_17+ROcp9_17*qdd(8)+qd(8)*(OMcp9_27*ROcp9_37-OMcp9_37*ROcp9_27);
    OPcp9_28 = OPcp9_27+ROcp9_27*qdd(8)-qd(8)*(OMcp9_17*ROcp9_37-OMcp9_37*ROcp9_17);
    OPcp9_38 = OPcp9_37+ROcp9_37*qdd(8)+qd(8)*(OMcp9_17*ROcp9_27-OMcp9_27*ROcp9_17);
    RLcp9_19 = ROcp9_78*s.dpt(3,8);
    RLcp9_29 = ROcp9_88*s.dpt(3,8);
    RLcp9_39 = ROcp9_98*s.dpt(3,8);
    OMcp9_19 = OMcp9_18+ROcp9_78*qd(9);
    OMcp9_29 = OMcp9_28+ROcp9_88*qd(9);
    OMcp9_39 = OMcp9_38+ROcp9_98*qd(9);
    ORcp9_19 = OMcp9_28*RLcp9_39-OMcp9_38*RLcp9_29;
    ORcp9_29 = -(OMcp9_18*RLcp9_39-OMcp9_38*RLcp9_19);
    ORcp9_39 = OMcp9_18*RLcp9_29-OMcp9_28*RLcp9_19;
    OPcp9_19 = OPcp9_18+ROcp9_78*qdd(9)+qd(9)*(OMcp9_28*ROcp9_98-OMcp9_38*ROcp9_88);
    OPcp9_29 = OPcp9_28+ROcp9_88*qdd(9)-qd(9)*(OMcp9_18*ROcp9_98-OMcp9_38*ROcp9_78);
    OPcp9_39 = OPcp9_38+ROcp9_98*qdd(9)+qd(9)*(OMcp9_18*ROcp9_88-OMcp9_28*ROcp9_78);
    RLcp9_141 = ROcp9_78*s.dpt(3,10);
    RLcp9_241 = ROcp9_88*s.dpt(3,10);
    RLcp9_341 = ROcp9_98*s.dpt(3,10);
    POcp9_141 = RLcp9_141+RLcp9_17+RLcp9_18+RLcp9_19+q(1);
    POcp9_241 = RLcp9_241+RLcp9_27+RLcp9_28+RLcp9_29+q(2);
    POcp9_341 = RLcp9_341+RLcp9_37+RLcp9_38+RLcp9_39+q(3);
    JTcp9_241_4 = -(RLcp9_341+RLcp9_37+RLcp9_38+RLcp9_39);
    JTcp9_341_4 = RLcp9_241+RLcp9_27+RLcp9_28+RLcp9_29;
    JTcp9_141_5 = C4*(RLcp9_341+RLcp9_37+RLcp9_38+RLcp9_39)-S4*(RLcp9_241+RLcp9_29)-S4*(RLcp9_27+RLcp9_28);
    JTcp9_241_5 = S4*(RLcp9_141+RLcp9_17+RLcp9_18+RLcp9_19);
    JTcp9_341_5 = -C4*(RLcp9_141+RLcp9_17+RLcp9_18+RLcp9_19);
    JTcp9_141_6 = ROcp9_85*(RLcp9_341+RLcp9_37+RLcp9_38+RLcp9_39)-ROcp9_95*(RLcp9_241+RLcp9_29)-ROcp9_95*(RLcp9_27+RLcp9_28);
    JTcp9_241_6 = RLcp9_141*ROcp9_95-RLcp9_341*S5-RLcp9_39*S5+ROcp9_95*(RLcp9_17+RLcp9_18+RLcp9_19)-S5*(RLcp9_37+RLcp9_38);
    JTcp9_341_6 = RLcp9_29*S5-ROcp9_85*(RLcp9_17+RLcp9_18+RLcp9_19)+S5*(RLcp9_27+RLcp9_28)-RLcp9_141*ROcp9_85+RLcp9_241*S5;
    JTcp9_141_7 = ROcp9_56*(RLcp9_38+RLcp9_39)-ROcp9_66*(RLcp9_28+RLcp9_29)-RLcp9_241*ROcp9_66+RLcp9_341*ROcp9_56;
    JTcp9_241_7 = RLcp9_141*ROcp9_66-RLcp9_341*ROcp9_46-ROcp9_46*(RLcp9_38+RLcp9_39)+ROcp9_66*(RLcp9_18+RLcp9_19);
    JTcp9_341_7 = ROcp9_46*(RLcp9_28+RLcp9_29)-ROcp9_56*(RLcp9_18+RLcp9_19)-RLcp9_141*ROcp9_56+RLcp9_241*ROcp9_46;
    JTcp9_141_8 = ROcp9_27*(RLcp9_341+RLcp9_39)-ROcp9_37*(RLcp9_241+RLcp9_29);
    JTcp9_241_8 = -(ROcp9_17*(RLcp9_341+RLcp9_39)-ROcp9_37*(RLcp9_141+RLcp9_19));
    JTcp9_341_8 = ROcp9_17*(RLcp9_241+RLcp9_29)-ROcp9_27*(RLcp9_141+RLcp9_19);
    JTcp9_141_9 = -(RLcp9_241*ROcp9_98-RLcp9_341*ROcp9_88);
    JTcp9_241_9 = RLcp9_141*ROcp9_98-RLcp9_341*ROcp9_78;
    JTcp9_341_9 = -(RLcp9_141*ROcp9_88-RLcp9_241*ROcp9_78);
    ORcp9_141 = OMcp9_29*RLcp9_341-OMcp9_39*RLcp9_241;
    ORcp9_241 = -(OMcp9_19*RLcp9_341-OMcp9_39*RLcp9_141);
    ORcp9_341 = OMcp9_19*RLcp9_241-OMcp9_29*RLcp9_141;
    VIcp9_141 = ORcp9_141+ORcp9_17+ORcp9_18+ORcp9_19+qd(1);
    VIcp9_241 = ORcp9_241+ORcp9_27+ORcp9_28+ORcp9_29+qd(2);
    VIcp9_341 = ORcp9_341+ORcp9_37+ORcp9_38+ORcp9_39+qd(3);
    ACcp9_141 = qdd(1)+OMcp9_26*ORcp9_37+OMcp9_27*ORcp9_38+OMcp9_28*ORcp9_39+OMcp9_29*ORcp9_341-OMcp9_36*ORcp9_27-OMcp9_37*ORcp9_28-OMcp9_38*...
 ORcp9_29-OMcp9_39*ORcp9_241+OPcp9_26*RLcp9_37+OPcp9_27*RLcp9_38+OPcp9_28*RLcp9_39+OPcp9_29*RLcp9_341-OPcp9_36*RLcp9_27-OPcp9_37*RLcp9_28-OPcp9_38*...
 RLcp9_29-OPcp9_39*RLcp9_241;
    ACcp9_241 = qdd(2)-OMcp9_16*ORcp9_37-OMcp9_17*ORcp9_38-OMcp9_18*ORcp9_39-OMcp9_19*ORcp9_341+OMcp9_36*ORcp9_17+OMcp9_37*ORcp9_18+OMcp9_38*...
 ORcp9_19+OMcp9_39*ORcp9_141-OPcp9_16*RLcp9_37-OPcp9_17*RLcp9_38-OPcp9_18*RLcp9_39-OPcp9_19*RLcp9_341+OPcp9_36*RLcp9_17+OPcp9_37*RLcp9_18+OPcp9_38*...
 RLcp9_19+OPcp9_39*RLcp9_141;
    ACcp9_341 = qdd(3)+OMcp9_16*ORcp9_27+OMcp9_17*ORcp9_28+OMcp9_18*ORcp9_29+OMcp9_19*ORcp9_241-OMcp9_26*ORcp9_17-OMcp9_27*ORcp9_18-OMcp9_28*...
 ORcp9_19-OMcp9_29*ORcp9_141+OPcp9_16*RLcp9_27+OPcp9_17*RLcp9_28+OPcp9_18*RLcp9_29+OPcp9_19*RLcp9_241-OPcp9_26*RLcp9_17-OPcp9_27*RLcp9_18-OPcp9_28*...
 RLcp9_19-OPcp9_29*RLcp9_141;

% = = Block_1_0_0_10_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp9_141;
    sens.P(2) = POcp9_241;
    sens.P(3) = POcp9_341;
    sens.R(1,1) = ROcp9_19;
    sens.R(1,2) = ROcp9_29;
    sens.R(1,3) = ROcp9_39;
    sens.R(2,1) = ROcp9_49;
    sens.R(2,2) = ROcp9_59;
    sens.R(2,3) = ROcp9_69;
    sens.R(3,1) = ROcp9_78;
    sens.R(3,2) = ROcp9_88;
    sens.R(3,3) = ROcp9_98;
    sens.V(1) = VIcp9_141;
    sens.V(2) = VIcp9_241;
    sens.V(3) = VIcp9_341;
    sens.OM(1) = OMcp9_19;
    sens.OM(2) = OMcp9_29;
    sens.OM(3) = OMcp9_39;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp9_141_5;
    sens.J(1,6) = JTcp9_141_6;
    sens.J(1,7) = JTcp9_141_7;
    sens.J(1,8) = JTcp9_141_8;
    sens.J(1,9) = JTcp9_141_9;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp9_241_4;
    sens.J(2,5) = JTcp9_241_5;
    sens.J(2,6) = JTcp9_241_6;
    sens.J(2,7) = JTcp9_241_7;
    sens.J(2,8) = JTcp9_241_8;
    sens.J(2,9) = JTcp9_241_9;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp9_341_4;
    sens.J(3,5) = JTcp9_341_5;
    sens.J(3,6) = JTcp9_341_6;
    sens.J(3,7) = JTcp9_341_7;
    sens.J(3,8) = JTcp9_341_8;
    sens.J(3,9) = JTcp9_341_9;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,7) = ROcp9_46;
    sens.J(4,8) = ROcp9_17;
    sens.J(4,9) = ROcp9_78;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp9_85;
    sens.J(5,7) = ROcp9_56;
    sens.J(5,8) = ROcp9_27;
    sens.J(5,9) = ROcp9_88;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp9_95;
    sens.J(6,7) = ROcp9_66;
    sens.J(6,8) = ROcp9_37;
    sens.J(6,9) = ROcp9_98;
    sens.A(1) = ACcp9_141;
    sens.A(2) = ACcp9_241;
    sens.A(3) = ACcp9_341;
    sens.OMP(1) = OPcp9_19;
    sens.OMP(2) = OPcp9_29;
    sens.OMP(3) = OPcp9_39;
 
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
    RLcp10_17 = ROcp10_46*s.dpt(2,1);
    RLcp10_27 = ROcp10_56*s.dpt(2,1);
    RLcp10_37 = ROcp10_66*s.dpt(2,1);
    OMcp10_17 = OMcp10_16+ROcp10_46*qd(7);
    OMcp10_27 = OMcp10_26+ROcp10_56*qd(7);
    OMcp10_37 = OMcp10_36+ROcp10_66*qd(7);
    ORcp10_17 = OMcp10_26*RLcp10_37-OMcp10_36*RLcp10_27;
    ORcp10_27 = -(OMcp10_16*RLcp10_37-OMcp10_36*RLcp10_17);
    ORcp10_37 = OMcp10_16*RLcp10_27-OMcp10_26*RLcp10_17;
    OPcp10_17 = OPcp10_16+ROcp10_46*qdd(7)+qd(7)*(OMcp10_26*ROcp10_66-OMcp10_36*ROcp10_56);
    OPcp10_27 = OPcp10_26+ROcp10_56*qdd(7)-qd(7)*(OMcp10_16*ROcp10_66-OMcp10_36*ROcp10_46);
    OPcp10_37 = OPcp10_36+ROcp10_66*qdd(7)+qd(7)*(OMcp10_16*ROcp10_56-OMcp10_26*ROcp10_46);
    RLcp10_18 = ROcp10_46*s.dpt(2,6);
    RLcp10_28 = ROcp10_56*s.dpt(2,6);
    RLcp10_38 = ROcp10_66*s.dpt(2,6);
    OMcp10_18 = OMcp10_17+ROcp10_17*qd(8);
    OMcp10_28 = OMcp10_27+ROcp10_27*qd(8);
    OMcp10_38 = OMcp10_37+ROcp10_37*qd(8);
    ORcp10_18 = OMcp10_27*RLcp10_38-OMcp10_37*RLcp10_28;
    ORcp10_28 = -(OMcp10_17*RLcp10_38-OMcp10_37*RLcp10_18);
    ORcp10_38 = OMcp10_17*RLcp10_28-OMcp10_27*RLcp10_18;
    OPcp10_18 = OPcp10_17+ROcp10_17*qdd(8)+qd(8)*(OMcp10_27*ROcp10_37-OMcp10_37*ROcp10_27);
    OPcp10_28 = OPcp10_27+ROcp10_27*qdd(8)-qd(8)*(OMcp10_17*ROcp10_37-OMcp10_37*ROcp10_17);
    OPcp10_38 = OPcp10_37+ROcp10_37*qdd(8)+qd(8)*(OMcp10_17*ROcp10_27-OMcp10_27*ROcp10_17);
    RLcp10_19 = ROcp10_78*s.dpt(3,8);
    RLcp10_29 = ROcp10_88*s.dpt(3,8);
    RLcp10_39 = ROcp10_98*s.dpt(3,8);
    OMcp10_19 = OMcp10_18+ROcp10_78*qd(9);
    OMcp10_29 = OMcp10_28+ROcp10_88*qd(9);
    OMcp10_39 = OMcp10_38+ROcp10_98*qd(9);
    ORcp10_19 = OMcp10_28*RLcp10_39-OMcp10_38*RLcp10_29;
    ORcp10_29 = -(OMcp10_18*RLcp10_39-OMcp10_38*RLcp10_19);
    ORcp10_39 = OMcp10_18*RLcp10_29-OMcp10_28*RLcp10_19;
    OPcp10_19 = OPcp10_18+ROcp10_78*qdd(9)+qd(9)*(OMcp10_28*ROcp10_98-OMcp10_38*ROcp10_88);
    OPcp10_29 = OPcp10_28+ROcp10_88*qdd(9)-qd(9)*(OMcp10_18*ROcp10_98-OMcp10_38*ROcp10_78);
    OPcp10_39 = OPcp10_38+ROcp10_98*qdd(9)+qd(9)*(OMcp10_18*ROcp10_88-OMcp10_28*ROcp10_78);
    RLcp10_142 = ROcp10_19*s.dpt(1,11)+ROcp10_49*s.dpt(2,11)+ROcp10_78*s.dpt(3,11);
    RLcp10_242 = ROcp10_29*s.dpt(1,11)+ROcp10_59*s.dpt(2,11)+ROcp10_88*s.dpt(3,11);
    RLcp10_342 = ROcp10_39*s.dpt(1,11)+ROcp10_69*s.dpt(2,11)+ROcp10_98*s.dpt(3,11);
    POcp10_142 = RLcp10_142+RLcp10_17+RLcp10_18+RLcp10_19+q(1);
    POcp10_242 = RLcp10_242+RLcp10_27+RLcp10_28+RLcp10_29+q(2);
    POcp10_342 = RLcp10_342+RLcp10_37+RLcp10_38+RLcp10_39+q(3);
    JTcp10_242_4 = -(RLcp10_342+RLcp10_37+RLcp10_38+RLcp10_39);
    JTcp10_342_4 = RLcp10_242+RLcp10_27+RLcp10_28+RLcp10_29;
    JTcp10_142_5 = C4*(RLcp10_342+RLcp10_37+RLcp10_38+RLcp10_39)-S4*(RLcp10_242+RLcp10_29)-S4*(RLcp10_27+RLcp10_28);
    JTcp10_242_5 = S4*(RLcp10_142+RLcp10_17+RLcp10_18+RLcp10_19);
    JTcp10_342_5 = -C4*(RLcp10_142+RLcp10_17+RLcp10_18+RLcp10_19);
    JTcp10_142_6 = ROcp10_85*(RLcp10_342+RLcp10_37+RLcp10_38+RLcp10_39)-ROcp10_95*(RLcp10_242+RLcp10_29)-ROcp10_95*(RLcp10_27+RLcp10_28);
    JTcp10_242_6 = RLcp10_142*ROcp10_95-RLcp10_342*S5-RLcp10_39*S5+ROcp10_95*(RLcp10_17+RLcp10_18+RLcp10_19)-S5*(RLcp10_37+RLcp10_38);
    JTcp10_342_6 = RLcp10_29*S5-ROcp10_85*(RLcp10_17+RLcp10_18+RLcp10_19)+S5*(RLcp10_27+RLcp10_28)-RLcp10_142*ROcp10_85+RLcp10_242*S5;
    JTcp10_142_7 = ROcp10_56*(RLcp10_38+RLcp10_39)-ROcp10_66*(RLcp10_28+RLcp10_29)-RLcp10_242*ROcp10_66+RLcp10_342*ROcp10_56;
    JTcp10_242_7 = RLcp10_142*ROcp10_66-RLcp10_342*ROcp10_46-ROcp10_46*(RLcp10_38+RLcp10_39)+ROcp10_66*(RLcp10_18+RLcp10_19);
    JTcp10_342_7 = ROcp10_46*(RLcp10_28+RLcp10_29)-ROcp10_56*(RLcp10_18+RLcp10_19)-RLcp10_142*ROcp10_56+RLcp10_242*ROcp10_46;
    JTcp10_142_8 = ROcp10_27*(RLcp10_342+RLcp10_39)-ROcp10_37*(RLcp10_242+RLcp10_29);
    JTcp10_242_8 = -(ROcp10_17*(RLcp10_342+RLcp10_39)-ROcp10_37*(RLcp10_142+RLcp10_19));
    JTcp10_342_8 = ROcp10_17*(RLcp10_242+RLcp10_29)-ROcp10_27*(RLcp10_142+RLcp10_19);
    JTcp10_142_9 = -(RLcp10_242*ROcp10_98-RLcp10_342*ROcp10_88);
    JTcp10_242_9 = RLcp10_142*ROcp10_98-RLcp10_342*ROcp10_78;
    JTcp10_342_9 = -(RLcp10_142*ROcp10_88-RLcp10_242*ROcp10_78);
    ORcp10_142 = OMcp10_29*RLcp10_342-OMcp10_39*RLcp10_242;
    ORcp10_242 = -(OMcp10_19*RLcp10_342-OMcp10_39*RLcp10_142);
    ORcp10_342 = OMcp10_19*RLcp10_242-OMcp10_29*RLcp10_142;
    VIcp10_142 = ORcp10_142+ORcp10_17+ORcp10_18+ORcp10_19+qd(1);
    VIcp10_242 = ORcp10_242+ORcp10_27+ORcp10_28+ORcp10_29+qd(2);
    VIcp10_342 = ORcp10_342+ORcp10_37+ORcp10_38+ORcp10_39+qd(3);
    ACcp10_142 = qdd(1)+OMcp10_26*ORcp10_37+OMcp10_27*ORcp10_38+OMcp10_28*ORcp10_39+OMcp10_29*ORcp10_342-OMcp10_36*ORcp10_27-OMcp10_37*ORcp10_28-...
 OMcp10_38*ORcp10_29-OMcp10_39*ORcp10_242+OPcp10_26*RLcp10_37+OPcp10_27*RLcp10_38+OPcp10_28*RLcp10_39+OPcp10_29*RLcp10_342-OPcp10_36*RLcp10_27-...
 OPcp10_37*RLcp10_28-OPcp10_38*RLcp10_29-OPcp10_39*RLcp10_242;
    ACcp10_242 = qdd(2)-OMcp10_16*ORcp10_37-OMcp10_17*ORcp10_38-OMcp10_18*ORcp10_39-OMcp10_19*ORcp10_342+OMcp10_36*ORcp10_17+OMcp10_37*ORcp10_18+...
 OMcp10_38*ORcp10_19+OMcp10_39*ORcp10_142-OPcp10_16*RLcp10_37-OPcp10_17*RLcp10_38-OPcp10_18*RLcp10_39-OPcp10_19*RLcp10_342+OPcp10_36*RLcp10_17+...
 OPcp10_37*RLcp10_18+OPcp10_38*RLcp10_19+OPcp10_39*RLcp10_142;
    ACcp10_342 = qdd(3)+OMcp10_16*ORcp10_27+OMcp10_17*ORcp10_28+OMcp10_18*ORcp10_29+OMcp10_19*ORcp10_242-OMcp10_26*ORcp10_17-OMcp10_27*ORcp10_18-...
 OMcp10_28*ORcp10_19-OMcp10_29*ORcp10_142+OPcp10_16*RLcp10_27+OPcp10_17*RLcp10_28+OPcp10_18*RLcp10_29+OPcp10_19*RLcp10_242-OPcp10_26*RLcp10_17-...
 OPcp10_27*RLcp10_18-OPcp10_28*RLcp10_19-OPcp10_29*RLcp10_142;

% = = Block_1_0_0_11_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp10_142;
    sens.P(2) = POcp10_242;
    sens.P(3) = POcp10_342;
    sens.R(1,1) = ROcp10_19;
    sens.R(1,2) = ROcp10_29;
    sens.R(1,3) = ROcp10_39;
    sens.R(2,1) = ROcp10_49;
    sens.R(2,2) = ROcp10_59;
    sens.R(2,3) = ROcp10_69;
    sens.R(3,1) = ROcp10_78;
    sens.R(3,2) = ROcp10_88;
    sens.R(3,3) = ROcp10_98;
    sens.V(1) = VIcp10_142;
    sens.V(2) = VIcp10_242;
    sens.V(3) = VIcp10_342;
    sens.OM(1) = OMcp10_19;
    sens.OM(2) = OMcp10_29;
    sens.OM(3) = OMcp10_39;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp10_142_5;
    sens.J(1,6) = JTcp10_142_6;
    sens.J(1,7) = JTcp10_142_7;
    sens.J(1,8) = JTcp10_142_8;
    sens.J(1,9) = JTcp10_142_9;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp10_242_4;
    sens.J(2,5) = JTcp10_242_5;
    sens.J(2,6) = JTcp10_242_6;
    sens.J(2,7) = JTcp10_242_7;
    sens.J(2,8) = JTcp10_242_8;
    sens.J(2,9) = JTcp10_242_9;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp10_342_4;
    sens.J(3,5) = JTcp10_342_5;
    sens.J(3,6) = JTcp10_342_6;
    sens.J(3,7) = JTcp10_342_7;
    sens.J(3,8) = JTcp10_342_8;
    sens.J(3,9) = JTcp10_342_9;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,7) = ROcp10_46;
    sens.J(4,8) = ROcp10_17;
    sens.J(4,9) = ROcp10_78;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp10_85;
    sens.J(5,7) = ROcp10_56;
    sens.J(5,8) = ROcp10_27;
    sens.J(5,9) = ROcp10_88;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp10_95;
    sens.J(6,7) = ROcp10_66;
    sens.J(6,8) = ROcp10_37;
    sens.J(6,9) = ROcp10_98;
    sens.A(1) = ACcp10_142;
    sens.A(2) = ACcp10_242;
    sens.A(3) = ACcp10_342;
    sens.OMP(1) = OPcp10_19;
    sens.OMP(2) = OPcp10_29;
    sens.OMP(3) = OPcp10_39;
 
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
    RLcp11_17 = ROcp11_46*s.dpt(2,1);
    RLcp11_27 = ROcp11_56*s.dpt(2,1);
    RLcp11_37 = ROcp11_66*s.dpt(2,1);
    OMcp11_17 = OMcp11_16+ROcp11_46*qd(7);
    OMcp11_27 = OMcp11_26+ROcp11_56*qd(7);
    OMcp11_37 = OMcp11_36+ROcp11_66*qd(7);
    ORcp11_17 = OMcp11_26*RLcp11_37-OMcp11_36*RLcp11_27;
    ORcp11_27 = -(OMcp11_16*RLcp11_37-OMcp11_36*RLcp11_17);
    ORcp11_37 = OMcp11_16*RLcp11_27-OMcp11_26*RLcp11_17;
    OPcp11_17 = OPcp11_16+ROcp11_46*qdd(7)+qd(7)*(OMcp11_26*ROcp11_66-OMcp11_36*ROcp11_56);
    OPcp11_27 = OPcp11_26+ROcp11_56*qdd(7)-qd(7)*(OMcp11_16*ROcp11_66-OMcp11_36*ROcp11_46);
    OPcp11_37 = OPcp11_36+ROcp11_66*qdd(7)+qd(7)*(OMcp11_16*ROcp11_56-OMcp11_26*ROcp11_46);
    RLcp11_18 = ROcp11_46*s.dpt(2,6);
    RLcp11_28 = ROcp11_56*s.dpt(2,6);
    RLcp11_38 = ROcp11_66*s.dpt(2,6);
    OMcp11_18 = OMcp11_17+ROcp11_17*qd(8);
    OMcp11_28 = OMcp11_27+ROcp11_27*qd(8);
    OMcp11_38 = OMcp11_37+ROcp11_37*qd(8);
    ORcp11_18 = OMcp11_27*RLcp11_38-OMcp11_37*RLcp11_28;
    ORcp11_28 = -(OMcp11_17*RLcp11_38-OMcp11_37*RLcp11_18);
    ORcp11_38 = OMcp11_17*RLcp11_28-OMcp11_27*RLcp11_18;
    OPcp11_18 = OPcp11_17+ROcp11_17*qdd(8)+qd(8)*(OMcp11_27*ROcp11_37-OMcp11_37*ROcp11_27);
    OPcp11_28 = OPcp11_27+ROcp11_27*qdd(8)-qd(8)*(OMcp11_17*ROcp11_37-OMcp11_37*ROcp11_17);
    OPcp11_38 = OPcp11_37+ROcp11_37*qdd(8)+qd(8)*(OMcp11_17*ROcp11_27-OMcp11_27*ROcp11_17);
    RLcp11_19 = ROcp11_78*s.dpt(3,8);
    RLcp11_29 = ROcp11_88*s.dpt(3,8);
    RLcp11_39 = ROcp11_98*s.dpt(3,8);
    OMcp11_19 = OMcp11_18+ROcp11_78*qd(9);
    OMcp11_29 = OMcp11_28+ROcp11_88*qd(9);
    OMcp11_39 = OMcp11_38+ROcp11_98*qd(9);
    ORcp11_19 = OMcp11_28*RLcp11_39-OMcp11_38*RLcp11_29;
    ORcp11_29 = -(OMcp11_18*RLcp11_39-OMcp11_38*RLcp11_19);
    ORcp11_39 = OMcp11_18*RLcp11_29-OMcp11_28*RLcp11_19;
    OPcp11_19 = OPcp11_18+ROcp11_78*qdd(9)+qd(9)*(OMcp11_28*ROcp11_98-OMcp11_38*ROcp11_88);
    OPcp11_29 = OPcp11_28+ROcp11_88*qdd(9)-qd(9)*(OMcp11_18*ROcp11_98-OMcp11_38*ROcp11_78);
    OPcp11_39 = OPcp11_38+ROcp11_98*qdd(9)+qd(9)*(OMcp11_18*ROcp11_88-OMcp11_28*ROcp11_78);
    RLcp11_110 = ROcp11_78*s.dpt(3,10);
    RLcp11_210 = ROcp11_88*s.dpt(3,10);
    RLcp11_310 = ROcp11_98*s.dpt(3,10);
    OMcp11_110 = OMcp11_19+ROcp11_49*qd(10);
    OMcp11_210 = OMcp11_29+ROcp11_59*qd(10);
    OMcp11_310 = OMcp11_39+ROcp11_69*qd(10);
    ORcp11_110 = OMcp11_29*RLcp11_310-OMcp11_39*RLcp11_210;
    ORcp11_210 = -(OMcp11_19*RLcp11_310-OMcp11_39*RLcp11_110);
    ORcp11_310 = OMcp11_19*RLcp11_210-OMcp11_29*RLcp11_110;
    OPcp11_110 = OPcp11_19+ROcp11_49*qdd(10)+qd(10)*(OMcp11_29*ROcp11_69-OMcp11_39*ROcp11_59);
    OPcp11_210 = OPcp11_29+ROcp11_59*qdd(10)-qd(10)*(OMcp11_19*ROcp11_69-OMcp11_39*ROcp11_49);
    OPcp11_310 = OPcp11_39+ROcp11_69*qdd(10)+qd(10)*(OMcp11_19*ROcp11_59-OMcp11_29*ROcp11_49);
    RLcp11_143 = ROcp11_710*s.dpt(3,12);
    RLcp11_243 = ROcp11_810*s.dpt(3,12);
    RLcp11_343 = ROcp11_910*s.dpt(3,12);
    POcp11_143 = RLcp11_110+RLcp11_143+RLcp11_17+RLcp11_18+RLcp11_19+q(1);
    POcp11_243 = RLcp11_210+RLcp11_243+RLcp11_27+RLcp11_28+RLcp11_29+q(2);
    POcp11_343 = RLcp11_310+RLcp11_343+RLcp11_37+RLcp11_38+RLcp11_39+q(3);
    JTcp11_243_4 = -(RLcp11_310+RLcp11_343+RLcp11_37+RLcp11_38+RLcp11_39);
    JTcp11_343_4 = RLcp11_210+RLcp11_243+RLcp11_27+RLcp11_28+RLcp11_29;
    JTcp11_143_5 = C4*(RLcp11_310+RLcp11_37+RLcp11_38+RLcp11_39)-S4*(RLcp11_210+RLcp11_29)-S4*(RLcp11_27+RLcp11_28)-RLcp11_243*S4+RLcp11_343*C4;
    JTcp11_243_5 = S4*(RLcp11_110+RLcp11_143+RLcp11_17+RLcp11_18+RLcp11_19);
    JTcp11_343_5 = -C4*(RLcp11_110+RLcp11_143+RLcp11_17+RLcp11_18+RLcp11_19);
    JTcp11_143_6 = ROcp11_85*(RLcp11_310+RLcp11_37+RLcp11_38+RLcp11_39)-ROcp11_95*(RLcp11_210+RLcp11_29)-ROcp11_95*(RLcp11_27+RLcp11_28)-RLcp11_243...
 *ROcp11_95+RLcp11_343*ROcp11_85;
    JTcp11_243_6 = -(RLcp11_343*S5-ROcp11_95*(RLcp11_110+RLcp11_143+RLcp11_17+RLcp11_18+RLcp11_19)+S5*(RLcp11_310+RLcp11_39)+S5*(RLcp11_37+...
 RLcp11_38));
    JTcp11_343_6 = RLcp11_243*S5-ROcp11_85*(RLcp11_110+RLcp11_143+RLcp11_17+RLcp11_18+RLcp11_19)+S5*(RLcp11_210+RLcp11_29)+S5*(RLcp11_27+RLcp11_28);
    JTcp11_143_7 = ROcp11_56*(RLcp11_310+RLcp11_343+RLcp11_38+RLcp11_39)-ROcp11_66*(RLcp11_210+RLcp11_243)-ROcp11_66*(RLcp11_28+RLcp11_29);
    JTcp11_243_7 = -(ROcp11_46*(RLcp11_310+RLcp11_343+RLcp11_38+RLcp11_39)-ROcp11_66*(RLcp11_110+RLcp11_143)-ROcp11_66*(RLcp11_18+RLcp11_19));
    JTcp11_343_7 = ROcp11_46*(RLcp11_210+RLcp11_243+RLcp11_28+RLcp11_29)-ROcp11_56*(RLcp11_110+RLcp11_143)-ROcp11_56*(RLcp11_18+RLcp11_19);
    JTcp11_143_8 = ROcp11_27*(RLcp11_310+RLcp11_39)-ROcp11_37*(RLcp11_210+RLcp11_29)-RLcp11_243*ROcp11_37+RLcp11_343*ROcp11_27;
    JTcp11_243_8 = RLcp11_143*ROcp11_37-RLcp11_343*ROcp11_17-ROcp11_17*(RLcp11_310+RLcp11_39)+ROcp11_37*(RLcp11_110+RLcp11_19);
    JTcp11_343_8 = ROcp11_17*(RLcp11_210+RLcp11_29)-ROcp11_27*(RLcp11_110+RLcp11_19)-RLcp11_143*ROcp11_27+RLcp11_243*ROcp11_17;
    JTcp11_143_9 = ROcp11_88*(RLcp11_310+RLcp11_343)-ROcp11_98*(RLcp11_210+RLcp11_243);
    JTcp11_243_9 = -(ROcp11_78*(RLcp11_310+RLcp11_343)-ROcp11_98*(RLcp11_110+RLcp11_143));
    JTcp11_343_9 = ROcp11_78*(RLcp11_210+RLcp11_243)-ROcp11_88*(RLcp11_110+RLcp11_143);
    JTcp11_143_10 = -(RLcp11_243*ROcp11_69-RLcp11_343*ROcp11_59);
    JTcp11_243_10 = RLcp11_143*ROcp11_69-RLcp11_343*ROcp11_49;
    JTcp11_343_10 = -(RLcp11_143*ROcp11_59-RLcp11_243*ROcp11_49);
    ORcp11_143 = OMcp11_210*RLcp11_343-OMcp11_310*RLcp11_243;
    ORcp11_243 = -(OMcp11_110*RLcp11_343-OMcp11_310*RLcp11_143);
    ORcp11_343 = OMcp11_110*RLcp11_243-OMcp11_210*RLcp11_143;
    VIcp11_143 = ORcp11_110+ORcp11_143+ORcp11_17+ORcp11_18+ORcp11_19+qd(1);
    VIcp11_243 = ORcp11_210+ORcp11_243+ORcp11_27+ORcp11_28+ORcp11_29+qd(2);
    VIcp11_343 = ORcp11_310+ORcp11_343+ORcp11_37+ORcp11_38+ORcp11_39+qd(3);
    ACcp11_143 = qdd(1)+OMcp11_210*ORcp11_343+OMcp11_26*ORcp11_37+OMcp11_27*ORcp11_38+OMcp11_28*ORcp11_39+OMcp11_29*ORcp11_310-OMcp11_310*...
 ORcp11_243-OMcp11_36*ORcp11_27-OMcp11_37*ORcp11_28-OMcp11_38*ORcp11_29-OMcp11_39*ORcp11_210+OPcp11_210*RLcp11_343+OPcp11_26*RLcp11_37+OPcp11_27*...
 RLcp11_38+OPcp11_28*RLcp11_39+OPcp11_29*RLcp11_310-OPcp11_310*RLcp11_243-OPcp11_36*RLcp11_27-OPcp11_37*RLcp11_28-OPcp11_38*RLcp11_29-OPcp11_39*...
 RLcp11_210;
    ACcp11_243 = qdd(2)-OMcp11_110*ORcp11_343-OMcp11_16*ORcp11_37-OMcp11_17*ORcp11_38-OMcp11_18*ORcp11_39-OMcp11_19*ORcp11_310+OMcp11_310*...
 ORcp11_143+OMcp11_36*ORcp11_17+OMcp11_37*ORcp11_18+OMcp11_38*ORcp11_19+OMcp11_39*ORcp11_110-OPcp11_110*RLcp11_343-OPcp11_16*RLcp11_37-OPcp11_17*...
 RLcp11_38-OPcp11_18*RLcp11_39-OPcp11_19*RLcp11_310+OPcp11_310*RLcp11_143+OPcp11_36*RLcp11_17+OPcp11_37*RLcp11_18+OPcp11_38*RLcp11_19+OPcp11_39*...
 RLcp11_110;
    ACcp11_343 = qdd(3)+OMcp11_110*ORcp11_243+OMcp11_16*ORcp11_27+OMcp11_17*ORcp11_28+OMcp11_18*ORcp11_29+OMcp11_19*ORcp11_210-OMcp11_210*...
 ORcp11_143-OMcp11_26*ORcp11_17-OMcp11_27*ORcp11_18-OMcp11_28*ORcp11_19-OMcp11_29*ORcp11_110+OPcp11_110*RLcp11_243+OPcp11_16*RLcp11_27+OPcp11_17*...
 RLcp11_28+OPcp11_18*RLcp11_29+OPcp11_19*RLcp11_210-OPcp11_210*RLcp11_143-OPcp11_26*RLcp11_17-OPcp11_27*RLcp11_18-OPcp11_28*RLcp11_19-OPcp11_29*...
 RLcp11_110;

% = = Block_1_0_0_12_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp11_143;
    sens.P(2) = POcp11_243;
    sens.P(3) = POcp11_343;
    sens.R(1,1) = ROcp11_110;
    sens.R(1,2) = ROcp11_210;
    sens.R(1,3) = ROcp11_310;
    sens.R(2,1) = ROcp11_49;
    sens.R(2,2) = ROcp11_59;
    sens.R(2,3) = ROcp11_69;
    sens.R(3,1) = ROcp11_710;
    sens.R(3,2) = ROcp11_810;
    sens.R(3,3) = ROcp11_910;
    sens.V(1) = VIcp11_143;
    sens.V(2) = VIcp11_243;
    sens.V(3) = VIcp11_343;
    sens.OM(1) = OMcp11_110;
    sens.OM(2) = OMcp11_210;
    sens.OM(3) = OMcp11_310;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp11_143_5;
    sens.J(1,6) = JTcp11_143_6;
    sens.J(1,7) = JTcp11_143_7;
    sens.J(1,8) = JTcp11_143_8;
    sens.J(1,9) = JTcp11_143_9;
    sens.J(1,10) = JTcp11_143_10;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp11_243_4;
    sens.J(2,5) = JTcp11_243_5;
    sens.J(2,6) = JTcp11_243_6;
    sens.J(2,7) = JTcp11_243_7;
    sens.J(2,8) = JTcp11_243_8;
    sens.J(2,9) = JTcp11_243_9;
    sens.J(2,10) = JTcp11_243_10;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp11_343_4;
    sens.J(3,5) = JTcp11_343_5;
    sens.J(3,6) = JTcp11_343_6;
    sens.J(3,7) = JTcp11_343_7;
    sens.J(3,8) = JTcp11_343_8;
    sens.J(3,9) = JTcp11_343_9;
    sens.J(3,10) = JTcp11_343_10;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,7) = ROcp11_46;
    sens.J(4,8) = ROcp11_17;
    sens.J(4,9) = ROcp11_78;
    sens.J(4,10) = ROcp11_49;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp11_85;
    sens.J(5,7) = ROcp11_56;
    sens.J(5,8) = ROcp11_27;
    sens.J(5,9) = ROcp11_88;
    sens.J(5,10) = ROcp11_59;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp11_95;
    sens.J(6,7) = ROcp11_66;
    sens.J(6,8) = ROcp11_37;
    sens.J(6,9) = ROcp11_98;
    sens.J(6,10) = ROcp11_69;
    sens.A(1) = ACcp11_143;
    sens.A(2) = ACcp11_243;
    sens.A(3) = ACcp11_343;
    sens.OMP(1) = OPcp11_110;
    sens.OMP(2) = OPcp11_210;
    sens.OMP(3) = OPcp11_310;
 
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

% = = Block_1_0_0_13_0_2 = = 
 
% Sensor Kinematics 


    ROcp12_17 = ROcp12_16*C7-S5*S7;
    ROcp12_27 = ROcp12_26*C7-ROcp12_85*S7;
    ROcp12_37 = ROcp12_36*C7-ROcp12_95*S7;
    ROcp12_77 = ROcp12_16*S7+S5*C7;
    ROcp12_87 = ROcp12_26*S7+ROcp12_85*C7;
    ROcp12_97 = ROcp12_36*S7+ROcp12_95*C7;
    ROcp12_48 = ROcp12_46*C8+ROcp12_77*S8;
    ROcp12_58 = ROcp12_56*C8+ROcp12_87*S8;
    ROcp12_68 = ROcp12_66*C8+ROcp12_97*S8;
    ROcp12_78 = -(ROcp12_46*S8-ROcp12_77*C8);
    ROcp12_88 = -(ROcp12_56*S8-ROcp12_87*C8);
    ROcp12_98 = -(ROcp12_66*S8-ROcp12_97*C8);
    ROcp12_19 = ROcp12_17*C9+ROcp12_48*S9;
    ROcp12_29 = ROcp12_27*C9+ROcp12_58*S9;
    ROcp12_39 = ROcp12_37*C9+ROcp12_68*S9;
    ROcp12_49 = -(ROcp12_17*S9-ROcp12_48*C9);
    ROcp12_59 = -(ROcp12_27*S9-ROcp12_58*C9);
    ROcp12_69 = -(ROcp12_37*S9-ROcp12_68*C9);
    ROcp12_110 = ROcp12_19*C10-ROcp12_78*S10;
    ROcp12_210 = ROcp12_29*C10-ROcp12_88*S10;
    ROcp12_310 = ROcp12_39*C10-ROcp12_98*S10;
    ROcp12_710 = ROcp12_19*S10+ROcp12_78*C10;
    ROcp12_810 = ROcp12_29*S10+ROcp12_88*C10;
    ROcp12_910 = ROcp12_39*S10+ROcp12_98*C10;
    RLcp12_17 = ROcp12_46*s.dpt(2,1);
    RLcp12_27 = ROcp12_56*s.dpt(2,1);
    RLcp12_37 = ROcp12_66*s.dpt(2,1);
    OMcp12_17 = OMcp12_16+ROcp12_46*qd(7);
    OMcp12_27 = OMcp12_26+ROcp12_56*qd(7);
    OMcp12_37 = OMcp12_36+ROcp12_66*qd(7);
    ORcp12_17 = OMcp12_26*RLcp12_37-OMcp12_36*RLcp12_27;
    ORcp12_27 = -(OMcp12_16*RLcp12_37-OMcp12_36*RLcp12_17);
    ORcp12_37 = OMcp12_16*RLcp12_27-OMcp12_26*RLcp12_17;
    OPcp12_17 = OPcp12_16+ROcp12_46*qdd(7)+qd(7)*(OMcp12_26*ROcp12_66-OMcp12_36*ROcp12_56);
    OPcp12_27 = OPcp12_26+ROcp12_56*qdd(7)-qd(7)*(OMcp12_16*ROcp12_66-OMcp12_36*ROcp12_46);
    OPcp12_37 = OPcp12_36+ROcp12_66*qdd(7)+qd(7)*(OMcp12_16*ROcp12_56-OMcp12_26*ROcp12_46);
    RLcp12_18 = ROcp12_46*s.dpt(2,6);
    RLcp12_28 = ROcp12_56*s.dpt(2,6);
    RLcp12_38 = ROcp12_66*s.dpt(2,6);
    OMcp12_18 = OMcp12_17+ROcp12_17*qd(8);
    OMcp12_28 = OMcp12_27+ROcp12_27*qd(8);
    OMcp12_38 = OMcp12_37+ROcp12_37*qd(8);
    ORcp12_18 = OMcp12_27*RLcp12_38-OMcp12_37*RLcp12_28;
    ORcp12_28 = -(OMcp12_17*RLcp12_38-OMcp12_37*RLcp12_18);
    ORcp12_38 = OMcp12_17*RLcp12_28-OMcp12_27*RLcp12_18;
    OPcp12_18 = OPcp12_17+ROcp12_17*qdd(8)+qd(8)*(OMcp12_27*ROcp12_37-OMcp12_37*ROcp12_27);
    OPcp12_28 = OPcp12_27+ROcp12_27*qdd(8)-qd(8)*(OMcp12_17*ROcp12_37-OMcp12_37*ROcp12_17);
    OPcp12_38 = OPcp12_37+ROcp12_37*qdd(8)+qd(8)*(OMcp12_17*ROcp12_27-OMcp12_27*ROcp12_17);
    RLcp12_19 = ROcp12_78*s.dpt(3,8);
    RLcp12_29 = ROcp12_88*s.dpt(3,8);
    RLcp12_39 = ROcp12_98*s.dpt(3,8);
    OMcp12_19 = OMcp12_18+ROcp12_78*qd(9);
    OMcp12_29 = OMcp12_28+ROcp12_88*qd(9);
    OMcp12_39 = OMcp12_38+ROcp12_98*qd(9);
    ORcp12_19 = OMcp12_28*RLcp12_39-OMcp12_38*RLcp12_29;
    ORcp12_29 = -(OMcp12_18*RLcp12_39-OMcp12_38*RLcp12_19);
    ORcp12_39 = OMcp12_18*RLcp12_29-OMcp12_28*RLcp12_19;
    OPcp12_19 = OPcp12_18+ROcp12_78*qdd(9)+qd(9)*(OMcp12_28*ROcp12_98-OMcp12_38*ROcp12_88);
    OPcp12_29 = OPcp12_28+ROcp12_88*qdd(9)-qd(9)*(OMcp12_18*ROcp12_98-OMcp12_38*ROcp12_78);
    OPcp12_39 = OPcp12_38+ROcp12_98*qdd(9)+qd(9)*(OMcp12_18*ROcp12_88-OMcp12_28*ROcp12_78);
    RLcp12_110 = ROcp12_78*s.dpt(3,10);
    RLcp12_210 = ROcp12_88*s.dpt(3,10);
    RLcp12_310 = ROcp12_98*s.dpt(3,10);
    OMcp12_110 = OMcp12_19+ROcp12_49*qd(10);
    OMcp12_210 = OMcp12_29+ROcp12_59*qd(10);
    OMcp12_310 = OMcp12_39+ROcp12_69*qd(10);
    ORcp12_110 = OMcp12_29*RLcp12_310-OMcp12_39*RLcp12_210;
    ORcp12_210 = -(OMcp12_19*RLcp12_310-OMcp12_39*RLcp12_110);
    ORcp12_310 = OMcp12_19*RLcp12_210-OMcp12_29*RLcp12_110;
    OPcp12_110 = OPcp12_19+ROcp12_49*qdd(10)+qd(10)*(OMcp12_29*ROcp12_69-OMcp12_39*ROcp12_59);
    OPcp12_210 = OPcp12_29+ROcp12_59*qdd(10)-qd(10)*(OMcp12_19*ROcp12_69-OMcp12_39*ROcp12_49);
    OPcp12_310 = OPcp12_39+ROcp12_69*qdd(10)+qd(10)*(OMcp12_19*ROcp12_59-OMcp12_29*ROcp12_49);
    RLcp12_144 = ROcp12_110*s.dpt(1,13)+ROcp12_49*s.dpt(2,13)+ROcp12_710*s.dpt(3,13);
    RLcp12_244 = ROcp12_210*s.dpt(1,13)+ROcp12_59*s.dpt(2,13)+ROcp12_810*s.dpt(3,13);
    RLcp12_344 = ROcp12_310*s.dpt(1,13)+ROcp12_69*s.dpt(2,13)+ROcp12_910*s.dpt(3,13);
    POcp12_144 = RLcp12_110+RLcp12_144+RLcp12_17+RLcp12_18+RLcp12_19+q(1);
    POcp12_244 = RLcp12_210+RLcp12_244+RLcp12_27+RLcp12_28+RLcp12_29+q(2);
    POcp12_344 = RLcp12_310+RLcp12_344+RLcp12_37+RLcp12_38+RLcp12_39+q(3);
    JTcp12_244_4 = -(RLcp12_310+RLcp12_344+RLcp12_37+RLcp12_38+RLcp12_39);
    JTcp12_344_4 = RLcp12_210+RLcp12_244+RLcp12_27+RLcp12_28+RLcp12_29;
    JTcp12_144_5 = C4*(RLcp12_310+RLcp12_37+RLcp12_38+RLcp12_39)-S4*(RLcp12_210+RLcp12_29)-S4*(RLcp12_27+RLcp12_28)-RLcp12_244*S4+RLcp12_344*C4;
    JTcp12_244_5 = S4*(RLcp12_110+RLcp12_144+RLcp12_17+RLcp12_18+RLcp12_19);
    JTcp12_344_5 = -C4*(RLcp12_110+RLcp12_144+RLcp12_17+RLcp12_18+RLcp12_19);
    JTcp12_144_6 = ROcp12_85*(RLcp12_310+RLcp12_37+RLcp12_38+RLcp12_39)-ROcp12_95*(RLcp12_210+RLcp12_29)-ROcp12_95*(RLcp12_27+RLcp12_28)-RLcp12_244...
 *ROcp12_95+RLcp12_344*ROcp12_85;
    JTcp12_244_6 = -(RLcp12_344*S5-ROcp12_95*(RLcp12_110+RLcp12_144+RLcp12_17+RLcp12_18+RLcp12_19)+S5*(RLcp12_310+RLcp12_39)+S5*(RLcp12_37+...
 RLcp12_38));
    JTcp12_344_6 = RLcp12_244*S5-ROcp12_85*(RLcp12_110+RLcp12_144+RLcp12_17+RLcp12_18+RLcp12_19)+S5*(RLcp12_210+RLcp12_29)+S5*(RLcp12_27+RLcp12_28);
    JTcp12_144_7 = ROcp12_56*(RLcp12_310+RLcp12_344+RLcp12_38+RLcp12_39)-ROcp12_66*(RLcp12_210+RLcp12_244)-ROcp12_66*(RLcp12_28+RLcp12_29);
    JTcp12_244_7 = -(ROcp12_46*(RLcp12_310+RLcp12_344+RLcp12_38+RLcp12_39)-ROcp12_66*(RLcp12_110+RLcp12_144)-ROcp12_66*(RLcp12_18+RLcp12_19));
    JTcp12_344_7 = ROcp12_46*(RLcp12_210+RLcp12_244+RLcp12_28+RLcp12_29)-ROcp12_56*(RLcp12_110+RLcp12_144)-ROcp12_56*(RLcp12_18+RLcp12_19);
    JTcp12_144_8 = ROcp12_27*(RLcp12_310+RLcp12_39)-ROcp12_37*(RLcp12_210+RLcp12_29)-RLcp12_244*ROcp12_37+RLcp12_344*ROcp12_27;
    JTcp12_244_8 = RLcp12_144*ROcp12_37-RLcp12_344*ROcp12_17-ROcp12_17*(RLcp12_310+RLcp12_39)+ROcp12_37*(RLcp12_110+RLcp12_19);
    JTcp12_344_8 = ROcp12_17*(RLcp12_210+RLcp12_29)-ROcp12_27*(RLcp12_110+RLcp12_19)-RLcp12_144*ROcp12_27+RLcp12_244*ROcp12_17;
    JTcp12_144_9 = ROcp12_88*(RLcp12_310+RLcp12_344)-ROcp12_98*(RLcp12_210+RLcp12_244);
    JTcp12_244_9 = -(ROcp12_78*(RLcp12_310+RLcp12_344)-ROcp12_98*(RLcp12_110+RLcp12_144));
    JTcp12_344_9 = ROcp12_78*(RLcp12_210+RLcp12_244)-ROcp12_88*(RLcp12_110+RLcp12_144);
    JTcp12_144_10 = -(RLcp12_244*ROcp12_69-RLcp12_344*ROcp12_59);
    JTcp12_244_10 = RLcp12_144*ROcp12_69-RLcp12_344*ROcp12_49;
    JTcp12_344_10 = -(RLcp12_144*ROcp12_59-RLcp12_244*ROcp12_49);
    ORcp12_144 = OMcp12_210*RLcp12_344-OMcp12_310*RLcp12_244;
    ORcp12_244 = -(OMcp12_110*RLcp12_344-OMcp12_310*RLcp12_144);
    ORcp12_344 = OMcp12_110*RLcp12_244-OMcp12_210*RLcp12_144;
    VIcp12_144 = ORcp12_110+ORcp12_144+ORcp12_17+ORcp12_18+ORcp12_19+qd(1);
    VIcp12_244 = ORcp12_210+ORcp12_244+ORcp12_27+ORcp12_28+ORcp12_29+qd(2);
    VIcp12_344 = ORcp12_310+ORcp12_344+ORcp12_37+ORcp12_38+ORcp12_39+qd(3);
    ACcp12_144 = qdd(1)+OMcp12_210*ORcp12_344+OMcp12_26*ORcp12_37+OMcp12_27*ORcp12_38+OMcp12_28*ORcp12_39+OMcp12_29*ORcp12_310-OMcp12_310*...
 ORcp12_244-OMcp12_36*ORcp12_27-OMcp12_37*ORcp12_28-OMcp12_38*ORcp12_29-OMcp12_39*ORcp12_210+OPcp12_210*RLcp12_344+OPcp12_26*RLcp12_37+OPcp12_27*...
 RLcp12_38+OPcp12_28*RLcp12_39+OPcp12_29*RLcp12_310-OPcp12_310*RLcp12_244-OPcp12_36*RLcp12_27-OPcp12_37*RLcp12_28-OPcp12_38*RLcp12_29-OPcp12_39*...
 RLcp12_210;
    ACcp12_244 = qdd(2)-OMcp12_110*ORcp12_344-OMcp12_16*ORcp12_37-OMcp12_17*ORcp12_38-OMcp12_18*ORcp12_39-OMcp12_19*ORcp12_310+OMcp12_310*...
 ORcp12_144+OMcp12_36*ORcp12_17+OMcp12_37*ORcp12_18+OMcp12_38*ORcp12_19+OMcp12_39*ORcp12_110-OPcp12_110*RLcp12_344-OPcp12_16*RLcp12_37-OPcp12_17*...
 RLcp12_38-OPcp12_18*RLcp12_39-OPcp12_19*RLcp12_310+OPcp12_310*RLcp12_144+OPcp12_36*RLcp12_17+OPcp12_37*RLcp12_18+OPcp12_38*RLcp12_19+OPcp12_39*...
 RLcp12_110;
    ACcp12_344 = qdd(3)+OMcp12_110*ORcp12_244+OMcp12_16*ORcp12_27+OMcp12_17*ORcp12_28+OMcp12_18*ORcp12_29+OMcp12_19*ORcp12_210-OMcp12_210*...
 ORcp12_144-OMcp12_26*ORcp12_17-OMcp12_27*ORcp12_18-OMcp12_28*ORcp12_19-OMcp12_29*ORcp12_110+OPcp12_110*RLcp12_244+OPcp12_16*RLcp12_27+OPcp12_17*...
 RLcp12_28+OPcp12_18*RLcp12_29+OPcp12_19*RLcp12_210-OPcp12_210*RLcp12_144-OPcp12_26*RLcp12_17-OPcp12_27*RLcp12_18-OPcp12_28*RLcp12_19-OPcp12_29*...
 RLcp12_110;

% = = Block_1_0_0_13_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp12_144;
    sens.P(2) = POcp12_244;
    sens.P(3) = POcp12_344;
    sens.R(1,1) = ROcp12_110;
    sens.R(1,2) = ROcp12_210;
    sens.R(1,3) = ROcp12_310;
    sens.R(2,1) = ROcp12_49;
    sens.R(2,2) = ROcp12_59;
    sens.R(2,3) = ROcp12_69;
    sens.R(3,1) = ROcp12_710;
    sens.R(3,2) = ROcp12_810;
    sens.R(3,3) = ROcp12_910;
    sens.V(1) = VIcp12_144;
    sens.V(2) = VIcp12_244;
    sens.V(3) = VIcp12_344;
    sens.OM(1) = OMcp12_110;
    sens.OM(2) = OMcp12_210;
    sens.OM(3) = OMcp12_310;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp12_144_5;
    sens.J(1,6) = JTcp12_144_6;
    sens.J(1,7) = JTcp12_144_7;
    sens.J(1,8) = JTcp12_144_8;
    sens.J(1,9) = JTcp12_144_9;
    sens.J(1,10) = JTcp12_144_10;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp12_244_4;
    sens.J(2,5) = JTcp12_244_5;
    sens.J(2,6) = JTcp12_244_6;
    sens.J(2,7) = JTcp12_244_7;
    sens.J(2,8) = JTcp12_244_8;
    sens.J(2,9) = JTcp12_244_9;
    sens.J(2,10) = JTcp12_244_10;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp12_344_4;
    sens.J(3,5) = JTcp12_344_5;
    sens.J(3,6) = JTcp12_344_6;
    sens.J(3,7) = JTcp12_344_7;
    sens.J(3,8) = JTcp12_344_8;
    sens.J(3,9) = JTcp12_344_9;
    sens.J(3,10) = JTcp12_344_10;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,7) = ROcp12_46;
    sens.J(4,8) = ROcp12_17;
    sens.J(4,9) = ROcp12_78;
    sens.J(4,10) = ROcp12_49;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp12_85;
    sens.J(5,7) = ROcp12_56;
    sens.J(5,8) = ROcp12_27;
    sens.J(5,9) = ROcp12_88;
    sens.J(5,10) = ROcp12_59;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp12_95;
    sens.J(6,7) = ROcp12_66;
    sens.J(6,8) = ROcp12_37;
    sens.J(6,9) = ROcp12_98;
    sens.J(6,10) = ROcp12_69;
    sens.A(1) = ACcp12_144;
    sens.A(2) = ACcp12_244;
    sens.A(3) = ACcp12_344;
    sens.OMP(1) = OPcp12_110;
    sens.OMP(2) = OPcp12_210;
    sens.OMP(3) = OPcp12_310;
 
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

% = = Block_1_0_0_14_0_2 = = 
 
% Sensor Kinematics 


    ROcp13_17 = ROcp13_16*C7-S5*S7;
    ROcp13_27 = ROcp13_26*C7-ROcp13_85*S7;
    ROcp13_37 = ROcp13_36*C7-ROcp13_95*S7;
    ROcp13_77 = ROcp13_16*S7+S5*C7;
    ROcp13_87 = ROcp13_26*S7+ROcp13_85*C7;
    ROcp13_97 = ROcp13_36*S7+ROcp13_95*C7;
    ROcp13_48 = ROcp13_46*C8+ROcp13_77*S8;
    ROcp13_58 = ROcp13_56*C8+ROcp13_87*S8;
    ROcp13_68 = ROcp13_66*C8+ROcp13_97*S8;
    ROcp13_78 = -(ROcp13_46*S8-ROcp13_77*C8);
    ROcp13_88 = -(ROcp13_56*S8-ROcp13_87*C8);
    ROcp13_98 = -(ROcp13_66*S8-ROcp13_97*C8);
    ROcp13_19 = ROcp13_17*C9+ROcp13_48*S9;
    ROcp13_29 = ROcp13_27*C9+ROcp13_58*S9;
    ROcp13_39 = ROcp13_37*C9+ROcp13_68*S9;
    ROcp13_49 = -(ROcp13_17*S9-ROcp13_48*C9);
    ROcp13_59 = -(ROcp13_27*S9-ROcp13_58*C9);
    ROcp13_69 = -(ROcp13_37*S9-ROcp13_68*C9);
    ROcp13_110 = ROcp13_19*C10-ROcp13_78*S10;
    ROcp13_210 = ROcp13_29*C10-ROcp13_88*S10;
    ROcp13_310 = ROcp13_39*C10-ROcp13_98*S10;
    ROcp13_710 = ROcp13_19*S10+ROcp13_78*C10;
    ROcp13_810 = ROcp13_29*S10+ROcp13_88*C10;
    ROcp13_910 = ROcp13_39*S10+ROcp13_98*C10;
    ROcp13_411 = ROcp13_49*C11+ROcp13_710*S11;
    ROcp13_511 = ROcp13_59*C11+ROcp13_810*S11;
    ROcp13_611 = ROcp13_69*C11+ROcp13_910*S11;
    ROcp13_711 = -(ROcp13_49*S11-ROcp13_710*C11);
    ROcp13_811 = -(ROcp13_59*S11-ROcp13_810*C11);
    ROcp13_911 = -(ROcp13_69*S11-ROcp13_910*C11);
    RLcp13_17 = ROcp13_46*s.dpt(2,1);
    RLcp13_27 = ROcp13_56*s.dpt(2,1);
    RLcp13_37 = ROcp13_66*s.dpt(2,1);
    OMcp13_17 = OMcp13_16+ROcp13_46*qd(7);
    OMcp13_27 = OMcp13_26+ROcp13_56*qd(7);
    OMcp13_37 = OMcp13_36+ROcp13_66*qd(7);
    ORcp13_17 = OMcp13_26*RLcp13_37-OMcp13_36*RLcp13_27;
    ORcp13_27 = -(OMcp13_16*RLcp13_37-OMcp13_36*RLcp13_17);
    ORcp13_37 = OMcp13_16*RLcp13_27-OMcp13_26*RLcp13_17;
    OPcp13_17 = OPcp13_16+ROcp13_46*qdd(7)+qd(7)*(OMcp13_26*ROcp13_66-OMcp13_36*ROcp13_56);
    OPcp13_27 = OPcp13_26+ROcp13_56*qdd(7)-qd(7)*(OMcp13_16*ROcp13_66-OMcp13_36*ROcp13_46);
    OPcp13_37 = OPcp13_36+ROcp13_66*qdd(7)+qd(7)*(OMcp13_16*ROcp13_56-OMcp13_26*ROcp13_46);
    RLcp13_18 = ROcp13_46*s.dpt(2,6);
    RLcp13_28 = ROcp13_56*s.dpt(2,6);
    RLcp13_38 = ROcp13_66*s.dpt(2,6);
    OMcp13_18 = OMcp13_17+ROcp13_17*qd(8);
    OMcp13_28 = OMcp13_27+ROcp13_27*qd(8);
    OMcp13_38 = OMcp13_37+ROcp13_37*qd(8);
    ORcp13_18 = OMcp13_27*RLcp13_38-OMcp13_37*RLcp13_28;
    ORcp13_28 = -(OMcp13_17*RLcp13_38-OMcp13_37*RLcp13_18);
    ORcp13_38 = OMcp13_17*RLcp13_28-OMcp13_27*RLcp13_18;
    OPcp13_18 = OPcp13_17+ROcp13_17*qdd(8)+qd(8)*(OMcp13_27*ROcp13_37-OMcp13_37*ROcp13_27);
    OPcp13_28 = OPcp13_27+ROcp13_27*qdd(8)-qd(8)*(OMcp13_17*ROcp13_37-OMcp13_37*ROcp13_17);
    OPcp13_38 = OPcp13_37+ROcp13_37*qdd(8)+qd(8)*(OMcp13_17*ROcp13_27-OMcp13_27*ROcp13_17);
    RLcp13_19 = ROcp13_78*s.dpt(3,8);
    RLcp13_29 = ROcp13_88*s.dpt(3,8);
    RLcp13_39 = ROcp13_98*s.dpt(3,8);
    OMcp13_19 = OMcp13_18+ROcp13_78*qd(9);
    OMcp13_29 = OMcp13_28+ROcp13_88*qd(9);
    OMcp13_39 = OMcp13_38+ROcp13_98*qd(9);
    ORcp13_19 = OMcp13_28*RLcp13_39-OMcp13_38*RLcp13_29;
    ORcp13_29 = -(OMcp13_18*RLcp13_39-OMcp13_38*RLcp13_19);
    ORcp13_39 = OMcp13_18*RLcp13_29-OMcp13_28*RLcp13_19;
    OPcp13_19 = OPcp13_18+ROcp13_78*qdd(9)+qd(9)*(OMcp13_28*ROcp13_98-OMcp13_38*ROcp13_88);
    OPcp13_29 = OPcp13_28+ROcp13_88*qdd(9)-qd(9)*(OMcp13_18*ROcp13_98-OMcp13_38*ROcp13_78);
    OPcp13_39 = OPcp13_38+ROcp13_98*qdd(9)+qd(9)*(OMcp13_18*ROcp13_88-OMcp13_28*ROcp13_78);
    RLcp13_110 = ROcp13_78*s.dpt(3,10);
    RLcp13_210 = ROcp13_88*s.dpt(3,10);
    RLcp13_310 = ROcp13_98*s.dpt(3,10);
    OMcp13_110 = OMcp13_19+ROcp13_49*qd(10);
    OMcp13_210 = OMcp13_29+ROcp13_59*qd(10);
    OMcp13_310 = OMcp13_39+ROcp13_69*qd(10);
    ORcp13_110 = OMcp13_29*RLcp13_310-OMcp13_39*RLcp13_210;
    ORcp13_210 = -(OMcp13_19*RLcp13_310-OMcp13_39*RLcp13_110);
    ORcp13_310 = OMcp13_19*RLcp13_210-OMcp13_29*RLcp13_110;
    OPcp13_110 = OPcp13_19+ROcp13_49*qdd(10)+qd(10)*(OMcp13_29*ROcp13_69-OMcp13_39*ROcp13_59);
    OPcp13_210 = OPcp13_29+ROcp13_59*qdd(10)-qd(10)*(OMcp13_19*ROcp13_69-OMcp13_39*ROcp13_49);
    OPcp13_310 = OPcp13_39+ROcp13_69*qdd(10)+qd(10)*(OMcp13_19*ROcp13_59-OMcp13_29*ROcp13_49);
    RLcp13_111 = ROcp13_710*s.dpt(3,12);
    RLcp13_211 = ROcp13_810*s.dpt(3,12);
    RLcp13_311 = ROcp13_910*s.dpt(3,12);
    OMcp13_111 = OMcp13_110+ROcp13_110*qd(11);
    OMcp13_211 = OMcp13_210+ROcp13_210*qd(11);
    OMcp13_311 = OMcp13_310+ROcp13_310*qd(11);
    ORcp13_111 = OMcp13_210*RLcp13_311-OMcp13_310*RLcp13_211;
    ORcp13_211 = -(OMcp13_110*RLcp13_311-OMcp13_310*RLcp13_111);
    ORcp13_311 = OMcp13_110*RLcp13_211-OMcp13_210*RLcp13_111;
    OPcp13_111 = OPcp13_110+ROcp13_110*qdd(11)+qd(11)*(OMcp13_210*ROcp13_310-OMcp13_310*ROcp13_210);
    OPcp13_211 = OPcp13_210+ROcp13_210*qdd(11)-qd(11)*(OMcp13_110*ROcp13_310-OMcp13_310*ROcp13_110);
    OPcp13_311 = OPcp13_310+ROcp13_310*qdd(11)+qd(11)*(OMcp13_110*ROcp13_210-OMcp13_210*ROcp13_110);
    RLcp13_145 = ROcp13_110*s.dpt(1,15)+ROcp13_411*s.dpt(2,15)+ROcp13_711*s.dpt(3,15);
    RLcp13_245 = ROcp13_210*s.dpt(1,15)+ROcp13_511*s.dpt(2,15)+ROcp13_811*s.dpt(3,15);
    RLcp13_345 = ROcp13_310*s.dpt(1,15)+ROcp13_611*s.dpt(2,15)+ROcp13_911*s.dpt(3,15);
    POcp13_145 = RLcp13_110+RLcp13_111+RLcp13_145+RLcp13_17+RLcp13_18+RLcp13_19+q(1);
    POcp13_245 = RLcp13_210+RLcp13_211+RLcp13_245+RLcp13_27+RLcp13_28+RLcp13_29+q(2);
    POcp13_345 = RLcp13_310+RLcp13_311+RLcp13_345+RLcp13_37+RLcp13_38+RLcp13_39+q(3);
    JTcp13_245_4 = -(RLcp13_310+RLcp13_311+RLcp13_345+RLcp13_37+RLcp13_38+RLcp13_39);
    JTcp13_345_4 = RLcp13_210+RLcp13_211+RLcp13_245+RLcp13_27+RLcp13_28+RLcp13_29;
    JTcp13_145_5 = C4*(RLcp13_310+RLcp13_311+RLcp13_345+RLcp13_37+RLcp13_38+RLcp13_39)-S4*(RLcp13_210+RLcp13_29)-S4*(RLcp13_211+RLcp13_245)-S4*(...
 RLcp13_27+RLcp13_28);
    JTcp13_245_5 = S4*(RLcp13_110+RLcp13_111+RLcp13_145+RLcp13_17+RLcp13_18+RLcp13_19);
    JTcp13_345_5 = -C4*(RLcp13_110+RLcp13_111+RLcp13_145+RLcp13_17+RLcp13_18+RLcp13_19);
    JTcp13_145_6 = ROcp13_85*(RLcp13_310+RLcp13_311+RLcp13_345+RLcp13_37+RLcp13_38+RLcp13_39)-ROcp13_95*(RLcp13_210+RLcp13_29)-ROcp13_95*(...
 RLcp13_211+RLcp13_245)-ROcp13_95*(RLcp13_27+RLcp13_28);
    JTcp13_245_6 = RLcp13_145*ROcp13_95-RLcp13_311*S5-RLcp13_345*S5+ROcp13_95*(RLcp13_110+RLcp13_111+RLcp13_17+RLcp13_18+RLcp13_19)-S5*(RLcp13_310+...
 RLcp13_39)-S5*(RLcp13_37+RLcp13_38);
    JTcp13_345_6 = RLcp13_211*S5-ROcp13_85*(RLcp13_110+RLcp13_111+RLcp13_17+RLcp13_18+RLcp13_19)+S5*(RLcp13_210+RLcp13_29)+S5*(RLcp13_27+RLcp13_28)...
 -RLcp13_145*ROcp13_85+RLcp13_245*S5;
    JTcp13_145_7 = ROcp13_56*(RLcp13_310+RLcp13_311+RLcp13_38+RLcp13_39)-ROcp13_66*(RLcp13_210+RLcp13_211)-ROcp13_66*(RLcp13_28+RLcp13_29)-...
 RLcp13_245*ROcp13_66+RLcp13_345*ROcp13_56;
    JTcp13_245_7 = RLcp13_145*ROcp13_66-RLcp13_345*ROcp13_46-ROcp13_46*(RLcp13_310+RLcp13_311+RLcp13_38+RLcp13_39)+ROcp13_66*(RLcp13_110+RLcp13_111...
 )+ROcp13_66*(RLcp13_18+RLcp13_19);
    JTcp13_345_7 = ROcp13_46*(RLcp13_210+RLcp13_211+RLcp13_28+RLcp13_29)-ROcp13_56*(RLcp13_110+RLcp13_111)-ROcp13_56*(RLcp13_18+RLcp13_19)-...
 RLcp13_145*ROcp13_56+RLcp13_245*ROcp13_46;
    JTcp13_145_8 = ROcp13_27*(RLcp13_310+RLcp13_311+RLcp13_345+RLcp13_39)-ROcp13_37*(RLcp13_210+RLcp13_29)-ROcp13_37*(RLcp13_211+RLcp13_245);
    JTcp13_245_8 = -(ROcp13_17*(RLcp13_310+RLcp13_311+RLcp13_345+RLcp13_39)-ROcp13_37*(RLcp13_110+RLcp13_19)-ROcp13_37*(RLcp13_111+RLcp13_145));
    JTcp13_345_8 = ROcp13_17*(RLcp13_210+RLcp13_211+RLcp13_245+RLcp13_29)-ROcp13_27*(RLcp13_110+RLcp13_19)-ROcp13_27*(RLcp13_111+RLcp13_145);
    JTcp13_145_9 = ROcp13_88*(RLcp13_310+RLcp13_311)-ROcp13_98*(RLcp13_210+RLcp13_211)-RLcp13_245*ROcp13_98+RLcp13_345*ROcp13_88;
    JTcp13_245_9 = RLcp13_145*ROcp13_98-RLcp13_345*ROcp13_78-ROcp13_78*(RLcp13_310+RLcp13_311)+ROcp13_98*(RLcp13_110+RLcp13_111);
    JTcp13_345_9 = ROcp13_78*(RLcp13_210+RLcp13_211)-ROcp13_88*(RLcp13_110+RLcp13_111)-RLcp13_145*ROcp13_88+RLcp13_245*ROcp13_78;
    JTcp13_145_10 = ROcp13_59*(RLcp13_311+RLcp13_345)-ROcp13_69*(RLcp13_211+RLcp13_245);
    JTcp13_245_10 = -(ROcp13_49*(RLcp13_311+RLcp13_345)-ROcp13_69*(RLcp13_111+RLcp13_145));
    JTcp13_345_10 = ROcp13_49*(RLcp13_211+RLcp13_245)-ROcp13_59*(RLcp13_111+RLcp13_145);
    JTcp13_145_11 = -(RLcp13_245*ROcp13_310-RLcp13_345*ROcp13_210);
    JTcp13_245_11 = RLcp13_145*ROcp13_310-RLcp13_345*ROcp13_110;
    JTcp13_345_11 = -(RLcp13_145*ROcp13_210-RLcp13_245*ROcp13_110);
    ORcp13_145 = OMcp13_211*RLcp13_345-OMcp13_311*RLcp13_245;
    ORcp13_245 = -(OMcp13_111*RLcp13_345-OMcp13_311*RLcp13_145);
    ORcp13_345 = OMcp13_111*RLcp13_245-OMcp13_211*RLcp13_145;
    VIcp13_145 = ORcp13_110+ORcp13_111+ORcp13_145+ORcp13_17+ORcp13_18+ORcp13_19+qd(1);
    VIcp13_245 = ORcp13_210+ORcp13_211+ORcp13_245+ORcp13_27+ORcp13_28+ORcp13_29+qd(2);
    VIcp13_345 = ORcp13_310+ORcp13_311+ORcp13_345+ORcp13_37+ORcp13_38+ORcp13_39+qd(3);
    ACcp13_145 = qdd(1)+OMcp13_210*ORcp13_311+OMcp13_211*ORcp13_345+OMcp13_26*ORcp13_37+OMcp13_27*ORcp13_38+OMcp13_28*ORcp13_39+OMcp13_29*...
 ORcp13_310-OMcp13_310*ORcp13_211-OMcp13_311*ORcp13_245-OMcp13_36*ORcp13_27-OMcp13_37*ORcp13_28-OMcp13_38*ORcp13_29-OMcp13_39*ORcp13_210+OPcp13_210*...
 RLcp13_311+OPcp13_211*RLcp13_345+OPcp13_26*RLcp13_37+OPcp13_27*RLcp13_38+OPcp13_28*RLcp13_39+OPcp13_29*RLcp13_310-OPcp13_310*RLcp13_211-OPcp13_311*...
 RLcp13_245-OPcp13_36*RLcp13_27-OPcp13_37*RLcp13_28-OPcp13_38*RLcp13_29-OPcp13_39*RLcp13_210;
    ACcp13_245 = qdd(2)-OMcp13_110*ORcp13_311-OMcp13_111*ORcp13_345-OMcp13_16*ORcp13_37-OMcp13_17*ORcp13_38-OMcp13_18*ORcp13_39-OMcp13_19*...
 ORcp13_310+OMcp13_310*ORcp13_111+OMcp13_311*ORcp13_145+OMcp13_36*ORcp13_17+OMcp13_37*ORcp13_18+OMcp13_38*ORcp13_19+OMcp13_39*ORcp13_110-OPcp13_110*...
 RLcp13_311-OPcp13_111*RLcp13_345-OPcp13_16*RLcp13_37-OPcp13_17*RLcp13_38-OPcp13_18*RLcp13_39-OPcp13_19*RLcp13_310+OPcp13_310*RLcp13_111+OPcp13_311*...
 RLcp13_145+OPcp13_36*RLcp13_17+OPcp13_37*RLcp13_18+OPcp13_38*RLcp13_19+OPcp13_39*RLcp13_110;
    ACcp13_345 = qdd(3)+OMcp13_110*ORcp13_211+OMcp13_111*ORcp13_245+OMcp13_16*ORcp13_27+OMcp13_17*ORcp13_28+OMcp13_18*ORcp13_29+OMcp13_19*...
 ORcp13_210-OMcp13_210*ORcp13_111-OMcp13_211*ORcp13_145-OMcp13_26*ORcp13_17-OMcp13_27*ORcp13_18-OMcp13_28*ORcp13_19-OMcp13_29*ORcp13_110+OPcp13_110*...
 RLcp13_211+OPcp13_111*RLcp13_245+OPcp13_16*RLcp13_27+OPcp13_17*RLcp13_28+OPcp13_18*RLcp13_29+OPcp13_19*RLcp13_210-OPcp13_210*RLcp13_111-OPcp13_211*...
 RLcp13_145-OPcp13_26*RLcp13_17-OPcp13_27*RLcp13_18-OPcp13_28*RLcp13_19-OPcp13_29*RLcp13_110;

% = = Block_1_0_0_14_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp13_145;
    sens.P(2) = POcp13_245;
    sens.P(3) = POcp13_345;
    sens.R(1,1) = ROcp13_110;
    sens.R(1,2) = ROcp13_210;
    sens.R(1,3) = ROcp13_310;
    sens.R(2,1) = ROcp13_411;
    sens.R(2,2) = ROcp13_511;
    sens.R(2,3) = ROcp13_611;
    sens.R(3,1) = ROcp13_711;
    sens.R(3,2) = ROcp13_811;
    sens.R(3,3) = ROcp13_911;
    sens.V(1) = VIcp13_145;
    sens.V(2) = VIcp13_245;
    sens.V(3) = VIcp13_345;
    sens.OM(1) = OMcp13_111;
    sens.OM(2) = OMcp13_211;
    sens.OM(3) = OMcp13_311;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp13_145_5;
    sens.J(1,6) = JTcp13_145_6;
    sens.J(1,7) = JTcp13_145_7;
    sens.J(1,8) = JTcp13_145_8;
    sens.J(1,9) = JTcp13_145_9;
    sens.J(1,10) = JTcp13_145_10;
    sens.J(1,11) = JTcp13_145_11;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp13_245_4;
    sens.J(2,5) = JTcp13_245_5;
    sens.J(2,6) = JTcp13_245_6;
    sens.J(2,7) = JTcp13_245_7;
    sens.J(2,8) = JTcp13_245_8;
    sens.J(2,9) = JTcp13_245_9;
    sens.J(2,10) = JTcp13_245_10;
    sens.J(2,11) = JTcp13_245_11;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp13_345_4;
    sens.J(3,5) = JTcp13_345_5;
    sens.J(3,6) = JTcp13_345_6;
    sens.J(3,7) = JTcp13_345_7;
    sens.J(3,8) = JTcp13_345_8;
    sens.J(3,9) = JTcp13_345_9;
    sens.J(3,10) = JTcp13_345_10;
    sens.J(3,11) = JTcp13_345_11;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,7) = ROcp13_46;
    sens.J(4,8) = ROcp13_17;
    sens.J(4,9) = ROcp13_78;
    sens.J(4,10) = ROcp13_49;
    sens.J(4,11) = ROcp13_110;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp13_85;
    sens.J(5,7) = ROcp13_56;
    sens.J(5,8) = ROcp13_27;
    sens.J(5,9) = ROcp13_88;
    sens.J(5,10) = ROcp13_59;
    sens.J(5,11) = ROcp13_210;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp13_95;
    sens.J(6,7) = ROcp13_66;
    sens.J(6,8) = ROcp13_37;
    sens.J(6,9) = ROcp13_98;
    sens.J(6,10) = ROcp13_69;
    sens.J(6,11) = ROcp13_310;
    sens.A(1) = ACcp13_145;
    sens.A(2) = ACcp13_245;
    sens.A(3) = ACcp13_345;
    sens.OMP(1) = OPcp13_111;
    sens.OMP(2) = OPcp13_211;
    sens.OMP(3) = OPcp13_311;
 
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

% = = Block_1_0_0_15_0_2 = = 
 
% Sensor Kinematics 


    ROcp14_17 = ROcp14_16*C7-S5*S7;
    ROcp14_27 = ROcp14_26*C7-ROcp14_85*S7;
    ROcp14_37 = ROcp14_36*C7-ROcp14_95*S7;
    ROcp14_77 = ROcp14_16*S7+S5*C7;
    ROcp14_87 = ROcp14_26*S7+ROcp14_85*C7;
    ROcp14_97 = ROcp14_36*S7+ROcp14_95*C7;
    ROcp14_48 = ROcp14_46*C8+ROcp14_77*S8;
    ROcp14_58 = ROcp14_56*C8+ROcp14_87*S8;
    ROcp14_68 = ROcp14_66*C8+ROcp14_97*S8;
    ROcp14_78 = -(ROcp14_46*S8-ROcp14_77*C8);
    ROcp14_88 = -(ROcp14_56*S8-ROcp14_87*C8);
    ROcp14_98 = -(ROcp14_66*S8-ROcp14_97*C8);
    ROcp14_19 = ROcp14_17*C9+ROcp14_48*S9;
    ROcp14_29 = ROcp14_27*C9+ROcp14_58*S9;
    ROcp14_39 = ROcp14_37*C9+ROcp14_68*S9;
    ROcp14_49 = -(ROcp14_17*S9-ROcp14_48*C9);
    ROcp14_59 = -(ROcp14_27*S9-ROcp14_58*C9);
    ROcp14_69 = -(ROcp14_37*S9-ROcp14_68*C9);
    ROcp14_110 = ROcp14_19*C10-ROcp14_78*S10;
    ROcp14_210 = ROcp14_29*C10-ROcp14_88*S10;
    ROcp14_310 = ROcp14_39*C10-ROcp14_98*S10;
    ROcp14_710 = ROcp14_19*S10+ROcp14_78*C10;
    ROcp14_810 = ROcp14_29*S10+ROcp14_88*C10;
    ROcp14_910 = ROcp14_39*S10+ROcp14_98*C10;
    ROcp14_411 = ROcp14_49*C11+ROcp14_710*S11;
    ROcp14_511 = ROcp14_59*C11+ROcp14_810*S11;
    ROcp14_611 = ROcp14_69*C11+ROcp14_910*S11;
    ROcp14_711 = -(ROcp14_49*S11-ROcp14_710*C11);
    ROcp14_811 = -(ROcp14_59*S11-ROcp14_810*C11);
    ROcp14_911 = -(ROcp14_69*S11-ROcp14_910*C11);
    ROcp14_112 = ROcp14_110*C12-ROcp14_711*S12;
    ROcp14_212 = ROcp14_210*C12-ROcp14_811*S12;
    ROcp14_312 = ROcp14_310*C12-ROcp14_911*S12;
    ROcp14_712 = ROcp14_110*S12+ROcp14_711*C12;
    ROcp14_812 = ROcp14_210*S12+ROcp14_811*C12;
    ROcp14_912 = ROcp14_310*S12+ROcp14_911*C12;
    RLcp14_17 = ROcp14_46*s.dpt(2,1);
    RLcp14_27 = ROcp14_56*s.dpt(2,1);
    RLcp14_37 = ROcp14_66*s.dpt(2,1);
    OMcp14_17 = OMcp14_16+ROcp14_46*qd(7);
    OMcp14_27 = OMcp14_26+ROcp14_56*qd(7);
    OMcp14_37 = OMcp14_36+ROcp14_66*qd(7);
    ORcp14_17 = OMcp14_26*RLcp14_37-OMcp14_36*RLcp14_27;
    ORcp14_27 = -(OMcp14_16*RLcp14_37-OMcp14_36*RLcp14_17);
    ORcp14_37 = OMcp14_16*RLcp14_27-OMcp14_26*RLcp14_17;
    OPcp14_17 = OPcp14_16+ROcp14_46*qdd(7)+qd(7)*(OMcp14_26*ROcp14_66-OMcp14_36*ROcp14_56);
    OPcp14_27 = OPcp14_26+ROcp14_56*qdd(7)-qd(7)*(OMcp14_16*ROcp14_66-OMcp14_36*ROcp14_46);
    OPcp14_37 = OPcp14_36+ROcp14_66*qdd(7)+qd(7)*(OMcp14_16*ROcp14_56-OMcp14_26*ROcp14_46);
    RLcp14_18 = ROcp14_46*s.dpt(2,6);
    RLcp14_28 = ROcp14_56*s.dpt(2,6);
    RLcp14_38 = ROcp14_66*s.dpt(2,6);
    OMcp14_18 = OMcp14_17+ROcp14_17*qd(8);
    OMcp14_28 = OMcp14_27+ROcp14_27*qd(8);
    OMcp14_38 = OMcp14_37+ROcp14_37*qd(8);
    ORcp14_18 = OMcp14_27*RLcp14_38-OMcp14_37*RLcp14_28;
    ORcp14_28 = -(OMcp14_17*RLcp14_38-OMcp14_37*RLcp14_18);
    ORcp14_38 = OMcp14_17*RLcp14_28-OMcp14_27*RLcp14_18;
    OPcp14_18 = OPcp14_17+ROcp14_17*qdd(8)+qd(8)*(OMcp14_27*ROcp14_37-OMcp14_37*ROcp14_27);
    OPcp14_28 = OPcp14_27+ROcp14_27*qdd(8)-qd(8)*(OMcp14_17*ROcp14_37-OMcp14_37*ROcp14_17);
    OPcp14_38 = OPcp14_37+ROcp14_37*qdd(8)+qd(8)*(OMcp14_17*ROcp14_27-OMcp14_27*ROcp14_17);
    RLcp14_19 = ROcp14_78*s.dpt(3,8);
    RLcp14_29 = ROcp14_88*s.dpt(3,8);
    RLcp14_39 = ROcp14_98*s.dpt(3,8);
    OMcp14_19 = OMcp14_18+ROcp14_78*qd(9);
    OMcp14_29 = OMcp14_28+ROcp14_88*qd(9);
    OMcp14_39 = OMcp14_38+ROcp14_98*qd(9);
    ORcp14_19 = OMcp14_28*RLcp14_39-OMcp14_38*RLcp14_29;
    ORcp14_29 = -(OMcp14_18*RLcp14_39-OMcp14_38*RLcp14_19);
    ORcp14_39 = OMcp14_18*RLcp14_29-OMcp14_28*RLcp14_19;
    OPcp14_19 = OPcp14_18+ROcp14_78*qdd(9)+qd(9)*(OMcp14_28*ROcp14_98-OMcp14_38*ROcp14_88);
    OPcp14_29 = OPcp14_28+ROcp14_88*qdd(9)-qd(9)*(OMcp14_18*ROcp14_98-OMcp14_38*ROcp14_78);
    OPcp14_39 = OPcp14_38+ROcp14_98*qdd(9)+qd(9)*(OMcp14_18*ROcp14_88-OMcp14_28*ROcp14_78);
    RLcp14_110 = ROcp14_78*s.dpt(3,10);
    RLcp14_210 = ROcp14_88*s.dpt(3,10);
    RLcp14_310 = ROcp14_98*s.dpt(3,10);
    OMcp14_110 = OMcp14_19+ROcp14_49*qd(10);
    OMcp14_210 = OMcp14_29+ROcp14_59*qd(10);
    OMcp14_310 = OMcp14_39+ROcp14_69*qd(10);
    ORcp14_110 = OMcp14_29*RLcp14_310-OMcp14_39*RLcp14_210;
    ORcp14_210 = -(OMcp14_19*RLcp14_310-OMcp14_39*RLcp14_110);
    ORcp14_310 = OMcp14_19*RLcp14_210-OMcp14_29*RLcp14_110;
    OPcp14_110 = OPcp14_19+ROcp14_49*qdd(10)+qd(10)*(OMcp14_29*ROcp14_69-OMcp14_39*ROcp14_59);
    OPcp14_210 = OPcp14_29+ROcp14_59*qdd(10)-qd(10)*(OMcp14_19*ROcp14_69-OMcp14_39*ROcp14_49);
    OPcp14_310 = OPcp14_39+ROcp14_69*qdd(10)+qd(10)*(OMcp14_19*ROcp14_59-OMcp14_29*ROcp14_49);
    RLcp14_111 = ROcp14_710*s.dpt(3,12);
    RLcp14_211 = ROcp14_810*s.dpt(3,12);
    RLcp14_311 = ROcp14_910*s.dpt(3,12);
    OMcp14_111 = OMcp14_110+ROcp14_110*qd(11);
    OMcp14_211 = OMcp14_210+ROcp14_210*qd(11);
    OMcp14_311 = OMcp14_310+ROcp14_310*qd(11);
    ORcp14_111 = OMcp14_210*RLcp14_311-OMcp14_310*RLcp14_211;
    ORcp14_211 = -(OMcp14_110*RLcp14_311-OMcp14_310*RLcp14_111);
    ORcp14_311 = OMcp14_110*RLcp14_211-OMcp14_210*RLcp14_111;
    OMcp14_112 = OMcp14_111+ROcp14_411*qd(12);
    OMcp14_212 = OMcp14_211+ROcp14_511*qd(12);
    OMcp14_312 = OMcp14_311+ROcp14_611*qd(12);
    OPcp14_112 = OPcp14_110+ROcp14_110*qdd(11)+ROcp14_411*qdd(12)+qd(11)*(OMcp14_210*ROcp14_310-OMcp14_310*ROcp14_210)+qd(12)*(OMcp14_211*...
 ROcp14_611-OMcp14_311*ROcp14_511);
    OPcp14_212 = OPcp14_210+ROcp14_210*qdd(11)+ROcp14_511*qdd(12)-qd(11)*(OMcp14_110*ROcp14_310-OMcp14_310*ROcp14_110)-qd(12)*(OMcp14_111*...
 ROcp14_611-OMcp14_311*ROcp14_411);
    OPcp14_312 = OPcp14_310+ROcp14_310*qdd(11)+ROcp14_611*qdd(12)+qd(11)*(OMcp14_110*ROcp14_210-OMcp14_210*ROcp14_110)+qd(12)*(OMcp14_111*...
 ROcp14_511-OMcp14_211*ROcp14_411);
    RLcp14_146 = ROcp14_712*s.dpt(3,16);
    RLcp14_246 = ROcp14_812*s.dpt(3,16);
    RLcp14_346 = ROcp14_912*s.dpt(3,16);
    POcp14_146 = RLcp14_110+RLcp14_111+RLcp14_146+RLcp14_17+RLcp14_18+RLcp14_19+q(1);
    POcp14_246 = RLcp14_210+RLcp14_211+RLcp14_246+RLcp14_27+RLcp14_28+RLcp14_29+q(2);
    POcp14_346 = RLcp14_310+RLcp14_311+RLcp14_346+RLcp14_37+RLcp14_38+RLcp14_39+q(3);
    JTcp14_246_4 = -(RLcp14_310+RLcp14_311+RLcp14_346+RLcp14_37+RLcp14_38+RLcp14_39);
    JTcp14_346_4 = RLcp14_210+RLcp14_211+RLcp14_246+RLcp14_27+RLcp14_28+RLcp14_29;
    JTcp14_146_5 = C4*(RLcp14_310+RLcp14_311+RLcp14_346+RLcp14_37+RLcp14_38+RLcp14_39)-S4*(RLcp14_210+RLcp14_29)-S4*(RLcp14_211+RLcp14_246)-S4*(...
 RLcp14_27+RLcp14_28);
    JTcp14_246_5 = S4*(RLcp14_110+RLcp14_111+RLcp14_146+RLcp14_17+RLcp14_18+RLcp14_19);
    JTcp14_346_5 = -C4*(RLcp14_110+RLcp14_111+RLcp14_146+RLcp14_17+RLcp14_18+RLcp14_19);
    JTcp14_146_6 = ROcp14_85*(RLcp14_310+RLcp14_311+RLcp14_346+RLcp14_37+RLcp14_38+RLcp14_39)-ROcp14_95*(RLcp14_210+RLcp14_29)-ROcp14_95*(...
 RLcp14_211+RLcp14_246)-ROcp14_95*(RLcp14_27+RLcp14_28);
    JTcp14_246_6 = RLcp14_146*ROcp14_95-RLcp14_311*S5-RLcp14_346*S5+ROcp14_95*(RLcp14_110+RLcp14_111+RLcp14_17+RLcp14_18+RLcp14_19)-S5*(RLcp14_310+...
 RLcp14_39)-S5*(RLcp14_37+RLcp14_38);
    JTcp14_346_6 = RLcp14_211*S5-ROcp14_85*(RLcp14_110+RLcp14_111+RLcp14_17+RLcp14_18+RLcp14_19)+S5*(RLcp14_210+RLcp14_29)+S5*(RLcp14_27+RLcp14_28)...
 -RLcp14_146*ROcp14_85+RLcp14_246*S5;
    JTcp14_146_7 = ROcp14_56*(RLcp14_310+RLcp14_311+RLcp14_38+RLcp14_39)-ROcp14_66*(RLcp14_210+RLcp14_211)-ROcp14_66*(RLcp14_28+RLcp14_29)-...
 RLcp14_246*ROcp14_66+RLcp14_346*ROcp14_56;
    JTcp14_246_7 = RLcp14_146*ROcp14_66-RLcp14_346*ROcp14_46-ROcp14_46*(RLcp14_310+RLcp14_311+RLcp14_38+RLcp14_39)+ROcp14_66*(RLcp14_110+RLcp14_111...
 )+ROcp14_66*(RLcp14_18+RLcp14_19);
    JTcp14_346_7 = ROcp14_46*(RLcp14_210+RLcp14_211+RLcp14_28+RLcp14_29)-ROcp14_56*(RLcp14_110+RLcp14_111)-ROcp14_56*(RLcp14_18+RLcp14_19)-...
 RLcp14_146*ROcp14_56+RLcp14_246*ROcp14_46;
    JTcp14_146_8 = ROcp14_27*(RLcp14_310+RLcp14_311+RLcp14_346+RLcp14_39)-ROcp14_37*(RLcp14_210+RLcp14_29)-ROcp14_37*(RLcp14_211+RLcp14_246);
    JTcp14_246_8 = -(ROcp14_17*(RLcp14_310+RLcp14_311+RLcp14_346+RLcp14_39)-ROcp14_37*(RLcp14_110+RLcp14_19)-ROcp14_37*(RLcp14_111+RLcp14_146));
    JTcp14_346_8 = ROcp14_17*(RLcp14_210+RLcp14_211+RLcp14_246+RLcp14_29)-ROcp14_27*(RLcp14_110+RLcp14_19)-ROcp14_27*(RLcp14_111+RLcp14_146);
    JTcp14_146_9 = ROcp14_88*(RLcp14_310+RLcp14_311)-ROcp14_98*(RLcp14_210+RLcp14_211)-RLcp14_246*ROcp14_98+RLcp14_346*ROcp14_88;
    JTcp14_246_9 = RLcp14_146*ROcp14_98-RLcp14_346*ROcp14_78-ROcp14_78*(RLcp14_310+RLcp14_311)+ROcp14_98*(RLcp14_110+RLcp14_111);
    JTcp14_346_9 = ROcp14_78*(RLcp14_210+RLcp14_211)-ROcp14_88*(RLcp14_110+RLcp14_111)-RLcp14_146*ROcp14_88+RLcp14_246*ROcp14_78;
    JTcp14_146_10 = ROcp14_59*(RLcp14_311+RLcp14_346)-ROcp14_69*(RLcp14_211+RLcp14_246);
    JTcp14_246_10 = -(ROcp14_49*(RLcp14_311+RLcp14_346)-ROcp14_69*(RLcp14_111+RLcp14_146));
    JTcp14_346_10 = ROcp14_49*(RLcp14_211+RLcp14_246)-ROcp14_59*(RLcp14_111+RLcp14_146);
    JTcp14_146_11 = -(RLcp14_246*ROcp14_310-RLcp14_346*ROcp14_210);
    JTcp14_246_11 = RLcp14_146*ROcp14_310-RLcp14_346*ROcp14_110;
    JTcp14_346_11 = -(RLcp14_146*ROcp14_210-RLcp14_246*ROcp14_110);
    JTcp14_146_12 = -(RLcp14_246*ROcp14_611-RLcp14_346*ROcp14_511);
    JTcp14_246_12 = RLcp14_146*ROcp14_611-RLcp14_346*ROcp14_411;
    JTcp14_346_12 = -(RLcp14_146*ROcp14_511-RLcp14_246*ROcp14_411);
    ORcp14_146 = OMcp14_212*RLcp14_346-OMcp14_312*RLcp14_246;
    ORcp14_246 = -(OMcp14_112*RLcp14_346-OMcp14_312*RLcp14_146);
    ORcp14_346 = OMcp14_112*RLcp14_246-OMcp14_212*RLcp14_146;
    VIcp14_146 = ORcp14_110+ORcp14_111+ORcp14_146+ORcp14_17+ORcp14_18+ORcp14_19+qd(1);
    VIcp14_246 = ORcp14_210+ORcp14_211+ORcp14_246+ORcp14_27+ORcp14_28+ORcp14_29+qd(2);
    VIcp14_346 = ORcp14_310+ORcp14_311+ORcp14_346+ORcp14_37+ORcp14_38+ORcp14_39+qd(3);
    ACcp14_146 = qdd(1)+OMcp14_210*ORcp14_311+OMcp14_212*ORcp14_346+OMcp14_26*ORcp14_37+OMcp14_27*ORcp14_38+OMcp14_28*ORcp14_39+OMcp14_29*...
 ORcp14_310-OMcp14_310*ORcp14_211-OMcp14_312*ORcp14_246-OMcp14_36*ORcp14_27-OMcp14_37*ORcp14_28-OMcp14_38*ORcp14_29-OMcp14_39*ORcp14_210+OPcp14_210*...
 RLcp14_311+OPcp14_212*RLcp14_346+OPcp14_26*RLcp14_37+OPcp14_27*RLcp14_38+OPcp14_28*RLcp14_39+OPcp14_29*RLcp14_310-OPcp14_310*RLcp14_211-OPcp14_312*...
 RLcp14_246-OPcp14_36*RLcp14_27-OPcp14_37*RLcp14_28-OPcp14_38*RLcp14_29-OPcp14_39*RLcp14_210;
    ACcp14_246 = qdd(2)-OMcp14_110*ORcp14_311-OMcp14_112*ORcp14_346-OMcp14_16*ORcp14_37-OMcp14_17*ORcp14_38-OMcp14_18*ORcp14_39-OMcp14_19*...
 ORcp14_310+OMcp14_310*ORcp14_111+OMcp14_312*ORcp14_146+OMcp14_36*ORcp14_17+OMcp14_37*ORcp14_18+OMcp14_38*ORcp14_19+OMcp14_39*ORcp14_110-OPcp14_110*...
 RLcp14_311-OPcp14_112*RLcp14_346-OPcp14_16*RLcp14_37-OPcp14_17*RLcp14_38-OPcp14_18*RLcp14_39-OPcp14_19*RLcp14_310+OPcp14_310*RLcp14_111+OPcp14_312*...
 RLcp14_146+OPcp14_36*RLcp14_17+OPcp14_37*RLcp14_18+OPcp14_38*RLcp14_19+OPcp14_39*RLcp14_110;
    ACcp14_346 = qdd(3)+OMcp14_110*ORcp14_211+OMcp14_112*ORcp14_246+OMcp14_16*ORcp14_27+OMcp14_17*ORcp14_28+OMcp14_18*ORcp14_29+OMcp14_19*...
 ORcp14_210-OMcp14_210*ORcp14_111-OMcp14_212*ORcp14_146-OMcp14_26*ORcp14_17-OMcp14_27*ORcp14_18-OMcp14_28*ORcp14_19-OMcp14_29*ORcp14_110+OPcp14_110*...
 RLcp14_211+OPcp14_112*RLcp14_246+OPcp14_16*RLcp14_27+OPcp14_17*RLcp14_28+OPcp14_18*RLcp14_29+OPcp14_19*RLcp14_210-OPcp14_210*RLcp14_111-OPcp14_212*...
 RLcp14_146-OPcp14_26*RLcp14_17-OPcp14_27*RLcp14_18-OPcp14_28*RLcp14_19-OPcp14_29*RLcp14_110;

% = = Block_1_0_0_15_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp14_146;
    sens.P(2) = POcp14_246;
    sens.P(3) = POcp14_346;
    sens.R(1,1) = ROcp14_112;
    sens.R(1,2) = ROcp14_212;
    sens.R(1,3) = ROcp14_312;
    sens.R(2,1) = ROcp14_411;
    sens.R(2,2) = ROcp14_511;
    sens.R(2,3) = ROcp14_611;
    sens.R(3,1) = ROcp14_712;
    sens.R(3,2) = ROcp14_812;
    sens.R(3,3) = ROcp14_912;
    sens.V(1) = VIcp14_146;
    sens.V(2) = VIcp14_246;
    sens.V(3) = VIcp14_346;
    sens.OM(1) = OMcp14_112;
    sens.OM(2) = OMcp14_212;
    sens.OM(3) = OMcp14_312;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp14_146_5;
    sens.J(1,6) = JTcp14_146_6;
    sens.J(1,7) = JTcp14_146_7;
    sens.J(1,8) = JTcp14_146_8;
    sens.J(1,9) = JTcp14_146_9;
    sens.J(1,10) = JTcp14_146_10;
    sens.J(1,11) = JTcp14_146_11;
    sens.J(1,12) = JTcp14_146_12;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp14_246_4;
    sens.J(2,5) = JTcp14_246_5;
    sens.J(2,6) = JTcp14_246_6;
    sens.J(2,7) = JTcp14_246_7;
    sens.J(2,8) = JTcp14_246_8;
    sens.J(2,9) = JTcp14_246_9;
    sens.J(2,10) = JTcp14_246_10;
    sens.J(2,11) = JTcp14_246_11;
    sens.J(2,12) = JTcp14_246_12;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp14_346_4;
    sens.J(3,5) = JTcp14_346_5;
    sens.J(3,6) = JTcp14_346_6;
    sens.J(3,7) = JTcp14_346_7;
    sens.J(3,8) = JTcp14_346_8;
    sens.J(3,9) = JTcp14_346_9;
    sens.J(3,10) = JTcp14_346_10;
    sens.J(3,11) = JTcp14_346_11;
    sens.J(3,12) = JTcp14_346_12;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,7) = ROcp14_46;
    sens.J(4,8) = ROcp14_17;
    sens.J(4,9) = ROcp14_78;
    sens.J(4,10) = ROcp14_49;
    sens.J(4,11) = ROcp14_110;
    sens.J(4,12) = ROcp14_411;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp14_85;
    sens.J(5,7) = ROcp14_56;
    sens.J(5,8) = ROcp14_27;
    sens.J(5,9) = ROcp14_88;
    sens.J(5,10) = ROcp14_59;
    sens.J(5,11) = ROcp14_210;
    sens.J(5,12) = ROcp14_511;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp14_95;
    sens.J(6,7) = ROcp14_66;
    sens.J(6,8) = ROcp14_37;
    sens.J(6,9) = ROcp14_98;
    sens.J(6,10) = ROcp14_69;
    sens.J(6,11) = ROcp14_310;
    sens.J(6,12) = ROcp14_611;
    sens.A(1) = ACcp14_146;
    sens.A(2) = ACcp14_246;
    sens.A(3) = ACcp14_346;
    sens.OMP(1) = OPcp14_112;
    sens.OMP(2) = OPcp14_212;
    sens.OMP(3) = OPcp14_312;
 
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

% = = Block_1_0_0_16_0_2 = = 
 
% Sensor Kinematics 


    ROcp15_17 = ROcp15_16*C7-S5*S7;
    ROcp15_27 = ROcp15_26*C7-ROcp15_85*S7;
    ROcp15_37 = ROcp15_36*C7-ROcp15_95*S7;
    ROcp15_77 = ROcp15_16*S7+S5*C7;
    ROcp15_87 = ROcp15_26*S7+ROcp15_85*C7;
    ROcp15_97 = ROcp15_36*S7+ROcp15_95*C7;
    ROcp15_48 = ROcp15_46*C8+ROcp15_77*S8;
    ROcp15_58 = ROcp15_56*C8+ROcp15_87*S8;
    ROcp15_68 = ROcp15_66*C8+ROcp15_97*S8;
    ROcp15_78 = -(ROcp15_46*S8-ROcp15_77*C8);
    ROcp15_88 = -(ROcp15_56*S8-ROcp15_87*C8);
    ROcp15_98 = -(ROcp15_66*S8-ROcp15_97*C8);
    ROcp15_19 = ROcp15_17*C9+ROcp15_48*S9;
    ROcp15_29 = ROcp15_27*C9+ROcp15_58*S9;
    ROcp15_39 = ROcp15_37*C9+ROcp15_68*S9;
    ROcp15_49 = -(ROcp15_17*S9-ROcp15_48*C9);
    ROcp15_59 = -(ROcp15_27*S9-ROcp15_58*C9);
    ROcp15_69 = -(ROcp15_37*S9-ROcp15_68*C9);
    ROcp15_110 = ROcp15_19*C10-ROcp15_78*S10;
    ROcp15_210 = ROcp15_29*C10-ROcp15_88*S10;
    ROcp15_310 = ROcp15_39*C10-ROcp15_98*S10;
    ROcp15_710 = ROcp15_19*S10+ROcp15_78*C10;
    ROcp15_810 = ROcp15_29*S10+ROcp15_88*C10;
    ROcp15_910 = ROcp15_39*S10+ROcp15_98*C10;
    ROcp15_411 = ROcp15_49*C11+ROcp15_710*S11;
    ROcp15_511 = ROcp15_59*C11+ROcp15_810*S11;
    ROcp15_611 = ROcp15_69*C11+ROcp15_910*S11;
    ROcp15_711 = -(ROcp15_49*S11-ROcp15_710*C11);
    ROcp15_811 = -(ROcp15_59*S11-ROcp15_810*C11);
    ROcp15_911 = -(ROcp15_69*S11-ROcp15_910*C11);
    ROcp15_112 = ROcp15_110*C12-ROcp15_711*S12;
    ROcp15_212 = ROcp15_210*C12-ROcp15_811*S12;
    ROcp15_312 = ROcp15_310*C12-ROcp15_911*S12;
    ROcp15_712 = ROcp15_110*S12+ROcp15_711*C12;
    ROcp15_812 = ROcp15_210*S12+ROcp15_811*C12;
    ROcp15_912 = ROcp15_310*S12+ROcp15_911*C12;
    RLcp15_17 = ROcp15_46*s.dpt(2,1);
    RLcp15_27 = ROcp15_56*s.dpt(2,1);
    RLcp15_37 = ROcp15_66*s.dpt(2,1);
    OMcp15_17 = OMcp15_16+ROcp15_46*qd(7);
    OMcp15_27 = OMcp15_26+ROcp15_56*qd(7);
    OMcp15_37 = OMcp15_36+ROcp15_66*qd(7);
    ORcp15_17 = OMcp15_26*RLcp15_37-OMcp15_36*RLcp15_27;
    ORcp15_27 = -(OMcp15_16*RLcp15_37-OMcp15_36*RLcp15_17);
    ORcp15_37 = OMcp15_16*RLcp15_27-OMcp15_26*RLcp15_17;
    OPcp15_17 = OPcp15_16+ROcp15_46*qdd(7)+qd(7)*(OMcp15_26*ROcp15_66-OMcp15_36*ROcp15_56);
    OPcp15_27 = OPcp15_26+ROcp15_56*qdd(7)-qd(7)*(OMcp15_16*ROcp15_66-OMcp15_36*ROcp15_46);
    OPcp15_37 = OPcp15_36+ROcp15_66*qdd(7)+qd(7)*(OMcp15_16*ROcp15_56-OMcp15_26*ROcp15_46);
    RLcp15_18 = ROcp15_46*s.dpt(2,6);
    RLcp15_28 = ROcp15_56*s.dpt(2,6);
    RLcp15_38 = ROcp15_66*s.dpt(2,6);
    OMcp15_18 = OMcp15_17+ROcp15_17*qd(8);
    OMcp15_28 = OMcp15_27+ROcp15_27*qd(8);
    OMcp15_38 = OMcp15_37+ROcp15_37*qd(8);
    ORcp15_18 = OMcp15_27*RLcp15_38-OMcp15_37*RLcp15_28;
    ORcp15_28 = -(OMcp15_17*RLcp15_38-OMcp15_37*RLcp15_18);
    ORcp15_38 = OMcp15_17*RLcp15_28-OMcp15_27*RLcp15_18;
    OPcp15_18 = OPcp15_17+ROcp15_17*qdd(8)+qd(8)*(OMcp15_27*ROcp15_37-OMcp15_37*ROcp15_27);
    OPcp15_28 = OPcp15_27+ROcp15_27*qdd(8)-qd(8)*(OMcp15_17*ROcp15_37-OMcp15_37*ROcp15_17);
    OPcp15_38 = OPcp15_37+ROcp15_37*qdd(8)+qd(8)*(OMcp15_17*ROcp15_27-OMcp15_27*ROcp15_17);
    RLcp15_19 = ROcp15_78*s.dpt(3,8);
    RLcp15_29 = ROcp15_88*s.dpt(3,8);
    RLcp15_39 = ROcp15_98*s.dpt(3,8);
    OMcp15_19 = OMcp15_18+ROcp15_78*qd(9);
    OMcp15_29 = OMcp15_28+ROcp15_88*qd(9);
    OMcp15_39 = OMcp15_38+ROcp15_98*qd(9);
    ORcp15_19 = OMcp15_28*RLcp15_39-OMcp15_38*RLcp15_29;
    ORcp15_29 = -(OMcp15_18*RLcp15_39-OMcp15_38*RLcp15_19);
    ORcp15_39 = OMcp15_18*RLcp15_29-OMcp15_28*RLcp15_19;
    OPcp15_19 = OPcp15_18+ROcp15_78*qdd(9)+qd(9)*(OMcp15_28*ROcp15_98-OMcp15_38*ROcp15_88);
    OPcp15_29 = OPcp15_28+ROcp15_88*qdd(9)-qd(9)*(OMcp15_18*ROcp15_98-OMcp15_38*ROcp15_78);
    OPcp15_39 = OPcp15_38+ROcp15_98*qdd(9)+qd(9)*(OMcp15_18*ROcp15_88-OMcp15_28*ROcp15_78);
    RLcp15_110 = ROcp15_78*s.dpt(3,10);
    RLcp15_210 = ROcp15_88*s.dpt(3,10);
    RLcp15_310 = ROcp15_98*s.dpt(3,10);
    OMcp15_110 = OMcp15_19+ROcp15_49*qd(10);
    OMcp15_210 = OMcp15_29+ROcp15_59*qd(10);
    OMcp15_310 = OMcp15_39+ROcp15_69*qd(10);
    ORcp15_110 = OMcp15_29*RLcp15_310-OMcp15_39*RLcp15_210;
    ORcp15_210 = -(OMcp15_19*RLcp15_310-OMcp15_39*RLcp15_110);
    ORcp15_310 = OMcp15_19*RLcp15_210-OMcp15_29*RLcp15_110;
    OPcp15_110 = OPcp15_19+ROcp15_49*qdd(10)+qd(10)*(OMcp15_29*ROcp15_69-OMcp15_39*ROcp15_59);
    OPcp15_210 = OPcp15_29+ROcp15_59*qdd(10)-qd(10)*(OMcp15_19*ROcp15_69-OMcp15_39*ROcp15_49);
    OPcp15_310 = OPcp15_39+ROcp15_69*qdd(10)+qd(10)*(OMcp15_19*ROcp15_59-OMcp15_29*ROcp15_49);
    RLcp15_111 = ROcp15_710*s.dpt(3,12);
    RLcp15_211 = ROcp15_810*s.dpt(3,12);
    RLcp15_311 = ROcp15_910*s.dpt(3,12);
    OMcp15_111 = OMcp15_110+ROcp15_110*qd(11);
    OMcp15_211 = OMcp15_210+ROcp15_210*qd(11);
    OMcp15_311 = OMcp15_310+ROcp15_310*qd(11);
    ORcp15_111 = OMcp15_210*RLcp15_311-OMcp15_310*RLcp15_211;
    ORcp15_211 = -(OMcp15_110*RLcp15_311-OMcp15_310*RLcp15_111);
    ORcp15_311 = OMcp15_110*RLcp15_211-OMcp15_210*RLcp15_111;
    OMcp15_112 = OMcp15_111+ROcp15_411*qd(12);
    OMcp15_212 = OMcp15_211+ROcp15_511*qd(12);
    OMcp15_312 = OMcp15_311+ROcp15_611*qd(12);
    OPcp15_112 = OPcp15_110+ROcp15_110*qdd(11)+ROcp15_411*qdd(12)+qd(11)*(OMcp15_210*ROcp15_310-OMcp15_310*ROcp15_210)+qd(12)*(OMcp15_211*...
 ROcp15_611-OMcp15_311*ROcp15_511);
    OPcp15_212 = OPcp15_210+ROcp15_210*qdd(11)+ROcp15_511*qdd(12)-qd(11)*(OMcp15_110*ROcp15_310-OMcp15_310*ROcp15_110)-qd(12)*(OMcp15_111*...
 ROcp15_611-OMcp15_311*ROcp15_411);
    OPcp15_312 = OPcp15_310+ROcp15_310*qdd(11)+ROcp15_611*qdd(12)+qd(11)*(OMcp15_110*ROcp15_210-OMcp15_210*ROcp15_110)+qd(12)*(OMcp15_111*...
 ROcp15_511-OMcp15_211*ROcp15_411);
    RLcp15_147 = ROcp15_112*s.dpt(1,17)+ROcp15_411*s.dpt(2,17)+ROcp15_712*s.dpt(3,17);
    RLcp15_247 = ROcp15_212*s.dpt(1,17)+ROcp15_511*s.dpt(2,17)+ROcp15_812*s.dpt(3,17);
    RLcp15_347 = ROcp15_312*s.dpt(1,17)+ROcp15_611*s.dpt(2,17)+ROcp15_912*s.dpt(3,17);
    POcp15_147 = RLcp15_110+RLcp15_111+RLcp15_147+RLcp15_17+RLcp15_18+RLcp15_19+q(1);
    POcp15_247 = RLcp15_210+RLcp15_211+RLcp15_247+RLcp15_27+RLcp15_28+RLcp15_29+q(2);
    POcp15_347 = RLcp15_310+RLcp15_311+RLcp15_347+RLcp15_37+RLcp15_38+RLcp15_39+q(3);
    JTcp15_247_4 = -(RLcp15_310+RLcp15_311+RLcp15_347+RLcp15_37+RLcp15_38+RLcp15_39);
    JTcp15_347_4 = RLcp15_210+RLcp15_211+RLcp15_247+RLcp15_27+RLcp15_28+RLcp15_29;
    JTcp15_147_5 = C4*(RLcp15_310+RLcp15_311+RLcp15_347+RLcp15_37+RLcp15_38+RLcp15_39)-S4*(RLcp15_210+RLcp15_29)-S4*(RLcp15_211+RLcp15_247)-S4*(...
 RLcp15_27+RLcp15_28);
    JTcp15_247_5 = S4*(RLcp15_110+RLcp15_111+RLcp15_147+RLcp15_17+RLcp15_18+RLcp15_19);
    JTcp15_347_5 = -C4*(RLcp15_110+RLcp15_111+RLcp15_147+RLcp15_17+RLcp15_18+RLcp15_19);
    JTcp15_147_6 = ROcp15_85*(RLcp15_310+RLcp15_311+RLcp15_347+RLcp15_37+RLcp15_38+RLcp15_39)-ROcp15_95*(RLcp15_210+RLcp15_29)-ROcp15_95*(...
 RLcp15_211+RLcp15_247)-ROcp15_95*(RLcp15_27+RLcp15_28);
    JTcp15_247_6 = RLcp15_147*ROcp15_95-RLcp15_311*S5-RLcp15_347*S5+ROcp15_95*(RLcp15_110+RLcp15_111+RLcp15_17+RLcp15_18+RLcp15_19)-S5*(RLcp15_310+...
 RLcp15_39)-S5*(RLcp15_37+RLcp15_38);
    JTcp15_347_6 = RLcp15_211*S5-ROcp15_85*(RLcp15_110+RLcp15_111+RLcp15_17+RLcp15_18+RLcp15_19)+S5*(RLcp15_210+RLcp15_29)+S5*(RLcp15_27+RLcp15_28)...
 -RLcp15_147*ROcp15_85+RLcp15_247*S5;
    JTcp15_147_7 = ROcp15_56*(RLcp15_310+RLcp15_311+RLcp15_38+RLcp15_39)-ROcp15_66*(RLcp15_210+RLcp15_211)-ROcp15_66*(RLcp15_28+RLcp15_29)-...
 RLcp15_247*ROcp15_66+RLcp15_347*ROcp15_56;
    JTcp15_247_7 = RLcp15_147*ROcp15_66-RLcp15_347*ROcp15_46-ROcp15_46*(RLcp15_310+RLcp15_311+RLcp15_38+RLcp15_39)+ROcp15_66*(RLcp15_110+RLcp15_111...
 )+ROcp15_66*(RLcp15_18+RLcp15_19);
    JTcp15_347_7 = ROcp15_46*(RLcp15_210+RLcp15_211+RLcp15_28+RLcp15_29)-ROcp15_56*(RLcp15_110+RLcp15_111)-ROcp15_56*(RLcp15_18+RLcp15_19)-...
 RLcp15_147*ROcp15_56+RLcp15_247*ROcp15_46;
    JTcp15_147_8 = ROcp15_27*(RLcp15_310+RLcp15_311+RLcp15_347+RLcp15_39)-ROcp15_37*(RLcp15_210+RLcp15_29)-ROcp15_37*(RLcp15_211+RLcp15_247);
    JTcp15_247_8 = -(ROcp15_17*(RLcp15_310+RLcp15_311+RLcp15_347+RLcp15_39)-ROcp15_37*(RLcp15_110+RLcp15_19)-ROcp15_37*(RLcp15_111+RLcp15_147));
    JTcp15_347_8 = ROcp15_17*(RLcp15_210+RLcp15_211+RLcp15_247+RLcp15_29)-ROcp15_27*(RLcp15_110+RLcp15_19)-ROcp15_27*(RLcp15_111+RLcp15_147);
    JTcp15_147_9 = ROcp15_88*(RLcp15_310+RLcp15_311)-ROcp15_98*(RLcp15_210+RLcp15_211)-RLcp15_247*ROcp15_98+RLcp15_347*ROcp15_88;
    JTcp15_247_9 = RLcp15_147*ROcp15_98-RLcp15_347*ROcp15_78-ROcp15_78*(RLcp15_310+RLcp15_311)+ROcp15_98*(RLcp15_110+RLcp15_111);
    JTcp15_347_9 = ROcp15_78*(RLcp15_210+RLcp15_211)-ROcp15_88*(RLcp15_110+RLcp15_111)-RLcp15_147*ROcp15_88+RLcp15_247*ROcp15_78;
    JTcp15_147_10 = ROcp15_59*(RLcp15_311+RLcp15_347)-ROcp15_69*(RLcp15_211+RLcp15_247);
    JTcp15_247_10 = -(ROcp15_49*(RLcp15_311+RLcp15_347)-ROcp15_69*(RLcp15_111+RLcp15_147));
    JTcp15_347_10 = ROcp15_49*(RLcp15_211+RLcp15_247)-ROcp15_59*(RLcp15_111+RLcp15_147);
    JTcp15_147_11 = -(RLcp15_247*ROcp15_310-RLcp15_347*ROcp15_210);
    JTcp15_247_11 = RLcp15_147*ROcp15_310-RLcp15_347*ROcp15_110;
    JTcp15_347_11 = -(RLcp15_147*ROcp15_210-RLcp15_247*ROcp15_110);
    JTcp15_147_12 = -(RLcp15_247*ROcp15_611-RLcp15_347*ROcp15_511);
    JTcp15_247_12 = RLcp15_147*ROcp15_611-RLcp15_347*ROcp15_411;
    JTcp15_347_12 = -(RLcp15_147*ROcp15_511-RLcp15_247*ROcp15_411);
    ORcp15_147 = OMcp15_212*RLcp15_347-OMcp15_312*RLcp15_247;
    ORcp15_247 = -(OMcp15_112*RLcp15_347-OMcp15_312*RLcp15_147);
    ORcp15_347 = OMcp15_112*RLcp15_247-OMcp15_212*RLcp15_147;
    VIcp15_147 = ORcp15_110+ORcp15_111+ORcp15_147+ORcp15_17+ORcp15_18+ORcp15_19+qd(1);
    VIcp15_247 = ORcp15_210+ORcp15_211+ORcp15_247+ORcp15_27+ORcp15_28+ORcp15_29+qd(2);
    VIcp15_347 = ORcp15_310+ORcp15_311+ORcp15_347+ORcp15_37+ORcp15_38+ORcp15_39+qd(3);
    ACcp15_147 = qdd(1)+OMcp15_210*ORcp15_311+OMcp15_212*ORcp15_347+OMcp15_26*ORcp15_37+OMcp15_27*ORcp15_38+OMcp15_28*ORcp15_39+OMcp15_29*...
 ORcp15_310-OMcp15_310*ORcp15_211-OMcp15_312*ORcp15_247-OMcp15_36*ORcp15_27-OMcp15_37*ORcp15_28-OMcp15_38*ORcp15_29-OMcp15_39*ORcp15_210+OPcp15_210*...
 RLcp15_311+OPcp15_212*RLcp15_347+OPcp15_26*RLcp15_37+OPcp15_27*RLcp15_38+OPcp15_28*RLcp15_39+OPcp15_29*RLcp15_310-OPcp15_310*RLcp15_211-OPcp15_312*...
 RLcp15_247-OPcp15_36*RLcp15_27-OPcp15_37*RLcp15_28-OPcp15_38*RLcp15_29-OPcp15_39*RLcp15_210;
    ACcp15_247 = qdd(2)-OMcp15_110*ORcp15_311-OMcp15_112*ORcp15_347-OMcp15_16*ORcp15_37-OMcp15_17*ORcp15_38-OMcp15_18*ORcp15_39-OMcp15_19*...
 ORcp15_310+OMcp15_310*ORcp15_111+OMcp15_312*ORcp15_147+OMcp15_36*ORcp15_17+OMcp15_37*ORcp15_18+OMcp15_38*ORcp15_19+OMcp15_39*ORcp15_110-OPcp15_110*...
 RLcp15_311-OPcp15_112*RLcp15_347-OPcp15_16*RLcp15_37-OPcp15_17*RLcp15_38-OPcp15_18*RLcp15_39-OPcp15_19*RLcp15_310+OPcp15_310*RLcp15_111+OPcp15_312*...
 RLcp15_147+OPcp15_36*RLcp15_17+OPcp15_37*RLcp15_18+OPcp15_38*RLcp15_19+OPcp15_39*RLcp15_110;
    ACcp15_347 = qdd(3)+OMcp15_110*ORcp15_211+OMcp15_112*ORcp15_247+OMcp15_16*ORcp15_27+OMcp15_17*ORcp15_28+OMcp15_18*ORcp15_29+OMcp15_19*...
 ORcp15_210-OMcp15_210*ORcp15_111-OMcp15_212*ORcp15_147-OMcp15_26*ORcp15_17-OMcp15_27*ORcp15_18-OMcp15_28*ORcp15_19-OMcp15_29*ORcp15_110+OPcp15_110*...
 RLcp15_211+OPcp15_112*RLcp15_247+OPcp15_16*RLcp15_27+OPcp15_17*RLcp15_28+OPcp15_18*RLcp15_29+OPcp15_19*RLcp15_210-OPcp15_210*RLcp15_111-OPcp15_212*...
 RLcp15_147-OPcp15_26*RLcp15_17-OPcp15_27*RLcp15_18-OPcp15_28*RLcp15_19-OPcp15_29*RLcp15_110;

% = = Block_1_0_0_16_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp15_147;
    sens.P(2) = POcp15_247;
    sens.P(3) = POcp15_347;
    sens.R(1,1) = ROcp15_112;
    sens.R(1,2) = ROcp15_212;
    sens.R(1,3) = ROcp15_312;
    sens.R(2,1) = ROcp15_411;
    sens.R(2,2) = ROcp15_511;
    sens.R(2,3) = ROcp15_611;
    sens.R(3,1) = ROcp15_712;
    sens.R(3,2) = ROcp15_812;
    sens.R(3,3) = ROcp15_912;
    sens.V(1) = VIcp15_147;
    sens.V(2) = VIcp15_247;
    sens.V(3) = VIcp15_347;
    sens.OM(1) = OMcp15_112;
    sens.OM(2) = OMcp15_212;
    sens.OM(3) = OMcp15_312;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp15_147_5;
    sens.J(1,6) = JTcp15_147_6;
    sens.J(1,7) = JTcp15_147_7;
    sens.J(1,8) = JTcp15_147_8;
    sens.J(1,9) = JTcp15_147_9;
    sens.J(1,10) = JTcp15_147_10;
    sens.J(1,11) = JTcp15_147_11;
    sens.J(1,12) = JTcp15_147_12;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp15_247_4;
    sens.J(2,5) = JTcp15_247_5;
    sens.J(2,6) = JTcp15_247_6;
    sens.J(2,7) = JTcp15_247_7;
    sens.J(2,8) = JTcp15_247_8;
    sens.J(2,9) = JTcp15_247_9;
    sens.J(2,10) = JTcp15_247_10;
    sens.J(2,11) = JTcp15_247_11;
    sens.J(2,12) = JTcp15_247_12;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp15_347_4;
    sens.J(3,5) = JTcp15_347_5;
    sens.J(3,6) = JTcp15_347_6;
    sens.J(3,7) = JTcp15_347_7;
    sens.J(3,8) = JTcp15_347_8;
    sens.J(3,9) = JTcp15_347_9;
    sens.J(3,10) = JTcp15_347_10;
    sens.J(3,11) = JTcp15_347_11;
    sens.J(3,12) = JTcp15_347_12;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,7) = ROcp15_46;
    sens.J(4,8) = ROcp15_17;
    sens.J(4,9) = ROcp15_78;
    sens.J(4,10) = ROcp15_49;
    sens.J(4,11) = ROcp15_110;
    sens.J(4,12) = ROcp15_411;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp15_85;
    sens.J(5,7) = ROcp15_56;
    sens.J(5,8) = ROcp15_27;
    sens.J(5,9) = ROcp15_88;
    sens.J(5,10) = ROcp15_59;
    sens.J(5,11) = ROcp15_210;
    sens.J(5,12) = ROcp15_511;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp15_95;
    sens.J(6,7) = ROcp15_66;
    sens.J(6,8) = ROcp15_37;
    sens.J(6,9) = ROcp15_98;
    sens.J(6,10) = ROcp15_69;
    sens.J(6,11) = ROcp15_310;
    sens.J(6,12) = ROcp15_611;
    sens.A(1) = ACcp15_147;
    sens.A(2) = ACcp15_247;
    sens.A(3) = ACcp15_347;
    sens.OMP(1) = OPcp15_112;
    sens.OMP(2) = OPcp15_212;
    sens.OMP(3) = OPcp15_312;
 
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

% = = Block_1_0_0_17_0_2 = = 
 
% Sensor Kinematics 


    ROcp16_17 = ROcp16_16*C7-S5*S7;
    ROcp16_27 = ROcp16_26*C7-ROcp16_85*S7;
    ROcp16_37 = ROcp16_36*C7-ROcp16_95*S7;
    ROcp16_77 = ROcp16_16*S7+S5*C7;
    ROcp16_87 = ROcp16_26*S7+ROcp16_85*C7;
    ROcp16_97 = ROcp16_36*S7+ROcp16_95*C7;
    ROcp16_48 = ROcp16_46*C8+ROcp16_77*S8;
    ROcp16_58 = ROcp16_56*C8+ROcp16_87*S8;
    ROcp16_68 = ROcp16_66*C8+ROcp16_97*S8;
    ROcp16_78 = -(ROcp16_46*S8-ROcp16_77*C8);
    ROcp16_88 = -(ROcp16_56*S8-ROcp16_87*C8);
    ROcp16_98 = -(ROcp16_66*S8-ROcp16_97*C8);
    ROcp16_19 = ROcp16_17*C9+ROcp16_48*S9;
    ROcp16_29 = ROcp16_27*C9+ROcp16_58*S9;
    ROcp16_39 = ROcp16_37*C9+ROcp16_68*S9;
    ROcp16_49 = -(ROcp16_17*S9-ROcp16_48*C9);
    ROcp16_59 = -(ROcp16_27*S9-ROcp16_58*C9);
    ROcp16_69 = -(ROcp16_37*S9-ROcp16_68*C9);
    ROcp16_110 = ROcp16_19*C10-ROcp16_78*S10;
    ROcp16_210 = ROcp16_29*C10-ROcp16_88*S10;
    ROcp16_310 = ROcp16_39*C10-ROcp16_98*S10;
    ROcp16_710 = ROcp16_19*S10+ROcp16_78*C10;
    ROcp16_810 = ROcp16_29*S10+ROcp16_88*C10;
    ROcp16_910 = ROcp16_39*S10+ROcp16_98*C10;
    ROcp16_411 = ROcp16_49*C11+ROcp16_710*S11;
    ROcp16_511 = ROcp16_59*C11+ROcp16_810*S11;
    ROcp16_611 = ROcp16_69*C11+ROcp16_910*S11;
    ROcp16_711 = -(ROcp16_49*S11-ROcp16_710*C11);
    ROcp16_811 = -(ROcp16_59*S11-ROcp16_810*C11);
    ROcp16_911 = -(ROcp16_69*S11-ROcp16_910*C11);
    ROcp16_112 = ROcp16_110*C12-ROcp16_711*S12;
    ROcp16_212 = ROcp16_210*C12-ROcp16_811*S12;
    ROcp16_312 = ROcp16_310*C12-ROcp16_911*S12;
    ROcp16_712 = ROcp16_110*S12+ROcp16_711*C12;
    ROcp16_812 = ROcp16_210*S12+ROcp16_811*C12;
    ROcp16_912 = ROcp16_310*S12+ROcp16_911*C12;
    ROcp16_113 = ROcp16_112*C13-ROcp16_712*S13;
    ROcp16_213 = ROcp16_212*C13-ROcp16_812*S13;
    ROcp16_313 = ROcp16_312*C13-ROcp16_912*S13;
    ROcp16_713 = ROcp16_112*S13+ROcp16_712*C13;
    ROcp16_813 = ROcp16_212*S13+ROcp16_812*C13;
    ROcp16_913 = ROcp16_312*S13+ROcp16_912*C13;
    RLcp16_17 = ROcp16_46*s.dpt(2,1);
    RLcp16_27 = ROcp16_56*s.dpt(2,1);
    RLcp16_37 = ROcp16_66*s.dpt(2,1);
    OMcp16_17 = OMcp16_16+ROcp16_46*qd(7);
    OMcp16_27 = OMcp16_26+ROcp16_56*qd(7);
    OMcp16_37 = OMcp16_36+ROcp16_66*qd(7);
    ORcp16_17 = OMcp16_26*RLcp16_37-OMcp16_36*RLcp16_27;
    ORcp16_27 = -(OMcp16_16*RLcp16_37-OMcp16_36*RLcp16_17);
    ORcp16_37 = OMcp16_16*RLcp16_27-OMcp16_26*RLcp16_17;
    OPcp16_17 = OPcp16_16+ROcp16_46*qdd(7)+qd(7)*(OMcp16_26*ROcp16_66-OMcp16_36*ROcp16_56);
    OPcp16_27 = OPcp16_26+ROcp16_56*qdd(7)-qd(7)*(OMcp16_16*ROcp16_66-OMcp16_36*ROcp16_46);
    OPcp16_37 = OPcp16_36+ROcp16_66*qdd(7)+qd(7)*(OMcp16_16*ROcp16_56-OMcp16_26*ROcp16_46);
    RLcp16_18 = ROcp16_46*s.dpt(2,6);
    RLcp16_28 = ROcp16_56*s.dpt(2,6);
    RLcp16_38 = ROcp16_66*s.dpt(2,6);
    OMcp16_18 = OMcp16_17+ROcp16_17*qd(8);
    OMcp16_28 = OMcp16_27+ROcp16_27*qd(8);
    OMcp16_38 = OMcp16_37+ROcp16_37*qd(8);
    ORcp16_18 = OMcp16_27*RLcp16_38-OMcp16_37*RLcp16_28;
    ORcp16_28 = -(OMcp16_17*RLcp16_38-OMcp16_37*RLcp16_18);
    ORcp16_38 = OMcp16_17*RLcp16_28-OMcp16_27*RLcp16_18;
    OPcp16_18 = OPcp16_17+ROcp16_17*qdd(8)+qd(8)*(OMcp16_27*ROcp16_37-OMcp16_37*ROcp16_27);
    OPcp16_28 = OPcp16_27+ROcp16_27*qdd(8)-qd(8)*(OMcp16_17*ROcp16_37-OMcp16_37*ROcp16_17);
    OPcp16_38 = OPcp16_37+ROcp16_37*qdd(8)+qd(8)*(OMcp16_17*ROcp16_27-OMcp16_27*ROcp16_17);
    RLcp16_19 = ROcp16_78*s.dpt(3,8);
    RLcp16_29 = ROcp16_88*s.dpt(3,8);
    RLcp16_39 = ROcp16_98*s.dpt(3,8);
    OMcp16_19 = OMcp16_18+ROcp16_78*qd(9);
    OMcp16_29 = OMcp16_28+ROcp16_88*qd(9);
    OMcp16_39 = OMcp16_38+ROcp16_98*qd(9);
    ORcp16_19 = OMcp16_28*RLcp16_39-OMcp16_38*RLcp16_29;
    ORcp16_29 = -(OMcp16_18*RLcp16_39-OMcp16_38*RLcp16_19);
    ORcp16_39 = OMcp16_18*RLcp16_29-OMcp16_28*RLcp16_19;
    OPcp16_19 = OPcp16_18+ROcp16_78*qdd(9)+qd(9)*(OMcp16_28*ROcp16_98-OMcp16_38*ROcp16_88);
    OPcp16_29 = OPcp16_28+ROcp16_88*qdd(9)-qd(9)*(OMcp16_18*ROcp16_98-OMcp16_38*ROcp16_78);
    OPcp16_39 = OPcp16_38+ROcp16_98*qdd(9)+qd(9)*(OMcp16_18*ROcp16_88-OMcp16_28*ROcp16_78);
    RLcp16_110 = ROcp16_78*s.dpt(3,10);
    RLcp16_210 = ROcp16_88*s.dpt(3,10);
    RLcp16_310 = ROcp16_98*s.dpt(3,10);
    OMcp16_110 = OMcp16_19+ROcp16_49*qd(10);
    OMcp16_210 = OMcp16_29+ROcp16_59*qd(10);
    OMcp16_310 = OMcp16_39+ROcp16_69*qd(10);
    ORcp16_110 = OMcp16_29*RLcp16_310-OMcp16_39*RLcp16_210;
    ORcp16_210 = -(OMcp16_19*RLcp16_310-OMcp16_39*RLcp16_110);
    ORcp16_310 = OMcp16_19*RLcp16_210-OMcp16_29*RLcp16_110;
    OPcp16_110 = OPcp16_19+ROcp16_49*qdd(10)+qd(10)*(OMcp16_29*ROcp16_69-OMcp16_39*ROcp16_59);
    OPcp16_210 = OPcp16_29+ROcp16_59*qdd(10)-qd(10)*(OMcp16_19*ROcp16_69-OMcp16_39*ROcp16_49);
    OPcp16_310 = OPcp16_39+ROcp16_69*qdd(10)+qd(10)*(OMcp16_19*ROcp16_59-OMcp16_29*ROcp16_49);
    RLcp16_111 = ROcp16_710*s.dpt(3,12);
    RLcp16_211 = ROcp16_810*s.dpt(3,12);
    RLcp16_311 = ROcp16_910*s.dpt(3,12);
    OMcp16_111 = OMcp16_110+ROcp16_110*qd(11);
    OMcp16_211 = OMcp16_210+ROcp16_210*qd(11);
    OMcp16_311 = OMcp16_310+ROcp16_310*qd(11);
    ORcp16_111 = OMcp16_210*RLcp16_311-OMcp16_310*RLcp16_211;
    ORcp16_211 = -(OMcp16_110*RLcp16_311-OMcp16_310*RLcp16_111);
    ORcp16_311 = OMcp16_110*RLcp16_211-OMcp16_210*RLcp16_111;
    OMcp16_112 = OMcp16_111+ROcp16_411*qd(12);
    OMcp16_212 = OMcp16_211+ROcp16_511*qd(12);
    OMcp16_312 = OMcp16_311+ROcp16_611*qd(12);
    OPcp16_112 = OPcp16_110+ROcp16_110*qdd(11)+ROcp16_411*qdd(12)+qd(11)*(OMcp16_210*ROcp16_310-OMcp16_310*ROcp16_210)+qd(12)*(OMcp16_211*...
 ROcp16_611-OMcp16_311*ROcp16_511);
    OPcp16_212 = OPcp16_210+ROcp16_210*qdd(11)+ROcp16_511*qdd(12)-qd(11)*(OMcp16_110*ROcp16_310-OMcp16_310*ROcp16_110)-qd(12)*(OMcp16_111*...
 ROcp16_611-OMcp16_311*ROcp16_411);
    OPcp16_312 = OPcp16_310+ROcp16_310*qdd(11)+ROcp16_611*qdd(12)+qd(11)*(OMcp16_110*ROcp16_210-OMcp16_210*ROcp16_110)+qd(12)*(OMcp16_111*...
 ROcp16_511-OMcp16_211*ROcp16_411);
    RLcp16_113 = ROcp16_112*s.dpt(1,18)+ROcp16_712*s.dpt(3,18);
    RLcp16_213 = ROcp16_212*s.dpt(1,18)+ROcp16_812*s.dpt(3,18);
    RLcp16_313 = ROcp16_312*s.dpt(1,18)+ROcp16_912*s.dpt(3,18);
    POcp16_113 = RLcp16_110+RLcp16_111+RLcp16_113+RLcp16_17+RLcp16_18+RLcp16_19+q(1);
    POcp16_213 = RLcp16_210+RLcp16_211+RLcp16_213+RLcp16_27+RLcp16_28+RLcp16_29+q(2);
    POcp16_313 = RLcp16_310+RLcp16_311+RLcp16_313+RLcp16_37+RLcp16_38+RLcp16_39+q(3);
    JTcp16_213_4 = -(RLcp16_310+RLcp16_311+RLcp16_313+RLcp16_37+RLcp16_38+RLcp16_39);
    JTcp16_313_4 = RLcp16_210+RLcp16_211+RLcp16_213+RLcp16_27+RLcp16_28+RLcp16_29;
    JTcp16_113_5 = C4*(RLcp16_310+RLcp16_311+RLcp16_313+RLcp16_37+RLcp16_38+RLcp16_39)-S4*(RLcp16_210+RLcp16_29)-S4*(RLcp16_211+RLcp16_213)-S4*(...
 RLcp16_27+RLcp16_28);
    JTcp16_213_5 = S4*(RLcp16_110+RLcp16_111+RLcp16_113+RLcp16_17+RLcp16_18+RLcp16_19);
    JTcp16_313_5 = -C4*(RLcp16_110+RLcp16_111+RLcp16_113+RLcp16_17+RLcp16_18+RLcp16_19);
    JTcp16_113_6 = ROcp16_85*(RLcp16_310+RLcp16_311+RLcp16_313+RLcp16_37+RLcp16_38+RLcp16_39)-ROcp16_95*(RLcp16_210+RLcp16_29)-ROcp16_95*(...
 RLcp16_211+RLcp16_213)-ROcp16_95*(RLcp16_27+RLcp16_28);
    JTcp16_213_6 = RLcp16_113*ROcp16_95-RLcp16_311*S5-RLcp16_313*S5+ROcp16_95*(RLcp16_110+RLcp16_111+RLcp16_17+RLcp16_18+RLcp16_19)-S5*(RLcp16_310+...
 RLcp16_39)-S5*(RLcp16_37+RLcp16_38);
    JTcp16_313_6 = RLcp16_211*S5-ROcp16_85*(RLcp16_110+RLcp16_111+RLcp16_17+RLcp16_18+RLcp16_19)+S5*(RLcp16_210+RLcp16_29)+S5*(RLcp16_27+RLcp16_28)...
 -RLcp16_113*ROcp16_85+RLcp16_213*S5;
    JTcp16_113_7 = ROcp16_56*(RLcp16_310+RLcp16_311+RLcp16_38+RLcp16_39)-ROcp16_66*(RLcp16_210+RLcp16_211)-ROcp16_66*(RLcp16_28+RLcp16_29)-...
 RLcp16_213*ROcp16_66+RLcp16_313*ROcp16_56;
    JTcp16_213_7 = RLcp16_113*ROcp16_66-RLcp16_313*ROcp16_46-ROcp16_46*(RLcp16_310+RLcp16_311+RLcp16_38+RLcp16_39)+ROcp16_66*(RLcp16_110+RLcp16_111...
 )+ROcp16_66*(RLcp16_18+RLcp16_19);
    JTcp16_313_7 = ROcp16_46*(RLcp16_210+RLcp16_211+RLcp16_28+RLcp16_29)-ROcp16_56*(RLcp16_110+RLcp16_111)-ROcp16_56*(RLcp16_18+RLcp16_19)-...
 RLcp16_113*ROcp16_56+RLcp16_213*ROcp16_46;
    JTcp16_113_8 = ROcp16_27*(RLcp16_310+RLcp16_311+RLcp16_313+RLcp16_39)-ROcp16_37*(RLcp16_210+RLcp16_29)-ROcp16_37*(RLcp16_211+RLcp16_213);
    JTcp16_213_8 = -(ROcp16_17*(RLcp16_310+RLcp16_311+RLcp16_313+RLcp16_39)-ROcp16_37*(RLcp16_110+RLcp16_19)-ROcp16_37*(RLcp16_111+RLcp16_113));
    JTcp16_313_8 = ROcp16_17*(RLcp16_210+RLcp16_211+RLcp16_213+RLcp16_29)-ROcp16_27*(RLcp16_110+RLcp16_19)-ROcp16_27*(RLcp16_111+RLcp16_113);
    JTcp16_113_9 = ROcp16_88*(RLcp16_310+RLcp16_311)-ROcp16_98*(RLcp16_210+RLcp16_211)-RLcp16_213*ROcp16_98+RLcp16_313*ROcp16_88;
    JTcp16_213_9 = RLcp16_113*ROcp16_98-RLcp16_313*ROcp16_78-ROcp16_78*(RLcp16_310+RLcp16_311)+ROcp16_98*(RLcp16_110+RLcp16_111);
    JTcp16_313_9 = ROcp16_78*(RLcp16_210+RLcp16_211)-ROcp16_88*(RLcp16_110+RLcp16_111)-RLcp16_113*ROcp16_88+RLcp16_213*ROcp16_78;
    JTcp16_113_10 = ROcp16_59*(RLcp16_311+RLcp16_313)-ROcp16_69*(RLcp16_211+RLcp16_213);
    JTcp16_213_10 = -(ROcp16_49*(RLcp16_311+RLcp16_313)-ROcp16_69*(RLcp16_111+RLcp16_113));
    JTcp16_313_10 = ROcp16_49*(RLcp16_211+RLcp16_213)-ROcp16_59*(RLcp16_111+RLcp16_113);
    JTcp16_113_11 = -(RLcp16_213*ROcp16_310-RLcp16_313*ROcp16_210);
    JTcp16_213_11 = RLcp16_113*ROcp16_310-RLcp16_313*ROcp16_110;
    JTcp16_313_11 = -(RLcp16_113*ROcp16_210-RLcp16_213*ROcp16_110);
    JTcp16_113_12 = -(RLcp16_213*ROcp16_611-RLcp16_313*ROcp16_511);
    JTcp16_213_12 = RLcp16_113*ROcp16_611-RLcp16_313*ROcp16_411;
    JTcp16_313_12 = -(RLcp16_113*ROcp16_511-RLcp16_213*ROcp16_411);
    OMcp16_113 = OMcp16_112+ROcp16_411*qd(13);
    OMcp16_213 = OMcp16_212+ROcp16_511*qd(13);
    OMcp16_313 = OMcp16_312+ROcp16_611*qd(13);
    ORcp16_113 = OMcp16_212*RLcp16_313-OMcp16_312*RLcp16_213;
    ORcp16_213 = -(OMcp16_112*RLcp16_313-OMcp16_312*RLcp16_113);
    ORcp16_313 = OMcp16_112*RLcp16_213-OMcp16_212*RLcp16_113;
    VIcp16_113 = ORcp16_110+ORcp16_111+ORcp16_113+ORcp16_17+ORcp16_18+ORcp16_19+qd(1);
    VIcp16_213 = ORcp16_210+ORcp16_211+ORcp16_213+ORcp16_27+ORcp16_28+ORcp16_29+qd(2);
    VIcp16_313 = ORcp16_310+ORcp16_311+ORcp16_313+ORcp16_37+ORcp16_38+ORcp16_39+qd(3);
    OPcp16_113 = OPcp16_112+ROcp16_411*qdd(13)+qd(13)*(OMcp16_212*ROcp16_611-OMcp16_312*ROcp16_511);
    OPcp16_213 = OPcp16_212+ROcp16_511*qdd(13)-qd(13)*(OMcp16_112*ROcp16_611-OMcp16_312*ROcp16_411);
    OPcp16_313 = OPcp16_312+ROcp16_611*qdd(13)+qd(13)*(OMcp16_112*ROcp16_511-OMcp16_212*ROcp16_411);
    ACcp16_113 = qdd(1)+OMcp16_210*ORcp16_311+OMcp16_212*ORcp16_313+OMcp16_26*ORcp16_37+OMcp16_27*ORcp16_38+OMcp16_28*ORcp16_39+OMcp16_29*...
 ORcp16_310-OMcp16_310*ORcp16_211-OMcp16_312*ORcp16_213-OMcp16_36*ORcp16_27-OMcp16_37*ORcp16_28-OMcp16_38*ORcp16_29-OMcp16_39*ORcp16_210+OPcp16_210*...
 RLcp16_311+OPcp16_212*RLcp16_313+OPcp16_26*RLcp16_37+OPcp16_27*RLcp16_38+OPcp16_28*RLcp16_39+OPcp16_29*RLcp16_310-OPcp16_310*RLcp16_211-OPcp16_312*...
 RLcp16_213-OPcp16_36*RLcp16_27-OPcp16_37*RLcp16_28-OPcp16_38*RLcp16_29-OPcp16_39*RLcp16_210;
    ACcp16_213 = qdd(2)-OMcp16_110*ORcp16_311-OMcp16_112*ORcp16_313-OMcp16_16*ORcp16_37-OMcp16_17*ORcp16_38-OMcp16_18*ORcp16_39-OMcp16_19*...
 ORcp16_310+OMcp16_310*ORcp16_111+OMcp16_312*ORcp16_113+OMcp16_36*ORcp16_17+OMcp16_37*ORcp16_18+OMcp16_38*ORcp16_19+OMcp16_39*ORcp16_110-OPcp16_110*...
 RLcp16_311-OPcp16_112*RLcp16_313-OPcp16_16*RLcp16_37-OPcp16_17*RLcp16_38-OPcp16_18*RLcp16_39-OPcp16_19*RLcp16_310+OPcp16_310*RLcp16_111+OPcp16_312*...
 RLcp16_113+OPcp16_36*RLcp16_17+OPcp16_37*RLcp16_18+OPcp16_38*RLcp16_19+OPcp16_39*RLcp16_110;
    ACcp16_313 = qdd(3)+OMcp16_110*ORcp16_211+OMcp16_112*ORcp16_213+OMcp16_16*ORcp16_27+OMcp16_17*ORcp16_28+OMcp16_18*ORcp16_29+OMcp16_19*...
 ORcp16_210-OMcp16_210*ORcp16_111-OMcp16_212*ORcp16_113-OMcp16_26*ORcp16_17-OMcp16_27*ORcp16_18-OMcp16_28*ORcp16_19-OMcp16_29*ORcp16_110+OPcp16_110*...
 RLcp16_211+OPcp16_112*RLcp16_213+OPcp16_16*RLcp16_27+OPcp16_17*RLcp16_28+OPcp16_18*RLcp16_29+OPcp16_19*RLcp16_210-OPcp16_210*RLcp16_111-OPcp16_212*...
 RLcp16_113-OPcp16_26*RLcp16_17-OPcp16_27*RLcp16_18-OPcp16_28*RLcp16_19-OPcp16_29*RLcp16_110;

% = = Block_1_0_0_17_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp16_113;
    sens.P(2) = POcp16_213;
    sens.P(3) = POcp16_313;
    sens.R(1,1) = ROcp16_113;
    sens.R(1,2) = ROcp16_213;
    sens.R(1,3) = ROcp16_313;
    sens.R(2,1) = ROcp16_411;
    sens.R(2,2) = ROcp16_511;
    sens.R(2,3) = ROcp16_611;
    sens.R(3,1) = ROcp16_713;
    sens.R(3,2) = ROcp16_813;
    sens.R(3,3) = ROcp16_913;
    sens.V(1) = VIcp16_113;
    sens.V(2) = VIcp16_213;
    sens.V(3) = VIcp16_313;
    sens.OM(1) = OMcp16_113;
    sens.OM(2) = OMcp16_213;
    sens.OM(3) = OMcp16_313;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp16_113_5;
    sens.J(1,6) = JTcp16_113_6;
    sens.J(1,7) = JTcp16_113_7;
    sens.J(1,8) = JTcp16_113_8;
    sens.J(1,9) = JTcp16_113_9;
    sens.J(1,10) = JTcp16_113_10;
    sens.J(1,11) = JTcp16_113_11;
    sens.J(1,12) = JTcp16_113_12;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp16_213_4;
    sens.J(2,5) = JTcp16_213_5;
    sens.J(2,6) = JTcp16_213_6;
    sens.J(2,7) = JTcp16_213_7;
    sens.J(2,8) = JTcp16_213_8;
    sens.J(2,9) = JTcp16_213_9;
    sens.J(2,10) = JTcp16_213_10;
    sens.J(2,11) = JTcp16_213_11;
    sens.J(2,12) = JTcp16_213_12;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp16_313_4;
    sens.J(3,5) = JTcp16_313_5;
    sens.J(3,6) = JTcp16_313_6;
    sens.J(3,7) = JTcp16_313_7;
    sens.J(3,8) = JTcp16_313_8;
    sens.J(3,9) = JTcp16_313_9;
    sens.J(3,10) = JTcp16_313_10;
    sens.J(3,11) = JTcp16_313_11;
    sens.J(3,12) = JTcp16_313_12;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,7) = ROcp16_46;
    sens.J(4,8) = ROcp16_17;
    sens.J(4,9) = ROcp16_78;
    sens.J(4,10) = ROcp16_49;
    sens.J(4,11) = ROcp16_110;
    sens.J(4,12) = ROcp16_411;
    sens.J(4,13) = ROcp16_411;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp16_85;
    sens.J(5,7) = ROcp16_56;
    sens.J(5,8) = ROcp16_27;
    sens.J(5,9) = ROcp16_88;
    sens.J(5,10) = ROcp16_59;
    sens.J(5,11) = ROcp16_210;
    sens.J(5,12) = ROcp16_511;
    sens.J(5,13) = ROcp16_511;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp16_95;
    sens.J(6,7) = ROcp16_66;
    sens.J(6,8) = ROcp16_37;
    sens.J(6,9) = ROcp16_98;
    sens.J(6,10) = ROcp16_69;
    sens.J(6,11) = ROcp16_310;
    sens.J(6,12) = ROcp16_611;
    sens.J(6,13) = ROcp16_611;
    sens.A(1) = ACcp16_113;
    sens.A(2) = ACcp16_213;
    sens.A(3) = ACcp16_313;
    sens.OMP(1) = OPcp16_113;
    sens.OMP(2) = OPcp16_213;
    sens.OMP(3) = OPcp16_313;
 
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


    ROcp17_114 = ROcp17_16*C14-S14*S5;
    ROcp17_214 = ROcp17_26*C14-ROcp17_85*S14;
    ROcp17_314 = ROcp17_36*C14-ROcp17_95*S14;
    ROcp17_714 = ROcp17_16*S14+C14*S5;
    ROcp17_814 = ROcp17_26*S14+ROcp17_85*C14;
    ROcp17_914 = ROcp17_36*S14+ROcp17_95*C14;
    RLcp17_114 = ROcp17_46*s.dpt(2,2);
    RLcp17_214 = ROcp17_56*s.dpt(2,2);
    RLcp17_314 = ROcp17_66*s.dpt(2,2);
    OMcp17_114 = OMcp17_16+ROcp17_46*qd(14);
    OMcp17_214 = OMcp17_26+ROcp17_56*qd(14);
    OMcp17_314 = OMcp17_36+ROcp17_66*qd(14);
    ORcp17_114 = OMcp17_26*RLcp17_314-OMcp17_36*RLcp17_214;
    ORcp17_214 = -(OMcp17_16*RLcp17_314-OMcp17_36*RLcp17_114);
    ORcp17_314 = OMcp17_16*RLcp17_214-OMcp17_26*RLcp17_114;
    OPcp17_114 = OPcp17_16+ROcp17_46*qdd(14)+qd(14)*(OMcp17_26*ROcp17_66-OMcp17_36*ROcp17_56);
    OPcp17_214 = OPcp17_26+ROcp17_56*qdd(14)-qd(14)*(OMcp17_16*ROcp17_66-OMcp17_36*ROcp17_46);
    OPcp17_314 = OPcp17_36+ROcp17_66*qdd(14)+qd(14)*(OMcp17_16*ROcp17_56-OMcp17_26*ROcp17_46);
    RLcp17_149 = ROcp17_46*s.dpt(2,20);
    RLcp17_249 = ROcp17_56*s.dpt(2,20);
    RLcp17_349 = ROcp17_66*s.dpt(2,20);
    POcp17_149 = RLcp17_114+RLcp17_149+q(1);
    POcp17_249 = RLcp17_214+RLcp17_249+q(2);
    POcp17_349 = RLcp17_314+RLcp17_349+q(3);
    JTcp17_249_4 = -(RLcp17_314+RLcp17_349);
    JTcp17_349_4 = RLcp17_214+RLcp17_249;
    JTcp17_149_5 = C4*(RLcp17_314+RLcp17_349)-S4*(RLcp17_214+RLcp17_249);
    JTcp17_249_5 = S4*(RLcp17_114+RLcp17_149);
    JTcp17_349_5 = -C4*(RLcp17_114+RLcp17_149);
    JTcp17_149_6 = ROcp17_85*(RLcp17_314+RLcp17_349)-ROcp17_95*(RLcp17_214+RLcp17_249);
    JTcp17_249_6 = ROcp17_95*(RLcp17_114+RLcp17_149)-S5*(RLcp17_314+RLcp17_349);
    JTcp17_349_6 = -(ROcp17_85*(RLcp17_114+RLcp17_149)-S5*(RLcp17_214+RLcp17_249));
    JTcp17_149_7 = -(RLcp17_249*ROcp17_66-RLcp17_349*ROcp17_56);
    JTcp17_249_7 = RLcp17_149*ROcp17_66-RLcp17_349*ROcp17_46;
    JTcp17_349_7 = -(RLcp17_149*ROcp17_56-RLcp17_249*ROcp17_46);
    ORcp17_149 = OMcp17_214*RLcp17_349-OMcp17_314*RLcp17_249;
    ORcp17_249 = -(OMcp17_114*RLcp17_349-OMcp17_314*RLcp17_149);
    ORcp17_349 = OMcp17_114*RLcp17_249-OMcp17_214*RLcp17_149;
    VIcp17_149 = ORcp17_114+ORcp17_149+qd(1);
    VIcp17_249 = ORcp17_214+ORcp17_249+qd(2);
    VIcp17_349 = ORcp17_314+ORcp17_349+qd(3);
    ACcp17_149 = qdd(1)+OMcp17_214*ORcp17_349+OMcp17_26*ORcp17_314-OMcp17_314*ORcp17_249-OMcp17_36*ORcp17_214+OPcp17_214*RLcp17_349+OPcp17_26*...
 RLcp17_314-OPcp17_314*RLcp17_249-OPcp17_36*RLcp17_214;
    ACcp17_249 = qdd(2)-OMcp17_114*ORcp17_349-OMcp17_16*ORcp17_314+OMcp17_314*ORcp17_149+OMcp17_36*ORcp17_114-OPcp17_114*RLcp17_349-OPcp17_16*...
 RLcp17_314+OPcp17_314*RLcp17_149+OPcp17_36*RLcp17_114;
    ACcp17_349 = qdd(3)+OMcp17_114*ORcp17_249+OMcp17_16*ORcp17_214-OMcp17_214*ORcp17_149-OMcp17_26*ORcp17_114+OPcp17_114*RLcp17_249+OPcp17_16*...
 RLcp17_214-OPcp17_214*RLcp17_149-OPcp17_26*RLcp17_114;

% = = Block_1_0_0_18_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp17_149;
    sens.P(2) = POcp17_249;
    sens.P(3) = POcp17_349;
    sens.R(1,1) = ROcp17_114;
    sens.R(1,2) = ROcp17_214;
    sens.R(1,3) = ROcp17_314;
    sens.R(2,1) = ROcp17_46;
    sens.R(2,2) = ROcp17_56;
    sens.R(2,3) = ROcp17_66;
    sens.R(3,1) = ROcp17_714;
    sens.R(3,2) = ROcp17_814;
    sens.R(3,3) = ROcp17_914;
    sens.V(1) = VIcp17_149;
    sens.V(2) = VIcp17_249;
    sens.V(3) = VIcp17_349;
    sens.OM(1) = OMcp17_114;
    sens.OM(2) = OMcp17_214;
    sens.OM(3) = OMcp17_314;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp17_149_5;
    sens.J(1,6) = JTcp17_149_6;
    sens.J(1,14) = JTcp17_149_7;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp17_249_4;
    sens.J(2,5) = JTcp17_249_5;
    sens.J(2,6) = JTcp17_249_6;
    sens.J(2,14) = JTcp17_249_7;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp17_349_4;
    sens.J(3,5) = JTcp17_349_5;
    sens.J(3,6) = JTcp17_349_6;
    sens.J(3,14) = JTcp17_349_7;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,14) = ROcp17_46;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp17_85;
    sens.J(5,14) = ROcp17_56;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp17_95;
    sens.J(6,14) = ROcp17_66;
    sens.A(1) = ACcp17_149;
    sens.A(2) = ACcp17_249;
    sens.A(3) = ACcp17_349;
    sens.OMP(1) = OPcp17_114;
    sens.OMP(2) = OPcp17_214;
    sens.OMP(3) = OPcp17_314;
 
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

% = = Block_1_0_0_19_0_3 = = 
 
% Sensor Kinematics 


    ROcp18_114 = ROcp18_16*C14-S14*S5;
    ROcp18_214 = ROcp18_26*C14-ROcp18_85*S14;
    ROcp18_314 = ROcp18_36*C14-ROcp18_95*S14;
    ROcp18_714 = ROcp18_16*S14+C14*S5;
    ROcp18_814 = ROcp18_26*S14+ROcp18_85*C14;
    ROcp18_914 = ROcp18_36*S14+ROcp18_95*C14;
    RLcp18_114 = ROcp18_46*s.dpt(2,2);
    RLcp18_214 = ROcp18_56*s.dpt(2,2);
    RLcp18_314 = ROcp18_66*s.dpt(2,2);
    OMcp18_114 = OMcp18_16+ROcp18_46*qd(14);
    OMcp18_214 = OMcp18_26+ROcp18_56*qd(14);
    OMcp18_314 = OMcp18_36+ROcp18_66*qd(14);
    ORcp18_114 = OMcp18_26*RLcp18_314-OMcp18_36*RLcp18_214;
    ORcp18_214 = -(OMcp18_16*RLcp18_314-OMcp18_36*RLcp18_114);
    ORcp18_314 = OMcp18_16*RLcp18_214-OMcp18_26*RLcp18_114;
    OPcp18_114 = OPcp18_16+ROcp18_46*qdd(14)+qd(14)*(OMcp18_26*ROcp18_66-OMcp18_36*ROcp18_56);
    OPcp18_214 = OPcp18_26+ROcp18_56*qdd(14)-qd(14)*(OMcp18_16*ROcp18_66-OMcp18_36*ROcp18_46);
    OPcp18_314 = OPcp18_36+ROcp18_66*qdd(14)+qd(14)*(OMcp18_16*ROcp18_56-OMcp18_26*ROcp18_46);
    RLcp18_150 = ROcp18_114*s.dpt(1,21)+ROcp18_46*s.dpt(2,21)+ROcp18_714*s.dpt(3,21);
    RLcp18_250 = ROcp18_214*s.dpt(1,21)+ROcp18_56*s.dpt(2,21)+ROcp18_814*s.dpt(3,21);
    RLcp18_350 = ROcp18_314*s.dpt(1,21)+ROcp18_66*s.dpt(2,21)+ROcp18_914*s.dpt(3,21);
    POcp18_150 = RLcp18_114+RLcp18_150+q(1);
    POcp18_250 = RLcp18_214+RLcp18_250+q(2);
    POcp18_350 = RLcp18_314+RLcp18_350+q(3);
    JTcp18_250_4 = -(RLcp18_314+RLcp18_350);
    JTcp18_350_4 = RLcp18_214+RLcp18_250;
    JTcp18_150_5 = C4*(RLcp18_314+RLcp18_350)-S4*(RLcp18_214+RLcp18_250);
    JTcp18_250_5 = S4*(RLcp18_114+RLcp18_150);
    JTcp18_350_5 = -C4*(RLcp18_114+RLcp18_150);
    JTcp18_150_6 = ROcp18_85*(RLcp18_314+RLcp18_350)-ROcp18_95*(RLcp18_214+RLcp18_250);
    JTcp18_250_6 = ROcp18_95*(RLcp18_114+RLcp18_150)-S5*(RLcp18_314+RLcp18_350);
    JTcp18_350_6 = -(ROcp18_85*(RLcp18_114+RLcp18_150)-S5*(RLcp18_214+RLcp18_250));
    JTcp18_150_7 = -(RLcp18_250*ROcp18_66-RLcp18_350*ROcp18_56);
    JTcp18_250_7 = RLcp18_150*ROcp18_66-RLcp18_350*ROcp18_46;
    JTcp18_350_7 = -(RLcp18_150*ROcp18_56-RLcp18_250*ROcp18_46);
    ORcp18_150 = OMcp18_214*RLcp18_350-OMcp18_314*RLcp18_250;
    ORcp18_250 = -(OMcp18_114*RLcp18_350-OMcp18_314*RLcp18_150);
    ORcp18_350 = OMcp18_114*RLcp18_250-OMcp18_214*RLcp18_150;
    VIcp18_150 = ORcp18_114+ORcp18_150+qd(1);
    VIcp18_250 = ORcp18_214+ORcp18_250+qd(2);
    VIcp18_350 = ORcp18_314+ORcp18_350+qd(3);
    ACcp18_150 = qdd(1)+OMcp18_214*ORcp18_350+OMcp18_26*ORcp18_314-OMcp18_314*ORcp18_250-OMcp18_36*ORcp18_214+OPcp18_214*RLcp18_350+OPcp18_26*...
 RLcp18_314-OPcp18_314*RLcp18_250-OPcp18_36*RLcp18_214;
    ACcp18_250 = qdd(2)-OMcp18_114*ORcp18_350-OMcp18_16*ORcp18_314+OMcp18_314*ORcp18_150+OMcp18_36*ORcp18_114-OPcp18_114*RLcp18_350-OPcp18_16*...
 RLcp18_314+OPcp18_314*RLcp18_150+OPcp18_36*RLcp18_114;
    ACcp18_350 = qdd(3)+OMcp18_114*ORcp18_250+OMcp18_16*ORcp18_214-OMcp18_214*ORcp18_150-OMcp18_26*ORcp18_114+OPcp18_114*RLcp18_250+OPcp18_16*...
 RLcp18_214-OPcp18_214*RLcp18_150-OPcp18_26*RLcp18_114;

% = = Block_1_0_0_19_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp18_150;
    sens.P(2) = POcp18_250;
    sens.P(3) = POcp18_350;
    sens.R(1,1) = ROcp18_114;
    sens.R(1,2) = ROcp18_214;
    sens.R(1,3) = ROcp18_314;
    sens.R(2,1) = ROcp18_46;
    sens.R(2,2) = ROcp18_56;
    sens.R(2,3) = ROcp18_66;
    sens.R(3,1) = ROcp18_714;
    sens.R(3,2) = ROcp18_814;
    sens.R(3,3) = ROcp18_914;
    sens.V(1) = VIcp18_150;
    sens.V(2) = VIcp18_250;
    sens.V(3) = VIcp18_350;
    sens.OM(1) = OMcp18_114;
    sens.OM(2) = OMcp18_214;
    sens.OM(3) = OMcp18_314;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp18_150_5;
    sens.J(1,6) = JTcp18_150_6;
    sens.J(1,14) = JTcp18_150_7;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp18_250_4;
    sens.J(2,5) = JTcp18_250_5;
    sens.J(2,6) = JTcp18_250_6;
    sens.J(2,14) = JTcp18_250_7;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp18_350_4;
    sens.J(3,5) = JTcp18_350_5;
    sens.J(3,6) = JTcp18_350_6;
    sens.J(3,14) = JTcp18_350_7;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,14) = ROcp18_46;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp18_85;
    sens.J(5,14) = ROcp18_56;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp18_95;
    sens.J(6,14) = ROcp18_66;
    sens.A(1) = ACcp18_150;
    sens.A(2) = ACcp18_250;
    sens.A(3) = ACcp18_350;
    sens.OMP(1) = OPcp18_114;
    sens.OMP(2) = OPcp18_214;
    sens.OMP(3) = OPcp18_314;
 
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

% = = Block_1_0_0_20_0_3 = = 
 
% Sensor Kinematics 


    ROcp19_114 = ROcp19_16*C14-S14*S5;
    ROcp19_214 = ROcp19_26*C14-ROcp19_85*S14;
    ROcp19_314 = ROcp19_36*C14-ROcp19_95*S14;
    ROcp19_714 = ROcp19_16*S14+C14*S5;
    ROcp19_814 = ROcp19_26*S14+ROcp19_85*C14;
    ROcp19_914 = ROcp19_36*S14+ROcp19_95*C14;
    ROcp19_415 = ROcp19_46*C15+ROcp19_714*S15;
    ROcp19_515 = ROcp19_56*C15+ROcp19_814*S15;
    ROcp19_615 = ROcp19_66*C15+ROcp19_914*S15;
    ROcp19_715 = -(ROcp19_46*S15-ROcp19_714*C15);
    ROcp19_815 = -(ROcp19_56*S15-ROcp19_814*C15);
    ROcp19_915 = -(ROcp19_66*S15-ROcp19_914*C15);
    RLcp19_114 = ROcp19_46*s.dpt(2,2);
    RLcp19_214 = ROcp19_56*s.dpt(2,2);
    RLcp19_314 = ROcp19_66*s.dpt(2,2);
    OMcp19_114 = OMcp19_16+ROcp19_46*qd(14);
    OMcp19_214 = OMcp19_26+ROcp19_56*qd(14);
    OMcp19_314 = OMcp19_36+ROcp19_66*qd(14);
    ORcp19_114 = OMcp19_26*RLcp19_314-OMcp19_36*RLcp19_214;
    ORcp19_214 = -(OMcp19_16*RLcp19_314-OMcp19_36*RLcp19_114);
    ORcp19_314 = OMcp19_16*RLcp19_214-OMcp19_26*RLcp19_114;
    OPcp19_114 = OPcp19_16+ROcp19_46*qdd(14)+qd(14)*(OMcp19_26*ROcp19_66-OMcp19_36*ROcp19_56);
    OPcp19_214 = OPcp19_26+ROcp19_56*qdd(14)-qd(14)*(OMcp19_16*ROcp19_66-OMcp19_36*ROcp19_46);
    OPcp19_314 = OPcp19_36+ROcp19_66*qdd(14)+qd(14)*(OMcp19_16*ROcp19_56-OMcp19_26*ROcp19_46);
    RLcp19_115 = ROcp19_46*s.dpt(2,20);
    RLcp19_215 = ROcp19_56*s.dpt(2,20);
    RLcp19_315 = ROcp19_66*s.dpt(2,20);
    OMcp19_115 = OMcp19_114+ROcp19_114*qd(15);
    OMcp19_215 = OMcp19_214+ROcp19_214*qd(15);
    OMcp19_315 = OMcp19_314+ROcp19_314*qd(15);
    ORcp19_115 = OMcp19_214*RLcp19_315-OMcp19_314*RLcp19_215;
    ORcp19_215 = -(OMcp19_114*RLcp19_315-OMcp19_314*RLcp19_115);
    ORcp19_315 = OMcp19_114*RLcp19_215-OMcp19_214*RLcp19_115;
    OPcp19_115 = OPcp19_114+ROcp19_114*qdd(15)+qd(15)*(OMcp19_214*ROcp19_314-OMcp19_314*ROcp19_214);
    OPcp19_215 = OPcp19_214+ROcp19_214*qdd(15)-qd(15)*(OMcp19_114*ROcp19_314-OMcp19_314*ROcp19_114);
    OPcp19_315 = OPcp19_314+ROcp19_314*qdd(15)+qd(15)*(OMcp19_114*ROcp19_214-OMcp19_214*ROcp19_114);
    RLcp19_151 = ROcp19_715*s.dpt(3,22);
    RLcp19_251 = ROcp19_815*s.dpt(3,22);
    RLcp19_351 = ROcp19_915*s.dpt(3,22);
    POcp19_151 = RLcp19_114+RLcp19_115+RLcp19_151+q(1);
    POcp19_251 = RLcp19_214+RLcp19_215+RLcp19_251+q(2);
    POcp19_351 = RLcp19_314+RLcp19_315+RLcp19_351+q(3);
    JTcp19_251_4 = -(RLcp19_314+RLcp19_315+RLcp19_351);
    JTcp19_351_4 = RLcp19_214+RLcp19_215+RLcp19_251;
    JTcp19_151_5 = C4*(RLcp19_314+RLcp19_315)-S4*(RLcp19_214+RLcp19_215)-RLcp19_251*S4+RLcp19_351*C4;
    JTcp19_251_5 = S4*(RLcp19_114+RLcp19_115+RLcp19_151);
    JTcp19_351_5 = -C4*(RLcp19_114+RLcp19_115+RLcp19_151);
    JTcp19_151_6 = ROcp19_85*(RLcp19_314+RLcp19_315)-ROcp19_95*(RLcp19_214+RLcp19_215)-RLcp19_251*ROcp19_95+RLcp19_351*ROcp19_85;
    JTcp19_251_6 = -(RLcp19_351*S5-ROcp19_95*(RLcp19_114+RLcp19_115+RLcp19_151)+S5*(RLcp19_314+RLcp19_315));
    JTcp19_351_6 = RLcp19_251*S5-ROcp19_85*(RLcp19_114+RLcp19_115+RLcp19_151)+S5*(RLcp19_214+RLcp19_215);
    JTcp19_151_7 = ROcp19_56*(RLcp19_315+RLcp19_351)-ROcp19_66*(RLcp19_215+RLcp19_251);
    JTcp19_251_7 = -(ROcp19_46*(RLcp19_315+RLcp19_351)-ROcp19_66*(RLcp19_115+RLcp19_151));
    JTcp19_351_7 = ROcp19_46*(RLcp19_215+RLcp19_251)-ROcp19_56*(RLcp19_115+RLcp19_151);
    JTcp19_151_8 = -(RLcp19_251*ROcp19_314-RLcp19_351*ROcp19_214);
    JTcp19_251_8 = RLcp19_151*ROcp19_314-RLcp19_351*ROcp19_114;
    JTcp19_351_8 = -(RLcp19_151*ROcp19_214-RLcp19_251*ROcp19_114);
    ORcp19_151 = OMcp19_215*RLcp19_351-OMcp19_315*RLcp19_251;
    ORcp19_251 = -(OMcp19_115*RLcp19_351-OMcp19_315*RLcp19_151);
    ORcp19_351 = OMcp19_115*RLcp19_251-OMcp19_215*RLcp19_151;
    VIcp19_151 = ORcp19_114+ORcp19_115+ORcp19_151+qd(1);
    VIcp19_251 = ORcp19_214+ORcp19_215+ORcp19_251+qd(2);
    VIcp19_351 = ORcp19_314+ORcp19_315+ORcp19_351+qd(3);
    ACcp19_151 = qdd(1)+OMcp19_214*ORcp19_315+OMcp19_215*ORcp19_351+OMcp19_26*ORcp19_314-OMcp19_314*ORcp19_215-OMcp19_315*ORcp19_251-OMcp19_36*...
 ORcp19_214+OPcp19_214*RLcp19_315+OPcp19_215*RLcp19_351+OPcp19_26*RLcp19_314-OPcp19_314*RLcp19_215-OPcp19_315*RLcp19_251-OPcp19_36*RLcp19_214;
    ACcp19_251 = qdd(2)-OMcp19_114*ORcp19_315-OMcp19_115*ORcp19_351-OMcp19_16*ORcp19_314+OMcp19_314*ORcp19_115+OMcp19_315*ORcp19_151+OMcp19_36*...
 ORcp19_114-OPcp19_114*RLcp19_315-OPcp19_115*RLcp19_351-OPcp19_16*RLcp19_314+OPcp19_314*RLcp19_115+OPcp19_315*RLcp19_151+OPcp19_36*RLcp19_114;
    ACcp19_351 = qdd(3)+OMcp19_114*ORcp19_215+OMcp19_115*ORcp19_251+OMcp19_16*ORcp19_214-OMcp19_214*ORcp19_115-OMcp19_215*ORcp19_151-OMcp19_26*...
 ORcp19_114+OPcp19_114*RLcp19_215+OPcp19_115*RLcp19_251+OPcp19_16*RLcp19_214-OPcp19_214*RLcp19_115-OPcp19_215*RLcp19_151-OPcp19_26*RLcp19_114;

% = = Block_1_0_0_20_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp19_151;
    sens.P(2) = POcp19_251;
    sens.P(3) = POcp19_351;
    sens.R(1,1) = ROcp19_114;
    sens.R(1,2) = ROcp19_214;
    sens.R(1,3) = ROcp19_314;
    sens.R(2,1) = ROcp19_415;
    sens.R(2,2) = ROcp19_515;
    sens.R(2,3) = ROcp19_615;
    sens.R(3,1) = ROcp19_715;
    sens.R(3,2) = ROcp19_815;
    sens.R(3,3) = ROcp19_915;
    sens.V(1) = VIcp19_151;
    sens.V(2) = VIcp19_251;
    sens.V(3) = VIcp19_351;
    sens.OM(1) = OMcp19_115;
    sens.OM(2) = OMcp19_215;
    sens.OM(3) = OMcp19_315;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp19_151_5;
    sens.J(1,6) = JTcp19_151_6;
    sens.J(1,14) = JTcp19_151_7;
    sens.J(1,15) = JTcp19_151_8;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp19_251_4;
    sens.J(2,5) = JTcp19_251_5;
    sens.J(2,6) = JTcp19_251_6;
    sens.J(2,14) = JTcp19_251_7;
    sens.J(2,15) = JTcp19_251_8;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp19_351_4;
    sens.J(3,5) = JTcp19_351_5;
    sens.J(3,6) = JTcp19_351_6;
    sens.J(3,14) = JTcp19_351_7;
    sens.J(3,15) = JTcp19_351_8;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,14) = ROcp19_46;
    sens.J(4,15) = ROcp19_114;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp19_85;
    sens.J(5,14) = ROcp19_56;
    sens.J(5,15) = ROcp19_214;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp19_95;
    sens.J(6,14) = ROcp19_66;
    sens.J(6,15) = ROcp19_314;
    sens.A(1) = ACcp19_151;
    sens.A(2) = ACcp19_251;
    sens.A(3) = ACcp19_351;
    sens.OMP(1) = OPcp19_115;
    sens.OMP(2) = OPcp19_215;
    sens.OMP(3) = OPcp19_315;
 
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

% = = Block_1_0_0_21_0_3 = = 
 
% Sensor Kinematics 


    ROcp20_114 = ROcp20_16*C14-S14*S5;
    ROcp20_214 = ROcp20_26*C14-ROcp20_85*S14;
    ROcp20_314 = ROcp20_36*C14-ROcp20_95*S14;
    ROcp20_714 = ROcp20_16*S14+C14*S5;
    ROcp20_814 = ROcp20_26*S14+ROcp20_85*C14;
    ROcp20_914 = ROcp20_36*S14+ROcp20_95*C14;
    ROcp20_415 = ROcp20_46*C15+ROcp20_714*S15;
    ROcp20_515 = ROcp20_56*C15+ROcp20_814*S15;
    ROcp20_615 = ROcp20_66*C15+ROcp20_914*S15;
    ROcp20_715 = -(ROcp20_46*S15-ROcp20_714*C15);
    ROcp20_815 = -(ROcp20_56*S15-ROcp20_814*C15);
    ROcp20_915 = -(ROcp20_66*S15-ROcp20_914*C15);
    RLcp20_114 = ROcp20_46*s.dpt(2,2);
    RLcp20_214 = ROcp20_56*s.dpt(2,2);
    RLcp20_314 = ROcp20_66*s.dpt(2,2);
    OMcp20_114 = OMcp20_16+ROcp20_46*qd(14);
    OMcp20_214 = OMcp20_26+ROcp20_56*qd(14);
    OMcp20_314 = OMcp20_36+ROcp20_66*qd(14);
    ORcp20_114 = OMcp20_26*RLcp20_314-OMcp20_36*RLcp20_214;
    ORcp20_214 = -(OMcp20_16*RLcp20_314-OMcp20_36*RLcp20_114);
    ORcp20_314 = OMcp20_16*RLcp20_214-OMcp20_26*RLcp20_114;
    OPcp20_114 = OPcp20_16+ROcp20_46*qdd(14)+qd(14)*(OMcp20_26*ROcp20_66-OMcp20_36*ROcp20_56);
    OPcp20_214 = OPcp20_26+ROcp20_56*qdd(14)-qd(14)*(OMcp20_16*ROcp20_66-OMcp20_36*ROcp20_46);
    OPcp20_314 = OPcp20_36+ROcp20_66*qdd(14)+qd(14)*(OMcp20_16*ROcp20_56-OMcp20_26*ROcp20_46);
    RLcp20_115 = ROcp20_46*s.dpt(2,20);
    RLcp20_215 = ROcp20_56*s.dpt(2,20);
    RLcp20_315 = ROcp20_66*s.dpt(2,20);
    OMcp20_115 = OMcp20_114+ROcp20_114*qd(15);
    OMcp20_215 = OMcp20_214+ROcp20_214*qd(15);
    OMcp20_315 = OMcp20_314+ROcp20_314*qd(15);
    ORcp20_115 = OMcp20_214*RLcp20_315-OMcp20_314*RLcp20_215;
    ORcp20_215 = -(OMcp20_114*RLcp20_315-OMcp20_314*RLcp20_115);
    ORcp20_315 = OMcp20_114*RLcp20_215-OMcp20_214*RLcp20_115;
    OPcp20_115 = OPcp20_114+ROcp20_114*qdd(15)+qd(15)*(OMcp20_214*ROcp20_314-OMcp20_314*ROcp20_214);
    OPcp20_215 = OPcp20_214+ROcp20_214*qdd(15)-qd(15)*(OMcp20_114*ROcp20_314-OMcp20_314*ROcp20_114);
    OPcp20_315 = OPcp20_314+ROcp20_314*qdd(15)+qd(15)*(OMcp20_114*ROcp20_214-OMcp20_214*ROcp20_114);
    RLcp20_152 = ROcp20_114*s.dpt(1,23)+ROcp20_415*s.dpt(2,23)+ROcp20_715*s.dpt(3,23);
    RLcp20_252 = ROcp20_214*s.dpt(1,23)+ROcp20_515*s.dpt(2,23)+ROcp20_815*s.dpt(3,23);
    RLcp20_352 = ROcp20_314*s.dpt(1,23)+ROcp20_615*s.dpt(2,23)+ROcp20_915*s.dpt(3,23);
    POcp20_152 = RLcp20_114+RLcp20_115+RLcp20_152+q(1);
    POcp20_252 = RLcp20_214+RLcp20_215+RLcp20_252+q(2);
    POcp20_352 = RLcp20_314+RLcp20_315+RLcp20_352+q(3);
    JTcp20_252_4 = -(RLcp20_314+RLcp20_315+RLcp20_352);
    JTcp20_352_4 = RLcp20_214+RLcp20_215+RLcp20_252;
    JTcp20_152_5 = C4*(RLcp20_314+RLcp20_315)-S4*(RLcp20_214+RLcp20_215)-RLcp20_252*S4+RLcp20_352*C4;
    JTcp20_252_5 = S4*(RLcp20_114+RLcp20_115+RLcp20_152);
    JTcp20_352_5 = -C4*(RLcp20_114+RLcp20_115+RLcp20_152);
    JTcp20_152_6 = ROcp20_85*(RLcp20_314+RLcp20_315)-ROcp20_95*(RLcp20_214+RLcp20_215)-RLcp20_252*ROcp20_95+RLcp20_352*ROcp20_85;
    JTcp20_252_6 = -(RLcp20_352*S5-ROcp20_95*(RLcp20_114+RLcp20_115+RLcp20_152)+S5*(RLcp20_314+RLcp20_315));
    JTcp20_352_6 = RLcp20_252*S5-ROcp20_85*(RLcp20_114+RLcp20_115+RLcp20_152)+S5*(RLcp20_214+RLcp20_215);
    JTcp20_152_7 = ROcp20_56*(RLcp20_315+RLcp20_352)-ROcp20_66*(RLcp20_215+RLcp20_252);
    JTcp20_252_7 = -(ROcp20_46*(RLcp20_315+RLcp20_352)-ROcp20_66*(RLcp20_115+RLcp20_152));
    JTcp20_352_7 = ROcp20_46*(RLcp20_215+RLcp20_252)-ROcp20_56*(RLcp20_115+RLcp20_152);
    JTcp20_152_8 = -(RLcp20_252*ROcp20_314-RLcp20_352*ROcp20_214);
    JTcp20_252_8 = RLcp20_152*ROcp20_314-RLcp20_352*ROcp20_114;
    JTcp20_352_8 = -(RLcp20_152*ROcp20_214-RLcp20_252*ROcp20_114);
    ORcp20_152 = OMcp20_215*RLcp20_352-OMcp20_315*RLcp20_252;
    ORcp20_252 = -(OMcp20_115*RLcp20_352-OMcp20_315*RLcp20_152);
    ORcp20_352 = OMcp20_115*RLcp20_252-OMcp20_215*RLcp20_152;
    VIcp20_152 = ORcp20_114+ORcp20_115+ORcp20_152+qd(1);
    VIcp20_252 = ORcp20_214+ORcp20_215+ORcp20_252+qd(2);
    VIcp20_352 = ORcp20_314+ORcp20_315+ORcp20_352+qd(3);
    ACcp20_152 = qdd(1)+OMcp20_214*ORcp20_315+OMcp20_215*ORcp20_352+OMcp20_26*ORcp20_314-OMcp20_314*ORcp20_215-OMcp20_315*ORcp20_252-OMcp20_36*...
 ORcp20_214+OPcp20_214*RLcp20_315+OPcp20_215*RLcp20_352+OPcp20_26*RLcp20_314-OPcp20_314*RLcp20_215-OPcp20_315*RLcp20_252-OPcp20_36*RLcp20_214;
    ACcp20_252 = qdd(2)-OMcp20_114*ORcp20_315-OMcp20_115*ORcp20_352-OMcp20_16*ORcp20_314+OMcp20_314*ORcp20_115+OMcp20_315*ORcp20_152+OMcp20_36*...
 ORcp20_114-OPcp20_114*RLcp20_315-OPcp20_115*RLcp20_352-OPcp20_16*RLcp20_314+OPcp20_314*RLcp20_115+OPcp20_315*RLcp20_152+OPcp20_36*RLcp20_114;
    ACcp20_352 = qdd(3)+OMcp20_114*ORcp20_215+OMcp20_115*ORcp20_252+OMcp20_16*ORcp20_214-OMcp20_214*ORcp20_115-OMcp20_215*ORcp20_152-OMcp20_26*...
 ORcp20_114+OPcp20_114*RLcp20_215+OPcp20_115*RLcp20_252+OPcp20_16*RLcp20_214-OPcp20_214*RLcp20_115-OPcp20_215*RLcp20_152-OPcp20_26*RLcp20_114;

% = = Block_1_0_0_21_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp20_152;
    sens.P(2) = POcp20_252;
    sens.P(3) = POcp20_352;
    sens.R(1,1) = ROcp20_114;
    sens.R(1,2) = ROcp20_214;
    sens.R(1,3) = ROcp20_314;
    sens.R(2,1) = ROcp20_415;
    sens.R(2,2) = ROcp20_515;
    sens.R(2,3) = ROcp20_615;
    sens.R(3,1) = ROcp20_715;
    sens.R(3,2) = ROcp20_815;
    sens.R(3,3) = ROcp20_915;
    sens.V(1) = VIcp20_152;
    sens.V(2) = VIcp20_252;
    sens.V(3) = VIcp20_352;
    sens.OM(1) = OMcp20_115;
    sens.OM(2) = OMcp20_215;
    sens.OM(3) = OMcp20_315;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp20_152_5;
    sens.J(1,6) = JTcp20_152_6;
    sens.J(1,14) = JTcp20_152_7;
    sens.J(1,15) = JTcp20_152_8;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp20_252_4;
    sens.J(2,5) = JTcp20_252_5;
    sens.J(2,6) = JTcp20_252_6;
    sens.J(2,14) = JTcp20_252_7;
    sens.J(2,15) = JTcp20_252_8;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp20_352_4;
    sens.J(3,5) = JTcp20_352_5;
    sens.J(3,6) = JTcp20_352_6;
    sens.J(3,14) = JTcp20_352_7;
    sens.J(3,15) = JTcp20_352_8;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,14) = ROcp20_46;
    sens.J(4,15) = ROcp20_114;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp20_85;
    sens.J(5,14) = ROcp20_56;
    sens.J(5,15) = ROcp20_214;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp20_95;
    sens.J(6,14) = ROcp20_66;
    sens.J(6,15) = ROcp20_314;
    sens.A(1) = ACcp20_152;
    sens.A(2) = ACcp20_252;
    sens.A(3) = ACcp20_352;
    sens.OMP(1) = OPcp20_115;
    sens.OMP(2) = OPcp20_215;
    sens.OMP(3) = OPcp20_315;
 
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

% = = Block_1_0_0_22_0_3 = = 
 
% Sensor Kinematics 


    ROcp21_114 = ROcp21_16*C14-S14*S5;
    ROcp21_214 = ROcp21_26*C14-ROcp21_85*S14;
    ROcp21_314 = ROcp21_36*C14-ROcp21_95*S14;
    ROcp21_714 = ROcp21_16*S14+C14*S5;
    ROcp21_814 = ROcp21_26*S14+ROcp21_85*C14;
    ROcp21_914 = ROcp21_36*S14+ROcp21_95*C14;
    ROcp21_415 = ROcp21_46*C15+ROcp21_714*S15;
    ROcp21_515 = ROcp21_56*C15+ROcp21_814*S15;
    ROcp21_615 = ROcp21_66*C15+ROcp21_914*S15;
    ROcp21_715 = -(ROcp21_46*S15-ROcp21_714*C15);
    ROcp21_815 = -(ROcp21_56*S15-ROcp21_814*C15);
    ROcp21_915 = -(ROcp21_66*S15-ROcp21_914*C15);
    ROcp21_116 = ROcp21_114*C16+ROcp21_415*S16;
    ROcp21_216 = ROcp21_214*C16+ROcp21_515*S16;
    ROcp21_316 = ROcp21_314*C16+ROcp21_615*S16;
    ROcp21_416 = -(ROcp21_114*S16-ROcp21_415*C16);
    ROcp21_516 = -(ROcp21_214*S16-ROcp21_515*C16);
    ROcp21_616 = -(ROcp21_314*S16-ROcp21_615*C16);
    RLcp21_114 = ROcp21_46*s.dpt(2,2);
    RLcp21_214 = ROcp21_56*s.dpt(2,2);
    RLcp21_314 = ROcp21_66*s.dpt(2,2);
    OMcp21_114 = OMcp21_16+ROcp21_46*qd(14);
    OMcp21_214 = OMcp21_26+ROcp21_56*qd(14);
    OMcp21_314 = OMcp21_36+ROcp21_66*qd(14);
    ORcp21_114 = OMcp21_26*RLcp21_314-OMcp21_36*RLcp21_214;
    ORcp21_214 = -(OMcp21_16*RLcp21_314-OMcp21_36*RLcp21_114);
    ORcp21_314 = OMcp21_16*RLcp21_214-OMcp21_26*RLcp21_114;
    OPcp21_114 = OPcp21_16+ROcp21_46*qdd(14)+qd(14)*(OMcp21_26*ROcp21_66-OMcp21_36*ROcp21_56);
    OPcp21_214 = OPcp21_26+ROcp21_56*qdd(14)-qd(14)*(OMcp21_16*ROcp21_66-OMcp21_36*ROcp21_46);
    OPcp21_314 = OPcp21_36+ROcp21_66*qdd(14)+qd(14)*(OMcp21_16*ROcp21_56-OMcp21_26*ROcp21_46);
    RLcp21_115 = ROcp21_46*s.dpt(2,20);
    RLcp21_215 = ROcp21_56*s.dpt(2,20);
    RLcp21_315 = ROcp21_66*s.dpt(2,20);
    OMcp21_115 = OMcp21_114+ROcp21_114*qd(15);
    OMcp21_215 = OMcp21_214+ROcp21_214*qd(15);
    OMcp21_315 = OMcp21_314+ROcp21_314*qd(15);
    ORcp21_115 = OMcp21_214*RLcp21_315-OMcp21_314*RLcp21_215;
    ORcp21_215 = -(OMcp21_114*RLcp21_315-OMcp21_314*RLcp21_115);
    ORcp21_315 = OMcp21_114*RLcp21_215-OMcp21_214*RLcp21_115;
    OPcp21_115 = OPcp21_114+ROcp21_114*qdd(15)+qd(15)*(OMcp21_214*ROcp21_314-OMcp21_314*ROcp21_214);
    OPcp21_215 = OPcp21_214+ROcp21_214*qdd(15)-qd(15)*(OMcp21_114*ROcp21_314-OMcp21_314*ROcp21_114);
    OPcp21_315 = OPcp21_314+ROcp21_314*qdd(15)+qd(15)*(OMcp21_114*ROcp21_214-OMcp21_214*ROcp21_114);
    RLcp21_116 = ROcp21_715*s.dpt(3,22);
    RLcp21_216 = ROcp21_815*s.dpt(3,22);
    RLcp21_316 = ROcp21_915*s.dpt(3,22);
    OMcp21_116 = OMcp21_115+ROcp21_715*qd(16);
    OMcp21_216 = OMcp21_215+ROcp21_815*qd(16);
    OMcp21_316 = OMcp21_315+ROcp21_915*qd(16);
    ORcp21_116 = OMcp21_215*RLcp21_316-OMcp21_315*RLcp21_216;
    ORcp21_216 = -(OMcp21_115*RLcp21_316-OMcp21_315*RLcp21_116);
    ORcp21_316 = OMcp21_115*RLcp21_216-OMcp21_215*RLcp21_116;
    OPcp21_116 = OPcp21_115+ROcp21_715*qdd(16)+qd(16)*(OMcp21_215*ROcp21_915-OMcp21_315*ROcp21_815);
    OPcp21_216 = OPcp21_215+ROcp21_815*qdd(16)-qd(16)*(OMcp21_115*ROcp21_915-OMcp21_315*ROcp21_715);
    OPcp21_316 = OPcp21_315+ROcp21_915*qdd(16)+qd(16)*(OMcp21_115*ROcp21_815-OMcp21_215*ROcp21_715);
    RLcp21_153 = ROcp21_715*s.dpt(3,24);
    RLcp21_253 = ROcp21_815*s.dpt(3,24);
    RLcp21_353 = ROcp21_915*s.dpt(3,24);
    POcp21_153 = RLcp21_114+RLcp21_115+RLcp21_116+RLcp21_153+q(1);
    POcp21_253 = RLcp21_214+RLcp21_215+RLcp21_216+RLcp21_253+q(2);
    POcp21_353 = RLcp21_314+RLcp21_315+RLcp21_316+RLcp21_353+q(3);
    JTcp21_253_4 = -(RLcp21_314+RLcp21_315+RLcp21_316+RLcp21_353);
    JTcp21_353_4 = RLcp21_214+RLcp21_215+RLcp21_216+RLcp21_253;
    JTcp21_153_5 = C4*(RLcp21_314+RLcp21_315+RLcp21_316+RLcp21_353)-S4*(RLcp21_214+RLcp21_215)-S4*(RLcp21_216+RLcp21_253);
    JTcp21_253_5 = S4*(RLcp21_114+RLcp21_115+RLcp21_116+RLcp21_153);
    JTcp21_353_5 = -C4*(RLcp21_114+RLcp21_115+RLcp21_116+RLcp21_153);
    JTcp21_153_6 = ROcp21_85*(RLcp21_314+RLcp21_315+RLcp21_316+RLcp21_353)-ROcp21_95*(RLcp21_214+RLcp21_215)-ROcp21_95*(RLcp21_216+RLcp21_253);
    JTcp21_253_6 = RLcp21_153*ROcp21_95-RLcp21_316*S5-RLcp21_353*S5+ROcp21_95*(RLcp21_114+RLcp21_115+RLcp21_116)-S5*(RLcp21_314+RLcp21_315);
    JTcp21_353_6 = RLcp21_216*S5-ROcp21_85*(RLcp21_114+RLcp21_115+RLcp21_116)+S5*(RLcp21_214+RLcp21_215)-RLcp21_153*ROcp21_85+RLcp21_253*S5;
    JTcp21_153_7 = ROcp21_56*(RLcp21_315+RLcp21_316)-ROcp21_66*(RLcp21_215+RLcp21_216)-RLcp21_253*ROcp21_66+RLcp21_353*ROcp21_56;
    JTcp21_253_7 = RLcp21_153*ROcp21_66-RLcp21_353*ROcp21_46-ROcp21_46*(RLcp21_315+RLcp21_316)+ROcp21_66*(RLcp21_115+RLcp21_116);
    JTcp21_353_7 = ROcp21_46*(RLcp21_215+RLcp21_216)-ROcp21_56*(RLcp21_115+RLcp21_116)-RLcp21_153*ROcp21_56+RLcp21_253*ROcp21_46;
    JTcp21_153_8 = ROcp21_214*(RLcp21_316+RLcp21_353)-ROcp21_314*(RLcp21_216+RLcp21_253);
    JTcp21_253_8 = -(ROcp21_114*(RLcp21_316+RLcp21_353)-ROcp21_314*(RLcp21_116+RLcp21_153));
    JTcp21_353_8 = ROcp21_114*(RLcp21_216+RLcp21_253)-ROcp21_214*(RLcp21_116+RLcp21_153);
    JTcp21_153_9 = -(RLcp21_253*ROcp21_915-RLcp21_353*ROcp21_815);
    JTcp21_253_9 = RLcp21_153*ROcp21_915-RLcp21_353*ROcp21_715;
    JTcp21_353_9 = -(RLcp21_153*ROcp21_815-RLcp21_253*ROcp21_715);
    ORcp21_153 = OMcp21_216*RLcp21_353-OMcp21_316*RLcp21_253;
    ORcp21_253 = -(OMcp21_116*RLcp21_353-OMcp21_316*RLcp21_153);
    ORcp21_353 = OMcp21_116*RLcp21_253-OMcp21_216*RLcp21_153;
    VIcp21_153 = ORcp21_114+ORcp21_115+ORcp21_116+ORcp21_153+qd(1);
    VIcp21_253 = ORcp21_214+ORcp21_215+ORcp21_216+ORcp21_253+qd(2);
    VIcp21_353 = ORcp21_314+ORcp21_315+ORcp21_316+ORcp21_353+qd(3);
    ACcp21_153 = qdd(1)+OMcp21_214*ORcp21_315+OMcp21_215*ORcp21_316+OMcp21_216*ORcp21_353+OMcp21_26*ORcp21_314-OMcp21_314*ORcp21_215-OMcp21_315*...
 ORcp21_216-OMcp21_316*ORcp21_253-OMcp21_36*ORcp21_214+OPcp21_214*RLcp21_315+OPcp21_215*RLcp21_316+OPcp21_216*RLcp21_353+OPcp21_26*RLcp21_314-...
 OPcp21_314*RLcp21_215-OPcp21_315*RLcp21_216-OPcp21_316*RLcp21_253-OPcp21_36*RLcp21_214;
    ACcp21_253 = qdd(2)-OMcp21_114*ORcp21_315-OMcp21_115*ORcp21_316-OMcp21_116*ORcp21_353-OMcp21_16*ORcp21_314+OMcp21_314*ORcp21_115+OMcp21_315*...
 ORcp21_116+OMcp21_316*ORcp21_153+OMcp21_36*ORcp21_114-OPcp21_114*RLcp21_315-OPcp21_115*RLcp21_316-OPcp21_116*RLcp21_353-OPcp21_16*RLcp21_314+...
 OPcp21_314*RLcp21_115+OPcp21_315*RLcp21_116+OPcp21_316*RLcp21_153+OPcp21_36*RLcp21_114;
    ACcp21_353 = qdd(3)+OMcp21_114*ORcp21_215+OMcp21_115*ORcp21_216+OMcp21_116*ORcp21_253+OMcp21_16*ORcp21_214-OMcp21_214*ORcp21_115-OMcp21_215*...
 ORcp21_116-OMcp21_216*ORcp21_153-OMcp21_26*ORcp21_114+OPcp21_114*RLcp21_215+OPcp21_115*RLcp21_216+OPcp21_116*RLcp21_253+OPcp21_16*RLcp21_214-...
 OPcp21_214*RLcp21_115-OPcp21_215*RLcp21_116-OPcp21_216*RLcp21_153-OPcp21_26*RLcp21_114;

% = = Block_1_0_0_22_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp21_153;
    sens.P(2) = POcp21_253;
    sens.P(3) = POcp21_353;
    sens.R(1,1) = ROcp21_116;
    sens.R(1,2) = ROcp21_216;
    sens.R(1,3) = ROcp21_316;
    sens.R(2,1) = ROcp21_416;
    sens.R(2,2) = ROcp21_516;
    sens.R(2,3) = ROcp21_616;
    sens.R(3,1) = ROcp21_715;
    sens.R(3,2) = ROcp21_815;
    sens.R(3,3) = ROcp21_915;
    sens.V(1) = VIcp21_153;
    sens.V(2) = VIcp21_253;
    sens.V(3) = VIcp21_353;
    sens.OM(1) = OMcp21_116;
    sens.OM(2) = OMcp21_216;
    sens.OM(3) = OMcp21_316;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp21_153_5;
    sens.J(1,6) = JTcp21_153_6;
    sens.J(1,14) = JTcp21_153_7;
    sens.J(1,15) = JTcp21_153_8;
    sens.J(1,16) = JTcp21_153_9;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp21_253_4;
    sens.J(2,5) = JTcp21_253_5;
    sens.J(2,6) = JTcp21_253_6;
    sens.J(2,14) = JTcp21_253_7;
    sens.J(2,15) = JTcp21_253_8;
    sens.J(2,16) = JTcp21_253_9;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp21_353_4;
    sens.J(3,5) = JTcp21_353_5;
    sens.J(3,6) = JTcp21_353_6;
    sens.J(3,14) = JTcp21_353_7;
    sens.J(3,15) = JTcp21_353_8;
    sens.J(3,16) = JTcp21_353_9;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,14) = ROcp21_46;
    sens.J(4,15) = ROcp21_114;
    sens.J(4,16) = ROcp21_715;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp21_85;
    sens.J(5,14) = ROcp21_56;
    sens.J(5,15) = ROcp21_214;
    sens.J(5,16) = ROcp21_815;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp21_95;
    sens.J(6,14) = ROcp21_66;
    sens.J(6,15) = ROcp21_314;
    sens.J(6,16) = ROcp21_915;
    sens.A(1) = ACcp21_153;
    sens.A(2) = ACcp21_253;
    sens.A(3) = ACcp21_353;
    sens.OMP(1) = OPcp21_116;
    sens.OMP(2) = OPcp21_216;
    sens.OMP(3) = OPcp21_316;
 
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

% = = Block_1_0_0_23_0_3 = = 
 
% Sensor Kinematics 


    ROcp22_114 = ROcp22_16*C14-S14*S5;
    ROcp22_214 = ROcp22_26*C14-ROcp22_85*S14;
    ROcp22_314 = ROcp22_36*C14-ROcp22_95*S14;
    ROcp22_714 = ROcp22_16*S14+C14*S5;
    ROcp22_814 = ROcp22_26*S14+ROcp22_85*C14;
    ROcp22_914 = ROcp22_36*S14+ROcp22_95*C14;
    ROcp22_415 = ROcp22_46*C15+ROcp22_714*S15;
    ROcp22_515 = ROcp22_56*C15+ROcp22_814*S15;
    ROcp22_615 = ROcp22_66*C15+ROcp22_914*S15;
    ROcp22_715 = -(ROcp22_46*S15-ROcp22_714*C15);
    ROcp22_815 = -(ROcp22_56*S15-ROcp22_814*C15);
    ROcp22_915 = -(ROcp22_66*S15-ROcp22_914*C15);
    ROcp22_116 = ROcp22_114*C16+ROcp22_415*S16;
    ROcp22_216 = ROcp22_214*C16+ROcp22_515*S16;
    ROcp22_316 = ROcp22_314*C16+ROcp22_615*S16;
    ROcp22_416 = -(ROcp22_114*S16-ROcp22_415*C16);
    ROcp22_516 = -(ROcp22_214*S16-ROcp22_515*C16);
    ROcp22_616 = -(ROcp22_314*S16-ROcp22_615*C16);
    RLcp22_114 = ROcp22_46*s.dpt(2,2);
    RLcp22_214 = ROcp22_56*s.dpt(2,2);
    RLcp22_314 = ROcp22_66*s.dpt(2,2);
    OMcp22_114 = OMcp22_16+ROcp22_46*qd(14);
    OMcp22_214 = OMcp22_26+ROcp22_56*qd(14);
    OMcp22_314 = OMcp22_36+ROcp22_66*qd(14);
    ORcp22_114 = OMcp22_26*RLcp22_314-OMcp22_36*RLcp22_214;
    ORcp22_214 = -(OMcp22_16*RLcp22_314-OMcp22_36*RLcp22_114);
    ORcp22_314 = OMcp22_16*RLcp22_214-OMcp22_26*RLcp22_114;
    OPcp22_114 = OPcp22_16+ROcp22_46*qdd(14)+qd(14)*(OMcp22_26*ROcp22_66-OMcp22_36*ROcp22_56);
    OPcp22_214 = OPcp22_26+ROcp22_56*qdd(14)-qd(14)*(OMcp22_16*ROcp22_66-OMcp22_36*ROcp22_46);
    OPcp22_314 = OPcp22_36+ROcp22_66*qdd(14)+qd(14)*(OMcp22_16*ROcp22_56-OMcp22_26*ROcp22_46);
    RLcp22_115 = ROcp22_46*s.dpt(2,20);
    RLcp22_215 = ROcp22_56*s.dpt(2,20);
    RLcp22_315 = ROcp22_66*s.dpt(2,20);
    OMcp22_115 = OMcp22_114+ROcp22_114*qd(15);
    OMcp22_215 = OMcp22_214+ROcp22_214*qd(15);
    OMcp22_315 = OMcp22_314+ROcp22_314*qd(15);
    ORcp22_115 = OMcp22_214*RLcp22_315-OMcp22_314*RLcp22_215;
    ORcp22_215 = -(OMcp22_114*RLcp22_315-OMcp22_314*RLcp22_115);
    ORcp22_315 = OMcp22_114*RLcp22_215-OMcp22_214*RLcp22_115;
    OPcp22_115 = OPcp22_114+ROcp22_114*qdd(15)+qd(15)*(OMcp22_214*ROcp22_314-OMcp22_314*ROcp22_214);
    OPcp22_215 = OPcp22_214+ROcp22_214*qdd(15)-qd(15)*(OMcp22_114*ROcp22_314-OMcp22_314*ROcp22_114);
    OPcp22_315 = OPcp22_314+ROcp22_314*qdd(15)+qd(15)*(OMcp22_114*ROcp22_214-OMcp22_214*ROcp22_114);
    RLcp22_116 = ROcp22_715*s.dpt(3,22);
    RLcp22_216 = ROcp22_815*s.dpt(3,22);
    RLcp22_316 = ROcp22_915*s.dpt(3,22);
    OMcp22_116 = OMcp22_115+ROcp22_715*qd(16);
    OMcp22_216 = OMcp22_215+ROcp22_815*qd(16);
    OMcp22_316 = OMcp22_315+ROcp22_915*qd(16);
    ORcp22_116 = OMcp22_215*RLcp22_316-OMcp22_315*RLcp22_216;
    ORcp22_216 = -(OMcp22_115*RLcp22_316-OMcp22_315*RLcp22_116);
    ORcp22_316 = OMcp22_115*RLcp22_216-OMcp22_215*RLcp22_116;
    OPcp22_116 = OPcp22_115+ROcp22_715*qdd(16)+qd(16)*(OMcp22_215*ROcp22_915-OMcp22_315*ROcp22_815);
    OPcp22_216 = OPcp22_215+ROcp22_815*qdd(16)-qd(16)*(OMcp22_115*ROcp22_915-OMcp22_315*ROcp22_715);
    OPcp22_316 = OPcp22_315+ROcp22_915*qdd(16)+qd(16)*(OMcp22_115*ROcp22_815-OMcp22_215*ROcp22_715);
    RLcp22_154 = ROcp22_116*s.dpt(1,25)+ROcp22_416*s.dpt(2,25)+ROcp22_715*s.dpt(3,25);
    RLcp22_254 = ROcp22_216*s.dpt(1,25)+ROcp22_516*s.dpt(2,25)+ROcp22_815*s.dpt(3,25);
    RLcp22_354 = ROcp22_316*s.dpt(1,25)+ROcp22_616*s.dpt(2,25)+ROcp22_915*s.dpt(3,25);
    POcp22_154 = RLcp22_114+RLcp22_115+RLcp22_116+RLcp22_154+q(1);
    POcp22_254 = RLcp22_214+RLcp22_215+RLcp22_216+RLcp22_254+q(2);
    POcp22_354 = RLcp22_314+RLcp22_315+RLcp22_316+RLcp22_354+q(3);
    JTcp22_254_4 = -(RLcp22_314+RLcp22_315+RLcp22_316+RLcp22_354);
    JTcp22_354_4 = RLcp22_214+RLcp22_215+RLcp22_216+RLcp22_254;
    JTcp22_154_5 = C4*(RLcp22_314+RLcp22_315+RLcp22_316+RLcp22_354)-S4*(RLcp22_214+RLcp22_215)-S4*(RLcp22_216+RLcp22_254);
    JTcp22_254_5 = S4*(RLcp22_114+RLcp22_115+RLcp22_116+RLcp22_154);
    JTcp22_354_5 = -C4*(RLcp22_114+RLcp22_115+RLcp22_116+RLcp22_154);
    JTcp22_154_6 = ROcp22_85*(RLcp22_314+RLcp22_315+RLcp22_316+RLcp22_354)-ROcp22_95*(RLcp22_214+RLcp22_215)-ROcp22_95*(RLcp22_216+RLcp22_254);
    JTcp22_254_6 = RLcp22_154*ROcp22_95-RLcp22_316*S5-RLcp22_354*S5+ROcp22_95*(RLcp22_114+RLcp22_115+RLcp22_116)-S5*(RLcp22_314+RLcp22_315);
    JTcp22_354_6 = RLcp22_216*S5-ROcp22_85*(RLcp22_114+RLcp22_115+RLcp22_116)+S5*(RLcp22_214+RLcp22_215)-RLcp22_154*ROcp22_85+RLcp22_254*S5;
    JTcp22_154_7 = ROcp22_56*(RLcp22_315+RLcp22_316)-ROcp22_66*(RLcp22_215+RLcp22_216)-RLcp22_254*ROcp22_66+RLcp22_354*ROcp22_56;
    JTcp22_254_7 = RLcp22_154*ROcp22_66-RLcp22_354*ROcp22_46-ROcp22_46*(RLcp22_315+RLcp22_316)+ROcp22_66*(RLcp22_115+RLcp22_116);
    JTcp22_354_7 = ROcp22_46*(RLcp22_215+RLcp22_216)-ROcp22_56*(RLcp22_115+RLcp22_116)-RLcp22_154*ROcp22_56+RLcp22_254*ROcp22_46;
    JTcp22_154_8 = ROcp22_214*(RLcp22_316+RLcp22_354)-ROcp22_314*(RLcp22_216+RLcp22_254);
    JTcp22_254_8 = -(ROcp22_114*(RLcp22_316+RLcp22_354)-ROcp22_314*(RLcp22_116+RLcp22_154));
    JTcp22_354_8 = ROcp22_114*(RLcp22_216+RLcp22_254)-ROcp22_214*(RLcp22_116+RLcp22_154);
    JTcp22_154_9 = -(RLcp22_254*ROcp22_915-RLcp22_354*ROcp22_815);
    JTcp22_254_9 = RLcp22_154*ROcp22_915-RLcp22_354*ROcp22_715;
    JTcp22_354_9 = -(RLcp22_154*ROcp22_815-RLcp22_254*ROcp22_715);
    ORcp22_154 = OMcp22_216*RLcp22_354-OMcp22_316*RLcp22_254;
    ORcp22_254 = -(OMcp22_116*RLcp22_354-OMcp22_316*RLcp22_154);
    ORcp22_354 = OMcp22_116*RLcp22_254-OMcp22_216*RLcp22_154;
    VIcp22_154 = ORcp22_114+ORcp22_115+ORcp22_116+ORcp22_154+qd(1);
    VIcp22_254 = ORcp22_214+ORcp22_215+ORcp22_216+ORcp22_254+qd(2);
    VIcp22_354 = ORcp22_314+ORcp22_315+ORcp22_316+ORcp22_354+qd(3);
    ACcp22_154 = qdd(1)+OMcp22_214*ORcp22_315+OMcp22_215*ORcp22_316+OMcp22_216*ORcp22_354+OMcp22_26*ORcp22_314-OMcp22_314*ORcp22_215-OMcp22_315*...
 ORcp22_216-OMcp22_316*ORcp22_254-OMcp22_36*ORcp22_214+OPcp22_214*RLcp22_315+OPcp22_215*RLcp22_316+OPcp22_216*RLcp22_354+OPcp22_26*RLcp22_314-...
 OPcp22_314*RLcp22_215-OPcp22_315*RLcp22_216-OPcp22_316*RLcp22_254-OPcp22_36*RLcp22_214;
    ACcp22_254 = qdd(2)-OMcp22_114*ORcp22_315-OMcp22_115*ORcp22_316-OMcp22_116*ORcp22_354-OMcp22_16*ORcp22_314+OMcp22_314*ORcp22_115+OMcp22_315*...
 ORcp22_116+OMcp22_316*ORcp22_154+OMcp22_36*ORcp22_114-OPcp22_114*RLcp22_315-OPcp22_115*RLcp22_316-OPcp22_116*RLcp22_354-OPcp22_16*RLcp22_314+...
 OPcp22_314*RLcp22_115+OPcp22_315*RLcp22_116+OPcp22_316*RLcp22_154+OPcp22_36*RLcp22_114;
    ACcp22_354 = qdd(3)+OMcp22_114*ORcp22_215+OMcp22_115*ORcp22_216+OMcp22_116*ORcp22_254+OMcp22_16*ORcp22_214-OMcp22_214*ORcp22_115-OMcp22_215*...
 ORcp22_116-OMcp22_216*ORcp22_154-OMcp22_26*ORcp22_114+OPcp22_114*RLcp22_215+OPcp22_115*RLcp22_216+OPcp22_116*RLcp22_254+OPcp22_16*RLcp22_214-...
 OPcp22_214*RLcp22_115-OPcp22_215*RLcp22_116-OPcp22_216*RLcp22_154-OPcp22_26*RLcp22_114;

% = = Block_1_0_0_23_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp22_154;
    sens.P(2) = POcp22_254;
    sens.P(3) = POcp22_354;
    sens.R(1,1) = ROcp22_116;
    sens.R(1,2) = ROcp22_216;
    sens.R(1,3) = ROcp22_316;
    sens.R(2,1) = ROcp22_416;
    sens.R(2,2) = ROcp22_516;
    sens.R(2,3) = ROcp22_616;
    sens.R(3,1) = ROcp22_715;
    sens.R(3,2) = ROcp22_815;
    sens.R(3,3) = ROcp22_915;
    sens.V(1) = VIcp22_154;
    sens.V(2) = VIcp22_254;
    sens.V(3) = VIcp22_354;
    sens.OM(1) = OMcp22_116;
    sens.OM(2) = OMcp22_216;
    sens.OM(3) = OMcp22_316;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp22_154_5;
    sens.J(1,6) = JTcp22_154_6;
    sens.J(1,14) = JTcp22_154_7;
    sens.J(1,15) = JTcp22_154_8;
    sens.J(1,16) = JTcp22_154_9;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp22_254_4;
    sens.J(2,5) = JTcp22_254_5;
    sens.J(2,6) = JTcp22_254_6;
    sens.J(2,14) = JTcp22_254_7;
    sens.J(2,15) = JTcp22_254_8;
    sens.J(2,16) = JTcp22_254_9;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp22_354_4;
    sens.J(3,5) = JTcp22_354_5;
    sens.J(3,6) = JTcp22_354_6;
    sens.J(3,14) = JTcp22_354_7;
    sens.J(3,15) = JTcp22_354_8;
    sens.J(3,16) = JTcp22_354_9;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,14) = ROcp22_46;
    sens.J(4,15) = ROcp22_114;
    sens.J(4,16) = ROcp22_715;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp22_85;
    sens.J(5,14) = ROcp22_56;
    sens.J(5,15) = ROcp22_214;
    sens.J(5,16) = ROcp22_815;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp22_95;
    sens.J(6,14) = ROcp22_66;
    sens.J(6,15) = ROcp22_314;
    sens.J(6,16) = ROcp22_915;
    sens.A(1) = ACcp22_154;
    sens.A(2) = ACcp22_254;
    sens.A(3) = ACcp22_354;
    sens.OMP(1) = OPcp22_116;
    sens.OMP(2) = OPcp22_216;
    sens.OMP(3) = OPcp22_316;
 
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

% = = Block_1_0_0_24_0_3 = = 
 
% Sensor Kinematics 


    ROcp23_114 = ROcp23_16*C14-S14*S5;
    ROcp23_214 = ROcp23_26*C14-ROcp23_85*S14;
    ROcp23_314 = ROcp23_36*C14-ROcp23_95*S14;
    ROcp23_714 = ROcp23_16*S14+C14*S5;
    ROcp23_814 = ROcp23_26*S14+ROcp23_85*C14;
    ROcp23_914 = ROcp23_36*S14+ROcp23_95*C14;
    ROcp23_415 = ROcp23_46*C15+ROcp23_714*S15;
    ROcp23_515 = ROcp23_56*C15+ROcp23_814*S15;
    ROcp23_615 = ROcp23_66*C15+ROcp23_914*S15;
    ROcp23_715 = -(ROcp23_46*S15-ROcp23_714*C15);
    ROcp23_815 = -(ROcp23_56*S15-ROcp23_814*C15);
    ROcp23_915 = -(ROcp23_66*S15-ROcp23_914*C15);
    ROcp23_116 = ROcp23_114*C16+ROcp23_415*S16;
    ROcp23_216 = ROcp23_214*C16+ROcp23_515*S16;
    ROcp23_316 = ROcp23_314*C16+ROcp23_615*S16;
    ROcp23_416 = -(ROcp23_114*S16-ROcp23_415*C16);
    ROcp23_516 = -(ROcp23_214*S16-ROcp23_515*C16);
    ROcp23_616 = -(ROcp23_314*S16-ROcp23_615*C16);
    ROcp23_117 = ROcp23_116*C17-ROcp23_715*S17;
    ROcp23_217 = ROcp23_216*C17-ROcp23_815*S17;
    ROcp23_317 = ROcp23_316*C17-ROcp23_915*S17;
    ROcp23_717 = ROcp23_116*S17+ROcp23_715*C17;
    ROcp23_817 = ROcp23_216*S17+ROcp23_815*C17;
    ROcp23_917 = ROcp23_316*S17+ROcp23_915*C17;
    RLcp23_114 = ROcp23_46*s.dpt(2,2);
    RLcp23_214 = ROcp23_56*s.dpt(2,2);
    RLcp23_314 = ROcp23_66*s.dpt(2,2);
    OMcp23_114 = OMcp23_16+ROcp23_46*qd(14);
    OMcp23_214 = OMcp23_26+ROcp23_56*qd(14);
    OMcp23_314 = OMcp23_36+ROcp23_66*qd(14);
    ORcp23_114 = OMcp23_26*RLcp23_314-OMcp23_36*RLcp23_214;
    ORcp23_214 = -(OMcp23_16*RLcp23_314-OMcp23_36*RLcp23_114);
    ORcp23_314 = OMcp23_16*RLcp23_214-OMcp23_26*RLcp23_114;
    OPcp23_114 = OPcp23_16+ROcp23_46*qdd(14)+qd(14)*(OMcp23_26*ROcp23_66-OMcp23_36*ROcp23_56);
    OPcp23_214 = OPcp23_26+ROcp23_56*qdd(14)-qd(14)*(OMcp23_16*ROcp23_66-OMcp23_36*ROcp23_46);
    OPcp23_314 = OPcp23_36+ROcp23_66*qdd(14)+qd(14)*(OMcp23_16*ROcp23_56-OMcp23_26*ROcp23_46);
    RLcp23_115 = ROcp23_46*s.dpt(2,20);
    RLcp23_215 = ROcp23_56*s.dpt(2,20);
    RLcp23_315 = ROcp23_66*s.dpt(2,20);
    OMcp23_115 = OMcp23_114+ROcp23_114*qd(15);
    OMcp23_215 = OMcp23_214+ROcp23_214*qd(15);
    OMcp23_315 = OMcp23_314+ROcp23_314*qd(15);
    ORcp23_115 = OMcp23_214*RLcp23_315-OMcp23_314*RLcp23_215;
    ORcp23_215 = -(OMcp23_114*RLcp23_315-OMcp23_314*RLcp23_115);
    ORcp23_315 = OMcp23_114*RLcp23_215-OMcp23_214*RLcp23_115;
    OPcp23_115 = OPcp23_114+ROcp23_114*qdd(15)+qd(15)*(OMcp23_214*ROcp23_314-OMcp23_314*ROcp23_214);
    OPcp23_215 = OPcp23_214+ROcp23_214*qdd(15)-qd(15)*(OMcp23_114*ROcp23_314-OMcp23_314*ROcp23_114);
    OPcp23_315 = OPcp23_314+ROcp23_314*qdd(15)+qd(15)*(OMcp23_114*ROcp23_214-OMcp23_214*ROcp23_114);
    RLcp23_116 = ROcp23_715*s.dpt(3,22);
    RLcp23_216 = ROcp23_815*s.dpt(3,22);
    RLcp23_316 = ROcp23_915*s.dpt(3,22);
    OMcp23_116 = OMcp23_115+ROcp23_715*qd(16);
    OMcp23_216 = OMcp23_215+ROcp23_815*qd(16);
    OMcp23_316 = OMcp23_315+ROcp23_915*qd(16);
    ORcp23_116 = OMcp23_215*RLcp23_316-OMcp23_315*RLcp23_216;
    ORcp23_216 = -(OMcp23_115*RLcp23_316-OMcp23_315*RLcp23_116);
    ORcp23_316 = OMcp23_115*RLcp23_216-OMcp23_215*RLcp23_116;
    OPcp23_116 = OPcp23_115+ROcp23_715*qdd(16)+qd(16)*(OMcp23_215*ROcp23_915-OMcp23_315*ROcp23_815);
    OPcp23_216 = OPcp23_215+ROcp23_815*qdd(16)-qd(16)*(OMcp23_115*ROcp23_915-OMcp23_315*ROcp23_715);
    OPcp23_316 = OPcp23_315+ROcp23_915*qdd(16)+qd(16)*(OMcp23_115*ROcp23_815-OMcp23_215*ROcp23_715);
    RLcp23_117 = ROcp23_715*s.dpt(3,24);
    RLcp23_217 = ROcp23_815*s.dpt(3,24);
    RLcp23_317 = ROcp23_915*s.dpt(3,24);
    OMcp23_117 = OMcp23_116+ROcp23_416*qd(17);
    OMcp23_217 = OMcp23_216+ROcp23_516*qd(17);
    OMcp23_317 = OMcp23_316+ROcp23_616*qd(17);
    ORcp23_117 = OMcp23_216*RLcp23_317-OMcp23_316*RLcp23_217;
    ORcp23_217 = -(OMcp23_116*RLcp23_317-OMcp23_316*RLcp23_117);
    ORcp23_317 = OMcp23_116*RLcp23_217-OMcp23_216*RLcp23_117;
    OPcp23_117 = OPcp23_116+ROcp23_416*qdd(17)+qd(17)*(OMcp23_216*ROcp23_616-OMcp23_316*ROcp23_516);
    OPcp23_217 = OPcp23_216+ROcp23_516*qdd(17)-qd(17)*(OMcp23_116*ROcp23_616-OMcp23_316*ROcp23_416);
    OPcp23_317 = OPcp23_316+ROcp23_616*qdd(17)+qd(17)*(OMcp23_116*ROcp23_516-OMcp23_216*ROcp23_416);
    RLcp23_155 = ROcp23_717*s.dpt(3,26);
    RLcp23_255 = ROcp23_817*s.dpt(3,26);
    RLcp23_355 = ROcp23_917*s.dpt(3,26);
    POcp23_155 = RLcp23_114+RLcp23_115+RLcp23_116+RLcp23_117+RLcp23_155+q(1);
    POcp23_255 = RLcp23_214+RLcp23_215+RLcp23_216+RLcp23_217+RLcp23_255+q(2);
    POcp23_355 = RLcp23_314+RLcp23_315+RLcp23_316+RLcp23_317+RLcp23_355+q(3);
    JTcp23_255_4 = -(RLcp23_314+RLcp23_315+RLcp23_316+RLcp23_317+RLcp23_355);
    JTcp23_355_4 = RLcp23_214+RLcp23_215+RLcp23_216+RLcp23_217+RLcp23_255;
    JTcp23_155_5 = C4*(RLcp23_314+RLcp23_315+RLcp23_316+RLcp23_317)-S4*(RLcp23_214+RLcp23_215)-S4*(RLcp23_216+RLcp23_217)-RLcp23_255*S4+RLcp23_355*...
 C4;
    JTcp23_255_5 = S4*(RLcp23_114+RLcp23_115+RLcp23_116+RLcp23_117+RLcp23_155);
    JTcp23_355_5 = -C4*(RLcp23_114+RLcp23_115+RLcp23_116+RLcp23_117+RLcp23_155);
    JTcp23_155_6 = ROcp23_85*(RLcp23_314+RLcp23_315+RLcp23_316+RLcp23_317)-ROcp23_95*(RLcp23_214+RLcp23_215)-ROcp23_95*(RLcp23_216+RLcp23_217)-...
 RLcp23_255*ROcp23_95+RLcp23_355*ROcp23_85;
    JTcp23_255_6 = -(RLcp23_355*S5-ROcp23_95*(RLcp23_114+RLcp23_115+RLcp23_116+RLcp23_117+RLcp23_155)+S5*(RLcp23_314+RLcp23_315)+S5*(RLcp23_316+...
 RLcp23_317));
    JTcp23_355_6 = RLcp23_255*S5-ROcp23_85*(RLcp23_114+RLcp23_115+RLcp23_116+RLcp23_117+RLcp23_155)+S5*(RLcp23_214+RLcp23_215)+S5*(RLcp23_216+...
 RLcp23_217);
    JTcp23_155_7 = ROcp23_56*(RLcp23_315+RLcp23_316+RLcp23_317+RLcp23_355)-ROcp23_66*(RLcp23_215+RLcp23_216)-ROcp23_66*(RLcp23_217+RLcp23_255);
    JTcp23_255_7 = -(ROcp23_46*(RLcp23_315+RLcp23_316+RLcp23_317+RLcp23_355)-ROcp23_66*(RLcp23_115+RLcp23_116)-ROcp23_66*(RLcp23_117+RLcp23_155));
    JTcp23_355_7 = ROcp23_46*(RLcp23_215+RLcp23_216+RLcp23_217+RLcp23_255)-ROcp23_56*(RLcp23_115+RLcp23_116)-ROcp23_56*(RLcp23_117+RLcp23_155);
    JTcp23_155_8 = ROcp23_214*(RLcp23_316+RLcp23_317)-ROcp23_314*(RLcp23_216+RLcp23_217)-RLcp23_255*ROcp23_314+RLcp23_355*ROcp23_214;
    JTcp23_255_8 = RLcp23_155*ROcp23_314-RLcp23_355*ROcp23_114-ROcp23_114*(RLcp23_316+RLcp23_317)+ROcp23_314*(RLcp23_116+RLcp23_117);
    JTcp23_355_8 = ROcp23_114*(RLcp23_216+RLcp23_217)-ROcp23_214*(RLcp23_116+RLcp23_117)-RLcp23_155*ROcp23_214+RLcp23_255*ROcp23_114;
    JTcp23_155_9 = ROcp23_815*(RLcp23_317+RLcp23_355)-ROcp23_915*(RLcp23_217+RLcp23_255);
    JTcp23_255_9 = -(ROcp23_715*(RLcp23_317+RLcp23_355)-ROcp23_915*(RLcp23_117+RLcp23_155));
    JTcp23_355_9 = ROcp23_715*(RLcp23_217+RLcp23_255)-ROcp23_815*(RLcp23_117+RLcp23_155);
    JTcp23_155_10 = -(RLcp23_255*ROcp23_616-RLcp23_355*ROcp23_516);
    JTcp23_255_10 = RLcp23_155*ROcp23_616-RLcp23_355*ROcp23_416;
    JTcp23_355_10 = -(RLcp23_155*ROcp23_516-RLcp23_255*ROcp23_416);
    ORcp23_155 = OMcp23_217*RLcp23_355-OMcp23_317*RLcp23_255;
    ORcp23_255 = -(OMcp23_117*RLcp23_355-OMcp23_317*RLcp23_155);
    ORcp23_355 = OMcp23_117*RLcp23_255-OMcp23_217*RLcp23_155;
    VIcp23_155 = ORcp23_114+ORcp23_115+ORcp23_116+ORcp23_117+ORcp23_155+qd(1);
    VIcp23_255 = ORcp23_214+ORcp23_215+ORcp23_216+ORcp23_217+ORcp23_255+qd(2);
    VIcp23_355 = ORcp23_314+ORcp23_315+ORcp23_316+ORcp23_317+ORcp23_355+qd(3);
    ACcp23_155 = qdd(1)+OMcp23_214*ORcp23_315+OMcp23_215*ORcp23_316+OMcp23_216*ORcp23_317+OMcp23_217*ORcp23_355+OMcp23_26*ORcp23_314-OMcp23_314*...
 ORcp23_215-OMcp23_315*ORcp23_216-OMcp23_316*ORcp23_217-OMcp23_317*ORcp23_255-OMcp23_36*ORcp23_214+OPcp23_214*RLcp23_315+OPcp23_215*RLcp23_316+...
 OPcp23_216*RLcp23_317+OPcp23_217*RLcp23_355+OPcp23_26*RLcp23_314-OPcp23_314*RLcp23_215-OPcp23_315*RLcp23_216-OPcp23_316*RLcp23_217-OPcp23_317*...
 RLcp23_255-OPcp23_36*RLcp23_214;
    ACcp23_255 = qdd(2)-OMcp23_114*ORcp23_315-OMcp23_115*ORcp23_316-OMcp23_116*ORcp23_317-OMcp23_117*ORcp23_355-OMcp23_16*ORcp23_314+OMcp23_314*...
 ORcp23_115+OMcp23_315*ORcp23_116+OMcp23_316*ORcp23_117+OMcp23_317*ORcp23_155+OMcp23_36*ORcp23_114-OPcp23_114*RLcp23_315-OPcp23_115*RLcp23_316-...
 OPcp23_116*RLcp23_317-OPcp23_117*RLcp23_355-OPcp23_16*RLcp23_314+OPcp23_314*RLcp23_115+OPcp23_315*RLcp23_116+OPcp23_316*RLcp23_117+OPcp23_317*...
 RLcp23_155+OPcp23_36*RLcp23_114;
    ACcp23_355 = qdd(3)+OMcp23_114*ORcp23_215+OMcp23_115*ORcp23_216+OMcp23_116*ORcp23_217+OMcp23_117*ORcp23_255+OMcp23_16*ORcp23_214-OMcp23_214*...
 ORcp23_115-OMcp23_215*ORcp23_116-OMcp23_216*ORcp23_117-OMcp23_217*ORcp23_155-OMcp23_26*ORcp23_114+OPcp23_114*RLcp23_215+OPcp23_115*RLcp23_216+...
 OPcp23_116*RLcp23_217+OPcp23_117*RLcp23_255+OPcp23_16*RLcp23_214-OPcp23_214*RLcp23_115-OPcp23_215*RLcp23_116-OPcp23_216*RLcp23_117-OPcp23_217*...
 RLcp23_155-OPcp23_26*RLcp23_114;

% = = Block_1_0_0_24_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp23_155;
    sens.P(2) = POcp23_255;
    sens.P(3) = POcp23_355;
    sens.R(1,1) = ROcp23_117;
    sens.R(1,2) = ROcp23_217;
    sens.R(1,3) = ROcp23_317;
    sens.R(2,1) = ROcp23_416;
    sens.R(2,2) = ROcp23_516;
    sens.R(2,3) = ROcp23_616;
    sens.R(3,1) = ROcp23_717;
    sens.R(3,2) = ROcp23_817;
    sens.R(3,3) = ROcp23_917;
    sens.V(1) = VIcp23_155;
    sens.V(2) = VIcp23_255;
    sens.V(3) = VIcp23_355;
    sens.OM(1) = OMcp23_117;
    sens.OM(2) = OMcp23_217;
    sens.OM(3) = OMcp23_317;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp23_155_5;
    sens.J(1,6) = JTcp23_155_6;
    sens.J(1,14) = JTcp23_155_7;
    sens.J(1,15) = JTcp23_155_8;
    sens.J(1,16) = JTcp23_155_9;
    sens.J(1,17) = JTcp23_155_10;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp23_255_4;
    sens.J(2,5) = JTcp23_255_5;
    sens.J(2,6) = JTcp23_255_6;
    sens.J(2,14) = JTcp23_255_7;
    sens.J(2,15) = JTcp23_255_8;
    sens.J(2,16) = JTcp23_255_9;
    sens.J(2,17) = JTcp23_255_10;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp23_355_4;
    sens.J(3,5) = JTcp23_355_5;
    sens.J(3,6) = JTcp23_355_6;
    sens.J(3,14) = JTcp23_355_7;
    sens.J(3,15) = JTcp23_355_8;
    sens.J(3,16) = JTcp23_355_9;
    sens.J(3,17) = JTcp23_355_10;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,14) = ROcp23_46;
    sens.J(4,15) = ROcp23_114;
    sens.J(4,16) = ROcp23_715;
    sens.J(4,17) = ROcp23_416;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp23_85;
    sens.J(5,14) = ROcp23_56;
    sens.J(5,15) = ROcp23_214;
    sens.J(5,16) = ROcp23_815;
    sens.J(5,17) = ROcp23_516;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp23_95;
    sens.J(6,14) = ROcp23_66;
    sens.J(6,15) = ROcp23_314;
    sens.J(6,16) = ROcp23_915;
    sens.J(6,17) = ROcp23_616;
    sens.A(1) = ACcp23_155;
    sens.A(2) = ACcp23_255;
    sens.A(3) = ACcp23_355;
    sens.OMP(1) = OPcp23_117;
    sens.OMP(2) = OPcp23_217;
    sens.OMP(3) = OPcp23_317;
 
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

% = = Block_1_0_0_25_0_3 = = 
 
% Sensor Kinematics 


    ROcp24_114 = ROcp24_16*C14-S14*S5;
    ROcp24_214 = ROcp24_26*C14-ROcp24_85*S14;
    ROcp24_314 = ROcp24_36*C14-ROcp24_95*S14;
    ROcp24_714 = ROcp24_16*S14+C14*S5;
    ROcp24_814 = ROcp24_26*S14+ROcp24_85*C14;
    ROcp24_914 = ROcp24_36*S14+ROcp24_95*C14;
    ROcp24_415 = ROcp24_46*C15+ROcp24_714*S15;
    ROcp24_515 = ROcp24_56*C15+ROcp24_814*S15;
    ROcp24_615 = ROcp24_66*C15+ROcp24_914*S15;
    ROcp24_715 = -(ROcp24_46*S15-ROcp24_714*C15);
    ROcp24_815 = -(ROcp24_56*S15-ROcp24_814*C15);
    ROcp24_915 = -(ROcp24_66*S15-ROcp24_914*C15);
    ROcp24_116 = ROcp24_114*C16+ROcp24_415*S16;
    ROcp24_216 = ROcp24_214*C16+ROcp24_515*S16;
    ROcp24_316 = ROcp24_314*C16+ROcp24_615*S16;
    ROcp24_416 = -(ROcp24_114*S16-ROcp24_415*C16);
    ROcp24_516 = -(ROcp24_214*S16-ROcp24_515*C16);
    ROcp24_616 = -(ROcp24_314*S16-ROcp24_615*C16);
    ROcp24_117 = ROcp24_116*C17-ROcp24_715*S17;
    ROcp24_217 = ROcp24_216*C17-ROcp24_815*S17;
    ROcp24_317 = ROcp24_316*C17-ROcp24_915*S17;
    ROcp24_717 = ROcp24_116*S17+ROcp24_715*C17;
    ROcp24_817 = ROcp24_216*S17+ROcp24_815*C17;
    ROcp24_917 = ROcp24_316*S17+ROcp24_915*C17;
    RLcp24_114 = ROcp24_46*s.dpt(2,2);
    RLcp24_214 = ROcp24_56*s.dpt(2,2);
    RLcp24_314 = ROcp24_66*s.dpt(2,2);
    OMcp24_114 = OMcp24_16+ROcp24_46*qd(14);
    OMcp24_214 = OMcp24_26+ROcp24_56*qd(14);
    OMcp24_314 = OMcp24_36+ROcp24_66*qd(14);
    ORcp24_114 = OMcp24_26*RLcp24_314-OMcp24_36*RLcp24_214;
    ORcp24_214 = -(OMcp24_16*RLcp24_314-OMcp24_36*RLcp24_114);
    ORcp24_314 = OMcp24_16*RLcp24_214-OMcp24_26*RLcp24_114;
    OPcp24_114 = OPcp24_16+ROcp24_46*qdd(14)+qd(14)*(OMcp24_26*ROcp24_66-OMcp24_36*ROcp24_56);
    OPcp24_214 = OPcp24_26+ROcp24_56*qdd(14)-qd(14)*(OMcp24_16*ROcp24_66-OMcp24_36*ROcp24_46);
    OPcp24_314 = OPcp24_36+ROcp24_66*qdd(14)+qd(14)*(OMcp24_16*ROcp24_56-OMcp24_26*ROcp24_46);
    RLcp24_115 = ROcp24_46*s.dpt(2,20);
    RLcp24_215 = ROcp24_56*s.dpt(2,20);
    RLcp24_315 = ROcp24_66*s.dpt(2,20);
    OMcp24_115 = OMcp24_114+ROcp24_114*qd(15);
    OMcp24_215 = OMcp24_214+ROcp24_214*qd(15);
    OMcp24_315 = OMcp24_314+ROcp24_314*qd(15);
    ORcp24_115 = OMcp24_214*RLcp24_315-OMcp24_314*RLcp24_215;
    ORcp24_215 = -(OMcp24_114*RLcp24_315-OMcp24_314*RLcp24_115);
    ORcp24_315 = OMcp24_114*RLcp24_215-OMcp24_214*RLcp24_115;
    OPcp24_115 = OPcp24_114+ROcp24_114*qdd(15)+qd(15)*(OMcp24_214*ROcp24_314-OMcp24_314*ROcp24_214);
    OPcp24_215 = OPcp24_214+ROcp24_214*qdd(15)-qd(15)*(OMcp24_114*ROcp24_314-OMcp24_314*ROcp24_114);
    OPcp24_315 = OPcp24_314+ROcp24_314*qdd(15)+qd(15)*(OMcp24_114*ROcp24_214-OMcp24_214*ROcp24_114);
    RLcp24_116 = ROcp24_715*s.dpt(3,22);
    RLcp24_216 = ROcp24_815*s.dpt(3,22);
    RLcp24_316 = ROcp24_915*s.dpt(3,22);
    OMcp24_116 = OMcp24_115+ROcp24_715*qd(16);
    OMcp24_216 = OMcp24_215+ROcp24_815*qd(16);
    OMcp24_316 = OMcp24_315+ROcp24_915*qd(16);
    ORcp24_116 = OMcp24_215*RLcp24_316-OMcp24_315*RLcp24_216;
    ORcp24_216 = -(OMcp24_115*RLcp24_316-OMcp24_315*RLcp24_116);
    ORcp24_316 = OMcp24_115*RLcp24_216-OMcp24_215*RLcp24_116;
    OPcp24_116 = OPcp24_115+ROcp24_715*qdd(16)+qd(16)*(OMcp24_215*ROcp24_915-OMcp24_315*ROcp24_815);
    OPcp24_216 = OPcp24_215+ROcp24_815*qdd(16)-qd(16)*(OMcp24_115*ROcp24_915-OMcp24_315*ROcp24_715);
    OPcp24_316 = OPcp24_315+ROcp24_915*qdd(16)+qd(16)*(OMcp24_115*ROcp24_815-OMcp24_215*ROcp24_715);
    RLcp24_117 = ROcp24_715*s.dpt(3,24);
    RLcp24_217 = ROcp24_815*s.dpt(3,24);
    RLcp24_317 = ROcp24_915*s.dpt(3,24);
    OMcp24_117 = OMcp24_116+ROcp24_416*qd(17);
    OMcp24_217 = OMcp24_216+ROcp24_516*qd(17);
    OMcp24_317 = OMcp24_316+ROcp24_616*qd(17);
    ORcp24_117 = OMcp24_216*RLcp24_317-OMcp24_316*RLcp24_217;
    ORcp24_217 = -(OMcp24_116*RLcp24_317-OMcp24_316*RLcp24_117);
    ORcp24_317 = OMcp24_116*RLcp24_217-OMcp24_216*RLcp24_117;
    OPcp24_117 = OPcp24_116+ROcp24_416*qdd(17)+qd(17)*(OMcp24_216*ROcp24_616-OMcp24_316*ROcp24_516);
    OPcp24_217 = OPcp24_216+ROcp24_516*qdd(17)-qd(17)*(OMcp24_116*ROcp24_616-OMcp24_316*ROcp24_416);
    OPcp24_317 = OPcp24_316+ROcp24_616*qdd(17)+qd(17)*(OMcp24_116*ROcp24_516-OMcp24_216*ROcp24_416);
    RLcp24_156 = ROcp24_117*s.dpt(1,27)+ROcp24_416*s.dpt(2,27)+ROcp24_717*s.dpt(3,27);
    RLcp24_256 = ROcp24_217*s.dpt(1,27)+ROcp24_516*s.dpt(2,27)+ROcp24_817*s.dpt(3,27);
    RLcp24_356 = ROcp24_317*s.dpt(1,27)+ROcp24_616*s.dpt(2,27)+ROcp24_917*s.dpt(3,27);
    POcp24_156 = RLcp24_114+RLcp24_115+RLcp24_116+RLcp24_117+RLcp24_156+q(1);
    POcp24_256 = RLcp24_214+RLcp24_215+RLcp24_216+RLcp24_217+RLcp24_256+q(2);
    POcp24_356 = RLcp24_314+RLcp24_315+RLcp24_316+RLcp24_317+RLcp24_356+q(3);
    JTcp24_256_4 = -(RLcp24_314+RLcp24_315+RLcp24_316+RLcp24_317+RLcp24_356);
    JTcp24_356_4 = RLcp24_214+RLcp24_215+RLcp24_216+RLcp24_217+RLcp24_256;
    JTcp24_156_5 = C4*(RLcp24_314+RLcp24_315+RLcp24_316+RLcp24_317)-S4*(RLcp24_214+RLcp24_215)-S4*(RLcp24_216+RLcp24_217)-RLcp24_256*S4+RLcp24_356*...
 C4;
    JTcp24_256_5 = S4*(RLcp24_114+RLcp24_115+RLcp24_116+RLcp24_117+RLcp24_156);
    JTcp24_356_5 = -C4*(RLcp24_114+RLcp24_115+RLcp24_116+RLcp24_117+RLcp24_156);
    JTcp24_156_6 = ROcp24_85*(RLcp24_314+RLcp24_315+RLcp24_316+RLcp24_317)-ROcp24_95*(RLcp24_214+RLcp24_215)-ROcp24_95*(RLcp24_216+RLcp24_217)-...
 RLcp24_256*ROcp24_95+RLcp24_356*ROcp24_85;
    JTcp24_256_6 = -(RLcp24_356*S5-ROcp24_95*(RLcp24_114+RLcp24_115+RLcp24_116+RLcp24_117+RLcp24_156)+S5*(RLcp24_314+RLcp24_315)+S5*(RLcp24_316+...
 RLcp24_317));
    JTcp24_356_6 = RLcp24_256*S5-ROcp24_85*(RLcp24_114+RLcp24_115+RLcp24_116+RLcp24_117+RLcp24_156)+S5*(RLcp24_214+RLcp24_215)+S5*(RLcp24_216+...
 RLcp24_217);
    JTcp24_156_7 = ROcp24_56*(RLcp24_315+RLcp24_316+RLcp24_317+RLcp24_356)-ROcp24_66*(RLcp24_215+RLcp24_216)-ROcp24_66*(RLcp24_217+RLcp24_256);
    JTcp24_256_7 = -(ROcp24_46*(RLcp24_315+RLcp24_316+RLcp24_317+RLcp24_356)-ROcp24_66*(RLcp24_115+RLcp24_116)-ROcp24_66*(RLcp24_117+RLcp24_156));
    JTcp24_356_7 = ROcp24_46*(RLcp24_215+RLcp24_216+RLcp24_217+RLcp24_256)-ROcp24_56*(RLcp24_115+RLcp24_116)-ROcp24_56*(RLcp24_117+RLcp24_156);
    JTcp24_156_8 = ROcp24_214*(RLcp24_316+RLcp24_317)-ROcp24_314*(RLcp24_216+RLcp24_217)-RLcp24_256*ROcp24_314+RLcp24_356*ROcp24_214;
    JTcp24_256_8 = RLcp24_156*ROcp24_314-RLcp24_356*ROcp24_114-ROcp24_114*(RLcp24_316+RLcp24_317)+ROcp24_314*(RLcp24_116+RLcp24_117);
    JTcp24_356_8 = ROcp24_114*(RLcp24_216+RLcp24_217)-ROcp24_214*(RLcp24_116+RLcp24_117)-RLcp24_156*ROcp24_214+RLcp24_256*ROcp24_114;
    JTcp24_156_9 = ROcp24_815*(RLcp24_317+RLcp24_356)-ROcp24_915*(RLcp24_217+RLcp24_256);
    JTcp24_256_9 = -(ROcp24_715*(RLcp24_317+RLcp24_356)-ROcp24_915*(RLcp24_117+RLcp24_156));
    JTcp24_356_9 = ROcp24_715*(RLcp24_217+RLcp24_256)-ROcp24_815*(RLcp24_117+RLcp24_156);
    JTcp24_156_10 = -(RLcp24_256*ROcp24_616-RLcp24_356*ROcp24_516);
    JTcp24_256_10 = RLcp24_156*ROcp24_616-RLcp24_356*ROcp24_416;
    JTcp24_356_10 = -(RLcp24_156*ROcp24_516-RLcp24_256*ROcp24_416);
    ORcp24_156 = OMcp24_217*RLcp24_356-OMcp24_317*RLcp24_256;
    ORcp24_256 = -(OMcp24_117*RLcp24_356-OMcp24_317*RLcp24_156);
    ORcp24_356 = OMcp24_117*RLcp24_256-OMcp24_217*RLcp24_156;
    VIcp24_156 = ORcp24_114+ORcp24_115+ORcp24_116+ORcp24_117+ORcp24_156+qd(1);
    VIcp24_256 = ORcp24_214+ORcp24_215+ORcp24_216+ORcp24_217+ORcp24_256+qd(2);
    VIcp24_356 = ORcp24_314+ORcp24_315+ORcp24_316+ORcp24_317+ORcp24_356+qd(3);
    ACcp24_156 = qdd(1)+OMcp24_214*ORcp24_315+OMcp24_215*ORcp24_316+OMcp24_216*ORcp24_317+OMcp24_217*ORcp24_356+OMcp24_26*ORcp24_314-OMcp24_314*...
 ORcp24_215-OMcp24_315*ORcp24_216-OMcp24_316*ORcp24_217-OMcp24_317*ORcp24_256-OMcp24_36*ORcp24_214+OPcp24_214*RLcp24_315+OPcp24_215*RLcp24_316+...
 OPcp24_216*RLcp24_317+OPcp24_217*RLcp24_356+OPcp24_26*RLcp24_314-OPcp24_314*RLcp24_215-OPcp24_315*RLcp24_216-OPcp24_316*RLcp24_217-OPcp24_317*...
 RLcp24_256-OPcp24_36*RLcp24_214;
    ACcp24_256 = qdd(2)-OMcp24_114*ORcp24_315-OMcp24_115*ORcp24_316-OMcp24_116*ORcp24_317-OMcp24_117*ORcp24_356-OMcp24_16*ORcp24_314+OMcp24_314*...
 ORcp24_115+OMcp24_315*ORcp24_116+OMcp24_316*ORcp24_117+OMcp24_317*ORcp24_156+OMcp24_36*ORcp24_114-OPcp24_114*RLcp24_315-OPcp24_115*RLcp24_316-...
 OPcp24_116*RLcp24_317-OPcp24_117*RLcp24_356-OPcp24_16*RLcp24_314+OPcp24_314*RLcp24_115+OPcp24_315*RLcp24_116+OPcp24_316*RLcp24_117+OPcp24_317*...
 RLcp24_156+OPcp24_36*RLcp24_114;
    ACcp24_356 = qdd(3)+OMcp24_114*ORcp24_215+OMcp24_115*ORcp24_216+OMcp24_116*ORcp24_217+OMcp24_117*ORcp24_256+OMcp24_16*ORcp24_214-OMcp24_214*...
 ORcp24_115-OMcp24_215*ORcp24_116-OMcp24_216*ORcp24_117-OMcp24_217*ORcp24_156-OMcp24_26*ORcp24_114+OPcp24_114*RLcp24_215+OPcp24_115*RLcp24_216+...
 OPcp24_116*RLcp24_217+OPcp24_117*RLcp24_256+OPcp24_16*RLcp24_214-OPcp24_214*RLcp24_115-OPcp24_215*RLcp24_116-OPcp24_216*RLcp24_117-OPcp24_217*...
 RLcp24_156-OPcp24_26*RLcp24_114;

% = = Block_1_0_0_25_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp24_156;
    sens.P(2) = POcp24_256;
    sens.P(3) = POcp24_356;
    sens.R(1,1) = ROcp24_117;
    sens.R(1,2) = ROcp24_217;
    sens.R(1,3) = ROcp24_317;
    sens.R(2,1) = ROcp24_416;
    sens.R(2,2) = ROcp24_516;
    sens.R(2,3) = ROcp24_616;
    sens.R(3,1) = ROcp24_717;
    sens.R(3,2) = ROcp24_817;
    sens.R(3,3) = ROcp24_917;
    sens.V(1) = VIcp24_156;
    sens.V(2) = VIcp24_256;
    sens.V(3) = VIcp24_356;
    sens.OM(1) = OMcp24_117;
    sens.OM(2) = OMcp24_217;
    sens.OM(3) = OMcp24_317;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp24_156_5;
    sens.J(1,6) = JTcp24_156_6;
    sens.J(1,14) = JTcp24_156_7;
    sens.J(1,15) = JTcp24_156_8;
    sens.J(1,16) = JTcp24_156_9;
    sens.J(1,17) = JTcp24_156_10;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp24_256_4;
    sens.J(2,5) = JTcp24_256_5;
    sens.J(2,6) = JTcp24_256_6;
    sens.J(2,14) = JTcp24_256_7;
    sens.J(2,15) = JTcp24_256_8;
    sens.J(2,16) = JTcp24_256_9;
    sens.J(2,17) = JTcp24_256_10;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp24_356_4;
    sens.J(3,5) = JTcp24_356_5;
    sens.J(3,6) = JTcp24_356_6;
    sens.J(3,14) = JTcp24_356_7;
    sens.J(3,15) = JTcp24_356_8;
    sens.J(3,16) = JTcp24_356_9;
    sens.J(3,17) = JTcp24_356_10;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,14) = ROcp24_46;
    sens.J(4,15) = ROcp24_114;
    sens.J(4,16) = ROcp24_715;
    sens.J(4,17) = ROcp24_416;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp24_85;
    sens.J(5,14) = ROcp24_56;
    sens.J(5,15) = ROcp24_214;
    sens.J(5,16) = ROcp24_815;
    sens.J(5,17) = ROcp24_516;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp24_95;
    sens.J(6,14) = ROcp24_66;
    sens.J(6,15) = ROcp24_314;
    sens.J(6,16) = ROcp24_915;
    sens.J(6,17) = ROcp24_616;
    sens.A(1) = ACcp24_156;
    sens.A(2) = ACcp24_256;
    sens.A(3) = ACcp24_356;
    sens.OMP(1) = OPcp24_117;
    sens.OMP(2) = OPcp24_217;
    sens.OMP(3) = OPcp24_317;
 
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

% = = Block_1_0_0_26_0_3 = = 
 
% Sensor Kinematics 


    ROcp25_114 = ROcp25_16*C14-S14*S5;
    ROcp25_214 = ROcp25_26*C14-ROcp25_85*S14;
    ROcp25_314 = ROcp25_36*C14-ROcp25_95*S14;
    ROcp25_714 = ROcp25_16*S14+C14*S5;
    ROcp25_814 = ROcp25_26*S14+ROcp25_85*C14;
    ROcp25_914 = ROcp25_36*S14+ROcp25_95*C14;
    ROcp25_415 = ROcp25_46*C15+ROcp25_714*S15;
    ROcp25_515 = ROcp25_56*C15+ROcp25_814*S15;
    ROcp25_615 = ROcp25_66*C15+ROcp25_914*S15;
    ROcp25_715 = -(ROcp25_46*S15-ROcp25_714*C15);
    ROcp25_815 = -(ROcp25_56*S15-ROcp25_814*C15);
    ROcp25_915 = -(ROcp25_66*S15-ROcp25_914*C15);
    ROcp25_116 = ROcp25_114*C16+ROcp25_415*S16;
    ROcp25_216 = ROcp25_214*C16+ROcp25_515*S16;
    ROcp25_316 = ROcp25_314*C16+ROcp25_615*S16;
    ROcp25_416 = -(ROcp25_114*S16-ROcp25_415*C16);
    ROcp25_516 = -(ROcp25_214*S16-ROcp25_515*C16);
    ROcp25_616 = -(ROcp25_314*S16-ROcp25_615*C16);
    ROcp25_117 = ROcp25_116*C17-ROcp25_715*S17;
    ROcp25_217 = ROcp25_216*C17-ROcp25_815*S17;
    ROcp25_317 = ROcp25_316*C17-ROcp25_915*S17;
    ROcp25_717 = ROcp25_116*S17+ROcp25_715*C17;
    ROcp25_817 = ROcp25_216*S17+ROcp25_815*C17;
    ROcp25_917 = ROcp25_316*S17+ROcp25_915*C17;
    ROcp25_418 = ROcp25_416*C18+ROcp25_717*S18;
    ROcp25_518 = ROcp25_516*C18+ROcp25_817*S18;
    ROcp25_618 = ROcp25_616*C18+ROcp25_917*S18;
    ROcp25_718 = -(ROcp25_416*S18-ROcp25_717*C18);
    ROcp25_818 = -(ROcp25_516*S18-ROcp25_817*C18);
    ROcp25_918 = -(ROcp25_616*S18-ROcp25_917*C18);
    RLcp25_114 = ROcp25_46*s.dpt(2,2);
    RLcp25_214 = ROcp25_56*s.dpt(2,2);
    RLcp25_314 = ROcp25_66*s.dpt(2,2);
    OMcp25_114 = OMcp25_16+ROcp25_46*qd(14);
    OMcp25_214 = OMcp25_26+ROcp25_56*qd(14);
    OMcp25_314 = OMcp25_36+ROcp25_66*qd(14);
    ORcp25_114 = OMcp25_26*RLcp25_314-OMcp25_36*RLcp25_214;
    ORcp25_214 = -(OMcp25_16*RLcp25_314-OMcp25_36*RLcp25_114);
    ORcp25_314 = OMcp25_16*RLcp25_214-OMcp25_26*RLcp25_114;
    OPcp25_114 = OPcp25_16+ROcp25_46*qdd(14)+qd(14)*(OMcp25_26*ROcp25_66-OMcp25_36*ROcp25_56);
    OPcp25_214 = OPcp25_26+ROcp25_56*qdd(14)-qd(14)*(OMcp25_16*ROcp25_66-OMcp25_36*ROcp25_46);
    OPcp25_314 = OPcp25_36+ROcp25_66*qdd(14)+qd(14)*(OMcp25_16*ROcp25_56-OMcp25_26*ROcp25_46);
    RLcp25_115 = ROcp25_46*s.dpt(2,20);
    RLcp25_215 = ROcp25_56*s.dpt(2,20);
    RLcp25_315 = ROcp25_66*s.dpt(2,20);
    OMcp25_115 = OMcp25_114+ROcp25_114*qd(15);
    OMcp25_215 = OMcp25_214+ROcp25_214*qd(15);
    OMcp25_315 = OMcp25_314+ROcp25_314*qd(15);
    ORcp25_115 = OMcp25_214*RLcp25_315-OMcp25_314*RLcp25_215;
    ORcp25_215 = -(OMcp25_114*RLcp25_315-OMcp25_314*RLcp25_115);
    ORcp25_315 = OMcp25_114*RLcp25_215-OMcp25_214*RLcp25_115;
    OPcp25_115 = OPcp25_114+ROcp25_114*qdd(15)+qd(15)*(OMcp25_214*ROcp25_314-OMcp25_314*ROcp25_214);
    OPcp25_215 = OPcp25_214+ROcp25_214*qdd(15)-qd(15)*(OMcp25_114*ROcp25_314-OMcp25_314*ROcp25_114);
    OPcp25_315 = OPcp25_314+ROcp25_314*qdd(15)+qd(15)*(OMcp25_114*ROcp25_214-OMcp25_214*ROcp25_114);
    RLcp25_116 = ROcp25_715*s.dpt(3,22);
    RLcp25_216 = ROcp25_815*s.dpt(3,22);
    RLcp25_316 = ROcp25_915*s.dpt(3,22);
    OMcp25_116 = OMcp25_115+ROcp25_715*qd(16);
    OMcp25_216 = OMcp25_215+ROcp25_815*qd(16);
    OMcp25_316 = OMcp25_315+ROcp25_915*qd(16);
    ORcp25_116 = OMcp25_215*RLcp25_316-OMcp25_315*RLcp25_216;
    ORcp25_216 = -(OMcp25_115*RLcp25_316-OMcp25_315*RLcp25_116);
    ORcp25_316 = OMcp25_115*RLcp25_216-OMcp25_215*RLcp25_116;
    OPcp25_116 = OPcp25_115+ROcp25_715*qdd(16)+qd(16)*(OMcp25_215*ROcp25_915-OMcp25_315*ROcp25_815);
    OPcp25_216 = OPcp25_215+ROcp25_815*qdd(16)-qd(16)*(OMcp25_115*ROcp25_915-OMcp25_315*ROcp25_715);
    OPcp25_316 = OPcp25_315+ROcp25_915*qdd(16)+qd(16)*(OMcp25_115*ROcp25_815-OMcp25_215*ROcp25_715);
    RLcp25_117 = ROcp25_715*s.dpt(3,24);
    RLcp25_217 = ROcp25_815*s.dpt(3,24);
    RLcp25_317 = ROcp25_915*s.dpt(3,24);
    OMcp25_117 = OMcp25_116+ROcp25_416*qd(17);
    OMcp25_217 = OMcp25_216+ROcp25_516*qd(17);
    OMcp25_317 = OMcp25_316+ROcp25_616*qd(17);
    ORcp25_117 = OMcp25_216*RLcp25_317-OMcp25_316*RLcp25_217;
    ORcp25_217 = -(OMcp25_116*RLcp25_317-OMcp25_316*RLcp25_117);
    ORcp25_317 = OMcp25_116*RLcp25_217-OMcp25_216*RLcp25_117;
    OPcp25_117 = OPcp25_116+ROcp25_416*qdd(17)+qd(17)*(OMcp25_216*ROcp25_616-OMcp25_316*ROcp25_516);
    OPcp25_217 = OPcp25_216+ROcp25_516*qdd(17)-qd(17)*(OMcp25_116*ROcp25_616-OMcp25_316*ROcp25_416);
    OPcp25_317 = OPcp25_316+ROcp25_616*qdd(17)+qd(17)*(OMcp25_116*ROcp25_516-OMcp25_216*ROcp25_416);
    RLcp25_118 = ROcp25_717*s.dpt(3,26);
    RLcp25_218 = ROcp25_817*s.dpt(3,26);
    RLcp25_318 = ROcp25_917*s.dpt(3,26);
    OMcp25_118 = OMcp25_117+ROcp25_117*qd(18);
    OMcp25_218 = OMcp25_217+ROcp25_217*qd(18);
    OMcp25_318 = OMcp25_317+ROcp25_317*qd(18);
    ORcp25_118 = OMcp25_217*RLcp25_318-OMcp25_317*RLcp25_218;
    ORcp25_218 = -(OMcp25_117*RLcp25_318-OMcp25_317*RLcp25_118);
    ORcp25_318 = OMcp25_117*RLcp25_218-OMcp25_217*RLcp25_118;
    OPcp25_118 = OPcp25_117+ROcp25_117*qdd(18)+qd(18)*(OMcp25_217*ROcp25_317-OMcp25_317*ROcp25_217);
    OPcp25_218 = OPcp25_217+ROcp25_217*qdd(18)-qd(18)*(OMcp25_117*ROcp25_317-OMcp25_317*ROcp25_117);
    OPcp25_318 = OPcp25_317+ROcp25_317*qdd(18)+qd(18)*(OMcp25_117*ROcp25_217-OMcp25_217*ROcp25_117);
    RLcp25_157 = ROcp25_117*s.dpt(1,29)+ROcp25_418*s.dpt(2,29)+ROcp25_718*s.dpt(3,29);
    RLcp25_257 = ROcp25_217*s.dpt(1,29)+ROcp25_518*s.dpt(2,29)+ROcp25_818*s.dpt(3,29);
    RLcp25_357 = ROcp25_317*s.dpt(1,29)+ROcp25_618*s.dpt(2,29)+ROcp25_918*s.dpt(3,29);
    POcp25_157 = RLcp25_114+RLcp25_115+RLcp25_116+RLcp25_117+RLcp25_118+RLcp25_157+q(1);
    POcp25_257 = RLcp25_214+RLcp25_215+RLcp25_216+RLcp25_217+RLcp25_218+RLcp25_257+q(2);
    POcp25_357 = RLcp25_314+RLcp25_315+RLcp25_316+RLcp25_317+RLcp25_318+RLcp25_357+q(3);
    JTcp25_257_4 = -(RLcp25_314+RLcp25_315+RLcp25_316+RLcp25_317+RLcp25_318+RLcp25_357);
    JTcp25_357_4 = RLcp25_214+RLcp25_215+RLcp25_216+RLcp25_217+RLcp25_218+RLcp25_257;
    JTcp25_157_5 = C4*(RLcp25_314+RLcp25_315+RLcp25_316+RLcp25_317+RLcp25_318+RLcp25_357)-S4*(RLcp25_214+RLcp25_215)-S4*(RLcp25_216+RLcp25_217)-S4*...
 (RLcp25_218+RLcp25_257);
    JTcp25_257_5 = S4*(RLcp25_114+RLcp25_115+RLcp25_116+RLcp25_117+RLcp25_118+RLcp25_157);
    JTcp25_357_5 = -C4*(RLcp25_114+RLcp25_115+RLcp25_116+RLcp25_117+RLcp25_118+RLcp25_157);
    JTcp25_157_6 = ROcp25_85*(RLcp25_314+RLcp25_315+RLcp25_316+RLcp25_317+RLcp25_318+RLcp25_357)-ROcp25_95*(RLcp25_214+RLcp25_215)-ROcp25_95*(...
 RLcp25_216+RLcp25_217)-ROcp25_95*(RLcp25_218+RLcp25_257);
    JTcp25_257_6 = RLcp25_157*ROcp25_95-RLcp25_318*S5-RLcp25_357*S5+ROcp25_95*(RLcp25_114+RLcp25_115+RLcp25_116+RLcp25_117+RLcp25_118)-S5*(...
 RLcp25_314+RLcp25_315)-S5*(RLcp25_316+RLcp25_317);
    JTcp25_357_6 = RLcp25_218*S5-ROcp25_85*(RLcp25_114+RLcp25_115+RLcp25_116+RLcp25_117+RLcp25_118)+S5*(RLcp25_214+RLcp25_215)+S5*(RLcp25_216+...
 RLcp25_217)-RLcp25_157*ROcp25_85+RLcp25_257*S5;
    JTcp25_157_7 = ROcp25_56*(RLcp25_315+RLcp25_316+RLcp25_317+RLcp25_318)-ROcp25_66*(RLcp25_215+RLcp25_216)-ROcp25_66*(RLcp25_217+RLcp25_218)-...
 RLcp25_257*ROcp25_66+RLcp25_357*ROcp25_56;
    JTcp25_257_7 = RLcp25_157*ROcp25_66-RLcp25_357*ROcp25_46-ROcp25_46*(RLcp25_315+RLcp25_316+RLcp25_317+RLcp25_318)+ROcp25_66*(RLcp25_115+...
 RLcp25_116)+ROcp25_66*(RLcp25_117+RLcp25_118);
    JTcp25_357_7 = ROcp25_46*(RLcp25_215+RLcp25_216+RLcp25_217+RLcp25_218)-ROcp25_56*(RLcp25_115+RLcp25_116)-ROcp25_56*(RLcp25_117+RLcp25_118)-...
 RLcp25_157*ROcp25_56+RLcp25_257*ROcp25_46;
    JTcp25_157_8 = ROcp25_214*(RLcp25_316+RLcp25_317+RLcp25_318+RLcp25_357)-ROcp25_314*(RLcp25_216+RLcp25_217)-ROcp25_314*(RLcp25_218+RLcp25_257);
    JTcp25_257_8 = -(ROcp25_114*(RLcp25_316+RLcp25_317+RLcp25_318+RLcp25_357)-ROcp25_314*(RLcp25_116+RLcp25_117)-ROcp25_314*(RLcp25_118+RLcp25_157)...
 );
    JTcp25_357_8 = ROcp25_114*(RLcp25_216+RLcp25_217+RLcp25_218+RLcp25_257)-ROcp25_214*(RLcp25_116+RLcp25_117)-ROcp25_214*(RLcp25_118+RLcp25_157);
    JTcp25_157_9 = ROcp25_815*(RLcp25_317+RLcp25_318)-ROcp25_915*(RLcp25_217+RLcp25_218)-RLcp25_257*ROcp25_915+RLcp25_357*ROcp25_815;
    JTcp25_257_9 = RLcp25_157*ROcp25_915-RLcp25_357*ROcp25_715-ROcp25_715*(RLcp25_317+RLcp25_318)+ROcp25_915*(RLcp25_117+RLcp25_118);
    JTcp25_357_9 = ROcp25_715*(RLcp25_217+RLcp25_218)-ROcp25_815*(RLcp25_117+RLcp25_118)-RLcp25_157*ROcp25_815+RLcp25_257*ROcp25_715;
    JTcp25_157_10 = ROcp25_516*(RLcp25_318+RLcp25_357)-ROcp25_616*(RLcp25_218+RLcp25_257);
    JTcp25_257_10 = -(ROcp25_416*(RLcp25_318+RLcp25_357)-ROcp25_616*(RLcp25_118+RLcp25_157));
    JTcp25_357_10 = ROcp25_416*(RLcp25_218+RLcp25_257)-ROcp25_516*(RLcp25_118+RLcp25_157);
    JTcp25_157_11 = -(RLcp25_257*ROcp25_317-RLcp25_357*ROcp25_217);
    JTcp25_257_11 = RLcp25_157*ROcp25_317-RLcp25_357*ROcp25_117;
    JTcp25_357_11 = -(RLcp25_157*ROcp25_217-RLcp25_257*ROcp25_117);
    ORcp25_157 = OMcp25_218*RLcp25_357-OMcp25_318*RLcp25_257;
    ORcp25_257 = -(OMcp25_118*RLcp25_357-OMcp25_318*RLcp25_157);
    ORcp25_357 = OMcp25_118*RLcp25_257-OMcp25_218*RLcp25_157;
    VIcp25_157 = ORcp25_114+ORcp25_115+ORcp25_116+ORcp25_117+ORcp25_118+ORcp25_157+qd(1);
    VIcp25_257 = ORcp25_214+ORcp25_215+ORcp25_216+ORcp25_217+ORcp25_218+ORcp25_257+qd(2);
    VIcp25_357 = ORcp25_314+ORcp25_315+ORcp25_316+ORcp25_317+ORcp25_318+ORcp25_357+qd(3);
    ACcp25_157 = qdd(1)+OMcp25_214*ORcp25_315+OMcp25_215*ORcp25_316+OMcp25_216*ORcp25_317+OMcp25_217*ORcp25_318+OMcp25_218*ORcp25_357+OMcp25_26*...
 ORcp25_314-OMcp25_314*ORcp25_215-OMcp25_315*ORcp25_216-OMcp25_316*ORcp25_217-OMcp25_317*ORcp25_218-OMcp25_318*ORcp25_257-OMcp25_36*ORcp25_214+...
 OPcp25_214*RLcp25_315+OPcp25_215*RLcp25_316+OPcp25_216*RLcp25_317+OPcp25_217*RLcp25_318+OPcp25_218*RLcp25_357+OPcp25_26*RLcp25_314-OPcp25_314*...
 RLcp25_215-OPcp25_315*RLcp25_216-OPcp25_316*RLcp25_217-OPcp25_317*RLcp25_218-OPcp25_318*RLcp25_257-OPcp25_36*RLcp25_214;
    ACcp25_257 = qdd(2)-OMcp25_114*ORcp25_315-OMcp25_115*ORcp25_316-OMcp25_116*ORcp25_317-OMcp25_117*ORcp25_318-OMcp25_118*ORcp25_357-OMcp25_16*...
 ORcp25_314+OMcp25_314*ORcp25_115+OMcp25_315*ORcp25_116+OMcp25_316*ORcp25_117+OMcp25_317*ORcp25_118+OMcp25_318*ORcp25_157+OMcp25_36*ORcp25_114-...
 OPcp25_114*RLcp25_315-OPcp25_115*RLcp25_316-OPcp25_116*RLcp25_317-OPcp25_117*RLcp25_318-OPcp25_118*RLcp25_357-OPcp25_16*RLcp25_314+OPcp25_314*...
 RLcp25_115+OPcp25_315*RLcp25_116+OPcp25_316*RLcp25_117+OPcp25_317*RLcp25_118+OPcp25_318*RLcp25_157+OPcp25_36*RLcp25_114;
    ACcp25_357 = qdd(3)+OMcp25_114*ORcp25_215+OMcp25_115*ORcp25_216+OMcp25_116*ORcp25_217+OMcp25_117*ORcp25_218+OMcp25_118*ORcp25_257+OMcp25_16*...
 ORcp25_214-OMcp25_214*ORcp25_115-OMcp25_215*ORcp25_116-OMcp25_216*ORcp25_117-OMcp25_217*ORcp25_118-OMcp25_218*ORcp25_157-OMcp25_26*ORcp25_114+...
 OPcp25_114*RLcp25_215+OPcp25_115*RLcp25_216+OPcp25_116*RLcp25_217+OPcp25_117*RLcp25_218+OPcp25_118*RLcp25_257+OPcp25_16*RLcp25_214-OPcp25_214*...
 RLcp25_115-OPcp25_215*RLcp25_116-OPcp25_216*RLcp25_117-OPcp25_217*RLcp25_118-OPcp25_218*RLcp25_157-OPcp25_26*RLcp25_114;

% = = Block_1_0_0_26_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp25_157;
    sens.P(2) = POcp25_257;
    sens.P(3) = POcp25_357;
    sens.R(1,1) = ROcp25_117;
    sens.R(1,2) = ROcp25_217;
    sens.R(1,3) = ROcp25_317;
    sens.R(2,1) = ROcp25_418;
    sens.R(2,2) = ROcp25_518;
    sens.R(2,3) = ROcp25_618;
    sens.R(3,1) = ROcp25_718;
    sens.R(3,2) = ROcp25_818;
    sens.R(3,3) = ROcp25_918;
    sens.V(1) = VIcp25_157;
    sens.V(2) = VIcp25_257;
    sens.V(3) = VIcp25_357;
    sens.OM(1) = OMcp25_118;
    sens.OM(2) = OMcp25_218;
    sens.OM(3) = OMcp25_318;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp25_157_5;
    sens.J(1,6) = JTcp25_157_6;
    sens.J(1,14) = JTcp25_157_7;
    sens.J(1,15) = JTcp25_157_8;
    sens.J(1,16) = JTcp25_157_9;
    sens.J(1,17) = JTcp25_157_10;
    sens.J(1,18) = JTcp25_157_11;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp25_257_4;
    sens.J(2,5) = JTcp25_257_5;
    sens.J(2,6) = JTcp25_257_6;
    sens.J(2,14) = JTcp25_257_7;
    sens.J(2,15) = JTcp25_257_8;
    sens.J(2,16) = JTcp25_257_9;
    sens.J(2,17) = JTcp25_257_10;
    sens.J(2,18) = JTcp25_257_11;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp25_357_4;
    sens.J(3,5) = JTcp25_357_5;
    sens.J(3,6) = JTcp25_357_6;
    sens.J(3,14) = JTcp25_357_7;
    sens.J(3,15) = JTcp25_357_8;
    sens.J(3,16) = JTcp25_357_9;
    sens.J(3,17) = JTcp25_357_10;
    sens.J(3,18) = JTcp25_357_11;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,14) = ROcp25_46;
    sens.J(4,15) = ROcp25_114;
    sens.J(4,16) = ROcp25_715;
    sens.J(4,17) = ROcp25_416;
    sens.J(4,18) = ROcp25_117;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp25_85;
    sens.J(5,14) = ROcp25_56;
    sens.J(5,15) = ROcp25_214;
    sens.J(5,16) = ROcp25_815;
    sens.J(5,17) = ROcp25_516;
    sens.J(5,18) = ROcp25_217;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp25_95;
    sens.J(6,14) = ROcp25_66;
    sens.J(6,15) = ROcp25_314;
    sens.J(6,16) = ROcp25_915;
    sens.J(6,17) = ROcp25_616;
    sens.J(6,18) = ROcp25_317;
    sens.A(1) = ACcp25_157;
    sens.A(2) = ACcp25_257;
    sens.A(3) = ACcp25_357;
    sens.OMP(1) = OPcp25_118;
    sens.OMP(2) = OPcp25_218;
    sens.OMP(3) = OPcp25_318;
 
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

% = = Block_1_0_0_27_0_3 = = 
 
% Sensor Kinematics 


    ROcp26_114 = ROcp26_16*C14-S14*S5;
    ROcp26_214 = ROcp26_26*C14-ROcp26_85*S14;
    ROcp26_314 = ROcp26_36*C14-ROcp26_95*S14;
    ROcp26_714 = ROcp26_16*S14+C14*S5;
    ROcp26_814 = ROcp26_26*S14+ROcp26_85*C14;
    ROcp26_914 = ROcp26_36*S14+ROcp26_95*C14;
    ROcp26_415 = ROcp26_46*C15+ROcp26_714*S15;
    ROcp26_515 = ROcp26_56*C15+ROcp26_814*S15;
    ROcp26_615 = ROcp26_66*C15+ROcp26_914*S15;
    ROcp26_715 = -(ROcp26_46*S15-ROcp26_714*C15);
    ROcp26_815 = -(ROcp26_56*S15-ROcp26_814*C15);
    ROcp26_915 = -(ROcp26_66*S15-ROcp26_914*C15);
    ROcp26_116 = ROcp26_114*C16+ROcp26_415*S16;
    ROcp26_216 = ROcp26_214*C16+ROcp26_515*S16;
    ROcp26_316 = ROcp26_314*C16+ROcp26_615*S16;
    ROcp26_416 = -(ROcp26_114*S16-ROcp26_415*C16);
    ROcp26_516 = -(ROcp26_214*S16-ROcp26_515*C16);
    ROcp26_616 = -(ROcp26_314*S16-ROcp26_615*C16);
    ROcp26_117 = ROcp26_116*C17-ROcp26_715*S17;
    ROcp26_217 = ROcp26_216*C17-ROcp26_815*S17;
    ROcp26_317 = ROcp26_316*C17-ROcp26_915*S17;
    ROcp26_717 = ROcp26_116*S17+ROcp26_715*C17;
    ROcp26_817 = ROcp26_216*S17+ROcp26_815*C17;
    ROcp26_917 = ROcp26_316*S17+ROcp26_915*C17;
    ROcp26_418 = ROcp26_416*C18+ROcp26_717*S18;
    ROcp26_518 = ROcp26_516*C18+ROcp26_817*S18;
    ROcp26_618 = ROcp26_616*C18+ROcp26_917*S18;
    ROcp26_718 = -(ROcp26_416*S18-ROcp26_717*C18);
    ROcp26_818 = -(ROcp26_516*S18-ROcp26_817*C18);
    ROcp26_918 = -(ROcp26_616*S18-ROcp26_917*C18);
    ROcp26_119 = ROcp26_117*C19-ROcp26_718*S19;
    ROcp26_219 = ROcp26_217*C19-ROcp26_818*S19;
    ROcp26_319 = ROcp26_317*C19-ROcp26_918*S19;
    ROcp26_719 = ROcp26_117*S19+ROcp26_718*C19;
    ROcp26_819 = ROcp26_217*S19+ROcp26_818*C19;
    ROcp26_919 = ROcp26_317*S19+ROcp26_918*C19;
    RLcp26_114 = ROcp26_46*s.dpt(2,2);
    RLcp26_214 = ROcp26_56*s.dpt(2,2);
    RLcp26_314 = ROcp26_66*s.dpt(2,2);
    OMcp26_114 = OMcp26_16+ROcp26_46*qd(14);
    OMcp26_214 = OMcp26_26+ROcp26_56*qd(14);
    OMcp26_314 = OMcp26_36+ROcp26_66*qd(14);
    ORcp26_114 = OMcp26_26*RLcp26_314-OMcp26_36*RLcp26_214;
    ORcp26_214 = -(OMcp26_16*RLcp26_314-OMcp26_36*RLcp26_114);
    ORcp26_314 = OMcp26_16*RLcp26_214-OMcp26_26*RLcp26_114;
    OPcp26_114 = OPcp26_16+ROcp26_46*qdd(14)+qd(14)*(OMcp26_26*ROcp26_66-OMcp26_36*ROcp26_56);
    OPcp26_214 = OPcp26_26+ROcp26_56*qdd(14)-qd(14)*(OMcp26_16*ROcp26_66-OMcp26_36*ROcp26_46);
    OPcp26_314 = OPcp26_36+ROcp26_66*qdd(14)+qd(14)*(OMcp26_16*ROcp26_56-OMcp26_26*ROcp26_46);
    RLcp26_115 = ROcp26_46*s.dpt(2,20);
    RLcp26_215 = ROcp26_56*s.dpt(2,20);
    RLcp26_315 = ROcp26_66*s.dpt(2,20);
    OMcp26_115 = OMcp26_114+ROcp26_114*qd(15);
    OMcp26_215 = OMcp26_214+ROcp26_214*qd(15);
    OMcp26_315 = OMcp26_314+ROcp26_314*qd(15);
    ORcp26_115 = OMcp26_214*RLcp26_315-OMcp26_314*RLcp26_215;
    ORcp26_215 = -(OMcp26_114*RLcp26_315-OMcp26_314*RLcp26_115);
    ORcp26_315 = OMcp26_114*RLcp26_215-OMcp26_214*RLcp26_115;
    OPcp26_115 = OPcp26_114+ROcp26_114*qdd(15)+qd(15)*(OMcp26_214*ROcp26_314-OMcp26_314*ROcp26_214);
    OPcp26_215 = OPcp26_214+ROcp26_214*qdd(15)-qd(15)*(OMcp26_114*ROcp26_314-OMcp26_314*ROcp26_114);
    OPcp26_315 = OPcp26_314+ROcp26_314*qdd(15)+qd(15)*(OMcp26_114*ROcp26_214-OMcp26_214*ROcp26_114);
    RLcp26_116 = ROcp26_715*s.dpt(3,22);
    RLcp26_216 = ROcp26_815*s.dpt(3,22);
    RLcp26_316 = ROcp26_915*s.dpt(3,22);
    OMcp26_116 = OMcp26_115+ROcp26_715*qd(16);
    OMcp26_216 = OMcp26_215+ROcp26_815*qd(16);
    OMcp26_316 = OMcp26_315+ROcp26_915*qd(16);
    ORcp26_116 = OMcp26_215*RLcp26_316-OMcp26_315*RLcp26_216;
    ORcp26_216 = -(OMcp26_115*RLcp26_316-OMcp26_315*RLcp26_116);
    ORcp26_316 = OMcp26_115*RLcp26_216-OMcp26_215*RLcp26_116;
    OPcp26_116 = OPcp26_115+ROcp26_715*qdd(16)+qd(16)*(OMcp26_215*ROcp26_915-OMcp26_315*ROcp26_815);
    OPcp26_216 = OPcp26_215+ROcp26_815*qdd(16)-qd(16)*(OMcp26_115*ROcp26_915-OMcp26_315*ROcp26_715);
    OPcp26_316 = OPcp26_315+ROcp26_915*qdd(16)+qd(16)*(OMcp26_115*ROcp26_815-OMcp26_215*ROcp26_715);
    RLcp26_117 = ROcp26_715*s.dpt(3,24);
    RLcp26_217 = ROcp26_815*s.dpt(3,24);
    RLcp26_317 = ROcp26_915*s.dpt(3,24);
    OMcp26_117 = OMcp26_116+ROcp26_416*qd(17);
    OMcp26_217 = OMcp26_216+ROcp26_516*qd(17);
    OMcp26_317 = OMcp26_316+ROcp26_616*qd(17);
    ORcp26_117 = OMcp26_216*RLcp26_317-OMcp26_316*RLcp26_217;
    ORcp26_217 = -(OMcp26_116*RLcp26_317-OMcp26_316*RLcp26_117);
    ORcp26_317 = OMcp26_116*RLcp26_217-OMcp26_216*RLcp26_117;
    OPcp26_117 = OPcp26_116+ROcp26_416*qdd(17)+qd(17)*(OMcp26_216*ROcp26_616-OMcp26_316*ROcp26_516);
    OPcp26_217 = OPcp26_216+ROcp26_516*qdd(17)-qd(17)*(OMcp26_116*ROcp26_616-OMcp26_316*ROcp26_416);
    OPcp26_317 = OPcp26_316+ROcp26_616*qdd(17)+qd(17)*(OMcp26_116*ROcp26_516-OMcp26_216*ROcp26_416);
    RLcp26_118 = ROcp26_717*s.dpt(3,26);
    RLcp26_218 = ROcp26_817*s.dpt(3,26);
    RLcp26_318 = ROcp26_917*s.dpt(3,26);
    OMcp26_118 = OMcp26_117+ROcp26_117*qd(18);
    OMcp26_218 = OMcp26_217+ROcp26_217*qd(18);
    OMcp26_318 = OMcp26_317+ROcp26_317*qd(18);
    ORcp26_118 = OMcp26_217*RLcp26_318-OMcp26_317*RLcp26_218;
    ORcp26_218 = -(OMcp26_117*RLcp26_318-OMcp26_317*RLcp26_118);
    ORcp26_318 = OMcp26_117*RLcp26_218-OMcp26_217*RLcp26_118;
    OMcp26_119 = OMcp26_118+ROcp26_418*qd(19);
    OMcp26_219 = OMcp26_218+ROcp26_518*qd(19);
    OMcp26_319 = OMcp26_318+ROcp26_618*qd(19);
    OPcp26_119 = OPcp26_117+ROcp26_117*qdd(18)+ROcp26_418*qdd(19)+qd(18)*(OMcp26_217*ROcp26_317-OMcp26_317*ROcp26_217)+qd(19)*(OMcp26_218*...
 ROcp26_618-OMcp26_318*ROcp26_518);
    OPcp26_219 = OPcp26_217+ROcp26_217*qdd(18)+ROcp26_518*qdd(19)-qd(18)*(OMcp26_117*ROcp26_317-OMcp26_317*ROcp26_117)-qd(19)*(OMcp26_118*...
 ROcp26_618-OMcp26_318*ROcp26_418);
    OPcp26_319 = OPcp26_317+ROcp26_317*qdd(18)+ROcp26_618*qdd(19)+qd(18)*(OMcp26_117*ROcp26_217-OMcp26_217*ROcp26_117)+qd(19)*(OMcp26_118*...
 ROcp26_518-OMcp26_218*ROcp26_418);
    RLcp26_158 = ROcp26_119*s.dpt(1,30)+ROcp26_418*s.dpt(2,30)+ROcp26_719*s.dpt(3,30);
    RLcp26_258 = ROcp26_219*s.dpt(1,30)+ROcp26_518*s.dpt(2,30)+ROcp26_819*s.dpt(3,30);
    RLcp26_358 = ROcp26_319*s.dpt(1,30)+ROcp26_618*s.dpt(2,30)+ROcp26_919*s.dpt(3,30);
    POcp26_158 = RLcp26_114+RLcp26_115+RLcp26_116+RLcp26_117+RLcp26_118+RLcp26_158+q(1);
    POcp26_258 = RLcp26_214+RLcp26_215+RLcp26_216+RLcp26_217+RLcp26_218+RLcp26_258+q(2);
    POcp26_358 = RLcp26_314+RLcp26_315+RLcp26_316+RLcp26_317+RLcp26_318+RLcp26_358+q(3);
    JTcp26_258_4 = -(RLcp26_314+RLcp26_315+RLcp26_316+RLcp26_317+RLcp26_318+RLcp26_358);
    JTcp26_358_4 = RLcp26_214+RLcp26_215+RLcp26_216+RLcp26_217+RLcp26_218+RLcp26_258;
    JTcp26_158_5 = C4*(RLcp26_314+RLcp26_315+RLcp26_316+RLcp26_317+RLcp26_318+RLcp26_358)-S4*(RLcp26_214+RLcp26_215)-S4*(RLcp26_216+RLcp26_217)-S4*...
 (RLcp26_218+RLcp26_258);
    JTcp26_258_5 = S4*(RLcp26_114+RLcp26_115+RLcp26_116+RLcp26_117+RLcp26_118+RLcp26_158);
    JTcp26_358_5 = -C4*(RLcp26_114+RLcp26_115+RLcp26_116+RLcp26_117+RLcp26_118+RLcp26_158);
    JTcp26_158_6 = ROcp26_85*(RLcp26_314+RLcp26_315+RLcp26_316+RLcp26_317+RLcp26_318+RLcp26_358)-ROcp26_95*(RLcp26_214+RLcp26_215)-ROcp26_95*(...
 RLcp26_216+RLcp26_217)-ROcp26_95*(RLcp26_218+RLcp26_258);
    JTcp26_258_6 = RLcp26_158*ROcp26_95-RLcp26_318*S5-RLcp26_358*S5+ROcp26_95*(RLcp26_114+RLcp26_115+RLcp26_116+RLcp26_117+RLcp26_118)-S5*(...
 RLcp26_314+RLcp26_315)-S5*(RLcp26_316+RLcp26_317);
    JTcp26_358_6 = RLcp26_218*S5-ROcp26_85*(RLcp26_114+RLcp26_115+RLcp26_116+RLcp26_117+RLcp26_118)+S5*(RLcp26_214+RLcp26_215)+S5*(RLcp26_216+...
 RLcp26_217)-RLcp26_158*ROcp26_85+RLcp26_258*S5;
    JTcp26_158_7 = ROcp26_56*(RLcp26_315+RLcp26_316+RLcp26_317+RLcp26_318)-ROcp26_66*(RLcp26_215+RLcp26_216)-ROcp26_66*(RLcp26_217+RLcp26_218)-...
 RLcp26_258*ROcp26_66+RLcp26_358*ROcp26_56;
    JTcp26_258_7 = RLcp26_158*ROcp26_66-RLcp26_358*ROcp26_46-ROcp26_46*(RLcp26_315+RLcp26_316+RLcp26_317+RLcp26_318)+ROcp26_66*(RLcp26_115+...
 RLcp26_116)+ROcp26_66*(RLcp26_117+RLcp26_118);
    JTcp26_358_7 = ROcp26_46*(RLcp26_215+RLcp26_216+RLcp26_217+RLcp26_218)-ROcp26_56*(RLcp26_115+RLcp26_116)-ROcp26_56*(RLcp26_117+RLcp26_118)-...
 RLcp26_158*ROcp26_56+RLcp26_258*ROcp26_46;
    JTcp26_158_8 = ROcp26_214*(RLcp26_316+RLcp26_317+RLcp26_318+RLcp26_358)-ROcp26_314*(RLcp26_216+RLcp26_217)-ROcp26_314*(RLcp26_218+RLcp26_258);
    JTcp26_258_8 = -(ROcp26_114*(RLcp26_316+RLcp26_317+RLcp26_318+RLcp26_358)-ROcp26_314*(RLcp26_116+RLcp26_117)-ROcp26_314*(RLcp26_118+RLcp26_158)...
 );
    JTcp26_358_8 = ROcp26_114*(RLcp26_216+RLcp26_217+RLcp26_218+RLcp26_258)-ROcp26_214*(RLcp26_116+RLcp26_117)-ROcp26_214*(RLcp26_118+RLcp26_158);
    JTcp26_158_9 = ROcp26_815*(RLcp26_317+RLcp26_318)-ROcp26_915*(RLcp26_217+RLcp26_218)-RLcp26_258*ROcp26_915+RLcp26_358*ROcp26_815;
    JTcp26_258_9 = RLcp26_158*ROcp26_915-RLcp26_358*ROcp26_715-ROcp26_715*(RLcp26_317+RLcp26_318)+ROcp26_915*(RLcp26_117+RLcp26_118);
    JTcp26_358_9 = ROcp26_715*(RLcp26_217+RLcp26_218)-ROcp26_815*(RLcp26_117+RLcp26_118)-RLcp26_158*ROcp26_815+RLcp26_258*ROcp26_715;
    JTcp26_158_10 = ROcp26_516*(RLcp26_318+RLcp26_358)-ROcp26_616*(RLcp26_218+RLcp26_258);
    JTcp26_258_10 = -(ROcp26_416*(RLcp26_318+RLcp26_358)-ROcp26_616*(RLcp26_118+RLcp26_158));
    JTcp26_358_10 = ROcp26_416*(RLcp26_218+RLcp26_258)-ROcp26_516*(RLcp26_118+RLcp26_158);
    JTcp26_158_11 = -(RLcp26_258*ROcp26_317-RLcp26_358*ROcp26_217);
    JTcp26_258_11 = RLcp26_158*ROcp26_317-RLcp26_358*ROcp26_117;
    JTcp26_358_11 = -(RLcp26_158*ROcp26_217-RLcp26_258*ROcp26_117);
    JTcp26_158_12 = -(RLcp26_258*ROcp26_618-RLcp26_358*ROcp26_518);
    JTcp26_258_12 = RLcp26_158*ROcp26_618-RLcp26_358*ROcp26_418;
    JTcp26_358_12 = -(RLcp26_158*ROcp26_518-RLcp26_258*ROcp26_418);
    ORcp26_158 = OMcp26_219*RLcp26_358-OMcp26_319*RLcp26_258;
    ORcp26_258 = -(OMcp26_119*RLcp26_358-OMcp26_319*RLcp26_158);
    ORcp26_358 = OMcp26_119*RLcp26_258-OMcp26_219*RLcp26_158;
    VIcp26_158 = ORcp26_114+ORcp26_115+ORcp26_116+ORcp26_117+ORcp26_118+ORcp26_158+qd(1);
    VIcp26_258 = ORcp26_214+ORcp26_215+ORcp26_216+ORcp26_217+ORcp26_218+ORcp26_258+qd(2);
    VIcp26_358 = ORcp26_314+ORcp26_315+ORcp26_316+ORcp26_317+ORcp26_318+ORcp26_358+qd(3);
    ACcp26_158 = qdd(1)+OMcp26_214*ORcp26_315+OMcp26_215*ORcp26_316+OMcp26_216*ORcp26_317+OMcp26_217*ORcp26_318+OMcp26_219*ORcp26_358+OMcp26_26*...
 ORcp26_314-OMcp26_314*ORcp26_215-OMcp26_315*ORcp26_216-OMcp26_316*ORcp26_217-OMcp26_317*ORcp26_218-OMcp26_319*ORcp26_258-OMcp26_36*ORcp26_214+...
 OPcp26_214*RLcp26_315+OPcp26_215*RLcp26_316+OPcp26_216*RLcp26_317+OPcp26_217*RLcp26_318+OPcp26_219*RLcp26_358+OPcp26_26*RLcp26_314-OPcp26_314*...
 RLcp26_215-OPcp26_315*RLcp26_216-OPcp26_316*RLcp26_217-OPcp26_317*RLcp26_218-OPcp26_319*RLcp26_258-OPcp26_36*RLcp26_214;
    ACcp26_258 = qdd(2)-OMcp26_114*ORcp26_315-OMcp26_115*ORcp26_316-OMcp26_116*ORcp26_317-OMcp26_117*ORcp26_318-OMcp26_119*ORcp26_358-OMcp26_16*...
 ORcp26_314+OMcp26_314*ORcp26_115+OMcp26_315*ORcp26_116+OMcp26_316*ORcp26_117+OMcp26_317*ORcp26_118+OMcp26_319*ORcp26_158+OMcp26_36*ORcp26_114-...
 OPcp26_114*RLcp26_315-OPcp26_115*RLcp26_316-OPcp26_116*RLcp26_317-OPcp26_117*RLcp26_318-OPcp26_119*RLcp26_358-OPcp26_16*RLcp26_314+OPcp26_314*...
 RLcp26_115+OPcp26_315*RLcp26_116+OPcp26_316*RLcp26_117+OPcp26_317*RLcp26_118+OPcp26_319*RLcp26_158+OPcp26_36*RLcp26_114;
    ACcp26_358 = qdd(3)+OMcp26_114*ORcp26_215+OMcp26_115*ORcp26_216+OMcp26_116*ORcp26_217+OMcp26_117*ORcp26_218+OMcp26_119*ORcp26_258+OMcp26_16*...
 ORcp26_214-OMcp26_214*ORcp26_115-OMcp26_215*ORcp26_116-OMcp26_216*ORcp26_117-OMcp26_217*ORcp26_118-OMcp26_219*ORcp26_158-OMcp26_26*ORcp26_114+...
 OPcp26_114*RLcp26_215+OPcp26_115*RLcp26_216+OPcp26_116*RLcp26_217+OPcp26_117*RLcp26_218+OPcp26_119*RLcp26_258+OPcp26_16*RLcp26_214-OPcp26_214*...
 RLcp26_115-OPcp26_215*RLcp26_116-OPcp26_216*RLcp26_117-OPcp26_217*RLcp26_118-OPcp26_219*RLcp26_158-OPcp26_26*RLcp26_114;

% = = Block_1_0_0_27_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp26_158;
    sens.P(2) = POcp26_258;
    sens.P(3) = POcp26_358;
    sens.R(1,1) = ROcp26_119;
    sens.R(1,2) = ROcp26_219;
    sens.R(1,3) = ROcp26_319;
    sens.R(2,1) = ROcp26_418;
    sens.R(2,2) = ROcp26_518;
    sens.R(2,3) = ROcp26_618;
    sens.R(3,1) = ROcp26_719;
    sens.R(3,2) = ROcp26_819;
    sens.R(3,3) = ROcp26_919;
    sens.V(1) = VIcp26_158;
    sens.V(2) = VIcp26_258;
    sens.V(3) = VIcp26_358;
    sens.OM(1) = OMcp26_119;
    sens.OM(2) = OMcp26_219;
    sens.OM(3) = OMcp26_319;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp26_158_5;
    sens.J(1,6) = JTcp26_158_6;
    sens.J(1,14) = JTcp26_158_7;
    sens.J(1,15) = JTcp26_158_8;
    sens.J(1,16) = JTcp26_158_9;
    sens.J(1,17) = JTcp26_158_10;
    sens.J(1,18) = JTcp26_158_11;
    sens.J(1,19) = JTcp26_158_12;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp26_258_4;
    sens.J(2,5) = JTcp26_258_5;
    sens.J(2,6) = JTcp26_258_6;
    sens.J(2,14) = JTcp26_258_7;
    sens.J(2,15) = JTcp26_258_8;
    sens.J(2,16) = JTcp26_258_9;
    sens.J(2,17) = JTcp26_258_10;
    sens.J(2,18) = JTcp26_258_11;
    sens.J(2,19) = JTcp26_258_12;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp26_358_4;
    sens.J(3,5) = JTcp26_358_5;
    sens.J(3,6) = JTcp26_358_6;
    sens.J(3,14) = JTcp26_358_7;
    sens.J(3,15) = JTcp26_358_8;
    sens.J(3,16) = JTcp26_358_9;
    sens.J(3,17) = JTcp26_358_10;
    sens.J(3,18) = JTcp26_358_11;
    sens.J(3,19) = JTcp26_358_12;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,14) = ROcp26_46;
    sens.J(4,15) = ROcp26_114;
    sens.J(4,16) = ROcp26_715;
    sens.J(4,17) = ROcp26_416;
    sens.J(4,18) = ROcp26_117;
    sens.J(4,19) = ROcp26_418;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp26_85;
    sens.J(5,14) = ROcp26_56;
    sens.J(5,15) = ROcp26_214;
    sens.J(5,16) = ROcp26_815;
    sens.J(5,17) = ROcp26_516;
    sens.J(5,18) = ROcp26_217;
    sens.J(5,19) = ROcp26_518;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp26_95;
    sens.J(6,14) = ROcp26_66;
    sens.J(6,15) = ROcp26_314;
    sens.J(6,16) = ROcp26_915;
    sens.J(6,17) = ROcp26_616;
    sens.J(6,18) = ROcp26_317;
    sens.J(6,19) = ROcp26_618;
    sens.A(1) = ACcp26_158;
    sens.A(2) = ACcp26_258;
    sens.A(3) = ACcp26_358;
    sens.OMP(1) = OPcp26_119;
    sens.OMP(2) = OPcp26_219;
    sens.OMP(3) = OPcp26_319;
 
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

% = = Block_1_0_0_28_0_3 = = 
 
% Sensor Kinematics 


    ROcp27_114 = ROcp27_16*C14-S14*S5;
    ROcp27_214 = ROcp27_26*C14-ROcp27_85*S14;
    ROcp27_314 = ROcp27_36*C14-ROcp27_95*S14;
    ROcp27_714 = ROcp27_16*S14+C14*S5;
    ROcp27_814 = ROcp27_26*S14+ROcp27_85*C14;
    ROcp27_914 = ROcp27_36*S14+ROcp27_95*C14;
    ROcp27_415 = ROcp27_46*C15+ROcp27_714*S15;
    ROcp27_515 = ROcp27_56*C15+ROcp27_814*S15;
    ROcp27_615 = ROcp27_66*C15+ROcp27_914*S15;
    ROcp27_715 = -(ROcp27_46*S15-ROcp27_714*C15);
    ROcp27_815 = -(ROcp27_56*S15-ROcp27_814*C15);
    ROcp27_915 = -(ROcp27_66*S15-ROcp27_914*C15);
    ROcp27_116 = ROcp27_114*C16+ROcp27_415*S16;
    ROcp27_216 = ROcp27_214*C16+ROcp27_515*S16;
    ROcp27_316 = ROcp27_314*C16+ROcp27_615*S16;
    ROcp27_416 = -(ROcp27_114*S16-ROcp27_415*C16);
    ROcp27_516 = -(ROcp27_214*S16-ROcp27_515*C16);
    ROcp27_616 = -(ROcp27_314*S16-ROcp27_615*C16);
    ROcp27_117 = ROcp27_116*C17-ROcp27_715*S17;
    ROcp27_217 = ROcp27_216*C17-ROcp27_815*S17;
    ROcp27_317 = ROcp27_316*C17-ROcp27_915*S17;
    ROcp27_717 = ROcp27_116*S17+ROcp27_715*C17;
    ROcp27_817 = ROcp27_216*S17+ROcp27_815*C17;
    ROcp27_917 = ROcp27_316*S17+ROcp27_915*C17;
    ROcp27_418 = ROcp27_416*C18+ROcp27_717*S18;
    ROcp27_518 = ROcp27_516*C18+ROcp27_817*S18;
    ROcp27_618 = ROcp27_616*C18+ROcp27_917*S18;
    ROcp27_718 = -(ROcp27_416*S18-ROcp27_717*C18);
    ROcp27_818 = -(ROcp27_516*S18-ROcp27_817*C18);
    ROcp27_918 = -(ROcp27_616*S18-ROcp27_917*C18);
    ROcp27_119 = ROcp27_117*C19-ROcp27_718*S19;
    ROcp27_219 = ROcp27_217*C19-ROcp27_818*S19;
    ROcp27_319 = ROcp27_317*C19-ROcp27_918*S19;
    ROcp27_719 = ROcp27_117*S19+ROcp27_718*C19;
    ROcp27_819 = ROcp27_217*S19+ROcp27_818*C19;
    ROcp27_919 = ROcp27_317*S19+ROcp27_918*C19;
    RLcp27_114 = ROcp27_46*s.dpt(2,2);
    RLcp27_214 = ROcp27_56*s.dpt(2,2);
    RLcp27_314 = ROcp27_66*s.dpt(2,2);
    OMcp27_114 = OMcp27_16+ROcp27_46*qd(14);
    OMcp27_214 = OMcp27_26+ROcp27_56*qd(14);
    OMcp27_314 = OMcp27_36+ROcp27_66*qd(14);
    ORcp27_114 = OMcp27_26*RLcp27_314-OMcp27_36*RLcp27_214;
    ORcp27_214 = -(OMcp27_16*RLcp27_314-OMcp27_36*RLcp27_114);
    ORcp27_314 = OMcp27_16*RLcp27_214-OMcp27_26*RLcp27_114;
    OPcp27_114 = OPcp27_16+ROcp27_46*qdd(14)+qd(14)*(OMcp27_26*ROcp27_66-OMcp27_36*ROcp27_56);
    OPcp27_214 = OPcp27_26+ROcp27_56*qdd(14)-qd(14)*(OMcp27_16*ROcp27_66-OMcp27_36*ROcp27_46);
    OPcp27_314 = OPcp27_36+ROcp27_66*qdd(14)+qd(14)*(OMcp27_16*ROcp27_56-OMcp27_26*ROcp27_46);
    RLcp27_115 = ROcp27_46*s.dpt(2,20);
    RLcp27_215 = ROcp27_56*s.dpt(2,20);
    RLcp27_315 = ROcp27_66*s.dpt(2,20);
    OMcp27_115 = OMcp27_114+ROcp27_114*qd(15);
    OMcp27_215 = OMcp27_214+ROcp27_214*qd(15);
    OMcp27_315 = OMcp27_314+ROcp27_314*qd(15);
    ORcp27_115 = OMcp27_214*RLcp27_315-OMcp27_314*RLcp27_215;
    ORcp27_215 = -(OMcp27_114*RLcp27_315-OMcp27_314*RLcp27_115);
    ORcp27_315 = OMcp27_114*RLcp27_215-OMcp27_214*RLcp27_115;
    OPcp27_115 = OPcp27_114+ROcp27_114*qdd(15)+qd(15)*(OMcp27_214*ROcp27_314-OMcp27_314*ROcp27_214);
    OPcp27_215 = OPcp27_214+ROcp27_214*qdd(15)-qd(15)*(OMcp27_114*ROcp27_314-OMcp27_314*ROcp27_114);
    OPcp27_315 = OPcp27_314+ROcp27_314*qdd(15)+qd(15)*(OMcp27_114*ROcp27_214-OMcp27_214*ROcp27_114);
    RLcp27_116 = ROcp27_715*s.dpt(3,22);
    RLcp27_216 = ROcp27_815*s.dpt(3,22);
    RLcp27_316 = ROcp27_915*s.dpt(3,22);
    OMcp27_116 = OMcp27_115+ROcp27_715*qd(16);
    OMcp27_216 = OMcp27_215+ROcp27_815*qd(16);
    OMcp27_316 = OMcp27_315+ROcp27_915*qd(16);
    ORcp27_116 = OMcp27_215*RLcp27_316-OMcp27_315*RLcp27_216;
    ORcp27_216 = -(OMcp27_115*RLcp27_316-OMcp27_315*RLcp27_116);
    ORcp27_316 = OMcp27_115*RLcp27_216-OMcp27_215*RLcp27_116;
    OPcp27_116 = OPcp27_115+ROcp27_715*qdd(16)+qd(16)*(OMcp27_215*ROcp27_915-OMcp27_315*ROcp27_815);
    OPcp27_216 = OPcp27_215+ROcp27_815*qdd(16)-qd(16)*(OMcp27_115*ROcp27_915-OMcp27_315*ROcp27_715);
    OPcp27_316 = OPcp27_315+ROcp27_915*qdd(16)+qd(16)*(OMcp27_115*ROcp27_815-OMcp27_215*ROcp27_715);
    RLcp27_117 = ROcp27_715*s.dpt(3,24);
    RLcp27_217 = ROcp27_815*s.dpt(3,24);
    RLcp27_317 = ROcp27_915*s.dpt(3,24);
    OMcp27_117 = OMcp27_116+ROcp27_416*qd(17);
    OMcp27_217 = OMcp27_216+ROcp27_516*qd(17);
    OMcp27_317 = OMcp27_316+ROcp27_616*qd(17);
    ORcp27_117 = OMcp27_216*RLcp27_317-OMcp27_316*RLcp27_217;
    ORcp27_217 = -(OMcp27_116*RLcp27_317-OMcp27_316*RLcp27_117);
    ORcp27_317 = OMcp27_116*RLcp27_217-OMcp27_216*RLcp27_117;
    OPcp27_117 = OPcp27_116+ROcp27_416*qdd(17)+qd(17)*(OMcp27_216*ROcp27_616-OMcp27_316*ROcp27_516);
    OPcp27_217 = OPcp27_216+ROcp27_516*qdd(17)-qd(17)*(OMcp27_116*ROcp27_616-OMcp27_316*ROcp27_416);
    OPcp27_317 = OPcp27_316+ROcp27_616*qdd(17)+qd(17)*(OMcp27_116*ROcp27_516-OMcp27_216*ROcp27_416);
    RLcp27_118 = ROcp27_717*s.dpt(3,26);
    RLcp27_218 = ROcp27_817*s.dpt(3,26);
    RLcp27_318 = ROcp27_917*s.dpt(3,26);
    OMcp27_118 = OMcp27_117+ROcp27_117*qd(18);
    OMcp27_218 = OMcp27_217+ROcp27_217*qd(18);
    OMcp27_318 = OMcp27_317+ROcp27_317*qd(18);
    ORcp27_118 = OMcp27_217*RLcp27_318-OMcp27_317*RLcp27_218;
    ORcp27_218 = -(OMcp27_117*RLcp27_318-OMcp27_317*RLcp27_118);
    ORcp27_318 = OMcp27_117*RLcp27_218-OMcp27_217*RLcp27_118;
    OMcp27_119 = OMcp27_118+ROcp27_418*qd(19);
    OMcp27_219 = OMcp27_218+ROcp27_518*qd(19);
    OMcp27_319 = OMcp27_318+ROcp27_618*qd(19);
    OPcp27_119 = OPcp27_117+ROcp27_117*qdd(18)+ROcp27_418*qdd(19)+qd(18)*(OMcp27_217*ROcp27_317-OMcp27_317*ROcp27_217)+qd(19)*(OMcp27_218*...
 ROcp27_618-OMcp27_318*ROcp27_518);
    OPcp27_219 = OPcp27_217+ROcp27_217*qdd(18)+ROcp27_518*qdd(19)-qd(18)*(OMcp27_117*ROcp27_317-OMcp27_317*ROcp27_117)-qd(19)*(OMcp27_118*...
 ROcp27_618-OMcp27_318*ROcp27_418);
    OPcp27_319 = OPcp27_317+ROcp27_317*qdd(18)+ROcp27_618*qdd(19)+qd(18)*(OMcp27_117*ROcp27_217-OMcp27_217*ROcp27_117)+qd(19)*(OMcp27_118*...
 ROcp27_518-OMcp27_218*ROcp27_418);
    RLcp27_159 = ROcp27_719*s.dpt(3,31);
    RLcp27_259 = ROcp27_819*s.dpt(3,31);
    RLcp27_359 = ROcp27_919*s.dpt(3,31);
    POcp27_159 = RLcp27_114+RLcp27_115+RLcp27_116+RLcp27_117+RLcp27_118+RLcp27_159+q(1);
    POcp27_259 = RLcp27_214+RLcp27_215+RLcp27_216+RLcp27_217+RLcp27_218+RLcp27_259+q(2);
    POcp27_359 = RLcp27_314+RLcp27_315+RLcp27_316+RLcp27_317+RLcp27_318+RLcp27_359+q(3);
    JTcp27_259_4 = -(RLcp27_314+RLcp27_315+RLcp27_316+RLcp27_317+RLcp27_318+RLcp27_359);
    JTcp27_359_4 = RLcp27_214+RLcp27_215+RLcp27_216+RLcp27_217+RLcp27_218+RLcp27_259;
    JTcp27_159_5 = C4*(RLcp27_314+RLcp27_315+RLcp27_316+RLcp27_317+RLcp27_318+RLcp27_359)-S4*(RLcp27_214+RLcp27_215)-S4*(RLcp27_216+RLcp27_217)-S4*...
 (RLcp27_218+RLcp27_259);
    JTcp27_259_5 = S4*(RLcp27_114+RLcp27_115+RLcp27_116+RLcp27_117+RLcp27_118+RLcp27_159);
    JTcp27_359_5 = -C4*(RLcp27_114+RLcp27_115+RLcp27_116+RLcp27_117+RLcp27_118+RLcp27_159);
    JTcp27_159_6 = ROcp27_85*(RLcp27_314+RLcp27_315+RLcp27_316+RLcp27_317+RLcp27_318+RLcp27_359)-ROcp27_95*(RLcp27_214+RLcp27_215)-ROcp27_95*(...
 RLcp27_216+RLcp27_217)-ROcp27_95*(RLcp27_218+RLcp27_259);
    JTcp27_259_6 = RLcp27_159*ROcp27_95-RLcp27_318*S5-RLcp27_359*S5+ROcp27_95*(RLcp27_114+RLcp27_115+RLcp27_116+RLcp27_117+RLcp27_118)-S5*(...
 RLcp27_314+RLcp27_315)-S5*(RLcp27_316+RLcp27_317);
    JTcp27_359_6 = RLcp27_218*S5-ROcp27_85*(RLcp27_114+RLcp27_115+RLcp27_116+RLcp27_117+RLcp27_118)+S5*(RLcp27_214+RLcp27_215)+S5*(RLcp27_216+...
 RLcp27_217)-RLcp27_159*ROcp27_85+RLcp27_259*S5;
    JTcp27_159_7 = ROcp27_56*(RLcp27_315+RLcp27_316+RLcp27_317+RLcp27_318)-ROcp27_66*(RLcp27_215+RLcp27_216)-ROcp27_66*(RLcp27_217+RLcp27_218)-...
 RLcp27_259*ROcp27_66+RLcp27_359*ROcp27_56;
    JTcp27_259_7 = RLcp27_159*ROcp27_66-RLcp27_359*ROcp27_46-ROcp27_46*(RLcp27_315+RLcp27_316+RLcp27_317+RLcp27_318)+ROcp27_66*(RLcp27_115+...
 RLcp27_116)+ROcp27_66*(RLcp27_117+RLcp27_118);
    JTcp27_359_7 = ROcp27_46*(RLcp27_215+RLcp27_216+RLcp27_217+RLcp27_218)-ROcp27_56*(RLcp27_115+RLcp27_116)-ROcp27_56*(RLcp27_117+RLcp27_118)-...
 RLcp27_159*ROcp27_56+RLcp27_259*ROcp27_46;
    JTcp27_159_8 = ROcp27_214*(RLcp27_316+RLcp27_317+RLcp27_318+RLcp27_359)-ROcp27_314*(RLcp27_216+RLcp27_217)-ROcp27_314*(RLcp27_218+RLcp27_259);
    JTcp27_259_8 = -(ROcp27_114*(RLcp27_316+RLcp27_317+RLcp27_318+RLcp27_359)-ROcp27_314*(RLcp27_116+RLcp27_117)-ROcp27_314*(RLcp27_118+RLcp27_159)...
 );
    JTcp27_359_8 = ROcp27_114*(RLcp27_216+RLcp27_217+RLcp27_218+RLcp27_259)-ROcp27_214*(RLcp27_116+RLcp27_117)-ROcp27_214*(RLcp27_118+RLcp27_159);
    JTcp27_159_9 = ROcp27_815*(RLcp27_317+RLcp27_318)-ROcp27_915*(RLcp27_217+RLcp27_218)-RLcp27_259*ROcp27_915+RLcp27_359*ROcp27_815;
    JTcp27_259_9 = RLcp27_159*ROcp27_915-RLcp27_359*ROcp27_715-ROcp27_715*(RLcp27_317+RLcp27_318)+ROcp27_915*(RLcp27_117+RLcp27_118);
    JTcp27_359_9 = ROcp27_715*(RLcp27_217+RLcp27_218)-ROcp27_815*(RLcp27_117+RLcp27_118)-RLcp27_159*ROcp27_815+RLcp27_259*ROcp27_715;
    JTcp27_159_10 = ROcp27_516*(RLcp27_318+RLcp27_359)-ROcp27_616*(RLcp27_218+RLcp27_259);
    JTcp27_259_10 = -(ROcp27_416*(RLcp27_318+RLcp27_359)-ROcp27_616*(RLcp27_118+RLcp27_159));
    JTcp27_359_10 = ROcp27_416*(RLcp27_218+RLcp27_259)-ROcp27_516*(RLcp27_118+RLcp27_159);
    JTcp27_159_11 = -(RLcp27_259*ROcp27_317-RLcp27_359*ROcp27_217);
    JTcp27_259_11 = RLcp27_159*ROcp27_317-RLcp27_359*ROcp27_117;
    JTcp27_359_11 = -(RLcp27_159*ROcp27_217-RLcp27_259*ROcp27_117);
    JTcp27_159_12 = -(RLcp27_259*ROcp27_618-RLcp27_359*ROcp27_518);
    JTcp27_259_12 = RLcp27_159*ROcp27_618-RLcp27_359*ROcp27_418;
    JTcp27_359_12 = -(RLcp27_159*ROcp27_518-RLcp27_259*ROcp27_418);
    ORcp27_159 = OMcp27_219*RLcp27_359-OMcp27_319*RLcp27_259;
    ORcp27_259 = -(OMcp27_119*RLcp27_359-OMcp27_319*RLcp27_159);
    ORcp27_359 = OMcp27_119*RLcp27_259-OMcp27_219*RLcp27_159;
    VIcp27_159 = ORcp27_114+ORcp27_115+ORcp27_116+ORcp27_117+ORcp27_118+ORcp27_159+qd(1);
    VIcp27_259 = ORcp27_214+ORcp27_215+ORcp27_216+ORcp27_217+ORcp27_218+ORcp27_259+qd(2);
    VIcp27_359 = ORcp27_314+ORcp27_315+ORcp27_316+ORcp27_317+ORcp27_318+ORcp27_359+qd(3);
    ACcp27_159 = qdd(1)+OMcp27_214*ORcp27_315+OMcp27_215*ORcp27_316+OMcp27_216*ORcp27_317+OMcp27_217*ORcp27_318+OMcp27_219*ORcp27_359+OMcp27_26*...
 ORcp27_314-OMcp27_314*ORcp27_215-OMcp27_315*ORcp27_216-OMcp27_316*ORcp27_217-OMcp27_317*ORcp27_218-OMcp27_319*ORcp27_259-OMcp27_36*ORcp27_214+...
 OPcp27_214*RLcp27_315+OPcp27_215*RLcp27_316+OPcp27_216*RLcp27_317+OPcp27_217*RLcp27_318+OPcp27_219*RLcp27_359+OPcp27_26*RLcp27_314-OPcp27_314*...
 RLcp27_215-OPcp27_315*RLcp27_216-OPcp27_316*RLcp27_217-OPcp27_317*RLcp27_218-OPcp27_319*RLcp27_259-OPcp27_36*RLcp27_214;
    ACcp27_259 = qdd(2)-OMcp27_114*ORcp27_315-OMcp27_115*ORcp27_316-OMcp27_116*ORcp27_317-OMcp27_117*ORcp27_318-OMcp27_119*ORcp27_359-OMcp27_16*...
 ORcp27_314+OMcp27_314*ORcp27_115+OMcp27_315*ORcp27_116+OMcp27_316*ORcp27_117+OMcp27_317*ORcp27_118+OMcp27_319*ORcp27_159+OMcp27_36*ORcp27_114-...
 OPcp27_114*RLcp27_315-OPcp27_115*RLcp27_316-OPcp27_116*RLcp27_317-OPcp27_117*RLcp27_318-OPcp27_119*RLcp27_359-OPcp27_16*RLcp27_314+OPcp27_314*...
 RLcp27_115+OPcp27_315*RLcp27_116+OPcp27_316*RLcp27_117+OPcp27_317*RLcp27_118+OPcp27_319*RLcp27_159+OPcp27_36*RLcp27_114;
    ACcp27_359 = qdd(3)+OMcp27_114*ORcp27_215+OMcp27_115*ORcp27_216+OMcp27_116*ORcp27_217+OMcp27_117*ORcp27_218+OMcp27_119*ORcp27_259+OMcp27_16*...
 ORcp27_214-OMcp27_214*ORcp27_115-OMcp27_215*ORcp27_116-OMcp27_216*ORcp27_117-OMcp27_217*ORcp27_118-OMcp27_219*ORcp27_159-OMcp27_26*ORcp27_114+...
 OPcp27_114*RLcp27_215+OPcp27_115*RLcp27_216+OPcp27_116*RLcp27_217+OPcp27_117*RLcp27_218+OPcp27_119*RLcp27_259+OPcp27_16*RLcp27_214-OPcp27_214*...
 RLcp27_115-OPcp27_215*RLcp27_116-OPcp27_216*RLcp27_117-OPcp27_217*RLcp27_118-OPcp27_219*RLcp27_159-OPcp27_26*RLcp27_114;

% = = Block_1_0_0_28_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp27_159;
    sens.P(2) = POcp27_259;
    sens.P(3) = POcp27_359;
    sens.R(1,1) = ROcp27_119;
    sens.R(1,2) = ROcp27_219;
    sens.R(1,3) = ROcp27_319;
    sens.R(2,1) = ROcp27_418;
    sens.R(2,2) = ROcp27_518;
    sens.R(2,3) = ROcp27_618;
    sens.R(3,1) = ROcp27_719;
    sens.R(3,2) = ROcp27_819;
    sens.R(3,3) = ROcp27_919;
    sens.V(1) = VIcp27_159;
    sens.V(2) = VIcp27_259;
    sens.V(3) = VIcp27_359;
    sens.OM(1) = OMcp27_119;
    sens.OM(2) = OMcp27_219;
    sens.OM(3) = OMcp27_319;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp27_159_5;
    sens.J(1,6) = JTcp27_159_6;
    sens.J(1,14) = JTcp27_159_7;
    sens.J(1,15) = JTcp27_159_8;
    sens.J(1,16) = JTcp27_159_9;
    sens.J(1,17) = JTcp27_159_10;
    sens.J(1,18) = JTcp27_159_11;
    sens.J(1,19) = JTcp27_159_12;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp27_259_4;
    sens.J(2,5) = JTcp27_259_5;
    sens.J(2,6) = JTcp27_259_6;
    sens.J(2,14) = JTcp27_259_7;
    sens.J(2,15) = JTcp27_259_8;
    sens.J(2,16) = JTcp27_259_9;
    sens.J(2,17) = JTcp27_259_10;
    sens.J(2,18) = JTcp27_259_11;
    sens.J(2,19) = JTcp27_259_12;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp27_359_4;
    sens.J(3,5) = JTcp27_359_5;
    sens.J(3,6) = JTcp27_359_6;
    sens.J(3,14) = JTcp27_359_7;
    sens.J(3,15) = JTcp27_359_8;
    sens.J(3,16) = JTcp27_359_9;
    sens.J(3,17) = JTcp27_359_10;
    sens.J(3,18) = JTcp27_359_11;
    sens.J(3,19) = JTcp27_359_12;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,14) = ROcp27_46;
    sens.J(4,15) = ROcp27_114;
    sens.J(4,16) = ROcp27_715;
    sens.J(4,17) = ROcp27_416;
    sens.J(4,18) = ROcp27_117;
    sens.J(4,19) = ROcp27_418;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp27_85;
    sens.J(5,14) = ROcp27_56;
    sens.J(5,15) = ROcp27_214;
    sens.J(5,16) = ROcp27_815;
    sens.J(5,17) = ROcp27_516;
    sens.J(5,18) = ROcp27_217;
    sens.J(5,19) = ROcp27_518;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp27_95;
    sens.J(6,14) = ROcp27_66;
    sens.J(6,15) = ROcp27_314;
    sens.J(6,16) = ROcp27_915;
    sens.J(6,17) = ROcp27_616;
    sens.J(6,18) = ROcp27_317;
    sens.J(6,19) = ROcp27_618;
    sens.A(1) = ACcp27_159;
    sens.A(2) = ACcp27_259;
    sens.A(3) = ACcp27_359;
    sens.OMP(1) = OPcp27_119;
    sens.OMP(2) = OPcp27_219;
    sens.OMP(3) = OPcp27_319;
 
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

% = = Block_1_0_0_29_0_3 = = 
 
% Sensor Kinematics 


    ROcp28_114 = ROcp28_16*C14-S14*S5;
    ROcp28_214 = ROcp28_26*C14-ROcp28_85*S14;
    ROcp28_314 = ROcp28_36*C14-ROcp28_95*S14;
    ROcp28_714 = ROcp28_16*S14+C14*S5;
    ROcp28_814 = ROcp28_26*S14+ROcp28_85*C14;
    ROcp28_914 = ROcp28_36*S14+ROcp28_95*C14;
    ROcp28_415 = ROcp28_46*C15+ROcp28_714*S15;
    ROcp28_515 = ROcp28_56*C15+ROcp28_814*S15;
    ROcp28_615 = ROcp28_66*C15+ROcp28_914*S15;
    ROcp28_715 = -(ROcp28_46*S15-ROcp28_714*C15);
    ROcp28_815 = -(ROcp28_56*S15-ROcp28_814*C15);
    ROcp28_915 = -(ROcp28_66*S15-ROcp28_914*C15);
    ROcp28_116 = ROcp28_114*C16+ROcp28_415*S16;
    ROcp28_216 = ROcp28_214*C16+ROcp28_515*S16;
    ROcp28_316 = ROcp28_314*C16+ROcp28_615*S16;
    ROcp28_416 = -(ROcp28_114*S16-ROcp28_415*C16);
    ROcp28_516 = -(ROcp28_214*S16-ROcp28_515*C16);
    ROcp28_616 = -(ROcp28_314*S16-ROcp28_615*C16);
    ROcp28_117 = ROcp28_116*C17-ROcp28_715*S17;
    ROcp28_217 = ROcp28_216*C17-ROcp28_815*S17;
    ROcp28_317 = ROcp28_316*C17-ROcp28_915*S17;
    ROcp28_717 = ROcp28_116*S17+ROcp28_715*C17;
    ROcp28_817 = ROcp28_216*S17+ROcp28_815*C17;
    ROcp28_917 = ROcp28_316*S17+ROcp28_915*C17;
    ROcp28_418 = ROcp28_416*C18+ROcp28_717*S18;
    ROcp28_518 = ROcp28_516*C18+ROcp28_817*S18;
    ROcp28_618 = ROcp28_616*C18+ROcp28_917*S18;
    ROcp28_718 = -(ROcp28_416*S18-ROcp28_717*C18);
    ROcp28_818 = -(ROcp28_516*S18-ROcp28_817*C18);
    ROcp28_918 = -(ROcp28_616*S18-ROcp28_917*C18);
    ROcp28_119 = ROcp28_117*C19-ROcp28_718*S19;
    ROcp28_219 = ROcp28_217*C19-ROcp28_818*S19;
    ROcp28_319 = ROcp28_317*C19-ROcp28_918*S19;
    ROcp28_719 = ROcp28_117*S19+ROcp28_718*C19;
    ROcp28_819 = ROcp28_217*S19+ROcp28_818*C19;
    ROcp28_919 = ROcp28_317*S19+ROcp28_918*C19;
    ROcp28_120 = ROcp28_119*C20-ROcp28_719*S20;
    ROcp28_220 = ROcp28_219*C20-ROcp28_819*S20;
    ROcp28_320 = ROcp28_319*C20-ROcp28_919*S20;
    ROcp28_720 = ROcp28_119*S20+ROcp28_719*C20;
    ROcp28_820 = ROcp28_219*S20+ROcp28_819*C20;
    ROcp28_920 = ROcp28_319*S20+ROcp28_919*C20;
    RLcp28_114 = ROcp28_46*s.dpt(2,2);
    RLcp28_214 = ROcp28_56*s.dpt(2,2);
    RLcp28_314 = ROcp28_66*s.dpt(2,2);
    OMcp28_114 = OMcp28_16+ROcp28_46*qd(14);
    OMcp28_214 = OMcp28_26+ROcp28_56*qd(14);
    OMcp28_314 = OMcp28_36+ROcp28_66*qd(14);
    ORcp28_114 = OMcp28_26*RLcp28_314-OMcp28_36*RLcp28_214;
    ORcp28_214 = -(OMcp28_16*RLcp28_314-OMcp28_36*RLcp28_114);
    ORcp28_314 = OMcp28_16*RLcp28_214-OMcp28_26*RLcp28_114;
    OPcp28_114 = OPcp28_16+ROcp28_46*qdd(14)+qd(14)*(OMcp28_26*ROcp28_66-OMcp28_36*ROcp28_56);
    OPcp28_214 = OPcp28_26+ROcp28_56*qdd(14)-qd(14)*(OMcp28_16*ROcp28_66-OMcp28_36*ROcp28_46);
    OPcp28_314 = OPcp28_36+ROcp28_66*qdd(14)+qd(14)*(OMcp28_16*ROcp28_56-OMcp28_26*ROcp28_46);
    RLcp28_115 = ROcp28_46*s.dpt(2,20);
    RLcp28_215 = ROcp28_56*s.dpt(2,20);
    RLcp28_315 = ROcp28_66*s.dpt(2,20);
    OMcp28_115 = OMcp28_114+ROcp28_114*qd(15);
    OMcp28_215 = OMcp28_214+ROcp28_214*qd(15);
    OMcp28_315 = OMcp28_314+ROcp28_314*qd(15);
    ORcp28_115 = OMcp28_214*RLcp28_315-OMcp28_314*RLcp28_215;
    ORcp28_215 = -(OMcp28_114*RLcp28_315-OMcp28_314*RLcp28_115);
    ORcp28_315 = OMcp28_114*RLcp28_215-OMcp28_214*RLcp28_115;
    OPcp28_115 = OPcp28_114+ROcp28_114*qdd(15)+qd(15)*(OMcp28_214*ROcp28_314-OMcp28_314*ROcp28_214);
    OPcp28_215 = OPcp28_214+ROcp28_214*qdd(15)-qd(15)*(OMcp28_114*ROcp28_314-OMcp28_314*ROcp28_114);
    OPcp28_315 = OPcp28_314+ROcp28_314*qdd(15)+qd(15)*(OMcp28_114*ROcp28_214-OMcp28_214*ROcp28_114);
    RLcp28_116 = ROcp28_715*s.dpt(3,22);
    RLcp28_216 = ROcp28_815*s.dpt(3,22);
    RLcp28_316 = ROcp28_915*s.dpt(3,22);
    OMcp28_116 = OMcp28_115+ROcp28_715*qd(16);
    OMcp28_216 = OMcp28_215+ROcp28_815*qd(16);
    OMcp28_316 = OMcp28_315+ROcp28_915*qd(16);
    ORcp28_116 = OMcp28_215*RLcp28_316-OMcp28_315*RLcp28_216;
    ORcp28_216 = -(OMcp28_115*RLcp28_316-OMcp28_315*RLcp28_116);
    ORcp28_316 = OMcp28_115*RLcp28_216-OMcp28_215*RLcp28_116;
    OPcp28_116 = OPcp28_115+ROcp28_715*qdd(16)+qd(16)*(OMcp28_215*ROcp28_915-OMcp28_315*ROcp28_815);
    OPcp28_216 = OPcp28_215+ROcp28_815*qdd(16)-qd(16)*(OMcp28_115*ROcp28_915-OMcp28_315*ROcp28_715);
    OPcp28_316 = OPcp28_315+ROcp28_915*qdd(16)+qd(16)*(OMcp28_115*ROcp28_815-OMcp28_215*ROcp28_715);
    RLcp28_117 = ROcp28_715*s.dpt(3,24);
    RLcp28_217 = ROcp28_815*s.dpt(3,24);
    RLcp28_317 = ROcp28_915*s.dpt(3,24);
    OMcp28_117 = OMcp28_116+ROcp28_416*qd(17);
    OMcp28_217 = OMcp28_216+ROcp28_516*qd(17);
    OMcp28_317 = OMcp28_316+ROcp28_616*qd(17);
    ORcp28_117 = OMcp28_216*RLcp28_317-OMcp28_316*RLcp28_217;
    ORcp28_217 = -(OMcp28_116*RLcp28_317-OMcp28_316*RLcp28_117);
    ORcp28_317 = OMcp28_116*RLcp28_217-OMcp28_216*RLcp28_117;
    OPcp28_117 = OPcp28_116+ROcp28_416*qdd(17)+qd(17)*(OMcp28_216*ROcp28_616-OMcp28_316*ROcp28_516);
    OPcp28_217 = OPcp28_216+ROcp28_516*qdd(17)-qd(17)*(OMcp28_116*ROcp28_616-OMcp28_316*ROcp28_416);
    OPcp28_317 = OPcp28_316+ROcp28_616*qdd(17)+qd(17)*(OMcp28_116*ROcp28_516-OMcp28_216*ROcp28_416);
    RLcp28_118 = ROcp28_717*s.dpt(3,26);
    RLcp28_218 = ROcp28_817*s.dpt(3,26);
    RLcp28_318 = ROcp28_917*s.dpt(3,26);
    OMcp28_118 = OMcp28_117+ROcp28_117*qd(18);
    OMcp28_218 = OMcp28_217+ROcp28_217*qd(18);
    OMcp28_318 = OMcp28_317+ROcp28_317*qd(18);
    ORcp28_118 = OMcp28_217*RLcp28_318-OMcp28_317*RLcp28_218;
    ORcp28_218 = -(OMcp28_117*RLcp28_318-OMcp28_317*RLcp28_118);
    ORcp28_318 = OMcp28_117*RLcp28_218-OMcp28_217*RLcp28_118;
    OMcp28_119 = OMcp28_118+ROcp28_418*qd(19);
    OMcp28_219 = OMcp28_218+ROcp28_518*qd(19);
    OMcp28_319 = OMcp28_318+ROcp28_618*qd(19);
    OPcp28_119 = OPcp28_117+ROcp28_117*qdd(18)+ROcp28_418*qdd(19)+qd(18)*(OMcp28_217*ROcp28_317-OMcp28_317*ROcp28_217)+qd(19)*(OMcp28_218*...
 ROcp28_618-OMcp28_318*ROcp28_518);
    OPcp28_219 = OPcp28_217+ROcp28_217*qdd(18)+ROcp28_518*qdd(19)-qd(18)*(OMcp28_117*ROcp28_317-OMcp28_317*ROcp28_117)-qd(19)*(OMcp28_118*...
 ROcp28_618-OMcp28_318*ROcp28_418);
    OPcp28_319 = OPcp28_317+ROcp28_317*qdd(18)+ROcp28_618*qdd(19)+qd(18)*(OMcp28_117*ROcp28_217-OMcp28_217*ROcp28_117)+qd(19)*(OMcp28_118*...
 ROcp28_518-OMcp28_218*ROcp28_418);
    RLcp28_120 = ROcp28_119*s.dpt(1,32)+ROcp28_719*s.dpt(3,32);
    RLcp28_220 = ROcp28_219*s.dpt(1,32)+ROcp28_819*s.dpt(3,32);
    RLcp28_320 = ROcp28_319*s.dpt(1,32)+ROcp28_919*s.dpt(3,32);
    POcp28_120 = RLcp28_114+RLcp28_115+RLcp28_116+RLcp28_117+RLcp28_118+RLcp28_120+q(1);
    POcp28_220 = RLcp28_214+RLcp28_215+RLcp28_216+RLcp28_217+RLcp28_218+RLcp28_220+q(2);
    POcp28_320 = RLcp28_314+RLcp28_315+RLcp28_316+RLcp28_317+RLcp28_318+RLcp28_320+q(3);
    JTcp28_220_4 = -(RLcp28_314+RLcp28_315+RLcp28_316+RLcp28_317+RLcp28_318+RLcp28_320);
    JTcp28_320_4 = RLcp28_214+RLcp28_215+RLcp28_216+RLcp28_217+RLcp28_218+RLcp28_220;
    JTcp28_120_5 = C4*(RLcp28_314+RLcp28_315+RLcp28_316+RLcp28_317+RLcp28_318+RLcp28_320)-S4*(RLcp28_214+RLcp28_215)-S4*(RLcp28_216+RLcp28_217)-S4*...
 (RLcp28_218+RLcp28_220);
    JTcp28_220_5 = S4*(RLcp28_114+RLcp28_115+RLcp28_116+RLcp28_117+RLcp28_118+RLcp28_120);
    JTcp28_320_5 = -C4*(RLcp28_114+RLcp28_115+RLcp28_116+RLcp28_117+RLcp28_118+RLcp28_120);
    JTcp28_120_6 = ROcp28_85*(RLcp28_314+RLcp28_315+RLcp28_316+RLcp28_317+RLcp28_318+RLcp28_320)-ROcp28_95*(RLcp28_214+RLcp28_215)-ROcp28_95*(...
 RLcp28_216+RLcp28_217)-ROcp28_95*(RLcp28_218+RLcp28_220);
    JTcp28_220_6 = RLcp28_120*ROcp28_95-RLcp28_318*S5-RLcp28_320*S5+ROcp28_95*(RLcp28_114+RLcp28_115+RLcp28_116+RLcp28_117+RLcp28_118)-S5*(...
 RLcp28_314+RLcp28_315)-S5*(RLcp28_316+RLcp28_317);
    JTcp28_320_6 = RLcp28_218*S5-ROcp28_85*(RLcp28_114+RLcp28_115+RLcp28_116+RLcp28_117+RLcp28_118)+S5*(RLcp28_214+RLcp28_215)+S5*(RLcp28_216+...
 RLcp28_217)-RLcp28_120*ROcp28_85+RLcp28_220*S5;
    JTcp28_120_7 = ROcp28_56*(RLcp28_315+RLcp28_316+RLcp28_317+RLcp28_318)-ROcp28_66*(RLcp28_215+RLcp28_216)-ROcp28_66*(RLcp28_217+RLcp28_218)-...
 RLcp28_220*ROcp28_66+RLcp28_320*ROcp28_56;
    JTcp28_220_7 = RLcp28_120*ROcp28_66-RLcp28_320*ROcp28_46-ROcp28_46*(RLcp28_315+RLcp28_316+RLcp28_317+RLcp28_318)+ROcp28_66*(RLcp28_115+...
 RLcp28_116)+ROcp28_66*(RLcp28_117+RLcp28_118);
    JTcp28_320_7 = ROcp28_46*(RLcp28_215+RLcp28_216+RLcp28_217+RLcp28_218)-ROcp28_56*(RLcp28_115+RLcp28_116)-ROcp28_56*(RLcp28_117+RLcp28_118)-...
 RLcp28_120*ROcp28_56+RLcp28_220*ROcp28_46;
    JTcp28_120_8 = ROcp28_214*(RLcp28_316+RLcp28_317+RLcp28_318+RLcp28_320)-ROcp28_314*(RLcp28_216+RLcp28_217)-ROcp28_314*(RLcp28_218+RLcp28_220);
    JTcp28_220_8 = -(ROcp28_114*(RLcp28_316+RLcp28_317+RLcp28_318+RLcp28_320)-ROcp28_314*(RLcp28_116+RLcp28_117)-ROcp28_314*(RLcp28_118+RLcp28_120)...
 );
    JTcp28_320_8 = ROcp28_114*(RLcp28_216+RLcp28_217+RLcp28_218+RLcp28_220)-ROcp28_214*(RLcp28_116+RLcp28_117)-ROcp28_214*(RLcp28_118+RLcp28_120);
    JTcp28_120_9 = ROcp28_815*(RLcp28_317+RLcp28_318)-ROcp28_915*(RLcp28_217+RLcp28_218)-RLcp28_220*ROcp28_915+RLcp28_320*ROcp28_815;
    JTcp28_220_9 = RLcp28_120*ROcp28_915-RLcp28_320*ROcp28_715-ROcp28_715*(RLcp28_317+RLcp28_318)+ROcp28_915*(RLcp28_117+RLcp28_118);
    JTcp28_320_9 = ROcp28_715*(RLcp28_217+RLcp28_218)-ROcp28_815*(RLcp28_117+RLcp28_118)-RLcp28_120*ROcp28_815+RLcp28_220*ROcp28_715;
    JTcp28_120_10 = ROcp28_516*(RLcp28_318+RLcp28_320)-ROcp28_616*(RLcp28_218+RLcp28_220);
    JTcp28_220_10 = -(ROcp28_416*(RLcp28_318+RLcp28_320)-ROcp28_616*(RLcp28_118+RLcp28_120));
    JTcp28_320_10 = ROcp28_416*(RLcp28_218+RLcp28_220)-ROcp28_516*(RLcp28_118+RLcp28_120);
    JTcp28_120_11 = -(RLcp28_220*ROcp28_317-RLcp28_320*ROcp28_217);
    JTcp28_220_11 = RLcp28_120*ROcp28_317-RLcp28_320*ROcp28_117;
    JTcp28_320_11 = -(RLcp28_120*ROcp28_217-RLcp28_220*ROcp28_117);
    JTcp28_120_12 = -(RLcp28_220*ROcp28_618-RLcp28_320*ROcp28_518);
    JTcp28_220_12 = RLcp28_120*ROcp28_618-RLcp28_320*ROcp28_418;
    JTcp28_320_12 = -(RLcp28_120*ROcp28_518-RLcp28_220*ROcp28_418);
    OMcp28_120 = OMcp28_119+ROcp28_418*qd(20);
    OMcp28_220 = OMcp28_219+ROcp28_518*qd(20);
    OMcp28_320 = OMcp28_319+ROcp28_618*qd(20);
    ORcp28_120 = OMcp28_219*RLcp28_320-OMcp28_319*RLcp28_220;
    ORcp28_220 = -(OMcp28_119*RLcp28_320-OMcp28_319*RLcp28_120);
    ORcp28_320 = OMcp28_119*RLcp28_220-OMcp28_219*RLcp28_120;
    VIcp28_120 = ORcp28_114+ORcp28_115+ORcp28_116+ORcp28_117+ORcp28_118+ORcp28_120+qd(1);
    VIcp28_220 = ORcp28_214+ORcp28_215+ORcp28_216+ORcp28_217+ORcp28_218+ORcp28_220+qd(2);
    VIcp28_320 = ORcp28_314+ORcp28_315+ORcp28_316+ORcp28_317+ORcp28_318+ORcp28_320+qd(3);
    OPcp28_120 = OPcp28_119+ROcp28_418*qdd(20)+qd(20)*(OMcp28_219*ROcp28_618-OMcp28_319*ROcp28_518);
    OPcp28_220 = OPcp28_219+ROcp28_518*qdd(20)-qd(20)*(OMcp28_119*ROcp28_618-OMcp28_319*ROcp28_418);
    OPcp28_320 = OPcp28_319+ROcp28_618*qdd(20)+qd(20)*(OMcp28_119*ROcp28_518-OMcp28_219*ROcp28_418);
    ACcp28_120 = qdd(1)+OMcp28_214*ORcp28_315+OMcp28_215*ORcp28_316+OMcp28_216*ORcp28_317+OMcp28_217*ORcp28_318+OMcp28_219*ORcp28_320+OMcp28_26*...
 ORcp28_314-OMcp28_314*ORcp28_215-OMcp28_315*ORcp28_216-OMcp28_316*ORcp28_217-OMcp28_317*ORcp28_218-OMcp28_319*ORcp28_220-OMcp28_36*ORcp28_214+...
 OPcp28_214*RLcp28_315+OPcp28_215*RLcp28_316+OPcp28_216*RLcp28_317+OPcp28_217*RLcp28_318+OPcp28_219*RLcp28_320+OPcp28_26*RLcp28_314-OPcp28_314*...
 RLcp28_215-OPcp28_315*RLcp28_216-OPcp28_316*RLcp28_217-OPcp28_317*RLcp28_218-OPcp28_319*RLcp28_220-OPcp28_36*RLcp28_214;
    ACcp28_220 = qdd(2)-OMcp28_114*ORcp28_315-OMcp28_115*ORcp28_316-OMcp28_116*ORcp28_317-OMcp28_117*ORcp28_318-OMcp28_119*ORcp28_320-OMcp28_16*...
 ORcp28_314+OMcp28_314*ORcp28_115+OMcp28_315*ORcp28_116+OMcp28_316*ORcp28_117+OMcp28_317*ORcp28_118+OMcp28_319*ORcp28_120+OMcp28_36*ORcp28_114-...
 OPcp28_114*RLcp28_315-OPcp28_115*RLcp28_316-OPcp28_116*RLcp28_317-OPcp28_117*RLcp28_318-OPcp28_119*RLcp28_320-OPcp28_16*RLcp28_314+OPcp28_314*...
 RLcp28_115+OPcp28_315*RLcp28_116+OPcp28_316*RLcp28_117+OPcp28_317*RLcp28_118+OPcp28_319*RLcp28_120+OPcp28_36*RLcp28_114;
    ACcp28_320 = qdd(3)+OMcp28_114*ORcp28_215+OMcp28_115*ORcp28_216+OMcp28_116*ORcp28_217+OMcp28_117*ORcp28_218+OMcp28_119*ORcp28_220+OMcp28_16*...
 ORcp28_214-OMcp28_214*ORcp28_115-OMcp28_215*ORcp28_116-OMcp28_216*ORcp28_117-OMcp28_217*ORcp28_118-OMcp28_219*ORcp28_120-OMcp28_26*ORcp28_114+...
 OPcp28_114*RLcp28_215+OPcp28_115*RLcp28_216+OPcp28_116*RLcp28_217+OPcp28_117*RLcp28_218+OPcp28_119*RLcp28_220+OPcp28_16*RLcp28_214-OPcp28_214*...
 RLcp28_115-OPcp28_215*RLcp28_116-OPcp28_216*RLcp28_117-OPcp28_217*RLcp28_118-OPcp28_219*RLcp28_120-OPcp28_26*RLcp28_114;

% = = Block_1_0_0_29_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp28_120;
    sens.P(2) = POcp28_220;
    sens.P(3) = POcp28_320;
    sens.R(1,1) = ROcp28_120;
    sens.R(1,2) = ROcp28_220;
    sens.R(1,3) = ROcp28_320;
    sens.R(2,1) = ROcp28_418;
    sens.R(2,2) = ROcp28_518;
    sens.R(2,3) = ROcp28_618;
    sens.R(3,1) = ROcp28_720;
    sens.R(3,2) = ROcp28_820;
    sens.R(3,3) = ROcp28_920;
    sens.V(1) = VIcp28_120;
    sens.V(2) = VIcp28_220;
    sens.V(3) = VIcp28_320;
    sens.OM(1) = OMcp28_120;
    sens.OM(2) = OMcp28_220;
    sens.OM(3) = OMcp28_320;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp28_120_5;
    sens.J(1,6) = JTcp28_120_6;
    sens.J(1,14) = JTcp28_120_7;
    sens.J(1,15) = JTcp28_120_8;
    sens.J(1,16) = JTcp28_120_9;
    sens.J(1,17) = JTcp28_120_10;
    sens.J(1,18) = JTcp28_120_11;
    sens.J(1,19) = JTcp28_120_12;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp28_220_4;
    sens.J(2,5) = JTcp28_220_5;
    sens.J(2,6) = JTcp28_220_6;
    sens.J(2,14) = JTcp28_220_7;
    sens.J(2,15) = JTcp28_220_8;
    sens.J(2,16) = JTcp28_220_9;
    sens.J(2,17) = JTcp28_220_10;
    sens.J(2,18) = JTcp28_220_11;
    sens.J(2,19) = JTcp28_220_12;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp28_320_4;
    sens.J(3,5) = JTcp28_320_5;
    sens.J(3,6) = JTcp28_320_6;
    sens.J(3,14) = JTcp28_320_7;
    sens.J(3,15) = JTcp28_320_8;
    sens.J(3,16) = JTcp28_320_9;
    sens.J(3,17) = JTcp28_320_10;
    sens.J(3,18) = JTcp28_320_11;
    sens.J(3,19) = JTcp28_320_12;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,14) = ROcp28_46;
    sens.J(4,15) = ROcp28_114;
    sens.J(4,16) = ROcp28_715;
    sens.J(4,17) = ROcp28_416;
    sens.J(4,18) = ROcp28_117;
    sens.J(4,19) = ROcp28_418;
    sens.J(4,20) = ROcp28_418;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp28_85;
    sens.J(5,14) = ROcp28_56;
    sens.J(5,15) = ROcp28_214;
    sens.J(5,16) = ROcp28_815;
    sens.J(5,17) = ROcp28_516;
    sens.J(5,18) = ROcp28_217;
    sens.J(5,19) = ROcp28_518;
    sens.J(5,20) = ROcp28_518;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp28_95;
    sens.J(6,14) = ROcp28_66;
    sens.J(6,15) = ROcp28_314;
    sens.J(6,16) = ROcp28_915;
    sens.J(6,17) = ROcp28_616;
    sens.J(6,18) = ROcp28_317;
    sens.J(6,19) = ROcp28_618;
    sens.J(6,20) = ROcp28_618;
    sens.A(1) = ACcp28_120;
    sens.A(2) = ACcp28_220;
    sens.A(3) = ACcp28_320;
    sens.OMP(1) = OPcp28_120;
    sens.OMP(2) = OPcp28_220;
    sens.OMP(3) = OPcp28_320;
 
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


    ROcp29_421 = ROcp29_46*C21+S21*S5;
    ROcp29_521 = ROcp29_56*C21+ROcp29_85*S21;
    ROcp29_621 = ROcp29_66*C21+ROcp29_95*S21;
    ROcp29_721 = -(ROcp29_46*S21-C21*S5);
    ROcp29_821 = -(ROcp29_56*S21-ROcp29_85*C21);
    ROcp29_921 = -(ROcp29_66*S21-ROcp29_95*C21);
    RLcp29_121 = ROcp29_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp29_221 = ROcp29_26*s.dpt(1,3)+ROcp29_85*s.dpt(3,3);
    RLcp29_321 = ROcp29_36*s.dpt(1,3)+ROcp29_95*s.dpt(3,3);
    POcp29_121 = RLcp29_121+q(1);
    POcp29_221 = RLcp29_221+q(2);
    POcp29_321 = RLcp29_321+q(3);
    JTcp29_121_5 = -(RLcp29_221*S4-RLcp29_321*C4);
    JTcp29_221_5 = RLcp29_121*S4;
    JTcp29_321_5 = -RLcp29_121*C4;
    JTcp29_121_6 = -(RLcp29_221*ROcp29_95-RLcp29_321*ROcp29_85);
    JTcp29_221_6 = RLcp29_121*ROcp29_95-RLcp29_321*S5;
    JTcp29_321_6 = -(RLcp29_121*ROcp29_85-RLcp29_221*S5);
    OMcp29_121 = OMcp29_16+ROcp29_16*qd(21);
    OMcp29_221 = OMcp29_26+ROcp29_26*qd(21);
    OMcp29_321 = OMcp29_36+ROcp29_36*qd(21);
    ORcp29_121 = OMcp29_26*RLcp29_321-OMcp29_36*RLcp29_221;
    ORcp29_221 = -(OMcp29_16*RLcp29_321-OMcp29_36*RLcp29_121);
    ORcp29_321 = OMcp29_16*RLcp29_221-OMcp29_26*RLcp29_121;
    VIcp29_121 = ORcp29_121+qd(1);
    VIcp29_221 = ORcp29_221+qd(2);
    VIcp29_321 = ORcp29_321+qd(3);
    OPcp29_121 = OPcp29_16+ROcp29_16*qdd(21)+qd(21)*(OMcp29_26*ROcp29_36-OMcp29_36*ROcp29_26);
    OPcp29_221 = OPcp29_26+ROcp29_26*qdd(21)-qd(21)*(OMcp29_16*ROcp29_36-OMcp29_36*ROcp29_16);
    OPcp29_321 = OPcp29_36+ROcp29_36*qdd(21)+qd(21)*(OMcp29_16*ROcp29_26-OMcp29_26*ROcp29_16);
    ACcp29_121 = qdd(1)+OMcp29_26*ORcp29_321-OMcp29_36*ORcp29_221+OPcp29_26*RLcp29_321-OPcp29_36*RLcp29_221;
    ACcp29_221 = qdd(2)-OMcp29_16*ORcp29_321+OMcp29_36*ORcp29_121-OPcp29_16*RLcp29_321+OPcp29_36*RLcp29_121;
    ACcp29_321 = qdd(3)+OMcp29_16*ORcp29_221-OMcp29_26*ORcp29_121+OPcp29_16*RLcp29_221-OPcp29_26*RLcp29_121;

% = = Block_1_0_0_30_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp29_121;
    sens.P(2) = POcp29_221;
    sens.P(3) = POcp29_321;
    sens.R(1,1) = ROcp29_16;
    sens.R(1,2) = ROcp29_26;
    sens.R(1,3) = ROcp29_36;
    sens.R(2,1) = ROcp29_421;
    sens.R(2,2) = ROcp29_521;
    sens.R(2,3) = ROcp29_621;
    sens.R(3,1) = ROcp29_721;
    sens.R(3,2) = ROcp29_821;
    sens.R(3,3) = ROcp29_921;
    sens.V(1) = VIcp29_121;
    sens.V(2) = VIcp29_221;
    sens.V(3) = VIcp29_321;
    sens.OM(1) = OMcp29_121;
    sens.OM(2) = OMcp29_221;
    sens.OM(3) = OMcp29_321;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp29_121_5;
    sens.J(1,6) = JTcp29_121_6;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = -RLcp29_321;
    sens.J(2,5) = JTcp29_221_5;
    sens.J(2,6) = JTcp29_221_6;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = RLcp29_221;
    sens.J(3,5) = JTcp29_321_5;
    sens.J(3,6) = JTcp29_321_6;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,21) = ROcp29_16;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp29_85;
    sens.J(5,21) = ROcp29_26;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp29_95;
    sens.J(6,21) = ROcp29_36;
    sens.A(1) = ACcp29_121;
    sens.A(2) = ACcp29_221;
    sens.A(3) = ACcp29_321;
    sens.OMP(1) = OPcp29_121;
    sens.OMP(2) = OPcp29_221;
    sens.OMP(3) = OPcp29_321;
 
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


    ROcp30_421 = ROcp30_46*C21+S21*S5;
    ROcp30_521 = ROcp30_56*C21+ROcp30_85*S21;
    ROcp30_621 = ROcp30_66*C21+ROcp30_95*S21;
    ROcp30_721 = -(ROcp30_46*S21-C21*S5);
    ROcp30_821 = -(ROcp30_56*S21-ROcp30_85*C21);
    ROcp30_921 = -(ROcp30_66*S21-ROcp30_95*C21);
    RLcp30_121 = ROcp30_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp30_221 = ROcp30_26*s.dpt(1,3)+ROcp30_85*s.dpt(3,3);
    RLcp30_321 = ROcp30_36*s.dpt(1,3)+ROcp30_95*s.dpt(3,3);
    OMcp30_121 = OMcp30_16+ROcp30_16*qd(21);
    OMcp30_221 = OMcp30_26+ROcp30_26*qd(21);
    OMcp30_321 = OMcp30_36+ROcp30_36*qd(21);
    ORcp30_121 = OMcp30_26*RLcp30_321-OMcp30_36*RLcp30_221;
    ORcp30_221 = -(OMcp30_16*RLcp30_321-OMcp30_36*RLcp30_121);
    ORcp30_321 = OMcp30_16*RLcp30_221-OMcp30_26*RLcp30_121;
    OPcp30_121 = OPcp30_16+ROcp30_16*qdd(21)+qd(21)*(OMcp30_26*ROcp30_36-OMcp30_36*ROcp30_26);
    OPcp30_221 = OPcp30_26+ROcp30_26*qdd(21)-qd(21)*(OMcp30_16*ROcp30_36-OMcp30_36*ROcp30_16);
    OPcp30_321 = OPcp30_36+ROcp30_36*qdd(21)+qd(21)*(OMcp30_16*ROcp30_26-OMcp30_26*ROcp30_16);
    RLcp30_162 = ROcp30_16*s.dpt(1,35)+ROcp30_421*s.dpt(2,35)+ROcp30_721*s.dpt(3,35);
    RLcp30_262 = ROcp30_26*s.dpt(1,35)+ROcp30_521*s.dpt(2,35)+ROcp30_821*s.dpt(3,35);
    RLcp30_362 = ROcp30_36*s.dpt(1,35)+ROcp30_621*s.dpt(2,35)+ROcp30_921*s.dpt(3,35);
    POcp30_162 = RLcp30_121+RLcp30_162+q(1);
    POcp30_262 = RLcp30_221+RLcp30_262+q(2);
    POcp30_362 = RLcp30_321+RLcp30_362+q(3);
    JTcp30_262_4 = -(RLcp30_321+RLcp30_362);
    JTcp30_362_4 = RLcp30_221+RLcp30_262;
    JTcp30_162_5 = C4*(RLcp30_321+RLcp30_362)-S4*(RLcp30_221+RLcp30_262);
    JTcp30_262_5 = S4*(RLcp30_121+RLcp30_162);
    JTcp30_362_5 = -C4*(RLcp30_121+RLcp30_162);
    JTcp30_162_6 = ROcp30_85*(RLcp30_321+RLcp30_362)-ROcp30_95*(RLcp30_221+RLcp30_262);
    JTcp30_262_6 = ROcp30_95*(RLcp30_121+RLcp30_162)-S5*(RLcp30_321+RLcp30_362);
    JTcp30_362_6 = -(ROcp30_85*(RLcp30_121+RLcp30_162)-S5*(RLcp30_221+RLcp30_262));
    JTcp30_162_7 = -(RLcp30_262*ROcp30_36-RLcp30_362*ROcp30_26);
    JTcp30_262_7 = RLcp30_162*ROcp30_36-RLcp30_362*ROcp30_16;
    JTcp30_362_7 = -(RLcp30_162*ROcp30_26-RLcp30_262*ROcp30_16);
    ORcp30_162 = OMcp30_221*RLcp30_362-OMcp30_321*RLcp30_262;
    ORcp30_262 = -(OMcp30_121*RLcp30_362-OMcp30_321*RLcp30_162);
    ORcp30_362 = OMcp30_121*RLcp30_262-OMcp30_221*RLcp30_162;
    VIcp30_162 = ORcp30_121+ORcp30_162+qd(1);
    VIcp30_262 = ORcp30_221+ORcp30_262+qd(2);
    VIcp30_362 = ORcp30_321+ORcp30_362+qd(3);
    ACcp30_162 = qdd(1)+OMcp30_221*ORcp30_362+OMcp30_26*ORcp30_321-OMcp30_321*ORcp30_262-OMcp30_36*ORcp30_221+OPcp30_221*RLcp30_362+OPcp30_26*...
 RLcp30_321-OPcp30_321*RLcp30_262-OPcp30_36*RLcp30_221;
    ACcp30_262 = qdd(2)-OMcp30_121*ORcp30_362-OMcp30_16*ORcp30_321+OMcp30_321*ORcp30_162+OMcp30_36*ORcp30_121-OPcp30_121*RLcp30_362-OPcp30_16*...
 RLcp30_321+OPcp30_321*RLcp30_162+OPcp30_36*RLcp30_121;
    ACcp30_362 = qdd(3)+OMcp30_121*ORcp30_262+OMcp30_16*ORcp30_221-OMcp30_221*ORcp30_162-OMcp30_26*ORcp30_121+OPcp30_121*RLcp30_262+OPcp30_16*...
 RLcp30_221-OPcp30_221*RLcp30_162-OPcp30_26*RLcp30_121;

% = = Block_1_0_0_31_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp30_162;
    sens.P(2) = POcp30_262;
    sens.P(3) = POcp30_362;
    sens.R(1,1) = ROcp30_16;
    sens.R(1,2) = ROcp30_26;
    sens.R(1,3) = ROcp30_36;
    sens.R(2,1) = ROcp30_421;
    sens.R(2,2) = ROcp30_521;
    sens.R(2,3) = ROcp30_621;
    sens.R(3,1) = ROcp30_721;
    sens.R(3,2) = ROcp30_821;
    sens.R(3,3) = ROcp30_921;
    sens.V(1) = VIcp30_162;
    sens.V(2) = VIcp30_262;
    sens.V(3) = VIcp30_362;
    sens.OM(1) = OMcp30_121;
    sens.OM(2) = OMcp30_221;
    sens.OM(3) = OMcp30_321;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp30_162_5;
    sens.J(1,6) = JTcp30_162_6;
    sens.J(1,21) = JTcp30_162_7;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp30_262_4;
    sens.J(2,5) = JTcp30_262_5;
    sens.J(2,6) = JTcp30_262_6;
    sens.J(2,21) = JTcp30_262_7;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp30_362_4;
    sens.J(3,5) = JTcp30_362_5;
    sens.J(3,6) = JTcp30_362_6;
    sens.J(3,21) = JTcp30_362_7;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,21) = ROcp30_16;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp30_85;
    sens.J(5,21) = ROcp30_26;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp30_95;
    sens.J(6,21) = ROcp30_36;
    sens.A(1) = ACcp30_162;
    sens.A(2) = ACcp30_262;
    sens.A(3) = ACcp30_362;
    sens.OMP(1) = OPcp30_121;
    sens.OMP(2) = OPcp30_221;
    sens.OMP(3) = OPcp30_321;
 
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


    ROcp31_421 = ROcp31_46*C21+S21*S5;
    ROcp31_521 = ROcp31_56*C21+ROcp31_85*S21;
    ROcp31_621 = ROcp31_66*C21+ROcp31_95*S21;
    ROcp31_721 = -(ROcp31_46*S21-C21*S5);
    ROcp31_821 = -(ROcp31_56*S21-ROcp31_85*C21);
    ROcp31_921 = -(ROcp31_66*S21-ROcp31_95*C21);
    RLcp31_121 = ROcp31_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp31_221 = ROcp31_26*s.dpt(1,3)+ROcp31_85*s.dpt(3,3);
    RLcp31_321 = ROcp31_36*s.dpt(1,3)+ROcp31_95*s.dpt(3,3);
    POcp31_121 = RLcp31_121+q(1);
    POcp31_221 = RLcp31_221+q(2);
    POcp31_321 = RLcp31_321+q(3);
    OMcp31_121 = OMcp31_16+ROcp31_16*qd(21);
    OMcp31_221 = OMcp31_26+ROcp31_26*qd(21);
    OMcp31_321 = OMcp31_36+ROcp31_36*qd(21);
    ORcp31_121 = OMcp31_26*RLcp31_321-OMcp31_36*RLcp31_221;
    ORcp31_221 = -(OMcp31_16*RLcp31_321-OMcp31_36*RLcp31_121);
    ORcp31_321 = OMcp31_16*RLcp31_221-OMcp31_26*RLcp31_121;
    VIcp31_121 = ORcp31_121+qd(1);
    VIcp31_221 = ORcp31_221+qd(2);
    VIcp31_321 = ORcp31_321+qd(3);
    OPcp31_121 = OPcp31_16+ROcp31_16*qdd(21)+qd(21)*(OMcp31_26*ROcp31_36-OMcp31_36*ROcp31_26);
    OPcp31_221 = OPcp31_26+ROcp31_26*qdd(21)-qd(21)*(OMcp31_16*ROcp31_36-OMcp31_36*ROcp31_16);
    OPcp31_321 = OPcp31_36+ROcp31_36*qdd(21)+qd(21)*(OMcp31_16*ROcp31_26-OMcp31_26*ROcp31_16);
    ACcp31_121 = qdd(1)+OMcp31_26*ORcp31_321-OMcp31_36*ORcp31_221+OPcp31_26*RLcp31_321-OPcp31_36*RLcp31_221;
    ACcp31_221 = qdd(2)-OMcp31_16*ORcp31_321+OMcp31_36*ORcp31_121-OPcp31_16*RLcp31_321+OPcp31_36*RLcp31_121;
    ACcp31_321 = qdd(3)+OMcp31_16*ORcp31_221-OMcp31_26*ORcp31_121+OPcp31_16*RLcp31_221-OPcp31_26*RLcp31_121;

% = = Block_1_0_0_32_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp31_121;
    sens.P(2) = POcp31_221;
    sens.P(3) = POcp31_321;
    sens.R(1,1) = ROcp31_16;
    sens.R(1,2) = ROcp31_26;
    sens.R(1,3) = ROcp31_36;
    sens.R(2,1) = ROcp31_421;
    sens.R(2,2) = ROcp31_521;
    sens.R(2,3) = ROcp31_621;
    sens.R(3,1) = ROcp31_721;
    sens.R(3,2) = ROcp31_821;
    sens.R(3,3) = ROcp31_921;
    sens.V(1) = VIcp31_121;
    sens.V(2) = VIcp31_221;
    sens.V(3) = VIcp31_321;
    sens.OM(1) = OMcp31_121;
    sens.OM(2) = OMcp31_221;
    sens.OM(3) = OMcp31_321;
    sens.A(1) = ACcp31_121;
    sens.A(2) = ACcp31_221;
    sens.A(3) = ACcp31_321;
    sens.OMP(1) = OPcp31_121;
    sens.OMP(2) = OPcp31_221;
    sens.OMP(3) = OPcp31_321;
 
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


    ROcp32_421 = ROcp32_46*C21+S21*S5;
    ROcp32_521 = ROcp32_56*C21+ROcp32_85*S21;
    ROcp32_621 = ROcp32_66*C21+ROcp32_95*S21;
    ROcp32_721 = -(ROcp32_46*S21-C21*S5);
    ROcp32_821 = -(ROcp32_56*S21-ROcp32_85*C21);
    ROcp32_921 = -(ROcp32_66*S21-ROcp32_95*C21);
    ROcp32_122 = ROcp32_16*C22-ROcp32_721*S22;
    ROcp32_222 = ROcp32_26*C22-ROcp32_821*S22;
    ROcp32_322 = ROcp32_36*C22-ROcp32_921*S22;
    ROcp32_722 = ROcp32_16*S22+ROcp32_721*C22;
    ROcp32_822 = ROcp32_26*S22+ROcp32_821*C22;
    ROcp32_922 = ROcp32_36*S22+ROcp32_921*C22;
    RLcp32_121 = ROcp32_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp32_221 = ROcp32_26*s.dpt(1,3)+ROcp32_85*s.dpt(3,3);
    RLcp32_321 = ROcp32_36*s.dpt(1,3)+ROcp32_95*s.dpt(3,3);
    OMcp32_121 = OMcp32_16+ROcp32_16*qd(21);
    OMcp32_221 = OMcp32_26+ROcp32_26*qd(21);
    OMcp32_321 = OMcp32_36+ROcp32_36*qd(21);
    ORcp32_121 = OMcp32_26*RLcp32_321-OMcp32_36*RLcp32_221;
    ORcp32_221 = -(OMcp32_16*RLcp32_321-OMcp32_36*RLcp32_121);
    ORcp32_321 = OMcp32_16*RLcp32_221-OMcp32_26*RLcp32_121;
    OMcp32_122 = OMcp32_121+ROcp32_421*qd(22);
    OMcp32_222 = OMcp32_221+ROcp32_521*qd(22);
    OMcp32_322 = OMcp32_321+ROcp32_621*qd(22);
    OPcp32_122 = OPcp32_16+ROcp32_16*qdd(21)+ROcp32_421*qdd(22)+qd(21)*(OMcp32_26*ROcp32_36-OMcp32_36*ROcp32_26)+qd(22)*(OMcp32_221*ROcp32_621-...
 OMcp32_321*ROcp32_521);
    OPcp32_222 = OPcp32_26+ROcp32_26*qdd(21)+ROcp32_521*qdd(22)-qd(21)*(OMcp32_16*ROcp32_36-OMcp32_36*ROcp32_16)-qd(22)*(OMcp32_121*ROcp32_621-...
 OMcp32_321*ROcp32_421);
    OPcp32_322 = OPcp32_36+ROcp32_36*qdd(21)+ROcp32_621*qdd(22)+qd(21)*(OMcp32_16*ROcp32_26-OMcp32_26*ROcp32_16)+qd(22)*(OMcp32_121*ROcp32_521-...
 OMcp32_221*ROcp32_421);
    RLcp32_164 = ROcp32_722*s.dpt(3,37);
    RLcp32_264 = ROcp32_822*s.dpt(3,37);
    RLcp32_364 = ROcp32_922*s.dpt(3,37);
    POcp32_164 = RLcp32_121+RLcp32_164+q(1);
    POcp32_264 = RLcp32_221+RLcp32_264+q(2);
    POcp32_364 = RLcp32_321+RLcp32_364+q(3);
    JTcp32_264_4 = -(RLcp32_321+RLcp32_364);
    JTcp32_364_4 = RLcp32_221+RLcp32_264;
    JTcp32_164_5 = C4*(RLcp32_321+RLcp32_364)-S4*(RLcp32_221+RLcp32_264);
    JTcp32_264_5 = S4*(RLcp32_121+RLcp32_164);
    JTcp32_364_5 = -C4*(RLcp32_121+RLcp32_164);
    JTcp32_164_6 = ROcp32_85*(RLcp32_321+RLcp32_364)-ROcp32_95*(RLcp32_221+RLcp32_264);
    JTcp32_264_6 = ROcp32_95*(RLcp32_121+RLcp32_164)-S5*(RLcp32_321+RLcp32_364);
    JTcp32_364_6 = -(ROcp32_85*(RLcp32_121+RLcp32_164)-S5*(RLcp32_221+RLcp32_264));
    JTcp32_164_7 = -(RLcp32_264*ROcp32_36-RLcp32_364*ROcp32_26);
    JTcp32_264_7 = RLcp32_164*ROcp32_36-RLcp32_364*ROcp32_16;
    JTcp32_364_7 = -(RLcp32_164*ROcp32_26-RLcp32_264*ROcp32_16);
    JTcp32_164_8 = -(RLcp32_264*ROcp32_621-RLcp32_364*ROcp32_521);
    JTcp32_264_8 = RLcp32_164*ROcp32_621-RLcp32_364*ROcp32_421;
    JTcp32_364_8 = -(RLcp32_164*ROcp32_521-RLcp32_264*ROcp32_421);
    ORcp32_164 = OMcp32_222*RLcp32_364-OMcp32_322*RLcp32_264;
    ORcp32_264 = -(OMcp32_122*RLcp32_364-OMcp32_322*RLcp32_164);
    ORcp32_364 = OMcp32_122*RLcp32_264-OMcp32_222*RLcp32_164;
    VIcp32_164 = ORcp32_121+ORcp32_164+qd(1);
    VIcp32_264 = ORcp32_221+ORcp32_264+qd(2);
    VIcp32_364 = ORcp32_321+ORcp32_364+qd(3);
    ACcp32_164 = qdd(1)+OMcp32_222*ORcp32_364+OMcp32_26*ORcp32_321-OMcp32_322*ORcp32_264-OMcp32_36*ORcp32_221+OPcp32_222*RLcp32_364+OPcp32_26*...
 RLcp32_321-OPcp32_322*RLcp32_264-OPcp32_36*RLcp32_221;
    ACcp32_264 = qdd(2)-OMcp32_122*ORcp32_364-OMcp32_16*ORcp32_321+OMcp32_322*ORcp32_164+OMcp32_36*ORcp32_121-OPcp32_122*RLcp32_364-OPcp32_16*...
 RLcp32_321+OPcp32_322*RLcp32_164+OPcp32_36*RLcp32_121;
    ACcp32_364 = qdd(3)+OMcp32_122*ORcp32_264+OMcp32_16*ORcp32_221-OMcp32_222*ORcp32_164-OMcp32_26*ORcp32_121+OPcp32_122*RLcp32_264+OPcp32_16*...
 RLcp32_221-OPcp32_222*RLcp32_164-OPcp32_26*RLcp32_121;

% = = Block_1_0_0_33_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp32_164;
    sens.P(2) = POcp32_264;
    sens.P(3) = POcp32_364;
    sens.R(1,1) = ROcp32_122;
    sens.R(1,2) = ROcp32_222;
    sens.R(1,3) = ROcp32_322;
    sens.R(2,1) = ROcp32_421;
    sens.R(2,2) = ROcp32_521;
    sens.R(2,3) = ROcp32_621;
    sens.R(3,1) = ROcp32_722;
    sens.R(3,2) = ROcp32_822;
    sens.R(3,3) = ROcp32_922;
    sens.V(1) = VIcp32_164;
    sens.V(2) = VIcp32_264;
    sens.V(3) = VIcp32_364;
    sens.OM(1) = OMcp32_122;
    sens.OM(2) = OMcp32_222;
    sens.OM(3) = OMcp32_322;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp32_164_5;
    sens.J(1,6) = JTcp32_164_6;
    sens.J(1,21) = JTcp32_164_7;
    sens.J(1,22) = JTcp32_164_8;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp32_264_4;
    sens.J(2,5) = JTcp32_264_5;
    sens.J(2,6) = JTcp32_264_6;
    sens.J(2,21) = JTcp32_264_7;
    sens.J(2,22) = JTcp32_264_8;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp32_364_4;
    sens.J(3,5) = JTcp32_364_5;
    sens.J(3,6) = JTcp32_364_6;
    sens.J(3,21) = JTcp32_364_7;
    sens.J(3,22) = JTcp32_364_8;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,21) = ROcp32_16;
    sens.J(4,22) = ROcp32_421;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp32_85;
    sens.J(5,21) = ROcp32_26;
    sens.J(5,22) = ROcp32_521;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp32_95;
    sens.J(6,21) = ROcp32_36;
    sens.J(6,22) = ROcp32_621;
    sens.A(1) = ACcp32_164;
    sens.A(2) = ACcp32_264;
    sens.A(3) = ACcp32_364;
    sens.OMP(1) = OPcp32_122;
    sens.OMP(2) = OPcp32_222;
    sens.OMP(3) = OPcp32_322;
 
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


    ROcp33_421 = ROcp33_46*C21+S21*S5;
    ROcp33_521 = ROcp33_56*C21+ROcp33_85*S21;
    ROcp33_621 = ROcp33_66*C21+ROcp33_95*S21;
    ROcp33_721 = -(ROcp33_46*S21-C21*S5);
    ROcp33_821 = -(ROcp33_56*S21-ROcp33_85*C21);
    ROcp33_921 = -(ROcp33_66*S21-ROcp33_95*C21);
    ROcp33_122 = ROcp33_16*C22-ROcp33_721*S22;
    ROcp33_222 = ROcp33_26*C22-ROcp33_821*S22;
    ROcp33_322 = ROcp33_36*C22-ROcp33_921*S22;
    ROcp33_722 = ROcp33_16*S22+ROcp33_721*C22;
    ROcp33_822 = ROcp33_26*S22+ROcp33_821*C22;
    ROcp33_922 = ROcp33_36*S22+ROcp33_921*C22;
    RLcp33_121 = ROcp33_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp33_221 = ROcp33_26*s.dpt(1,3)+ROcp33_85*s.dpt(3,3);
    RLcp33_321 = ROcp33_36*s.dpt(1,3)+ROcp33_95*s.dpt(3,3);
    OMcp33_121 = OMcp33_16+ROcp33_16*qd(21);
    OMcp33_221 = OMcp33_26+ROcp33_26*qd(21);
    OMcp33_321 = OMcp33_36+ROcp33_36*qd(21);
    ORcp33_121 = OMcp33_26*RLcp33_321-OMcp33_36*RLcp33_221;
    ORcp33_221 = -(OMcp33_16*RLcp33_321-OMcp33_36*RLcp33_121);
    ORcp33_321 = OMcp33_16*RLcp33_221-OMcp33_26*RLcp33_121;
    OMcp33_122 = OMcp33_121+ROcp33_421*qd(22);
    OMcp33_222 = OMcp33_221+ROcp33_521*qd(22);
    OMcp33_322 = OMcp33_321+ROcp33_621*qd(22);
    OPcp33_122 = OPcp33_16+ROcp33_16*qdd(21)+ROcp33_421*qdd(22)+qd(21)*(OMcp33_26*ROcp33_36-OMcp33_36*ROcp33_26)+qd(22)*(OMcp33_221*ROcp33_621-...
 OMcp33_321*ROcp33_521);
    OPcp33_222 = OPcp33_26+ROcp33_26*qdd(21)+ROcp33_521*qdd(22)-qd(21)*(OMcp33_16*ROcp33_36-OMcp33_36*ROcp33_16)-qd(22)*(OMcp33_121*ROcp33_621-...
 OMcp33_321*ROcp33_421);
    OPcp33_322 = OPcp33_36+ROcp33_36*qdd(21)+ROcp33_621*qdd(22)+qd(21)*(OMcp33_16*ROcp33_26-OMcp33_26*ROcp33_16)+qd(22)*(OMcp33_121*ROcp33_521-...
 OMcp33_221*ROcp33_421);
    RLcp33_165 = ROcp33_122*s.dpt(1,38)+ROcp33_421*s.dpt(2,38)+ROcp33_722*s.dpt(3,38);
    RLcp33_265 = ROcp33_222*s.dpt(1,38)+ROcp33_521*s.dpt(2,38)+ROcp33_822*s.dpt(3,38);
    RLcp33_365 = ROcp33_322*s.dpt(1,38)+ROcp33_621*s.dpt(2,38)+ROcp33_922*s.dpt(3,38);
    POcp33_165 = RLcp33_121+RLcp33_165+q(1);
    POcp33_265 = RLcp33_221+RLcp33_265+q(2);
    POcp33_365 = RLcp33_321+RLcp33_365+q(3);
    JTcp33_265_4 = -(RLcp33_321+RLcp33_365);
    JTcp33_365_4 = RLcp33_221+RLcp33_265;
    JTcp33_165_5 = C4*(RLcp33_321+RLcp33_365)-S4*(RLcp33_221+RLcp33_265);
    JTcp33_265_5 = S4*(RLcp33_121+RLcp33_165);
    JTcp33_365_5 = -C4*(RLcp33_121+RLcp33_165);
    JTcp33_165_6 = ROcp33_85*(RLcp33_321+RLcp33_365)-ROcp33_95*(RLcp33_221+RLcp33_265);
    JTcp33_265_6 = ROcp33_95*(RLcp33_121+RLcp33_165)-S5*(RLcp33_321+RLcp33_365);
    JTcp33_365_6 = -(ROcp33_85*(RLcp33_121+RLcp33_165)-S5*(RLcp33_221+RLcp33_265));
    JTcp33_165_7 = -(RLcp33_265*ROcp33_36-RLcp33_365*ROcp33_26);
    JTcp33_265_7 = RLcp33_165*ROcp33_36-RLcp33_365*ROcp33_16;
    JTcp33_365_7 = -(RLcp33_165*ROcp33_26-RLcp33_265*ROcp33_16);
    JTcp33_165_8 = -(RLcp33_265*ROcp33_621-RLcp33_365*ROcp33_521);
    JTcp33_265_8 = RLcp33_165*ROcp33_621-RLcp33_365*ROcp33_421;
    JTcp33_365_8 = -(RLcp33_165*ROcp33_521-RLcp33_265*ROcp33_421);
    ORcp33_165 = OMcp33_222*RLcp33_365-OMcp33_322*RLcp33_265;
    ORcp33_265 = -(OMcp33_122*RLcp33_365-OMcp33_322*RLcp33_165);
    ORcp33_365 = OMcp33_122*RLcp33_265-OMcp33_222*RLcp33_165;
    VIcp33_165 = ORcp33_121+ORcp33_165+qd(1);
    VIcp33_265 = ORcp33_221+ORcp33_265+qd(2);
    VIcp33_365 = ORcp33_321+ORcp33_365+qd(3);
    ACcp33_165 = qdd(1)+OMcp33_222*ORcp33_365+OMcp33_26*ORcp33_321-OMcp33_322*ORcp33_265-OMcp33_36*ORcp33_221+OPcp33_222*RLcp33_365+OPcp33_26*...
 RLcp33_321-OPcp33_322*RLcp33_265-OPcp33_36*RLcp33_221;
    ACcp33_265 = qdd(2)-OMcp33_122*ORcp33_365-OMcp33_16*ORcp33_321+OMcp33_322*ORcp33_165+OMcp33_36*ORcp33_121-OPcp33_122*RLcp33_365-OPcp33_16*...
 RLcp33_321+OPcp33_322*RLcp33_165+OPcp33_36*RLcp33_121;
    ACcp33_365 = qdd(3)+OMcp33_122*ORcp33_265+OMcp33_16*ORcp33_221-OMcp33_222*ORcp33_165-OMcp33_26*ORcp33_121+OPcp33_122*RLcp33_265+OPcp33_16*...
 RLcp33_221-OPcp33_222*RLcp33_165-OPcp33_26*RLcp33_121;

% = = Block_1_0_0_34_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp33_165;
    sens.P(2) = POcp33_265;
    sens.P(3) = POcp33_365;
    sens.R(1,1) = ROcp33_122;
    sens.R(1,2) = ROcp33_222;
    sens.R(1,3) = ROcp33_322;
    sens.R(2,1) = ROcp33_421;
    sens.R(2,2) = ROcp33_521;
    sens.R(2,3) = ROcp33_621;
    sens.R(3,1) = ROcp33_722;
    sens.R(3,2) = ROcp33_822;
    sens.R(3,3) = ROcp33_922;
    sens.V(1) = VIcp33_165;
    sens.V(2) = VIcp33_265;
    sens.V(3) = VIcp33_365;
    sens.OM(1) = OMcp33_122;
    sens.OM(2) = OMcp33_222;
    sens.OM(3) = OMcp33_322;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp33_165_5;
    sens.J(1,6) = JTcp33_165_6;
    sens.J(1,21) = JTcp33_165_7;
    sens.J(1,22) = JTcp33_165_8;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp33_265_4;
    sens.J(2,5) = JTcp33_265_5;
    sens.J(2,6) = JTcp33_265_6;
    sens.J(2,21) = JTcp33_265_7;
    sens.J(2,22) = JTcp33_265_8;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp33_365_4;
    sens.J(3,5) = JTcp33_365_5;
    sens.J(3,6) = JTcp33_365_6;
    sens.J(3,21) = JTcp33_365_7;
    sens.J(3,22) = JTcp33_365_8;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,21) = ROcp33_16;
    sens.J(4,22) = ROcp33_421;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp33_85;
    sens.J(5,21) = ROcp33_26;
    sens.J(5,22) = ROcp33_521;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp33_95;
    sens.J(6,21) = ROcp33_36;
    sens.J(6,22) = ROcp33_621;
    sens.A(1) = ACcp33_165;
    sens.A(2) = ACcp33_265;
    sens.A(3) = ACcp33_365;
    sens.OMP(1) = OPcp33_122;
    sens.OMP(2) = OPcp33_222;
    sens.OMP(3) = OPcp33_322;
 
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


    ROcp34_421 = ROcp34_46*C21+S21*S5;
    ROcp34_521 = ROcp34_56*C21+ROcp34_85*S21;
    ROcp34_621 = ROcp34_66*C21+ROcp34_95*S21;
    ROcp34_721 = -(ROcp34_46*S21-C21*S5);
    ROcp34_821 = -(ROcp34_56*S21-ROcp34_85*C21);
    ROcp34_921 = -(ROcp34_66*S21-ROcp34_95*C21);
    ROcp34_122 = ROcp34_16*C22-ROcp34_721*S22;
    ROcp34_222 = ROcp34_26*C22-ROcp34_821*S22;
    ROcp34_322 = ROcp34_36*C22-ROcp34_921*S22;
    ROcp34_722 = ROcp34_16*S22+ROcp34_721*C22;
    ROcp34_822 = ROcp34_26*S22+ROcp34_821*C22;
    ROcp34_922 = ROcp34_36*S22+ROcp34_921*C22;
    ROcp34_123 = ROcp34_122*C23+ROcp34_421*S23;
    ROcp34_223 = ROcp34_222*C23+ROcp34_521*S23;
    ROcp34_323 = ROcp34_322*C23+ROcp34_621*S23;
    ROcp34_423 = -(ROcp34_122*S23-ROcp34_421*C23);
    ROcp34_523 = -(ROcp34_222*S23-ROcp34_521*C23);
    ROcp34_623 = -(ROcp34_322*S23-ROcp34_621*C23);
    RLcp34_121 = ROcp34_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp34_221 = ROcp34_26*s.dpt(1,3)+ROcp34_85*s.dpt(3,3);
    RLcp34_321 = ROcp34_36*s.dpt(1,3)+ROcp34_95*s.dpt(3,3);
    OMcp34_121 = OMcp34_16+ROcp34_16*qd(21);
    OMcp34_221 = OMcp34_26+ROcp34_26*qd(21);
    OMcp34_321 = OMcp34_36+ROcp34_36*qd(21);
    ORcp34_121 = OMcp34_26*RLcp34_321-OMcp34_36*RLcp34_221;
    ORcp34_221 = -(OMcp34_16*RLcp34_321-OMcp34_36*RLcp34_121);
    ORcp34_321 = OMcp34_16*RLcp34_221-OMcp34_26*RLcp34_121;
    OMcp34_122 = OMcp34_121+ROcp34_421*qd(22);
    OMcp34_222 = OMcp34_221+ROcp34_521*qd(22);
    OMcp34_322 = OMcp34_321+ROcp34_621*qd(22);
    OPcp34_122 = OPcp34_16+ROcp34_16*qdd(21)+ROcp34_421*qdd(22)+qd(21)*(OMcp34_26*ROcp34_36-OMcp34_36*ROcp34_26)+qd(22)*(OMcp34_221*ROcp34_621-...
 OMcp34_321*ROcp34_521);
    OPcp34_222 = OPcp34_26+ROcp34_26*qdd(21)+ROcp34_521*qdd(22)-qd(21)*(OMcp34_16*ROcp34_36-OMcp34_36*ROcp34_16)-qd(22)*(OMcp34_121*ROcp34_621-...
 OMcp34_321*ROcp34_421);
    OPcp34_322 = OPcp34_36+ROcp34_36*qdd(21)+ROcp34_621*qdd(22)+qd(21)*(OMcp34_16*ROcp34_26-OMcp34_26*ROcp34_16)+qd(22)*(OMcp34_121*ROcp34_521-...
 OMcp34_221*ROcp34_421);
    RLcp34_123 = ROcp34_722*s.dpt(3,37);
    RLcp34_223 = ROcp34_822*s.dpt(3,37);
    RLcp34_323 = ROcp34_922*s.dpt(3,37);
    OMcp34_123 = OMcp34_122+ROcp34_722*qd(23);
    OMcp34_223 = OMcp34_222+ROcp34_822*qd(23);
    OMcp34_323 = OMcp34_322+ROcp34_922*qd(23);
    ORcp34_123 = OMcp34_222*RLcp34_323-OMcp34_322*RLcp34_223;
    ORcp34_223 = -(OMcp34_122*RLcp34_323-OMcp34_322*RLcp34_123);
    ORcp34_323 = OMcp34_122*RLcp34_223-OMcp34_222*RLcp34_123;
    OPcp34_123 = OPcp34_122+ROcp34_722*qdd(23)+qd(23)*(OMcp34_222*ROcp34_922-OMcp34_322*ROcp34_822);
    OPcp34_223 = OPcp34_222+ROcp34_822*qdd(23)-qd(23)*(OMcp34_122*ROcp34_922-OMcp34_322*ROcp34_722);
    OPcp34_323 = OPcp34_322+ROcp34_922*qdd(23)+qd(23)*(OMcp34_122*ROcp34_822-OMcp34_222*ROcp34_722);
    RLcp34_166 = ROcp34_722*s.dpt(3,39);
    RLcp34_266 = ROcp34_822*s.dpt(3,39);
    RLcp34_366 = ROcp34_922*s.dpt(3,39);
    POcp34_166 = RLcp34_121+RLcp34_123+RLcp34_166+q(1);
    POcp34_266 = RLcp34_221+RLcp34_223+RLcp34_266+q(2);
    POcp34_366 = RLcp34_321+RLcp34_323+RLcp34_366+q(3);
    JTcp34_266_4 = -(RLcp34_321+RLcp34_323+RLcp34_366);
    JTcp34_366_4 = RLcp34_221+RLcp34_223+RLcp34_266;
    JTcp34_166_5 = C4*(RLcp34_321+RLcp34_323)-S4*(RLcp34_221+RLcp34_223)-RLcp34_266*S4+RLcp34_366*C4;
    JTcp34_266_5 = S4*(RLcp34_121+RLcp34_123+RLcp34_166);
    JTcp34_366_5 = -C4*(RLcp34_121+RLcp34_123+RLcp34_166);
    JTcp34_166_6 = ROcp34_85*(RLcp34_321+RLcp34_323)-ROcp34_95*(RLcp34_221+RLcp34_223)-RLcp34_266*ROcp34_95+RLcp34_366*ROcp34_85;
    JTcp34_266_6 = -(RLcp34_366*S5-ROcp34_95*(RLcp34_121+RLcp34_123+RLcp34_166)+S5*(RLcp34_321+RLcp34_323));
    JTcp34_366_6 = RLcp34_266*S5-ROcp34_85*(RLcp34_121+RLcp34_123+RLcp34_166)+S5*(RLcp34_221+RLcp34_223);
    JTcp34_166_7 = ROcp34_26*(RLcp34_323+RLcp34_366)-ROcp34_36*(RLcp34_223+RLcp34_266);
    JTcp34_266_7 = -(ROcp34_16*(RLcp34_323+RLcp34_366)-ROcp34_36*(RLcp34_123+RLcp34_166));
    JTcp34_366_7 = ROcp34_16*(RLcp34_223+RLcp34_266)-ROcp34_26*(RLcp34_123+RLcp34_166);
    JTcp34_166_8 = ROcp34_521*(RLcp34_323+RLcp34_366)-ROcp34_621*(RLcp34_223+RLcp34_266);
    JTcp34_266_8 = -(ROcp34_421*(RLcp34_323+RLcp34_366)-ROcp34_621*(RLcp34_123+RLcp34_166));
    JTcp34_366_8 = ROcp34_421*(RLcp34_223+RLcp34_266)-ROcp34_521*(RLcp34_123+RLcp34_166);
    JTcp34_166_9 = -(RLcp34_266*ROcp34_922-RLcp34_366*ROcp34_822);
    JTcp34_266_9 = RLcp34_166*ROcp34_922-RLcp34_366*ROcp34_722;
    JTcp34_366_9 = -(RLcp34_166*ROcp34_822-RLcp34_266*ROcp34_722);
    ORcp34_166 = OMcp34_223*RLcp34_366-OMcp34_323*RLcp34_266;
    ORcp34_266 = -(OMcp34_123*RLcp34_366-OMcp34_323*RLcp34_166);
    ORcp34_366 = OMcp34_123*RLcp34_266-OMcp34_223*RLcp34_166;
    VIcp34_166 = ORcp34_121+ORcp34_123+ORcp34_166+qd(1);
    VIcp34_266 = ORcp34_221+ORcp34_223+ORcp34_266+qd(2);
    VIcp34_366 = ORcp34_321+ORcp34_323+ORcp34_366+qd(3);
    ACcp34_166 = qdd(1)+OMcp34_222*ORcp34_323+OMcp34_223*ORcp34_366+OMcp34_26*ORcp34_321-OMcp34_322*ORcp34_223-OMcp34_323*ORcp34_266-OMcp34_36*...
 ORcp34_221+OPcp34_222*RLcp34_323+OPcp34_223*RLcp34_366+OPcp34_26*RLcp34_321-OPcp34_322*RLcp34_223-OPcp34_323*RLcp34_266-OPcp34_36*RLcp34_221;
    ACcp34_266 = qdd(2)-OMcp34_122*ORcp34_323-OMcp34_123*ORcp34_366-OMcp34_16*ORcp34_321+OMcp34_322*ORcp34_123+OMcp34_323*ORcp34_166+OMcp34_36*...
 ORcp34_121-OPcp34_122*RLcp34_323-OPcp34_123*RLcp34_366-OPcp34_16*RLcp34_321+OPcp34_322*RLcp34_123+OPcp34_323*RLcp34_166+OPcp34_36*RLcp34_121;
    ACcp34_366 = qdd(3)+OMcp34_122*ORcp34_223+OMcp34_123*ORcp34_266+OMcp34_16*ORcp34_221-OMcp34_222*ORcp34_123-OMcp34_223*ORcp34_166-OMcp34_26*...
 ORcp34_121+OPcp34_122*RLcp34_223+OPcp34_123*RLcp34_266+OPcp34_16*RLcp34_221-OPcp34_222*RLcp34_123-OPcp34_223*RLcp34_166-OPcp34_26*RLcp34_121;

% = = Block_1_0_0_35_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp34_166;
    sens.P(2) = POcp34_266;
    sens.P(3) = POcp34_366;
    sens.R(1,1) = ROcp34_123;
    sens.R(1,2) = ROcp34_223;
    sens.R(1,3) = ROcp34_323;
    sens.R(2,1) = ROcp34_423;
    sens.R(2,2) = ROcp34_523;
    sens.R(2,3) = ROcp34_623;
    sens.R(3,1) = ROcp34_722;
    sens.R(3,2) = ROcp34_822;
    sens.R(3,3) = ROcp34_922;
    sens.V(1) = VIcp34_166;
    sens.V(2) = VIcp34_266;
    sens.V(3) = VIcp34_366;
    sens.OM(1) = OMcp34_123;
    sens.OM(2) = OMcp34_223;
    sens.OM(3) = OMcp34_323;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp34_166_5;
    sens.J(1,6) = JTcp34_166_6;
    sens.J(1,21) = JTcp34_166_7;
    sens.J(1,22) = JTcp34_166_8;
    sens.J(1,23) = JTcp34_166_9;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp34_266_4;
    sens.J(2,5) = JTcp34_266_5;
    sens.J(2,6) = JTcp34_266_6;
    sens.J(2,21) = JTcp34_266_7;
    sens.J(2,22) = JTcp34_266_8;
    sens.J(2,23) = JTcp34_266_9;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp34_366_4;
    sens.J(3,5) = JTcp34_366_5;
    sens.J(3,6) = JTcp34_366_6;
    sens.J(3,21) = JTcp34_366_7;
    sens.J(3,22) = JTcp34_366_8;
    sens.J(3,23) = JTcp34_366_9;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,21) = ROcp34_16;
    sens.J(4,22) = ROcp34_421;
    sens.J(4,23) = ROcp34_722;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp34_85;
    sens.J(5,21) = ROcp34_26;
    sens.J(5,22) = ROcp34_521;
    sens.J(5,23) = ROcp34_822;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp34_95;
    sens.J(6,21) = ROcp34_36;
    sens.J(6,22) = ROcp34_621;
    sens.J(6,23) = ROcp34_922;
    sens.A(1) = ACcp34_166;
    sens.A(2) = ACcp34_266;
    sens.A(3) = ACcp34_366;
    sens.OMP(1) = OPcp34_123;
    sens.OMP(2) = OPcp34_223;
    sens.OMP(3) = OPcp34_323;
 
% 
case 36, 


% = = Block_1_0_0_36_0_1 = = 
 
% Sensor Kinematics 


    ROcp35_25 = S4*S5;
    ROcp35_35 = -C4*S5;
    ROcp35_85 = -S4*C5;
    ROcp35_95 = C4*C5;
    ROcp35_16 = C5*C6;
    ROcp35_26 = ROcp35_25*C6+C4*S6;
    ROcp35_36 = ROcp35_35*C6+S4*S6;
    ROcp35_46 = -C5*S6;
    ROcp35_56 = -(ROcp35_25*S6-C4*C6);
    ROcp35_66 = -(ROcp35_35*S6-S4*C6);
    OMcp35_25 = qd(5)*C4;
    OMcp35_35 = qd(5)*S4;
    OMcp35_16 = qd(4)+qd(6)*S5;
    OMcp35_26 = OMcp35_25+ROcp35_85*qd(6);
    OMcp35_36 = OMcp35_35+ROcp35_95*qd(6);
    OPcp35_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp35_26 = ROcp35_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp35_35*S5-ROcp35_95*qd(4));
    OPcp35_36 = ROcp35_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp35_25*S5-ROcp35_85*qd(4));

% = = Block_1_0_0_36_0_4 = = 
 
% Sensor Kinematics 


    ROcp35_421 = ROcp35_46*C21+S21*S5;
    ROcp35_521 = ROcp35_56*C21+ROcp35_85*S21;
    ROcp35_621 = ROcp35_66*C21+ROcp35_95*S21;
    ROcp35_721 = -(ROcp35_46*S21-C21*S5);
    ROcp35_821 = -(ROcp35_56*S21-ROcp35_85*C21);
    ROcp35_921 = -(ROcp35_66*S21-ROcp35_95*C21);
    ROcp35_122 = ROcp35_16*C22-ROcp35_721*S22;
    ROcp35_222 = ROcp35_26*C22-ROcp35_821*S22;
    ROcp35_322 = ROcp35_36*C22-ROcp35_921*S22;
    ROcp35_722 = ROcp35_16*S22+ROcp35_721*C22;
    ROcp35_822 = ROcp35_26*S22+ROcp35_821*C22;
    ROcp35_922 = ROcp35_36*S22+ROcp35_921*C22;
    ROcp35_123 = ROcp35_122*C23+ROcp35_421*S23;
    ROcp35_223 = ROcp35_222*C23+ROcp35_521*S23;
    ROcp35_323 = ROcp35_322*C23+ROcp35_621*S23;
    ROcp35_423 = -(ROcp35_122*S23-ROcp35_421*C23);
    ROcp35_523 = -(ROcp35_222*S23-ROcp35_521*C23);
    ROcp35_623 = -(ROcp35_322*S23-ROcp35_621*C23);
    RLcp35_121 = ROcp35_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp35_221 = ROcp35_26*s.dpt(1,3)+ROcp35_85*s.dpt(3,3);
    RLcp35_321 = ROcp35_36*s.dpt(1,3)+ROcp35_95*s.dpt(3,3);
    OMcp35_121 = OMcp35_16+ROcp35_16*qd(21);
    OMcp35_221 = OMcp35_26+ROcp35_26*qd(21);
    OMcp35_321 = OMcp35_36+ROcp35_36*qd(21);
    ORcp35_121 = OMcp35_26*RLcp35_321-OMcp35_36*RLcp35_221;
    ORcp35_221 = -(OMcp35_16*RLcp35_321-OMcp35_36*RLcp35_121);
    ORcp35_321 = OMcp35_16*RLcp35_221-OMcp35_26*RLcp35_121;
    OMcp35_122 = OMcp35_121+ROcp35_421*qd(22);
    OMcp35_222 = OMcp35_221+ROcp35_521*qd(22);
    OMcp35_322 = OMcp35_321+ROcp35_621*qd(22);
    OPcp35_122 = OPcp35_16+ROcp35_16*qdd(21)+ROcp35_421*qdd(22)+qd(21)*(OMcp35_26*ROcp35_36-OMcp35_36*ROcp35_26)+qd(22)*(OMcp35_221*ROcp35_621-...
 OMcp35_321*ROcp35_521);
    OPcp35_222 = OPcp35_26+ROcp35_26*qdd(21)+ROcp35_521*qdd(22)-qd(21)*(OMcp35_16*ROcp35_36-OMcp35_36*ROcp35_16)-qd(22)*(OMcp35_121*ROcp35_621-...
 OMcp35_321*ROcp35_421);
    OPcp35_322 = OPcp35_36+ROcp35_36*qdd(21)+ROcp35_621*qdd(22)+qd(21)*(OMcp35_16*ROcp35_26-OMcp35_26*ROcp35_16)+qd(22)*(OMcp35_121*ROcp35_521-...
 OMcp35_221*ROcp35_421);
    RLcp35_123 = ROcp35_722*s.dpt(3,37);
    RLcp35_223 = ROcp35_822*s.dpt(3,37);
    RLcp35_323 = ROcp35_922*s.dpt(3,37);
    OMcp35_123 = OMcp35_122+ROcp35_722*qd(23);
    OMcp35_223 = OMcp35_222+ROcp35_822*qd(23);
    OMcp35_323 = OMcp35_322+ROcp35_922*qd(23);
    ORcp35_123 = OMcp35_222*RLcp35_323-OMcp35_322*RLcp35_223;
    ORcp35_223 = -(OMcp35_122*RLcp35_323-OMcp35_322*RLcp35_123);
    ORcp35_323 = OMcp35_122*RLcp35_223-OMcp35_222*RLcp35_123;
    OPcp35_123 = OPcp35_122+ROcp35_722*qdd(23)+qd(23)*(OMcp35_222*ROcp35_922-OMcp35_322*ROcp35_822);
    OPcp35_223 = OPcp35_222+ROcp35_822*qdd(23)-qd(23)*(OMcp35_122*ROcp35_922-OMcp35_322*ROcp35_722);
    OPcp35_323 = OPcp35_322+ROcp35_922*qdd(23)+qd(23)*(OMcp35_122*ROcp35_822-OMcp35_222*ROcp35_722);
    RLcp35_167 = ROcp35_123*s.dpt(1,40)+ROcp35_423*s.dpt(2,40)+ROcp35_722*s.dpt(3,40);
    RLcp35_267 = ROcp35_223*s.dpt(1,40)+ROcp35_523*s.dpt(2,40)+ROcp35_822*s.dpt(3,40);
    RLcp35_367 = ROcp35_323*s.dpt(1,40)+ROcp35_623*s.dpt(2,40)+ROcp35_922*s.dpt(3,40);
    POcp35_167 = RLcp35_121+RLcp35_123+RLcp35_167+q(1);
    POcp35_267 = RLcp35_221+RLcp35_223+RLcp35_267+q(2);
    POcp35_367 = RLcp35_321+RLcp35_323+RLcp35_367+q(3);
    JTcp35_267_4 = -(RLcp35_321+RLcp35_323+RLcp35_367);
    JTcp35_367_4 = RLcp35_221+RLcp35_223+RLcp35_267;
    JTcp35_167_5 = C4*(RLcp35_321+RLcp35_323)-S4*(RLcp35_221+RLcp35_223)-RLcp35_267*S4+RLcp35_367*C4;
    JTcp35_267_5 = S4*(RLcp35_121+RLcp35_123+RLcp35_167);
    JTcp35_367_5 = -C4*(RLcp35_121+RLcp35_123+RLcp35_167);
    JTcp35_167_6 = ROcp35_85*(RLcp35_321+RLcp35_323)-ROcp35_95*(RLcp35_221+RLcp35_223)-RLcp35_267*ROcp35_95+RLcp35_367*ROcp35_85;
    JTcp35_267_6 = -(RLcp35_367*S5-ROcp35_95*(RLcp35_121+RLcp35_123+RLcp35_167)+S5*(RLcp35_321+RLcp35_323));
    JTcp35_367_6 = RLcp35_267*S5-ROcp35_85*(RLcp35_121+RLcp35_123+RLcp35_167)+S5*(RLcp35_221+RLcp35_223);
    JTcp35_167_7 = ROcp35_26*(RLcp35_323+RLcp35_367)-ROcp35_36*(RLcp35_223+RLcp35_267);
    JTcp35_267_7 = -(ROcp35_16*(RLcp35_323+RLcp35_367)-ROcp35_36*(RLcp35_123+RLcp35_167));
    JTcp35_367_7 = ROcp35_16*(RLcp35_223+RLcp35_267)-ROcp35_26*(RLcp35_123+RLcp35_167);
    JTcp35_167_8 = ROcp35_521*(RLcp35_323+RLcp35_367)-ROcp35_621*(RLcp35_223+RLcp35_267);
    JTcp35_267_8 = -(ROcp35_421*(RLcp35_323+RLcp35_367)-ROcp35_621*(RLcp35_123+RLcp35_167));
    JTcp35_367_8 = ROcp35_421*(RLcp35_223+RLcp35_267)-ROcp35_521*(RLcp35_123+RLcp35_167);
    JTcp35_167_9 = -(RLcp35_267*ROcp35_922-RLcp35_367*ROcp35_822);
    JTcp35_267_9 = RLcp35_167*ROcp35_922-RLcp35_367*ROcp35_722;
    JTcp35_367_9 = -(RLcp35_167*ROcp35_822-RLcp35_267*ROcp35_722);
    ORcp35_167 = OMcp35_223*RLcp35_367-OMcp35_323*RLcp35_267;
    ORcp35_267 = -(OMcp35_123*RLcp35_367-OMcp35_323*RLcp35_167);
    ORcp35_367 = OMcp35_123*RLcp35_267-OMcp35_223*RLcp35_167;
    VIcp35_167 = ORcp35_121+ORcp35_123+ORcp35_167+qd(1);
    VIcp35_267 = ORcp35_221+ORcp35_223+ORcp35_267+qd(2);
    VIcp35_367 = ORcp35_321+ORcp35_323+ORcp35_367+qd(3);
    ACcp35_167 = qdd(1)+OMcp35_222*ORcp35_323+OMcp35_223*ORcp35_367+OMcp35_26*ORcp35_321-OMcp35_322*ORcp35_223-OMcp35_323*ORcp35_267-OMcp35_36*...
 ORcp35_221+OPcp35_222*RLcp35_323+OPcp35_223*RLcp35_367+OPcp35_26*RLcp35_321-OPcp35_322*RLcp35_223-OPcp35_323*RLcp35_267-OPcp35_36*RLcp35_221;
    ACcp35_267 = qdd(2)-OMcp35_122*ORcp35_323-OMcp35_123*ORcp35_367-OMcp35_16*ORcp35_321+OMcp35_322*ORcp35_123+OMcp35_323*ORcp35_167+OMcp35_36*...
 ORcp35_121-OPcp35_122*RLcp35_323-OPcp35_123*RLcp35_367-OPcp35_16*RLcp35_321+OPcp35_322*RLcp35_123+OPcp35_323*RLcp35_167+OPcp35_36*RLcp35_121;
    ACcp35_367 = qdd(3)+OMcp35_122*ORcp35_223+OMcp35_123*ORcp35_267+OMcp35_16*ORcp35_221-OMcp35_222*ORcp35_123-OMcp35_223*ORcp35_167-OMcp35_26*...
 ORcp35_121+OPcp35_122*RLcp35_223+OPcp35_123*RLcp35_267+OPcp35_16*RLcp35_221-OPcp35_222*RLcp35_123-OPcp35_223*RLcp35_167-OPcp35_26*RLcp35_121;

% = = Block_1_0_0_36_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp35_167;
    sens.P(2) = POcp35_267;
    sens.P(3) = POcp35_367;
    sens.R(1,1) = ROcp35_123;
    sens.R(1,2) = ROcp35_223;
    sens.R(1,3) = ROcp35_323;
    sens.R(2,1) = ROcp35_423;
    sens.R(2,2) = ROcp35_523;
    sens.R(2,3) = ROcp35_623;
    sens.R(3,1) = ROcp35_722;
    sens.R(3,2) = ROcp35_822;
    sens.R(3,3) = ROcp35_922;
    sens.V(1) = VIcp35_167;
    sens.V(2) = VIcp35_267;
    sens.V(3) = VIcp35_367;
    sens.OM(1) = OMcp35_123;
    sens.OM(2) = OMcp35_223;
    sens.OM(3) = OMcp35_323;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp35_167_5;
    sens.J(1,6) = JTcp35_167_6;
    sens.J(1,21) = JTcp35_167_7;
    sens.J(1,22) = JTcp35_167_8;
    sens.J(1,23) = JTcp35_167_9;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp35_267_4;
    sens.J(2,5) = JTcp35_267_5;
    sens.J(2,6) = JTcp35_267_6;
    sens.J(2,21) = JTcp35_267_7;
    sens.J(2,22) = JTcp35_267_8;
    sens.J(2,23) = JTcp35_267_9;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp35_367_4;
    sens.J(3,5) = JTcp35_367_5;
    sens.J(3,6) = JTcp35_367_6;
    sens.J(3,21) = JTcp35_367_7;
    sens.J(3,22) = JTcp35_367_8;
    sens.J(3,23) = JTcp35_367_9;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,21) = ROcp35_16;
    sens.J(4,22) = ROcp35_421;
    sens.J(4,23) = ROcp35_722;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp35_85;
    sens.J(5,21) = ROcp35_26;
    sens.J(5,22) = ROcp35_521;
    sens.J(5,23) = ROcp35_822;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp35_95;
    sens.J(6,21) = ROcp35_36;
    sens.J(6,22) = ROcp35_621;
    sens.J(6,23) = ROcp35_922;
    sens.A(1) = ACcp35_167;
    sens.A(2) = ACcp35_267;
    sens.A(3) = ACcp35_367;
    sens.OMP(1) = OPcp35_123;
    sens.OMP(2) = OPcp35_223;
    sens.OMP(3) = OPcp35_323;
 
% 
case 37, 


% = = Block_1_0_0_37_0_1 = = 
 
% Sensor Kinematics 


    ROcp36_25 = S4*S5;
    ROcp36_35 = -C4*S5;
    ROcp36_85 = -S4*C5;
    ROcp36_95 = C4*C5;
    ROcp36_16 = C5*C6;
    ROcp36_26 = ROcp36_25*C6+C4*S6;
    ROcp36_36 = ROcp36_35*C6+S4*S6;
    ROcp36_46 = -C5*S6;
    ROcp36_56 = -(ROcp36_25*S6-C4*C6);
    ROcp36_66 = -(ROcp36_35*S6-S4*C6);
    OMcp36_25 = qd(5)*C4;
    OMcp36_35 = qd(5)*S4;
    OMcp36_16 = qd(4)+qd(6)*S5;
    OMcp36_26 = OMcp36_25+ROcp36_85*qd(6);
    OMcp36_36 = OMcp36_35+ROcp36_95*qd(6);
    OPcp36_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp36_26 = ROcp36_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp36_35*S5-ROcp36_95*qd(4));
    OPcp36_36 = ROcp36_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp36_25*S5-ROcp36_85*qd(4));

% = = Block_1_0_0_37_0_4 = = 
 
% Sensor Kinematics 


    ROcp36_421 = ROcp36_46*C21+S21*S5;
    ROcp36_521 = ROcp36_56*C21+ROcp36_85*S21;
    ROcp36_621 = ROcp36_66*C21+ROcp36_95*S21;
    ROcp36_721 = -(ROcp36_46*S21-C21*S5);
    ROcp36_821 = -(ROcp36_56*S21-ROcp36_85*C21);
    ROcp36_921 = -(ROcp36_66*S21-ROcp36_95*C21);
    ROcp36_122 = ROcp36_16*C22-ROcp36_721*S22;
    ROcp36_222 = ROcp36_26*C22-ROcp36_821*S22;
    ROcp36_322 = ROcp36_36*C22-ROcp36_921*S22;
    ROcp36_722 = ROcp36_16*S22+ROcp36_721*C22;
    ROcp36_822 = ROcp36_26*S22+ROcp36_821*C22;
    ROcp36_922 = ROcp36_36*S22+ROcp36_921*C22;
    ROcp36_123 = ROcp36_122*C23+ROcp36_421*S23;
    ROcp36_223 = ROcp36_222*C23+ROcp36_521*S23;
    ROcp36_323 = ROcp36_322*C23+ROcp36_621*S23;
    ROcp36_423 = -(ROcp36_122*S23-ROcp36_421*C23);
    ROcp36_523 = -(ROcp36_222*S23-ROcp36_521*C23);
    ROcp36_623 = -(ROcp36_322*S23-ROcp36_621*C23);
    RLcp36_121 = ROcp36_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp36_221 = ROcp36_26*s.dpt(1,3)+ROcp36_85*s.dpt(3,3);
    RLcp36_321 = ROcp36_36*s.dpt(1,3)+ROcp36_95*s.dpt(3,3);
    OMcp36_121 = OMcp36_16+ROcp36_16*qd(21);
    OMcp36_221 = OMcp36_26+ROcp36_26*qd(21);
    OMcp36_321 = OMcp36_36+ROcp36_36*qd(21);
    ORcp36_121 = OMcp36_26*RLcp36_321-OMcp36_36*RLcp36_221;
    ORcp36_221 = -(OMcp36_16*RLcp36_321-OMcp36_36*RLcp36_121);
    ORcp36_321 = OMcp36_16*RLcp36_221-OMcp36_26*RLcp36_121;
    OMcp36_122 = OMcp36_121+ROcp36_421*qd(22);
    OMcp36_222 = OMcp36_221+ROcp36_521*qd(22);
    OMcp36_322 = OMcp36_321+ROcp36_621*qd(22);
    OPcp36_122 = OPcp36_16+ROcp36_16*qdd(21)+ROcp36_421*qdd(22)+qd(21)*(OMcp36_26*ROcp36_36-OMcp36_36*ROcp36_26)+qd(22)*(OMcp36_221*ROcp36_621-...
 OMcp36_321*ROcp36_521);
    OPcp36_222 = OPcp36_26+ROcp36_26*qdd(21)+ROcp36_521*qdd(22)-qd(21)*(OMcp36_16*ROcp36_36-OMcp36_36*ROcp36_16)-qd(22)*(OMcp36_121*ROcp36_621-...
 OMcp36_321*ROcp36_421);
    OPcp36_322 = OPcp36_36+ROcp36_36*qdd(21)+ROcp36_621*qdd(22)+qd(21)*(OMcp36_16*ROcp36_26-OMcp36_26*ROcp36_16)+qd(22)*(OMcp36_121*ROcp36_521-...
 OMcp36_221*ROcp36_421);
    RLcp36_123 = ROcp36_722*s.dpt(3,37);
    RLcp36_223 = ROcp36_822*s.dpt(3,37);
    RLcp36_323 = ROcp36_922*s.dpt(3,37);
    OMcp36_123 = OMcp36_122+ROcp36_722*qd(23);
    OMcp36_223 = OMcp36_222+ROcp36_822*qd(23);
    OMcp36_323 = OMcp36_322+ROcp36_922*qd(23);
    ORcp36_123 = OMcp36_222*RLcp36_323-OMcp36_322*RLcp36_223;
    ORcp36_223 = -(OMcp36_122*RLcp36_323-OMcp36_322*RLcp36_123);
    ORcp36_323 = OMcp36_122*RLcp36_223-OMcp36_222*RLcp36_123;
    OPcp36_123 = OPcp36_122+ROcp36_722*qdd(23)+qd(23)*(OMcp36_222*ROcp36_922-OMcp36_322*ROcp36_822);
    OPcp36_223 = OPcp36_222+ROcp36_822*qdd(23)-qd(23)*(OMcp36_122*ROcp36_922-OMcp36_322*ROcp36_722);
    OPcp36_323 = OPcp36_322+ROcp36_922*qdd(23)+qd(23)*(OMcp36_122*ROcp36_822-OMcp36_222*ROcp36_722);
    RLcp36_168 = ROcp36_123*s.dpt(1,41)+ROcp36_423*s.dpt(2,41)+ROcp36_722*s.dpt(3,41);
    RLcp36_268 = ROcp36_223*s.dpt(1,41)+ROcp36_523*s.dpt(2,41)+ROcp36_822*s.dpt(3,41);
    RLcp36_368 = ROcp36_323*s.dpt(1,41)+ROcp36_623*s.dpt(2,41)+ROcp36_922*s.dpt(3,41);
    POcp36_168 = RLcp36_121+RLcp36_123+RLcp36_168+q(1);
    POcp36_268 = RLcp36_221+RLcp36_223+RLcp36_268+q(2);
    POcp36_368 = RLcp36_321+RLcp36_323+RLcp36_368+q(3);
    JTcp36_268_4 = -(RLcp36_321+RLcp36_323+RLcp36_368);
    JTcp36_368_4 = RLcp36_221+RLcp36_223+RLcp36_268;
    JTcp36_168_5 = C4*(RLcp36_321+RLcp36_323)-S4*(RLcp36_221+RLcp36_223)-RLcp36_268*S4+RLcp36_368*C4;
    JTcp36_268_5 = S4*(RLcp36_121+RLcp36_123+RLcp36_168);
    JTcp36_368_5 = -C4*(RLcp36_121+RLcp36_123+RLcp36_168);
    JTcp36_168_6 = ROcp36_85*(RLcp36_321+RLcp36_323)-ROcp36_95*(RLcp36_221+RLcp36_223)-RLcp36_268*ROcp36_95+RLcp36_368*ROcp36_85;
    JTcp36_268_6 = -(RLcp36_368*S5-ROcp36_95*(RLcp36_121+RLcp36_123+RLcp36_168)+S5*(RLcp36_321+RLcp36_323));
    JTcp36_368_6 = RLcp36_268*S5-ROcp36_85*(RLcp36_121+RLcp36_123+RLcp36_168)+S5*(RLcp36_221+RLcp36_223);
    JTcp36_168_7 = ROcp36_26*(RLcp36_323+RLcp36_368)-ROcp36_36*(RLcp36_223+RLcp36_268);
    JTcp36_268_7 = -(ROcp36_16*(RLcp36_323+RLcp36_368)-ROcp36_36*(RLcp36_123+RLcp36_168));
    JTcp36_368_7 = ROcp36_16*(RLcp36_223+RLcp36_268)-ROcp36_26*(RLcp36_123+RLcp36_168);
    JTcp36_168_8 = ROcp36_521*(RLcp36_323+RLcp36_368)-ROcp36_621*(RLcp36_223+RLcp36_268);
    JTcp36_268_8 = -(ROcp36_421*(RLcp36_323+RLcp36_368)-ROcp36_621*(RLcp36_123+RLcp36_168));
    JTcp36_368_8 = ROcp36_421*(RLcp36_223+RLcp36_268)-ROcp36_521*(RLcp36_123+RLcp36_168);
    JTcp36_168_9 = -(RLcp36_268*ROcp36_922-RLcp36_368*ROcp36_822);
    JTcp36_268_9 = RLcp36_168*ROcp36_922-RLcp36_368*ROcp36_722;
    JTcp36_368_9 = -(RLcp36_168*ROcp36_822-RLcp36_268*ROcp36_722);
    ORcp36_168 = OMcp36_223*RLcp36_368-OMcp36_323*RLcp36_268;
    ORcp36_268 = -(OMcp36_123*RLcp36_368-OMcp36_323*RLcp36_168);
    ORcp36_368 = OMcp36_123*RLcp36_268-OMcp36_223*RLcp36_168;
    VIcp36_168 = ORcp36_121+ORcp36_123+ORcp36_168+qd(1);
    VIcp36_268 = ORcp36_221+ORcp36_223+ORcp36_268+qd(2);
    VIcp36_368 = ORcp36_321+ORcp36_323+ORcp36_368+qd(3);
    ACcp36_168 = qdd(1)+OMcp36_222*ORcp36_323+OMcp36_223*ORcp36_368+OMcp36_26*ORcp36_321-OMcp36_322*ORcp36_223-OMcp36_323*ORcp36_268-OMcp36_36*...
 ORcp36_221+OPcp36_222*RLcp36_323+OPcp36_223*RLcp36_368+OPcp36_26*RLcp36_321-OPcp36_322*RLcp36_223-OPcp36_323*RLcp36_268-OPcp36_36*RLcp36_221;
    ACcp36_268 = qdd(2)-OMcp36_122*ORcp36_323-OMcp36_123*ORcp36_368-OMcp36_16*ORcp36_321+OMcp36_322*ORcp36_123+OMcp36_323*ORcp36_168+OMcp36_36*...
 ORcp36_121-OPcp36_122*RLcp36_323-OPcp36_123*RLcp36_368-OPcp36_16*RLcp36_321+OPcp36_322*RLcp36_123+OPcp36_323*RLcp36_168+OPcp36_36*RLcp36_121;
    ACcp36_368 = qdd(3)+OMcp36_122*ORcp36_223+OMcp36_123*ORcp36_268+OMcp36_16*ORcp36_221-OMcp36_222*ORcp36_123-OMcp36_223*ORcp36_168-OMcp36_26*...
 ORcp36_121+OPcp36_122*RLcp36_223+OPcp36_123*RLcp36_268+OPcp36_16*RLcp36_221-OPcp36_222*RLcp36_123-OPcp36_223*RLcp36_168-OPcp36_26*RLcp36_121;

% = = Block_1_0_0_37_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp36_168;
    sens.P(2) = POcp36_268;
    sens.P(3) = POcp36_368;
    sens.R(1,1) = ROcp36_123;
    sens.R(1,2) = ROcp36_223;
    sens.R(1,3) = ROcp36_323;
    sens.R(2,1) = ROcp36_423;
    sens.R(2,2) = ROcp36_523;
    sens.R(2,3) = ROcp36_623;
    sens.R(3,1) = ROcp36_722;
    sens.R(3,2) = ROcp36_822;
    sens.R(3,3) = ROcp36_922;
    sens.V(1) = VIcp36_168;
    sens.V(2) = VIcp36_268;
    sens.V(3) = VIcp36_368;
    sens.OM(1) = OMcp36_123;
    sens.OM(2) = OMcp36_223;
    sens.OM(3) = OMcp36_323;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp36_168_5;
    sens.J(1,6) = JTcp36_168_6;
    sens.J(1,21) = JTcp36_168_7;
    sens.J(1,22) = JTcp36_168_8;
    sens.J(1,23) = JTcp36_168_9;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp36_268_4;
    sens.J(2,5) = JTcp36_268_5;
    sens.J(2,6) = JTcp36_268_6;
    sens.J(2,21) = JTcp36_268_7;
    sens.J(2,22) = JTcp36_268_8;
    sens.J(2,23) = JTcp36_268_9;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp36_368_4;
    sens.J(3,5) = JTcp36_368_5;
    sens.J(3,6) = JTcp36_368_6;
    sens.J(3,21) = JTcp36_368_7;
    sens.J(3,22) = JTcp36_368_8;
    sens.J(3,23) = JTcp36_368_9;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,21) = ROcp36_16;
    sens.J(4,22) = ROcp36_421;
    sens.J(4,23) = ROcp36_722;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp36_85;
    sens.J(5,21) = ROcp36_26;
    sens.J(5,22) = ROcp36_521;
    sens.J(5,23) = ROcp36_822;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp36_95;
    sens.J(6,21) = ROcp36_36;
    sens.J(6,22) = ROcp36_621;
    sens.J(6,23) = ROcp36_922;
    sens.A(1) = ACcp36_168;
    sens.A(2) = ACcp36_268;
    sens.A(3) = ACcp36_368;
    sens.OMP(1) = OPcp36_123;
    sens.OMP(2) = OPcp36_223;
    sens.OMP(3) = OPcp36_323;
 
% 
case 38, 


% = = Block_1_0_0_38_0_1 = = 
 
% Sensor Kinematics 


    ROcp37_25 = S4*S5;
    ROcp37_35 = -C4*S5;
    ROcp37_85 = -S4*C5;
    ROcp37_95 = C4*C5;
    ROcp37_16 = C5*C6;
    ROcp37_26 = ROcp37_25*C6+C4*S6;
    ROcp37_36 = ROcp37_35*C6+S4*S6;
    ROcp37_46 = -C5*S6;
    ROcp37_56 = -(ROcp37_25*S6-C4*C6);
    ROcp37_66 = -(ROcp37_35*S6-S4*C6);
    OMcp37_25 = qd(5)*C4;
    OMcp37_35 = qd(5)*S4;
    OMcp37_16 = qd(4)+qd(6)*S5;
    OMcp37_26 = OMcp37_25+ROcp37_85*qd(6);
    OMcp37_36 = OMcp37_35+ROcp37_95*qd(6);
    OPcp37_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp37_26 = ROcp37_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp37_35*S5-ROcp37_95*qd(4));
    OPcp37_36 = ROcp37_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp37_25*S5-ROcp37_85*qd(4));

% = = Block_1_0_0_38_0_4 = = 
 
% Sensor Kinematics 


    ROcp37_421 = ROcp37_46*C21+S21*S5;
    ROcp37_521 = ROcp37_56*C21+ROcp37_85*S21;
    ROcp37_621 = ROcp37_66*C21+ROcp37_95*S21;
    ROcp37_721 = -(ROcp37_46*S21-C21*S5);
    ROcp37_821 = -(ROcp37_56*S21-ROcp37_85*C21);
    ROcp37_921 = -(ROcp37_66*S21-ROcp37_95*C21);
    ROcp37_122 = ROcp37_16*C22-ROcp37_721*S22;
    ROcp37_222 = ROcp37_26*C22-ROcp37_821*S22;
    ROcp37_322 = ROcp37_36*C22-ROcp37_921*S22;
    ROcp37_722 = ROcp37_16*S22+ROcp37_721*C22;
    ROcp37_822 = ROcp37_26*S22+ROcp37_821*C22;
    ROcp37_922 = ROcp37_36*S22+ROcp37_921*C22;
    ROcp37_123 = ROcp37_122*C23+ROcp37_421*S23;
    ROcp37_223 = ROcp37_222*C23+ROcp37_521*S23;
    ROcp37_323 = ROcp37_322*C23+ROcp37_621*S23;
    ROcp37_423 = -(ROcp37_122*S23-ROcp37_421*C23);
    ROcp37_523 = -(ROcp37_222*S23-ROcp37_521*C23);
    ROcp37_623 = -(ROcp37_322*S23-ROcp37_621*C23);
    RLcp37_121 = ROcp37_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp37_221 = ROcp37_26*s.dpt(1,3)+ROcp37_85*s.dpt(3,3);
    RLcp37_321 = ROcp37_36*s.dpt(1,3)+ROcp37_95*s.dpt(3,3);
    OMcp37_121 = OMcp37_16+ROcp37_16*qd(21);
    OMcp37_221 = OMcp37_26+ROcp37_26*qd(21);
    OMcp37_321 = OMcp37_36+ROcp37_36*qd(21);
    ORcp37_121 = OMcp37_26*RLcp37_321-OMcp37_36*RLcp37_221;
    ORcp37_221 = -(OMcp37_16*RLcp37_321-OMcp37_36*RLcp37_121);
    ORcp37_321 = OMcp37_16*RLcp37_221-OMcp37_26*RLcp37_121;
    OMcp37_122 = OMcp37_121+ROcp37_421*qd(22);
    OMcp37_222 = OMcp37_221+ROcp37_521*qd(22);
    OMcp37_322 = OMcp37_321+ROcp37_621*qd(22);
    OPcp37_122 = OPcp37_16+ROcp37_16*qdd(21)+ROcp37_421*qdd(22)+qd(21)*(OMcp37_26*ROcp37_36-OMcp37_36*ROcp37_26)+qd(22)*(OMcp37_221*ROcp37_621-...
 OMcp37_321*ROcp37_521);
    OPcp37_222 = OPcp37_26+ROcp37_26*qdd(21)+ROcp37_521*qdd(22)-qd(21)*(OMcp37_16*ROcp37_36-OMcp37_36*ROcp37_16)-qd(22)*(OMcp37_121*ROcp37_621-...
 OMcp37_321*ROcp37_421);
    OPcp37_322 = OPcp37_36+ROcp37_36*qdd(21)+ROcp37_621*qdd(22)+qd(21)*(OMcp37_16*ROcp37_26-OMcp37_26*ROcp37_16)+qd(22)*(OMcp37_121*ROcp37_521-...
 OMcp37_221*ROcp37_421);
    RLcp37_123 = ROcp37_722*s.dpt(3,37);
    RLcp37_223 = ROcp37_822*s.dpt(3,37);
    RLcp37_323 = ROcp37_922*s.dpt(3,37);
    OMcp37_123 = OMcp37_122+ROcp37_722*qd(23);
    OMcp37_223 = OMcp37_222+ROcp37_822*qd(23);
    OMcp37_323 = OMcp37_322+ROcp37_922*qd(23);
    ORcp37_123 = OMcp37_222*RLcp37_323-OMcp37_322*RLcp37_223;
    ORcp37_223 = -(OMcp37_122*RLcp37_323-OMcp37_322*RLcp37_123);
    ORcp37_323 = OMcp37_122*RLcp37_223-OMcp37_222*RLcp37_123;
    OPcp37_123 = OPcp37_122+ROcp37_722*qdd(23)+qd(23)*(OMcp37_222*ROcp37_922-OMcp37_322*ROcp37_822);
    OPcp37_223 = OPcp37_222+ROcp37_822*qdd(23)-qd(23)*(OMcp37_122*ROcp37_922-OMcp37_322*ROcp37_722);
    OPcp37_323 = OPcp37_322+ROcp37_922*qdd(23)+qd(23)*(OMcp37_122*ROcp37_822-OMcp37_222*ROcp37_722);
    RLcp37_169 = ROcp37_123*s.dpt(1,42)+ROcp37_423*s.dpt(2,42)+ROcp37_722*s.dpt(3,42);
    RLcp37_269 = ROcp37_223*s.dpt(1,42)+ROcp37_523*s.dpt(2,42)+ROcp37_822*s.dpt(3,42);
    RLcp37_369 = ROcp37_323*s.dpt(1,42)+ROcp37_623*s.dpt(2,42)+ROcp37_922*s.dpt(3,42);
    POcp37_169 = RLcp37_121+RLcp37_123+RLcp37_169+q(1);
    POcp37_269 = RLcp37_221+RLcp37_223+RLcp37_269+q(2);
    POcp37_369 = RLcp37_321+RLcp37_323+RLcp37_369+q(3);
    JTcp37_269_4 = -(RLcp37_321+RLcp37_323+RLcp37_369);
    JTcp37_369_4 = RLcp37_221+RLcp37_223+RLcp37_269;
    JTcp37_169_5 = C4*(RLcp37_321+RLcp37_323)-S4*(RLcp37_221+RLcp37_223)-RLcp37_269*S4+RLcp37_369*C4;
    JTcp37_269_5 = S4*(RLcp37_121+RLcp37_123+RLcp37_169);
    JTcp37_369_5 = -C4*(RLcp37_121+RLcp37_123+RLcp37_169);
    JTcp37_169_6 = ROcp37_85*(RLcp37_321+RLcp37_323)-ROcp37_95*(RLcp37_221+RLcp37_223)-RLcp37_269*ROcp37_95+RLcp37_369*ROcp37_85;
    JTcp37_269_6 = -(RLcp37_369*S5-ROcp37_95*(RLcp37_121+RLcp37_123+RLcp37_169)+S5*(RLcp37_321+RLcp37_323));
    JTcp37_369_6 = RLcp37_269*S5-ROcp37_85*(RLcp37_121+RLcp37_123+RLcp37_169)+S5*(RLcp37_221+RLcp37_223);
    JTcp37_169_7 = ROcp37_26*(RLcp37_323+RLcp37_369)-ROcp37_36*(RLcp37_223+RLcp37_269);
    JTcp37_269_7 = -(ROcp37_16*(RLcp37_323+RLcp37_369)-ROcp37_36*(RLcp37_123+RLcp37_169));
    JTcp37_369_7 = ROcp37_16*(RLcp37_223+RLcp37_269)-ROcp37_26*(RLcp37_123+RLcp37_169);
    JTcp37_169_8 = ROcp37_521*(RLcp37_323+RLcp37_369)-ROcp37_621*(RLcp37_223+RLcp37_269);
    JTcp37_269_8 = -(ROcp37_421*(RLcp37_323+RLcp37_369)-ROcp37_621*(RLcp37_123+RLcp37_169));
    JTcp37_369_8 = ROcp37_421*(RLcp37_223+RLcp37_269)-ROcp37_521*(RLcp37_123+RLcp37_169);
    JTcp37_169_9 = -(RLcp37_269*ROcp37_922-RLcp37_369*ROcp37_822);
    JTcp37_269_9 = RLcp37_169*ROcp37_922-RLcp37_369*ROcp37_722;
    JTcp37_369_9 = -(RLcp37_169*ROcp37_822-RLcp37_269*ROcp37_722);
    ORcp37_169 = OMcp37_223*RLcp37_369-OMcp37_323*RLcp37_269;
    ORcp37_269 = -(OMcp37_123*RLcp37_369-OMcp37_323*RLcp37_169);
    ORcp37_369 = OMcp37_123*RLcp37_269-OMcp37_223*RLcp37_169;
    VIcp37_169 = ORcp37_121+ORcp37_123+ORcp37_169+qd(1);
    VIcp37_269 = ORcp37_221+ORcp37_223+ORcp37_269+qd(2);
    VIcp37_369 = ORcp37_321+ORcp37_323+ORcp37_369+qd(3);
    ACcp37_169 = qdd(1)+OMcp37_222*ORcp37_323+OMcp37_223*ORcp37_369+OMcp37_26*ORcp37_321-OMcp37_322*ORcp37_223-OMcp37_323*ORcp37_269-OMcp37_36*...
 ORcp37_221+OPcp37_222*RLcp37_323+OPcp37_223*RLcp37_369+OPcp37_26*RLcp37_321-OPcp37_322*RLcp37_223-OPcp37_323*RLcp37_269-OPcp37_36*RLcp37_221;
    ACcp37_269 = qdd(2)-OMcp37_122*ORcp37_323-OMcp37_123*ORcp37_369-OMcp37_16*ORcp37_321+OMcp37_322*ORcp37_123+OMcp37_323*ORcp37_169+OMcp37_36*...
 ORcp37_121-OPcp37_122*RLcp37_323-OPcp37_123*RLcp37_369-OPcp37_16*RLcp37_321+OPcp37_322*RLcp37_123+OPcp37_323*RLcp37_169+OPcp37_36*RLcp37_121;
    ACcp37_369 = qdd(3)+OMcp37_122*ORcp37_223+OMcp37_123*ORcp37_269+OMcp37_16*ORcp37_221-OMcp37_222*ORcp37_123-OMcp37_223*ORcp37_169-OMcp37_26*...
 ORcp37_121+OPcp37_122*RLcp37_223+OPcp37_123*RLcp37_269+OPcp37_16*RLcp37_221-OPcp37_222*RLcp37_123-OPcp37_223*RLcp37_169-OPcp37_26*RLcp37_121;

% = = Block_1_0_0_38_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp37_169;
    sens.P(2) = POcp37_269;
    sens.P(3) = POcp37_369;
    sens.R(1,1) = ROcp37_123;
    sens.R(1,2) = ROcp37_223;
    sens.R(1,3) = ROcp37_323;
    sens.R(2,1) = ROcp37_423;
    sens.R(2,2) = ROcp37_523;
    sens.R(2,3) = ROcp37_623;
    sens.R(3,1) = ROcp37_722;
    sens.R(3,2) = ROcp37_822;
    sens.R(3,3) = ROcp37_922;
    sens.V(1) = VIcp37_169;
    sens.V(2) = VIcp37_269;
    sens.V(3) = VIcp37_369;
    sens.OM(1) = OMcp37_123;
    sens.OM(2) = OMcp37_223;
    sens.OM(3) = OMcp37_323;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp37_169_5;
    sens.J(1,6) = JTcp37_169_6;
    sens.J(1,21) = JTcp37_169_7;
    sens.J(1,22) = JTcp37_169_8;
    sens.J(1,23) = JTcp37_169_9;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp37_269_4;
    sens.J(2,5) = JTcp37_269_5;
    sens.J(2,6) = JTcp37_269_6;
    sens.J(2,21) = JTcp37_269_7;
    sens.J(2,22) = JTcp37_269_8;
    sens.J(2,23) = JTcp37_269_9;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp37_369_4;
    sens.J(3,5) = JTcp37_369_5;
    sens.J(3,6) = JTcp37_369_6;
    sens.J(3,21) = JTcp37_369_7;
    sens.J(3,22) = JTcp37_369_8;
    sens.J(3,23) = JTcp37_369_9;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,21) = ROcp37_16;
    sens.J(4,22) = ROcp37_421;
    sens.J(4,23) = ROcp37_722;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp37_85;
    sens.J(5,21) = ROcp37_26;
    sens.J(5,22) = ROcp37_521;
    sens.J(5,23) = ROcp37_822;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp37_95;
    sens.J(6,21) = ROcp37_36;
    sens.J(6,22) = ROcp37_621;
    sens.J(6,23) = ROcp37_922;
    sens.A(1) = ACcp37_169;
    sens.A(2) = ACcp37_269;
    sens.A(3) = ACcp37_369;
    sens.OMP(1) = OPcp37_123;
    sens.OMP(2) = OPcp37_223;
    sens.OMP(3) = OPcp37_323;
 
% 
case 39, 


% = = Block_1_0_0_39_0_1 = = 
 
% Sensor Kinematics 


    ROcp38_25 = S4*S5;
    ROcp38_35 = -C4*S5;
    ROcp38_85 = -S4*C5;
    ROcp38_95 = C4*C5;
    ROcp38_16 = C5*C6;
    ROcp38_26 = ROcp38_25*C6+C4*S6;
    ROcp38_36 = ROcp38_35*C6+S4*S6;
    ROcp38_46 = -C5*S6;
    ROcp38_56 = -(ROcp38_25*S6-C4*C6);
    ROcp38_66 = -(ROcp38_35*S6-S4*C6);
    OMcp38_25 = qd(5)*C4;
    OMcp38_35 = qd(5)*S4;
    OMcp38_16 = qd(4)+qd(6)*S5;
    OMcp38_26 = OMcp38_25+ROcp38_85*qd(6);
    OMcp38_36 = OMcp38_35+ROcp38_95*qd(6);
    OPcp38_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp38_26 = ROcp38_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp38_35*S5-ROcp38_95*qd(4));
    OPcp38_36 = ROcp38_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp38_25*S5-ROcp38_85*qd(4));

% = = Block_1_0_0_39_0_4 = = 
 
% Sensor Kinematics 


    ROcp38_421 = ROcp38_46*C21+S21*S5;
    ROcp38_521 = ROcp38_56*C21+ROcp38_85*S21;
    ROcp38_621 = ROcp38_66*C21+ROcp38_95*S21;
    ROcp38_721 = -(ROcp38_46*S21-C21*S5);
    ROcp38_821 = -(ROcp38_56*S21-ROcp38_85*C21);
    ROcp38_921 = -(ROcp38_66*S21-ROcp38_95*C21);
    ROcp38_122 = ROcp38_16*C22-ROcp38_721*S22;
    ROcp38_222 = ROcp38_26*C22-ROcp38_821*S22;
    ROcp38_322 = ROcp38_36*C22-ROcp38_921*S22;
    ROcp38_722 = ROcp38_16*S22+ROcp38_721*C22;
    ROcp38_822 = ROcp38_26*S22+ROcp38_821*C22;
    ROcp38_922 = ROcp38_36*S22+ROcp38_921*C22;
    ROcp38_123 = ROcp38_122*C23+ROcp38_421*S23;
    ROcp38_223 = ROcp38_222*C23+ROcp38_521*S23;
    ROcp38_323 = ROcp38_322*C23+ROcp38_621*S23;
    ROcp38_423 = -(ROcp38_122*S23-ROcp38_421*C23);
    ROcp38_523 = -(ROcp38_222*S23-ROcp38_521*C23);
    ROcp38_623 = -(ROcp38_322*S23-ROcp38_621*C23);
    RLcp38_121 = ROcp38_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp38_221 = ROcp38_26*s.dpt(1,3)+ROcp38_85*s.dpt(3,3);
    RLcp38_321 = ROcp38_36*s.dpt(1,3)+ROcp38_95*s.dpt(3,3);
    OMcp38_121 = OMcp38_16+ROcp38_16*qd(21);
    OMcp38_221 = OMcp38_26+ROcp38_26*qd(21);
    OMcp38_321 = OMcp38_36+ROcp38_36*qd(21);
    ORcp38_121 = OMcp38_26*RLcp38_321-OMcp38_36*RLcp38_221;
    ORcp38_221 = -(OMcp38_16*RLcp38_321-OMcp38_36*RLcp38_121);
    ORcp38_321 = OMcp38_16*RLcp38_221-OMcp38_26*RLcp38_121;
    OMcp38_122 = OMcp38_121+ROcp38_421*qd(22);
    OMcp38_222 = OMcp38_221+ROcp38_521*qd(22);
    OMcp38_322 = OMcp38_321+ROcp38_621*qd(22);
    OPcp38_122 = OPcp38_16+ROcp38_16*qdd(21)+ROcp38_421*qdd(22)+qd(21)*(OMcp38_26*ROcp38_36-OMcp38_36*ROcp38_26)+qd(22)*(OMcp38_221*ROcp38_621-...
 OMcp38_321*ROcp38_521);
    OPcp38_222 = OPcp38_26+ROcp38_26*qdd(21)+ROcp38_521*qdd(22)-qd(21)*(OMcp38_16*ROcp38_36-OMcp38_36*ROcp38_16)-qd(22)*(OMcp38_121*ROcp38_621-...
 OMcp38_321*ROcp38_421);
    OPcp38_322 = OPcp38_36+ROcp38_36*qdd(21)+ROcp38_621*qdd(22)+qd(21)*(OMcp38_16*ROcp38_26-OMcp38_26*ROcp38_16)+qd(22)*(OMcp38_121*ROcp38_521-...
 OMcp38_221*ROcp38_421);
    RLcp38_123 = ROcp38_722*s.dpt(3,37);
    RLcp38_223 = ROcp38_822*s.dpt(3,37);
    RLcp38_323 = ROcp38_922*s.dpt(3,37);
    OMcp38_123 = OMcp38_122+ROcp38_722*qd(23);
    OMcp38_223 = OMcp38_222+ROcp38_822*qd(23);
    OMcp38_323 = OMcp38_322+ROcp38_922*qd(23);
    ORcp38_123 = OMcp38_222*RLcp38_323-OMcp38_322*RLcp38_223;
    ORcp38_223 = -(OMcp38_122*RLcp38_323-OMcp38_322*RLcp38_123);
    ORcp38_323 = OMcp38_122*RLcp38_223-OMcp38_222*RLcp38_123;
    OPcp38_123 = OPcp38_122+ROcp38_722*qdd(23)+qd(23)*(OMcp38_222*ROcp38_922-OMcp38_322*ROcp38_822);
    OPcp38_223 = OPcp38_222+ROcp38_822*qdd(23)-qd(23)*(OMcp38_122*ROcp38_922-OMcp38_322*ROcp38_722);
    OPcp38_323 = OPcp38_322+ROcp38_922*qdd(23)+qd(23)*(OMcp38_122*ROcp38_822-OMcp38_222*ROcp38_722);

% = = Block_1_0_0_39_0_5 = = 
 
% Sensor Kinematics 


    ROcp38_124 = ROcp38_123*C24-ROcp38_722*S24;
    ROcp38_224 = ROcp38_223*C24-ROcp38_822*S24;
    ROcp38_324 = ROcp38_323*C24-ROcp38_922*S24;
    ROcp38_724 = ROcp38_123*S24+ROcp38_722*C24;
    ROcp38_824 = ROcp38_223*S24+ROcp38_822*C24;
    ROcp38_924 = ROcp38_323*S24+ROcp38_922*C24;
    RLcp38_124 = ROcp38_123*s.dpt(1,41)+ROcp38_423*s.dpt(2,41)+ROcp38_722*s.dpt(3,41);
    RLcp38_224 = ROcp38_223*s.dpt(1,41)+ROcp38_523*s.dpt(2,41)+ROcp38_822*s.dpt(3,41);
    RLcp38_324 = ROcp38_323*s.dpt(1,41)+ROcp38_623*s.dpt(2,41)+ROcp38_922*s.dpt(3,41);
    OMcp38_124 = OMcp38_123+ROcp38_423*qd(24);
    OMcp38_224 = OMcp38_223+ROcp38_523*qd(24);
    OMcp38_324 = OMcp38_323+ROcp38_623*qd(24);
    ORcp38_124 = OMcp38_223*RLcp38_324-OMcp38_323*RLcp38_224;
    ORcp38_224 = -(OMcp38_123*RLcp38_324-OMcp38_323*RLcp38_124);
    ORcp38_324 = OMcp38_123*RLcp38_224-OMcp38_223*RLcp38_124;
    OPcp38_124 = OPcp38_123+ROcp38_423*qdd(24)+qd(24)*(OMcp38_223*ROcp38_623-OMcp38_323*ROcp38_523);
    OPcp38_224 = OPcp38_223+ROcp38_523*qdd(24)-qd(24)*(OMcp38_123*ROcp38_623-OMcp38_323*ROcp38_423);
    OPcp38_324 = OPcp38_323+ROcp38_623*qdd(24)+qd(24)*(OMcp38_123*ROcp38_523-OMcp38_223*ROcp38_423);
    RLcp38_170 = ROcp38_124*s.dpt(1,43)+ROcp38_423*s.dpt(2,43)+ROcp38_724*s.dpt(3,43);
    RLcp38_270 = ROcp38_224*s.dpt(1,43)+ROcp38_523*s.dpt(2,43)+ROcp38_824*s.dpt(3,43);
    RLcp38_370 = ROcp38_324*s.dpt(1,43)+ROcp38_623*s.dpt(2,43)+ROcp38_924*s.dpt(3,43);
    POcp38_170 = RLcp38_121+RLcp38_123+RLcp38_124+RLcp38_170+q(1);
    POcp38_270 = RLcp38_221+RLcp38_223+RLcp38_224+RLcp38_270+q(2);
    POcp38_370 = RLcp38_321+RLcp38_323+RLcp38_324+RLcp38_370+q(3);
    JTcp38_270_4 = -(RLcp38_321+RLcp38_323+RLcp38_324+RLcp38_370);
    JTcp38_370_4 = RLcp38_221+RLcp38_223+RLcp38_224+RLcp38_270;
    JTcp38_170_5 = C4*(RLcp38_321+RLcp38_323+RLcp38_324+RLcp38_370)-S4*(RLcp38_221+RLcp38_223)-S4*(RLcp38_224+RLcp38_270);
    JTcp38_270_5 = S4*(RLcp38_121+RLcp38_123+RLcp38_124+RLcp38_170);
    JTcp38_370_5 = -C4*(RLcp38_121+RLcp38_123+RLcp38_124+RLcp38_170);
    JTcp38_170_6 = ROcp38_85*(RLcp38_321+RLcp38_323+RLcp38_324+RLcp38_370)-ROcp38_95*(RLcp38_221+RLcp38_223)-ROcp38_95*(RLcp38_224+RLcp38_270);
    JTcp38_270_6 = RLcp38_170*ROcp38_95-RLcp38_324*S5-RLcp38_370*S5+ROcp38_95*(RLcp38_121+RLcp38_123+RLcp38_124)-S5*(RLcp38_321+RLcp38_323);
    JTcp38_370_6 = RLcp38_224*S5-ROcp38_85*(RLcp38_121+RLcp38_123+RLcp38_124)+S5*(RLcp38_221+RLcp38_223)-RLcp38_170*ROcp38_85+RLcp38_270*S5;
    JTcp38_170_7 = ROcp38_26*(RLcp38_323+RLcp38_324)-ROcp38_36*(RLcp38_223+RLcp38_224)-RLcp38_270*ROcp38_36+RLcp38_370*ROcp38_26;
    JTcp38_270_7 = RLcp38_170*ROcp38_36-RLcp38_370*ROcp38_16-ROcp38_16*(RLcp38_323+RLcp38_324)+ROcp38_36*(RLcp38_123+RLcp38_124);
    JTcp38_370_7 = ROcp38_16*(RLcp38_223+RLcp38_224)-ROcp38_26*(RLcp38_123+RLcp38_124)-RLcp38_170*ROcp38_26+RLcp38_270*ROcp38_16;
    JTcp38_170_8 = ROcp38_521*(RLcp38_323+RLcp38_324)-ROcp38_621*(RLcp38_223+RLcp38_224)-RLcp38_270*ROcp38_621+RLcp38_370*ROcp38_521;
    JTcp38_270_8 = RLcp38_170*ROcp38_621-RLcp38_370*ROcp38_421-ROcp38_421*(RLcp38_323+RLcp38_324)+ROcp38_621*(RLcp38_123+RLcp38_124);
    JTcp38_370_8 = ROcp38_421*(RLcp38_223+RLcp38_224)-ROcp38_521*(RLcp38_123+RLcp38_124)-RLcp38_170*ROcp38_521+RLcp38_270*ROcp38_421;
    JTcp38_170_9 = ROcp38_822*(RLcp38_324+RLcp38_370)-ROcp38_922*(RLcp38_224+RLcp38_270);
    JTcp38_270_9 = -(ROcp38_722*(RLcp38_324+RLcp38_370)-ROcp38_922*(RLcp38_124+RLcp38_170));
    JTcp38_370_9 = ROcp38_722*(RLcp38_224+RLcp38_270)-ROcp38_822*(RLcp38_124+RLcp38_170);
    JTcp38_170_10 = -(RLcp38_270*ROcp38_623-RLcp38_370*ROcp38_523);
    JTcp38_270_10 = RLcp38_170*ROcp38_623-RLcp38_370*ROcp38_423;
    JTcp38_370_10 = -(RLcp38_170*ROcp38_523-RLcp38_270*ROcp38_423);
    ORcp38_170 = OMcp38_224*RLcp38_370-OMcp38_324*RLcp38_270;
    ORcp38_270 = -(OMcp38_124*RLcp38_370-OMcp38_324*RLcp38_170);
    ORcp38_370 = OMcp38_124*RLcp38_270-OMcp38_224*RLcp38_170;
    VIcp38_170 = ORcp38_121+ORcp38_123+ORcp38_124+ORcp38_170+qd(1);
    VIcp38_270 = ORcp38_221+ORcp38_223+ORcp38_224+ORcp38_270+qd(2);
    VIcp38_370 = ORcp38_321+ORcp38_323+ORcp38_324+ORcp38_370+qd(3);
    ACcp38_170 = qdd(1)+OMcp38_222*ORcp38_323+OMcp38_223*ORcp38_324+OMcp38_224*ORcp38_370+OMcp38_26*ORcp38_321-OMcp38_322*ORcp38_223-OMcp38_323*...
 ORcp38_224-OMcp38_324*ORcp38_270-OMcp38_36*ORcp38_221+OPcp38_222*RLcp38_323+OPcp38_223*RLcp38_324+OPcp38_224*RLcp38_370+OPcp38_26*RLcp38_321-...
 OPcp38_322*RLcp38_223-OPcp38_323*RLcp38_224-OPcp38_324*RLcp38_270-OPcp38_36*RLcp38_221;
    ACcp38_270 = qdd(2)-OMcp38_122*ORcp38_323-OMcp38_123*ORcp38_324-OMcp38_124*ORcp38_370-OMcp38_16*ORcp38_321+OMcp38_322*ORcp38_123+OMcp38_323*...
 ORcp38_124+OMcp38_324*ORcp38_170+OMcp38_36*ORcp38_121-OPcp38_122*RLcp38_323-OPcp38_123*RLcp38_324-OPcp38_124*RLcp38_370-OPcp38_16*RLcp38_321+...
 OPcp38_322*RLcp38_123+OPcp38_323*RLcp38_124+OPcp38_324*RLcp38_170+OPcp38_36*RLcp38_121;
    ACcp38_370 = qdd(3)+OMcp38_122*ORcp38_223+OMcp38_123*ORcp38_224+OMcp38_124*ORcp38_270+OMcp38_16*ORcp38_221-OMcp38_222*ORcp38_123-OMcp38_223*...
 ORcp38_124-OMcp38_224*ORcp38_170-OMcp38_26*ORcp38_121+OPcp38_122*RLcp38_223+OPcp38_123*RLcp38_224+OPcp38_124*RLcp38_270+OPcp38_16*RLcp38_221-...
 OPcp38_222*RLcp38_123-OPcp38_223*RLcp38_124-OPcp38_224*RLcp38_170-OPcp38_26*RLcp38_121;

% = = Block_1_0_0_39_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp38_170;
    sens.P(2) = POcp38_270;
    sens.P(3) = POcp38_370;
    sens.R(1,1) = ROcp38_124;
    sens.R(1,2) = ROcp38_224;
    sens.R(1,3) = ROcp38_324;
    sens.R(2,1) = ROcp38_423;
    sens.R(2,2) = ROcp38_523;
    sens.R(2,3) = ROcp38_623;
    sens.R(3,1) = ROcp38_724;
    sens.R(3,2) = ROcp38_824;
    sens.R(3,3) = ROcp38_924;
    sens.V(1) = VIcp38_170;
    sens.V(2) = VIcp38_270;
    sens.V(3) = VIcp38_370;
    sens.OM(1) = OMcp38_124;
    sens.OM(2) = OMcp38_224;
    sens.OM(3) = OMcp38_324;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp38_170_5;
    sens.J(1,6) = JTcp38_170_6;
    sens.J(1,21) = JTcp38_170_7;
    sens.J(1,22) = JTcp38_170_8;
    sens.J(1,23) = JTcp38_170_9;
    sens.J(1,24) = JTcp38_170_10;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp38_270_4;
    sens.J(2,5) = JTcp38_270_5;
    sens.J(2,6) = JTcp38_270_6;
    sens.J(2,21) = JTcp38_270_7;
    sens.J(2,22) = JTcp38_270_8;
    sens.J(2,23) = JTcp38_270_9;
    sens.J(2,24) = JTcp38_270_10;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp38_370_4;
    sens.J(3,5) = JTcp38_370_5;
    sens.J(3,6) = JTcp38_370_6;
    sens.J(3,21) = JTcp38_370_7;
    sens.J(3,22) = JTcp38_370_8;
    sens.J(3,23) = JTcp38_370_9;
    sens.J(3,24) = JTcp38_370_10;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,21) = ROcp38_16;
    sens.J(4,22) = ROcp38_421;
    sens.J(4,23) = ROcp38_722;
    sens.J(4,24) = ROcp38_423;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp38_85;
    sens.J(5,21) = ROcp38_26;
    sens.J(5,22) = ROcp38_521;
    sens.J(5,23) = ROcp38_822;
    sens.J(5,24) = ROcp38_523;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp38_95;
    sens.J(6,21) = ROcp38_36;
    sens.J(6,22) = ROcp38_621;
    sens.J(6,23) = ROcp38_922;
    sens.J(6,24) = ROcp38_623;
    sens.A(1) = ACcp38_170;
    sens.A(2) = ACcp38_270;
    sens.A(3) = ACcp38_370;
    sens.OMP(1) = OPcp38_124;
    sens.OMP(2) = OPcp38_224;
    sens.OMP(3) = OPcp38_324;
 
% 
case 40, 


% = = Block_1_0_0_40_0_1 = = 
 
% Sensor Kinematics 


    ROcp39_25 = S4*S5;
    ROcp39_35 = -C4*S5;
    ROcp39_85 = -S4*C5;
    ROcp39_95 = C4*C5;
    ROcp39_16 = C5*C6;
    ROcp39_26 = ROcp39_25*C6+C4*S6;
    ROcp39_36 = ROcp39_35*C6+S4*S6;
    ROcp39_46 = -C5*S6;
    ROcp39_56 = -(ROcp39_25*S6-C4*C6);
    ROcp39_66 = -(ROcp39_35*S6-S4*C6);
    OMcp39_25 = qd(5)*C4;
    OMcp39_35 = qd(5)*S4;
    OMcp39_16 = qd(4)+qd(6)*S5;
    OMcp39_26 = OMcp39_25+ROcp39_85*qd(6);
    OMcp39_36 = OMcp39_35+ROcp39_95*qd(6);
    OPcp39_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp39_26 = ROcp39_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp39_35*S5-ROcp39_95*qd(4));
    OPcp39_36 = ROcp39_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp39_25*S5-ROcp39_85*qd(4));

% = = Block_1_0_0_40_0_4 = = 
 
% Sensor Kinematics 


    ROcp39_421 = ROcp39_46*C21+S21*S5;
    ROcp39_521 = ROcp39_56*C21+ROcp39_85*S21;
    ROcp39_621 = ROcp39_66*C21+ROcp39_95*S21;
    ROcp39_721 = -(ROcp39_46*S21-C21*S5);
    ROcp39_821 = -(ROcp39_56*S21-ROcp39_85*C21);
    ROcp39_921 = -(ROcp39_66*S21-ROcp39_95*C21);
    ROcp39_122 = ROcp39_16*C22-ROcp39_721*S22;
    ROcp39_222 = ROcp39_26*C22-ROcp39_821*S22;
    ROcp39_322 = ROcp39_36*C22-ROcp39_921*S22;
    ROcp39_722 = ROcp39_16*S22+ROcp39_721*C22;
    ROcp39_822 = ROcp39_26*S22+ROcp39_821*C22;
    ROcp39_922 = ROcp39_36*S22+ROcp39_921*C22;
    ROcp39_123 = ROcp39_122*C23+ROcp39_421*S23;
    ROcp39_223 = ROcp39_222*C23+ROcp39_521*S23;
    ROcp39_323 = ROcp39_322*C23+ROcp39_621*S23;
    ROcp39_423 = -(ROcp39_122*S23-ROcp39_421*C23);
    ROcp39_523 = -(ROcp39_222*S23-ROcp39_521*C23);
    ROcp39_623 = -(ROcp39_322*S23-ROcp39_621*C23);
    RLcp39_121 = ROcp39_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp39_221 = ROcp39_26*s.dpt(1,3)+ROcp39_85*s.dpt(3,3);
    RLcp39_321 = ROcp39_36*s.dpt(1,3)+ROcp39_95*s.dpt(3,3);
    OMcp39_121 = OMcp39_16+ROcp39_16*qd(21);
    OMcp39_221 = OMcp39_26+ROcp39_26*qd(21);
    OMcp39_321 = OMcp39_36+ROcp39_36*qd(21);
    ORcp39_121 = OMcp39_26*RLcp39_321-OMcp39_36*RLcp39_221;
    ORcp39_221 = -(OMcp39_16*RLcp39_321-OMcp39_36*RLcp39_121);
    ORcp39_321 = OMcp39_16*RLcp39_221-OMcp39_26*RLcp39_121;
    OMcp39_122 = OMcp39_121+ROcp39_421*qd(22);
    OMcp39_222 = OMcp39_221+ROcp39_521*qd(22);
    OMcp39_322 = OMcp39_321+ROcp39_621*qd(22);
    OPcp39_122 = OPcp39_16+ROcp39_16*qdd(21)+ROcp39_421*qdd(22)+qd(21)*(OMcp39_26*ROcp39_36-OMcp39_36*ROcp39_26)+qd(22)*(OMcp39_221*ROcp39_621-...
 OMcp39_321*ROcp39_521);
    OPcp39_222 = OPcp39_26+ROcp39_26*qdd(21)+ROcp39_521*qdd(22)-qd(21)*(OMcp39_16*ROcp39_36-OMcp39_36*ROcp39_16)-qd(22)*(OMcp39_121*ROcp39_621-...
 OMcp39_321*ROcp39_421);
    OPcp39_322 = OPcp39_36+ROcp39_36*qdd(21)+ROcp39_621*qdd(22)+qd(21)*(OMcp39_16*ROcp39_26-OMcp39_26*ROcp39_16)+qd(22)*(OMcp39_121*ROcp39_521-...
 OMcp39_221*ROcp39_421);
    RLcp39_123 = ROcp39_722*s.dpt(3,37);
    RLcp39_223 = ROcp39_822*s.dpt(3,37);
    RLcp39_323 = ROcp39_922*s.dpt(3,37);
    OMcp39_123 = OMcp39_122+ROcp39_722*qd(23);
    OMcp39_223 = OMcp39_222+ROcp39_822*qd(23);
    OMcp39_323 = OMcp39_322+ROcp39_922*qd(23);
    ORcp39_123 = OMcp39_222*RLcp39_323-OMcp39_322*RLcp39_223;
    ORcp39_223 = -(OMcp39_122*RLcp39_323-OMcp39_322*RLcp39_123);
    ORcp39_323 = OMcp39_122*RLcp39_223-OMcp39_222*RLcp39_123;
    OPcp39_123 = OPcp39_122+ROcp39_722*qdd(23)+qd(23)*(OMcp39_222*ROcp39_922-OMcp39_322*ROcp39_822);
    OPcp39_223 = OPcp39_222+ROcp39_822*qdd(23)-qd(23)*(OMcp39_122*ROcp39_922-OMcp39_322*ROcp39_722);
    OPcp39_323 = OPcp39_322+ROcp39_922*qdd(23)+qd(23)*(OMcp39_122*ROcp39_822-OMcp39_222*ROcp39_722);

% = = Block_1_0_0_40_0_5 = = 
 
% Sensor Kinematics 


    ROcp39_124 = ROcp39_123*C24-ROcp39_722*S24;
    ROcp39_224 = ROcp39_223*C24-ROcp39_822*S24;
    ROcp39_324 = ROcp39_323*C24-ROcp39_922*S24;
    ROcp39_724 = ROcp39_123*S24+ROcp39_722*C24;
    ROcp39_824 = ROcp39_223*S24+ROcp39_822*C24;
    ROcp39_924 = ROcp39_323*S24+ROcp39_922*C24;
    RLcp39_124 = ROcp39_123*s.dpt(1,41)+ROcp39_423*s.dpt(2,41)+ROcp39_722*s.dpt(3,41);
    RLcp39_224 = ROcp39_223*s.dpt(1,41)+ROcp39_523*s.dpt(2,41)+ROcp39_822*s.dpt(3,41);
    RLcp39_324 = ROcp39_323*s.dpt(1,41)+ROcp39_623*s.dpt(2,41)+ROcp39_922*s.dpt(3,41);
    OMcp39_124 = OMcp39_123+ROcp39_423*qd(24);
    OMcp39_224 = OMcp39_223+ROcp39_523*qd(24);
    OMcp39_324 = OMcp39_323+ROcp39_623*qd(24);
    ORcp39_124 = OMcp39_223*RLcp39_324-OMcp39_323*RLcp39_224;
    ORcp39_224 = -(OMcp39_123*RLcp39_324-OMcp39_323*RLcp39_124);
    ORcp39_324 = OMcp39_123*RLcp39_224-OMcp39_223*RLcp39_124;
    OPcp39_124 = OPcp39_123+ROcp39_423*qdd(24)+qd(24)*(OMcp39_223*ROcp39_623-OMcp39_323*ROcp39_523);
    OPcp39_224 = OPcp39_223+ROcp39_523*qdd(24)-qd(24)*(OMcp39_123*ROcp39_623-OMcp39_323*ROcp39_423);
    OPcp39_324 = OPcp39_323+ROcp39_623*qdd(24)+qd(24)*(OMcp39_123*ROcp39_523-OMcp39_223*ROcp39_423);
    RLcp39_171 = ROcp39_423*s.dpt(2,44);
    RLcp39_271 = ROcp39_523*s.dpt(2,44);
    RLcp39_371 = ROcp39_623*s.dpt(2,44);
    POcp39_171 = RLcp39_121+RLcp39_123+RLcp39_124+RLcp39_171+q(1);
    POcp39_271 = RLcp39_221+RLcp39_223+RLcp39_224+RLcp39_271+q(2);
    POcp39_371 = RLcp39_321+RLcp39_323+RLcp39_324+RLcp39_371+q(3);
    JTcp39_271_4 = -(RLcp39_321+RLcp39_323+RLcp39_324+RLcp39_371);
    JTcp39_371_4 = RLcp39_221+RLcp39_223+RLcp39_224+RLcp39_271;
    JTcp39_171_5 = C4*(RLcp39_321+RLcp39_323+RLcp39_324+RLcp39_371)-S4*(RLcp39_221+RLcp39_223)-S4*(RLcp39_224+RLcp39_271);
    JTcp39_271_5 = S4*(RLcp39_121+RLcp39_123+RLcp39_124+RLcp39_171);
    JTcp39_371_5 = -C4*(RLcp39_121+RLcp39_123+RLcp39_124+RLcp39_171);
    JTcp39_171_6 = ROcp39_85*(RLcp39_321+RLcp39_323+RLcp39_324+RLcp39_371)-ROcp39_95*(RLcp39_221+RLcp39_223)-ROcp39_95*(RLcp39_224+RLcp39_271);
    JTcp39_271_6 = RLcp39_171*ROcp39_95-RLcp39_324*S5-RLcp39_371*S5+ROcp39_95*(RLcp39_121+RLcp39_123+RLcp39_124)-S5*(RLcp39_321+RLcp39_323);
    JTcp39_371_6 = RLcp39_224*S5-ROcp39_85*(RLcp39_121+RLcp39_123+RLcp39_124)+S5*(RLcp39_221+RLcp39_223)-RLcp39_171*ROcp39_85+RLcp39_271*S5;
    JTcp39_171_7 = ROcp39_26*(RLcp39_323+RLcp39_324)-ROcp39_36*(RLcp39_223+RLcp39_224)-RLcp39_271*ROcp39_36+RLcp39_371*ROcp39_26;
    JTcp39_271_7 = RLcp39_171*ROcp39_36-RLcp39_371*ROcp39_16-ROcp39_16*(RLcp39_323+RLcp39_324)+ROcp39_36*(RLcp39_123+RLcp39_124);
    JTcp39_371_7 = ROcp39_16*(RLcp39_223+RLcp39_224)-ROcp39_26*(RLcp39_123+RLcp39_124)-RLcp39_171*ROcp39_26+RLcp39_271*ROcp39_16;
    JTcp39_171_8 = ROcp39_521*(RLcp39_323+RLcp39_324)-ROcp39_621*(RLcp39_223+RLcp39_224)-RLcp39_271*ROcp39_621+RLcp39_371*ROcp39_521;
    JTcp39_271_8 = RLcp39_171*ROcp39_621-RLcp39_371*ROcp39_421-ROcp39_421*(RLcp39_323+RLcp39_324)+ROcp39_621*(RLcp39_123+RLcp39_124);
    JTcp39_371_8 = ROcp39_421*(RLcp39_223+RLcp39_224)-ROcp39_521*(RLcp39_123+RLcp39_124)-RLcp39_171*ROcp39_521+RLcp39_271*ROcp39_421;
    JTcp39_171_9 = ROcp39_822*(RLcp39_324+RLcp39_371)-ROcp39_922*(RLcp39_224+RLcp39_271);
    JTcp39_271_9 = -(ROcp39_722*(RLcp39_324+RLcp39_371)-ROcp39_922*(RLcp39_124+RLcp39_171));
    JTcp39_371_9 = ROcp39_722*(RLcp39_224+RLcp39_271)-ROcp39_822*(RLcp39_124+RLcp39_171);
    JTcp39_171_10 = -(RLcp39_271*ROcp39_623-RLcp39_371*ROcp39_523);
    JTcp39_271_10 = RLcp39_171*ROcp39_623-RLcp39_371*ROcp39_423;
    JTcp39_371_10 = -(RLcp39_171*ROcp39_523-RLcp39_271*ROcp39_423);
    ORcp39_171 = OMcp39_224*RLcp39_371-OMcp39_324*RLcp39_271;
    ORcp39_271 = -(OMcp39_124*RLcp39_371-OMcp39_324*RLcp39_171);
    ORcp39_371 = OMcp39_124*RLcp39_271-OMcp39_224*RLcp39_171;
    VIcp39_171 = ORcp39_121+ORcp39_123+ORcp39_124+ORcp39_171+qd(1);
    VIcp39_271 = ORcp39_221+ORcp39_223+ORcp39_224+ORcp39_271+qd(2);
    VIcp39_371 = ORcp39_321+ORcp39_323+ORcp39_324+ORcp39_371+qd(3);
    ACcp39_171 = qdd(1)+OMcp39_222*ORcp39_323+OMcp39_223*ORcp39_324+OMcp39_224*ORcp39_371+OMcp39_26*ORcp39_321-OMcp39_322*ORcp39_223-OMcp39_323*...
 ORcp39_224-OMcp39_324*ORcp39_271-OMcp39_36*ORcp39_221+OPcp39_222*RLcp39_323+OPcp39_223*RLcp39_324+OPcp39_224*RLcp39_371+OPcp39_26*RLcp39_321-...
 OPcp39_322*RLcp39_223-OPcp39_323*RLcp39_224-OPcp39_324*RLcp39_271-OPcp39_36*RLcp39_221;
    ACcp39_271 = qdd(2)-OMcp39_122*ORcp39_323-OMcp39_123*ORcp39_324-OMcp39_124*ORcp39_371-OMcp39_16*ORcp39_321+OMcp39_322*ORcp39_123+OMcp39_323*...
 ORcp39_124+OMcp39_324*ORcp39_171+OMcp39_36*ORcp39_121-OPcp39_122*RLcp39_323-OPcp39_123*RLcp39_324-OPcp39_124*RLcp39_371-OPcp39_16*RLcp39_321+...
 OPcp39_322*RLcp39_123+OPcp39_323*RLcp39_124+OPcp39_324*RLcp39_171+OPcp39_36*RLcp39_121;
    ACcp39_371 = qdd(3)+OMcp39_122*ORcp39_223+OMcp39_123*ORcp39_224+OMcp39_124*ORcp39_271+OMcp39_16*ORcp39_221-OMcp39_222*ORcp39_123-OMcp39_223*...
 ORcp39_124-OMcp39_224*ORcp39_171-OMcp39_26*ORcp39_121+OPcp39_122*RLcp39_223+OPcp39_123*RLcp39_224+OPcp39_124*RLcp39_271+OPcp39_16*RLcp39_221-...
 OPcp39_222*RLcp39_123-OPcp39_223*RLcp39_124-OPcp39_224*RLcp39_171-OPcp39_26*RLcp39_121;

% = = Block_1_0_0_40_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp39_171;
    sens.P(2) = POcp39_271;
    sens.P(3) = POcp39_371;
    sens.R(1,1) = ROcp39_124;
    sens.R(1,2) = ROcp39_224;
    sens.R(1,3) = ROcp39_324;
    sens.R(2,1) = ROcp39_423;
    sens.R(2,2) = ROcp39_523;
    sens.R(2,3) = ROcp39_623;
    sens.R(3,1) = ROcp39_724;
    sens.R(3,2) = ROcp39_824;
    sens.R(3,3) = ROcp39_924;
    sens.V(1) = VIcp39_171;
    sens.V(2) = VIcp39_271;
    sens.V(3) = VIcp39_371;
    sens.OM(1) = OMcp39_124;
    sens.OM(2) = OMcp39_224;
    sens.OM(3) = OMcp39_324;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp39_171_5;
    sens.J(1,6) = JTcp39_171_6;
    sens.J(1,21) = JTcp39_171_7;
    sens.J(1,22) = JTcp39_171_8;
    sens.J(1,23) = JTcp39_171_9;
    sens.J(1,24) = JTcp39_171_10;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp39_271_4;
    sens.J(2,5) = JTcp39_271_5;
    sens.J(2,6) = JTcp39_271_6;
    sens.J(2,21) = JTcp39_271_7;
    sens.J(2,22) = JTcp39_271_8;
    sens.J(2,23) = JTcp39_271_9;
    sens.J(2,24) = JTcp39_271_10;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp39_371_4;
    sens.J(3,5) = JTcp39_371_5;
    sens.J(3,6) = JTcp39_371_6;
    sens.J(3,21) = JTcp39_371_7;
    sens.J(3,22) = JTcp39_371_8;
    sens.J(3,23) = JTcp39_371_9;
    sens.J(3,24) = JTcp39_371_10;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,21) = ROcp39_16;
    sens.J(4,22) = ROcp39_421;
    sens.J(4,23) = ROcp39_722;
    sens.J(4,24) = ROcp39_423;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp39_85;
    sens.J(5,21) = ROcp39_26;
    sens.J(5,22) = ROcp39_521;
    sens.J(5,23) = ROcp39_822;
    sens.J(5,24) = ROcp39_523;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp39_95;
    sens.J(6,21) = ROcp39_36;
    sens.J(6,22) = ROcp39_621;
    sens.J(6,23) = ROcp39_922;
    sens.J(6,24) = ROcp39_623;
    sens.A(1) = ACcp39_171;
    sens.A(2) = ACcp39_271;
    sens.A(3) = ACcp39_371;
    sens.OMP(1) = OPcp39_124;
    sens.OMP(2) = OPcp39_224;
    sens.OMP(3) = OPcp39_324;
 
% 
case 41, 


% = = Block_1_0_0_41_0_1 = = 
 
% Sensor Kinematics 


    ROcp40_25 = S4*S5;
    ROcp40_35 = -C4*S5;
    ROcp40_85 = -S4*C5;
    ROcp40_95 = C4*C5;
    ROcp40_16 = C5*C6;
    ROcp40_26 = ROcp40_25*C6+C4*S6;
    ROcp40_36 = ROcp40_35*C6+S4*S6;
    ROcp40_46 = -C5*S6;
    ROcp40_56 = -(ROcp40_25*S6-C4*C6);
    ROcp40_66 = -(ROcp40_35*S6-S4*C6);
    OMcp40_25 = qd(5)*C4;
    OMcp40_35 = qd(5)*S4;
    OMcp40_16 = qd(4)+qd(6)*S5;
    OMcp40_26 = OMcp40_25+ROcp40_85*qd(6);
    OMcp40_36 = OMcp40_35+ROcp40_95*qd(6);
    OPcp40_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp40_26 = ROcp40_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp40_35*S5-ROcp40_95*qd(4));
    OPcp40_36 = ROcp40_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp40_25*S5-ROcp40_85*qd(4));

% = = Block_1_0_0_41_0_4 = = 
 
% Sensor Kinematics 


    ROcp40_421 = ROcp40_46*C21+S21*S5;
    ROcp40_521 = ROcp40_56*C21+ROcp40_85*S21;
    ROcp40_621 = ROcp40_66*C21+ROcp40_95*S21;
    ROcp40_721 = -(ROcp40_46*S21-C21*S5);
    ROcp40_821 = -(ROcp40_56*S21-ROcp40_85*C21);
    ROcp40_921 = -(ROcp40_66*S21-ROcp40_95*C21);
    ROcp40_122 = ROcp40_16*C22-ROcp40_721*S22;
    ROcp40_222 = ROcp40_26*C22-ROcp40_821*S22;
    ROcp40_322 = ROcp40_36*C22-ROcp40_921*S22;
    ROcp40_722 = ROcp40_16*S22+ROcp40_721*C22;
    ROcp40_822 = ROcp40_26*S22+ROcp40_821*C22;
    ROcp40_922 = ROcp40_36*S22+ROcp40_921*C22;
    ROcp40_123 = ROcp40_122*C23+ROcp40_421*S23;
    ROcp40_223 = ROcp40_222*C23+ROcp40_521*S23;
    ROcp40_323 = ROcp40_322*C23+ROcp40_621*S23;
    ROcp40_423 = -(ROcp40_122*S23-ROcp40_421*C23);
    ROcp40_523 = -(ROcp40_222*S23-ROcp40_521*C23);
    ROcp40_623 = -(ROcp40_322*S23-ROcp40_621*C23);
    RLcp40_121 = ROcp40_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp40_221 = ROcp40_26*s.dpt(1,3)+ROcp40_85*s.dpt(3,3);
    RLcp40_321 = ROcp40_36*s.dpt(1,3)+ROcp40_95*s.dpt(3,3);
    OMcp40_121 = OMcp40_16+ROcp40_16*qd(21);
    OMcp40_221 = OMcp40_26+ROcp40_26*qd(21);
    OMcp40_321 = OMcp40_36+ROcp40_36*qd(21);
    ORcp40_121 = OMcp40_26*RLcp40_321-OMcp40_36*RLcp40_221;
    ORcp40_221 = -(OMcp40_16*RLcp40_321-OMcp40_36*RLcp40_121);
    ORcp40_321 = OMcp40_16*RLcp40_221-OMcp40_26*RLcp40_121;
    OMcp40_122 = OMcp40_121+ROcp40_421*qd(22);
    OMcp40_222 = OMcp40_221+ROcp40_521*qd(22);
    OMcp40_322 = OMcp40_321+ROcp40_621*qd(22);
    OPcp40_122 = OPcp40_16+ROcp40_16*qdd(21)+ROcp40_421*qdd(22)+qd(21)*(OMcp40_26*ROcp40_36-OMcp40_36*ROcp40_26)+qd(22)*(OMcp40_221*ROcp40_621-...
 OMcp40_321*ROcp40_521);
    OPcp40_222 = OPcp40_26+ROcp40_26*qdd(21)+ROcp40_521*qdd(22)-qd(21)*(OMcp40_16*ROcp40_36-OMcp40_36*ROcp40_16)-qd(22)*(OMcp40_121*ROcp40_621-...
 OMcp40_321*ROcp40_421);
    OPcp40_322 = OPcp40_36+ROcp40_36*qdd(21)+ROcp40_621*qdd(22)+qd(21)*(OMcp40_16*ROcp40_26-OMcp40_26*ROcp40_16)+qd(22)*(OMcp40_121*ROcp40_521-...
 OMcp40_221*ROcp40_421);
    RLcp40_123 = ROcp40_722*s.dpt(3,37);
    RLcp40_223 = ROcp40_822*s.dpt(3,37);
    RLcp40_323 = ROcp40_922*s.dpt(3,37);
    OMcp40_123 = OMcp40_122+ROcp40_722*qd(23);
    OMcp40_223 = OMcp40_222+ROcp40_822*qd(23);
    OMcp40_323 = OMcp40_322+ROcp40_922*qd(23);
    ORcp40_123 = OMcp40_222*RLcp40_323-OMcp40_322*RLcp40_223;
    ORcp40_223 = -(OMcp40_122*RLcp40_323-OMcp40_322*RLcp40_123);
    ORcp40_323 = OMcp40_122*RLcp40_223-OMcp40_222*RLcp40_123;
    OPcp40_123 = OPcp40_122+ROcp40_722*qdd(23)+qd(23)*(OMcp40_222*ROcp40_922-OMcp40_322*ROcp40_822);
    OPcp40_223 = OPcp40_222+ROcp40_822*qdd(23)-qd(23)*(OMcp40_122*ROcp40_922-OMcp40_322*ROcp40_722);
    OPcp40_323 = OPcp40_322+ROcp40_922*qdd(23)+qd(23)*(OMcp40_122*ROcp40_822-OMcp40_222*ROcp40_722);

% = = Block_1_0_0_41_0_5 = = 
 
% Sensor Kinematics 


    ROcp40_124 = ROcp40_123*C24-ROcp40_722*S24;
    ROcp40_224 = ROcp40_223*C24-ROcp40_822*S24;
    ROcp40_324 = ROcp40_323*C24-ROcp40_922*S24;
    ROcp40_724 = ROcp40_123*S24+ROcp40_722*C24;
    ROcp40_824 = ROcp40_223*S24+ROcp40_822*C24;
    ROcp40_924 = ROcp40_323*S24+ROcp40_922*C24;
    ROcp40_425 = ROcp40_423*C25+ROcp40_724*S25;
    ROcp40_525 = ROcp40_523*C25+ROcp40_824*S25;
    ROcp40_625 = ROcp40_623*C25+ROcp40_924*S25;
    ROcp40_725 = -(ROcp40_423*S25-ROcp40_724*C25);
    ROcp40_825 = -(ROcp40_523*S25-ROcp40_824*C25);
    ROcp40_925 = -(ROcp40_623*S25-ROcp40_924*C25);
    RLcp40_124 = ROcp40_123*s.dpt(1,41)+ROcp40_423*s.dpt(2,41)+ROcp40_722*s.dpt(3,41);
    RLcp40_224 = ROcp40_223*s.dpt(1,41)+ROcp40_523*s.dpt(2,41)+ROcp40_822*s.dpt(3,41);
    RLcp40_324 = ROcp40_323*s.dpt(1,41)+ROcp40_623*s.dpt(2,41)+ROcp40_922*s.dpt(3,41);
    OMcp40_124 = OMcp40_123+ROcp40_423*qd(24);
    OMcp40_224 = OMcp40_223+ROcp40_523*qd(24);
    OMcp40_324 = OMcp40_323+ROcp40_623*qd(24);
    ORcp40_124 = OMcp40_223*RLcp40_324-OMcp40_323*RLcp40_224;
    ORcp40_224 = -(OMcp40_123*RLcp40_324-OMcp40_323*RLcp40_124);
    ORcp40_324 = OMcp40_123*RLcp40_224-OMcp40_223*RLcp40_124;
    OPcp40_124 = OPcp40_123+ROcp40_423*qdd(24)+qd(24)*(OMcp40_223*ROcp40_623-OMcp40_323*ROcp40_523);
    OPcp40_224 = OPcp40_223+ROcp40_523*qdd(24)-qd(24)*(OMcp40_123*ROcp40_623-OMcp40_323*ROcp40_423);
    OPcp40_324 = OPcp40_323+ROcp40_623*qdd(24)+qd(24)*(OMcp40_123*ROcp40_523-OMcp40_223*ROcp40_423);
    RLcp40_125 = ROcp40_423*s.dpt(2,44);
    RLcp40_225 = ROcp40_523*s.dpt(2,44);
    RLcp40_325 = ROcp40_623*s.dpt(2,44);
    OMcp40_125 = OMcp40_124+ROcp40_124*qd(25);
    OMcp40_225 = OMcp40_224+ROcp40_224*qd(25);
    OMcp40_325 = OMcp40_324+ROcp40_324*qd(25);
    ORcp40_125 = OMcp40_224*RLcp40_325-OMcp40_324*RLcp40_225;
    ORcp40_225 = -(OMcp40_124*RLcp40_325-OMcp40_324*RLcp40_125);
    ORcp40_325 = OMcp40_124*RLcp40_225-OMcp40_224*RLcp40_125;
    OPcp40_125 = OPcp40_124+ROcp40_124*qdd(25)+qd(25)*(OMcp40_224*ROcp40_324-OMcp40_324*ROcp40_224);
    OPcp40_225 = OPcp40_224+ROcp40_224*qdd(25)-qd(25)*(OMcp40_124*ROcp40_324-OMcp40_324*ROcp40_124);
    OPcp40_325 = OPcp40_324+ROcp40_324*qdd(25)+qd(25)*(OMcp40_124*ROcp40_224-OMcp40_224*ROcp40_124);
    RLcp40_172 = ROcp40_124*s.dpt(1,45)+ROcp40_425*s.dpt(2,45)+ROcp40_725*s.dpt(3,45);
    RLcp40_272 = ROcp40_224*s.dpt(1,45)+ROcp40_525*s.dpt(2,45)+ROcp40_825*s.dpt(3,45);
    RLcp40_372 = ROcp40_324*s.dpt(1,45)+ROcp40_625*s.dpt(2,45)+ROcp40_925*s.dpt(3,45);
    POcp40_172 = RLcp40_121+RLcp40_123+RLcp40_124+RLcp40_125+RLcp40_172+q(1);
    POcp40_272 = RLcp40_221+RLcp40_223+RLcp40_224+RLcp40_225+RLcp40_272+q(2);
    POcp40_372 = RLcp40_321+RLcp40_323+RLcp40_324+RLcp40_325+RLcp40_372+q(3);
    JTcp40_272_4 = -(RLcp40_321+RLcp40_323+RLcp40_324+RLcp40_325+RLcp40_372);
    JTcp40_372_4 = RLcp40_221+RLcp40_223+RLcp40_224+RLcp40_225+RLcp40_272;
    JTcp40_172_5 = C4*(RLcp40_321+RLcp40_323+RLcp40_324+RLcp40_325)-S4*(RLcp40_221+RLcp40_223)-S4*(RLcp40_224+RLcp40_225)-RLcp40_272*S4+RLcp40_372*...
 C4;
    JTcp40_272_5 = S4*(RLcp40_121+RLcp40_123+RLcp40_124+RLcp40_125+RLcp40_172);
    JTcp40_372_5 = -C4*(RLcp40_121+RLcp40_123+RLcp40_124+RLcp40_125+RLcp40_172);
    JTcp40_172_6 = ROcp40_85*(RLcp40_321+RLcp40_323+RLcp40_324+RLcp40_325)-ROcp40_95*(RLcp40_221+RLcp40_223)-ROcp40_95*(RLcp40_224+RLcp40_225)-...
 RLcp40_272*ROcp40_95+RLcp40_372*ROcp40_85;
    JTcp40_272_6 = -(RLcp40_372*S5-ROcp40_95*(RLcp40_121+RLcp40_123+RLcp40_124+RLcp40_125+RLcp40_172)+S5*(RLcp40_321+RLcp40_323)+S5*(RLcp40_324+...
 RLcp40_325));
    JTcp40_372_6 = RLcp40_272*S5-ROcp40_85*(RLcp40_121+RLcp40_123+RLcp40_124+RLcp40_125+RLcp40_172)+S5*(RLcp40_221+RLcp40_223)+S5*(RLcp40_224+...
 RLcp40_225);
    JTcp40_172_7 = ROcp40_26*(RLcp40_323+RLcp40_324+RLcp40_325+RLcp40_372)-ROcp40_36*(RLcp40_223+RLcp40_224)-ROcp40_36*(RLcp40_225+RLcp40_272);
    JTcp40_272_7 = -(ROcp40_16*(RLcp40_323+RLcp40_324+RLcp40_325+RLcp40_372)-ROcp40_36*(RLcp40_123+RLcp40_124)-ROcp40_36*(RLcp40_125+RLcp40_172));
    JTcp40_372_7 = ROcp40_16*(RLcp40_223+RLcp40_224+RLcp40_225+RLcp40_272)-ROcp40_26*(RLcp40_123+RLcp40_124)-ROcp40_26*(RLcp40_125+RLcp40_172);
    JTcp40_172_8 = ROcp40_521*(RLcp40_323+RLcp40_324+RLcp40_325+RLcp40_372)-ROcp40_621*(RLcp40_223+RLcp40_224)-ROcp40_621*(RLcp40_225+RLcp40_272);
    JTcp40_272_8 = -(ROcp40_421*(RLcp40_323+RLcp40_324+RLcp40_325+RLcp40_372)-ROcp40_621*(RLcp40_123+RLcp40_124)-ROcp40_621*(RLcp40_125+RLcp40_172)...
 );
    JTcp40_372_8 = ROcp40_421*(RLcp40_223+RLcp40_224+RLcp40_225+RLcp40_272)-ROcp40_521*(RLcp40_123+RLcp40_124)-ROcp40_521*(RLcp40_125+RLcp40_172);
    JTcp40_172_9 = ROcp40_822*(RLcp40_324+RLcp40_325)-ROcp40_922*(RLcp40_224+RLcp40_225)-RLcp40_272*ROcp40_922+RLcp40_372*ROcp40_822;
    JTcp40_272_9 = RLcp40_172*ROcp40_922-RLcp40_372*ROcp40_722-ROcp40_722*(RLcp40_324+RLcp40_325)+ROcp40_922*(RLcp40_124+RLcp40_125);
    JTcp40_372_9 = ROcp40_722*(RLcp40_224+RLcp40_225)-ROcp40_822*(RLcp40_124+RLcp40_125)-RLcp40_172*ROcp40_822+RLcp40_272*ROcp40_722;
    JTcp40_172_10 = ROcp40_523*(RLcp40_325+RLcp40_372)-ROcp40_623*(RLcp40_225+RLcp40_272);
    JTcp40_272_10 = -(ROcp40_423*(RLcp40_325+RLcp40_372)-ROcp40_623*(RLcp40_125+RLcp40_172));
    JTcp40_372_10 = ROcp40_423*(RLcp40_225+RLcp40_272)-ROcp40_523*(RLcp40_125+RLcp40_172);
    JTcp40_172_11 = -(RLcp40_272*ROcp40_324-RLcp40_372*ROcp40_224);
    JTcp40_272_11 = RLcp40_172*ROcp40_324-RLcp40_372*ROcp40_124;
    JTcp40_372_11 = -(RLcp40_172*ROcp40_224-RLcp40_272*ROcp40_124);
    ORcp40_172 = OMcp40_225*RLcp40_372-OMcp40_325*RLcp40_272;
    ORcp40_272 = -(OMcp40_125*RLcp40_372-OMcp40_325*RLcp40_172);
    ORcp40_372 = OMcp40_125*RLcp40_272-OMcp40_225*RLcp40_172;
    VIcp40_172 = ORcp40_121+ORcp40_123+ORcp40_124+ORcp40_125+ORcp40_172+qd(1);
    VIcp40_272 = ORcp40_221+ORcp40_223+ORcp40_224+ORcp40_225+ORcp40_272+qd(2);
    VIcp40_372 = ORcp40_321+ORcp40_323+ORcp40_324+ORcp40_325+ORcp40_372+qd(3);
    ACcp40_172 = qdd(1)+OMcp40_222*ORcp40_323+OMcp40_223*ORcp40_324+OMcp40_224*ORcp40_325+OMcp40_225*ORcp40_372+OMcp40_26*ORcp40_321-OMcp40_322*...
 ORcp40_223-OMcp40_323*ORcp40_224-OMcp40_324*ORcp40_225-OMcp40_325*ORcp40_272-OMcp40_36*ORcp40_221+OPcp40_222*RLcp40_323+OPcp40_223*RLcp40_324+...
 OPcp40_224*RLcp40_325+OPcp40_225*RLcp40_372+OPcp40_26*RLcp40_321-OPcp40_322*RLcp40_223-OPcp40_323*RLcp40_224-OPcp40_324*RLcp40_225-OPcp40_325*...
 RLcp40_272-OPcp40_36*RLcp40_221;
    ACcp40_272 = qdd(2)-OMcp40_122*ORcp40_323-OMcp40_123*ORcp40_324-OMcp40_124*ORcp40_325-OMcp40_125*ORcp40_372-OMcp40_16*ORcp40_321+OMcp40_322*...
 ORcp40_123+OMcp40_323*ORcp40_124+OMcp40_324*ORcp40_125+OMcp40_325*ORcp40_172+OMcp40_36*ORcp40_121-OPcp40_122*RLcp40_323-OPcp40_123*RLcp40_324-...
 OPcp40_124*RLcp40_325-OPcp40_125*RLcp40_372-OPcp40_16*RLcp40_321+OPcp40_322*RLcp40_123+OPcp40_323*RLcp40_124+OPcp40_324*RLcp40_125+OPcp40_325*...
 RLcp40_172+OPcp40_36*RLcp40_121;
    ACcp40_372 = qdd(3)+OMcp40_122*ORcp40_223+OMcp40_123*ORcp40_224+OMcp40_124*ORcp40_225+OMcp40_125*ORcp40_272+OMcp40_16*ORcp40_221-OMcp40_222*...
 ORcp40_123-OMcp40_223*ORcp40_124-OMcp40_224*ORcp40_125-OMcp40_225*ORcp40_172-OMcp40_26*ORcp40_121+OPcp40_122*RLcp40_223+OPcp40_123*RLcp40_224+...
 OPcp40_124*RLcp40_225+OPcp40_125*RLcp40_272+OPcp40_16*RLcp40_221-OPcp40_222*RLcp40_123-OPcp40_223*RLcp40_124-OPcp40_224*RLcp40_125-OPcp40_225*...
 RLcp40_172-OPcp40_26*RLcp40_121;

% = = Block_1_0_0_41_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp40_172;
    sens.P(2) = POcp40_272;
    sens.P(3) = POcp40_372;
    sens.R(1,1) = ROcp40_124;
    sens.R(1,2) = ROcp40_224;
    sens.R(1,3) = ROcp40_324;
    sens.R(2,1) = ROcp40_425;
    sens.R(2,2) = ROcp40_525;
    sens.R(2,3) = ROcp40_625;
    sens.R(3,1) = ROcp40_725;
    sens.R(3,2) = ROcp40_825;
    sens.R(3,3) = ROcp40_925;
    sens.V(1) = VIcp40_172;
    sens.V(2) = VIcp40_272;
    sens.V(3) = VIcp40_372;
    sens.OM(1) = OMcp40_125;
    sens.OM(2) = OMcp40_225;
    sens.OM(3) = OMcp40_325;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp40_172_5;
    sens.J(1,6) = JTcp40_172_6;
    sens.J(1,21) = JTcp40_172_7;
    sens.J(1,22) = JTcp40_172_8;
    sens.J(1,23) = JTcp40_172_9;
    sens.J(1,24) = JTcp40_172_10;
    sens.J(1,25) = JTcp40_172_11;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp40_272_4;
    sens.J(2,5) = JTcp40_272_5;
    sens.J(2,6) = JTcp40_272_6;
    sens.J(2,21) = JTcp40_272_7;
    sens.J(2,22) = JTcp40_272_8;
    sens.J(2,23) = JTcp40_272_9;
    sens.J(2,24) = JTcp40_272_10;
    sens.J(2,25) = JTcp40_272_11;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp40_372_4;
    sens.J(3,5) = JTcp40_372_5;
    sens.J(3,6) = JTcp40_372_6;
    sens.J(3,21) = JTcp40_372_7;
    sens.J(3,22) = JTcp40_372_8;
    sens.J(3,23) = JTcp40_372_9;
    sens.J(3,24) = JTcp40_372_10;
    sens.J(3,25) = JTcp40_372_11;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,21) = ROcp40_16;
    sens.J(4,22) = ROcp40_421;
    sens.J(4,23) = ROcp40_722;
    sens.J(4,24) = ROcp40_423;
    sens.J(4,25) = ROcp40_124;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp40_85;
    sens.J(5,21) = ROcp40_26;
    sens.J(5,22) = ROcp40_521;
    sens.J(5,23) = ROcp40_822;
    sens.J(5,24) = ROcp40_523;
    sens.J(5,25) = ROcp40_224;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp40_95;
    sens.J(6,21) = ROcp40_36;
    sens.J(6,22) = ROcp40_621;
    sens.J(6,23) = ROcp40_922;
    sens.J(6,24) = ROcp40_623;
    sens.J(6,25) = ROcp40_324;
    sens.A(1) = ACcp40_172;
    sens.A(2) = ACcp40_272;
    sens.A(3) = ACcp40_372;
    sens.OMP(1) = OPcp40_125;
    sens.OMP(2) = OPcp40_225;
    sens.OMP(3) = OPcp40_325;
 
% 
case 42, 


% = = Block_1_0_0_42_0_1 = = 
 
% Sensor Kinematics 


    ROcp41_25 = S4*S5;
    ROcp41_35 = -C4*S5;
    ROcp41_85 = -S4*C5;
    ROcp41_95 = C4*C5;
    ROcp41_16 = C5*C6;
    ROcp41_26 = ROcp41_25*C6+C4*S6;
    ROcp41_36 = ROcp41_35*C6+S4*S6;
    ROcp41_46 = -C5*S6;
    ROcp41_56 = -(ROcp41_25*S6-C4*C6);
    ROcp41_66 = -(ROcp41_35*S6-S4*C6);
    OMcp41_25 = qd(5)*C4;
    OMcp41_35 = qd(5)*S4;
    OMcp41_16 = qd(4)+qd(6)*S5;
    OMcp41_26 = OMcp41_25+ROcp41_85*qd(6);
    OMcp41_36 = OMcp41_35+ROcp41_95*qd(6);
    OPcp41_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp41_26 = ROcp41_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp41_35*S5-ROcp41_95*qd(4));
    OPcp41_36 = ROcp41_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp41_25*S5-ROcp41_85*qd(4));

% = = Block_1_0_0_42_0_4 = = 
 
% Sensor Kinematics 


    ROcp41_421 = ROcp41_46*C21+S21*S5;
    ROcp41_521 = ROcp41_56*C21+ROcp41_85*S21;
    ROcp41_621 = ROcp41_66*C21+ROcp41_95*S21;
    ROcp41_721 = -(ROcp41_46*S21-C21*S5);
    ROcp41_821 = -(ROcp41_56*S21-ROcp41_85*C21);
    ROcp41_921 = -(ROcp41_66*S21-ROcp41_95*C21);
    ROcp41_122 = ROcp41_16*C22-ROcp41_721*S22;
    ROcp41_222 = ROcp41_26*C22-ROcp41_821*S22;
    ROcp41_322 = ROcp41_36*C22-ROcp41_921*S22;
    ROcp41_722 = ROcp41_16*S22+ROcp41_721*C22;
    ROcp41_822 = ROcp41_26*S22+ROcp41_821*C22;
    ROcp41_922 = ROcp41_36*S22+ROcp41_921*C22;
    ROcp41_123 = ROcp41_122*C23+ROcp41_421*S23;
    ROcp41_223 = ROcp41_222*C23+ROcp41_521*S23;
    ROcp41_323 = ROcp41_322*C23+ROcp41_621*S23;
    ROcp41_423 = -(ROcp41_122*S23-ROcp41_421*C23);
    ROcp41_523 = -(ROcp41_222*S23-ROcp41_521*C23);
    ROcp41_623 = -(ROcp41_322*S23-ROcp41_621*C23);
    RLcp41_121 = ROcp41_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp41_221 = ROcp41_26*s.dpt(1,3)+ROcp41_85*s.dpt(3,3);
    RLcp41_321 = ROcp41_36*s.dpt(1,3)+ROcp41_95*s.dpt(3,3);
    OMcp41_121 = OMcp41_16+ROcp41_16*qd(21);
    OMcp41_221 = OMcp41_26+ROcp41_26*qd(21);
    OMcp41_321 = OMcp41_36+ROcp41_36*qd(21);
    ORcp41_121 = OMcp41_26*RLcp41_321-OMcp41_36*RLcp41_221;
    ORcp41_221 = -(OMcp41_16*RLcp41_321-OMcp41_36*RLcp41_121);
    ORcp41_321 = OMcp41_16*RLcp41_221-OMcp41_26*RLcp41_121;
    OMcp41_122 = OMcp41_121+ROcp41_421*qd(22);
    OMcp41_222 = OMcp41_221+ROcp41_521*qd(22);
    OMcp41_322 = OMcp41_321+ROcp41_621*qd(22);
    OPcp41_122 = OPcp41_16+ROcp41_16*qdd(21)+ROcp41_421*qdd(22)+qd(21)*(OMcp41_26*ROcp41_36-OMcp41_36*ROcp41_26)+qd(22)*(OMcp41_221*ROcp41_621-...
 OMcp41_321*ROcp41_521);
    OPcp41_222 = OPcp41_26+ROcp41_26*qdd(21)+ROcp41_521*qdd(22)-qd(21)*(OMcp41_16*ROcp41_36-OMcp41_36*ROcp41_16)-qd(22)*(OMcp41_121*ROcp41_621-...
 OMcp41_321*ROcp41_421);
    OPcp41_322 = OPcp41_36+ROcp41_36*qdd(21)+ROcp41_621*qdd(22)+qd(21)*(OMcp41_16*ROcp41_26-OMcp41_26*ROcp41_16)+qd(22)*(OMcp41_121*ROcp41_521-...
 OMcp41_221*ROcp41_421);
    RLcp41_123 = ROcp41_722*s.dpt(3,37);
    RLcp41_223 = ROcp41_822*s.dpt(3,37);
    RLcp41_323 = ROcp41_922*s.dpt(3,37);
    OMcp41_123 = OMcp41_122+ROcp41_722*qd(23);
    OMcp41_223 = OMcp41_222+ROcp41_822*qd(23);
    OMcp41_323 = OMcp41_322+ROcp41_922*qd(23);
    ORcp41_123 = OMcp41_222*RLcp41_323-OMcp41_322*RLcp41_223;
    ORcp41_223 = -(OMcp41_122*RLcp41_323-OMcp41_322*RLcp41_123);
    ORcp41_323 = OMcp41_122*RLcp41_223-OMcp41_222*RLcp41_123;
    OPcp41_123 = OPcp41_122+ROcp41_722*qdd(23)+qd(23)*(OMcp41_222*ROcp41_922-OMcp41_322*ROcp41_822);
    OPcp41_223 = OPcp41_222+ROcp41_822*qdd(23)-qd(23)*(OMcp41_122*ROcp41_922-OMcp41_322*ROcp41_722);
    OPcp41_323 = OPcp41_322+ROcp41_922*qdd(23)+qd(23)*(OMcp41_122*ROcp41_822-OMcp41_222*ROcp41_722);

% = = Block_1_0_0_42_0_5 = = 
 
% Sensor Kinematics 


    ROcp41_124 = ROcp41_123*C24-ROcp41_722*S24;
    ROcp41_224 = ROcp41_223*C24-ROcp41_822*S24;
    ROcp41_324 = ROcp41_323*C24-ROcp41_922*S24;
    ROcp41_724 = ROcp41_123*S24+ROcp41_722*C24;
    ROcp41_824 = ROcp41_223*S24+ROcp41_822*C24;
    ROcp41_924 = ROcp41_323*S24+ROcp41_922*C24;
    ROcp41_425 = ROcp41_423*C25+ROcp41_724*S25;
    ROcp41_525 = ROcp41_523*C25+ROcp41_824*S25;
    ROcp41_625 = ROcp41_623*C25+ROcp41_924*S25;
    ROcp41_725 = -(ROcp41_423*S25-ROcp41_724*C25);
    ROcp41_825 = -(ROcp41_523*S25-ROcp41_824*C25);
    ROcp41_925 = -(ROcp41_623*S25-ROcp41_924*C25);
    RLcp41_124 = ROcp41_123*s.dpt(1,41)+ROcp41_423*s.dpt(2,41)+ROcp41_722*s.dpt(3,41);
    RLcp41_224 = ROcp41_223*s.dpt(1,41)+ROcp41_523*s.dpt(2,41)+ROcp41_822*s.dpt(3,41);
    RLcp41_324 = ROcp41_323*s.dpt(1,41)+ROcp41_623*s.dpt(2,41)+ROcp41_922*s.dpt(3,41);
    OMcp41_124 = OMcp41_123+ROcp41_423*qd(24);
    OMcp41_224 = OMcp41_223+ROcp41_523*qd(24);
    OMcp41_324 = OMcp41_323+ROcp41_623*qd(24);
    ORcp41_124 = OMcp41_223*RLcp41_324-OMcp41_323*RLcp41_224;
    ORcp41_224 = -(OMcp41_123*RLcp41_324-OMcp41_323*RLcp41_124);
    ORcp41_324 = OMcp41_123*RLcp41_224-OMcp41_223*RLcp41_124;
    OPcp41_124 = OPcp41_123+ROcp41_423*qdd(24)+qd(24)*(OMcp41_223*ROcp41_623-OMcp41_323*ROcp41_523);
    OPcp41_224 = OPcp41_223+ROcp41_523*qdd(24)-qd(24)*(OMcp41_123*ROcp41_623-OMcp41_323*ROcp41_423);
    OPcp41_324 = OPcp41_323+ROcp41_623*qdd(24)+qd(24)*(OMcp41_123*ROcp41_523-OMcp41_223*ROcp41_423);
    RLcp41_125 = ROcp41_423*s.dpt(2,44);
    RLcp41_225 = ROcp41_523*s.dpt(2,44);
    RLcp41_325 = ROcp41_623*s.dpt(2,44);
    OMcp41_125 = OMcp41_124+ROcp41_124*qd(25);
    OMcp41_225 = OMcp41_224+ROcp41_224*qd(25);
    OMcp41_325 = OMcp41_324+ROcp41_324*qd(25);
    ORcp41_125 = OMcp41_224*RLcp41_325-OMcp41_324*RLcp41_225;
    ORcp41_225 = -(OMcp41_124*RLcp41_325-OMcp41_324*RLcp41_125);
    ORcp41_325 = OMcp41_124*RLcp41_225-OMcp41_224*RLcp41_125;
    OPcp41_125 = OPcp41_124+ROcp41_124*qdd(25)+qd(25)*(OMcp41_224*ROcp41_324-OMcp41_324*ROcp41_224);
    OPcp41_225 = OPcp41_224+ROcp41_224*qdd(25)-qd(25)*(OMcp41_124*ROcp41_324-OMcp41_324*ROcp41_124);
    OPcp41_325 = OPcp41_324+ROcp41_324*qdd(25)+qd(25)*(OMcp41_124*ROcp41_224-OMcp41_224*ROcp41_124);
    RLcp41_173 = ROcp41_725*s.dpt(3,46);
    RLcp41_273 = ROcp41_825*s.dpt(3,46);
    RLcp41_373 = ROcp41_925*s.dpt(3,46);
    POcp41_173 = RLcp41_121+RLcp41_123+RLcp41_124+RLcp41_125+RLcp41_173+q(1);
    POcp41_273 = RLcp41_221+RLcp41_223+RLcp41_224+RLcp41_225+RLcp41_273+q(2);
    POcp41_373 = RLcp41_321+RLcp41_323+RLcp41_324+RLcp41_325+RLcp41_373+q(3);
    JTcp41_273_4 = -(RLcp41_321+RLcp41_323+RLcp41_324+RLcp41_325+RLcp41_373);
    JTcp41_373_4 = RLcp41_221+RLcp41_223+RLcp41_224+RLcp41_225+RLcp41_273;
    JTcp41_173_5 = C4*(RLcp41_321+RLcp41_323+RLcp41_324+RLcp41_325)-S4*(RLcp41_221+RLcp41_223)-S4*(RLcp41_224+RLcp41_225)-RLcp41_273*S4+RLcp41_373*...
 C4;
    JTcp41_273_5 = S4*(RLcp41_121+RLcp41_123+RLcp41_124+RLcp41_125+RLcp41_173);
    JTcp41_373_5 = -C4*(RLcp41_121+RLcp41_123+RLcp41_124+RLcp41_125+RLcp41_173);
    JTcp41_173_6 = ROcp41_85*(RLcp41_321+RLcp41_323+RLcp41_324+RLcp41_325)-ROcp41_95*(RLcp41_221+RLcp41_223)-ROcp41_95*(RLcp41_224+RLcp41_225)-...
 RLcp41_273*ROcp41_95+RLcp41_373*ROcp41_85;
    JTcp41_273_6 = -(RLcp41_373*S5-ROcp41_95*(RLcp41_121+RLcp41_123+RLcp41_124+RLcp41_125+RLcp41_173)+S5*(RLcp41_321+RLcp41_323)+S5*(RLcp41_324+...
 RLcp41_325));
    JTcp41_373_6 = RLcp41_273*S5-ROcp41_85*(RLcp41_121+RLcp41_123+RLcp41_124+RLcp41_125+RLcp41_173)+S5*(RLcp41_221+RLcp41_223)+S5*(RLcp41_224+...
 RLcp41_225);
    JTcp41_173_7 = ROcp41_26*(RLcp41_323+RLcp41_324+RLcp41_325+RLcp41_373)-ROcp41_36*(RLcp41_223+RLcp41_224)-ROcp41_36*(RLcp41_225+RLcp41_273);
    JTcp41_273_7 = -(ROcp41_16*(RLcp41_323+RLcp41_324+RLcp41_325+RLcp41_373)-ROcp41_36*(RLcp41_123+RLcp41_124)-ROcp41_36*(RLcp41_125+RLcp41_173));
    JTcp41_373_7 = ROcp41_16*(RLcp41_223+RLcp41_224+RLcp41_225+RLcp41_273)-ROcp41_26*(RLcp41_123+RLcp41_124)-ROcp41_26*(RLcp41_125+RLcp41_173);
    JTcp41_173_8 = ROcp41_521*(RLcp41_323+RLcp41_324+RLcp41_325+RLcp41_373)-ROcp41_621*(RLcp41_223+RLcp41_224)-ROcp41_621*(RLcp41_225+RLcp41_273);
    JTcp41_273_8 = -(ROcp41_421*(RLcp41_323+RLcp41_324+RLcp41_325+RLcp41_373)-ROcp41_621*(RLcp41_123+RLcp41_124)-ROcp41_621*(RLcp41_125+RLcp41_173)...
 );
    JTcp41_373_8 = ROcp41_421*(RLcp41_223+RLcp41_224+RLcp41_225+RLcp41_273)-ROcp41_521*(RLcp41_123+RLcp41_124)-ROcp41_521*(RLcp41_125+RLcp41_173);
    JTcp41_173_9 = ROcp41_822*(RLcp41_324+RLcp41_325)-ROcp41_922*(RLcp41_224+RLcp41_225)-RLcp41_273*ROcp41_922+RLcp41_373*ROcp41_822;
    JTcp41_273_9 = RLcp41_173*ROcp41_922-RLcp41_373*ROcp41_722-ROcp41_722*(RLcp41_324+RLcp41_325)+ROcp41_922*(RLcp41_124+RLcp41_125);
    JTcp41_373_9 = ROcp41_722*(RLcp41_224+RLcp41_225)-ROcp41_822*(RLcp41_124+RLcp41_125)-RLcp41_173*ROcp41_822+RLcp41_273*ROcp41_722;
    JTcp41_173_10 = ROcp41_523*(RLcp41_325+RLcp41_373)-ROcp41_623*(RLcp41_225+RLcp41_273);
    JTcp41_273_10 = -(ROcp41_423*(RLcp41_325+RLcp41_373)-ROcp41_623*(RLcp41_125+RLcp41_173));
    JTcp41_373_10 = ROcp41_423*(RLcp41_225+RLcp41_273)-ROcp41_523*(RLcp41_125+RLcp41_173);
    JTcp41_173_11 = -(RLcp41_273*ROcp41_324-RLcp41_373*ROcp41_224);
    JTcp41_273_11 = RLcp41_173*ROcp41_324-RLcp41_373*ROcp41_124;
    JTcp41_373_11 = -(RLcp41_173*ROcp41_224-RLcp41_273*ROcp41_124);
    ORcp41_173 = OMcp41_225*RLcp41_373-OMcp41_325*RLcp41_273;
    ORcp41_273 = -(OMcp41_125*RLcp41_373-OMcp41_325*RLcp41_173);
    ORcp41_373 = OMcp41_125*RLcp41_273-OMcp41_225*RLcp41_173;
    VIcp41_173 = ORcp41_121+ORcp41_123+ORcp41_124+ORcp41_125+ORcp41_173+qd(1);
    VIcp41_273 = ORcp41_221+ORcp41_223+ORcp41_224+ORcp41_225+ORcp41_273+qd(2);
    VIcp41_373 = ORcp41_321+ORcp41_323+ORcp41_324+ORcp41_325+ORcp41_373+qd(3);
    ACcp41_173 = qdd(1)+OMcp41_222*ORcp41_323+OMcp41_223*ORcp41_324+OMcp41_224*ORcp41_325+OMcp41_225*ORcp41_373+OMcp41_26*ORcp41_321-OMcp41_322*...
 ORcp41_223-OMcp41_323*ORcp41_224-OMcp41_324*ORcp41_225-OMcp41_325*ORcp41_273-OMcp41_36*ORcp41_221+OPcp41_222*RLcp41_323+OPcp41_223*RLcp41_324+...
 OPcp41_224*RLcp41_325+OPcp41_225*RLcp41_373+OPcp41_26*RLcp41_321-OPcp41_322*RLcp41_223-OPcp41_323*RLcp41_224-OPcp41_324*RLcp41_225-OPcp41_325*...
 RLcp41_273-OPcp41_36*RLcp41_221;
    ACcp41_273 = qdd(2)-OMcp41_122*ORcp41_323-OMcp41_123*ORcp41_324-OMcp41_124*ORcp41_325-OMcp41_125*ORcp41_373-OMcp41_16*ORcp41_321+OMcp41_322*...
 ORcp41_123+OMcp41_323*ORcp41_124+OMcp41_324*ORcp41_125+OMcp41_325*ORcp41_173+OMcp41_36*ORcp41_121-OPcp41_122*RLcp41_323-OPcp41_123*RLcp41_324-...
 OPcp41_124*RLcp41_325-OPcp41_125*RLcp41_373-OPcp41_16*RLcp41_321+OPcp41_322*RLcp41_123+OPcp41_323*RLcp41_124+OPcp41_324*RLcp41_125+OPcp41_325*...
 RLcp41_173+OPcp41_36*RLcp41_121;
    ACcp41_373 = qdd(3)+OMcp41_122*ORcp41_223+OMcp41_123*ORcp41_224+OMcp41_124*ORcp41_225+OMcp41_125*ORcp41_273+OMcp41_16*ORcp41_221-OMcp41_222*...
 ORcp41_123-OMcp41_223*ORcp41_124-OMcp41_224*ORcp41_125-OMcp41_225*ORcp41_173-OMcp41_26*ORcp41_121+OPcp41_122*RLcp41_223+OPcp41_123*RLcp41_224+...
 OPcp41_124*RLcp41_225+OPcp41_125*RLcp41_273+OPcp41_16*RLcp41_221-OPcp41_222*RLcp41_123-OPcp41_223*RLcp41_124-OPcp41_224*RLcp41_125-OPcp41_225*...
 RLcp41_173-OPcp41_26*RLcp41_121;

% = = Block_1_0_0_42_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp41_173;
    sens.P(2) = POcp41_273;
    sens.P(3) = POcp41_373;
    sens.R(1,1) = ROcp41_124;
    sens.R(1,2) = ROcp41_224;
    sens.R(1,3) = ROcp41_324;
    sens.R(2,1) = ROcp41_425;
    sens.R(2,2) = ROcp41_525;
    sens.R(2,3) = ROcp41_625;
    sens.R(3,1) = ROcp41_725;
    sens.R(3,2) = ROcp41_825;
    sens.R(3,3) = ROcp41_925;
    sens.V(1) = VIcp41_173;
    sens.V(2) = VIcp41_273;
    sens.V(3) = VIcp41_373;
    sens.OM(1) = OMcp41_125;
    sens.OM(2) = OMcp41_225;
    sens.OM(3) = OMcp41_325;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp41_173_5;
    sens.J(1,6) = JTcp41_173_6;
    sens.J(1,21) = JTcp41_173_7;
    sens.J(1,22) = JTcp41_173_8;
    sens.J(1,23) = JTcp41_173_9;
    sens.J(1,24) = JTcp41_173_10;
    sens.J(1,25) = JTcp41_173_11;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp41_273_4;
    sens.J(2,5) = JTcp41_273_5;
    sens.J(2,6) = JTcp41_273_6;
    sens.J(2,21) = JTcp41_273_7;
    sens.J(2,22) = JTcp41_273_8;
    sens.J(2,23) = JTcp41_273_9;
    sens.J(2,24) = JTcp41_273_10;
    sens.J(2,25) = JTcp41_273_11;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp41_373_4;
    sens.J(3,5) = JTcp41_373_5;
    sens.J(3,6) = JTcp41_373_6;
    sens.J(3,21) = JTcp41_373_7;
    sens.J(3,22) = JTcp41_373_8;
    sens.J(3,23) = JTcp41_373_9;
    sens.J(3,24) = JTcp41_373_10;
    sens.J(3,25) = JTcp41_373_11;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,21) = ROcp41_16;
    sens.J(4,22) = ROcp41_421;
    sens.J(4,23) = ROcp41_722;
    sens.J(4,24) = ROcp41_423;
    sens.J(4,25) = ROcp41_124;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp41_85;
    sens.J(5,21) = ROcp41_26;
    sens.J(5,22) = ROcp41_521;
    sens.J(5,23) = ROcp41_822;
    sens.J(5,24) = ROcp41_523;
    sens.J(5,25) = ROcp41_224;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp41_95;
    sens.J(6,21) = ROcp41_36;
    sens.J(6,22) = ROcp41_621;
    sens.J(6,23) = ROcp41_922;
    sens.J(6,24) = ROcp41_623;
    sens.J(6,25) = ROcp41_324;
    sens.A(1) = ACcp41_173;
    sens.A(2) = ACcp41_273;
    sens.A(3) = ACcp41_373;
    sens.OMP(1) = OPcp41_125;
    sens.OMP(2) = OPcp41_225;
    sens.OMP(3) = OPcp41_325;
 
% 
case 43, 


% = = Block_1_0_0_43_0_1 = = 
 
% Sensor Kinematics 


    ROcp42_25 = S4*S5;
    ROcp42_35 = -C4*S5;
    ROcp42_85 = -S4*C5;
    ROcp42_95 = C4*C5;
    ROcp42_16 = C5*C6;
    ROcp42_26 = ROcp42_25*C6+C4*S6;
    ROcp42_36 = ROcp42_35*C6+S4*S6;
    ROcp42_46 = -C5*S6;
    ROcp42_56 = -(ROcp42_25*S6-C4*C6);
    ROcp42_66 = -(ROcp42_35*S6-S4*C6);
    OMcp42_25 = qd(5)*C4;
    OMcp42_35 = qd(5)*S4;
    OMcp42_16 = qd(4)+qd(6)*S5;
    OMcp42_26 = OMcp42_25+ROcp42_85*qd(6);
    OMcp42_36 = OMcp42_35+ROcp42_95*qd(6);
    OPcp42_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp42_26 = ROcp42_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp42_35*S5-ROcp42_95*qd(4));
    OPcp42_36 = ROcp42_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp42_25*S5-ROcp42_85*qd(4));

% = = Block_1_0_0_43_0_4 = = 
 
% Sensor Kinematics 


    ROcp42_421 = ROcp42_46*C21+S21*S5;
    ROcp42_521 = ROcp42_56*C21+ROcp42_85*S21;
    ROcp42_621 = ROcp42_66*C21+ROcp42_95*S21;
    ROcp42_721 = -(ROcp42_46*S21-C21*S5);
    ROcp42_821 = -(ROcp42_56*S21-ROcp42_85*C21);
    ROcp42_921 = -(ROcp42_66*S21-ROcp42_95*C21);
    ROcp42_122 = ROcp42_16*C22-ROcp42_721*S22;
    ROcp42_222 = ROcp42_26*C22-ROcp42_821*S22;
    ROcp42_322 = ROcp42_36*C22-ROcp42_921*S22;
    ROcp42_722 = ROcp42_16*S22+ROcp42_721*C22;
    ROcp42_822 = ROcp42_26*S22+ROcp42_821*C22;
    ROcp42_922 = ROcp42_36*S22+ROcp42_921*C22;
    ROcp42_123 = ROcp42_122*C23+ROcp42_421*S23;
    ROcp42_223 = ROcp42_222*C23+ROcp42_521*S23;
    ROcp42_323 = ROcp42_322*C23+ROcp42_621*S23;
    ROcp42_423 = -(ROcp42_122*S23-ROcp42_421*C23);
    ROcp42_523 = -(ROcp42_222*S23-ROcp42_521*C23);
    ROcp42_623 = -(ROcp42_322*S23-ROcp42_621*C23);
    RLcp42_121 = ROcp42_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp42_221 = ROcp42_26*s.dpt(1,3)+ROcp42_85*s.dpt(3,3);
    RLcp42_321 = ROcp42_36*s.dpt(1,3)+ROcp42_95*s.dpt(3,3);
    OMcp42_121 = OMcp42_16+ROcp42_16*qd(21);
    OMcp42_221 = OMcp42_26+ROcp42_26*qd(21);
    OMcp42_321 = OMcp42_36+ROcp42_36*qd(21);
    ORcp42_121 = OMcp42_26*RLcp42_321-OMcp42_36*RLcp42_221;
    ORcp42_221 = -(OMcp42_16*RLcp42_321-OMcp42_36*RLcp42_121);
    ORcp42_321 = OMcp42_16*RLcp42_221-OMcp42_26*RLcp42_121;
    OMcp42_122 = OMcp42_121+ROcp42_421*qd(22);
    OMcp42_222 = OMcp42_221+ROcp42_521*qd(22);
    OMcp42_322 = OMcp42_321+ROcp42_621*qd(22);
    OPcp42_122 = OPcp42_16+ROcp42_16*qdd(21)+ROcp42_421*qdd(22)+qd(21)*(OMcp42_26*ROcp42_36-OMcp42_36*ROcp42_26)+qd(22)*(OMcp42_221*ROcp42_621-...
 OMcp42_321*ROcp42_521);
    OPcp42_222 = OPcp42_26+ROcp42_26*qdd(21)+ROcp42_521*qdd(22)-qd(21)*(OMcp42_16*ROcp42_36-OMcp42_36*ROcp42_16)-qd(22)*(OMcp42_121*ROcp42_621-...
 OMcp42_321*ROcp42_421);
    OPcp42_322 = OPcp42_36+ROcp42_36*qdd(21)+ROcp42_621*qdd(22)+qd(21)*(OMcp42_16*ROcp42_26-OMcp42_26*ROcp42_16)+qd(22)*(OMcp42_121*ROcp42_521-...
 OMcp42_221*ROcp42_421);
    RLcp42_123 = ROcp42_722*s.dpt(3,37);
    RLcp42_223 = ROcp42_822*s.dpt(3,37);
    RLcp42_323 = ROcp42_922*s.dpt(3,37);
    OMcp42_123 = OMcp42_122+ROcp42_722*qd(23);
    OMcp42_223 = OMcp42_222+ROcp42_822*qd(23);
    OMcp42_323 = OMcp42_322+ROcp42_922*qd(23);
    ORcp42_123 = OMcp42_222*RLcp42_323-OMcp42_322*RLcp42_223;
    ORcp42_223 = -(OMcp42_122*RLcp42_323-OMcp42_322*RLcp42_123);
    ORcp42_323 = OMcp42_122*RLcp42_223-OMcp42_222*RLcp42_123;
    OPcp42_123 = OPcp42_122+ROcp42_722*qdd(23)+qd(23)*(OMcp42_222*ROcp42_922-OMcp42_322*ROcp42_822);
    OPcp42_223 = OPcp42_222+ROcp42_822*qdd(23)-qd(23)*(OMcp42_122*ROcp42_922-OMcp42_322*ROcp42_722);
    OPcp42_323 = OPcp42_322+ROcp42_922*qdd(23)+qd(23)*(OMcp42_122*ROcp42_822-OMcp42_222*ROcp42_722);

% = = Block_1_0_0_43_0_5 = = 
 
% Sensor Kinematics 


    ROcp42_124 = ROcp42_123*C24-ROcp42_722*S24;
    ROcp42_224 = ROcp42_223*C24-ROcp42_822*S24;
    ROcp42_324 = ROcp42_323*C24-ROcp42_922*S24;
    ROcp42_724 = ROcp42_123*S24+ROcp42_722*C24;
    ROcp42_824 = ROcp42_223*S24+ROcp42_822*C24;
    ROcp42_924 = ROcp42_323*S24+ROcp42_922*C24;
    ROcp42_425 = ROcp42_423*C25+ROcp42_724*S25;
    ROcp42_525 = ROcp42_523*C25+ROcp42_824*S25;
    ROcp42_625 = ROcp42_623*C25+ROcp42_924*S25;
    ROcp42_725 = -(ROcp42_423*S25-ROcp42_724*C25);
    ROcp42_825 = -(ROcp42_523*S25-ROcp42_824*C25);
    ROcp42_925 = -(ROcp42_623*S25-ROcp42_924*C25);
    ROcp42_126 = ROcp42_124*C26+ROcp42_425*S26;
    ROcp42_226 = ROcp42_224*C26+ROcp42_525*S26;
    ROcp42_326 = ROcp42_324*C26+ROcp42_625*S26;
    ROcp42_426 = -(ROcp42_124*S26-ROcp42_425*C26);
    ROcp42_526 = -(ROcp42_224*S26-ROcp42_525*C26);
    ROcp42_626 = -(ROcp42_324*S26-ROcp42_625*C26);
    RLcp42_124 = ROcp42_123*s.dpt(1,41)+ROcp42_423*s.dpt(2,41)+ROcp42_722*s.dpt(3,41);
    RLcp42_224 = ROcp42_223*s.dpt(1,41)+ROcp42_523*s.dpt(2,41)+ROcp42_822*s.dpt(3,41);
    RLcp42_324 = ROcp42_323*s.dpt(1,41)+ROcp42_623*s.dpt(2,41)+ROcp42_922*s.dpt(3,41);
    OMcp42_124 = OMcp42_123+ROcp42_423*qd(24);
    OMcp42_224 = OMcp42_223+ROcp42_523*qd(24);
    OMcp42_324 = OMcp42_323+ROcp42_623*qd(24);
    ORcp42_124 = OMcp42_223*RLcp42_324-OMcp42_323*RLcp42_224;
    ORcp42_224 = -(OMcp42_123*RLcp42_324-OMcp42_323*RLcp42_124);
    ORcp42_324 = OMcp42_123*RLcp42_224-OMcp42_223*RLcp42_124;
    OPcp42_124 = OPcp42_123+ROcp42_423*qdd(24)+qd(24)*(OMcp42_223*ROcp42_623-OMcp42_323*ROcp42_523);
    OPcp42_224 = OPcp42_223+ROcp42_523*qdd(24)-qd(24)*(OMcp42_123*ROcp42_623-OMcp42_323*ROcp42_423);
    OPcp42_324 = OPcp42_323+ROcp42_623*qdd(24)+qd(24)*(OMcp42_123*ROcp42_523-OMcp42_223*ROcp42_423);
    RLcp42_125 = ROcp42_423*s.dpt(2,44);
    RLcp42_225 = ROcp42_523*s.dpt(2,44);
    RLcp42_325 = ROcp42_623*s.dpt(2,44);
    OMcp42_125 = OMcp42_124+ROcp42_124*qd(25);
    OMcp42_225 = OMcp42_224+ROcp42_224*qd(25);
    OMcp42_325 = OMcp42_324+ROcp42_324*qd(25);
    ORcp42_125 = OMcp42_224*RLcp42_325-OMcp42_324*RLcp42_225;
    ORcp42_225 = -(OMcp42_124*RLcp42_325-OMcp42_324*RLcp42_125);
    ORcp42_325 = OMcp42_124*RLcp42_225-OMcp42_224*RLcp42_125;
    OPcp42_125 = OPcp42_124+ROcp42_124*qdd(25)+qd(25)*(OMcp42_224*ROcp42_324-OMcp42_324*ROcp42_224);
    OPcp42_225 = OPcp42_224+ROcp42_224*qdd(25)-qd(25)*(OMcp42_124*ROcp42_324-OMcp42_324*ROcp42_124);
    OPcp42_325 = OPcp42_324+ROcp42_324*qdd(25)+qd(25)*(OMcp42_124*ROcp42_224-OMcp42_224*ROcp42_124);
    RLcp42_126 = ROcp42_725*s.dpt(3,46);
    RLcp42_226 = ROcp42_825*s.dpt(3,46);
    RLcp42_326 = ROcp42_925*s.dpt(3,46);
    OMcp42_126 = OMcp42_125+ROcp42_725*qd(26);
    OMcp42_226 = OMcp42_225+ROcp42_825*qd(26);
    OMcp42_326 = OMcp42_325+ROcp42_925*qd(26);
    ORcp42_126 = OMcp42_225*RLcp42_326-OMcp42_325*RLcp42_226;
    ORcp42_226 = -(OMcp42_125*RLcp42_326-OMcp42_325*RLcp42_126);
    ORcp42_326 = OMcp42_125*RLcp42_226-OMcp42_225*RLcp42_126;
    OPcp42_126 = OPcp42_125+ROcp42_725*qdd(26)+qd(26)*(OMcp42_225*ROcp42_925-OMcp42_325*ROcp42_825);
    OPcp42_226 = OPcp42_225+ROcp42_825*qdd(26)-qd(26)*(OMcp42_125*ROcp42_925-OMcp42_325*ROcp42_725);
    OPcp42_326 = OPcp42_325+ROcp42_925*qdd(26)+qd(26)*(OMcp42_125*ROcp42_825-OMcp42_225*ROcp42_725);
    RLcp42_174 = ROcp42_126*s.dpt(1,47)+ROcp42_426*s.dpt(2,47)+ROcp42_725*s.dpt(3,47);
    RLcp42_274 = ROcp42_226*s.dpt(1,47)+ROcp42_526*s.dpt(2,47)+ROcp42_825*s.dpt(3,47);
    RLcp42_374 = ROcp42_326*s.dpt(1,47)+ROcp42_626*s.dpt(2,47)+ROcp42_925*s.dpt(3,47);
    POcp42_174 = RLcp42_121+RLcp42_123+RLcp42_124+RLcp42_125+RLcp42_126+RLcp42_174+q(1);
    POcp42_274 = RLcp42_221+RLcp42_223+RLcp42_224+RLcp42_225+RLcp42_226+RLcp42_274+q(2);
    POcp42_374 = RLcp42_321+RLcp42_323+RLcp42_324+RLcp42_325+RLcp42_326+RLcp42_374+q(3);
    JTcp42_274_4 = -(RLcp42_321+RLcp42_323+RLcp42_324+RLcp42_325+RLcp42_326+RLcp42_374);
    JTcp42_374_4 = RLcp42_221+RLcp42_223+RLcp42_224+RLcp42_225+RLcp42_226+RLcp42_274;
    JTcp42_174_5 = C4*(RLcp42_321+RLcp42_323+RLcp42_324+RLcp42_325+RLcp42_326+RLcp42_374)-S4*(RLcp42_221+RLcp42_223)-S4*(RLcp42_224+RLcp42_225)-S4*...
 (RLcp42_226+RLcp42_274);
    JTcp42_274_5 = S4*(RLcp42_121+RLcp42_123+RLcp42_124+RLcp42_125+RLcp42_126+RLcp42_174);
    JTcp42_374_5 = -C4*(RLcp42_121+RLcp42_123+RLcp42_124+RLcp42_125+RLcp42_126+RLcp42_174);
    JTcp42_174_6 = ROcp42_85*(RLcp42_321+RLcp42_323+RLcp42_324+RLcp42_325+RLcp42_326+RLcp42_374)-ROcp42_95*(RLcp42_221+RLcp42_223)-ROcp42_95*(...
 RLcp42_224+RLcp42_225)-ROcp42_95*(RLcp42_226+RLcp42_274);
    JTcp42_274_6 = RLcp42_174*ROcp42_95-RLcp42_326*S5-RLcp42_374*S5+ROcp42_95*(RLcp42_121+RLcp42_123+RLcp42_124+RLcp42_125+RLcp42_126)-S5*(...
 RLcp42_321+RLcp42_323)-S5*(RLcp42_324+RLcp42_325);
    JTcp42_374_6 = RLcp42_226*S5-ROcp42_85*(RLcp42_121+RLcp42_123+RLcp42_124+RLcp42_125+RLcp42_126)+S5*(RLcp42_221+RLcp42_223)+S5*(RLcp42_224+...
 RLcp42_225)-RLcp42_174*ROcp42_85+RLcp42_274*S5;
    JTcp42_174_7 = ROcp42_26*(RLcp42_323+RLcp42_324+RLcp42_325+RLcp42_326)-ROcp42_36*(RLcp42_223+RLcp42_224)-ROcp42_36*(RLcp42_225+RLcp42_226)-...
 RLcp42_274*ROcp42_36+RLcp42_374*ROcp42_26;
    JTcp42_274_7 = RLcp42_174*ROcp42_36-RLcp42_374*ROcp42_16-ROcp42_16*(RLcp42_323+RLcp42_324+RLcp42_325+RLcp42_326)+ROcp42_36*(RLcp42_123+...
 RLcp42_124)+ROcp42_36*(RLcp42_125+RLcp42_126);
    JTcp42_374_7 = ROcp42_16*(RLcp42_223+RLcp42_224+RLcp42_225+RLcp42_226)-ROcp42_26*(RLcp42_123+RLcp42_124)-ROcp42_26*(RLcp42_125+RLcp42_126)-...
 RLcp42_174*ROcp42_26+RLcp42_274*ROcp42_16;
    JTcp42_174_8 = ROcp42_521*(RLcp42_323+RLcp42_324+RLcp42_325+RLcp42_326)-ROcp42_621*(RLcp42_223+RLcp42_224)-ROcp42_621*(RLcp42_225+RLcp42_226)-...
 RLcp42_274*ROcp42_621+RLcp42_374*ROcp42_521;
    JTcp42_274_8 = RLcp42_174*ROcp42_621-RLcp42_374*ROcp42_421-ROcp42_421*(RLcp42_323+RLcp42_324+RLcp42_325+RLcp42_326)+ROcp42_621*(RLcp42_123+...
 RLcp42_124)+ROcp42_621*(RLcp42_125+RLcp42_126);
    JTcp42_374_8 = ROcp42_421*(RLcp42_223+RLcp42_224+RLcp42_225+RLcp42_226)-ROcp42_521*(RLcp42_123+RLcp42_124)-ROcp42_521*(RLcp42_125+RLcp42_126)-...
 RLcp42_174*ROcp42_521+RLcp42_274*ROcp42_421;
    JTcp42_174_9 = ROcp42_822*(RLcp42_324+RLcp42_325+RLcp42_326+RLcp42_374)-ROcp42_922*(RLcp42_224+RLcp42_225)-ROcp42_922*(RLcp42_226+RLcp42_274);
    JTcp42_274_9 = -(ROcp42_722*(RLcp42_324+RLcp42_325+RLcp42_326+RLcp42_374)-ROcp42_922*(RLcp42_124+RLcp42_125)-ROcp42_922*(RLcp42_126+RLcp42_174)...
 );
    JTcp42_374_9 = ROcp42_722*(RLcp42_224+RLcp42_225+RLcp42_226+RLcp42_274)-ROcp42_822*(RLcp42_124+RLcp42_125)-ROcp42_822*(RLcp42_126+RLcp42_174);
    JTcp42_174_10 = ROcp42_523*(RLcp42_325+RLcp42_326)-ROcp42_623*(RLcp42_225+RLcp42_226)-RLcp42_274*ROcp42_623+RLcp42_374*ROcp42_523;
    JTcp42_274_10 = RLcp42_174*ROcp42_623-RLcp42_374*ROcp42_423-ROcp42_423*(RLcp42_325+RLcp42_326)+ROcp42_623*(RLcp42_125+RLcp42_126);
    JTcp42_374_10 = ROcp42_423*(RLcp42_225+RLcp42_226)-ROcp42_523*(RLcp42_125+RLcp42_126)-RLcp42_174*ROcp42_523+RLcp42_274*ROcp42_423;
    JTcp42_174_11 = ROcp42_224*(RLcp42_326+RLcp42_374)-ROcp42_324*(RLcp42_226+RLcp42_274);
    JTcp42_274_11 = -(ROcp42_124*(RLcp42_326+RLcp42_374)-ROcp42_324*(RLcp42_126+RLcp42_174));
    JTcp42_374_11 = ROcp42_124*(RLcp42_226+RLcp42_274)-ROcp42_224*(RLcp42_126+RLcp42_174);
    JTcp42_174_12 = -(RLcp42_274*ROcp42_925-RLcp42_374*ROcp42_825);
    JTcp42_274_12 = RLcp42_174*ROcp42_925-RLcp42_374*ROcp42_725;
    JTcp42_374_12 = -(RLcp42_174*ROcp42_825-RLcp42_274*ROcp42_725);
    ORcp42_174 = OMcp42_226*RLcp42_374-OMcp42_326*RLcp42_274;
    ORcp42_274 = -(OMcp42_126*RLcp42_374-OMcp42_326*RLcp42_174);
    ORcp42_374 = OMcp42_126*RLcp42_274-OMcp42_226*RLcp42_174;
    VIcp42_174 = ORcp42_121+ORcp42_123+ORcp42_124+ORcp42_125+ORcp42_126+ORcp42_174+qd(1);
    VIcp42_274 = ORcp42_221+ORcp42_223+ORcp42_224+ORcp42_225+ORcp42_226+ORcp42_274+qd(2);
    VIcp42_374 = ORcp42_321+ORcp42_323+ORcp42_324+ORcp42_325+ORcp42_326+ORcp42_374+qd(3);
    ACcp42_174 = qdd(1)+OMcp42_222*ORcp42_323+OMcp42_223*ORcp42_324+OMcp42_224*ORcp42_325+OMcp42_225*ORcp42_326+OMcp42_226*ORcp42_374+OMcp42_26*...
 ORcp42_321-OMcp42_322*ORcp42_223-OMcp42_323*ORcp42_224-OMcp42_324*ORcp42_225-OMcp42_325*ORcp42_226-OMcp42_326*ORcp42_274-OMcp42_36*ORcp42_221+...
 OPcp42_222*RLcp42_323+OPcp42_223*RLcp42_324+OPcp42_224*RLcp42_325+OPcp42_225*RLcp42_326+OPcp42_226*RLcp42_374+OPcp42_26*RLcp42_321-OPcp42_322*...
 RLcp42_223-OPcp42_323*RLcp42_224-OPcp42_324*RLcp42_225-OPcp42_325*RLcp42_226-OPcp42_326*RLcp42_274-OPcp42_36*RLcp42_221;
    ACcp42_274 = qdd(2)-OMcp42_122*ORcp42_323-OMcp42_123*ORcp42_324-OMcp42_124*ORcp42_325-OMcp42_125*ORcp42_326-OMcp42_126*ORcp42_374-OMcp42_16*...
 ORcp42_321+OMcp42_322*ORcp42_123+OMcp42_323*ORcp42_124+OMcp42_324*ORcp42_125+OMcp42_325*ORcp42_126+OMcp42_326*ORcp42_174+OMcp42_36*ORcp42_121-...
 OPcp42_122*RLcp42_323-OPcp42_123*RLcp42_324-OPcp42_124*RLcp42_325-OPcp42_125*RLcp42_326-OPcp42_126*RLcp42_374-OPcp42_16*RLcp42_321+OPcp42_322*...
 RLcp42_123+OPcp42_323*RLcp42_124+OPcp42_324*RLcp42_125+OPcp42_325*RLcp42_126+OPcp42_326*RLcp42_174+OPcp42_36*RLcp42_121;
    ACcp42_374 = qdd(3)+OMcp42_122*ORcp42_223+OMcp42_123*ORcp42_224+OMcp42_124*ORcp42_225+OMcp42_125*ORcp42_226+OMcp42_126*ORcp42_274+OMcp42_16*...
 ORcp42_221-OMcp42_222*ORcp42_123-OMcp42_223*ORcp42_124-OMcp42_224*ORcp42_125-OMcp42_225*ORcp42_126-OMcp42_226*ORcp42_174-OMcp42_26*ORcp42_121+...
 OPcp42_122*RLcp42_223+OPcp42_123*RLcp42_224+OPcp42_124*RLcp42_225+OPcp42_125*RLcp42_226+OPcp42_126*RLcp42_274+OPcp42_16*RLcp42_221-OPcp42_222*...
 RLcp42_123-OPcp42_223*RLcp42_124-OPcp42_224*RLcp42_125-OPcp42_225*RLcp42_126-OPcp42_226*RLcp42_174-OPcp42_26*RLcp42_121;

% = = Block_1_0_0_43_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp42_174;
    sens.P(2) = POcp42_274;
    sens.P(3) = POcp42_374;
    sens.R(1,1) = ROcp42_126;
    sens.R(1,2) = ROcp42_226;
    sens.R(1,3) = ROcp42_326;
    sens.R(2,1) = ROcp42_426;
    sens.R(2,2) = ROcp42_526;
    sens.R(2,3) = ROcp42_626;
    sens.R(3,1) = ROcp42_725;
    sens.R(3,2) = ROcp42_825;
    sens.R(3,3) = ROcp42_925;
    sens.V(1) = VIcp42_174;
    sens.V(2) = VIcp42_274;
    sens.V(3) = VIcp42_374;
    sens.OM(1) = OMcp42_126;
    sens.OM(2) = OMcp42_226;
    sens.OM(3) = OMcp42_326;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp42_174_5;
    sens.J(1,6) = JTcp42_174_6;
    sens.J(1,21) = JTcp42_174_7;
    sens.J(1,22) = JTcp42_174_8;
    sens.J(1,23) = JTcp42_174_9;
    sens.J(1,24) = JTcp42_174_10;
    sens.J(1,25) = JTcp42_174_11;
    sens.J(1,26) = JTcp42_174_12;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp42_274_4;
    sens.J(2,5) = JTcp42_274_5;
    sens.J(2,6) = JTcp42_274_6;
    sens.J(2,21) = JTcp42_274_7;
    sens.J(2,22) = JTcp42_274_8;
    sens.J(2,23) = JTcp42_274_9;
    sens.J(2,24) = JTcp42_274_10;
    sens.J(2,25) = JTcp42_274_11;
    sens.J(2,26) = JTcp42_274_12;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp42_374_4;
    sens.J(3,5) = JTcp42_374_5;
    sens.J(3,6) = JTcp42_374_6;
    sens.J(3,21) = JTcp42_374_7;
    sens.J(3,22) = JTcp42_374_8;
    sens.J(3,23) = JTcp42_374_9;
    sens.J(3,24) = JTcp42_374_10;
    sens.J(3,25) = JTcp42_374_11;
    sens.J(3,26) = JTcp42_374_12;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,21) = ROcp42_16;
    sens.J(4,22) = ROcp42_421;
    sens.J(4,23) = ROcp42_722;
    sens.J(4,24) = ROcp42_423;
    sens.J(4,25) = ROcp42_124;
    sens.J(4,26) = ROcp42_725;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp42_85;
    sens.J(5,21) = ROcp42_26;
    sens.J(5,22) = ROcp42_521;
    sens.J(5,23) = ROcp42_822;
    sens.J(5,24) = ROcp42_523;
    sens.J(5,25) = ROcp42_224;
    sens.J(5,26) = ROcp42_825;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp42_95;
    sens.J(6,21) = ROcp42_36;
    sens.J(6,22) = ROcp42_621;
    sens.J(6,23) = ROcp42_922;
    sens.J(6,24) = ROcp42_623;
    sens.J(6,25) = ROcp42_324;
    sens.J(6,26) = ROcp42_925;
    sens.A(1) = ACcp42_174;
    sens.A(2) = ACcp42_274;
    sens.A(3) = ACcp42_374;
    sens.OMP(1) = OPcp42_126;
    sens.OMP(2) = OPcp42_226;
    sens.OMP(3) = OPcp42_326;
 
% 
case 44, 


% = = Block_1_0_0_44_0_1 = = 
 
% Sensor Kinematics 


    ROcp43_25 = S4*S5;
    ROcp43_35 = -C4*S5;
    ROcp43_85 = -S4*C5;
    ROcp43_95 = C4*C5;
    ROcp43_16 = C5*C6;
    ROcp43_26 = ROcp43_25*C6+C4*S6;
    ROcp43_36 = ROcp43_35*C6+S4*S6;
    ROcp43_46 = -C5*S6;
    ROcp43_56 = -(ROcp43_25*S6-C4*C6);
    ROcp43_66 = -(ROcp43_35*S6-S4*C6);
    OMcp43_25 = qd(5)*C4;
    OMcp43_35 = qd(5)*S4;
    OMcp43_16 = qd(4)+qd(6)*S5;
    OMcp43_26 = OMcp43_25+ROcp43_85*qd(6);
    OMcp43_36 = OMcp43_35+ROcp43_95*qd(6);
    OPcp43_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp43_26 = ROcp43_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp43_35*S5-ROcp43_95*qd(4));
    OPcp43_36 = ROcp43_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp43_25*S5-ROcp43_85*qd(4));

% = = Block_1_0_0_44_0_4 = = 
 
% Sensor Kinematics 


    ROcp43_421 = ROcp43_46*C21+S21*S5;
    ROcp43_521 = ROcp43_56*C21+ROcp43_85*S21;
    ROcp43_621 = ROcp43_66*C21+ROcp43_95*S21;
    ROcp43_721 = -(ROcp43_46*S21-C21*S5);
    ROcp43_821 = -(ROcp43_56*S21-ROcp43_85*C21);
    ROcp43_921 = -(ROcp43_66*S21-ROcp43_95*C21);
    ROcp43_122 = ROcp43_16*C22-ROcp43_721*S22;
    ROcp43_222 = ROcp43_26*C22-ROcp43_821*S22;
    ROcp43_322 = ROcp43_36*C22-ROcp43_921*S22;
    ROcp43_722 = ROcp43_16*S22+ROcp43_721*C22;
    ROcp43_822 = ROcp43_26*S22+ROcp43_821*C22;
    ROcp43_922 = ROcp43_36*S22+ROcp43_921*C22;
    ROcp43_123 = ROcp43_122*C23+ROcp43_421*S23;
    ROcp43_223 = ROcp43_222*C23+ROcp43_521*S23;
    ROcp43_323 = ROcp43_322*C23+ROcp43_621*S23;
    ROcp43_423 = -(ROcp43_122*S23-ROcp43_421*C23);
    ROcp43_523 = -(ROcp43_222*S23-ROcp43_521*C23);
    ROcp43_623 = -(ROcp43_322*S23-ROcp43_621*C23);
    RLcp43_121 = ROcp43_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp43_221 = ROcp43_26*s.dpt(1,3)+ROcp43_85*s.dpt(3,3);
    RLcp43_321 = ROcp43_36*s.dpt(1,3)+ROcp43_95*s.dpt(3,3);
    OMcp43_121 = OMcp43_16+ROcp43_16*qd(21);
    OMcp43_221 = OMcp43_26+ROcp43_26*qd(21);
    OMcp43_321 = OMcp43_36+ROcp43_36*qd(21);
    ORcp43_121 = OMcp43_26*RLcp43_321-OMcp43_36*RLcp43_221;
    ORcp43_221 = -(OMcp43_16*RLcp43_321-OMcp43_36*RLcp43_121);
    ORcp43_321 = OMcp43_16*RLcp43_221-OMcp43_26*RLcp43_121;
    OMcp43_122 = OMcp43_121+ROcp43_421*qd(22);
    OMcp43_222 = OMcp43_221+ROcp43_521*qd(22);
    OMcp43_322 = OMcp43_321+ROcp43_621*qd(22);
    OPcp43_122 = OPcp43_16+ROcp43_16*qdd(21)+ROcp43_421*qdd(22)+qd(21)*(OMcp43_26*ROcp43_36-OMcp43_36*ROcp43_26)+qd(22)*(OMcp43_221*ROcp43_621-...
 OMcp43_321*ROcp43_521);
    OPcp43_222 = OPcp43_26+ROcp43_26*qdd(21)+ROcp43_521*qdd(22)-qd(21)*(OMcp43_16*ROcp43_36-OMcp43_36*ROcp43_16)-qd(22)*(OMcp43_121*ROcp43_621-...
 OMcp43_321*ROcp43_421);
    OPcp43_322 = OPcp43_36+ROcp43_36*qdd(21)+ROcp43_621*qdd(22)+qd(21)*(OMcp43_16*ROcp43_26-OMcp43_26*ROcp43_16)+qd(22)*(OMcp43_121*ROcp43_521-...
 OMcp43_221*ROcp43_421);
    RLcp43_123 = ROcp43_722*s.dpt(3,37);
    RLcp43_223 = ROcp43_822*s.dpt(3,37);
    RLcp43_323 = ROcp43_922*s.dpt(3,37);
    OMcp43_123 = OMcp43_122+ROcp43_722*qd(23);
    OMcp43_223 = OMcp43_222+ROcp43_822*qd(23);
    OMcp43_323 = OMcp43_322+ROcp43_922*qd(23);
    ORcp43_123 = OMcp43_222*RLcp43_323-OMcp43_322*RLcp43_223;
    ORcp43_223 = -(OMcp43_122*RLcp43_323-OMcp43_322*RLcp43_123);
    ORcp43_323 = OMcp43_122*RLcp43_223-OMcp43_222*RLcp43_123;
    OPcp43_123 = OPcp43_122+ROcp43_722*qdd(23)+qd(23)*(OMcp43_222*ROcp43_922-OMcp43_322*ROcp43_822);
    OPcp43_223 = OPcp43_222+ROcp43_822*qdd(23)-qd(23)*(OMcp43_122*ROcp43_922-OMcp43_322*ROcp43_722);
    OPcp43_323 = OPcp43_322+ROcp43_922*qdd(23)+qd(23)*(OMcp43_122*ROcp43_822-OMcp43_222*ROcp43_722);

% = = Block_1_0_0_44_0_5 = = 
 
% Sensor Kinematics 


    ROcp43_124 = ROcp43_123*C24-ROcp43_722*S24;
    ROcp43_224 = ROcp43_223*C24-ROcp43_822*S24;
    ROcp43_324 = ROcp43_323*C24-ROcp43_922*S24;
    ROcp43_724 = ROcp43_123*S24+ROcp43_722*C24;
    ROcp43_824 = ROcp43_223*S24+ROcp43_822*C24;
    ROcp43_924 = ROcp43_323*S24+ROcp43_922*C24;
    ROcp43_425 = ROcp43_423*C25+ROcp43_724*S25;
    ROcp43_525 = ROcp43_523*C25+ROcp43_824*S25;
    ROcp43_625 = ROcp43_623*C25+ROcp43_924*S25;
    ROcp43_725 = -(ROcp43_423*S25-ROcp43_724*C25);
    ROcp43_825 = -(ROcp43_523*S25-ROcp43_824*C25);
    ROcp43_925 = -(ROcp43_623*S25-ROcp43_924*C25);
    ROcp43_126 = ROcp43_124*C26+ROcp43_425*S26;
    ROcp43_226 = ROcp43_224*C26+ROcp43_525*S26;
    ROcp43_326 = ROcp43_324*C26+ROcp43_625*S26;
    ROcp43_426 = -(ROcp43_124*S26-ROcp43_425*C26);
    ROcp43_526 = -(ROcp43_224*S26-ROcp43_525*C26);
    ROcp43_626 = -(ROcp43_324*S26-ROcp43_625*C26);
    RLcp43_124 = ROcp43_123*s.dpt(1,41)+ROcp43_423*s.dpt(2,41)+ROcp43_722*s.dpt(3,41);
    RLcp43_224 = ROcp43_223*s.dpt(1,41)+ROcp43_523*s.dpt(2,41)+ROcp43_822*s.dpt(3,41);
    RLcp43_324 = ROcp43_323*s.dpt(1,41)+ROcp43_623*s.dpt(2,41)+ROcp43_922*s.dpt(3,41);
    OMcp43_124 = OMcp43_123+ROcp43_423*qd(24);
    OMcp43_224 = OMcp43_223+ROcp43_523*qd(24);
    OMcp43_324 = OMcp43_323+ROcp43_623*qd(24);
    ORcp43_124 = OMcp43_223*RLcp43_324-OMcp43_323*RLcp43_224;
    ORcp43_224 = -(OMcp43_123*RLcp43_324-OMcp43_323*RLcp43_124);
    ORcp43_324 = OMcp43_123*RLcp43_224-OMcp43_223*RLcp43_124;
    OPcp43_124 = OPcp43_123+ROcp43_423*qdd(24)+qd(24)*(OMcp43_223*ROcp43_623-OMcp43_323*ROcp43_523);
    OPcp43_224 = OPcp43_223+ROcp43_523*qdd(24)-qd(24)*(OMcp43_123*ROcp43_623-OMcp43_323*ROcp43_423);
    OPcp43_324 = OPcp43_323+ROcp43_623*qdd(24)+qd(24)*(OMcp43_123*ROcp43_523-OMcp43_223*ROcp43_423);
    RLcp43_125 = ROcp43_423*s.dpt(2,44);
    RLcp43_225 = ROcp43_523*s.dpt(2,44);
    RLcp43_325 = ROcp43_623*s.dpt(2,44);
    OMcp43_125 = OMcp43_124+ROcp43_124*qd(25);
    OMcp43_225 = OMcp43_224+ROcp43_224*qd(25);
    OMcp43_325 = OMcp43_324+ROcp43_324*qd(25);
    ORcp43_125 = OMcp43_224*RLcp43_325-OMcp43_324*RLcp43_225;
    ORcp43_225 = -(OMcp43_124*RLcp43_325-OMcp43_324*RLcp43_125);
    ORcp43_325 = OMcp43_124*RLcp43_225-OMcp43_224*RLcp43_125;
    OPcp43_125 = OPcp43_124+ROcp43_124*qdd(25)+qd(25)*(OMcp43_224*ROcp43_324-OMcp43_324*ROcp43_224);
    OPcp43_225 = OPcp43_224+ROcp43_224*qdd(25)-qd(25)*(OMcp43_124*ROcp43_324-OMcp43_324*ROcp43_124);
    OPcp43_325 = OPcp43_324+ROcp43_324*qdd(25)+qd(25)*(OMcp43_124*ROcp43_224-OMcp43_224*ROcp43_124);
    RLcp43_126 = ROcp43_725*s.dpt(3,46);
    RLcp43_226 = ROcp43_825*s.dpt(3,46);
    RLcp43_326 = ROcp43_925*s.dpt(3,46);
    OMcp43_126 = OMcp43_125+ROcp43_725*qd(26);
    OMcp43_226 = OMcp43_225+ROcp43_825*qd(26);
    OMcp43_326 = OMcp43_325+ROcp43_925*qd(26);
    ORcp43_126 = OMcp43_225*RLcp43_326-OMcp43_325*RLcp43_226;
    ORcp43_226 = -(OMcp43_125*RLcp43_326-OMcp43_325*RLcp43_126);
    ORcp43_326 = OMcp43_125*RLcp43_226-OMcp43_225*RLcp43_126;
    OPcp43_126 = OPcp43_125+ROcp43_725*qdd(26)+qd(26)*(OMcp43_225*ROcp43_925-OMcp43_325*ROcp43_825);
    OPcp43_226 = OPcp43_225+ROcp43_825*qdd(26)-qd(26)*(OMcp43_125*ROcp43_925-OMcp43_325*ROcp43_725);
    OPcp43_326 = OPcp43_325+ROcp43_925*qdd(26)+qd(26)*(OMcp43_125*ROcp43_825-OMcp43_225*ROcp43_725);
    RLcp43_175 = ROcp43_725*s.dpt(3,48);
    RLcp43_275 = ROcp43_825*s.dpt(3,48);
    RLcp43_375 = ROcp43_925*s.dpt(3,48);
    POcp43_175 = RLcp43_121+RLcp43_123+RLcp43_124+RLcp43_125+RLcp43_126+RLcp43_175+q(1);
    POcp43_275 = RLcp43_221+RLcp43_223+RLcp43_224+RLcp43_225+RLcp43_226+RLcp43_275+q(2);
    POcp43_375 = RLcp43_321+RLcp43_323+RLcp43_324+RLcp43_325+RLcp43_326+RLcp43_375+q(3);
    JTcp43_275_4 = -(RLcp43_321+RLcp43_323+RLcp43_324+RLcp43_325+RLcp43_326+RLcp43_375);
    JTcp43_375_4 = RLcp43_221+RLcp43_223+RLcp43_224+RLcp43_225+RLcp43_226+RLcp43_275;
    JTcp43_175_5 = C4*(RLcp43_321+RLcp43_323+RLcp43_324+RLcp43_325+RLcp43_326+RLcp43_375)-S4*(RLcp43_221+RLcp43_223)-S4*(RLcp43_224+RLcp43_225)-S4*...
 (RLcp43_226+RLcp43_275);
    JTcp43_275_5 = S4*(RLcp43_121+RLcp43_123+RLcp43_124+RLcp43_125+RLcp43_126+RLcp43_175);
    JTcp43_375_5 = -C4*(RLcp43_121+RLcp43_123+RLcp43_124+RLcp43_125+RLcp43_126+RLcp43_175);
    JTcp43_175_6 = ROcp43_85*(RLcp43_321+RLcp43_323+RLcp43_324+RLcp43_325+RLcp43_326+RLcp43_375)-ROcp43_95*(RLcp43_221+RLcp43_223)-ROcp43_95*(...
 RLcp43_224+RLcp43_225)-ROcp43_95*(RLcp43_226+RLcp43_275);
    JTcp43_275_6 = RLcp43_175*ROcp43_95-RLcp43_326*S5-RLcp43_375*S5+ROcp43_95*(RLcp43_121+RLcp43_123+RLcp43_124+RLcp43_125+RLcp43_126)-S5*(...
 RLcp43_321+RLcp43_323)-S5*(RLcp43_324+RLcp43_325);
    JTcp43_375_6 = RLcp43_226*S5-ROcp43_85*(RLcp43_121+RLcp43_123+RLcp43_124+RLcp43_125+RLcp43_126)+S5*(RLcp43_221+RLcp43_223)+S5*(RLcp43_224+...
 RLcp43_225)-RLcp43_175*ROcp43_85+RLcp43_275*S5;
    JTcp43_175_7 = ROcp43_26*(RLcp43_323+RLcp43_324+RLcp43_325+RLcp43_326)-ROcp43_36*(RLcp43_223+RLcp43_224)-ROcp43_36*(RLcp43_225+RLcp43_226)-...
 RLcp43_275*ROcp43_36+RLcp43_375*ROcp43_26;
    JTcp43_275_7 = RLcp43_175*ROcp43_36-RLcp43_375*ROcp43_16-ROcp43_16*(RLcp43_323+RLcp43_324+RLcp43_325+RLcp43_326)+ROcp43_36*(RLcp43_123+...
 RLcp43_124)+ROcp43_36*(RLcp43_125+RLcp43_126);
    JTcp43_375_7 = ROcp43_16*(RLcp43_223+RLcp43_224+RLcp43_225+RLcp43_226)-ROcp43_26*(RLcp43_123+RLcp43_124)-ROcp43_26*(RLcp43_125+RLcp43_126)-...
 RLcp43_175*ROcp43_26+RLcp43_275*ROcp43_16;
    JTcp43_175_8 = ROcp43_521*(RLcp43_323+RLcp43_324+RLcp43_325+RLcp43_326)-ROcp43_621*(RLcp43_223+RLcp43_224)-ROcp43_621*(RLcp43_225+RLcp43_226)-...
 RLcp43_275*ROcp43_621+RLcp43_375*ROcp43_521;
    JTcp43_275_8 = RLcp43_175*ROcp43_621-RLcp43_375*ROcp43_421-ROcp43_421*(RLcp43_323+RLcp43_324+RLcp43_325+RLcp43_326)+ROcp43_621*(RLcp43_123+...
 RLcp43_124)+ROcp43_621*(RLcp43_125+RLcp43_126);
    JTcp43_375_8 = ROcp43_421*(RLcp43_223+RLcp43_224+RLcp43_225+RLcp43_226)-ROcp43_521*(RLcp43_123+RLcp43_124)-ROcp43_521*(RLcp43_125+RLcp43_126)-...
 RLcp43_175*ROcp43_521+RLcp43_275*ROcp43_421;
    JTcp43_175_9 = ROcp43_822*(RLcp43_324+RLcp43_325+RLcp43_326+RLcp43_375)-ROcp43_922*(RLcp43_224+RLcp43_225)-ROcp43_922*(RLcp43_226+RLcp43_275);
    JTcp43_275_9 = -(ROcp43_722*(RLcp43_324+RLcp43_325+RLcp43_326+RLcp43_375)-ROcp43_922*(RLcp43_124+RLcp43_125)-ROcp43_922*(RLcp43_126+RLcp43_175)...
 );
    JTcp43_375_9 = ROcp43_722*(RLcp43_224+RLcp43_225+RLcp43_226+RLcp43_275)-ROcp43_822*(RLcp43_124+RLcp43_125)-ROcp43_822*(RLcp43_126+RLcp43_175);
    JTcp43_175_10 = ROcp43_523*(RLcp43_325+RLcp43_326)-ROcp43_623*(RLcp43_225+RLcp43_226)-RLcp43_275*ROcp43_623+RLcp43_375*ROcp43_523;
    JTcp43_275_10 = RLcp43_175*ROcp43_623-RLcp43_375*ROcp43_423-ROcp43_423*(RLcp43_325+RLcp43_326)+ROcp43_623*(RLcp43_125+RLcp43_126);
    JTcp43_375_10 = ROcp43_423*(RLcp43_225+RLcp43_226)-ROcp43_523*(RLcp43_125+RLcp43_126)-RLcp43_175*ROcp43_523+RLcp43_275*ROcp43_423;
    JTcp43_175_11 = ROcp43_224*(RLcp43_326+RLcp43_375)-ROcp43_324*(RLcp43_226+RLcp43_275);
    JTcp43_275_11 = -(ROcp43_124*(RLcp43_326+RLcp43_375)-ROcp43_324*(RLcp43_126+RLcp43_175));
    JTcp43_375_11 = ROcp43_124*(RLcp43_226+RLcp43_275)-ROcp43_224*(RLcp43_126+RLcp43_175);
    JTcp43_175_12 = -(RLcp43_275*ROcp43_925-RLcp43_375*ROcp43_825);
    JTcp43_275_12 = RLcp43_175*ROcp43_925-RLcp43_375*ROcp43_725;
    JTcp43_375_12 = -(RLcp43_175*ROcp43_825-RLcp43_275*ROcp43_725);
    ORcp43_175 = OMcp43_226*RLcp43_375-OMcp43_326*RLcp43_275;
    ORcp43_275 = -(OMcp43_126*RLcp43_375-OMcp43_326*RLcp43_175);
    ORcp43_375 = OMcp43_126*RLcp43_275-OMcp43_226*RLcp43_175;
    VIcp43_175 = ORcp43_121+ORcp43_123+ORcp43_124+ORcp43_125+ORcp43_126+ORcp43_175+qd(1);
    VIcp43_275 = ORcp43_221+ORcp43_223+ORcp43_224+ORcp43_225+ORcp43_226+ORcp43_275+qd(2);
    VIcp43_375 = ORcp43_321+ORcp43_323+ORcp43_324+ORcp43_325+ORcp43_326+ORcp43_375+qd(3);
    ACcp43_175 = qdd(1)+OMcp43_222*ORcp43_323+OMcp43_223*ORcp43_324+OMcp43_224*ORcp43_325+OMcp43_225*ORcp43_326+OMcp43_226*ORcp43_375+OMcp43_26*...
 ORcp43_321-OMcp43_322*ORcp43_223-OMcp43_323*ORcp43_224-OMcp43_324*ORcp43_225-OMcp43_325*ORcp43_226-OMcp43_326*ORcp43_275-OMcp43_36*ORcp43_221+...
 OPcp43_222*RLcp43_323+OPcp43_223*RLcp43_324+OPcp43_224*RLcp43_325+OPcp43_225*RLcp43_326+OPcp43_226*RLcp43_375+OPcp43_26*RLcp43_321-OPcp43_322*...
 RLcp43_223-OPcp43_323*RLcp43_224-OPcp43_324*RLcp43_225-OPcp43_325*RLcp43_226-OPcp43_326*RLcp43_275-OPcp43_36*RLcp43_221;
    ACcp43_275 = qdd(2)-OMcp43_122*ORcp43_323-OMcp43_123*ORcp43_324-OMcp43_124*ORcp43_325-OMcp43_125*ORcp43_326-OMcp43_126*ORcp43_375-OMcp43_16*...
 ORcp43_321+OMcp43_322*ORcp43_123+OMcp43_323*ORcp43_124+OMcp43_324*ORcp43_125+OMcp43_325*ORcp43_126+OMcp43_326*ORcp43_175+OMcp43_36*ORcp43_121-...
 OPcp43_122*RLcp43_323-OPcp43_123*RLcp43_324-OPcp43_124*RLcp43_325-OPcp43_125*RLcp43_326-OPcp43_126*RLcp43_375-OPcp43_16*RLcp43_321+OPcp43_322*...
 RLcp43_123+OPcp43_323*RLcp43_124+OPcp43_324*RLcp43_125+OPcp43_325*RLcp43_126+OPcp43_326*RLcp43_175+OPcp43_36*RLcp43_121;
    ACcp43_375 = qdd(3)+OMcp43_122*ORcp43_223+OMcp43_123*ORcp43_224+OMcp43_124*ORcp43_225+OMcp43_125*ORcp43_226+OMcp43_126*ORcp43_275+OMcp43_16*...
 ORcp43_221-OMcp43_222*ORcp43_123-OMcp43_223*ORcp43_124-OMcp43_224*ORcp43_125-OMcp43_225*ORcp43_126-OMcp43_226*ORcp43_175-OMcp43_26*ORcp43_121+...
 OPcp43_122*RLcp43_223+OPcp43_123*RLcp43_224+OPcp43_124*RLcp43_225+OPcp43_125*RLcp43_226+OPcp43_126*RLcp43_275+OPcp43_16*RLcp43_221-OPcp43_222*...
 RLcp43_123-OPcp43_223*RLcp43_124-OPcp43_224*RLcp43_125-OPcp43_225*RLcp43_126-OPcp43_226*RLcp43_175-OPcp43_26*RLcp43_121;

% = = Block_1_0_0_44_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp43_175;
    sens.P(2) = POcp43_275;
    sens.P(3) = POcp43_375;
    sens.R(1,1) = ROcp43_126;
    sens.R(1,2) = ROcp43_226;
    sens.R(1,3) = ROcp43_326;
    sens.R(2,1) = ROcp43_426;
    sens.R(2,2) = ROcp43_526;
    sens.R(2,3) = ROcp43_626;
    sens.R(3,1) = ROcp43_725;
    sens.R(3,2) = ROcp43_825;
    sens.R(3,3) = ROcp43_925;
    sens.V(1) = VIcp43_175;
    sens.V(2) = VIcp43_275;
    sens.V(3) = VIcp43_375;
    sens.OM(1) = OMcp43_126;
    sens.OM(2) = OMcp43_226;
    sens.OM(3) = OMcp43_326;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp43_175_5;
    sens.J(1,6) = JTcp43_175_6;
    sens.J(1,21) = JTcp43_175_7;
    sens.J(1,22) = JTcp43_175_8;
    sens.J(1,23) = JTcp43_175_9;
    sens.J(1,24) = JTcp43_175_10;
    sens.J(1,25) = JTcp43_175_11;
    sens.J(1,26) = JTcp43_175_12;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp43_275_4;
    sens.J(2,5) = JTcp43_275_5;
    sens.J(2,6) = JTcp43_275_6;
    sens.J(2,21) = JTcp43_275_7;
    sens.J(2,22) = JTcp43_275_8;
    sens.J(2,23) = JTcp43_275_9;
    sens.J(2,24) = JTcp43_275_10;
    sens.J(2,25) = JTcp43_275_11;
    sens.J(2,26) = JTcp43_275_12;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp43_375_4;
    sens.J(3,5) = JTcp43_375_5;
    sens.J(3,6) = JTcp43_375_6;
    sens.J(3,21) = JTcp43_375_7;
    sens.J(3,22) = JTcp43_375_8;
    sens.J(3,23) = JTcp43_375_9;
    sens.J(3,24) = JTcp43_375_10;
    sens.J(3,25) = JTcp43_375_11;
    sens.J(3,26) = JTcp43_375_12;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,21) = ROcp43_16;
    sens.J(4,22) = ROcp43_421;
    sens.J(4,23) = ROcp43_722;
    sens.J(4,24) = ROcp43_423;
    sens.J(4,25) = ROcp43_124;
    sens.J(4,26) = ROcp43_725;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp43_85;
    sens.J(5,21) = ROcp43_26;
    sens.J(5,22) = ROcp43_521;
    sens.J(5,23) = ROcp43_822;
    sens.J(5,24) = ROcp43_523;
    sens.J(5,25) = ROcp43_224;
    sens.J(5,26) = ROcp43_825;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp43_95;
    sens.J(6,21) = ROcp43_36;
    sens.J(6,22) = ROcp43_621;
    sens.J(6,23) = ROcp43_922;
    sens.J(6,24) = ROcp43_623;
    sens.J(6,25) = ROcp43_324;
    sens.J(6,26) = ROcp43_925;
    sens.A(1) = ACcp43_175;
    sens.A(2) = ACcp43_275;
    sens.A(3) = ACcp43_375;
    sens.OMP(1) = OPcp43_126;
    sens.OMP(2) = OPcp43_226;
    sens.OMP(3) = OPcp43_326;
 
% 
case 45, 


% = = Block_1_0_0_45_0_1 = = 
 
% Sensor Kinematics 


    ROcp44_25 = S4*S5;
    ROcp44_35 = -C4*S5;
    ROcp44_85 = -S4*C5;
    ROcp44_95 = C4*C5;
    ROcp44_16 = C5*C6;
    ROcp44_26 = ROcp44_25*C6+C4*S6;
    ROcp44_36 = ROcp44_35*C6+S4*S6;
    ROcp44_46 = -C5*S6;
    ROcp44_56 = -(ROcp44_25*S6-C4*C6);
    ROcp44_66 = -(ROcp44_35*S6-S4*C6);
    OMcp44_25 = qd(5)*C4;
    OMcp44_35 = qd(5)*S4;
    OMcp44_16 = qd(4)+qd(6)*S5;
    OMcp44_26 = OMcp44_25+ROcp44_85*qd(6);
    OMcp44_36 = OMcp44_35+ROcp44_95*qd(6);
    OPcp44_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp44_26 = ROcp44_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp44_35*S5-ROcp44_95*qd(4));
    OPcp44_36 = ROcp44_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp44_25*S5-ROcp44_85*qd(4));

% = = Block_1_0_0_45_0_4 = = 
 
% Sensor Kinematics 


    ROcp44_421 = ROcp44_46*C21+S21*S5;
    ROcp44_521 = ROcp44_56*C21+ROcp44_85*S21;
    ROcp44_621 = ROcp44_66*C21+ROcp44_95*S21;
    ROcp44_721 = -(ROcp44_46*S21-C21*S5);
    ROcp44_821 = -(ROcp44_56*S21-ROcp44_85*C21);
    ROcp44_921 = -(ROcp44_66*S21-ROcp44_95*C21);
    ROcp44_122 = ROcp44_16*C22-ROcp44_721*S22;
    ROcp44_222 = ROcp44_26*C22-ROcp44_821*S22;
    ROcp44_322 = ROcp44_36*C22-ROcp44_921*S22;
    ROcp44_722 = ROcp44_16*S22+ROcp44_721*C22;
    ROcp44_822 = ROcp44_26*S22+ROcp44_821*C22;
    ROcp44_922 = ROcp44_36*S22+ROcp44_921*C22;
    ROcp44_123 = ROcp44_122*C23+ROcp44_421*S23;
    ROcp44_223 = ROcp44_222*C23+ROcp44_521*S23;
    ROcp44_323 = ROcp44_322*C23+ROcp44_621*S23;
    ROcp44_423 = -(ROcp44_122*S23-ROcp44_421*C23);
    ROcp44_523 = -(ROcp44_222*S23-ROcp44_521*C23);
    ROcp44_623 = -(ROcp44_322*S23-ROcp44_621*C23);
    RLcp44_121 = ROcp44_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp44_221 = ROcp44_26*s.dpt(1,3)+ROcp44_85*s.dpt(3,3);
    RLcp44_321 = ROcp44_36*s.dpt(1,3)+ROcp44_95*s.dpt(3,3);
    OMcp44_121 = OMcp44_16+ROcp44_16*qd(21);
    OMcp44_221 = OMcp44_26+ROcp44_26*qd(21);
    OMcp44_321 = OMcp44_36+ROcp44_36*qd(21);
    ORcp44_121 = OMcp44_26*RLcp44_321-OMcp44_36*RLcp44_221;
    ORcp44_221 = -(OMcp44_16*RLcp44_321-OMcp44_36*RLcp44_121);
    ORcp44_321 = OMcp44_16*RLcp44_221-OMcp44_26*RLcp44_121;
    OMcp44_122 = OMcp44_121+ROcp44_421*qd(22);
    OMcp44_222 = OMcp44_221+ROcp44_521*qd(22);
    OMcp44_322 = OMcp44_321+ROcp44_621*qd(22);
    OPcp44_122 = OPcp44_16+ROcp44_16*qdd(21)+ROcp44_421*qdd(22)+qd(21)*(OMcp44_26*ROcp44_36-OMcp44_36*ROcp44_26)+qd(22)*(OMcp44_221*ROcp44_621-...
 OMcp44_321*ROcp44_521);
    OPcp44_222 = OPcp44_26+ROcp44_26*qdd(21)+ROcp44_521*qdd(22)-qd(21)*(OMcp44_16*ROcp44_36-OMcp44_36*ROcp44_16)-qd(22)*(OMcp44_121*ROcp44_621-...
 OMcp44_321*ROcp44_421);
    OPcp44_322 = OPcp44_36+ROcp44_36*qdd(21)+ROcp44_621*qdd(22)+qd(21)*(OMcp44_16*ROcp44_26-OMcp44_26*ROcp44_16)+qd(22)*(OMcp44_121*ROcp44_521-...
 OMcp44_221*ROcp44_421);
    RLcp44_123 = ROcp44_722*s.dpt(3,37);
    RLcp44_223 = ROcp44_822*s.dpt(3,37);
    RLcp44_323 = ROcp44_922*s.dpt(3,37);
    OMcp44_123 = OMcp44_122+ROcp44_722*qd(23);
    OMcp44_223 = OMcp44_222+ROcp44_822*qd(23);
    OMcp44_323 = OMcp44_322+ROcp44_922*qd(23);
    ORcp44_123 = OMcp44_222*RLcp44_323-OMcp44_322*RLcp44_223;
    ORcp44_223 = -(OMcp44_122*RLcp44_323-OMcp44_322*RLcp44_123);
    ORcp44_323 = OMcp44_122*RLcp44_223-OMcp44_222*RLcp44_123;
    OPcp44_123 = OPcp44_122+ROcp44_722*qdd(23)+qd(23)*(OMcp44_222*ROcp44_922-OMcp44_322*ROcp44_822);
    OPcp44_223 = OPcp44_222+ROcp44_822*qdd(23)-qd(23)*(OMcp44_122*ROcp44_922-OMcp44_322*ROcp44_722);
    OPcp44_323 = OPcp44_322+ROcp44_922*qdd(23)+qd(23)*(OMcp44_122*ROcp44_822-OMcp44_222*ROcp44_722);

% = = Block_1_0_0_45_0_5 = = 
 
% Sensor Kinematics 


    ROcp44_124 = ROcp44_123*C24-ROcp44_722*S24;
    ROcp44_224 = ROcp44_223*C24-ROcp44_822*S24;
    ROcp44_324 = ROcp44_323*C24-ROcp44_922*S24;
    ROcp44_724 = ROcp44_123*S24+ROcp44_722*C24;
    ROcp44_824 = ROcp44_223*S24+ROcp44_822*C24;
    ROcp44_924 = ROcp44_323*S24+ROcp44_922*C24;
    ROcp44_425 = ROcp44_423*C25+ROcp44_724*S25;
    ROcp44_525 = ROcp44_523*C25+ROcp44_824*S25;
    ROcp44_625 = ROcp44_623*C25+ROcp44_924*S25;
    ROcp44_725 = -(ROcp44_423*S25-ROcp44_724*C25);
    ROcp44_825 = -(ROcp44_523*S25-ROcp44_824*C25);
    ROcp44_925 = -(ROcp44_623*S25-ROcp44_924*C25);
    ROcp44_126 = ROcp44_124*C26+ROcp44_425*S26;
    ROcp44_226 = ROcp44_224*C26+ROcp44_525*S26;
    ROcp44_326 = ROcp44_324*C26+ROcp44_625*S26;
    ROcp44_426 = -(ROcp44_124*S26-ROcp44_425*C26);
    ROcp44_526 = -(ROcp44_224*S26-ROcp44_525*C26);
    ROcp44_626 = -(ROcp44_324*S26-ROcp44_625*C26);
    ROcp44_127 = ROcp44_126*C27-ROcp44_725*S27;
    ROcp44_227 = ROcp44_226*C27-ROcp44_825*S27;
    ROcp44_327 = ROcp44_326*C27-ROcp44_925*S27;
    ROcp44_727 = ROcp44_126*S27+ROcp44_725*C27;
    ROcp44_827 = ROcp44_226*S27+ROcp44_825*C27;
    ROcp44_927 = ROcp44_326*S27+ROcp44_925*C27;
    RLcp44_124 = ROcp44_123*s.dpt(1,41)+ROcp44_423*s.dpt(2,41)+ROcp44_722*s.dpt(3,41);
    RLcp44_224 = ROcp44_223*s.dpt(1,41)+ROcp44_523*s.dpt(2,41)+ROcp44_822*s.dpt(3,41);
    RLcp44_324 = ROcp44_323*s.dpt(1,41)+ROcp44_623*s.dpt(2,41)+ROcp44_922*s.dpt(3,41);
    OMcp44_124 = OMcp44_123+ROcp44_423*qd(24);
    OMcp44_224 = OMcp44_223+ROcp44_523*qd(24);
    OMcp44_324 = OMcp44_323+ROcp44_623*qd(24);
    ORcp44_124 = OMcp44_223*RLcp44_324-OMcp44_323*RLcp44_224;
    ORcp44_224 = -(OMcp44_123*RLcp44_324-OMcp44_323*RLcp44_124);
    ORcp44_324 = OMcp44_123*RLcp44_224-OMcp44_223*RLcp44_124;
    OPcp44_124 = OPcp44_123+ROcp44_423*qdd(24)+qd(24)*(OMcp44_223*ROcp44_623-OMcp44_323*ROcp44_523);
    OPcp44_224 = OPcp44_223+ROcp44_523*qdd(24)-qd(24)*(OMcp44_123*ROcp44_623-OMcp44_323*ROcp44_423);
    OPcp44_324 = OPcp44_323+ROcp44_623*qdd(24)+qd(24)*(OMcp44_123*ROcp44_523-OMcp44_223*ROcp44_423);
    RLcp44_125 = ROcp44_423*s.dpt(2,44);
    RLcp44_225 = ROcp44_523*s.dpt(2,44);
    RLcp44_325 = ROcp44_623*s.dpt(2,44);
    OMcp44_125 = OMcp44_124+ROcp44_124*qd(25);
    OMcp44_225 = OMcp44_224+ROcp44_224*qd(25);
    OMcp44_325 = OMcp44_324+ROcp44_324*qd(25);
    ORcp44_125 = OMcp44_224*RLcp44_325-OMcp44_324*RLcp44_225;
    ORcp44_225 = -(OMcp44_124*RLcp44_325-OMcp44_324*RLcp44_125);
    ORcp44_325 = OMcp44_124*RLcp44_225-OMcp44_224*RLcp44_125;
    OPcp44_125 = OPcp44_124+ROcp44_124*qdd(25)+qd(25)*(OMcp44_224*ROcp44_324-OMcp44_324*ROcp44_224);
    OPcp44_225 = OPcp44_224+ROcp44_224*qdd(25)-qd(25)*(OMcp44_124*ROcp44_324-OMcp44_324*ROcp44_124);
    OPcp44_325 = OPcp44_324+ROcp44_324*qdd(25)+qd(25)*(OMcp44_124*ROcp44_224-OMcp44_224*ROcp44_124);
    RLcp44_126 = ROcp44_725*s.dpt(3,46);
    RLcp44_226 = ROcp44_825*s.dpt(3,46);
    RLcp44_326 = ROcp44_925*s.dpt(3,46);
    OMcp44_126 = OMcp44_125+ROcp44_725*qd(26);
    OMcp44_226 = OMcp44_225+ROcp44_825*qd(26);
    OMcp44_326 = OMcp44_325+ROcp44_925*qd(26);
    ORcp44_126 = OMcp44_225*RLcp44_326-OMcp44_325*RLcp44_226;
    ORcp44_226 = -(OMcp44_125*RLcp44_326-OMcp44_325*RLcp44_126);
    ORcp44_326 = OMcp44_125*RLcp44_226-OMcp44_225*RLcp44_126;
    OPcp44_126 = OPcp44_125+ROcp44_725*qdd(26)+qd(26)*(OMcp44_225*ROcp44_925-OMcp44_325*ROcp44_825);
    OPcp44_226 = OPcp44_225+ROcp44_825*qdd(26)-qd(26)*(OMcp44_125*ROcp44_925-OMcp44_325*ROcp44_725);
    OPcp44_326 = OPcp44_325+ROcp44_925*qdd(26)+qd(26)*(OMcp44_125*ROcp44_825-OMcp44_225*ROcp44_725);
    RLcp44_127 = ROcp44_725*s.dpt(3,48);
    RLcp44_227 = ROcp44_825*s.dpt(3,48);
    RLcp44_327 = ROcp44_925*s.dpt(3,48);
    OMcp44_127 = OMcp44_126+ROcp44_426*qd(27);
    OMcp44_227 = OMcp44_226+ROcp44_526*qd(27);
    OMcp44_327 = OMcp44_326+ROcp44_626*qd(27);
    ORcp44_127 = OMcp44_226*RLcp44_327-OMcp44_326*RLcp44_227;
    ORcp44_227 = -(OMcp44_126*RLcp44_327-OMcp44_326*RLcp44_127);
    ORcp44_327 = OMcp44_126*RLcp44_227-OMcp44_226*RLcp44_127;
    OPcp44_127 = OPcp44_126+ROcp44_426*qdd(27)+qd(27)*(OMcp44_226*ROcp44_626-OMcp44_326*ROcp44_526);
    OPcp44_227 = OPcp44_226+ROcp44_526*qdd(27)-qd(27)*(OMcp44_126*ROcp44_626-OMcp44_326*ROcp44_426);
    OPcp44_327 = OPcp44_326+ROcp44_626*qdd(27)+qd(27)*(OMcp44_126*ROcp44_526-OMcp44_226*ROcp44_426);
    RLcp44_176 = ROcp44_127*s.dpt(1,49)+ROcp44_426*s.dpt(2,49)+ROcp44_727*s.dpt(3,49);
    RLcp44_276 = ROcp44_227*s.dpt(1,49)+ROcp44_526*s.dpt(2,49)+ROcp44_827*s.dpt(3,49);
    RLcp44_376 = ROcp44_327*s.dpt(1,49)+ROcp44_626*s.dpt(2,49)+ROcp44_927*s.dpt(3,49);
    POcp44_176 = RLcp44_121+RLcp44_123+RLcp44_124+RLcp44_125+RLcp44_126+RLcp44_127+RLcp44_176+q(1);
    POcp44_276 = RLcp44_221+RLcp44_223+RLcp44_224+RLcp44_225+RLcp44_226+RLcp44_227+RLcp44_276+q(2);
    POcp44_376 = RLcp44_321+RLcp44_323+RLcp44_324+RLcp44_325+RLcp44_326+RLcp44_327+RLcp44_376+q(3);
    JTcp44_276_4 = -(RLcp44_321+RLcp44_323+RLcp44_324+RLcp44_325+RLcp44_326+RLcp44_327+RLcp44_376);
    JTcp44_376_4 = RLcp44_221+RLcp44_223+RLcp44_224+RLcp44_225+RLcp44_226+RLcp44_227+RLcp44_276;
    JTcp44_176_5 = C4*(RLcp44_321+RLcp44_323+RLcp44_324+RLcp44_325+RLcp44_326+RLcp44_327)-S4*(RLcp44_221+RLcp44_223)-S4*(RLcp44_224+RLcp44_225)-S4*...
 (RLcp44_226+RLcp44_227)-RLcp44_276*S4+RLcp44_376*C4;
    JTcp44_276_5 = S4*(RLcp44_121+RLcp44_123+RLcp44_124+RLcp44_125+RLcp44_126+RLcp44_127+RLcp44_176);
    JTcp44_376_5 = -C4*(RLcp44_121+RLcp44_123+RLcp44_124+RLcp44_125+RLcp44_126+RLcp44_127+RLcp44_176);
    JTcp44_176_6 = ROcp44_85*(RLcp44_321+RLcp44_323+RLcp44_324+RLcp44_325+RLcp44_326+RLcp44_327)-ROcp44_95*(RLcp44_221+RLcp44_223)-ROcp44_95*(...
 RLcp44_224+RLcp44_225)-ROcp44_95*(RLcp44_226+RLcp44_227)-RLcp44_276*ROcp44_95+RLcp44_376*ROcp44_85;
    JTcp44_276_6 = -(RLcp44_376*S5-ROcp44_95*(RLcp44_121+RLcp44_123+RLcp44_124+RLcp44_125+RLcp44_126+RLcp44_127+RLcp44_176)+S5*(RLcp44_321+...
 RLcp44_323)+S5*(RLcp44_324+RLcp44_325)+S5*(RLcp44_326+RLcp44_327));
    JTcp44_376_6 = RLcp44_276*S5-ROcp44_85*(RLcp44_121+RLcp44_123+RLcp44_124+RLcp44_125+RLcp44_126+RLcp44_127+RLcp44_176)+S5*(RLcp44_221+RLcp44_223...
 )+S5*(RLcp44_224+RLcp44_225)+S5*(RLcp44_226+RLcp44_227);
    JTcp44_176_7 = ROcp44_26*(RLcp44_323+RLcp44_324+RLcp44_325+RLcp44_326+RLcp44_327+RLcp44_376)-ROcp44_36*(RLcp44_223+RLcp44_224)-ROcp44_36*(...
 RLcp44_225+RLcp44_226)-ROcp44_36*(RLcp44_227+RLcp44_276);
    JTcp44_276_7 = -(ROcp44_16*(RLcp44_323+RLcp44_324+RLcp44_325+RLcp44_326+RLcp44_327+RLcp44_376)-ROcp44_36*(RLcp44_123+RLcp44_124)-ROcp44_36*(...
 RLcp44_125+RLcp44_126)-ROcp44_36*(RLcp44_127+RLcp44_176));
    JTcp44_376_7 = ROcp44_16*(RLcp44_223+RLcp44_224+RLcp44_225+RLcp44_226+RLcp44_227+RLcp44_276)-ROcp44_26*(RLcp44_123+RLcp44_124)-ROcp44_26*(...
 RLcp44_125+RLcp44_126)-ROcp44_26*(RLcp44_127+RLcp44_176);
    JTcp44_176_8 = ROcp44_521*(RLcp44_323+RLcp44_324+RLcp44_325+RLcp44_326+RLcp44_327+RLcp44_376)-ROcp44_621*(RLcp44_223+RLcp44_224)-ROcp44_621*(...
 RLcp44_225+RLcp44_226)-ROcp44_621*(RLcp44_227+RLcp44_276);
    JTcp44_276_8 = -(ROcp44_421*(RLcp44_323+RLcp44_324+RLcp44_325+RLcp44_326+RLcp44_327+RLcp44_376)-ROcp44_621*(RLcp44_123+RLcp44_124)-ROcp44_621*(...
 RLcp44_125+RLcp44_126)-ROcp44_621*(RLcp44_127+RLcp44_176));
    JTcp44_376_8 = ROcp44_421*(RLcp44_223+RLcp44_224+RLcp44_225+RLcp44_226+RLcp44_227+RLcp44_276)-ROcp44_521*(RLcp44_123+RLcp44_124)-ROcp44_521*(...
 RLcp44_125+RLcp44_126)-ROcp44_521*(RLcp44_127+RLcp44_176);
    JTcp44_176_9 = ROcp44_822*(RLcp44_324+RLcp44_325+RLcp44_326+RLcp44_327)-ROcp44_922*(RLcp44_224+RLcp44_225)-ROcp44_922*(RLcp44_226+RLcp44_227)-...
 RLcp44_276*ROcp44_922+RLcp44_376*ROcp44_822;
    JTcp44_276_9 = RLcp44_176*ROcp44_922-RLcp44_376*ROcp44_722-ROcp44_722*(RLcp44_324+RLcp44_325+RLcp44_326+RLcp44_327)+ROcp44_922*(RLcp44_124+...
 RLcp44_125)+ROcp44_922*(RLcp44_126+RLcp44_127);
    JTcp44_376_9 = ROcp44_722*(RLcp44_224+RLcp44_225+RLcp44_226+RLcp44_227)-ROcp44_822*(RLcp44_124+RLcp44_125)-ROcp44_822*(RLcp44_126+RLcp44_127)-...
 RLcp44_176*ROcp44_822+RLcp44_276*ROcp44_722;
    JTcp44_176_10 = ROcp44_523*(RLcp44_325+RLcp44_326+RLcp44_327+RLcp44_376)-ROcp44_623*(RLcp44_225+RLcp44_226)-ROcp44_623*(RLcp44_227+RLcp44_276);
    JTcp44_276_10 = -(ROcp44_423*(RLcp44_325+RLcp44_326+RLcp44_327+RLcp44_376)-ROcp44_623*(RLcp44_125+RLcp44_126)-ROcp44_623*(RLcp44_127+RLcp44_176...
 ));
    JTcp44_376_10 = ROcp44_423*(RLcp44_225+RLcp44_226+RLcp44_227+RLcp44_276)-ROcp44_523*(RLcp44_125+RLcp44_126)-ROcp44_523*(RLcp44_127+RLcp44_176);
    JTcp44_176_11 = ROcp44_224*(RLcp44_326+RLcp44_327)-ROcp44_324*(RLcp44_226+RLcp44_227)-RLcp44_276*ROcp44_324+RLcp44_376*ROcp44_224;
    JTcp44_276_11 = RLcp44_176*ROcp44_324-RLcp44_376*ROcp44_124-ROcp44_124*(RLcp44_326+RLcp44_327)+ROcp44_324*(RLcp44_126+RLcp44_127);
    JTcp44_376_11 = ROcp44_124*(RLcp44_226+RLcp44_227)-ROcp44_224*(RLcp44_126+RLcp44_127)-RLcp44_176*ROcp44_224+RLcp44_276*ROcp44_124;
    JTcp44_176_12 = ROcp44_825*(RLcp44_327+RLcp44_376)-ROcp44_925*(RLcp44_227+RLcp44_276);
    JTcp44_276_12 = -(ROcp44_725*(RLcp44_327+RLcp44_376)-ROcp44_925*(RLcp44_127+RLcp44_176));
    JTcp44_376_12 = ROcp44_725*(RLcp44_227+RLcp44_276)-ROcp44_825*(RLcp44_127+RLcp44_176);
    JTcp44_176_13 = -(RLcp44_276*ROcp44_626-RLcp44_376*ROcp44_526);
    JTcp44_276_13 = RLcp44_176*ROcp44_626-RLcp44_376*ROcp44_426;
    JTcp44_376_13 = -(RLcp44_176*ROcp44_526-RLcp44_276*ROcp44_426);
    ORcp44_176 = OMcp44_227*RLcp44_376-OMcp44_327*RLcp44_276;
    ORcp44_276 = -(OMcp44_127*RLcp44_376-OMcp44_327*RLcp44_176);
    ORcp44_376 = OMcp44_127*RLcp44_276-OMcp44_227*RLcp44_176;
    VIcp44_176 = ORcp44_121+ORcp44_123+ORcp44_124+ORcp44_125+ORcp44_126+ORcp44_127+ORcp44_176+qd(1);
    VIcp44_276 = ORcp44_221+ORcp44_223+ORcp44_224+ORcp44_225+ORcp44_226+ORcp44_227+ORcp44_276+qd(2);
    VIcp44_376 = ORcp44_321+ORcp44_323+ORcp44_324+ORcp44_325+ORcp44_326+ORcp44_327+ORcp44_376+qd(3);
    ACcp44_176 = qdd(1)+OMcp44_222*ORcp44_323+OMcp44_223*ORcp44_324+OMcp44_224*ORcp44_325+OMcp44_225*ORcp44_326+OMcp44_226*ORcp44_327+OMcp44_227*...
 ORcp44_376+OMcp44_26*ORcp44_321-OMcp44_322*ORcp44_223-OMcp44_323*ORcp44_224-OMcp44_324*ORcp44_225-OMcp44_325*ORcp44_226-OMcp44_326*ORcp44_227-...
 OMcp44_327*ORcp44_276-OMcp44_36*ORcp44_221+OPcp44_222*RLcp44_323+OPcp44_223*RLcp44_324+OPcp44_224*RLcp44_325+OPcp44_225*RLcp44_326+OPcp44_226*...
 RLcp44_327+OPcp44_227*RLcp44_376+OPcp44_26*RLcp44_321-OPcp44_322*RLcp44_223-OPcp44_323*RLcp44_224-OPcp44_324*RLcp44_225-OPcp44_325*RLcp44_226-...
 OPcp44_326*RLcp44_227-OPcp44_327*RLcp44_276-OPcp44_36*RLcp44_221;
    ACcp44_276 = qdd(2)-OMcp44_122*ORcp44_323-OMcp44_123*ORcp44_324-OMcp44_124*ORcp44_325-OMcp44_125*ORcp44_326-OMcp44_126*ORcp44_327-OMcp44_127*...
 ORcp44_376-OMcp44_16*ORcp44_321+OMcp44_322*ORcp44_123+OMcp44_323*ORcp44_124+OMcp44_324*ORcp44_125+OMcp44_325*ORcp44_126+OMcp44_326*ORcp44_127+...
 OMcp44_327*ORcp44_176+OMcp44_36*ORcp44_121-OPcp44_122*RLcp44_323-OPcp44_123*RLcp44_324-OPcp44_124*RLcp44_325-OPcp44_125*RLcp44_326-OPcp44_126*...
 RLcp44_327-OPcp44_127*RLcp44_376-OPcp44_16*RLcp44_321+OPcp44_322*RLcp44_123+OPcp44_323*RLcp44_124+OPcp44_324*RLcp44_125+OPcp44_325*RLcp44_126+...
 OPcp44_326*RLcp44_127+OPcp44_327*RLcp44_176+OPcp44_36*RLcp44_121;
    ACcp44_376 = qdd(3)+OMcp44_122*ORcp44_223+OMcp44_123*ORcp44_224+OMcp44_124*ORcp44_225+OMcp44_125*ORcp44_226+OMcp44_126*ORcp44_227+OMcp44_127*...
 ORcp44_276+OMcp44_16*ORcp44_221-OMcp44_222*ORcp44_123-OMcp44_223*ORcp44_124-OMcp44_224*ORcp44_125-OMcp44_225*ORcp44_126-OMcp44_226*ORcp44_127-...
 OMcp44_227*ORcp44_176-OMcp44_26*ORcp44_121+OPcp44_122*RLcp44_223+OPcp44_123*RLcp44_224+OPcp44_124*RLcp44_225+OPcp44_125*RLcp44_226+OPcp44_126*...
 RLcp44_227+OPcp44_127*RLcp44_276+OPcp44_16*RLcp44_221-OPcp44_222*RLcp44_123-OPcp44_223*RLcp44_124-OPcp44_224*RLcp44_125-OPcp44_225*RLcp44_126-...
 OPcp44_226*RLcp44_127-OPcp44_227*RLcp44_176-OPcp44_26*RLcp44_121;

% = = Block_1_0_0_45_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp44_176;
    sens.P(2) = POcp44_276;
    sens.P(3) = POcp44_376;
    sens.R(1,1) = ROcp44_127;
    sens.R(1,2) = ROcp44_227;
    sens.R(1,3) = ROcp44_327;
    sens.R(2,1) = ROcp44_426;
    sens.R(2,2) = ROcp44_526;
    sens.R(2,3) = ROcp44_626;
    sens.R(3,1) = ROcp44_727;
    sens.R(3,2) = ROcp44_827;
    sens.R(3,3) = ROcp44_927;
    sens.V(1) = VIcp44_176;
    sens.V(2) = VIcp44_276;
    sens.V(3) = VIcp44_376;
    sens.OM(1) = OMcp44_127;
    sens.OM(2) = OMcp44_227;
    sens.OM(3) = OMcp44_327;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp44_176_5;
    sens.J(1,6) = JTcp44_176_6;
    sens.J(1,21) = JTcp44_176_7;
    sens.J(1,22) = JTcp44_176_8;
    sens.J(1,23) = JTcp44_176_9;
    sens.J(1,24) = JTcp44_176_10;
    sens.J(1,25) = JTcp44_176_11;
    sens.J(1,26) = JTcp44_176_12;
    sens.J(1,27) = JTcp44_176_13;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp44_276_4;
    sens.J(2,5) = JTcp44_276_5;
    sens.J(2,6) = JTcp44_276_6;
    sens.J(2,21) = JTcp44_276_7;
    sens.J(2,22) = JTcp44_276_8;
    sens.J(2,23) = JTcp44_276_9;
    sens.J(2,24) = JTcp44_276_10;
    sens.J(2,25) = JTcp44_276_11;
    sens.J(2,26) = JTcp44_276_12;
    sens.J(2,27) = JTcp44_276_13;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp44_376_4;
    sens.J(3,5) = JTcp44_376_5;
    sens.J(3,6) = JTcp44_376_6;
    sens.J(3,21) = JTcp44_376_7;
    sens.J(3,22) = JTcp44_376_8;
    sens.J(3,23) = JTcp44_376_9;
    sens.J(3,24) = JTcp44_376_10;
    sens.J(3,25) = JTcp44_376_11;
    sens.J(3,26) = JTcp44_376_12;
    sens.J(3,27) = JTcp44_376_13;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,21) = ROcp44_16;
    sens.J(4,22) = ROcp44_421;
    sens.J(4,23) = ROcp44_722;
    sens.J(4,24) = ROcp44_423;
    sens.J(4,25) = ROcp44_124;
    sens.J(4,26) = ROcp44_725;
    sens.J(4,27) = ROcp44_426;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp44_85;
    sens.J(5,21) = ROcp44_26;
    sens.J(5,22) = ROcp44_521;
    sens.J(5,23) = ROcp44_822;
    sens.J(5,24) = ROcp44_523;
    sens.J(5,25) = ROcp44_224;
    sens.J(5,26) = ROcp44_825;
    sens.J(5,27) = ROcp44_526;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp44_95;
    sens.J(6,21) = ROcp44_36;
    sens.J(6,22) = ROcp44_621;
    sens.J(6,23) = ROcp44_922;
    sens.J(6,24) = ROcp44_623;
    sens.J(6,25) = ROcp44_324;
    sens.J(6,26) = ROcp44_925;
    sens.J(6,27) = ROcp44_626;
    sens.A(1) = ACcp44_176;
    sens.A(2) = ACcp44_276;
    sens.A(3) = ACcp44_376;
    sens.OMP(1) = OPcp44_127;
    sens.OMP(2) = OPcp44_227;
    sens.OMP(3) = OPcp44_327;
 
% 
case 46, 


% = = Block_1_0_0_46_0_1 = = 
 
% Sensor Kinematics 


    ROcp45_25 = S4*S5;
    ROcp45_35 = -C4*S5;
    ROcp45_85 = -S4*C5;
    ROcp45_95 = C4*C5;
    ROcp45_16 = C5*C6;
    ROcp45_26 = ROcp45_25*C6+C4*S6;
    ROcp45_36 = ROcp45_35*C6+S4*S6;
    ROcp45_46 = -C5*S6;
    ROcp45_56 = -(ROcp45_25*S6-C4*C6);
    ROcp45_66 = -(ROcp45_35*S6-S4*C6);
    OMcp45_25 = qd(5)*C4;
    OMcp45_35 = qd(5)*S4;
    OMcp45_16 = qd(4)+qd(6)*S5;
    OMcp45_26 = OMcp45_25+ROcp45_85*qd(6);
    OMcp45_36 = OMcp45_35+ROcp45_95*qd(6);
    OPcp45_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp45_26 = ROcp45_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp45_35*S5-ROcp45_95*qd(4));
    OPcp45_36 = ROcp45_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp45_25*S5-ROcp45_85*qd(4));

% = = Block_1_0_0_46_0_4 = = 
 
% Sensor Kinematics 


    ROcp45_421 = ROcp45_46*C21+S21*S5;
    ROcp45_521 = ROcp45_56*C21+ROcp45_85*S21;
    ROcp45_621 = ROcp45_66*C21+ROcp45_95*S21;
    ROcp45_721 = -(ROcp45_46*S21-C21*S5);
    ROcp45_821 = -(ROcp45_56*S21-ROcp45_85*C21);
    ROcp45_921 = -(ROcp45_66*S21-ROcp45_95*C21);
    ROcp45_122 = ROcp45_16*C22-ROcp45_721*S22;
    ROcp45_222 = ROcp45_26*C22-ROcp45_821*S22;
    ROcp45_322 = ROcp45_36*C22-ROcp45_921*S22;
    ROcp45_722 = ROcp45_16*S22+ROcp45_721*C22;
    ROcp45_822 = ROcp45_26*S22+ROcp45_821*C22;
    ROcp45_922 = ROcp45_36*S22+ROcp45_921*C22;
    ROcp45_123 = ROcp45_122*C23+ROcp45_421*S23;
    ROcp45_223 = ROcp45_222*C23+ROcp45_521*S23;
    ROcp45_323 = ROcp45_322*C23+ROcp45_621*S23;
    ROcp45_423 = -(ROcp45_122*S23-ROcp45_421*C23);
    ROcp45_523 = -(ROcp45_222*S23-ROcp45_521*C23);
    ROcp45_623 = -(ROcp45_322*S23-ROcp45_621*C23);
    RLcp45_121 = ROcp45_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp45_221 = ROcp45_26*s.dpt(1,3)+ROcp45_85*s.dpt(3,3);
    RLcp45_321 = ROcp45_36*s.dpt(1,3)+ROcp45_95*s.dpt(3,3);
    OMcp45_121 = OMcp45_16+ROcp45_16*qd(21);
    OMcp45_221 = OMcp45_26+ROcp45_26*qd(21);
    OMcp45_321 = OMcp45_36+ROcp45_36*qd(21);
    ORcp45_121 = OMcp45_26*RLcp45_321-OMcp45_36*RLcp45_221;
    ORcp45_221 = -(OMcp45_16*RLcp45_321-OMcp45_36*RLcp45_121);
    ORcp45_321 = OMcp45_16*RLcp45_221-OMcp45_26*RLcp45_121;
    OMcp45_122 = OMcp45_121+ROcp45_421*qd(22);
    OMcp45_222 = OMcp45_221+ROcp45_521*qd(22);
    OMcp45_322 = OMcp45_321+ROcp45_621*qd(22);
    OPcp45_122 = OPcp45_16+ROcp45_16*qdd(21)+ROcp45_421*qdd(22)+qd(21)*(OMcp45_26*ROcp45_36-OMcp45_36*ROcp45_26)+qd(22)*(OMcp45_221*ROcp45_621-...
 OMcp45_321*ROcp45_521);
    OPcp45_222 = OPcp45_26+ROcp45_26*qdd(21)+ROcp45_521*qdd(22)-qd(21)*(OMcp45_16*ROcp45_36-OMcp45_36*ROcp45_16)-qd(22)*(OMcp45_121*ROcp45_621-...
 OMcp45_321*ROcp45_421);
    OPcp45_322 = OPcp45_36+ROcp45_36*qdd(21)+ROcp45_621*qdd(22)+qd(21)*(OMcp45_16*ROcp45_26-OMcp45_26*ROcp45_16)+qd(22)*(OMcp45_121*ROcp45_521-...
 OMcp45_221*ROcp45_421);
    RLcp45_123 = ROcp45_722*s.dpt(3,37);
    RLcp45_223 = ROcp45_822*s.dpt(3,37);
    RLcp45_323 = ROcp45_922*s.dpt(3,37);
    OMcp45_123 = OMcp45_122+ROcp45_722*qd(23);
    OMcp45_223 = OMcp45_222+ROcp45_822*qd(23);
    OMcp45_323 = OMcp45_322+ROcp45_922*qd(23);
    ORcp45_123 = OMcp45_222*RLcp45_323-OMcp45_322*RLcp45_223;
    ORcp45_223 = -(OMcp45_122*RLcp45_323-OMcp45_322*RLcp45_123);
    ORcp45_323 = OMcp45_122*RLcp45_223-OMcp45_222*RLcp45_123;
    OPcp45_123 = OPcp45_122+ROcp45_722*qdd(23)+qd(23)*(OMcp45_222*ROcp45_922-OMcp45_322*ROcp45_822);
    OPcp45_223 = OPcp45_222+ROcp45_822*qdd(23)-qd(23)*(OMcp45_122*ROcp45_922-OMcp45_322*ROcp45_722);
    OPcp45_323 = OPcp45_322+ROcp45_922*qdd(23)+qd(23)*(OMcp45_122*ROcp45_822-OMcp45_222*ROcp45_722);

% = = Block_1_0_0_46_0_5 = = 
 
% Sensor Kinematics 


    ROcp45_124 = ROcp45_123*C24-ROcp45_722*S24;
    ROcp45_224 = ROcp45_223*C24-ROcp45_822*S24;
    ROcp45_324 = ROcp45_323*C24-ROcp45_922*S24;
    ROcp45_724 = ROcp45_123*S24+ROcp45_722*C24;
    ROcp45_824 = ROcp45_223*S24+ROcp45_822*C24;
    ROcp45_924 = ROcp45_323*S24+ROcp45_922*C24;
    ROcp45_425 = ROcp45_423*C25+ROcp45_724*S25;
    ROcp45_525 = ROcp45_523*C25+ROcp45_824*S25;
    ROcp45_625 = ROcp45_623*C25+ROcp45_924*S25;
    ROcp45_725 = -(ROcp45_423*S25-ROcp45_724*C25);
    ROcp45_825 = -(ROcp45_523*S25-ROcp45_824*C25);
    ROcp45_925 = -(ROcp45_623*S25-ROcp45_924*C25);
    ROcp45_126 = ROcp45_124*C26+ROcp45_425*S26;
    ROcp45_226 = ROcp45_224*C26+ROcp45_525*S26;
    ROcp45_326 = ROcp45_324*C26+ROcp45_625*S26;
    ROcp45_426 = -(ROcp45_124*S26-ROcp45_425*C26);
    ROcp45_526 = -(ROcp45_224*S26-ROcp45_525*C26);
    ROcp45_626 = -(ROcp45_324*S26-ROcp45_625*C26);
    ROcp45_127 = ROcp45_126*C27-ROcp45_725*S27;
    ROcp45_227 = ROcp45_226*C27-ROcp45_825*S27;
    ROcp45_327 = ROcp45_326*C27-ROcp45_925*S27;
    ROcp45_727 = ROcp45_126*S27+ROcp45_725*C27;
    ROcp45_827 = ROcp45_226*S27+ROcp45_825*C27;
    ROcp45_927 = ROcp45_326*S27+ROcp45_925*C27;
    RLcp45_124 = ROcp45_123*s.dpt(1,41)+ROcp45_423*s.dpt(2,41)+ROcp45_722*s.dpt(3,41);
    RLcp45_224 = ROcp45_223*s.dpt(1,41)+ROcp45_523*s.dpt(2,41)+ROcp45_822*s.dpt(3,41);
    RLcp45_324 = ROcp45_323*s.dpt(1,41)+ROcp45_623*s.dpt(2,41)+ROcp45_922*s.dpt(3,41);
    OMcp45_124 = OMcp45_123+ROcp45_423*qd(24);
    OMcp45_224 = OMcp45_223+ROcp45_523*qd(24);
    OMcp45_324 = OMcp45_323+ROcp45_623*qd(24);
    ORcp45_124 = OMcp45_223*RLcp45_324-OMcp45_323*RLcp45_224;
    ORcp45_224 = -(OMcp45_123*RLcp45_324-OMcp45_323*RLcp45_124);
    ORcp45_324 = OMcp45_123*RLcp45_224-OMcp45_223*RLcp45_124;
    OPcp45_124 = OPcp45_123+ROcp45_423*qdd(24)+qd(24)*(OMcp45_223*ROcp45_623-OMcp45_323*ROcp45_523);
    OPcp45_224 = OPcp45_223+ROcp45_523*qdd(24)-qd(24)*(OMcp45_123*ROcp45_623-OMcp45_323*ROcp45_423);
    OPcp45_324 = OPcp45_323+ROcp45_623*qdd(24)+qd(24)*(OMcp45_123*ROcp45_523-OMcp45_223*ROcp45_423);
    RLcp45_125 = ROcp45_423*s.dpt(2,44);
    RLcp45_225 = ROcp45_523*s.dpt(2,44);
    RLcp45_325 = ROcp45_623*s.dpt(2,44);
    OMcp45_125 = OMcp45_124+ROcp45_124*qd(25);
    OMcp45_225 = OMcp45_224+ROcp45_224*qd(25);
    OMcp45_325 = OMcp45_324+ROcp45_324*qd(25);
    ORcp45_125 = OMcp45_224*RLcp45_325-OMcp45_324*RLcp45_225;
    ORcp45_225 = -(OMcp45_124*RLcp45_325-OMcp45_324*RLcp45_125);
    ORcp45_325 = OMcp45_124*RLcp45_225-OMcp45_224*RLcp45_125;
    OPcp45_125 = OPcp45_124+ROcp45_124*qdd(25)+qd(25)*(OMcp45_224*ROcp45_324-OMcp45_324*ROcp45_224);
    OPcp45_225 = OPcp45_224+ROcp45_224*qdd(25)-qd(25)*(OMcp45_124*ROcp45_324-OMcp45_324*ROcp45_124);
    OPcp45_325 = OPcp45_324+ROcp45_324*qdd(25)+qd(25)*(OMcp45_124*ROcp45_224-OMcp45_224*ROcp45_124);
    RLcp45_126 = ROcp45_725*s.dpt(3,46);
    RLcp45_226 = ROcp45_825*s.dpt(3,46);
    RLcp45_326 = ROcp45_925*s.dpt(3,46);
    OMcp45_126 = OMcp45_125+ROcp45_725*qd(26);
    OMcp45_226 = OMcp45_225+ROcp45_825*qd(26);
    OMcp45_326 = OMcp45_325+ROcp45_925*qd(26);
    ORcp45_126 = OMcp45_225*RLcp45_326-OMcp45_325*RLcp45_226;
    ORcp45_226 = -(OMcp45_125*RLcp45_326-OMcp45_325*RLcp45_126);
    ORcp45_326 = OMcp45_125*RLcp45_226-OMcp45_225*RLcp45_126;
    OPcp45_126 = OPcp45_125+ROcp45_725*qdd(26)+qd(26)*(OMcp45_225*ROcp45_925-OMcp45_325*ROcp45_825);
    OPcp45_226 = OPcp45_225+ROcp45_825*qdd(26)-qd(26)*(OMcp45_125*ROcp45_925-OMcp45_325*ROcp45_725);
    OPcp45_326 = OPcp45_325+ROcp45_925*qdd(26)+qd(26)*(OMcp45_125*ROcp45_825-OMcp45_225*ROcp45_725);
    RLcp45_127 = ROcp45_725*s.dpt(3,48);
    RLcp45_227 = ROcp45_825*s.dpt(3,48);
    RLcp45_327 = ROcp45_925*s.dpt(3,48);
    OMcp45_127 = OMcp45_126+ROcp45_426*qd(27);
    OMcp45_227 = OMcp45_226+ROcp45_526*qd(27);
    OMcp45_327 = OMcp45_326+ROcp45_626*qd(27);
    ORcp45_127 = OMcp45_226*RLcp45_327-OMcp45_326*RLcp45_227;
    ORcp45_227 = -(OMcp45_126*RLcp45_327-OMcp45_326*RLcp45_127);
    ORcp45_327 = OMcp45_126*RLcp45_227-OMcp45_226*RLcp45_127;
    OPcp45_127 = OPcp45_126+ROcp45_426*qdd(27)+qd(27)*(OMcp45_226*ROcp45_626-OMcp45_326*ROcp45_526);
    OPcp45_227 = OPcp45_226+ROcp45_526*qdd(27)-qd(27)*(OMcp45_126*ROcp45_626-OMcp45_326*ROcp45_426);
    OPcp45_327 = OPcp45_326+ROcp45_626*qdd(27)+qd(27)*(OMcp45_126*ROcp45_526-OMcp45_226*ROcp45_426);
    RLcp45_177 = ROcp45_727*s.dpt(3,50);
    RLcp45_277 = ROcp45_827*s.dpt(3,50);
    RLcp45_377 = ROcp45_927*s.dpt(3,50);
    POcp45_177 = RLcp45_121+RLcp45_123+RLcp45_124+RLcp45_125+RLcp45_126+RLcp45_127+RLcp45_177+q(1);
    POcp45_277 = RLcp45_221+RLcp45_223+RLcp45_224+RLcp45_225+RLcp45_226+RLcp45_227+RLcp45_277+q(2);
    POcp45_377 = RLcp45_321+RLcp45_323+RLcp45_324+RLcp45_325+RLcp45_326+RLcp45_327+RLcp45_377+q(3);
    JTcp45_277_4 = -(RLcp45_321+RLcp45_323+RLcp45_324+RLcp45_325+RLcp45_326+RLcp45_327+RLcp45_377);
    JTcp45_377_4 = RLcp45_221+RLcp45_223+RLcp45_224+RLcp45_225+RLcp45_226+RLcp45_227+RLcp45_277;
    JTcp45_177_5 = C4*(RLcp45_321+RLcp45_323+RLcp45_324+RLcp45_325+RLcp45_326+RLcp45_327)-S4*(RLcp45_221+RLcp45_223)-S4*(RLcp45_224+RLcp45_225)-S4*...
 (RLcp45_226+RLcp45_227)-RLcp45_277*S4+RLcp45_377*C4;
    JTcp45_277_5 = S4*(RLcp45_121+RLcp45_123+RLcp45_124+RLcp45_125+RLcp45_126+RLcp45_127+RLcp45_177);
    JTcp45_377_5 = -C4*(RLcp45_121+RLcp45_123+RLcp45_124+RLcp45_125+RLcp45_126+RLcp45_127+RLcp45_177);
    JTcp45_177_6 = ROcp45_85*(RLcp45_321+RLcp45_323+RLcp45_324+RLcp45_325+RLcp45_326+RLcp45_327)-ROcp45_95*(RLcp45_221+RLcp45_223)-ROcp45_95*(...
 RLcp45_224+RLcp45_225)-ROcp45_95*(RLcp45_226+RLcp45_227)-RLcp45_277*ROcp45_95+RLcp45_377*ROcp45_85;
    JTcp45_277_6 = -(RLcp45_377*S5-ROcp45_95*(RLcp45_121+RLcp45_123+RLcp45_124+RLcp45_125+RLcp45_126+RLcp45_127+RLcp45_177)+S5*(RLcp45_321+...
 RLcp45_323)+S5*(RLcp45_324+RLcp45_325)+S5*(RLcp45_326+RLcp45_327));
    JTcp45_377_6 = RLcp45_277*S5-ROcp45_85*(RLcp45_121+RLcp45_123+RLcp45_124+RLcp45_125+RLcp45_126+RLcp45_127+RLcp45_177)+S5*(RLcp45_221+RLcp45_223...
 )+S5*(RLcp45_224+RLcp45_225)+S5*(RLcp45_226+RLcp45_227);
    JTcp45_177_7 = ROcp45_26*(RLcp45_323+RLcp45_324+RLcp45_325+RLcp45_326+RLcp45_327+RLcp45_377)-ROcp45_36*(RLcp45_223+RLcp45_224)-ROcp45_36*(...
 RLcp45_225+RLcp45_226)-ROcp45_36*(RLcp45_227+RLcp45_277);
    JTcp45_277_7 = -(ROcp45_16*(RLcp45_323+RLcp45_324+RLcp45_325+RLcp45_326+RLcp45_327+RLcp45_377)-ROcp45_36*(RLcp45_123+RLcp45_124)-ROcp45_36*(...
 RLcp45_125+RLcp45_126)-ROcp45_36*(RLcp45_127+RLcp45_177));
    JTcp45_377_7 = ROcp45_16*(RLcp45_223+RLcp45_224+RLcp45_225+RLcp45_226+RLcp45_227+RLcp45_277)-ROcp45_26*(RLcp45_123+RLcp45_124)-ROcp45_26*(...
 RLcp45_125+RLcp45_126)-ROcp45_26*(RLcp45_127+RLcp45_177);
    JTcp45_177_8 = ROcp45_521*(RLcp45_323+RLcp45_324+RLcp45_325+RLcp45_326+RLcp45_327+RLcp45_377)-ROcp45_621*(RLcp45_223+RLcp45_224)-ROcp45_621*(...
 RLcp45_225+RLcp45_226)-ROcp45_621*(RLcp45_227+RLcp45_277);
    JTcp45_277_8 = -(ROcp45_421*(RLcp45_323+RLcp45_324+RLcp45_325+RLcp45_326+RLcp45_327+RLcp45_377)-ROcp45_621*(RLcp45_123+RLcp45_124)-ROcp45_621*(...
 RLcp45_125+RLcp45_126)-ROcp45_621*(RLcp45_127+RLcp45_177));
    JTcp45_377_8 = ROcp45_421*(RLcp45_223+RLcp45_224+RLcp45_225+RLcp45_226+RLcp45_227+RLcp45_277)-ROcp45_521*(RLcp45_123+RLcp45_124)-ROcp45_521*(...
 RLcp45_125+RLcp45_126)-ROcp45_521*(RLcp45_127+RLcp45_177);
    JTcp45_177_9 = ROcp45_822*(RLcp45_324+RLcp45_325+RLcp45_326+RLcp45_327)-ROcp45_922*(RLcp45_224+RLcp45_225)-ROcp45_922*(RLcp45_226+RLcp45_227)-...
 RLcp45_277*ROcp45_922+RLcp45_377*ROcp45_822;
    JTcp45_277_9 = RLcp45_177*ROcp45_922-RLcp45_377*ROcp45_722-ROcp45_722*(RLcp45_324+RLcp45_325+RLcp45_326+RLcp45_327)+ROcp45_922*(RLcp45_124+...
 RLcp45_125)+ROcp45_922*(RLcp45_126+RLcp45_127);
    JTcp45_377_9 = ROcp45_722*(RLcp45_224+RLcp45_225+RLcp45_226+RLcp45_227)-ROcp45_822*(RLcp45_124+RLcp45_125)-ROcp45_822*(RLcp45_126+RLcp45_127)-...
 RLcp45_177*ROcp45_822+RLcp45_277*ROcp45_722;
    JTcp45_177_10 = ROcp45_523*(RLcp45_325+RLcp45_326+RLcp45_327+RLcp45_377)-ROcp45_623*(RLcp45_225+RLcp45_226)-ROcp45_623*(RLcp45_227+RLcp45_277);
    JTcp45_277_10 = -(ROcp45_423*(RLcp45_325+RLcp45_326+RLcp45_327+RLcp45_377)-ROcp45_623*(RLcp45_125+RLcp45_126)-ROcp45_623*(RLcp45_127+RLcp45_177...
 ));
    JTcp45_377_10 = ROcp45_423*(RLcp45_225+RLcp45_226+RLcp45_227+RLcp45_277)-ROcp45_523*(RLcp45_125+RLcp45_126)-ROcp45_523*(RLcp45_127+RLcp45_177);
    JTcp45_177_11 = ROcp45_224*(RLcp45_326+RLcp45_327)-ROcp45_324*(RLcp45_226+RLcp45_227)-RLcp45_277*ROcp45_324+RLcp45_377*ROcp45_224;
    JTcp45_277_11 = RLcp45_177*ROcp45_324-RLcp45_377*ROcp45_124-ROcp45_124*(RLcp45_326+RLcp45_327)+ROcp45_324*(RLcp45_126+RLcp45_127);
    JTcp45_377_11 = ROcp45_124*(RLcp45_226+RLcp45_227)-ROcp45_224*(RLcp45_126+RLcp45_127)-RLcp45_177*ROcp45_224+RLcp45_277*ROcp45_124;
    JTcp45_177_12 = ROcp45_825*(RLcp45_327+RLcp45_377)-ROcp45_925*(RLcp45_227+RLcp45_277);
    JTcp45_277_12 = -(ROcp45_725*(RLcp45_327+RLcp45_377)-ROcp45_925*(RLcp45_127+RLcp45_177));
    JTcp45_377_12 = ROcp45_725*(RLcp45_227+RLcp45_277)-ROcp45_825*(RLcp45_127+RLcp45_177);
    JTcp45_177_13 = -(RLcp45_277*ROcp45_626-RLcp45_377*ROcp45_526);
    JTcp45_277_13 = RLcp45_177*ROcp45_626-RLcp45_377*ROcp45_426;
    JTcp45_377_13 = -(RLcp45_177*ROcp45_526-RLcp45_277*ROcp45_426);
    ORcp45_177 = OMcp45_227*RLcp45_377-OMcp45_327*RLcp45_277;
    ORcp45_277 = -(OMcp45_127*RLcp45_377-OMcp45_327*RLcp45_177);
    ORcp45_377 = OMcp45_127*RLcp45_277-OMcp45_227*RLcp45_177;
    VIcp45_177 = ORcp45_121+ORcp45_123+ORcp45_124+ORcp45_125+ORcp45_126+ORcp45_127+ORcp45_177+qd(1);
    VIcp45_277 = ORcp45_221+ORcp45_223+ORcp45_224+ORcp45_225+ORcp45_226+ORcp45_227+ORcp45_277+qd(2);
    VIcp45_377 = ORcp45_321+ORcp45_323+ORcp45_324+ORcp45_325+ORcp45_326+ORcp45_327+ORcp45_377+qd(3);
    ACcp45_177 = qdd(1)+OMcp45_222*ORcp45_323+OMcp45_223*ORcp45_324+OMcp45_224*ORcp45_325+OMcp45_225*ORcp45_326+OMcp45_226*ORcp45_327+OMcp45_227*...
 ORcp45_377+OMcp45_26*ORcp45_321-OMcp45_322*ORcp45_223-OMcp45_323*ORcp45_224-OMcp45_324*ORcp45_225-OMcp45_325*ORcp45_226-OMcp45_326*ORcp45_227-...
 OMcp45_327*ORcp45_277-OMcp45_36*ORcp45_221+OPcp45_222*RLcp45_323+OPcp45_223*RLcp45_324+OPcp45_224*RLcp45_325+OPcp45_225*RLcp45_326+OPcp45_226*...
 RLcp45_327+OPcp45_227*RLcp45_377+OPcp45_26*RLcp45_321-OPcp45_322*RLcp45_223-OPcp45_323*RLcp45_224-OPcp45_324*RLcp45_225-OPcp45_325*RLcp45_226-...
 OPcp45_326*RLcp45_227-OPcp45_327*RLcp45_277-OPcp45_36*RLcp45_221;
    ACcp45_277 = qdd(2)-OMcp45_122*ORcp45_323-OMcp45_123*ORcp45_324-OMcp45_124*ORcp45_325-OMcp45_125*ORcp45_326-OMcp45_126*ORcp45_327-OMcp45_127*...
 ORcp45_377-OMcp45_16*ORcp45_321+OMcp45_322*ORcp45_123+OMcp45_323*ORcp45_124+OMcp45_324*ORcp45_125+OMcp45_325*ORcp45_126+OMcp45_326*ORcp45_127+...
 OMcp45_327*ORcp45_177+OMcp45_36*ORcp45_121-OPcp45_122*RLcp45_323-OPcp45_123*RLcp45_324-OPcp45_124*RLcp45_325-OPcp45_125*RLcp45_326-OPcp45_126*...
 RLcp45_327-OPcp45_127*RLcp45_377-OPcp45_16*RLcp45_321+OPcp45_322*RLcp45_123+OPcp45_323*RLcp45_124+OPcp45_324*RLcp45_125+OPcp45_325*RLcp45_126+...
 OPcp45_326*RLcp45_127+OPcp45_327*RLcp45_177+OPcp45_36*RLcp45_121;
    ACcp45_377 = qdd(3)+OMcp45_122*ORcp45_223+OMcp45_123*ORcp45_224+OMcp45_124*ORcp45_225+OMcp45_125*ORcp45_226+OMcp45_126*ORcp45_227+OMcp45_127*...
 ORcp45_277+OMcp45_16*ORcp45_221-OMcp45_222*ORcp45_123-OMcp45_223*ORcp45_124-OMcp45_224*ORcp45_125-OMcp45_225*ORcp45_126-OMcp45_226*ORcp45_127-...
 OMcp45_227*ORcp45_177-OMcp45_26*ORcp45_121+OPcp45_122*RLcp45_223+OPcp45_123*RLcp45_224+OPcp45_124*RLcp45_225+OPcp45_125*RLcp45_226+OPcp45_126*...
 RLcp45_227+OPcp45_127*RLcp45_277+OPcp45_16*RLcp45_221-OPcp45_222*RLcp45_123-OPcp45_223*RLcp45_124-OPcp45_224*RLcp45_125-OPcp45_225*RLcp45_126-...
 OPcp45_226*RLcp45_127-OPcp45_227*RLcp45_177-OPcp45_26*RLcp45_121;

% = = Block_1_0_0_46_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp45_177;
    sens.P(2) = POcp45_277;
    sens.P(3) = POcp45_377;
    sens.R(1,1) = ROcp45_127;
    sens.R(1,2) = ROcp45_227;
    sens.R(1,3) = ROcp45_327;
    sens.R(2,1) = ROcp45_426;
    sens.R(2,2) = ROcp45_526;
    sens.R(2,3) = ROcp45_626;
    sens.R(3,1) = ROcp45_727;
    sens.R(3,2) = ROcp45_827;
    sens.R(3,3) = ROcp45_927;
    sens.V(1) = VIcp45_177;
    sens.V(2) = VIcp45_277;
    sens.V(3) = VIcp45_377;
    sens.OM(1) = OMcp45_127;
    sens.OM(2) = OMcp45_227;
    sens.OM(3) = OMcp45_327;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp45_177_5;
    sens.J(1,6) = JTcp45_177_6;
    sens.J(1,21) = JTcp45_177_7;
    sens.J(1,22) = JTcp45_177_8;
    sens.J(1,23) = JTcp45_177_9;
    sens.J(1,24) = JTcp45_177_10;
    sens.J(1,25) = JTcp45_177_11;
    sens.J(1,26) = JTcp45_177_12;
    sens.J(1,27) = JTcp45_177_13;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp45_277_4;
    sens.J(2,5) = JTcp45_277_5;
    sens.J(2,6) = JTcp45_277_6;
    sens.J(2,21) = JTcp45_277_7;
    sens.J(2,22) = JTcp45_277_8;
    sens.J(2,23) = JTcp45_277_9;
    sens.J(2,24) = JTcp45_277_10;
    sens.J(2,25) = JTcp45_277_11;
    sens.J(2,26) = JTcp45_277_12;
    sens.J(2,27) = JTcp45_277_13;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp45_377_4;
    sens.J(3,5) = JTcp45_377_5;
    sens.J(3,6) = JTcp45_377_6;
    sens.J(3,21) = JTcp45_377_7;
    sens.J(3,22) = JTcp45_377_8;
    sens.J(3,23) = JTcp45_377_9;
    sens.J(3,24) = JTcp45_377_10;
    sens.J(3,25) = JTcp45_377_11;
    sens.J(3,26) = JTcp45_377_12;
    sens.J(3,27) = JTcp45_377_13;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,21) = ROcp45_16;
    sens.J(4,22) = ROcp45_421;
    sens.J(4,23) = ROcp45_722;
    sens.J(4,24) = ROcp45_423;
    sens.J(4,25) = ROcp45_124;
    sens.J(4,26) = ROcp45_725;
    sens.J(4,27) = ROcp45_426;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp45_85;
    sens.J(5,21) = ROcp45_26;
    sens.J(5,22) = ROcp45_521;
    sens.J(5,23) = ROcp45_822;
    sens.J(5,24) = ROcp45_523;
    sens.J(5,25) = ROcp45_224;
    sens.J(5,26) = ROcp45_825;
    sens.J(5,27) = ROcp45_526;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp45_95;
    sens.J(6,21) = ROcp45_36;
    sens.J(6,22) = ROcp45_621;
    sens.J(6,23) = ROcp45_922;
    sens.J(6,24) = ROcp45_623;
    sens.J(6,25) = ROcp45_324;
    sens.J(6,26) = ROcp45_925;
    sens.J(6,27) = ROcp45_626;
    sens.A(1) = ACcp45_177;
    sens.A(2) = ACcp45_277;
    sens.A(3) = ACcp45_377;
    sens.OMP(1) = OPcp45_127;
    sens.OMP(2) = OPcp45_227;
    sens.OMP(3) = OPcp45_327;
 
% 
case 47, 


% = = Block_1_0_0_47_0_1 = = 
 
% Sensor Kinematics 


    ROcp46_25 = S4*S5;
    ROcp46_35 = -C4*S5;
    ROcp46_85 = -S4*C5;
    ROcp46_95 = C4*C5;
    ROcp46_16 = C5*C6;
    ROcp46_26 = ROcp46_25*C6+C4*S6;
    ROcp46_36 = ROcp46_35*C6+S4*S6;
    ROcp46_46 = -C5*S6;
    ROcp46_56 = -(ROcp46_25*S6-C4*C6);
    ROcp46_66 = -(ROcp46_35*S6-S4*C6);
    OMcp46_25 = qd(5)*C4;
    OMcp46_35 = qd(5)*S4;
    OMcp46_16 = qd(4)+qd(6)*S5;
    OMcp46_26 = OMcp46_25+ROcp46_85*qd(6);
    OMcp46_36 = OMcp46_35+ROcp46_95*qd(6);
    OPcp46_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp46_26 = ROcp46_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp46_35*S5-ROcp46_95*qd(4));
    OPcp46_36 = ROcp46_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp46_25*S5-ROcp46_85*qd(4));

% = = Block_1_0_0_47_0_4 = = 
 
% Sensor Kinematics 


    ROcp46_421 = ROcp46_46*C21+S21*S5;
    ROcp46_521 = ROcp46_56*C21+ROcp46_85*S21;
    ROcp46_621 = ROcp46_66*C21+ROcp46_95*S21;
    ROcp46_721 = -(ROcp46_46*S21-C21*S5);
    ROcp46_821 = -(ROcp46_56*S21-ROcp46_85*C21);
    ROcp46_921 = -(ROcp46_66*S21-ROcp46_95*C21);
    ROcp46_122 = ROcp46_16*C22-ROcp46_721*S22;
    ROcp46_222 = ROcp46_26*C22-ROcp46_821*S22;
    ROcp46_322 = ROcp46_36*C22-ROcp46_921*S22;
    ROcp46_722 = ROcp46_16*S22+ROcp46_721*C22;
    ROcp46_822 = ROcp46_26*S22+ROcp46_821*C22;
    ROcp46_922 = ROcp46_36*S22+ROcp46_921*C22;
    ROcp46_123 = ROcp46_122*C23+ROcp46_421*S23;
    ROcp46_223 = ROcp46_222*C23+ROcp46_521*S23;
    ROcp46_323 = ROcp46_322*C23+ROcp46_621*S23;
    ROcp46_423 = -(ROcp46_122*S23-ROcp46_421*C23);
    ROcp46_523 = -(ROcp46_222*S23-ROcp46_521*C23);
    ROcp46_623 = -(ROcp46_322*S23-ROcp46_621*C23);
    RLcp46_121 = ROcp46_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp46_221 = ROcp46_26*s.dpt(1,3)+ROcp46_85*s.dpt(3,3);
    RLcp46_321 = ROcp46_36*s.dpt(1,3)+ROcp46_95*s.dpt(3,3);
    OMcp46_121 = OMcp46_16+ROcp46_16*qd(21);
    OMcp46_221 = OMcp46_26+ROcp46_26*qd(21);
    OMcp46_321 = OMcp46_36+ROcp46_36*qd(21);
    ORcp46_121 = OMcp46_26*RLcp46_321-OMcp46_36*RLcp46_221;
    ORcp46_221 = -(OMcp46_16*RLcp46_321-OMcp46_36*RLcp46_121);
    ORcp46_321 = OMcp46_16*RLcp46_221-OMcp46_26*RLcp46_121;
    OMcp46_122 = OMcp46_121+ROcp46_421*qd(22);
    OMcp46_222 = OMcp46_221+ROcp46_521*qd(22);
    OMcp46_322 = OMcp46_321+ROcp46_621*qd(22);
    OPcp46_122 = OPcp46_16+ROcp46_16*qdd(21)+ROcp46_421*qdd(22)+qd(21)*(OMcp46_26*ROcp46_36-OMcp46_36*ROcp46_26)+qd(22)*(OMcp46_221*ROcp46_621-...
 OMcp46_321*ROcp46_521);
    OPcp46_222 = OPcp46_26+ROcp46_26*qdd(21)+ROcp46_521*qdd(22)-qd(21)*(OMcp46_16*ROcp46_36-OMcp46_36*ROcp46_16)-qd(22)*(OMcp46_121*ROcp46_621-...
 OMcp46_321*ROcp46_421);
    OPcp46_322 = OPcp46_36+ROcp46_36*qdd(21)+ROcp46_621*qdd(22)+qd(21)*(OMcp46_16*ROcp46_26-OMcp46_26*ROcp46_16)+qd(22)*(OMcp46_121*ROcp46_521-...
 OMcp46_221*ROcp46_421);
    RLcp46_123 = ROcp46_722*s.dpt(3,37);
    RLcp46_223 = ROcp46_822*s.dpt(3,37);
    RLcp46_323 = ROcp46_922*s.dpt(3,37);
    OMcp46_123 = OMcp46_122+ROcp46_722*qd(23);
    OMcp46_223 = OMcp46_222+ROcp46_822*qd(23);
    OMcp46_323 = OMcp46_322+ROcp46_922*qd(23);
    ORcp46_123 = OMcp46_222*RLcp46_323-OMcp46_322*RLcp46_223;
    ORcp46_223 = -(OMcp46_122*RLcp46_323-OMcp46_322*RLcp46_123);
    ORcp46_323 = OMcp46_122*RLcp46_223-OMcp46_222*RLcp46_123;
    OPcp46_123 = OPcp46_122+ROcp46_722*qdd(23)+qd(23)*(OMcp46_222*ROcp46_922-OMcp46_322*ROcp46_822);
    OPcp46_223 = OPcp46_222+ROcp46_822*qdd(23)-qd(23)*(OMcp46_122*ROcp46_922-OMcp46_322*ROcp46_722);
    OPcp46_323 = OPcp46_322+ROcp46_922*qdd(23)+qd(23)*(OMcp46_122*ROcp46_822-OMcp46_222*ROcp46_722);

% = = Block_1_0_0_47_0_6 = = 
 
% Sensor Kinematics 


    ROcp46_128 = ROcp46_123*C28-ROcp46_722*S28;
    ROcp46_228 = ROcp46_223*C28-ROcp46_822*S28;
    ROcp46_328 = ROcp46_323*C28-ROcp46_922*S28;
    ROcp46_728 = ROcp46_123*S28+ROcp46_722*C28;
    ROcp46_828 = ROcp46_223*S28+ROcp46_822*C28;
    ROcp46_928 = ROcp46_323*S28+ROcp46_922*C28;
    RLcp46_128 = ROcp46_123*s.dpt(1,42)+ROcp46_423*s.dpt(2,42)+ROcp46_722*s.dpt(3,42);
    RLcp46_228 = ROcp46_223*s.dpt(1,42)+ROcp46_523*s.dpt(2,42)+ROcp46_822*s.dpt(3,42);
    RLcp46_328 = ROcp46_323*s.dpt(1,42)+ROcp46_623*s.dpt(2,42)+ROcp46_922*s.dpt(3,42);
    OMcp46_128 = OMcp46_123+ROcp46_423*qd(28);
    OMcp46_228 = OMcp46_223+ROcp46_523*qd(28);
    OMcp46_328 = OMcp46_323+ROcp46_623*qd(28);
    ORcp46_128 = OMcp46_223*RLcp46_328-OMcp46_323*RLcp46_228;
    ORcp46_228 = -(OMcp46_123*RLcp46_328-OMcp46_323*RLcp46_128);
    ORcp46_328 = OMcp46_123*RLcp46_228-OMcp46_223*RLcp46_128;
    OPcp46_128 = OPcp46_123+ROcp46_423*qdd(28)+qd(28)*(OMcp46_223*ROcp46_623-OMcp46_323*ROcp46_523);
    OPcp46_228 = OPcp46_223+ROcp46_523*qdd(28)-qd(28)*(OMcp46_123*ROcp46_623-OMcp46_323*ROcp46_423);
    OPcp46_328 = OPcp46_323+ROcp46_623*qdd(28)+qd(28)*(OMcp46_123*ROcp46_523-OMcp46_223*ROcp46_423);
    RLcp46_178 = ROcp46_128*s.dpt(1,51)+ROcp46_423*s.dpt(2,51)+ROcp46_728*s.dpt(3,51);
    RLcp46_278 = ROcp46_228*s.dpt(1,51)+ROcp46_523*s.dpt(2,51)+ROcp46_828*s.dpt(3,51);
    RLcp46_378 = ROcp46_328*s.dpt(1,51)+ROcp46_623*s.dpt(2,51)+ROcp46_928*s.dpt(3,51);
    POcp46_178 = RLcp46_121+RLcp46_123+RLcp46_128+RLcp46_178+q(1);
    POcp46_278 = RLcp46_221+RLcp46_223+RLcp46_228+RLcp46_278+q(2);
    POcp46_378 = RLcp46_321+RLcp46_323+RLcp46_328+RLcp46_378+q(3);
    JTcp46_278_4 = -(RLcp46_321+RLcp46_323+RLcp46_328+RLcp46_378);
    JTcp46_378_4 = RLcp46_221+RLcp46_223+RLcp46_228+RLcp46_278;
    JTcp46_178_5 = C4*(RLcp46_321+RLcp46_323+RLcp46_328+RLcp46_378)-S4*(RLcp46_221+RLcp46_223)-S4*(RLcp46_228+RLcp46_278);
    JTcp46_278_5 = S4*(RLcp46_121+RLcp46_123+RLcp46_128+RLcp46_178);
    JTcp46_378_5 = -C4*(RLcp46_121+RLcp46_123+RLcp46_128+RLcp46_178);
    JTcp46_178_6 = ROcp46_85*(RLcp46_321+RLcp46_323+RLcp46_328+RLcp46_378)-ROcp46_95*(RLcp46_221+RLcp46_223)-ROcp46_95*(RLcp46_228+RLcp46_278);
    JTcp46_278_6 = RLcp46_178*ROcp46_95-RLcp46_328*S5-RLcp46_378*S5+ROcp46_95*(RLcp46_121+RLcp46_123+RLcp46_128)-S5*(RLcp46_321+RLcp46_323);
    JTcp46_378_6 = RLcp46_228*S5-ROcp46_85*(RLcp46_121+RLcp46_123+RLcp46_128)+S5*(RLcp46_221+RLcp46_223)-RLcp46_178*ROcp46_85+RLcp46_278*S5;
    JTcp46_178_7 = ROcp46_26*(RLcp46_323+RLcp46_328)-ROcp46_36*(RLcp46_223+RLcp46_228)-RLcp46_278*ROcp46_36+RLcp46_378*ROcp46_26;
    JTcp46_278_7 = RLcp46_178*ROcp46_36-RLcp46_378*ROcp46_16-ROcp46_16*(RLcp46_323+RLcp46_328)+ROcp46_36*(RLcp46_123+RLcp46_128);
    JTcp46_378_7 = ROcp46_16*(RLcp46_223+RLcp46_228)-ROcp46_26*(RLcp46_123+RLcp46_128)-RLcp46_178*ROcp46_26+RLcp46_278*ROcp46_16;
    JTcp46_178_8 = ROcp46_521*(RLcp46_323+RLcp46_328)-ROcp46_621*(RLcp46_223+RLcp46_228)-RLcp46_278*ROcp46_621+RLcp46_378*ROcp46_521;
    JTcp46_278_8 = RLcp46_178*ROcp46_621-RLcp46_378*ROcp46_421-ROcp46_421*(RLcp46_323+RLcp46_328)+ROcp46_621*(RLcp46_123+RLcp46_128);
    JTcp46_378_8 = ROcp46_421*(RLcp46_223+RLcp46_228)-ROcp46_521*(RLcp46_123+RLcp46_128)-RLcp46_178*ROcp46_521+RLcp46_278*ROcp46_421;
    JTcp46_178_9 = ROcp46_822*(RLcp46_328+RLcp46_378)-ROcp46_922*(RLcp46_228+RLcp46_278);
    JTcp46_278_9 = -(ROcp46_722*(RLcp46_328+RLcp46_378)-ROcp46_922*(RLcp46_128+RLcp46_178));
    JTcp46_378_9 = ROcp46_722*(RLcp46_228+RLcp46_278)-ROcp46_822*(RLcp46_128+RLcp46_178);
    JTcp46_178_10 = -(RLcp46_278*ROcp46_623-RLcp46_378*ROcp46_523);
    JTcp46_278_10 = RLcp46_178*ROcp46_623-RLcp46_378*ROcp46_423;
    JTcp46_378_10 = -(RLcp46_178*ROcp46_523-RLcp46_278*ROcp46_423);
    ORcp46_178 = OMcp46_228*RLcp46_378-OMcp46_328*RLcp46_278;
    ORcp46_278 = -(OMcp46_128*RLcp46_378-OMcp46_328*RLcp46_178);
    ORcp46_378 = OMcp46_128*RLcp46_278-OMcp46_228*RLcp46_178;
    VIcp46_178 = ORcp46_121+ORcp46_123+ORcp46_128+ORcp46_178+qd(1);
    VIcp46_278 = ORcp46_221+ORcp46_223+ORcp46_228+ORcp46_278+qd(2);
    VIcp46_378 = ORcp46_321+ORcp46_323+ORcp46_328+ORcp46_378+qd(3);
    ACcp46_178 = qdd(1)+OMcp46_222*ORcp46_323+OMcp46_223*ORcp46_328+OMcp46_228*ORcp46_378+OMcp46_26*ORcp46_321-OMcp46_322*ORcp46_223-OMcp46_323*...
 ORcp46_228-OMcp46_328*ORcp46_278-OMcp46_36*ORcp46_221+OPcp46_222*RLcp46_323+OPcp46_223*RLcp46_328+OPcp46_228*RLcp46_378+OPcp46_26*RLcp46_321-...
 OPcp46_322*RLcp46_223-OPcp46_323*RLcp46_228-OPcp46_328*RLcp46_278-OPcp46_36*RLcp46_221;
    ACcp46_278 = qdd(2)-OMcp46_122*ORcp46_323-OMcp46_123*ORcp46_328-OMcp46_128*ORcp46_378-OMcp46_16*ORcp46_321+OMcp46_322*ORcp46_123+OMcp46_323*...
 ORcp46_128+OMcp46_328*ORcp46_178+OMcp46_36*ORcp46_121-OPcp46_122*RLcp46_323-OPcp46_123*RLcp46_328-OPcp46_128*RLcp46_378-OPcp46_16*RLcp46_321+...
 OPcp46_322*RLcp46_123+OPcp46_323*RLcp46_128+OPcp46_328*RLcp46_178+OPcp46_36*RLcp46_121;
    ACcp46_378 = qdd(3)+OMcp46_122*ORcp46_223+OMcp46_123*ORcp46_228+OMcp46_128*ORcp46_278+OMcp46_16*ORcp46_221-OMcp46_222*ORcp46_123-OMcp46_223*...
 ORcp46_128-OMcp46_228*ORcp46_178-OMcp46_26*ORcp46_121+OPcp46_122*RLcp46_223+OPcp46_123*RLcp46_228+OPcp46_128*RLcp46_278+OPcp46_16*RLcp46_221-...
 OPcp46_222*RLcp46_123-OPcp46_223*RLcp46_128-OPcp46_228*RLcp46_178-OPcp46_26*RLcp46_121;

% = = Block_1_0_0_47_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp46_178;
    sens.P(2) = POcp46_278;
    sens.P(3) = POcp46_378;
    sens.R(1,1) = ROcp46_128;
    sens.R(1,2) = ROcp46_228;
    sens.R(1,3) = ROcp46_328;
    sens.R(2,1) = ROcp46_423;
    sens.R(2,2) = ROcp46_523;
    sens.R(2,3) = ROcp46_623;
    sens.R(3,1) = ROcp46_728;
    sens.R(3,2) = ROcp46_828;
    sens.R(3,3) = ROcp46_928;
    sens.V(1) = VIcp46_178;
    sens.V(2) = VIcp46_278;
    sens.V(3) = VIcp46_378;
    sens.OM(1) = OMcp46_128;
    sens.OM(2) = OMcp46_228;
    sens.OM(3) = OMcp46_328;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp46_178_5;
    sens.J(1,6) = JTcp46_178_6;
    sens.J(1,21) = JTcp46_178_7;
    sens.J(1,22) = JTcp46_178_8;
    sens.J(1,23) = JTcp46_178_9;
    sens.J(1,28) = JTcp46_178_10;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp46_278_4;
    sens.J(2,5) = JTcp46_278_5;
    sens.J(2,6) = JTcp46_278_6;
    sens.J(2,21) = JTcp46_278_7;
    sens.J(2,22) = JTcp46_278_8;
    sens.J(2,23) = JTcp46_278_9;
    sens.J(2,28) = JTcp46_278_10;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp46_378_4;
    sens.J(3,5) = JTcp46_378_5;
    sens.J(3,6) = JTcp46_378_6;
    sens.J(3,21) = JTcp46_378_7;
    sens.J(3,22) = JTcp46_378_8;
    sens.J(3,23) = JTcp46_378_9;
    sens.J(3,28) = JTcp46_378_10;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,21) = ROcp46_16;
    sens.J(4,22) = ROcp46_421;
    sens.J(4,23) = ROcp46_722;
    sens.J(4,28) = ROcp46_423;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp46_85;
    sens.J(5,21) = ROcp46_26;
    sens.J(5,22) = ROcp46_521;
    sens.J(5,23) = ROcp46_822;
    sens.J(5,28) = ROcp46_523;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp46_95;
    sens.J(6,21) = ROcp46_36;
    sens.J(6,22) = ROcp46_621;
    sens.J(6,23) = ROcp46_922;
    sens.J(6,28) = ROcp46_623;
    sens.A(1) = ACcp46_178;
    sens.A(2) = ACcp46_278;
    sens.A(3) = ACcp46_378;
    sens.OMP(1) = OPcp46_128;
    sens.OMP(2) = OPcp46_228;
    sens.OMP(3) = OPcp46_328;
 
% 
case 48, 


% = = Block_1_0_0_48_0_1 = = 
 
% Sensor Kinematics 


    ROcp47_25 = S4*S5;
    ROcp47_35 = -C4*S5;
    ROcp47_85 = -S4*C5;
    ROcp47_95 = C4*C5;
    ROcp47_16 = C5*C6;
    ROcp47_26 = ROcp47_25*C6+C4*S6;
    ROcp47_36 = ROcp47_35*C6+S4*S6;
    ROcp47_46 = -C5*S6;
    ROcp47_56 = -(ROcp47_25*S6-C4*C6);
    ROcp47_66 = -(ROcp47_35*S6-S4*C6);
    OMcp47_25 = qd(5)*C4;
    OMcp47_35 = qd(5)*S4;
    OMcp47_16 = qd(4)+qd(6)*S5;
    OMcp47_26 = OMcp47_25+ROcp47_85*qd(6);
    OMcp47_36 = OMcp47_35+ROcp47_95*qd(6);
    OPcp47_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp47_26 = ROcp47_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp47_35*S5-ROcp47_95*qd(4));
    OPcp47_36 = ROcp47_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp47_25*S5-ROcp47_85*qd(4));

% = = Block_1_0_0_48_0_4 = = 
 
% Sensor Kinematics 


    ROcp47_421 = ROcp47_46*C21+S21*S5;
    ROcp47_521 = ROcp47_56*C21+ROcp47_85*S21;
    ROcp47_621 = ROcp47_66*C21+ROcp47_95*S21;
    ROcp47_721 = -(ROcp47_46*S21-C21*S5);
    ROcp47_821 = -(ROcp47_56*S21-ROcp47_85*C21);
    ROcp47_921 = -(ROcp47_66*S21-ROcp47_95*C21);
    ROcp47_122 = ROcp47_16*C22-ROcp47_721*S22;
    ROcp47_222 = ROcp47_26*C22-ROcp47_821*S22;
    ROcp47_322 = ROcp47_36*C22-ROcp47_921*S22;
    ROcp47_722 = ROcp47_16*S22+ROcp47_721*C22;
    ROcp47_822 = ROcp47_26*S22+ROcp47_821*C22;
    ROcp47_922 = ROcp47_36*S22+ROcp47_921*C22;
    ROcp47_123 = ROcp47_122*C23+ROcp47_421*S23;
    ROcp47_223 = ROcp47_222*C23+ROcp47_521*S23;
    ROcp47_323 = ROcp47_322*C23+ROcp47_621*S23;
    ROcp47_423 = -(ROcp47_122*S23-ROcp47_421*C23);
    ROcp47_523 = -(ROcp47_222*S23-ROcp47_521*C23);
    ROcp47_623 = -(ROcp47_322*S23-ROcp47_621*C23);
    RLcp47_121 = ROcp47_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp47_221 = ROcp47_26*s.dpt(1,3)+ROcp47_85*s.dpt(3,3);
    RLcp47_321 = ROcp47_36*s.dpt(1,3)+ROcp47_95*s.dpt(3,3);
    OMcp47_121 = OMcp47_16+ROcp47_16*qd(21);
    OMcp47_221 = OMcp47_26+ROcp47_26*qd(21);
    OMcp47_321 = OMcp47_36+ROcp47_36*qd(21);
    ORcp47_121 = OMcp47_26*RLcp47_321-OMcp47_36*RLcp47_221;
    ORcp47_221 = -(OMcp47_16*RLcp47_321-OMcp47_36*RLcp47_121);
    ORcp47_321 = OMcp47_16*RLcp47_221-OMcp47_26*RLcp47_121;
    OMcp47_122 = OMcp47_121+ROcp47_421*qd(22);
    OMcp47_222 = OMcp47_221+ROcp47_521*qd(22);
    OMcp47_322 = OMcp47_321+ROcp47_621*qd(22);
    OPcp47_122 = OPcp47_16+ROcp47_16*qdd(21)+ROcp47_421*qdd(22)+qd(21)*(OMcp47_26*ROcp47_36-OMcp47_36*ROcp47_26)+qd(22)*(OMcp47_221*ROcp47_621-...
 OMcp47_321*ROcp47_521);
    OPcp47_222 = OPcp47_26+ROcp47_26*qdd(21)+ROcp47_521*qdd(22)-qd(21)*(OMcp47_16*ROcp47_36-OMcp47_36*ROcp47_16)-qd(22)*(OMcp47_121*ROcp47_621-...
 OMcp47_321*ROcp47_421);
    OPcp47_322 = OPcp47_36+ROcp47_36*qdd(21)+ROcp47_621*qdd(22)+qd(21)*(OMcp47_16*ROcp47_26-OMcp47_26*ROcp47_16)+qd(22)*(OMcp47_121*ROcp47_521-...
 OMcp47_221*ROcp47_421);
    RLcp47_123 = ROcp47_722*s.dpt(3,37);
    RLcp47_223 = ROcp47_822*s.dpt(3,37);
    RLcp47_323 = ROcp47_922*s.dpt(3,37);
    OMcp47_123 = OMcp47_122+ROcp47_722*qd(23);
    OMcp47_223 = OMcp47_222+ROcp47_822*qd(23);
    OMcp47_323 = OMcp47_322+ROcp47_922*qd(23);
    ORcp47_123 = OMcp47_222*RLcp47_323-OMcp47_322*RLcp47_223;
    ORcp47_223 = -(OMcp47_122*RLcp47_323-OMcp47_322*RLcp47_123);
    ORcp47_323 = OMcp47_122*RLcp47_223-OMcp47_222*RLcp47_123;
    OPcp47_123 = OPcp47_122+ROcp47_722*qdd(23)+qd(23)*(OMcp47_222*ROcp47_922-OMcp47_322*ROcp47_822);
    OPcp47_223 = OPcp47_222+ROcp47_822*qdd(23)-qd(23)*(OMcp47_122*ROcp47_922-OMcp47_322*ROcp47_722);
    OPcp47_323 = OPcp47_322+ROcp47_922*qdd(23)+qd(23)*(OMcp47_122*ROcp47_822-OMcp47_222*ROcp47_722);

% = = Block_1_0_0_48_0_6 = = 
 
% Sensor Kinematics 


    ROcp47_128 = ROcp47_123*C28-ROcp47_722*S28;
    ROcp47_228 = ROcp47_223*C28-ROcp47_822*S28;
    ROcp47_328 = ROcp47_323*C28-ROcp47_922*S28;
    ROcp47_728 = ROcp47_123*S28+ROcp47_722*C28;
    ROcp47_828 = ROcp47_223*S28+ROcp47_822*C28;
    ROcp47_928 = ROcp47_323*S28+ROcp47_922*C28;
    RLcp47_128 = ROcp47_123*s.dpt(1,42)+ROcp47_423*s.dpt(2,42)+ROcp47_722*s.dpt(3,42);
    RLcp47_228 = ROcp47_223*s.dpt(1,42)+ROcp47_523*s.dpt(2,42)+ROcp47_822*s.dpt(3,42);
    RLcp47_328 = ROcp47_323*s.dpt(1,42)+ROcp47_623*s.dpt(2,42)+ROcp47_922*s.dpt(3,42);
    OMcp47_128 = OMcp47_123+ROcp47_423*qd(28);
    OMcp47_228 = OMcp47_223+ROcp47_523*qd(28);
    OMcp47_328 = OMcp47_323+ROcp47_623*qd(28);
    ORcp47_128 = OMcp47_223*RLcp47_328-OMcp47_323*RLcp47_228;
    ORcp47_228 = -(OMcp47_123*RLcp47_328-OMcp47_323*RLcp47_128);
    ORcp47_328 = OMcp47_123*RLcp47_228-OMcp47_223*RLcp47_128;
    OPcp47_128 = OPcp47_123+ROcp47_423*qdd(28)+qd(28)*(OMcp47_223*ROcp47_623-OMcp47_323*ROcp47_523);
    OPcp47_228 = OPcp47_223+ROcp47_523*qdd(28)-qd(28)*(OMcp47_123*ROcp47_623-OMcp47_323*ROcp47_423);
    OPcp47_328 = OPcp47_323+ROcp47_623*qdd(28)+qd(28)*(OMcp47_123*ROcp47_523-OMcp47_223*ROcp47_423);
    RLcp47_179 = ROcp47_423*s.dpt(2,52);
    RLcp47_279 = ROcp47_523*s.dpt(2,52);
    RLcp47_379 = ROcp47_623*s.dpt(2,52);
    POcp47_179 = RLcp47_121+RLcp47_123+RLcp47_128+RLcp47_179+q(1);
    POcp47_279 = RLcp47_221+RLcp47_223+RLcp47_228+RLcp47_279+q(2);
    POcp47_379 = RLcp47_321+RLcp47_323+RLcp47_328+RLcp47_379+q(3);
    JTcp47_279_4 = -(RLcp47_321+RLcp47_323+RLcp47_328+RLcp47_379);
    JTcp47_379_4 = RLcp47_221+RLcp47_223+RLcp47_228+RLcp47_279;
    JTcp47_179_5 = C4*(RLcp47_321+RLcp47_323+RLcp47_328+RLcp47_379)-S4*(RLcp47_221+RLcp47_223)-S4*(RLcp47_228+RLcp47_279);
    JTcp47_279_5 = S4*(RLcp47_121+RLcp47_123+RLcp47_128+RLcp47_179);
    JTcp47_379_5 = -C4*(RLcp47_121+RLcp47_123+RLcp47_128+RLcp47_179);
    JTcp47_179_6 = ROcp47_85*(RLcp47_321+RLcp47_323+RLcp47_328+RLcp47_379)-ROcp47_95*(RLcp47_221+RLcp47_223)-ROcp47_95*(RLcp47_228+RLcp47_279);
    JTcp47_279_6 = RLcp47_179*ROcp47_95-RLcp47_328*S5-RLcp47_379*S5+ROcp47_95*(RLcp47_121+RLcp47_123+RLcp47_128)-S5*(RLcp47_321+RLcp47_323);
    JTcp47_379_6 = RLcp47_228*S5-ROcp47_85*(RLcp47_121+RLcp47_123+RLcp47_128)+S5*(RLcp47_221+RLcp47_223)-RLcp47_179*ROcp47_85+RLcp47_279*S5;
    JTcp47_179_7 = ROcp47_26*(RLcp47_323+RLcp47_328)-ROcp47_36*(RLcp47_223+RLcp47_228)-RLcp47_279*ROcp47_36+RLcp47_379*ROcp47_26;
    JTcp47_279_7 = RLcp47_179*ROcp47_36-RLcp47_379*ROcp47_16-ROcp47_16*(RLcp47_323+RLcp47_328)+ROcp47_36*(RLcp47_123+RLcp47_128);
    JTcp47_379_7 = ROcp47_16*(RLcp47_223+RLcp47_228)-ROcp47_26*(RLcp47_123+RLcp47_128)-RLcp47_179*ROcp47_26+RLcp47_279*ROcp47_16;
    JTcp47_179_8 = ROcp47_521*(RLcp47_323+RLcp47_328)-ROcp47_621*(RLcp47_223+RLcp47_228)-RLcp47_279*ROcp47_621+RLcp47_379*ROcp47_521;
    JTcp47_279_8 = RLcp47_179*ROcp47_621-RLcp47_379*ROcp47_421-ROcp47_421*(RLcp47_323+RLcp47_328)+ROcp47_621*(RLcp47_123+RLcp47_128);
    JTcp47_379_8 = ROcp47_421*(RLcp47_223+RLcp47_228)-ROcp47_521*(RLcp47_123+RLcp47_128)-RLcp47_179*ROcp47_521+RLcp47_279*ROcp47_421;
    JTcp47_179_9 = ROcp47_822*(RLcp47_328+RLcp47_379)-ROcp47_922*(RLcp47_228+RLcp47_279);
    JTcp47_279_9 = -(ROcp47_722*(RLcp47_328+RLcp47_379)-ROcp47_922*(RLcp47_128+RLcp47_179));
    JTcp47_379_9 = ROcp47_722*(RLcp47_228+RLcp47_279)-ROcp47_822*(RLcp47_128+RLcp47_179);
    JTcp47_179_10 = -(RLcp47_279*ROcp47_623-RLcp47_379*ROcp47_523);
    JTcp47_279_10 = RLcp47_179*ROcp47_623-RLcp47_379*ROcp47_423;
    JTcp47_379_10 = -(RLcp47_179*ROcp47_523-RLcp47_279*ROcp47_423);
    ORcp47_179 = OMcp47_228*RLcp47_379-OMcp47_328*RLcp47_279;
    ORcp47_279 = -(OMcp47_128*RLcp47_379-OMcp47_328*RLcp47_179);
    ORcp47_379 = OMcp47_128*RLcp47_279-OMcp47_228*RLcp47_179;
    VIcp47_179 = ORcp47_121+ORcp47_123+ORcp47_128+ORcp47_179+qd(1);
    VIcp47_279 = ORcp47_221+ORcp47_223+ORcp47_228+ORcp47_279+qd(2);
    VIcp47_379 = ORcp47_321+ORcp47_323+ORcp47_328+ORcp47_379+qd(3);
    ACcp47_179 = qdd(1)+OMcp47_222*ORcp47_323+OMcp47_223*ORcp47_328+OMcp47_228*ORcp47_379+OMcp47_26*ORcp47_321-OMcp47_322*ORcp47_223-OMcp47_323*...
 ORcp47_228-OMcp47_328*ORcp47_279-OMcp47_36*ORcp47_221+OPcp47_222*RLcp47_323+OPcp47_223*RLcp47_328+OPcp47_228*RLcp47_379+OPcp47_26*RLcp47_321-...
 OPcp47_322*RLcp47_223-OPcp47_323*RLcp47_228-OPcp47_328*RLcp47_279-OPcp47_36*RLcp47_221;
    ACcp47_279 = qdd(2)-OMcp47_122*ORcp47_323-OMcp47_123*ORcp47_328-OMcp47_128*ORcp47_379-OMcp47_16*ORcp47_321+OMcp47_322*ORcp47_123+OMcp47_323*...
 ORcp47_128+OMcp47_328*ORcp47_179+OMcp47_36*ORcp47_121-OPcp47_122*RLcp47_323-OPcp47_123*RLcp47_328-OPcp47_128*RLcp47_379-OPcp47_16*RLcp47_321+...
 OPcp47_322*RLcp47_123+OPcp47_323*RLcp47_128+OPcp47_328*RLcp47_179+OPcp47_36*RLcp47_121;
    ACcp47_379 = qdd(3)+OMcp47_122*ORcp47_223+OMcp47_123*ORcp47_228+OMcp47_128*ORcp47_279+OMcp47_16*ORcp47_221-OMcp47_222*ORcp47_123-OMcp47_223*...
 ORcp47_128-OMcp47_228*ORcp47_179-OMcp47_26*ORcp47_121+OPcp47_122*RLcp47_223+OPcp47_123*RLcp47_228+OPcp47_128*RLcp47_279+OPcp47_16*RLcp47_221-...
 OPcp47_222*RLcp47_123-OPcp47_223*RLcp47_128-OPcp47_228*RLcp47_179-OPcp47_26*RLcp47_121;

% = = Block_1_0_0_48_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp47_179;
    sens.P(2) = POcp47_279;
    sens.P(3) = POcp47_379;
    sens.R(1,1) = ROcp47_128;
    sens.R(1,2) = ROcp47_228;
    sens.R(1,3) = ROcp47_328;
    sens.R(2,1) = ROcp47_423;
    sens.R(2,2) = ROcp47_523;
    sens.R(2,3) = ROcp47_623;
    sens.R(3,1) = ROcp47_728;
    sens.R(3,2) = ROcp47_828;
    sens.R(3,3) = ROcp47_928;
    sens.V(1) = VIcp47_179;
    sens.V(2) = VIcp47_279;
    sens.V(3) = VIcp47_379;
    sens.OM(1) = OMcp47_128;
    sens.OM(2) = OMcp47_228;
    sens.OM(3) = OMcp47_328;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp47_179_5;
    sens.J(1,6) = JTcp47_179_6;
    sens.J(1,21) = JTcp47_179_7;
    sens.J(1,22) = JTcp47_179_8;
    sens.J(1,23) = JTcp47_179_9;
    sens.J(1,28) = JTcp47_179_10;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp47_279_4;
    sens.J(2,5) = JTcp47_279_5;
    sens.J(2,6) = JTcp47_279_6;
    sens.J(2,21) = JTcp47_279_7;
    sens.J(2,22) = JTcp47_279_8;
    sens.J(2,23) = JTcp47_279_9;
    sens.J(2,28) = JTcp47_279_10;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp47_379_4;
    sens.J(3,5) = JTcp47_379_5;
    sens.J(3,6) = JTcp47_379_6;
    sens.J(3,21) = JTcp47_379_7;
    sens.J(3,22) = JTcp47_379_8;
    sens.J(3,23) = JTcp47_379_9;
    sens.J(3,28) = JTcp47_379_10;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,21) = ROcp47_16;
    sens.J(4,22) = ROcp47_421;
    sens.J(4,23) = ROcp47_722;
    sens.J(4,28) = ROcp47_423;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp47_85;
    sens.J(5,21) = ROcp47_26;
    sens.J(5,22) = ROcp47_521;
    sens.J(5,23) = ROcp47_822;
    sens.J(5,28) = ROcp47_523;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp47_95;
    sens.J(6,21) = ROcp47_36;
    sens.J(6,22) = ROcp47_621;
    sens.J(6,23) = ROcp47_922;
    sens.J(6,28) = ROcp47_623;
    sens.A(1) = ACcp47_179;
    sens.A(2) = ACcp47_279;
    sens.A(3) = ACcp47_379;
    sens.OMP(1) = OPcp47_128;
    sens.OMP(2) = OPcp47_228;
    sens.OMP(3) = OPcp47_328;
 
% 
case 49, 


% = = Block_1_0_0_49_0_1 = = 
 
% Sensor Kinematics 


    ROcp48_25 = S4*S5;
    ROcp48_35 = -C4*S5;
    ROcp48_85 = -S4*C5;
    ROcp48_95 = C4*C5;
    ROcp48_16 = C5*C6;
    ROcp48_26 = ROcp48_25*C6+C4*S6;
    ROcp48_36 = ROcp48_35*C6+S4*S6;
    ROcp48_46 = -C5*S6;
    ROcp48_56 = -(ROcp48_25*S6-C4*C6);
    ROcp48_66 = -(ROcp48_35*S6-S4*C6);
    OMcp48_25 = qd(5)*C4;
    OMcp48_35 = qd(5)*S4;
    OMcp48_16 = qd(4)+qd(6)*S5;
    OMcp48_26 = OMcp48_25+ROcp48_85*qd(6);
    OMcp48_36 = OMcp48_35+ROcp48_95*qd(6);
    OPcp48_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp48_26 = ROcp48_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp48_35*S5-ROcp48_95*qd(4));
    OPcp48_36 = ROcp48_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp48_25*S5-ROcp48_85*qd(4));

% = = Block_1_0_0_49_0_4 = = 
 
% Sensor Kinematics 


    ROcp48_421 = ROcp48_46*C21+S21*S5;
    ROcp48_521 = ROcp48_56*C21+ROcp48_85*S21;
    ROcp48_621 = ROcp48_66*C21+ROcp48_95*S21;
    ROcp48_721 = -(ROcp48_46*S21-C21*S5);
    ROcp48_821 = -(ROcp48_56*S21-ROcp48_85*C21);
    ROcp48_921 = -(ROcp48_66*S21-ROcp48_95*C21);
    ROcp48_122 = ROcp48_16*C22-ROcp48_721*S22;
    ROcp48_222 = ROcp48_26*C22-ROcp48_821*S22;
    ROcp48_322 = ROcp48_36*C22-ROcp48_921*S22;
    ROcp48_722 = ROcp48_16*S22+ROcp48_721*C22;
    ROcp48_822 = ROcp48_26*S22+ROcp48_821*C22;
    ROcp48_922 = ROcp48_36*S22+ROcp48_921*C22;
    ROcp48_123 = ROcp48_122*C23+ROcp48_421*S23;
    ROcp48_223 = ROcp48_222*C23+ROcp48_521*S23;
    ROcp48_323 = ROcp48_322*C23+ROcp48_621*S23;
    ROcp48_423 = -(ROcp48_122*S23-ROcp48_421*C23);
    ROcp48_523 = -(ROcp48_222*S23-ROcp48_521*C23);
    ROcp48_623 = -(ROcp48_322*S23-ROcp48_621*C23);
    RLcp48_121 = ROcp48_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp48_221 = ROcp48_26*s.dpt(1,3)+ROcp48_85*s.dpt(3,3);
    RLcp48_321 = ROcp48_36*s.dpt(1,3)+ROcp48_95*s.dpt(3,3);
    OMcp48_121 = OMcp48_16+ROcp48_16*qd(21);
    OMcp48_221 = OMcp48_26+ROcp48_26*qd(21);
    OMcp48_321 = OMcp48_36+ROcp48_36*qd(21);
    ORcp48_121 = OMcp48_26*RLcp48_321-OMcp48_36*RLcp48_221;
    ORcp48_221 = -(OMcp48_16*RLcp48_321-OMcp48_36*RLcp48_121);
    ORcp48_321 = OMcp48_16*RLcp48_221-OMcp48_26*RLcp48_121;
    OMcp48_122 = OMcp48_121+ROcp48_421*qd(22);
    OMcp48_222 = OMcp48_221+ROcp48_521*qd(22);
    OMcp48_322 = OMcp48_321+ROcp48_621*qd(22);
    OPcp48_122 = OPcp48_16+ROcp48_16*qdd(21)+ROcp48_421*qdd(22)+qd(21)*(OMcp48_26*ROcp48_36-OMcp48_36*ROcp48_26)+qd(22)*(OMcp48_221*ROcp48_621-...
 OMcp48_321*ROcp48_521);
    OPcp48_222 = OPcp48_26+ROcp48_26*qdd(21)+ROcp48_521*qdd(22)-qd(21)*(OMcp48_16*ROcp48_36-OMcp48_36*ROcp48_16)-qd(22)*(OMcp48_121*ROcp48_621-...
 OMcp48_321*ROcp48_421);
    OPcp48_322 = OPcp48_36+ROcp48_36*qdd(21)+ROcp48_621*qdd(22)+qd(21)*(OMcp48_16*ROcp48_26-OMcp48_26*ROcp48_16)+qd(22)*(OMcp48_121*ROcp48_521-...
 OMcp48_221*ROcp48_421);
    RLcp48_123 = ROcp48_722*s.dpt(3,37);
    RLcp48_223 = ROcp48_822*s.dpt(3,37);
    RLcp48_323 = ROcp48_922*s.dpt(3,37);
    OMcp48_123 = OMcp48_122+ROcp48_722*qd(23);
    OMcp48_223 = OMcp48_222+ROcp48_822*qd(23);
    OMcp48_323 = OMcp48_322+ROcp48_922*qd(23);
    ORcp48_123 = OMcp48_222*RLcp48_323-OMcp48_322*RLcp48_223;
    ORcp48_223 = -(OMcp48_122*RLcp48_323-OMcp48_322*RLcp48_123);
    ORcp48_323 = OMcp48_122*RLcp48_223-OMcp48_222*RLcp48_123;
    OPcp48_123 = OPcp48_122+ROcp48_722*qdd(23)+qd(23)*(OMcp48_222*ROcp48_922-OMcp48_322*ROcp48_822);
    OPcp48_223 = OPcp48_222+ROcp48_822*qdd(23)-qd(23)*(OMcp48_122*ROcp48_922-OMcp48_322*ROcp48_722);
    OPcp48_323 = OPcp48_322+ROcp48_922*qdd(23)+qd(23)*(OMcp48_122*ROcp48_822-OMcp48_222*ROcp48_722);

% = = Block_1_0_0_49_0_6 = = 
 
% Sensor Kinematics 


    ROcp48_128 = ROcp48_123*C28-ROcp48_722*S28;
    ROcp48_228 = ROcp48_223*C28-ROcp48_822*S28;
    ROcp48_328 = ROcp48_323*C28-ROcp48_922*S28;
    ROcp48_728 = ROcp48_123*S28+ROcp48_722*C28;
    ROcp48_828 = ROcp48_223*S28+ROcp48_822*C28;
    ROcp48_928 = ROcp48_323*S28+ROcp48_922*C28;
    ROcp48_429 = ROcp48_423*C29+ROcp48_728*S29;
    ROcp48_529 = ROcp48_523*C29+ROcp48_828*S29;
    ROcp48_629 = ROcp48_623*C29+ROcp48_928*S29;
    ROcp48_729 = -(ROcp48_423*S29-ROcp48_728*C29);
    ROcp48_829 = -(ROcp48_523*S29-ROcp48_828*C29);
    ROcp48_929 = -(ROcp48_623*S29-ROcp48_928*C29);
    RLcp48_128 = ROcp48_123*s.dpt(1,42)+ROcp48_423*s.dpt(2,42)+ROcp48_722*s.dpt(3,42);
    RLcp48_228 = ROcp48_223*s.dpt(1,42)+ROcp48_523*s.dpt(2,42)+ROcp48_822*s.dpt(3,42);
    RLcp48_328 = ROcp48_323*s.dpt(1,42)+ROcp48_623*s.dpt(2,42)+ROcp48_922*s.dpt(3,42);
    OMcp48_128 = OMcp48_123+ROcp48_423*qd(28);
    OMcp48_228 = OMcp48_223+ROcp48_523*qd(28);
    OMcp48_328 = OMcp48_323+ROcp48_623*qd(28);
    ORcp48_128 = OMcp48_223*RLcp48_328-OMcp48_323*RLcp48_228;
    ORcp48_228 = -(OMcp48_123*RLcp48_328-OMcp48_323*RLcp48_128);
    ORcp48_328 = OMcp48_123*RLcp48_228-OMcp48_223*RLcp48_128;
    OPcp48_128 = OPcp48_123+ROcp48_423*qdd(28)+qd(28)*(OMcp48_223*ROcp48_623-OMcp48_323*ROcp48_523);
    OPcp48_228 = OPcp48_223+ROcp48_523*qdd(28)-qd(28)*(OMcp48_123*ROcp48_623-OMcp48_323*ROcp48_423);
    OPcp48_328 = OPcp48_323+ROcp48_623*qdd(28)+qd(28)*(OMcp48_123*ROcp48_523-OMcp48_223*ROcp48_423);
    RLcp48_129 = ROcp48_423*s.dpt(2,52);
    RLcp48_229 = ROcp48_523*s.dpt(2,52);
    RLcp48_329 = ROcp48_623*s.dpt(2,52);
    OMcp48_129 = OMcp48_128+ROcp48_128*qd(29);
    OMcp48_229 = OMcp48_228+ROcp48_228*qd(29);
    OMcp48_329 = OMcp48_328+ROcp48_328*qd(29);
    ORcp48_129 = OMcp48_228*RLcp48_329-OMcp48_328*RLcp48_229;
    ORcp48_229 = -(OMcp48_128*RLcp48_329-OMcp48_328*RLcp48_129);
    ORcp48_329 = OMcp48_128*RLcp48_229-OMcp48_228*RLcp48_129;
    OPcp48_129 = OPcp48_128+ROcp48_128*qdd(29)+qd(29)*(OMcp48_228*ROcp48_328-OMcp48_328*ROcp48_228);
    OPcp48_229 = OPcp48_228+ROcp48_228*qdd(29)-qd(29)*(OMcp48_128*ROcp48_328-OMcp48_328*ROcp48_128);
    OPcp48_329 = OPcp48_328+ROcp48_328*qdd(29)+qd(29)*(OMcp48_128*ROcp48_228-OMcp48_228*ROcp48_128);
    RLcp48_180 = ROcp48_128*s.dpt(1,53)+ROcp48_429*s.dpt(2,53)+ROcp48_729*s.dpt(3,53);
    RLcp48_280 = ROcp48_228*s.dpt(1,53)+ROcp48_529*s.dpt(2,53)+ROcp48_829*s.dpt(3,53);
    RLcp48_380 = ROcp48_328*s.dpt(1,53)+ROcp48_629*s.dpt(2,53)+ROcp48_929*s.dpt(3,53);
    POcp48_180 = RLcp48_121+RLcp48_123+RLcp48_128+RLcp48_129+RLcp48_180+q(1);
    POcp48_280 = RLcp48_221+RLcp48_223+RLcp48_228+RLcp48_229+RLcp48_280+q(2);
    POcp48_380 = RLcp48_321+RLcp48_323+RLcp48_328+RLcp48_329+RLcp48_380+q(3);
    JTcp48_280_4 = -(RLcp48_321+RLcp48_323+RLcp48_328+RLcp48_329+RLcp48_380);
    JTcp48_380_4 = RLcp48_221+RLcp48_223+RLcp48_228+RLcp48_229+RLcp48_280;
    JTcp48_180_5 = C4*(RLcp48_321+RLcp48_323+RLcp48_328+RLcp48_329)-S4*(RLcp48_221+RLcp48_223)-S4*(RLcp48_228+RLcp48_229)-RLcp48_280*S4+RLcp48_380*...
 C4;
    JTcp48_280_5 = S4*(RLcp48_121+RLcp48_123+RLcp48_128+RLcp48_129+RLcp48_180);
    JTcp48_380_5 = -C4*(RLcp48_121+RLcp48_123+RLcp48_128+RLcp48_129+RLcp48_180);
    JTcp48_180_6 = ROcp48_85*(RLcp48_321+RLcp48_323+RLcp48_328+RLcp48_329)-ROcp48_95*(RLcp48_221+RLcp48_223)-ROcp48_95*(RLcp48_228+RLcp48_229)-...
 RLcp48_280*ROcp48_95+RLcp48_380*ROcp48_85;
    JTcp48_280_6 = -(RLcp48_380*S5-ROcp48_95*(RLcp48_121+RLcp48_123+RLcp48_128+RLcp48_129+RLcp48_180)+S5*(RLcp48_321+RLcp48_323)+S5*(RLcp48_328+...
 RLcp48_329));
    JTcp48_380_6 = RLcp48_280*S5-ROcp48_85*(RLcp48_121+RLcp48_123+RLcp48_128+RLcp48_129+RLcp48_180)+S5*(RLcp48_221+RLcp48_223)+S5*(RLcp48_228+...
 RLcp48_229);
    JTcp48_180_7 = ROcp48_26*(RLcp48_323+RLcp48_328+RLcp48_329+RLcp48_380)-ROcp48_36*(RLcp48_223+RLcp48_228)-ROcp48_36*(RLcp48_229+RLcp48_280);
    JTcp48_280_7 = -(ROcp48_16*(RLcp48_323+RLcp48_328+RLcp48_329+RLcp48_380)-ROcp48_36*(RLcp48_123+RLcp48_128)-ROcp48_36*(RLcp48_129+RLcp48_180));
    JTcp48_380_7 = ROcp48_16*(RLcp48_223+RLcp48_228+RLcp48_229+RLcp48_280)-ROcp48_26*(RLcp48_123+RLcp48_128)-ROcp48_26*(RLcp48_129+RLcp48_180);
    JTcp48_180_8 = ROcp48_521*(RLcp48_323+RLcp48_328+RLcp48_329+RLcp48_380)-ROcp48_621*(RLcp48_223+RLcp48_228)-ROcp48_621*(RLcp48_229+RLcp48_280);
    JTcp48_280_8 = -(ROcp48_421*(RLcp48_323+RLcp48_328+RLcp48_329+RLcp48_380)-ROcp48_621*(RLcp48_123+RLcp48_128)-ROcp48_621*(RLcp48_129+RLcp48_180)...
 );
    JTcp48_380_8 = ROcp48_421*(RLcp48_223+RLcp48_228+RLcp48_229+RLcp48_280)-ROcp48_521*(RLcp48_123+RLcp48_128)-ROcp48_521*(RLcp48_129+RLcp48_180);
    JTcp48_180_9 = ROcp48_822*(RLcp48_328+RLcp48_329)-ROcp48_922*(RLcp48_228+RLcp48_229)-RLcp48_280*ROcp48_922+RLcp48_380*ROcp48_822;
    JTcp48_280_9 = RLcp48_180*ROcp48_922-RLcp48_380*ROcp48_722-ROcp48_722*(RLcp48_328+RLcp48_329)+ROcp48_922*(RLcp48_128+RLcp48_129);
    JTcp48_380_9 = ROcp48_722*(RLcp48_228+RLcp48_229)-ROcp48_822*(RLcp48_128+RLcp48_129)-RLcp48_180*ROcp48_822+RLcp48_280*ROcp48_722;
    JTcp48_180_10 = ROcp48_523*(RLcp48_329+RLcp48_380)-ROcp48_623*(RLcp48_229+RLcp48_280);
    JTcp48_280_10 = -(ROcp48_423*(RLcp48_329+RLcp48_380)-ROcp48_623*(RLcp48_129+RLcp48_180));
    JTcp48_380_10 = ROcp48_423*(RLcp48_229+RLcp48_280)-ROcp48_523*(RLcp48_129+RLcp48_180);
    JTcp48_180_11 = -(RLcp48_280*ROcp48_328-RLcp48_380*ROcp48_228);
    JTcp48_280_11 = RLcp48_180*ROcp48_328-RLcp48_380*ROcp48_128;
    JTcp48_380_11 = -(RLcp48_180*ROcp48_228-RLcp48_280*ROcp48_128);
    ORcp48_180 = OMcp48_229*RLcp48_380-OMcp48_329*RLcp48_280;
    ORcp48_280 = -(OMcp48_129*RLcp48_380-OMcp48_329*RLcp48_180);
    ORcp48_380 = OMcp48_129*RLcp48_280-OMcp48_229*RLcp48_180;
    VIcp48_180 = ORcp48_121+ORcp48_123+ORcp48_128+ORcp48_129+ORcp48_180+qd(1);
    VIcp48_280 = ORcp48_221+ORcp48_223+ORcp48_228+ORcp48_229+ORcp48_280+qd(2);
    VIcp48_380 = ORcp48_321+ORcp48_323+ORcp48_328+ORcp48_329+ORcp48_380+qd(3);
    ACcp48_180 = qdd(1)+OMcp48_222*ORcp48_323+OMcp48_223*ORcp48_328+OMcp48_228*ORcp48_329+OMcp48_229*ORcp48_380+OMcp48_26*ORcp48_321-OMcp48_322*...
 ORcp48_223-OMcp48_323*ORcp48_228-OMcp48_328*ORcp48_229-OMcp48_329*ORcp48_280-OMcp48_36*ORcp48_221+OPcp48_222*RLcp48_323+OPcp48_223*RLcp48_328+...
 OPcp48_228*RLcp48_329+OPcp48_229*RLcp48_380+OPcp48_26*RLcp48_321-OPcp48_322*RLcp48_223-OPcp48_323*RLcp48_228-OPcp48_328*RLcp48_229-OPcp48_329*...
 RLcp48_280-OPcp48_36*RLcp48_221;
    ACcp48_280 = qdd(2)-OMcp48_122*ORcp48_323-OMcp48_123*ORcp48_328-OMcp48_128*ORcp48_329-OMcp48_129*ORcp48_380-OMcp48_16*ORcp48_321+OMcp48_322*...
 ORcp48_123+OMcp48_323*ORcp48_128+OMcp48_328*ORcp48_129+OMcp48_329*ORcp48_180+OMcp48_36*ORcp48_121-OPcp48_122*RLcp48_323-OPcp48_123*RLcp48_328-...
 OPcp48_128*RLcp48_329-OPcp48_129*RLcp48_380-OPcp48_16*RLcp48_321+OPcp48_322*RLcp48_123+OPcp48_323*RLcp48_128+OPcp48_328*RLcp48_129+OPcp48_329*...
 RLcp48_180+OPcp48_36*RLcp48_121;
    ACcp48_380 = qdd(3)+OMcp48_122*ORcp48_223+OMcp48_123*ORcp48_228+OMcp48_128*ORcp48_229+OMcp48_129*ORcp48_280+OMcp48_16*ORcp48_221-OMcp48_222*...
 ORcp48_123-OMcp48_223*ORcp48_128-OMcp48_228*ORcp48_129-OMcp48_229*ORcp48_180-OMcp48_26*ORcp48_121+OPcp48_122*RLcp48_223+OPcp48_123*RLcp48_228+...
 OPcp48_128*RLcp48_229+OPcp48_129*RLcp48_280+OPcp48_16*RLcp48_221-OPcp48_222*RLcp48_123-OPcp48_223*RLcp48_128-OPcp48_228*RLcp48_129-OPcp48_229*...
 RLcp48_180-OPcp48_26*RLcp48_121;

% = = Block_1_0_0_49_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp48_180;
    sens.P(2) = POcp48_280;
    sens.P(3) = POcp48_380;
    sens.R(1,1) = ROcp48_128;
    sens.R(1,2) = ROcp48_228;
    sens.R(1,3) = ROcp48_328;
    sens.R(2,1) = ROcp48_429;
    sens.R(2,2) = ROcp48_529;
    sens.R(2,3) = ROcp48_629;
    sens.R(3,1) = ROcp48_729;
    sens.R(3,2) = ROcp48_829;
    sens.R(3,3) = ROcp48_929;
    sens.V(1) = VIcp48_180;
    sens.V(2) = VIcp48_280;
    sens.V(3) = VIcp48_380;
    sens.OM(1) = OMcp48_129;
    sens.OM(2) = OMcp48_229;
    sens.OM(3) = OMcp48_329;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp48_180_5;
    sens.J(1,6) = JTcp48_180_6;
    sens.J(1,21) = JTcp48_180_7;
    sens.J(1,22) = JTcp48_180_8;
    sens.J(1,23) = JTcp48_180_9;
    sens.J(1,28) = JTcp48_180_10;
    sens.J(1,29) = JTcp48_180_11;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp48_280_4;
    sens.J(2,5) = JTcp48_280_5;
    sens.J(2,6) = JTcp48_280_6;
    sens.J(2,21) = JTcp48_280_7;
    sens.J(2,22) = JTcp48_280_8;
    sens.J(2,23) = JTcp48_280_9;
    sens.J(2,28) = JTcp48_280_10;
    sens.J(2,29) = JTcp48_280_11;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp48_380_4;
    sens.J(3,5) = JTcp48_380_5;
    sens.J(3,6) = JTcp48_380_6;
    sens.J(3,21) = JTcp48_380_7;
    sens.J(3,22) = JTcp48_380_8;
    sens.J(3,23) = JTcp48_380_9;
    sens.J(3,28) = JTcp48_380_10;
    sens.J(3,29) = JTcp48_380_11;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,21) = ROcp48_16;
    sens.J(4,22) = ROcp48_421;
    sens.J(4,23) = ROcp48_722;
    sens.J(4,28) = ROcp48_423;
    sens.J(4,29) = ROcp48_128;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp48_85;
    sens.J(5,21) = ROcp48_26;
    sens.J(5,22) = ROcp48_521;
    sens.J(5,23) = ROcp48_822;
    sens.J(5,28) = ROcp48_523;
    sens.J(5,29) = ROcp48_228;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp48_95;
    sens.J(6,21) = ROcp48_36;
    sens.J(6,22) = ROcp48_621;
    sens.J(6,23) = ROcp48_922;
    sens.J(6,28) = ROcp48_623;
    sens.J(6,29) = ROcp48_328;
    sens.A(1) = ACcp48_180;
    sens.A(2) = ACcp48_280;
    sens.A(3) = ACcp48_380;
    sens.OMP(1) = OPcp48_129;
    sens.OMP(2) = OPcp48_229;
    sens.OMP(3) = OPcp48_329;
 
% 
case 50, 


% = = Block_1_0_0_50_0_1 = = 
 
% Sensor Kinematics 


    ROcp49_25 = S4*S5;
    ROcp49_35 = -C4*S5;
    ROcp49_85 = -S4*C5;
    ROcp49_95 = C4*C5;
    ROcp49_16 = C5*C6;
    ROcp49_26 = ROcp49_25*C6+C4*S6;
    ROcp49_36 = ROcp49_35*C6+S4*S6;
    ROcp49_46 = -C5*S6;
    ROcp49_56 = -(ROcp49_25*S6-C4*C6);
    ROcp49_66 = -(ROcp49_35*S6-S4*C6);
    OMcp49_25 = qd(5)*C4;
    OMcp49_35 = qd(5)*S4;
    OMcp49_16 = qd(4)+qd(6)*S5;
    OMcp49_26 = OMcp49_25+ROcp49_85*qd(6);
    OMcp49_36 = OMcp49_35+ROcp49_95*qd(6);
    OPcp49_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp49_26 = ROcp49_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp49_35*S5-ROcp49_95*qd(4));
    OPcp49_36 = ROcp49_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp49_25*S5-ROcp49_85*qd(4));

% = = Block_1_0_0_50_0_4 = = 
 
% Sensor Kinematics 


    ROcp49_421 = ROcp49_46*C21+S21*S5;
    ROcp49_521 = ROcp49_56*C21+ROcp49_85*S21;
    ROcp49_621 = ROcp49_66*C21+ROcp49_95*S21;
    ROcp49_721 = -(ROcp49_46*S21-C21*S5);
    ROcp49_821 = -(ROcp49_56*S21-ROcp49_85*C21);
    ROcp49_921 = -(ROcp49_66*S21-ROcp49_95*C21);
    ROcp49_122 = ROcp49_16*C22-ROcp49_721*S22;
    ROcp49_222 = ROcp49_26*C22-ROcp49_821*S22;
    ROcp49_322 = ROcp49_36*C22-ROcp49_921*S22;
    ROcp49_722 = ROcp49_16*S22+ROcp49_721*C22;
    ROcp49_822 = ROcp49_26*S22+ROcp49_821*C22;
    ROcp49_922 = ROcp49_36*S22+ROcp49_921*C22;
    ROcp49_123 = ROcp49_122*C23+ROcp49_421*S23;
    ROcp49_223 = ROcp49_222*C23+ROcp49_521*S23;
    ROcp49_323 = ROcp49_322*C23+ROcp49_621*S23;
    ROcp49_423 = -(ROcp49_122*S23-ROcp49_421*C23);
    ROcp49_523 = -(ROcp49_222*S23-ROcp49_521*C23);
    ROcp49_623 = -(ROcp49_322*S23-ROcp49_621*C23);
    RLcp49_121 = ROcp49_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp49_221 = ROcp49_26*s.dpt(1,3)+ROcp49_85*s.dpt(3,3);
    RLcp49_321 = ROcp49_36*s.dpt(1,3)+ROcp49_95*s.dpt(3,3);
    OMcp49_121 = OMcp49_16+ROcp49_16*qd(21);
    OMcp49_221 = OMcp49_26+ROcp49_26*qd(21);
    OMcp49_321 = OMcp49_36+ROcp49_36*qd(21);
    ORcp49_121 = OMcp49_26*RLcp49_321-OMcp49_36*RLcp49_221;
    ORcp49_221 = -(OMcp49_16*RLcp49_321-OMcp49_36*RLcp49_121);
    ORcp49_321 = OMcp49_16*RLcp49_221-OMcp49_26*RLcp49_121;
    OMcp49_122 = OMcp49_121+ROcp49_421*qd(22);
    OMcp49_222 = OMcp49_221+ROcp49_521*qd(22);
    OMcp49_322 = OMcp49_321+ROcp49_621*qd(22);
    OPcp49_122 = OPcp49_16+ROcp49_16*qdd(21)+ROcp49_421*qdd(22)+qd(21)*(OMcp49_26*ROcp49_36-OMcp49_36*ROcp49_26)+qd(22)*(OMcp49_221*ROcp49_621-...
 OMcp49_321*ROcp49_521);
    OPcp49_222 = OPcp49_26+ROcp49_26*qdd(21)+ROcp49_521*qdd(22)-qd(21)*(OMcp49_16*ROcp49_36-OMcp49_36*ROcp49_16)-qd(22)*(OMcp49_121*ROcp49_621-...
 OMcp49_321*ROcp49_421);
    OPcp49_322 = OPcp49_36+ROcp49_36*qdd(21)+ROcp49_621*qdd(22)+qd(21)*(OMcp49_16*ROcp49_26-OMcp49_26*ROcp49_16)+qd(22)*(OMcp49_121*ROcp49_521-...
 OMcp49_221*ROcp49_421);
    RLcp49_123 = ROcp49_722*s.dpt(3,37);
    RLcp49_223 = ROcp49_822*s.dpt(3,37);
    RLcp49_323 = ROcp49_922*s.dpt(3,37);
    OMcp49_123 = OMcp49_122+ROcp49_722*qd(23);
    OMcp49_223 = OMcp49_222+ROcp49_822*qd(23);
    OMcp49_323 = OMcp49_322+ROcp49_922*qd(23);
    ORcp49_123 = OMcp49_222*RLcp49_323-OMcp49_322*RLcp49_223;
    ORcp49_223 = -(OMcp49_122*RLcp49_323-OMcp49_322*RLcp49_123);
    ORcp49_323 = OMcp49_122*RLcp49_223-OMcp49_222*RLcp49_123;
    OPcp49_123 = OPcp49_122+ROcp49_722*qdd(23)+qd(23)*(OMcp49_222*ROcp49_922-OMcp49_322*ROcp49_822);
    OPcp49_223 = OPcp49_222+ROcp49_822*qdd(23)-qd(23)*(OMcp49_122*ROcp49_922-OMcp49_322*ROcp49_722);
    OPcp49_323 = OPcp49_322+ROcp49_922*qdd(23)+qd(23)*(OMcp49_122*ROcp49_822-OMcp49_222*ROcp49_722);

% = = Block_1_0_0_50_0_6 = = 
 
% Sensor Kinematics 


    ROcp49_128 = ROcp49_123*C28-ROcp49_722*S28;
    ROcp49_228 = ROcp49_223*C28-ROcp49_822*S28;
    ROcp49_328 = ROcp49_323*C28-ROcp49_922*S28;
    ROcp49_728 = ROcp49_123*S28+ROcp49_722*C28;
    ROcp49_828 = ROcp49_223*S28+ROcp49_822*C28;
    ROcp49_928 = ROcp49_323*S28+ROcp49_922*C28;
    ROcp49_429 = ROcp49_423*C29+ROcp49_728*S29;
    ROcp49_529 = ROcp49_523*C29+ROcp49_828*S29;
    ROcp49_629 = ROcp49_623*C29+ROcp49_928*S29;
    ROcp49_729 = -(ROcp49_423*S29-ROcp49_728*C29);
    ROcp49_829 = -(ROcp49_523*S29-ROcp49_828*C29);
    ROcp49_929 = -(ROcp49_623*S29-ROcp49_928*C29);
    RLcp49_128 = ROcp49_123*s.dpt(1,42)+ROcp49_423*s.dpt(2,42)+ROcp49_722*s.dpt(3,42);
    RLcp49_228 = ROcp49_223*s.dpt(1,42)+ROcp49_523*s.dpt(2,42)+ROcp49_822*s.dpt(3,42);
    RLcp49_328 = ROcp49_323*s.dpt(1,42)+ROcp49_623*s.dpt(2,42)+ROcp49_922*s.dpt(3,42);
    OMcp49_128 = OMcp49_123+ROcp49_423*qd(28);
    OMcp49_228 = OMcp49_223+ROcp49_523*qd(28);
    OMcp49_328 = OMcp49_323+ROcp49_623*qd(28);
    ORcp49_128 = OMcp49_223*RLcp49_328-OMcp49_323*RLcp49_228;
    ORcp49_228 = -(OMcp49_123*RLcp49_328-OMcp49_323*RLcp49_128);
    ORcp49_328 = OMcp49_123*RLcp49_228-OMcp49_223*RLcp49_128;
    OPcp49_128 = OPcp49_123+ROcp49_423*qdd(28)+qd(28)*(OMcp49_223*ROcp49_623-OMcp49_323*ROcp49_523);
    OPcp49_228 = OPcp49_223+ROcp49_523*qdd(28)-qd(28)*(OMcp49_123*ROcp49_623-OMcp49_323*ROcp49_423);
    OPcp49_328 = OPcp49_323+ROcp49_623*qdd(28)+qd(28)*(OMcp49_123*ROcp49_523-OMcp49_223*ROcp49_423);
    RLcp49_129 = ROcp49_423*s.dpt(2,52);
    RLcp49_229 = ROcp49_523*s.dpt(2,52);
    RLcp49_329 = ROcp49_623*s.dpt(2,52);
    OMcp49_129 = OMcp49_128+ROcp49_128*qd(29);
    OMcp49_229 = OMcp49_228+ROcp49_228*qd(29);
    OMcp49_329 = OMcp49_328+ROcp49_328*qd(29);
    ORcp49_129 = OMcp49_228*RLcp49_329-OMcp49_328*RLcp49_229;
    ORcp49_229 = -(OMcp49_128*RLcp49_329-OMcp49_328*RLcp49_129);
    ORcp49_329 = OMcp49_128*RLcp49_229-OMcp49_228*RLcp49_129;
    OPcp49_129 = OPcp49_128+ROcp49_128*qdd(29)+qd(29)*(OMcp49_228*ROcp49_328-OMcp49_328*ROcp49_228);
    OPcp49_229 = OPcp49_228+ROcp49_228*qdd(29)-qd(29)*(OMcp49_128*ROcp49_328-OMcp49_328*ROcp49_128);
    OPcp49_329 = OPcp49_328+ROcp49_328*qdd(29)+qd(29)*(OMcp49_128*ROcp49_228-OMcp49_228*ROcp49_128);
    RLcp49_181 = ROcp49_729*s.dpt(3,54);
    RLcp49_281 = ROcp49_829*s.dpt(3,54);
    RLcp49_381 = ROcp49_929*s.dpt(3,54);
    POcp49_181 = RLcp49_121+RLcp49_123+RLcp49_128+RLcp49_129+RLcp49_181+q(1);
    POcp49_281 = RLcp49_221+RLcp49_223+RLcp49_228+RLcp49_229+RLcp49_281+q(2);
    POcp49_381 = RLcp49_321+RLcp49_323+RLcp49_328+RLcp49_329+RLcp49_381+q(3);
    JTcp49_281_4 = -(RLcp49_321+RLcp49_323+RLcp49_328+RLcp49_329+RLcp49_381);
    JTcp49_381_4 = RLcp49_221+RLcp49_223+RLcp49_228+RLcp49_229+RLcp49_281;
    JTcp49_181_5 = C4*(RLcp49_321+RLcp49_323+RLcp49_328+RLcp49_329)-S4*(RLcp49_221+RLcp49_223)-S4*(RLcp49_228+RLcp49_229)-RLcp49_281*S4+RLcp49_381*...
 C4;
    JTcp49_281_5 = S4*(RLcp49_121+RLcp49_123+RLcp49_128+RLcp49_129+RLcp49_181);
    JTcp49_381_5 = -C4*(RLcp49_121+RLcp49_123+RLcp49_128+RLcp49_129+RLcp49_181);
    JTcp49_181_6 = ROcp49_85*(RLcp49_321+RLcp49_323+RLcp49_328+RLcp49_329)-ROcp49_95*(RLcp49_221+RLcp49_223)-ROcp49_95*(RLcp49_228+RLcp49_229)-...
 RLcp49_281*ROcp49_95+RLcp49_381*ROcp49_85;
    JTcp49_281_6 = -(RLcp49_381*S5-ROcp49_95*(RLcp49_121+RLcp49_123+RLcp49_128+RLcp49_129+RLcp49_181)+S5*(RLcp49_321+RLcp49_323)+S5*(RLcp49_328+...
 RLcp49_329));
    JTcp49_381_6 = RLcp49_281*S5-ROcp49_85*(RLcp49_121+RLcp49_123+RLcp49_128+RLcp49_129+RLcp49_181)+S5*(RLcp49_221+RLcp49_223)+S5*(RLcp49_228+...
 RLcp49_229);
    JTcp49_181_7 = ROcp49_26*(RLcp49_323+RLcp49_328+RLcp49_329+RLcp49_381)-ROcp49_36*(RLcp49_223+RLcp49_228)-ROcp49_36*(RLcp49_229+RLcp49_281);
    JTcp49_281_7 = -(ROcp49_16*(RLcp49_323+RLcp49_328+RLcp49_329+RLcp49_381)-ROcp49_36*(RLcp49_123+RLcp49_128)-ROcp49_36*(RLcp49_129+RLcp49_181));
    JTcp49_381_7 = ROcp49_16*(RLcp49_223+RLcp49_228+RLcp49_229+RLcp49_281)-ROcp49_26*(RLcp49_123+RLcp49_128)-ROcp49_26*(RLcp49_129+RLcp49_181);
    JTcp49_181_8 = ROcp49_521*(RLcp49_323+RLcp49_328+RLcp49_329+RLcp49_381)-ROcp49_621*(RLcp49_223+RLcp49_228)-ROcp49_621*(RLcp49_229+RLcp49_281);
    JTcp49_281_8 = -(ROcp49_421*(RLcp49_323+RLcp49_328+RLcp49_329+RLcp49_381)-ROcp49_621*(RLcp49_123+RLcp49_128)-ROcp49_621*(RLcp49_129+RLcp49_181)...
 );
    JTcp49_381_8 = ROcp49_421*(RLcp49_223+RLcp49_228+RLcp49_229+RLcp49_281)-ROcp49_521*(RLcp49_123+RLcp49_128)-ROcp49_521*(RLcp49_129+RLcp49_181);
    JTcp49_181_9 = ROcp49_822*(RLcp49_328+RLcp49_329)-ROcp49_922*(RLcp49_228+RLcp49_229)-RLcp49_281*ROcp49_922+RLcp49_381*ROcp49_822;
    JTcp49_281_9 = RLcp49_181*ROcp49_922-RLcp49_381*ROcp49_722-ROcp49_722*(RLcp49_328+RLcp49_329)+ROcp49_922*(RLcp49_128+RLcp49_129);
    JTcp49_381_9 = ROcp49_722*(RLcp49_228+RLcp49_229)-ROcp49_822*(RLcp49_128+RLcp49_129)-RLcp49_181*ROcp49_822+RLcp49_281*ROcp49_722;
    JTcp49_181_10 = ROcp49_523*(RLcp49_329+RLcp49_381)-ROcp49_623*(RLcp49_229+RLcp49_281);
    JTcp49_281_10 = -(ROcp49_423*(RLcp49_329+RLcp49_381)-ROcp49_623*(RLcp49_129+RLcp49_181));
    JTcp49_381_10 = ROcp49_423*(RLcp49_229+RLcp49_281)-ROcp49_523*(RLcp49_129+RLcp49_181);
    JTcp49_181_11 = -(RLcp49_281*ROcp49_328-RLcp49_381*ROcp49_228);
    JTcp49_281_11 = RLcp49_181*ROcp49_328-RLcp49_381*ROcp49_128;
    JTcp49_381_11 = -(RLcp49_181*ROcp49_228-RLcp49_281*ROcp49_128);
    ORcp49_181 = OMcp49_229*RLcp49_381-OMcp49_329*RLcp49_281;
    ORcp49_281 = -(OMcp49_129*RLcp49_381-OMcp49_329*RLcp49_181);
    ORcp49_381 = OMcp49_129*RLcp49_281-OMcp49_229*RLcp49_181;
    VIcp49_181 = ORcp49_121+ORcp49_123+ORcp49_128+ORcp49_129+ORcp49_181+qd(1);
    VIcp49_281 = ORcp49_221+ORcp49_223+ORcp49_228+ORcp49_229+ORcp49_281+qd(2);
    VIcp49_381 = ORcp49_321+ORcp49_323+ORcp49_328+ORcp49_329+ORcp49_381+qd(3);
    ACcp49_181 = qdd(1)+OMcp49_222*ORcp49_323+OMcp49_223*ORcp49_328+OMcp49_228*ORcp49_329+OMcp49_229*ORcp49_381+OMcp49_26*ORcp49_321-OMcp49_322*...
 ORcp49_223-OMcp49_323*ORcp49_228-OMcp49_328*ORcp49_229-OMcp49_329*ORcp49_281-OMcp49_36*ORcp49_221+OPcp49_222*RLcp49_323+OPcp49_223*RLcp49_328+...
 OPcp49_228*RLcp49_329+OPcp49_229*RLcp49_381+OPcp49_26*RLcp49_321-OPcp49_322*RLcp49_223-OPcp49_323*RLcp49_228-OPcp49_328*RLcp49_229-OPcp49_329*...
 RLcp49_281-OPcp49_36*RLcp49_221;
    ACcp49_281 = qdd(2)-OMcp49_122*ORcp49_323-OMcp49_123*ORcp49_328-OMcp49_128*ORcp49_329-OMcp49_129*ORcp49_381-OMcp49_16*ORcp49_321+OMcp49_322*...
 ORcp49_123+OMcp49_323*ORcp49_128+OMcp49_328*ORcp49_129+OMcp49_329*ORcp49_181+OMcp49_36*ORcp49_121-OPcp49_122*RLcp49_323-OPcp49_123*RLcp49_328-...
 OPcp49_128*RLcp49_329-OPcp49_129*RLcp49_381-OPcp49_16*RLcp49_321+OPcp49_322*RLcp49_123+OPcp49_323*RLcp49_128+OPcp49_328*RLcp49_129+OPcp49_329*...
 RLcp49_181+OPcp49_36*RLcp49_121;
    ACcp49_381 = qdd(3)+OMcp49_122*ORcp49_223+OMcp49_123*ORcp49_228+OMcp49_128*ORcp49_229+OMcp49_129*ORcp49_281+OMcp49_16*ORcp49_221-OMcp49_222*...
 ORcp49_123-OMcp49_223*ORcp49_128-OMcp49_228*ORcp49_129-OMcp49_229*ORcp49_181-OMcp49_26*ORcp49_121+OPcp49_122*RLcp49_223+OPcp49_123*RLcp49_228+...
 OPcp49_128*RLcp49_229+OPcp49_129*RLcp49_281+OPcp49_16*RLcp49_221-OPcp49_222*RLcp49_123-OPcp49_223*RLcp49_128-OPcp49_228*RLcp49_129-OPcp49_229*...
 RLcp49_181-OPcp49_26*RLcp49_121;

% = = Block_1_0_0_50_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp49_181;
    sens.P(2) = POcp49_281;
    sens.P(3) = POcp49_381;
    sens.R(1,1) = ROcp49_128;
    sens.R(1,2) = ROcp49_228;
    sens.R(1,3) = ROcp49_328;
    sens.R(2,1) = ROcp49_429;
    sens.R(2,2) = ROcp49_529;
    sens.R(2,3) = ROcp49_629;
    sens.R(3,1) = ROcp49_729;
    sens.R(3,2) = ROcp49_829;
    sens.R(3,3) = ROcp49_929;
    sens.V(1) = VIcp49_181;
    sens.V(2) = VIcp49_281;
    sens.V(3) = VIcp49_381;
    sens.OM(1) = OMcp49_129;
    sens.OM(2) = OMcp49_229;
    sens.OM(3) = OMcp49_329;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp49_181_5;
    sens.J(1,6) = JTcp49_181_6;
    sens.J(1,21) = JTcp49_181_7;
    sens.J(1,22) = JTcp49_181_8;
    sens.J(1,23) = JTcp49_181_9;
    sens.J(1,28) = JTcp49_181_10;
    sens.J(1,29) = JTcp49_181_11;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp49_281_4;
    sens.J(2,5) = JTcp49_281_5;
    sens.J(2,6) = JTcp49_281_6;
    sens.J(2,21) = JTcp49_281_7;
    sens.J(2,22) = JTcp49_281_8;
    sens.J(2,23) = JTcp49_281_9;
    sens.J(2,28) = JTcp49_281_10;
    sens.J(2,29) = JTcp49_281_11;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp49_381_4;
    sens.J(3,5) = JTcp49_381_5;
    sens.J(3,6) = JTcp49_381_6;
    sens.J(3,21) = JTcp49_381_7;
    sens.J(3,22) = JTcp49_381_8;
    sens.J(3,23) = JTcp49_381_9;
    sens.J(3,28) = JTcp49_381_10;
    sens.J(3,29) = JTcp49_381_11;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,21) = ROcp49_16;
    sens.J(4,22) = ROcp49_421;
    sens.J(4,23) = ROcp49_722;
    sens.J(4,28) = ROcp49_423;
    sens.J(4,29) = ROcp49_128;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp49_85;
    sens.J(5,21) = ROcp49_26;
    sens.J(5,22) = ROcp49_521;
    sens.J(5,23) = ROcp49_822;
    sens.J(5,28) = ROcp49_523;
    sens.J(5,29) = ROcp49_228;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp49_95;
    sens.J(6,21) = ROcp49_36;
    sens.J(6,22) = ROcp49_621;
    sens.J(6,23) = ROcp49_922;
    sens.J(6,28) = ROcp49_623;
    sens.J(6,29) = ROcp49_328;
    sens.A(1) = ACcp49_181;
    sens.A(2) = ACcp49_281;
    sens.A(3) = ACcp49_381;
    sens.OMP(1) = OPcp49_129;
    sens.OMP(2) = OPcp49_229;
    sens.OMP(3) = OPcp49_329;
 
% 
case 51, 


% = = Block_1_0_0_51_0_1 = = 
 
% Sensor Kinematics 


    ROcp50_25 = S4*S5;
    ROcp50_35 = -C4*S5;
    ROcp50_85 = -S4*C5;
    ROcp50_95 = C4*C5;
    ROcp50_16 = C5*C6;
    ROcp50_26 = ROcp50_25*C6+C4*S6;
    ROcp50_36 = ROcp50_35*C6+S4*S6;
    ROcp50_46 = -C5*S6;
    ROcp50_56 = -(ROcp50_25*S6-C4*C6);
    ROcp50_66 = -(ROcp50_35*S6-S4*C6);
    OMcp50_25 = qd(5)*C4;
    OMcp50_35 = qd(5)*S4;
    OMcp50_16 = qd(4)+qd(6)*S5;
    OMcp50_26 = OMcp50_25+ROcp50_85*qd(6);
    OMcp50_36 = OMcp50_35+ROcp50_95*qd(6);
    OPcp50_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp50_26 = ROcp50_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp50_35*S5-ROcp50_95*qd(4));
    OPcp50_36 = ROcp50_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp50_25*S5-ROcp50_85*qd(4));

% = = Block_1_0_0_51_0_4 = = 
 
% Sensor Kinematics 


    ROcp50_421 = ROcp50_46*C21+S21*S5;
    ROcp50_521 = ROcp50_56*C21+ROcp50_85*S21;
    ROcp50_621 = ROcp50_66*C21+ROcp50_95*S21;
    ROcp50_721 = -(ROcp50_46*S21-C21*S5);
    ROcp50_821 = -(ROcp50_56*S21-ROcp50_85*C21);
    ROcp50_921 = -(ROcp50_66*S21-ROcp50_95*C21);
    ROcp50_122 = ROcp50_16*C22-ROcp50_721*S22;
    ROcp50_222 = ROcp50_26*C22-ROcp50_821*S22;
    ROcp50_322 = ROcp50_36*C22-ROcp50_921*S22;
    ROcp50_722 = ROcp50_16*S22+ROcp50_721*C22;
    ROcp50_822 = ROcp50_26*S22+ROcp50_821*C22;
    ROcp50_922 = ROcp50_36*S22+ROcp50_921*C22;
    ROcp50_123 = ROcp50_122*C23+ROcp50_421*S23;
    ROcp50_223 = ROcp50_222*C23+ROcp50_521*S23;
    ROcp50_323 = ROcp50_322*C23+ROcp50_621*S23;
    ROcp50_423 = -(ROcp50_122*S23-ROcp50_421*C23);
    ROcp50_523 = -(ROcp50_222*S23-ROcp50_521*C23);
    ROcp50_623 = -(ROcp50_322*S23-ROcp50_621*C23);
    RLcp50_121 = ROcp50_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp50_221 = ROcp50_26*s.dpt(1,3)+ROcp50_85*s.dpt(3,3);
    RLcp50_321 = ROcp50_36*s.dpt(1,3)+ROcp50_95*s.dpt(3,3);
    OMcp50_121 = OMcp50_16+ROcp50_16*qd(21);
    OMcp50_221 = OMcp50_26+ROcp50_26*qd(21);
    OMcp50_321 = OMcp50_36+ROcp50_36*qd(21);
    ORcp50_121 = OMcp50_26*RLcp50_321-OMcp50_36*RLcp50_221;
    ORcp50_221 = -(OMcp50_16*RLcp50_321-OMcp50_36*RLcp50_121);
    ORcp50_321 = OMcp50_16*RLcp50_221-OMcp50_26*RLcp50_121;
    OMcp50_122 = OMcp50_121+ROcp50_421*qd(22);
    OMcp50_222 = OMcp50_221+ROcp50_521*qd(22);
    OMcp50_322 = OMcp50_321+ROcp50_621*qd(22);
    OPcp50_122 = OPcp50_16+ROcp50_16*qdd(21)+ROcp50_421*qdd(22)+qd(21)*(OMcp50_26*ROcp50_36-OMcp50_36*ROcp50_26)+qd(22)*(OMcp50_221*ROcp50_621-...
 OMcp50_321*ROcp50_521);
    OPcp50_222 = OPcp50_26+ROcp50_26*qdd(21)+ROcp50_521*qdd(22)-qd(21)*(OMcp50_16*ROcp50_36-OMcp50_36*ROcp50_16)-qd(22)*(OMcp50_121*ROcp50_621-...
 OMcp50_321*ROcp50_421);
    OPcp50_322 = OPcp50_36+ROcp50_36*qdd(21)+ROcp50_621*qdd(22)+qd(21)*(OMcp50_16*ROcp50_26-OMcp50_26*ROcp50_16)+qd(22)*(OMcp50_121*ROcp50_521-...
 OMcp50_221*ROcp50_421);
    RLcp50_123 = ROcp50_722*s.dpt(3,37);
    RLcp50_223 = ROcp50_822*s.dpt(3,37);
    RLcp50_323 = ROcp50_922*s.dpt(3,37);
    OMcp50_123 = OMcp50_122+ROcp50_722*qd(23);
    OMcp50_223 = OMcp50_222+ROcp50_822*qd(23);
    OMcp50_323 = OMcp50_322+ROcp50_922*qd(23);
    ORcp50_123 = OMcp50_222*RLcp50_323-OMcp50_322*RLcp50_223;
    ORcp50_223 = -(OMcp50_122*RLcp50_323-OMcp50_322*RLcp50_123);
    ORcp50_323 = OMcp50_122*RLcp50_223-OMcp50_222*RLcp50_123;
    OPcp50_123 = OPcp50_122+ROcp50_722*qdd(23)+qd(23)*(OMcp50_222*ROcp50_922-OMcp50_322*ROcp50_822);
    OPcp50_223 = OPcp50_222+ROcp50_822*qdd(23)-qd(23)*(OMcp50_122*ROcp50_922-OMcp50_322*ROcp50_722);
    OPcp50_323 = OPcp50_322+ROcp50_922*qdd(23)+qd(23)*(OMcp50_122*ROcp50_822-OMcp50_222*ROcp50_722);

% = = Block_1_0_0_51_0_6 = = 
 
% Sensor Kinematics 


    ROcp50_128 = ROcp50_123*C28-ROcp50_722*S28;
    ROcp50_228 = ROcp50_223*C28-ROcp50_822*S28;
    ROcp50_328 = ROcp50_323*C28-ROcp50_922*S28;
    ROcp50_728 = ROcp50_123*S28+ROcp50_722*C28;
    ROcp50_828 = ROcp50_223*S28+ROcp50_822*C28;
    ROcp50_928 = ROcp50_323*S28+ROcp50_922*C28;
    ROcp50_429 = ROcp50_423*C29+ROcp50_728*S29;
    ROcp50_529 = ROcp50_523*C29+ROcp50_828*S29;
    ROcp50_629 = ROcp50_623*C29+ROcp50_928*S29;
    ROcp50_729 = -(ROcp50_423*S29-ROcp50_728*C29);
    ROcp50_829 = -(ROcp50_523*S29-ROcp50_828*C29);
    ROcp50_929 = -(ROcp50_623*S29-ROcp50_928*C29);
    ROcp50_130 = ROcp50_128*C30+ROcp50_429*S30;
    ROcp50_230 = ROcp50_228*C30+ROcp50_529*S30;
    ROcp50_330 = ROcp50_328*C30+ROcp50_629*S30;
    ROcp50_430 = -(ROcp50_128*S30-ROcp50_429*C30);
    ROcp50_530 = -(ROcp50_228*S30-ROcp50_529*C30);
    ROcp50_630 = -(ROcp50_328*S30-ROcp50_629*C30);
    RLcp50_128 = ROcp50_123*s.dpt(1,42)+ROcp50_423*s.dpt(2,42)+ROcp50_722*s.dpt(3,42);
    RLcp50_228 = ROcp50_223*s.dpt(1,42)+ROcp50_523*s.dpt(2,42)+ROcp50_822*s.dpt(3,42);
    RLcp50_328 = ROcp50_323*s.dpt(1,42)+ROcp50_623*s.dpt(2,42)+ROcp50_922*s.dpt(3,42);
    OMcp50_128 = OMcp50_123+ROcp50_423*qd(28);
    OMcp50_228 = OMcp50_223+ROcp50_523*qd(28);
    OMcp50_328 = OMcp50_323+ROcp50_623*qd(28);
    ORcp50_128 = OMcp50_223*RLcp50_328-OMcp50_323*RLcp50_228;
    ORcp50_228 = -(OMcp50_123*RLcp50_328-OMcp50_323*RLcp50_128);
    ORcp50_328 = OMcp50_123*RLcp50_228-OMcp50_223*RLcp50_128;
    OPcp50_128 = OPcp50_123+ROcp50_423*qdd(28)+qd(28)*(OMcp50_223*ROcp50_623-OMcp50_323*ROcp50_523);
    OPcp50_228 = OPcp50_223+ROcp50_523*qdd(28)-qd(28)*(OMcp50_123*ROcp50_623-OMcp50_323*ROcp50_423);
    OPcp50_328 = OPcp50_323+ROcp50_623*qdd(28)+qd(28)*(OMcp50_123*ROcp50_523-OMcp50_223*ROcp50_423);
    RLcp50_129 = ROcp50_423*s.dpt(2,52);
    RLcp50_229 = ROcp50_523*s.dpt(2,52);
    RLcp50_329 = ROcp50_623*s.dpt(2,52);
    OMcp50_129 = OMcp50_128+ROcp50_128*qd(29);
    OMcp50_229 = OMcp50_228+ROcp50_228*qd(29);
    OMcp50_329 = OMcp50_328+ROcp50_328*qd(29);
    ORcp50_129 = OMcp50_228*RLcp50_329-OMcp50_328*RLcp50_229;
    ORcp50_229 = -(OMcp50_128*RLcp50_329-OMcp50_328*RLcp50_129);
    ORcp50_329 = OMcp50_128*RLcp50_229-OMcp50_228*RLcp50_129;
    OPcp50_129 = OPcp50_128+ROcp50_128*qdd(29)+qd(29)*(OMcp50_228*ROcp50_328-OMcp50_328*ROcp50_228);
    OPcp50_229 = OPcp50_228+ROcp50_228*qdd(29)-qd(29)*(OMcp50_128*ROcp50_328-OMcp50_328*ROcp50_128);
    OPcp50_329 = OPcp50_328+ROcp50_328*qdd(29)+qd(29)*(OMcp50_128*ROcp50_228-OMcp50_228*ROcp50_128);
    RLcp50_130 = ROcp50_729*s.dpt(3,54);
    RLcp50_230 = ROcp50_829*s.dpt(3,54);
    RLcp50_330 = ROcp50_929*s.dpt(3,54);
    OMcp50_130 = OMcp50_129+ROcp50_729*qd(30);
    OMcp50_230 = OMcp50_229+ROcp50_829*qd(30);
    OMcp50_330 = OMcp50_329+ROcp50_929*qd(30);
    ORcp50_130 = OMcp50_229*RLcp50_330-OMcp50_329*RLcp50_230;
    ORcp50_230 = -(OMcp50_129*RLcp50_330-OMcp50_329*RLcp50_130);
    ORcp50_330 = OMcp50_129*RLcp50_230-OMcp50_229*RLcp50_130;
    OPcp50_130 = OPcp50_129+ROcp50_729*qdd(30)+qd(30)*(OMcp50_229*ROcp50_929-OMcp50_329*ROcp50_829);
    OPcp50_230 = OPcp50_229+ROcp50_829*qdd(30)-qd(30)*(OMcp50_129*ROcp50_929-OMcp50_329*ROcp50_729);
    OPcp50_330 = OPcp50_329+ROcp50_929*qdd(30)+qd(30)*(OMcp50_129*ROcp50_829-OMcp50_229*ROcp50_729);
    RLcp50_182 = ROcp50_130*s.dpt(1,55)+ROcp50_430*s.dpt(2,55)+ROcp50_729*s.dpt(3,55);
    RLcp50_282 = ROcp50_230*s.dpt(1,55)+ROcp50_530*s.dpt(2,55)+ROcp50_829*s.dpt(3,55);
    RLcp50_382 = ROcp50_330*s.dpt(1,55)+ROcp50_630*s.dpt(2,55)+ROcp50_929*s.dpt(3,55);
    POcp50_182 = RLcp50_121+RLcp50_123+RLcp50_128+RLcp50_129+RLcp50_130+RLcp50_182+q(1);
    POcp50_282 = RLcp50_221+RLcp50_223+RLcp50_228+RLcp50_229+RLcp50_230+RLcp50_282+q(2);
    POcp50_382 = RLcp50_321+RLcp50_323+RLcp50_328+RLcp50_329+RLcp50_330+RLcp50_382+q(3);
    JTcp50_282_4 = -(RLcp50_321+RLcp50_323+RLcp50_328+RLcp50_329+RLcp50_330+RLcp50_382);
    JTcp50_382_4 = RLcp50_221+RLcp50_223+RLcp50_228+RLcp50_229+RLcp50_230+RLcp50_282;
    JTcp50_182_5 = C4*(RLcp50_321+RLcp50_323+RLcp50_328+RLcp50_329+RLcp50_330+RLcp50_382)-S4*(RLcp50_221+RLcp50_223)-S4*(RLcp50_228+RLcp50_229)-S4*...
 (RLcp50_230+RLcp50_282);
    JTcp50_282_5 = S4*(RLcp50_121+RLcp50_123+RLcp50_128+RLcp50_129+RLcp50_130+RLcp50_182);
    JTcp50_382_5 = -C4*(RLcp50_121+RLcp50_123+RLcp50_128+RLcp50_129+RLcp50_130+RLcp50_182);
    JTcp50_182_6 = ROcp50_85*(RLcp50_321+RLcp50_323+RLcp50_328+RLcp50_329+RLcp50_330+RLcp50_382)-ROcp50_95*(RLcp50_221+RLcp50_223)-ROcp50_95*(...
 RLcp50_228+RLcp50_229)-ROcp50_95*(RLcp50_230+RLcp50_282);
    JTcp50_282_6 = RLcp50_182*ROcp50_95-RLcp50_330*S5-RLcp50_382*S5+ROcp50_95*(RLcp50_121+RLcp50_123+RLcp50_128+RLcp50_129+RLcp50_130)-S5*(...
 RLcp50_321+RLcp50_323)-S5*(RLcp50_328+RLcp50_329);
    JTcp50_382_6 = RLcp50_230*S5-ROcp50_85*(RLcp50_121+RLcp50_123+RLcp50_128+RLcp50_129+RLcp50_130)+S5*(RLcp50_221+RLcp50_223)+S5*(RLcp50_228+...
 RLcp50_229)-RLcp50_182*ROcp50_85+RLcp50_282*S5;
    JTcp50_182_7 = ROcp50_26*(RLcp50_323+RLcp50_328+RLcp50_329+RLcp50_330)-ROcp50_36*(RLcp50_223+RLcp50_228)-ROcp50_36*(RLcp50_229+RLcp50_230)-...
 RLcp50_282*ROcp50_36+RLcp50_382*ROcp50_26;
    JTcp50_282_7 = RLcp50_182*ROcp50_36-RLcp50_382*ROcp50_16-ROcp50_16*(RLcp50_323+RLcp50_328+RLcp50_329+RLcp50_330)+ROcp50_36*(RLcp50_123+...
 RLcp50_128)+ROcp50_36*(RLcp50_129+RLcp50_130);
    JTcp50_382_7 = ROcp50_16*(RLcp50_223+RLcp50_228+RLcp50_229+RLcp50_230)-ROcp50_26*(RLcp50_123+RLcp50_128)-ROcp50_26*(RLcp50_129+RLcp50_130)-...
 RLcp50_182*ROcp50_26+RLcp50_282*ROcp50_16;
    JTcp50_182_8 = ROcp50_521*(RLcp50_323+RLcp50_328+RLcp50_329+RLcp50_330)-ROcp50_621*(RLcp50_223+RLcp50_228)-ROcp50_621*(RLcp50_229+RLcp50_230)-...
 RLcp50_282*ROcp50_621+RLcp50_382*ROcp50_521;
    JTcp50_282_8 = RLcp50_182*ROcp50_621-RLcp50_382*ROcp50_421-ROcp50_421*(RLcp50_323+RLcp50_328+RLcp50_329+RLcp50_330)+ROcp50_621*(RLcp50_123+...
 RLcp50_128)+ROcp50_621*(RLcp50_129+RLcp50_130);
    JTcp50_382_8 = ROcp50_421*(RLcp50_223+RLcp50_228+RLcp50_229+RLcp50_230)-ROcp50_521*(RLcp50_123+RLcp50_128)-ROcp50_521*(RLcp50_129+RLcp50_130)-...
 RLcp50_182*ROcp50_521+RLcp50_282*ROcp50_421;
    JTcp50_182_9 = ROcp50_822*(RLcp50_328+RLcp50_329+RLcp50_330+RLcp50_382)-ROcp50_922*(RLcp50_228+RLcp50_229)-ROcp50_922*(RLcp50_230+RLcp50_282);
    JTcp50_282_9 = -(ROcp50_722*(RLcp50_328+RLcp50_329+RLcp50_330+RLcp50_382)-ROcp50_922*(RLcp50_128+RLcp50_129)-ROcp50_922*(RLcp50_130+RLcp50_182)...
 );
    JTcp50_382_9 = ROcp50_722*(RLcp50_228+RLcp50_229+RLcp50_230+RLcp50_282)-ROcp50_822*(RLcp50_128+RLcp50_129)-ROcp50_822*(RLcp50_130+RLcp50_182);
    JTcp50_182_10 = ROcp50_523*(RLcp50_329+RLcp50_330)-ROcp50_623*(RLcp50_229+RLcp50_230)-RLcp50_282*ROcp50_623+RLcp50_382*ROcp50_523;
    JTcp50_282_10 = RLcp50_182*ROcp50_623-RLcp50_382*ROcp50_423-ROcp50_423*(RLcp50_329+RLcp50_330)+ROcp50_623*(RLcp50_129+RLcp50_130);
    JTcp50_382_10 = ROcp50_423*(RLcp50_229+RLcp50_230)-ROcp50_523*(RLcp50_129+RLcp50_130)-RLcp50_182*ROcp50_523+RLcp50_282*ROcp50_423;
    JTcp50_182_11 = ROcp50_228*(RLcp50_330+RLcp50_382)-ROcp50_328*(RLcp50_230+RLcp50_282);
    JTcp50_282_11 = -(ROcp50_128*(RLcp50_330+RLcp50_382)-ROcp50_328*(RLcp50_130+RLcp50_182));
    JTcp50_382_11 = ROcp50_128*(RLcp50_230+RLcp50_282)-ROcp50_228*(RLcp50_130+RLcp50_182);
    JTcp50_182_12 = -(RLcp50_282*ROcp50_929-RLcp50_382*ROcp50_829);
    JTcp50_282_12 = RLcp50_182*ROcp50_929-RLcp50_382*ROcp50_729;
    JTcp50_382_12 = -(RLcp50_182*ROcp50_829-RLcp50_282*ROcp50_729);
    ORcp50_182 = OMcp50_230*RLcp50_382-OMcp50_330*RLcp50_282;
    ORcp50_282 = -(OMcp50_130*RLcp50_382-OMcp50_330*RLcp50_182);
    ORcp50_382 = OMcp50_130*RLcp50_282-OMcp50_230*RLcp50_182;
    VIcp50_182 = ORcp50_121+ORcp50_123+ORcp50_128+ORcp50_129+ORcp50_130+ORcp50_182+qd(1);
    VIcp50_282 = ORcp50_221+ORcp50_223+ORcp50_228+ORcp50_229+ORcp50_230+ORcp50_282+qd(2);
    VIcp50_382 = ORcp50_321+ORcp50_323+ORcp50_328+ORcp50_329+ORcp50_330+ORcp50_382+qd(3);
    ACcp50_182 = qdd(1)+OMcp50_222*ORcp50_323+OMcp50_223*ORcp50_328+OMcp50_228*ORcp50_329+OMcp50_229*ORcp50_330+OMcp50_230*ORcp50_382+OMcp50_26*...
 ORcp50_321-OMcp50_322*ORcp50_223-OMcp50_323*ORcp50_228-OMcp50_328*ORcp50_229-OMcp50_329*ORcp50_230-OMcp50_330*ORcp50_282-OMcp50_36*ORcp50_221+...
 OPcp50_222*RLcp50_323+OPcp50_223*RLcp50_328+OPcp50_228*RLcp50_329+OPcp50_229*RLcp50_330+OPcp50_230*RLcp50_382+OPcp50_26*RLcp50_321-OPcp50_322*...
 RLcp50_223-OPcp50_323*RLcp50_228-OPcp50_328*RLcp50_229-OPcp50_329*RLcp50_230-OPcp50_330*RLcp50_282-OPcp50_36*RLcp50_221;
    ACcp50_282 = qdd(2)-OMcp50_122*ORcp50_323-OMcp50_123*ORcp50_328-OMcp50_128*ORcp50_329-OMcp50_129*ORcp50_330-OMcp50_130*ORcp50_382-OMcp50_16*...
 ORcp50_321+OMcp50_322*ORcp50_123+OMcp50_323*ORcp50_128+OMcp50_328*ORcp50_129+OMcp50_329*ORcp50_130+OMcp50_330*ORcp50_182+OMcp50_36*ORcp50_121-...
 OPcp50_122*RLcp50_323-OPcp50_123*RLcp50_328-OPcp50_128*RLcp50_329-OPcp50_129*RLcp50_330-OPcp50_130*RLcp50_382-OPcp50_16*RLcp50_321+OPcp50_322*...
 RLcp50_123+OPcp50_323*RLcp50_128+OPcp50_328*RLcp50_129+OPcp50_329*RLcp50_130+OPcp50_330*RLcp50_182+OPcp50_36*RLcp50_121;
    ACcp50_382 = qdd(3)+OMcp50_122*ORcp50_223+OMcp50_123*ORcp50_228+OMcp50_128*ORcp50_229+OMcp50_129*ORcp50_230+OMcp50_130*ORcp50_282+OMcp50_16*...
 ORcp50_221-OMcp50_222*ORcp50_123-OMcp50_223*ORcp50_128-OMcp50_228*ORcp50_129-OMcp50_229*ORcp50_130-OMcp50_230*ORcp50_182-OMcp50_26*ORcp50_121+...
 OPcp50_122*RLcp50_223+OPcp50_123*RLcp50_228+OPcp50_128*RLcp50_229+OPcp50_129*RLcp50_230+OPcp50_130*RLcp50_282+OPcp50_16*RLcp50_221-OPcp50_222*...
 RLcp50_123-OPcp50_223*RLcp50_128-OPcp50_228*RLcp50_129-OPcp50_229*RLcp50_130-OPcp50_230*RLcp50_182-OPcp50_26*RLcp50_121;

% = = Block_1_0_0_51_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp50_182;
    sens.P(2) = POcp50_282;
    sens.P(3) = POcp50_382;
    sens.R(1,1) = ROcp50_130;
    sens.R(1,2) = ROcp50_230;
    sens.R(1,3) = ROcp50_330;
    sens.R(2,1) = ROcp50_430;
    sens.R(2,2) = ROcp50_530;
    sens.R(2,3) = ROcp50_630;
    sens.R(3,1) = ROcp50_729;
    sens.R(3,2) = ROcp50_829;
    sens.R(3,3) = ROcp50_929;
    sens.V(1) = VIcp50_182;
    sens.V(2) = VIcp50_282;
    sens.V(3) = VIcp50_382;
    sens.OM(1) = OMcp50_130;
    sens.OM(2) = OMcp50_230;
    sens.OM(3) = OMcp50_330;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp50_182_5;
    sens.J(1,6) = JTcp50_182_6;
    sens.J(1,21) = JTcp50_182_7;
    sens.J(1,22) = JTcp50_182_8;
    sens.J(1,23) = JTcp50_182_9;
    sens.J(1,28) = JTcp50_182_10;
    sens.J(1,29) = JTcp50_182_11;
    sens.J(1,30) = JTcp50_182_12;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp50_282_4;
    sens.J(2,5) = JTcp50_282_5;
    sens.J(2,6) = JTcp50_282_6;
    sens.J(2,21) = JTcp50_282_7;
    sens.J(2,22) = JTcp50_282_8;
    sens.J(2,23) = JTcp50_282_9;
    sens.J(2,28) = JTcp50_282_10;
    sens.J(2,29) = JTcp50_282_11;
    sens.J(2,30) = JTcp50_282_12;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp50_382_4;
    sens.J(3,5) = JTcp50_382_5;
    sens.J(3,6) = JTcp50_382_6;
    sens.J(3,21) = JTcp50_382_7;
    sens.J(3,22) = JTcp50_382_8;
    sens.J(3,23) = JTcp50_382_9;
    sens.J(3,28) = JTcp50_382_10;
    sens.J(3,29) = JTcp50_382_11;
    sens.J(3,30) = JTcp50_382_12;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,21) = ROcp50_16;
    sens.J(4,22) = ROcp50_421;
    sens.J(4,23) = ROcp50_722;
    sens.J(4,28) = ROcp50_423;
    sens.J(4,29) = ROcp50_128;
    sens.J(4,30) = ROcp50_729;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp50_85;
    sens.J(5,21) = ROcp50_26;
    sens.J(5,22) = ROcp50_521;
    sens.J(5,23) = ROcp50_822;
    sens.J(5,28) = ROcp50_523;
    sens.J(5,29) = ROcp50_228;
    sens.J(5,30) = ROcp50_829;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp50_95;
    sens.J(6,21) = ROcp50_36;
    sens.J(6,22) = ROcp50_621;
    sens.J(6,23) = ROcp50_922;
    sens.J(6,28) = ROcp50_623;
    sens.J(6,29) = ROcp50_328;
    sens.J(6,30) = ROcp50_929;
    sens.A(1) = ACcp50_182;
    sens.A(2) = ACcp50_282;
    sens.A(3) = ACcp50_382;
    sens.OMP(1) = OPcp50_130;
    sens.OMP(2) = OPcp50_230;
    sens.OMP(3) = OPcp50_330;
 
% 
case 52, 


% = = Block_1_0_0_52_0_1 = = 
 
% Sensor Kinematics 


    ROcp51_25 = S4*S5;
    ROcp51_35 = -C4*S5;
    ROcp51_85 = -S4*C5;
    ROcp51_95 = C4*C5;
    ROcp51_16 = C5*C6;
    ROcp51_26 = ROcp51_25*C6+C4*S6;
    ROcp51_36 = ROcp51_35*C6+S4*S6;
    ROcp51_46 = -C5*S6;
    ROcp51_56 = -(ROcp51_25*S6-C4*C6);
    ROcp51_66 = -(ROcp51_35*S6-S4*C6);
    OMcp51_25 = qd(5)*C4;
    OMcp51_35 = qd(5)*S4;
    OMcp51_16 = qd(4)+qd(6)*S5;
    OMcp51_26 = OMcp51_25+ROcp51_85*qd(6);
    OMcp51_36 = OMcp51_35+ROcp51_95*qd(6);
    OPcp51_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp51_26 = ROcp51_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp51_35*S5-ROcp51_95*qd(4));
    OPcp51_36 = ROcp51_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp51_25*S5-ROcp51_85*qd(4));

% = = Block_1_0_0_52_0_4 = = 
 
% Sensor Kinematics 


    ROcp51_421 = ROcp51_46*C21+S21*S5;
    ROcp51_521 = ROcp51_56*C21+ROcp51_85*S21;
    ROcp51_621 = ROcp51_66*C21+ROcp51_95*S21;
    ROcp51_721 = -(ROcp51_46*S21-C21*S5);
    ROcp51_821 = -(ROcp51_56*S21-ROcp51_85*C21);
    ROcp51_921 = -(ROcp51_66*S21-ROcp51_95*C21);
    ROcp51_122 = ROcp51_16*C22-ROcp51_721*S22;
    ROcp51_222 = ROcp51_26*C22-ROcp51_821*S22;
    ROcp51_322 = ROcp51_36*C22-ROcp51_921*S22;
    ROcp51_722 = ROcp51_16*S22+ROcp51_721*C22;
    ROcp51_822 = ROcp51_26*S22+ROcp51_821*C22;
    ROcp51_922 = ROcp51_36*S22+ROcp51_921*C22;
    ROcp51_123 = ROcp51_122*C23+ROcp51_421*S23;
    ROcp51_223 = ROcp51_222*C23+ROcp51_521*S23;
    ROcp51_323 = ROcp51_322*C23+ROcp51_621*S23;
    ROcp51_423 = -(ROcp51_122*S23-ROcp51_421*C23);
    ROcp51_523 = -(ROcp51_222*S23-ROcp51_521*C23);
    ROcp51_623 = -(ROcp51_322*S23-ROcp51_621*C23);
    RLcp51_121 = ROcp51_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp51_221 = ROcp51_26*s.dpt(1,3)+ROcp51_85*s.dpt(3,3);
    RLcp51_321 = ROcp51_36*s.dpt(1,3)+ROcp51_95*s.dpt(3,3);
    OMcp51_121 = OMcp51_16+ROcp51_16*qd(21);
    OMcp51_221 = OMcp51_26+ROcp51_26*qd(21);
    OMcp51_321 = OMcp51_36+ROcp51_36*qd(21);
    ORcp51_121 = OMcp51_26*RLcp51_321-OMcp51_36*RLcp51_221;
    ORcp51_221 = -(OMcp51_16*RLcp51_321-OMcp51_36*RLcp51_121);
    ORcp51_321 = OMcp51_16*RLcp51_221-OMcp51_26*RLcp51_121;
    OMcp51_122 = OMcp51_121+ROcp51_421*qd(22);
    OMcp51_222 = OMcp51_221+ROcp51_521*qd(22);
    OMcp51_322 = OMcp51_321+ROcp51_621*qd(22);
    OPcp51_122 = OPcp51_16+ROcp51_16*qdd(21)+ROcp51_421*qdd(22)+qd(21)*(OMcp51_26*ROcp51_36-OMcp51_36*ROcp51_26)+qd(22)*(OMcp51_221*ROcp51_621-...
 OMcp51_321*ROcp51_521);
    OPcp51_222 = OPcp51_26+ROcp51_26*qdd(21)+ROcp51_521*qdd(22)-qd(21)*(OMcp51_16*ROcp51_36-OMcp51_36*ROcp51_16)-qd(22)*(OMcp51_121*ROcp51_621-...
 OMcp51_321*ROcp51_421);
    OPcp51_322 = OPcp51_36+ROcp51_36*qdd(21)+ROcp51_621*qdd(22)+qd(21)*(OMcp51_16*ROcp51_26-OMcp51_26*ROcp51_16)+qd(22)*(OMcp51_121*ROcp51_521-...
 OMcp51_221*ROcp51_421);
    RLcp51_123 = ROcp51_722*s.dpt(3,37);
    RLcp51_223 = ROcp51_822*s.dpt(3,37);
    RLcp51_323 = ROcp51_922*s.dpt(3,37);
    OMcp51_123 = OMcp51_122+ROcp51_722*qd(23);
    OMcp51_223 = OMcp51_222+ROcp51_822*qd(23);
    OMcp51_323 = OMcp51_322+ROcp51_922*qd(23);
    ORcp51_123 = OMcp51_222*RLcp51_323-OMcp51_322*RLcp51_223;
    ORcp51_223 = -(OMcp51_122*RLcp51_323-OMcp51_322*RLcp51_123);
    ORcp51_323 = OMcp51_122*RLcp51_223-OMcp51_222*RLcp51_123;
    OPcp51_123 = OPcp51_122+ROcp51_722*qdd(23)+qd(23)*(OMcp51_222*ROcp51_922-OMcp51_322*ROcp51_822);
    OPcp51_223 = OPcp51_222+ROcp51_822*qdd(23)-qd(23)*(OMcp51_122*ROcp51_922-OMcp51_322*ROcp51_722);
    OPcp51_323 = OPcp51_322+ROcp51_922*qdd(23)+qd(23)*(OMcp51_122*ROcp51_822-OMcp51_222*ROcp51_722);

% = = Block_1_0_0_52_0_6 = = 
 
% Sensor Kinematics 


    ROcp51_128 = ROcp51_123*C28-ROcp51_722*S28;
    ROcp51_228 = ROcp51_223*C28-ROcp51_822*S28;
    ROcp51_328 = ROcp51_323*C28-ROcp51_922*S28;
    ROcp51_728 = ROcp51_123*S28+ROcp51_722*C28;
    ROcp51_828 = ROcp51_223*S28+ROcp51_822*C28;
    ROcp51_928 = ROcp51_323*S28+ROcp51_922*C28;
    ROcp51_429 = ROcp51_423*C29+ROcp51_728*S29;
    ROcp51_529 = ROcp51_523*C29+ROcp51_828*S29;
    ROcp51_629 = ROcp51_623*C29+ROcp51_928*S29;
    ROcp51_729 = -(ROcp51_423*S29-ROcp51_728*C29);
    ROcp51_829 = -(ROcp51_523*S29-ROcp51_828*C29);
    ROcp51_929 = -(ROcp51_623*S29-ROcp51_928*C29);
    ROcp51_130 = ROcp51_128*C30+ROcp51_429*S30;
    ROcp51_230 = ROcp51_228*C30+ROcp51_529*S30;
    ROcp51_330 = ROcp51_328*C30+ROcp51_629*S30;
    ROcp51_430 = -(ROcp51_128*S30-ROcp51_429*C30);
    ROcp51_530 = -(ROcp51_228*S30-ROcp51_529*C30);
    ROcp51_630 = -(ROcp51_328*S30-ROcp51_629*C30);
    RLcp51_128 = ROcp51_123*s.dpt(1,42)+ROcp51_423*s.dpt(2,42)+ROcp51_722*s.dpt(3,42);
    RLcp51_228 = ROcp51_223*s.dpt(1,42)+ROcp51_523*s.dpt(2,42)+ROcp51_822*s.dpt(3,42);
    RLcp51_328 = ROcp51_323*s.dpt(1,42)+ROcp51_623*s.dpt(2,42)+ROcp51_922*s.dpt(3,42);
    OMcp51_128 = OMcp51_123+ROcp51_423*qd(28);
    OMcp51_228 = OMcp51_223+ROcp51_523*qd(28);
    OMcp51_328 = OMcp51_323+ROcp51_623*qd(28);
    ORcp51_128 = OMcp51_223*RLcp51_328-OMcp51_323*RLcp51_228;
    ORcp51_228 = -(OMcp51_123*RLcp51_328-OMcp51_323*RLcp51_128);
    ORcp51_328 = OMcp51_123*RLcp51_228-OMcp51_223*RLcp51_128;
    OPcp51_128 = OPcp51_123+ROcp51_423*qdd(28)+qd(28)*(OMcp51_223*ROcp51_623-OMcp51_323*ROcp51_523);
    OPcp51_228 = OPcp51_223+ROcp51_523*qdd(28)-qd(28)*(OMcp51_123*ROcp51_623-OMcp51_323*ROcp51_423);
    OPcp51_328 = OPcp51_323+ROcp51_623*qdd(28)+qd(28)*(OMcp51_123*ROcp51_523-OMcp51_223*ROcp51_423);
    RLcp51_129 = ROcp51_423*s.dpt(2,52);
    RLcp51_229 = ROcp51_523*s.dpt(2,52);
    RLcp51_329 = ROcp51_623*s.dpt(2,52);
    OMcp51_129 = OMcp51_128+ROcp51_128*qd(29);
    OMcp51_229 = OMcp51_228+ROcp51_228*qd(29);
    OMcp51_329 = OMcp51_328+ROcp51_328*qd(29);
    ORcp51_129 = OMcp51_228*RLcp51_329-OMcp51_328*RLcp51_229;
    ORcp51_229 = -(OMcp51_128*RLcp51_329-OMcp51_328*RLcp51_129);
    ORcp51_329 = OMcp51_128*RLcp51_229-OMcp51_228*RLcp51_129;
    OPcp51_129 = OPcp51_128+ROcp51_128*qdd(29)+qd(29)*(OMcp51_228*ROcp51_328-OMcp51_328*ROcp51_228);
    OPcp51_229 = OPcp51_228+ROcp51_228*qdd(29)-qd(29)*(OMcp51_128*ROcp51_328-OMcp51_328*ROcp51_128);
    OPcp51_329 = OPcp51_328+ROcp51_328*qdd(29)+qd(29)*(OMcp51_128*ROcp51_228-OMcp51_228*ROcp51_128);
    RLcp51_130 = ROcp51_729*s.dpt(3,54);
    RLcp51_230 = ROcp51_829*s.dpt(3,54);
    RLcp51_330 = ROcp51_929*s.dpt(3,54);
    OMcp51_130 = OMcp51_129+ROcp51_729*qd(30);
    OMcp51_230 = OMcp51_229+ROcp51_829*qd(30);
    OMcp51_330 = OMcp51_329+ROcp51_929*qd(30);
    ORcp51_130 = OMcp51_229*RLcp51_330-OMcp51_329*RLcp51_230;
    ORcp51_230 = -(OMcp51_129*RLcp51_330-OMcp51_329*RLcp51_130);
    ORcp51_330 = OMcp51_129*RLcp51_230-OMcp51_229*RLcp51_130;
    OPcp51_130 = OPcp51_129+ROcp51_729*qdd(30)+qd(30)*(OMcp51_229*ROcp51_929-OMcp51_329*ROcp51_829);
    OPcp51_230 = OPcp51_229+ROcp51_829*qdd(30)-qd(30)*(OMcp51_129*ROcp51_929-OMcp51_329*ROcp51_729);
    OPcp51_330 = OPcp51_329+ROcp51_929*qdd(30)+qd(30)*(OMcp51_129*ROcp51_829-OMcp51_229*ROcp51_729);
    RLcp51_183 = ROcp51_729*s.dpt(3,56);
    RLcp51_283 = ROcp51_829*s.dpt(3,56);
    RLcp51_383 = ROcp51_929*s.dpt(3,56);
    POcp51_183 = RLcp51_121+RLcp51_123+RLcp51_128+RLcp51_129+RLcp51_130+RLcp51_183+q(1);
    POcp51_283 = RLcp51_221+RLcp51_223+RLcp51_228+RLcp51_229+RLcp51_230+RLcp51_283+q(2);
    POcp51_383 = RLcp51_321+RLcp51_323+RLcp51_328+RLcp51_329+RLcp51_330+RLcp51_383+q(3);
    JTcp51_283_4 = -(RLcp51_321+RLcp51_323+RLcp51_328+RLcp51_329+RLcp51_330+RLcp51_383);
    JTcp51_383_4 = RLcp51_221+RLcp51_223+RLcp51_228+RLcp51_229+RLcp51_230+RLcp51_283;
    JTcp51_183_5 = C4*(RLcp51_321+RLcp51_323+RLcp51_328+RLcp51_329+RLcp51_330+RLcp51_383)-S4*(RLcp51_221+RLcp51_223)-S4*(RLcp51_228+RLcp51_229)-S4*...
 (RLcp51_230+RLcp51_283);
    JTcp51_283_5 = S4*(RLcp51_121+RLcp51_123+RLcp51_128+RLcp51_129+RLcp51_130+RLcp51_183);
    JTcp51_383_5 = -C4*(RLcp51_121+RLcp51_123+RLcp51_128+RLcp51_129+RLcp51_130+RLcp51_183);
    JTcp51_183_6 = ROcp51_85*(RLcp51_321+RLcp51_323+RLcp51_328+RLcp51_329+RLcp51_330+RLcp51_383)-ROcp51_95*(RLcp51_221+RLcp51_223)-ROcp51_95*(...
 RLcp51_228+RLcp51_229)-ROcp51_95*(RLcp51_230+RLcp51_283);
    JTcp51_283_6 = RLcp51_183*ROcp51_95-RLcp51_330*S5-RLcp51_383*S5+ROcp51_95*(RLcp51_121+RLcp51_123+RLcp51_128+RLcp51_129+RLcp51_130)-S5*(...
 RLcp51_321+RLcp51_323)-S5*(RLcp51_328+RLcp51_329);
    JTcp51_383_6 = RLcp51_230*S5-ROcp51_85*(RLcp51_121+RLcp51_123+RLcp51_128+RLcp51_129+RLcp51_130)+S5*(RLcp51_221+RLcp51_223)+S5*(RLcp51_228+...
 RLcp51_229)-RLcp51_183*ROcp51_85+RLcp51_283*S5;
    JTcp51_183_7 = ROcp51_26*(RLcp51_323+RLcp51_328+RLcp51_329+RLcp51_330)-ROcp51_36*(RLcp51_223+RLcp51_228)-ROcp51_36*(RLcp51_229+RLcp51_230)-...
 RLcp51_283*ROcp51_36+RLcp51_383*ROcp51_26;
    JTcp51_283_7 = RLcp51_183*ROcp51_36-RLcp51_383*ROcp51_16-ROcp51_16*(RLcp51_323+RLcp51_328+RLcp51_329+RLcp51_330)+ROcp51_36*(RLcp51_123+...
 RLcp51_128)+ROcp51_36*(RLcp51_129+RLcp51_130);
    JTcp51_383_7 = ROcp51_16*(RLcp51_223+RLcp51_228+RLcp51_229+RLcp51_230)-ROcp51_26*(RLcp51_123+RLcp51_128)-ROcp51_26*(RLcp51_129+RLcp51_130)-...
 RLcp51_183*ROcp51_26+RLcp51_283*ROcp51_16;
    JTcp51_183_8 = ROcp51_521*(RLcp51_323+RLcp51_328+RLcp51_329+RLcp51_330)-ROcp51_621*(RLcp51_223+RLcp51_228)-ROcp51_621*(RLcp51_229+RLcp51_230)-...
 RLcp51_283*ROcp51_621+RLcp51_383*ROcp51_521;
    JTcp51_283_8 = RLcp51_183*ROcp51_621-RLcp51_383*ROcp51_421-ROcp51_421*(RLcp51_323+RLcp51_328+RLcp51_329+RLcp51_330)+ROcp51_621*(RLcp51_123+...
 RLcp51_128)+ROcp51_621*(RLcp51_129+RLcp51_130);
    JTcp51_383_8 = ROcp51_421*(RLcp51_223+RLcp51_228+RLcp51_229+RLcp51_230)-ROcp51_521*(RLcp51_123+RLcp51_128)-ROcp51_521*(RLcp51_129+RLcp51_130)-...
 RLcp51_183*ROcp51_521+RLcp51_283*ROcp51_421;
    JTcp51_183_9 = ROcp51_822*(RLcp51_328+RLcp51_329+RLcp51_330+RLcp51_383)-ROcp51_922*(RLcp51_228+RLcp51_229)-ROcp51_922*(RLcp51_230+RLcp51_283);
    JTcp51_283_9 = -(ROcp51_722*(RLcp51_328+RLcp51_329+RLcp51_330+RLcp51_383)-ROcp51_922*(RLcp51_128+RLcp51_129)-ROcp51_922*(RLcp51_130+RLcp51_183)...
 );
    JTcp51_383_9 = ROcp51_722*(RLcp51_228+RLcp51_229+RLcp51_230+RLcp51_283)-ROcp51_822*(RLcp51_128+RLcp51_129)-ROcp51_822*(RLcp51_130+RLcp51_183);
    JTcp51_183_10 = ROcp51_523*(RLcp51_329+RLcp51_330)-ROcp51_623*(RLcp51_229+RLcp51_230)-RLcp51_283*ROcp51_623+RLcp51_383*ROcp51_523;
    JTcp51_283_10 = RLcp51_183*ROcp51_623-RLcp51_383*ROcp51_423-ROcp51_423*(RLcp51_329+RLcp51_330)+ROcp51_623*(RLcp51_129+RLcp51_130);
    JTcp51_383_10 = ROcp51_423*(RLcp51_229+RLcp51_230)-ROcp51_523*(RLcp51_129+RLcp51_130)-RLcp51_183*ROcp51_523+RLcp51_283*ROcp51_423;
    JTcp51_183_11 = ROcp51_228*(RLcp51_330+RLcp51_383)-ROcp51_328*(RLcp51_230+RLcp51_283);
    JTcp51_283_11 = -(ROcp51_128*(RLcp51_330+RLcp51_383)-ROcp51_328*(RLcp51_130+RLcp51_183));
    JTcp51_383_11 = ROcp51_128*(RLcp51_230+RLcp51_283)-ROcp51_228*(RLcp51_130+RLcp51_183);
    JTcp51_183_12 = -(RLcp51_283*ROcp51_929-RLcp51_383*ROcp51_829);
    JTcp51_283_12 = RLcp51_183*ROcp51_929-RLcp51_383*ROcp51_729;
    JTcp51_383_12 = -(RLcp51_183*ROcp51_829-RLcp51_283*ROcp51_729);
    ORcp51_183 = OMcp51_230*RLcp51_383-OMcp51_330*RLcp51_283;
    ORcp51_283 = -(OMcp51_130*RLcp51_383-OMcp51_330*RLcp51_183);
    ORcp51_383 = OMcp51_130*RLcp51_283-OMcp51_230*RLcp51_183;
    VIcp51_183 = ORcp51_121+ORcp51_123+ORcp51_128+ORcp51_129+ORcp51_130+ORcp51_183+qd(1);
    VIcp51_283 = ORcp51_221+ORcp51_223+ORcp51_228+ORcp51_229+ORcp51_230+ORcp51_283+qd(2);
    VIcp51_383 = ORcp51_321+ORcp51_323+ORcp51_328+ORcp51_329+ORcp51_330+ORcp51_383+qd(3);
    ACcp51_183 = qdd(1)+OMcp51_222*ORcp51_323+OMcp51_223*ORcp51_328+OMcp51_228*ORcp51_329+OMcp51_229*ORcp51_330+OMcp51_230*ORcp51_383+OMcp51_26*...
 ORcp51_321-OMcp51_322*ORcp51_223-OMcp51_323*ORcp51_228-OMcp51_328*ORcp51_229-OMcp51_329*ORcp51_230-OMcp51_330*ORcp51_283-OMcp51_36*ORcp51_221+...
 OPcp51_222*RLcp51_323+OPcp51_223*RLcp51_328+OPcp51_228*RLcp51_329+OPcp51_229*RLcp51_330+OPcp51_230*RLcp51_383+OPcp51_26*RLcp51_321-OPcp51_322*...
 RLcp51_223-OPcp51_323*RLcp51_228-OPcp51_328*RLcp51_229-OPcp51_329*RLcp51_230-OPcp51_330*RLcp51_283-OPcp51_36*RLcp51_221;
    ACcp51_283 = qdd(2)-OMcp51_122*ORcp51_323-OMcp51_123*ORcp51_328-OMcp51_128*ORcp51_329-OMcp51_129*ORcp51_330-OMcp51_130*ORcp51_383-OMcp51_16*...
 ORcp51_321+OMcp51_322*ORcp51_123+OMcp51_323*ORcp51_128+OMcp51_328*ORcp51_129+OMcp51_329*ORcp51_130+OMcp51_330*ORcp51_183+OMcp51_36*ORcp51_121-...
 OPcp51_122*RLcp51_323-OPcp51_123*RLcp51_328-OPcp51_128*RLcp51_329-OPcp51_129*RLcp51_330-OPcp51_130*RLcp51_383-OPcp51_16*RLcp51_321+OPcp51_322*...
 RLcp51_123+OPcp51_323*RLcp51_128+OPcp51_328*RLcp51_129+OPcp51_329*RLcp51_130+OPcp51_330*RLcp51_183+OPcp51_36*RLcp51_121;
    ACcp51_383 = qdd(3)+OMcp51_122*ORcp51_223+OMcp51_123*ORcp51_228+OMcp51_128*ORcp51_229+OMcp51_129*ORcp51_230+OMcp51_130*ORcp51_283+OMcp51_16*...
 ORcp51_221-OMcp51_222*ORcp51_123-OMcp51_223*ORcp51_128-OMcp51_228*ORcp51_129-OMcp51_229*ORcp51_130-OMcp51_230*ORcp51_183-OMcp51_26*ORcp51_121+...
 OPcp51_122*RLcp51_223+OPcp51_123*RLcp51_228+OPcp51_128*RLcp51_229+OPcp51_129*RLcp51_230+OPcp51_130*RLcp51_283+OPcp51_16*RLcp51_221-OPcp51_222*...
 RLcp51_123-OPcp51_223*RLcp51_128-OPcp51_228*RLcp51_129-OPcp51_229*RLcp51_130-OPcp51_230*RLcp51_183-OPcp51_26*RLcp51_121;

% = = Block_1_0_0_52_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp51_183;
    sens.P(2) = POcp51_283;
    sens.P(3) = POcp51_383;
    sens.R(1,1) = ROcp51_130;
    sens.R(1,2) = ROcp51_230;
    sens.R(1,3) = ROcp51_330;
    sens.R(2,1) = ROcp51_430;
    sens.R(2,2) = ROcp51_530;
    sens.R(2,3) = ROcp51_630;
    sens.R(3,1) = ROcp51_729;
    sens.R(3,2) = ROcp51_829;
    sens.R(3,3) = ROcp51_929;
    sens.V(1) = VIcp51_183;
    sens.V(2) = VIcp51_283;
    sens.V(3) = VIcp51_383;
    sens.OM(1) = OMcp51_130;
    sens.OM(2) = OMcp51_230;
    sens.OM(3) = OMcp51_330;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp51_183_5;
    sens.J(1,6) = JTcp51_183_6;
    sens.J(1,21) = JTcp51_183_7;
    sens.J(1,22) = JTcp51_183_8;
    sens.J(1,23) = JTcp51_183_9;
    sens.J(1,28) = JTcp51_183_10;
    sens.J(1,29) = JTcp51_183_11;
    sens.J(1,30) = JTcp51_183_12;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp51_283_4;
    sens.J(2,5) = JTcp51_283_5;
    sens.J(2,6) = JTcp51_283_6;
    sens.J(2,21) = JTcp51_283_7;
    sens.J(2,22) = JTcp51_283_8;
    sens.J(2,23) = JTcp51_283_9;
    sens.J(2,28) = JTcp51_283_10;
    sens.J(2,29) = JTcp51_283_11;
    sens.J(2,30) = JTcp51_283_12;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp51_383_4;
    sens.J(3,5) = JTcp51_383_5;
    sens.J(3,6) = JTcp51_383_6;
    sens.J(3,21) = JTcp51_383_7;
    sens.J(3,22) = JTcp51_383_8;
    sens.J(3,23) = JTcp51_383_9;
    sens.J(3,28) = JTcp51_383_10;
    sens.J(3,29) = JTcp51_383_11;
    sens.J(3,30) = JTcp51_383_12;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,21) = ROcp51_16;
    sens.J(4,22) = ROcp51_421;
    sens.J(4,23) = ROcp51_722;
    sens.J(4,28) = ROcp51_423;
    sens.J(4,29) = ROcp51_128;
    sens.J(4,30) = ROcp51_729;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp51_85;
    sens.J(5,21) = ROcp51_26;
    sens.J(5,22) = ROcp51_521;
    sens.J(5,23) = ROcp51_822;
    sens.J(5,28) = ROcp51_523;
    sens.J(5,29) = ROcp51_228;
    sens.J(5,30) = ROcp51_829;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp51_95;
    sens.J(6,21) = ROcp51_36;
    sens.J(6,22) = ROcp51_621;
    sens.J(6,23) = ROcp51_922;
    sens.J(6,28) = ROcp51_623;
    sens.J(6,29) = ROcp51_328;
    sens.J(6,30) = ROcp51_929;
    sens.A(1) = ACcp51_183;
    sens.A(2) = ACcp51_283;
    sens.A(3) = ACcp51_383;
    sens.OMP(1) = OPcp51_130;
    sens.OMP(2) = OPcp51_230;
    sens.OMP(3) = OPcp51_330;
 
% 
case 53, 


% = = Block_1_0_0_53_0_1 = = 
 
% Sensor Kinematics 


    ROcp52_25 = S4*S5;
    ROcp52_35 = -C4*S5;
    ROcp52_85 = -S4*C5;
    ROcp52_95 = C4*C5;
    ROcp52_16 = C5*C6;
    ROcp52_26 = ROcp52_25*C6+C4*S6;
    ROcp52_36 = ROcp52_35*C6+S4*S6;
    ROcp52_46 = -C5*S6;
    ROcp52_56 = -(ROcp52_25*S6-C4*C6);
    ROcp52_66 = -(ROcp52_35*S6-S4*C6);
    OMcp52_25 = qd(5)*C4;
    OMcp52_35 = qd(5)*S4;
    OMcp52_16 = qd(4)+qd(6)*S5;
    OMcp52_26 = OMcp52_25+ROcp52_85*qd(6);
    OMcp52_36 = OMcp52_35+ROcp52_95*qd(6);
    OPcp52_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp52_26 = ROcp52_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp52_35*S5-ROcp52_95*qd(4));
    OPcp52_36 = ROcp52_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp52_25*S5-ROcp52_85*qd(4));

% = = Block_1_0_0_53_0_4 = = 
 
% Sensor Kinematics 


    ROcp52_421 = ROcp52_46*C21+S21*S5;
    ROcp52_521 = ROcp52_56*C21+ROcp52_85*S21;
    ROcp52_621 = ROcp52_66*C21+ROcp52_95*S21;
    ROcp52_721 = -(ROcp52_46*S21-C21*S5);
    ROcp52_821 = -(ROcp52_56*S21-ROcp52_85*C21);
    ROcp52_921 = -(ROcp52_66*S21-ROcp52_95*C21);
    ROcp52_122 = ROcp52_16*C22-ROcp52_721*S22;
    ROcp52_222 = ROcp52_26*C22-ROcp52_821*S22;
    ROcp52_322 = ROcp52_36*C22-ROcp52_921*S22;
    ROcp52_722 = ROcp52_16*S22+ROcp52_721*C22;
    ROcp52_822 = ROcp52_26*S22+ROcp52_821*C22;
    ROcp52_922 = ROcp52_36*S22+ROcp52_921*C22;
    ROcp52_123 = ROcp52_122*C23+ROcp52_421*S23;
    ROcp52_223 = ROcp52_222*C23+ROcp52_521*S23;
    ROcp52_323 = ROcp52_322*C23+ROcp52_621*S23;
    ROcp52_423 = -(ROcp52_122*S23-ROcp52_421*C23);
    ROcp52_523 = -(ROcp52_222*S23-ROcp52_521*C23);
    ROcp52_623 = -(ROcp52_322*S23-ROcp52_621*C23);
    RLcp52_121 = ROcp52_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp52_221 = ROcp52_26*s.dpt(1,3)+ROcp52_85*s.dpt(3,3);
    RLcp52_321 = ROcp52_36*s.dpt(1,3)+ROcp52_95*s.dpt(3,3);
    OMcp52_121 = OMcp52_16+ROcp52_16*qd(21);
    OMcp52_221 = OMcp52_26+ROcp52_26*qd(21);
    OMcp52_321 = OMcp52_36+ROcp52_36*qd(21);
    ORcp52_121 = OMcp52_26*RLcp52_321-OMcp52_36*RLcp52_221;
    ORcp52_221 = -(OMcp52_16*RLcp52_321-OMcp52_36*RLcp52_121);
    ORcp52_321 = OMcp52_16*RLcp52_221-OMcp52_26*RLcp52_121;
    OMcp52_122 = OMcp52_121+ROcp52_421*qd(22);
    OMcp52_222 = OMcp52_221+ROcp52_521*qd(22);
    OMcp52_322 = OMcp52_321+ROcp52_621*qd(22);
    OPcp52_122 = OPcp52_16+ROcp52_16*qdd(21)+ROcp52_421*qdd(22)+qd(21)*(OMcp52_26*ROcp52_36-OMcp52_36*ROcp52_26)+qd(22)*(OMcp52_221*ROcp52_621-...
 OMcp52_321*ROcp52_521);
    OPcp52_222 = OPcp52_26+ROcp52_26*qdd(21)+ROcp52_521*qdd(22)-qd(21)*(OMcp52_16*ROcp52_36-OMcp52_36*ROcp52_16)-qd(22)*(OMcp52_121*ROcp52_621-...
 OMcp52_321*ROcp52_421);
    OPcp52_322 = OPcp52_36+ROcp52_36*qdd(21)+ROcp52_621*qdd(22)+qd(21)*(OMcp52_16*ROcp52_26-OMcp52_26*ROcp52_16)+qd(22)*(OMcp52_121*ROcp52_521-...
 OMcp52_221*ROcp52_421);
    RLcp52_123 = ROcp52_722*s.dpt(3,37);
    RLcp52_223 = ROcp52_822*s.dpt(3,37);
    RLcp52_323 = ROcp52_922*s.dpt(3,37);
    OMcp52_123 = OMcp52_122+ROcp52_722*qd(23);
    OMcp52_223 = OMcp52_222+ROcp52_822*qd(23);
    OMcp52_323 = OMcp52_322+ROcp52_922*qd(23);
    ORcp52_123 = OMcp52_222*RLcp52_323-OMcp52_322*RLcp52_223;
    ORcp52_223 = -(OMcp52_122*RLcp52_323-OMcp52_322*RLcp52_123);
    ORcp52_323 = OMcp52_122*RLcp52_223-OMcp52_222*RLcp52_123;
    OPcp52_123 = OPcp52_122+ROcp52_722*qdd(23)+qd(23)*(OMcp52_222*ROcp52_922-OMcp52_322*ROcp52_822);
    OPcp52_223 = OPcp52_222+ROcp52_822*qdd(23)-qd(23)*(OMcp52_122*ROcp52_922-OMcp52_322*ROcp52_722);
    OPcp52_323 = OPcp52_322+ROcp52_922*qdd(23)+qd(23)*(OMcp52_122*ROcp52_822-OMcp52_222*ROcp52_722);

% = = Block_1_0_0_53_0_6 = = 
 
% Sensor Kinematics 


    ROcp52_128 = ROcp52_123*C28-ROcp52_722*S28;
    ROcp52_228 = ROcp52_223*C28-ROcp52_822*S28;
    ROcp52_328 = ROcp52_323*C28-ROcp52_922*S28;
    ROcp52_728 = ROcp52_123*S28+ROcp52_722*C28;
    ROcp52_828 = ROcp52_223*S28+ROcp52_822*C28;
    ROcp52_928 = ROcp52_323*S28+ROcp52_922*C28;
    ROcp52_429 = ROcp52_423*C29+ROcp52_728*S29;
    ROcp52_529 = ROcp52_523*C29+ROcp52_828*S29;
    ROcp52_629 = ROcp52_623*C29+ROcp52_928*S29;
    ROcp52_729 = -(ROcp52_423*S29-ROcp52_728*C29);
    ROcp52_829 = -(ROcp52_523*S29-ROcp52_828*C29);
    ROcp52_929 = -(ROcp52_623*S29-ROcp52_928*C29);
    ROcp52_130 = ROcp52_128*C30+ROcp52_429*S30;
    ROcp52_230 = ROcp52_228*C30+ROcp52_529*S30;
    ROcp52_330 = ROcp52_328*C30+ROcp52_629*S30;
    ROcp52_430 = -(ROcp52_128*S30-ROcp52_429*C30);
    ROcp52_530 = -(ROcp52_228*S30-ROcp52_529*C30);
    ROcp52_630 = -(ROcp52_328*S30-ROcp52_629*C30);
    ROcp52_131 = ROcp52_130*C31-ROcp52_729*S31;
    ROcp52_231 = ROcp52_230*C31-ROcp52_829*S31;
    ROcp52_331 = ROcp52_330*C31-ROcp52_929*S31;
    ROcp52_731 = ROcp52_130*S31+ROcp52_729*C31;
    ROcp52_831 = ROcp52_230*S31+ROcp52_829*C31;
    ROcp52_931 = ROcp52_330*S31+ROcp52_929*C31;
    RLcp52_128 = ROcp52_123*s.dpt(1,42)+ROcp52_423*s.dpt(2,42)+ROcp52_722*s.dpt(3,42);
    RLcp52_228 = ROcp52_223*s.dpt(1,42)+ROcp52_523*s.dpt(2,42)+ROcp52_822*s.dpt(3,42);
    RLcp52_328 = ROcp52_323*s.dpt(1,42)+ROcp52_623*s.dpt(2,42)+ROcp52_922*s.dpt(3,42);
    OMcp52_128 = OMcp52_123+ROcp52_423*qd(28);
    OMcp52_228 = OMcp52_223+ROcp52_523*qd(28);
    OMcp52_328 = OMcp52_323+ROcp52_623*qd(28);
    ORcp52_128 = OMcp52_223*RLcp52_328-OMcp52_323*RLcp52_228;
    ORcp52_228 = -(OMcp52_123*RLcp52_328-OMcp52_323*RLcp52_128);
    ORcp52_328 = OMcp52_123*RLcp52_228-OMcp52_223*RLcp52_128;
    OPcp52_128 = OPcp52_123+ROcp52_423*qdd(28)+qd(28)*(OMcp52_223*ROcp52_623-OMcp52_323*ROcp52_523);
    OPcp52_228 = OPcp52_223+ROcp52_523*qdd(28)-qd(28)*(OMcp52_123*ROcp52_623-OMcp52_323*ROcp52_423);
    OPcp52_328 = OPcp52_323+ROcp52_623*qdd(28)+qd(28)*(OMcp52_123*ROcp52_523-OMcp52_223*ROcp52_423);
    RLcp52_129 = ROcp52_423*s.dpt(2,52);
    RLcp52_229 = ROcp52_523*s.dpt(2,52);
    RLcp52_329 = ROcp52_623*s.dpt(2,52);
    OMcp52_129 = OMcp52_128+ROcp52_128*qd(29);
    OMcp52_229 = OMcp52_228+ROcp52_228*qd(29);
    OMcp52_329 = OMcp52_328+ROcp52_328*qd(29);
    ORcp52_129 = OMcp52_228*RLcp52_329-OMcp52_328*RLcp52_229;
    ORcp52_229 = -(OMcp52_128*RLcp52_329-OMcp52_328*RLcp52_129);
    ORcp52_329 = OMcp52_128*RLcp52_229-OMcp52_228*RLcp52_129;
    OPcp52_129 = OPcp52_128+ROcp52_128*qdd(29)+qd(29)*(OMcp52_228*ROcp52_328-OMcp52_328*ROcp52_228);
    OPcp52_229 = OPcp52_228+ROcp52_228*qdd(29)-qd(29)*(OMcp52_128*ROcp52_328-OMcp52_328*ROcp52_128);
    OPcp52_329 = OPcp52_328+ROcp52_328*qdd(29)+qd(29)*(OMcp52_128*ROcp52_228-OMcp52_228*ROcp52_128);
    RLcp52_130 = ROcp52_729*s.dpt(3,54);
    RLcp52_230 = ROcp52_829*s.dpt(3,54);
    RLcp52_330 = ROcp52_929*s.dpt(3,54);
    OMcp52_130 = OMcp52_129+ROcp52_729*qd(30);
    OMcp52_230 = OMcp52_229+ROcp52_829*qd(30);
    OMcp52_330 = OMcp52_329+ROcp52_929*qd(30);
    ORcp52_130 = OMcp52_229*RLcp52_330-OMcp52_329*RLcp52_230;
    ORcp52_230 = -(OMcp52_129*RLcp52_330-OMcp52_329*RLcp52_130);
    ORcp52_330 = OMcp52_129*RLcp52_230-OMcp52_229*RLcp52_130;
    OPcp52_130 = OPcp52_129+ROcp52_729*qdd(30)+qd(30)*(OMcp52_229*ROcp52_929-OMcp52_329*ROcp52_829);
    OPcp52_230 = OPcp52_229+ROcp52_829*qdd(30)-qd(30)*(OMcp52_129*ROcp52_929-OMcp52_329*ROcp52_729);
    OPcp52_330 = OPcp52_329+ROcp52_929*qdd(30)+qd(30)*(OMcp52_129*ROcp52_829-OMcp52_229*ROcp52_729);
    RLcp52_131 = ROcp52_729*s.dpt(3,56);
    RLcp52_231 = ROcp52_829*s.dpt(3,56);
    RLcp52_331 = ROcp52_929*s.dpt(3,56);
    OMcp52_131 = OMcp52_130+ROcp52_430*qd(31);
    OMcp52_231 = OMcp52_230+ROcp52_530*qd(31);
    OMcp52_331 = OMcp52_330+ROcp52_630*qd(31);
    ORcp52_131 = OMcp52_230*RLcp52_331-OMcp52_330*RLcp52_231;
    ORcp52_231 = -(OMcp52_130*RLcp52_331-OMcp52_330*RLcp52_131);
    ORcp52_331 = OMcp52_130*RLcp52_231-OMcp52_230*RLcp52_131;
    OPcp52_131 = OPcp52_130+ROcp52_430*qdd(31)+qd(31)*(OMcp52_230*ROcp52_630-OMcp52_330*ROcp52_530);
    OPcp52_231 = OPcp52_230+ROcp52_530*qdd(31)-qd(31)*(OMcp52_130*ROcp52_630-OMcp52_330*ROcp52_430);
    OPcp52_331 = OPcp52_330+ROcp52_630*qdd(31)+qd(31)*(OMcp52_130*ROcp52_530-OMcp52_230*ROcp52_430);
    RLcp52_184 = ROcp52_131*s.dpt(1,57)+ROcp52_430*s.dpt(2,57)+ROcp52_731*s.dpt(3,57);
    RLcp52_284 = ROcp52_231*s.dpt(1,57)+ROcp52_530*s.dpt(2,57)+ROcp52_831*s.dpt(3,57);
    RLcp52_384 = ROcp52_331*s.dpt(1,57)+ROcp52_630*s.dpt(2,57)+ROcp52_931*s.dpt(3,57);
    POcp52_184 = RLcp52_121+RLcp52_123+RLcp52_128+RLcp52_129+RLcp52_130+RLcp52_131+RLcp52_184+q(1);
    POcp52_284 = RLcp52_221+RLcp52_223+RLcp52_228+RLcp52_229+RLcp52_230+RLcp52_231+RLcp52_284+q(2);
    POcp52_384 = RLcp52_321+RLcp52_323+RLcp52_328+RLcp52_329+RLcp52_330+RLcp52_331+RLcp52_384+q(3);
    JTcp52_284_4 = -(RLcp52_321+RLcp52_323+RLcp52_328+RLcp52_329+RLcp52_330+RLcp52_331+RLcp52_384);
    JTcp52_384_4 = RLcp52_221+RLcp52_223+RLcp52_228+RLcp52_229+RLcp52_230+RLcp52_231+RLcp52_284;
    JTcp52_184_5 = C4*(RLcp52_321+RLcp52_323+RLcp52_328+RLcp52_329+RLcp52_330+RLcp52_331)-S4*(RLcp52_221+RLcp52_223)-S4*(RLcp52_228+RLcp52_229)-S4*...
 (RLcp52_230+RLcp52_231)-RLcp52_284*S4+RLcp52_384*C4;
    JTcp52_284_5 = S4*(RLcp52_121+RLcp52_123+RLcp52_128+RLcp52_129+RLcp52_130+RLcp52_131+RLcp52_184);
    JTcp52_384_5 = -C4*(RLcp52_121+RLcp52_123+RLcp52_128+RLcp52_129+RLcp52_130+RLcp52_131+RLcp52_184);
    JTcp52_184_6 = ROcp52_85*(RLcp52_321+RLcp52_323+RLcp52_328+RLcp52_329+RLcp52_330+RLcp52_331)-ROcp52_95*(RLcp52_221+RLcp52_223)-ROcp52_95*(...
 RLcp52_228+RLcp52_229)-ROcp52_95*(RLcp52_230+RLcp52_231)-RLcp52_284*ROcp52_95+RLcp52_384*ROcp52_85;
    JTcp52_284_6 = -(RLcp52_384*S5-ROcp52_95*(RLcp52_121+RLcp52_123+RLcp52_128+RLcp52_129+RLcp52_130+RLcp52_131+RLcp52_184)+S5*(RLcp52_321+...
 RLcp52_323)+S5*(RLcp52_328+RLcp52_329)+S5*(RLcp52_330+RLcp52_331));
    JTcp52_384_6 = RLcp52_284*S5-ROcp52_85*(RLcp52_121+RLcp52_123+RLcp52_128+RLcp52_129+RLcp52_130+RLcp52_131+RLcp52_184)+S5*(RLcp52_221+RLcp52_223...
 )+S5*(RLcp52_228+RLcp52_229)+S5*(RLcp52_230+RLcp52_231);
    JTcp52_184_7 = ROcp52_26*(RLcp52_323+RLcp52_328+RLcp52_329+RLcp52_330+RLcp52_331+RLcp52_384)-ROcp52_36*(RLcp52_223+RLcp52_228)-ROcp52_36*(...
 RLcp52_229+RLcp52_230)-ROcp52_36*(RLcp52_231+RLcp52_284);
    JTcp52_284_7 = -(ROcp52_16*(RLcp52_323+RLcp52_328+RLcp52_329+RLcp52_330+RLcp52_331+RLcp52_384)-ROcp52_36*(RLcp52_123+RLcp52_128)-ROcp52_36*(...
 RLcp52_129+RLcp52_130)-ROcp52_36*(RLcp52_131+RLcp52_184));
    JTcp52_384_7 = ROcp52_16*(RLcp52_223+RLcp52_228+RLcp52_229+RLcp52_230+RLcp52_231+RLcp52_284)-ROcp52_26*(RLcp52_123+RLcp52_128)-ROcp52_26*(...
 RLcp52_129+RLcp52_130)-ROcp52_26*(RLcp52_131+RLcp52_184);
    JTcp52_184_8 = ROcp52_521*(RLcp52_323+RLcp52_328+RLcp52_329+RLcp52_330+RLcp52_331+RLcp52_384)-ROcp52_621*(RLcp52_223+RLcp52_228)-ROcp52_621*(...
 RLcp52_229+RLcp52_230)-ROcp52_621*(RLcp52_231+RLcp52_284);
    JTcp52_284_8 = -(ROcp52_421*(RLcp52_323+RLcp52_328+RLcp52_329+RLcp52_330+RLcp52_331+RLcp52_384)-ROcp52_621*(RLcp52_123+RLcp52_128)-ROcp52_621*(...
 RLcp52_129+RLcp52_130)-ROcp52_621*(RLcp52_131+RLcp52_184));
    JTcp52_384_8 = ROcp52_421*(RLcp52_223+RLcp52_228+RLcp52_229+RLcp52_230+RLcp52_231+RLcp52_284)-ROcp52_521*(RLcp52_123+RLcp52_128)-ROcp52_521*(...
 RLcp52_129+RLcp52_130)-ROcp52_521*(RLcp52_131+RLcp52_184);
    JTcp52_184_9 = ROcp52_822*(RLcp52_328+RLcp52_329+RLcp52_330+RLcp52_331)-ROcp52_922*(RLcp52_228+RLcp52_229)-ROcp52_922*(RLcp52_230+RLcp52_231)-...
 RLcp52_284*ROcp52_922+RLcp52_384*ROcp52_822;
    JTcp52_284_9 = RLcp52_184*ROcp52_922-RLcp52_384*ROcp52_722-ROcp52_722*(RLcp52_328+RLcp52_329+RLcp52_330+RLcp52_331)+ROcp52_922*(RLcp52_128+...
 RLcp52_129)+ROcp52_922*(RLcp52_130+RLcp52_131);
    JTcp52_384_9 = ROcp52_722*(RLcp52_228+RLcp52_229+RLcp52_230+RLcp52_231)-ROcp52_822*(RLcp52_128+RLcp52_129)-ROcp52_822*(RLcp52_130+RLcp52_131)-...
 RLcp52_184*ROcp52_822+RLcp52_284*ROcp52_722;
    JTcp52_184_10 = ROcp52_523*(RLcp52_329+RLcp52_330+RLcp52_331+RLcp52_384)-ROcp52_623*(RLcp52_229+RLcp52_230)-ROcp52_623*(RLcp52_231+RLcp52_284);
    JTcp52_284_10 = -(ROcp52_423*(RLcp52_329+RLcp52_330+RLcp52_331+RLcp52_384)-ROcp52_623*(RLcp52_129+RLcp52_130)-ROcp52_623*(RLcp52_131+RLcp52_184...
 ));
    JTcp52_384_10 = ROcp52_423*(RLcp52_229+RLcp52_230+RLcp52_231+RLcp52_284)-ROcp52_523*(RLcp52_129+RLcp52_130)-ROcp52_523*(RLcp52_131+RLcp52_184);
    JTcp52_184_11 = ROcp52_228*(RLcp52_330+RLcp52_331)-ROcp52_328*(RLcp52_230+RLcp52_231)-RLcp52_284*ROcp52_328+RLcp52_384*ROcp52_228;
    JTcp52_284_11 = RLcp52_184*ROcp52_328-RLcp52_384*ROcp52_128-ROcp52_128*(RLcp52_330+RLcp52_331)+ROcp52_328*(RLcp52_130+RLcp52_131);
    JTcp52_384_11 = ROcp52_128*(RLcp52_230+RLcp52_231)-ROcp52_228*(RLcp52_130+RLcp52_131)-RLcp52_184*ROcp52_228+RLcp52_284*ROcp52_128;
    JTcp52_184_12 = ROcp52_829*(RLcp52_331+RLcp52_384)-ROcp52_929*(RLcp52_231+RLcp52_284);
    JTcp52_284_12 = -(ROcp52_729*(RLcp52_331+RLcp52_384)-ROcp52_929*(RLcp52_131+RLcp52_184));
    JTcp52_384_12 = ROcp52_729*(RLcp52_231+RLcp52_284)-ROcp52_829*(RLcp52_131+RLcp52_184);
    JTcp52_184_13 = -(RLcp52_284*ROcp52_630-RLcp52_384*ROcp52_530);
    JTcp52_284_13 = RLcp52_184*ROcp52_630-RLcp52_384*ROcp52_430;
    JTcp52_384_13 = -(RLcp52_184*ROcp52_530-RLcp52_284*ROcp52_430);
    ORcp52_184 = OMcp52_231*RLcp52_384-OMcp52_331*RLcp52_284;
    ORcp52_284 = -(OMcp52_131*RLcp52_384-OMcp52_331*RLcp52_184);
    ORcp52_384 = OMcp52_131*RLcp52_284-OMcp52_231*RLcp52_184;
    VIcp52_184 = ORcp52_121+ORcp52_123+ORcp52_128+ORcp52_129+ORcp52_130+ORcp52_131+ORcp52_184+qd(1);
    VIcp52_284 = ORcp52_221+ORcp52_223+ORcp52_228+ORcp52_229+ORcp52_230+ORcp52_231+ORcp52_284+qd(2);
    VIcp52_384 = ORcp52_321+ORcp52_323+ORcp52_328+ORcp52_329+ORcp52_330+ORcp52_331+ORcp52_384+qd(3);
    ACcp52_184 = qdd(1)+OMcp52_222*ORcp52_323+OMcp52_223*ORcp52_328+OMcp52_228*ORcp52_329+OMcp52_229*ORcp52_330+OMcp52_230*ORcp52_331+OMcp52_231*...
 ORcp52_384+OMcp52_26*ORcp52_321-OMcp52_322*ORcp52_223-OMcp52_323*ORcp52_228-OMcp52_328*ORcp52_229-OMcp52_329*ORcp52_230-OMcp52_330*ORcp52_231-...
 OMcp52_331*ORcp52_284-OMcp52_36*ORcp52_221+OPcp52_222*RLcp52_323+OPcp52_223*RLcp52_328+OPcp52_228*RLcp52_329+OPcp52_229*RLcp52_330+OPcp52_230*...
 RLcp52_331+OPcp52_231*RLcp52_384+OPcp52_26*RLcp52_321-OPcp52_322*RLcp52_223-OPcp52_323*RLcp52_228-OPcp52_328*RLcp52_229-OPcp52_329*RLcp52_230-...
 OPcp52_330*RLcp52_231-OPcp52_331*RLcp52_284-OPcp52_36*RLcp52_221;
    ACcp52_284 = qdd(2)-OMcp52_122*ORcp52_323-OMcp52_123*ORcp52_328-OMcp52_128*ORcp52_329-OMcp52_129*ORcp52_330-OMcp52_130*ORcp52_331-OMcp52_131*...
 ORcp52_384-OMcp52_16*ORcp52_321+OMcp52_322*ORcp52_123+OMcp52_323*ORcp52_128+OMcp52_328*ORcp52_129+OMcp52_329*ORcp52_130+OMcp52_330*ORcp52_131+...
 OMcp52_331*ORcp52_184+OMcp52_36*ORcp52_121-OPcp52_122*RLcp52_323-OPcp52_123*RLcp52_328-OPcp52_128*RLcp52_329-OPcp52_129*RLcp52_330-OPcp52_130*...
 RLcp52_331-OPcp52_131*RLcp52_384-OPcp52_16*RLcp52_321+OPcp52_322*RLcp52_123+OPcp52_323*RLcp52_128+OPcp52_328*RLcp52_129+OPcp52_329*RLcp52_130+...
 OPcp52_330*RLcp52_131+OPcp52_331*RLcp52_184+OPcp52_36*RLcp52_121;
    ACcp52_384 = qdd(3)+OMcp52_122*ORcp52_223+OMcp52_123*ORcp52_228+OMcp52_128*ORcp52_229+OMcp52_129*ORcp52_230+OMcp52_130*ORcp52_231+OMcp52_131*...
 ORcp52_284+OMcp52_16*ORcp52_221-OMcp52_222*ORcp52_123-OMcp52_223*ORcp52_128-OMcp52_228*ORcp52_129-OMcp52_229*ORcp52_130-OMcp52_230*ORcp52_131-...
 OMcp52_231*ORcp52_184-OMcp52_26*ORcp52_121+OPcp52_122*RLcp52_223+OPcp52_123*RLcp52_228+OPcp52_128*RLcp52_229+OPcp52_129*RLcp52_230+OPcp52_130*...
 RLcp52_231+OPcp52_131*RLcp52_284+OPcp52_16*RLcp52_221-OPcp52_222*RLcp52_123-OPcp52_223*RLcp52_128-OPcp52_228*RLcp52_129-OPcp52_229*RLcp52_130-...
 OPcp52_230*RLcp52_131-OPcp52_231*RLcp52_184-OPcp52_26*RLcp52_121;

% = = Block_1_0_0_53_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp52_184;
    sens.P(2) = POcp52_284;
    sens.P(3) = POcp52_384;
    sens.R(1,1) = ROcp52_131;
    sens.R(1,2) = ROcp52_231;
    sens.R(1,3) = ROcp52_331;
    sens.R(2,1) = ROcp52_430;
    sens.R(2,2) = ROcp52_530;
    sens.R(2,3) = ROcp52_630;
    sens.R(3,1) = ROcp52_731;
    sens.R(3,2) = ROcp52_831;
    sens.R(3,3) = ROcp52_931;
    sens.V(1) = VIcp52_184;
    sens.V(2) = VIcp52_284;
    sens.V(3) = VIcp52_384;
    sens.OM(1) = OMcp52_131;
    sens.OM(2) = OMcp52_231;
    sens.OM(3) = OMcp52_331;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp52_184_5;
    sens.J(1,6) = JTcp52_184_6;
    sens.J(1,21) = JTcp52_184_7;
    sens.J(1,22) = JTcp52_184_8;
    sens.J(1,23) = JTcp52_184_9;
    sens.J(1,28) = JTcp52_184_10;
    sens.J(1,29) = JTcp52_184_11;
    sens.J(1,30) = JTcp52_184_12;
    sens.J(1,31) = JTcp52_184_13;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp52_284_4;
    sens.J(2,5) = JTcp52_284_5;
    sens.J(2,6) = JTcp52_284_6;
    sens.J(2,21) = JTcp52_284_7;
    sens.J(2,22) = JTcp52_284_8;
    sens.J(2,23) = JTcp52_284_9;
    sens.J(2,28) = JTcp52_284_10;
    sens.J(2,29) = JTcp52_284_11;
    sens.J(2,30) = JTcp52_284_12;
    sens.J(2,31) = JTcp52_284_13;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp52_384_4;
    sens.J(3,5) = JTcp52_384_5;
    sens.J(3,6) = JTcp52_384_6;
    sens.J(3,21) = JTcp52_384_7;
    sens.J(3,22) = JTcp52_384_8;
    sens.J(3,23) = JTcp52_384_9;
    sens.J(3,28) = JTcp52_384_10;
    sens.J(3,29) = JTcp52_384_11;
    sens.J(3,30) = JTcp52_384_12;
    sens.J(3,31) = JTcp52_384_13;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,21) = ROcp52_16;
    sens.J(4,22) = ROcp52_421;
    sens.J(4,23) = ROcp52_722;
    sens.J(4,28) = ROcp52_423;
    sens.J(4,29) = ROcp52_128;
    sens.J(4,30) = ROcp52_729;
    sens.J(4,31) = ROcp52_430;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp52_85;
    sens.J(5,21) = ROcp52_26;
    sens.J(5,22) = ROcp52_521;
    sens.J(5,23) = ROcp52_822;
    sens.J(5,28) = ROcp52_523;
    sens.J(5,29) = ROcp52_228;
    sens.J(5,30) = ROcp52_829;
    sens.J(5,31) = ROcp52_530;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp52_95;
    sens.J(6,21) = ROcp52_36;
    sens.J(6,22) = ROcp52_621;
    sens.J(6,23) = ROcp52_922;
    sens.J(6,28) = ROcp52_623;
    sens.J(6,29) = ROcp52_328;
    sens.J(6,30) = ROcp52_929;
    sens.J(6,31) = ROcp52_630;
    sens.A(1) = ACcp52_184;
    sens.A(2) = ACcp52_284;
    sens.A(3) = ACcp52_384;
    sens.OMP(1) = OPcp52_131;
    sens.OMP(2) = OPcp52_231;
    sens.OMP(3) = OPcp52_331;
 
% 
case 54, 


% = = Block_1_0_0_54_0_1 = = 
 
% Sensor Kinematics 


    ROcp53_25 = S4*S5;
    ROcp53_35 = -C4*S5;
    ROcp53_85 = -S4*C5;
    ROcp53_95 = C4*C5;
    ROcp53_16 = C5*C6;
    ROcp53_26 = ROcp53_25*C6+C4*S6;
    ROcp53_36 = ROcp53_35*C6+S4*S6;
    ROcp53_46 = -C5*S6;
    ROcp53_56 = -(ROcp53_25*S6-C4*C6);
    ROcp53_66 = -(ROcp53_35*S6-S4*C6);
    OMcp53_25 = qd(5)*C4;
    OMcp53_35 = qd(5)*S4;
    OMcp53_16 = qd(4)+qd(6)*S5;
    OMcp53_26 = OMcp53_25+ROcp53_85*qd(6);
    OMcp53_36 = OMcp53_35+ROcp53_95*qd(6);
    OPcp53_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp53_26 = ROcp53_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp53_35*S5-ROcp53_95*qd(4));
    OPcp53_36 = ROcp53_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp53_25*S5-ROcp53_85*qd(4));

% = = Block_1_0_0_54_0_4 = = 
 
% Sensor Kinematics 


    ROcp53_421 = ROcp53_46*C21+S21*S5;
    ROcp53_521 = ROcp53_56*C21+ROcp53_85*S21;
    ROcp53_621 = ROcp53_66*C21+ROcp53_95*S21;
    ROcp53_721 = -(ROcp53_46*S21-C21*S5);
    ROcp53_821 = -(ROcp53_56*S21-ROcp53_85*C21);
    ROcp53_921 = -(ROcp53_66*S21-ROcp53_95*C21);
    ROcp53_122 = ROcp53_16*C22-ROcp53_721*S22;
    ROcp53_222 = ROcp53_26*C22-ROcp53_821*S22;
    ROcp53_322 = ROcp53_36*C22-ROcp53_921*S22;
    ROcp53_722 = ROcp53_16*S22+ROcp53_721*C22;
    ROcp53_822 = ROcp53_26*S22+ROcp53_821*C22;
    ROcp53_922 = ROcp53_36*S22+ROcp53_921*C22;
    ROcp53_123 = ROcp53_122*C23+ROcp53_421*S23;
    ROcp53_223 = ROcp53_222*C23+ROcp53_521*S23;
    ROcp53_323 = ROcp53_322*C23+ROcp53_621*S23;
    ROcp53_423 = -(ROcp53_122*S23-ROcp53_421*C23);
    ROcp53_523 = -(ROcp53_222*S23-ROcp53_521*C23);
    ROcp53_623 = -(ROcp53_322*S23-ROcp53_621*C23);
    RLcp53_121 = ROcp53_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp53_221 = ROcp53_26*s.dpt(1,3)+ROcp53_85*s.dpt(3,3);
    RLcp53_321 = ROcp53_36*s.dpt(1,3)+ROcp53_95*s.dpt(3,3);
    OMcp53_121 = OMcp53_16+ROcp53_16*qd(21);
    OMcp53_221 = OMcp53_26+ROcp53_26*qd(21);
    OMcp53_321 = OMcp53_36+ROcp53_36*qd(21);
    ORcp53_121 = OMcp53_26*RLcp53_321-OMcp53_36*RLcp53_221;
    ORcp53_221 = -(OMcp53_16*RLcp53_321-OMcp53_36*RLcp53_121);
    ORcp53_321 = OMcp53_16*RLcp53_221-OMcp53_26*RLcp53_121;
    OMcp53_122 = OMcp53_121+ROcp53_421*qd(22);
    OMcp53_222 = OMcp53_221+ROcp53_521*qd(22);
    OMcp53_322 = OMcp53_321+ROcp53_621*qd(22);
    OPcp53_122 = OPcp53_16+ROcp53_16*qdd(21)+ROcp53_421*qdd(22)+qd(21)*(OMcp53_26*ROcp53_36-OMcp53_36*ROcp53_26)+qd(22)*(OMcp53_221*ROcp53_621-...
 OMcp53_321*ROcp53_521);
    OPcp53_222 = OPcp53_26+ROcp53_26*qdd(21)+ROcp53_521*qdd(22)-qd(21)*(OMcp53_16*ROcp53_36-OMcp53_36*ROcp53_16)-qd(22)*(OMcp53_121*ROcp53_621-...
 OMcp53_321*ROcp53_421);
    OPcp53_322 = OPcp53_36+ROcp53_36*qdd(21)+ROcp53_621*qdd(22)+qd(21)*(OMcp53_16*ROcp53_26-OMcp53_26*ROcp53_16)+qd(22)*(OMcp53_121*ROcp53_521-...
 OMcp53_221*ROcp53_421);
    RLcp53_123 = ROcp53_722*s.dpt(3,37);
    RLcp53_223 = ROcp53_822*s.dpt(3,37);
    RLcp53_323 = ROcp53_922*s.dpt(3,37);
    OMcp53_123 = OMcp53_122+ROcp53_722*qd(23);
    OMcp53_223 = OMcp53_222+ROcp53_822*qd(23);
    OMcp53_323 = OMcp53_322+ROcp53_922*qd(23);
    ORcp53_123 = OMcp53_222*RLcp53_323-OMcp53_322*RLcp53_223;
    ORcp53_223 = -(OMcp53_122*RLcp53_323-OMcp53_322*RLcp53_123);
    ORcp53_323 = OMcp53_122*RLcp53_223-OMcp53_222*RLcp53_123;
    OPcp53_123 = OPcp53_122+ROcp53_722*qdd(23)+qd(23)*(OMcp53_222*ROcp53_922-OMcp53_322*ROcp53_822);
    OPcp53_223 = OPcp53_222+ROcp53_822*qdd(23)-qd(23)*(OMcp53_122*ROcp53_922-OMcp53_322*ROcp53_722);
    OPcp53_323 = OPcp53_322+ROcp53_922*qdd(23)+qd(23)*(OMcp53_122*ROcp53_822-OMcp53_222*ROcp53_722);

% = = Block_1_0_0_54_0_6 = = 
 
% Sensor Kinematics 


    ROcp53_128 = ROcp53_123*C28-ROcp53_722*S28;
    ROcp53_228 = ROcp53_223*C28-ROcp53_822*S28;
    ROcp53_328 = ROcp53_323*C28-ROcp53_922*S28;
    ROcp53_728 = ROcp53_123*S28+ROcp53_722*C28;
    ROcp53_828 = ROcp53_223*S28+ROcp53_822*C28;
    ROcp53_928 = ROcp53_323*S28+ROcp53_922*C28;
    ROcp53_429 = ROcp53_423*C29+ROcp53_728*S29;
    ROcp53_529 = ROcp53_523*C29+ROcp53_828*S29;
    ROcp53_629 = ROcp53_623*C29+ROcp53_928*S29;
    ROcp53_729 = -(ROcp53_423*S29-ROcp53_728*C29);
    ROcp53_829 = -(ROcp53_523*S29-ROcp53_828*C29);
    ROcp53_929 = -(ROcp53_623*S29-ROcp53_928*C29);
    ROcp53_130 = ROcp53_128*C30+ROcp53_429*S30;
    ROcp53_230 = ROcp53_228*C30+ROcp53_529*S30;
    ROcp53_330 = ROcp53_328*C30+ROcp53_629*S30;
    ROcp53_430 = -(ROcp53_128*S30-ROcp53_429*C30);
    ROcp53_530 = -(ROcp53_228*S30-ROcp53_529*C30);
    ROcp53_630 = -(ROcp53_328*S30-ROcp53_629*C30);
    ROcp53_131 = ROcp53_130*C31-ROcp53_729*S31;
    ROcp53_231 = ROcp53_230*C31-ROcp53_829*S31;
    ROcp53_331 = ROcp53_330*C31-ROcp53_929*S31;
    ROcp53_731 = ROcp53_130*S31+ROcp53_729*C31;
    ROcp53_831 = ROcp53_230*S31+ROcp53_829*C31;
    ROcp53_931 = ROcp53_330*S31+ROcp53_929*C31;
    RLcp53_128 = ROcp53_123*s.dpt(1,42)+ROcp53_423*s.dpt(2,42)+ROcp53_722*s.dpt(3,42);
    RLcp53_228 = ROcp53_223*s.dpt(1,42)+ROcp53_523*s.dpt(2,42)+ROcp53_822*s.dpt(3,42);
    RLcp53_328 = ROcp53_323*s.dpt(1,42)+ROcp53_623*s.dpt(2,42)+ROcp53_922*s.dpt(3,42);
    OMcp53_128 = OMcp53_123+ROcp53_423*qd(28);
    OMcp53_228 = OMcp53_223+ROcp53_523*qd(28);
    OMcp53_328 = OMcp53_323+ROcp53_623*qd(28);
    ORcp53_128 = OMcp53_223*RLcp53_328-OMcp53_323*RLcp53_228;
    ORcp53_228 = -(OMcp53_123*RLcp53_328-OMcp53_323*RLcp53_128);
    ORcp53_328 = OMcp53_123*RLcp53_228-OMcp53_223*RLcp53_128;
    OPcp53_128 = OPcp53_123+ROcp53_423*qdd(28)+qd(28)*(OMcp53_223*ROcp53_623-OMcp53_323*ROcp53_523);
    OPcp53_228 = OPcp53_223+ROcp53_523*qdd(28)-qd(28)*(OMcp53_123*ROcp53_623-OMcp53_323*ROcp53_423);
    OPcp53_328 = OPcp53_323+ROcp53_623*qdd(28)+qd(28)*(OMcp53_123*ROcp53_523-OMcp53_223*ROcp53_423);
    RLcp53_129 = ROcp53_423*s.dpt(2,52);
    RLcp53_229 = ROcp53_523*s.dpt(2,52);
    RLcp53_329 = ROcp53_623*s.dpt(2,52);
    OMcp53_129 = OMcp53_128+ROcp53_128*qd(29);
    OMcp53_229 = OMcp53_228+ROcp53_228*qd(29);
    OMcp53_329 = OMcp53_328+ROcp53_328*qd(29);
    ORcp53_129 = OMcp53_228*RLcp53_329-OMcp53_328*RLcp53_229;
    ORcp53_229 = -(OMcp53_128*RLcp53_329-OMcp53_328*RLcp53_129);
    ORcp53_329 = OMcp53_128*RLcp53_229-OMcp53_228*RLcp53_129;
    OPcp53_129 = OPcp53_128+ROcp53_128*qdd(29)+qd(29)*(OMcp53_228*ROcp53_328-OMcp53_328*ROcp53_228);
    OPcp53_229 = OPcp53_228+ROcp53_228*qdd(29)-qd(29)*(OMcp53_128*ROcp53_328-OMcp53_328*ROcp53_128);
    OPcp53_329 = OPcp53_328+ROcp53_328*qdd(29)+qd(29)*(OMcp53_128*ROcp53_228-OMcp53_228*ROcp53_128);
    RLcp53_130 = ROcp53_729*s.dpt(3,54);
    RLcp53_230 = ROcp53_829*s.dpt(3,54);
    RLcp53_330 = ROcp53_929*s.dpt(3,54);
    OMcp53_130 = OMcp53_129+ROcp53_729*qd(30);
    OMcp53_230 = OMcp53_229+ROcp53_829*qd(30);
    OMcp53_330 = OMcp53_329+ROcp53_929*qd(30);
    ORcp53_130 = OMcp53_229*RLcp53_330-OMcp53_329*RLcp53_230;
    ORcp53_230 = -(OMcp53_129*RLcp53_330-OMcp53_329*RLcp53_130);
    ORcp53_330 = OMcp53_129*RLcp53_230-OMcp53_229*RLcp53_130;
    OPcp53_130 = OPcp53_129+ROcp53_729*qdd(30)+qd(30)*(OMcp53_229*ROcp53_929-OMcp53_329*ROcp53_829);
    OPcp53_230 = OPcp53_229+ROcp53_829*qdd(30)-qd(30)*(OMcp53_129*ROcp53_929-OMcp53_329*ROcp53_729);
    OPcp53_330 = OPcp53_329+ROcp53_929*qdd(30)+qd(30)*(OMcp53_129*ROcp53_829-OMcp53_229*ROcp53_729);
    RLcp53_131 = ROcp53_729*s.dpt(3,56);
    RLcp53_231 = ROcp53_829*s.dpt(3,56);
    RLcp53_331 = ROcp53_929*s.dpt(3,56);
    OMcp53_131 = OMcp53_130+ROcp53_430*qd(31);
    OMcp53_231 = OMcp53_230+ROcp53_530*qd(31);
    OMcp53_331 = OMcp53_330+ROcp53_630*qd(31);
    ORcp53_131 = OMcp53_230*RLcp53_331-OMcp53_330*RLcp53_231;
    ORcp53_231 = -(OMcp53_130*RLcp53_331-OMcp53_330*RLcp53_131);
    ORcp53_331 = OMcp53_130*RLcp53_231-OMcp53_230*RLcp53_131;
    OPcp53_131 = OPcp53_130+ROcp53_430*qdd(31)+qd(31)*(OMcp53_230*ROcp53_630-OMcp53_330*ROcp53_530);
    OPcp53_231 = OPcp53_230+ROcp53_530*qdd(31)-qd(31)*(OMcp53_130*ROcp53_630-OMcp53_330*ROcp53_430);
    OPcp53_331 = OPcp53_330+ROcp53_630*qdd(31)+qd(31)*(OMcp53_130*ROcp53_530-OMcp53_230*ROcp53_430);
    RLcp53_185 = ROcp53_731*s.dpt(3,58);
    RLcp53_285 = ROcp53_831*s.dpt(3,58);
    RLcp53_385 = ROcp53_931*s.dpt(3,58);
    POcp53_185 = RLcp53_121+RLcp53_123+RLcp53_128+RLcp53_129+RLcp53_130+RLcp53_131+RLcp53_185+q(1);
    POcp53_285 = RLcp53_221+RLcp53_223+RLcp53_228+RLcp53_229+RLcp53_230+RLcp53_231+RLcp53_285+q(2);
    POcp53_385 = RLcp53_321+RLcp53_323+RLcp53_328+RLcp53_329+RLcp53_330+RLcp53_331+RLcp53_385+q(3);
    JTcp53_285_4 = -(RLcp53_321+RLcp53_323+RLcp53_328+RLcp53_329+RLcp53_330+RLcp53_331+RLcp53_385);
    JTcp53_385_4 = RLcp53_221+RLcp53_223+RLcp53_228+RLcp53_229+RLcp53_230+RLcp53_231+RLcp53_285;
    JTcp53_185_5 = C4*(RLcp53_321+RLcp53_323+RLcp53_328+RLcp53_329+RLcp53_330+RLcp53_331)-S4*(RLcp53_221+RLcp53_223)-S4*(RLcp53_228+RLcp53_229)-S4*...
 (RLcp53_230+RLcp53_231)-RLcp53_285*S4+RLcp53_385*C4;
    JTcp53_285_5 = S4*(RLcp53_121+RLcp53_123+RLcp53_128+RLcp53_129+RLcp53_130+RLcp53_131+RLcp53_185);
    JTcp53_385_5 = -C4*(RLcp53_121+RLcp53_123+RLcp53_128+RLcp53_129+RLcp53_130+RLcp53_131+RLcp53_185);
    JTcp53_185_6 = ROcp53_85*(RLcp53_321+RLcp53_323+RLcp53_328+RLcp53_329+RLcp53_330+RLcp53_331)-ROcp53_95*(RLcp53_221+RLcp53_223)-ROcp53_95*(...
 RLcp53_228+RLcp53_229)-ROcp53_95*(RLcp53_230+RLcp53_231)-RLcp53_285*ROcp53_95+RLcp53_385*ROcp53_85;
    JTcp53_285_6 = -(RLcp53_385*S5-ROcp53_95*(RLcp53_121+RLcp53_123+RLcp53_128+RLcp53_129+RLcp53_130+RLcp53_131+RLcp53_185)+S5*(RLcp53_321+...
 RLcp53_323)+S5*(RLcp53_328+RLcp53_329)+S5*(RLcp53_330+RLcp53_331));
    JTcp53_385_6 = RLcp53_285*S5-ROcp53_85*(RLcp53_121+RLcp53_123+RLcp53_128+RLcp53_129+RLcp53_130+RLcp53_131+RLcp53_185)+S5*(RLcp53_221+RLcp53_223...
 )+S5*(RLcp53_228+RLcp53_229)+S5*(RLcp53_230+RLcp53_231);
    JTcp53_185_7 = ROcp53_26*(RLcp53_323+RLcp53_328+RLcp53_329+RLcp53_330+RLcp53_331+RLcp53_385)-ROcp53_36*(RLcp53_223+RLcp53_228)-ROcp53_36*(...
 RLcp53_229+RLcp53_230)-ROcp53_36*(RLcp53_231+RLcp53_285);
    JTcp53_285_7 = -(ROcp53_16*(RLcp53_323+RLcp53_328+RLcp53_329+RLcp53_330+RLcp53_331+RLcp53_385)-ROcp53_36*(RLcp53_123+RLcp53_128)-ROcp53_36*(...
 RLcp53_129+RLcp53_130)-ROcp53_36*(RLcp53_131+RLcp53_185));
    JTcp53_385_7 = ROcp53_16*(RLcp53_223+RLcp53_228+RLcp53_229+RLcp53_230+RLcp53_231+RLcp53_285)-ROcp53_26*(RLcp53_123+RLcp53_128)-ROcp53_26*(...
 RLcp53_129+RLcp53_130)-ROcp53_26*(RLcp53_131+RLcp53_185);
    JTcp53_185_8 = ROcp53_521*(RLcp53_323+RLcp53_328+RLcp53_329+RLcp53_330+RLcp53_331+RLcp53_385)-ROcp53_621*(RLcp53_223+RLcp53_228)-ROcp53_621*(...
 RLcp53_229+RLcp53_230)-ROcp53_621*(RLcp53_231+RLcp53_285);
    JTcp53_285_8 = -(ROcp53_421*(RLcp53_323+RLcp53_328+RLcp53_329+RLcp53_330+RLcp53_331+RLcp53_385)-ROcp53_621*(RLcp53_123+RLcp53_128)-ROcp53_621*(...
 RLcp53_129+RLcp53_130)-ROcp53_621*(RLcp53_131+RLcp53_185));
    JTcp53_385_8 = ROcp53_421*(RLcp53_223+RLcp53_228+RLcp53_229+RLcp53_230+RLcp53_231+RLcp53_285)-ROcp53_521*(RLcp53_123+RLcp53_128)-ROcp53_521*(...
 RLcp53_129+RLcp53_130)-ROcp53_521*(RLcp53_131+RLcp53_185);
    JTcp53_185_9 = ROcp53_822*(RLcp53_328+RLcp53_329+RLcp53_330+RLcp53_331)-ROcp53_922*(RLcp53_228+RLcp53_229)-ROcp53_922*(RLcp53_230+RLcp53_231)-...
 RLcp53_285*ROcp53_922+RLcp53_385*ROcp53_822;
    JTcp53_285_9 = RLcp53_185*ROcp53_922-RLcp53_385*ROcp53_722-ROcp53_722*(RLcp53_328+RLcp53_329+RLcp53_330+RLcp53_331)+ROcp53_922*(RLcp53_128+...
 RLcp53_129)+ROcp53_922*(RLcp53_130+RLcp53_131);
    JTcp53_385_9 = ROcp53_722*(RLcp53_228+RLcp53_229+RLcp53_230+RLcp53_231)-ROcp53_822*(RLcp53_128+RLcp53_129)-ROcp53_822*(RLcp53_130+RLcp53_131)-...
 RLcp53_185*ROcp53_822+RLcp53_285*ROcp53_722;
    JTcp53_185_10 = ROcp53_523*(RLcp53_329+RLcp53_330+RLcp53_331+RLcp53_385)-ROcp53_623*(RLcp53_229+RLcp53_230)-ROcp53_623*(RLcp53_231+RLcp53_285);
    JTcp53_285_10 = -(ROcp53_423*(RLcp53_329+RLcp53_330+RLcp53_331+RLcp53_385)-ROcp53_623*(RLcp53_129+RLcp53_130)-ROcp53_623*(RLcp53_131+RLcp53_185...
 ));
    JTcp53_385_10 = ROcp53_423*(RLcp53_229+RLcp53_230+RLcp53_231+RLcp53_285)-ROcp53_523*(RLcp53_129+RLcp53_130)-ROcp53_523*(RLcp53_131+RLcp53_185);
    JTcp53_185_11 = ROcp53_228*(RLcp53_330+RLcp53_331)-ROcp53_328*(RLcp53_230+RLcp53_231)-RLcp53_285*ROcp53_328+RLcp53_385*ROcp53_228;
    JTcp53_285_11 = RLcp53_185*ROcp53_328-RLcp53_385*ROcp53_128-ROcp53_128*(RLcp53_330+RLcp53_331)+ROcp53_328*(RLcp53_130+RLcp53_131);
    JTcp53_385_11 = ROcp53_128*(RLcp53_230+RLcp53_231)-ROcp53_228*(RLcp53_130+RLcp53_131)-RLcp53_185*ROcp53_228+RLcp53_285*ROcp53_128;
    JTcp53_185_12 = ROcp53_829*(RLcp53_331+RLcp53_385)-ROcp53_929*(RLcp53_231+RLcp53_285);
    JTcp53_285_12 = -(ROcp53_729*(RLcp53_331+RLcp53_385)-ROcp53_929*(RLcp53_131+RLcp53_185));
    JTcp53_385_12 = ROcp53_729*(RLcp53_231+RLcp53_285)-ROcp53_829*(RLcp53_131+RLcp53_185);
    JTcp53_185_13 = -(RLcp53_285*ROcp53_630-RLcp53_385*ROcp53_530);
    JTcp53_285_13 = RLcp53_185*ROcp53_630-RLcp53_385*ROcp53_430;
    JTcp53_385_13 = -(RLcp53_185*ROcp53_530-RLcp53_285*ROcp53_430);
    ORcp53_185 = OMcp53_231*RLcp53_385-OMcp53_331*RLcp53_285;
    ORcp53_285 = -(OMcp53_131*RLcp53_385-OMcp53_331*RLcp53_185);
    ORcp53_385 = OMcp53_131*RLcp53_285-OMcp53_231*RLcp53_185;
    VIcp53_185 = ORcp53_121+ORcp53_123+ORcp53_128+ORcp53_129+ORcp53_130+ORcp53_131+ORcp53_185+qd(1);
    VIcp53_285 = ORcp53_221+ORcp53_223+ORcp53_228+ORcp53_229+ORcp53_230+ORcp53_231+ORcp53_285+qd(2);
    VIcp53_385 = ORcp53_321+ORcp53_323+ORcp53_328+ORcp53_329+ORcp53_330+ORcp53_331+ORcp53_385+qd(3);
    ACcp53_185 = qdd(1)+OMcp53_222*ORcp53_323+OMcp53_223*ORcp53_328+OMcp53_228*ORcp53_329+OMcp53_229*ORcp53_330+OMcp53_230*ORcp53_331+OMcp53_231*...
 ORcp53_385+OMcp53_26*ORcp53_321-OMcp53_322*ORcp53_223-OMcp53_323*ORcp53_228-OMcp53_328*ORcp53_229-OMcp53_329*ORcp53_230-OMcp53_330*ORcp53_231-...
 OMcp53_331*ORcp53_285-OMcp53_36*ORcp53_221+OPcp53_222*RLcp53_323+OPcp53_223*RLcp53_328+OPcp53_228*RLcp53_329+OPcp53_229*RLcp53_330+OPcp53_230*...
 RLcp53_331+OPcp53_231*RLcp53_385+OPcp53_26*RLcp53_321-OPcp53_322*RLcp53_223-OPcp53_323*RLcp53_228-OPcp53_328*RLcp53_229-OPcp53_329*RLcp53_230-...
 OPcp53_330*RLcp53_231-OPcp53_331*RLcp53_285-OPcp53_36*RLcp53_221;
    ACcp53_285 = qdd(2)-OMcp53_122*ORcp53_323-OMcp53_123*ORcp53_328-OMcp53_128*ORcp53_329-OMcp53_129*ORcp53_330-OMcp53_130*ORcp53_331-OMcp53_131*...
 ORcp53_385-OMcp53_16*ORcp53_321+OMcp53_322*ORcp53_123+OMcp53_323*ORcp53_128+OMcp53_328*ORcp53_129+OMcp53_329*ORcp53_130+OMcp53_330*ORcp53_131+...
 OMcp53_331*ORcp53_185+OMcp53_36*ORcp53_121-OPcp53_122*RLcp53_323-OPcp53_123*RLcp53_328-OPcp53_128*RLcp53_329-OPcp53_129*RLcp53_330-OPcp53_130*...
 RLcp53_331-OPcp53_131*RLcp53_385-OPcp53_16*RLcp53_321+OPcp53_322*RLcp53_123+OPcp53_323*RLcp53_128+OPcp53_328*RLcp53_129+OPcp53_329*RLcp53_130+...
 OPcp53_330*RLcp53_131+OPcp53_331*RLcp53_185+OPcp53_36*RLcp53_121;
    ACcp53_385 = qdd(3)+OMcp53_122*ORcp53_223+OMcp53_123*ORcp53_228+OMcp53_128*ORcp53_229+OMcp53_129*ORcp53_230+OMcp53_130*ORcp53_231+OMcp53_131*...
 ORcp53_285+OMcp53_16*ORcp53_221-OMcp53_222*ORcp53_123-OMcp53_223*ORcp53_128-OMcp53_228*ORcp53_129-OMcp53_229*ORcp53_130-OMcp53_230*ORcp53_131-...
 OMcp53_231*ORcp53_185-OMcp53_26*ORcp53_121+OPcp53_122*RLcp53_223+OPcp53_123*RLcp53_228+OPcp53_128*RLcp53_229+OPcp53_129*RLcp53_230+OPcp53_130*...
 RLcp53_231+OPcp53_131*RLcp53_285+OPcp53_16*RLcp53_221-OPcp53_222*RLcp53_123-OPcp53_223*RLcp53_128-OPcp53_228*RLcp53_129-OPcp53_229*RLcp53_130-...
 OPcp53_230*RLcp53_131-OPcp53_231*RLcp53_185-OPcp53_26*RLcp53_121;

% = = Block_1_0_0_54_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp53_185;
    sens.P(2) = POcp53_285;
    sens.P(3) = POcp53_385;
    sens.R(1,1) = ROcp53_131;
    sens.R(1,2) = ROcp53_231;
    sens.R(1,3) = ROcp53_331;
    sens.R(2,1) = ROcp53_430;
    sens.R(2,2) = ROcp53_530;
    sens.R(2,3) = ROcp53_630;
    sens.R(3,1) = ROcp53_731;
    sens.R(3,2) = ROcp53_831;
    sens.R(3,3) = ROcp53_931;
    sens.V(1) = VIcp53_185;
    sens.V(2) = VIcp53_285;
    sens.V(3) = VIcp53_385;
    sens.OM(1) = OMcp53_131;
    sens.OM(2) = OMcp53_231;
    sens.OM(3) = OMcp53_331;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp53_185_5;
    sens.J(1,6) = JTcp53_185_6;
    sens.J(1,21) = JTcp53_185_7;
    sens.J(1,22) = JTcp53_185_8;
    sens.J(1,23) = JTcp53_185_9;
    sens.J(1,28) = JTcp53_185_10;
    sens.J(1,29) = JTcp53_185_11;
    sens.J(1,30) = JTcp53_185_12;
    sens.J(1,31) = JTcp53_185_13;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = JTcp53_285_4;
    sens.J(2,5) = JTcp53_285_5;
    sens.J(2,6) = JTcp53_285_6;
    sens.J(2,21) = JTcp53_285_7;
    sens.J(2,22) = JTcp53_285_8;
    sens.J(2,23) = JTcp53_285_9;
    sens.J(2,28) = JTcp53_285_10;
    sens.J(2,29) = JTcp53_285_11;
    sens.J(2,30) = JTcp53_285_12;
    sens.J(2,31) = JTcp53_285_13;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = JTcp53_385_4;
    sens.J(3,5) = JTcp53_385_5;
    sens.J(3,6) = JTcp53_385_6;
    sens.J(3,21) = JTcp53_385_7;
    sens.J(3,22) = JTcp53_385_8;
    sens.J(3,23) = JTcp53_385_9;
    sens.J(3,28) = JTcp53_385_10;
    sens.J(3,29) = JTcp53_385_11;
    sens.J(3,30) = JTcp53_385_12;
    sens.J(3,31) = JTcp53_385_13;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(4,21) = ROcp53_16;
    sens.J(4,22) = ROcp53_421;
    sens.J(4,23) = ROcp53_722;
    sens.J(4,28) = ROcp53_423;
    sens.J(4,29) = ROcp53_128;
    sens.J(4,30) = ROcp53_729;
    sens.J(4,31) = ROcp53_430;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp53_85;
    sens.J(5,21) = ROcp53_26;
    sens.J(5,22) = ROcp53_521;
    sens.J(5,23) = ROcp53_822;
    sens.J(5,28) = ROcp53_523;
    sens.J(5,29) = ROcp53_228;
    sens.J(5,30) = ROcp53_829;
    sens.J(5,31) = ROcp53_530;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp53_95;
    sens.J(6,21) = ROcp53_36;
    sens.J(6,22) = ROcp53_621;
    sens.J(6,23) = ROcp53_922;
    sens.J(6,28) = ROcp53_623;
    sens.J(6,29) = ROcp53_328;
    sens.J(6,30) = ROcp53_929;
    sens.J(6,31) = ROcp53_630;
    sens.A(1) = ACcp53_185;
    sens.A(2) = ACcp53_285;
    sens.A(3) = ACcp53_385;
    sens.OMP(1) = OPcp53_131;
    sens.OMP(2) = OPcp53_231;
    sens.OMP(3) = OPcp53_331;
 
% 
case 55, 


% = = Block_1_0_0_55_0_1 = = 
 
% Sensor Kinematics 


    ROcp54_25 = S4*S5;
    ROcp54_35 = -C4*S5;
    ROcp54_85 = -S4*C5;
    ROcp54_95 = C4*C5;
    ROcp54_16 = C5*C6;
    ROcp54_26 = ROcp54_25*C6+C4*S6;
    ROcp54_36 = ROcp54_35*C6+S4*S6;
    ROcp54_46 = -C5*S6;
    ROcp54_56 = -(ROcp54_25*S6-C4*C6);
    ROcp54_66 = -(ROcp54_35*S6-S4*C6);
    OMcp54_25 = qd(5)*C4;
    OMcp54_35 = qd(5)*S4;
    OMcp54_16 = qd(4)+qd(6)*S5;
    OMcp54_26 = OMcp54_25+ROcp54_85*qd(6);
    OMcp54_36 = OMcp54_35+ROcp54_95*qd(6);
    OPcp54_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp54_26 = ROcp54_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp54_35*S5-ROcp54_95*qd(4));
    OPcp54_36 = ROcp54_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp54_25*S5-ROcp54_85*qd(4));

% = = Block_1_0_0_55_0_2 = = 
 
% Sensor Kinematics 


    ROcp54_17 = ROcp54_16*C7-S5*S7;
    ROcp54_27 = ROcp54_26*C7-ROcp54_85*S7;
    ROcp54_37 = ROcp54_36*C7-ROcp54_95*S7;
    ROcp54_77 = ROcp54_16*S7+S5*C7;
    ROcp54_87 = ROcp54_26*S7+ROcp54_85*C7;
    ROcp54_97 = ROcp54_36*S7+ROcp54_95*C7;
    ROcp54_48 = ROcp54_46*C8+ROcp54_77*S8;
    ROcp54_58 = ROcp54_56*C8+ROcp54_87*S8;
    ROcp54_68 = ROcp54_66*C8+ROcp54_97*S8;
    ROcp54_78 = -(ROcp54_46*S8-ROcp54_77*C8);
    ROcp54_88 = -(ROcp54_56*S8-ROcp54_87*C8);
    ROcp54_98 = -(ROcp54_66*S8-ROcp54_97*C8);
    ROcp54_19 = ROcp54_17*C9+ROcp54_48*S9;
    ROcp54_29 = ROcp54_27*C9+ROcp54_58*S9;
    ROcp54_39 = ROcp54_37*C9+ROcp54_68*S9;
    ROcp54_49 = -(ROcp54_17*S9-ROcp54_48*C9);
    ROcp54_59 = -(ROcp54_27*S9-ROcp54_58*C9);
    ROcp54_69 = -(ROcp54_37*S9-ROcp54_68*C9);
    ROcp54_110 = ROcp54_19*C10-ROcp54_78*S10;
    ROcp54_210 = ROcp54_29*C10-ROcp54_88*S10;
    ROcp54_310 = ROcp54_39*C10-ROcp54_98*S10;
    ROcp54_710 = ROcp54_19*S10+ROcp54_78*C10;
    ROcp54_810 = ROcp54_29*S10+ROcp54_88*C10;
    ROcp54_910 = ROcp54_39*S10+ROcp54_98*C10;
    ROcp54_411 = ROcp54_49*C11+ROcp54_710*S11;
    ROcp54_511 = ROcp54_59*C11+ROcp54_810*S11;
    ROcp54_611 = ROcp54_69*C11+ROcp54_910*S11;
    ROcp54_711 = -(ROcp54_49*S11-ROcp54_710*C11);
    ROcp54_811 = -(ROcp54_59*S11-ROcp54_810*C11);
    ROcp54_911 = -(ROcp54_69*S11-ROcp54_910*C11);
    ROcp54_112 = ROcp54_110*C12-ROcp54_711*S12;
    ROcp54_212 = ROcp54_210*C12-ROcp54_811*S12;
    ROcp54_312 = ROcp54_310*C12-ROcp54_911*S12;
    ROcp54_712 = ROcp54_110*S12+ROcp54_711*C12;
    ROcp54_812 = ROcp54_210*S12+ROcp54_811*C12;
    ROcp54_912 = ROcp54_310*S12+ROcp54_911*C12;
    RLcp54_17 = ROcp54_46*s.dpt(2,1);
    RLcp54_27 = ROcp54_56*s.dpt(2,1);
    RLcp54_37 = ROcp54_66*s.dpt(2,1);
    OMcp54_17 = OMcp54_16+ROcp54_46*qd(7);
    OMcp54_27 = OMcp54_26+ROcp54_56*qd(7);
    OMcp54_37 = OMcp54_36+ROcp54_66*qd(7);
    ORcp54_17 = OMcp54_26*RLcp54_37-OMcp54_36*RLcp54_27;
    ORcp54_27 = -(OMcp54_16*RLcp54_37-OMcp54_36*RLcp54_17);
    ORcp54_37 = OMcp54_16*RLcp54_27-OMcp54_26*RLcp54_17;
    OPcp54_17 = OPcp54_16+ROcp54_46*qdd(7)+qd(7)*(OMcp54_26*ROcp54_66-OMcp54_36*ROcp54_56);
    OPcp54_27 = OPcp54_26+ROcp54_56*qdd(7)-qd(7)*(OMcp54_16*ROcp54_66-OMcp54_36*ROcp54_46);
    OPcp54_37 = OPcp54_36+ROcp54_66*qdd(7)+qd(7)*(OMcp54_16*ROcp54_56-OMcp54_26*ROcp54_46);
    RLcp54_18 = ROcp54_46*s.dpt(2,6);
    RLcp54_28 = ROcp54_56*s.dpt(2,6);
    RLcp54_38 = ROcp54_66*s.dpt(2,6);
    OMcp54_18 = OMcp54_17+ROcp54_17*qd(8);
    OMcp54_28 = OMcp54_27+ROcp54_27*qd(8);
    OMcp54_38 = OMcp54_37+ROcp54_37*qd(8);
    ORcp54_18 = OMcp54_27*RLcp54_38-OMcp54_37*RLcp54_28;
    ORcp54_28 = -(OMcp54_17*RLcp54_38-OMcp54_37*RLcp54_18);
    ORcp54_38 = OMcp54_17*RLcp54_28-OMcp54_27*RLcp54_18;
    OPcp54_18 = OPcp54_17+ROcp54_17*qdd(8)+qd(8)*(OMcp54_27*ROcp54_37-OMcp54_37*ROcp54_27);
    OPcp54_28 = OPcp54_27+ROcp54_27*qdd(8)-qd(8)*(OMcp54_17*ROcp54_37-OMcp54_37*ROcp54_17);
    OPcp54_38 = OPcp54_37+ROcp54_37*qdd(8)+qd(8)*(OMcp54_17*ROcp54_27-OMcp54_27*ROcp54_17);
    RLcp54_19 = ROcp54_78*s.dpt(3,8);
    RLcp54_29 = ROcp54_88*s.dpt(3,8);
    RLcp54_39 = ROcp54_98*s.dpt(3,8);
    OMcp54_19 = OMcp54_18+ROcp54_78*qd(9);
    OMcp54_29 = OMcp54_28+ROcp54_88*qd(9);
    OMcp54_39 = OMcp54_38+ROcp54_98*qd(9);
    ORcp54_19 = OMcp54_28*RLcp54_39-OMcp54_38*RLcp54_29;
    ORcp54_29 = -(OMcp54_18*RLcp54_39-OMcp54_38*RLcp54_19);
    ORcp54_39 = OMcp54_18*RLcp54_29-OMcp54_28*RLcp54_19;
    OPcp54_19 = OPcp54_18+ROcp54_78*qdd(9)+qd(9)*(OMcp54_28*ROcp54_98-OMcp54_38*ROcp54_88);
    OPcp54_29 = OPcp54_28+ROcp54_88*qdd(9)-qd(9)*(OMcp54_18*ROcp54_98-OMcp54_38*ROcp54_78);
    OPcp54_39 = OPcp54_38+ROcp54_98*qdd(9)+qd(9)*(OMcp54_18*ROcp54_88-OMcp54_28*ROcp54_78);
    RLcp54_110 = ROcp54_78*s.dpt(3,10);
    RLcp54_210 = ROcp54_88*s.dpt(3,10);
    RLcp54_310 = ROcp54_98*s.dpt(3,10);
    OMcp54_110 = OMcp54_19+ROcp54_49*qd(10);
    OMcp54_210 = OMcp54_29+ROcp54_59*qd(10);
    OMcp54_310 = OMcp54_39+ROcp54_69*qd(10);
    ORcp54_110 = OMcp54_29*RLcp54_310-OMcp54_39*RLcp54_210;
    ORcp54_210 = -(OMcp54_19*RLcp54_310-OMcp54_39*RLcp54_110);
    ORcp54_310 = OMcp54_19*RLcp54_210-OMcp54_29*RLcp54_110;
    OPcp54_110 = OPcp54_19+ROcp54_49*qdd(10)+qd(10)*(OMcp54_29*ROcp54_69-OMcp54_39*ROcp54_59);
    OPcp54_210 = OPcp54_29+ROcp54_59*qdd(10)-qd(10)*(OMcp54_19*ROcp54_69-OMcp54_39*ROcp54_49);
    OPcp54_310 = OPcp54_39+ROcp54_69*qdd(10)+qd(10)*(OMcp54_19*ROcp54_59-OMcp54_29*ROcp54_49);
    RLcp54_111 = ROcp54_710*s.dpt(3,12);
    RLcp54_211 = ROcp54_810*s.dpt(3,12);
    RLcp54_311 = ROcp54_910*s.dpt(3,12);
    OMcp54_111 = OMcp54_110+ROcp54_110*qd(11);
    OMcp54_211 = OMcp54_210+ROcp54_210*qd(11);
    OMcp54_311 = OMcp54_310+ROcp54_310*qd(11);
    ORcp54_111 = OMcp54_210*RLcp54_311-OMcp54_310*RLcp54_211;
    ORcp54_211 = -(OMcp54_110*RLcp54_311-OMcp54_310*RLcp54_111);
    ORcp54_311 = OMcp54_110*RLcp54_211-OMcp54_210*RLcp54_111;
    OMcp54_112 = OMcp54_111+ROcp54_411*qd(12);
    OMcp54_212 = OMcp54_211+ROcp54_511*qd(12);
    OMcp54_312 = OMcp54_311+ROcp54_611*qd(12);
    OPcp54_112 = OPcp54_110+ROcp54_110*qdd(11)+ROcp54_411*qdd(12)+qd(11)*(OMcp54_210*ROcp54_310-OMcp54_310*ROcp54_210)+qd(12)*(OMcp54_211*...
 ROcp54_611-OMcp54_311*ROcp54_511);
    OPcp54_212 = OPcp54_210+ROcp54_210*qdd(11)+ROcp54_511*qdd(12)-qd(11)*(OMcp54_110*ROcp54_310-OMcp54_310*ROcp54_110)-qd(12)*(OMcp54_111*...
 ROcp54_611-OMcp54_311*ROcp54_411);
    OPcp54_312 = OPcp54_310+ROcp54_310*qdd(11)+ROcp54_611*qdd(12)+qd(11)*(OMcp54_110*ROcp54_210-OMcp54_210*ROcp54_110)+qd(12)*(OMcp54_111*...
 ROcp54_511-OMcp54_211*ROcp54_411);
    RLcp54_186 = ROcp54_712*s.dpt(3,16);
    RLcp54_286 = ROcp54_812*s.dpt(3,16);
    RLcp54_386 = ROcp54_912*s.dpt(3,16);
    POcp54_186 = RLcp54_110+RLcp54_111+RLcp54_17+RLcp54_18+RLcp54_186+RLcp54_19+q(1);
    POcp54_286 = RLcp54_210+RLcp54_211+RLcp54_27+RLcp54_28+RLcp54_286+RLcp54_29+q(2);
    POcp54_386 = RLcp54_310+RLcp54_311+RLcp54_37+RLcp54_38+RLcp54_386+RLcp54_39+q(3);
    ORcp54_186 = OMcp54_212*RLcp54_386-OMcp54_312*RLcp54_286;
    ORcp54_286 = -(OMcp54_112*RLcp54_386-OMcp54_312*RLcp54_186);
    ORcp54_386 = OMcp54_112*RLcp54_286-OMcp54_212*RLcp54_186;
    VIcp54_186 = ORcp54_110+ORcp54_111+ORcp54_17+ORcp54_18+ORcp54_186+ORcp54_19+qd(1);
    VIcp54_286 = ORcp54_210+ORcp54_211+ORcp54_27+ORcp54_28+ORcp54_286+ORcp54_29+qd(2);
    VIcp54_386 = ORcp54_310+ORcp54_311+ORcp54_37+ORcp54_38+ORcp54_386+ORcp54_39+qd(3);
    ACcp54_186 = qdd(1)+OMcp54_210*ORcp54_311+OMcp54_212*ORcp54_386+OMcp54_26*ORcp54_37+OMcp54_27*ORcp54_38+OMcp54_28*ORcp54_39+OMcp54_29*...
 ORcp54_310-OMcp54_310*ORcp54_211-OMcp54_312*ORcp54_286-OMcp54_36*ORcp54_27-OMcp54_37*ORcp54_28-OMcp54_38*ORcp54_29-OMcp54_39*ORcp54_210+OPcp54_210*...
 RLcp54_311+OPcp54_212*RLcp54_386+OPcp54_26*RLcp54_37+OPcp54_27*RLcp54_38+OPcp54_28*RLcp54_39+OPcp54_29*RLcp54_310-OPcp54_310*RLcp54_211-OPcp54_312*...
 RLcp54_286-OPcp54_36*RLcp54_27-OPcp54_37*RLcp54_28-OPcp54_38*RLcp54_29-OPcp54_39*RLcp54_210;
    ACcp54_286 = qdd(2)-OMcp54_110*ORcp54_311-OMcp54_112*ORcp54_386-OMcp54_16*ORcp54_37-OMcp54_17*ORcp54_38-OMcp54_18*ORcp54_39-OMcp54_19*...
 ORcp54_310+OMcp54_310*ORcp54_111+OMcp54_312*ORcp54_186+OMcp54_36*ORcp54_17+OMcp54_37*ORcp54_18+OMcp54_38*ORcp54_19+OMcp54_39*ORcp54_110-OPcp54_110*...
 RLcp54_311-OPcp54_112*RLcp54_386-OPcp54_16*RLcp54_37-OPcp54_17*RLcp54_38-OPcp54_18*RLcp54_39-OPcp54_19*RLcp54_310+OPcp54_310*RLcp54_111+OPcp54_312*...
 RLcp54_186+OPcp54_36*RLcp54_17+OPcp54_37*RLcp54_18+OPcp54_38*RLcp54_19+OPcp54_39*RLcp54_110;
    ACcp54_386 = qdd(3)+OMcp54_110*ORcp54_211+OMcp54_112*ORcp54_286+OMcp54_16*ORcp54_27+OMcp54_17*ORcp54_28+OMcp54_18*ORcp54_29+OMcp54_19*...
 ORcp54_210-OMcp54_210*ORcp54_111-OMcp54_212*ORcp54_186-OMcp54_26*ORcp54_17-OMcp54_27*ORcp54_18-OMcp54_28*ORcp54_19-OMcp54_29*ORcp54_110+OPcp54_110*...
 RLcp54_211+OPcp54_112*RLcp54_286+OPcp54_16*RLcp54_27+OPcp54_17*RLcp54_28+OPcp54_18*RLcp54_29+OPcp54_19*RLcp54_210-OPcp54_210*RLcp54_111-OPcp54_212*...
 RLcp54_186-OPcp54_26*RLcp54_17-OPcp54_27*RLcp54_18-OPcp54_28*RLcp54_19-OPcp54_29*RLcp54_110;

% = = Block_1_0_0_55_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp54_186;
    sens.P(2) = POcp54_286;
    sens.P(3) = POcp54_386;
    sens.R(1,1) = ROcp54_112;
    sens.R(1,2) = ROcp54_212;
    sens.R(1,3) = ROcp54_312;
    sens.R(2,1) = ROcp54_411;
    sens.R(2,2) = ROcp54_511;
    sens.R(2,3) = ROcp54_611;
    sens.R(3,1) = ROcp54_712;
    sens.R(3,2) = ROcp54_812;
    sens.R(3,3) = ROcp54_912;
    sens.V(1) = VIcp54_186;
    sens.V(2) = VIcp54_286;
    sens.V(3) = VIcp54_386;
    sens.OM(1) = OMcp54_112;
    sens.OM(2) = OMcp54_212;
    sens.OM(3) = OMcp54_312;
    sens.A(1) = ACcp54_186;
    sens.A(2) = ACcp54_286;
    sens.A(3) = ACcp54_386;
    sens.OMP(1) = OPcp54_112;
    sens.OMP(2) = OPcp54_212;
    sens.OMP(3) = OPcp54_312;
 
% 
case 56, 


% = = Block_1_0_0_56_0_1 = = 
 
% Sensor Kinematics 


    ROcp55_25 = S4*S5;
    ROcp55_35 = -C4*S5;
    ROcp55_85 = -S4*C5;
    ROcp55_95 = C4*C5;
    ROcp55_16 = C5*C6;
    ROcp55_26 = ROcp55_25*C6+C4*S6;
    ROcp55_36 = ROcp55_35*C6+S4*S6;
    ROcp55_46 = -C5*S6;
    ROcp55_56 = -(ROcp55_25*S6-C4*C6);
    ROcp55_66 = -(ROcp55_35*S6-S4*C6);
    OMcp55_25 = qd(5)*C4;
    OMcp55_35 = qd(5)*S4;
    OMcp55_16 = qd(4)+qd(6)*S5;
    OMcp55_26 = OMcp55_25+ROcp55_85*qd(6);
    OMcp55_36 = OMcp55_35+ROcp55_95*qd(6);
    OPcp55_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp55_26 = ROcp55_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp55_35*S5-ROcp55_95*qd(4));
    OPcp55_36 = ROcp55_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp55_25*S5-ROcp55_85*qd(4));

% = = Block_1_0_0_56_0_2 = = 
 
% Sensor Kinematics 


    ROcp55_17 = ROcp55_16*C7-S5*S7;
    ROcp55_27 = ROcp55_26*C7-ROcp55_85*S7;
    ROcp55_37 = ROcp55_36*C7-ROcp55_95*S7;
    ROcp55_77 = ROcp55_16*S7+S5*C7;
    ROcp55_87 = ROcp55_26*S7+ROcp55_85*C7;
    ROcp55_97 = ROcp55_36*S7+ROcp55_95*C7;
    ROcp55_48 = ROcp55_46*C8+ROcp55_77*S8;
    ROcp55_58 = ROcp55_56*C8+ROcp55_87*S8;
    ROcp55_68 = ROcp55_66*C8+ROcp55_97*S8;
    ROcp55_78 = -(ROcp55_46*S8-ROcp55_77*C8);
    ROcp55_88 = -(ROcp55_56*S8-ROcp55_87*C8);
    ROcp55_98 = -(ROcp55_66*S8-ROcp55_97*C8);
    ROcp55_19 = ROcp55_17*C9+ROcp55_48*S9;
    ROcp55_29 = ROcp55_27*C9+ROcp55_58*S9;
    ROcp55_39 = ROcp55_37*C9+ROcp55_68*S9;
    ROcp55_49 = -(ROcp55_17*S9-ROcp55_48*C9);
    ROcp55_59 = -(ROcp55_27*S9-ROcp55_58*C9);
    ROcp55_69 = -(ROcp55_37*S9-ROcp55_68*C9);
    ROcp55_110 = ROcp55_19*C10-ROcp55_78*S10;
    ROcp55_210 = ROcp55_29*C10-ROcp55_88*S10;
    ROcp55_310 = ROcp55_39*C10-ROcp55_98*S10;
    ROcp55_710 = ROcp55_19*S10+ROcp55_78*C10;
    ROcp55_810 = ROcp55_29*S10+ROcp55_88*C10;
    ROcp55_910 = ROcp55_39*S10+ROcp55_98*C10;
    ROcp55_411 = ROcp55_49*C11+ROcp55_710*S11;
    ROcp55_511 = ROcp55_59*C11+ROcp55_810*S11;
    ROcp55_611 = ROcp55_69*C11+ROcp55_910*S11;
    ROcp55_711 = -(ROcp55_49*S11-ROcp55_710*C11);
    ROcp55_811 = -(ROcp55_59*S11-ROcp55_810*C11);
    ROcp55_911 = -(ROcp55_69*S11-ROcp55_910*C11);
    ROcp55_112 = ROcp55_110*C12-ROcp55_711*S12;
    ROcp55_212 = ROcp55_210*C12-ROcp55_811*S12;
    ROcp55_312 = ROcp55_310*C12-ROcp55_911*S12;
    ROcp55_712 = ROcp55_110*S12+ROcp55_711*C12;
    ROcp55_812 = ROcp55_210*S12+ROcp55_811*C12;
    ROcp55_912 = ROcp55_310*S12+ROcp55_911*C12;
    ROcp55_113 = ROcp55_112*C13-ROcp55_712*S13;
    ROcp55_213 = ROcp55_212*C13-ROcp55_812*S13;
    ROcp55_313 = ROcp55_312*C13-ROcp55_912*S13;
    ROcp55_713 = ROcp55_112*S13+ROcp55_712*C13;
    ROcp55_813 = ROcp55_212*S13+ROcp55_812*C13;
    ROcp55_913 = ROcp55_312*S13+ROcp55_912*C13;
    RLcp55_17 = ROcp55_46*s.dpt(2,1);
    RLcp55_27 = ROcp55_56*s.dpt(2,1);
    RLcp55_37 = ROcp55_66*s.dpt(2,1);
    OMcp55_17 = OMcp55_16+ROcp55_46*qd(7);
    OMcp55_27 = OMcp55_26+ROcp55_56*qd(7);
    OMcp55_37 = OMcp55_36+ROcp55_66*qd(7);
    ORcp55_17 = OMcp55_26*RLcp55_37-OMcp55_36*RLcp55_27;
    ORcp55_27 = -(OMcp55_16*RLcp55_37-OMcp55_36*RLcp55_17);
    ORcp55_37 = OMcp55_16*RLcp55_27-OMcp55_26*RLcp55_17;
    OPcp55_17 = OPcp55_16+ROcp55_46*qdd(7)+qd(7)*(OMcp55_26*ROcp55_66-OMcp55_36*ROcp55_56);
    OPcp55_27 = OPcp55_26+ROcp55_56*qdd(7)-qd(7)*(OMcp55_16*ROcp55_66-OMcp55_36*ROcp55_46);
    OPcp55_37 = OPcp55_36+ROcp55_66*qdd(7)+qd(7)*(OMcp55_16*ROcp55_56-OMcp55_26*ROcp55_46);
    RLcp55_18 = ROcp55_46*s.dpt(2,6);
    RLcp55_28 = ROcp55_56*s.dpt(2,6);
    RLcp55_38 = ROcp55_66*s.dpt(2,6);
    OMcp55_18 = OMcp55_17+ROcp55_17*qd(8);
    OMcp55_28 = OMcp55_27+ROcp55_27*qd(8);
    OMcp55_38 = OMcp55_37+ROcp55_37*qd(8);
    ORcp55_18 = OMcp55_27*RLcp55_38-OMcp55_37*RLcp55_28;
    ORcp55_28 = -(OMcp55_17*RLcp55_38-OMcp55_37*RLcp55_18);
    ORcp55_38 = OMcp55_17*RLcp55_28-OMcp55_27*RLcp55_18;
    OPcp55_18 = OPcp55_17+ROcp55_17*qdd(8)+qd(8)*(OMcp55_27*ROcp55_37-OMcp55_37*ROcp55_27);
    OPcp55_28 = OPcp55_27+ROcp55_27*qdd(8)-qd(8)*(OMcp55_17*ROcp55_37-OMcp55_37*ROcp55_17);
    OPcp55_38 = OPcp55_37+ROcp55_37*qdd(8)+qd(8)*(OMcp55_17*ROcp55_27-OMcp55_27*ROcp55_17);
    RLcp55_19 = ROcp55_78*s.dpt(3,8);
    RLcp55_29 = ROcp55_88*s.dpt(3,8);
    RLcp55_39 = ROcp55_98*s.dpt(3,8);
    OMcp55_19 = OMcp55_18+ROcp55_78*qd(9);
    OMcp55_29 = OMcp55_28+ROcp55_88*qd(9);
    OMcp55_39 = OMcp55_38+ROcp55_98*qd(9);
    ORcp55_19 = OMcp55_28*RLcp55_39-OMcp55_38*RLcp55_29;
    ORcp55_29 = -(OMcp55_18*RLcp55_39-OMcp55_38*RLcp55_19);
    ORcp55_39 = OMcp55_18*RLcp55_29-OMcp55_28*RLcp55_19;
    OPcp55_19 = OPcp55_18+ROcp55_78*qdd(9)+qd(9)*(OMcp55_28*ROcp55_98-OMcp55_38*ROcp55_88);
    OPcp55_29 = OPcp55_28+ROcp55_88*qdd(9)-qd(9)*(OMcp55_18*ROcp55_98-OMcp55_38*ROcp55_78);
    OPcp55_39 = OPcp55_38+ROcp55_98*qdd(9)+qd(9)*(OMcp55_18*ROcp55_88-OMcp55_28*ROcp55_78);
    RLcp55_110 = ROcp55_78*s.dpt(3,10);
    RLcp55_210 = ROcp55_88*s.dpt(3,10);
    RLcp55_310 = ROcp55_98*s.dpt(3,10);
    OMcp55_110 = OMcp55_19+ROcp55_49*qd(10);
    OMcp55_210 = OMcp55_29+ROcp55_59*qd(10);
    OMcp55_310 = OMcp55_39+ROcp55_69*qd(10);
    ORcp55_110 = OMcp55_29*RLcp55_310-OMcp55_39*RLcp55_210;
    ORcp55_210 = -(OMcp55_19*RLcp55_310-OMcp55_39*RLcp55_110);
    ORcp55_310 = OMcp55_19*RLcp55_210-OMcp55_29*RLcp55_110;
    OPcp55_110 = OPcp55_19+ROcp55_49*qdd(10)+qd(10)*(OMcp55_29*ROcp55_69-OMcp55_39*ROcp55_59);
    OPcp55_210 = OPcp55_29+ROcp55_59*qdd(10)-qd(10)*(OMcp55_19*ROcp55_69-OMcp55_39*ROcp55_49);
    OPcp55_310 = OPcp55_39+ROcp55_69*qdd(10)+qd(10)*(OMcp55_19*ROcp55_59-OMcp55_29*ROcp55_49);
    RLcp55_111 = ROcp55_710*s.dpt(3,12);
    RLcp55_211 = ROcp55_810*s.dpt(3,12);
    RLcp55_311 = ROcp55_910*s.dpt(3,12);
    OMcp55_111 = OMcp55_110+ROcp55_110*qd(11);
    OMcp55_211 = OMcp55_210+ROcp55_210*qd(11);
    OMcp55_311 = OMcp55_310+ROcp55_310*qd(11);
    ORcp55_111 = OMcp55_210*RLcp55_311-OMcp55_310*RLcp55_211;
    ORcp55_211 = -(OMcp55_110*RLcp55_311-OMcp55_310*RLcp55_111);
    ORcp55_311 = OMcp55_110*RLcp55_211-OMcp55_210*RLcp55_111;
    OMcp55_112 = OMcp55_111+ROcp55_411*qd(12);
    OMcp55_212 = OMcp55_211+ROcp55_511*qd(12);
    OMcp55_312 = OMcp55_311+ROcp55_611*qd(12);
    OPcp55_112 = OPcp55_110+ROcp55_110*qdd(11)+ROcp55_411*qdd(12)+qd(11)*(OMcp55_210*ROcp55_310-OMcp55_310*ROcp55_210)+qd(12)*(OMcp55_211*...
 ROcp55_611-OMcp55_311*ROcp55_511);
    OPcp55_212 = OPcp55_210+ROcp55_210*qdd(11)+ROcp55_511*qdd(12)-qd(11)*(OMcp55_110*ROcp55_310-OMcp55_310*ROcp55_110)-qd(12)*(OMcp55_111*...
 ROcp55_611-OMcp55_311*ROcp55_411);
    OPcp55_312 = OPcp55_310+ROcp55_310*qdd(11)+ROcp55_611*qdd(12)+qd(11)*(OMcp55_110*ROcp55_210-OMcp55_210*ROcp55_110)+qd(12)*(OMcp55_111*...
 ROcp55_511-OMcp55_211*ROcp55_411);
    RLcp55_113 = ROcp55_112*s.dpt(1,18)+ROcp55_712*s.dpt(3,18);
    RLcp55_213 = ROcp55_212*s.dpt(1,18)+ROcp55_812*s.dpt(3,18);
    RLcp55_313 = ROcp55_312*s.dpt(1,18)+ROcp55_912*s.dpt(3,18);
    POcp55_113 = RLcp55_110+RLcp55_111+RLcp55_113+RLcp55_17+RLcp55_18+RLcp55_19+q(1);
    POcp55_213 = RLcp55_210+RLcp55_211+RLcp55_213+RLcp55_27+RLcp55_28+RLcp55_29+q(2);
    POcp55_313 = RLcp55_310+RLcp55_311+RLcp55_313+RLcp55_37+RLcp55_38+RLcp55_39+q(3);
    OMcp55_113 = OMcp55_112+ROcp55_411*qd(13);
    OMcp55_213 = OMcp55_212+ROcp55_511*qd(13);
    OMcp55_313 = OMcp55_312+ROcp55_611*qd(13);
    ORcp55_113 = OMcp55_212*RLcp55_313-OMcp55_312*RLcp55_213;
    ORcp55_213 = -(OMcp55_112*RLcp55_313-OMcp55_312*RLcp55_113);
    ORcp55_313 = OMcp55_112*RLcp55_213-OMcp55_212*RLcp55_113;
    VIcp55_113 = ORcp55_110+ORcp55_111+ORcp55_113+ORcp55_17+ORcp55_18+ORcp55_19+qd(1);
    VIcp55_213 = ORcp55_210+ORcp55_211+ORcp55_213+ORcp55_27+ORcp55_28+ORcp55_29+qd(2);
    VIcp55_313 = ORcp55_310+ORcp55_311+ORcp55_313+ORcp55_37+ORcp55_38+ORcp55_39+qd(3);
    OPcp55_113 = OPcp55_112+ROcp55_411*qdd(13)+qd(13)*(OMcp55_212*ROcp55_611-OMcp55_312*ROcp55_511);
    OPcp55_213 = OPcp55_212+ROcp55_511*qdd(13)-qd(13)*(OMcp55_112*ROcp55_611-OMcp55_312*ROcp55_411);
    OPcp55_313 = OPcp55_312+ROcp55_611*qdd(13)+qd(13)*(OMcp55_112*ROcp55_511-OMcp55_212*ROcp55_411);
    ACcp55_113 = qdd(1)+OMcp55_210*ORcp55_311+OMcp55_212*ORcp55_313+OMcp55_26*ORcp55_37+OMcp55_27*ORcp55_38+OMcp55_28*ORcp55_39+OMcp55_29*...
 ORcp55_310-OMcp55_310*ORcp55_211-OMcp55_312*ORcp55_213-OMcp55_36*ORcp55_27-OMcp55_37*ORcp55_28-OMcp55_38*ORcp55_29-OMcp55_39*ORcp55_210+OPcp55_210*...
 RLcp55_311+OPcp55_212*RLcp55_313+OPcp55_26*RLcp55_37+OPcp55_27*RLcp55_38+OPcp55_28*RLcp55_39+OPcp55_29*RLcp55_310-OPcp55_310*RLcp55_211-OPcp55_312*...
 RLcp55_213-OPcp55_36*RLcp55_27-OPcp55_37*RLcp55_28-OPcp55_38*RLcp55_29-OPcp55_39*RLcp55_210;
    ACcp55_213 = qdd(2)-OMcp55_110*ORcp55_311-OMcp55_112*ORcp55_313-OMcp55_16*ORcp55_37-OMcp55_17*ORcp55_38-OMcp55_18*ORcp55_39-OMcp55_19*...
 ORcp55_310+OMcp55_310*ORcp55_111+OMcp55_312*ORcp55_113+OMcp55_36*ORcp55_17+OMcp55_37*ORcp55_18+OMcp55_38*ORcp55_19+OMcp55_39*ORcp55_110-OPcp55_110*...
 RLcp55_311-OPcp55_112*RLcp55_313-OPcp55_16*RLcp55_37-OPcp55_17*RLcp55_38-OPcp55_18*RLcp55_39-OPcp55_19*RLcp55_310+OPcp55_310*RLcp55_111+OPcp55_312*...
 RLcp55_113+OPcp55_36*RLcp55_17+OPcp55_37*RLcp55_18+OPcp55_38*RLcp55_19+OPcp55_39*RLcp55_110;
    ACcp55_313 = qdd(3)+OMcp55_110*ORcp55_211+OMcp55_112*ORcp55_213+OMcp55_16*ORcp55_27+OMcp55_17*ORcp55_28+OMcp55_18*ORcp55_29+OMcp55_19*...
 ORcp55_210-OMcp55_210*ORcp55_111-OMcp55_212*ORcp55_113-OMcp55_26*ORcp55_17-OMcp55_27*ORcp55_18-OMcp55_28*ORcp55_19-OMcp55_29*ORcp55_110+OPcp55_110*...
 RLcp55_211+OPcp55_112*RLcp55_213+OPcp55_16*RLcp55_27+OPcp55_17*RLcp55_28+OPcp55_18*RLcp55_29+OPcp55_19*RLcp55_210-OPcp55_210*RLcp55_111-OPcp55_212*...
 RLcp55_113-OPcp55_26*RLcp55_17-OPcp55_27*RLcp55_18-OPcp55_28*RLcp55_19-OPcp55_29*RLcp55_110;

% = = Block_1_0_0_56_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp55_113;
    sens.P(2) = POcp55_213;
    sens.P(3) = POcp55_313;
    sens.R(1,1) = ROcp55_113;
    sens.R(1,2) = ROcp55_213;
    sens.R(1,3) = ROcp55_313;
    sens.R(2,1) = ROcp55_411;
    sens.R(2,2) = ROcp55_511;
    sens.R(2,3) = ROcp55_611;
    sens.R(3,1) = ROcp55_713;
    sens.R(3,2) = ROcp55_813;
    sens.R(3,3) = ROcp55_913;
    sens.V(1) = VIcp55_113;
    sens.V(2) = VIcp55_213;
    sens.V(3) = VIcp55_313;
    sens.OM(1) = OMcp55_113;
    sens.OM(2) = OMcp55_213;
    sens.OM(3) = OMcp55_313;
    sens.A(1) = ACcp55_113;
    sens.A(2) = ACcp55_213;
    sens.A(3) = ACcp55_313;
    sens.OMP(1) = OPcp55_113;
    sens.OMP(2) = OPcp55_213;
    sens.OMP(3) = OPcp55_313;
 
% 
case 57, 


% = = Block_1_0_0_57_0_1 = = 
 
% Sensor Kinematics 


    ROcp56_25 = S4*S5;
    ROcp56_35 = -C4*S5;
    ROcp56_85 = -S4*C5;
    ROcp56_95 = C4*C5;
    ROcp56_16 = C5*C6;
    ROcp56_26 = ROcp56_25*C6+C4*S6;
    ROcp56_36 = ROcp56_35*C6+S4*S6;
    ROcp56_46 = -C5*S6;
    ROcp56_56 = -(ROcp56_25*S6-C4*C6);
    ROcp56_66 = -(ROcp56_35*S6-S4*C6);
    OMcp56_25 = qd(5)*C4;
    OMcp56_35 = qd(5)*S4;
    OMcp56_16 = qd(4)+qd(6)*S5;
    OMcp56_26 = OMcp56_25+ROcp56_85*qd(6);
    OMcp56_36 = OMcp56_35+ROcp56_95*qd(6);
    OPcp56_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp56_26 = ROcp56_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp56_35*S5-ROcp56_95*qd(4));
    OPcp56_36 = ROcp56_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp56_25*S5-ROcp56_85*qd(4));

% = = Block_1_0_0_57_0_3 = = 
 
% Sensor Kinematics 


    ROcp56_114 = ROcp56_16*C14-S14*S5;
    ROcp56_214 = ROcp56_26*C14-ROcp56_85*S14;
    ROcp56_314 = ROcp56_36*C14-ROcp56_95*S14;
    ROcp56_714 = ROcp56_16*S14+C14*S5;
    ROcp56_814 = ROcp56_26*S14+ROcp56_85*C14;
    ROcp56_914 = ROcp56_36*S14+ROcp56_95*C14;
    ROcp56_415 = ROcp56_46*C15+ROcp56_714*S15;
    ROcp56_515 = ROcp56_56*C15+ROcp56_814*S15;
    ROcp56_615 = ROcp56_66*C15+ROcp56_914*S15;
    ROcp56_715 = -(ROcp56_46*S15-ROcp56_714*C15);
    ROcp56_815 = -(ROcp56_56*S15-ROcp56_814*C15);
    ROcp56_915 = -(ROcp56_66*S15-ROcp56_914*C15);
    ROcp56_116 = ROcp56_114*C16+ROcp56_415*S16;
    ROcp56_216 = ROcp56_214*C16+ROcp56_515*S16;
    ROcp56_316 = ROcp56_314*C16+ROcp56_615*S16;
    ROcp56_416 = -(ROcp56_114*S16-ROcp56_415*C16);
    ROcp56_516 = -(ROcp56_214*S16-ROcp56_515*C16);
    ROcp56_616 = -(ROcp56_314*S16-ROcp56_615*C16);
    ROcp56_117 = ROcp56_116*C17-ROcp56_715*S17;
    ROcp56_217 = ROcp56_216*C17-ROcp56_815*S17;
    ROcp56_317 = ROcp56_316*C17-ROcp56_915*S17;
    ROcp56_717 = ROcp56_116*S17+ROcp56_715*C17;
    ROcp56_817 = ROcp56_216*S17+ROcp56_815*C17;
    ROcp56_917 = ROcp56_316*S17+ROcp56_915*C17;
    ROcp56_418 = ROcp56_416*C18+ROcp56_717*S18;
    ROcp56_518 = ROcp56_516*C18+ROcp56_817*S18;
    ROcp56_618 = ROcp56_616*C18+ROcp56_917*S18;
    ROcp56_718 = -(ROcp56_416*S18-ROcp56_717*C18);
    ROcp56_818 = -(ROcp56_516*S18-ROcp56_817*C18);
    ROcp56_918 = -(ROcp56_616*S18-ROcp56_917*C18);
    ROcp56_119 = ROcp56_117*C19-ROcp56_718*S19;
    ROcp56_219 = ROcp56_217*C19-ROcp56_818*S19;
    ROcp56_319 = ROcp56_317*C19-ROcp56_918*S19;
    ROcp56_719 = ROcp56_117*S19+ROcp56_718*C19;
    ROcp56_819 = ROcp56_217*S19+ROcp56_818*C19;
    ROcp56_919 = ROcp56_317*S19+ROcp56_918*C19;
    RLcp56_114 = ROcp56_46*s.dpt(2,2);
    RLcp56_214 = ROcp56_56*s.dpt(2,2);
    RLcp56_314 = ROcp56_66*s.dpt(2,2);
    OMcp56_114 = OMcp56_16+ROcp56_46*qd(14);
    OMcp56_214 = OMcp56_26+ROcp56_56*qd(14);
    OMcp56_314 = OMcp56_36+ROcp56_66*qd(14);
    ORcp56_114 = OMcp56_26*RLcp56_314-OMcp56_36*RLcp56_214;
    ORcp56_214 = -(OMcp56_16*RLcp56_314-OMcp56_36*RLcp56_114);
    ORcp56_314 = OMcp56_16*RLcp56_214-OMcp56_26*RLcp56_114;
    OPcp56_114 = OPcp56_16+ROcp56_46*qdd(14)+qd(14)*(OMcp56_26*ROcp56_66-OMcp56_36*ROcp56_56);
    OPcp56_214 = OPcp56_26+ROcp56_56*qdd(14)-qd(14)*(OMcp56_16*ROcp56_66-OMcp56_36*ROcp56_46);
    OPcp56_314 = OPcp56_36+ROcp56_66*qdd(14)+qd(14)*(OMcp56_16*ROcp56_56-OMcp56_26*ROcp56_46);
    RLcp56_115 = ROcp56_46*s.dpt(2,20);
    RLcp56_215 = ROcp56_56*s.dpt(2,20);
    RLcp56_315 = ROcp56_66*s.dpt(2,20);
    OMcp56_115 = OMcp56_114+ROcp56_114*qd(15);
    OMcp56_215 = OMcp56_214+ROcp56_214*qd(15);
    OMcp56_315 = OMcp56_314+ROcp56_314*qd(15);
    ORcp56_115 = OMcp56_214*RLcp56_315-OMcp56_314*RLcp56_215;
    ORcp56_215 = -(OMcp56_114*RLcp56_315-OMcp56_314*RLcp56_115);
    ORcp56_315 = OMcp56_114*RLcp56_215-OMcp56_214*RLcp56_115;
    OPcp56_115 = OPcp56_114+ROcp56_114*qdd(15)+qd(15)*(OMcp56_214*ROcp56_314-OMcp56_314*ROcp56_214);
    OPcp56_215 = OPcp56_214+ROcp56_214*qdd(15)-qd(15)*(OMcp56_114*ROcp56_314-OMcp56_314*ROcp56_114);
    OPcp56_315 = OPcp56_314+ROcp56_314*qdd(15)+qd(15)*(OMcp56_114*ROcp56_214-OMcp56_214*ROcp56_114);
    RLcp56_116 = ROcp56_715*s.dpt(3,22);
    RLcp56_216 = ROcp56_815*s.dpt(3,22);
    RLcp56_316 = ROcp56_915*s.dpt(3,22);
    OMcp56_116 = OMcp56_115+ROcp56_715*qd(16);
    OMcp56_216 = OMcp56_215+ROcp56_815*qd(16);
    OMcp56_316 = OMcp56_315+ROcp56_915*qd(16);
    ORcp56_116 = OMcp56_215*RLcp56_316-OMcp56_315*RLcp56_216;
    ORcp56_216 = -(OMcp56_115*RLcp56_316-OMcp56_315*RLcp56_116);
    ORcp56_316 = OMcp56_115*RLcp56_216-OMcp56_215*RLcp56_116;
    OPcp56_116 = OPcp56_115+ROcp56_715*qdd(16)+qd(16)*(OMcp56_215*ROcp56_915-OMcp56_315*ROcp56_815);
    OPcp56_216 = OPcp56_215+ROcp56_815*qdd(16)-qd(16)*(OMcp56_115*ROcp56_915-OMcp56_315*ROcp56_715);
    OPcp56_316 = OPcp56_315+ROcp56_915*qdd(16)+qd(16)*(OMcp56_115*ROcp56_815-OMcp56_215*ROcp56_715);
    RLcp56_117 = ROcp56_715*s.dpt(3,24);
    RLcp56_217 = ROcp56_815*s.dpt(3,24);
    RLcp56_317 = ROcp56_915*s.dpt(3,24);
    OMcp56_117 = OMcp56_116+ROcp56_416*qd(17);
    OMcp56_217 = OMcp56_216+ROcp56_516*qd(17);
    OMcp56_317 = OMcp56_316+ROcp56_616*qd(17);
    ORcp56_117 = OMcp56_216*RLcp56_317-OMcp56_316*RLcp56_217;
    ORcp56_217 = -(OMcp56_116*RLcp56_317-OMcp56_316*RLcp56_117);
    ORcp56_317 = OMcp56_116*RLcp56_217-OMcp56_216*RLcp56_117;
    OPcp56_117 = OPcp56_116+ROcp56_416*qdd(17)+qd(17)*(OMcp56_216*ROcp56_616-OMcp56_316*ROcp56_516);
    OPcp56_217 = OPcp56_216+ROcp56_516*qdd(17)-qd(17)*(OMcp56_116*ROcp56_616-OMcp56_316*ROcp56_416);
    OPcp56_317 = OPcp56_316+ROcp56_616*qdd(17)+qd(17)*(OMcp56_116*ROcp56_516-OMcp56_216*ROcp56_416);
    RLcp56_118 = ROcp56_717*s.dpt(3,26);
    RLcp56_218 = ROcp56_817*s.dpt(3,26);
    RLcp56_318 = ROcp56_917*s.dpt(3,26);
    OMcp56_118 = OMcp56_117+ROcp56_117*qd(18);
    OMcp56_218 = OMcp56_217+ROcp56_217*qd(18);
    OMcp56_318 = OMcp56_317+ROcp56_317*qd(18);
    ORcp56_118 = OMcp56_217*RLcp56_318-OMcp56_317*RLcp56_218;
    ORcp56_218 = -(OMcp56_117*RLcp56_318-OMcp56_317*RLcp56_118);
    ORcp56_318 = OMcp56_117*RLcp56_218-OMcp56_217*RLcp56_118;
    OMcp56_119 = OMcp56_118+ROcp56_418*qd(19);
    OMcp56_219 = OMcp56_218+ROcp56_518*qd(19);
    OMcp56_319 = OMcp56_318+ROcp56_618*qd(19);
    OPcp56_119 = OPcp56_117+ROcp56_117*qdd(18)+ROcp56_418*qdd(19)+qd(18)*(OMcp56_217*ROcp56_317-OMcp56_317*ROcp56_217)+qd(19)*(OMcp56_218*...
 ROcp56_618-OMcp56_318*ROcp56_518);
    OPcp56_219 = OPcp56_217+ROcp56_217*qdd(18)+ROcp56_518*qdd(19)-qd(18)*(OMcp56_117*ROcp56_317-OMcp56_317*ROcp56_117)-qd(19)*(OMcp56_118*...
 ROcp56_618-OMcp56_318*ROcp56_418);
    OPcp56_319 = OPcp56_317+ROcp56_317*qdd(18)+ROcp56_618*qdd(19)+qd(18)*(OMcp56_117*ROcp56_217-OMcp56_217*ROcp56_117)+qd(19)*(OMcp56_118*...
 ROcp56_518-OMcp56_218*ROcp56_418);
    RLcp56_188 = ROcp56_719*s.dpt(3,31);
    RLcp56_288 = ROcp56_819*s.dpt(3,31);
    RLcp56_388 = ROcp56_919*s.dpt(3,31);
    POcp56_188 = RLcp56_114+RLcp56_115+RLcp56_116+RLcp56_117+RLcp56_118+RLcp56_188+q(1);
    POcp56_288 = RLcp56_214+RLcp56_215+RLcp56_216+RLcp56_217+RLcp56_218+RLcp56_288+q(2);
    POcp56_388 = RLcp56_314+RLcp56_315+RLcp56_316+RLcp56_317+RLcp56_318+RLcp56_388+q(3);
    ORcp56_188 = OMcp56_219*RLcp56_388-OMcp56_319*RLcp56_288;
    ORcp56_288 = -(OMcp56_119*RLcp56_388-OMcp56_319*RLcp56_188);
    ORcp56_388 = OMcp56_119*RLcp56_288-OMcp56_219*RLcp56_188;
    VIcp56_188 = ORcp56_114+ORcp56_115+ORcp56_116+ORcp56_117+ORcp56_118+ORcp56_188+qd(1);
    VIcp56_288 = ORcp56_214+ORcp56_215+ORcp56_216+ORcp56_217+ORcp56_218+ORcp56_288+qd(2);
    VIcp56_388 = ORcp56_314+ORcp56_315+ORcp56_316+ORcp56_317+ORcp56_318+ORcp56_388+qd(3);
    ACcp56_188 = qdd(1)+OMcp56_214*ORcp56_315+OMcp56_215*ORcp56_316+OMcp56_216*ORcp56_317+OMcp56_217*ORcp56_318+OMcp56_219*ORcp56_388+OMcp56_26*...
 ORcp56_314-OMcp56_314*ORcp56_215-OMcp56_315*ORcp56_216-OMcp56_316*ORcp56_217-OMcp56_317*ORcp56_218-OMcp56_319*ORcp56_288-OMcp56_36*ORcp56_214+...
 OPcp56_214*RLcp56_315+OPcp56_215*RLcp56_316+OPcp56_216*RLcp56_317+OPcp56_217*RLcp56_318+OPcp56_219*RLcp56_388+OPcp56_26*RLcp56_314-OPcp56_314*...
 RLcp56_215-OPcp56_315*RLcp56_216-OPcp56_316*RLcp56_217-OPcp56_317*RLcp56_218-OPcp56_319*RLcp56_288-OPcp56_36*RLcp56_214;
    ACcp56_288 = qdd(2)-OMcp56_114*ORcp56_315-OMcp56_115*ORcp56_316-OMcp56_116*ORcp56_317-OMcp56_117*ORcp56_318-OMcp56_119*ORcp56_388-OMcp56_16*...
 ORcp56_314+OMcp56_314*ORcp56_115+OMcp56_315*ORcp56_116+OMcp56_316*ORcp56_117+OMcp56_317*ORcp56_118+OMcp56_319*ORcp56_188+OMcp56_36*ORcp56_114-...
 OPcp56_114*RLcp56_315-OPcp56_115*RLcp56_316-OPcp56_116*RLcp56_317-OPcp56_117*RLcp56_318-OPcp56_119*RLcp56_388-OPcp56_16*RLcp56_314+OPcp56_314*...
 RLcp56_115+OPcp56_315*RLcp56_116+OPcp56_316*RLcp56_117+OPcp56_317*RLcp56_118+OPcp56_319*RLcp56_188+OPcp56_36*RLcp56_114;
    ACcp56_388 = qdd(3)+OMcp56_114*ORcp56_215+OMcp56_115*ORcp56_216+OMcp56_116*ORcp56_217+OMcp56_117*ORcp56_218+OMcp56_119*ORcp56_288+OMcp56_16*...
 ORcp56_214-OMcp56_214*ORcp56_115-OMcp56_215*ORcp56_116-OMcp56_216*ORcp56_117-OMcp56_217*ORcp56_118-OMcp56_219*ORcp56_188-OMcp56_26*ORcp56_114+...
 OPcp56_114*RLcp56_215+OPcp56_115*RLcp56_216+OPcp56_116*RLcp56_217+OPcp56_117*RLcp56_218+OPcp56_119*RLcp56_288+OPcp56_16*RLcp56_214-OPcp56_214*...
 RLcp56_115-OPcp56_215*RLcp56_116-OPcp56_216*RLcp56_117-OPcp56_217*RLcp56_118-OPcp56_219*RLcp56_188-OPcp56_26*RLcp56_114;

% = = Block_1_0_0_57_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp56_188;
    sens.P(2) = POcp56_288;
    sens.P(3) = POcp56_388;
    sens.R(1,1) = ROcp56_119;
    sens.R(1,2) = ROcp56_219;
    sens.R(1,3) = ROcp56_319;
    sens.R(2,1) = ROcp56_418;
    sens.R(2,2) = ROcp56_518;
    sens.R(2,3) = ROcp56_618;
    sens.R(3,1) = ROcp56_719;
    sens.R(3,2) = ROcp56_819;
    sens.R(3,3) = ROcp56_919;
    sens.V(1) = VIcp56_188;
    sens.V(2) = VIcp56_288;
    sens.V(3) = VIcp56_388;
    sens.OM(1) = OMcp56_119;
    sens.OM(2) = OMcp56_219;
    sens.OM(3) = OMcp56_319;
    sens.A(1) = ACcp56_188;
    sens.A(2) = ACcp56_288;
    sens.A(3) = ACcp56_388;
    sens.OMP(1) = OPcp56_119;
    sens.OMP(2) = OPcp56_219;
    sens.OMP(3) = OPcp56_319;
 
% 
case 58, 


% = = Block_1_0_0_58_0_1 = = 
 
% Sensor Kinematics 


    ROcp57_25 = S4*S5;
    ROcp57_35 = -C4*S5;
    ROcp57_85 = -S4*C5;
    ROcp57_95 = C4*C5;
    ROcp57_16 = C5*C6;
    ROcp57_26 = ROcp57_25*C6+C4*S6;
    ROcp57_36 = ROcp57_35*C6+S4*S6;
    ROcp57_46 = -C5*S6;
    ROcp57_56 = -(ROcp57_25*S6-C4*C6);
    ROcp57_66 = -(ROcp57_35*S6-S4*C6);
    OMcp57_25 = qd(5)*C4;
    OMcp57_35 = qd(5)*S4;
    OMcp57_16 = qd(4)+qd(6)*S5;
    OMcp57_26 = OMcp57_25+ROcp57_85*qd(6);
    OMcp57_36 = OMcp57_35+ROcp57_95*qd(6);
    OPcp57_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp57_26 = ROcp57_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp57_35*S5-ROcp57_95*qd(4));
    OPcp57_36 = ROcp57_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp57_25*S5-ROcp57_85*qd(4));

% = = Block_1_0_0_58_0_3 = = 
 
% Sensor Kinematics 


    ROcp57_114 = ROcp57_16*C14-S14*S5;
    ROcp57_214 = ROcp57_26*C14-ROcp57_85*S14;
    ROcp57_314 = ROcp57_36*C14-ROcp57_95*S14;
    ROcp57_714 = ROcp57_16*S14+C14*S5;
    ROcp57_814 = ROcp57_26*S14+ROcp57_85*C14;
    ROcp57_914 = ROcp57_36*S14+ROcp57_95*C14;
    ROcp57_415 = ROcp57_46*C15+ROcp57_714*S15;
    ROcp57_515 = ROcp57_56*C15+ROcp57_814*S15;
    ROcp57_615 = ROcp57_66*C15+ROcp57_914*S15;
    ROcp57_715 = -(ROcp57_46*S15-ROcp57_714*C15);
    ROcp57_815 = -(ROcp57_56*S15-ROcp57_814*C15);
    ROcp57_915 = -(ROcp57_66*S15-ROcp57_914*C15);
    ROcp57_116 = ROcp57_114*C16+ROcp57_415*S16;
    ROcp57_216 = ROcp57_214*C16+ROcp57_515*S16;
    ROcp57_316 = ROcp57_314*C16+ROcp57_615*S16;
    ROcp57_416 = -(ROcp57_114*S16-ROcp57_415*C16);
    ROcp57_516 = -(ROcp57_214*S16-ROcp57_515*C16);
    ROcp57_616 = -(ROcp57_314*S16-ROcp57_615*C16);
    ROcp57_117 = ROcp57_116*C17-ROcp57_715*S17;
    ROcp57_217 = ROcp57_216*C17-ROcp57_815*S17;
    ROcp57_317 = ROcp57_316*C17-ROcp57_915*S17;
    ROcp57_717 = ROcp57_116*S17+ROcp57_715*C17;
    ROcp57_817 = ROcp57_216*S17+ROcp57_815*C17;
    ROcp57_917 = ROcp57_316*S17+ROcp57_915*C17;
    ROcp57_418 = ROcp57_416*C18+ROcp57_717*S18;
    ROcp57_518 = ROcp57_516*C18+ROcp57_817*S18;
    ROcp57_618 = ROcp57_616*C18+ROcp57_917*S18;
    ROcp57_718 = -(ROcp57_416*S18-ROcp57_717*C18);
    ROcp57_818 = -(ROcp57_516*S18-ROcp57_817*C18);
    ROcp57_918 = -(ROcp57_616*S18-ROcp57_917*C18);
    ROcp57_119 = ROcp57_117*C19-ROcp57_718*S19;
    ROcp57_219 = ROcp57_217*C19-ROcp57_818*S19;
    ROcp57_319 = ROcp57_317*C19-ROcp57_918*S19;
    ROcp57_719 = ROcp57_117*S19+ROcp57_718*C19;
    ROcp57_819 = ROcp57_217*S19+ROcp57_818*C19;
    ROcp57_919 = ROcp57_317*S19+ROcp57_918*C19;
    ROcp57_120 = ROcp57_119*C20-ROcp57_719*S20;
    ROcp57_220 = ROcp57_219*C20-ROcp57_819*S20;
    ROcp57_320 = ROcp57_319*C20-ROcp57_919*S20;
    ROcp57_720 = ROcp57_119*S20+ROcp57_719*C20;
    ROcp57_820 = ROcp57_219*S20+ROcp57_819*C20;
    ROcp57_920 = ROcp57_319*S20+ROcp57_919*C20;
    RLcp57_114 = ROcp57_46*s.dpt(2,2);
    RLcp57_214 = ROcp57_56*s.dpt(2,2);
    RLcp57_314 = ROcp57_66*s.dpt(2,2);
    OMcp57_114 = OMcp57_16+ROcp57_46*qd(14);
    OMcp57_214 = OMcp57_26+ROcp57_56*qd(14);
    OMcp57_314 = OMcp57_36+ROcp57_66*qd(14);
    ORcp57_114 = OMcp57_26*RLcp57_314-OMcp57_36*RLcp57_214;
    ORcp57_214 = -(OMcp57_16*RLcp57_314-OMcp57_36*RLcp57_114);
    ORcp57_314 = OMcp57_16*RLcp57_214-OMcp57_26*RLcp57_114;
    OPcp57_114 = OPcp57_16+ROcp57_46*qdd(14)+qd(14)*(OMcp57_26*ROcp57_66-OMcp57_36*ROcp57_56);
    OPcp57_214 = OPcp57_26+ROcp57_56*qdd(14)-qd(14)*(OMcp57_16*ROcp57_66-OMcp57_36*ROcp57_46);
    OPcp57_314 = OPcp57_36+ROcp57_66*qdd(14)+qd(14)*(OMcp57_16*ROcp57_56-OMcp57_26*ROcp57_46);
    RLcp57_115 = ROcp57_46*s.dpt(2,20);
    RLcp57_215 = ROcp57_56*s.dpt(2,20);
    RLcp57_315 = ROcp57_66*s.dpt(2,20);
    OMcp57_115 = OMcp57_114+ROcp57_114*qd(15);
    OMcp57_215 = OMcp57_214+ROcp57_214*qd(15);
    OMcp57_315 = OMcp57_314+ROcp57_314*qd(15);
    ORcp57_115 = OMcp57_214*RLcp57_315-OMcp57_314*RLcp57_215;
    ORcp57_215 = -(OMcp57_114*RLcp57_315-OMcp57_314*RLcp57_115);
    ORcp57_315 = OMcp57_114*RLcp57_215-OMcp57_214*RLcp57_115;
    OPcp57_115 = OPcp57_114+ROcp57_114*qdd(15)+qd(15)*(OMcp57_214*ROcp57_314-OMcp57_314*ROcp57_214);
    OPcp57_215 = OPcp57_214+ROcp57_214*qdd(15)-qd(15)*(OMcp57_114*ROcp57_314-OMcp57_314*ROcp57_114);
    OPcp57_315 = OPcp57_314+ROcp57_314*qdd(15)+qd(15)*(OMcp57_114*ROcp57_214-OMcp57_214*ROcp57_114);
    RLcp57_116 = ROcp57_715*s.dpt(3,22);
    RLcp57_216 = ROcp57_815*s.dpt(3,22);
    RLcp57_316 = ROcp57_915*s.dpt(3,22);
    OMcp57_116 = OMcp57_115+ROcp57_715*qd(16);
    OMcp57_216 = OMcp57_215+ROcp57_815*qd(16);
    OMcp57_316 = OMcp57_315+ROcp57_915*qd(16);
    ORcp57_116 = OMcp57_215*RLcp57_316-OMcp57_315*RLcp57_216;
    ORcp57_216 = -(OMcp57_115*RLcp57_316-OMcp57_315*RLcp57_116);
    ORcp57_316 = OMcp57_115*RLcp57_216-OMcp57_215*RLcp57_116;
    OPcp57_116 = OPcp57_115+ROcp57_715*qdd(16)+qd(16)*(OMcp57_215*ROcp57_915-OMcp57_315*ROcp57_815);
    OPcp57_216 = OPcp57_215+ROcp57_815*qdd(16)-qd(16)*(OMcp57_115*ROcp57_915-OMcp57_315*ROcp57_715);
    OPcp57_316 = OPcp57_315+ROcp57_915*qdd(16)+qd(16)*(OMcp57_115*ROcp57_815-OMcp57_215*ROcp57_715);
    RLcp57_117 = ROcp57_715*s.dpt(3,24);
    RLcp57_217 = ROcp57_815*s.dpt(3,24);
    RLcp57_317 = ROcp57_915*s.dpt(3,24);
    OMcp57_117 = OMcp57_116+ROcp57_416*qd(17);
    OMcp57_217 = OMcp57_216+ROcp57_516*qd(17);
    OMcp57_317 = OMcp57_316+ROcp57_616*qd(17);
    ORcp57_117 = OMcp57_216*RLcp57_317-OMcp57_316*RLcp57_217;
    ORcp57_217 = -(OMcp57_116*RLcp57_317-OMcp57_316*RLcp57_117);
    ORcp57_317 = OMcp57_116*RLcp57_217-OMcp57_216*RLcp57_117;
    OPcp57_117 = OPcp57_116+ROcp57_416*qdd(17)+qd(17)*(OMcp57_216*ROcp57_616-OMcp57_316*ROcp57_516);
    OPcp57_217 = OPcp57_216+ROcp57_516*qdd(17)-qd(17)*(OMcp57_116*ROcp57_616-OMcp57_316*ROcp57_416);
    OPcp57_317 = OPcp57_316+ROcp57_616*qdd(17)+qd(17)*(OMcp57_116*ROcp57_516-OMcp57_216*ROcp57_416);
    RLcp57_118 = ROcp57_717*s.dpt(3,26);
    RLcp57_218 = ROcp57_817*s.dpt(3,26);
    RLcp57_318 = ROcp57_917*s.dpt(3,26);
    OMcp57_118 = OMcp57_117+ROcp57_117*qd(18);
    OMcp57_218 = OMcp57_217+ROcp57_217*qd(18);
    OMcp57_318 = OMcp57_317+ROcp57_317*qd(18);
    ORcp57_118 = OMcp57_217*RLcp57_318-OMcp57_317*RLcp57_218;
    ORcp57_218 = -(OMcp57_117*RLcp57_318-OMcp57_317*RLcp57_118);
    ORcp57_318 = OMcp57_117*RLcp57_218-OMcp57_217*RLcp57_118;
    OMcp57_119 = OMcp57_118+ROcp57_418*qd(19);
    OMcp57_219 = OMcp57_218+ROcp57_518*qd(19);
    OMcp57_319 = OMcp57_318+ROcp57_618*qd(19);
    OPcp57_119 = OPcp57_117+ROcp57_117*qdd(18)+ROcp57_418*qdd(19)+qd(18)*(OMcp57_217*ROcp57_317-OMcp57_317*ROcp57_217)+qd(19)*(OMcp57_218*...
 ROcp57_618-OMcp57_318*ROcp57_518);
    OPcp57_219 = OPcp57_217+ROcp57_217*qdd(18)+ROcp57_518*qdd(19)-qd(18)*(OMcp57_117*ROcp57_317-OMcp57_317*ROcp57_117)-qd(19)*(OMcp57_118*...
 ROcp57_618-OMcp57_318*ROcp57_418);
    OPcp57_319 = OPcp57_317+ROcp57_317*qdd(18)+ROcp57_618*qdd(19)+qd(18)*(OMcp57_117*ROcp57_217-OMcp57_217*ROcp57_117)+qd(19)*(OMcp57_118*...
 ROcp57_518-OMcp57_218*ROcp57_418);
    RLcp57_120 = ROcp57_119*s.dpt(1,32)+ROcp57_719*s.dpt(3,32);
    RLcp57_220 = ROcp57_219*s.dpt(1,32)+ROcp57_819*s.dpt(3,32);
    RLcp57_320 = ROcp57_319*s.dpt(1,32)+ROcp57_919*s.dpt(3,32);
    POcp57_120 = RLcp57_114+RLcp57_115+RLcp57_116+RLcp57_117+RLcp57_118+RLcp57_120+q(1);
    POcp57_220 = RLcp57_214+RLcp57_215+RLcp57_216+RLcp57_217+RLcp57_218+RLcp57_220+q(2);
    POcp57_320 = RLcp57_314+RLcp57_315+RLcp57_316+RLcp57_317+RLcp57_318+RLcp57_320+q(3);
    OMcp57_120 = OMcp57_119+ROcp57_418*qd(20);
    OMcp57_220 = OMcp57_219+ROcp57_518*qd(20);
    OMcp57_320 = OMcp57_319+ROcp57_618*qd(20);
    ORcp57_120 = OMcp57_219*RLcp57_320-OMcp57_319*RLcp57_220;
    ORcp57_220 = -(OMcp57_119*RLcp57_320-OMcp57_319*RLcp57_120);
    ORcp57_320 = OMcp57_119*RLcp57_220-OMcp57_219*RLcp57_120;
    VIcp57_120 = ORcp57_114+ORcp57_115+ORcp57_116+ORcp57_117+ORcp57_118+ORcp57_120+qd(1);
    VIcp57_220 = ORcp57_214+ORcp57_215+ORcp57_216+ORcp57_217+ORcp57_218+ORcp57_220+qd(2);
    VIcp57_320 = ORcp57_314+ORcp57_315+ORcp57_316+ORcp57_317+ORcp57_318+ORcp57_320+qd(3);
    OPcp57_120 = OPcp57_119+ROcp57_418*qdd(20)+qd(20)*(OMcp57_219*ROcp57_618-OMcp57_319*ROcp57_518);
    OPcp57_220 = OPcp57_219+ROcp57_518*qdd(20)-qd(20)*(OMcp57_119*ROcp57_618-OMcp57_319*ROcp57_418);
    OPcp57_320 = OPcp57_319+ROcp57_618*qdd(20)+qd(20)*(OMcp57_119*ROcp57_518-OMcp57_219*ROcp57_418);
    ACcp57_120 = qdd(1)+OMcp57_214*ORcp57_315+OMcp57_215*ORcp57_316+OMcp57_216*ORcp57_317+OMcp57_217*ORcp57_318+OMcp57_219*ORcp57_320+OMcp57_26*...
 ORcp57_314-OMcp57_314*ORcp57_215-OMcp57_315*ORcp57_216-OMcp57_316*ORcp57_217-OMcp57_317*ORcp57_218-OMcp57_319*ORcp57_220-OMcp57_36*ORcp57_214+...
 OPcp57_214*RLcp57_315+OPcp57_215*RLcp57_316+OPcp57_216*RLcp57_317+OPcp57_217*RLcp57_318+OPcp57_219*RLcp57_320+OPcp57_26*RLcp57_314-OPcp57_314*...
 RLcp57_215-OPcp57_315*RLcp57_216-OPcp57_316*RLcp57_217-OPcp57_317*RLcp57_218-OPcp57_319*RLcp57_220-OPcp57_36*RLcp57_214;
    ACcp57_220 = qdd(2)-OMcp57_114*ORcp57_315-OMcp57_115*ORcp57_316-OMcp57_116*ORcp57_317-OMcp57_117*ORcp57_318-OMcp57_119*ORcp57_320-OMcp57_16*...
 ORcp57_314+OMcp57_314*ORcp57_115+OMcp57_315*ORcp57_116+OMcp57_316*ORcp57_117+OMcp57_317*ORcp57_118+OMcp57_319*ORcp57_120+OMcp57_36*ORcp57_114-...
 OPcp57_114*RLcp57_315-OPcp57_115*RLcp57_316-OPcp57_116*RLcp57_317-OPcp57_117*RLcp57_318-OPcp57_119*RLcp57_320-OPcp57_16*RLcp57_314+OPcp57_314*...
 RLcp57_115+OPcp57_315*RLcp57_116+OPcp57_316*RLcp57_117+OPcp57_317*RLcp57_118+OPcp57_319*RLcp57_120+OPcp57_36*RLcp57_114;
    ACcp57_320 = qdd(3)+OMcp57_114*ORcp57_215+OMcp57_115*ORcp57_216+OMcp57_116*ORcp57_217+OMcp57_117*ORcp57_218+OMcp57_119*ORcp57_220+OMcp57_16*...
 ORcp57_214-OMcp57_214*ORcp57_115-OMcp57_215*ORcp57_116-OMcp57_216*ORcp57_117-OMcp57_217*ORcp57_118-OMcp57_219*ORcp57_120-OMcp57_26*ORcp57_114+...
 OPcp57_114*RLcp57_215+OPcp57_115*RLcp57_216+OPcp57_116*RLcp57_217+OPcp57_117*RLcp57_218+OPcp57_119*RLcp57_220+OPcp57_16*RLcp57_214-OPcp57_214*...
 RLcp57_115-OPcp57_215*RLcp57_116-OPcp57_216*RLcp57_117-OPcp57_217*RLcp57_118-OPcp57_219*RLcp57_120-OPcp57_26*RLcp57_114;

% = = Block_1_0_0_58_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp57_120;
    sens.P(2) = POcp57_220;
    sens.P(3) = POcp57_320;
    sens.R(1,1) = ROcp57_120;
    sens.R(1,2) = ROcp57_220;
    sens.R(1,3) = ROcp57_320;
    sens.R(2,1) = ROcp57_418;
    sens.R(2,2) = ROcp57_518;
    sens.R(2,3) = ROcp57_618;
    sens.R(3,1) = ROcp57_720;
    sens.R(3,2) = ROcp57_820;
    sens.R(3,3) = ROcp57_920;
    sens.V(1) = VIcp57_120;
    sens.V(2) = VIcp57_220;
    sens.V(3) = VIcp57_320;
    sens.OM(1) = OMcp57_120;
    sens.OM(2) = OMcp57_220;
    sens.OM(3) = OMcp57_320;
    sens.A(1) = ACcp57_120;
    sens.A(2) = ACcp57_220;
    sens.A(3) = ACcp57_320;
    sens.OMP(1) = OPcp57_120;
    sens.OMP(2) = OPcp57_220;
    sens.OMP(3) = OPcp57_320;
 
% 
case 59, 


% = = Block_1_0_0_59_0_1 = = 
 
% Sensor Kinematics 


    ROcp58_25 = S4*S5;
    ROcp58_35 = -C4*S5;
    ROcp58_85 = -S4*C5;
    ROcp58_95 = C4*C5;
    ROcp58_16 = C5*C6;
    ROcp58_26 = ROcp58_25*C6+C4*S6;
    ROcp58_36 = ROcp58_35*C6+S4*S6;
    ROcp58_46 = -C5*S6;
    ROcp58_56 = -(ROcp58_25*S6-C4*C6);
    ROcp58_66 = -(ROcp58_35*S6-S4*C6);
    OMcp58_25 = qd(5)*C4;
    OMcp58_35 = qd(5)*S4;
    OMcp58_16 = qd(4)+qd(6)*S5;
    OMcp58_26 = OMcp58_25+ROcp58_85*qd(6);
    OMcp58_36 = OMcp58_35+ROcp58_95*qd(6);
    OPcp58_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp58_26 = ROcp58_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp58_35*S5-ROcp58_95*qd(4));
    OPcp58_36 = ROcp58_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp58_25*S5-ROcp58_85*qd(4));

% = = Block_1_0_0_59_0_4 = = 
 
% Sensor Kinematics 


    ROcp58_421 = ROcp58_46*C21+S21*S5;
    ROcp58_521 = ROcp58_56*C21+ROcp58_85*S21;
    ROcp58_621 = ROcp58_66*C21+ROcp58_95*S21;
    ROcp58_721 = -(ROcp58_46*S21-C21*S5);
    ROcp58_821 = -(ROcp58_56*S21-ROcp58_85*C21);
    ROcp58_921 = -(ROcp58_66*S21-ROcp58_95*C21);
    ROcp58_122 = ROcp58_16*C22-ROcp58_721*S22;
    ROcp58_222 = ROcp58_26*C22-ROcp58_821*S22;
    ROcp58_322 = ROcp58_36*C22-ROcp58_921*S22;
    ROcp58_722 = ROcp58_16*S22+ROcp58_721*C22;
    ROcp58_822 = ROcp58_26*S22+ROcp58_821*C22;
    ROcp58_922 = ROcp58_36*S22+ROcp58_921*C22;
    ROcp58_123 = ROcp58_122*C23+ROcp58_421*S23;
    ROcp58_223 = ROcp58_222*C23+ROcp58_521*S23;
    ROcp58_323 = ROcp58_322*C23+ROcp58_621*S23;
    ROcp58_423 = -(ROcp58_122*S23-ROcp58_421*C23);
    ROcp58_523 = -(ROcp58_222*S23-ROcp58_521*C23);
    ROcp58_623 = -(ROcp58_322*S23-ROcp58_621*C23);
    RLcp58_121 = ROcp58_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp58_221 = ROcp58_26*s.dpt(1,3)+ROcp58_85*s.dpt(3,3);
    RLcp58_321 = ROcp58_36*s.dpt(1,3)+ROcp58_95*s.dpt(3,3);
    OMcp58_121 = OMcp58_16+ROcp58_16*qd(21);
    OMcp58_221 = OMcp58_26+ROcp58_26*qd(21);
    OMcp58_321 = OMcp58_36+ROcp58_36*qd(21);
    ORcp58_121 = OMcp58_26*RLcp58_321-OMcp58_36*RLcp58_221;
    ORcp58_221 = -(OMcp58_16*RLcp58_321-OMcp58_36*RLcp58_121);
    ORcp58_321 = OMcp58_16*RLcp58_221-OMcp58_26*RLcp58_121;
    OMcp58_122 = OMcp58_121+ROcp58_421*qd(22);
    OMcp58_222 = OMcp58_221+ROcp58_521*qd(22);
    OMcp58_322 = OMcp58_321+ROcp58_621*qd(22);
    OPcp58_122 = OPcp58_16+ROcp58_16*qdd(21)+ROcp58_421*qdd(22)+qd(21)*(OMcp58_26*ROcp58_36-OMcp58_36*ROcp58_26)+qd(22)*(OMcp58_221*ROcp58_621-...
 OMcp58_321*ROcp58_521);
    OPcp58_222 = OPcp58_26+ROcp58_26*qdd(21)+ROcp58_521*qdd(22)-qd(21)*(OMcp58_16*ROcp58_36-OMcp58_36*ROcp58_16)-qd(22)*(OMcp58_121*ROcp58_621-...
 OMcp58_321*ROcp58_421);
    OPcp58_322 = OPcp58_36+ROcp58_36*qdd(21)+ROcp58_621*qdd(22)+qd(21)*(OMcp58_16*ROcp58_26-OMcp58_26*ROcp58_16)+qd(22)*(OMcp58_121*ROcp58_521-...
 OMcp58_221*ROcp58_421);
    RLcp58_123 = ROcp58_722*s.dpt(3,37);
    RLcp58_223 = ROcp58_822*s.dpt(3,37);
    RLcp58_323 = ROcp58_922*s.dpt(3,37);
    OMcp58_123 = OMcp58_122+ROcp58_722*qd(23);
    OMcp58_223 = OMcp58_222+ROcp58_822*qd(23);
    OMcp58_323 = OMcp58_322+ROcp58_922*qd(23);
    ORcp58_123 = OMcp58_222*RLcp58_323-OMcp58_322*RLcp58_223;
    ORcp58_223 = -(OMcp58_122*RLcp58_323-OMcp58_322*RLcp58_123);
    ORcp58_323 = OMcp58_122*RLcp58_223-OMcp58_222*RLcp58_123;
    OPcp58_123 = OPcp58_122+ROcp58_722*qdd(23)+qd(23)*(OMcp58_222*ROcp58_922-OMcp58_322*ROcp58_822);
    OPcp58_223 = OPcp58_222+ROcp58_822*qdd(23)-qd(23)*(OMcp58_122*ROcp58_922-OMcp58_322*ROcp58_722);
    OPcp58_323 = OPcp58_322+ROcp58_922*qdd(23)+qd(23)*(OMcp58_122*ROcp58_822-OMcp58_222*ROcp58_722);
    RLcp58_190 = ROcp58_722*s.dpt(3,39);
    RLcp58_290 = ROcp58_822*s.dpt(3,39);
    RLcp58_390 = ROcp58_922*s.dpt(3,39);
    POcp58_190 = RLcp58_121+RLcp58_123+RLcp58_190+q(1);
    POcp58_290 = RLcp58_221+RLcp58_223+RLcp58_290+q(2);
    POcp58_390 = RLcp58_321+RLcp58_323+RLcp58_390+q(3);
    ORcp58_190 = OMcp58_223*RLcp58_390-OMcp58_323*RLcp58_290;
    ORcp58_290 = -(OMcp58_123*RLcp58_390-OMcp58_323*RLcp58_190);
    ORcp58_390 = OMcp58_123*RLcp58_290-OMcp58_223*RLcp58_190;
    VIcp58_190 = ORcp58_121+ORcp58_123+ORcp58_190+qd(1);
    VIcp58_290 = ORcp58_221+ORcp58_223+ORcp58_290+qd(2);
    VIcp58_390 = ORcp58_321+ORcp58_323+ORcp58_390+qd(3);
    ACcp58_190 = qdd(1)+OMcp58_222*ORcp58_323+OMcp58_223*ORcp58_390+OMcp58_26*ORcp58_321-OMcp58_322*ORcp58_223-OMcp58_323*ORcp58_290-OMcp58_36*...
 ORcp58_221+OPcp58_222*RLcp58_323+OPcp58_223*RLcp58_390+OPcp58_26*RLcp58_321-OPcp58_322*RLcp58_223-OPcp58_323*RLcp58_290-OPcp58_36*RLcp58_221;
    ACcp58_290 = qdd(2)-OMcp58_122*ORcp58_323-OMcp58_123*ORcp58_390-OMcp58_16*ORcp58_321+OMcp58_322*ORcp58_123+OMcp58_323*ORcp58_190+OMcp58_36*...
 ORcp58_121-OPcp58_122*RLcp58_323-OPcp58_123*RLcp58_390-OPcp58_16*RLcp58_321+OPcp58_322*RLcp58_123+OPcp58_323*RLcp58_190+OPcp58_36*RLcp58_121;
    ACcp58_390 = qdd(3)+OMcp58_122*ORcp58_223+OMcp58_123*ORcp58_290+OMcp58_16*ORcp58_221-OMcp58_222*ORcp58_123-OMcp58_223*ORcp58_190-OMcp58_26*...
 ORcp58_121+OPcp58_122*RLcp58_223+OPcp58_123*RLcp58_290+OPcp58_16*RLcp58_221-OPcp58_222*RLcp58_123-OPcp58_223*RLcp58_190-OPcp58_26*RLcp58_121;

% = = Block_1_0_0_59_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp58_190;
    sens.P(2) = POcp58_290;
    sens.P(3) = POcp58_390;
    sens.R(1,1) = ROcp58_123;
    sens.R(1,2) = ROcp58_223;
    sens.R(1,3) = ROcp58_323;
    sens.R(2,1) = ROcp58_423;
    sens.R(2,2) = ROcp58_523;
    sens.R(2,3) = ROcp58_623;
    sens.R(3,1) = ROcp58_722;
    sens.R(3,2) = ROcp58_822;
    sens.R(3,3) = ROcp58_922;
    sens.V(1) = VIcp58_190;
    sens.V(2) = VIcp58_290;
    sens.V(3) = VIcp58_390;
    sens.OM(1) = OMcp58_123;
    sens.OM(2) = OMcp58_223;
    sens.OM(3) = OMcp58_323;
    sens.A(1) = ACcp58_190;
    sens.A(2) = ACcp58_290;
    sens.A(3) = ACcp58_390;
    sens.OMP(1) = OPcp58_123;
    sens.OMP(2) = OPcp58_223;
    sens.OMP(3) = OPcp58_323;

end


% ====== END Task 1 ====== 

  

