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
%	==> Generation Date : Tue Nov  4 14:52:29 2014
%
%	==> Project name : coman_robotran
%	==> using XML input file 
%
%	==> Number of joints : 33
%
%	==> Function : F19 : External Forces
%	==> Flops complexity : 1656
%
%	==> Generation Time :  0.040 seconds
%	==> Post-Processing :  0.030 seconds
%
%-------------------------------------------------------------
%
function [frc,trq] = extforces(s,tsim,usrfun)

 frc = zeros(3,33);
 trq = zeros(3,33);

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

% = = Block_0_0_1_1_0_1 = = 
 
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
  OMcp52_26 = OMcp52_25+qd(6)*ROcp52_85;
  OMcp52_36 = OMcp52_35+qd(6)*ROcp52_95;
  OPcp52_16 = qdd(4)+qd(5)*qd(6)*C5+qdd(6)*S5;
  OPcp52_26 = -(qd(4)*qd(5)*S4+qd(6)*(qd(4)*ROcp52_95-OMcp52_35*S5)-qdd(5)*C4-qdd(6)*ROcp52_85);
  OPcp52_36 = qd(4)*qd(5)*C4+qd(6)*(qd(4)*ROcp52_85-OMcp52_25*S5)+qdd(5)*S4+qdd(6)*ROcp52_95;

% = = Block_0_0_1_1_0_2 = = 
 
% Sensor Kinematics 


  ROcp52_17 = ROcp52_16*C7-S5*S7;
  ROcp52_27 = ROcp52_26*C7-ROcp52_85*S7;
  ROcp52_37 = ROcp52_36*C7-ROcp52_95*S7;
  ROcp52_77 = ROcp52_16*S7+S5*C7;
  ROcp52_87 = ROcp52_26*S7+ROcp52_85*C7;
  ROcp52_97 = ROcp52_36*S7+ROcp52_95*C7;
  ROcp52_48 = ROcp52_46*C8+ROcp52_77*S8;
  ROcp52_58 = ROcp52_56*C8+ROcp52_87*S8;
  ROcp52_68 = ROcp52_66*C8+ROcp52_97*S8;
  ROcp52_78 = -(ROcp52_46*S8-ROcp52_77*C8);
  ROcp52_88 = -(ROcp52_56*S8-ROcp52_87*C8);
  ROcp52_98 = -(ROcp52_66*S8-ROcp52_97*C8);
  ROcp52_19 = ROcp52_17*C9+ROcp52_48*S9;
  ROcp52_29 = ROcp52_27*C9+ROcp52_58*S9;
  ROcp52_39 = ROcp52_37*C9+ROcp52_68*S9;
  ROcp52_49 = -(ROcp52_17*S9-ROcp52_48*C9);
  ROcp52_59 = -(ROcp52_27*S9-ROcp52_58*C9);
  ROcp52_69 = -(ROcp52_37*S9-ROcp52_68*C9);
  ROcp52_110 = ROcp52_19*C10-ROcp52_78*S10;
  ROcp52_210 = ROcp52_29*C10-ROcp52_88*S10;
  ROcp52_310 = ROcp52_39*C10-ROcp52_98*S10;
  ROcp52_710 = ROcp52_19*S10+ROcp52_78*C10;
  ROcp52_810 = ROcp52_29*S10+ROcp52_88*C10;
  ROcp52_910 = ROcp52_39*S10+ROcp52_98*C10;
  ROcp52_411 = ROcp52_49*C11+ROcp52_710*S11;
  ROcp52_511 = ROcp52_59*C11+ROcp52_810*S11;
  ROcp52_611 = ROcp52_69*C11+ROcp52_910*S11;
  ROcp52_711 = -(ROcp52_49*S11-ROcp52_710*C11);
  ROcp52_811 = -(ROcp52_59*S11-ROcp52_810*C11);
  ROcp52_911 = -(ROcp52_69*S11-ROcp52_910*C11);
  ROcp52_112 = ROcp52_110*C12-ROcp52_711*S12;
  ROcp52_212 = ROcp52_210*C12-ROcp52_811*S12;
  ROcp52_312 = ROcp52_310*C12-ROcp52_911*S12;
  ROcp52_712 = ROcp52_110*S12+ROcp52_711*C12;
  ROcp52_812 = ROcp52_210*S12+ROcp52_811*C12;
  ROcp52_912 = ROcp52_310*S12+ROcp52_911*C12;
  RLcp52_17 = ROcp52_46*s.dpt(2,3);
  RLcp52_27 = ROcp52_56*s.dpt(2,3);
  RLcp52_37 = ROcp52_66*s.dpt(2,3);
  OMcp52_17 = OMcp52_16+qd(7)*ROcp52_46;
  OMcp52_27 = OMcp52_26+qd(7)*ROcp52_56;
  OMcp52_37 = OMcp52_36+qd(7)*ROcp52_66;
  ORcp52_17 = OMcp52_26*RLcp52_37-OMcp52_36*RLcp52_27;
  ORcp52_27 = -(OMcp52_16*RLcp52_37-OMcp52_36*RLcp52_17);
  ORcp52_37 = OMcp52_16*RLcp52_27-OMcp52_26*RLcp52_17;
  OPcp52_17 = OPcp52_16+qd(7)*(OMcp52_26*ROcp52_66-OMcp52_36*ROcp52_56)+qdd(7)*ROcp52_46;
  OPcp52_27 = OPcp52_26-qd(7)*(OMcp52_16*ROcp52_66-OMcp52_36*ROcp52_46)+qdd(7)*ROcp52_56;
  OPcp52_37 = OPcp52_36+qd(7)*(OMcp52_16*ROcp52_56-OMcp52_26*ROcp52_46)+qdd(7)*ROcp52_66;
  RLcp52_18 = ROcp52_46*s.dpt(2,8);
  RLcp52_28 = ROcp52_56*s.dpt(2,8);
  RLcp52_38 = ROcp52_66*s.dpt(2,8);
  OMcp52_18 = OMcp52_17+qd(8)*ROcp52_17;
  OMcp52_28 = OMcp52_27+qd(8)*ROcp52_27;
  OMcp52_38 = OMcp52_37+qd(8)*ROcp52_37;
  ORcp52_18 = OMcp52_27*RLcp52_38-OMcp52_37*RLcp52_28;
  ORcp52_28 = -(OMcp52_17*RLcp52_38-OMcp52_37*RLcp52_18);
  ORcp52_38 = OMcp52_17*RLcp52_28-OMcp52_27*RLcp52_18;
  OPcp52_18 = OPcp52_17+qd(8)*(OMcp52_27*ROcp52_37-OMcp52_37*ROcp52_27)+qdd(8)*ROcp52_17;
  OPcp52_28 = OPcp52_27-qd(8)*(OMcp52_17*ROcp52_37-OMcp52_37*ROcp52_17)+qdd(8)*ROcp52_27;
  OPcp52_38 = OPcp52_37+qd(8)*(OMcp52_17*ROcp52_27-OMcp52_27*ROcp52_17)+qdd(8)*ROcp52_37;
  RLcp52_19 = ROcp52_78*s.dpt(3,10);
  RLcp52_29 = ROcp52_88*s.dpt(3,10);
  RLcp52_39 = ROcp52_98*s.dpt(3,10);
  OMcp52_19 = OMcp52_18+qd(9)*ROcp52_78;
  OMcp52_29 = OMcp52_28+qd(9)*ROcp52_88;
  OMcp52_39 = OMcp52_38+qd(9)*ROcp52_98;
  ORcp52_19 = OMcp52_28*RLcp52_39-OMcp52_38*RLcp52_29;
  ORcp52_29 = -(OMcp52_18*RLcp52_39-OMcp52_38*RLcp52_19);
  ORcp52_39 = OMcp52_18*RLcp52_29-OMcp52_28*RLcp52_19;
  OPcp52_19 = OPcp52_18+qd(9)*(OMcp52_28*ROcp52_98-OMcp52_38*ROcp52_88)+qdd(9)*ROcp52_78;
  OPcp52_29 = OPcp52_28-qd(9)*(OMcp52_18*ROcp52_98-OMcp52_38*ROcp52_78)+qdd(9)*ROcp52_88;
  OPcp52_39 = OPcp52_38+qd(9)*(OMcp52_18*ROcp52_88-OMcp52_28*ROcp52_78)+qdd(9)*ROcp52_98;
  RLcp52_110 = ROcp52_78*s.dpt(3,12);
  RLcp52_210 = ROcp52_88*s.dpt(3,12);
  RLcp52_310 = ROcp52_98*s.dpt(3,12);
  OMcp52_110 = OMcp52_19+qd(10)*ROcp52_49;
  OMcp52_210 = OMcp52_29+qd(10)*ROcp52_59;
  OMcp52_310 = OMcp52_39+qd(10)*ROcp52_69;
  ORcp52_110 = OMcp52_29*RLcp52_310-OMcp52_39*RLcp52_210;
  ORcp52_210 = -(OMcp52_19*RLcp52_310-OMcp52_39*RLcp52_110);
  ORcp52_310 = OMcp52_19*RLcp52_210-OMcp52_29*RLcp52_110;
  OPcp52_110 = OPcp52_19+qd(10)*(OMcp52_29*ROcp52_69-OMcp52_39*ROcp52_59)+qdd(10)*ROcp52_49;
  OPcp52_210 = OPcp52_29-qd(10)*(OMcp52_19*ROcp52_69-OMcp52_39*ROcp52_49)+qdd(10)*ROcp52_59;
  OPcp52_310 = OPcp52_39+qd(10)*(OMcp52_19*ROcp52_59-OMcp52_29*ROcp52_49)+qdd(10)*ROcp52_69;
  RLcp52_111 = ROcp52_710*s.dpt(3,14);
  RLcp52_211 = ROcp52_810*s.dpt(3,14);
  RLcp52_311 = ROcp52_910*s.dpt(3,14);
  OMcp52_111 = OMcp52_110+qd(11)*ROcp52_110;
  OMcp52_211 = OMcp52_210+qd(11)*ROcp52_210;
  OMcp52_311 = OMcp52_310+qd(11)*ROcp52_310;
  ORcp52_111 = OMcp52_210*RLcp52_311-OMcp52_310*RLcp52_211;
  ORcp52_211 = -(OMcp52_110*RLcp52_311-OMcp52_310*RLcp52_111);
  ORcp52_311 = OMcp52_110*RLcp52_211-OMcp52_210*RLcp52_111;
  OMcp52_112 = OMcp52_111+qd(12)*ROcp52_411;
  OMcp52_212 = OMcp52_211+qd(12)*ROcp52_511;
  OMcp52_312 = OMcp52_311+qd(12)*ROcp52_611;
  OPcp52_112 = OPcp52_110+qd(11)*(OMcp52_210*ROcp52_310-OMcp52_310*ROcp52_210)+qd(12)*(OMcp52_211*ROcp52_611-OMcp52_311*ROcp52_511)+qdd(11)*...
 ROcp52_110+qdd(12)*ROcp52_411;
  OPcp52_212 = OPcp52_210-qd(11)*(OMcp52_110*ROcp52_310-OMcp52_310*ROcp52_110)-qd(12)*(OMcp52_111*ROcp52_611-OMcp52_311*ROcp52_411)+qdd(11)*...
 ROcp52_210+qdd(12)*ROcp52_511;
  OPcp52_312 = OPcp52_310+qd(11)*(OMcp52_110*ROcp52_210-OMcp52_210*ROcp52_110)+qd(12)*(OMcp52_111*ROcp52_511-OMcp52_211*ROcp52_411)+qdd(11)*...
 ROcp52_310+qdd(12)*ROcp52_611;
  RLcp52_190 = ROcp52_712*s.dpt(3,18);
  RLcp52_290 = ROcp52_812*s.dpt(3,18);
  RLcp52_390 = ROcp52_912*s.dpt(3,18);
  ORcp52_190 = OMcp52_212*RLcp52_390-OMcp52_312*RLcp52_290;
  ORcp52_290 = -(OMcp52_112*RLcp52_390-OMcp52_312*RLcp52_190);
  ORcp52_390 = OMcp52_112*RLcp52_290-OMcp52_212*RLcp52_190;
  PxF1(1) = q(1)+RLcp52_110+RLcp52_111+RLcp52_17+RLcp52_18+RLcp52_19+RLcp52_190;
  PxF1(2) = q(2)+RLcp52_210+RLcp52_211+RLcp52_27+RLcp52_28+RLcp52_29+RLcp52_290;
  PxF1(3) = q(3)+RLcp52_310+RLcp52_311+RLcp52_37+RLcp52_38+RLcp52_39+RLcp52_390;
  RxF1(1,1) = ROcp52_112;
  RxF1(1,2) = ROcp52_212;
  RxF1(1,3) = ROcp52_312;
  RxF1(2,1) = ROcp52_411;
  RxF1(2,2) = ROcp52_511;
  RxF1(2,3) = ROcp52_611;
  RxF1(3,1) = ROcp52_712;
  RxF1(3,2) = ROcp52_812;
  RxF1(3,3) = ROcp52_912;
  VxF1(1) = qd(1)+ORcp52_110+ORcp52_111+ORcp52_17+ORcp52_18+ORcp52_19+ORcp52_190;
  VxF1(2) = qd(2)+ORcp52_210+ORcp52_211+ORcp52_27+ORcp52_28+ORcp52_29+ORcp52_290;
  VxF1(3) = qd(3)+ORcp52_310+ORcp52_311+ORcp52_37+ORcp52_38+ORcp52_39+ORcp52_390;
  OMxF1(1) = OMcp52_112;
  OMxF1(2) = OMcp52_212;
  OMxF1(3) = OMcp52_312;
  AxF1(1) = qdd(1)+OMcp52_210*ORcp52_311+OMcp52_212*ORcp52_390+OMcp52_26*ORcp52_37+OMcp52_27*ORcp52_38+OMcp52_28*ORcp52_39+OMcp52_29*ORcp52_310-...
 OMcp52_310*ORcp52_211-OMcp52_312*ORcp52_290-OMcp52_36*ORcp52_27-OMcp52_37*ORcp52_28-OMcp52_38*ORcp52_29-OMcp52_39*ORcp52_210+OPcp52_210*RLcp52_311+...
 OPcp52_212*RLcp52_390+OPcp52_26*RLcp52_37+OPcp52_27*RLcp52_38+OPcp52_28*RLcp52_39+OPcp52_29*RLcp52_310-OPcp52_310*RLcp52_211-OPcp52_312*RLcp52_290-...
 OPcp52_36*RLcp52_27-OPcp52_37*RLcp52_28-OPcp52_38*RLcp52_29-OPcp52_39*RLcp52_210;
  AxF1(2) = qdd(2)-OMcp52_110*ORcp52_311-OMcp52_112*ORcp52_390-OMcp52_16*ORcp52_37-OMcp52_17*ORcp52_38-OMcp52_18*ORcp52_39-OMcp52_19*ORcp52_310+...
 OMcp52_310*ORcp52_111+OMcp52_312*ORcp52_190+OMcp52_36*ORcp52_17+OMcp52_37*ORcp52_18+OMcp52_38*ORcp52_19+OMcp52_39*ORcp52_110-OPcp52_110*RLcp52_311-...
 OPcp52_112*RLcp52_390-OPcp52_16*RLcp52_37-OPcp52_17*RLcp52_38-OPcp52_18*RLcp52_39-OPcp52_19*RLcp52_310+OPcp52_310*RLcp52_111+OPcp52_312*RLcp52_190+...
 OPcp52_36*RLcp52_17+OPcp52_37*RLcp52_18+OPcp52_38*RLcp52_19+OPcp52_39*RLcp52_110;
  AxF1(3) = qdd(3)+OMcp52_110*ORcp52_211+OMcp52_112*ORcp52_290+OMcp52_16*ORcp52_27+OMcp52_17*ORcp52_28+OMcp52_18*ORcp52_29+OMcp52_19*ORcp52_210-...
 OMcp52_210*ORcp52_111-OMcp52_212*ORcp52_190-OMcp52_26*ORcp52_17-OMcp52_27*ORcp52_18-OMcp52_28*ORcp52_19-OMcp52_29*ORcp52_110+OPcp52_110*RLcp52_211+...
 OPcp52_112*RLcp52_290+OPcp52_16*RLcp52_27+OPcp52_17*RLcp52_28+OPcp52_18*RLcp52_29+OPcp52_19*RLcp52_210-OPcp52_210*RLcp52_111-OPcp52_212*RLcp52_190-...
 OPcp52_26*RLcp52_17-OPcp52_27*RLcp52_18-OPcp52_28*RLcp52_19-OPcp52_29*RLcp52_110;
  OMPxF1(1) = OPcp52_112;
  OMPxF1(2) = OPcp52_212;
  OMPxF1(3) = OPcp52_312;
 
% Sensor Forces Computation 

  SWr1 = usrfun.fext(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc153 = ROcp52_112*SWr1(1)+ROcp52_212*SWr1(2)+ROcp52_312*SWr1(3);
  xfrc253 = ROcp52_411*SWr1(1)+ROcp52_511*SWr1(2)+ROcp52_611*SWr1(3);
  xfrc353 = ROcp52_712*SWr1(1)+ROcp52_812*SWr1(2)+ROcp52_912*SWr1(3);
  frc(1,12) = s.frc(1,12)+xfrc153;
  frc(2,12) = s.frc(2,12)+xfrc253;
  frc(3,12) = s.frc(3,12)+xfrc353;
  xtrq153 = ROcp52_112*SWr1(4)+ROcp52_212*SWr1(5)+ROcp52_312*SWr1(6);
  xtrq253 = ROcp52_411*SWr1(4)+ROcp52_511*SWr1(5)+ROcp52_611*SWr1(6);
  xtrq353 = ROcp52_712*SWr1(4)+ROcp52_812*SWr1(5)+ROcp52_912*SWr1(6);
  trq(1,12) = s.trq(1,12)+xtrq153-xfrc253*(SWr1(9)-s.l(3,12))+xfrc353*(SWr1(8)-s.l(2,12));
  trq(2,12) = s.trq(2,12)+xtrq253+xfrc153*(SWr1(9)-s.l(3,12))-xfrc353*(SWr1(7)-s.l(1,12));
  trq(3,12) = s.trq(3,12)+xtrq353-xfrc153*(SWr1(8)-s.l(2,12))+xfrc253*(SWr1(7)-s.l(1,12));

% = = Block_0_0_1_2_0_1 = = 
 
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
  OMcp53_26 = OMcp53_25+qd(6)*ROcp53_85;
  OMcp53_36 = OMcp53_35+qd(6)*ROcp53_95;
  OPcp53_16 = qdd(4)+qd(5)*qd(6)*C5+qdd(6)*S5;
  OPcp53_26 = -(qd(4)*qd(5)*S4+qd(6)*(qd(4)*ROcp53_95-OMcp53_35*S5)-qdd(5)*C4-qdd(6)*ROcp53_85);
  OPcp53_36 = qd(4)*qd(5)*C4+qd(6)*(qd(4)*ROcp53_85-OMcp53_25*S5)+qdd(5)*S4+qdd(6)*ROcp53_95;

% = = Block_0_0_1_2_0_3 = = 
 
% Sensor Kinematics 


  ROcp53_113 = ROcp53_16*C13-S13*S5;
  ROcp53_213 = ROcp53_26*C13-ROcp53_85*S13;
  ROcp53_313 = ROcp53_36*C13-ROcp53_95*S13;
  ROcp53_713 = ROcp53_16*S13+C13*S5;
  ROcp53_813 = ROcp53_26*S13+ROcp53_85*C13;
  ROcp53_913 = ROcp53_36*S13+ROcp53_95*C13;
  ROcp53_414 = ROcp53_46*C14+ROcp53_713*S14;
  ROcp53_514 = ROcp53_56*C14+ROcp53_813*S14;
  ROcp53_614 = ROcp53_66*C14+ROcp53_913*S14;
  ROcp53_714 = -(ROcp53_46*S14-ROcp53_713*C14);
  ROcp53_814 = -(ROcp53_56*S14-ROcp53_813*C14);
  ROcp53_914 = -(ROcp53_66*S14-ROcp53_913*C14);
  ROcp53_115 = ROcp53_113*C15+ROcp53_414*S15;
  ROcp53_215 = ROcp53_213*C15+ROcp53_514*S15;
  ROcp53_315 = ROcp53_313*C15+ROcp53_614*S15;
  ROcp53_415 = -(ROcp53_113*S15-ROcp53_414*C15);
  ROcp53_515 = -(ROcp53_213*S15-ROcp53_514*C15);
  ROcp53_615 = -(ROcp53_313*S15-ROcp53_614*C15);
  ROcp53_116 = ROcp53_115*C16-ROcp53_714*S16;
  ROcp53_216 = ROcp53_215*C16-ROcp53_814*S16;
  ROcp53_316 = ROcp53_315*C16-ROcp53_914*S16;
  ROcp53_716 = ROcp53_115*S16+ROcp53_714*C16;
  ROcp53_816 = ROcp53_215*S16+ROcp53_814*C16;
  ROcp53_916 = ROcp53_315*S16+ROcp53_914*C16;
  ROcp53_417 = ROcp53_415*C17+ROcp53_716*S17;
  ROcp53_517 = ROcp53_515*C17+ROcp53_816*S17;
  ROcp53_617 = ROcp53_615*C17+ROcp53_916*S17;
  ROcp53_717 = -(ROcp53_415*S17-ROcp53_716*C17);
  ROcp53_817 = -(ROcp53_515*S17-ROcp53_816*C17);
  ROcp53_917 = -(ROcp53_615*S17-ROcp53_916*C17);
  ROcp53_118 = ROcp53_116*C18-ROcp53_717*S18;
  ROcp53_218 = ROcp53_216*C18-ROcp53_817*S18;
  ROcp53_318 = ROcp53_316*C18-ROcp53_917*S18;
  ROcp53_718 = ROcp53_116*S18+ROcp53_717*C18;
  ROcp53_818 = ROcp53_216*S18+ROcp53_817*C18;
  ROcp53_918 = ROcp53_316*S18+ROcp53_917*C18;
  RLcp53_113 = ROcp53_46*s.dpt(2,4);
  RLcp53_213 = ROcp53_56*s.dpt(2,4);
  RLcp53_313 = ROcp53_66*s.dpt(2,4);
  OMcp53_113 = OMcp53_16+qd(13)*ROcp53_46;
  OMcp53_213 = OMcp53_26+qd(13)*ROcp53_56;
  OMcp53_313 = OMcp53_36+qd(13)*ROcp53_66;
  ORcp53_113 = OMcp53_26*RLcp53_313-OMcp53_36*RLcp53_213;
  ORcp53_213 = -(OMcp53_16*RLcp53_313-OMcp53_36*RLcp53_113);
  ORcp53_313 = OMcp53_16*RLcp53_213-OMcp53_26*RLcp53_113;
  OPcp53_113 = OPcp53_16+qd(13)*(OMcp53_26*ROcp53_66-OMcp53_36*ROcp53_56)+qdd(13)*ROcp53_46;
  OPcp53_213 = OPcp53_26-qd(13)*(OMcp53_16*ROcp53_66-OMcp53_36*ROcp53_46)+qdd(13)*ROcp53_56;
  OPcp53_313 = OPcp53_36+qd(13)*(OMcp53_16*ROcp53_56-OMcp53_26*ROcp53_46)+qdd(13)*ROcp53_66;
  RLcp53_114 = ROcp53_46*s.dpt(2,20);
  RLcp53_214 = ROcp53_56*s.dpt(2,20);
  RLcp53_314 = ROcp53_66*s.dpt(2,20);
  OMcp53_114 = OMcp53_113+qd(14)*ROcp53_113;
  OMcp53_214 = OMcp53_213+qd(14)*ROcp53_213;
  OMcp53_314 = OMcp53_313+qd(14)*ROcp53_313;
  ORcp53_114 = OMcp53_213*RLcp53_314-OMcp53_313*RLcp53_214;
  ORcp53_214 = -(OMcp53_113*RLcp53_314-OMcp53_313*RLcp53_114);
  ORcp53_314 = OMcp53_113*RLcp53_214-OMcp53_213*RLcp53_114;
  OPcp53_114 = OPcp53_113+qd(14)*(OMcp53_213*ROcp53_313-OMcp53_313*ROcp53_213)+qdd(14)*ROcp53_113;
  OPcp53_214 = OPcp53_213-qd(14)*(OMcp53_113*ROcp53_313-OMcp53_313*ROcp53_113)+qdd(14)*ROcp53_213;
  OPcp53_314 = OPcp53_313+qd(14)*(OMcp53_113*ROcp53_213-OMcp53_213*ROcp53_113)+qdd(14)*ROcp53_313;
  RLcp53_115 = ROcp53_714*s.dpt(3,22);
  RLcp53_215 = ROcp53_814*s.dpt(3,22);
  RLcp53_315 = ROcp53_914*s.dpt(3,22);
  OMcp53_115 = OMcp53_114+qd(15)*ROcp53_714;
  OMcp53_215 = OMcp53_214+qd(15)*ROcp53_814;
  OMcp53_315 = OMcp53_314+qd(15)*ROcp53_914;
  ORcp53_115 = OMcp53_214*RLcp53_315-OMcp53_314*RLcp53_215;
  ORcp53_215 = -(OMcp53_114*RLcp53_315-OMcp53_314*RLcp53_115);
  ORcp53_315 = OMcp53_114*RLcp53_215-OMcp53_214*RLcp53_115;
  OPcp53_115 = OPcp53_114+qd(15)*(OMcp53_214*ROcp53_914-OMcp53_314*ROcp53_814)+qdd(15)*ROcp53_714;
  OPcp53_215 = OPcp53_214-qd(15)*(OMcp53_114*ROcp53_914-OMcp53_314*ROcp53_714)+qdd(15)*ROcp53_814;
  OPcp53_315 = OPcp53_314+qd(15)*(OMcp53_114*ROcp53_814-OMcp53_214*ROcp53_714)+qdd(15)*ROcp53_914;
  RLcp53_116 = ROcp53_714*s.dpt(3,24);
  RLcp53_216 = ROcp53_814*s.dpt(3,24);
  RLcp53_316 = ROcp53_914*s.dpt(3,24);
  OMcp53_116 = OMcp53_115+qd(16)*ROcp53_415;
  OMcp53_216 = OMcp53_215+qd(16)*ROcp53_515;
  OMcp53_316 = OMcp53_315+qd(16)*ROcp53_615;
  ORcp53_116 = OMcp53_215*RLcp53_316-OMcp53_315*RLcp53_216;
  ORcp53_216 = -(OMcp53_115*RLcp53_316-OMcp53_315*RLcp53_116);
  ORcp53_316 = OMcp53_115*RLcp53_216-OMcp53_215*RLcp53_116;
  OPcp53_116 = OPcp53_115+qd(16)*(OMcp53_215*ROcp53_615-OMcp53_315*ROcp53_515)+qdd(16)*ROcp53_415;
  OPcp53_216 = OPcp53_215-qd(16)*(OMcp53_115*ROcp53_615-OMcp53_315*ROcp53_415)+qdd(16)*ROcp53_515;
  OPcp53_316 = OPcp53_315+qd(16)*(OMcp53_115*ROcp53_515-OMcp53_215*ROcp53_415)+qdd(16)*ROcp53_615;
  RLcp53_117 = ROcp53_716*s.dpt(3,26);
  RLcp53_217 = ROcp53_816*s.dpt(3,26);
  RLcp53_317 = ROcp53_916*s.dpt(3,26);
  OMcp53_117 = OMcp53_116+qd(17)*ROcp53_116;
  OMcp53_217 = OMcp53_216+qd(17)*ROcp53_216;
  OMcp53_317 = OMcp53_316+qd(17)*ROcp53_316;
  ORcp53_117 = OMcp53_216*RLcp53_317-OMcp53_316*RLcp53_217;
  ORcp53_217 = -(OMcp53_116*RLcp53_317-OMcp53_316*RLcp53_117);
  ORcp53_317 = OMcp53_116*RLcp53_217-OMcp53_216*RLcp53_117;
  OMcp53_118 = OMcp53_117+qd(18)*ROcp53_417;
  OMcp53_218 = OMcp53_217+qd(18)*ROcp53_517;
  OMcp53_318 = OMcp53_317+qd(18)*ROcp53_617;
  OPcp53_118 = OPcp53_116+qd(17)*(OMcp53_216*ROcp53_316-OMcp53_316*ROcp53_216)+qd(18)*(OMcp53_217*ROcp53_617-OMcp53_317*ROcp53_517)+qdd(17)*...
 ROcp53_116+qdd(18)*ROcp53_417;
  OPcp53_218 = OPcp53_216-qd(17)*(OMcp53_116*ROcp53_316-OMcp53_316*ROcp53_116)-qd(18)*(OMcp53_117*ROcp53_617-OMcp53_317*ROcp53_417)+qdd(17)*...
 ROcp53_216+qdd(18)*ROcp53_517;
  OPcp53_318 = OPcp53_316+qd(17)*(OMcp53_116*ROcp53_216-OMcp53_216*ROcp53_116)+qd(18)*(OMcp53_117*ROcp53_517-OMcp53_217*ROcp53_417)+qdd(17)*...
 ROcp53_316+qdd(18)*ROcp53_617;
  RLcp53_191 = ROcp53_718*s.dpt(3,31);
  RLcp53_291 = ROcp53_818*s.dpt(3,31);
  RLcp53_391 = ROcp53_918*s.dpt(3,31);
  ORcp53_191 = OMcp53_218*RLcp53_391-OMcp53_318*RLcp53_291;
  ORcp53_291 = -(OMcp53_118*RLcp53_391-OMcp53_318*RLcp53_191);
  ORcp53_391 = OMcp53_118*RLcp53_291-OMcp53_218*RLcp53_191;
  PxF2(1) = q(1)+RLcp53_113+RLcp53_114+RLcp53_115+RLcp53_116+RLcp53_117+RLcp53_191;
  PxF2(2) = q(2)+RLcp53_213+RLcp53_214+RLcp53_215+RLcp53_216+RLcp53_217+RLcp53_291;
  PxF2(3) = q(3)+RLcp53_313+RLcp53_314+RLcp53_315+RLcp53_316+RLcp53_317+RLcp53_391;
  RxF2(1,1) = ROcp53_118;
  RxF2(1,2) = ROcp53_218;
  RxF2(1,3) = ROcp53_318;
  RxF2(2,1) = ROcp53_417;
  RxF2(2,2) = ROcp53_517;
  RxF2(2,3) = ROcp53_617;
  RxF2(3,1) = ROcp53_718;
  RxF2(3,2) = ROcp53_818;
  RxF2(3,3) = ROcp53_918;
  VxF2(1) = qd(1)+ORcp53_113+ORcp53_114+ORcp53_115+ORcp53_116+ORcp53_117+ORcp53_191;
  VxF2(2) = qd(2)+ORcp53_213+ORcp53_214+ORcp53_215+ORcp53_216+ORcp53_217+ORcp53_291;
  VxF2(3) = qd(3)+ORcp53_313+ORcp53_314+ORcp53_315+ORcp53_316+ORcp53_317+ORcp53_391;
  OMxF2(1) = OMcp53_118;
  OMxF2(2) = OMcp53_218;
  OMxF2(3) = OMcp53_318;
  AxF2(1) = qdd(1)+OMcp53_213*ORcp53_314+OMcp53_214*ORcp53_315+OMcp53_215*ORcp53_316+OMcp53_216*ORcp53_317+OMcp53_218*ORcp53_391+OMcp53_26*...
 ORcp53_313-OMcp53_313*ORcp53_214-OMcp53_314*ORcp53_215-OMcp53_315*ORcp53_216-OMcp53_316*ORcp53_217-OMcp53_318*ORcp53_291-OMcp53_36*ORcp53_213+...
 OPcp53_213*RLcp53_314+OPcp53_214*RLcp53_315+OPcp53_215*RLcp53_316+OPcp53_216*RLcp53_317+OPcp53_218*RLcp53_391+OPcp53_26*RLcp53_313-OPcp53_313*...
 RLcp53_214-OPcp53_314*RLcp53_215-OPcp53_315*RLcp53_216-OPcp53_316*RLcp53_217-OPcp53_318*RLcp53_291-OPcp53_36*RLcp53_213;
  AxF2(2) = qdd(2)-OMcp53_113*ORcp53_314-OMcp53_114*ORcp53_315-OMcp53_115*ORcp53_316-OMcp53_116*ORcp53_317-OMcp53_118*ORcp53_391-OMcp53_16*...
 ORcp53_313+OMcp53_313*ORcp53_114+OMcp53_314*ORcp53_115+OMcp53_315*ORcp53_116+OMcp53_316*ORcp53_117+OMcp53_318*ORcp53_191+OMcp53_36*ORcp53_113-...
 OPcp53_113*RLcp53_314-OPcp53_114*RLcp53_315-OPcp53_115*RLcp53_316-OPcp53_116*RLcp53_317-OPcp53_118*RLcp53_391-OPcp53_16*RLcp53_313+OPcp53_313*...
 RLcp53_114+OPcp53_314*RLcp53_115+OPcp53_315*RLcp53_116+OPcp53_316*RLcp53_117+OPcp53_318*RLcp53_191+OPcp53_36*RLcp53_113;
  AxF2(3) = qdd(3)+OMcp53_113*ORcp53_214+OMcp53_114*ORcp53_215+OMcp53_115*ORcp53_216+OMcp53_116*ORcp53_217+OMcp53_118*ORcp53_291+OMcp53_16*...
 ORcp53_213-OMcp53_213*ORcp53_114-OMcp53_214*ORcp53_115-OMcp53_215*ORcp53_116-OMcp53_216*ORcp53_117-OMcp53_218*ORcp53_191-OMcp53_26*ORcp53_113+...
 OPcp53_113*RLcp53_214+OPcp53_114*RLcp53_215+OPcp53_115*RLcp53_216+OPcp53_116*RLcp53_217+OPcp53_118*RLcp53_291+OPcp53_16*RLcp53_213-OPcp53_213*...
 RLcp53_114-OPcp53_214*RLcp53_115-OPcp53_215*RLcp53_116-OPcp53_216*RLcp53_117-OPcp53_218*RLcp53_191-OPcp53_26*RLcp53_113;
  OMPxF2(1) = OPcp53_118;
  OMPxF2(2) = OPcp53_218;
  OMPxF2(3) = OPcp53_318;
 
% Sensor Forces Computation 

  SWr2 = usrfun.fext(PxF2,RxF2,VxF2,OMxF2,AxF2,OMPxF2,s,tsim,2);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc154 = ROcp53_118*SWr2(1)+ROcp53_218*SWr2(2)+ROcp53_318*SWr2(3);
  xfrc254 = ROcp53_417*SWr2(1)+ROcp53_517*SWr2(2)+ROcp53_617*SWr2(3);
  xfrc354 = ROcp53_718*SWr2(1)+ROcp53_818*SWr2(2)+ROcp53_918*SWr2(3);
  frc(1,18) = s.frc(1,18)+xfrc154;
  frc(2,18) = s.frc(2,18)+xfrc254;
  frc(3,18) = s.frc(3,18)+xfrc354;
  xtrq154 = ROcp53_118*SWr2(4)+ROcp53_218*SWr2(5)+ROcp53_318*SWr2(6);
  xtrq254 = ROcp53_417*SWr2(4)+ROcp53_517*SWr2(5)+ROcp53_617*SWr2(6);
  xtrq354 = ROcp53_718*SWr2(4)+ROcp53_818*SWr2(5)+ROcp53_918*SWr2(6);
  trq(1,18) = s.trq(1,18)+xtrq154-xfrc254*(SWr2(9)-s.l(3,18))+xfrc354*(SWr2(8)-s.l(2,18));
  trq(2,18) = s.trq(2,18)+xtrq254+xfrc154*(SWr2(9)-s.l(3,18))-xfrc354*(SWr2(7)-s.l(1,18));
  trq(3,18) = s.trq(3,18)+xtrq354-xfrc154*(SWr2(8)-s.l(2,18))+xfrc254*(SWr2(7)-s.l(1,18));

% = = Block_0_0_1_3_0_1 = = 
 
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
  OMcp54_26 = OMcp54_25+qd(6)*ROcp54_85;
  OMcp54_36 = OMcp54_35+qd(6)*ROcp54_95;
  OPcp54_16 = qdd(4)+qd(5)*qd(6)*C5+qdd(6)*S5;
  OPcp54_26 = -(qd(4)*qd(5)*S4+qd(6)*(qd(4)*ROcp54_95-OMcp54_35*S5)-qdd(5)*C4-qdd(6)*ROcp54_85);
  OPcp54_36 = qd(4)*qd(5)*C4+qd(6)*(qd(4)*ROcp54_85-OMcp54_25*S5)+qdd(5)*S4+qdd(6)*ROcp54_95;

% = = Block_0_0_1_3_0_4 = = 
 
% Sensor Kinematics 


  ROcp54_419 = ROcp54_46*C19+S19*S5;
  ROcp54_519 = ROcp54_56*C19+ROcp54_85*S19;
  ROcp54_619 = ROcp54_66*C19+ROcp54_95*S19;
  ROcp54_719 = -(ROcp54_46*S19-C19*S5);
  ROcp54_819 = -(ROcp54_56*S19-ROcp54_85*C19);
  ROcp54_919 = -(ROcp54_66*S19-ROcp54_95*C19);
  ROcp54_120 = ROcp54_16*C20-ROcp54_719*S20;
  ROcp54_220 = ROcp54_26*C20-ROcp54_819*S20;
  ROcp54_320 = ROcp54_36*C20-ROcp54_919*S20;
  ROcp54_720 = ROcp54_16*S20+ROcp54_719*C20;
  ROcp54_820 = ROcp54_26*S20+ROcp54_819*C20;
  ROcp54_920 = ROcp54_36*S20+ROcp54_919*C20;
  ROcp54_121 = ROcp54_120*C21+ROcp54_419*S21;
  ROcp54_221 = ROcp54_220*C21+ROcp54_519*S21;
  ROcp54_321 = ROcp54_320*C21+ROcp54_619*S21;
  ROcp54_421 = -(ROcp54_120*S21-ROcp54_419*C21);
  ROcp54_521 = -(ROcp54_220*S21-ROcp54_519*C21);
  ROcp54_621 = -(ROcp54_320*S21-ROcp54_619*C21);
  RLcp54_119 = ROcp54_16*s.dpt(1,5)+s.dpt(3,5)*S5;
  RLcp54_219 = ROcp54_26*s.dpt(1,5)+ROcp54_85*s.dpt(3,5);
  RLcp54_319 = ROcp54_36*s.dpt(1,5)+ROcp54_95*s.dpt(3,5);
  OMcp54_119 = OMcp54_16+qd(19)*ROcp54_16;
  OMcp54_219 = OMcp54_26+qd(19)*ROcp54_26;
  OMcp54_319 = OMcp54_36+qd(19)*ROcp54_36;
  ORcp54_119 = OMcp54_26*RLcp54_319-OMcp54_36*RLcp54_219;
  ORcp54_219 = -(OMcp54_16*RLcp54_319-OMcp54_36*RLcp54_119);
  ORcp54_319 = OMcp54_16*RLcp54_219-OMcp54_26*RLcp54_119;
  OMcp54_120 = OMcp54_119+qd(20)*ROcp54_419;
  OMcp54_220 = OMcp54_219+qd(20)*ROcp54_519;
  OMcp54_320 = OMcp54_319+qd(20)*ROcp54_619;
  OPcp54_120 = OPcp54_16+qd(19)*(OMcp54_26*ROcp54_36-OMcp54_36*ROcp54_26)+qd(20)*(OMcp54_219*ROcp54_619-OMcp54_319*ROcp54_519)+qdd(19)*ROcp54_16+...
 qdd(20)*ROcp54_419;
  OPcp54_220 = OPcp54_26-qd(19)*(OMcp54_16*ROcp54_36-OMcp54_36*ROcp54_16)-qd(20)*(OMcp54_119*ROcp54_619-OMcp54_319*ROcp54_419)+qdd(19)*ROcp54_26+...
 qdd(20)*ROcp54_519;
  OPcp54_320 = OPcp54_36+qd(19)*(OMcp54_16*ROcp54_26-OMcp54_26*ROcp54_16)+qd(20)*(OMcp54_119*ROcp54_519-OMcp54_219*ROcp54_419)+qdd(19)*ROcp54_36+...
 qdd(20)*ROcp54_619;
  RLcp54_121 = ROcp54_720*s.dpt(3,35);
  RLcp54_221 = ROcp54_820*s.dpt(3,35);
  RLcp54_321 = ROcp54_920*s.dpt(3,35);
  OMcp54_121 = OMcp54_120+qd(21)*ROcp54_720;
  OMcp54_221 = OMcp54_220+qd(21)*ROcp54_820;
  OMcp54_321 = OMcp54_320+qd(21)*ROcp54_920;
  ORcp54_121 = OMcp54_220*RLcp54_321-OMcp54_320*RLcp54_221;
  ORcp54_221 = -(OMcp54_120*RLcp54_321-OMcp54_320*RLcp54_121);
  ORcp54_321 = OMcp54_120*RLcp54_221-OMcp54_220*RLcp54_121;
  OPcp54_121 = OPcp54_120+qd(21)*(OMcp54_220*ROcp54_920-OMcp54_320*ROcp54_820)+qdd(21)*ROcp54_720;
  OPcp54_221 = OPcp54_220-qd(21)*(OMcp54_120*ROcp54_920-OMcp54_320*ROcp54_720)+qdd(21)*ROcp54_820;
  OPcp54_321 = OPcp54_320+qd(21)*(OMcp54_120*ROcp54_820-OMcp54_220*ROcp54_720)+qdd(21)*ROcp54_920;
  RLcp54_192 = ROcp54_720*s.dpt(3,37);
  RLcp54_292 = ROcp54_820*s.dpt(3,37);
  RLcp54_392 = ROcp54_920*s.dpt(3,37);
  ORcp54_192 = OMcp54_221*RLcp54_392-OMcp54_321*RLcp54_292;
  ORcp54_292 = -(OMcp54_121*RLcp54_392-OMcp54_321*RLcp54_192);
  ORcp54_392 = OMcp54_121*RLcp54_292-OMcp54_221*RLcp54_192;
  PxF3(1) = q(1)+RLcp54_119+RLcp54_121+RLcp54_192;
  PxF3(2) = q(2)+RLcp54_219+RLcp54_221+RLcp54_292;
  PxF3(3) = q(3)+RLcp54_319+RLcp54_321+RLcp54_392;
  RxF3(1,1) = ROcp54_121;
  RxF3(1,2) = ROcp54_221;
  RxF3(1,3) = ROcp54_321;
  RxF3(2,1) = ROcp54_421;
  RxF3(2,2) = ROcp54_521;
  RxF3(2,3) = ROcp54_621;
  RxF3(3,1) = ROcp54_720;
  RxF3(3,2) = ROcp54_820;
  RxF3(3,3) = ROcp54_920;
  VxF3(1) = qd(1)+ORcp54_119+ORcp54_121+ORcp54_192;
  VxF3(2) = qd(2)+ORcp54_219+ORcp54_221+ORcp54_292;
  VxF3(3) = qd(3)+ORcp54_319+ORcp54_321+ORcp54_392;
  OMxF3(1) = OMcp54_121;
  OMxF3(2) = OMcp54_221;
  OMxF3(3) = OMcp54_321;
  AxF3(1) = qdd(1)+OMcp54_220*ORcp54_321+OMcp54_221*ORcp54_392+OMcp54_26*ORcp54_319-OMcp54_320*ORcp54_221-OMcp54_321*ORcp54_292-OMcp54_36*...
 ORcp54_219+OPcp54_220*RLcp54_321+OPcp54_221*RLcp54_392+OPcp54_26*RLcp54_319-OPcp54_320*RLcp54_221-OPcp54_321*RLcp54_292-OPcp54_36*RLcp54_219;
  AxF3(2) = qdd(2)-OMcp54_120*ORcp54_321-OMcp54_121*ORcp54_392-OMcp54_16*ORcp54_319+OMcp54_320*ORcp54_121+OMcp54_321*ORcp54_192+OMcp54_36*...
 ORcp54_119-OPcp54_120*RLcp54_321-OPcp54_121*RLcp54_392-OPcp54_16*RLcp54_319+OPcp54_320*RLcp54_121+OPcp54_321*RLcp54_192+OPcp54_36*RLcp54_119;
  AxF3(3) = qdd(3)+OMcp54_120*ORcp54_221+OMcp54_121*ORcp54_292+OMcp54_16*ORcp54_219-OMcp54_220*ORcp54_121-OMcp54_221*ORcp54_192-OMcp54_26*...
 ORcp54_119+OPcp54_120*RLcp54_221+OPcp54_121*RLcp54_292+OPcp54_16*RLcp54_219-OPcp54_220*RLcp54_121-OPcp54_221*RLcp54_192-OPcp54_26*RLcp54_119;
  OMPxF3(1) = OPcp54_121;
  OMPxF3(2) = OPcp54_221;
  OMPxF3(3) = OPcp54_321;
 
% Sensor Forces Computation 

  SWr3 = usrfun.fext(PxF3,RxF3,VxF3,OMxF3,AxF3,OMPxF3,s,tsim,3);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc155 = ROcp54_121*SWr3(1)+ROcp54_221*SWr3(2)+ROcp54_321*SWr3(3);
  xfrc255 = ROcp54_421*SWr3(1)+ROcp54_521*SWr3(2)+ROcp54_621*SWr3(3);
  xfrc355 = ROcp54_720*SWr3(1)+ROcp54_820*SWr3(2)+ROcp54_920*SWr3(3);
  frc(1,21) = s.frc(1,21)+xfrc155;
  frc(2,21) = s.frc(2,21)+xfrc255;
  frc(3,21) = s.frc(3,21)+xfrc355;
  xtrq155 = ROcp54_121*SWr3(4)+ROcp54_221*SWr3(5)+ROcp54_321*SWr3(6);
  xtrq255 = ROcp54_421*SWr3(4)+ROcp54_521*SWr3(5)+ROcp54_621*SWr3(6);
  xtrq355 = ROcp54_720*SWr3(4)+ROcp54_820*SWr3(5)+ROcp54_920*SWr3(6);
  trq(1,21) = s.trq(1,21)+xtrq155-xfrc255*(SWr3(9)-s.l(3,21))+xfrc355*(SWr3(8)-s.l(2,21));
  trq(2,21) = s.trq(2,21)+xtrq255+xfrc155*(SWr3(9)-s.l(3,21))-xfrc355*(SWr3(7)-s.l(1,21));
  trq(3,21) = s.trq(3,21)+xtrq355-xfrc155*(SWr3(8)-s.l(2,21))+xfrc255*(SWr3(7)-s.l(1,21));

% = = Block_0_0_1_3_1_0 = = 
 
% Symbolic Outputs  

  frc(1,6) = s.frc(1,6);
  frc(2,6) = s.frc(2,6);
  frc(3,6) = s.frc(3,6);
  frc(1,7) = s.frc(1,7);
  frc(2,7) = s.frc(2,7);
  frc(3,7) = s.frc(3,7);
  frc(1,8) = s.frc(1,8);
  frc(2,8) = s.frc(2,8);
  frc(3,8) = s.frc(3,8);
  frc(1,9) = s.frc(1,9);
  frc(2,9) = s.frc(2,9);
  frc(3,9) = s.frc(3,9);
  frc(1,10) = s.frc(1,10);
  frc(2,10) = s.frc(2,10);
  frc(3,10) = s.frc(3,10);
  frc(1,11) = s.frc(1,11);
  frc(2,11) = s.frc(2,11);
  frc(3,11) = s.frc(3,11);
  frc(1,13) = s.frc(1,13);
  frc(2,13) = s.frc(2,13);
  frc(3,13) = s.frc(3,13);
  frc(1,14) = s.frc(1,14);
  frc(2,14) = s.frc(2,14);
  frc(3,14) = s.frc(3,14);
  frc(1,15) = s.frc(1,15);
  frc(2,15) = s.frc(2,15);
  frc(3,15) = s.frc(3,15);
  frc(1,16) = s.frc(1,16);
  frc(2,16) = s.frc(2,16);
  frc(3,16) = s.frc(3,16);
  frc(1,17) = s.frc(1,17);
  frc(2,17) = s.frc(2,17);
  frc(3,17) = s.frc(3,17);
  frc(1,19) = s.frc(1,19);
  frc(2,19) = s.frc(2,19);
  frc(3,19) = s.frc(3,19);
  frc(1,20) = s.frc(1,20);
  frc(2,20) = s.frc(2,20);
  frc(3,20) = s.frc(3,20);
  frc(1,22) = s.frc(1,22);
  frc(2,22) = s.frc(2,22);
  frc(3,22) = s.frc(3,22);
  frc(1,23) = s.frc(1,23);
  frc(2,23) = s.frc(2,23);
  frc(3,23) = s.frc(3,23);
  frc(1,24) = s.frc(1,24);
  frc(2,24) = s.frc(2,24);
  frc(3,24) = s.frc(3,24);
  frc(1,25) = s.frc(1,25);
  frc(2,25) = s.frc(2,25);
  frc(3,25) = s.frc(3,25);
  frc(1,26) = s.frc(1,26);
  frc(2,26) = s.frc(2,26);
  frc(3,26) = s.frc(3,26);
  frc(1,27) = s.frc(1,27);
  frc(2,27) = s.frc(2,27);
  frc(3,27) = s.frc(3,27);
  frc(1,28) = s.frc(1,28);
  frc(2,28) = s.frc(2,28);
  frc(3,28) = s.frc(3,28);
  frc(1,29) = s.frc(1,29);
  frc(2,29) = s.frc(2,29);
  frc(3,29) = s.frc(3,29);
  frc(1,31) = s.frc(1,31);
  frc(2,31) = s.frc(2,31);
  frc(3,31) = s.frc(3,31);
  frc(1,33) = s.frc(1,33);
  frc(2,33) = s.frc(2,33);
  frc(3,33) = s.frc(3,33);
  trq(1,6) = s.trq(1,6);
  trq(2,6) = s.trq(2,6);
  trq(3,6) = s.trq(3,6);
  trq(1,7) = s.trq(1,7);
  trq(2,7) = s.trq(2,7);
  trq(3,7) = s.trq(3,7);
  trq(1,8) = s.trq(1,8);
  trq(2,8) = s.trq(2,8);
  trq(3,8) = s.trq(3,8);
  trq(1,9) = s.trq(1,9);
  trq(2,9) = s.trq(2,9);
  trq(3,9) = s.trq(3,9);
  trq(1,10) = s.trq(1,10);
  trq(2,10) = s.trq(2,10);
  trq(3,10) = s.trq(3,10);
  trq(1,11) = s.trq(1,11);
  trq(2,11) = s.trq(2,11);
  trq(3,11) = s.trq(3,11);
  trq(1,13) = s.trq(1,13);
  trq(2,13) = s.trq(2,13);
  trq(3,13) = s.trq(3,13);
  trq(1,14) = s.trq(1,14);
  trq(2,14) = s.trq(2,14);
  trq(3,14) = s.trq(3,14);
  trq(1,15) = s.trq(1,15);
  trq(2,15) = s.trq(2,15);
  trq(3,15) = s.trq(3,15);
  trq(1,16) = s.trq(1,16);
  trq(2,16) = s.trq(2,16);
  trq(3,16) = s.trq(3,16);
  trq(1,17) = s.trq(1,17);
  trq(2,17) = s.trq(2,17);
  trq(3,17) = s.trq(3,17);
  trq(1,19) = s.trq(1,19);
  trq(2,19) = s.trq(2,19);
  trq(3,19) = s.trq(3,19);
  trq(1,20) = s.trq(1,20);
  trq(2,20) = s.trq(2,20);
  trq(3,20) = s.trq(3,20);
  trq(1,22) = s.trq(1,22);
  trq(2,22) = s.trq(2,22);
  trq(3,22) = s.trq(3,22);
  trq(1,23) = s.trq(1,23);
  trq(2,23) = s.trq(2,23);
  trq(3,23) = s.trq(3,23);
  trq(1,24) = s.trq(1,24);
  trq(2,24) = s.trq(2,24);
  trq(3,24) = s.trq(3,24);
  trq(1,25) = s.trq(1,25);
  trq(2,25) = s.trq(2,25);
  trq(3,25) = s.trq(3,25);
  trq(1,26) = s.trq(1,26);
  trq(2,26) = s.trq(2,26);
  trq(3,26) = s.trq(3,26);
  trq(1,27) = s.trq(1,27);
  trq(2,27) = s.trq(2,27);
  trq(3,27) = s.trq(3,27);
  trq(1,28) = s.trq(1,28);
  trq(2,28) = s.trq(2,28);
  trq(3,28) = s.trq(3,28);
  trq(1,29) = s.trq(1,29);
  trq(2,29) = s.trq(2,29);
  trq(3,29) = s.trq(3,29);
  trq(1,31) = s.trq(1,31);
  trq(2,31) = s.trq(2,31);
  trq(3,31) = s.trq(3,31);
  trq(1,33) = s.trq(1,33);
  trq(2,33) = s.trq(2,33);
  trq(3,33) = s.trq(3,33);

% ====== END Task 0 ====== 

  

