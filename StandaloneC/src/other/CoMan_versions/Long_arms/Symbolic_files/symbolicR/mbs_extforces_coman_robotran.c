//
//-------------------------------------------------------------
//
//	ROBOTRAN - Version 6.6 (build : february 22, 2008)
//
//	Copyright 
//	Universite catholique de Louvain 
//	Departement de Mecanique 
//	Unite de Production Mecanique et Machines 
//	2, Place du Levant 
//	1348 Louvain-la-Neuve 
//	http://www.robotran.be// 
//
//	==> Generation Date : Mon Jul 14 11:56:58 2014
//
//	==> Project name : coman_robotran
//	==> using XML input file 
//
//	==> Number of joints : 35
//
//	==> Function : F19 : External Forces
//	==> Flops complexity : 3994
//
//	==> Generation Time :  0.080 seconds
//	==> Post-Processing :  0.080 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "MBSdataStructR7.h"
#include "MBSfunR7.h"
 
void extforces(double **frc,double **trq,
MBSdataStruct *s, double tsim)

// double frc[3][35];
// double trq[3][35];
{ 
double PxF1[4]; 
double RxF1[4][4]; 
double VxF1[4]; 
double OMxF1[4]; 
double AxF1[4]; 
double OMPxF1[4]; 
double *SWr1; 
double PxF2[4]; 
double RxF2[4][4]; 
double VxF2[4]; 
double OMxF2[4]; 
double AxF2[4]; 
double OMPxF2[4]; 
double *SWr2; 
double PxF3[4]; 
double RxF3[4][4]; 
double VxF3[4]; 
double OMxF3[4]; 
double AxF3[4]; 
double OMPxF3[4]; 
double *SWr3; 
double PxF4[4]; 
double RxF4[4][4]; 
double VxF4[4]; 
double OMxF4[4]; 
double AxF4[4]; 
double OMPxF4[4]; 
double *SWr4; 
double PxF5[4]; 
double RxF5[4][4]; 
double VxF5[4]; 
double OMxF5[4]; 
double AxF5[4]; 
double OMPxF5[4]; 
double *SWr5; 
double PxF6[4]; 
double RxF6[4][4]; 
double VxF6[4]; 
double OMxF6[4]; 
double AxF6[4]; 
double OMPxF6[4]; 
double *SWr6; 
 
#include "mbs_extforces_coman_robotran.h" 
#define q s->q 
#define qd s->qd 
#define qdd s->qdd 
 
 

// === begin imp_aux === 

// === end imp_aux === 

// ===== BEGIN task 0 ===== 
 
// Sensor Kinematics 



// = = Block_0_0_0_0_0_1 = = 
 
// Trigonometric Variables  

  C4 = cos(q[4]);
  S4 = sin(q[4]);
  C5 = cos(q[5]);
  S5 = sin(q[5]);
  C6 = cos(q[6]);
  S6 = sin(q[6]);

// = = Block_0_0_0_0_0_2 = = 
 
// Trigonometric Variables  

  C7 = cos(q[7]);
  S7 = sin(q[7]);
  C8 = cos(q[8]);
  S8 = sin(q[8]);
  C9 = cos(q[9]);
  S9 = sin(q[9]);
  C10 = cos(q[10]);
  S10 = sin(q[10]);
  C11 = cos(q[11]);
  S11 = sin(q[11]);
  C12 = cos(q[12]);
  S12 = sin(q[12]);

// = = Block_0_0_0_0_0_3 = = 
 
// Trigonometric Variables  

  C13 = cos(q[13]);
  S13 = sin(q[13]);
  C14 = cos(q[14]);
  S14 = sin(q[14]);
  C15 = cos(q[15]);
  S15 = sin(q[15]);
  C16 = cos(q[16]);
  S16 = sin(q[16]);
  C17 = cos(q[17]);
  S17 = sin(q[17]);
  C18 = cos(q[18]);
  S18 = sin(q[18]);

// = = Block_0_0_0_0_0_4 = = 
 
// Trigonometric Variables  

  C19 = cos(q[19]);
  S19 = sin(q[19]);
  C20 = cos(q[20]);
  S20 = sin(q[20]);
  C21 = cos(q[21]);
  S21 = sin(q[21]);

// = = Block_0_0_0_0_0_5 = = 
 
// Trigonometric Variables  

  C22 = cos(q[22]);
  S22 = sin(q[22]);
  C23 = cos(q[23]);
  S23 = sin(q[23]);
  C24 = cos(q[24]);
  S24 = sin(q[24]);
  C25 = cos(q[25]);
  S25 = sin(q[25]);
  C26 = cos(q[26]);
  S26 = sin(q[26]);
  C27 = cos(q[27]);
  S27 = sin(q[27]);
  C28 = cos(q[28]);
  S28 = sin(q[28]);

// = = Block_0_0_0_0_0_6 = = 
 
// Trigonometric Variables  

  C29 = cos(q[29]);
  S29 = sin(q[29]);
  C30 = cos(q[30]);
  S30 = sin(q[30]);
  C31 = cos(q[31]);
  S31 = sin(q[31]);
  C32 = cos(q[32]);
  S32 = sin(q[32]);
  C33 = cos(q[33]);
  S33 = sin(q[33]);
  C34 = cos(q[34]);
  S34 = sin(q[34]);
  C35 = cos(q[35]);
  S35 = sin(q[35]);

// = = Block_0_0_1_1_0_1 = = 
 
// Sensor Kinematics 


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
  OMcp51_25 = qd[5]*C4;
  OMcp51_35 = qd[5]*S4;
  OMcp51_16 = qd[4]+qd[6]*S5;
  OMcp51_26 = OMcp51_25+qd[6]*ROcp51_85;
  OMcp51_36 = OMcp51_35+qd[6]*ROcp51_95;
  OPcp51_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp51_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp51_95-OMcp51_35*S5)-qdd[5]*C4-qdd[6]*ROcp51_85);
  OPcp51_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp51_85-OMcp51_25*S5)+qdd[5]*S4+qdd[6]*ROcp51_95;

// = = Block_0_0_1_1_0_2 = = 
 
// Sensor Kinematics 


  ROcp51_17 = ROcp51_16*C7-S5*S7;
  ROcp51_27 = ROcp51_26*C7-ROcp51_85*S7;
  ROcp51_37 = ROcp51_36*C7-ROcp51_95*S7;
  ROcp51_77 = ROcp51_16*S7+S5*C7;
  ROcp51_87 = ROcp51_26*S7+ROcp51_85*C7;
  ROcp51_97 = ROcp51_36*S7+ROcp51_95*C7;
  ROcp51_48 = ROcp51_46*C8+ROcp51_77*S8;
  ROcp51_58 = ROcp51_56*C8+ROcp51_87*S8;
  ROcp51_68 = ROcp51_66*C8+ROcp51_97*S8;
  ROcp51_78 = -(ROcp51_46*S8-ROcp51_77*C8);
  ROcp51_88 = -(ROcp51_56*S8-ROcp51_87*C8);
  ROcp51_98 = -(ROcp51_66*S8-ROcp51_97*C8);
  ROcp51_19 = ROcp51_17*C9+ROcp51_48*S9;
  ROcp51_29 = ROcp51_27*C9+ROcp51_58*S9;
  ROcp51_39 = ROcp51_37*C9+ROcp51_68*S9;
  ROcp51_49 = -(ROcp51_17*S9-ROcp51_48*C9);
  ROcp51_59 = -(ROcp51_27*S9-ROcp51_58*C9);
  ROcp51_69 = -(ROcp51_37*S9-ROcp51_68*C9);
  ROcp51_110 = ROcp51_19*C10-ROcp51_78*S10;
  ROcp51_210 = ROcp51_29*C10-ROcp51_88*S10;
  ROcp51_310 = ROcp51_39*C10-ROcp51_98*S10;
  ROcp51_710 = ROcp51_19*S10+ROcp51_78*C10;
  ROcp51_810 = ROcp51_29*S10+ROcp51_88*C10;
  ROcp51_910 = ROcp51_39*S10+ROcp51_98*C10;
  ROcp51_411 = ROcp51_49*C11+ROcp51_710*S11;
  ROcp51_511 = ROcp51_59*C11+ROcp51_810*S11;
  ROcp51_611 = ROcp51_69*C11+ROcp51_910*S11;
  ROcp51_711 = -(ROcp51_49*S11-ROcp51_710*C11);
  ROcp51_811 = -(ROcp51_59*S11-ROcp51_810*C11);
  ROcp51_911 = -(ROcp51_69*S11-ROcp51_910*C11);
  ROcp51_112 = ROcp51_110*C12-ROcp51_711*S12;
  ROcp51_212 = ROcp51_210*C12-ROcp51_811*S12;
  ROcp51_312 = ROcp51_310*C12-ROcp51_911*S12;
  ROcp51_712 = ROcp51_110*S12+ROcp51_711*C12;
  ROcp51_812 = ROcp51_210*S12+ROcp51_811*C12;
  ROcp51_912 = ROcp51_310*S12+ROcp51_911*C12;
  RLcp51_17 = ROcp51_46*s->dpt[2][1];
  RLcp51_27 = ROcp51_56*s->dpt[2][1];
  RLcp51_37 = ROcp51_66*s->dpt[2][1];
  OMcp51_17 = OMcp51_16+qd[7]*ROcp51_46;
  OMcp51_27 = OMcp51_26+qd[7]*ROcp51_56;
  OMcp51_37 = OMcp51_36+qd[7]*ROcp51_66;
  ORcp51_17 = OMcp51_26*RLcp51_37-OMcp51_36*RLcp51_27;
  ORcp51_27 = -(OMcp51_16*RLcp51_37-OMcp51_36*RLcp51_17);
  ORcp51_37 = OMcp51_16*RLcp51_27-OMcp51_26*RLcp51_17;
  OPcp51_17 = OPcp51_16+qd[7]*(OMcp51_26*ROcp51_66-OMcp51_36*ROcp51_56)+qdd[7]*ROcp51_46;
  OPcp51_27 = OPcp51_26-qd[7]*(OMcp51_16*ROcp51_66-OMcp51_36*ROcp51_46)+qdd[7]*ROcp51_56;
  OPcp51_37 = OPcp51_36+qd[7]*(OMcp51_16*ROcp51_56-OMcp51_26*ROcp51_46)+qdd[7]*ROcp51_66;
  RLcp51_18 = ROcp51_46*s->dpt[2][6];
  RLcp51_28 = ROcp51_56*s->dpt[2][6];
  RLcp51_38 = ROcp51_66*s->dpt[2][6];
  OMcp51_18 = OMcp51_17+qd[8]*ROcp51_17;
  OMcp51_28 = OMcp51_27+qd[8]*ROcp51_27;
  OMcp51_38 = OMcp51_37+qd[8]*ROcp51_37;
  ORcp51_18 = OMcp51_27*RLcp51_38-OMcp51_37*RLcp51_28;
  ORcp51_28 = -(OMcp51_17*RLcp51_38-OMcp51_37*RLcp51_18);
  ORcp51_38 = OMcp51_17*RLcp51_28-OMcp51_27*RLcp51_18;
  OPcp51_18 = OPcp51_17+qd[8]*(OMcp51_27*ROcp51_37-OMcp51_37*ROcp51_27)+qdd[8]*ROcp51_17;
  OPcp51_28 = OPcp51_27-qd[8]*(OMcp51_17*ROcp51_37-OMcp51_37*ROcp51_17)+qdd[8]*ROcp51_27;
  OPcp51_38 = OPcp51_37+qd[8]*(OMcp51_17*ROcp51_27-OMcp51_27*ROcp51_17)+qdd[8]*ROcp51_37;
  RLcp51_19 = ROcp51_78*s->dpt[3][8];
  RLcp51_29 = ROcp51_88*s->dpt[3][8];
  RLcp51_39 = ROcp51_98*s->dpt[3][8];
  OMcp51_19 = OMcp51_18+qd[9]*ROcp51_78;
  OMcp51_29 = OMcp51_28+qd[9]*ROcp51_88;
  OMcp51_39 = OMcp51_38+qd[9]*ROcp51_98;
  ORcp51_19 = OMcp51_28*RLcp51_39-OMcp51_38*RLcp51_29;
  ORcp51_29 = -(OMcp51_18*RLcp51_39-OMcp51_38*RLcp51_19);
  ORcp51_39 = OMcp51_18*RLcp51_29-OMcp51_28*RLcp51_19;
  OPcp51_19 = OPcp51_18+qd[9]*(OMcp51_28*ROcp51_98-OMcp51_38*ROcp51_88)+qdd[9]*ROcp51_78;
  OPcp51_29 = OPcp51_28-qd[9]*(OMcp51_18*ROcp51_98-OMcp51_38*ROcp51_78)+qdd[9]*ROcp51_88;
  OPcp51_39 = OPcp51_38+qd[9]*(OMcp51_18*ROcp51_88-OMcp51_28*ROcp51_78)+qdd[9]*ROcp51_98;
  RLcp51_110 = ROcp51_78*s->dpt[3][10];
  RLcp51_210 = ROcp51_88*s->dpt[3][10];
  RLcp51_310 = ROcp51_98*s->dpt[3][10];
  OMcp51_110 = OMcp51_19+qd[10]*ROcp51_49;
  OMcp51_210 = OMcp51_29+qd[10]*ROcp51_59;
  OMcp51_310 = OMcp51_39+qd[10]*ROcp51_69;
  ORcp51_110 = OMcp51_29*RLcp51_310-OMcp51_39*RLcp51_210;
  ORcp51_210 = -(OMcp51_19*RLcp51_310-OMcp51_39*RLcp51_110);
  ORcp51_310 = OMcp51_19*RLcp51_210-OMcp51_29*RLcp51_110;
  OPcp51_110 = OPcp51_19+qd[10]*(OMcp51_29*ROcp51_69-OMcp51_39*ROcp51_59)+qdd[10]*ROcp51_49;
  OPcp51_210 = OPcp51_29-qd[10]*(OMcp51_19*ROcp51_69-OMcp51_39*ROcp51_49)+qdd[10]*ROcp51_59;
  OPcp51_310 = OPcp51_39+qd[10]*(OMcp51_19*ROcp51_59-OMcp51_29*ROcp51_49)+qdd[10]*ROcp51_69;
  RLcp51_111 = ROcp51_710*s->dpt[3][12];
  RLcp51_211 = ROcp51_810*s->dpt[3][12];
  RLcp51_311 = ROcp51_910*s->dpt[3][12];
  OMcp51_111 = OMcp51_110+qd[11]*ROcp51_110;
  OMcp51_211 = OMcp51_210+qd[11]*ROcp51_210;
  OMcp51_311 = OMcp51_310+qd[11]*ROcp51_310;
  ORcp51_111 = OMcp51_210*RLcp51_311-OMcp51_310*RLcp51_211;
  ORcp51_211 = -(OMcp51_110*RLcp51_311-OMcp51_310*RLcp51_111);
  ORcp51_311 = OMcp51_110*RLcp51_211-OMcp51_210*RLcp51_111;
  OMcp51_112 = OMcp51_111+qd[12]*ROcp51_411;
  OMcp51_212 = OMcp51_211+qd[12]*ROcp51_511;
  OMcp51_312 = OMcp51_311+qd[12]*ROcp51_611;
  OPcp51_112 = OPcp51_110+qd[11]*(OMcp51_210*ROcp51_310-OMcp51_310*ROcp51_210)+qd[12]*(OMcp51_211*ROcp51_611-OMcp51_311*
 ROcp51_511)+qdd[11]*ROcp51_110+qdd[12]*ROcp51_411;
  OPcp51_212 = OPcp51_210-qd[11]*(OMcp51_110*ROcp51_310-OMcp51_310*ROcp51_110)-qd[12]*(OMcp51_111*ROcp51_611-OMcp51_311*
 ROcp51_411)+qdd[11]*ROcp51_210+qdd[12]*ROcp51_511;
  OPcp51_312 = OPcp51_310+qd[11]*(OMcp51_110*ROcp51_210-OMcp51_210*ROcp51_110)+qd[12]*(OMcp51_111*ROcp51_511-OMcp51_211*
 ROcp51_411)+qdd[11]*ROcp51_310+qdd[12]*ROcp51_611;
  RLcp51_187 = ROcp51_712*s->dpt[3][16];
  RLcp51_287 = ROcp51_812*s->dpt[3][16];
  RLcp51_387 = ROcp51_912*s->dpt[3][16];
  ORcp51_187 = OMcp51_212*RLcp51_387-OMcp51_312*RLcp51_287;
  ORcp51_287 = -(OMcp51_112*RLcp51_387-OMcp51_312*RLcp51_187);
  ORcp51_387 = OMcp51_112*RLcp51_287-OMcp51_212*RLcp51_187;
  PxF1[1] = q[1]+RLcp51_110+RLcp51_111+RLcp51_17+RLcp51_18+RLcp51_187+RLcp51_19;
  PxF1[2] = q[2]+RLcp51_210+RLcp51_211+RLcp51_27+RLcp51_28+RLcp51_287+RLcp51_29;
  PxF1[3] = q[3]+RLcp51_310+RLcp51_311+RLcp51_37+RLcp51_38+RLcp51_387+RLcp51_39;
  RxF1[1][1] = ROcp51_112;
  RxF1[1][2] = ROcp51_212;
  RxF1[1][3] = ROcp51_312;
  RxF1[2][1] = ROcp51_411;
  RxF1[2][2] = ROcp51_511;
  RxF1[2][3] = ROcp51_611;
  RxF1[3][1] = ROcp51_712;
  RxF1[3][2] = ROcp51_812;
  RxF1[3][3] = ROcp51_912;
  VxF1[1] = qd[1]+ORcp51_110+ORcp51_111+ORcp51_17+ORcp51_18+ORcp51_187+ORcp51_19;
  VxF1[2] = qd[2]+ORcp51_210+ORcp51_211+ORcp51_27+ORcp51_28+ORcp51_287+ORcp51_29;
  VxF1[3] = qd[3]+ORcp51_310+ORcp51_311+ORcp51_37+ORcp51_38+ORcp51_387+ORcp51_39;
  OMxF1[1] = OMcp51_112;
  OMxF1[2] = OMcp51_212;
  OMxF1[3] = OMcp51_312;
  AxF1[1] = qdd[1]+OMcp51_210*ORcp51_311+OMcp51_212*ORcp51_387+OMcp51_26*ORcp51_37+OMcp51_27*ORcp51_38+OMcp51_28*
 ORcp51_39+OMcp51_29*ORcp51_310-OMcp51_310*ORcp51_211-OMcp51_312*ORcp51_287-OMcp51_36*ORcp51_27-OMcp51_37*ORcp51_28-OMcp51_38
 *ORcp51_29-OMcp51_39*ORcp51_210+OPcp51_210*RLcp51_311+OPcp51_212*RLcp51_387+OPcp51_26*RLcp51_37+OPcp51_27*RLcp51_38+
 OPcp51_28*RLcp51_39+OPcp51_29*RLcp51_310-OPcp51_310*RLcp51_211-OPcp51_312*RLcp51_287-OPcp51_36*RLcp51_27-OPcp51_37*RLcp51_28
 -OPcp51_38*RLcp51_29-OPcp51_39*RLcp51_210;
  AxF1[2] = qdd[2]-OMcp51_110*ORcp51_311-OMcp51_112*ORcp51_387-OMcp51_16*ORcp51_37-OMcp51_17*ORcp51_38-OMcp51_18*
 ORcp51_39-OMcp51_19*ORcp51_310+OMcp51_310*ORcp51_111+OMcp51_312*ORcp51_187+OMcp51_36*ORcp51_17+OMcp51_37*ORcp51_18+OMcp51_38
 *ORcp51_19+OMcp51_39*ORcp51_110-OPcp51_110*RLcp51_311-OPcp51_112*RLcp51_387-OPcp51_16*RLcp51_37-OPcp51_17*RLcp51_38-
 OPcp51_18*RLcp51_39-OPcp51_19*RLcp51_310+OPcp51_310*RLcp51_111+OPcp51_312*RLcp51_187+OPcp51_36*RLcp51_17+OPcp51_37*RLcp51_18
 +OPcp51_38*RLcp51_19+OPcp51_39*RLcp51_110;
  AxF1[3] = qdd[3]+OMcp51_110*ORcp51_211+OMcp51_112*ORcp51_287+OMcp51_16*ORcp51_27+OMcp51_17*ORcp51_28+OMcp51_18*
 ORcp51_29+OMcp51_19*ORcp51_210-OMcp51_210*ORcp51_111-OMcp51_212*ORcp51_187-OMcp51_26*ORcp51_17-OMcp51_27*ORcp51_18-OMcp51_28
 *ORcp51_19-OMcp51_29*ORcp51_110+OPcp51_110*RLcp51_211+OPcp51_112*RLcp51_287+OPcp51_16*RLcp51_27+OPcp51_17*RLcp51_28+
 OPcp51_18*RLcp51_29+OPcp51_19*RLcp51_210-OPcp51_210*RLcp51_111-OPcp51_212*RLcp51_187-OPcp51_26*RLcp51_17-OPcp51_27*RLcp51_18
 -OPcp51_28*RLcp51_19-OPcp51_29*RLcp51_110;
  OMPxF1[1] = OPcp51_112;
  OMPxF1[2] = OPcp51_212;
  OMPxF1[3] = OPcp51_312;
 
// Sensor Forces Computation 

  SWr1 = user_ExtForces(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc152 = ROcp51_112*SWr1[1]+ROcp51_212*SWr1[2]+ROcp51_312*SWr1[3];
  xfrc252 = ROcp51_411*SWr1[1]+ROcp51_511*SWr1[2]+ROcp51_611*SWr1[3];
  xfrc352 = ROcp51_712*SWr1[1]+ROcp51_812*SWr1[2]+ROcp51_912*SWr1[3];
  frc[1][12] = s->frc[1][12]+xfrc152;
  frc[2][12] = s->frc[2][12]+xfrc252;
  frc[3][12] = s->frc[3][12]+xfrc352;
  xtrq152 = ROcp51_112*SWr1[4]+ROcp51_212*SWr1[5]+ROcp51_312*SWr1[6];
  xtrq252 = ROcp51_411*SWr1[4]+ROcp51_511*SWr1[5]+ROcp51_611*SWr1[6];
  xtrq352 = ROcp51_712*SWr1[4]+ROcp51_812*SWr1[5]+ROcp51_912*SWr1[6];
  trq[1][12] = s->trq[1][12]+xtrq152-xfrc252*(SWr1[9]-s->l[3][12])+xfrc352*(SWr1[8]-s->l[2][12]);
  trq[2][12] = s->trq[2][12]+xtrq252+xfrc152*(SWr1[9]-s->l[3][12])-xfrc352*(SWr1[7]-s->l[1][12]);
  trq[3][12] = s->trq[3][12]+xtrq352-xfrc152*(SWr1[8]-s->l[2][12])+xfrc252*(SWr1[7]-s->l[1][12]);

// = = Block_0_0_1_2_0_1 = = 
 
// Sensor Kinematics 


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
  OMcp52_25 = qd[5]*C4;
  OMcp52_35 = qd[5]*S4;
  OMcp52_16 = qd[4]+qd[6]*S5;
  OMcp52_26 = OMcp52_25+qd[6]*ROcp52_85;
  OMcp52_36 = OMcp52_35+qd[6]*ROcp52_95;
  OPcp52_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp52_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp52_95-OMcp52_35*S5)-qdd[5]*C4-qdd[6]*ROcp52_85);
  OPcp52_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp52_85-OMcp52_25*S5)+qdd[5]*S4+qdd[6]*ROcp52_95;

// = = Block_0_0_1_2_0_3 = = 
 
// Sensor Kinematics 


  ROcp52_113 = ROcp52_16*C13-S13*S5;
  ROcp52_213 = ROcp52_26*C13-ROcp52_85*S13;
  ROcp52_313 = ROcp52_36*C13-ROcp52_95*S13;
  ROcp52_713 = ROcp52_16*S13+C13*S5;
  ROcp52_813 = ROcp52_26*S13+ROcp52_85*C13;
  ROcp52_913 = ROcp52_36*S13+ROcp52_95*C13;
  ROcp52_414 = ROcp52_46*C14+ROcp52_713*S14;
  ROcp52_514 = ROcp52_56*C14+ROcp52_813*S14;
  ROcp52_614 = ROcp52_66*C14+ROcp52_913*S14;
  ROcp52_714 = -(ROcp52_46*S14-ROcp52_713*C14);
  ROcp52_814 = -(ROcp52_56*S14-ROcp52_813*C14);
  ROcp52_914 = -(ROcp52_66*S14-ROcp52_913*C14);
  ROcp52_115 = ROcp52_113*C15+ROcp52_414*S15;
  ROcp52_215 = ROcp52_213*C15+ROcp52_514*S15;
  ROcp52_315 = ROcp52_313*C15+ROcp52_614*S15;
  ROcp52_415 = -(ROcp52_113*S15-ROcp52_414*C15);
  ROcp52_515 = -(ROcp52_213*S15-ROcp52_514*C15);
  ROcp52_615 = -(ROcp52_313*S15-ROcp52_614*C15);
  ROcp52_116 = ROcp52_115*C16-ROcp52_714*S16;
  ROcp52_216 = ROcp52_215*C16-ROcp52_814*S16;
  ROcp52_316 = ROcp52_315*C16-ROcp52_914*S16;
  ROcp52_716 = ROcp52_115*S16+ROcp52_714*C16;
  ROcp52_816 = ROcp52_215*S16+ROcp52_814*C16;
  ROcp52_916 = ROcp52_315*S16+ROcp52_914*C16;
  ROcp52_417 = ROcp52_415*C17+ROcp52_716*S17;
  ROcp52_517 = ROcp52_515*C17+ROcp52_816*S17;
  ROcp52_617 = ROcp52_615*C17+ROcp52_916*S17;
  ROcp52_717 = -(ROcp52_415*S17-ROcp52_716*C17);
  ROcp52_817 = -(ROcp52_515*S17-ROcp52_816*C17);
  ROcp52_917 = -(ROcp52_615*S17-ROcp52_916*C17);
  ROcp52_118 = ROcp52_116*C18-ROcp52_717*S18;
  ROcp52_218 = ROcp52_216*C18-ROcp52_817*S18;
  ROcp52_318 = ROcp52_316*C18-ROcp52_917*S18;
  ROcp52_718 = ROcp52_116*S18+ROcp52_717*C18;
  ROcp52_818 = ROcp52_216*S18+ROcp52_817*C18;
  ROcp52_918 = ROcp52_316*S18+ROcp52_917*C18;
  RLcp52_113 = ROcp52_46*s->dpt[2][2];
  RLcp52_213 = ROcp52_56*s->dpt[2][2];
  RLcp52_313 = ROcp52_66*s->dpt[2][2];
  OMcp52_113 = OMcp52_16+qd[13]*ROcp52_46;
  OMcp52_213 = OMcp52_26+qd[13]*ROcp52_56;
  OMcp52_313 = OMcp52_36+qd[13]*ROcp52_66;
  ORcp52_113 = OMcp52_26*RLcp52_313-OMcp52_36*RLcp52_213;
  ORcp52_213 = -(OMcp52_16*RLcp52_313-OMcp52_36*RLcp52_113);
  ORcp52_313 = OMcp52_16*RLcp52_213-OMcp52_26*RLcp52_113;
  OPcp52_113 = OPcp52_16+qd[13]*(OMcp52_26*ROcp52_66-OMcp52_36*ROcp52_56)+qdd[13]*ROcp52_46;
  OPcp52_213 = OPcp52_26-qd[13]*(OMcp52_16*ROcp52_66-OMcp52_36*ROcp52_46)+qdd[13]*ROcp52_56;
  OPcp52_313 = OPcp52_36+qd[13]*(OMcp52_16*ROcp52_56-OMcp52_26*ROcp52_46)+qdd[13]*ROcp52_66;
  RLcp52_114 = ROcp52_46*s->dpt[2][18];
  RLcp52_214 = ROcp52_56*s->dpt[2][18];
  RLcp52_314 = ROcp52_66*s->dpt[2][18];
  OMcp52_114 = OMcp52_113+qd[14]*ROcp52_113;
  OMcp52_214 = OMcp52_213+qd[14]*ROcp52_213;
  OMcp52_314 = OMcp52_313+qd[14]*ROcp52_313;
  ORcp52_114 = OMcp52_213*RLcp52_314-OMcp52_313*RLcp52_214;
  ORcp52_214 = -(OMcp52_113*RLcp52_314-OMcp52_313*RLcp52_114);
  ORcp52_314 = OMcp52_113*RLcp52_214-OMcp52_213*RLcp52_114;
  OPcp52_114 = OPcp52_113+qd[14]*(OMcp52_213*ROcp52_313-OMcp52_313*ROcp52_213)+qdd[14]*ROcp52_113;
  OPcp52_214 = OPcp52_213-qd[14]*(OMcp52_113*ROcp52_313-OMcp52_313*ROcp52_113)+qdd[14]*ROcp52_213;
  OPcp52_314 = OPcp52_313+qd[14]*(OMcp52_113*ROcp52_213-OMcp52_213*ROcp52_113)+qdd[14]*ROcp52_313;
  RLcp52_115 = ROcp52_714*s->dpt[3][20];
  RLcp52_215 = ROcp52_814*s->dpt[3][20];
  RLcp52_315 = ROcp52_914*s->dpt[3][20];
  OMcp52_115 = OMcp52_114+qd[15]*ROcp52_714;
  OMcp52_215 = OMcp52_214+qd[15]*ROcp52_814;
  OMcp52_315 = OMcp52_314+qd[15]*ROcp52_914;
  ORcp52_115 = OMcp52_214*RLcp52_315-OMcp52_314*RLcp52_215;
  ORcp52_215 = -(OMcp52_114*RLcp52_315-OMcp52_314*RLcp52_115);
  ORcp52_315 = OMcp52_114*RLcp52_215-OMcp52_214*RLcp52_115;
  OPcp52_115 = OPcp52_114+qd[15]*(OMcp52_214*ROcp52_914-OMcp52_314*ROcp52_814)+qdd[15]*ROcp52_714;
  OPcp52_215 = OPcp52_214-qd[15]*(OMcp52_114*ROcp52_914-OMcp52_314*ROcp52_714)+qdd[15]*ROcp52_814;
  OPcp52_315 = OPcp52_314+qd[15]*(OMcp52_114*ROcp52_814-OMcp52_214*ROcp52_714)+qdd[15]*ROcp52_914;
  RLcp52_116 = ROcp52_714*s->dpt[3][22];
  RLcp52_216 = ROcp52_814*s->dpt[3][22];
  RLcp52_316 = ROcp52_914*s->dpt[3][22];
  OMcp52_116 = OMcp52_115+qd[16]*ROcp52_415;
  OMcp52_216 = OMcp52_215+qd[16]*ROcp52_515;
  OMcp52_316 = OMcp52_315+qd[16]*ROcp52_615;
  ORcp52_116 = OMcp52_215*RLcp52_316-OMcp52_315*RLcp52_216;
  ORcp52_216 = -(OMcp52_115*RLcp52_316-OMcp52_315*RLcp52_116);
  ORcp52_316 = OMcp52_115*RLcp52_216-OMcp52_215*RLcp52_116;
  OPcp52_116 = OPcp52_115+qd[16]*(OMcp52_215*ROcp52_615-OMcp52_315*ROcp52_515)+qdd[16]*ROcp52_415;
  OPcp52_216 = OPcp52_215-qd[16]*(OMcp52_115*ROcp52_615-OMcp52_315*ROcp52_415)+qdd[16]*ROcp52_515;
  OPcp52_316 = OPcp52_315+qd[16]*(OMcp52_115*ROcp52_515-OMcp52_215*ROcp52_415)+qdd[16]*ROcp52_615;
  RLcp52_117 = ROcp52_716*s->dpt[3][24];
  RLcp52_217 = ROcp52_816*s->dpt[3][24];
  RLcp52_317 = ROcp52_916*s->dpt[3][24];
  OMcp52_117 = OMcp52_116+qd[17]*ROcp52_116;
  OMcp52_217 = OMcp52_216+qd[17]*ROcp52_216;
  OMcp52_317 = OMcp52_316+qd[17]*ROcp52_316;
  ORcp52_117 = OMcp52_216*RLcp52_317-OMcp52_316*RLcp52_217;
  ORcp52_217 = -(OMcp52_116*RLcp52_317-OMcp52_316*RLcp52_117);
  ORcp52_317 = OMcp52_116*RLcp52_217-OMcp52_216*RLcp52_117;
  OMcp52_118 = OMcp52_117+qd[18]*ROcp52_417;
  OMcp52_218 = OMcp52_217+qd[18]*ROcp52_517;
  OMcp52_318 = OMcp52_317+qd[18]*ROcp52_617;
  OPcp52_118 = OPcp52_116+qd[17]*(OMcp52_216*ROcp52_316-OMcp52_316*ROcp52_216)+qd[18]*(OMcp52_217*ROcp52_617-OMcp52_317*
 ROcp52_517)+qdd[17]*ROcp52_116+qdd[18]*ROcp52_417;
  OPcp52_218 = OPcp52_216-qd[17]*(OMcp52_116*ROcp52_316-OMcp52_316*ROcp52_116)-qd[18]*(OMcp52_117*ROcp52_617-OMcp52_317*
 ROcp52_417)+qdd[17]*ROcp52_216+qdd[18]*ROcp52_517;
  OPcp52_318 = OPcp52_316+qd[17]*(OMcp52_116*ROcp52_216-OMcp52_216*ROcp52_116)+qd[18]*(OMcp52_117*ROcp52_517-OMcp52_217*
 ROcp52_417)+qdd[17]*ROcp52_316+qdd[18]*ROcp52_617;
  RLcp52_188 = ROcp52_718*s->dpt[3][28];
  RLcp52_288 = ROcp52_818*s->dpt[3][28];
  RLcp52_388 = ROcp52_918*s->dpt[3][28];
  ORcp52_188 = OMcp52_218*RLcp52_388-OMcp52_318*RLcp52_288;
  ORcp52_288 = -(OMcp52_118*RLcp52_388-OMcp52_318*RLcp52_188);
  ORcp52_388 = OMcp52_118*RLcp52_288-OMcp52_218*RLcp52_188;
  PxF2[1] = q[1]+RLcp52_113+RLcp52_114+RLcp52_115+RLcp52_116+RLcp52_117+RLcp52_188;
  PxF2[2] = q[2]+RLcp52_213+RLcp52_214+RLcp52_215+RLcp52_216+RLcp52_217+RLcp52_288;
  PxF2[3] = q[3]+RLcp52_313+RLcp52_314+RLcp52_315+RLcp52_316+RLcp52_317+RLcp52_388;
  RxF2[1][1] = ROcp52_118;
  RxF2[1][2] = ROcp52_218;
  RxF2[1][3] = ROcp52_318;
  RxF2[2][1] = ROcp52_417;
  RxF2[2][2] = ROcp52_517;
  RxF2[2][3] = ROcp52_617;
  RxF2[3][1] = ROcp52_718;
  RxF2[3][2] = ROcp52_818;
  RxF2[3][3] = ROcp52_918;
  VxF2[1] = qd[1]+ORcp52_113+ORcp52_114+ORcp52_115+ORcp52_116+ORcp52_117+ORcp52_188;
  VxF2[2] = qd[2]+ORcp52_213+ORcp52_214+ORcp52_215+ORcp52_216+ORcp52_217+ORcp52_288;
  VxF2[3] = qd[3]+ORcp52_313+ORcp52_314+ORcp52_315+ORcp52_316+ORcp52_317+ORcp52_388;
  OMxF2[1] = OMcp52_118;
  OMxF2[2] = OMcp52_218;
  OMxF2[3] = OMcp52_318;
  AxF2[1] = qdd[1]+OMcp52_213*ORcp52_314+OMcp52_214*ORcp52_315+OMcp52_215*ORcp52_316+OMcp52_216*ORcp52_317+OMcp52_218*
 ORcp52_388+OMcp52_26*ORcp52_313-OMcp52_313*ORcp52_214-OMcp52_314*ORcp52_215-OMcp52_315*ORcp52_216-OMcp52_316*ORcp52_217-
 OMcp52_318*ORcp52_288-OMcp52_36*ORcp52_213+OPcp52_213*RLcp52_314+OPcp52_214*RLcp52_315+OPcp52_215*RLcp52_316+OPcp52_216*
 RLcp52_317+OPcp52_218*RLcp52_388+OPcp52_26*RLcp52_313-OPcp52_313*RLcp52_214-OPcp52_314*RLcp52_215-OPcp52_315*RLcp52_216-
 OPcp52_316*RLcp52_217-OPcp52_318*RLcp52_288-OPcp52_36*RLcp52_213;
  AxF2[2] = qdd[2]-OMcp52_113*ORcp52_314-OMcp52_114*ORcp52_315-OMcp52_115*ORcp52_316-OMcp52_116*ORcp52_317-OMcp52_118*
 ORcp52_388-OMcp52_16*ORcp52_313+OMcp52_313*ORcp52_114+OMcp52_314*ORcp52_115+OMcp52_315*ORcp52_116+OMcp52_316*ORcp52_117+
 OMcp52_318*ORcp52_188+OMcp52_36*ORcp52_113-OPcp52_113*RLcp52_314-OPcp52_114*RLcp52_315-OPcp52_115*RLcp52_316-OPcp52_116*
 RLcp52_317-OPcp52_118*RLcp52_388-OPcp52_16*RLcp52_313+OPcp52_313*RLcp52_114+OPcp52_314*RLcp52_115+OPcp52_315*RLcp52_116+
 OPcp52_316*RLcp52_117+OPcp52_318*RLcp52_188+OPcp52_36*RLcp52_113;
  AxF2[3] = qdd[3]+OMcp52_113*ORcp52_214+OMcp52_114*ORcp52_215+OMcp52_115*ORcp52_216+OMcp52_116*ORcp52_217+OMcp52_118*
 ORcp52_288+OMcp52_16*ORcp52_213-OMcp52_213*ORcp52_114-OMcp52_214*ORcp52_115-OMcp52_215*ORcp52_116-OMcp52_216*ORcp52_117-
 OMcp52_218*ORcp52_188-OMcp52_26*ORcp52_113+OPcp52_113*RLcp52_214+OPcp52_114*RLcp52_215+OPcp52_115*RLcp52_216+OPcp52_116*
 RLcp52_217+OPcp52_118*RLcp52_288+OPcp52_16*RLcp52_213-OPcp52_213*RLcp52_114-OPcp52_214*RLcp52_115-OPcp52_215*RLcp52_116-
 OPcp52_216*RLcp52_117-OPcp52_218*RLcp52_188-OPcp52_26*RLcp52_113;
  OMPxF2[1] = OPcp52_118;
  OMPxF2[2] = OPcp52_218;
  OMPxF2[3] = OPcp52_318;
 
// Sensor Forces Computation 

  SWr2 = user_ExtForces(PxF2,RxF2,VxF2,OMxF2,AxF2,OMPxF2,s,tsim,2);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc153 = ROcp52_118*SWr2[1]+ROcp52_218*SWr2[2]+ROcp52_318*SWr2[3];
  xfrc253 = ROcp52_417*SWr2[1]+ROcp52_517*SWr2[2]+ROcp52_617*SWr2[3];
  xfrc353 = ROcp52_718*SWr2[1]+ROcp52_818*SWr2[2]+ROcp52_918*SWr2[3];
  frc[1][18] = s->frc[1][18]+xfrc153;
  frc[2][18] = s->frc[2][18]+xfrc253;
  frc[3][18] = s->frc[3][18]+xfrc353;
  xtrq153 = ROcp52_118*SWr2[4]+ROcp52_218*SWr2[5]+ROcp52_318*SWr2[6];
  xtrq253 = ROcp52_417*SWr2[4]+ROcp52_517*SWr2[5]+ROcp52_617*SWr2[6];
  xtrq353 = ROcp52_718*SWr2[4]+ROcp52_818*SWr2[5]+ROcp52_918*SWr2[6];
  trq[1][18] = s->trq[1][18]+xtrq153-xfrc253*(SWr2[9]-s->l[3][18])+xfrc353*(SWr2[8]-s->l[2][18]);
  trq[2][18] = s->trq[2][18]+xtrq253+xfrc153*(SWr2[9]-s->l[3][18])-xfrc353*(SWr2[7]-s->l[1][18]);
  trq[3][18] = s->trq[3][18]+xtrq353-xfrc153*(SWr2[8]-s->l[2][18])+xfrc253*(SWr2[7]-s->l[1][18]);

// = = Block_0_0_1_3_0_1 = = 
 
// Sensor Kinematics 


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
  OMcp53_25 = qd[5]*C4;
  OMcp53_35 = qd[5]*S4;
  OMcp53_16 = qd[4]+qd[6]*S5;
  OMcp53_26 = OMcp53_25+qd[6]*ROcp53_85;
  OMcp53_36 = OMcp53_35+qd[6]*ROcp53_95;
  OPcp53_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp53_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp53_95-OMcp53_35*S5)-qdd[5]*C4-qdd[6]*ROcp53_85);
  OPcp53_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp53_85-OMcp53_25*S5)+qdd[5]*S4+qdd[6]*ROcp53_95;

// = = Block_0_0_1_3_0_4 = = 
 
// Sensor Kinematics 


  ROcp53_419 = ROcp53_46*C19+S19*S5;
  ROcp53_519 = ROcp53_56*C19+ROcp53_85*S19;
  ROcp53_619 = ROcp53_66*C19+ROcp53_95*S19;
  ROcp53_719 = -(ROcp53_46*S19-C19*S5);
  ROcp53_819 = -(ROcp53_56*S19-ROcp53_85*C19);
  ROcp53_919 = -(ROcp53_66*S19-ROcp53_95*C19);
  ROcp53_120 = ROcp53_16*C20-ROcp53_719*S20;
  ROcp53_220 = ROcp53_26*C20-ROcp53_819*S20;
  ROcp53_320 = ROcp53_36*C20-ROcp53_919*S20;
  ROcp53_720 = ROcp53_16*S20+ROcp53_719*C20;
  ROcp53_820 = ROcp53_26*S20+ROcp53_819*C20;
  ROcp53_920 = ROcp53_36*S20+ROcp53_919*C20;
  ROcp53_121 = ROcp53_120*C21+ROcp53_419*S21;
  ROcp53_221 = ROcp53_220*C21+ROcp53_519*S21;
  ROcp53_321 = ROcp53_320*C21+ROcp53_619*S21;
  ROcp53_421 = -(ROcp53_120*S21-ROcp53_419*C21);
  ROcp53_521 = -(ROcp53_220*S21-ROcp53_519*C21);
  ROcp53_621 = -(ROcp53_320*S21-ROcp53_619*C21);
  RLcp53_119 = ROcp53_16*s->dpt[1][3]+s->dpt[3][3]*S5;
  RLcp53_219 = ROcp53_26*s->dpt[1][3]+ROcp53_85*s->dpt[3][3];
  RLcp53_319 = ROcp53_36*s->dpt[1][3]+ROcp53_95*s->dpt[3][3];
  OMcp53_119 = OMcp53_16+qd[19]*ROcp53_16;
  OMcp53_219 = OMcp53_26+qd[19]*ROcp53_26;
  OMcp53_319 = OMcp53_36+qd[19]*ROcp53_36;
  ORcp53_119 = OMcp53_26*RLcp53_319-OMcp53_36*RLcp53_219;
  ORcp53_219 = -(OMcp53_16*RLcp53_319-OMcp53_36*RLcp53_119);
  ORcp53_319 = OMcp53_16*RLcp53_219-OMcp53_26*RLcp53_119;
  OMcp53_120 = OMcp53_119+qd[20]*ROcp53_419;
  OMcp53_220 = OMcp53_219+qd[20]*ROcp53_519;
  OMcp53_320 = OMcp53_319+qd[20]*ROcp53_619;
  OPcp53_120 = OPcp53_16+qd[19]*(OMcp53_26*ROcp53_36-OMcp53_36*ROcp53_26)+qd[20]*(OMcp53_219*ROcp53_619-OMcp53_319*
 ROcp53_519)+qdd[19]*ROcp53_16+qdd[20]*ROcp53_419;
  OPcp53_220 = OPcp53_26-qd[19]*(OMcp53_16*ROcp53_36-OMcp53_36*ROcp53_16)-qd[20]*(OMcp53_119*ROcp53_619-OMcp53_319*
 ROcp53_419)+qdd[19]*ROcp53_26+qdd[20]*ROcp53_519;
  OPcp53_320 = OPcp53_36+qd[19]*(OMcp53_16*ROcp53_26-OMcp53_26*ROcp53_16)+qd[20]*(OMcp53_119*ROcp53_519-OMcp53_219*
 ROcp53_419)+qdd[19]*ROcp53_36+qdd[20]*ROcp53_619;
  RLcp53_121 = ROcp53_720*s->dpt[3][32];
  RLcp53_221 = ROcp53_820*s->dpt[3][32];
  RLcp53_321 = ROcp53_920*s->dpt[3][32];
  OMcp53_121 = OMcp53_120+qd[21]*ROcp53_720;
  OMcp53_221 = OMcp53_220+qd[21]*ROcp53_820;
  OMcp53_321 = OMcp53_320+qd[21]*ROcp53_920;
  ORcp53_121 = OMcp53_220*RLcp53_321-OMcp53_320*RLcp53_221;
  ORcp53_221 = -(OMcp53_120*RLcp53_321-OMcp53_320*RLcp53_121);
  ORcp53_321 = OMcp53_120*RLcp53_221-OMcp53_220*RLcp53_121;
  OPcp53_121 = OPcp53_120+qd[21]*(OMcp53_220*ROcp53_920-OMcp53_320*ROcp53_820)+qdd[21]*ROcp53_720;
  OPcp53_221 = OPcp53_220-qd[21]*(OMcp53_120*ROcp53_920-OMcp53_320*ROcp53_720)+qdd[21]*ROcp53_820;
  OPcp53_321 = OPcp53_320+qd[21]*(OMcp53_120*ROcp53_820-OMcp53_220*ROcp53_720)+qdd[21]*ROcp53_920;
  RLcp53_189 = ROcp53_720*s->dpt[3][34];
  RLcp53_289 = ROcp53_820*s->dpt[3][34];
  RLcp53_389 = ROcp53_920*s->dpt[3][34];
  ORcp53_189 = OMcp53_221*RLcp53_389-OMcp53_321*RLcp53_289;
  ORcp53_289 = -(OMcp53_121*RLcp53_389-OMcp53_321*RLcp53_189);
  ORcp53_389 = OMcp53_121*RLcp53_289-OMcp53_221*RLcp53_189;
  PxF3[1] = q[1]+RLcp53_119+RLcp53_121+RLcp53_189;
  PxF3[2] = q[2]+RLcp53_219+RLcp53_221+RLcp53_289;
  PxF3[3] = q[3]+RLcp53_319+RLcp53_321+RLcp53_389;
  RxF3[1][1] = ROcp53_121;
  RxF3[1][2] = ROcp53_221;
  RxF3[1][3] = ROcp53_321;
  RxF3[2][1] = ROcp53_421;
  RxF3[2][2] = ROcp53_521;
  RxF3[2][3] = ROcp53_621;
  RxF3[3][1] = ROcp53_720;
  RxF3[3][2] = ROcp53_820;
  RxF3[3][3] = ROcp53_920;
  VxF3[1] = qd[1]+ORcp53_119+ORcp53_121+ORcp53_189;
  VxF3[2] = qd[2]+ORcp53_219+ORcp53_221+ORcp53_289;
  VxF3[3] = qd[3]+ORcp53_319+ORcp53_321+ORcp53_389;
  OMxF3[1] = OMcp53_121;
  OMxF3[2] = OMcp53_221;
  OMxF3[3] = OMcp53_321;
  AxF3[1] = qdd[1]+OMcp53_220*ORcp53_321+OMcp53_221*ORcp53_389+OMcp53_26*ORcp53_319-OMcp53_320*ORcp53_221-OMcp53_321*
 ORcp53_289-OMcp53_36*ORcp53_219+OPcp53_220*RLcp53_321+OPcp53_221*RLcp53_389+OPcp53_26*RLcp53_319-OPcp53_320*RLcp53_221-
 OPcp53_321*RLcp53_289-OPcp53_36*RLcp53_219;
  AxF3[2] = qdd[2]-OMcp53_120*ORcp53_321-OMcp53_121*ORcp53_389-OMcp53_16*ORcp53_319+OMcp53_320*ORcp53_121+OMcp53_321*
 ORcp53_189+OMcp53_36*ORcp53_119-OPcp53_120*RLcp53_321-OPcp53_121*RLcp53_389-OPcp53_16*RLcp53_319+OPcp53_320*RLcp53_121+
 OPcp53_321*RLcp53_189+OPcp53_36*RLcp53_119;
  AxF3[3] = qdd[3]+OMcp53_120*ORcp53_221+OMcp53_121*ORcp53_289+OMcp53_16*ORcp53_219-OMcp53_220*ORcp53_121-OMcp53_221*
 ORcp53_189-OMcp53_26*ORcp53_119+OPcp53_120*RLcp53_221+OPcp53_121*RLcp53_289+OPcp53_16*RLcp53_219-OPcp53_220*RLcp53_121-
 OPcp53_221*RLcp53_189-OPcp53_26*RLcp53_119;
  OMPxF3[1] = OPcp53_121;
  OMPxF3[2] = OPcp53_221;
  OMPxF3[3] = OPcp53_321;
 
// Sensor Forces Computation 

  SWr3 = user_ExtForces(PxF3,RxF3,VxF3,OMxF3,AxF3,OMPxF3,s,tsim,3);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc154 = ROcp53_121*SWr3[1]+ROcp53_221*SWr3[2]+ROcp53_321*SWr3[3];
  xfrc254 = ROcp53_421*SWr3[1]+ROcp53_521*SWr3[2]+ROcp53_621*SWr3[3];
  xfrc354 = ROcp53_720*SWr3[1]+ROcp53_820*SWr3[2]+ROcp53_920*SWr3[3];
  s->frc[1][21] = s->frc[1][21]+xfrc154;
  s->frc[2][21] = s->frc[2][21]+xfrc254;
  s->frc[3][21] = s->frc[3][21]+xfrc354;
  xtrq154 = ROcp53_121*SWr3[4]+ROcp53_221*SWr3[5]+ROcp53_321*SWr3[6];
  xtrq254 = ROcp53_421*SWr3[4]+ROcp53_521*SWr3[5]+ROcp53_621*SWr3[6];
  xtrq354 = ROcp53_720*SWr3[4]+ROcp53_820*SWr3[5]+ROcp53_920*SWr3[6];
  s->trq[1][21] = s->trq[1][21]+xtrq154-xfrc254*(SWr3[9]-s->l[3][21])+xfrc354*(SWr3[8]-s->l[2][21]);
  s->trq[2][21] = s->trq[2][21]+xtrq254+xfrc154*(SWr3[9]-s->l[3][21])-xfrc354*(SWr3[7]-s->l[1][21]);
  s->trq[3][21] = s->trq[3][21]+xtrq354-xfrc154*(SWr3[8]-s->l[2][21])+xfrc254*(SWr3[7]-s->l[1][21]);

// = = Block_0_0_1_4_0_1 = = 
 
// Sensor Kinematics 


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
  OMcp54_25 = qd[5]*C4;
  OMcp54_35 = qd[5]*S4;
  OMcp54_16 = qd[4]+qd[6]*S5;
  OMcp54_26 = OMcp54_25+qd[6]*ROcp54_85;
  OMcp54_36 = OMcp54_35+qd[6]*ROcp54_95;
  OPcp54_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp54_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp54_95-OMcp54_35*S5)-qdd[5]*C4-qdd[6]*ROcp54_85);
  OPcp54_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp54_85-OMcp54_25*S5)+qdd[5]*S4+qdd[6]*ROcp54_95;

// = = Block_0_0_1_4_0_4 = = 
 
// Sensor Kinematics 


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
  RLcp54_119 = ROcp54_16*s->dpt[1][3]+s->dpt[3][3]*S5;
  RLcp54_219 = ROcp54_26*s->dpt[1][3]+ROcp54_85*s->dpt[3][3];
  RLcp54_319 = ROcp54_36*s->dpt[1][3]+ROcp54_95*s->dpt[3][3];
  OMcp54_119 = OMcp54_16+qd[19]*ROcp54_16;
  OMcp54_219 = OMcp54_26+qd[19]*ROcp54_26;
  OMcp54_319 = OMcp54_36+qd[19]*ROcp54_36;
  ORcp54_119 = OMcp54_26*RLcp54_319-OMcp54_36*RLcp54_219;
  ORcp54_219 = -(OMcp54_16*RLcp54_319-OMcp54_36*RLcp54_119);
  ORcp54_319 = OMcp54_16*RLcp54_219-OMcp54_26*RLcp54_119;
  OMcp54_120 = OMcp54_119+qd[20]*ROcp54_419;
  OMcp54_220 = OMcp54_219+qd[20]*ROcp54_519;
  OMcp54_320 = OMcp54_319+qd[20]*ROcp54_619;
  OPcp54_120 = OPcp54_16+qd[19]*(OMcp54_26*ROcp54_36-OMcp54_36*ROcp54_26)+qd[20]*(OMcp54_219*ROcp54_619-OMcp54_319*
 ROcp54_519)+qdd[19]*ROcp54_16+qdd[20]*ROcp54_419;
  OPcp54_220 = OPcp54_26-qd[19]*(OMcp54_16*ROcp54_36-OMcp54_36*ROcp54_16)-qd[20]*(OMcp54_119*ROcp54_619-OMcp54_319*
 ROcp54_419)+qdd[19]*ROcp54_26+qdd[20]*ROcp54_519;
  OPcp54_320 = OPcp54_36+qd[19]*(OMcp54_16*ROcp54_26-OMcp54_26*ROcp54_16)+qd[20]*(OMcp54_119*ROcp54_519-OMcp54_219*
 ROcp54_419)+qdd[19]*ROcp54_36+qdd[20]*ROcp54_619;
  RLcp54_121 = ROcp54_720*s->dpt[3][32];
  RLcp54_221 = ROcp54_820*s->dpt[3][32];
  RLcp54_321 = ROcp54_920*s->dpt[3][32];
  OMcp54_121 = OMcp54_120+qd[21]*ROcp54_720;
  OMcp54_221 = OMcp54_220+qd[21]*ROcp54_820;
  OMcp54_321 = OMcp54_320+qd[21]*ROcp54_920;
  ORcp54_121 = OMcp54_220*RLcp54_321-OMcp54_320*RLcp54_221;
  ORcp54_221 = -(OMcp54_120*RLcp54_321-OMcp54_320*RLcp54_121);
  ORcp54_321 = OMcp54_120*RLcp54_221-OMcp54_220*RLcp54_121;
  OPcp54_121 = OPcp54_120+qd[21]*(OMcp54_220*ROcp54_920-OMcp54_320*ROcp54_820)+qdd[21]*ROcp54_720;
  OPcp54_221 = OPcp54_220-qd[21]*(OMcp54_120*ROcp54_920-OMcp54_320*ROcp54_720)+qdd[21]*ROcp54_820;
  OPcp54_321 = OPcp54_320+qd[21]*(OMcp54_120*ROcp54_820-OMcp54_220*ROcp54_720)+qdd[21]*ROcp54_920;
  RLcp54_190 = ROcp54_121*s->dpt[1][38]+ROcp54_720*s->dpt[3][38];
  RLcp54_290 = ROcp54_221*s->dpt[1][38]+ROcp54_820*s->dpt[3][38];
  RLcp54_390 = ROcp54_321*s->dpt[1][38]+ROcp54_920*s->dpt[3][38];
  ORcp54_190 = OMcp54_221*RLcp54_390-OMcp54_321*RLcp54_290;
  ORcp54_290 = -(OMcp54_121*RLcp54_390-OMcp54_321*RLcp54_190);
  ORcp54_390 = OMcp54_121*RLcp54_290-OMcp54_221*RLcp54_190;
  PxF4[1] = q[1]+RLcp54_119+RLcp54_121+RLcp54_190;
  PxF4[2] = q[2]+RLcp54_219+RLcp54_221+RLcp54_290;
  PxF4[3] = q[3]+RLcp54_319+RLcp54_321+RLcp54_390;
  RxF4[1][1] = ROcp54_121;
  RxF4[1][2] = ROcp54_221;
  RxF4[1][3] = ROcp54_321;
  RxF4[2][1] = ROcp54_421;
  RxF4[2][2] = ROcp54_521;
  RxF4[2][3] = ROcp54_621;
  RxF4[3][1] = ROcp54_720;
  RxF4[3][2] = ROcp54_820;
  RxF4[3][3] = ROcp54_920;
  VxF4[1] = qd[1]+ORcp54_119+ORcp54_121+ORcp54_190;
  VxF4[2] = qd[2]+ORcp54_219+ORcp54_221+ORcp54_290;
  VxF4[3] = qd[3]+ORcp54_319+ORcp54_321+ORcp54_390;
  OMxF4[1] = OMcp54_121;
  OMxF4[2] = OMcp54_221;
  OMxF4[3] = OMcp54_321;
  AxF4[1] = qdd[1]+OMcp54_220*ORcp54_321+OMcp54_221*ORcp54_390+OMcp54_26*ORcp54_319-OMcp54_320*ORcp54_221-OMcp54_321*
 ORcp54_290-OMcp54_36*ORcp54_219+OPcp54_220*RLcp54_321+OPcp54_221*RLcp54_390+OPcp54_26*RLcp54_319-OPcp54_320*RLcp54_221-
 OPcp54_321*RLcp54_290-OPcp54_36*RLcp54_219;
  AxF4[2] = qdd[2]-OMcp54_120*ORcp54_321-OMcp54_121*ORcp54_390-OMcp54_16*ORcp54_319+OMcp54_320*ORcp54_121+OMcp54_321*
 ORcp54_190+OMcp54_36*ORcp54_119-OPcp54_120*RLcp54_321-OPcp54_121*RLcp54_390-OPcp54_16*RLcp54_319+OPcp54_320*RLcp54_121+
 OPcp54_321*RLcp54_190+OPcp54_36*RLcp54_119;
  AxF4[3] = qdd[3]+OMcp54_120*ORcp54_221+OMcp54_121*ORcp54_290+OMcp54_16*ORcp54_219-OMcp54_220*ORcp54_121-OMcp54_221*
 ORcp54_190-OMcp54_26*ORcp54_119+OPcp54_120*RLcp54_221+OPcp54_121*RLcp54_290+OPcp54_16*RLcp54_219-OPcp54_220*RLcp54_121-
 OPcp54_221*RLcp54_190-OPcp54_26*RLcp54_119;
  OMPxF4[1] = OPcp54_121;
  OMPxF4[2] = OPcp54_221;
  OMPxF4[3] = OPcp54_321;
 
// Sensor Forces Computation 

  SWr4 = user_ExtForces(PxF4,RxF4,VxF4,OMxF4,AxF4,OMPxF4,s,tsim,4);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc155 = ROcp54_121*SWr4[1]+ROcp54_221*SWr4[2]+ROcp54_321*SWr4[3];
  xfrc255 = ROcp54_421*SWr4[1]+ROcp54_521*SWr4[2]+ROcp54_621*SWr4[3];
  xfrc355 = ROcp54_720*SWr4[1]+ROcp54_820*SWr4[2]+ROcp54_920*SWr4[3];
  frc[1][21] = s->frc[1][21]+xfrc155;
  frc[2][21] = s->frc[2][21]+xfrc255;
  frc[3][21] = s->frc[3][21]+xfrc355;
  xtrq155 = ROcp54_121*SWr4[4]+ROcp54_221*SWr4[5]+ROcp54_321*SWr4[6];
  xtrq255 = ROcp54_421*SWr4[4]+ROcp54_521*SWr4[5]+ROcp54_621*SWr4[6];
  xtrq355 = ROcp54_720*SWr4[4]+ROcp54_820*SWr4[5]+ROcp54_920*SWr4[6];
  trq[1][21] = s->trq[1][21]+xtrq155-xfrc255*(SWr4[9]-s->l[3][21])+xfrc355*(SWr4[8]-s->l[2][21]);
  trq[2][21] = s->trq[2][21]+xtrq255+xfrc155*(SWr4[9]-s->l[3][21])-xfrc355*(SWr4[7]-s->l[1][21]);
  trq[3][21] = s->trq[3][21]+xtrq355-xfrc155*(SWr4[8]-s->l[2][21])+xfrc255*(SWr4[7]-s->l[1][21]);

// = = Block_0_0_1_5_0_1 = = 
 
// Sensor Kinematics 


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
  OMcp55_25 = qd[5]*C4;
  OMcp55_35 = qd[5]*S4;
  OMcp55_16 = qd[4]+qd[6]*S5;
  OMcp55_26 = OMcp55_25+qd[6]*ROcp55_85;
  OMcp55_36 = OMcp55_35+qd[6]*ROcp55_95;
  OPcp55_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp55_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp55_95-OMcp55_35*S5)-qdd[5]*C4-qdd[6]*ROcp55_85);
  OPcp55_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp55_85-OMcp55_25*S5)+qdd[5]*S4+qdd[6]*ROcp55_95;

// = = Block_0_0_1_5_0_4 = = 
 
// Sensor Kinematics 


  ROcp55_419 = ROcp55_46*C19+S19*S5;
  ROcp55_519 = ROcp55_56*C19+ROcp55_85*S19;
  ROcp55_619 = ROcp55_66*C19+ROcp55_95*S19;
  ROcp55_719 = -(ROcp55_46*S19-C19*S5);
  ROcp55_819 = -(ROcp55_56*S19-ROcp55_85*C19);
  ROcp55_919 = -(ROcp55_66*S19-ROcp55_95*C19);
  ROcp55_120 = ROcp55_16*C20-ROcp55_719*S20;
  ROcp55_220 = ROcp55_26*C20-ROcp55_819*S20;
  ROcp55_320 = ROcp55_36*C20-ROcp55_919*S20;
  ROcp55_720 = ROcp55_16*S20+ROcp55_719*C20;
  ROcp55_820 = ROcp55_26*S20+ROcp55_819*C20;
  ROcp55_920 = ROcp55_36*S20+ROcp55_919*C20;
  ROcp55_121 = ROcp55_120*C21+ROcp55_419*S21;
  ROcp55_221 = ROcp55_220*C21+ROcp55_519*S21;
  ROcp55_321 = ROcp55_320*C21+ROcp55_619*S21;
  ROcp55_421 = -(ROcp55_120*S21-ROcp55_419*C21);
  ROcp55_521 = -(ROcp55_220*S21-ROcp55_519*C21);
  ROcp55_621 = -(ROcp55_320*S21-ROcp55_619*C21);
  RLcp55_119 = ROcp55_16*s->dpt[1][3]+s->dpt[3][3]*S5;
  RLcp55_219 = ROcp55_26*s->dpt[1][3]+ROcp55_85*s->dpt[3][3];
  RLcp55_319 = ROcp55_36*s->dpt[1][3]+ROcp55_95*s->dpt[3][3];
  OMcp55_119 = OMcp55_16+qd[19]*ROcp55_16;
  OMcp55_219 = OMcp55_26+qd[19]*ROcp55_26;
  OMcp55_319 = OMcp55_36+qd[19]*ROcp55_36;
  ORcp55_119 = OMcp55_26*RLcp55_319-OMcp55_36*RLcp55_219;
  ORcp55_219 = -(OMcp55_16*RLcp55_319-OMcp55_36*RLcp55_119);
  ORcp55_319 = OMcp55_16*RLcp55_219-OMcp55_26*RLcp55_119;
  OMcp55_120 = OMcp55_119+qd[20]*ROcp55_419;
  OMcp55_220 = OMcp55_219+qd[20]*ROcp55_519;
  OMcp55_320 = OMcp55_319+qd[20]*ROcp55_619;
  OPcp55_120 = OPcp55_16+qd[19]*(OMcp55_26*ROcp55_36-OMcp55_36*ROcp55_26)+qd[20]*(OMcp55_219*ROcp55_619-OMcp55_319*
 ROcp55_519)+qdd[19]*ROcp55_16+qdd[20]*ROcp55_419;
  OPcp55_220 = OPcp55_26-qd[19]*(OMcp55_16*ROcp55_36-OMcp55_36*ROcp55_16)-qd[20]*(OMcp55_119*ROcp55_619-OMcp55_319*
 ROcp55_419)+qdd[19]*ROcp55_26+qdd[20]*ROcp55_519;
  OPcp55_320 = OPcp55_36+qd[19]*(OMcp55_16*ROcp55_26-OMcp55_26*ROcp55_16)+qd[20]*(OMcp55_119*ROcp55_519-OMcp55_219*
 ROcp55_419)+qdd[19]*ROcp55_36+qdd[20]*ROcp55_619;
  RLcp55_121 = ROcp55_720*s->dpt[3][32];
  RLcp55_221 = ROcp55_820*s->dpt[3][32];
  RLcp55_321 = ROcp55_920*s->dpt[3][32];
  OMcp55_121 = OMcp55_120+qd[21]*ROcp55_720;
  OMcp55_221 = OMcp55_220+qd[21]*ROcp55_820;
  OMcp55_321 = OMcp55_320+qd[21]*ROcp55_920;
  ORcp55_121 = OMcp55_220*RLcp55_321-OMcp55_320*RLcp55_221;
  ORcp55_221 = -(OMcp55_120*RLcp55_321-OMcp55_320*RLcp55_121);
  ORcp55_321 = OMcp55_120*RLcp55_221-OMcp55_220*RLcp55_121;
  OPcp55_121 = OPcp55_120+qd[21]*(OMcp55_220*ROcp55_920-OMcp55_320*ROcp55_820)+qdd[21]*ROcp55_720;
  OPcp55_221 = OPcp55_220-qd[21]*(OMcp55_120*ROcp55_920-OMcp55_320*ROcp55_720)+qdd[21]*ROcp55_820;
  OPcp55_321 = OPcp55_320+qd[21]*(OMcp55_120*ROcp55_820-OMcp55_220*ROcp55_720)+qdd[21]*ROcp55_920;

// = = Block_0_0_1_5_0_5 = = 
 
// Sensor Kinematics 


  ROcp55_122 = ROcp55_121*C22-ROcp55_720*S22;
  ROcp55_222 = ROcp55_221*C22-ROcp55_820*S22;
  ROcp55_322 = ROcp55_321*C22-ROcp55_920*S22;
  ROcp55_722 = ROcp55_121*S22+ROcp55_720*C22;
  ROcp55_822 = ROcp55_221*S22+ROcp55_820*C22;
  ROcp55_922 = ROcp55_321*S22+ROcp55_920*C22;
  ROcp55_423 = ROcp55_421*C23+ROcp55_722*S23;
  ROcp55_523 = ROcp55_521*C23+ROcp55_822*S23;
  ROcp55_623 = ROcp55_621*C23+ROcp55_922*S23;
  ROcp55_723 = -(ROcp55_421*S23-ROcp55_722*C23);
  ROcp55_823 = -(ROcp55_521*S23-ROcp55_822*C23);
  ROcp55_923 = -(ROcp55_621*S23-ROcp55_922*C23);
  ROcp55_124 = ROcp55_122*C24-ROcp55_723*S24;
  ROcp55_224 = ROcp55_222*C24-ROcp55_823*S24;
  ROcp55_324 = ROcp55_322*C24-ROcp55_923*S24;
  ROcp55_724 = ROcp55_122*S24+ROcp55_723*C24;
  ROcp55_824 = ROcp55_222*S24+ROcp55_823*C24;
  ROcp55_924 = ROcp55_322*S24+ROcp55_923*C24;
  ROcp55_125 = ROcp55_124*C25+ROcp55_423*S25;
  ROcp55_225 = ROcp55_224*C25+ROcp55_523*S25;
  ROcp55_325 = ROcp55_324*C25+ROcp55_623*S25;
  ROcp55_425 = -(ROcp55_124*S25-ROcp55_423*C25);
  ROcp55_525 = -(ROcp55_224*S25-ROcp55_523*C25);
  ROcp55_625 = -(ROcp55_324*S25-ROcp55_623*C25);
  ROcp55_126 = ROcp55_125*C26-ROcp55_724*S26;
  ROcp55_226 = ROcp55_225*C26-ROcp55_824*S26;
  ROcp55_326 = ROcp55_325*C26-ROcp55_924*S26;
  ROcp55_726 = ROcp55_125*S26+ROcp55_724*C26;
  ROcp55_826 = ROcp55_225*S26+ROcp55_824*C26;
  ROcp55_926 = ROcp55_325*S26+ROcp55_924*C26;
  ROcp55_127 = ROcp55_126*C27+ROcp55_425*S27;
  ROcp55_227 = ROcp55_226*C27+ROcp55_525*S27;
  ROcp55_327 = ROcp55_326*C27+ROcp55_625*S27;
  ROcp55_427 = -(ROcp55_126*S27-ROcp55_425*C27);
  ROcp55_527 = -(ROcp55_226*S27-ROcp55_525*C27);
  ROcp55_627 = -(ROcp55_326*S27-ROcp55_625*C27);
  ROcp55_428 = ROcp55_427*C28+ROcp55_726*S28;
  ROcp55_528 = ROcp55_527*C28+ROcp55_826*S28;
  ROcp55_628 = ROcp55_627*C28+ROcp55_926*S28;
  ROcp55_728 = -(ROcp55_427*S28-ROcp55_726*C28);
  ROcp55_828 = -(ROcp55_527*S28-ROcp55_826*C28);
  ROcp55_928 = -(ROcp55_627*S28-ROcp55_926*C28);
  RLcp55_122 = ROcp55_121*s->dpt[1][36]+ROcp55_421*s->dpt[2][36]+ROcp55_720*s->dpt[3][36];
  RLcp55_222 = ROcp55_221*s->dpt[1][36]+ROcp55_521*s->dpt[2][36]+ROcp55_820*s->dpt[3][36];
  RLcp55_322 = ROcp55_321*s->dpt[1][36]+ROcp55_621*s->dpt[2][36]+ROcp55_920*s->dpt[3][36];
  OMcp55_122 = OMcp55_121+qd[22]*ROcp55_421;
  OMcp55_222 = OMcp55_221+qd[22]*ROcp55_521;
  OMcp55_322 = OMcp55_321+qd[22]*ROcp55_621;
  ORcp55_122 = OMcp55_221*RLcp55_322-OMcp55_321*RLcp55_222;
  ORcp55_222 = -(OMcp55_121*RLcp55_322-OMcp55_321*RLcp55_122);
  ORcp55_322 = OMcp55_121*RLcp55_222-OMcp55_221*RLcp55_122;
  OPcp55_122 = OPcp55_121+qd[22]*(OMcp55_221*ROcp55_621-OMcp55_321*ROcp55_521)+qdd[22]*ROcp55_421;
  OPcp55_222 = OPcp55_221-qd[22]*(OMcp55_121*ROcp55_621-OMcp55_321*ROcp55_421)+qdd[22]*ROcp55_521;
  OPcp55_322 = OPcp55_321+qd[22]*(OMcp55_121*ROcp55_521-OMcp55_221*ROcp55_421)+qdd[22]*ROcp55_621;
  RLcp55_123 = ROcp55_421*s->dpt[2][40];
  RLcp55_223 = ROcp55_521*s->dpt[2][40];
  RLcp55_323 = ROcp55_621*s->dpt[2][40];
  OMcp55_123 = OMcp55_122+qd[23]*ROcp55_122;
  OMcp55_223 = OMcp55_222+qd[23]*ROcp55_222;
  OMcp55_323 = OMcp55_322+qd[23]*ROcp55_322;
  ORcp55_123 = OMcp55_222*RLcp55_323-OMcp55_322*RLcp55_223;
  ORcp55_223 = -(OMcp55_122*RLcp55_323-OMcp55_322*RLcp55_123);
  ORcp55_323 = OMcp55_122*RLcp55_223-OMcp55_222*RLcp55_123;
  OPcp55_123 = OPcp55_122+qd[23]*(OMcp55_222*ROcp55_322-OMcp55_322*ROcp55_222)+qdd[23]*ROcp55_122;
  OPcp55_223 = OPcp55_222-qd[23]*(OMcp55_122*ROcp55_322-OMcp55_322*ROcp55_122)+qdd[23]*ROcp55_222;
  OPcp55_323 = OPcp55_322+qd[23]*(OMcp55_122*ROcp55_222-OMcp55_222*ROcp55_122)+qdd[23]*ROcp55_322;
  RLcp55_124 = ROcp55_423*s->dpt[2][42];
  RLcp55_224 = ROcp55_523*s->dpt[2][42];
  RLcp55_324 = ROcp55_623*s->dpt[2][42];
  OMcp55_124 = OMcp55_123+qd[24]*ROcp55_423;
  OMcp55_224 = OMcp55_223+qd[24]*ROcp55_523;
  OMcp55_324 = OMcp55_323+qd[24]*ROcp55_623;
  ORcp55_124 = OMcp55_223*RLcp55_324-OMcp55_323*RLcp55_224;
  ORcp55_224 = -(OMcp55_123*RLcp55_324-OMcp55_323*RLcp55_124);
  ORcp55_324 = OMcp55_123*RLcp55_224-OMcp55_223*RLcp55_124;
  OPcp55_124 = OPcp55_123+qd[24]*(OMcp55_223*ROcp55_623-OMcp55_323*ROcp55_523)+qdd[24]*ROcp55_423;
  OPcp55_224 = OPcp55_223-qd[24]*(OMcp55_123*ROcp55_623-OMcp55_323*ROcp55_423)+qdd[24]*ROcp55_523;
  OPcp55_324 = OPcp55_323+qd[24]*(OMcp55_123*ROcp55_523-OMcp55_223*ROcp55_423)+qdd[24]*ROcp55_623;
  RLcp55_125 = ROcp55_124*s->dpt[1][44]+ROcp55_423*s->dpt[2][44];
  RLcp55_225 = ROcp55_224*s->dpt[1][44]+ROcp55_523*s->dpt[2][44];
  RLcp55_325 = ROcp55_324*s->dpt[1][44]+ROcp55_623*s->dpt[2][44];
  OMcp55_125 = OMcp55_124+qd[25]*ROcp55_724;
  OMcp55_225 = OMcp55_224+qd[25]*ROcp55_824;
  OMcp55_325 = OMcp55_324+qd[25]*ROcp55_924;
  ORcp55_125 = OMcp55_224*RLcp55_325-OMcp55_324*RLcp55_225;
  ORcp55_225 = -(OMcp55_124*RLcp55_325-OMcp55_324*RLcp55_125);
  ORcp55_325 = OMcp55_124*RLcp55_225-OMcp55_224*RLcp55_125;
  OPcp55_125 = OPcp55_124+qd[25]*(OMcp55_224*ROcp55_924-OMcp55_324*ROcp55_824)+qdd[25]*ROcp55_724;
  OPcp55_225 = OPcp55_224-qd[25]*(OMcp55_124*ROcp55_924-OMcp55_324*ROcp55_724)+qdd[25]*ROcp55_824;
  OPcp55_325 = OPcp55_324+qd[25]*(OMcp55_124*ROcp55_824-OMcp55_224*ROcp55_724)+qdd[25]*ROcp55_924;
  RLcp55_126 = ROcp55_125*s->dpt[1][46];
  RLcp55_226 = ROcp55_225*s->dpt[1][46];
  RLcp55_326 = ROcp55_325*s->dpt[1][46];
  OMcp55_126 = OMcp55_125+qd[26]*ROcp55_425;
  OMcp55_226 = OMcp55_225+qd[26]*ROcp55_525;
  OMcp55_326 = OMcp55_325+qd[26]*ROcp55_625;
  ORcp55_126 = OMcp55_225*RLcp55_326-OMcp55_325*RLcp55_226;
  ORcp55_226 = -(OMcp55_125*RLcp55_326-OMcp55_325*RLcp55_126);
  ORcp55_326 = OMcp55_125*RLcp55_226-OMcp55_225*RLcp55_126;
  OPcp55_126 = OPcp55_125+qd[26]*(OMcp55_225*ROcp55_625-OMcp55_325*ROcp55_525)+qdd[26]*ROcp55_425;
  OPcp55_226 = OPcp55_225-qd[26]*(OMcp55_125*ROcp55_625-OMcp55_325*ROcp55_425)+qdd[26]*ROcp55_525;
  OPcp55_326 = OPcp55_325+qd[26]*(OMcp55_125*ROcp55_525-OMcp55_225*ROcp55_425)+qdd[26]*ROcp55_625;
  RLcp55_127 = ROcp55_425*s->dpt[2][47];
  RLcp55_227 = ROcp55_525*s->dpt[2][47];
  RLcp55_327 = ROcp55_625*s->dpt[2][47];
  OMcp55_127 = OMcp55_126+qd[27]*ROcp55_726;
  OMcp55_227 = OMcp55_226+qd[27]*ROcp55_826;
  OMcp55_327 = OMcp55_326+qd[27]*ROcp55_926;
  ORcp55_127 = OMcp55_226*RLcp55_327-OMcp55_326*RLcp55_227;
  ORcp55_227 = -(OMcp55_126*RLcp55_327-OMcp55_326*RLcp55_127);
  ORcp55_327 = OMcp55_126*RLcp55_227-OMcp55_226*RLcp55_127;
  OMcp55_128 = OMcp55_127+qd[28]*ROcp55_127;
  OMcp55_228 = OMcp55_227+qd[28]*ROcp55_227;
  OMcp55_328 = OMcp55_327+qd[28]*ROcp55_327;
  OPcp55_128 = OPcp55_126+qd[27]*(OMcp55_226*ROcp55_926-OMcp55_326*ROcp55_826)+qd[28]*(OMcp55_227*ROcp55_327-OMcp55_327*
 ROcp55_227)+qdd[27]*ROcp55_726+qdd[28]*ROcp55_127;
  OPcp55_228 = OPcp55_226-qd[27]*(OMcp55_126*ROcp55_926-OMcp55_326*ROcp55_726)-qd[28]*(OMcp55_127*ROcp55_327-OMcp55_327*
 ROcp55_127)+qdd[27]*ROcp55_826+qdd[28]*ROcp55_227;
  OPcp55_328 = OPcp55_326+qd[27]*(OMcp55_126*ROcp55_826-OMcp55_226*ROcp55_726)+qd[28]*(OMcp55_127*ROcp55_227-OMcp55_227*
 ROcp55_127)+qdd[27]*ROcp55_926+qdd[28]*ROcp55_327;
  RLcp55_191 = ROcp55_428*s->dpt[2][49];
  RLcp55_291 = ROcp55_528*s->dpt[2][49];
  RLcp55_391 = ROcp55_628*s->dpt[2][49];
  ORcp55_191 = OMcp55_228*RLcp55_391-OMcp55_328*RLcp55_291;
  ORcp55_291 = -(OMcp55_128*RLcp55_391-OMcp55_328*RLcp55_191);
  ORcp55_391 = OMcp55_128*RLcp55_291-OMcp55_228*RLcp55_191;
  PxF5[1] = q[1]+RLcp55_119+RLcp55_121+RLcp55_122+RLcp55_123+RLcp55_124+RLcp55_125+RLcp55_126+RLcp55_127+RLcp55_191;
  PxF5[2] = q[2]+RLcp55_219+RLcp55_221+RLcp55_222+RLcp55_223+RLcp55_224+RLcp55_225+RLcp55_226+RLcp55_227+RLcp55_291;
  PxF5[3] = q[3]+RLcp55_319+RLcp55_321+RLcp55_322+RLcp55_323+RLcp55_324+RLcp55_325+RLcp55_326+RLcp55_327+RLcp55_391;
  RxF5[1][1] = ROcp55_127;
  RxF5[1][2] = ROcp55_227;
  RxF5[1][3] = ROcp55_327;
  RxF5[2][1] = ROcp55_428;
  RxF5[2][2] = ROcp55_528;
  RxF5[2][3] = ROcp55_628;
  RxF5[3][1] = ROcp55_728;
  RxF5[3][2] = ROcp55_828;
  RxF5[3][3] = ROcp55_928;
  VxF5[1] = qd[1]+ORcp55_119+ORcp55_121+ORcp55_122+ORcp55_123+ORcp55_124+ORcp55_125+ORcp55_126+ORcp55_127+ORcp55_191;
  VxF5[2] = qd[2]+ORcp55_219+ORcp55_221+ORcp55_222+ORcp55_223+ORcp55_224+ORcp55_225+ORcp55_226+ORcp55_227+ORcp55_291;
  VxF5[3] = qd[3]+ORcp55_319+ORcp55_321+ORcp55_322+ORcp55_323+ORcp55_324+ORcp55_325+ORcp55_326+ORcp55_327+ORcp55_391;
  OMxF5[1] = OMcp55_128;
  OMxF5[2] = OMcp55_228;
  OMxF5[3] = OMcp55_328;
  AxF5[1] = qdd[1]+OMcp55_220*ORcp55_321+OMcp55_221*ORcp55_322+OMcp55_222*ORcp55_323+OMcp55_223*ORcp55_324+OMcp55_224*
 ORcp55_325+OMcp55_225*ORcp55_326+OMcp55_226*ORcp55_327+OMcp55_228*ORcp55_391+OMcp55_26*ORcp55_319-OMcp55_320*ORcp55_221-
 OMcp55_321*ORcp55_222-OMcp55_322*ORcp55_223-OMcp55_323*ORcp55_224-OMcp55_324*ORcp55_225-OMcp55_325*ORcp55_226-OMcp55_326*
 ORcp55_227-OMcp55_328*ORcp55_291-OMcp55_36*ORcp55_219+OPcp55_220*RLcp55_321+OPcp55_221*RLcp55_322+OPcp55_222*RLcp55_323+
 OPcp55_223*RLcp55_324+OPcp55_224*RLcp55_325+OPcp55_225*RLcp55_326+OPcp55_226*RLcp55_327+OPcp55_228*RLcp55_391+OPcp55_26*
 RLcp55_319-OPcp55_320*RLcp55_221-OPcp55_321*RLcp55_222-OPcp55_322*RLcp55_223-OPcp55_323*RLcp55_224-OPcp55_324*RLcp55_225-
 OPcp55_325*RLcp55_226-OPcp55_326*RLcp55_227-OPcp55_328*RLcp55_291-OPcp55_36*RLcp55_219;
  AxF5[2] = qdd[2]-OMcp55_120*ORcp55_321-OMcp55_121*ORcp55_322-OMcp55_122*ORcp55_323-OMcp55_123*ORcp55_324-OMcp55_124*
 ORcp55_325-OMcp55_125*ORcp55_326-OMcp55_126*ORcp55_327-OMcp55_128*ORcp55_391-OMcp55_16*ORcp55_319+OMcp55_320*ORcp55_121+
 OMcp55_321*ORcp55_122+OMcp55_322*ORcp55_123+OMcp55_323*ORcp55_124+OMcp55_324*ORcp55_125+OMcp55_325*ORcp55_126+OMcp55_326*
 ORcp55_127+OMcp55_328*ORcp55_191+OMcp55_36*ORcp55_119-OPcp55_120*RLcp55_321-OPcp55_121*RLcp55_322-OPcp55_122*RLcp55_323-
 OPcp55_123*RLcp55_324-OPcp55_124*RLcp55_325-OPcp55_125*RLcp55_326-OPcp55_126*RLcp55_327-OPcp55_128*RLcp55_391-OPcp55_16*
 RLcp55_319+OPcp55_320*RLcp55_121+OPcp55_321*RLcp55_122+OPcp55_322*RLcp55_123+OPcp55_323*RLcp55_124+OPcp55_324*RLcp55_125+
 OPcp55_325*RLcp55_126+OPcp55_326*RLcp55_127+OPcp55_328*RLcp55_191+OPcp55_36*RLcp55_119;
  AxF5[3] = qdd[3]+OMcp55_120*ORcp55_221+OMcp55_121*ORcp55_222+OMcp55_122*ORcp55_223+OMcp55_123*ORcp55_224+OMcp55_124*
 ORcp55_225+OMcp55_125*ORcp55_226+OMcp55_126*ORcp55_227+OMcp55_128*ORcp55_291+OMcp55_16*ORcp55_219-OMcp55_220*ORcp55_121-
 OMcp55_221*ORcp55_122-OMcp55_222*ORcp55_123-OMcp55_223*ORcp55_124-OMcp55_224*ORcp55_125-OMcp55_225*ORcp55_126-OMcp55_226*
 ORcp55_127-OMcp55_228*ORcp55_191-OMcp55_26*ORcp55_119+OPcp55_120*RLcp55_221+OPcp55_121*RLcp55_222+OPcp55_122*RLcp55_223+
 OPcp55_123*RLcp55_224+OPcp55_124*RLcp55_225+OPcp55_125*RLcp55_226+OPcp55_126*RLcp55_227+OPcp55_128*RLcp55_291+OPcp55_16*
 RLcp55_219-OPcp55_220*RLcp55_121-OPcp55_221*RLcp55_122-OPcp55_222*RLcp55_123-OPcp55_223*RLcp55_124-OPcp55_224*RLcp55_125-
 OPcp55_225*RLcp55_126-OPcp55_226*RLcp55_127-OPcp55_228*RLcp55_191-OPcp55_26*RLcp55_119;
  OMPxF5[1] = OPcp55_128;
  OMPxF5[2] = OPcp55_228;
  OMPxF5[3] = OPcp55_328;
 
// Sensor Forces Computation 

  SWr5 = user_ExtForces(PxF5,RxF5,VxF5,OMxF5,AxF5,OMPxF5,s,tsim,5);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc156 = ROcp55_127*SWr5[1]+ROcp55_227*SWr5[2]+ROcp55_327*SWr5[3];
  xfrc256 = ROcp55_428*SWr5[1]+ROcp55_528*SWr5[2]+ROcp55_628*SWr5[3];
  xfrc356 = ROcp55_728*SWr5[1]+ROcp55_828*SWr5[2]+ROcp55_928*SWr5[3];
  frc[1][28] = s->frc[1][28]+xfrc156;
  frc[2][28] = s->frc[2][28]+xfrc256;
  frc[3][28] = s->frc[3][28]+xfrc356;
  xtrq156 = ROcp55_127*SWr5[4]+ROcp55_227*SWr5[5]+ROcp55_327*SWr5[6];
  xtrq256 = ROcp55_428*SWr5[4]+ROcp55_528*SWr5[5]+ROcp55_628*SWr5[6];
  xtrq356 = ROcp55_728*SWr5[4]+ROcp55_828*SWr5[5]+ROcp55_928*SWr5[6];
  trq[1][28] = s->trq[1][28]+xtrq156-xfrc256*SWr5[9]+xfrc356*SWr5[8];
  trq[2][28] = s->trq[2][28]+xtrq256+xfrc156*SWr5[9]-xfrc356*SWr5[7];
  trq[3][28] = s->trq[3][28]+xtrq356-xfrc156*SWr5[8]+xfrc256*SWr5[7];

// = = Block_0_0_1_6_0_1 = = 
 
// Sensor Kinematics 


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
  OMcp56_25 = qd[5]*C4;
  OMcp56_35 = qd[5]*S4;
  OMcp56_16 = qd[4]+qd[6]*S5;
  OMcp56_26 = OMcp56_25+qd[6]*ROcp56_85;
  OMcp56_36 = OMcp56_35+qd[6]*ROcp56_95;
  OPcp56_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp56_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp56_95-OMcp56_35*S5)-qdd[5]*C4-qdd[6]*ROcp56_85);
  OPcp56_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp56_85-OMcp56_25*S5)+qdd[5]*S4+qdd[6]*ROcp56_95;

// = = Block_0_0_1_6_0_4 = = 
 
// Sensor Kinematics 


  ROcp56_419 = ROcp56_46*C19+S19*S5;
  ROcp56_519 = ROcp56_56*C19+ROcp56_85*S19;
  ROcp56_619 = ROcp56_66*C19+ROcp56_95*S19;
  ROcp56_719 = -(ROcp56_46*S19-C19*S5);
  ROcp56_819 = -(ROcp56_56*S19-ROcp56_85*C19);
  ROcp56_919 = -(ROcp56_66*S19-ROcp56_95*C19);
  ROcp56_120 = ROcp56_16*C20-ROcp56_719*S20;
  ROcp56_220 = ROcp56_26*C20-ROcp56_819*S20;
  ROcp56_320 = ROcp56_36*C20-ROcp56_919*S20;
  ROcp56_720 = ROcp56_16*S20+ROcp56_719*C20;
  ROcp56_820 = ROcp56_26*S20+ROcp56_819*C20;
  ROcp56_920 = ROcp56_36*S20+ROcp56_919*C20;
  ROcp56_121 = ROcp56_120*C21+ROcp56_419*S21;
  ROcp56_221 = ROcp56_220*C21+ROcp56_519*S21;
  ROcp56_321 = ROcp56_320*C21+ROcp56_619*S21;
  ROcp56_421 = -(ROcp56_120*S21-ROcp56_419*C21);
  ROcp56_521 = -(ROcp56_220*S21-ROcp56_519*C21);
  ROcp56_621 = -(ROcp56_320*S21-ROcp56_619*C21);
  RLcp56_119 = ROcp56_16*s->dpt[1][3]+s->dpt[3][3]*S5;
  RLcp56_219 = ROcp56_26*s->dpt[1][3]+ROcp56_85*s->dpt[3][3];
  RLcp56_319 = ROcp56_36*s->dpt[1][3]+ROcp56_95*s->dpt[3][3];
  OMcp56_119 = OMcp56_16+qd[19]*ROcp56_16;
  OMcp56_219 = OMcp56_26+qd[19]*ROcp56_26;
  OMcp56_319 = OMcp56_36+qd[19]*ROcp56_36;
  ORcp56_119 = OMcp56_26*RLcp56_319-OMcp56_36*RLcp56_219;
  ORcp56_219 = -(OMcp56_16*RLcp56_319-OMcp56_36*RLcp56_119);
  ORcp56_319 = OMcp56_16*RLcp56_219-OMcp56_26*RLcp56_119;
  OMcp56_120 = OMcp56_119+qd[20]*ROcp56_419;
  OMcp56_220 = OMcp56_219+qd[20]*ROcp56_519;
  OMcp56_320 = OMcp56_319+qd[20]*ROcp56_619;
  OPcp56_120 = OPcp56_16+qd[19]*(OMcp56_26*ROcp56_36-OMcp56_36*ROcp56_26)+qd[20]*(OMcp56_219*ROcp56_619-OMcp56_319*
 ROcp56_519)+qdd[19]*ROcp56_16+qdd[20]*ROcp56_419;
  OPcp56_220 = OPcp56_26-qd[19]*(OMcp56_16*ROcp56_36-OMcp56_36*ROcp56_16)-qd[20]*(OMcp56_119*ROcp56_619-OMcp56_319*
 ROcp56_419)+qdd[19]*ROcp56_26+qdd[20]*ROcp56_519;
  OPcp56_320 = OPcp56_36+qd[19]*(OMcp56_16*ROcp56_26-OMcp56_26*ROcp56_16)+qd[20]*(OMcp56_119*ROcp56_519-OMcp56_219*
 ROcp56_419)+qdd[19]*ROcp56_36+qdd[20]*ROcp56_619;
  RLcp56_121 = ROcp56_720*s->dpt[3][32];
  RLcp56_221 = ROcp56_820*s->dpt[3][32];
  RLcp56_321 = ROcp56_920*s->dpt[3][32];
  OMcp56_121 = OMcp56_120+qd[21]*ROcp56_720;
  OMcp56_221 = OMcp56_220+qd[21]*ROcp56_820;
  OMcp56_321 = OMcp56_320+qd[21]*ROcp56_920;
  ORcp56_121 = OMcp56_220*RLcp56_321-OMcp56_320*RLcp56_221;
  ORcp56_221 = -(OMcp56_120*RLcp56_321-OMcp56_320*RLcp56_121);
  ORcp56_321 = OMcp56_120*RLcp56_221-OMcp56_220*RLcp56_121;
  OPcp56_121 = OPcp56_120+qd[21]*(OMcp56_220*ROcp56_920-OMcp56_320*ROcp56_820)+qdd[21]*ROcp56_720;
  OPcp56_221 = OPcp56_220-qd[21]*(OMcp56_120*ROcp56_920-OMcp56_320*ROcp56_720)+qdd[21]*ROcp56_820;
  OPcp56_321 = OPcp56_320+qd[21]*(OMcp56_120*ROcp56_820-OMcp56_220*ROcp56_720)+qdd[21]*ROcp56_920;

// = = Block_0_0_1_6_0_6 = = 
 
// Sensor Kinematics 


  ROcp56_129 = ROcp56_121*C29-ROcp56_720*S29;
  ROcp56_229 = ROcp56_221*C29-ROcp56_820*S29;
  ROcp56_329 = ROcp56_321*C29-ROcp56_920*S29;
  ROcp56_729 = ROcp56_121*S29+ROcp56_720*C29;
  ROcp56_829 = ROcp56_221*S29+ROcp56_820*C29;
  ROcp56_929 = ROcp56_321*S29+ROcp56_920*C29;
  ROcp56_430 = ROcp56_421*C30+ROcp56_729*S30;
  ROcp56_530 = ROcp56_521*C30+ROcp56_829*S30;
  ROcp56_630 = ROcp56_621*C30+ROcp56_929*S30;
  ROcp56_730 = -(ROcp56_421*S30-ROcp56_729*C30);
  ROcp56_830 = -(ROcp56_521*S30-ROcp56_829*C30);
  ROcp56_930 = -(ROcp56_621*S30-ROcp56_929*C30);
  ROcp56_131 = ROcp56_129*C31-ROcp56_730*S31;
  ROcp56_231 = ROcp56_229*C31-ROcp56_830*S31;
  ROcp56_331 = ROcp56_329*C31-ROcp56_930*S31;
  ROcp56_731 = ROcp56_129*S31+ROcp56_730*C31;
  ROcp56_831 = ROcp56_229*S31+ROcp56_830*C31;
  ROcp56_931 = ROcp56_329*S31+ROcp56_930*C31;
  ROcp56_132 = ROcp56_131*C32+ROcp56_430*S32;
  ROcp56_232 = ROcp56_231*C32+ROcp56_530*S32;
  ROcp56_332 = ROcp56_331*C32+ROcp56_630*S32;
  ROcp56_432 = -(ROcp56_131*S32-ROcp56_430*C32);
  ROcp56_532 = -(ROcp56_231*S32-ROcp56_530*C32);
  ROcp56_632 = -(ROcp56_331*S32-ROcp56_630*C32);
  ROcp56_133 = ROcp56_132*C33-ROcp56_731*S33;
  ROcp56_233 = ROcp56_232*C33-ROcp56_831*S33;
  ROcp56_333 = ROcp56_332*C33-ROcp56_931*S33;
  ROcp56_733 = ROcp56_132*S33+ROcp56_731*C33;
  ROcp56_833 = ROcp56_232*S33+ROcp56_831*C33;
  ROcp56_933 = ROcp56_332*S33+ROcp56_931*C33;
  ROcp56_134 = ROcp56_133*C34+ROcp56_432*S34;
  ROcp56_234 = ROcp56_233*C34+ROcp56_532*S34;
  ROcp56_334 = ROcp56_333*C34+ROcp56_632*S34;
  ROcp56_434 = -(ROcp56_133*S34-ROcp56_432*C34);
  ROcp56_534 = -(ROcp56_233*S34-ROcp56_532*C34);
  ROcp56_634 = -(ROcp56_333*S34-ROcp56_632*C34);
  ROcp56_435 = ROcp56_434*C35+ROcp56_733*S35;
  ROcp56_535 = ROcp56_534*C35+ROcp56_833*S35;
  ROcp56_635 = ROcp56_634*C35+ROcp56_933*S35;
  ROcp56_735 = -(ROcp56_434*S35-ROcp56_733*C35);
  ROcp56_835 = -(ROcp56_534*S35-ROcp56_833*C35);
  ROcp56_935 = -(ROcp56_634*S35-ROcp56_933*C35);
  RLcp56_129 = ROcp56_121*s->dpt[1][37]+ROcp56_421*s->dpt[2][37]+ROcp56_720*s->dpt[3][37];
  RLcp56_229 = ROcp56_221*s->dpt[1][37]+ROcp56_521*s->dpt[2][37]+ROcp56_820*s->dpt[3][37];
  RLcp56_329 = ROcp56_321*s->dpt[1][37]+ROcp56_621*s->dpt[2][37]+ROcp56_920*s->dpt[3][37];
  OMcp56_129 = OMcp56_121+qd[29]*ROcp56_421;
  OMcp56_229 = OMcp56_221+qd[29]*ROcp56_521;
  OMcp56_329 = OMcp56_321+qd[29]*ROcp56_621;
  ORcp56_129 = OMcp56_221*RLcp56_329-OMcp56_321*RLcp56_229;
  ORcp56_229 = -(OMcp56_121*RLcp56_329-OMcp56_321*RLcp56_129);
  ORcp56_329 = OMcp56_121*RLcp56_229-OMcp56_221*RLcp56_129;
  OPcp56_129 = OPcp56_121+qd[29]*(OMcp56_221*ROcp56_621-OMcp56_321*ROcp56_521)+qdd[29]*ROcp56_421;
  OPcp56_229 = OPcp56_221-qd[29]*(OMcp56_121*ROcp56_621-OMcp56_321*ROcp56_421)+qdd[29]*ROcp56_521;
  OPcp56_329 = OPcp56_321+qd[29]*(OMcp56_121*ROcp56_521-OMcp56_221*ROcp56_421)+qdd[29]*ROcp56_621;
  RLcp56_130 = ROcp56_421*s->dpt[2][51];
  RLcp56_230 = ROcp56_521*s->dpt[2][51];
  RLcp56_330 = ROcp56_621*s->dpt[2][51];
  OMcp56_130 = OMcp56_129+qd[30]*ROcp56_129;
  OMcp56_230 = OMcp56_229+qd[30]*ROcp56_229;
  OMcp56_330 = OMcp56_329+qd[30]*ROcp56_329;
  ORcp56_130 = OMcp56_229*RLcp56_330-OMcp56_329*RLcp56_230;
  ORcp56_230 = -(OMcp56_129*RLcp56_330-OMcp56_329*RLcp56_130);
  ORcp56_330 = OMcp56_129*RLcp56_230-OMcp56_229*RLcp56_130;
  OPcp56_130 = OPcp56_129+qd[30]*(OMcp56_229*ROcp56_329-OMcp56_329*ROcp56_229)+qdd[30]*ROcp56_129;
  OPcp56_230 = OPcp56_229-qd[30]*(OMcp56_129*ROcp56_329-OMcp56_329*ROcp56_129)+qdd[30]*ROcp56_229;
  OPcp56_330 = OPcp56_329+qd[30]*(OMcp56_129*ROcp56_229-OMcp56_229*ROcp56_129)+qdd[30]*ROcp56_329;
  RLcp56_131 = ROcp56_430*s->dpt[2][53];
  RLcp56_231 = ROcp56_530*s->dpt[2][53];
  RLcp56_331 = ROcp56_630*s->dpt[2][53];
  OMcp56_131 = OMcp56_130+qd[31]*ROcp56_430;
  OMcp56_231 = OMcp56_230+qd[31]*ROcp56_530;
  OMcp56_331 = OMcp56_330+qd[31]*ROcp56_630;
  ORcp56_131 = OMcp56_230*RLcp56_331-OMcp56_330*RLcp56_231;
  ORcp56_231 = -(OMcp56_130*RLcp56_331-OMcp56_330*RLcp56_131);
  ORcp56_331 = OMcp56_130*RLcp56_231-OMcp56_230*RLcp56_131;
  OPcp56_131 = OPcp56_130+qd[31]*(OMcp56_230*ROcp56_630-OMcp56_330*ROcp56_530)+qdd[31]*ROcp56_430;
  OPcp56_231 = OPcp56_230-qd[31]*(OMcp56_130*ROcp56_630-OMcp56_330*ROcp56_430)+qdd[31]*ROcp56_530;
  OPcp56_331 = OPcp56_330+qd[31]*(OMcp56_130*ROcp56_530-OMcp56_230*ROcp56_430)+qdd[31]*ROcp56_630;
  RLcp56_132 = ROcp56_131*s->dpt[1][55]+ROcp56_430*s->dpt[2][55];
  RLcp56_232 = ROcp56_231*s->dpt[1][55]+ROcp56_530*s->dpt[2][55];
  RLcp56_332 = ROcp56_331*s->dpt[1][55]+ROcp56_630*s->dpt[2][55];
  OMcp56_132 = OMcp56_131+qd[32]*ROcp56_731;
  OMcp56_232 = OMcp56_231+qd[32]*ROcp56_831;
  OMcp56_332 = OMcp56_331+qd[32]*ROcp56_931;
  ORcp56_132 = OMcp56_231*RLcp56_332-OMcp56_331*RLcp56_232;
  ORcp56_232 = -(OMcp56_131*RLcp56_332-OMcp56_331*RLcp56_132);
  ORcp56_332 = OMcp56_131*RLcp56_232-OMcp56_231*RLcp56_132;
  OPcp56_132 = OPcp56_131+qd[32]*(OMcp56_231*ROcp56_931-OMcp56_331*ROcp56_831)+qdd[32]*ROcp56_731;
  OPcp56_232 = OPcp56_231-qd[32]*(OMcp56_131*ROcp56_931-OMcp56_331*ROcp56_731)+qdd[32]*ROcp56_831;
  OPcp56_332 = OPcp56_331+qd[32]*(OMcp56_131*ROcp56_831-OMcp56_231*ROcp56_731)+qdd[32]*ROcp56_931;
  RLcp56_133 = ROcp56_132*s->dpt[1][57];
  RLcp56_233 = ROcp56_232*s->dpt[1][57];
  RLcp56_333 = ROcp56_332*s->dpt[1][57];
  OMcp56_133 = OMcp56_132+qd[33]*ROcp56_432;
  OMcp56_233 = OMcp56_232+qd[33]*ROcp56_532;
  OMcp56_333 = OMcp56_332+qd[33]*ROcp56_632;
  ORcp56_133 = OMcp56_232*RLcp56_333-OMcp56_332*RLcp56_233;
  ORcp56_233 = -(OMcp56_132*RLcp56_333-OMcp56_332*RLcp56_133);
  ORcp56_333 = OMcp56_132*RLcp56_233-OMcp56_232*RLcp56_133;
  OPcp56_133 = OPcp56_132+qd[33]*(OMcp56_232*ROcp56_632-OMcp56_332*ROcp56_532)+qdd[33]*ROcp56_432;
  OPcp56_233 = OPcp56_232-qd[33]*(OMcp56_132*ROcp56_632-OMcp56_332*ROcp56_432)+qdd[33]*ROcp56_532;
  OPcp56_333 = OPcp56_332+qd[33]*(OMcp56_132*ROcp56_532-OMcp56_232*ROcp56_432)+qdd[33]*ROcp56_632;
  RLcp56_134 = ROcp56_432*s->dpt[2][58];
  RLcp56_234 = ROcp56_532*s->dpt[2][58];
  RLcp56_334 = ROcp56_632*s->dpt[2][58];
  OMcp56_134 = OMcp56_133+qd[34]*ROcp56_733;
  OMcp56_234 = OMcp56_233+qd[34]*ROcp56_833;
  OMcp56_334 = OMcp56_333+qd[34]*ROcp56_933;
  ORcp56_134 = OMcp56_233*RLcp56_334-OMcp56_333*RLcp56_234;
  ORcp56_234 = -(OMcp56_133*RLcp56_334-OMcp56_333*RLcp56_134);
  ORcp56_334 = OMcp56_133*RLcp56_234-OMcp56_233*RLcp56_134;
  OMcp56_135 = OMcp56_134+qd[35]*ROcp56_134;
  OMcp56_235 = OMcp56_234+qd[35]*ROcp56_234;
  OMcp56_335 = OMcp56_334+qd[35]*ROcp56_334;
  OPcp56_135 = OPcp56_133+qd[34]*(OMcp56_233*ROcp56_933-OMcp56_333*ROcp56_833)+qd[35]*(OMcp56_234*ROcp56_334-OMcp56_334*
 ROcp56_234)+qdd[34]*ROcp56_733+qdd[35]*ROcp56_134;
  OPcp56_235 = OPcp56_233-qd[34]*(OMcp56_133*ROcp56_933-OMcp56_333*ROcp56_733)-qd[35]*(OMcp56_134*ROcp56_334-OMcp56_334*
 ROcp56_134)+qdd[34]*ROcp56_833+qdd[35]*ROcp56_234;
  OPcp56_335 = OPcp56_333+qd[34]*(OMcp56_133*ROcp56_833-OMcp56_233*ROcp56_733)+qd[35]*(OMcp56_134*ROcp56_234-OMcp56_234*
 ROcp56_134)+qdd[34]*ROcp56_933+qdd[35]*ROcp56_334;
  RLcp56_192 = ROcp56_435*s->dpt[2][60];
  RLcp56_292 = ROcp56_535*s->dpt[2][60];
  RLcp56_392 = ROcp56_635*s->dpt[2][60];
  ORcp56_192 = OMcp56_235*RLcp56_392-OMcp56_335*RLcp56_292;
  ORcp56_292 = -(OMcp56_135*RLcp56_392-OMcp56_335*RLcp56_192);
  ORcp56_392 = OMcp56_135*RLcp56_292-OMcp56_235*RLcp56_192;
  PxF6[1] = q[1]+RLcp56_119+RLcp56_121+RLcp56_129+RLcp56_130+RLcp56_131+RLcp56_132+RLcp56_133+RLcp56_134+RLcp56_192;
  PxF6[2] = q[2]+RLcp56_219+RLcp56_221+RLcp56_229+RLcp56_230+RLcp56_231+RLcp56_232+RLcp56_233+RLcp56_234+RLcp56_292;
  PxF6[3] = q[3]+RLcp56_319+RLcp56_321+RLcp56_329+RLcp56_330+RLcp56_331+RLcp56_332+RLcp56_333+RLcp56_334+RLcp56_392;
  RxF6[1][1] = ROcp56_134;
  RxF6[1][2] = ROcp56_234;
  RxF6[1][3] = ROcp56_334;
  RxF6[2][1] = ROcp56_435;
  RxF6[2][2] = ROcp56_535;
  RxF6[2][3] = ROcp56_635;
  RxF6[3][1] = ROcp56_735;
  RxF6[3][2] = ROcp56_835;
  RxF6[3][3] = ROcp56_935;
  VxF6[1] = qd[1]+ORcp56_119+ORcp56_121+ORcp56_129+ORcp56_130+ORcp56_131+ORcp56_132+ORcp56_133+ORcp56_134+ORcp56_192;
  VxF6[2] = qd[2]+ORcp56_219+ORcp56_221+ORcp56_229+ORcp56_230+ORcp56_231+ORcp56_232+ORcp56_233+ORcp56_234+ORcp56_292;
  VxF6[3] = qd[3]+ORcp56_319+ORcp56_321+ORcp56_329+ORcp56_330+ORcp56_331+ORcp56_332+ORcp56_333+ORcp56_334+ORcp56_392;
  OMxF6[1] = OMcp56_135;
  OMxF6[2] = OMcp56_235;
  OMxF6[3] = OMcp56_335;
  AxF6[1] = qdd[1]+OMcp56_220*ORcp56_321+OMcp56_221*ORcp56_329+OMcp56_229*ORcp56_330+OMcp56_230*ORcp56_331+OMcp56_231*
 ORcp56_332+OMcp56_232*ORcp56_333+OMcp56_233*ORcp56_334+OMcp56_235*ORcp56_392+OMcp56_26*ORcp56_319-OMcp56_320*ORcp56_221-
 OMcp56_321*ORcp56_229-OMcp56_329*ORcp56_230-OMcp56_330*ORcp56_231-OMcp56_331*ORcp56_232-OMcp56_332*ORcp56_233-OMcp56_333*
 ORcp56_234-OMcp56_335*ORcp56_292-OMcp56_36*ORcp56_219+OPcp56_220*RLcp56_321+OPcp56_221*RLcp56_329+OPcp56_229*RLcp56_330+
 OPcp56_230*RLcp56_331+OPcp56_231*RLcp56_332+OPcp56_232*RLcp56_333+OPcp56_233*RLcp56_334+OPcp56_235*RLcp56_392+OPcp56_26*
 RLcp56_319-OPcp56_320*RLcp56_221-OPcp56_321*RLcp56_229-OPcp56_329*RLcp56_230-OPcp56_330*RLcp56_231-OPcp56_331*RLcp56_232-
 OPcp56_332*RLcp56_233-OPcp56_333*RLcp56_234-OPcp56_335*RLcp56_292-OPcp56_36*RLcp56_219;
  AxF6[2] = qdd[2]-OMcp56_120*ORcp56_321-OMcp56_121*ORcp56_329-OMcp56_129*ORcp56_330-OMcp56_130*ORcp56_331-OMcp56_131*
 ORcp56_332-OMcp56_132*ORcp56_333-OMcp56_133*ORcp56_334-OMcp56_135*ORcp56_392-OMcp56_16*ORcp56_319+OMcp56_320*ORcp56_121+
 OMcp56_321*ORcp56_129+OMcp56_329*ORcp56_130+OMcp56_330*ORcp56_131+OMcp56_331*ORcp56_132+OMcp56_332*ORcp56_133+OMcp56_333*
 ORcp56_134+OMcp56_335*ORcp56_192+OMcp56_36*ORcp56_119-OPcp56_120*RLcp56_321-OPcp56_121*RLcp56_329-OPcp56_129*RLcp56_330-
 OPcp56_130*RLcp56_331-OPcp56_131*RLcp56_332-OPcp56_132*RLcp56_333-OPcp56_133*RLcp56_334-OPcp56_135*RLcp56_392-OPcp56_16*
 RLcp56_319+OPcp56_320*RLcp56_121+OPcp56_321*RLcp56_129+OPcp56_329*RLcp56_130+OPcp56_330*RLcp56_131+OPcp56_331*RLcp56_132+
 OPcp56_332*RLcp56_133+OPcp56_333*RLcp56_134+OPcp56_335*RLcp56_192+OPcp56_36*RLcp56_119;
  AxF6[3] = qdd[3]+OMcp56_120*ORcp56_221+OMcp56_121*ORcp56_229+OMcp56_129*ORcp56_230+OMcp56_130*ORcp56_231+OMcp56_131*
 ORcp56_232+OMcp56_132*ORcp56_233+OMcp56_133*ORcp56_234+OMcp56_135*ORcp56_292+OMcp56_16*ORcp56_219-OMcp56_220*ORcp56_121-
 OMcp56_221*ORcp56_129-OMcp56_229*ORcp56_130-OMcp56_230*ORcp56_131-OMcp56_231*ORcp56_132-OMcp56_232*ORcp56_133-OMcp56_233*
 ORcp56_134-OMcp56_235*ORcp56_192-OMcp56_26*ORcp56_119+OPcp56_120*RLcp56_221+OPcp56_121*RLcp56_229+OPcp56_129*RLcp56_230+
 OPcp56_130*RLcp56_231+OPcp56_131*RLcp56_232+OPcp56_132*RLcp56_233+OPcp56_133*RLcp56_234+OPcp56_135*RLcp56_292+OPcp56_16*
 RLcp56_219-OPcp56_220*RLcp56_121-OPcp56_221*RLcp56_129-OPcp56_229*RLcp56_130-OPcp56_230*RLcp56_131-OPcp56_231*RLcp56_132-
 OPcp56_232*RLcp56_133-OPcp56_233*RLcp56_134-OPcp56_235*RLcp56_192-OPcp56_26*RLcp56_119;
  OMPxF6[1] = OPcp56_135;
  OMPxF6[2] = OPcp56_235;
  OMPxF6[3] = OPcp56_335;
 
// Sensor Forces Computation 

  SWr6 = user_ExtForces(PxF6,RxF6,VxF6,OMxF6,AxF6,OMPxF6,s,tsim,6);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc157 = ROcp56_134*SWr6[1]+ROcp56_234*SWr6[2]+ROcp56_334*SWr6[3];
  xfrc257 = ROcp56_435*SWr6[1]+ROcp56_535*SWr6[2]+ROcp56_635*SWr6[3];
  xfrc357 = ROcp56_735*SWr6[1]+ROcp56_835*SWr6[2]+ROcp56_935*SWr6[3];
  frc[1][35] = s->frc[1][35]+xfrc157;
  frc[2][35] = s->frc[2][35]+xfrc257;
  frc[3][35] = s->frc[3][35]+xfrc357;
  xtrq157 = ROcp56_134*SWr6[4]+ROcp56_234*SWr6[5]+ROcp56_334*SWr6[6];
  xtrq257 = ROcp56_435*SWr6[4]+ROcp56_535*SWr6[5]+ROcp56_635*SWr6[6];
  xtrq357 = ROcp56_735*SWr6[4]+ROcp56_835*SWr6[5]+ROcp56_935*SWr6[6];
  trq[1][35] = s->trq[1][35]+xtrq157-xfrc257*SWr6[9]+xfrc357*SWr6[8];
  trq[2][35] = s->trq[2][35]+xtrq257+xfrc157*SWr6[9]-xfrc357*SWr6[7];
  trq[3][35] = s->trq[3][35]+xtrq357-xfrc157*SWr6[8]+xfrc257*SWr6[7];

// = = Block_0_0_1_6_1_0 = = 
 
// Symbolic Outputs  

  frc[1][6] = s->frc[1][6];
  frc[2][6] = s->frc[2][6];
  frc[3][6] = s->frc[3][6];
  frc[1][7] = s->frc[1][7];
  frc[2][7] = s->frc[2][7];
  frc[3][7] = s->frc[3][7];
  frc[1][8] = s->frc[1][8];
  frc[2][8] = s->frc[2][8];
  frc[3][8] = s->frc[3][8];
  frc[1][9] = s->frc[1][9];
  frc[2][9] = s->frc[2][9];
  frc[3][9] = s->frc[3][9];
  frc[1][10] = s->frc[1][10];
  frc[2][10] = s->frc[2][10];
  frc[3][10] = s->frc[3][10];
  frc[1][11] = s->frc[1][11];
  frc[2][11] = s->frc[2][11];
  frc[3][11] = s->frc[3][11];
  frc[1][13] = s->frc[1][13];
  frc[2][13] = s->frc[2][13];
  frc[3][13] = s->frc[3][13];
  frc[1][14] = s->frc[1][14];
  frc[2][14] = s->frc[2][14];
  frc[3][14] = s->frc[3][14];
  frc[1][15] = s->frc[1][15];
  frc[2][15] = s->frc[2][15];
  frc[3][15] = s->frc[3][15];
  frc[1][16] = s->frc[1][16];
  frc[2][16] = s->frc[2][16];
  frc[3][16] = s->frc[3][16];
  frc[1][17] = s->frc[1][17];
  frc[2][17] = s->frc[2][17];
  frc[3][17] = s->frc[3][17];
  frc[1][19] = s->frc[1][19];
  frc[2][19] = s->frc[2][19];
  frc[3][19] = s->frc[3][19];
  frc[1][20] = s->frc[1][20];
  frc[2][20] = s->frc[2][20];
  frc[3][20] = s->frc[3][20];
  frc[1][22] = s->frc[1][22];
  frc[2][22] = s->frc[2][22];
  frc[3][22] = s->frc[3][22];
  frc[1][23] = s->frc[1][23];
  frc[2][23] = s->frc[2][23];
  frc[3][23] = s->frc[3][23];
  frc[1][24] = s->frc[1][24];
  frc[2][24] = s->frc[2][24];
  frc[3][24] = s->frc[3][24];
  frc[1][25] = s->frc[1][25];
  frc[2][25] = s->frc[2][25];
  frc[3][25] = s->frc[3][25];
  frc[1][26] = s->frc[1][26];
  frc[2][26] = s->frc[2][26];
  frc[3][26] = s->frc[3][26];
  frc[1][27] = s->frc[1][27];
  frc[2][27] = s->frc[2][27];
  frc[3][27] = s->frc[3][27];
  frc[1][29] = s->frc[1][29];
  frc[2][29] = s->frc[2][29];
  frc[3][29] = s->frc[3][29];
  frc[1][30] = s->frc[1][30];
  frc[2][30] = s->frc[2][30];
  frc[3][30] = s->frc[3][30];
  frc[1][31] = s->frc[1][31];
  frc[2][31] = s->frc[2][31];
  frc[3][31] = s->frc[3][31];
  frc[1][32] = s->frc[1][32];
  frc[2][32] = s->frc[2][32];
  frc[3][32] = s->frc[3][32];
  frc[1][33] = s->frc[1][33];
  frc[2][33] = s->frc[2][33];
  frc[3][33] = s->frc[3][33];
  frc[1][34] = s->frc[1][34];
  frc[2][34] = s->frc[2][34];
  frc[3][34] = s->frc[3][34];
  trq[1][6] = s->trq[1][6];
  trq[2][6] = s->trq[2][6];
  trq[3][6] = s->trq[3][6];
  trq[1][7] = s->trq[1][7];
  trq[2][7] = s->trq[2][7];
  trq[3][7] = s->trq[3][7];
  trq[1][8] = s->trq[1][8];
  trq[2][8] = s->trq[2][8];
  trq[3][8] = s->trq[3][8];
  trq[1][9] = s->trq[1][9];
  trq[2][9] = s->trq[2][9];
  trq[3][9] = s->trq[3][9];
  trq[1][10] = s->trq[1][10];
  trq[2][10] = s->trq[2][10];
  trq[3][10] = s->trq[3][10];
  trq[1][11] = s->trq[1][11];
  trq[2][11] = s->trq[2][11];
  trq[3][11] = s->trq[3][11];
  trq[1][13] = s->trq[1][13];
  trq[2][13] = s->trq[2][13];
  trq[3][13] = s->trq[3][13];
  trq[1][14] = s->trq[1][14];
  trq[2][14] = s->trq[2][14];
  trq[3][14] = s->trq[3][14];
  trq[1][15] = s->trq[1][15];
  trq[2][15] = s->trq[2][15];
  trq[3][15] = s->trq[3][15];
  trq[1][16] = s->trq[1][16];
  trq[2][16] = s->trq[2][16];
  trq[3][16] = s->trq[3][16];
  trq[1][17] = s->trq[1][17];
  trq[2][17] = s->trq[2][17];
  trq[3][17] = s->trq[3][17];
  trq[1][19] = s->trq[1][19];
  trq[2][19] = s->trq[2][19];
  trq[3][19] = s->trq[3][19];
  trq[1][20] = s->trq[1][20];
  trq[2][20] = s->trq[2][20];
  trq[3][20] = s->trq[3][20];
  trq[1][22] = s->trq[1][22];
  trq[2][22] = s->trq[2][22];
  trq[3][22] = s->trq[3][22];
  trq[1][23] = s->trq[1][23];
  trq[2][23] = s->trq[2][23];
  trq[3][23] = s->trq[3][23];
  trq[1][24] = s->trq[1][24];
  trq[2][24] = s->trq[2][24];
  trq[3][24] = s->trq[3][24];
  trq[1][25] = s->trq[1][25];
  trq[2][25] = s->trq[2][25];
  trq[3][25] = s->trq[3][25];
  trq[1][26] = s->trq[1][26];
  trq[2][26] = s->trq[2][26];
  trq[3][26] = s->trq[3][26];
  trq[1][27] = s->trq[1][27];
  trq[2][27] = s->trq[2][27];
  trq[3][27] = s->trq[3][27];
  trq[1][29] = s->trq[1][29];
  trq[2][29] = s->trq[2][29];
  trq[3][29] = s->trq[3][29];
  trq[1][30] = s->trq[1][30];
  trq[2][30] = s->trq[2][30];
  trq[3][30] = s->trq[3][30];
  trq[1][31] = s->trq[1][31];
  trq[2][31] = s->trq[2][31];
  trq[3][31] = s->trq[3][31];
  trq[1][32] = s->trq[1][32];
  trq[2][32] = s->trq[2][32];
  trq[3][32] = s->trq[3][32];
  trq[1][33] = s->trq[1][33];
  trq[2][33] = s->trq[2][33];
  trq[3][33] = s->trq[3][33];
  trq[1][34] = s->trq[1][34];
  trq[2][34] = s->trq[2][34];
  trq[3][34] = s->trq[3][34];

// ====== END Task 0 ====== 


}
 

