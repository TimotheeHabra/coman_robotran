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
//	==> Generation Date : Wed Mar  5 10:45:11 2014
//
//	==> Project name : coman_robotran
//	==> using XML input file 
//
//	==> Number of joints : 31
//
//	==> Function : F19 : External Forces
//	==> Flops complexity : 3016
//
//	==> Generation Time :  0.060 seconds
//	==> Post-Processing :  0.060 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "MBSdataStructR7.h"
#include "MBSfunR7.h"
 
void extforces(double **frc,double **trq,
MBSdataStruct *s, double tsim)

// double frc[3][31];
// double trq[3][31];
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
  C13 = cos(q[13]);
  S13 = sin(q[13]);

// = = Block_0_0_0_0_0_3 = = 
 
// Trigonometric Variables  

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
  C19 = cos(q[19]);
  S19 = sin(q[19]);
  C20 = cos(q[20]);
  S20 = sin(q[20]);

// = = Block_0_0_0_0_0_4 = = 
 
// Trigonometric Variables  

  C21 = cos(q[21]);
  S21 = sin(q[21]);
  C22 = cos(q[22]);
  S22 = sin(q[22]);
  C23 = cos(q[23]);
  S23 = sin(q[23]);

// = = Block_0_0_1_1_0_1 = = 
 
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

// = = Block_0_0_1_1_0_2 = = 
 
// Sensor Kinematics 


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
  RLcp54_17 = ROcp54_46*s->dpt[2][1];
  RLcp54_27 = ROcp54_56*s->dpt[2][1];
  RLcp54_37 = ROcp54_66*s->dpt[2][1];
  OMcp54_17 = OMcp54_16+qd[7]*ROcp54_46;
  OMcp54_27 = OMcp54_26+qd[7]*ROcp54_56;
  OMcp54_37 = OMcp54_36+qd[7]*ROcp54_66;
  ORcp54_17 = OMcp54_26*RLcp54_37-OMcp54_36*RLcp54_27;
  ORcp54_27 = -(OMcp54_16*RLcp54_37-OMcp54_36*RLcp54_17);
  ORcp54_37 = OMcp54_16*RLcp54_27-OMcp54_26*RLcp54_17;
  OPcp54_17 = OPcp54_16+qd[7]*(OMcp54_26*ROcp54_66-OMcp54_36*ROcp54_56)+qdd[7]*ROcp54_46;
  OPcp54_27 = OPcp54_26-qd[7]*(OMcp54_16*ROcp54_66-OMcp54_36*ROcp54_46)+qdd[7]*ROcp54_56;
  OPcp54_37 = OPcp54_36+qd[7]*(OMcp54_16*ROcp54_56-OMcp54_26*ROcp54_46)+qdd[7]*ROcp54_66;
  RLcp54_18 = ROcp54_46*s->dpt[2][6];
  RLcp54_28 = ROcp54_56*s->dpt[2][6];
  RLcp54_38 = ROcp54_66*s->dpt[2][6];
  OMcp54_18 = OMcp54_17+qd[8]*ROcp54_17;
  OMcp54_28 = OMcp54_27+qd[8]*ROcp54_27;
  OMcp54_38 = OMcp54_37+qd[8]*ROcp54_37;
  ORcp54_18 = OMcp54_27*RLcp54_38-OMcp54_37*RLcp54_28;
  ORcp54_28 = -(OMcp54_17*RLcp54_38-OMcp54_37*RLcp54_18);
  ORcp54_38 = OMcp54_17*RLcp54_28-OMcp54_27*RLcp54_18;
  OPcp54_18 = OPcp54_17+qd[8]*(OMcp54_27*ROcp54_37-OMcp54_37*ROcp54_27)+qdd[8]*ROcp54_17;
  OPcp54_28 = OPcp54_27-qd[8]*(OMcp54_17*ROcp54_37-OMcp54_37*ROcp54_17)+qdd[8]*ROcp54_27;
  OPcp54_38 = OPcp54_37+qd[8]*(OMcp54_17*ROcp54_27-OMcp54_27*ROcp54_17)+qdd[8]*ROcp54_37;
  RLcp54_19 = ROcp54_78*s->dpt[3][8];
  RLcp54_29 = ROcp54_88*s->dpt[3][8];
  RLcp54_39 = ROcp54_98*s->dpt[3][8];
  OMcp54_19 = OMcp54_18+qd[9]*ROcp54_78;
  OMcp54_29 = OMcp54_28+qd[9]*ROcp54_88;
  OMcp54_39 = OMcp54_38+qd[9]*ROcp54_98;
  ORcp54_19 = OMcp54_28*RLcp54_39-OMcp54_38*RLcp54_29;
  ORcp54_29 = -(OMcp54_18*RLcp54_39-OMcp54_38*RLcp54_19);
  ORcp54_39 = OMcp54_18*RLcp54_29-OMcp54_28*RLcp54_19;
  OPcp54_19 = OPcp54_18+qd[9]*(OMcp54_28*ROcp54_98-OMcp54_38*ROcp54_88)+qdd[9]*ROcp54_78;
  OPcp54_29 = OPcp54_28-qd[9]*(OMcp54_18*ROcp54_98-OMcp54_38*ROcp54_78)+qdd[9]*ROcp54_88;
  OPcp54_39 = OPcp54_38+qd[9]*(OMcp54_18*ROcp54_88-OMcp54_28*ROcp54_78)+qdd[9]*ROcp54_98;
  RLcp54_110 = ROcp54_78*s->dpt[3][10];
  RLcp54_210 = ROcp54_88*s->dpt[3][10];
  RLcp54_310 = ROcp54_98*s->dpt[3][10];
  OMcp54_110 = OMcp54_19+qd[10]*ROcp54_49;
  OMcp54_210 = OMcp54_29+qd[10]*ROcp54_59;
  OMcp54_310 = OMcp54_39+qd[10]*ROcp54_69;
  ORcp54_110 = OMcp54_29*RLcp54_310-OMcp54_39*RLcp54_210;
  ORcp54_210 = -(OMcp54_19*RLcp54_310-OMcp54_39*RLcp54_110);
  ORcp54_310 = OMcp54_19*RLcp54_210-OMcp54_29*RLcp54_110;
  OPcp54_110 = OPcp54_19+qd[10]*(OMcp54_29*ROcp54_69-OMcp54_39*ROcp54_59)+qdd[10]*ROcp54_49;
  OPcp54_210 = OPcp54_29-qd[10]*(OMcp54_19*ROcp54_69-OMcp54_39*ROcp54_49)+qdd[10]*ROcp54_59;
  OPcp54_310 = OPcp54_39+qd[10]*(OMcp54_19*ROcp54_59-OMcp54_29*ROcp54_49)+qdd[10]*ROcp54_69;
  RLcp54_111 = ROcp54_710*s->dpt[3][12];
  RLcp54_211 = ROcp54_810*s->dpt[3][12];
  RLcp54_311 = ROcp54_910*s->dpt[3][12];
  OMcp54_111 = OMcp54_110+qd[11]*ROcp54_110;
  OMcp54_211 = OMcp54_210+qd[11]*ROcp54_210;
  OMcp54_311 = OMcp54_310+qd[11]*ROcp54_310;
  ORcp54_111 = OMcp54_210*RLcp54_311-OMcp54_310*RLcp54_211;
  ORcp54_211 = -(OMcp54_110*RLcp54_311-OMcp54_310*RLcp54_111);
  ORcp54_311 = OMcp54_110*RLcp54_211-OMcp54_210*RLcp54_111;
  OMcp54_112 = OMcp54_111+qd[12]*ROcp54_411;
  OMcp54_212 = OMcp54_211+qd[12]*ROcp54_511;
  OMcp54_312 = OMcp54_311+qd[12]*ROcp54_611;
  OPcp54_112 = OPcp54_110+qd[11]*(OMcp54_210*ROcp54_310-OMcp54_310*ROcp54_210)+qd[12]*(OMcp54_211*ROcp54_611-OMcp54_311*
 ROcp54_511)+qdd[11]*ROcp54_110+qdd[12]*ROcp54_411;
  OPcp54_212 = OPcp54_210-qd[11]*(OMcp54_110*ROcp54_310-OMcp54_310*ROcp54_110)-qd[12]*(OMcp54_111*ROcp54_611-OMcp54_311*
 ROcp54_411)+qdd[11]*ROcp54_210+qdd[12]*ROcp54_511;
  OPcp54_312 = OPcp54_310+qd[11]*(OMcp54_110*ROcp54_210-OMcp54_210*ROcp54_110)+qd[12]*(OMcp54_111*ROcp54_511-OMcp54_211*
 ROcp54_411)+qdd[11]*ROcp54_310+qdd[12]*ROcp54_611;
  RLcp54_186 = ROcp54_712*s->dpt[3][16];
  RLcp54_286 = ROcp54_812*s->dpt[3][16];
  RLcp54_386 = ROcp54_912*s->dpt[3][16];
  ORcp54_186 = OMcp54_212*RLcp54_386-OMcp54_312*RLcp54_286;
  ORcp54_286 = -(OMcp54_112*RLcp54_386-OMcp54_312*RLcp54_186);
  ORcp54_386 = OMcp54_112*RLcp54_286-OMcp54_212*RLcp54_186;
  PxF1[1] = q[1]+RLcp54_110+RLcp54_111+RLcp54_17+RLcp54_18+RLcp54_186+RLcp54_19;
  PxF1[2] = q[2]+RLcp54_210+RLcp54_211+RLcp54_27+RLcp54_28+RLcp54_286+RLcp54_29;
  PxF1[3] = q[3]+RLcp54_310+RLcp54_311+RLcp54_37+RLcp54_38+RLcp54_386+RLcp54_39;
  RxF1[1][1] = ROcp54_112;
  RxF1[1][2] = ROcp54_212;
  RxF1[1][3] = ROcp54_312;
  RxF1[2][1] = ROcp54_411;
  RxF1[2][2] = ROcp54_511;
  RxF1[2][3] = ROcp54_611;
  RxF1[3][1] = ROcp54_712;
  RxF1[3][2] = ROcp54_812;
  RxF1[3][3] = ROcp54_912;
  VxF1[1] = qd[1]+ORcp54_110+ORcp54_111+ORcp54_17+ORcp54_18+ORcp54_186+ORcp54_19;
  VxF1[2] = qd[2]+ORcp54_210+ORcp54_211+ORcp54_27+ORcp54_28+ORcp54_286+ORcp54_29;
  VxF1[3] = qd[3]+ORcp54_310+ORcp54_311+ORcp54_37+ORcp54_38+ORcp54_386+ORcp54_39;
  OMxF1[1] = OMcp54_112;
  OMxF1[2] = OMcp54_212;
  OMxF1[3] = OMcp54_312;
  AxF1[1] = qdd[1]+OMcp54_210*ORcp54_311+OMcp54_212*ORcp54_386+OMcp54_26*ORcp54_37+OMcp54_27*ORcp54_38+OMcp54_28*
 ORcp54_39+OMcp54_29*ORcp54_310-OMcp54_310*ORcp54_211-OMcp54_312*ORcp54_286-OMcp54_36*ORcp54_27-OMcp54_37*ORcp54_28-OMcp54_38
 *ORcp54_29-OMcp54_39*ORcp54_210+OPcp54_210*RLcp54_311+OPcp54_212*RLcp54_386+OPcp54_26*RLcp54_37+OPcp54_27*RLcp54_38+
 OPcp54_28*RLcp54_39+OPcp54_29*RLcp54_310-OPcp54_310*RLcp54_211-OPcp54_312*RLcp54_286-OPcp54_36*RLcp54_27-OPcp54_37*RLcp54_28
 -OPcp54_38*RLcp54_29-OPcp54_39*RLcp54_210;
  AxF1[2] = qdd[2]-OMcp54_110*ORcp54_311-OMcp54_112*ORcp54_386-OMcp54_16*ORcp54_37-OMcp54_17*ORcp54_38-OMcp54_18*
 ORcp54_39-OMcp54_19*ORcp54_310+OMcp54_310*ORcp54_111+OMcp54_312*ORcp54_186+OMcp54_36*ORcp54_17+OMcp54_37*ORcp54_18+OMcp54_38
 *ORcp54_19+OMcp54_39*ORcp54_110-OPcp54_110*RLcp54_311-OPcp54_112*RLcp54_386-OPcp54_16*RLcp54_37-OPcp54_17*RLcp54_38-
 OPcp54_18*RLcp54_39-OPcp54_19*RLcp54_310+OPcp54_310*RLcp54_111+OPcp54_312*RLcp54_186+OPcp54_36*RLcp54_17+OPcp54_37*RLcp54_18
 +OPcp54_38*RLcp54_19+OPcp54_39*RLcp54_110;
  AxF1[3] = qdd[3]+OMcp54_110*ORcp54_211+OMcp54_112*ORcp54_286+OMcp54_16*ORcp54_27+OMcp54_17*ORcp54_28+OMcp54_18*
 ORcp54_29+OMcp54_19*ORcp54_210-OMcp54_210*ORcp54_111-OMcp54_212*ORcp54_186-OMcp54_26*ORcp54_17-OMcp54_27*ORcp54_18-OMcp54_28
 *ORcp54_19-OMcp54_29*ORcp54_110+OPcp54_110*RLcp54_211+OPcp54_112*RLcp54_286+OPcp54_16*RLcp54_27+OPcp54_17*RLcp54_28+
 OPcp54_18*RLcp54_29+OPcp54_19*RLcp54_210-OPcp54_210*RLcp54_111-OPcp54_212*RLcp54_186-OPcp54_26*RLcp54_17-OPcp54_27*RLcp54_18
 -OPcp54_28*RLcp54_19-OPcp54_29*RLcp54_110;
  OMPxF1[1] = OPcp54_112;
  OMPxF1[2] = OPcp54_212;
  OMPxF1[3] = OPcp54_312;
 
// Sensor Forces Computation 

  SWr1 = user_ExtForces(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc155 = ROcp54_112*SWr1[1]+ROcp54_212*SWr1[2]+ROcp54_312*SWr1[3];
  xfrc255 = ROcp54_411*SWr1[1]+ROcp54_511*SWr1[2]+ROcp54_611*SWr1[3];
  xfrc355 = ROcp54_712*SWr1[1]+ROcp54_812*SWr1[2]+ROcp54_912*SWr1[3];
  frc[1][12] = s->frc[1][12]+xfrc155;
  frc[2][12] = s->frc[2][12]+xfrc255;
  frc[3][12] = s->frc[3][12]+xfrc355;
  xtrq155 = ROcp54_112*SWr1[4]+ROcp54_212*SWr1[5]+ROcp54_312*SWr1[6];
  xtrq255 = ROcp54_411*SWr1[4]+ROcp54_511*SWr1[5]+ROcp54_611*SWr1[6];
  xtrq355 = ROcp54_712*SWr1[4]+ROcp54_812*SWr1[5]+ROcp54_912*SWr1[6];
  trq[1][12] = s->trq[1][12]+xtrq155-xfrc255*(SWr1[9]-s->l[3][12])+xfrc355*SWr1[8];
  trq[2][12] = s->trq[2][12]+xtrq255+xfrc155*(SWr1[9]-s->l[3][12])-xfrc355*(SWr1[7]-s->l[1][12]);
  trq[3][12] = s->trq[3][12]+xtrq355-xfrc155*SWr1[8]+xfrc255*(SWr1[7]-s->l[1][12]);

// = = Block_0_0_1_2_0_1 = = 
 
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

// = = Block_0_0_1_2_0_2 = = 
 
// Sensor Kinematics 


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
  RLcp55_17 = ROcp55_46*s->dpt[2][1];
  RLcp55_27 = ROcp55_56*s->dpt[2][1];
  RLcp55_37 = ROcp55_66*s->dpt[2][1];
  OMcp55_17 = OMcp55_16+qd[7]*ROcp55_46;
  OMcp55_27 = OMcp55_26+qd[7]*ROcp55_56;
  OMcp55_37 = OMcp55_36+qd[7]*ROcp55_66;
  ORcp55_17 = OMcp55_26*RLcp55_37-OMcp55_36*RLcp55_27;
  ORcp55_27 = -(OMcp55_16*RLcp55_37-OMcp55_36*RLcp55_17);
  ORcp55_37 = OMcp55_16*RLcp55_27-OMcp55_26*RLcp55_17;
  OPcp55_17 = OPcp55_16+qd[7]*(OMcp55_26*ROcp55_66-OMcp55_36*ROcp55_56)+qdd[7]*ROcp55_46;
  OPcp55_27 = OPcp55_26-qd[7]*(OMcp55_16*ROcp55_66-OMcp55_36*ROcp55_46)+qdd[7]*ROcp55_56;
  OPcp55_37 = OPcp55_36+qd[7]*(OMcp55_16*ROcp55_56-OMcp55_26*ROcp55_46)+qdd[7]*ROcp55_66;
  RLcp55_18 = ROcp55_46*s->dpt[2][6];
  RLcp55_28 = ROcp55_56*s->dpt[2][6];
  RLcp55_38 = ROcp55_66*s->dpt[2][6];
  OMcp55_18 = OMcp55_17+qd[8]*ROcp55_17;
  OMcp55_28 = OMcp55_27+qd[8]*ROcp55_27;
  OMcp55_38 = OMcp55_37+qd[8]*ROcp55_37;
  ORcp55_18 = OMcp55_27*RLcp55_38-OMcp55_37*RLcp55_28;
  ORcp55_28 = -(OMcp55_17*RLcp55_38-OMcp55_37*RLcp55_18);
  ORcp55_38 = OMcp55_17*RLcp55_28-OMcp55_27*RLcp55_18;
  OPcp55_18 = OPcp55_17+qd[8]*(OMcp55_27*ROcp55_37-OMcp55_37*ROcp55_27)+qdd[8]*ROcp55_17;
  OPcp55_28 = OPcp55_27-qd[8]*(OMcp55_17*ROcp55_37-OMcp55_37*ROcp55_17)+qdd[8]*ROcp55_27;
  OPcp55_38 = OPcp55_37+qd[8]*(OMcp55_17*ROcp55_27-OMcp55_27*ROcp55_17)+qdd[8]*ROcp55_37;
  RLcp55_19 = ROcp55_78*s->dpt[3][8];
  RLcp55_29 = ROcp55_88*s->dpt[3][8];
  RLcp55_39 = ROcp55_98*s->dpt[3][8];
  OMcp55_19 = OMcp55_18+qd[9]*ROcp55_78;
  OMcp55_29 = OMcp55_28+qd[9]*ROcp55_88;
  OMcp55_39 = OMcp55_38+qd[9]*ROcp55_98;
  ORcp55_19 = OMcp55_28*RLcp55_39-OMcp55_38*RLcp55_29;
  ORcp55_29 = -(OMcp55_18*RLcp55_39-OMcp55_38*RLcp55_19);
  ORcp55_39 = OMcp55_18*RLcp55_29-OMcp55_28*RLcp55_19;
  OPcp55_19 = OPcp55_18+qd[9]*(OMcp55_28*ROcp55_98-OMcp55_38*ROcp55_88)+qdd[9]*ROcp55_78;
  OPcp55_29 = OPcp55_28-qd[9]*(OMcp55_18*ROcp55_98-OMcp55_38*ROcp55_78)+qdd[9]*ROcp55_88;
  OPcp55_39 = OPcp55_38+qd[9]*(OMcp55_18*ROcp55_88-OMcp55_28*ROcp55_78)+qdd[9]*ROcp55_98;
  RLcp55_110 = ROcp55_78*s->dpt[3][10];
  RLcp55_210 = ROcp55_88*s->dpt[3][10];
  RLcp55_310 = ROcp55_98*s->dpt[3][10];
  OMcp55_110 = OMcp55_19+qd[10]*ROcp55_49;
  OMcp55_210 = OMcp55_29+qd[10]*ROcp55_59;
  OMcp55_310 = OMcp55_39+qd[10]*ROcp55_69;
  ORcp55_110 = OMcp55_29*RLcp55_310-OMcp55_39*RLcp55_210;
  ORcp55_210 = -(OMcp55_19*RLcp55_310-OMcp55_39*RLcp55_110);
  ORcp55_310 = OMcp55_19*RLcp55_210-OMcp55_29*RLcp55_110;
  OPcp55_110 = OPcp55_19+qd[10]*(OMcp55_29*ROcp55_69-OMcp55_39*ROcp55_59)+qdd[10]*ROcp55_49;
  OPcp55_210 = OPcp55_29-qd[10]*(OMcp55_19*ROcp55_69-OMcp55_39*ROcp55_49)+qdd[10]*ROcp55_59;
  OPcp55_310 = OPcp55_39+qd[10]*(OMcp55_19*ROcp55_59-OMcp55_29*ROcp55_49)+qdd[10]*ROcp55_69;
  RLcp55_111 = ROcp55_710*s->dpt[3][12];
  RLcp55_211 = ROcp55_810*s->dpt[3][12];
  RLcp55_311 = ROcp55_910*s->dpt[3][12];
  OMcp55_111 = OMcp55_110+qd[11]*ROcp55_110;
  OMcp55_211 = OMcp55_210+qd[11]*ROcp55_210;
  OMcp55_311 = OMcp55_310+qd[11]*ROcp55_310;
  ORcp55_111 = OMcp55_210*RLcp55_311-OMcp55_310*RLcp55_211;
  ORcp55_211 = -(OMcp55_110*RLcp55_311-OMcp55_310*RLcp55_111);
  ORcp55_311 = OMcp55_110*RLcp55_211-OMcp55_210*RLcp55_111;
  OMcp55_112 = OMcp55_111+qd[12]*ROcp55_411;
  OMcp55_212 = OMcp55_211+qd[12]*ROcp55_511;
  OMcp55_312 = OMcp55_311+qd[12]*ROcp55_611;
  OPcp55_112 = OPcp55_110+qd[11]*(OMcp55_210*ROcp55_310-OMcp55_310*ROcp55_210)+qd[12]*(OMcp55_211*ROcp55_611-OMcp55_311*
 ROcp55_511)+qdd[11]*ROcp55_110+qdd[12]*ROcp55_411;
  OPcp55_212 = OPcp55_210-qd[11]*(OMcp55_110*ROcp55_310-OMcp55_310*ROcp55_110)-qd[12]*(OMcp55_111*ROcp55_611-OMcp55_311*
 ROcp55_411)+qdd[11]*ROcp55_210+qdd[12]*ROcp55_511;
  OPcp55_312 = OPcp55_310+qd[11]*(OMcp55_110*ROcp55_210-OMcp55_210*ROcp55_110)+qd[12]*(OMcp55_111*ROcp55_511-OMcp55_211*
 ROcp55_411)+qdd[11]*ROcp55_310+qdd[12]*ROcp55_611;
  RLcp55_113 = ROcp55_112*s->dpt[1][18]+ROcp55_712*s->dpt[3][18];
  RLcp55_213 = ROcp55_212*s->dpt[1][18]+ROcp55_812*s->dpt[3][18];
  RLcp55_313 = ROcp55_312*s->dpt[1][18]+ROcp55_912*s->dpt[3][18];
  ORcp55_113 = OMcp55_212*RLcp55_313-OMcp55_312*RLcp55_213;
  ORcp55_213 = -(OMcp55_112*RLcp55_313-OMcp55_312*RLcp55_113);
  ORcp55_313 = OMcp55_112*RLcp55_213-OMcp55_212*RLcp55_113;
  PxF2[1] = q[1]+RLcp55_110+RLcp55_111+RLcp55_113+RLcp55_17+RLcp55_18+RLcp55_19;
  PxF2[2] = q[2]+RLcp55_210+RLcp55_211+RLcp55_213+RLcp55_27+RLcp55_28+RLcp55_29;
  PxF2[3] = q[3]+RLcp55_310+RLcp55_311+RLcp55_313+RLcp55_37+RLcp55_38+RLcp55_39;
  RxF2[1][1] = ROcp55_113;
  RxF2[1][2] = ROcp55_213;
  RxF2[1][3] = ROcp55_313;
  RxF2[2][1] = ROcp55_411;
  RxF2[2][2] = ROcp55_511;
  RxF2[2][3] = ROcp55_611;
  RxF2[3][1] = ROcp55_713;
  RxF2[3][2] = ROcp55_813;
  RxF2[3][3] = ROcp55_913;
  VxF2[1] = qd[1]+ORcp55_110+ORcp55_111+ORcp55_113+ORcp55_17+ORcp55_18+ORcp55_19;
  VxF2[2] = qd[2]+ORcp55_210+ORcp55_211+ORcp55_213+ORcp55_27+ORcp55_28+ORcp55_29;
  VxF2[3] = qd[3]+ORcp55_310+ORcp55_311+ORcp55_313+ORcp55_37+ORcp55_38+ORcp55_39;
  OMxF2[1] = OMcp55_112+qd[13]*ROcp55_411;
  OMxF2[2] = OMcp55_212+qd[13]*ROcp55_511;
  OMxF2[3] = OMcp55_312+qd[13]*ROcp55_611;
  AxF2[1] = qdd[1]+OMcp55_210*ORcp55_311+OMcp55_212*ORcp55_313+OMcp55_26*ORcp55_37+OMcp55_27*ORcp55_38+OMcp55_28*
 ORcp55_39+OMcp55_29*ORcp55_310-OMcp55_310*ORcp55_211-OMcp55_312*ORcp55_213-OMcp55_36*ORcp55_27-OMcp55_37*ORcp55_28-OMcp55_38
 *ORcp55_29-OMcp55_39*ORcp55_210+OPcp55_210*RLcp55_311+OPcp55_212*RLcp55_313+OPcp55_26*RLcp55_37+OPcp55_27*RLcp55_38+
 OPcp55_28*RLcp55_39+OPcp55_29*RLcp55_310-OPcp55_310*RLcp55_211-OPcp55_312*RLcp55_213-OPcp55_36*RLcp55_27-OPcp55_37*RLcp55_28
 -OPcp55_38*RLcp55_29-OPcp55_39*RLcp55_210;
  AxF2[2] = qdd[2]-OMcp55_110*ORcp55_311-OMcp55_112*ORcp55_313-OMcp55_16*ORcp55_37-OMcp55_17*ORcp55_38-OMcp55_18*
 ORcp55_39-OMcp55_19*ORcp55_310+OMcp55_310*ORcp55_111+OMcp55_312*ORcp55_113+OMcp55_36*ORcp55_17+OMcp55_37*ORcp55_18+OMcp55_38
 *ORcp55_19+OMcp55_39*ORcp55_110-OPcp55_110*RLcp55_311-OPcp55_112*RLcp55_313-OPcp55_16*RLcp55_37-OPcp55_17*RLcp55_38-
 OPcp55_18*RLcp55_39-OPcp55_19*RLcp55_310+OPcp55_310*RLcp55_111+OPcp55_312*RLcp55_113+OPcp55_36*RLcp55_17+OPcp55_37*RLcp55_18
 +OPcp55_38*RLcp55_19+OPcp55_39*RLcp55_110;
  AxF2[3] = qdd[3]+OMcp55_110*ORcp55_211+OMcp55_112*ORcp55_213+OMcp55_16*ORcp55_27+OMcp55_17*ORcp55_28+OMcp55_18*
 ORcp55_29+OMcp55_19*ORcp55_210-OMcp55_210*ORcp55_111-OMcp55_212*ORcp55_113-OMcp55_26*ORcp55_17-OMcp55_27*ORcp55_18-OMcp55_28
 *ORcp55_19-OMcp55_29*ORcp55_110+OPcp55_110*RLcp55_211+OPcp55_112*RLcp55_213+OPcp55_16*RLcp55_27+OPcp55_17*RLcp55_28+
 OPcp55_18*RLcp55_29+OPcp55_19*RLcp55_210-OPcp55_210*RLcp55_111-OPcp55_212*RLcp55_113-OPcp55_26*RLcp55_17-OPcp55_27*RLcp55_18
 -OPcp55_28*RLcp55_19-OPcp55_29*RLcp55_110;
  OMPxF2[1] = OPcp55_112+qd[13]*(OMcp55_212*ROcp55_611-OMcp55_312*ROcp55_511)+qdd[13]*ROcp55_411;
  OMPxF2[2] = OPcp55_212-qd[13]*(OMcp55_112*ROcp55_611-OMcp55_312*ROcp55_411)+qdd[13]*ROcp55_511;
  OMPxF2[3] = OPcp55_312+qd[13]*(OMcp55_112*ROcp55_511-OMcp55_212*ROcp55_411)+qdd[13]*ROcp55_611;
 
// Sensor Forces Computation 

  SWr2 = user_ExtForces(PxF2,RxF2,VxF2,OMxF2,AxF2,OMPxF2,s,tsim,2);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc156 = ROcp55_113*SWr2[1]+ROcp55_213*SWr2[2]+ROcp55_313*SWr2[3];
  xfrc256 = ROcp55_411*SWr2[1]+ROcp55_511*SWr2[2]+ROcp55_611*SWr2[3];
  xfrc356 = ROcp55_713*SWr2[1]+ROcp55_813*SWr2[2]+ROcp55_913*SWr2[3];
  frc[1][13] = s->frc[1][13]+xfrc156;
  frc[2][13] = s->frc[2][13]+xfrc256;
  frc[3][13] = s->frc[3][13]+xfrc356;
  xtrq156 = ROcp55_113*SWr2[4]+ROcp55_213*SWr2[5]+ROcp55_313*SWr2[6];
  xtrq256 = ROcp55_411*SWr2[4]+ROcp55_511*SWr2[5]+ROcp55_611*SWr2[6];
  xtrq356 = ROcp55_713*SWr2[4]+ROcp55_813*SWr2[5]+ROcp55_913*SWr2[6];
  trq[1][13] = s->trq[1][13]+xtrq156-xfrc256*(SWr2[9]-s->l[3][13])+xfrc356*SWr2[8];
  trq[2][13] = s->trq[2][13]+xtrq256+xfrc156*(SWr2[9]-s->l[3][13])-xfrc356*(SWr2[7]-s->l[1][13]);
  trq[3][13] = s->trq[3][13]+xtrq356-xfrc156*SWr2[8]+xfrc256*(SWr2[7]-s->l[1][13]);

// = = Block_0_0_1_3_0_1 = = 
 
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

// = = Block_0_0_1_3_0_3 = = 
 
// Sensor Kinematics 


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
  RLcp56_114 = ROcp56_46*s->dpt[2][2];
  RLcp56_214 = ROcp56_56*s->dpt[2][2];
  RLcp56_314 = ROcp56_66*s->dpt[2][2];
  OMcp56_114 = OMcp56_16+qd[14]*ROcp56_46;
  OMcp56_214 = OMcp56_26+qd[14]*ROcp56_56;
  OMcp56_314 = OMcp56_36+qd[14]*ROcp56_66;
  ORcp56_114 = OMcp56_26*RLcp56_314-OMcp56_36*RLcp56_214;
  ORcp56_214 = -(OMcp56_16*RLcp56_314-OMcp56_36*RLcp56_114);
  ORcp56_314 = OMcp56_16*RLcp56_214-OMcp56_26*RLcp56_114;
  OPcp56_114 = OPcp56_16+qd[14]*(OMcp56_26*ROcp56_66-OMcp56_36*ROcp56_56)+qdd[14]*ROcp56_46;
  OPcp56_214 = OPcp56_26-qd[14]*(OMcp56_16*ROcp56_66-OMcp56_36*ROcp56_46)+qdd[14]*ROcp56_56;
  OPcp56_314 = OPcp56_36+qd[14]*(OMcp56_16*ROcp56_56-OMcp56_26*ROcp56_46)+qdd[14]*ROcp56_66;
  RLcp56_115 = ROcp56_46*s->dpt[2][20];
  RLcp56_215 = ROcp56_56*s->dpt[2][20];
  RLcp56_315 = ROcp56_66*s->dpt[2][20];
  OMcp56_115 = OMcp56_114+qd[15]*ROcp56_114;
  OMcp56_215 = OMcp56_214+qd[15]*ROcp56_214;
  OMcp56_315 = OMcp56_314+qd[15]*ROcp56_314;
  ORcp56_115 = OMcp56_214*RLcp56_315-OMcp56_314*RLcp56_215;
  ORcp56_215 = -(OMcp56_114*RLcp56_315-OMcp56_314*RLcp56_115);
  ORcp56_315 = OMcp56_114*RLcp56_215-OMcp56_214*RLcp56_115;
  OPcp56_115 = OPcp56_114+qd[15]*(OMcp56_214*ROcp56_314-OMcp56_314*ROcp56_214)+qdd[15]*ROcp56_114;
  OPcp56_215 = OPcp56_214-qd[15]*(OMcp56_114*ROcp56_314-OMcp56_314*ROcp56_114)+qdd[15]*ROcp56_214;
  OPcp56_315 = OPcp56_314+qd[15]*(OMcp56_114*ROcp56_214-OMcp56_214*ROcp56_114)+qdd[15]*ROcp56_314;
  RLcp56_116 = ROcp56_715*s->dpt[3][22];
  RLcp56_216 = ROcp56_815*s->dpt[3][22];
  RLcp56_316 = ROcp56_915*s->dpt[3][22];
  OMcp56_116 = OMcp56_115+qd[16]*ROcp56_715;
  OMcp56_216 = OMcp56_215+qd[16]*ROcp56_815;
  OMcp56_316 = OMcp56_315+qd[16]*ROcp56_915;
  ORcp56_116 = OMcp56_215*RLcp56_316-OMcp56_315*RLcp56_216;
  ORcp56_216 = -(OMcp56_115*RLcp56_316-OMcp56_315*RLcp56_116);
  ORcp56_316 = OMcp56_115*RLcp56_216-OMcp56_215*RLcp56_116;
  OPcp56_116 = OPcp56_115+qd[16]*(OMcp56_215*ROcp56_915-OMcp56_315*ROcp56_815)+qdd[16]*ROcp56_715;
  OPcp56_216 = OPcp56_215-qd[16]*(OMcp56_115*ROcp56_915-OMcp56_315*ROcp56_715)+qdd[16]*ROcp56_815;
  OPcp56_316 = OPcp56_315+qd[16]*(OMcp56_115*ROcp56_815-OMcp56_215*ROcp56_715)+qdd[16]*ROcp56_915;
  RLcp56_117 = ROcp56_715*s->dpt[3][24];
  RLcp56_217 = ROcp56_815*s->dpt[3][24];
  RLcp56_317 = ROcp56_915*s->dpt[3][24];
  OMcp56_117 = OMcp56_116+qd[17]*ROcp56_416;
  OMcp56_217 = OMcp56_216+qd[17]*ROcp56_516;
  OMcp56_317 = OMcp56_316+qd[17]*ROcp56_616;
  ORcp56_117 = OMcp56_216*RLcp56_317-OMcp56_316*RLcp56_217;
  ORcp56_217 = -(OMcp56_116*RLcp56_317-OMcp56_316*RLcp56_117);
  ORcp56_317 = OMcp56_116*RLcp56_217-OMcp56_216*RLcp56_117;
  OPcp56_117 = OPcp56_116+qd[17]*(OMcp56_216*ROcp56_616-OMcp56_316*ROcp56_516)+qdd[17]*ROcp56_416;
  OPcp56_217 = OPcp56_216-qd[17]*(OMcp56_116*ROcp56_616-OMcp56_316*ROcp56_416)+qdd[17]*ROcp56_516;
  OPcp56_317 = OPcp56_316+qd[17]*(OMcp56_116*ROcp56_516-OMcp56_216*ROcp56_416)+qdd[17]*ROcp56_616;
  RLcp56_118 = ROcp56_717*s->dpt[3][26];
  RLcp56_218 = ROcp56_817*s->dpt[3][26];
  RLcp56_318 = ROcp56_917*s->dpt[3][26];
  OMcp56_118 = OMcp56_117+qd[18]*ROcp56_117;
  OMcp56_218 = OMcp56_217+qd[18]*ROcp56_217;
  OMcp56_318 = OMcp56_317+qd[18]*ROcp56_317;
  ORcp56_118 = OMcp56_217*RLcp56_318-OMcp56_317*RLcp56_218;
  ORcp56_218 = -(OMcp56_117*RLcp56_318-OMcp56_317*RLcp56_118);
  ORcp56_318 = OMcp56_117*RLcp56_218-OMcp56_217*RLcp56_118;
  OMcp56_119 = OMcp56_118+qd[19]*ROcp56_418;
  OMcp56_219 = OMcp56_218+qd[19]*ROcp56_518;
  OMcp56_319 = OMcp56_318+qd[19]*ROcp56_618;
  OPcp56_119 = OPcp56_117+qd[18]*(OMcp56_217*ROcp56_317-OMcp56_317*ROcp56_217)+qd[19]*(OMcp56_218*ROcp56_618-OMcp56_318*
 ROcp56_518)+qdd[18]*ROcp56_117+qdd[19]*ROcp56_418;
  OPcp56_219 = OPcp56_217-qd[18]*(OMcp56_117*ROcp56_317-OMcp56_317*ROcp56_117)-qd[19]*(OMcp56_118*ROcp56_618-OMcp56_318*
 ROcp56_418)+qdd[18]*ROcp56_217+qdd[19]*ROcp56_518;
  OPcp56_319 = OPcp56_317+qd[18]*(OMcp56_117*ROcp56_217-OMcp56_217*ROcp56_117)+qd[19]*(OMcp56_118*ROcp56_518-OMcp56_218*
 ROcp56_418)+qdd[18]*ROcp56_317+qdd[19]*ROcp56_618;
  RLcp56_188 = ROcp56_719*s->dpt[3][31];
  RLcp56_288 = ROcp56_819*s->dpt[3][31];
  RLcp56_388 = ROcp56_919*s->dpt[3][31];
  ORcp56_188 = OMcp56_219*RLcp56_388-OMcp56_319*RLcp56_288;
  ORcp56_288 = -(OMcp56_119*RLcp56_388-OMcp56_319*RLcp56_188);
  ORcp56_388 = OMcp56_119*RLcp56_288-OMcp56_219*RLcp56_188;
  PxF3[1] = q[1]+RLcp56_114+RLcp56_115+RLcp56_116+RLcp56_117+RLcp56_118+RLcp56_188;
  PxF3[2] = q[2]+RLcp56_214+RLcp56_215+RLcp56_216+RLcp56_217+RLcp56_218+RLcp56_288;
  PxF3[3] = q[3]+RLcp56_314+RLcp56_315+RLcp56_316+RLcp56_317+RLcp56_318+RLcp56_388;
  RxF3[1][1] = ROcp56_119;
  RxF3[1][2] = ROcp56_219;
  RxF3[1][3] = ROcp56_319;
  RxF3[2][1] = ROcp56_418;
  RxF3[2][2] = ROcp56_518;
  RxF3[2][3] = ROcp56_618;
  RxF3[3][1] = ROcp56_719;
  RxF3[3][2] = ROcp56_819;
  RxF3[3][3] = ROcp56_919;
  VxF3[1] = qd[1]+ORcp56_114+ORcp56_115+ORcp56_116+ORcp56_117+ORcp56_118+ORcp56_188;
  VxF3[2] = qd[2]+ORcp56_214+ORcp56_215+ORcp56_216+ORcp56_217+ORcp56_218+ORcp56_288;
  VxF3[3] = qd[3]+ORcp56_314+ORcp56_315+ORcp56_316+ORcp56_317+ORcp56_318+ORcp56_388;
  OMxF3[1] = OMcp56_119;
  OMxF3[2] = OMcp56_219;
  OMxF3[3] = OMcp56_319;
  AxF3[1] = qdd[1]+OMcp56_214*ORcp56_315+OMcp56_215*ORcp56_316+OMcp56_216*ORcp56_317+OMcp56_217*ORcp56_318+OMcp56_219*
 ORcp56_388+OMcp56_26*ORcp56_314-OMcp56_314*ORcp56_215-OMcp56_315*ORcp56_216-OMcp56_316*ORcp56_217-OMcp56_317*ORcp56_218-
 OMcp56_319*ORcp56_288-OMcp56_36*ORcp56_214+OPcp56_214*RLcp56_315+OPcp56_215*RLcp56_316+OPcp56_216*RLcp56_317+OPcp56_217*
 RLcp56_318+OPcp56_219*RLcp56_388+OPcp56_26*RLcp56_314-OPcp56_314*RLcp56_215-OPcp56_315*RLcp56_216-OPcp56_316*RLcp56_217-
 OPcp56_317*RLcp56_218-OPcp56_319*RLcp56_288-OPcp56_36*RLcp56_214;
  AxF3[2] = qdd[2]-OMcp56_114*ORcp56_315-OMcp56_115*ORcp56_316-OMcp56_116*ORcp56_317-OMcp56_117*ORcp56_318-OMcp56_119*
 ORcp56_388-OMcp56_16*ORcp56_314+OMcp56_314*ORcp56_115+OMcp56_315*ORcp56_116+OMcp56_316*ORcp56_117+OMcp56_317*ORcp56_118+
 OMcp56_319*ORcp56_188+OMcp56_36*ORcp56_114-OPcp56_114*RLcp56_315-OPcp56_115*RLcp56_316-OPcp56_116*RLcp56_317-OPcp56_117*
 RLcp56_318-OPcp56_119*RLcp56_388-OPcp56_16*RLcp56_314+OPcp56_314*RLcp56_115+OPcp56_315*RLcp56_116+OPcp56_316*RLcp56_117+
 OPcp56_317*RLcp56_118+OPcp56_319*RLcp56_188+OPcp56_36*RLcp56_114;
  AxF3[3] = qdd[3]+OMcp56_114*ORcp56_215+OMcp56_115*ORcp56_216+OMcp56_116*ORcp56_217+OMcp56_117*ORcp56_218+OMcp56_119*
 ORcp56_288+OMcp56_16*ORcp56_214-OMcp56_214*ORcp56_115-OMcp56_215*ORcp56_116-OMcp56_216*ORcp56_117-OMcp56_217*ORcp56_118-
 OMcp56_219*ORcp56_188-OMcp56_26*ORcp56_114+OPcp56_114*RLcp56_215+OPcp56_115*RLcp56_216+OPcp56_116*RLcp56_217+OPcp56_117*
 RLcp56_218+OPcp56_119*RLcp56_288+OPcp56_16*RLcp56_214-OPcp56_214*RLcp56_115-OPcp56_215*RLcp56_116-OPcp56_216*RLcp56_117-
 OPcp56_217*RLcp56_118-OPcp56_219*RLcp56_188-OPcp56_26*RLcp56_114;
  OMPxF3[1] = OPcp56_119;
  OMPxF3[2] = OPcp56_219;
  OMPxF3[3] = OPcp56_319;
 
// Sensor Forces Computation 

  SWr3 = user_ExtForces(PxF3,RxF3,VxF3,OMxF3,AxF3,OMPxF3,s,tsim,3);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc157 = ROcp56_119*SWr3[1]+ROcp56_219*SWr3[2]+ROcp56_319*SWr3[3];
  xfrc257 = ROcp56_418*SWr3[1]+ROcp56_518*SWr3[2]+ROcp56_618*SWr3[3];
  xfrc357 = ROcp56_719*SWr3[1]+ROcp56_819*SWr3[2]+ROcp56_919*SWr3[3];
  frc[1][19] = s->frc[1][19]+xfrc157;
  frc[2][19] = s->frc[2][19]+xfrc257;
  frc[3][19] = s->frc[3][19]+xfrc357;
  xtrq157 = ROcp56_119*SWr3[4]+ROcp56_219*SWr3[5]+ROcp56_319*SWr3[6];
  xtrq257 = ROcp56_418*SWr3[4]+ROcp56_518*SWr3[5]+ROcp56_618*SWr3[6];
  xtrq357 = ROcp56_719*SWr3[4]+ROcp56_819*SWr3[5]+ROcp56_919*SWr3[6];
  trq[1][19] = s->trq[1][19]+xtrq157-xfrc257*(SWr3[9]-s->l[3][19])+xfrc357*SWr3[8];
  trq[2][19] = s->trq[2][19]+xtrq257+xfrc157*(SWr3[9]-s->l[3][19])-xfrc357*(SWr3[7]-s->l[1][19]);
  trq[3][19] = s->trq[3][19]+xtrq357-xfrc157*SWr3[8]+xfrc257*(SWr3[7]-s->l[1][19]);

// = = Block_0_0_1_4_0_1 = = 
 
// Sensor Kinematics 


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
  OMcp57_25 = qd[5]*C4;
  OMcp57_35 = qd[5]*S4;
  OMcp57_16 = qd[4]+qd[6]*S5;
  OMcp57_26 = OMcp57_25+qd[6]*ROcp57_85;
  OMcp57_36 = OMcp57_35+qd[6]*ROcp57_95;
  OPcp57_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp57_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp57_95-OMcp57_35*S5)-qdd[5]*C4-qdd[6]*ROcp57_85);
  OPcp57_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp57_85-OMcp57_25*S5)+qdd[5]*S4+qdd[6]*ROcp57_95;

// = = Block_0_0_1_4_0_3 = = 
 
// Sensor Kinematics 


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
  RLcp57_114 = ROcp57_46*s->dpt[2][2];
  RLcp57_214 = ROcp57_56*s->dpt[2][2];
  RLcp57_314 = ROcp57_66*s->dpt[2][2];
  OMcp57_114 = OMcp57_16+qd[14]*ROcp57_46;
  OMcp57_214 = OMcp57_26+qd[14]*ROcp57_56;
  OMcp57_314 = OMcp57_36+qd[14]*ROcp57_66;
  ORcp57_114 = OMcp57_26*RLcp57_314-OMcp57_36*RLcp57_214;
  ORcp57_214 = -(OMcp57_16*RLcp57_314-OMcp57_36*RLcp57_114);
  ORcp57_314 = OMcp57_16*RLcp57_214-OMcp57_26*RLcp57_114;
  OPcp57_114 = OPcp57_16+qd[14]*(OMcp57_26*ROcp57_66-OMcp57_36*ROcp57_56)+qdd[14]*ROcp57_46;
  OPcp57_214 = OPcp57_26-qd[14]*(OMcp57_16*ROcp57_66-OMcp57_36*ROcp57_46)+qdd[14]*ROcp57_56;
  OPcp57_314 = OPcp57_36+qd[14]*(OMcp57_16*ROcp57_56-OMcp57_26*ROcp57_46)+qdd[14]*ROcp57_66;
  RLcp57_115 = ROcp57_46*s->dpt[2][20];
  RLcp57_215 = ROcp57_56*s->dpt[2][20];
  RLcp57_315 = ROcp57_66*s->dpt[2][20];
  OMcp57_115 = OMcp57_114+qd[15]*ROcp57_114;
  OMcp57_215 = OMcp57_214+qd[15]*ROcp57_214;
  OMcp57_315 = OMcp57_314+qd[15]*ROcp57_314;
  ORcp57_115 = OMcp57_214*RLcp57_315-OMcp57_314*RLcp57_215;
  ORcp57_215 = -(OMcp57_114*RLcp57_315-OMcp57_314*RLcp57_115);
  ORcp57_315 = OMcp57_114*RLcp57_215-OMcp57_214*RLcp57_115;
  OPcp57_115 = OPcp57_114+qd[15]*(OMcp57_214*ROcp57_314-OMcp57_314*ROcp57_214)+qdd[15]*ROcp57_114;
  OPcp57_215 = OPcp57_214-qd[15]*(OMcp57_114*ROcp57_314-OMcp57_314*ROcp57_114)+qdd[15]*ROcp57_214;
  OPcp57_315 = OPcp57_314+qd[15]*(OMcp57_114*ROcp57_214-OMcp57_214*ROcp57_114)+qdd[15]*ROcp57_314;
  RLcp57_116 = ROcp57_715*s->dpt[3][22];
  RLcp57_216 = ROcp57_815*s->dpt[3][22];
  RLcp57_316 = ROcp57_915*s->dpt[3][22];
  OMcp57_116 = OMcp57_115+qd[16]*ROcp57_715;
  OMcp57_216 = OMcp57_215+qd[16]*ROcp57_815;
  OMcp57_316 = OMcp57_315+qd[16]*ROcp57_915;
  ORcp57_116 = OMcp57_215*RLcp57_316-OMcp57_315*RLcp57_216;
  ORcp57_216 = -(OMcp57_115*RLcp57_316-OMcp57_315*RLcp57_116);
  ORcp57_316 = OMcp57_115*RLcp57_216-OMcp57_215*RLcp57_116;
  OPcp57_116 = OPcp57_115+qd[16]*(OMcp57_215*ROcp57_915-OMcp57_315*ROcp57_815)+qdd[16]*ROcp57_715;
  OPcp57_216 = OPcp57_215-qd[16]*(OMcp57_115*ROcp57_915-OMcp57_315*ROcp57_715)+qdd[16]*ROcp57_815;
  OPcp57_316 = OPcp57_315+qd[16]*(OMcp57_115*ROcp57_815-OMcp57_215*ROcp57_715)+qdd[16]*ROcp57_915;
  RLcp57_117 = ROcp57_715*s->dpt[3][24];
  RLcp57_217 = ROcp57_815*s->dpt[3][24];
  RLcp57_317 = ROcp57_915*s->dpt[3][24];
  OMcp57_117 = OMcp57_116+qd[17]*ROcp57_416;
  OMcp57_217 = OMcp57_216+qd[17]*ROcp57_516;
  OMcp57_317 = OMcp57_316+qd[17]*ROcp57_616;
  ORcp57_117 = OMcp57_216*RLcp57_317-OMcp57_316*RLcp57_217;
  ORcp57_217 = -(OMcp57_116*RLcp57_317-OMcp57_316*RLcp57_117);
  ORcp57_317 = OMcp57_116*RLcp57_217-OMcp57_216*RLcp57_117;
  OPcp57_117 = OPcp57_116+qd[17]*(OMcp57_216*ROcp57_616-OMcp57_316*ROcp57_516)+qdd[17]*ROcp57_416;
  OPcp57_217 = OPcp57_216-qd[17]*(OMcp57_116*ROcp57_616-OMcp57_316*ROcp57_416)+qdd[17]*ROcp57_516;
  OPcp57_317 = OPcp57_316+qd[17]*(OMcp57_116*ROcp57_516-OMcp57_216*ROcp57_416)+qdd[17]*ROcp57_616;
  RLcp57_118 = ROcp57_717*s->dpt[3][26];
  RLcp57_218 = ROcp57_817*s->dpt[3][26];
  RLcp57_318 = ROcp57_917*s->dpt[3][26];
  OMcp57_118 = OMcp57_117+qd[18]*ROcp57_117;
  OMcp57_218 = OMcp57_217+qd[18]*ROcp57_217;
  OMcp57_318 = OMcp57_317+qd[18]*ROcp57_317;
  ORcp57_118 = OMcp57_217*RLcp57_318-OMcp57_317*RLcp57_218;
  ORcp57_218 = -(OMcp57_117*RLcp57_318-OMcp57_317*RLcp57_118);
  ORcp57_318 = OMcp57_117*RLcp57_218-OMcp57_217*RLcp57_118;
  OMcp57_119 = OMcp57_118+qd[19]*ROcp57_418;
  OMcp57_219 = OMcp57_218+qd[19]*ROcp57_518;
  OMcp57_319 = OMcp57_318+qd[19]*ROcp57_618;
  OPcp57_119 = OPcp57_117+qd[18]*(OMcp57_217*ROcp57_317-OMcp57_317*ROcp57_217)+qd[19]*(OMcp57_218*ROcp57_618-OMcp57_318*
 ROcp57_518)+qdd[18]*ROcp57_117+qdd[19]*ROcp57_418;
  OPcp57_219 = OPcp57_217-qd[18]*(OMcp57_117*ROcp57_317-OMcp57_317*ROcp57_117)-qd[19]*(OMcp57_118*ROcp57_618-OMcp57_318*
 ROcp57_418)+qdd[18]*ROcp57_217+qdd[19]*ROcp57_518;
  OPcp57_319 = OPcp57_317+qd[18]*(OMcp57_117*ROcp57_217-OMcp57_217*ROcp57_117)+qd[19]*(OMcp57_118*ROcp57_518-OMcp57_218*
 ROcp57_418)+qdd[18]*ROcp57_317+qdd[19]*ROcp57_618;
  RLcp57_120 = ROcp57_119*s->dpt[1][32]+ROcp57_719*s->dpt[3][32];
  RLcp57_220 = ROcp57_219*s->dpt[1][32]+ROcp57_819*s->dpt[3][32];
  RLcp57_320 = ROcp57_319*s->dpt[1][32]+ROcp57_919*s->dpt[3][32];
  ORcp57_120 = OMcp57_219*RLcp57_320-OMcp57_319*RLcp57_220;
  ORcp57_220 = -(OMcp57_119*RLcp57_320-OMcp57_319*RLcp57_120);
  ORcp57_320 = OMcp57_119*RLcp57_220-OMcp57_219*RLcp57_120;
  PxF4[1] = q[1]+RLcp57_114+RLcp57_115+RLcp57_116+RLcp57_117+RLcp57_118+RLcp57_120;
  PxF4[2] = q[2]+RLcp57_214+RLcp57_215+RLcp57_216+RLcp57_217+RLcp57_218+RLcp57_220;
  PxF4[3] = q[3]+RLcp57_314+RLcp57_315+RLcp57_316+RLcp57_317+RLcp57_318+RLcp57_320;
  RxF4[1][1] = ROcp57_120;
  RxF4[1][2] = ROcp57_220;
  RxF4[1][3] = ROcp57_320;
  RxF4[2][1] = ROcp57_418;
  RxF4[2][2] = ROcp57_518;
  RxF4[2][3] = ROcp57_618;
  RxF4[3][1] = ROcp57_720;
  RxF4[3][2] = ROcp57_820;
  RxF4[3][3] = ROcp57_920;
  VxF4[1] = qd[1]+ORcp57_114+ORcp57_115+ORcp57_116+ORcp57_117+ORcp57_118+ORcp57_120;
  VxF4[2] = qd[2]+ORcp57_214+ORcp57_215+ORcp57_216+ORcp57_217+ORcp57_218+ORcp57_220;
  VxF4[3] = qd[3]+ORcp57_314+ORcp57_315+ORcp57_316+ORcp57_317+ORcp57_318+ORcp57_320;
  OMxF4[1] = OMcp57_119+qd[20]*ROcp57_418;
  OMxF4[2] = OMcp57_219+qd[20]*ROcp57_518;
  OMxF4[3] = OMcp57_319+qd[20]*ROcp57_618;
  AxF4[1] = qdd[1]+OMcp57_214*ORcp57_315+OMcp57_215*ORcp57_316+OMcp57_216*ORcp57_317+OMcp57_217*ORcp57_318+OMcp57_219*
 ORcp57_320+OMcp57_26*ORcp57_314-OMcp57_314*ORcp57_215-OMcp57_315*ORcp57_216-OMcp57_316*ORcp57_217-OMcp57_317*ORcp57_218-
 OMcp57_319*ORcp57_220-OMcp57_36*ORcp57_214+OPcp57_214*RLcp57_315+OPcp57_215*RLcp57_316+OPcp57_216*RLcp57_317+OPcp57_217*
 RLcp57_318+OPcp57_219*RLcp57_320+OPcp57_26*RLcp57_314-OPcp57_314*RLcp57_215-OPcp57_315*RLcp57_216-OPcp57_316*RLcp57_217-
 OPcp57_317*RLcp57_218-OPcp57_319*RLcp57_220-OPcp57_36*RLcp57_214;
  AxF4[2] = qdd[2]-OMcp57_114*ORcp57_315-OMcp57_115*ORcp57_316-OMcp57_116*ORcp57_317-OMcp57_117*ORcp57_318-OMcp57_119*
 ORcp57_320-OMcp57_16*ORcp57_314+OMcp57_314*ORcp57_115+OMcp57_315*ORcp57_116+OMcp57_316*ORcp57_117+OMcp57_317*ORcp57_118+
 OMcp57_319*ORcp57_120+OMcp57_36*ORcp57_114-OPcp57_114*RLcp57_315-OPcp57_115*RLcp57_316-OPcp57_116*RLcp57_317-OPcp57_117*
 RLcp57_318-OPcp57_119*RLcp57_320-OPcp57_16*RLcp57_314+OPcp57_314*RLcp57_115+OPcp57_315*RLcp57_116+OPcp57_316*RLcp57_117+
 OPcp57_317*RLcp57_118+OPcp57_319*RLcp57_120+OPcp57_36*RLcp57_114;
  AxF4[3] = qdd[3]+OMcp57_114*ORcp57_215+OMcp57_115*ORcp57_216+OMcp57_116*ORcp57_217+OMcp57_117*ORcp57_218+OMcp57_119*
 ORcp57_220+OMcp57_16*ORcp57_214-OMcp57_214*ORcp57_115-OMcp57_215*ORcp57_116-OMcp57_216*ORcp57_117-OMcp57_217*ORcp57_118-
 OMcp57_219*ORcp57_120-OMcp57_26*ORcp57_114+OPcp57_114*RLcp57_215+OPcp57_115*RLcp57_216+OPcp57_116*RLcp57_217+OPcp57_117*
 RLcp57_218+OPcp57_119*RLcp57_220+OPcp57_16*RLcp57_214-OPcp57_214*RLcp57_115-OPcp57_215*RLcp57_116-OPcp57_216*RLcp57_117-
 OPcp57_217*RLcp57_118-OPcp57_219*RLcp57_120-OPcp57_26*RLcp57_114;
  OMPxF4[1] = OPcp57_119+qd[20]*(OMcp57_219*ROcp57_618-OMcp57_319*ROcp57_518)+qdd[20]*ROcp57_418;
  OMPxF4[2] = OPcp57_219-qd[20]*(OMcp57_119*ROcp57_618-OMcp57_319*ROcp57_418)+qdd[20]*ROcp57_518;
  OMPxF4[3] = OPcp57_319+qd[20]*(OMcp57_119*ROcp57_518-OMcp57_219*ROcp57_418)+qdd[20]*ROcp57_618;
 
// Sensor Forces Computation 

  SWr4 = user_ExtForces(PxF4,RxF4,VxF4,OMxF4,AxF4,OMPxF4,s,tsim,4);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc158 = ROcp57_120*SWr4[1]+ROcp57_220*SWr4[2]+ROcp57_320*SWr4[3];
  xfrc258 = ROcp57_418*SWr4[1]+ROcp57_518*SWr4[2]+ROcp57_618*SWr4[3];
  xfrc358 = ROcp57_720*SWr4[1]+ROcp57_820*SWr4[2]+ROcp57_920*SWr4[3];
  frc[1][20] = s->frc[1][20]+xfrc158;
  frc[2][20] = s->frc[2][20]+xfrc258;
  frc[3][20] = s->frc[3][20]+xfrc358;
  xtrq158 = ROcp57_120*SWr4[4]+ROcp57_220*SWr4[5]+ROcp57_320*SWr4[6];
  xtrq258 = ROcp57_418*SWr4[4]+ROcp57_518*SWr4[5]+ROcp57_618*SWr4[6];
  xtrq358 = ROcp57_720*SWr4[4]+ROcp57_820*SWr4[5]+ROcp57_920*SWr4[6];
  trq[1][20] = s->trq[1][20]+xtrq158-xfrc258*(SWr4[9]-s->l[3][20])+xfrc358*SWr4[8];
  trq[2][20] = s->trq[2][20]+xtrq258+xfrc158*(SWr4[9]-s->l[3][20])-xfrc358*(SWr4[7]-s->l[1][20]);
  trq[3][20] = s->trq[3][20]+xtrq358-xfrc158*SWr4[8]+xfrc258*(SWr4[7]-s->l[1][20]);

// = = Block_0_0_1_5_0_1 = = 
 
// Sensor Kinematics 


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
  OMcp58_25 = qd[5]*C4;
  OMcp58_35 = qd[5]*S4;
  OMcp58_16 = qd[4]+qd[6]*S5;
  OMcp58_26 = OMcp58_25+qd[6]*ROcp58_85;
  OMcp58_36 = OMcp58_35+qd[6]*ROcp58_95;
  OPcp58_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp58_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp58_95-OMcp58_35*S5)-qdd[5]*C4-qdd[6]*ROcp58_85);
  OPcp58_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp58_85-OMcp58_25*S5)+qdd[5]*S4+qdd[6]*ROcp58_95;

// = = Block_0_0_1_5_0_4 = = 
 
// Sensor Kinematics 


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
  RLcp58_121 = ROcp58_16*s->dpt[1][3]+s->dpt[3][3]*S5;
  RLcp58_221 = ROcp58_26*s->dpt[1][3]+ROcp58_85*s->dpt[3][3];
  RLcp58_321 = ROcp58_36*s->dpt[1][3]+ROcp58_95*s->dpt[3][3];
  OMcp58_121 = OMcp58_16+qd[21]*ROcp58_16;
  OMcp58_221 = OMcp58_26+qd[21]*ROcp58_26;
  OMcp58_321 = OMcp58_36+qd[21]*ROcp58_36;
  ORcp58_121 = OMcp58_26*RLcp58_321-OMcp58_36*RLcp58_221;
  ORcp58_221 = -(OMcp58_16*RLcp58_321-OMcp58_36*RLcp58_121);
  ORcp58_321 = OMcp58_16*RLcp58_221-OMcp58_26*RLcp58_121;
  OMcp58_122 = OMcp58_121+qd[22]*ROcp58_421;
  OMcp58_222 = OMcp58_221+qd[22]*ROcp58_521;
  OMcp58_322 = OMcp58_321+qd[22]*ROcp58_621;
  OPcp58_122 = OPcp58_16+qd[21]*(OMcp58_26*ROcp58_36-OMcp58_36*ROcp58_26)+qd[22]*(OMcp58_221*ROcp58_621-OMcp58_321*
 ROcp58_521)+qdd[21]*ROcp58_16+qdd[22]*ROcp58_421;
  OPcp58_222 = OPcp58_26-qd[21]*(OMcp58_16*ROcp58_36-OMcp58_36*ROcp58_16)-qd[22]*(OMcp58_121*ROcp58_621-OMcp58_321*
 ROcp58_421)+qdd[21]*ROcp58_26+qdd[22]*ROcp58_521;
  OPcp58_322 = OPcp58_36+qd[21]*(OMcp58_16*ROcp58_26-OMcp58_26*ROcp58_16)+qd[22]*(OMcp58_121*ROcp58_521-OMcp58_221*
 ROcp58_421)+qdd[21]*ROcp58_36+qdd[22]*ROcp58_621;
  RLcp58_123 = ROcp58_722*s->dpt[3][37];
  RLcp58_223 = ROcp58_822*s->dpt[3][37];
  RLcp58_323 = ROcp58_922*s->dpt[3][37];
  OMcp58_123 = OMcp58_122+qd[23]*ROcp58_722;
  OMcp58_223 = OMcp58_222+qd[23]*ROcp58_822;
  OMcp58_323 = OMcp58_322+qd[23]*ROcp58_922;
  ORcp58_123 = OMcp58_222*RLcp58_323-OMcp58_322*RLcp58_223;
  ORcp58_223 = -(OMcp58_122*RLcp58_323-OMcp58_322*RLcp58_123);
  ORcp58_323 = OMcp58_122*RLcp58_223-OMcp58_222*RLcp58_123;
  OPcp58_123 = OPcp58_122+qd[23]*(OMcp58_222*ROcp58_922-OMcp58_322*ROcp58_822)+qdd[23]*ROcp58_722;
  OPcp58_223 = OPcp58_222-qd[23]*(OMcp58_122*ROcp58_922-OMcp58_322*ROcp58_722)+qdd[23]*ROcp58_822;
  OPcp58_323 = OPcp58_322+qd[23]*(OMcp58_122*ROcp58_822-OMcp58_222*ROcp58_722)+qdd[23]*ROcp58_922;
  RLcp58_190 = ROcp58_722*s->dpt[3][39];
  RLcp58_290 = ROcp58_822*s->dpt[3][39];
  RLcp58_390 = ROcp58_922*s->dpt[3][39];
  ORcp58_190 = OMcp58_223*RLcp58_390-OMcp58_323*RLcp58_290;
  ORcp58_290 = -(OMcp58_123*RLcp58_390-OMcp58_323*RLcp58_190);
  ORcp58_390 = OMcp58_123*RLcp58_290-OMcp58_223*RLcp58_190;
  PxF5[1] = q[1]+RLcp58_121+RLcp58_123+RLcp58_190;
  PxF5[2] = q[2]+RLcp58_221+RLcp58_223+RLcp58_290;
  PxF5[3] = q[3]+RLcp58_321+RLcp58_323+RLcp58_390;
  RxF5[1][1] = ROcp58_123;
  RxF5[1][2] = ROcp58_223;
  RxF5[1][3] = ROcp58_323;
  RxF5[2][1] = ROcp58_423;
  RxF5[2][2] = ROcp58_523;
  RxF5[2][3] = ROcp58_623;
  RxF5[3][1] = ROcp58_722;
  RxF5[3][2] = ROcp58_822;
  RxF5[3][3] = ROcp58_922;
  VxF5[1] = qd[1]+ORcp58_121+ORcp58_123+ORcp58_190;
  VxF5[2] = qd[2]+ORcp58_221+ORcp58_223+ORcp58_290;
  VxF5[3] = qd[3]+ORcp58_321+ORcp58_323+ORcp58_390;
  OMxF5[1] = OMcp58_123;
  OMxF5[2] = OMcp58_223;
  OMxF5[3] = OMcp58_323;
  AxF5[1] = qdd[1]+OMcp58_222*ORcp58_323+OMcp58_223*ORcp58_390+OMcp58_26*ORcp58_321-OMcp58_322*ORcp58_223-OMcp58_323*
 ORcp58_290-OMcp58_36*ORcp58_221+OPcp58_222*RLcp58_323+OPcp58_223*RLcp58_390+OPcp58_26*RLcp58_321-OPcp58_322*RLcp58_223-
 OPcp58_323*RLcp58_290-OPcp58_36*RLcp58_221;
  AxF5[2] = qdd[2]-OMcp58_122*ORcp58_323-OMcp58_123*ORcp58_390-OMcp58_16*ORcp58_321+OMcp58_322*ORcp58_123+OMcp58_323*
 ORcp58_190+OMcp58_36*ORcp58_121-OPcp58_122*RLcp58_323-OPcp58_123*RLcp58_390-OPcp58_16*RLcp58_321+OPcp58_322*RLcp58_123+
 OPcp58_323*RLcp58_190+OPcp58_36*RLcp58_121;
  AxF5[3] = qdd[3]+OMcp58_122*ORcp58_223+OMcp58_123*ORcp58_290+OMcp58_16*ORcp58_221-OMcp58_222*ORcp58_123-OMcp58_223*
 ORcp58_190-OMcp58_26*ORcp58_121+OPcp58_122*RLcp58_223+OPcp58_123*RLcp58_290+OPcp58_16*RLcp58_221-OPcp58_222*RLcp58_123-
 OPcp58_223*RLcp58_190-OPcp58_26*RLcp58_121;
  OMPxF5[1] = OPcp58_123;
  OMPxF5[2] = OPcp58_223;
  OMPxF5[3] = OPcp58_323;
 
// Sensor Forces Computation 

  SWr5 = user_ExtForces(PxF5,RxF5,VxF5,OMxF5,AxF5,OMPxF5,s,tsim,5);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc159 = ROcp58_123*SWr5[1]+ROcp58_223*SWr5[2]+ROcp58_323*SWr5[3];
  xfrc259 = ROcp58_423*SWr5[1]+ROcp58_523*SWr5[2]+ROcp58_623*SWr5[3];
  xfrc359 = ROcp58_722*SWr5[1]+ROcp58_822*SWr5[2]+ROcp58_922*SWr5[3];
  frc[1][23] = s->frc[1][23]+xfrc159;
  frc[2][23] = s->frc[2][23]+xfrc259;
  frc[3][23] = s->frc[3][23]+xfrc359;
  xtrq159 = ROcp58_123*SWr5[4]+ROcp58_223*SWr5[5]+ROcp58_323*SWr5[6];
  xtrq259 = ROcp58_423*SWr5[4]+ROcp58_523*SWr5[5]+ROcp58_623*SWr5[6];
  xtrq359 = ROcp58_722*SWr5[4]+ROcp58_822*SWr5[5]+ROcp58_922*SWr5[6];
  trq[1][23] = s->trq[1][23]+xtrq159-xfrc259*(SWr5[9]-s->l[3][23])+xfrc359*(SWr5[8]-s->l[2][23]);
  trq[2][23] = s->trq[2][23]+xtrq259+xfrc159*(SWr5[9]-s->l[3][23])-xfrc359*(SWr5[7]-s->l[1][23]);
  trq[3][23] = s->trq[3][23]+xtrq359-xfrc159*(SWr5[8]-s->l[2][23])+xfrc259*(SWr5[7]-s->l[1][23]);

// = = Block_0_0_1_5_1_0 = = 
 
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
  frc[1][18] = s->frc[1][18];
  frc[2][18] = s->frc[2][18];
  frc[3][18] = s->frc[3][18];
  frc[1][21] = s->frc[1][21];
  frc[2][21] = s->frc[2][21];
  frc[3][21] = s->frc[3][21];
  frc[1][22] = s->frc[1][22];
  frc[2][22] = s->frc[2][22];
  frc[3][22] = s->frc[3][22];
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
  frc[1][28] = s->frc[1][28];
  frc[2][28] = s->frc[2][28];
  frc[3][28] = s->frc[3][28];
  frc[1][29] = s->frc[1][29];
  frc[2][29] = s->frc[2][29];
  frc[3][29] = s->frc[3][29];
  frc[1][30] = s->frc[1][30];
  frc[2][30] = s->frc[2][30];
  frc[3][30] = s->frc[3][30];
  frc[1][31] = s->frc[1][31];
  frc[2][31] = s->frc[2][31];
  frc[3][31] = s->frc[3][31];
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
  trq[1][18] = s->trq[1][18];
  trq[2][18] = s->trq[2][18];
  trq[3][18] = s->trq[3][18];
  trq[1][21] = s->trq[1][21];
  trq[2][21] = s->trq[2][21];
  trq[3][21] = s->trq[3][21];
  trq[1][22] = s->trq[1][22];
  trq[2][22] = s->trq[2][22];
  trq[3][22] = s->trq[3][22];
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
  trq[1][28] = s->trq[1][28];
  trq[2][28] = s->trq[2][28];
  trq[3][28] = s->trq[3][28];
  trq[1][29] = s->trq[1][29];
  trq[2][29] = s->trq[2][29];
  trq[3][29] = s->trq[3][29];
  trq[1][30] = s->trq[1][30];
  trq[2][30] = s->trq[2][30];
  trq[3][30] = s->trq[3][30];
  trq[1][31] = s->trq[1][31];
  trq[2][31] = s->trq[2][31];
  trq[3][31] = s->trq[3][31];

// ====== END Task 0 ====== 


}
 

