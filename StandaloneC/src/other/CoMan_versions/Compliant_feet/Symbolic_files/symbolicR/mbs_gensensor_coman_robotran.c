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
//	==> Generation Date : Wed Mar  5 10:45:10 2014
//
//	==> Project name : coman_robotran
//	==> using XML input file 
//
//	==> Number of joints : 31
//
//	==> Function : F 6 : Sensors Kinematical Informations (sens) 
//	==> Flops complexity : 10654
//
//	==> Generation Time :  0.190 seconds
//	==> Post-Processing :  0.190 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "MBSdataStructR7.h"
#include "MBSfunR7.h"
 
void  gensensor(MBSsensorStruct *sens, 
              MBSdataStruct *s,
              int isens)
{ 
 
#include "mbs_gensensor_coman_robotran.h" 
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

// = = Block_0_0_0_0_0_5 = = 
 
// Trigonometric Variables  

  C24 = cos(q[24]);
  S24 = sin(q[24]);
  C25 = cos(q[25]);
  S25 = sin(q[25]);
  C26 = cos(q[26]);
  S26 = sin(q[26]);
  C27 = cos(q[27]);
  S27 = sin(q[27]);

// = = Block_0_0_0_0_0_6 = = 
 
// Trigonometric Variables  

  C28 = cos(q[28]);
  S28 = sin(q[28]);
  C29 = cos(q[29]);
  S29 = sin(q[29]);
  C30 = cos(q[30]);
  S30 = sin(q[30]);
  C31 = cos(q[31]);
  S31 = sin(q[31]);

// ====== END Task 0 ====== 

// ===== BEGIN task 1 ===== 
 
switch(isens)
{
 
// 
break;
case 1:
 


// = = Block_1_0_0_1_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = q[1];
    sens->R[1][1] = (1.0);
    sens->R[2][2] = (1.0);
    sens->R[3][3] = (1.0);
    sens->V[1] = qd[1];
    sens->A[1] = qdd[1];
 
// 
break;
case 2:
 


// = = Block_1_0_0_2_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = q[1];
    sens->P[2] = q[2];
    sens->R[1][1] = (1.0);
    sens->R[2][2] = (1.0);
    sens->R[3][3] = (1.0);
    sens->V[1] = qd[1];
    sens->V[2] = qd[2];
    sens->A[1] = qdd[1];
    sens->A[2] = qdd[2];
 
// 
break;
case 3:
 


// = = Block_1_0_0_3_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = q[1];
    sens->P[2] = q[2];
    sens->P[3] = q[3];
    sens->R[1][1] = (1.0);
    sens->R[2][2] = (1.0);
    sens->R[3][3] = (1.0);
    sens->V[1] = qd[1];
    sens->V[2] = qd[2];
    sens->V[3] = qd[3];
    sens->A[1] = qdd[1];
    sens->A[2] = qdd[2];
    sens->A[3] = qdd[3];
 
// 
break;
case 4:
 


// = = Block_1_0_0_4_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = q[1];
    sens->P[2] = q[2];
    sens->P[3] = q[3];
    sens->R[1][1] = (1.0);
    sens->R[2][2] = C4;
    sens->R[2][3] = S4;
    sens->R[3][2] = -S4;
    sens->R[3][3] = C4;
    sens->V[1] = qd[1];
    sens->V[2] = qd[2];
    sens->V[3] = qd[3];
    sens->OM[1] = qd[4];
    sens->A[1] = qdd[1];
    sens->A[2] = qdd[2];
    sens->A[3] = qdd[3];
    sens->OMP[1] = qdd[4];
 
// 
break;
case 5:
 


// = = Block_1_0_0_5_0_1 = = 
 
// Sensor Kinematics 


    ROcp4_25 = S4*S5;
    ROcp4_35 = -C4*S5;
    ROcp4_85 = -S4*C5;
    ROcp4_95 = C4*C5;
    OMcp4_25 = qd[5]*C4;
    OMcp4_35 = qd[5]*S4;
    OPcp4_25 = qdd[5]*C4-qd[4]*qd[5]*S4;
    OPcp4_35 = qdd[5]*S4+qd[4]*qd[5]*C4;

// = = Block_1_0_0_5_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = q[1];
    sens->P[2] = q[2];
    sens->P[3] = q[3];
    sens->R[1][1] = C5;
    sens->R[1][2] = ROcp4_25;
    sens->R[1][3] = ROcp4_35;
    sens->R[2][2] = C4;
    sens->R[2][3] = S4;
    sens->R[3][1] = S5;
    sens->R[3][2] = ROcp4_85;
    sens->R[3][3] = ROcp4_95;
    sens->V[1] = qd[1];
    sens->V[2] = qd[2];
    sens->V[3] = qd[3];
    sens->OM[1] = qd[4];
    sens->OM[2] = OMcp4_25;
    sens->OM[3] = OMcp4_35;
    sens->A[1] = qdd[1];
    sens->A[2] = qdd[2];
    sens->A[3] = qdd[3];
    sens->OMP[1] = qdd[4];
    sens->OMP[2] = OPcp4_25;
    sens->OMP[3] = OPcp4_35;
 
// 
break;
case 6:
 


// = = Block_1_0_0_6_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp5_25 = qd[5]*C4;
    OMcp5_35 = qd[5]*S4;
    OMcp5_16 = qd[4]+qd[6]*S5;
    OMcp5_26 = OMcp5_25+ROcp5_85*qd[6];
    OMcp5_36 = OMcp5_35+ROcp5_95*qd[6];
    OPcp5_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp5_26 = ROcp5_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp5_35*S5-ROcp5_95*qd[4]);
    OPcp5_36 = ROcp5_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp5_25*S5-ROcp5_85*qd[4]);

// = = Block_1_0_0_6_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = q[1];
    sens->P[2] = q[2];
    sens->P[3] = q[3];
    sens->R[1][1] = ROcp5_16;
    sens->R[1][2] = ROcp5_26;
    sens->R[1][3] = ROcp5_36;
    sens->R[2][1] = ROcp5_46;
    sens->R[2][2] = ROcp5_56;
    sens->R[2][3] = ROcp5_66;
    sens->R[3][1] = S5;
    sens->R[3][2] = ROcp5_85;
    sens->R[3][3] = ROcp5_95;
    sens->V[1] = qd[1];
    sens->V[2] = qd[2];
    sens->V[3] = qd[3];
    sens->OM[1] = OMcp5_16;
    sens->OM[2] = OMcp5_26;
    sens->OM[3] = OMcp5_36;
    sens->A[1] = qdd[1];
    sens->A[2] = qdd[2];
    sens->A[3] = qdd[3];
    sens->OMP[1] = OPcp5_16;
    sens->OMP[2] = OPcp5_26;
    sens->OMP[3] = OPcp5_36;
 
// 
break;
case 7:
 


// = = Block_1_0_0_7_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp6_25 = qd[5]*C4;
    OMcp6_35 = qd[5]*S4;
    OMcp6_16 = qd[4]+qd[6]*S5;
    OMcp6_26 = OMcp6_25+ROcp6_85*qd[6];
    OMcp6_36 = OMcp6_35+ROcp6_95*qd[6];
    OPcp6_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp6_26 = ROcp6_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp6_35*S5-ROcp6_95*qd[4]);
    OPcp6_36 = ROcp6_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp6_25*S5-ROcp6_85*qd[4]);

// = = Block_1_0_0_7_0_2 = = 
 
// Sensor Kinematics 


    ROcp6_17 = ROcp6_16*C7-S5*S7;
    ROcp6_27 = ROcp6_26*C7-ROcp6_85*S7;
    ROcp6_37 = ROcp6_36*C7-ROcp6_95*S7;
    ROcp6_77 = ROcp6_16*S7+S5*C7;
    ROcp6_87 = ROcp6_26*S7+ROcp6_85*C7;
    ROcp6_97 = ROcp6_36*S7+ROcp6_95*C7;
    RLcp6_17 = ROcp6_46*s->dpt[2][1];
    RLcp6_27 = ROcp6_56*s->dpt[2][1];
    RLcp6_37 = ROcp6_66*s->dpt[2][1];
    POcp6_17 = RLcp6_17+q[1];
    POcp6_27 = RLcp6_27+q[2];
    POcp6_37 = RLcp6_37+q[3];
    OMcp6_17 = OMcp6_16+ROcp6_46*qd[7];
    OMcp6_27 = OMcp6_26+ROcp6_56*qd[7];
    OMcp6_37 = OMcp6_36+ROcp6_66*qd[7];
    ORcp6_17 = OMcp6_26*RLcp6_37-OMcp6_36*RLcp6_27;
    ORcp6_27 = -(OMcp6_16*RLcp6_37-OMcp6_36*RLcp6_17);
    ORcp6_37 = OMcp6_16*RLcp6_27-OMcp6_26*RLcp6_17;
    VIcp6_17 = ORcp6_17+qd[1];
    VIcp6_27 = ORcp6_27+qd[2];
    VIcp6_37 = ORcp6_37+qd[3];
    OPcp6_17 = OPcp6_16+ROcp6_46*qdd[7]+qd[7]*(OMcp6_26*ROcp6_66-OMcp6_36*ROcp6_56);
    OPcp6_27 = OPcp6_26+ROcp6_56*qdd[7]-qd[7]*(OMcp6_16*ROcp6_66-OMcp6_36*ROcp6_46);
    OPcp6_37 = OPcp6_36+ROcp6_66*qdd[7]+qd[7]*(OMcp6_16*ROcp6_56-OMcp6_26*ROcp6_46);
    ACcp6_17 = qdd[1]+OMcp6_26*ORcp6_37-OMcp6_36*ORcp6_27+OPcp6_26*RLcp6_37-OPcp6_36*RLcp6_27;
    ACcp6_27 = qdd[2]-OMcp6_16*ORcp6_37+OMcp6_36*ORcp6_17-OPcp6_16*RLcp6_37+OPcp6_36*RLcp6_17;
    ACcp6_37 = qdd[3]+OMcp6_16*ORcp6_27-OMcp6_26*ORcp6_17+OPcp6_16*RLcp6_27-OPcp6_26*RLcp6_17;

// = = Block_1_0_0_7_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp6_17;
    sens->P[2] = POcp6_27;
    sens->P[3] = POcp6_37;
    sens->R[1][1] = ROcp6_17;
    sens->R[1][2] = ROcp6_27;
    sens->R[1][3] = ROcp6_37;
    sens->R[2][1] = ROcp6_46;
    sens->R[2][2] = ROcp6_56;
    sens->R[2][3] = ROcp6_66;
    sens->R[3][1] = ROcp6_77;
    sens->R[3][2] = ROcp6_87;
    sens->R[3][3] = ROcp6_97;
    sens->V[1] = VIcp6_17;
    sens->V[2] = VIcp6_27;
    sens->V[3] = VIcp6_37;
    sens->OM[1] = OMcp6_17;
    sens->OM[2] = OMcp6_27;
    sens->OM[3] = OMcp6_37;
    sens->A[1] = ACcp6_17;
    sens->A[2] = ACcp6_27;
    sens->A[3] = ACcp6_37;
    sens->OMP[1] = OPcp6_17;
    sens->OMP[2] = OPcp6_27;
    sens->OMP[3] = OPcp6_37;
 
// 
break;
case 8:
 


// = = Block_1_0_0_8_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp7_25 = qd[5]*C4;
    OMcp7_35 = qd[5]*S4;
    OMcp7_16 = qd[4]+qd[6]*S5;
    OMcp7_26 = OMcp7_25+ROcp7_85*qd[6];
    OMcp7_36 = OMcp7_35+ROcp7_95*qd[6];
    OPcp7_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp7_26 = ROcp7_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp7_35*S5-ROcp7_95*qd[4]);
    OPcp7_36 = ROcp7_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp7_25*S5-ROcp7_85*qd[4]);

// = = Block_1_0_0_8_0_2 = = 
 
// Sensor Kinematics 


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
    RLcp7_17 = ROcp7_46*s->dpt[2][1];
    RLcp7_27 = ROcp7_56*s->dpt[2][1];
    RLcp7_37 = ROcp7_66*s->dpt[2][1];
    OMcp7_17 = OMcp7_16+ROcp7_46*qd[7];
    OMcp7_27 = OMcp7_26+ROcp7_56*qd[7];
    OMcp7_37 = OMcp7_36+ROcp7_66*qd[7];
    ORcp7_17 = OMcp7_26*RLcp7_37-OMcp7_36*RLcp7_27;
    ORcp7_27 = -(OMcp7_16*RLcp7_37-OMcp7_36*RLcp7_17);
    ORcp7_37 = OMcp7_16*RLcp7_27-OMcp7_26*RLcp7_17;
    OPcp7_17 = OPcp7_16+ROcp7_46*qdd[7]+qd[7]*(OMcp7_26*ROcp7_66-OMcp7_36*ROcp7_56);
    OPcp7_27 = OPcp7_26+ROcp7_56*qdd[7]-qd[7]*(OMcp7_16*ROcp7_66-OMcp7_36*ROcp7_46);
    OPcp7_37 = OPcp7_36+ROcp7_66*qdd[7]+qd[7]*(OMcp7_16*ROcp7_56-OMcp7_26*ROcp7_46);
    RLcp7_18 = ROcp7_46*s->dpt[2][6];
    RLcp7_28 = ROcp7_56*s->dpt[2][6];
    RLcp7_38 = ROcp7_66*s->dpt[2][6];
    POcp7_18 = RLcp7_17+RLcp7_18+q[1];
    POcp7_28 = RLcp7_27+RLcp7_28+q[2];
    POcp7_38 = RLcp7_37+RLcp7_38+q[3];
    OMcp7_18 = OMcp7_17+ROcp7_17*qd[8];
    OMcp7_28 = OMcp7_27+ROcp7_27*qd[8];
    OMcp7_38 = OMcp7_37+ROcp7_37*qd[8];
    ORcp7_18 = OMcp7_27*RLcp7_38-OMcp7_37*RLcp7_28;
    ORcp7_28 = -(OMcp7_17*RLcp7_38-OMcp7_37*RLcp7_18);
    ORcp7_38 = OMcp7_17*RLcp7_28-OMcp7_27*RLcp7_18;
    VIcp7_18 = ORcp7_17+ORcp7_18+qd[1];
    VIcp7_28 = ORcp7_27+ORcp7_28+qd[2];
    VIcp7_38 = ORcp7_37+ORcp7_38+qd[3];
    OPcp7_18 = OPcp7_17+ROcp7_17*qdd[8]+qd[8]*(OMcp7_27*ROcp7_37-OMcp7_37*ROcp7_27);
    OPcp7_28 = OPcp7_27+ROcp7_27*qdd[8]-qd[8]*(OMcp7_17*ROcp7_37-OMcp7_37*ROcp7_17);
    OPcp7_38 = OPcp7_37+ROcp7_37*qdd[8]+qd[8]*(OMcp7_17*ROcp7_27-OMcp7_27*ROcp7_17);
    ACcp7_18 = qdd[1]+OMcp7_26*ORcp7_37+OMcp7_27*ORcp7_38-OMcp7_36*ORcp7_27-OMcp7_37*ORcp7_28+OPcp7_26*RLcp7_37+OPcp7_27*
 RLcp7_38-OPcp7_36*RLcp7_27-OPcp7_37*RLcp7_28;
    ACcp7_28 = qdd[2]-OMcp7_16*ORcp7_37-OMcp7_17*ORcp7_38+OMcp7_36*ORcp7_17+OMcp7_37*ORcp7_18-OPcp7_16*RLcp7_37-OPcp7_17*
 RLcp7_38+OPcp7_36*RLcp7_17+OPcp7_37*RLcp7_18;
    ACcp7_38 = qdd[3]+OMcp7_16*ORcp7_27+OMcp7_17*ORcp7_28-OMcp7_26*ORcp7_17-OMcp7_27*ORcp7_18+OPcp7_16*RLcp7_27+OPcp7_17*
 RLcp7_28-OPcp7_26*RLcp7_17-OPcp7_27*RLcp7_18;

// = = Block_1_0_0_8_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp7_18;
    sens->P[2] = POcp7_28;
    sens->P[3] = POcp7_38;
    sens->R[1][1] = ROcp7_17;
    sens->R[1][2] = ROcp7_27;
    sens->R[1][3] = ROcp7_37;
    sens->R[2][1] = ROcp7_48;
    sens->R[2][2] = ROcp7_58;
    sens->R[2][3] = ROcp7_68;
    sens->R[3][1] = ROcp7_78;
    sens->R[3][2] = ROcp7_88;
    sens->R[3][3] = ROcp7_98;
    sens->V[1] = VIcp7_18;
    sens->V[2] = VIcp7_28;
    sens->V[3] = VIcp7_38;
    sens->OM[1] = OMcp7_18;
    sens->OM[2] = OMcp7_28;
    sens->OM[3] = OMcp7_38;
    sens->A[1] = ACcp7_18;
    sens->A[2] = ACcp7_28;
    sens->A[3] = ACcp7_38;
    sens->OMP[1] = OPcp7_18;
    sens->OMP[2] = OPcp7_28;
    sens->OMP[3] = OPcp7_38;
 
// 
break;
case 9:
 


// = = Block_1_0_0_9_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp8_25 = qd[5]*C4;
    OMcp8_35 = qd[5]*S4;
    OMcp8_16 = qd[4]+qd[6]*S5;
    OMcp8_26 = OMcp8_25+ROcp8_85*qd[6];
    OMcp8_36 = OMcp8_35+ROcp8_95*qd[6];
    OPcp8_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp8_26 = ROcp8_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp8_35*S5-ROcp8_95*qd[4]);
    OPcp8_36 = ROcp8_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp8_25*S5-ROcp8_85*qd[4]);

// = = Block_1_0_0_9_0_2 = = 
 
// Sensor Kinematics 


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
    RLcp8_17 = ROcp8_46*s->dpt[2][1];
    RLcp8_27 = ROcp8_56*s->dpt[2][1];
    RLcp8_37 = ROcp8_66*s->dpt[2][1];
    OMcp8_17 = OMcp8_16+ROcp8_46*qd[7];
    OMcp8_27 = OMcp8_26+ROcp8_56*qd[7];
    OMcp8_37 = OMcp8_36+ROcp8_66*qd[7];
    ORcp8_17 = OMcp8_26*RLcp8_37-OMcp8_36*RLcp8_27;
    ORcp8_27 = -(OMcp8_16*RLcp8_37-OMcp8_36*RLcp8_17);
    ORcp8_37 = OMcp8_16*RLcp8_27-OMcp8_26*RLcp8_17;
    OPcp8_17 = OPcp8_16+ROcp8_46*qdd[7]+qd[7]*(OMcp8_26*ROcp8_66-OMcp8_36*ROcp8_56);
    OPcp8_27 = OPcp8_26+ROcp8_56*qdd[7]-qd[7]*(OMcp8_16*ROcp8_66-OMcp8_36*ROcp8_46);
    OPcp8_37 = OPcp8_36+ROcp8_66*qdd[7]+qd[7]*(OMcp8_16*ROcp8_56-OMcp8_26*ROcp8_46);
    RLcp8_18 = ROcp8_46*s->dpt[2][6];
    RLcp8_28 = ROcp8_56*s->dpt[2][6];
    RLcp8_38 = ROcp8_66*s->dpt[2][6];
    OMcp8_18 = OMcp8_17+ROcp8_17*qd[8];
    OMcp8_28 = OMcp8_27+ROcp8_27*qd[8];
    OMcp8_38 = OMcp8_37+ROcp8_37*qd[8];
    ORcp8_18 = OMcp8_27*RLcp8_38-OMcp8_37*RLcp8_28;
    ORcp8_28 = -(OMcp8_17*RLcp8_38-OMcp8_37*RLcp8_18);
    ORcp8_38 = OMcp8_17*RLcp8_28-OMcp8_27*RLcp8_18;
    OPcp8_18 = OPcp8_17+ROcp8_17*qdd[8]+qd[8]*(OMcp8_27*ROcp8_37-OMcp8_37*ROcp8_27);
    OPcp8_28 = OPcp8_27+ROcp8_27*qdd[8]-qd[8]*(OMcp8_17*ROcp8_37-OMcp8_37*ROcp8_17);
    OPcp8_38 = OPcp8_37+ROcp8_37*qdd[8]+qd[8]*(OMcp8_17*ROcp8_27-OMcp8_27*ROcp8_17);
    RLcp8_19 = ROcp8_78*s->dpt[3][8];
    RLcp8_29 = ROcp8_88*s->dpt[3][8];
    RLcp8_39 = ROcp8_98*s->dpt[3][8];
    POcp8_19 = RLcp8_17+RLcp8_18+RLcp8_19+q[1];
    POcp8_29 = RLcp8_27+RLcp8_28+RLcp8_29+q[2];
    POcp8_39 = RLcp8_37+RLcp8_38+RLcp8_39+q[3];
    OMcp8_19 = OMcp8_18+ROcp8_78*qd[9];
    OMcp8_29 = OMcp8_28+ROcp8_88*qd[9];
    OMcp8_39 = OMcp8_38+ROcp8_98*qd[9];
    ORcp8_19 = OMcp8_28*RLcp8_39-OMcp8_38*RLcp8_29;
    ORcp8_29 = -(OMcp8_18*RLcp8_39-OMcp8_38*RLcp8_19);
    ORcp8_39 = OMcp8_18*RLcp8_29-OMcp8_28*RLcp8_19;
    VIcp8_19 = ORcp8_17+ORcp8_18+ORcp8_19+qd[1];
    VIcp8_29 = ORcp8_27+ORcp8_28+ORcp8_29+qd[2];
    VIcp8_39 = ORcp8_37+ORcp8_38+ORcp8_39+qd[3];
    OPcp8_19 = OPcp8_18+ROcp8_78*qdd[9]+qd[9]*(OMcp8_28*ROcp8_98-OMcp8_38*ROcp8_88);
    OPcp8_29 = OPcp8_28+ROcp8_88*qdd[9]-qd[9]*(OMcp8_18*ROcp8_98-OMcp8_38*ROcp8_78);
    OPcp8_39 = OPcp8_38+ROcp8_98*qdd[9]+qd[9]*(OMcp8_18*ROcp8_88-OMcp8_28*ROcp8_78);
    ACcp8_19 = qdd[1]+OMcp8_26*ORcp8_37+OMcp8_27*ORcp8_38+OMcp8_28*ORcp8_39-OMcp8_36*ORcp8_27-OMcp8_37*ORcp8_28-OMcp8_38*
 ORcp8_29+OPcp8_26*RLcp8_37+OPcp8_27*RLcp8_38+OPcp8_28*RLcp8_39-OPcp8_36*RLcp8_27-OPcp8_37*RLcp8_28-OPcp8_38*RLcp8_29;
    ACcp8_29 = qdd[2]-OMcp8_16*ORcp8_37-OMcp8_17*ORcp8_38-OMcp8_18*ORcp8_39+OMcp8_36*ORcp8_17+OMcp8_37*ORcp8_18+OMcp8_38*
 ORcp8_19-OPcp8_16*RLcp8_37-OPcp8_17*RLcp8_38-OPcp8_18*RLcp8_39+OPcp8_36*RLcp8_17+OPcp8_37*RLcp8_18+OPcp8_38*RLcp8_19;
    ACcp8_39 = qdd[3]+OMcp8_16*ORcp8_27+OMcp8_17*ORcp8_28+OMcp8_18*ORcp8_29-OMcp8_26*ORcp8_17-OMcp8_27*ORcp8_18-OMcp8_28*
 ORcp8_19+OPcp8_16*RLcp8_27+OPcp8_17*RLcp8_28+OPcp8_18*RLcp8_29-OPcp8_26*RLcp8_17-OPcp8_27*RLcp8_18-OPcp8_28*RLcp8_19;

// = = Block_1_0_0_9_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp8_19;
    sens->P[2] = POcp8_29;
    sens->P[3] = POcp8_39;
    sens->R[1][1] = ROcp8_19;
    sens->R[1][2] = ROcp8_29;
    sens->R[1][3] = ROcp8_39;
    sens->R[2][1] = ROcp8_49;
    sens->R[2][2] = ROcp8_59;
    sens->R[2][3] = ROcp8_69;
    sens->R[3][1] = ROcp8_78;
    sens->R[3][2] = ROcp8_88;
    sens->R[3][3] = ROcp8_98;
    sens->V[1] = VIcp8_19;
    sens->V[2] = VIcp8_29;
    sens->V[3] = VIcp8_39;
    sens->OM[1] = OMcp8_19;
    sens->OM[2] = OMcp8_29;
    sens->OM[3] = OMcp8_39;
    sens->A[1] = ACcp8_19;
    sens->A[2] = ACcp8_29;
    sens->A[3] = ACcp8_39;
    sens->OMP[1] = OPcp8_19;
    sens->OMP[2] = OPcp8_29;
    sens->OMP[3] = OPcp8_39;
 
// 
break;
case 10:
 


// = = Block_1_0_0_10_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp9_25 = qd[5]*C4;
    OMcp9_35 = qd[5]*S4;
    OMcp9_16 = qd[4]+qd[6]*S5;
    OMcp9_26 = OMcp9_25+ROcp9_85*qd[6];
    OMcp9_36 = OMcp9_35+ROcp9_95*qd[6];
    OPcp9_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp9_26 = ROcp9_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp9_35*S5-ROcp9_95*qd[4]);
    OPcp9_36 = ROcp9_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp9_25*S5-ROcp9_85*qd[4]);

// = = Block_1_0_0_10_0_2 = = 
 
// Sensor Kinematics 


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
    RLcp9_17 = ROcp9_46*s->dpt[2][1];
    RLcp9_27 = ROcp9_56*s->dpt[2][1];
    RLcp9_37 = ROcp9_66*s->dpt[2][1];
    OMcp9_17 = OMcp9_16+ROcp9_46*qd[7];
    OMcp9_27 = OMcp9_26+ROcp9_56*qd[7];
    OMcp9_37 = OMcp9_36+ROcp9_66*qd[7];
    ORcp9_17 = OMcp9_26*RLcp9_37-OMcp9_36*RLcp9_27;
    ORcp9_27 = -(OMcp9_16*RLcp9_37-OMcp9_36*RLcp9_17);
    ORcp9_37 = OMcp9_16*RLcp9_27-OMcp9_26*RLcp9_17;
    OPcp9_17 = OPcp9_16+ROcp9_46*qdd[7]+qd[7]*(OMcp9_26*ROcp9_66-OMcp9_36*ROcp9_56);
    OPcp9_27 = OPcp9_26+ROcp9_56*qdd[7]-qd[7]*(OMcp9_16*ROcp9_66-OMcp9_36*ROcp9_46);
    OPcp9_37 = OPcp9_36+ROcp9_66*qdd[7]+qd[7]*(OMcp9_16*ROcp9_56-OMcp9_26*ROcp9_46);
    RLcp9_18 = ROcp9_46*s->dpt[2][6];
    RLcp9_28 = ROcp9_56*s->dpt[2][6];
    RLcp9_38 = ROcp9_66*s->dpt[2][6];
    OMcp9_18 = OMcp9_17+ROcp9_17*qd[8];
    OMcp9_28 = OMcp9_27+ROcp9_27*qd[8];
    OMcp9_38 = OMcp9_37+ROcp9_37*qd[8];
    ORcp9_18 = OMcp9_27*RLcp9_38-OMcp9_37*RLcp9_28;
    ORcp9_28 = -(OMcp9_17*RLcp9_38-OMcp9_37*RLcp9_18);
    ORcp9_38 = OMcp9_17*RLcp9_28-OMcp9_27*RLcp9_18;
    OPcp9_18 = OPcp9_17+ROcp9_17*qdd[8]+qd[8]*(OMcp9_27*ROcp9_37-OMcp9_37*ROcp9_27);
    OPcp9_28 = OPcp9_27+ROcp9_27*qdd[8]-qd[8]*(OMcp9_17*ROcp9_37-OMcp9_37*ROcp9_17);
    OPcp9_38 = OPcp9_37+ROcp9_37*qdd[8]+qd[8]*(OMcp9_17*ROcp9_27-OMcp9_27*ROcp9_17);
    RLcp9_19 = ROcp9_78*s->dpt[3][8];
    RLcp9_29 = ROcp9_88*s->dpt[3][8];
    RLcp9_39 = ROcp9_98*s->dpt[3][8];
    OMcp9_19 = OMcp9_18+ROcp9_78*qd[9];
    OMcp9_29 = OMcp9_28+ROcp9_88*qd[9];
    OMcp9_39 = OMcp9_38+ROcp9_98*qd[9];
    ORcp9_19 = OMcp9_28*RLcp9_39-OMcp9_38*RLcp9_29;
    ORcp9_29 = -(OMcp9_18*RLcp9_39-OMcp9_38*RLcp9_19);
    ORcp9_39 = OMcp9_18*RLcp9_29-OMcp9_28*RLcp9_19;
    OPcp9_19 = OPcp9_18+ROcp9_78*qdd[9]+qd[9]*(OMcp9_28*ROcp9_98-OMcp9_38*ROcp9_88);
    OPcp9_29 = OPcp9_28+ROcp9_88*qdd[9]-qd[9]*(OMcp9_18*ROcp9_98-OMcp9_38*ROcp9_78);
    OPcp9_39 = OPcp9_38+ROcp9_98*qdd[9]+qd[9]*(OMcp9_18*ROcp9_88-OMcp9_28*ROcp9_78);
    RLcp9_110 = ROcp9_78*s->dpt[3][10];
    RLcp9_210 = ROcp9_88*s->dpt[3][10];
    RLcp9_310 = ROcp9_98*s->dpt[3][10];
    POcp9_110 = RLcp9_110+RLcp9_17+RLcp9_18+RLcp9_19+q[1];
    POcp9_210 = RLcp9_210+RLcp9_27+RLcp9_28+RLcp9_29+q[2];
    POcp9_310 = RLcp9_310+RLcp9_37+RLcp9_38+RLcp9_39+q[3];
    OMcp9_110 = OMcp9_19+ROcp9_49*qd[10];
    OMcp9_210 = OMcp9_29+ROcp9_59*qd[10];
    OMcp9_310 = OMcp9_39+ROcp9_69*qd[10];
    ORcp9_110 = OMcp9_29*RLcp9_310-OMcp9_39*RLcp9_210;
    ORcp9_210 = -(OMcp9_19*RLcp9_310-OMcp9_39*RLcp9_110);
    ORcp9_310 = OMcp9_19*RLcp9_210-OMcp9_29*RLcp9_110;
    VIcp9_110 = ORcp9_110+ORcp9_17+ORcp9_18+ORcp9_19+qd[1];
    VIcp9_210 = ORcp9_210+ORcp9_27+ORcp9_28+ORcp9_29+qd[2];
    VIcp9_310 = ORcp9_310+ORcp9_37+ORcp9_38+ORcp9_39+qd[3];
    OPcp9_110 = OPcp9_19+ROcp9_49*qdd[10]+qd[10]*(OMcp9_29*ROcp9_69-OMcp9_39*ROcp9_59);
    OPcp9_210 = OPcp9_29+ROcp9_59*qdd[10]-qd[10]*(OMcp9_19*ROcp9_69-OMcp9_39*ROcp9_49);
    OPcp9_310 = OPcp9_39+ROcp9_69*qdd[10]+qd[10]*(OMcp9_19*ROcp9_59-OMcp9_29*ROcp9_49);
    ACcp9_110 = qdd[1]+OMcp9_26*ORcp9_37+OMcp9_27*ORcp9_38+OMcp9_28*ORcp9_39+OMcp9_29*ORcp9_310-OMcp9_36*ORcp9_27-OMcp9_37
 *ORcp9_28-OMcp9_38*ORcp9_29-OMcp9_39*ORcp9_210+OPcp9_26*RLcp9_37+OPcp9_27*RLcp9_38+OPcp9_28*RLcp9_39+OPcp9_29*RLcp9_310-
 OPcp9_36*RLcp9_27-OPcp9_37*RLcp9_28-OPcp9_38*RLcp9_29-OPcp9_39*RLcp9_210;
    ACcp9_210 = qdd[2]-OMcp9_16*ORcp9_37-OMcp9_17*ORcp9_38-OMcp9_18*ORcp9_39-OMcp9_19*ORcp9_310+OMcp9_36*ORcp9_17+OMcp9_37
 *ORcp9_18+OMcp9_38*ORcp9_19+OMcp9_39*ORcp9_110-OPcp9_16*RLcp9_37-OPcp9_17*RLcp9_38-OPcp9_18*RLcp9_39-OPcp9_19*RLcp9_310+
 OPcp9_36*RLcp9_17+OPcp9_37*RLcp9_18+OPcp9_38*RLcp9_19+OPcp9_39*RLcp9_110;
    ACcp9_310 = qdd[3]+OMcp9_16*ORcp9_27+OMcp9_17*ORcp9_28+OMcp9_18*ORcp9_29+OMcp9_19*ORcp9_210-OMcp9_26*ORcp9_17-OMcp9_27
 *ORcp9_18-OMcp9_28*ORcp9_19-OMcp9_29*ORcp9_110+OPcp9_16*RLcp9_27+OPcp9_17*RLcp9_28+OPcp9_18*RLcp9_29+OPcp9_19*RLcp9_210-
 OPcp9_26*RLcp9_17-OPcp9_27*RLcp9_18-OPcp9_28*RLcp9_19-OPcp9_29*RLcp9_110;

// = = Block_1_0_0_10_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp9_110;
    sens->P[2] = POcp9_210;
    sens->P[3] = POcp9_310;
    sens->R[1][1] = ROcp9_110;
    sens->R[1][2] = ROcp9_210;
    sens->R[1][3] = ROcp9_310;
    sens->R[2][1] = ROcp9_49;
    sens->R[2][2] = ROcp9_59;
    sens->R[2][3] = ROcp9_69;
    sens->R[3][1] = ROcp9_710;
    sens->R[3][2] = ROcp9_810;
    sens->R[3][3] = ROcp9_910;
    sens->V[1] = VIcp9_110;
    sens->V[2] = VIcp9_210;
    sens->V[3] = VIcp9_310;
    sens->OM[1] = OMcp9_110;
    sens->OM[2] = OMcp9_210;
    sens->OM[3] = OMcp9_310;
    sens->A[1] = ACcp9_110;
    sens->A[2] = ACcp9_210;
    sens->A[3] = ACcp9_310;
    sens->OMP[1] = OPcp9_110;
    sens->OMP[2] = OPcp9_210;
    sens->OMP[3] = OPcp9_310;
 
// 
break;
case 11:
 


// = = Block_1_0_0_11_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp10_25 = qd[5]*C4;
    OMcp10_35 = qd[5]*S4;
    OMcp10_16 = qd[4]+qd[6]*S5;
    OMcp10_26 = OMcp10_25+ROcp10_85*qd[6];
    OMcp10_36 = OMcp10_35+ROcp10_95*qd[6];
    OPcp10_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp10_26 = ROcp10_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp10_35*S5-ROcp10_95*qd[4]);
    OPcp10_36 = ROcp10_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp10_25*S5-ROcp10_85*qd[4]);

// = = Block_1_0_0_11_0_2 = = 
 
// Sensor Kinematics 


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
    RLcp10_17 = ROcp10_46*s->dpt[2][1];
    RLcp10_27 = ROcp10_56*s->dpt[2][1];
    RLcp10_37 = ROcp10_66*s->dpt[2][1];
    OMcp10_17 = OMcp10_16+ROcp10_46*qd[7];
    OMcp10_27 = OMcp10_26+ROcp10_56*qd[7];
    OMcp10_37 = OMcp10_36+ROcp10_66*qd[7];
    ORcp10_17 = OMcp10_26*RLcp10_37-OMcp10_36*RLcp10_27;
    ORcp10_27 = -(OMcp10_16*RLcp10_37-OMcp10_36*RLcp10_17);
    ORcp10_37 = OMcp10_16*RLcp10_27-OMcp10_26*RLcp10_17;
    OPcp10_17 = OPcp10_16+ROcp10_46*qdd[7]+qd[7]*(OMcp10_26*ROcp10_66-OMcp10_36*ROcp10_56);
    OPcp10_27 = OPcp10_26+ROcp10_56*qdd[7]-qd[7]*(OMcp10_16*ROcp10_66-OMcp10_36*ROcp10_46);
    OPcp10_37 = OPcp10_36+ROcp10_66*qdd[7]+qd[7]*(OMcp10_16*ROcp10_56-OMcp10_26*ROcp10_46);
    RLcp10_18 = ROcp10_46*s->dpt[2][6];
    RLcp10_28 = ROcp10_56*s->dpt[2][6];
    RLcp10_38 = ROcp10_66*s->dpt[2][6];
    OMcp10_18 = OMcp10_17+ROcp10_17*qd[8];
    OMcp10_28 = OMcp10_27+ROcp10_27*qd[8];
    OMcp10_38 = OMcp10_37+ROcp10_37*qd[8];
    ORcp10_18 = OMcp10_27*RLcp10_38-OMcp10_37*RLcp10_28;
    ORcp10_28 = -(OMcp10_17*RLcp10_38-OMcp10_37*RLcp10_18);
    ORcp10_38 = OMcp10_17*RLcp10_28-OMcp10_27*RLcp10_18;
    OPcp10_18 = OPcp10_17+ROcp10_17*qdd[8]+qd[8]*(OMcp10_27*ROcp10_37-OMcp10_37*ROcp10_27);
    OPcp10_28 = OPcp10_27+ROcp10_27*qdd[8]-qd[8]*(OMcp10_17*ROcp10_37-OMcp10_37*ROcp10_17);
    OPcp10_38 = OPcp10_37+ROcp10_37*qdd[8]+qd[8]*(OMcp10_17*ROcp10_27-OMcp10_27*ROcp10_17);
    RLcp10_19 = ROcp10_78*s->dpt[3][8];
    RLcp10_29 = ROcp10_88*s->dpt[3][8];
    RLcp10_39 = ROcp10_98*s->dpt[3][8];
    OMcp10_19 = OMcp10_18+ROcp10_78*qd[9];
    OMcp10_29 = OMcp10_28+ROcp10_88*qd[9];
    OMcp10_39 = OMcp10_38+ROcp10_98*qd[9];
    ORcp10_19 = OMcp10_28*RLcp10_39-OMcp10_38*RLcp10_29;
    ORcp10_29 = -(OMcp10_18*RLcp10_39-OMcp10_38*RLcp10_19);
    ORcp10_39 = OMcp10_18*RLcp10_29-OMcp10_28*RLcp10_19;
    OPcp10_19 = OPcp10_18+ROcp10_78*qdd[9]+qd[9]*(OMcp10_28*ROcp10_98-OMcp10_38*ROcp10_88);
    OPcp10_29 = OPcp10_28+ROcp10_88*qdd[9]-qd[9]*(OMcp10_18*ROcp10_98-OMcp10_38*ROcp10_78);
    OPcp10_39 = OPcp10_38+ROcp10_98*qdd[9]+qd[9]*(OMcp10_18*ROcp10_88-OMcp10_28*ROcp10_78);
    RLcp10_110 = ROcp10_78*s->dpt[3][10];
    RLcp10_210 = ROcp10_88*s->dpt[3][10];
    RLcp10_310 = ROcp10_98*s->dpt[3][10];
    OMcp10_110 = OMcp10_19+ROcp10_49*qd[10];
    OMcp10_210 = OMcp10_29+ROcp10_59*qd[10];
    OMcp10_310 = OMcp10_39+ROcp10_69*qd[10];
    ORcp10_110 = OMcp10_29*RLcp10_310-OMcp10_39*RLcp10_210;
    ORcp10_210 = -(OMcp10_19*RLcp10_310-OMcp10_39*RLcp10_110);
    ORcp10_310 = OMcp10_19*RLcp10_210-OMcp10_29*RLcp10_110;
    OPcp10_110 = OPcp10_19+ROcp10_49*qdd[10]+qd[10]*(OMcp10_29*ROcp10_69-OMcp10_39*ROcp10_59);
    OPcp10_210 = OPcp10_29+ROcp10_59*qdd[10]-qd[10]*(OMcp10_19*ROcp10_69-OMcp10_39*ROcp10_49);
    OPcp10_310 = OPcp10_39+ROcp10_69*qdd[10]+qd[10]*(OMcp10_19*ROcp10_59-OMcp10_29*ROcp10_49);
    RLcp10_111 = ROcp10_710*s->dpt[3][12];
    RLcp10_211 = ROcp10_810*s->dpt[3][12];
    RLcp10_311 = ROcp10_910*s->dpt[3][12];
    POcp10_111 = RLcp10_110+RLcp10_111+RLcp10_17+RLcp10_18+RLcp10_19+q[1];
    POcp10_211 = RLcp10_210+RLcp10_211+RLcp10_27+RLcp10_28+RLcp10_29+q[2];
    POcp10_311 = RLcp10_310+RLcp10_311+RLcp10_37+RLcp10_38+RLcp10_39+q[3];
    OMcp10_111 = OMcp10_110+ROcp10_110*qd[11];
    OMcp10_211 = OMcp10_210+ROcp10_210*qd[11];
    OMcp10_311 = OMcp10_310+ROcp10_310*qd[11];
    ORcp10_111 = OMcp10_210*RLcp10_311-OMcp10_310*RLcp10_211;
    ORcp10_211 = -(OMcp10_110*RLcp10_311-OMcp10_310*RLcp10_111);
    ORcp10_311 = OMcp10_110*RLcp10_211-OMcp10_210*RLcp10_111;
    VIcp10_111 = ORcp10_110+ORcp10_111+ORcp10_17+ORcp10_18+ORcp10_19+qd[1];
    VIcp10_211 = ORcp10_210+ORcp10_211+ORcp10_27+ORcp10_28+ORcp10_29+qd[2];
    VIcp10_311 = ORcp10_310+ORcp10_311+ORcp10_37+ORcp10_38+ORcp10_39+qd[3];
    OPcp10_111 = OPcp10_110+ROcp10_110*qdd[11]+qd[11]*(OMcp10_210*ROcp10_310-OMcp10_310*ROcp10_210);
    OPcp10_211 = OPcp10_210+ROcp10_210*qdd[11]-qd[11]*(OMcp10_110*ROcp10_310-OMcp10_310*ROcp10_110);
    OPcp10_311 = OPcp10_310+ROcp10_310*qdd[11]+qd[11]*(OMcp10_110*ROcp10_210-OMcp10_210*ROcp10_110);
    ACcp10_111 = qdd[1]+OMcp10_210*ORcp10_311+OMcp10_26*ORcp10_37+OMcp10_27*ORcp10_38+OMcp10_28*ORcp10_39+OMcp10_29*
 ORcp10_310-OMcp10_310*ORcp10_211-OMcp10_36*ORcp10_27-OMcp10_37*ORcp10_28-OMcp10_38*ORcp10_29-OMcp10_39*ORcp10_210+OPcp10_210
 *RLcp10_311+OPcp10_26*RLcp10_37+OPcp10_27*RLcp10_38+OPcp10_28*RLcp10_39+OPcp10_29*RLcp10_310-OPcp10_310*RLcp10_211-OPcp10_36
 *RLcp10_27-OPcp10_37*RLcp10_28-OPcp10_38*RLcp10_29-OPcp10_39*RLcp10_210;
    ACcp10_211 = qdd[2]-OMcp10_110*ORcp10_311-OMcp10_16*ORcp10_37-OMcp10_17*ORcp10_38-OMcp10_18*ORcp10_39-OMcp10_19*
 ORcp10_310+OMcp10_310*ORcp10_111+OMcp10_36*ORcp10_17+OMcp10_37*ORcp10_18+OMcp10_38*ORcp10_19+OMcp10_39*ORcp10_110-OPcp10_110
 *RLcp10_311-OPcp10_16*RLcp10_37-OPcp10_17*RLcp10_38-OPcp10_18*RLcp10_39-OPcp10_19*RLcp10_310+OPcp10_310*RLcp10_111+OPcp10_36
 *RLcp10_17+OPcp10_37*RLcp10_18+OPcp10_38*RLcp10_19+OPcp10_39*RLcp10_110;
    ACcp10_311 = qdd[3]+OMcp10_110*ORcp10_211+OMcp10_16*ORcp10_27+OMcp10_17*ORcp10_28+OMcp10_18*ORcp10_29+OMcp10_19*
 ORcp10_210-OMcp10_210*ORcp10_111-OMcp10_26*ORcp10_17-OMcp10_27*ORcp10_18-OMcp10_28*ORcp10_19-OMcp10_29*ORcp10_110+OPcp10_110
 *RLcp10_211+OPcp10_16*RLcp10_27+OPcp10_17*RLcp10_28+OPcp10_18*RLcp10_29+OPcp10_19*RLcp10_210-OPcp10_210*RLcp10_111-OPcp10_26
 *RLcp10_17-OPcp10_27*RLcp10_18-OPcp10_28*RLcp10_19-OPcp10_29*RLcp10_110;

// = = Block_1_0_0_11_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp10_111;
    sens->P[2] = POcp10_211;
    sens->P[3] = POcp10_311;
    sens->R[1][1] = ROcp10_110;
    sens->R[1][2] = ROcp10_210;
    sens->R[1][3] = ROcp10_310;
    sens->R[2][1] = ROcp10_411;
    sens->R[2][2] = ROcp10_511;
    sens->R[2][3] = ROcp10_611;
    sens->R[3][1] = ROcp10_711;
    sens->R[3][2] = ROcp10_811;
    sens->R[3][3] = ROcp10_911;
    sens->V[1] = VIcp10_111;
    sens->V[2] = VIcp10_211;
    sens->V[3] = VIcp10_311;
    sens->OM[1] = OMcp10_111;
    sens->OM[2] = OMcp10_211;
    sens->OM[3] = OMcp10_311;
    sens->A[1] = ACcp10_111;
    sens->A[2] = ACcp10_211;
    sens->A[3] = ACcp10_311;
    sens->OMP[1] = OPcp10_111;
    sens->OMP[2] = OPcp10_211;
    sens->OMP[3] = OPcp10_311;
 
// 
break;
case 12:
 


// = = Block_1_0_0_12_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp11_25 = qd[5]*C4;
    OMcp11_35 = qd[5]*S4;
    OMcp11_16 = qd[4]+qd[6]*S5;
    OMcp11_26 = OMcp11_25+ROcp11_85*qd[6];
    OMcp11_36 = OMcp11_35+ROcp11_95*qd[6];
    OPcp11_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp11_26 = ROcp11_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp11_35*S5-ROcp11_95*qd[4]);
    OPcp11_36 = ROcp11_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp11_25*S5-ROcp11_85*qd[4]);

// = = Block_1_0_0_12_0_2 = = 
 
// Sensor Kinematics 


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
    RLcp11_17 = ROcp11_46*s->dpt[2][1];
    RLcp11_27 = ROcp11_56*s->dpt[2][1];
    RLcp11_37 = ROcp11_66*s->dpt[2][1];
    OMcp11_17 = OMcp11_16+ROcp11_46*qd[7];
    OMcp11_27 = OMcp11_26+ROcp11_56*qd[7];
    OMcp11_37 = OMcp11_36+ROcp11_66*qd[7];
    ORcp11_17 = OMcp11_26*RLcp11_37-OMcp11_36*RLcp11_27;
    ORcp11_27 = -(OMcp11_16*RLcp11_37-OMcp11_36*RLcp11_17);
    ORcp11_37 = OMcp11_16*RLcp11_27-OMcp11_26*RLcp11_17;
    OPcp11_17 = OPcp11_16+ROcp11_46*qdd[7]+qd[7]*(OMcp11_26*ROcp11_66-OMcp11_36*ROcp11_56);
    OPcp11_27 = OPcp11_26+ROcp11_56*qdd[7]-qd[7]*(OMcp11_16*ROcp11_66-OMcp11_36*ROcp11_46);
    OPcp11_37 = OPcp11_36+ROcp11_66*qdd[7]+qd[7]*(OMcp11_16*ROcp11_56-OMcp11_26*ROcp11_46);
    RLcp11_18 = ROcp11_46*s->dpt[2][6];
    RLcp11_28 = ROcp11_56*s->dpt[2][6];
    RLcp11_38 = ROcp11_66*s->dpt[2][6];
    OMcp11_18 = OMcp11_17+ROcp11_17*qd[8];
    OMcp11_28 = OMcp11_27+ROcp11_27*qd[8];
    OMcp11_38 = OMcp11_37+ROcp11_37*qd[8];
    ORcp11_18 = OMcp11_27*RLcp11_38-OMcp11_37*RLcp11_28;
    ORcp11_28 = -(OMcp11_17*RLcp11_38-OMcp11_37*RLcp11_18);
    ORcp11_38 = OMcp11_17*RLcp11_28-OMcp11_27*RLcp11_18;
    OPcp11_18 = OPcp11_17+ROcp11_17*qdd[8]+qd[8]*(OMcp11_27*ROcp11_37-OMcp11_37*ROcp11_27);
    OPcp11_28 = OPcp11_27+ROcp11_27*qdd[8]-qd[8]*(OMcp11_17*ROcp11_37-OMcp11_37*ROcp11_17);
    OPcp11_38 = OPcp11_37+ROcp11_37*qdd[8]+qd[8]*(OMcp11_17*ROcp11_27-OMcp11_27*ROcp11_17);
    RLcp11_19 = ROcp11_78*s->dpt[3][8];
    RLcp11_29 = ROcp11_88*s->dpt[3][8];
    RLcp11_39 = ROcp11_98*s->dpt[3][8];
    OMcp11_19 = OMcp11_18+ROcp11_78*qd[9];
    OMcp11_29 = OMcp11_28+ROcp11_88*qd[9];
    OMcp11_39 = OMcp11_38+ROcp11_98*qd[9];
    ORcp11_19 = OMcp11_28*RLcp11_39-OMcp11_38*RLcp11_29;
    ORcp11_29 = -(OMcp11_18*RLcp11_39-OMcp11_38*RLcp11_19);
    ORcp11_39 = OMcp11_18*RLcp11_29-OMcp11_28*RLcp11_19;
    OPcp11_19 = OPcp11_18+ROcp11_78*qdd[9]+qd[9]*(OMcp11_28*ROcp11_98-OMcp11_38*ROcp11_88);
    OPcp11_29 = OPcp11_28+ROcp11_88*qdd[9]-qd[9]*(OMcp11_18*ROcp11_98-OMcp11_38*ROcp11_78);
    OPcp11_39 = OPcp11_38+ROcp11_98*qdd[9]+qd[9]*(OMcp11_18*ROcp11_88-OMcp11_28*ROcp11_78);
    RLcp11_110 = ROcp11_78*s->dpt[3][10];
    RLcp11_210 = ROcp11_88*s->dpt[3][10];
    RLcp11_310 = ROcp11_98*s->dpt[3][10];
    OMcp11_110 = OMcp11_19+ROcp11_49*qd[10];
    OMcp11_210 = OMcp11_29+ROcp11_59*qd[10];
    OMcp11_310 = OMcp11_39+ROcp11_69*qd[10];
    ORcp11_110 = OMcp11_29*RLcp11_310-OMcp11_39*RLcp11_210;
    ORcp11_210 = -(OMcp11_19*RLcp11_310-OMcp11_39*RLcp11_110);
    ORcp11_310 = OMcp11_19*RLcp11_210-OMcp11_29*RLcp11_110;
    OPcp11_110 = OPcp11_19+ROcp11_49*qdd[10]+qd[10]*(OMcp11_29*ROcp11_69-OMcp11_39*ROcp11_59);
    OPcp11_210 = OPcp11_29+ROcp11_59*qdd[10]-qd[10]*(OMcp11_19*ROcp11_69-OMcp11_39*ROcp11_49);
    OPcp11_310 = OPcp11_39+ROcp11_69*qdd[10]+qd[10]*(OMcp11_19*ROcp11_59-OMcp11_29*ROcp11_49);
    RLcp11_111 = ROcp11_710*s->dpt[3][12];
    RLcp11_211 = ROcp11_810*s->dpt[3][12];
    RLcp11_311 = ROcp11_910*s->dpt[3][12];
    POcp11_111 = RLcp11_110+RLcp11_111+RLcp11_17+RLcp11_18+RLcp11_19+q[1];
    POcp11_211 = RLcp11_210+RLcp11_211+RLcp11_27+RLcp11_28+RLcp11_29+q[2];
    POcp11_311 = RLcp11_310+RLcp11_311+RLcp11_37+RLcp11_38+RLcp11_39+q[3];
    OMcp11_111 = OMcp11_110+ROcp11_110*qd[11];
    OMcp11_211 = OMcp11_210+ROcp11_210*qd[11];
    OMcp11_311 = OMcp11_310+ROcp11_310*qd[11];
    ORcp11_111 = OMcp11_210*RLcp11_311-OMcp11_310*RLcp11_211;
    ORcp11_211 = -(OMcp11_110*RLcp11_311-OMcp11_310*RLcp11_111);
    ORcp11_311 = OMcp11_110*RLcp11_211-OMcp11_210*RLcp11_111;
    VIcp11_111 = ORcp11_110+ORcp11_111+ORcp11_17+ORcp11_18+ORcp11_19+qd[1];
    VIcp11_211 = ORcp11_210+ORcp11_211+ORcp11_27+ORcp11_28+ORcp11_29+qd[2];
    VIcp11_311 = ORcp11_310+ORcp11_311+ORcp11_37+ORcp11_38+ORcp11_39+qd[3];
    ACcp11_111 = qdd[1]+OMcp11_210*ORcp11_311+OMcp11_26*ORcp11_37+OMcp11_27*ORcp11_38+OMcp11_28*ORcp11_39+OMcp11_29*
 ORcp11_310-OMcp11_310*ORcp11_211-OMcp11_36*ORcp11_27-OMcp11_37*ORcp11_28-OMcp11_38*ORcp11_29-OMcp11_39*ORcp11_210+OPcp11_210
 *RLcp11_311+OPcp11_26*RLcp11_37+OPcp11_27*RLcp11_38+OPcp11_28*RLcp11_39+OPcp11_29*RLcp11_310-OPcp11_310*RLcp11_211-OPcp11_36
 *RLcp11_27-OPcp11_37*RLcp11_28-OPcp11_38*RLcp11_29-OPcp11_39*RLcp11_210;
    ACcp11_211 = qdd[2]-OMcp11_110*ORcp11_311-OMcp11_16*ORcp11_37-OMcp11_17*ORcp11_38-OMcp11_18*ORcp11_39-OMcp11_19*
 ORcp11_310+OMcp11_310*ORcp11_111+OMcp11_36*ORcp11_17+OMcp11_37*ORcp11_18+OMcp11_38*ORcp11_19+OMcp11_39*ORcp11_110-OPcp11_110
 *RLcp11_311-OPcp11_16*RLcp11_37-OPcp11_17*RLcp11_38-OPcp11_18*RLcp11_39-OPcp11_19*RLcp11_310+OPcp11_310*RLcp11_111+OPcp11_36
 *RLcp11_17+OPcp11_37*RLcp11_18+OPcp11_38*RLcp11_19+OPcp11_39*RLcp11_110;
    ACcp11_311 = qdd[3]+OMcp11_110*ORcp11_211+OMcp11_16*ORcp11_27+OMcp11_17*ORcp11_28+OMcp11_18*ORcp11_29+OMcp11_19*
 ORcp11_210-OMcp11_210*ORcp11_111-OMcp11_26*ORcp11_17-OMcp11_27*ORcp11_18-OMcp11_28*ORcp11_19-OMcp11_29*ORcp11_110+OPcp11_110
 *RLcp11_211+OPcp11_16*RLcp11_27+OPcp11_17*RLcp11_28+OPcp11_18*RLcp11_29+OPcp11_19*RLcp11_210-OPcp11_210*RLcp11_111-OPcp11_26
 *RLcp11_17-OPcp11_27*RLcp11_18-OPcp11_28*RLcp11_19-OPcp11_29*RLcp11_110;
    OMcp11_112 = OMcp11_111+ROcp11_411*qd[12];
    OMcp11_212 = OMcp11_211+ROcp11_511*qd[12];
    OMcp11_312 = OMcp11_311+ROcp11_611*qd[12];
    OPcp11_112 = OPcp11_110+ROcp11_110*qdd[11]+ROcp11_411*qdd[12]+qd[11]*(OMcp11_210*ROcp11_310-OMcp11_310*ROcp11_210)+
 qd[12]*(OMcp11_211*ROcp11_611-OMcp11_311*ROcp11_511);
    OPcp11_212 = OPcp11_210+ROcp11_210*qdd[11]+ROcp11_511*qdd[12]-qd[11]*(OMcp11_110*ROcp11_310-OMcp11_310*ROcp11_110)-
 qd[12]*(OMcp11_111*ROcp11_611-OMcp11_311*ROcp11_411);
    OPcp11_312 = OPcp11_310+ROcp11_310*qdd[11]+ROcp11_611*qdd[12]+qd[11]*(OMcp11_110*ROcp11_210-OMcp11_210*ROcp11_110)+
 qd[12]*(OMcp11_111*ROcp11_511-OMcp11_211*ROcp11_411);

// = = Block_1_0_0_12_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp11_111;
    sens->P[2] = POcp11_211;
    sens->P[3] = POcp11_311;
    sens->R[1][1] = ROcp11_112;
    sens->R[1][2] = ROcp11_212;
    sens->R[1][3] = ROcp11_312;
    sens->R[2][1] = ROcp11_411;
    sens->R[2][2] = ROcp11_511;
    sens->R[2][3] = ROcp11_611;
    sens->R[3][1] = ROcp11_712;
    sens->R[3][2] = ROcp11_812;
    sens->R[3][3] = ROcp11_912;
    sens->V[1] = VIcp11_111;
    sens->V[2] = VIcp11_211;
    sens->V[3] = VIcp11_311;
    sens->OM[1] = OMcp11_112;
    sens->OM[2] = OMcp11_212;
    sens->OM[3] = OMcp11_312;
    sens->A[1] = ACcp11_111;
    sens->A[2] = ACcp11_211;
    sens->A[3] = ACcp11_311;
    sens->OMP[1] = OPcp11_112;
    sens->OMP[2] = OPcp11_212;
    sens->OMP[3] = OPcp11_312;
 
// 
break;
case 13:
 


// = = Block_1_0_0_13_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp12_25 = qd[5]*C4;
    OMcp12_35 = qd[5]*S4;
    OMcp12_16 = qd[4]+qd[6]*S5;
    OMcp12_26 = OMcp12_25+ROcp12_85*qd[6];
    OMcp12_36 = OMcp12_35+ROcp12_95*qd[6];
    OPcp12_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp12_26 = ROcp12_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp12_35*S5-ROcp12_95*qd[4]);
    OPcp12_36 = ROcp12_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp12_25*S5-ROcp12_85*qd[4]);

// = = Block_1_0_0_13_0_2 = = 
 
// Sensor Kinematics 


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
    ROcp12_411 = ROcp12_49*C11+ROcp12_710*S11;
    ROcp12_511 = ROcp12_59*C11+ROcp12_810*S11;
    ROcp12_611 = ROcp12_69*C11+ROcp12_910*S11;
    ROcp12_711 = -(ROcp12_49*S11-ROcp12_710*C11);
    ROcp12_811 = -(ROcp12_59*S11-ROcp12_810*C11);
    ROcp12_911 = -(ROcp12_69*S11-ROcp12_910*C11);
    ROcp12_112 = ROcp12_110*C12-ROcp12_711*S12;
    ROcp12_212 = ROcp12_210*C12-ROcp12_811*S12;
    ROcp12_312 = ROcp12_310*C12-ROcp12_911*S12;
    ROcp12_712 = ROcp12_110*S12+ROcp12_711*C12;
    ROcp12_812 = ROcp12_210*S12+ROcp12_811*C12;
    ROcp12_912 = ROcp12_310*S12+ROcp12_911*C12;
    ROcp12_113 = ROcp12_112*C13-ROcp12_712*S13;
    ROcp12_213 = ROcp12_212*C13-ROcp12_812*S13;
    ROcp12_313 = ROcp12_312*C13-ROcp12_912*S13;
    ROcp12_713 = ROcp12_112*S13+ROcp12_712*C13;
    ROcp12_813 = ROcp12_212*S13+ROcp12_812*C13;
    ROcp12_913 = ROcp12_312*S13+ROcp12_912*C13;
    RLcp12_17 = ROcp12_46*s->dpt[2][1];
    RLcp12_27 = ROcp12_56*s->dpt[2][1];
    RLcp12_37 = ROcp12_66*s->dpt[2][1];
    OMcp12_17 = OMcp12_16+ROcp12_46*qd[7];
    OMcp12_27 = OMcp12_26+ROcp12_56*qd[7];
    OMcp12_37 = OMcp12_36+ROcp12_66*qd[7];
    ORcp12_17 = OMcp12_26*RLcp12_37-OMcp12_36*RLcp12_27;
    ORcp12_27 = -(OMcp12_16*RLcp12_37-OMcp12_36*RLcp12_17);
    ORcp12_37 = OMcp12_16*RLcp12_27-OMcp12_26*RLcp12_17;
    OPcp12_17 = OPcp12_16+ROcp12_46*qdd[7]+qd[7]*(OMcp12_26*ROcp12_66-OMcp12_36*ROcp12_56);
    OPcp12_27 = OPcp12_26+ROcp12_56*qdd[7]-qd[7]*(OMcp12_16*ROcp12_66-OMcp12_36*ROcp12_46);
    OPcp12_37 = OPcp12_36+ROcp12_66*qdd[7]+qd[7]*(OMcp12_16*ROcp12_56-OMcp12_26*ROcp12_46);
    RLcp12_18 = ROcp12_46*s->dpt[2][6];
    RLcp12_28 = ROcp12_56*s->dpt[2][6];
    RLcp12_38 = ROcp12_66*s->dpt[2][6];
    OMcp12_18 = OMcp12_17+ROcp12_17*qd[8];
    OMcp12_28 = OMcp12_27+ROcp12_27*qd[8];
    OMcp12_38 = OMcp12_37+ROcp12_37*qd[8];
    ORcp12_18 = OMcp12_27*RLcp12_38-OMcp12_37*RLcp12_28;
    ORcp12_28 = -(OMcp12_17*RLcp12_38-OMcp12_37*RLcp12_18);
    ORcp12_38 = OMcp12_17*RLcp12_28-OMcp12_27*RLcp12_18;
    OPcp12_18 = OPcp12_17+ROcp12_17*qdd[8]+qd[8]*(OMcp12_27*ROcp12_37-OMcp12_37*ROcp12_27);
    OPcp12_28 = OPcp12_27+ROcp12_27*qdd[8]-qd[8]*(OMcp12_17*ROcp12_37-OMcp12_37*ROcp12_17);
    OPcp12_38 = OPcp12_37+ROcp12_37*qdd[8]+qd[8]*(OMcp12_17*ROcp12_27-OMcp12_27*ROcp12_17);
    RLcp12_19 = ROcp12_78*s->dpt[3][8];
    RLcp12_29 = ROcp12_88*s->dpt[3][8];
    RLcp12_39 = ROcp12_98*s->dpt[3][8];
    OMcp12_19 = OMcp12_18+ROcp12_78*qd[9];
    OMcp12_29 = OMcp12_28+ROcp12_88*qd[9];
    OMcp12_39 = OMcp12_38+ROcp12_98*qd[9];
    ORcp12_19 = OMcp12_28*RLcp12_39-OMcp12_38*RLcp12_29;
    ORcp12_29 = -(OMcp12_18*RLcp12_39-OMcp12_38*RLcp12_19);
    ORcp12_39 = OMcp12_18*RLcp12_29-OMcp12_28*RLcp12_19;
    OPcp12_19 = OPcp12_18+ROcp12_78*qdd[9]+qd[9]*(OMcp12_28*ROcp12_98-OMcp12_38*ROcp12_88);
    OPcp12_29 = OPcp12_28+ROcp12_88*qdd[9]-qd[9]*(OMcp12_18*ROcp12_98-OMcp12_38*ROcp12_78);
    OPcp12_39 = OPcp12_38+ROcp12_98*qdd[9]+qd[9]*(OMcp12_18*ROcp12_88-OMcp12_28*ROcp12_78);
    RLcp12_110 = ROcp12_78*s->dpt[3][10];
    RLcp12_210 = ROcp12_88*s->dpt[3][10];
    RLcp12_310 = ROcp12_98*s->dpt[3][10];
    OMcp12_110 = OMcp12_19+ROcp12_49*qd[10];
    OMcp12_210 = OMcp12_29+ROcp12_59*qd[10];
    OMcp12_310 = OMcp12_39+ROcp12_69*qd[10];
    ORcp12_110 = OMcp12_29*RLcp12_310-OMcp12_39*RLcp12_210;
    ORcp12_210 = -(OMcp12_19*RLcp12_310-OMcp12_39*RLcp12_110);
    ORcp12_310 = OMcp12_19*RLcp12_210-OMcp12_29*RLcp12_110;
    OPcp12_110 = OPcp12_19+ROcp12_49*qdd[10]+qd[10]*(OMcp12_29*ROcp12_69-OMcp12_39*ROcp12_59);
    OPcp12_210 = OPcp12_29+ROcp12_59*qdd[10]-qd[10]*(OMcp12_19*ROcp12_69-OMcp12_39*ROcp12_49);
    OPcp12_310 = OPcp12_39+ROcp12_69*qdd[10]+qd[10]*(OMcp12_19*ROcp12_59-OMcp12_29*ROcp12_49);
    RLcp12_111 = ROcp12_710*s->dpt[3][12];
    RLcp12_211 = ROcp12_810*s->dpt[3][12];
    RLcp12_311 = ROcp12_910*s->dpt[3][12];
    OMcp12_111 = OMcp12_110+ROcp12_110*qd[11];
    OMcp12_211 = OMcp12_210+ROcp12_210*qd[11];
    OMcp12_311 = OMcp12_310+ROcp12_310*qd[11];
    ORcp12_111 = OMcp12_210*RLcp12_311-OMcp12_310*RLcp12_211;
    ORcp12_211 = -(OMcp12_110*RLcp12_311-OMcp12_310*RLcp12_111);
    ORcp12_311 = OMcp12_110*RLcp12_211-OMcp12_210*RLcp12_111;
    OMcp12_112 = OMcp12_111+ROcp12_411*qd[12];
    OMcp12_212 = OMcp12_211+ROcp12_511*qd[12];
    OMcp12_312 = OMcp12_311+ROcp12_611*qd[12];
    OPcp12_112 = OPcp12_110+ROcp12_110*qdd[11]+ROcp12_411*qdd[12]+qd[11]*(OMcp12_210*ROcp12_310-OMcp12_310*ROcp12_210)+
 qd[12]*(OMcp12_211*ROcp12_611-OMcp12_311*ROcp12_511);
    OPcp12_212 = OPcp12_210+ROcp12_210*qdd[11]+ROcp12_511*qdd[12]-qd[11]*(OMcp12_110*ROcp12_310-OMcp12_310*ROcp12_110)-
 qd[12]*(OMcp12_111*ROcp12_611-OMcp12_311*ROcp12_411);
    OPcp12_312 = OPcp12_310+ROcp12_310*qdd[11]+ROcp12_611*qdd[12]+qd[11]*(OMcp12_110*ROcp12_210-OMcp12_210*ROcp12_110)+
 qd[12]*(OMcp12_111*ROcp12_511-OMcp12_211*ROcp12_411);
    RLcp12_113 = ROcp12_112*s->dpt[1][18]+ROcp12_712*s->dpt[3][18];
    RLcp12_213 = ROcp12_212*s->dpt[1][18]+ROcp12_812*s->dpt[3][18];
    RLcp12_313 = ROcp12_312*s->dpt[1][18]+ROcp12_912*s->dpt[3][18];
    POcp12_113 = RLcp12_110+RLcp12_111+RLcp12_113+RLcp12_17+RLcp12_18+RLcp12_19+q[1];
    POcp12_213 = RLcp12_210+RLcp12_211+RLcp12_213+RLcp12_27+RLcp12_28+RLcp12_29+q[2];
    POcp12_313 = RLcp12_310+RLcp12_311+RLcp12_313+RLcp12_37+RLcp12_38+RLcp12_39+q[3];
    OMcp12_113 = OMcp12_112+ROcp12_411*qd[13];
    OMcp12_213 = OMcp12_212+ROcp12_511*qd[13];
    OMcp12_313 = OMcp12_312+ROcp12_611*qd[13];
    ORcp12_113 = OMcp12_212*RLcp12_313-OMcp12_312*RLcp12_213;
    ORcp12_213 = -(OMcp12_112*RLcp12_313-OMcp12_312*RLcp12_113);
    ORcp12_313 = OMcp12_112*RLcp12_213-OMcp12_212*RLcp12_113;
    VIcp12_113 = ORcp12_110+ORcp12_111+ORcp12_113+ORcp12_17+ORcp12_18+ORcp12_19+qd[1];
    VIcp12_213 = ORcp12_210+ORcp12_211+ORcp12_213+ORcp12_27+ORcp12_28+ORcp12_29+qd[2];
    VIcp12_313 = ORcp12_310+ORcp12_311+ORcp12_313+ORcp12_37+ORcp12_38+ORcp12_39+qd[3];
    OPcp12_113 = OPcp12_112+ROcp12_411*qdd[13]+qd[13]*(OMcp12_212*ROcp12_611-OMcp12_312*ROcp12_511);
    OPcp12_213 = OPcp12_212+ROcp12_511*qdd[13]-qd[13]*(OMcp12_112*ROcp12_611-OMcp12_312*ROcp12_411);
    OPcp12_313 = OPcp12_312+ROcp12_611*qdd[13]+qd[13]*(OMcp12_112*ROcp12_511-OMcp12_212*ROcp12_411);
    ACcp12_113 = qdd[1]+OMcp12_210*ORcp12_311+OMcp12_212*ORcp12_313+OMcp12_26*ORcp12_37+OMcp12_27*ORcp12_38+OMcp12_28*
 ORcp12_39+OMcp12_29*ORcp12_310-OMcp12_310*ORcp12_211-OMcp12_312*ORcp12_213-OMcp12_36*ORcp12_27-OMcp12_37*ORcp12_28-OMcp12_38
 *ORcp12_29-OMcp12_39*ORcp12_210+OPcp12_210*RLcp12_311+OPcp12_212*RLcp12_313+OPcp12_26*RLcp12_37+OPcp12_27*RLcp12_38+
 OPcp12_28*RLcp12_39+OPcp12_29*RLcp12_310-OPcp12_310*RLcp12_211-OPcp12_312*RLcp12_213-OPcp12_36*RLcp12_27-OPcp12_37*RLcp12_28
 -OPcp12_38*RLcp12_29-OPcp12_39*RLcp12_210;
    ACcp12_213 = qdd[2]-OMcp12_110*ORcp12_311-OMcp12_112*ORcp12_313-OMcp12_16*ORcp12_37-OMcp12_17*ORcp12_38-OMcp12_18*
 ORcp12_39-OMcp12_19*ORcp12_310+OMcp12_310*ORcp12_111+OMcp12_312*ORcp12_113+OMcp12_36*ORcp12_17+OMcp12_37*ORcp12_18+OMcp12_38
 *ORcp12_19+OMcp12_39*ORcp12_110-OPcp12_110*RLcp12_311-OPcp12_112*RLcp12_313-OPcp12_16*RLcp12_37-OPcp12_17*RLcp12_38-
 OPcp12_18*RLcp12_39-OPcp12_19*RLcp12_310+OPcp12_310*RLcp12_111+OPcp12_312*RLcp12_113+OPcp12_36*RLcp12_17+OPcp12_37*RLcp12_18
 +OPcp12_38*RLcp12_19+OPcp12_39*RLcp12_110;
    ACcp12_313 = qdd[3]+OMcp12_110*ORcp12_211+OMcp12_112*ORcp12_213+OMcp12_16*ORcp12_27+OMcp12_17*ORcp12_28+OMcp12_18*
 ORcp12_29+OMcp12_19*ORcp12_210-OMcp12_210*ORcp12_111-OMcp12_212*ORcp12_113-OMcp12_26*ORcp12_17-OMcp12_27*ORcp12_18-OMcp12_28
 *ORcp12_19-OMcp12_29*ORcp12_110+OPcp12_110*RLcp12_211+OPcp12_112*RLcp12_213+OPcp12_16*RLcp12_27+OPcp12_17*RLcp12_28+
 OPcp12_18*RLcp12_29+OPcp12_19*RLcp12_210-OPcp12_210*RLcp12_111-OPcp12_212*RLcp12_113-OPcp12_26*RLcp12_17-OPcp12_27*RLcp12_18
 -OPcp12_28*RLcp12_19-OPcp12_29*RLcp12_110;

// = = Block_1_0_0_13_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp12_113;
    sens->P[2] = POcp12_213;
    sens->P[3] = POcp12_313;
    sens->R[1][1] = ROcp12_113;
    sens->R[1][2] = ROcp12_213;
    sens->R[1][3] = ROcp12_313;
    sens->R[2][1] = ROcp12_411;
    sens->R[2][2] = ROcp12_511;
    sens->R[2][3] = ROcp12_611;
    sens->R[3][1] = ROcp12_713;
    sens->R[3][2] = ROcp12_813;
    sens->R[3][3] = ROcp12_913;
    sens->V[1] = VIcp12_113;
    sens->V[2] = VIcp12_213;
    sens->V[3] = VIcp12_313;
    sens->OM[1] = OMcp12_113;
    sens->OM[2] = OMcp12_213;
    sens->OM[3] = OMcp12_313;
    sens->A[1] = ACcp12_113;
    sens->A[2] = ACcp12_213;
    sens->A[3] = ACcp12_313;
    sens->OMP[1] = OPcp12_113;
    sens->OMP[2] = OPcp12_213;
    sens->OMP[3] = OPcp12_313;
 
// 
break;
case 14:
 


// = = Block_1_0_0_14_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp13_25 = qd[5]*C4;
    OMcp13_35 = qd[5]*S4;
    OMcp13_16 = qd[4]+qd[6]*S5;
    OMcp13_26 = OMcp13_25+ROcp13_85*qd[6];
    OMcp13_36 = OMcp13_35+ROcp13_95*qd[6];
    OPcp13_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp13_26 = ROcp13_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp13_35*S5-ROcp13_95*qd[4]);
    OPcp13_36 = ROcp13_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp13_25*S5-ROcp13_85*qd[4]);

// = = Block_1_0_0_14_0_3 = = 
 
// Sensor Kinematics 


    ROcp13_114 = ROcp13_16*C14-S14*S5;
    ROcp13_214 = ROcp13_26*C14-ROcp13_85*S14;
    ROcp13_314 = ROcp13_36*C14-ROcp13_95*S14;
    ROcp13_714 = ROcp13_16*S14+C14*S5;
    ROcp13_814 = ROcp13_26*S14+ROcp13_85*C14;
    ROcp13_914 = ROcp13_36*S14+ROcp13_95*C14;
    RLcp13_114 = ROcp13_46*s->dpt[2][2];
    RLcp13_214 = ROcp13_56*s->dpt[2][2];
    RLcp13_314 = ROcp13_66*s->dpt[2][2];
    POcp13_114 = RLcp13_114+q[1];
    POcp13_214 = RLcp13_214+q[2];
    POcp13_314 = RLcp13_314+q[3];
    OMcp13_114 = OMcp13_16+ROcp13_46*qd[14];
    OMcp13_214 = OMcp13_26+ROcp13_56*qd[14];
    OMcp13_314 = OMcp13_36+ROcp13_66*qd[14];
    ORcp13_114 = OMcp13_26*RLcp13_314-OMcp13_36*RLcp13_214;
    ORcp13_214 = -(OMcp13_16*RLcp13_314-OMcp13_36*RLcp13_114);
    ORcp13_314 = OMcp13_16*RLcp13_214-OMcp13_26*RLcp13_114;
    VIcp13_114 = ORcp13_114+qd[1];
    VIcp13_214 = ORcp13_214+qd[2];
    VIcp13_314 = ORcp13_314+qd[3];
    OPcp13_114 = OPcp13_16+ROcp13_46*qdd[14]+qd[14]*(OMcp13_26*ROcp13_66-OMcp13_36*ROcp13_56);
    OPcp13_214 = OPcp13_26+ROcp13_56*qdd[14]-qd[14]*(OMcp13_16*ROcp13_66-OMcp13_36*ROcp13_46);
    OPcp13_314 = OPcp13_36+ROcp13_66*qdd[14]+qd[14]*(OMcp13_16*ROcp13_56-OMcp13_26*ROcp13_46);
    ACcp13_114 = qdd[1]+OMcp13_26*ORcp13_314-OMcp13_36*ORcp13_214+OPcp13_26*RLcp13_314-OPcp13_36*RLcp13_214;
    ACcp13_214 = qdd[2]-OMcp13_16*ORcp13_314+OMcp13_36*ORcp13_114-OPcp13_16*RLcp13_314+OPcp13_36*RLcp13_114;
    ACcp13_314 = qdd[3]+OMcp13_16*ORcp13_214-OMcp13_26*ORcp13_114+OPcp13_16*RLcp13_214-OPcp13_26*RLcp13_114;

// = = Block_1_0_0_14_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp13_114;
    sens->P[2] = POcp13_214;
    sens->P[3] = POcp13_314;
    sens->R[1][1] = ROcp13_114;
    sens->R[1][2] = ROcp13_214;
    sens->R[1][3] = ROcp13_314;
    sens->R[2][1] = ROcp13_46;
    sens->R[2][2] = ROcp13_56;
    sens->R[2][3] = ROcp13_66;
    sens->R[3][1] = ROcp13_714;
    sens->R[3][2] = ROcp13_814;
    sens->R[3][3] = ROcp13_914;
    sens->V[1] = VIcp13_114;
    sens->V[2] = VIcp13_214;
    sens->V[3] = VIcp13_314;
    sens->OM[1] = OMcp13_114;
    sens->OM[2] = OMcp13_214;
    sens->OM[3] = OMcp13_314;
    sens->A[1] = ACcp13_114;
    sens->A[2] = ACcp13_214;
    sens->A[3] = ACcp13_314;
    sens->OMP[1] = OPcp13_114;
    sens->OMP[2] = OPcp13_214;
    sens->OMP[3] = OPcp13_314;
 
// 
break;
case 15:
 


// = = Block_1_0_0_15_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp14_25 = qd[5]*C4;
    OMcp14_35 = qd[5]*S4;
    OMcp14_16 = qd[4]+qd[6]*S5;
    OMcp14_26 = OMcp14_25+ROcp14_85*qd[6];
    OMcp14_36 = OMcp14_35+ROcp14_95*qd[6];
    OPcp14_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp14_26 = ROcp14_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp14_35*S5-ROcp14_95*qd[4]);
    OPcp14_36 = ROcp14_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp14_25*S5-ROcp14_85*qd[4]);

// = = Block_1_0_0_15_0_3 = = 
 
// Sensor Kinematics 


    ROcp14_114 = ROcp14_16*C14-S14*S5;
    ROcp14_214 = ROcp14_26*C14-ROcp14_85*S14;
    ROcp14_314 = ROcp14_36*C14-ROcp14_95*S14;
    ROcp14_714 = ROcp14_16*S14+C14*S5;
    ROcp14_814 = ROcp14_26*S14+ROcp14_85*C14;
    ROcp14_914 = ROcp14_36*S14+ROcp14_95*C14;
    ROcp14_415 = ROcp14_46*C15+ROcp14_714*S15;
    ROcp14_515 = ROcp14_56*C15+ROcp14_814*S15;
    ROcp14_615 = ROcp14_66*C15+ROcp14_914*S15;
    ROcp14_715 = -(ROcp14_46*S15-ROcp14_714*C15);
    ROcp14_815 = -(ROcp14_56*S15-ROcp14_814*C15);
    ROcp14_915 = -(ROcp14_66*S15-ROcp14_914*C15);
    RLcp14_114 = ROcp14_46*s->dpt[2][2];
    RLcp14_214 = ROcp14_56*s->dpt[2][2];
    RLcp14_314 = ROcp14_66*s->dpt[2][2];
    OMcp14_114 = OMcp14_16+ROcp14_46*qd[14];
    OMcp14_214 = OMcp14_26+ROcp14_56*qd[14];
    OMcp14_314 = OMcp14_36+ROcp14_66*qd[14];
    ORcp14_114 = OMcp14_26*RLcp14_314-OMcp14_36*RLcp14_214;
    ORcp14_214 = -(OMcp14_16*RLcp14_314-OMcp14_36*RLcp14_114);
    ORcp14_314 = OMcp14_16*RLcp14_214-OMcp14_26*RLcp14_114;
    OPcp14_114 = OPcp14_16+ROcp14_46*qdd[14]+qd[14]*(OMcp14_26*ROcp14_66-OMcp14_36*ROcp14_56);
    OPcp14_214 = OPcp14_26+ROcp14_56*qdd[14]-qd[14]*(OMcp14_16*ROcp14_66-OMcp14_36*ROcp14_46);
    OPcp14_314 = OPcp14_36+ROcp14_66*qdd[14]+qd[14]*(OMcp14_16*ROcp14_56-OMcp14_26*ROcp14_46);
    RLcp14_115 = ROcp14_46*s->dpt[2][20];
    RLcp14_215 = ROcp14_56*s->dpt[2][20];
    RLcp14_315 = ROcp14_66*s->dpt[2][20];
    POcp14_115 = RLcp14_114+RLcp14_115+q[1];
    POcp14_215 = RLcp14_214+RLcp14_215+q[2];
    POcp14_315 = RLcp14_314+RLcp14_315+q[3];
    OMcp14_115 = OMcp14_114+ROcp14_114*qd[15];
    OMcp14_215 = OMcp14_214+ROcp14_214*qd[15];
    OMcp14_315 = OMcp14_314+ROcp14_314*qd[15];
    ORcp14_115 = OMcp14_214*RLcp14_315-OMcp14_314*RLcp14_215;
    ORcp14_215 = -(OMcp14_114*RLcp14_315-OMcp14_314*RLcp14_115);
    ORcp14_315 = OMcp14_114*RLcp14_215-OMcp14_214*RLcp14_115;
    VIcp14_115 = ORcp14_114+ORcp14_115+qd[1];
    VIcp14_215 = ORcp14_214+ORcp14_215+qd[2];
    VIcp14_315 = ORcp14_314+ORcp14_315+qd[3];
    OPcp14_115 = OPcp14_114+ROcp14_114*qdd[15]+qd[15]*(OMcp14_214*ROcp14_314-OMcp14_314*ROcp14_214);
    OPcp14_215 = OPcp14_214+ROcp14_214*qdd[15]-qd[15]*(OMcp14_114*ROcp14_314-OMcp14_314*ROcp14_114);
    OPcp14_315 = OPcp14_314+ROcp14_314*qdd[15]+qd[15]*(OMcp14_114*ROcp14_214-OMcp14_214*ROcp14_114);
    ACcp14_115 = qdd[1]+OMcp14_214*ORcp14_315+OMcp14_26*ORcp14_314-OMcp14_314*ORcp14_215-OMcp14_36*ORcp14_214+OPcp14_214*
 RLcp14_315+OPcp14_26*RLcp14_314-OPcp14_314*RLcp14_215-OPcp14_36*RLcp14_214;
    ACcp14_215 = qdd[2]-OMcp14_114*ORcp14_315-OMcp14_16*ORcp14_314+OMcp14_314*ORcp14_115+OMcp14_36*ORcp14_114-OPcp14_114*
 RLcp14_315-OPcp14_16*RLcp14_314+OPcp14_314*RLcp14_115+OPcp14_36*RLcp14_114;
    ACcp14_315 = qdd[3]+OMcp14_114*ORcp14_215+OMcp14_16*ORcp14_214-OMcp14_214*ORcp14_115-OMcp14_26*ORcp14_114+OPcp14_114*
 RLcp14_215+OPcp14_16*RLcp14_214-OPcp14_214*RLcp14_115-OPcp14_26*RLcp14_114;

// = = Block_1_0_0_15_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp14_115;
    sens->P[2] = POcp14_215;
    sens->P[3] = POcp14_315;
    sens->R[1][1] = ROcp14_114;
    sens->R[1][2] = ROcp14_214;
    sens->R[1][3] = ROcp14_314;
    sens->R[2][1] = ROcp14_415;
    sens->R[2][2] = ROcp14_515;
    sens->R[2][3] = ROcp14_615;
    sens->R[3][1] = ROcp14_715;
    sens->R[3][2] = ROcp14_815;
    sens->R[3][3] = ROcp14_915;
    sens->V[1] = VIcp14_115;
    sens->V[2] = VIcp14_215;
    sens->V[3] = VIcp14_315;
    sens->OM[1] = OMcp14_115;
    sens->OM[2] = OMcp14_215;
    sens->OM[3] = OMcp14_315;
    sens->A[1] = ACcp14_115;
    sens->A[2] = ACcp14_215;
    sens->A[3] = ACcp14_315;
    sens->OMP[1] = OPcp14_115;
    sens->OMP[2] = OPcp14_215;
    sens->OMP[3] = OPcp14_315;
 
// 
break;
case 16:
 


// = = Block_1_0_0_16_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp15_25 = qd[5]*C4;
    OMcp15_35 = qd[5]*S4;
    OMcp15_16 = qd[4]+qd[6]*S5;
    OMcp15_26 = OMcp15_25+ROcp15_85*qd[6];
    OMcp15_36 = OMcp15_35+ROcp15_95*qd[6];
    OPcp15_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp15_26 = ROcp15_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp15_35*S5-ROcp15_95*qd[4]);
    OPcp15_36 = ROcp15_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp15_25*S5-ROcp15_85*qd[4]);

// = = Block_1_0_0_16_0_3 = = 
 
// Sensor Kinematics 


    ROcp15_114 = ROcp15_16*C14-S14*S5;
    ROcp15_214 = ROcp15_26*C14-ROcp15_85*S14;
    ROcp15_314 = ROcp15_36*C14-ROcp15_95*S14;
    ROcp15_714 = ROcp15_16*S14+C14*S5;
    ROcp15_814 = ROcp15_26*S14+ROcp15_85*C14;
    ROcp15_914 = ROcp15_36*S14+ROcp15_95*C14;
    ROcp15_415 = ROcp15_46*C15+ROcp15_714*S15;
    ROcp15_515 = ROcp15_56*C15+ROcp15_814*S15;
    ROcp15_615 = ROcp15_66*C15+ROcp15_914*S15;
    ROcp15_715 = -(ROcp15_46*S15-ROcp15_714*C15);
    ROcp15_815 = -(ROcp15_56*S15-ROcp15_814*C15);
    ROcp15_915 = -(ROcp15_66*S15-ROcp15_914*C15);
    ROcp15_116 = ROcp15_114*C16+ROcp15_415*S16;
    ROcp15_216 = ROcp15_214*C16+ROcp15_515*S16;
    ROcp15_316 = ROcp15_314*C16+ROcp15_615*S16;
    ROcp15_416 = -(ROcp15_114*S16-ROcp15_415*C16);
    ROcp15_516 = -(ROcp15_214*S16-ROcp15_515*C16);
    ROcp15_616 = -(ROcp15_314*S16-ROcp15_615*C16);
    RLcp15_114 = ROcp15_46*s->dpt[2][2];
    RLcp15_214 = ROcp15_56*s->dpt[2][2];
    RLcp15_314 = ROcp15_66*s->dpt[2][2];
    OMcp15_114 = OMcp15_16+ROcp15_46*qd[14];
    OMcp15_214 = OMcp15_26+ROcp15_56*qd[14];
    OMcp15_314 = OMcp15_36+ROcp15_66*qd[14];
    ORcp15_114 = OMcp15_26*RLcp15_314-OMcp15_36*RLcp15_214;
    ORcp15_214 = -(OMcp15_16*RLcp15_314-OMcp15_36*RLcp15_114);
    ORcp15_314 = OMcp15_16*RLcp15_214-OMcp15_26*RLcp15_114;
    OPcp15_114 = OPcp15_16+ROcp15_46*qdd[14]+qd[14]*(OMcp15_26*ROcp15_66-OMcp15_36*ROcp15_56);
    OPcp15_214 = OPcp15_26+ROcp15_56*qdd[14]-qd[14]*(OMcp15_16*ROcp15_66-OMcp15_36*ROcp15_46);
    OPcp15_314 = OPcp15_36+ROcp15_66*qdd[14]+qd[14]*(OMcp15_16*ROcp15_56-OMcp15_26*ROcp15_46);
    RLcp15_115 = ROcp15_46*s->dpt[2][20];
    RLcp15_215 = ROcp15_56*s->dpt[2][20];
    RLcp15_315 = ROcp15_66*s->dpt[2][20];
    OMcp15_115 = OMcp15_114+ROcp15_114*qd[15];
    OMcp15_215 = OMcp15_214+ROcp15_214*qd[15];
    OMcp15_315 = OMcp15_314+ROcp15_314*qd[15];
    ORcp15_115 = OMcp15_214*RLcp15_315-OMcp15_314*RLcp15_215;
    ORcp15_215 = -(OMcp15_114*RLcp15_315-OMcp15_314*RLcp15_115);
    ORcp15_315 = OMcp15_114*RLcp15_215-OMcp15_214*RLcp15_115;
    OPcp15_115 = OPcp15_114+ROcp15_114*qdd[15]+qd[15]*(OMcp15_214*ROcp15_314-OMcp15_314*ROcp15_214);
    OPcp15_215 = OPcp15_214+ROcp15_214*qdd[15]-qd[15]*(OMcp15_114*ROcp15_314-OMcp15_314*ROcp15_114);
    OPcp15_315 = OPcp15_314+ROcp15_314*qdd[15]+qd[15]*(OMcp15_114*ROcp15_214-OMcp15_214*ROcp15_114);
    RLcp15_116 = ROcp15_715*s->dpt[3][22];
    RLcp15_216 = ROcp15_815*s->dpt[3][22];
    RLcp15_316 = ROcp15_915*s->dpt[3][22];
    POcp15_116 = RLcp15_114+RLcp15_115+RLcp15_116+q[1];
    POcp15_216 = RLcp15_214+RLcp15_215+RLcp15_216+q[2];
    POcp15_316 = RLcp15_314+RLcp15_315+RLcp15_316+q[3];
    OMcp15_116 = OMcp15_115+ROcp15_715*qd[16];
    OMcp15_216 = OMcp15_215+ROcp15_815*qd[16];
    OMcp15_316 = OMcp15_315+ROcp15_915*qd[16];
    ORcp15_116 = OMcp15_215*RLcp15_316-OMcp15_315*RLcp15_216;
    ORcp15_216 = -(OMcp15_115*RLcp15_316-OMcp15_315*RLcp15_116);
    ORcp15_316 = OMcp15_115*RLcp15_216-OMcp15_215*RLcp15_116;
    VIcp15_116 = ORcp15_114+ORcp15_115+ORcp15_116+qd[1];
    VIcp15_216 = ORcp15_214+ORcp15_215+ORcp15_216+qd[2];
    VIcp15_316 = ORcp15_314+ORcp15_315+ORcp15_316+qd[3];
    OPcp15_116 = OPcp15_115+ROcp15_715*qdd[16]+qd[16]*(OMcp15_215*ROcp15_915-OMcp15_315*ROcp15_815);
    OPcp15_216 = OPcp15_215+ROcp15_815*qdd[16]-qd[16]*(OMcp15_115*ROcp15_915-OMcp15_315*ROcp15_715);
    OPcp15_316 = OPcp15_315+ROcp15_915*qdd[16]+qd[16]*(OMcp15_115*ROcp15_815-OMcp15_215*ROcp15_715);
    ACcp15_116 = qdd[1]+OMcp15_214*ORcp15_315+OMcp15_215*ORcp15_316+OMcp15_26*ORcp15_314-OMcp15_314*ORcp15_215-OMcp15_315*
 ORcp15_216-OMcp15_36*ORcp15_214+OPcp15_214*RLcp15_315+OPcp15_215*RLcp15_316+OPcp15_26*RLcp15_314-OPcp15_314*RLcp15_215-
 OPcp15_315*RLcp15_216-OPcp15_36*RLcp15_214;
    ACcp15_216 = qdd[2]-OMcp15_114*ORcp15_315-OMcp15_115*ORcp15_316-OMcp15_16*ORcp15_314+OMcp15_314*ORcp15_115+OMcp15_315*
 ORcp15_116+OMcp15_36*ORcp15_114-OPcp15_114*RLcp15_315-OPcp15_115*RLcp15_316-OPcp15_16*RLcp15_314+OPcp15_314*RLcp15_115+
 OPcp15_315*RLcp15_116+OPcp15_36*RLcp15_114;
    ACcp15_316 = qdd[3]+OMcp15_114*ORcp15_215+OMcp15_115*ORcp15_216+OMcp15_16*ORcp15_214-OMcp15_214*ORcp15_115-OMcp15_215*
 ORcp15_116-OMcp15_26*ORcp15_114+OPcp15_114*RLcp15_215+OPcp15_115*RLcp15_216+OPcp15_16*RLcp15_214-OPcp15_214*RLcp15_115-
 OPcp15_215*RLcp15_116-OPcp15_26*RLcp15_114;

// = = Block_1_0_0_16_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp15_116;
    sens->P[2] = POcp15_216;
    sens->P[3] = POcp15_316;
    sens->R[1][1] = ROcp15_116;
    sens->R[1][2] = ROcp15_216;
    sens->R[1][3] = ROcp15_316;
    sens->R[2][1] = ROcp15_416;
    sens->R[2][2] = ROcp15_516;
    sens->R[2][3] = ROcp15_616;
    sens->R[3][1] = ROcp15_715;
    sens->R[3][2] = ROcp15_815;
    sens->R[3][3] = ROcp15_915;
    sens->V[1] = VIcp15_116;
    sens->V[2] = VIcp15_216;
    sens->V[3] = VIcp15_316;
    sens->OM[1] = OMcp15_116;
    sens->OM[2] = OMcp15_216;
    sens->OM[3] = OMcp15_316;
    sens->A[1] = ACcp15_116;
    sens->A[2] = ACcp15_216;
    sens->A[3] = ACcp15_316;
    sens->OMP[1] = OPcp15_116;
    sens->OMP[2] = OPcp15_216;
    sens->OMP[3] = OPcp15_316;
 
// 
break;
case 17:
 


// = = Block_1_0_0_17_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp16_25 = qd[5]*C4;
    OMcp16_35 = qd[5]*S4;
    OMcp16_16 = qd[4]+qd[6]*S5;
    OMcp16_26 = OMcp16_25+ROcp16_85*qd[6];
    OMcp16_36 = OMcp16_35+ROcp16_95*qd[6];
    OPcp16_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp16_26 = ROcp16_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp16_35*S5-ROcp16_95*qd[4]);
    OPcp16_36 = ROcp16_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp16_25*S5-ROcp16_85*qd[4]);

// = = Block_1_0_0_17_0_3 = = 
 
// Sensor Kinematics 


    ROcp16_114 = ROcp16_16*C14-S14*S5;
    ROcp16_214 = ROcp16_26*C14-ROcp16_85*S14;
    ROcp16_314 = ROcp16_36*C14-ROcp16_95*S14;
    ROcp16_714 = ROcp16_16*S14+C14*S5;
    ROcp16_814 = ROcp16_26*S14+ROcp16_85*C14;
    ROcp16_914 = ROcp16_36*S14+ROcp16_95*C14;
    ROcp16_415 = ROcp16_46*C15+ROcp16_714*S15;
    ROcp16_515 = ROcp16_56*C15+ROcp16_814*S15;
    ROcp16_615 = ROcp16_66*C15+ROcp16_914*S15;
    ROcp16_715 = -(ROcp16_46*S15-ROcp16_714*C15);
    ROcp16_815 = -(ROcp16_56*S15-ROcp16_814*C15);
    ROcp16_915 = -(ROcp16_66*S15-ROcp16_914*C15);
    ROcp16_116 = ROcp16_114*C16+ROcp16_415*S16;
    ROcp16_216 = ROcp16_214*C16+ROcp16_515*S16;
    ROcp16_316 = ROcp16_314*C16+ROcp16_615*S16;
    ROcp16_416 = -(ROcp16_114*S16-ROcp16_415*C16);
    ROcp16_516 = -(ROcp16_214*S16-ROcp16_515*C16);
    ROcp16_616 = -(ROcp16_314*S16-ROcp16_615*C16);
    ROcp16_117 = ROcp16_116*C17-ROcp16_715*S17;
    ROcp16_217 = ROcp16_216*C17-ROcp16_815*S17;
    ROcp16_317 = ROcp16_316*C17-ROcp16_915*S17;
    ROcp16_717 = ROcp16_116*S17+ROcp16_715*C17;
    ROcp16_817 = ROcp16_216*S17+ROcp16_815*C17;
    ROcp16_917 = ROcp16_316*S17+ROcp16_915*C17;
    RLcp16_114 = ROcp16_46*s->dpt[2][2];
    RLcp16_214 = ROcp16_56*s->dpt[2][2];
    RLcp16_314 = ROcp16_66*s->dpt[2][2];
    OMcp16_114 = OMcp16_16+ROcp16_46*qd[14];
    OMcp16_214 = OMcp16_26+ROcp16_56*qd[14];
    OMcp16_314 = OMcp16_36+ROcp16_66*qd[14];
    ORcp16_114 = OMcp16_26*RLcp16_314-OMcp16_36*RLcp16_214;
    ORcp16_214 = -(OMcp16_16*RLcp16_314-OMcp16_36*RLcp16_114);
    ORcp16_314 = OMcp16_16*RLcp16_214-OMcp16_26*RLcp16_114;
    OPcp16_114 = OPcp16_16+ROcp16_46*qdd[14]+qd[14]*(OMcp16_26*ROcp16_66-OMcp16_36*ROcp16_56);
    OPcp16_214 = OPcp16_26+ROcp16_56*qdd[14]-qd[14]*(OMcp16_16*ROcp16_66-OMcp16_36*ROcp16_46);
    OPcp16_314 = OPcp16_36+ROcp16_66*qdd[14]+qd[14]*(OMcp16_16*ROcp16_56-OMcp16_26*ROcp16_46);
    RLcp16_115 = ROcp16_46*s->dpt[2][20];
    RLcp16_215 = ROcp16_56*s->dpt[2][20];
    RLcp16_315 = ROcp16_66*s->dpt[2][20];
    OMcp16_115 = OMcp16_114+ROcp16_114*qd[15];
    OMcp16_215 = OMcp16_214+ROcp16_214*qd[15];
    OMcp16_315 = OMcp16_314+ROcp16_314*qd[15];
    ORcp16_115 = OMcp16_214*RLcp16_315-OMcp16_314*RLcp16_215;
    ORcp16_215 = -(OMcp16_114*RLcp16_315-OMcp16_314*RLcp16_115);
    ORcp16_315 = OMcp16_114*RLcp16_215-OMcp16_214*RLcp16_115;
    OPcp16_115 = OPcp16_114+ROcp16_114*qdd[15]+qd[15]*(OMcp16_214*ROcp16_314-OMcp16_314*ROcp16_214);
    OPcp16_215 = OPcp16_214+ROcp16_214*qdd[15]-qd[15]*(OMcp16_114*ROcp16_314-OMcp16_314*ROcp16_114);
    OPcp16_315 = OPcp16_314+ROcp16_314*qdd[15]+qd[15]*(OMcp16_114*ROcp16_214-OMcp16_214*ROcp16_114);
    RLcp16_116 = ROcp16_715*s->dpt[3][22];
    RLcp16_216 = ROcp16_815*s->dpt[3][22];
    RLcp16_316 = ROcp16_915*s->dpt[3][22];
    OMcp16_116 = OMcp16_115+ROcp16_715*qd[16];
    OMcp16_216 = OMcp16_215+ROcp16_815*qd[16];
    OMcp16_316 = OMcp16_315+ROcp16_915*qd[16];
    ORcp16_116 = OMcp16_215*RLcp16_316-OMcp16_315*RLcp16_216;
    ORcp16_216 = -(OMcp16_115*RLcp16_316-OMcp16_315*RLcp16_116);
    ORcp16_316 = OMcp16_115*RLcp16_216-OMcp16_215*RLcp16_116;
    OPcp16_116 = OPcp16_115+ROcp16_715*qdd[16]+qd[16]*(OMcp16_215*ROcp16_915-OMcp16_315*ROcp16_815);
    OPcp16_216 = OPcp16_215+ROcp16_815*qdd[16]-qd[16]*(OMcp16_115*ROcp16_915-OMcp16_315*ROcp16_715);
    OPcp16_316 = OPcp16_315+ROcp16_915*qdd[16]+qd[16]*(OMcp16_115*ROcp16_815-OMcp16_215*ROcp16_715);
    RLcp16_117 = ROcp16_715*s->dpt[3][24];
    RLcp16_217 = ROcp16_815*s->dpt[3][24];
    RLcp16_317 = ROcp16_915*s->dpt[3][24];
    POcp16_117 = RLcp16_114+RLcp16_115+RLcp16_116+RLcp16_117+q[1];
    POcp16_217 = RLcp16_214+RLcp16_215+RLcp16_216+RLcp16_217+q[2];
    POcp16_317 = RLcp16_314+RLcp16_315+RLcp16_316+RLcp16_317+q[3];
    OMcp16_117 = OMcp16_116+ROcp16_416*qd[17];
    OMcp16_217 = OMcp16_216+ROcp16_516*qd[17];
    OMcp16_317 = OMcp16_316+ROcp16_616*qd[17];
    ORcp16_117 = OMcp16_216*RLcp16_317-OMcp16_316*RLcp16_217;
    ORcp16_217 = -(OMcp16_116*RLcp16_317-OMcp16_316*RLcp16_117);
    ORcp16_317 = OMcp16_116*RLcp16_217-OMcp16_216*RLcp16_117;
    VIcp16_117 = ORcp16_114+ORcp16_115+ORcp16_116+ORcp16_117+qd[1];
    VIcp16_217 = ORcp16_214+ORcp16_215+ORcp16_216+ORcp16_217+qd[2];
    VIcp16_317 = ORcp16_314+ORcp16_315+ORcp16_316+ORcp16_317+qd[3];
    OPcp16_117 = OPcp16_116+ROcp16_416*qdd[17]+qd[17]*(OMcp16_216*ROcp16_616-OMcp16_316*ROcp16_516);
    OPcp16_217 = OPcp16_216+ROcp16_516*qdd[17]-qd[17]*(OMcp16_116*ROcp16_616-OMcp16_316*ROcp16_416);
    OPcp16_317 = OPcp16_316+ROcp16_616*qdd[17]+qd[17]*(OMcp16_116*ROcp16_516-OMcp16_216*ROcp16_416);
    ACcp16_117 = qdd[1]+OMcp16_214*ORcp16_315+OMcp16_215*ORcp16_316+OMcp16_216*ORcp16_317+OMcp16_26*ORcp16_314-OMcp16_314*
 ORcp16_215-OMcp16_315*ORcp16_216-OMcp16_316*ORcp16_217-OMcp16_36*ORcp16_214+OPcp16_214*RLcp16_315+OPcp16_215*RLcp16_316+
 OPcp16_216*RLcp16_317+OPcp16_26*RLcp16_314-OPcp16_314*RLcp16_215-OPcp16_315*RLcp16_216-OPcp16_316*RLcp16_217-OPcp16_36*
 RLcp16_214;
    ACcp16_217 = qdd[2]-OMcp16_114*ORcp16_315-OMcp16_115*ORcp16_316-OMcp16_116*ORcp16_317-OMcp16_16*ORcp16_314+OMcp16_314*
 ORcp16_115+OMcp16_315*ORcp16_116+OMcp16_316*ORcp16_117+OMcp16_36*ORcp16_114-OPcp16_114*RLcp16_315-OPcp16_115*RLcp16_316-
 OPcp16_116*RLcp16_317-OPcp16_16*RLcp16_314+OPcp16_314*RLcp16_115+OPcp16_315*RLcp16_116+OPcp16_316*RLcp16_117+OPcp16_36*
 RLcp16_114;
    ACcp16_317 = qdd[3]+OMcp16_114*ORcp16_215+OMcp16_115*ORcp16_216+OMcp16_116*ORcp16_217+OMcp16_16*ORcp16_214-OMcp16_214*
 ORcp16_115-OMcp16_215*ORcp16_116-OMcp16_216*ORcp16_117-OMcp16_26*ORcp16_114+OPcp16_114*RLcp16_215+OPcp16_115*RLcp16_216+
 OPcp16_116*RLcp16_217+OPcp16_16*RLcp16_214-OPcp16_214*RLcp16_115-OPcp16_215*RLcp16_116-OPcp16_216*RLcp16_117-OPcp16_26*
 RLcp16_114;

// = = Block_1_0_0_17_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp16_117;
    sens->P[2] = POcp16_217;
    sens->P[3] = POcp16_317;
    sens->R[1][1] = ROcp16_117;
    sens->R[1][2] = ROcp16_217;
    sens->R[1][3] = ROcp16_317;
    sens->R[2][1] = ROcp16_416;
    sens->R[2][2] = ROcp16_516;
    sens->R[2][3] = ROcp16_616;
    sens->R[3][1] = ROcp16_717;
    sens->R[3][2] = ROcp16_817;
    sens->R[3][3] = ROcp16_917;
    sens->V[1] = VIcp16_117;
    sens->V[2] = VIcp16_217;
    sens->V[3] = VIcp16_317;
    sens->OM[1] = OMcp16_117;
    sens->OM[2] = OMcp16_217;
    sens->OM[3] = OMcp16_317;
    sens->A[1] = ACcp16_117;
    sens->A[2] = ACcp16_217;
    sens->A[3] = ACcp16_317;
    sens->OMP[1] = OPcp16_117;
    sens->OMP[2] = OPcp16_217;
    sens->OMP[3] = OPcp16_317;
 
// 
break;
case 18:
 


// = = Block_1_0_0_18_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp17_25 = qd[5]*C4;
    OMcp17_35 = qd[5]*S4;
    OMcp17_16 = qd[4]+qd[6]*S5;
    OMcp17_26 = OMcp17_25+ROcp17_85*qd[6];
    OMcp17_36 = OMcp17_35+ROcp17_95*qd[6];
    OPcp17_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp17_26 = ROcp17_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp17_35*S5-ROcp17_95*qd[4]);
    OPcp17_36 = ROcp17_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp17_25*S5-ROcp17_85*qd[4]);

// = = Block_1_0_0_18_0_3 = = 
 
// Sensor Kinematics 


    ROcp17_114 = ROcp17_16*C14-S14*S5;
    ROcp17_214 = ROcp17_26*C14-ROcp17_85*S14;
    ROcp17_314 = ROcp17_36*C14-ROcp17_95*S14;
    ROcp17_714 = ROcp17_16*S14+C14*S5;
    ROcp17_814 = ROcp17_26*S14+ROcp17_85*C14;
    ROcp17_914 = ROcp17_36*S14+ROcp17_95*C14;
    ROcp17_415 = ROcp17_46*C15+ROcp17_714*S15;
    ROcp17_515 = ROcp17_56*C15+ROcp17_814*S15;
    ROcp17_615 = ROcp17_66*C15+ROcp17_914*S15;
    ROcp17_715 = -(ROcp17_46*S15-ROcp17_714*C15);
    ROcp17_815 = -(ROcp17_56*S15-ROcp17_814*C15);
    ROcp17_915 = -(ROcp17_66*S15-ROcp17_914*C15);
    ROcp17_116 = ROcp17_114*C16+ROcp17_415*S16;
    ROcp17_216 = ROcp17_214*C16+ROcp17_515*S16;
    ROcp17_316 = ROcp17_314*C16+ROcp17_615*S16;
    ROcp17_416 = -(ROcp17_114*S16-ROcp17_415*C16);
    ROcp17_516 = -(ROcp17_214*S16-ROcp17_515*C16);
    ROcp17_616 = -(ROcp17_314*S16-ROcp17_615*C16);
    ROcp17_117 = ROcp17_116*C17-ROcp17_715*S17;
    ROcp17_217 = ROcp17_216*C17-ROcp17_815*S17;
    ROcp17_317 = ROcp17_316*C17-ROcp17_915*S17;
    ROcp17_717 = ROcp17_116*S17+ROcp17_715*C17;
    ROcp17_817 = ROcp17_216*S17+ROcp17_815*C17;
    ROcp17_917 = ROcp17_316*S17+ROcp17_915*C17;
    ROcp17_418 = ROcp17_416*C18+ROcp17_717*S18;
    ROcp17_518 = ROcp17_516*C18+ROcp17_817*S18;
    ROcp17_618 = ROcp17_616*C18+ROcp17_917*S18;
    ROcp17_718 = -(ROcp17_416*S18-ROcp17_717*C18);
    ROcp17_818 = -(ROcp17_516*S18-ROcp17_817*C18);
    ROcp17_918 = -(ROcp17_616*S18-ROcp17_917*C18);
    RLcp17_114 = ROcp17_46*s->dpt[2][2];
    RLcp17_214 = ROcp17_56*s->dpt[2][2];
    RLcp17_314 = ROcp17_66*s->dpt[2][2];
    OMcp17_114 = OMcp17_16+ROcp17_46*qd[14];
    OMcp17_214 = OMcp17_26+ROcp17_56*qd[14];
    OMcp17_314 = OMcp17_36+ROcp17_66*qd[14];
    ORcp17_114 = OMcp17_26*RLcp17_314-OMcp17_36*RLcp17_214;
    ORcp17_214 = -(OMcp17_16*RLcp17_314-OMcp17_36*RLcp17_114);
    ORcp17_314 = OMcp17_16*RLcp17_214-OMcp17_26*RLcp17_114;
    OPcp17_114 = OPcp17_16+ROcp17_46*qdd[14]+qd[14]*(OMcp17_26*ROcp17_66-OMcp17_36*ROcp17_56);
    OPcp17_214 = OPcp17_26+ROcp17_56*qdd[14]-qd[14]*(OMcp17_16*ROcp17_66-OMcp17_36*ROcp17_46);
    OPcp17_314 = OPcp17_36+ROcp17_66*qdd[14]+qd[14]*(OMcp17_16*ROcp17_56-OMcp17_26*ROcp17_46);
    RLcp17_115 = ROcp17_46*s->dpt[2][20];
    RLcp17_215 = ROcp17_56*s->dpt[2][20];
    RLcp17_315 = ROcp17_66*s->dpt[2][20];
    OMcp17_115 = OMcp17_114+ROcp17_114*qd[15];
    OMcp17_215 = OMcp17_214+ROcp17_214*qd[15];
    OMcp17_315 = OMcp17_314+ROcp17_314*qd[15];
    ORcp17_115 = OMcp17_214*RLcp17_315-OMcp17_314*RLcp17_215;
    ORcp17_215 = -(OMcp17_114*RLcp17_315-OMcp17_314*RLcp17_115);
    ORcp17_315 = OMcp17_114*RLcp17_215-OMcp17_214*RLcp17_115;
    OPcp17_115 = OPcp17_114+ROcp17_114*qdd[15]+qd[15]*(OMcp17_214*ROcp17_314-OMcp17_314*ROcp17_214);
    OPcp17_215 = OPcp17_214+ROcp17_214*qdd[15]-qd[15]*(OMcp17_114*ROcp17_314-OMcp17_314*ROcp17_114);
    OPcp17_315 = OPcp17_314+ROcp17_314*qdd[15]+qd[15]*(OMcp17_114*ROcp17_214-OMcp17_214*ROcp17_114);
    RLcp17_116 = ROcp17_715*s->dpt[3][22];
    RLcp17_216 = ROcp17_815*s->dpt[3][22];
    RLcp17_316 = ROcp17_915*s->dpt[3][22];
    OMcp17_116 = OMcp17_115+ROcp17_715*qd[16];
    OMcp17_216 = OMcp17_215+ROcp17_815*qd[16];
    OMcp17_316 = OMcp17_315+ROcp17_915*qd[16];
    ORcp17_116 = OMcp17_215*RLcp17_316-OMcp17_315*RLcp17_216;
    ORcp17_216 = -(OMcp17_115*RLcp17_316-OMcp17_315*RLcp17_116);
    ORcp17_316 = OMcp17_115*RLcp17_216-OMcp17_215*RLcp17_116;
    OPcp17_116 = OPcp17_115+ROcp17_715*qdd[16]+qd[16]*(OMcp17_215*ROcp17_915-OMcp17_315*ROcp17_815);
    OPcp17_216 = OPcp17_215+ROcp17_815*qdd[16]-qd[16]*(OMcp17_115*ROcp17_915-OMcp17_315*ROcp17_715);
    OPcp17_316 = OPcp17_315+ROcp17_915*qdd[16]+qd[16]*(OMcp17_115*ROcp17_815-OMcp17_215*ROcp17_715);
    RLcp17_117 = ROcp17_715*s->dpt[3][24];
    RLcp17_217 = ROcp17_815*s->dpt[3][24];
    RLcp17_317 = ROcp17_915*s->dpt[3][24];
    OMcp17_117 = OMcp17_116+ROcp17_416*qd[17];
    OMcp17_217 = OMcp17_216+ROcp17_516*qd[17];
    OMcp17_317 = OMcp17_316+ROcp17_616*qd[17];
    ORcp17_117 = OMcp17_216*RLcp17_317-OMcp17_316*RLcp17_217;
    ORcp17_217 = -(OMcp17_116*RLcp17_317-OMcp17_316*RLcp17_117);
    ORcp17_317 = OMcp17_116*RLcp17_217-OMcp17_216*RLcp17_117;
    OPcp17_117 = OPcp17_116+ROcp17_416*qdd[17]+qd[17]*(OMcp17_216*ROcp17_616-OMcp17_316*ROcp17_516);
    OPcp17_217 = OPcp17_216+ROcp17_516*qdd[17]-qd[17]*(OMcp17_116*ROcp17_616-OMcp17_316*ROcp17_416);
    OPcp17_317 = OPcp17_316+ROcp17_616*qdd[17]+qd[17]*(OMcp17_116*ROcp17_516-OMcp17_216*ROcp17_416);
    RLcp17_118 = ROcp17_717*s->dpt[3][26];
    RLcp17_218 = ROcp17_817*s->dpt[3][26];
    RLcp17_318 = ROcp17_917*s->dpt[3][26];
    POcp17_118 = RLcp17_114+RLcp17_115+RLcp17_116+RLcp17_117+RLcp17_118+q[1];
    POcp17_218 = RLcp17_214+RLcp17_215+RLcp17_216+RLcp17_217+RLcp17_218+q[2];
    POcp17_318 = RLcp17_314+RLcp17_315+RLcp17_316+RLcp17_317+RLcp17_318+q[3];
    OMcp17_118 = OMcp17_117+ROcp17_117*qd[18];
    OMcp17_218 = OMcp17_217+ROcp17_217*qd[18];
    OMcp17_318 = OMcp17_317+ROcp17_317*qd[18];
    ORcp17_118 = OMcp17_217*RLcp17_318-OMcp17_317*RLcp17_218;
    ORcp17_218 = -(OMcp17_117*RLcp17_318-OMcp17_317*RLcp17_118);
    ORcp17_318 = OMcp17_117*RLcp17_218-OMcp17_217*RLcp17_118;
    VIcp17_118 = ORcp17_114+ORcp17_115+ORcp17_116+ORcp17_117+ORcp17_118+qd[1];
    VIcp17_218 = ORcp17_214+ORcp17_215+ORcp17_216+ORcp17_217+ORcp17_218+qd[2];
    VIcp17_318 = ORcp17_314+ORcp17_315+ORcp17_316+ORcp17_317+ORcp17_318+qd[3];
    OPcp17_118 = OPcp17_117+ROcp17_117*qdd[18]+qd[18]*(OMcp17_217*ROcp17_317-OMcp17_317*ROcp17_217);
    OPcp17_218 = OPcp17_217+ROcp17_217*qdd[18]-qd[18]*(OMcp17_117*ROcp17_317-OMcp17_317*ROcp17_117);
    OPcp17_318 = OPcp17_317+ROcp17_317*qdd[18]+qd[18]*(OMcp17_117*ROcp17_217-OMcp17_217*ROcp17_117);
    ACcp17_118 = qdd[1]+OMcp17_214*ORcp17_315+OMcp17_215*ORcp17_316+OMcp17_216*ORcp17_317+OMcp17_217*ORcp17_318+OMcp17_26*
 ORcp17_314-OMcp17_314*ORcp17_215-OMcp17_315*ORcp17_216-OMcp17_316*ORcp17_217-OMcp17_317*ORcp17_218-OMcp17_36*ORcp17_214+
 OPcp17_214*RLcp17_315+OPcp17_215*RLcp17_316+OPcp17_216*RLcp17_317+OPcp17_217*RLcp17_318+OPcp17_26*RLcp17_314-OPcp17_314*
 RLcp17_215-OPcp17_315*RLcp17_216-OPcp17_316*RLcp17_217-OPcp17_317*RLcp17_218-OPcp17_36*RLcp17_214;
    ACcp17_218 = qdd[2]-OMcp17_114*ORcp17_315-OMcp17_115*ORcp17_316-OMcp17_116*ORcp17_317-OMcp17_117*ORcp17_318-OMcp17_16*
 ORcp17_314+OMcp17_314*ORcp17_115+OMcp17_315*ORcp17_116+OMcp17_316*ORcp17_117+OMcp17_317*ORcp17_118+OMcp17_36*ORcp17_114-
 OPcp17_114*RLcp17_315-OPcp17_115*RLcp17_316-OPcp17_116*RLcp17_317-OPcp17_117*RLcp17_318-OPcp17_16*RLcp17_314+OPcp17_314*
 RLcp17_115+OPcp17_315*RLcp17_116+OPcp17_316*RLcp17_117+OPcp17_317*RLcp17_118+OPcp17_36*RLcp17_114;
    ACcp17_318 = qdd[3]+OMcp17_114*ORcp17_215+OMcp17_115*ORcp17_216+OMcp17_116*ORcp17_217+OMcp17_117*ORcp17_218+OMcp17_16*
 ORcp17_214-OMcp17_214*ORcp17_115-OMcp17_215*ORcp17_116-OMcp17_216*ORcp17_117-OMcp17_217*ORcp17_118-OMcp17_26*ORcp17_114+
 OPcp17_114*RLcp17_215+OPcp17_115*RLcp17_216+OPcp17_116*RLcp17_217+OPcp17_117*RLcp17_218+OPcp17_16*RLcp17_214-OPcp17_214*
 RLcp17_115-OPcp17_215*RLcp17_116-OPcp17_216*RLcp17_117-OPcp17_217*RLcp17_118-OPcp17_26*RLcp17_114;

// = = Block_1_0_0_18_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp17_118;
    sens->P[2] = POcp17_218;
    sens->P[3] = POcp17_318;
    sens->R[1][1] = ROcp17_117;
    sens->R[1][2] = ROcp17_217;
    sens->R[1][3] = ROcp17_317;
    sens->R[2][1] = ROcp17_418;
    sens->R[2][2] = ROcp17_518;
    sens->R[2][3] = ROcp17_618;
    sens->R[3][1] = ROcp17_718;
    sens->R[3][2] = ROcp17_818;
    sens->R[3][3] = ROcp17_918;
    sens->V[1] = VIcp17_118;
    sens->V[2] = VIcp17_218;
    sens->V[3] = VIcp17_318;
    sens->OM[1] = OMcp17_118;
    sens->OM[2] = OMcp17_218;
    sens->OM[3] = OMcp17_318;
    sens->A[1] = ACcp17_118;
    sens->A[2] = ACcp17_218;
    sens->A[3] = ACcp17_318;
    sens->OMP[1] = OPcp17_118;
    sens->OMP[2] = OPcp17_218;
    sens->OMP[3] = OPcp17_318;
 
// 
break;
case 19:
 


// = = Block_1_0_0_19_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp18_25 = qd[5]*C4;
    OMcp18_35 = qd[5]*S4;
    OMcp18_16 = qd[4]+qd[6]*S5;
    OMcp18_26 = OMcp18_25+ROcp18_85*qd[6];
    OMcp18_36 = OMcp18_35+ROcp18_95*qd[6];
    OPcp18_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp18_26 = ROcp18_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp18_35*S5-ROcp18_95*qd[4]);
    OPcp18_36 = ROcp18_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp18_25*S5-ROcp18_85*qd[4]);

// = = Block_1_0_0_19_0_3 = = 
 
// Sensor Kinematics 


    ROcp18_114 = ROcp18_16*C14-S14*S5;
    ROcp18_214 = ROcp18_26*C14-ROcp18_85*S14;
    ROcp18_314 = ROcp18_36*C14-ROcp18_95*S14;
    ROcp18_714 = ROcp18_16*S14+C14*S5;
    ROcp18_814 = ROcp18_26*S14+ROcp18_85*C14;
    ROcp18_914 = ROcp18_36*S14+ROcp18_95*C14;
    ROcp18_415 = ROcp18_46*C15+ROcp18_714*S15;
    ROcp18_515 = ROcp18_56*C15+ROcp18_814*S15;
    ROcp18_615 = ROcp18_66*C15+ROcp18_914*S15;
    ROcp18_715 = -(ROcp18_46*S15-ROcp18_714*C15);
    ROcp18_815 = -(ROcp18_56*S15-ROcp18_814*C15);
    ROcp18_915 = -(ROcp18_66*S15-ROcp18_914*C15);
    ROcp18_116 = ROcp18_114*C16+ROcp18_415*S16;
    ROcp18_216 = ROcp18_214*C16+ROcp18_515*S16;
    ROcp18_316 = ROcp18_314*C16+ROcp18_615*S16;
    ROcp18_416 = -(ROcp18_114*S16-ROcp18_415*C16);
    ROcp18_516 = -(ROcp18_214*S16-ROcp18_515*C16);
    ROcp18_616 = -(ROcp18_314*S16-ROcp18_615*C16);
    ROcp18_117 = ROcp18_116*C17-ROcp18_715*S17;
    ROcp18_217 = ROcp18_216*C17-ROcp18_815*S17;
    ROcp18_317 = ROcp18_316*C17-ROcp18_915*S17;
    ROcp18_717 = ROcp18_116*S17+ROcp18_715*C17;
    ROcp18_817 = ROcp18_216*S17+ROcp18_815*C17;
    ROcp18_917 = ROcp18_316*S17+ROcp18_915*C17;
    ROcp18_418 = ROcp18_416*C18+ROcp18_717*S18;
    ROcp18_518 = ROcp18_516*C18+ROcp18_817*S18;
    ROcp18_618 = ROcp18_616*C18+ROcp18_917*S18;
    ROcp18_718 = -(ROcp18_416*S18-ROcp18_717*C18);
    ROcp18_818 = -(ROcp18_516*S18-ROcp18_817*C18);
    ROcp18_918 = -(ROcp18_616*S18-ROcp18_917*C18);
    ROcp18_119 = ROcp18_117*C19-ROcp18_718*S19;
    ROcp18_219 = ROcp18_217*C19-ROcp18_818*S19;
    ROcp18_319 = ROcp18_317*C19-ROcp18_918*S19;
    ROcp18_719 = ROcp18_117*S19+ROcp18_718*C19;
    ROcp18_819 = ROcp18_217*S19+ROcp18_818*C19;
    ROcp18_919 = ROcp18_317*S19+ROcp18_918*C19;
    RLcp18_114 = ROcp18_46*s->dpt[2][2];
    RLcp18_214 = ROcp18_56*s->dpt[2][2];
    RLcp18_314 = ROcp18_66*s->dpt[2][2];
    OMcp18_114 = OMcp18_16+ROcp18_46*qd[14];
    OMcp18_214 = OMcp18_26+ROcp18_56*qd[14];
    OMcp18_314 = OMcp18_36+ROcp18_66*qd[14];
    ORcp18_114 = OMcp18_26*RLcp18_314-OMcp18_36*RLcp18_214;
    ORcp18_214 = -(OMcp18_16*RLcp18_314-OMcp18_36*RLcp18_114);
    ORcp18_314 = OMcp18_16*RLcp18_214-OMcp18_26*RLcp18_114;
    OPcp18_114 = OPcp18_16+ROcp18_46*qdd[14]+qd[14]*(OMcp18_26*ROcp18_66-OMcp18_36*ROcp18_56);
    OPcp18_214 = OPcp18_26+ROcp18_56*qdd[14]-qd[14]*(OMcp18_16*ROcp18_66-OMcp18_36*ROcp18_46);
    OPcp18_314 = OPcp18_36+ROcp18_66*qdd[14]+qd[14]*(OMcp18_16*ROcp18_56-OMcp18_26*ROcp18_46);
    RLcp18_115 = ROcp18_46*s->dpt[2][20];
    RLcp18_215 = ROcp18_56*s->dpt[2][20];
    RLcp18_315 = ROcp18_66*s->dpt[2][20];
    OMcp18_115 = OMcp18_114+ROcp18_114*qd[15];
    OMcp18_215 = OMcp18_214+ROcp18_214*qd[15];
    OMcp18_315 = OMcp18_314+ROcp18_314*qd[15];
    ORcp18_115 = OMcp18_214*RLcp18_315-OMcp18_314*RLcp18_215;
    ORcp18_215 = -(OMcp18_114*RLcp18_315-OMcp18_314*RLcp18_115);
    ORcp18_315 = OMcp18_114*RLcp18_215-OMcp18_214*RLcp18_115;
    OPcp18_115 = OPcp18_114+ROcp18_114*qdd[15]+qd[15]*(OMcp18_214*ROcp18_314-OMcp18_314*ROcp18_214);
    OPcp18_215 = OPcp18_214+ROcp18_214*qdd[15]-qd[15]*(OMcp18_114*ROcp18_314-OMcp18_314*ROcp18_114);
    OPcp18_315 = OPcp18_314+ROcp18_314*qdd[15]+qd[15]*(OMcp18_114*ROcp18_214-OMcp18_214*ROcp18_114);
    RLcp18_116 = ROcp18_715*s->dpt[3][22];
    RLcp18_216 = ROcp18_815*s->dpt[3][22];
    RLcp18_316 = ROcp18_915*s->dpt[3][22];
    OMcp18_116 = OMcp18_115+ROcp18_715*qd[16];
    OMcp18_216 = OMcp18_215+ROcp18_815*qd[16];
    OMcp18_316 = OMcp18_315+ROcp18_915*qd[16];
    ORcp18_116 = OMcp18_215*RLcp18_316-OMcp18_315*RLcp18_216;
    ORcp18_216 = -(OMcp18_115*RLcp18_316-OMcp18_315*RLcp18_116);
    ORcp18_316 = OMcp18_115*RLcp18_216-OMcp18_215*RLcp18_116;
    OPcp18_116 = OPcp18_115+ROcp18_715*qdd[16]+qd[16]*(OMcp18_215*ROcp18_915-OMcp18_315*ROcp18_815);
    OPcp18_216 = OPcp18_215+ROcp18_815*qdd[16]-qd[16]*(OMcp18_115*ROcp18_915-OMcp18_315*ROcp18_715);
    OPcp18_316 = OPcp18_315+ROcp18_915*qdd[16]+qd[16]*(OMcp18_115*ROcp18_815-OMcp18_215*ROcp18_715);
    RLcp18_117 = ROcp18_715*s->dpt[3][24];
    RLcp18_217 = ROcp18_815*s->dpt[3][24];
    RLcp18_317 = ROcp18_915*s->dpt[3][24];
    OMcp18_117 = OMcp18_116+ROcp18_416*qd[17];
    OMcp18_217 = OMcp18_216+ROcp18_516*qd[17];
    OMcp18_317 = OMcp18_316+ROcp18_616*qd[17];
    ORcp18_117 = OMcp18_216*RLcp18_317-OMcp18_316*RLcp18_217;
    ORcp18_217 = -(OMcp18_116*RLcp18_317-OMcp18_316*RLcp18_117);
    ORcp18_317 = OMcp18_116*RLcp18_217-OMcp18_216*RLcp18_117;
    OPcp18_117 = OPcp18_116+ROcp18_416*qdd[17]+qd[17]*(OMcp18_216*ROcp18_616-OMcp18_316*ROcp18_516);
    OPcp18_217 = OPcp18_216+ROcp18_516*qdd[17]-qd[17]*(OMcp18_116*ROcp18_616-OMcp18_316*ROcp18_416);
    OPcp18_317 = OPcp18_316+ROcp18_616*qdd[17]+qd[17]*(OMcp18_116*ROcp18_516-OMcp18_216*ROcp18_416);
    RLcp18_118 = ROcp18_717*s->dpt[3][26];
    RLcp18_218 = ROcp18_817*s->dpt[3][26];
    RLcp18_318 = ROcp18_917*s->dpt[3][26];
    POcp18_118 = RLcp18_114+RLcp18_115+RLcp18_116+RLcp18_117+RLcp18_118+q[1];
    POcp18_218 = RLcp18_214+RLcp18_215+RLcp18_216+RLcp18_217+RLcp18_218+q[2];
    POcp18_318 = RLcp18_314+RLcp18_315+RLcp18_316+RLcp18_317+RLcp18_318+q[3];
    OMcp18_118 = OMcp18_117+ROcp18_117*qd[18];
    OMcp18_218 = OMcp18_217+ROcp18_217*qd[18];
    OMcp18_318 = OMcp18_317+ROcp18_317*qd[18];
    ORcp18_118 = OMcp18_217*RLcp18_318-OMcp18_317*RLcp18_218;
    ORcp18_218 = -(OMcp18_117*RLcp18_318-OMcp18_317*RLcp18_118);
    ORcp18_318 = OMcp18_117*RLcp18_218-OMcp18_217*RLcp18_118;
    VIcp18_118 = ORcp18_114+ORcp18_115+ORcp18_116+ORcp18_117+ORcp18_118+qd[1];
    VIcp18_218 = ORcp18_214+ORcp18_215+ORcp18_216+ORcp18_217+ORcp18_218+qd[2];
    VIcp18_318 = ORcp18_314+ORcp18_315+ORcp18_316+ORcp18_317+ORcp18_318+qd[3];
    ACcp18_118 = qdd[1]+OMcp18_214*ORcp18_315+OMcp18_215*ORcp18_316+OMcp18_216*ORcp18_317+OMcp18_217*ORcp18_318+OMcp18_26*
 ORcp18_314-OMcp18_314*ORcp18_215-OMcp18_315*ORcp18_216-OMcp18_316*ORcp18_217-OMcp18_317*ORcp18_218-OMcp18_36*ORcp18_214+
 OPcp18_214*RLcp18_315+OPcp18_215*RLcp18_316+OPcp18_216*RLcp18_317+OPcp18_217*RLcp18_318+OPcp18_26*RLcp18_314-OPcp18_314*
 RLcp18_215-OPcp18_315*RLcp18_216-OPcp18_316*RLcp18_217-OPcp18_317*RLcp18_218-OPcp18_36*RLcp18_214;
    ACcp18_218 = qdd[2]-OMcp18_114*ORcp18_315-OMcp18_115*ORcp18_316-OMcp18_116*ORcp18_317-OMcp18_117*ORcp18_318-OMcp18_16*
 ORcp18_314+OMcp18_314*ORcp18_115+OMcp18_315*ORcp18_116+OMcp18_316*ORcp18_117+OMcp18_317*ORcp18_118+OMcp18_36*ORcp18_114-
 OPcp18_114*RLcp18_315-OPcp18_115*RLcp18_316-OPcp18_116*RLcp18_317-OPcp18_117*RLcp18_318-OPcp18_16*RLcp18_314+OPcp18_314*
 RLcp18_115+OPcp18_315*RLcp18_116+OPcp18_316*RLcp18_117+OPcp18_317*RLcp18_118+OPcp18_36*RLcp18_114;
    ACcp18_318 = qdd[3]+OMcp18_114*ORcp18_215+OMcp18_115*ORcp18_216+OMcp18_116*ORcp18_217+OMcp18_117*ORcp18_218+OMcp18_16*
 ORcp18_214-OMcp18_214*ORcp18_115-OMcp18_215*ORcp18_116-OMcp18_216*ORcp18_117-OMcp18_217*ORcp18_118-OMcp18_26*ORcp18_114+
 OPcp18_114*RLcp18_215+OPcp18_115*RLcp18_216+OPcp18_116*RLcp18_217+OPcp18_117*RLcp18_218+OPcp18_16*RLcp18_214-OPcp18_214*
 RLcp18_115-OPcp18_215*RLcp18_116-OPcp18_216*RLcp18_117-OPcp18_217*RLcp18_118-OPcp18_26*RLcp18_114;
    OMcp18_119 = OMcp18_118+ROcp18_418*qd[19];
    OMcp18_219 = OMcp18_218+ROcp18_518*qd[19];
    OMcp18_319 = OMcp18_318+ROcp18_618*qd[19];
    OPcp18_119 = OPcp18_117+ROcp18_117*qdd[18]+ROcp18_418*qdd[19]+qd[18]*(OMcp18_217*ROcp18_317-OMcp18_317*ROcp18_217)+
 qd[19]*(OMcp18_218*ROcp18_618-OMcp18_318*ROcp18_518);
    OPcp18_219 = OPcp18_217+ROcp18_217*qdd[18]+ROcp18_518*qdd[19]-qd[18]*(OMcp18_117*ROcp18_317-OMcp18_317*ROcp18_117)-
 qd[19]*(OMcp18_118*ROcp18_618-OMcp18_318*ROcp18_418);
    OPcp18_319 = OPcp18_317+ROcp18_317*qdd[18]+ROcp18_618*qdd[19]+qd[18]*(OMcp18_117*ROcp18_217-OMcp18_217*ROcp18_117)+
 qd[19]*(OMcp18_118*ROcp18_518-OMcp18_218*ROcp18_418);

// = = Block_1_0_0_19_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp18_118;
    sens->P[2] = POcp18_218;
    sens->P[3] = POcp18_318;
    sens->R[1][1] = ROcp18_119;
    sens->R[1][2] = ROcp18_219;
    sens->R[1][3] = ROcp18_319;
    sens->R[2][1] = ROcp18_418;
    sens->R[2][2] = ROcp18_518;
    sens->R[2][3] = ROcp18_618;
    sens->R[3][1] = ROcp18_719;
    sens->R[3][2] = ROcp18_819;
    sens->R[3][3] = ROcp18_919;
    sens->V[1] = VIcp18_118;
    sens->V[2] = VIcp18_218;
    sens->V[3] = VIcp18_318;
    sens->OM[1] = OMcp18_119;
    sens->OM[2] = OMcp18_219;
    sens->OM[3] = OMcp18_319;
    sens->A[1] = ACcp18_118;
    sens->A[2] = ACcp18_218;
    sens->A[3] = ACcp18_318;
    sens->OMP[1] = OPcp18_119;
    sens->OMP[2] = OPcp18_219;
    sens->OMP[3] = OPcp18_319;
 
// 
break;
case 20:
 


// = = Block_1_0_0_20_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp19_25 = qd[5]*C4;
    OMcp19_35 = qd[5]*S4;
    OMcp19_16 = qd[4]+qd[6]*S5;
    OMcp19_26 = OMcp19_25+ROcp19_85*qd[6];
    OMcp19_36 = OMcp19_35+ROcp19_95*qd[6];
    OPcp19_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp19_26 = ROcp19_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp19_35*S5-ROcp19_95*qd[4]);
    OPcp19_36 = ROcp19_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp19_25*S5-ROcp19_85*qd[4]);

// = = Block_1_0_0_20_0_3 = = 
 
// Sensor Kinematics 


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
    ROcp19_116 = ROcp19_114*C16+ROcp19_415*S16;
    ROcp19_216 = ROcp19_214*C16+ROcp19_515*S16;
    ROcp19_316 = ROcp19_314*C16+ROcp19_615*S16;
    ROcp19_416 = -(ROcp19_114*S16-ROcp19_415*C16);
    ROcp19_516 = -(ROcp19_214*S16-ROcp19_515*C16);
    ROcp19_616 = -(ROcp19_314*S16-ROcp19_615*C16);
    ROcp19_117 = ROcp19_116*C17-ROcp19_715*S17;
    ROcp19_217 = ROcp19_216*C17-ROcp19_815*S17;
    ROcp19_317 = ROcp19_316*C17-ROcp19_915*S17;
    ROcp19_717 = ROcp19_116*S17+ROcp19_715*C17;
    ROcp19_817 = ROcp19_216*S17+ROcp19_815*C17;
    ROcp19_917 = ROcp19_316*S17+ROcp19_915*C17;
    ROcp19_418 = ROcp19_416*C18+ROcp19_717*S18;
    ROcp19_518 = ROcp19_516*C18+ROcp19_817*S18;
    ROcp19_618 = ROcp19_616*C18+ROcp19_917*S18;
    ROcp19_718 = -(ROcp19_416*S18-ROcp19_717*C18);
    ROcp19_818 = -(ROcp19_516*S18-ROcp19_817*C18);
    ROcp19_918 = -(ROcp19_616*S18-ROcp19_917*C18);
    ROcp19_119 = ROcp19_117*C19-ROcp19_718*S19;
    ROcp19_219 = ROcp19_217*C19-ROcp19_818*S19;
    ROcp19_319 = ROcp19_317*C19-ROcp19_918*S19;
    ROcp19_719 = ROcp19_117*S19+ROcp19_718*C19;
    ROcp19_819 = ROcp19_217*S19+ROcp19_818*C19;
    ROcp19_919 = ROcp19_317*S19+ROcp19_918*C19;
    ROcp19_120 = ROcp19_119*C20-ROcp19_719*S20;
    ROcp19_220 = ROcp19_219*C20-ROcp19_819*S20;
    ROcp19_320 = ROcp19_319*C20-ROcp19_919*S20;
    ROcp19_720 = ROcp19_119*S20+ROcp19_719*C20;
    ROcp19_820 = ROcp19_219*S20+ROcp19_819*C20;
    ROcp19_920 = ROcp19_319*S20+ROcp19_919*C20;
    RLcp19_114 = ROcp19_46*s->dpt[2][2];
    RLcp19_214 = ROcp19_56*s->dpt[2][2];
    RLcp19_314 = ROcp19_66*s->dpt[2][2];
    OMcp19_114 = OMcp19_16+ROcp19_46*qd[14];
    OMcp19_214 = OMcp19_26+ROcp19_56*qd[14];
    OMcp19_314 = OMcp19_36+ROcp19_66*qd[14];
    ORcp19_114 = OMcp19_26*RLcp19_314-OMcp19_36*RLcp19_214;
    ORcp19_214 = -(OMcp19_16*RLcp19_314-OMcp19_36*RLcp19_114);
    ORcp19_314 = OMcp19_16*RLcp19_214-OMcp19_26*RLcp19_114;
    OPcp19_114 = OPcp19_16+ROcp19_46*qdd[14]+qd[14]*(OMcp19_26*ROcp19_66-OMcp19_36*ROcp19_56);
    OPcp19_214 = OPcp19_26+ROcp19_56*qdd[14]-qd[14]*(OMcp19_16*ROcp19_66-OMcp19_36*ROcp19_46);
    OPcp19_314 = OPcp19_36+ROcp19_66*qdd[14]+qd[14]*(OMcp19_16*ROcp19_56-OMcp19_26*ROcp19_46);
    RLcp19_115 = ROcp19_46*s->dpt[2][20];
    RLcp19_215 = ROcp19_56*s->dpt[2][20];
    RLcp19_315 = ROcp19_66*s->dpt[2][20];
    OMcp19_115 = OMcp19_114+ROcp19_114*qd[15];
    OMcp19_215 = OMcp19_214+ROcp19_214*qd[15];
    OMcp19_315 = OMcp19_314+ROcp19_314*qd[15];
    ORcp19_115 = OMcp19_214*RLcp19_315-OMcp19_314*RLcp19_215;
    ORcp19_215 = -(OMcp19_114*RLcp19_315-OMcp19_314*RLcp19_115);
    ORcp19_315 = OMcp19_114*RLcp19_215-OMcp19_214*RLcp19_115;
    OPcp19_115 = OPcp19_114+ROcp19_114*qdd[15]+qd[15]*(OMcp19_214*ROcp19_314-OMcp19_314*ROcp19_214);
    OPcp19_215 = OPcp19_214+ROcp19_214*qdd[15]-qd[15]*(OMcp19_114*ROcp19_314-OMcp19_314*ROcp19_114);
    OPcp19_315 = OPcp19_314+ROcp19_314*qdd[15]+qd[15]*(OMcp19_114*ROcp19_214-OMcp19_214*ROcp19_114);
    RLcp19_116 = ROcp19_715*s->dpt[3][22];
    RLcp19_216 = ROcp19_815*s->dpt[3][22];
    RLcp19_316 = ROcp19_915*s->dpt[3][22];
    OMcp19_116 = OMcp19_115+ROcp19_715*qd[16];
    OMcp19_216 = OMcp19_215+ROcp19_815*qd[16];
    OMcp19_316 = OMcp19_315+ROcp19_915*qd[16];
    ORcp19_116 = OMcp19_215*RLcp19_316-OMcp19_315*RLcp19_216;
    ORcp19_216 = -(OMcp19_115*RLcp19_316-OMcp19_315*RLcp19_116);
    ORcp19_316 = OMcp19_115*RLcp19_216-OMcp19_215*RLcp19_116;
    OPcp19_116 = OPcp19_115+ROcp19_715*qdd[16]+qd[16]*(OMcp19_215*ROcp19_915-OMcp19_315*ROcp19_815);
    OPcp19_216 = OPcp19_215+ROcp19_815*qdd[16]-qd[16]*(OMcp19_115*ROcp19_915-OMcp19_315*ROcp19_715);
    OPcp19_316 = OPcp19_315+ROcp19_915*qdd[16]+qd[16]*(OMcp19_115*ROcp19_815-OMcp19_215*ROcp19_715);
    RLcp19_117 = ROcp19_715*s->dpt[3][24];
    RLcp19_217 = ROcp19_815*s->dpt[3][24];
    RLcp19_317 = ROcp19_915*s->dpt[3][24];
    OMcp19_117 = OMcp19_116+ROcp19_416*qd[17];
    OMcp19_217 = OMcp19_216+ROcp19_516*qd[17];
    OMcp19_317 = OMcp19_316+ROcp19_616*qd[17];
    ORcp19_117 = OMcp19_216*RLcp19_317-OMcp19_316*RLcp19_217;
    ORcp19_217 = -(OMcp19_116*RLcp19_317-OMcp19_316*RLcp19_117);
    ORcp19_317 = OMcp19_116*RLcp19_217-OMcp19_216*RLcp19_117;
    OPcp19_117 = OPcp19_116+ROcp19_416*qdd[17]+qd[17]*(OMcp19_216*ROcp19_616-OMcp19_316*ROcp19_516);
    OPcp19_217 = OPcp19_216+ROcp19_516*qdd[17]-qd[17]*(OMcp19_116*ROcp19_616-OMcp19_316*ROcp19_416);
    OPcp19_317 = OPcp19_316+ROcp19_616*qdd[17]+qd[17]*(OMcp19_116*ROcp19_516-OMcp19_216*ROcp19_416);
    RLcp19_118 = ROcp19_717*s->dpt[3][26];
    RLcp19_218 = ROcp19_817*s->dpt[3][26];
    RLcp19_318 = ROcp19_917*s->dpt[3][26];
    OMcp19_118 = OMcp19_117+ROcp19_117*qd[18];
    OMcp19_218 = OMcp19_217+ROcp19_217*qd[18];
    OMcp19_318 = OMcp19_317+ROcp19_317*qd[18];
    ORcp19_118 = OMcp19_217*RLcp19_318-OMcp19_317*RLcp19_218;
    ORcp19_218 = -(OMcp19_117*RLcp19_318-OMcp19_317*RLcp19_118);
    ORcp19_318 = OMcp19_117*RLcp19_218-OMcp19_217*RLcp19_118;
    OMcp19_119 = OMcp19_118+ROcp19_418*qd[19];
    OMcp19_219 = OMcp19_218+ROcp19_518*qd[19];
    OMcp19_319 = OMcp19_318+ROcp19_618*qd[19];
    OPcp19_119 = OPcp19_117+ROcp19_117*qdd[18]+ROcp19_418*qdd[19]+qd[18]*(OMcp19_217*ROcp19_317-OMcp19_317*ROcp19_217)+
 qd[19]*(OMcp19_218*ROcp19_618-OMcp19_318*ROcp19_518);
    OPcp19_219 = OPcp19_217+ROcp19_217*qdd[18]+ROcp19_518*qdd[19]-qd[18]*(OMcp19_117*ROcp19_317-OMcp19_317*ROcp19_117)-
 qd[19]*(OMcp19_118*ROcp19_618-OMcp19_318*ROcp19_418);
    OPcp19_319 = OPcp19_317+ROcp19_317*qdd[18]+ROcp19_618*qdd[19]+qd[18]*(OMcp19_117*ROcp19_217-OMcp19_217*ROcp19_117)+
 qd[19]*(OMcp19_118*ROcp19_518-OMcp19_218*ROcp19_418);
    RLcp19_120 = ROcp19_119*s->dpt[1][32]+ROcp19_719*s->dpt[3][32];
    RLcp19_220 = ROcp19_219*s->dpt[1][32]+ROcp19_819*s->dpt[3][32];
    RLcp19_320 = ROcp19_319*s->dpt[1][32]+ROcp19_919*s->dpt[3][32];
    POcp19_120 = RLcp19_114+RLcp19_115+RLcp19_116+RLcp19_117+RLcp19_118+RLcp19_120+q[1];
    POcp19_220 = RLcp19_214+RLcp19_215+RLcp19_216+RLcp19_217+RLcp19_218+RLcp19_220+q[2];
    POcp19_320 = RLcp19_314+RLcp19_315+RLcp19_316+RLcp19_317+RLcp19_318+RLcp19_320+q[3];
    OMcp19_120 = OMcp19_119+ROcp19_418*qd[20];
    OMcp19_220 = OMcp19_219+ROcp19_518*qd[20];
    OMcp19_320 = OMcp19_319+ROcp19_618*qd[20];
    ORcp19_120 = OMcp19_219*RLcp19_320-OMcp19_319*RLcp19_220;
    ORcp19_220 = -(OMcp19_119*RLcp19_320-OMcp19_319*RLcp19_120);
    ORcp19_320 = OMcp19_119*RLcp19_220-OMcp19_219*RLcp19_120;
    VIcp19_120 = ORcp19_114+ORcp19_115+ORcp19_116+ORcp19_117+ORcp19_118+ORcp19_120+qd[1];
    VIcp19_220 = ORcp19_214+ORcp19_215+ORcp19_216+ORcp19_217+ORcp19_218+ORcp19_220+qd[2];
    VIcp19_320 = ORcp19_314+ORcp19_315+ORcp19_316+ORcp19_317+ORcp19_318+ORcp19_320+qd[3];
    OPcp19_120 = OPcp19_119+ROcp19_418*qdd[20]+qd[20]*(OMcp19_219*ROcp19_618-OMcp19_319*ROcp19_518);
    OPcp19_220 = OPcp19_219+ROcp19_518*qdd[20]-qd[20]*(OMcp19_119*ROcp19_618-OMcp19_319*ROcp19_418);
    OPcp19_320 = OPcp19_319+ROcp19_618*qdd[20]+qd[20]*(OMcp19_119*ROcp19_518-OMcp19_219*ROcp19_418);
    ACcp19_120 = qdd[1]+OMcp19_214*ORcp19_315+OMcp19_215*ORcp19_316+OMcp19_216*ORcp19_317+OMcp19_217*ORcp19_318+OMcp19_219
 *ORcp19_320+OMcp19_26*ORcp19_314-OMcp19_314*ORcp19_215-OMcp19_315*ORcp19_216-OMcp19_316*ORcp19_217-OMcp19_317*ORcp19_218-
 OMcp19_319*ORcp19_220-OMcp19_36*ORcp19_214+OPcp19_214*RLcp19_315+OPcp19_215*RLcp19_316+OPcp19_216*RLcp19_317+OPcp19_217*
 RLcp19_318+OPcp19_219*RLcp19_320+OPcp19_26*RLcp19_314-OPcp19_314*RLcp19_215-OPcp19_315*RLcp19_216-OPcp19_316*RLcp19_217-
 OPcp19_317*RLcp19_218-OPcp19_319*RLcp19_220-OPcp19_36*RLcp19_214;
    ACcp19_220 = qdd[2]-OMcp19_114*ORcp19_315-OMcp19_115*ORcp19_316-OMcp19_116*ORcp19_317-OMcp19_117*ORcp19_318-OMcp19_119
 *ORcp19_320-OMcp19_16*ORcp19_314+OMcp19_314*ORcp19_115+OMcp19_315*ORcp19_116+OMcp19_316*ORcp19_117+OMcp19_317*ORcp19_118+
 OMcp19_319*ORcp19_120+OMcp19_36*ORcp19_114-OPcp19_114*RLcp19_315-OPcp19_115*RLcp19_316-OPcp19_116*RLcp19_317-OPcp19_117*
 RLcp19_318-OPcp19_119*RLcp19_320-OPcp19_16*RLcp19_314+OPcp19_314*RLcp19_115+OPcp19_315*RLcp19_116+OPcp19_316*RLcp19_117+
 OPcp19_317*RLcp19_118+OPcp19_319*RLcp19_120+OPcp19_36*RLcp19_114;
    ACcp19_320 = qdd[3]+OMcp19_114*ORcp19_215+OMcp19_115*ORcp19_216+OMcp19_116*ORcp19_217+OMcp19_117*ORcp19_218+OMcp19_119
 *ORcp19_220+OMcp19_16*ORcp19_214-OMcp19_214*ORcp19_115-OMcp19_215*ORcp19_116-OMcp19_216*ORcp19_117-OMcp19_217*ORcp19_118-
 OMcp19_219*ORcp19_120-OMcp19_26*ORcp19_114+OPcp19_114*RLcp19_215+OPcp19_115*RLcp19_216+OPcp19_116*RLcp19_217+OPcp19_117*
 RLcp19_218+OPcp19_119*RLcp19_220+OPcp19_16*RLcp19_214-OPcp19_214*RLcp19_115-OPcp19_215*RLcp19_116-OPcp19_216*RLcp19_117-
 OPcp19_217*RLcp19_118-OPcp19_219*RLcp19_120-OPcp19_26*RLcp19_114;

// = = Block_1_0_0_20_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp19_120;
    sens->P[2] = POcp19_220;
    sens->P[3] = POcp19_320;
    sens->R[1][1] = ROcp19_120;
    sens->R[1][2] = ROcp19_220;
    sens->R[1][3] = ROcp19_320;
    sens->R[2][1] = ROcp19_418;
    sens->R[2][2] = ROcp19_518;
    sens->R[2][3] = ROcp19_618;
    sens->R[3][1] = ROcp19_720;
    sens->R[3][2] = ROcp19_820;
    sens->R[3][3] = ROcp19_920;
    sens->V[1] = VIcp19_120;
    sens->V[2] = VIcp19_220;
    sens->V[3] = VIcp19_320;
    sens->OM[1] = OMcp19_120;
    sens->OM[2] = OMcp19_220;
    sens->OM[3] = OMcp19_320;
    sens->A[1] = ACcp19_120;
    sens->A[2] = ACcp19_220;
    sens->A[3] = ACcp19_320;
    sens->OMP[1] = OPcp19_120;
    sens->OMP[2] = OPcp19_220;
    sens->OMP[3] = OPcp19_320;
 
// 
break;
case 21:
 


// = = Block_1_0_0_21_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp20_25 = qd[5]*C4;
    OMcp20_35 = qd[5]*S4;
    OMcp20_16 = qd[4]+qd[6]*S5;
    OMcp20_26 = OMcp20_25+ROcp20_85*qd[6];
    OMcp20_36 = OMcp20_35+ROcp20_95*qd[6];
    OPcp20_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp20_26 = ROcp20_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp20_35*S5-ROcp20_95*qd[4]);
    OPcp20_36 = ROcp20_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp20_25*S5-ROcp20_85*qd[4]);

// = = Block_1_0_0_21_0_4 = = 
 
// Sensor Kinematics 


    ROcp20_421 = ROcp20_46*C21+S21*S5;
    ROcp20_521 = ROcp20_56*C21+ROcp20_85*S21;
    ROcp20_621 = ROcp20_66*C21+ROcp20_95*S21;
    ROcp20_721 = -(ROcp20_46*S21-C21*S5);
    ROcp20_821 = -(ROcp20_56*S21-ROcp20_85*C21);
    ROcp20_921 = -(ROcp20_66*S21-ROcp20_95*C21);
    RLcp20_121 = ROcp20_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp20_221 = ROcp20_26*s->dpt[1][3]+ROcp20_85*s->dpt[3][3];
    RLcp20_321 = ROcp20_36*s->dpt[1][3]+ROcp20_95*s->dpt[3][3];
    POcp20_121 = RLcp20_121+q[1];
    POcp20_221 = RLcp20_221+q[2];
    POcp20_321 = RLcp20_321+q[3];
    OMcp20_121 = OMcp20_16+ROcp20_16*qd[21];
    OMcp20_221 = OMcp20_26+ROcp20_26*qd[21];
    OMcp20_321 = OMcp20_36+ROcp20_36*qd[21];
    ORcp20_121 = OMcp20_26*RLcp20_321-OMcp20_36*RLcp20_221;
    ORcp20_221 = -(OMcp20_16*RLcp20_321-OMcp20_36*RLcp20_121);
    ORcp20_321 = OMcp20_16*RLcp20_221-OMcp20_26*RLcp20_121;
    VIcp20_121 = ORcp20_121+qd[1];
    VIcp20_221 = ORcp20_221+qd[2];
    VIcp20_321 = ORcp20_321+qd[3];
    OPcp20_121 = OPcp20_16+ROcp20_16*qdd[21]+qd[21]*(OMcp20_26*ROcp20_36-OMcp20_36*ROcp20_26);
    OPcp20_221 = OPcp20_26+ROcp20_26*qdd[21]-qd[21]*(OMcp20_16*ROcp20_36-OMcp20_36*ROcp20_16);
    OPcp20_321 = OPcp20_36+ROcp20_36*qdd[21]+qd[21]*(OMcp20_16*ROcp20_26-OMcp20_26*ROcp20_16);
    ACcp20_121 = qdd[1]+OMcp20_26*ORcp20_321-OMcp20_36*ORcp20_221+OPcp20_26*RLcp20_321-OPcp20_36*RLcp20_221;
    ACcp20_221 = qdd[2]-OMcp20_16*ORcp20_321+OMcp20_36*ORcp20_121-OPcp20_16*RLcp20_321+OPcp20_36*RLcp20_121;
    ACcp20_321 = qdd[3]+OMcp20_16*ORcp20_221-OMcp20_26*ORcp20_121+OPcp20_16*RLcp20_221-OPcp20_26*RLcp20_121;

// = = Block_1_0_0_21_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp20_121;
    sens->P[2] = POcp20_221;
    sens->P[3] = POcp20_321;
    sens->R[1][1] = ROcp20_16;
    sens->R[1][2] = ROcp20_26;
    sens->R[1][3] = ROcp20_36;
    sens->R[2][1] = ROcp20_421;
    sens->R[2][2] = ROcp20_521;
    sens->R[2][3] = ROcp20_621;
    sens->R[3][1] = ROcp20_721;
    sens->R[3][2] = ROcp20_821;
    sens->R[3][3] = ROcp20_921;
    sens->V[1] = VIcp20_121;
    sens->V[2] = VIcp20_221;
    sens->V[3] = VIcp20_321;
    sens->OM[1] = OMcp20_121;
    sens->OM[2] = OMcp20_221;
    sens->OM[3] = OMcp20_321;
    sens->A[1] = ACcp20_121;
    sens->A[2] = ACcp20_221;
    sens->A[3] = ACcp20_321;
    sens->OMP[1] = OPcp20_121;
    sens->OMP[2] = OPcp20_221;
    sens->OMP[3] = OPcp20_321;
 
// 
break;
case 22:
 


// = = Block_1_0_0_22_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp21_25 = qd[5]*C4;
    OMcp21_35 = qd[5]*S4;
    OMcp21_16 = qd[4]+qd[6]*S5;
    OMcp21_26 = OMcp21_25+ROcp21_85*qd[6];
    OMcp21_36 = OMcp21_35+ROcp21_95*qd[6];
    OPcp21_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp21_26 = ROcp21_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp21_35*S5-ROcp21_95*qd[4]);
    OPcp21_36 = ROcp21_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp21_25*S5-ROcp21_85*qd[4]);

// = = Block_1_0_0_22_0_4 = = 
 
// Sensor Kinematics 


    ROcp21_421 = ROcp21_46*C21+S21*S5;
    ROcp21_521 = ROcp21_56*C21+ROcp21_85*S21;
    ROcp21_621 = ROcp21_66*C21+ROcp21_95*S21;
    ROcp21_721 = -(ROcp21_46*S21-C21*S5);
    ROcp21_821 = -(ROcp21_56*S21-ROcp21_85*C21);
    ROcp21_921 = -(ROcp21_66*S21-ROcp21_95*C21);
    ROcp21_122 = ROcp21_16*C22-ROcp21_721*S22;
    ROcp21_222 = ROcp21_26*C22-ROcp21_821*S22;
    ROcp21_322 = ROcp21_36*C22-ROcp21_921*S22;
    ROcp21_722 = ROcp21_16*S22+ROcp21_721*C22;
    ROcp21_822 = ROcp21_26*S22+ROcp21_821*C22;
    ROcp21_922 = ROcp21_36*S22+ROcp21_921*C22;
    RLcp21_121 = ROcp21_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp21_221 = ROcp21_26*s->dpt[1][3]+ROcp21_85*s->dpt[3][3];
    RLcp21_321 = ROcp21_36*s->dpt[1][3]+ROcp21_95*s->dpt[3][3];
    POcp21_121 = RLcp21_121+q[1];
    POcp21_221 = RLcp21_221+q[2];
    POcp21_321 = RLcp21_321+q[3];
    OMcp21_121 = OMcp21_16+ROcp21_16*qd[21];
    OMcp21_221 = OMcp21_26+ROcp21_26*qd[21];
    OMcp21_321 = OMcp21_36+ROcp21_36*qd[21];
    ORcp21_121 = OMcp21_26*RLcp21_321-OMcp21_36*RLcp21_221;
    ORcp21_221 = -(OMcp21_16*RLcp21_321-OMcp21_36*RLcp21_121);
    ORcp21_321 = OMcp21_16*RLcp21_221-OMcp21_26*RLcp21_121;
    VIcp21_121 = ORcp21_121+qd[1];
    VIcp21_221 = ORcp21_221+qd[2];
    VIcp21_321 = ORcp21_321+qd[3];
    ACcp21_121 = qdd[1]+OMcp21_26*ORcp21_321-OMcp21_36*ORcp21_221+OPcp21_26*RLcp21_321-OPcp21_36*RLcp21_221;
    ACcp21_221 = qdd[2]-OMcp21_16*ORcp21_321+OMcp21_36*ORcp21_121-OPcp21_16*RLcp21_321+OPcp21_36*RLcp21_121;
    ACcp21_321 = qdd[3]+OMcp21_16*ORcp21_221-OMcp21_26*ORcp21_121+OPcp21_16*RLcp21_221-OPcp21_26*RLcp21_121;
    OMcp21_122 = OMcp21_121+ROcp21_421*qd[22];
    OMcp21_222 = OMcp21_221+ROcp21_521*qd[22];
    OMcp21_322 = OMcp21_321+ROcp21_621*qd[22];
    OPcp21_122 = OPcp21_16+ROcp21_16*qdd[21]+ROcp21_421*qdd[22]+qd[21]*(OMcp21_26*ROcp21_36-OMcp21_36*ROcp21_26)+qd[22]*(
 OMcp21_221*ROcp21_621-OMcp21_321*ROcp21_521);
    OPcp21_222 = OPcp21_26+ROcp21_26*qdd[21]+ROcp21_521*qdd[22]-qd[21]*(OMcp21_16*ROcp21_36-OMcp21_36*ROcp21_16)-qd[22]*(
 OMcp21_121*ROcp21_621-OMcp21_321*ROcp21_421);
    OPcp21_322 = OPcp21_36+ROcp21_36*qdd[21]+ROcp21_621*qdd[22]+qd[21]*(OMcp21_16*ROcp21_26-OMcp21_26*ROcp21_16)+qd[22]*(
 OMcp21_121*ROcp21_521-OMcp21_221*ROcp21_421);

// = = Block_1_0_0_22_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp21_121;
    sens->P[2] = POcp21_221;
    sens->P[3] = POcp21_321;
    sens->R[1][1] = ROcp21_122;
    sens->R[1][2] = ROcp21_222;
    sens->R[1][3] = ROcp21_322;
    sens->R[2][1] = ROcp21_421;
    sens->R[2][2] = ROcp21_521;
    sens->R[2][3] = ROcp21_621;
    sens->R[3][1] = ROcp21_722;
    sens->R[3][2] = ROcp21_822;
    sens->R[3][3] = ROcp21_922;
    sens->V[1] = VIcp21_121;
    sens->V[2] = VIcp21_221;
    sens->V[3] = VIcp21_321;
    sens->OM[1] = OMcp21_122;
    sens->OM[2] = OMcp21_222;
    sens->OM[3] = OMcp21_322;
    sens->A[1] = ACcp21_121;
    sens->A[2] = ACcp21_221;
    sens->A[3] = ACcp21_321;
    sens->OMP[1] = OPcp21_122;
    sens->OMP[2] = OPcp21_222;
    sens->OMP[3] = OPcp21_322;
 
// 
break;
case 23:
 


// = = Block_1_0_0_23_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp22_25 = qd[5]*C4;
    OMcp22_35 = qd[5]*S4;
    OMcp22_16 = qd[4]+qd[6]*S5;
    OMcp22_26 = OMcp22_25+ROcp22_85*qd[6];
    OMcp22_36 = OMcp22_35+ROcp22_95*qd[6];
    OPcp22_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp22_26 = ROcp22_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp22_35*S5-ROcp22_95*qd[4]);
    OPcp22_36 = ROcp22_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp22_25*S5-ROcp22_85*qd[4]);

// = = Block_1_0_0_23_0_4 = = 
 
// Sensor Kinematics 


    ROcp22_421 = ROcp22_46*C21+S21*S5;
    ROcp22_521 = ROcp22_56*C21+ROcp22_85*S21;
    ROcp22_621 = ROcp22_66*C21+ROcp22_95*S21;
    ROcp22_721 = -(ROcp22_46*S21-C21*S5);
    ROcp22_821 = -(ROcp22_56*S21-ROcp22_85*C21);
    ROcp22_921 = -(ROcp22_66*S21-ROcp22_95*C21);
    ROcp22_122 = ROcp22_16*C22-ROcp22_721*S22;
    ROcp22_222 = ROcp22_26*C22-ROcp22_821*S22;
    ROcp22_322 = ROcp22_36*C22-ROcp22_921*S22;
    ROcp22_722 = ROcp22_16*S22+ROcp22_721*C22;
    ROcp22_822 = ROcp22_26*S22+ROcp22_821*C22;
    ROcp22_922 = ROcp22_36*S22+ROcp22_921*C22;
    ROcp22_123 = ROcp22_122*C23+ROcp22_421*S23;
    ROcp22_223 = ROcp22_222*C23+ROcp22_521*S23;
    ROcp22_323 = ROcp22_322*C23+ROcp22_621*S23;
    ROcp22_423 = -(ROcp22_122*S23-ROcp22_421*C23);
    ROcp22_523 = -(ROcp22_222*S23-ROcp22_521*C23);
    ROcp22_623 = -(ROcp22_322*S23-ROcp22_621*C23);
    RLcp22_121 = ROcp22_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp22_221 = ROcp22_26*s->dpt[1][3]+ROcp22_85*s->dpt[3][3];
    RLcp22_321 = ROcp22_36*s->dpt[1][3]+ROcp22_95*s->dpt[3][3];
    OMcp22_121 = OMcp22_16+ROcp22_16*qd[21];
    OMcp22_221 = OMcp22_26+ROcp22_26*qd[21];
    OMcp22_321 = OMcp22_36+ROcp22_36*qd[21];
    ORcp22_121 = OMcp22_26*RLcp22_321-OMcp22_36*RLcp22_221;
    ORcp22_221 = -(OMcp22_16*RLcp22_321-OMcp22_36*RLcp22_121);
    ORcp22_321 = OMcp22_16*RLcp22_221-OMcp22_26*RLcp22_121;
    OMcp22_122 = OMcp22_121+ROcp22_421*qd[22];
    OMcp22_222 = OMcp22_221+ROcp22_521*qd[22];
    OMcp22_322 = OMcp22_321+ROcp22_621*qd[22];
    OPcp22_122 = OPcp22_16+ROcp22_16*qdd[21]+ROcp22_421*qdd[22]+qd[21]*(OMcp22_26*ROcp22_36-OMcp22_36*ROcp22_26)+qd[22]*(
 OMcp22_221*ROcp22_621-OMcp22_321*ROcp22_521);
    OPcp22_222 = OPcp22_26+ROcp22_26*qdd[21]+ROcp22_521*qdd[22]-qd[21]*(OMcp22_16*ROcp22_36-OMcp22_36*ROcp22_16)-qd[22]*(
 OMcp22_121*ROcp22_621-OMcp22_321*ROcp22_421);
    OPcp22_322 = OPcp22_36+ROcp22_36*qdd[21]+ROcp22_621*qdd[22]+qd[21]*(OMcp22_16*ROcp22_26-OMcp22_26*ROcp22_16)+qd[22]*(
 OMcp22_121*ROcp22_521-OMcp22_221*ROcp22_421);
    RLcp22_123 = ROcp22_722*s->dpt[3][37];
    RLcp22_223 = ROcp22_822*s->dpt[3][37];
    RLcp22_323 = ROcp22_922*s->dpt[3][37];
    POcp22_123 = RLcp22_121+RLcp22_123+q[1];
    POcp22_223 = RLcp22_221+RLcp22_223+q[2];
    POcp22_323 = RLcp22_321+RLcp22_323+q[3];
    OMcp22_123 = OMcp22_122+ROcp22_722*qd[23];
    OMcp22_223 = OMcp22_222+ROcp22_822*qd[23];
    OMcp22_323 = OMcp22_322+ROcp22_922*qd[23];
    ORcp22_123 = OMcp22_222*RLcp22_323-OMcp22_322*RLcp22_223;
    ORcp22_223 = -(OMcp22_122*RLcp22_323-OMcp22_322*RLcp22_123);
    ORcp22_323 = OMcp22_122*RLcp22_223-OMcp22_222*RLcp22_123;
    VIcp22_123 = ORcp22_121+ORcp22_123+qd[1];
    VIcp22_223 = ORcp22_221+ORcp22_223+qd[2];
    VIcp22_323 = ORcp22_321+ORcp22_323+qd[3];
    OPcp22_123 = OPcp22_122+ROcp22_722*qdd[23]+qd[23]*(OMcp22_222*ROcp22_922-OMcp22_322*ROcp22_822);
    OPcp22_223 = OPcp22_222+ROcp22_822*qdd[23]-qd[23]*(OMcp22_122*ROcp22_922-OMcp22_322*ROcp22_722);
    OPcp22_323 = OPcp22_322+ROcp22_922*qdd[23]+qd[23]*(OMcp22_122*ROcp22_822-OMcp22_222*ROcp22_722);
    ACcp22_123 = qdd[1]+OMcp22_222*ORcp22_323+OMcp22_26*ORcp22_321-OMcp22_322*ORcp22_223-OMcp22_36*ORcp22_221+OPcp22_222*
 RLcp22_323+OPcp22_26*RLcp22_321-OPcp22_322*RLcp22_223-OPcp22_36*RLcp22_221;
    ACcp22_223 = qdd[2]-OMcp22_122*ORcp22_323-OMcp22_16*ORcp22_321+OMcp22_322*ORcp22_123+OMcp22_36*ORcp22_121-OPcp22_122*
 RLcp22_323-OPcp22_16*RLcp22_321+OPcp22_322*RLcp22_123+OPcp22_36*RLcp22_121;
    ACcp22_323 = qdd[3]+OMcp22_122*ORcp22_223+OMcp22_16*ORcp22_221-OMcp22_222*ORcp22_123-OMcp22_26*ORcp22_121+OPcp22_122*
 RLcp22_223+OPcp22_16*RLcp22_221-OPcp22_222*RLcp22_123-OPcp22_26*RLcp22_121;

// = = Block_1_0_0_23_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp22_123;
    sens->P[2] = POcp22_223;
    sens->P[3] = POcp22_323;
    sens->R[1][1] = ROcp22_123;
    sens->R[1][2] = ROcp22_223;
    sens->R[1][3] = ROcp22_323;
    sens->R[2][1] = ROcp22_423;
    sens->R[2][2] = ROcp22_523;
    sens->R[2][3] = ROcp22_623;
    sens->R[3][1] = ROcp22_722;
    sens->R[3][2] = ROcp22_822;
    sens->R[3][3] = ROcp22_922;
    sens->V[1] = VIcp22_123;
    sens->V[2] = VIcp22_223;
    sens->V[3] = VIcp22_323;
    sens->OM[1] = OMcp22_123;
    sens->OM[2] = OMcp22_223;
    sens->OM[3] = OMcp22_323;
    sens->A[1] = ACcp22_123;
    sens->A[2] = ACcp22_223;
    sens->A[3] = ACcp22_323;
    sens->OMP[1] = OPcp22_123;
    sens->OMP[2] = OPcp22_223;
    sens->OMP[3] = OPcp22_323;
 
// 
break;
case 24:
 


// = = Block_1_0_0_24_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp23_25 = qd[5]*C4;
    OMcp23_35 = qd[5]*S4;
    OMcp23_16 = qd[4]+qd[6]*S5;
    OMcp23_26 = OMcp23_25+ROcp23_85*qd[6];
    OMcp23_36 = OMcp23_35+ROcp23_95*qd[6];
    OPcp23_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp23_26 = ROcp23_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp23_35*S5-ROcp23_95*qd[4]);
    OPcp23_36 = ROcp23_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp23_25*S5-ROcp23_85*qd[4]);

// = = Block_1_0_0_24_0_4 = = 
 
// Sensor Kinematics 


    ROcp23_421 = ROcp23_46*C21+S21*S5;
    ROcp23_521 = ROcp23_56*C21+ROcp23_85*S21;
    ROcp23_621 = ROcp23_66*C21+ROcp23_95*S21;
    ROcp23_721 = -(ROcp23_46*S21-C21*S5);
    ROcp23_821 = -(ROcp23_56*S21-ROcp23_85*C21);
    ROcp23_921 = -(ROcp23_66*S21-ROcp23_95*C21);
    ROcp23_122 = ROcp23_16*C22-ROcp23_721*S22;
    ROcp23_222 = ROcp23_26*C22-ROcp23_821*S22;
    ROcp23_322 = ROcp23_36*C22-ROcp23_921*S22;
    ROcp23_722 = ROcp23_16*S22+ROcp23_721*C22;
    ROcp23_822 = ROcp23_26*S22+ROcp23_821*C22;
    ROcp23_922 = ROcp23_36*S22+ROcp23_921*C22;
    ROcp23_123 = ROcp23_122*C23+ROcp23_421*S23;
    ROcp23_223 = ROcp23_222*C23+ROcp23_521*S23;
    ROcp23_323 = ROcp23_322*C23+ROcp23_621*S23;
    ROcp23_423 = -(ROcp23_122*S23-ROcp23_421*C23);
    ROcp23_523 = -(ROcp23_222*S23-ROcp23_521*C23);
    ROcp23_623 = -(ROcp23_322*S23-ROcp23_621*C23);
    RLcp23_121 = ROcp23_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp23_221 = ROcp23_26*s->dpt[1][3]+ROcp23_85*s->dpt[3][3];
    RLcp23_321 = ROcp23_36*s->dpt[1][3]+ROcp23_95*s->dpt[3][3];
    OMcp23_121 = OMcp23_16+ROcp23_16*qd[21];
    OMcp23_221 = OMcp23_26+ROcp23_26*qd[21];
    OMcp23_321 = OMcp23_36+ROcp23_36*qd[21];
    ORcp23_121 = OMcp23_26*RLcp23_321-OMcp23_36*RLcp23_221;
    ORcp23_221 = -(OMcp23_16*RLcp23_321-OMcp23_36*RLcp23_121);
    ORcp23_321 = OMcp23_16*RLcp23_221-OMcp23_26*RLcp23_121;
    OMcp23_122 = OMcp23_121+ROcp23_421*qd[22];
    OMcp23_222 = OMcp23_221+ROcp23_521*qd[22];
    OMcp23_322 = OMcp23_321+ROcp23_621*qd[22];
    OPcp23_122 = OPcp23_16+ROcp23_16*qdd[21]+ROcp23_421*qdd[22]+qd[21]*(OMcp23_26*ROcp23_36-OMcp23_36*ROcp23_26)+qd[22]*(
 OMcp23_221*ROcp23_621-OMcp23_321*ROcp23_521);
    OPcp23_222 = OPcp23_26+ROcp23_26*qdd[21]+ROcp23_521*qdd[22]-qd[21]*(OMcp23_16*ROcp23_36-OMcp23_36*ROcp23_16)-qd[22]*(
 OMcp23_121*ROcp23_621-OMcp23_321*ROcp23_421);
    OPcp23_322 = OPcp23_36+ROcp23_36*qdd[21]+ROcp23_621*qdd[22]+qd[21]*(OMcp23_16*ROcp23_26-OMcp23_26*ROcp23_16)+qd[22]*(
 OMcp23_121*ROcp23_521-OMcp23_221*ROcp23_421);
    RLcp23_123 = ROcp23_722*s->dpt[3][37];
    RLcp23_223 = ROcp23_822*s->dpt[3][37];
    RLcp23_323 = ROcp23_922*s->dpt[3][37];
    OMcp23_123 = OMcp23_122+ROcp23_722*qd[23];
    OMcp23_223 = OMcp23_222+ROcp23_822*qd[23];
    OMcp23_323 = OMcp23_322+ROcp23_922*qd[23];
    ORcp23_123 = OMcp23_222*RLcp23_323-OMcp23_322*RLcp23_223;
    ORcp23_223 = -(OMcp23_122*RLcp23_323-OMcp23_322*RLcp23_123);
    ORcp23_323 = OMcp23_122*RLcp23_223-OMcp23_222*RLcp23_123;
    OPcp23_123 = OPcp23_122+ROcp23_722*qdd[23]+qd[23]*(OMcp23_222*ROcp23_922-OMcp23_322*ROcp23_822);
    OPcp23_223 = OPcp23_222+ROcp23_822*qdd[23]-qd[23]*(OMcp23_122*ROcp23_922-OMcp23_322*ROcp23_722);
    OPcp23_323 = OPcp23_322+ROcp23_922*qdd[23]+qd[23]*(OMcp23_122*ROcp23_822-OMcp23_222*ROcp23_722);

// = = Block_1_0_0_24_0_5 = = 
 
// Sensor Kinematics 


    ROcp23_124 = ROcp23_123*C24-ROcp23_722*S24;
    ROcp23_224 = ROcp23_223*C24-ROcp23_822*S24;
    ROcp23_324 = ROcp23_323*C24-ROcp23_922*S24;
    ROcp23_724 = ROcp23_123*S24+ROcp23_722*C24;
    ROcp23_824 = ROcp23_223*S24+ROcp23_822*C24;
    ROcp23_924 = ROcp23_323*S24+ROcp23_922*C24;
    RLcp23_124 = ROcp23_123*s->dpt[1][41]+ROcp23_423*s->dpt[2][41]+ROcp23_722*s->dpt[3][41];
    RLcp23_224 = ROcp23_223*s->dpt[1][41]+ROcp23_523*s->dpt[2][41]+ROcp23_822*s->dpt[3][41];
    RLcp23_324 = ROcp23_323*s->dpt[1][41]+ROcp23_623*s->dpt[2][41]+ROcp23_922*s->dpt[3][41];
    POcp23_124 = RLcp23_121+RLcp23_123+RLcp23_124+q[1];
    POcp23_224 = RLcp23_221+RLcp23_223+RLcp23_224+q[2];
    POcp23_324 = RLcp23_321+RLcp23_323+RLcp23_324+q[3];
    OMcp23_124 = OMcp23_123+ROcp23_423*qd[24];
    OMcp23_224 = OMcp23_223+ROcp23_523*qd[24];
    OMcp23_324 = OMcp23_323+ROcp23_623*qd[24];
    ORcp23_124 = OMcp23_223*RLcp23_324-OMcp23_323*RLcp23_224;
    ORcp23_224 = -(OMcp23_123*RLcp23_324-OMcp23_323*RLcp23_124);
    ORcp23_324 = OMcp23_123*RLcp23_224-OMcp23_223*RLcp23_124;
    VIcp23_124 = ORcp23_121+ORcp23_123+ORcp23_124+qd[1];
    VIcp23_224 = ORcp23_221+ORcp23_223+ORcp23_224+qd[2];
    VIcp23_324 = ORcp23_321+ORcp23_323+ORcp23_324+qd[3];
    OPcp23_124 = OPcp23_123+ROcp23_423*qdd[24]+qd[24]*(OMcp23_223*ROcp23_623-OMcp23_323*ROcp23_523);
    OPcp23_224 = OPcp23_223+ROcp23_523*qdd[24]-qd[24]*(OMcp23_123*ROcp23_623-OMcp23_323*ROcp23_423);
    OPcp23_324 = OPcp23_323+ROcp23_623*qdd[24]+qd[24]*(OMcp23_123*ROcp23_523-OMcp23_223*ROcp23_423);
    ACcp23_124 = qdd[1]+OMcp23_222*ORcp23_323+OMcp23_223*ORcp23_324+OMcp23_26*ORcp23_321-OMcp23_322*ORcp23_223-OMcp23_323*
 ORcp23_224-OMcp23_36*ORcp23_221+OPcp23_222*RLcp23_323+OPcp23_223*RLcp23_324+OPcp23_26*RLcp23_321-OPcp23_322*RLcp23_223-
 OPcp23_323*RLcp23_224-OPcp23_36*RLcp23_221;
    ACcp23_224 = qdd[2]-OMcp23_122*ORcp23_323-OMcp23_123*ORcp23_324-OMcp23_16*ORcp23_321+OMcp23_322*ORcp23_123+OMcp23_323*
 ORcp23_124+OMcp23_36*ORcp23_121-OPcp23_122*RLcp23_323-OPcp23_123*RLcp23_324-OPcp23_16*RLcp23_321+OPcp23_322*RLcp23_123+
 OPcp23_323*RLcp23_124+OPcp23_36*RLcp23_121;
    ACcp23_324 = qdd[3]+OMcp23_122*ORcp23_223+OMcp23_123*ORcp23_224+OMcp23_16*ORcp23_221-OMcp23_222*ORcp23_123-OMcp23_223*
 ORcp23_124-OMcp23_26*ORcp23_121+OPcp23_122*RLcp23_223+OPcp23_123*RLcp23_224+OPcp23_16*RLcp23_221-OPcp23_222*RLcp23_123-
 OPcp23_223*RLcp23_124-OPcp23_26*RLcp23_121;

// = = Block_1_0_0_24_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp23_124;
    sens->P[2] = POcp23_224;
    sens->P[3] = POcp23_324;
    sens->R[1][1] = ROcp23_124;
    sens->R[1][2] = ROcp23_224;
    sens->R[1][3] = ROcp23_324;
    sens->R[2][1] = ROcp23_423;
    sens->R[2][2] = ROcp23_523;
    sens->R[2][3] = ROcp23_623;
    sens->R[3][1] = ROcp23_724;
    sens->R[3][2] = ROcp23_824;
    sens->R[3][3] = ROcp23_924;
    sens->V[1] = VIcp23_124;
    sens->V[2] = VIcp23_224;
    sens->V[3] = VIcp23_324;
    sens->OM[1] = OMcp23_124;
    sens->OM[2] = OMcp23_224;
    sens->OM[3] = OMcp23_324;
    sens->A[1] = ACcp23_124;
    sens->A[2] = ACcp23_224;
    sens->A[3] = ACcp23_324;
    sens->OMP[1] = OPcp23_124;
    sens->OMP[2] = OPcp23_224;
    sens->OMP[3] = OPcp23_324;
 
// 
break;
case 25:
 


// = = Block_1_0_0_25_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp24_25 = qd[5]*C4;
    OMcp24_35 = qd[5]*S4;
    OMcp24_16 = qd[4]+qd[6]*S5;
    OMcp24_26 = OMcp24_25+ROcp24_85*qd[6];
    OMcp24_36 = OMcp24_35+ROcp24_95*qd[6];
    OPcp24_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp24_26 = ROcp24_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp24_35*S5-ROcp24_95*qd[4]);
    OPcp24_36 = ROcp24_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp24_25*S5-ROcp24_85*qd[4]);

// = = Block_1_0_0_25_0_4 = = 
 
// Sensor Kinematics 


    ROcp24_421 = ROcp24_46*C21+S21*S5;
    ROcp24_521 = ROcp24_56*C21+ROcp24_85*S21;
    ROcp24_621 = ROcp24_66*C21+ROcp24_95*S21;
    ROcp24_721 = -(ROcp24_46*S21-C21*S5);
    ROcp24_821 = -(ROcp24_56*S21-ROcp24_85*C21);
    ROcp24_921 = -(ROcp24_66*S21-ROcp24_95*C21);
    ROcp24_122 = ROcp24_16*C22-ROcp24_721*S22;
    ROcp24_222 = ROcp24_26*C22-ROcp24_821*S22;
    ROcp24_322 = ROcp24_36*C22-ROcp24_921*S22;
    ROcp24_722 = ROcp24_16*S22+ROcp24_721*C22;
    ROcp24_822 = ROcp24_26*S22+ROcp24_821*C22;
    ROcp24_922 = ROcp24_36*S22+ROcp24_921*C22;
    ROcp24_123 = ROcp24_122*C23+ROcp24_421*S23;
    ROcp24_223 = ROcp24_222*C23+ROcp24_521*S23;
    ROcp24_323 = ROcp24_322*C23+ROcp24_621*S23;
    ROcp24_423 = -(ROcp24_122*S23-ROcp24_421*C23);
    ROcp24_523 = -(ROcp24_222*S23-ROcp24_521*C23);
    ROcp24_623 = -(ROcp24_322*S23-ROcp24_621*C23);
    RLcp24_121 = ROcp24_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp24_221 = ROcp24_26*s->dpt[1][3]+ROcp24_85*s->dpt[3][3];
    RLcp24_321 = ROcp24_36*s->dpt[1][3]+ROcp24_95*s->dpt[3][3];
    OMcp24_121 = OMcp24_16+ROcp24_16*qd[21];
    OMcp24_221 = OMcp24_26+ROcp24_26*qd[21];
    OMcp24_321 = OMcp24_36+ROcp24_36*qd[21];
    ORcp24_121 = OMcp24_26*RLcp24_321-OMcp24_36*RLcp24_221;
    ORcp24_221 = -(OMcp24_16*RLcp24_321-OMcp24_36*RLcp24_121);
    ORcp24_321 = OMcp24_16*RLcp24_221-OMcp24_26*RLcp24_121;
    OMcp24_122 = OMcp24_121+ROcp24_421*qd[22];
    OMcp24_222 = OMcp24_221+ROcp24_521*qd[22];
    OMcp24_322 = OMcp24_321+ROcp24_621*qd[22];
    OPcp24_122 = OPcp24_16+ROcp24_16*qdd[21]+ROcp24_421*qdd[22]+qd[21]*(OMcp24_26*ROcp24_36-OMcp24_36*ROcp24_26)+qd[22]*(
 OMcp24_221*ROcp24_621-OMcp24_321*ROcp24_521);
    OPcp24_222 = OPcp24_26+ROcp24_26*qdd[21]+ROcp24_521*qdd[22]-qd[21]*(OMcp24_16*ROcp24_36-OMcp24_36*ROcp24_16)-qd[22]*(
 OMcp24_121*ROcp24_621-OMcp24_321*ROcp24_421);
    OPcp24_322 = OPcp24_36+ROcp24_36*qdd[21]+ROcp24_621*qdd[22]+qd[21]*(OMcp24_16*ROcp24_26-OMcp24_26*ROcp24_16)+qd[22]*(
 OMcp24_121*ROcp24_521-OMcp24_221*ROcp24_421);
    RLcp24_123 = ROcp24_722*s->dpt[3][37];
    RLcp24_223 = ROcp24_822*s->dpt[3][37];
    RLcp24_323 = ROcp24_922*s->dpt[3][37];
    OMcp24_123 = OMcp24_122+ROcp24_722*qd[23];
    OMcp24_223 = OMcp24_222+ROcp24_822*qd[23];
    OMcp24_323 = OMcp24_322+ROcp24_922*qd[23];
    ORcp24_123 = OMcp24_222*RLcp24_323-OMcp24_322*RLcp24_223;
    ORcp24_223 = -(OMcp24_122*RLcp24_323-OMcp24_322*RLcp24_123);
    ORcp24_323 = OMcp24_122*RLcp24_223-OMcp24_222*RLcp24_123;
    OPcp24_123 = OPcp24_122+ROcp24_722*qdd[23]+qd[23]*(OMcp24_222*ROcp24_922-OMcp24_322*ROcp24_822);
    OPcp24_223 = OPcp24_222+ROcp24_822*qdd[23]-qd[23]*(OMcp24_122*ROcp24_922-OMcp24_322*ROcp24_722);
    OPcp24_323 = OPcp24_322+ROcp24_922*qdd[23]+qd[23]*(OMcp24_122*ROcp24_822-OMcp24_222*ROcp24_722);

// = = Block_1_0_0_25_0_5 = = 
 
// Sensor Kinematics 


    ROcp24_124 = ROcp24_123*C24-ROcp24_722*S24;
    ROcp24_224 = ROcp24_223*C24-ROcp24_822*S24;
    ROcp24_324 = ROcp24_323*C24-ROcp24_922*S24;
    ROcp24_724 = ROcp24_123*S24+ROcp24_722*C24;
    ROcp24_824 = ROcp24_223*S24+ROcp24_822*C24;
    ROcp24_924 = ROcp24_323*S24+ROcp24_922*C24;
    ROcp24_425 = ROcp24_423*C25+ROcp24_724*S25;
    ROcp24_525 = ROcp24_523*C25+ROcp24_824*S25;
    ROcp24_625 = ROcp24_623*C25+ROcp24_924*S25;
    ROcp24_725 = -(ROcp24_423*S25-ROcp24_724*C25);
    ROcp24_825 = -(ROcp24_523*S25-ROcp24_824*C25);
    ROcp24_925 = -(ROcp24_623*S25-ROcp24_924*C25);
    RLcp24_124 = ROcp24_123*s->dpt[1][41]+ROcp24_423*s->dpt[2][41]+ROcp24_722*s->dpt[3][41];
    RLcp24_224 = ROcp24_223*s->dpt[1][41]+ROcp24_523*s->dpt[2][41]+ROcp24_822*s->dpt[3][41];
    RLcp24_324 = ROcp24_323*s->dpt[1][41]+ROcp24_623*s->dpt[2][41]+ROcp24_922*s->dpt[3][41];
    OMcp24_124 = OMcp24_123+ROcp24_423*qd[24];
    OMcp24_224 = OMcp24_223+ROcp24_523*qd[24];
    OMcp24_324 = OMcp24_323+ROcp24_623*qd[24];
    ORcp24_124 = OMcp24_223*RLcp24_324-OMcp24_323*RLcp24_224;
    ORcp24_224 = -(OMcp24_123*RLcp24_324-OMcp24_323*RLcp24_124);
    ORcp24_324 = OMcp24_123*RLcp24_224-OMcp24_223*RLcp24_124;
    OPcp24_124 = OPcp24_123+ROcp24_423*qdd[24]+qd[24]*(OMcp24_223*ROcp24_623-OMcp24_323*ROcp24_523);
    OPcp24_224 = OPcp24_223+ROcp24_523*qdd[24]-qd[24]*(OMcp24_123*ROcp24_623-OMcp24_323*ROcp24_423);
    OPcp24_324 = OPcp24_323+ROcp24_623*qdd[24]+qd[24]*(OMcp24_123*ROcp24_523-OMcp24_223*ROcp24_423);
    RLcp24_125 = ROcp24_423*s->dpt[2][44];
    RLcp24_225 = ROcp24_523*s->dpt[2][44];
    RLcp24_325 = ROcp24_623*s->dpt[2][44];
    POcp24_125 = RLcp24_121+RLcp24_123+RLcp24_124+RLcp24_125+q[1];
    POcp24_225 = RLcp24_221+RLcp24_223+RLcp24_224+RLcp24_225+q[2];
    POcp24_325 = RLcp24_321+RLcp24_323+RLcp24_324+RLcp24_325+q[3];
    OMcp24_125 = OMcp24_124+ROcp24_124*qd[25];
    OMcp24_225 = OMcp24_224+ROcp24_224*qd[25];
    OMcp24_325 = OMcp24_324+ROcp24_324*qd[25];
    ORcp24_125 = OMcp24_224*RLcp24_325-OMcp24_324*RLcp24_225;
    ORcp24_225 = -(OMcp24_124*RLcp24_325-OMcp24_324*RLcp24_125);
    ORcp24_325 = OMcp24_124*RLcp24_225-OMcp24_224*RLcp24_125;
    VIcp24_125 = ORcp24_121+ORcp24_123+ORcp24_124+ORcp24_125+qd[1];
    VIcp24_225 = ORcp24_221+ORcp24_223+ORcp24_224+ORcp24_225+qd[2];
    VIcp24_325 = ORcp24_321+ORcp24_323+ORcp24_324+ORcp24_325+qd[3];
    OPcp24_125 = OPcp24_124+ROcp24_124*qdd[25]+qd[25]*(OMcp24_224*ROcp24_324-OMcp24_324*ROcp24_224);
    OPcp24_225 = OPcp24_224+ROcp24_224*qdd[25]-qd[25]*(OMcp24_124*ROcp24_324-OMcp24_324*ROcp24_124);
    OPcp24_325 = OPcp24_324+ROcp24_324*qdd[25]+qd[25]*(OMcp24_124*ROcp24_224-OMcp24_224*ROcp24_124);
    ACcp24_125 = qdd[1]+OMcp24_222*ORcp24_323+OMcp24_223*ORcp24_324+OMcp24_224*ORcp24_325+OMcp24_26*ORcp24_321-OMcp24_322*
 ORcp24_223-OMcp24_323*ORcp24_224-OMcp24_324*ORcp24_225-OMcp24_36*ORcp24_221+OPcp24_222*RLcp24_323+OPcp24_223*RLcp24_324+
 OPcp24_224*RLcp24_325+OPcp24_26*RLcp24_321-OPcp24_322*RLcp24_223-OPcp24_323*RLcp24_224-OPcp24_324*RLcp24_225-OPcp24_36*
 RLcp24_221;
    ACcp24_225 = qdd[2]-OMcp24_122*ORcp24_323-OMcp24_123*ORcp24_324-OMcp24_124*ORcp24_325-OMcp24_16*ORcp24_321+OMcp24_322*
 ORcp24_123+OMcp24_323*ORcp24_124+OMcp24_324*ORcp24_125+OMcp24_36*ORcp24_121-OPcp24_122*RLcp24_323-OPcp24_123*RLcp24_324-
 OPcp24_124*RLcp24_325-OPcp24_16*RLcp24_321+OPcp24_322*RLcp24_123+OPcp24_323*RLcp24_124+OPcp24_324*RLcp24_125+OPcp24_36*
 RLcp24_121;
    ACcp24_325 = qdd[3]+OMcp24_122*ORcp24_223+OMcp24_123*ORcp24_224+OMcp24_124*ORcp24_225+OMcp24_16*ORcp24_221-OMcp24_222*
 ORcp24_123-OMcp24_223*ORcp24_124-OMcp24_224*ORcp24_125-OMcp24_26*ORcp24_121+OPcp24_122*RLcp24_223+OPcp24_123*RLcp24_224+
 OPcp24_124*RLcp24_225+OPcp24_16*RLcp24_221-OPcp24_222*RLcp24_123-OPcp24_223*RLcp24_124-OPcp24_224*RLcp24_125-OPcp24_26*
 RLcp24_121;

// = = Block_1_0_0_25_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp24_125;
    sens->P[2] = POcp24_225;
    sens->P[3] = POcp24_325;
    sens->R[1][1] = ROcp24_124;
    sens->R[1][2] = ROcp24_224;
    sens->R[1][3] = ROcp24_324;
    sens->R[2][1] = ROcp24_425;
    sens->R[2][2] = ROcp24_525;
    sens->R[2][3] = ROcp24_625;
    sens->R[3][1] = ROcp24_725;
    sens->R[3][2] = ROcp24_825;
    sens->R[3][3] = ROcp24_925;
    sens->V[1] = VIcp24_125;
    sens->V[2] = VIcp24_225;
    sens->V[3] = VIcp24_325;
    sens->OM[1] = OMcp24_125;
    sens->OM[2] = OMcp24_225;
    sens->OM[3] = OMcp24_325;
    sens->A[1] = ACcp24_125;
    sens->A[2] = ACcp24_225;
    sens->A[3] = ACcp24_325;
    sens->OMP[1] = OPcp24_125;
    sens->OMP[2] = OPcp24_225;
    sens->OMP[3] = OPcp24_325;
 
// 
break;
case 26:
 


// = = Block_1_0_0_26_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp25_25 = qd[5]*C4;
    OMcp25_35 = qd[5]*S4;
    OMcp25_16 = qd[4]+qd[6]*S5;
    OMcp25_26 = OMcp25_25+ROcp25_85*qd[6];
    OMcp25_36 = OMcp25_35+ROcp25_95*qd[6];
    OPcp25_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp25_26 = ROcp25_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp25_35*S5-ROcp25_95*qd[4]);
    OPcp25_36 = ROcp25_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp25_25*S5-ROcp25_85*qd[4]);

// = = Block_1_0_0_26_0_4 = = 
 
// Sensor Kinematics 


    ROcp25_421 = ROcp25_46*C21+S21*S5;
    ROcp25_521 = ROcp25_56*C21+ROcp25_85*S21;
    ROcp25_621 = ROcp25_66*C21+ROcp25_95*S21;
    ROcp25_721 = -(ROcp25_46*S21-C21*S5);
    ROcp25_821 = -(ROcp25_56*S21-ROcp25_85*C21);
    ROcp25_921 = -(ROcp25_66*S21-ROcp25_95*C21);
    ROcp25_122 = ROcp25_16*C22-ROcp25_721*S22;
    ROcp25_222 = ROcp25_26*C22-ROcp25_821*S22;
    ROcp25_322 = ROcp25_36*C22-ROcp25_921*S22;
    ROcp25_722 = ROcp25_16*S22+ROcp25_721*C22;
    ROcp25_822 = ROcp25_26*S22+ROcp25_821*C22;
    ROcp25_922 = ROcp25_36*S22+ROcp25_921*C22;
    ROcp25_123 = ROcp25_122*C23+ROcp25_421*S23;
    ROcp25_223 = ROcp25_222*C23+ROcp25_521*S23;
    ROcp25_323 = ROcp25_322*C23+ROcp25_621*S23;
    ROcp25_423 = -(ROcp25_122*S23-ROcp25_421*C23);
    ROcp25_523 = -(ROcp25_222*S23-ROcp25_521*C23);
    ROcp25_623 = -(ROcp25_322*S23-ROcp25_621*C23);
    RLcp25_121 = ROcp25_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp25_221 = ROcp25_26*s->dpt[1][3]+ROcp25_85*s->dpt[3][3];
    RLcp25_321 = ROcp25_36*s->dpt[1][3]+ROcp25_95*s->dpt[3][3];
    OMcp25_121 = OMcp25_16+ROcp25_16*qd[21];
    OMcp25_221 = OMcp25_26+ROcp25_26*qd[21];
    OMcp25_321 = OMcp25_36+ROcp25_36*qd[21];
    ORcp25_121 = OMcp25_26*RLcp25_321-OMcp25_36*RLcp25_221;
    ORcp25_221 = -(OMcp25_16*RLcp25_321-OMcp25_36*RLcp25_121);
    ORcp25_321 = OMcp25_16*RLcp25_221-OMcp25_26*RLcp25_121;
    OMcp25_122 = OMcp25_121+ROcp25_421*qd[22];
    OMcp25_222 = OMcp25_221+ROcp25_521*qd[22];
    OMcp25_322 = OMcp25_321+ROcp25_621*qd[22];
    OPcp25_122 = OPcp25_16+ROcp25_16*qdd[21]+ROcp25_421*qdd[22]+qd[21]*(OMcp25_26*ROcp25_36-OMcp25_36*ROcp25_26)+qd[22]*(
 OMcp25_221*ROcp25_621-OMcp25_321*ROcp25_521);
    OPcp25_222 = OPcp25_26+ROcp25_26*qdd[21]+ROcp25_521*qdd[22]-qd[21]*(OMcp25_16*ROcp25_36-OMcp25_36*ROcp25_16)-qd[22]*(
 OMcp25_121*ROcp25_621-OMcp25_321*ROcp25_421);
    OPcp25_322 = OPcp25_36+ROcp25_36*qdd[21]+ROcp25_621*qdd[22]+qd[21]*(OMcp25_16*ROcp25_26-OMcp25_26*ROcp25_16)+qd[22]*(
 OMcp25_121*ROcp25_521-OMcp25_221*ROcp25_421);
    RLcp25_123 = ROcp25_722*s->dpt[3][37];
    RLcp25_223 = ROcp25_822*s->dpt[3][37];
    RLcp25_323 = ROcp25_922*s->dpt[3][37];
    OMcp25_123 = OMcp25_122+ROcp25_722*qd[23];
    OMcp25_223 = OMcp25_222+ROcp25_822*qd[23];
    OMcp25_323 = OMcp25_322+ROcp25_922*qd[23];
    ORcp25_123 = OMcp25_222*RLcp25_323-OMcp25_322*RLcp25_223;
    ORcp25_223 = -(OMcp25_122*RLcp25_323-OMcp25_322*RLcp25_123);
    ORcp25_323 = OMcp25_122*RLcp25_223-OMcp25_222*RLcp25_123;
    OPcp25_123 = OPcp25_122+ROcp25_722*qdd[23]+qd[23]*(OMcp25_222*ROcp25_922-OMcp25_322*ROcp25_822);
    OPcp25_223 = OPcp25_222+ROcp25_822*qdd[23]-qd[23]*(OMcp25_122*ROcp25_922-OMcp25_322*ROcp25_722);
    OPcp25_323 = OPcp25_322+ROcp25_922*qdd[23]+qd[23]*(OMcp25_122*ROcp25_822-OMcp25_222*ROcp25_722);

// = = Block_1_0_0_26_0_5 = = 
 
// Sensor Kinematics 


    ROcp25_124 = ROcp25_123*C24-ROcp25_722*S24;
    ROcp25_224 = ROcp25_223*C24-ROcp25_822*S24;
    ROcp25_324 = ROcp25_323*C24-ROcp25_922*S24;
    ROcp25_724 = ROcp25_123*S24+ROcp25_722*C24;
    ROcp25_824 = ROcp25_223*S24+ROcp25_822*C24;
    ROcp25_924 = ROcp25_323*S24+ROcp25_922*C24;
    ROcp25_425 = ROcp25_423*C25+ROcp25_724*S25;
    ROcp25_525 = ROcp25_523*C25+ROcp25_824*S25;
    ROcp25_625 = ROcp25_623*C25+ROcp25_924*S25;
    ROcp25_725 = -(ROcp25_423*S25-ROcp25_724*C25);
    ROcp25_825 = -(ROcp25_523*S25-ROcp25_824*C25);
    ROcp25_925 = -(ROcp25_623*S25-ROcp25_924*C25);
    ROcp25_126 = ROcp25_124*C26+ROcp25_425*S26;
    ROcp25_226 = ROcp25_224*C26+ROcp25_525*S26;
    ROcp25_326 = ROcp25_324*C26+ROcp25_625*S26;
    ROcp25_426 = -(ROcp25_124*S26-ROcp25_425*C26);
    ROcp25_526 = -(ROcp25_224*S26-ROcp25_525*C26);
    ROcp25_626 = -(ROcp25_324*S26-ROcp25_625*C26);
    RLcp25_124 = ROcp25_123*s->dpt[1][41]+ROcp25_423*s->dpt[2][41]+ROcp25_722*s->dpt[3][41];
    RLcp25_224 = ROcp25_223*s->dpt[1][41]+ROcp25_523*s->dpt[2][41]+ROcp25_822*s->dpt[3][41];
    RLcp25_324 = ROcp25_323*s->dpt[1][41]+ROcp25_623*s->dpt[2][41]+ROcp25_922*s->dpt[3][41];
    OMcp25_124 = OMcp25_123+ROcp25_423*qd[24];
    OMcp25_224 = OMcp25_223+ROcp25_523*qd[24];
    OMcp25_324 = OMcp25_323+ROcp25_623*qd[24];
    ORcp25_124 = OMcp25_223*RLcp25_324-OMcp25_323*RLcp25_224;
    ORcp25_224 = -(OMcp25_123*RLcp25_324-OMcp25_323*RLcp25_124);
    ORcp25_324 = OMcp25_123*RLcp25_224-OMcp25_223*RLcp25_124;
    OPcp25_124 = OPcp25_123+ROcp25_423*qdd[24]+qd[24]*(OMcp25_223*ROcp25_623-OMcp25_323*ROcp25_523);
    OPcp25_224 = OPcp25_223+ROcp25_523*qdd[24]-qd[24]*(OMcp25_123*ROcp25_623-OMcp25_323*ROcp25_423);
    OPcp25_324 = OPcp25_323+ROcp25_623*qdd[24]+qd[24]*(OMcp25_123*ROcp25_523-OMcp25_223*ROcp25_423);
    RLcp25_125 = ROcp25_423*s->dpt[2][44];
    RLcp25_225 = ROcp25_523*s->dpt[2][44];
    RLcp25_325 = ROcp25_623*s->dpt[2][44];
    OMcp25_125 = OMcp25_124+ROcp25_124*qd[25];
    OMcp25_225 = OMcp25_224+ROcp25_224*qd[25];
    OMcp25_325 = OMcp25_324+ROcp25_324*qd[25];
    ORcp25_125 = OMcp25_224*RLcp25_325-OMcp25_324*RLcp25_225;
    ORcp25_225 = -(OMcp25_124*RLcp25_325-OMcp25_324*RLcp25_125);
    ORcp25_325 = OMcp25_124*RLcp25_225-OMcp25_224*RLcp25_125;
    OPcp25_125 = OPcp25_124+ROcp25_124*qdd[25]+qd[25]*(OMcp25_224*ROcp25_324-OMcp25_324*ROcp25_224);
    OPcp25_225 = OPcp25_224+ROcp25_224*qdd[25]-qd[25]*(OMcp25_124*ROcp25_324-OMcp25_324*ROcp25_124);
    OPcp25_325 = OPcp25_324+ROcp25_324*qdd[25]+qd[25]*(OMcp25_124*ROcp25_224-OMcp25_224*ROcp25_124);
    RLcp25_126 = ROcp25_725*s->dpt[3][46];
    RLcp25_226 = ROcp25_825*s->dpt[3][46];
    RLcp25_326 = ROcp25_925*s->dpt[3][46];
    POcp25_126 = RLcp25_121+RLcp25_123+RLcp25_124+RLcp25_125+RLcp25_126+q[1];
    POcp25_226 = RLcp25_221+RLcp25_223+RLcp25_224+RLcp25_225+RLcp25_226+q[2];
    POcp25_326 = RLcp25_321+RLcp25_323+RLcp25_324+RLcp25_325+RLcp25_326+q[3];
    OMcp25_126 = OMcp25_125+ROcp25_725*qd[26];
    OMcp25_226 = OMcp25_225+ROcp25_825*qd[26];
    OMcp25_326 = OMcp25_325+ROcp25_925*qd[26];
    ORcp25_126 = OMcp25_225*RLcp25_326-OMcp25_325*RLcp25_226;
    ORcp25_226 = -(OMcp25_125*RLcp25_326-OMcp25_325*RLcp25_126);
    ORcp25_326 = OMcp25_125*RLcp25_226-OMcp25_225*RLcp25_126;
    VIcp25_126 = ORcp25_121+ORcp25_123+ORcp25_124+ORcp25_125+ORcp25_126+qd[1];
    VIcp25_226 = ORcp25_221+ORcp25_223+ORcp25_224+ORcp25_225+ORcp25_226+qd[2];
    VIcp25_326 = ORcp25_321+ORcp25_323+ORcp25_324+ORcp25_325+ORcp25_326+qd[3];
    OPcp25_126 = OPcp25_125+ROcp25_725*qdd[26]+qd[26]*(OMcp25_225*ROcp25_925-OMcp25_325*ROcp25_825);
    OPcp25_226 = OPcp25_225+ROcp25_825*qdd[26]-qd[26]*(OMcp25_125*ROcp25_925-OMcp25_325*ROcp25_725);
    OPcp25_326 = OPcp25_325+ROcp25_925*qdd[26]+qd[26]*(OMcp25_125*ROcp25_825-OMcp25_225*ROcp25_725);
    ACcp25_126 = qdd[1]+OMcp25_222*ORcp25_323+OMcp25_223*ORcp25_324+OMcp25_224*ORcp25_325+OMcp25_225*ORcp25_326+OMcp25_26*
 ORcp25_321-OMcp25_322*ORcp25_223-OMcp25_323*ORcp25_224-OMcp25_324*ORcp25_225-OMcp25_325*ORcp25_226-OMcp25_36*ORcp25_221+
 OPcp25_222*RLcp25_323+OPcp25_223*RLcp25_324+OPcp25_224*RLcp25_325+OPcp25_225*RLcp25_326+OPcp25_26*RLcp25_321-OPcp25_322*
 RLcp25_223-OPcp25_323*RLcp25_224-OPcp25_324*RLcp25_225-OPcp25_325*RLcp25_226-OPcp25_36*RLcp25_221;
    ACcp25_226 = qdd[2]-OMcp25_122*ORcp25_323-OMcp25_123*ORcp25_324-OMcp25_124*ORcp25_325-OMcp25_125*ORcp25_326-OMcp25_16*
 ORcp25_321+OMcp25_322*ORcp25_123+OMcp25_323*ORcp25_124+OMcp25_324*ORcp25_125+OMcp25_325*ORcp25_126+OMcp25_36*ORcp25_121-
 OPcp25_122*RLcp25_323-OPcp25_123*RLcp25_324-OPcp25_124*RLcp25_325-OPcp25_125*RLcp25_326-OPcp25_16*RLcp25_321+OPcp25_322*
 RLcp25_123+OPcp25_323*RLcp25_124+OPcp25_324*RLcp25_125+OPcp25_325*RLcp25_126+OPcp25_36*RLcp25_121;
    ACcp25_326 = qdd[3]+OMcp25_122*ORcp25_223+OMcp25_123*ORcp25_224+OMcp25_124*ORcp25_225+OMcp25_125*ORcp25_226+OMcp25_16*
 ORcp25_221-OMcp25_222*ORcp25_123-OMcp25_223*ORcp25_124-OMcp25_224*ORcp25_125-OMcp25_225*ORcp25_126-OMcp25_26*ORcp25_121+
 OPcp25_122*RLcp25_223+OPcp25_123*RLcp25_224+OPcp25_124*RLcp25_225+OPcp25_125*RLcp25_226+OPcp25_16*RLcp25_221-OPcp25_222*
 RLcp25_123-OPcp25_223*RLcp25_124-OPcp25_224*RLcp25_125-OPcp25_225*RLcp25_126-OPcp25_26*RLcp25_121;

// = = Block_1_0_0_26_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp25_126;
    sens->P[2] = POcp25_226;
    sens->P[3] = POcp25_326;
    sens->R[1][1] = ROcp25_126;
    sens->R[1][2] = ROcp25_226;
    sens->R[1][3] = ROcp25_326;
    sens->R[2][1] = ROcp25_426;
    sens->R[2][2] = ROcp25_526;
    sens->R[2][3] = ROcp25_626;
    sens->R[3][1] = ROcp25_725;
    sens->R[3][2] = ROcp25_825;
    sens->R[3][3] = ROcp25_925;
    sens->V[1] = VIcp25_126;
    sens->V[2] = VIcp25_226;
    sens->V[3] = VIcp25_326;
    sens->OM[1] = OMcp25_126;
    sens->OM[2] = OMcp25_226;
    sens->OM[3] = OMcp25_326;
    sens->A[1] = ACcp25_126;
    sens->A[2] = ACcp25_226;
    sens->A[3] = ACcp25_326;
    sens->OMP[1] = OPcp25_126;
    sens->OMP[2] = OPcp25_226;
    sens->OMP[3] = OPcp25_326;
 
// 
break;
case 27:
 


// = = Block_1_0_0_27_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp26_25 = qd[5]*C4;
    OMcp26_35 = qd[5]*S4;
    OMcp26_16 = qd[4]+qd[6]*S5;
    OMcp26_26 = OMcp26_25+ROcp26_85*qd[6];
    OMcp26_36 = OMcp26_35+ROcp26_95*qd[6];
    OPcp26_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp26_26 = ROcp26_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp26_35*S5-ROcp26_95*qd[4]);
    OPcp26_36 = ROcp26_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp26_25*S5-ROcp26_85*qd[4]);

// = = Block_1_0_0_27_0_4 = = 
 
// Sensor Kinematics 


    ROcp26_421 = ROcp26_46*C21+S21*S5;
    ROcp26_521 = ROcp26_56*C21+ROcp26_85*S21;
    ROcp26_621 = ROcp26_66*C21+ROcp26_95*S21;
    ROcp26_721 = -(ROcp26_46*S21-C21*S5);
    ROcp26_821 = -(ROcp26_56*S21-ROcp26_85*C21);
    ROcp26_921 = -(ROcp26_66*S21-ROcp26_95*C21);
    ROcp26_122 = ROcp26_16*C22-ROcp26_721*S22;
    ROcp26_222 = ROcp26_26*C22-ROcp26_821*S22;
    ROcp26_322 = ROcp26_36*C22-ROcp26_921*S22;
    ROcp26_722 = ROcp26_16*S22+ROcp26_721*C22;
    ROcp26_822 = ROcp26_26*S22+ROcp26_821*C22;
    ROcp26_922 = ROcp26_36*S22+ROcp26_921*C22;
    ROcp26_123 = ROcp26_122*C23+ROcp26_421*S23;
    ROcp26_223 = ROcp26_222*C23+ROcp26_521*S23;
    ROcp26_323 = ROcp26_322*C23+ROcp26_621*S23;
    ROcp26_423 = -(ROcp26_122*S23-ROcp26_421*C23);
    ROcp26_523 = -(ROcp26_222*S23-ROcp26_521*C23);
    ROcp26_623 = -(ROcp26_322*S23-ROcp26_621*C23);
    RLcp26_121 = ROcp26_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp26_221 = ROcp26_26*s->dpt[1][3]+ROcp26_85*s->dpt[3][3];
    RLcp26_321 = ROcp26_36*s->dpt[1][3]+ROcp26_95*s->dpt[3][3];
    OMcp26_121 = OMcp26_16+ROcp26_16*qd[21];
    OMcp26_221 = OMcp26_26+ROcp26_26*qd[21];
    OMcp26_321 = OMcp26_36+ROcp26_36*qd[21];
    ORcp26_121 = OMcp26_26*RLcp26_321-OMcp26_36*RLcp26_221;
    ORcp26_221 = -(OMcp26_16*RLcp26_321-OMcp26_36*RLcp26_121);
    ORcp26_321 = OMcp26_16*RLcp26_221-OMcp26_26*RLcp26_121;
    OMcp26_122 = OMcp26_121+ROcp26_421*qd[22];
    OMcp26_222 = OMcp26_221+ROcp26_521*qd[22];
    OMcp26_322 = OMcp26_321+ROcp26_621*qd[22];
    OPcp26_122 = OPcp26_16+ROcp26_16*qdd[21]+ROcp26_421*qdd[22]+qd[21]*(OMcp26_26*ROcp26_36-OMcp26_36*ROcp26_26)+qd[22]*(
 OMcp26_221*ROcp26_621-OMcp26_321*ROcp26_521);
    OPcp26_222 = OPcp26_26+ROcp26_26*qdd[21]+ROcp26_521*qdd[22]-qd[21]*(OMcp26_16*ROcp26_36-OMcp26_36*ROcp26_16)-qd[22]*(
 OMcp26_121*ROcp26_621-OMcp26_321*ROcp26_421);
    OPcp26_322 = OPcp26_36+ROcp26_36*qdd[21]+ROcp26_621*qdd[22]+qd[21]*(OMcp26_16*ROcp26_26-OMcp26_26*ROcp26_16)+qd[22]*(
 OMcp26_121*ROcp26_521-OMcp26_221*ROcp26_421);
    RLcp26_123 = ROcp26_722*s->dpt[3][37];
    RLcp26_223 = ROcp26_822*s->dpt[3][37];
    RLcp26_323 = ROcp26_922*s->dpt[3][37];
    OMcp26_123 = OMcp26_122+ROcp26_722*qd[23];
    OMcp26_223 = OMcp26_222+ROcp26_822*qd[23];
    OMcp26_323 = OMcp26_322+ROcp26_922*qd[23];
    ORcp26_123 = OMcp26_222*RLcp26_323-OMcp26_322*RLcp26_223;
    ORcp26_223 = -(OMcp26_122*RLcp26_323-OMcp26_322*RLcp26_123);
    ORcp26_323 = OMcp26_122*RLcp26_223-OMcp26_222*RLcp26_123;
    OPcp26_123 = OPcp26_122+ROcp26_722*qdd[23]+qd[23]*(OMcp26_222*ROcp26_922-OMcp26_322*ROcp26_822);
    OPcp26_223 = OPcp26_222+ROcp26_822*qdd[23]-qd[23]*(OMcp26_122*ROcp26_922-OMcp26_322*ROcp26_722);
    OPcp26_323 = OPcp26_322+ROcp26_922*qdd[23]+qd[23]*(OMcp26_122*ROcp26_822-OMcp26_222*ROcp26_722);

// = = Block_1_0_0_27_0_5 = = 
 
// Sensor Kinematics 


    ROcp26_124 = ROcp26_123*C24-ROcp26_722*S24;
    ROcp26_224 = ROcp26_223*C24-ROcp26_822*S24;
    ROcp26_324 = ROcp26_323*C24-ROcp26_922*S24;
    ROcp26_724 = ROcp26_123*S24+ROcp26_722*C24;
    ROcp26_824 = ROcp26_223*S24+ROcp26_822*C24;
    ROcp26_924 = ROcp26_323*S24+ROcp26_922*C24;
    ROcp26_425 = ROcp26_423*C25+ROcp26_724*S25;
    ROcp26_525 = ROcp26_523*C25+ROcp26_824*S25;
    ROcp26_625 = ROcp26_623*C25+ROcp26_924*S25;
    ROcp26_725 = -(ROcp26_423*S25-ROcp26_724*C25);
    ROcp26_825 = -(ROcp26_523*S25-ROcp26_824*C25);
    ROcp26_925 = -(ROcp26_623*S25-ROcp26_924*C25);
    ROcp26_126 = ROcp26_124*C26+ROcp26_425*S26;
    ROcp26_226 = ROcp26_224*C26+ROcp26_525*S26;
    ROcp26_326 = ROcp26_324*C26+ROcp26_625*S26;
    ROcp26_426 = -(ROcp26_124*S26-ROcp26_425*C26);
    ROcp26_526 = -(ROcp26_224*S26-ROcp26_525*C26);
    ROcp26_626 = -(ROcp26_324*S26-ROcp26_625*C26);
    ROcp26_127 = ROcp26_126*C27-ROcp26_725*S27;
    ROcp26_227 = ROcp26_226*C27-ROcp26_825*S27;
    ROcp26_327 = ROcp26_326*C27-ROcp26_925*S27;
    ROcp26_727 = ROcp26_126*S27+ROcp26_725*C27;
    ROcp26_827 = ROcp26_226*S27+ROcp26_825*C27;
    ROcp26_927 = ROcp26_326*S27+ROcp26_925*C27;
    RLcp26_124 = ROcp26_123*s->dpt[1][41]+ROcp26_423*s->dpt[2][41]+ROcp26_722*s->dpt[3][41];
    RLcp26_224 = ROcp26_223*s->dpt[1][41]+ROcp26_523*s->dpt[2][41]+ROcp26_822*s->dpt[3][41];
    RLcp26_324 = ROcp26_323*s->dpt[1][41]+ROcp26_623*s->dpt[2][41]+ROcp26_922*s->dpt[3][41];
    OMcp26_124 = OMcp26_123+ROcp26_423*qd[24];
    OMcp26_224 = OMcp26_223+ROcp26_523*qd[24];
    OMcp26_324 = OMcp26_323+ROcp26_623*qd[24];
    ORcp26_124 = OMcp26_223*RLcp26_324-OMcp26_323*RLcp26_224;
    ORcp26_224 = -(OMcp26_123*RLcp26_324-OMcp26_323*RLcp26_124);
    ORcp26_324 = OMcp26_123*RLcp26_224-OMcp26_223*RLcp26_124;
    OPcp26_124 = OPcp26_123+ROcp26_423*qdd[24]+qd[24]*(OMcp26_223*ROcp26_623-OMcp26_323*ROcp26_523);
    OPcp26_224 = OPcp26_223+ROcp26_523*qdd[24]-qd[24]*(OMcp26_123*ROcp26_623-OMcp26_323*ROcp26_423);
    OPcp26_324 = OPcp26_323+ROcp26_623*qdd[24]+qd[24]*(OMcp26_123*ROcp26_523-OMcp26_223*ROcp26_423);
    RLcp26_125 = ROcp26_423*s->dpt[2][44];
    RLcp26_225 = ROcp26_523*s->dpt[2][44];
    RLcp26_325 = ROcp26_623*s->dpt[2][44];
    OMcp26_125 = OMcp26_124+ROcp26_124*qd[25];
    OMcp26_225 = OMcp26_224+ROcp26_224*qd[25];
    OMcp26_325 = OMcp26_324+ROcp26_324*qd[25];
    ORcp26_125 = OMcp26_224*RLcp26_325-OMcp26_324*RLcp26_225;
    ORcp26_225 = -(OMcp26_124*RLcp26_325-OMcp26_324*RLcp26_125);
    ORcp26_325 = OMcp26_124*RLcp26_225-OMcp26_224*RLcp26_125;
    OPcp26_125 = OPcp26_124+ROcp26_124*qdd[25]+qd[25]*(OMcp26_224*ROcp26_324-OMcp26_324*ROcp26_224);
    OPcp26_225 = OPcp26_224+ROcp26_224*qdd[25]-qd[25]*(OMcp26_124*ROcp26_324-OMcp26_324*ROcp26_124);
    OPcp26_325 = OPcp26_324+ROcp26_324*qdd[25]+qd[25]*(OMcp26_124*ROcp26_224-OMcp26_224*ROcp26_124);
    RLcp26_126 = ROcp26_725*s->dpt[3][46];
    RLcp26_226 = ROcp26_825*s->dpt[3][46];
    RLcp26_326 = ROcp26_925*s->dpt[3][46];
    OMcp26_126 = OMcp26_125+ROcp26_725*qd[26];
    OMcp26_226 = OMcp26_225+ROcp26_825*qd[26];
    OMcp26_326 = OMcp26_325+ROcp26_925*qd[26];
    ORcp26_126 = OMcp26_225*RLcp26_326-OMcp26_325*RLcp26_226;
    ORcp26_226 = -(OMcp26_125*RLcp26_326-OMcp26_325*RLcp26_126);
    ORcp26_326 = OMcp26_125*RLcp26_226-OMcp26_225*RLcp26_126;
    OPcp26_126 = OPcp26_125+ROcp26_725*qdd[26]+qd[26]*(OMcp26_225*ROcp26_925-OMcp26_325*ROcp26_825);
    OPcp26_226 = OPcp26_225+ROcp26_825*qdd[26]-qd[26]*(OMcp26_125*ROcp26_925-OMcp26_325*ROcp26_725);
    OPcp26_326 = OPcp26_325+ROcp26_925*qdd[26]+qd[26]*(OMcp26_125*ROcp26_825-OMcp26_225*ROcp26_725);
    RLcp26_127 = ROcp26_725*s->dpt[3][48];
    RLcp26_227 = ROcp26_825*s->dpt[3][48];
    RLcp26_327 = ROcp26_925*s->dpt[3][48];
    POcp26_127 = RLcp26_121+RLcp26_123+RLcp26_124+RLcp26_125+RLcp26_126+RLcp26_127+q[1];
    POcp26_227 = RLcp26_221+RLcp26_223+RLcp26_224+RLcp26_225+RLcp26_226+RLcp26_227+q[2];
    POcp26_327 = RLcp26_321+RLcp26_323+RLcp26_324+RLcp26_325+RLcp26_326+RLcp26_327+q[3];
    OMcp26_127 = OMcp26_126+ROcp26_426*qd[27];
    OMcp26_227 = OMcp26_226+ROcp26_526*qd[27];
    OMcp26_327 = OMcp26_326+ROcp26_626*qd[27];
    ORcp26_127 = OMcp26_226*RLcp26_327-OMcp26_326*RLcp26_227;
    ORcp26_227 = -(OMcp26_126*RLcp26_327-OMcp26_326*RLcp26_127);
    ORcp26_327 = OMcp26_126*RLcp26_227-OMcp26_226*RLcp26_127;
    VIcp26_127 = ORcp26_121+ORcp26_123+ORcp26_124+ORcp26_125+ORcp26_126+ORcp26_127+qd[1];
    VIcp26_227 = ORcp26_221+ORcp26_223+ORcp26_224+ORcp26_225+ORcp26_226+ORcp26_227+qd[2];
    VIcp26_327 = ORcp26_321+ORcp26_323+ORcp26_324+ORcp26_325+ORcp26_326+ORcp26_327+qd[3];
    OPcp26_127 = OPcp26_126+ROcp26_426*qdd[27]+qd[27]*(OMcp26_226*ROcp26_626-OMcp26_326*ROcp26_526);
    OPcp26_227 = OPcp26_226+ROcp26_526*qdd[27]-qd[27]*(OMcp26_126*ROcp26_626-OMcp26_326*ROcp26_426);
    OPcp26_327 = OPcp26_326+ROcp26_626*qdd[27]+qd[27]*(OMcp26_126*ROcp26_526-OMcp26_226*ROcp26_426);
    ACcp26_127 = qdd[1]+OMcp26_222*ORcp26_323+OMcp26_223*ORcp26_324+OMcp26_224*ORcp26_325+OMcp26_225*ORcp26_326+OMcp26_226
 *ORcp26_327+OMcp26_26*ORcp26_321-OMcp26_322*ORcp26_223-OMcp26_323*ORcp26_224-OMcp26_324*ORcp26_225-OMcp26_325*ORcp26_226-
 OMcp26_326*ORcp26_227-OMcp26_36*ORcp26_221+OPcp26_222*RLcp26_323+OPcp26_223*RLcp26_324+OPcp26_224*RLcp26_325+OPcp26_225*
 RLcp26_326+OPcp26_226*RLcp26_327+OPcp26_26*RLcp26_321-OPcp26_322*RLcp26_223-OPcp26_323*RLcp26_224-OPcp26_324*RLcp26_225-
 OPcp26_325*RLcp26_226-OPcp26_326*RLcp26_227-OPcp26_36*RLcp26_221;
    ACcp26_227 = qdd[2]-OMcp26_122*ORcp26_323-OMcp26_123*ORcp26_324-OMcp26_124*ORcp26_325-OMcp26_125*ORcp26_326-OMcp26_126
 *ORcp26_327-OMcp26_16*ORcp26_321+OMcp26_322*ORcp26_123+OMcp26_323*ORcp26_124+OMcp26_324*ORcp26_125+OMcp26_325*ORcp26_126+
 OMcp26_326*ORcp26_127+OMcp26_36*ORcp26_121-OPcp26_122*RLcp26_323-OPcp26_123*RLcp26_324-OPcp26_124*RLcp26_325-OPcp26_125*
 RLcp26_326-OPcp26_126*RLcp26_327-OPcp26_16*RLcp26_321+OPcp26_322*RLcp26_123+OPcp26_323*RLcp26_124+OPcp26_324*RLcp26_125+
 OPcp26_325*RLcp26_126+OPcp26_326*RLcp26_127+OPcp26_36*RLcp26_121;
    ACcp26_327 = qdd[3]+OMcp26_122*ORcp26_223+OMcp26_123*ORcp26_224+OMcp26_124*ORcp26_225+OMcp26_125*ORcp26_226+OMcp26_126
 *ORcp26_227+OMcp26_16*ORcp26_221-OMcp26_222*ORcp26_123-OMcp26_223*ORcp26_124-OMcp26_224*ORcp26_125-OMcp26_225*ORcp26_126-
 OMcp26_226*ORcp26_127-OMcp26_26*ORcp26_121+OPcp26_122*RLcp26_223+OPcp26_123*RLcp26_224+OPcp26_124*RLcp26_225+OPcp26_125*
 RLcp26_226+OPcp26_126*RLcp26_227+OPcp26_16*RLcp26_221-OPcp26_222*RLcp26_123-OPcp26_223*RLcp26_124-OPcp26_224*RLcp26_125-
 OPcp26_225*RLcp26_126-OPcp26_226*RLcp26_127-OPcp26_26*RLcp26_121;

// = = Block_1_0_0_27_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp26_127;
    sens->P[2] = POcp26_227;
    sens->P[3] = POcp26_327;
    sens->R[1][1] = ROcp26_127;
    sens->R[1][2] = ROcp26_227;
    sens->R[1][3] = ROcp26_327;
    sens->R[2][1] = ROcp26_426;
    sens->R[2][2] = ROcp26_526;
    sens->R[2][3] = ROcp26_626;
    sens->R[3][1] = ROcp26_727;
    sens->R[3][2] = ROcp26_827;
    sens->R[3][3] = ROcp26_927;
    sens->V[1] = VIcp26_127;
    sens->V[2] = VIcp26_227;
    sens->V[3] = VIcp26_327;
    sens->OM[1] = OMcp26_127;
    sens->OM[2] = OMcp26_227;
    sens->OM[3] = OMcp26_327;
    sens->A[1] = ACcp26_127;
    sens->A[2] = ACcp26_227;
    sens->A[3] = ACcp26_327;
    sens->OMP[1] = OPcp26_127;
    sens->OMP[2] = OPcp26_227;
    sens->OMP[3] = OPcp26_327;
 
// 
break;
case 28:
 


// = = Block_1_0_0_28_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp27_25 = qd[5]*C4;
    OMcp27_35 = qd[5]*S4;
    OMcp27_16 = qd[4]+qd[6]*S5;
    OMcp27_26 = OMcp27_25+ROcp27_85*qd[6];
    OMcp27_36 = OMcp27_35+ROcp27_95*qd[6];
    OPcp27_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp27_26 = ROcp27_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp27_35*S5-ROcp27_95*qd[4]);
    OPcp27_36 = ROcp27_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp27_25*S5-ROcp27_85*qd[4]);

// = = Block_1_0_0_28_0_4 = = 
 
// Sensor Kinematics 


    ROcp27_421 = ROcp27_46*C21+S21*S5;
    ROcp27_521 = ROcp27_56*C21+ROcp27_85*S21;
    ROcp27_621 = ROcp27_66*C21+ROcp27_95*S21;
    ROcp27_721 = -(ROcp27_46*S21-C21*S5);
    ROcp27_821 = -(ROcp27_56*S21-ROcp27_85*C21);
    ROcp27_921 = -(ROcp27_66*S21-ROcp27_95*C21);
    ROcp27_122 = ROcp27_16*C22-ROcp27_721*S22;
    ROcp27_222 = ROcp27_26*C22-ROcp27_821*S22;
    ROcp27_322 = ROcp27_36*C22-ROcp27_921*S22;
    ROcp27_722 = ROcp27_16*S22+ROcp27_721*C22;
    ROcp27_822 = ROcp27_26*S22+ROcp27_821*C22;
    ROcp27_922 = ROcp27_36*S22+ROcp27_921*C22;
    ROcp27_123 = ROcp27_122*C23+ROcp27_421*S23;
    ROcp27_223 = ROcp27_222*C23+ROcp27_521*S23;
    ROcp27_323 = ROcp27_322*C23+ROcp27_621*S23;
    ROcp27_423 = -(ROcp27_122*S23-ROcp27_421*C23);
    ROcp27_523 = -(ROcp27_222*S23-ROcp27_521*C23);
    ROcp27_623 = -(ROcp27_322*S23-ROcp27_621*C23);
    RLcp27_121 = ROcp27_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp27_221 = ROcp27_26*s->dpt[1][3]+ROcp27_85*s->dpt[3][3];
    RLcp27_321 = ROcp27_36*s->dpt[1][3]+ROcp27_95*s->dpt[3][3];
    OMcp27_121 = OMcp27_16+ROcp27_16*qd[21];
    OMcp27_221 = OMcp27_26+ROcp27_26*qd[21];
    OMcp27_321 = OMcp27_36+ROcp27_36*qd[21];
    ORcp27_121 = OMcp27_26*RLcp27_321-OMcp27_36*RLcp27_221;
    ORcp27_221 = -(OMcp27_16*RLcp27_321-OMcp27_36*RLcp27_121);
    ORcp27_321 = OMcp27_16*RLcp27_221-OMcp27_26*RLcp27_121;
    OMcp27_122 = OMcp27_121+ROcp27_421*qd[22];
    OMcp27_222 = OMcp27_221+ROcp27_521*qd[22];
    OMcp27_322 = OMcp27_321+ROcp27_621*qd[22];
    OPcp27_122 = OPcp27_16+ROcp27_16*qdd[21]+ROcp27_421*qdd[22]+qd[21]*(OMcp27_26*ROcp27_36-OMcp27_36*ROcp27_26)+qd[22]*(
 OMcp27_221*ROcp27_621-OMcp27_321*ROcp27_521);
    OPcp27_222 = OPcp27_26+ROcp27_26*qdd[21]+ROcp27_521*qdd[22]-qd[21]*(OMcp27_16*ROcp27_36-OMcp27_36*ROcp27_16)-qd[22]*(
 OMcp27_121*ROcp27_621-OMcp27_321*ROcp27_421);
    OPcp27_322 = OPcp27_36+ROcp27_36*qdd[21]+ROcp27_621*qdd[22]+qd[21]*(OMcp27_16*ROcp27_26-OMcp27_26*ROcp27_16)+qd[22]*(
 OMcp27_121*ROcp27_521-OMcp27_221*ROcp27_421);
    RLcp27_123 = ROcp27_722*s->dpt[3][37];
    RLcp27_223 = ROcp27_822*s->dpt[3][37];
    RLcp27_323 = ROcp27_922*s->dpt[3][37];
    OMcp27_123 = OMcp27_122+ROcp27_722*qd[23];
    OMcp27_223 = OMcp27_222+ROcp27_822*qd[23];
    OMcp27_323 = OMcp27_322+ROcp27_922*qd[23];
    ORcp27_123 = OMcp27_222*RLcp27_323-OMcp27_322*RLcp27_223;
    ORcp27_223 = -(OMcp27_122*RLcp27_323-OMcp27_322*RLcp27_123);
    ORcp27_323 = OMcp27_122*RLcp27_223-OMcp27_222*RLcp27_123;
    OPcp27_123 = OPcp27_122+ROcp27_722*qdd[23]+qd[23]*(OMcp27_222*ROcp27_922-OMcp27_322*ROcp27_822);
    OPcp27_223 = OPcp27_222+ROcp27_822*qdd[23]-qd[23]*(OMcp27_122*ROcp27_922-OMcp27_322*ROcp27_722);
    OPcp27_323 = OPcp27_322+ROcp27_922*qdd[23]+qd[23]*(OMcp27_122*ROcp27_822-OMcp27_222*ROcp27_722);

// = = Block_1_0_0_28_0_6 = = 
 
// Sensor Kinematics 


    ROcp27_128 = ROcp27_123*C28-ROcp27_722*S28;
    ROcp27_228 = ROcp27_223*C28-ROcp27_822*S28;
    ROcp27_328 = ROcp27_323*C28-ROcp27_922*S28;
    ROcp27_728 = ROcp27_123*S28+ROcp27_722*C28;
    ROcp27_828 = ROcp27_223*S28+ROcp27_822*C28;
    ROcp27_928 = ROcp27_323*S28+ROcp27_922*C28;
    RLcp27_128 = ROcp27_123*s->dpt[1][42]+ROcp27_423*s->dpt[2][42]+ROcp27_722*s->dpt[3][42];
    RLcp27_228 = ROcp27_223*s->dpt[1][42]+ROcp27_523*s->dpt[2][42]+ROcp27_822*s->dpt[3][42];
    RLcp27_328 = ROcp27_323*s->dpt[1][42]+ROcp27_623*s->dpt[2][42]+ROcp27_922*s->dpt[3][42];
    POcp27_128 = RLcp27_121+RLcp27_123+RLcp27_128+q[1];
    POcp27_228 = RLcp27_221+RLcp27_223+RLcp27_228+q[2];
    POcp27_328 = RLcp27_321+RLcp27_323+RLcp27_328+q[3];
    OMcp27_128 = OMcp27_123+ROcp27_423*qd[28];
    OMcp27_228 = OMcp27_223+ROcp27_523*qd[28];
    OMcp27_328 = OMcp27_323+ROcp27_623*qd[28];
    ORcp27_128 = OMcp27_223*RLcp27_328-OMcp27_323*RLcp27_228;
    ORcp27_228 = -(OMcp27_123*RLcp27_328-OMcp27_323*RLcp27_128);
    ORcp27_328 = OMcp27_123*RLcp27_228-OMcp27_223*RLcp27_128;
    VIcp27_128 = ORcp27_121+ORcp27_123+ORcp27_128+qd[1];
    VIcp27_228 = ORcp27_221+ORcp27_223+ORcp27_228+qd[2];
    VIcp27_328 = ORcp27_321+ORcp27_323+ORcp27_328+qd[3];
    OPcp27_128 = OPcp27_123+ROcp27_423*qdd[28]+qd[28]*(OMcp27_223*ROcp27_623-OMcp27_323*ROcp27_523);
    OPcp27_228 = OPcp27_223+ROcp27_523*qdd[28]-qd[28]*(OMcp27_123*ROcp27_623-OMcp27_323*ROcp27_423);
    OPcp27_328 = OPcp27_323+ROcp27_623*qdd[28]+qd[28]*(OMcp27_123*ROcp27_523-OMcp27_223*ROcp27_423);
    ACcp27_128 = qdd[1]+OMcp27_222*ORcp27_323+OMcp27_223*ORcp27_328+OMcp27_26*ORcp27_321-OMcp27_322*ORcp27_223-OMcp27_323*
 ORcp27_228-OMcp27_36*ORcp27_221+OPcp27_222*RLcp27_323+OPcp27_223*RLcp27_328+OPcp27_26*RLcp27_321-OPcp27_322*RLcp27_223-
 OPcp27_323*RLcp27_228-OPcp27_36*RLcp27_221;
    ACcp27_228 = qdd[2]-OMcp27_122*ORcp27_323-OMcp27_123*ORcp27_328-OMcp27_16*ORcp27_321+OMcp27_322*ORcp27_123+OMcp27_323*
 ORcp27_128+OMcp27_36*ORcp27_121-OPcp27_122*RLcp27_323-OPcp27_123*RLcp27_328-OPcp27_16*RLcp27_321+OPcp27_322*RLcp27_123+
 OPcp27_323*RLcp27_128+OPcp27_36*RLcp27_121;
    ACcp27_328 = qdd[3]+OMcp27_122*ORcp27_223+OMcp27_123*ORcp27_228+OMcp27_16*ORcp27_221-OMcp27_222*ORcp27_123-OMcp27_223*
 ORcp27_128-OMcp27_26*ORcp27_121+OPcp27_122*RLcp27_223+OPcp27_123*RLcp27_228+OPcp27_16*RLcp27_221-OPcp27_222*RLcp27_123-
 OPcp27_223*RLcp27_128-OPcp27_26*RLcp27_121;

// = = Block_1_0_0_28_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp27_128;
    sens->P[2] = POcp27_228;
    sens->P[3] = POcp27_328;
    sens->R[1][1] = ROcp27_128;
    sens->R[1][2] = ROcp27_228;
    sens->R[1][3] = ROcp27_328;
    sens->R[2][1] = ROcp27_423;
    sens->R[2][2] = ROcp27_523;
    sens->R[2][3] = ROcp27_623;
    sens->R[3][1] = ROcp27_728;
    sens->R[3][2] = ROcp27_828;
    sens->R[3][3] = ROcp27_928;
    sens->V[1] = VIcp27_128;
    sens->V[2] = VIcp27_228;
    sens->V[3] = VIcp27_328;
    sens->OM[1] = OMcp27_128;
    sens->OM[2] = OMcp27_228;
    sens->OM[3] = OMcp27_328;
    sens->A[1] = ACcp27_128;
    sens->A[2] = ACcp27_228;
    sens->A[3] = ACcp27_328;
    sens->OMP[1] = OPcp27_128;
    sens->OMP[2] = OPcp27_228;
    sens->OMP[3] = OPcp27_328;
 
// 
break;
case 29:
 


// = = Block_1_0_0_29_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp28_25 = qd[5]*C4;
    OMcp28_35 = qd[5]*S4;
    OMcp28_16 = qd[4]+qd[6]*S5;
    OMcp28_26 = OMcp28_25+ROcp28_85*qd[6];
    OMcp28_36 = OMcp28_35+ROcp28_95*qd[6];
    OPcp28_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp28_26 = ROcp28_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp28_35*S5-ROcp28_95*qd[4]);
    OPcp28_36 = ROcp28_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp28_25*S5-ROcp28_85*qd[4]);

// = = Block_1_0_0_29_0_4 = = 
 
// Sensor Kinematics 


    ROcp28_421 = ROcp28_46*C21+S21*S5;
    ROcp28_521 = ROcp28_56*C21+ROcp28_85*S21;
    ROcp28_621 = ROcp28_66*C21+ROcp28_95*S21;
    ROcp28_721 = -(ROcp28_46*S21-C21*S5);
    ROcp28_821 = -(ROcp28_56*S21-ROcp28_85*C21);
    ROcp28_921 = -(ROcp28_66*S21-ROcp28_95*C21);
    ROcp28_122 = ROcp28_16*C22-ROcp28_721*S22;
    ROcp28_222 = ROcp28_26*C22-ROcp28_821*S22;
    ROcp28_322 = ROcp28_36*C22-ROcp28_921*S22;
    ROcp28_722 = ROcp28_16*S22+ROcp28_721*C22;
    ROcp28_822 = ROcp28_26*S22+ROcp28_821*C22;
    ROcp28_922 = ROcp28_36*S22+ROcp28_921*C22;
    ROcp28_123 = ROcp28_122*C23+ROcp28_421*S23;
    ROcp28_223 = ROcp28_222*C23+ROcp28_521*S23;
    ROcp28_323 = ROcp28_322*C23+ROcp28_621*S23;
    ROcp28_423 = -(ROcp28_122*S23-ROcp28_421*C23);
    ROcp28_523 = -(ROcp28_222*S23-ROcp28_521*C23);
    ROcp28_623 = -(ROcp28_322*S23-ROcp28_621*C23);
    RLcp28_121 = ROcp28_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp28_221 = ROcp28_26*s->dpt[1][3]+ROcp28_85*s->dpt[3][3];
    RLcp28_321 = ROcp28_36*s->dpt[1][3]+ROcp28_95*s->dpt[3][3];
    OMcp28_121 = OMcp28_16+ROcp28_16*qd[21];
    OMcp28_221 = OMcp28_26+ROcp28_26*qd[21];
    OMcp28_321 = OMcp28_36+ROcp28_36*qd[21];
    ORcp28_121 = OMcp28_26*RLcp28_321-OMcp28_36*RLcp28_221;
    ORcp28_221 = -(OMcp28_16*RLcp28_321-OMcp28_36*RLcp28_121);
    ORcp28_321 = OMcp28_16*RLcp28_221-OMcp28_26*RLcp28_121;
    OMcp28_122 = OMcp28_121+ROcp28_421*qd[22];
    OMcp28_222 = OMcp28_221+ROcp28_521*qd[22];
    OMcp28_322 = OMcp28_321+ROcp28_621*qd[22];
    OPcp28_122 = OPcp28_16+ROcp28_16*qdd[21]+ROcp28_421*qdd[22]+qd[21]*(OMcp28_26*ROcp28_36-OMcp28_36*ROcp28_26)+qd[22]*(
 OMcp28_221*ROcp28_621-OMcp28_321*ROcp28_521);
    OPcp28_222 = OPcp28_26+ROcp28_26*qdd[21]+ROcp28_521*qdd[22]-qd[21]*(OMcp28_16*ROcp28_36-OMcp28_36*ROcp28_16)-qd[22]*(
 OMcp28_121*ROcp28_621-OMcp28_321*ROcp28_421);
    OPcp28_322 = OPcp28_36+ROcp28_36*qdd[21]+ROcp28_621*qdd[22]+qd[21]*(OMcp28_16*ROcp28_26-OMcp28_26*ROcp28_16)+qd[22]*(
 OMcp28_121*ROcp28_521-OMcp28_221*ROcp28_421);
    RLcp28_123 = ROcp28_722*s->dpt[3][37];
    RLcp28_223 = ROcp28_822*s->dpt[3][37];
    RLcp28_323 = ROcp28_922*s->dpt[3][37];
    OMcp28_123 = OMcp28_122+ROcp28_722*qd[23];
    OMcp28_223 = OMcp28_222+ROcp28_822*qd[23];
    OMcp28_323 = OMcp28_322+ROcp28_922*qd[23];
    ORcp28_123 = OMcp28_222*RLcp28_323-OMcp28_322*RLcp28_223;
    ORcp28_223 = -(OMcp28_122*RLcp28_323-OMcp28_322*RLcp28_123);
    ORcp28_323 = OMcp28_122*RLcp28_223-OMcp28_222*RLcp28_123;
    OPcp28_123 = OPcp28_122+ROcp28_722*qdd[23]+qd[23]*(OMcp28_222*ROcp28_922-OMcp28_322*ROcp28_822);
    OPcp28_223 = OPcp28_222+ROcp28_822*qdd[23]-qd[23]*(OMcp28_122*ROcp28_922-OMcp28_322*ROcp28_722);
    OPcp28_323 = OPcp28_322+ROcp28_922*qdd[23]+qd[23]*(OMcp28_122*ROcp28_822-OMcp28_222*ROcp28_722);

// = = Block_1_0_0_29_0_6 = = 
 
// Sensor Kinematics 


    ROcp28_128 = ROcp28_123*C28-ROcp28_722*S28;
    ROcp28_228 = ROcp28_223*C28-ROcp28_822*S28;
    ROcp28_328 = ROcp28_323*C28-ROcp28_922*S28;
    ROcp28_728 = ROcp28_123*S28+ROcp28_722*C28;
    ROcp28_828 = ROcp28_223*S28+ROcp28_822*C28;
    ROcp28_928 = ROcp28_323*S28+ROcp28_922*C28;
    ROcp28_429 = ROcp28_423*C29+ROcp28_728*S29;
    ROcp28_529 = ROcp28_523*C29+ROcp28_828*S29;
    ROcp28_629 = ROcp28_623*C29+ROcp28_928*S29;
    ROcp28_729 = -(ROcp28_423*S29-ROcp28_728*C29);
    ROcp28_829 = -(ROcp28_523*S29-ROcp28_828*C29);
    ROcp28_929 = -(ROcp28_623*S29-ROcp28_928*C29);
    RLcp28_128 = ROcp28_123*s->dpt[1][42]+ROcp28_423*s->dpt[2][42]+ROcp28_722*s->dpt[3][42];
    RLcp28_228 = ROcp28_223*s->dpt[1][42]+ROcp28_523*s->dpt[2][42]+ROcp28_822*s->dpt[3][42];
    RLcp28_328 = ROcp28_323*s->dpt[1][42]+ROcp28_623*s->dpt[2][42]+ROcp28_922*s->dpt[3][42];
    OMcp28_128 = OMcp28_123+ROcp28_423*qd[28];
    OMcp28_228 = OMcp28_223+ROcp28_523*qd[28];
    OMcp28_328 = OMcp28_323+ROcp28_623*qd[28];
    ORcp28_128 = OMcp28_223*RLcp28_328-OMcp28_323*RLcp28_228;
    ORcp28_228 = -(OMcp28_123*RLcp28_328-OMcp28_323*RLcp28_128);
    ORcp28_328 = OMcp28_123*RLcp28_228-OMcp28_223*RLcp28_128;
    OPcp28_128 = OPcp28_123+ROcp28_423*qdd[28]+qd[28]*(OMcp28_223*ROcp28_623-OMcp28_323*ROcp28_523);
    OPcp28_228 = OPcp28_223+ROcp28_523*qdd[28]-qd[28]*(OMcp28_123*ROcp28_623-OMcp28_323*ROcp28_423);
    OPcp28_328 = OPcp28_323+ROcp28_623*qdd[28]+qd[28]*(OMcp28_123*ROcp28_523-OMcp28_223*ROcp28_423);
    RLcp28_129 = ROcp28_423*s->dpt[2][52];
    RLcp28_229 = ROcp28_523*s->dpt[2][52];
    RLcp28_329 = ROcp28_623*s->dpt[2][52];
    POcp28_129 = RLcp28_121+RLcp28_123+RLcp28_128+RLcp28_129+q[1];
    POcp28_229 = RLcp28_221+RLcp28_223+RLcp28_228+RLcp28_229+q[2];
    POcp28_329 = RLcp28_321+RLcp28_323+RLcp28_328+RLcp28_329+q[3];
    OMcp28_129 = OMcp28_128+ROcp28_128*qd[29];
    OMcp28_229 = OMcp28_228+ROcp28_228*qd[29];
    OMcp28_329 = OMcp28_328+ROcp28_328*qd[29];
    ORcp28_129 = OMcp28_228*RLcp28_329-OMcp28_328*RLcp28_229;
    ORcp28_229 = -(OMcp28_128*RLcp28_329-OMcp28_328*RLcp28_129);
    ORcp28_329 = OMcp28_128*RLcp28_229-OMcp28_228*RLcp28_129;
    VIcp28_129 = ORcp28_121+ORcp28_123+ORcp28_128+ORcp28_129+qd[1];
    VIcp28_229 = ORcp28_221+ORcp28_223+ORcp28_228+ORcp28_229+qd[2];
    VIcp28_329 = ORcp28_321+ORcp28_323+ORcp28_328+ORcp28_329+qd[3];
    OPcp28_129 = OPcp28_128+ROcp28_128*qdd[29]+qd[29]*(OMcp28_228*ROcp28_328-OMcp28_328*ROcp28_228);
    OPcp28_229 = OPcp28_228+ROcp28_228*qdd[29]-qd[29]*(OMcp28_128*ROcp28_328-OMcp28_328*ROcp28_128);
    OPcp28_329 = OPcp28_328+ROcp28_328*qdd[29]+qd[29]*(OMcp28_128*ROcp28_228-OMcp28_228*ROcp28_128);
    ACcp28_129 = qdd[1]+OMcp28_222*ORcp28_323+OMcp28_223*ORcp28_328+OMcp28_228*ORcp28_329+OMcp28_26*ORcp28_321-OMcp28_322*
 ORcp28_223-OMcp28_323*ORcp28_228-OMcp28_328*ORcp28_229-OMcp28_36*ORcp28_221+OPcp28_222*RLcp28_323+OPcp28_223*RLcp28_328+
 OPcp28_228*RLcp28_329+OPcp28_26*RLcp28_321-OPcp28_322*RLcp28_223-OPcp28_323*RLcp28_228-OPcp28_328*RLcp28_229-OPcp28_36*
 RLcp28_221;
    ACcp28_229 = qdd[2]-OMcp28_122*ORcp28_323-OMcp28_123*ORcp28_328-OMcp28_128*ORcp28_329-OMcp28_16*ORcp28_321+OMcp28_322*
 ORcp28_123+OMcp28_323*ORcp28_128+OMcp28_328*ORcp28_129+OMcp28_36*ORcp28_121-OPcp28_122*RLcp28_323-OPcp28_123*RLcp28_328-
 OPcp28_128*RLcp28_329-OPcp28_16*RLcp28_321+OPcp28_322*RLcp28_123+OPcp28_323*RLcp28_128+OPcp28_328*RLcp28_129+OPcp28_36*
 RLcp28_121;
    ACcp28_329 = qdd[3]+OMcp28_122*ORcp28_223+OMcp28_123*ORcp28_228+OMcp28_128*ORcp28_229+OMcp28_16*ORcp28_221-OMcp28_222*
 ORcp28_123-OMcp28_223*ORcp28_128-OMcp28_228*ORcp28_129-OMcp28_26*ORcp28_121+OPcp28_122*RLcp28_223+OPcp28_123*RLcp28_228+
 OPcp28_128*RLcp28_229+OPcp28_16*RLcp28_221-OPcp28_222*RLcp28_123-OPcp28_223*RLcp28_128-OPcp28_228*RLcp28_129-OPcp28_26*
 RLcp28_121;

// = = Block_1_0_0_29_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp28_129;
    sens->P[2] = POcp28_229;
    sens->P[3] = POcp28_329;
    sens->R[1][1] = ROcp28_128;
    sens->R[1][2] = ROcp28_228;
    sens->R[1][3] = ROcp28_328;
    sens->R[2][1] = ROcp28_429;
    sens->R[2][2] = ROcp28_529;
    sens->R[2][3] = ROcp28_629;
    sens->R[3][1] = ROcp28_729;
    sens->R[3][2] = ROcp28_829;
    sens->R[3][3] = ROcp28_929;
    sens->V[1] = VIcp28_129;
    sens->V[2] = VIcp28_229;
    sens->V[3] = VIcp28_329;
    sens->OM[1] = OMcp28_129;
    sens->OM[2] = OMcp28_229;
    sens->OM[3] = OMcp28_329;
    sens->A[1] = ACcp28_129;
    sens->A[2] = ACcp28_229;
    sens->A[3] = ACcp28_329;
    sens->OMP[1] = OPcp28_129;
    sens->OMP[2] = OPcp28_229;
    sens->OMP[3] = OPcp28_329;
 
// 
break;
case 30:
 


// = = Block_1_0_0_30_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp29_25 = qd[5]*C4;
    OMcp29_35 = qd[5]*S4;
    OMcp29_16 = qd[4]+qd[6]*S5;
    OMcp29_26 = OMcp29_25+ROcp29_85*qd[6];
    OMcp29_36 = OMcp29_35+ROcp29_95*qd[6];
    OPcp29_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp29_26 = ROcp29_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp29_35*S5-ROcp29_95*qd[4]);
    OPcp29_36 = ROcp29_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp29_25*S5-ROcp29_85*qd[4]);

// = = Block_1_0_0_30_0_4 = = 
 
// Sensor Kinematics 


    ROcp29_421 = ROcp29_46*C21+S21*S5;
    ROcp29_521 = ROcp29_56*C21+ROcp29_85*S21;
    ROcp29_621 = ROcp29_66*C21+ROcp29_95*S21;
    ROcp29_721 = -(ROcp29_46*S21-C21*S5);
    ROcp29_821 = -(ROcp29_56*S21-ROcp29_85*C21);
    ROcp29_921 = -(ROcp29_66*S21-ROcp29_95*C21);
    ROcp29_122 = ROcp29_16*C22-ROcp29_721*S22;
    ROcp29_222 = ROcp29_26*C22-ROcp29_821*S22;
    ROcp29_322 = ROcp29_36*C22-ROcp29_921*S22;
    ROcp29_722 = ROcp29_16*S22+ROcp29_721*C22;
    ROcp29_822 = ROcp29_26*S22+ROcp29_821*C22;
    ROcp29_922 = ROcp29_36*S22+ROcp29_921*C22;
    ROcp29_123 = ROcp29_122*C23+ROcp29_421*S23;
    ROcp29_223 = ROcp29_222*C23+ROcp29_521*S23;
    ROcp29_323 = ROcp29_322*C23+ROcp29_621*S23;
    ROcp29_423 = -(ROcp29_122*S23-ROcp29_421*C23);
    ROcp29_523 = -(ROcp29_222*S23-ROcp29_521*C23);
    ROcp29_623 = -(ROcp29_322*S23-ROcp29_621*C23);
    RLcp29_121 = ROcp29_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp29_221 = ROcp29_26*s->dpt[1][3]+ROcp29_85*s->dpt[3][3];
    RLcp29_321 = ROcp29_36*s->dpt[1][3]+ROcp29_95*s->dpt[3][3];
    OMcp29_121 = OMcp29_16+ROcp29_16*qd[21];
    OMcp29_221 = OMcp29_26+ROcp29_26*qd[21];
    OMcp29_321 = OMcp29_36+ROcp29_36*qd[21];
    ORcp29_121 = OMcp29_26*RLcp29_321-OMcp29_36*RLcp29_221;
    ORcp29_221 = -(OMcp29_16*RLcp29_321-OMcp29_36*RLcp29_121);
    ORcp29_321 = OMcp29_16*RLcp29_221-OMcp29_26*RLcp29_121;
    OMcp29_122 = OMcp29_121+ROcp29_421*qd[22];
    OMcp29_222 = OMcp29_221+ROcp29_521*qd[22];
    OMcp29_322 = OMcp29_321+ROcp29_621*qd[22];
    OPcp29_122 = OPcp29_16+ROcp29_16*qdd[21]+ROcp29_421*qdd[22]+qd[21]*(OMcp29_26*ROcp29_36-OMcp29_36*ROcp29_26)+qd[22]*(
 OMcp29_221*ROcp29_621-OMcp29_321*ROcp29_521);
    OPcp29_222 = OPcp29_26+ROcp29_26*qdd[21]+ROcp29_521*qdd[22]-qd[21]*(OMcp29_16*ROcp29_36-OMcp29_36*ROcp29_16)-qd[22]*(
 OMcp29_121*ROcp29_621-OMcp29_321*ROcp29_421);
    OPcp29_322 = OPcp29_36+ROcp29_36*qdd[21]+ROcp29_621*qdd[22]+qd[21]*(OMcp29_16*ROcp29_26-OMcp29_26*ROcp29_16)+qd[22]*(
 OMcp29_121*ROcp29_521-OMcp29_221*ROcp29_421);
    RLcp29_123 = ROcp29_722*s->dpt[3][37];
    RLcp29_223 = ROcp29_822*s->dpt[3][37];
    RLcp29_323 = ROcp29_922*s->dpt[3][37];
    OMcp29_123 = OMcp29_122+ROcp29_722*qd[23];
    OMcp29_223 = OMcp29_222+ROcp29_822*qd[23];
    OMcp29_323 = OMcp29_322+ROcp29_922*qd[23];
    ORcp29_123 = OMcp29_222*RLcp29_323-OMcp29_322*RLcp29_223;
    ORcp29_223 = -(OMcp29_122*RLcp29_323-OMcp29_322*RLcp29_123);
    ORcp29_323 = OMcp29_122*RLcp29_223-OMcp29_222*RLcp29_123;
    OPcp29_123 = OPcp29_122+ROcp29_722*qdd[23]+qd[23]*(OMcp29_222*ROcp29_922-OMcp29_322*ROcp29_822);
    OPcp29_223 = OPcp29_222+ROcp29_822*qdd[23]-qd[23]*(OMcp29_122*ROcp29_922-OMcp29_322*ROcp29_722);
    OPcp29_323 = OPcp29_322+ROcp29_922*qdd[23]+qd[23]*(OMcp29_122*ROcp29_822-OMcp29_222*ROcp29_722);

// = = Block_1_0_0_30_0_6 = = 
 
// Sensor Kinematics 


    ROcp29_128 = ROcp29_123*C28-ROcp29_722*S28;
    ROcp29_228 = ROcp29_223*C28-ROcp29_822*S28;
    ROcp29_328 = ROcp29_323*C28-ROcp29_922*S28;
    ROcp29_728 = ROcp29_123*S28+ROcp29_722*C28;
    ROcp29_828 = ROcp29_223*S28+ROcp29_822*C28;
    ROcp29_928 = ROcp29_323*S28+ROcp29_922*C28;
    ROcp29_429 = ROcp29_423*C29+ROcp29_728*S29;
    ROcp29_529 = ROcp29_523*C29+ROcp29_828*S29;
    ROcp29_629 = ROcp29_623*C29+ROcp29_928*S29;
    ROcp29_729 = -(ROcp29_423*S29-ROcp29_728*C29);
    ROcp29_829 = -(ROcp29_523*S29-ROcp29_828*C29);
    ROcp29_929 = -(ROcp29_623*S29-ROcp29_928*C29);
    ROcp29_130 = ROcp29_128*C30+ROcp29_429*S30;
    ROcp29_230 = ROcp29_228*C30+ROcp29_529*S30;
    ROcp29_330 = ROcp29_328*C30+ROcp29_629*S30;
    ROcp29_430 = -(ROcp29_128*S30-ROcp29_429*C30);
    ROcp29_530 = -(ROcp29_228*S30-ROcp29_529*C30);
    ROcp29_630 = -(ROcp29_328*S30-ROcp29_629*C30);
    RLcp29_128 = ROcp29_123*s->dpt[1][42]+ROcp29_423*s->dpt[2][42]+ROcp29_722*s->dpt[3][42];
    RLcp29_228 = ROcp29_223*s->dpt[1][42]+ROcp29_523*s->dpt[2][42]+ROcp29_822*s->dpt[3][42];
    RLcp29_328 = ROcp29_323*s->dpt[1][42]+ROcp29_623*s->dpt[2][42]+ROcp29_922*s->dpt[3][42];
    OMcp29_128 = OMcp29_123+ROcp29_423*qd[28];
    OMcp29_228 = OMcp29_223+ROcp29_523*qd[28];
    OMcp29_328 = OMcp29_323+ROcp29_623*qd[28];
    ORcp29_128 = OMcp29_223*RLcp29_328-OMcp29_323*RLcp29_228;
    ORcp29_228 = -(OMcp29_123*RLcp29_328-OMcp29_323*RLcp29_128);
    ORcp29_328 = OMcp29_123*RLcp29_228-OMcp29_223*RLcp29_128;
    OPcp29_128 = OPcp29_123+ROcp29_423*qdd[28]+qd[28]*(OMcp29_223*ROcp29_623-OMcp29_323*ROcp29_523);
    OPcp29_228 = OPcp29_223+ROcp29_523*qdd[28]-qd[28]*(OMcp29_123*ROcp29_623-OMcp29_323*ROcp29_423);
    OPcp29_328 = OPcp29_323+ROcp29_623*qdd[28]+qd[28]*(OMcp29_123*ROcp29_523-OMcp29_223*ROcp29_423);
    RLcp29_129 = ROcp29_423*s->dpt[2][52];
    RLcp29_229 = ROcp29_523*s->dpt[2][52];
    RLcp29_329 = ROcp29_623*s->dpt[2][52];
    OMcp29_129 = OMcp29_128+ROcp29_128*qd[29];
    OMcp29_229 = OMcp29_228+ROcp29_228*qd[29];
    OMcp29_329 = OMcp29_328+ROcp29_328*qd[29];
    ORcp29_129 = OMcp29_228*RLcp29_329-OMcp29_328*RLcp29_229;
    ORcp29_229 = -(OMcp29_128*RLcp29_329-OMcp29_328*RLcp29_129);
    ORcp29_329 = OMcp29_128*RLcp29_229-OMcp29_228*RLcp29_129;
    OPcp29_129 = OPcp29_128+ROcp29_128*qdd[29]+qd[29]*(OMcp29_228*ROcp29_328-OMcp29_328*ROcp29_228);
    OPcp29_229 = OPcp29_228+ROcp29_228*qdd[29]-qd[29]*(OMcp29_128*ROcp29_328-OMcp29_328*ROcp29_128);
    OPcp29_329 = OPcp29_328+ROcp29_328*qdd[29]+qd[29]*(OMcp29_128*ROcp29_228-OMcp29_228*ROcp29_128);
    RLcp29_130 = ROcp29_729*s->dpt[3][54];
    RLcp29_230 = ROcp29_829*s->dpt[3][54];
    RLcp29_330 = ROcp29_929*s->dpt[3][54];
    POcp29_130 = RLcp29_121+RLcp29_123+RLcp29_128+RLcp29_129+RLcp29_130+q[1];
    POcp29_230 = RLcp29_221+RLcp29_223+RLcp29_228+RLcp29_229+RLcp29_230+q[2];
    POcp29_330 = RLcp29_321+RLcp29_323+RLcp29_328+RLcp29_329+RLcp29_330+q[3];
    OMcp29_130 = OMcp29_129+ROcp29_729*qd[30];
    OMcp29_230 = OMcp29_229+ROcp29_829*qd[30];
    OMcp29_330 = OMcp29_329+ROcp29_929*qd[30];
    ORcp29_130 = OMcp29_229*RLcp29_330-OMcp29_329*RLcp29_230;
    ORcp29_230 = -(OMcp29_129*RLcp29_330-OMcp29_329*RLcp29_130);
    ORcp29_330 = OMcp29_129*RLcp29_230-OMcp29_229*RLcp29_130;
    VIcp29_130 = ORcp29_121+ORcp29_123+ORcp29_128+ORcp29_129+ORcp29_130+qd[1];
    VIcp29_230 = ORcp29_221+ORcp29_223+ORcp29_228+ORcp29_229+ORcp29_230+qd[2];
    VIcp29_330 = ORcp29_321+ORcp29_323+ORcp29_328+ORcp29_329+ORcp29_330+qd[3];
    OPcp29_130 = OPcp29_129+ROcp29_729*qdd[30]+qd[30]*(OMcp29_229*ROcp29_929-OMcp29_329*ROcp29_829);
    OPcp29_230 = OPcp29_229+ROcp29_829*qdd[30]-qd[30]*(OMcp29_129*ROcp29_929-OMcp29_329*ROcp29_729);
    OPcp29_330 = OPcp29_329+ROcp29_929*qdd[30]+qd[30]*(OMcp29_129*ROcp29_829-OMcp29_229*ROcp29_729);
    ACcp29_130 = qdd[1]+OMcp29_222*ORcp29_323+OMcp29_223*ORcp29_328+OMcp29_228*ORcp29_329+OMcp29_229*ORcp29_330+OMcp29_26*
 ORcp29_321-OMcp29_322*ORcp29_223-OMcp29_323*ORcp29_228-OMcp29_328*ORcp29_229-OMcp29_329*ORcp29_230-OMcp29_36*ORcp29_221+
 OPcp29_222*RLcp29_323+OPcp29_223*RLcp29_328+OPcp29_228*RLcp29_329+OPcp29_229*RLcp29_330+OPcp29_26*RLcp29_321-OPcp29_322*
 RLcp29_223-OPcp29_323*RLcp29_228-OPcp29_328*RLcp29_229-OPcp29_329*RLcp29_230-OPcp29_36*RLcp29_221;
    ACcp29_230 = qdd[2]-OMcp29_122*ORcp29_323-OMcp29_123*ORcp29_328-OMcp29_128*ORcp29_329-OMcp29_129*ORcp29_330-OMcp29_16*
 ORcp29_321+OMcp29_322*ORcp29_123+OMcp29_323*ORcp29_128+OMcp29_328*ORcp29_129+OMcp29_329*ORcp29_130+OMcp29_36*ORcp29_121-
 OPcp29_122*RLcp29_323-OPcp29_123*RLcp29_328-OPcp29_128*RLcp29_329-OPcp29_129*RLcp29_330-OPcp29_16*RLcp29_321+OPcp29_322*
 RLcp29_123+OPcp29_323*RLcp29_128+OPcp29_328*RLcp29_129+OPcp29_329*RLcp29_130+OPcp29_36*RLcp29_121;
    ACcp29_330 = qdd[3]+OMcp29_122*ORcp29_223+OMcp29_123*ORcp29_228+OMcp29_128*ORcp29_229+OMcp29_129*ORcp29_230+OMcp29_16*
 ORcp29_221-OMcp29_222*ORcp29_123-OMcp29_223*ORcp29_128-OMcp29_228*ORcp29_129-OMcp29_229*ORcp29_130-OMcp29_26*ORcp29_121+
 OPcp29_122*RLcp29_223+OPcp29_123*RLcp29_228+OPcp29_128*RLcp29_229+OPcp29_129*RLcp29_230+OPcp29_16*RLcp29_221-OPcp29_222*
 RLcp29_123-OPcp29_223*RLcp29_128-OPcp29_228*RLcp29_129-OPcp29_229*RLcp29_130-OPcp29_26*RLcp29_121;

// = = Block_1_0_0_30_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp29_130;
    sens->P[2] = POcp29_230;
    sens->P[3] = POcp29_330;
    sens->R[1][1] = ROcp29_130;
    sens->R[1][2] = ROcp29_230;
    sens->R[1][3] = ROcp29_330;
    sens->R[2][1] = ROcp29_430;
    sens->R[2][2] = ROcp29_530;
    sens->R[2][3] = ROcp29_630;
    sens->R[3][1] = ROcp29_729;
    sens->R[3][2] = ROcp29_829;
    sens->R[3][3] = ROcp29_929;
    sens->V[1] = VIcp29_130;
    sens->V[2] = VIcp29_230;
    sens->V[3] = VIcp29_330;
    sens->OM[1] = OMcp29_130;
    sens->OM[2] = OMcp29_230;
    sens->OM[3] = OMcp29_330;
    sens->A[1] = ACcp29_130;
    sens->A[2] = ACcp29_230;
    sens->A[3] = ACcp29_330;
    sens->OMP[1] = OPcp29_130;
    sens->OMP[2] = OPcp29_230;
    sens->OMP[3] = OPcp29_330;
 
// 
break;
case 31:
 


// = = Block_1_0_0_31_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp30_25 = qd[5]*C4;
    OMcp30_35 = qd[5]*S4;
    OMcp30_16 = qd[4]+qd[6]*S5;
    OMcp30_26 = OMcp30_25+ROcp30_85*qd[6];
    OMcp30_36 = OMcp30_35+ROcp30_95*qd[6];
    OPcp30_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp30_26 = ROcp30_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp30_35*S5-ROcp30_95*qd[4]);
    OPcp30_36 = ROcp30_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp30_25*S5-ROcp30_85*qd[4]);

// = = Block_1_0_0_31_0_4 = = 
 
// Sensor Kinematics 


    ROcp30_421 = ROcp30_46*C21+S21*S5;
    ROcp30_521 = ROcp30_56*C21+ROcp30_85*S21;
    ROcp30_621 = ROcp30_66*C21+ROcp30_95*S21;
    ROcp30_721 = -(ROcp30_46*S21-C21*S5);
    ROcp30_821 = -(ROcp30_56*S21-ROcp30_85*C21);
    ROcp30_921 = -(ROcp30_66*S21-ROcp30_95*C21);
    ROcp30_122 = ROcp30_16*C22-ROcp30_721*S22;
    ROcp30_222 = ROcp30_26*C22-ROcp30_821*S22;
    ROcp30_322 = ROcp30_36*C22-ROcp30_921*S22;
    ROcp30_722 = ROcp30_16*S22+ROcp30_721*C22;
    ROcp30_822 = ROcp30_26*S22+ROcp30_821*C22;
    ROcp30_922 = ROcp30_36*S22+ROcp30_921*C22;
    ROcp30_123 = ROcp30_122*C23+ROcp30_421*S23;
    ROcp30_223 = ROcp30_222*C23+ROcp30_521*S23;
    ROcp30_323 = ROcp30_322*C23+ROcp30_621*S23;
    ROcp30_423 = -(ROcp30_122*S23-ROcp30_421*C23);
    ROcp30_523 = -(ROcp30_222*S23-ROcp30_521*C23);
    ROcp30_623 = -(ROcp30_322*S23-ROcp30_621*C23);
    RLcp30_121 = ROcp30_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp30_221 = ROcp30_26*s->dpt[1][3]+ROcp30_85*s->dpt[3][3];
    RLcp30_321 = ROcp30_36*s->dpt[1][3]+ROcp30_95*s->dpt[3][3];
    OMcp30_121 = OMcp30_16+ROcp30_16*qd[21];
    OMcp30_221 = OMcp30_26+ROcp30_26*qd[21];
    OMcp30_321 = OMcp30_36+ROcp30_36*qd[21];
    ORcp30_121 = OMcp30_26*RLcp30_321-OMcp30_36*RLcp30_221;
    ORcp30_221 = -(OMcp30_16*RLcp30_321-OMcp30_36*RLcp30_121);
    ORcp30_321 = OMcp30_16*RLcp30_221-OMcp30_26*RLcp30_121;
    OMcp30_122 = OMcp30_121+ROcp30_421*qd[22];
    OMcp30_222 = OMcp30_221+ROcp30_521*qd[22];
    OMcp30_322 = OMcp30_321+ROcp30_621*qd[22];
    OPcp30_122 = OPcp30_16+ROcp30_16*qdd[21]+ROcp30_421*qdd[22]+qd[21]*(OMcp30_26*ROcp30_36-OMcp30_36*ROcp30_26)+qd[22]*(
 OMcp30_221*ROcp30_621-OMcp30_321*ROcp30_521);
    OPcp30_222 = OPcp30_26+ROcp30_26*qdd[21]+ROcp30_521*qdd[22]-qd[21]*(OMcp30_16*ROcp30_36-OMcp30_36*ROcp30_16)-qd[22]*(
 OMcp30_121*ROcp30_621-OMcp30_321*ROcp30_421);
    OPcp30_322 = OPcp30_36+ROcp30_36*qdd[21]+ROcp30_621*qdd[22]+qd[21]*(OMcp30_16*ROcp30_26-OMcp30_26*ROcp30_16)+qd[22]*(
 OMcp30_121*ROcp30_521-OMcp30_221*ROcp30_421);
    RLcp30_123 = ROcp30_722*s->dpt[3][37];
    RLcp30_223 = ROcp30_822*s->dpt[3][37];
    RLcp30_323 = ROcp30_922*s->dpt[3][37];
    OMcp30_123 = OMcp30_122+ROcp30_722*qd[23];
    OMcp30_223 = OMcp30_222+ROcp30_822*qd[23];
    OMcp30_323 = OMcp30_322+ROcp30_922*qd[23];
    ORcp30_123 = OMcp30_222*RLcp30_323-OMcp30_322*RLcp30_223;
    ORcp30_223 = -(OMcp30_122*RLcp30_323-OMcp30_322*RLcp30_123);
    ORcp30_323 = OMcp30_122*RLcp30_223-OMcp30_222*RLcp30_123;
    OPcp30_123 = OPcp30_122+ROcp30_722*qdd[23]+qd[23]*(OMcp30_222*ROcp30_922-OMcp30_322*ROcp30_822);
    OPcp30_223 = OPcp30_222+ROcp30_822*qdd[23]-qd[23]*(OMcp30_122*ROcp30_922-OMcp30_322*ROcp30_722);
    OPcp30_323 = OPcp30_322+ROcp30_922*qdd[23]+qd[23]*(OMcp30_122*ROcp30_822-OMcp30_222*ROcp30_722);

// = = Block_1_0_0_31_0_6 = = 
 
// Sensor Kinematics 


    ROcp30_128 = ROcp30_123*C28-ROcp30_722*S28;
    ROcp30_228 = ROcp30_223*C28-ROcp30_822*S28;
    ROcp30_328 = ROcp30_323*C28-ROcp30_922*S28;
    ROcp30_728 = ROcp30_123*S28+ROcp30_722*C28;
    ROcp30_828 = ROcp30_223*S28+ROcp30_822*C28;
    ROcp30_928 = ROcp30_323*S28+ROcp30_922*C28;
    ROcp30_429 = ROcp30_423*C29+ROcp30_728*S29;
    ROcp30_529 = ROcp30_523*C29+ROcp30_828*S29;
    ROcp30_629 = ROcp30_623*C29+ROcp30_928*S29;
    ROcp30_729 = -(ROcp30_423*S29-ROcp30_728*C29);
    ROcp30_829 = -(ROcp30_523*S29-ROcp30_828*C29);
    ROcp30_929 = -(ROcp30_623*S29-ROcp30_928*C29);
    ROcp30_130 = ROcp30_128*C30+ROcp30_429*S30;
    ROcp30_230 = ROcp30_228*C30+ROcp30_529*S30;
    ROcp30_330 = ROcp30_328*C30+ROcp30_629*S30;
    ROcp30_430 = -(ROcp30_128*S30-ROcp30_429*C30);
    ROcp30_530 = -(ROcp30_228*S30-ROcp30_529*C30);
    ROcp30_630 = -(ROcp30_328*S30-ROcp30_629*C30);
    ROcp30_131 = ROcp30_130*C31-ROcp30_729*S31;
    ROcp30_231 = ROcp30_230*C31-ROcp30_829*S31;
    ROcp30_331 = ROcp30_330*C31-ROcp30_929*S31;
    ROcp30_731 = ROcp30_130*S31+ROcp30_729*C31;
    ROcp30_831 = ROcp30_230*S31+ROcp30_829*C31;
    ROcp30_931 = ROcp30_330*S31+ROcp30_929*C31;
    RLcp30_128 = ROcp30_123*s->dpt[1][42]+ROcp30_423*s->dpt[2][42]+ROcp30_722*s->dpt[3][42];
    RLcp30_228 = ROcp30_223*s->dpt[1][42]+ROcp30_523*s->dpt[2][42]+ROcp30_822*s->dpt[3][42];
    RLcp30_328 = ROcp30_323*s->dpt[1][42]+ROcp30_623*s->dpt[2][42]+ROcp30_922*s->dpt[3][42];
    OMcp30_128 = OMcp30_123+ROcp30_423*qd[28];
    OMcp30_228 = OMcp30_223+ROcp30_523*qd[28];
    OMcp30_328 = OMcp30_323+ROcp30_623*qd[28];
    ORcp30_128 = OMcp30_223*RLcp30_328-OMcp30_323*RLcp30_228;
    ORcp30_228 = -(OMcp30_123*RLcp30_328-OMcp30_323*RLcp30_128);
    ORcp30_328 = OMcp30_123*RLcp30_228-OMcp30_223*RLcp30_128;
    OPcp30_128 = OPcp30_123+ROcp30_423*qdd[28]+qd[28]*(OMcp30_223*ROcp30_623-OMcp30_323*ROcp30_523);
    OPcp30_228 = OPcp30_223+ROcp30_523*qdd[28]-qd[28]*(OMcp30_123*ROcp30_623-OMcp30_323*ROcp30_423);
    OPcp30_328 = OPcp30_323+ROcp30_623*qdd[28]+qd[28]*(OMcp30_123*ROcp30_523-OMcp30_223*ROcp30_423);
    RLcp30_129 = ROcp30_423*s->dpt[2][52];
    RLcp30_229 = ROcp30_523*s->dpt[2][52];
    RLcp30_329 = ROcp30_623*s->dpt[2][52];
    OMcp30_129 = OMcp30_128+ROcp30_128*qd[29];
    OMcp30_229 = OMcp30_228+ROcp30_228*qd[29];
    OMcp30_329 = OMcp30_328+ROcp30_328*qd[29];
    ORcp30_129 = OMcp30_228*RLcp30_329-OMcp30_328*RLcp30_229;
    ORcp30_229 = -(OMcp30_128*RLcp30_329-OMcp30_328*RLcp30_129);
    ORcp30_329 = OMcp30_128*RLcp30_229-OMcp30_228*RLcp30_129;
    OPcp30_129 = OPcp30_128+ROcp30_128*qdd[29]+qd[29]*(OMcp30_228*ROcp30_328-OMcp30_328*ROcp30_228);
    OPcp30_229 = OPcp30_228+ROcp30_228*qdd[29]-qd[29]*(OMcp30_128*ROcp30_328-OMcp30_328*ROcp30_128);
    OPcp30_329 = OPcp30_328+ROcp30_328*qdd[29]+qd[29]*(OMcp30_128*ROcp30_228-OMcp30_228*ROcp30_128);
    RLcp30_130 = ROcp30_729*s->dpt[3][54];
    RLcp30_230 = ROcp30_829*s->dpt[3][54];
    RLcp30_330 = ROcp30_929*s->dpt[3][54];
    OMcp30_130 = OMcp30_129+ROcp30_729*qd[30];
    OMcp30_230 = OMcp30_229+ROcp30_829*qd[30];
    OMcp30_330 = OMcp30_329+ROcp30_929*qd[30];
    ORcp30_130 = OMcp30_229*RLcp30_330-OMcp30_329*RLcp30_230;
    ORcp30_230 = -(OMcp30_129*RLcp30_330-OMcp30_329*RLcp30_130);
    ORcp30_330 = OMcp30_129*RLcp30_230-OMcp30_229*RLcp30_130;
    OPcp30_130 = OPcp30_129+ROcp30_729*qdd[30]+qd[30]*(OMcp30_229*ROcp30_929-OMcp30_329*ROcp30_829);
    OPcp30_230 = OPcp30_229+ROcp30_829*qdd[30]-qd[30]*(OMcp30_129*ROcp30_929-OMcp30_329*ROcp30_729);
    OPcp30_330 = OPcp30_329+ROcp30_929*qdd[30]+qd[30]*(OMcp30_129*ROcp30_829-OMcp30_229*ROcp30_729);
    RLcp30_131 = ROcp30_729*s->dpt[3][56];
    RLcp30_231 = ROcp30_829*s->dpt[3][56];
    RLcp30_331 = ROcp30_929*s->dpt[3][56];
    POcp30_131 = RLcp30_121+RLcp30_123+RLcp30_128+RLcp30_129+RLcp30_130+RLcp30_131+q[1];
    POcp30_231 = RLcp30_221+RLcp30_223+RLcp30_228+RLcp30_229+RLcp30_230+RLcp30_231+q[2];
    POcp30_331 = RLcp30_321+RLcp30_323+RLcp30_328+RLcp30_329+RLcp30_330+RLcp30_331+q[3];
    OMcp30_131 = OMcp30_130+ROcp30_430*qd[31];
    OMcp30_231 = OMcp30_230+ROcp30_530*qd[31];
    OMcp30_331 = OMcp30_330+ROcp30_630*qd[31];
    ORcp30_131 = OMcp30_230*RLcp30_331-OMcp30_330*RLcp30_231;
    ORcp30_231 = -(OMcp30_130*RLcp30_331-OMcp30_330*RLcp30_131);
    ORcp30_331 = OMcp30_130*RLcp30_231-OMcp30_230*RLcp30_131;
    VIcp30_131 = ORcp30_121+ORcp30_123+ORcp30_128+ORcp30_129+ORcp30_130+ORcp30_131+qd[1];
    VIcp30_231 = ORcp30_221+ORcp30_223+ORcp30_228+ORcp30_229+ORcp30_230+ORcp30_231+qd[2];
    VIcp30_331 = ORcp30_321+ORcp30_323+ORcp30_328+ORcp30_329+ORcp30_330+ORcp30_331+qd[3];
    OPcp30_131 = OPcp30_130+ROcp30_430*qdd[31]+qd[31]*(OMcp30_230*ROcp30_630-OMcp30_330*ROcp30_530);
    OPcp30_231 = OPcp30_230+ROcp30_530*qdd[31]-qd[31]*(OMcp30_130*ROcp30_630-OMcp30_330*ROcp30_430);
    OPcp30_331 = OPcp30_330+ROcp30_630*qdd[31]+qd[31]*(OMcp30_130*ROcp30_530-OMcp30_230*ROcp30_430);
    ACcp30_131 = qdd[1]+OMcp30_222*ORcp30_323+OMcp30_223*ORcp30_328+OMcp30_228*ORcp30_329+OMcp30_229*ORcp30_330+OMcp30_230
 *ORcp30_331+OMcp30_26*ORcp30_321-OMcp30_322*ORcp30_223-OMcp30_323*ORcp30_228-OMcp30_328*ORcp30_229-OMcp30_329*ORcp30_230-
 OMcp30_330*ORcp30_231-OMcp30_36*ORcp30_221+OPcp30_222*RLcp30_323+OPcp30_223*RLcp30_328+OPcp30_228*RLcp30_329+OPcp30_229*
 RLcp30_330+OPcp30_230*RLcp30_331+OPcp30_26*RLcp30_321-OPcp30_322*RLcp30_223-OPcp30_323*RLcp30_228-OPcp30_328*RLcp30_229-
 OPcp30_329*RLcp30_230-OPcp30_330*RLcp30_231-OPcp30_36*RLcp30_221;
    ACcp30_231 = qdd[2]-OMcp30_122*ORcp30_323-OMcp30_123*ORcp30_328-OMcp30_128*ORcp30_329-OMcp30_129*ORcp30_330-OMcp30_130
 *ORcp30_331-OMcp30_16*ORcp30_321+OMcp30_322*ORcp30_123+OMcp30_323*ORcp30_128+OMcp30_328*ORcp30_129+OMcp30_329*ORcp30_130+
 OMcp30_330*ORcp30_131+OMcp30_36*ORcp30_121-OPcp30_122*RLcp30_323-OPcp30_123*RLcp30_328-OPcp30_128*RLcp30_329-OPcp30_129*
 RLcp30_330-OPcp30_130*RLcp30_331-OPcp30_16*RLcp30_321+OPcp30_322*RLcp30_123+OPcp30_323*RLcp30_128+OPcp30_328*RLcp30_129+
 OPcp30_329*RLcp30_130+OPcp30_330*RLcp30_131+OPcp30_36*RLcp30_121;
    ACcp30_331 = qdd[3]+OMcp30_122*ORcp30_223+OMcp30_123*ORcp30_228+OMcp30_128*ORcp30_229+OMcp30_129*ORcp30_230+OMcp30_130
 *ORcp30_231+OMcp30_16*ORcp30_221-OMcp30_222*ORcp30_123-OMcp30_223*ORcp30_128-OMcp30_228*ORcp30_129-OMcp30_229*ORcp30_130-
 OMcp30_230*ORcp30_131-OMcp30_26*ORcp30_121+OPcp30_122*RLcp30_223+OPcp30_123*RLcp30_228+OPcp30_128*RLcp30_229+OPcp30_129*
 RLcp30_230+OPcp30_130*RLcp30_231+OPcp30_16*RLcp30_221-OPcp30_222*RLcp30_123-OPcp30_223*RLcp30_128-OPcp30_228*RLcp30_129-
 OPcp30_229*RLcp30_130-OPcp30_230*RLcp30_131-OPcp30_26*RLcp30_121;

// = = Block_1_0_0_31_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp30_131;
    sens->P[2] = POcp30_231;
    sens->P[3] = POcp30_331;
    sens->R[1][1] = ROcp30_131;
    sens->R[1][2] = ROcp30_231;
    sens->R[1][3] = ROcp30_331;
    sens->R[2][1] = ROcp30_430;
    sens->R[2][2] = ROcp30_530;
    sens->R[2][3] = ROcp30_630;
    sens->R[3][1] = ROcp30_731;
    sens->R[3][2] = ROcp30_831;
    sens->R[3][3] = ROcp30_931;
    sens->V[1] = VIcp30_131;
    sens->V[2] = VIcp30_231;
    sens->V[3] = VIcp30_331;
    sens->OM[1] = OMcp30_131;
    sens->OM[2] = OMcp30_231;
    sens->OM[3] = OMcp30_331;
    sens->A[1] = ACcp30_131;
    sens->A[2] = ACcp30_231;
    sens->A[3] = ACcp30_331;
    sens->OMP[1] = OPcp30_131;
    sens->OMP[2] = OPcp30_231;
    sens->OMP[3] = OPcp30_331;

break;
default:
break;
}


// ====== END Task 1 ====== 


}
 

