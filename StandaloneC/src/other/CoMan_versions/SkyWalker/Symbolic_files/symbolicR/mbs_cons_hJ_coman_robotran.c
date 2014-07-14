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
//	==> Generation Date : Wed Apr 30 10:12:04 2014
//
//	==> Project name : coman_robotran
//	==> using XML input file 
//
//	==> Number of joints : 33
//
//	==> Function : F 8 : Constraints Vector (h) and Jacobian Matrix (Jac) 
//	==> Flops complexity : 354
//
//	==> Generation Time :  0.010 seconds
//	==> Post-Processing :  0.010 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "MBSdataStructR7.h"
#include "MBSfunR7.h"
 
void cons_hJ(double *h,double **Jac,
MBSdataStruct *s, double tsim)

// double h[6];
// double Jac[6][33];
{ 
 
#include "mbs_cons_hJ_coman_robotran.h" 
#define q s->q 
#define qd s->qd 
#define qdd s->qdd 
 
 

// === begin imp_aux === 

// === end imp_aux === 

// ===== BEGIN task 0 ===== 

// = = Block_0_0_0_0_0_1 = = 
 
// Trigonometric Variables  

  C4 = cos(q[4]);
  S4 = sin(q[4]);
  C5 = cos(q[5]);
  S5 = sin(q[5]);
  C6 = cos(q[6]);
  S6 = sin(q[6]);

// = = Block_0_0_0_0_0_4 = = 
 
// Trigonometric Variables  

  C19 = cos(q[19]);
  S19 = sin(q[19]);
  C20 = cos(q[20]);
  S20 = sin(q[20]);
  C21 = cos(q[21]);
  S21 = sin(q[21]);

// = = Block_0_0_0_0_0_7 = = 
 
// Trigonometric Variables  

  C30 = cos(q[30]);
  S30 = sin(q[30]);
  C31 = cos(q[31]);
  S31 = sin(q[31]);

// = = Block_0_0_0_0_0_8 = = 
 
// Trigonometric Variables  

  C32 = cos(q[32]);
  S32 = sin(q[32]);
  C33 = cos(q[33]);
  S33 = sin(q[33]);

// = = Block_0_1_0_0_0_1 = = 
 
// Constraints and Constraints Jacobian 

//
  RO_1_25 = S4*S5;
  RO_1_35 = -C4*S5;
  RO_1_85 = -S4*C5;
  RO_1_95 = C4*C5;
  RO_1_16 = C5*C6;
  RO_1_26 = RO_1_25*C6+C4*S6;
  RO_1_36 = RO_1_35*C6+S4*S6;
  RO_1_46 = -C5*S6;
  RO_1_56 = -(RO_1_25*S6-C4*C6);
  RO_1_66 = -(RO_1_35*S6-S4*C6);

// = = Block_0_1_0_0_0_4 = = 
 
// Constraints and Constraints Jacobian 

//
  RO_1_419 = RO_1_46*C19+S19*S5;
  RO_1_519 = RO_1_56*C19+RO_1_85*S19;
  RO_1_619 = RO_1_66*C19+RO_1_95*S19;
  RO_1_719 = -(RO_1_46*S19-C19*S5);
  RO_1_819 = -(RO_1_56*S19-RO_1_85*C19);
  RO_1_919 = -(RO_1_66*S19-RO_1_95*C19);
  RO_1_120 = RO_1_16*C20-RO_1_719*S20;
  RO_1_220 = RO_1_26*C20-RO_1_819*S20;
  RO_1_320 = RO_1_36*C20-RO_1_919*S20;
  RO_1_720 = RO_1_16*S20+RO_1_719*C20;
  RO_1_820 = RO_1_26*S20+RO_1_819*C20;
  RO_1_920 = RO_1_36*S20+RO_1_919*C20;
  RO_1_121 = RO_1_120*C21+RO_1_419*S21;
  RO_1_221 = RO_1_220*C21+RO_1_519*S21;
  RO_1_321 = RO_1_320*C21+RO_1_619*S21;
  RO_1_421 = -(RO_1_120*S21-RO_1_419*C21);
  RO_1_521 = -(RO_1_220*S21-RO_1_519*C21);
  RO_1_621 = -(RO_1_320*S21-RO_1_619*C21);
  RL_1_119 = RO_1_16*s->dpt[1][5]+s->dpt[3][5]*S5;
  RL_1_219 = RO_1_26*s->dpt[1][5]+RO_1_85*s->dpt[3][5];
  RL_1_319 = RO_1_36*s->dpt[1][5]+RO_1_95*s->dpt[3][5];
  RL_1_121 = RO_1_720*s->dpt[3][35];
  RL_1_221 = RO_1_820*s->dpt[3][35];
  RL_1_321 = RO_1_920*s->dpt[3][35];
  PO_1_121 = RL_1_119+RL_1_121+q[1];
  PO_1_221 = RL_1_219+RL_1_221+q[2];
  PO_1_321 = RL_1_319+RL_1_321+q[3];
  JT_1_221_4 = -(RL_1_319+RL_1_321);
  JT_1_321_4 = RL_1_219+RL_1_221;
  JT_1_121_5 = C4*(RL_1_319+RL_1_321)-S4*(RL_1_219+RL_1_221);
  JT_1_221_5 = S4*(RL_1_119+RL_1_121);
  JT_1_321_5 = -C4*(RL_1_119+RL_1_121);
  JT_1_121_6 = RO_1_85*(RL_1_319+RL_1_321)-RO_1_95*(RL_1_219+RL_1_221);
  JT_1_221_6 = RO_1_95*(RL_1_119+RL_1_121)-S5*(RL_1_319+RL_1_321);
  JT_1_321_6 = -(RO_1_85*(RL_1_119+RL_1_121)-S5*(RL_1_219+RL_1_221));
  JT_1_121_19 = -(RL_1_221*RO_1_36-RL_1_321*RO_1_26);
  JT_1_221_19 = RL_1_121*RO_1_36-RL_1_321*RO_1_16;
  JT_1_321_19 = -(RL_1_121*RO_1_26-RL_1_221*RO_1_16);
  JT_1_121_20 = -(RL_1_221*RO_1_619-RL_1_321*RO_1_519);
  JT_1_221_20 = RL_1_121*RO_1_619-RL_1_321*RO_1_419;
  JT_1_321_20 = -(RL_1_121*RO_1_519-RL_1_221*RO_1_419);
  RL_1_135 = RO_1_121*s->dpt[1][42]+RO_1_421*s->dpt[2][42]+RO_1_720*s->dpt[3][42];
  RL_1_235 = RO_1_221*s->dpt[1][42]+RO_1_521*s->dpt[2][42]+RO_1_820*s->dpt[3][42];
  RL_1_335 = RO_1_321*s->dpt[1][42]+RO_1_621*s->dpt[2][42]+RO_1_920*s->dpt[3][42];
  JT_1_235_4 = JT_1_221_4-RL_1_335;
  JT_1_335_4 = JT_1_321_4+RL_1_235;
  JT_1_135_5 = JT_1_121_5-RL_1_235*S4+RL_1_335*C4;
  JT_1_235_5 = JT_1_221_5+RL_1_135*S4;
  JT_1_335_5 = JT_1_321_5-RL_1_135*C4;
  JT_1_135_6 = JT_1_121_6-RL_1_235*RO_1_95+RL_1_335*RO_1_85;
  JT_1_235_6 = JT_1_221_6+RL_1_135*RO_1_95-RL_1_335*S5;
  JT_1_335_6 = JT_1_321_6-RL_1_135*RO_1_85+RL_1_235*S5;
  JT_1_135_19 = JT_1_121_19-RL_1_235*RO_1_36+RL_1_335*RO_1_26;
  JT_1_235_19 = JT_1_221_19+RL_1_135*RO_1_36-RL_1_335*RO_1_16;
  JT_1_335_19 = JT_1_321_19-RL_1_135*RO_1_26+RL_1_235*RO_1_16;
  JT_1_135_20 = JT_1_121_20-RL_1_235*RO_1_619+RL_1_335*RO_1_519;
  JT_1_235_20 = JT_1_221_20+RL_1_135*RO_1_619-RL_1_335*RO_1_419;
  JT_1_335_20 = JT_1_321_20-RL_1_135*RO_1_519+RL_1_235*RO_1_419;
  JT_1_135_21 = -(RL_1_235*RO_1_920-RL_1_335*RO_1_820);
  JT_1_235_21 = RL_1_135*RO_1_920-RL_1_335*RO_1_720;
  JT_1_335_21 = -(RL_1_135*RO_1_820-RL_1_235*RO_1_720);
//
  RL_3_137 = RO_1_121*s->dpt[1][41]+RO_1_421*s->dpt[2][41]+RO_1_720*s->dpt[3][41];
  RL_3_237 = RO_1_221*s->dpt[1][41]+RO_1_521*s->dpt[2][41]+RO_1_820*s->dpt[3][41];
  RL_3_337 = RO_1_321*s->dpt[1][41]+RO_1_621*s->dpt[2][41]+RO_1_920*s->dpt[3][41];
  JT_3_237_4 = JT_1_221_4-RL_3_337;
  JT_3_337_4 = JT_1_321_4+RL_3_237;
  JT_3_137_5 = JT_1_121_5-RL_3_237*S4+RL_3_337*C4;
  JT_3_237_5 = JT_1_221_5+RL_3_137*S4;
  JT_3_337_5 = JT_1_321_5-RL_3_137*C4;
  JT_3_137_6 = JT_1_121_6-RL_3_237*RO_1_95+RL_3_337*RO_1_85;
  JT_3_237_6 = JT_1_221_6+RL_3_137*RO_1_95-RL_3_337*S5;
  JT_3_337_6 = JT_1_321_6-RL_3_137*RO_1_85+RL_3_237*S5;
  JT_3_137_19 = JT_1_121_19-RL_3_237*RO_1_36+RL_3_337*RO_1_26;
  JT_3_237_19 = JT_1_221_19+RL_3_137*RO_1_36-RL_3_337*RO_1_16;
  JT_3_337_19 = JT_1_321_19-RL_3_137*RO_1_26+RL_3_237*RO_1_16;
  JT_3_137_20 = JT_1_121_20-RL_3_237*RO_1_619+RL_3_337*RO_1_519;
  JT_3_237_20 = JT_1_221_20+RL_3_137*RO_1_619-RL_3_337*RO_1_419;
  JT_3_337_20 = JT_1_321_20-RL_3_137*RO_1_519+RL_3_237*RO_1_419;
  JT_3_137_21 = -(RL_3_237*RO_1_920-RL_3_337*RO_1_820);
  JT_3_237_21 = RL_3_137*RO_1_920-RL_3_337*RO_1_720;
  JT_3_337_21 = -(RL_3_137*RO_1_820-RL_3_237*RO_1_720);

// = = Block_0_1_0_0_0_7 = = 
 
// Constraints and Constraints Jacobian 

//
  RL_0_134 = s->dpt[3][59]*S31;
  RL_0_234 = -s->dpt[3][59]*S30*C31;
  RL_0_334 = s->dpt[3][59]*C30*C31;
  JT_0_134_31 = s->dpt[3][59]*C31;
  JT_0_234_31 = RL_0_134*S30;
  JT_0_334_31 = -RL_0_134*C30;

// = = Block_0_1_0_0_0_8 = = 
 
// Constraints and Constraints Jacobian 

//
  RL_2_136 = s->dpt[3][60]*S33;
  RL_2_236 = -s->dpt[3][60]*S32*C33;
  RL_2_336 = s->dpt[3][60]*C32*C33;
  JT_2_136_33 = s->dpt[3][60]*C33;
  JT_2_236_33 = RL_2_136*S32;
  JT_2_336_33 = -RL_2_136*C32;

// = = Block_0_1_0_0_1_0 = = 
 
// Constraints and Constraints Jacobian 

//
  h_1 = -(PO_1_121-RL_0_134+RL_1_135);
  h_2 = RL_0_234+s->dpt[2][1]-(PO_1_221+RL_1_235);
  h_3 = RL_0_334+s->dpt[3][1]-(PO_1_321+RL_1_335);
//
  h_4 = -(PO_1_121-RL_2_136+RL_3_137);
  h_5 = RL_2_236+s->dpt[2][2]-(PO_1_221+RL_3_237);
  h_6 = RL_2_336+s->dpt[3][2]-(PO_1_321+RL_3_337);

// = = Block_0_3_0_0_0_0 = = 
 
// Symbolic Outputs  

  h[1] = h_1;
  h[2] = h_2;
  h[3] = h_3;
  h[4] = h_4;
  h[5] = h_5;
  h[6] = h_6;
  Jac[1][1] = -(1.0);
  Jac[1][5] = -JT_1_135_5;
  Jac[1][6] = -JT_1_135_6;
  Jac[1][19] = -JT_1_135_19;
  Jac[1][20] = -JT_1_135_20;
  Jac[1][21] = -JT_1_135_21;
  Jac[1][31] = JT_0_134_31;
  Jac[2][2] = -(1.0);
  Jac[2][4] = -JT_1_235_4;
  Jac[2][5] = -JT_1_235_5;
  Jac[2][6] = -JT_1_235_6;
  Jac[2][19] = -JT_1_235_19;
  Jac[2][20] = -JT_1_235_20;
  Jac[2][21] = -JT_1_235_21;
  Jac[2][30] = -RL_0_334;
  Jac[2][31] = JT_0_234_31;
  Jac[3][3] = -(1.0);
  Jac[3][4] = -JT_1_335_4;
  Jac[3][5] = -JT_1_335_5;
  Jac[3][6] = -JT_1_335_6;
  Jac[3][19] = -JT_1_335_19;
  Jac[3][20] = -JT_1_335_20;
  Jac[3][21] = -JT_1_335_21;
  Jac[3][30] = RL_0_234;
  Jac[3][31] = JT_0_334_31;
  Jac[4][1] = -(1.0);
  Jac[4][5] = -JT_3_137_5;
  Jac[4][6] = -JT_3_137_6;
  Jac[4][19] = -JT_3_137_19;
  Jac[4][20] = -JT_3_137_20;
  Jac[4][21] = -JT_3_137_21;
  Jac[4][33] = JT_2_136_33;
  Jac[5][2] = -(1.0);
  Jac[5][4] = -JT_3_237_4;
  Jac[5][5] = -JT_3_237_5;
  Jac[5][6] = -JT_3_237_6;
  Jac[5][19] = -JT_3_237_19;
  Jac[5][20] = -JT_3_237_20;
  Jac[5][21] = -JT_3_237_21;
  Jac[5][32] = -RL_2_336;
  Jac[5][33] = JT_2_236_33;
  Jac[6][3] = -(1.0);
  Jac[6][4] = -JT_3_337_4;
  Jac[6][5] = -JT_3_337_5;
  Jac[6][6] = -JT_3_337_6;
  Jac[6][19] = -JT_3_337_19;
  Jac[6][20] = -JT_3_337_20;
  Jac[6][21] = -JT_3_337_21;
  Jac[6][32] = RL_2_236;
  Jac[6][33] = JT_2_336_33;

// ====== END Task 0 ====== 


}
 

