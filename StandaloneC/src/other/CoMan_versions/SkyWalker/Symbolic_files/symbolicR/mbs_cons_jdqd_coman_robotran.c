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
//	==> Generation Date : Tue Nov  4 14:52:29 2014
//
//	==> Project name : coman_robotran
//	==> using XML input file 
//
//	==> Number of joints : 33
//
//	==> Function : F18 : Constraints Quadratic Velocity Terms (Jdqd)
//	==> Flops complexity : 453
//
//	==> Generation Time :  0.010 seconds
//	==> Post-Processing :  0.010 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "MBSdataStructR7.h"
#include "MBSfunR7.h"
 
void cons_jdqd(double *Jdqd,
MBSdataStruct *s, double tsim)

// double Jdqd[6];
{ 
 
#include "mbs_cons_jdqd_coman_robotran.h" 
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
  RL_1_135 = RO_1_121*s->dpt[1][42]+RO_1_421*s->dpt[2][42]+RO_1_720*s->dpt[3][42];
  RL_1_235 = RO_1_221*s->dpt[1][42]+RO_1_521*s->dpt[2][42]+RO_1_820*s->dpt[3][42];
  RL_1_335 = RO_1_321*s->dpt[1][42]+RO_1_621*s->dpt[2][42]+RO_1_920*s->dpt[3][42];
//
  RL_3_137 = RO_1_121*s->dpt[1][41]+RO_1_421*s->dpt[2][41]+RO_1_720*s->dpt[3][41];
  RL_3_237 = RO_1_221*s->dpt[1][41]+RO_1_521*s->dpt[2][41]+RO_1_820*s->dpt[3][41];
  RL_3_337 = RO_1_321*s->dpt[1][41]+RO_1_621*s->dpt[2][41]+RO_1_920*s->dpt[3][41];

// = = Block_0_1_0_0_0_7 = = 
 
// Constraints and Constraints Jacobian 

//
  RL_0_134 = s->dpt[3][59]*S31;
  RL_0_234 = -s->dpt[3][59]*S30*C31;
  RL_0_334 = s->dpt[3][59]*C30*C31;

// = = Block_0_1_0_0_0_8 = = 
 
// Constraints and Constraints Jacobian 

//
  RL_2_136 = s->dpt[3][60]*S33;
  RL_2_236 = -s->dpt[3][60]*S32*C33;
  RL_2_336 = s->dpt[3][60]*C32*C33;

// = = Block_0_2_0_0_0_0 = = 
 
// Constraints Quadratic Terms 

//
  OM_0_231 = qd[31]*C30;
  OM_0_331 = qd[31]*S30;
  Ompqp_0_231 = -qd[30]*qd[31]*S30;
  Ompqp_0_331 = qd[30]*qd[31]*C30;
  OR_0_134 = qd[31]*s->dpt[3][59]*C31;
  OR_0_234 = OM_0_331*RL_0_134-RL_0_334*qd[30];
  OR_0_334 = -(OM_0_231*RL_0_134-RL_0_234*qd[30]);
//
  OM_1_25 = qd[5]*C4;
  OM_1_35 = qd[5]*S4;
  OM_1_16 = qd[4]+qd[6]*S5;
  OM_1_26 = OM_1_25+RO_1_85*qd[6];
  OM_1_36 = OM_1_35+RO_1_95*qd[6];
  Ompqp_1_16 = qd[5]*qd[6]*C5;
  Ompqp_1_26 = -(qd[4]*qd[5]*S4-qd[6]*(OM_1_35*S5-RO_1_95*qd[4]));
  Ompqp_1_36 = qd[4]*qd[5]*C4-qd[6]*(OM_1_25*S5-RO_1_85*qd[4]);
  OM_1_119 = OM_1_16+RO_1_16*qd[19];
  OM_1_219 = OM_1_26+RO_1_26*qd[19];
  OM_1_319 = OM_1_36+RO_1_36*qd[19];
  OR_1_119 = OM_1_26*RL_1_319-OM_1_36*RL_1_219;
  OR_1_219 = -(OM_1_16*RL_1_319-OM_1_36*RL_1_119);
  OR_1_319 = OM_1_16*RL_1_219-OM_1_26*RL_1_119;
  OM_1_120 = OM_1_119+RO_1_419*qd[20];
  OM_1_220 = OM_1_219+RO_1_519*qd[20];
  OM_1_320 = OM_1_319+RO_1_619*qd[20];
  Ompqp_1_120 = Ompqp_1_16+qd[19]*(OM_1_26*RO_1_36-OM_1_36*RO_1_26)+qd[20]*(OM_1_219*RO_1_619-OM_1_319*RO_1_519);
  Ompqp_1_220 = Ompqp_1_26-qd[19]*(OM_1_16*RO_1_36-OM_1_36*RO_1_16)-qd[20]*(OM_1_119*RO_1_619-OM_1_319*RO_1_419);
  Ompqp_1_320 = Ompqp_1_36+qd[19]*(OM_1_16*RO_1_26-OM_1_26*RO_1_16)+qd[20]*(OM_1_119*RO_1_519-OM_1_219*RO_1_419);
  OM_1_121 = OM_1_120+RO_1_720*qd[21];
  OM_1_221 = OM_1_220+RO_1_820*qd[21];
  OM_1_321 = OM_1_320+RO_1_920*qd[21];
  OR_1_121 = OM_1_220*RL_1_321-OM_1_320*RL_1_221;
  OR_1_221 = -(OM_1_120*RL_1_321-OM_1_320*RL_1_121);
  OR_1_321 = OM_1_120*RL_1_221-OM_1_220*RL_1_121;
  Ompqp_1_121 = Ompqp_1_120+qd[21]*(OM_1_220*RO_1_920-OM_1_320*RO_1_820);
  Ompqp_1_221 = Ompqp_1_220-qd[21]*(OM_1_120*RO_1_920-OM_1_320*RO_1_720);
  Ompqp_1_321 = Ompqp_1_320+qd[21]*(OM_1_120*RO_1_820-OM_1_220*RO_1_720);
  Apqp_1_121 = OM_1_220*OR_1_321+OM_1_26*OR_1_319-OM_1_320*OR_1_221-OM_1_36*OR_1_219+Ompqp_1_220*RL_1_321+Ompqp_1_26*
 RL_1_319-Ompqp_1_320*RL_1_221-Ompqp_1_36*RL_1_219;
  Apqp_1_221 = -(OM_1_120*OR_1_321+OM_1_16*OR_1_319-OM_1_320*OR_1_121-OM_1_36*OR_1_119+Ompqp_1_120*RL_1_321+Ompqp_1_16*
 RL_1_319-Ompqp_1_320*RL_1_121-Ompqp_1_36*RL_1_119);
  Apqp_1_321 = OM_1_120*OR_1_221+OM_1_16*OR_1_219-OM_1_220*OR_1_121-OM_1_26*OR_1_119+Ompqp_1_120*RL_1_221+Ompqp_1_16*
 RL_1_219-Ompqp_1_220*RL_1_121-Ompqp_1_26*RL_1_119;
  OR_1_135 = OM_1_221*RL_1_335-OM_1_321*RL_1_235;
  OR_1_235 = -(OM_1_121*RL_1_335-OM_1_321*RL_1_135);
  OR_1_335 = OM_1_121*RL_1_235-OM_1_221*RL_1_135;
//
  OM_2_233 = qd[33]*C32;
  OM_2_333 = qd[33]*S32;
  Ompqp_2_233 = -qd[32]*qd[33]*S32;
  Ompqp_2_333 = qd[32]*qd[33]*C32;
  OR_2_136 = qd[33]*s->dpt[3][60]*C33;
  OR_2_236 = OM_2_333*RL_2_136-RL_2_336*qd[32];
  OR_2_336 = -(OM_2_233*RL_2_136-RL_2_236*qd[32]);
//
  OR_3_137 = OM_1_221*RL_3_337-OM_1_321*RL_3_237;
  OR_3_237 = -(OM_1_121*RL_3_337-OM_1_321*RL_3_137);
  OR_3_337 = OM_1_121*RL_3_237-OM_1_221*RL_3_137;

// = = Block_0_2_0_0_0_1 = = 
 
// Constraints Quadratic Terms 

//
  jdqd1 = OM_0_231*OR_0_334-OM_0_331*OR_0_234+Ompqp_0_231*RL_0_334-Ompqp_0_331*RL_0_234-(Apqp_1_121+OM_1_221*OR_1_335-
 OM_1_321*OR_1_235+Ompqp_1_221*RL_1_335-Ompqp_1_321*RL_1_235);
  jdqd2 = OM_0_331*OR_0_134-OR_0_334*qd[30]+Ompqp_0_331*RL_0_134-(Apqp_1_221-OM_1_121*OR_1_335+OM_1_321*OR_1_135-
 Ompqp_1_121*RL_1_335+Ompqp_1_321*RL_1_135);
  jdqd3 = -(Apqp_1_321+OM_0_231*OR_0_134+OM_1_121*OR_1_235-OM_1_221*OR_1_135-OR_0_234*qd[30]+Ompqp_0_231*RL_0_134+
 Ompqp_1_121*RL_1_235-Ompqp_1_221*RL_1_135);
//
  jdqd4 = OM_2_233*OR_2_336-OM_2_333*OR_2_236+Ompqp_2_233*RL_2_336-Ompqp_2_333*RL_2_236-(Apqp_1_121+OM_1_221*OR_3_337-
 OM_1_321*OR_3_237+Ompqp_1_221*RL_3_337-Ompqp_1_321*RL_3_237);
  jdqd5 = OM_2_333*OR_2_136-OR_2_336*qd[32]+Ompqp_2_333*RL_2_136-(Apqp_1_221-OM_1_121*OR_3_337+OM_1_321*OR_3_137-
 Ompqp_1_121*RL_3_337+Ompqp_1_321*RL_3_137);
  jdqd6 = -(Apqp_1_321+OM_1_121*OR_3_237-OM_1_221*OR_3_137+OM_2_233*OR_2_136-OR_2_236*qd[32]+Ompqp_1_121*RL_3_237-
 Ompqp_1_221*RL_3_137+Ompqp_2_233*RL_2_136);

// = = Block_0_3_0_0_0_0 = = 
 
// Symbolic Outputs  

  Jdqd[1] = jdqd1;
  Jdqd[2] = jdqd2;
  Jdqd[3] = jdqd3;
  Jdqd[4] = jdqd4;
  Jdqd[5] = jdqd5;
  Jdqd[6] = jdqd6;

// ====== END Task 0 ====== 


}
 

