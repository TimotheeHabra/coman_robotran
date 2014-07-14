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
//	==> Generation Date : Mon Jul 14 11:56:57 2014
//
//	==> Project name : coman_robotran
//	==> using XML input file 
//
//	==> Number of joints : 35
//
//	==> Function : F 1 : Direct Dynamics (Semi-Explicit formulation) : RNEA
//	==> Flops complexity : 21418
//
//	==> Generation Time :  0.240 seconds
//	==> Post-Processing :  0.280 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "MBSdataStructR7.h"
#include "MBSfunR7.h"
 
void dirdyna(double **M,double *c,
MBSdataStruct *s, double tsim)

// double M[35][35];
// double c[35];
{ 
 
#include "mbs_dirdyna_coman_robotran.h" 
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

// = = Block_0_1_0_0_0_1 = = 
 
// Forward Kinematics 

  AlF24 = -s->g[3]*S4;
  AlF34 = -s->g[3]*C4;
  OM15 = qd[4]*C5;
  OpF15 = -qd[4]*qd[5]*S5;
  OpF35 = qd[4]*qd[5]*C5;
  AlF15 = -AlF34*S5;
  AlF35 = AlF34*C5;
  AlM15_2 = S4*S5;
  AlM35_2 = -S4*C5;
  AlM15_3 = -C4*S5;
  AlM35_3 = C4*C5;
  OM16 = qd[5]*S6+OM15*C6;
  OM26 = qd[5]*C6-OM15*S6;
  OM36 = qd[6]+qd[4]*S5;
  OpF16 = -(qd[6]*OM15*S6-C6*(OpF15+qd[5]*qd[6]));
  OpF26 = -(qd[6]*OM15*C6+S6*(OpF15+qd[5]*qd[6]));
  BS16 = -(OM26*OM26+OM36*OM36);
  BS26 = OM16*OM26;
  BS36 = OM16*OM36;
  BS56 = -(OM16*OM16+OM36*OM36);
  BS66 = OM26*OM36;
  BS96 = -(OM16*OM16+OM26*OM26);
  BeF26 = BS26-OpF35;
  BeF36 = BS36+OpF26;
  BeF46 = BS26+OpF35;
  BeF66 = BS66-OpF16;
  BeF76 = BS36-OpF26;
  BeF86 = BS66+OpF16;
  AlF16 = AlF15*C6+AlF24*S6;
  AlF26 = -(AlF15*S6-AlF24*C6);
  AlM16_1 = C5*C6;
  AlM26_1 = -C5*S6;
  AlM16_2 = AlM15_2*C6+C4*S6;
  AlM26_2 = -(AlM15_2*S6-C4*C6);
  AlM16_3 = AlM15_3*C6+S4*S6;
  AlM26_3 = -(AlM15_3*S6-S4*C6);
  OpM16_4 = C5*C6;
  OpM26_4 = -C5*S6;

// = = Block_0_1_0_1_0_2 = = 
 
// Forward Kinematics 

  OM17 = OM16*C7-OM36*S7;
  OM27 = qd[7]+OM26;
  OM37 = OM16*S7+OM36*C7;
  OpF17 = C7*(OpF16-qd[7]*OM36)-S7*(OpF35+qd[7]*OM16);
  OpF37 = C7*(OpF35+qd[7]*OM16)+S7*(OpF16-qd[7]*OM36);
  BS27 = OM17*OM27;
  BS37 = OM17*OM37;
  BS57 = -(OM17*OM17+OM37*OM37);
  BS67 = OM27*OM37;
  BeF27 = BS27-OpF37;
  BeF87 = BS67+OpF17;
  AlF17 = C7*(AlF16+BeF26*s->dpt[2][1])-S7*(AlF35+BeF86*s->dpt[2][1]);
  AlF27 = AlF26+BS56*s->dpt[2][1];
  AlF37 = C7*(AlF35+BeF86*s->dpt[2][1])+S7*(AlF16+BeF26*s->dpt[2][1]);
  AlM17_1 = AlM16_1*C7-S5*S7;
  AlM37_1 = AlM16_1*S7+S5*C7;
  AlM17_2 = AlM16_2*C7-AlM35_2*S7;
  AlM37_2 = AlM16_2*S7+AlM35_2*C7;
  AlM17_3 = AlM16_3*C7-AlM35_3*S7;
  AlM37_3 = AlM16_3*S7+AlM35_3*C7;
  OpM17_4 = OpM16_4*C7-S5*S7;
  OpM37_4 = OpM16_4*S7+S5*C7;
  AlM17_4 = -s->dpt[2][1]*(OpM16_4*S7+S5*C7);
  AlM37_4 = s->dpt[2][1]*(OpM16_4*C7-S5*S7);
  OpM17_5 = S6*C7;
  OpM37_5 = S6*S7;
  AlM17_5 = -s->dpt[2][1]*S6*S7;
  AlM37_5 = s->dpt[2][1]*S6*C7;
  AlM17_6 = -s->dpt[2][1]*C7;
  AlM37_6 = -s->dpt[2][1]*S7;
  OM18 = qd[8]+OM17;
  OM28 = OM27*C8+OM37*S8;
  OM38 = -(OM27*S8-OM37*C8);
  OpF28 = C8*(OpF26+qd[8]*OM37)+S8*(OpF37-qd[8]*OM27);
  OpF38 = C8*(OpF37-qd[8]*OM27)-S8*(OpF26+qd[8]*OM37);
  BS28 = OM18*OM28;
  BS38 = OM18*OM38;
  BS68 = OM28*OM38;
  BS98 = -(OM18*OM18+OM28*OM28);
  BeF38 = BS38+OpF28;
  BeF68 = BS68-OpF17;
  AlF18 = AlF17+BeF27*s->dpt[2][6];
  AlF28 = C8*(AlF27+BS57*s->dpt[2][6])+S8*(AlF37+BeF87*s->dpt[2][6]);
  AlF38 = C8*(AlF37+BeF87*s->dpt[2][6])-S8*(AlF27+BS57*s->dpt[2][6]);
  AlM28_1 = AlM26_1*C8+AlM37_1*S8;
  AlM38_1 = -(AlM26_1*S8-AlM37_1*C8);
  AlM28_2 = AlM26_2*C8+AlM37_2*S8;
  AlM38_2 = -(AlM26_2*S8-AlM37_2*C8);
  AlM28_3 = AlM26_3*C8+AlM37_3*S8;
  AlM38_3 = -(AlM26_3*S8-AlM37_3*C8);
  OpM28_4 = OpM26_4*C8+OpM37_4*S8;
  OpM38_4 = -(OpM26_4*S8-OpM37_4*C8);
  AlM18_4 = AlM17_4-OpM37_4*s->dpt[2][6];
  AlM28_4 = S8*(AlM37_4+OpM17_4*s->dpt[2][6]);
  AlM38_4 = C8*(AlM37_4+OpM17_4*s->dpt[2][6]);
  OpM28_5 = OpM37_5*S8+C6*C8;
  OpM38_5 = OpM37_5*C8-C6*S8;
  AlM18_5 = AlM17_5-OpM37_5*s->dpt[2][6];
  AlM28_5 = S8*(AlM37_5+OpM17_5*s->dpt[2][6]);
  AlM38_5 = C8*(AlM37_5+OpM17_5*s->dpt[2][6]);
  OpM28_6 = C7*S8;
  OpM38_6 = C7*C8;
  AlM18_6 = AlM17_6-s->dpt[2][6]*C7;
  AlM28_6 = S8*(AlM37_6-s->dpt[2][6]*S7);
  AlM38_6 = C8*(AlM37_6-s->dpt[2][6]*S7);
  OM19 = OM18*C9+OM28*S9;
  OM29 = -(OM18*S9-OM28*C9);
  OM39 = qd[9]+OM38;
  OpF19 = C9*(OpF17+qd[9]*OM28)+S9*(OpF28-qd[9]*OM18);
  OpF29 = C9*(OpF28-qd[9]*OM18)-S9*(OpF17+qd[9]*OM28);
  BS29 = OM19*OM29;
  BS39 = OM19*OM39;
  BS69 = OM29*OM39;
  BS99 = -(OM19*OM19+OM29*OM29);
  BeF39 = BS39+OpF29;
  BeF69 = BS69-OpF19;
  AlF19 = C9*(AlF18+BeF38*s->dpt[3][8])+S9*(AlF28+BeF68*s->dpt[3][8]);
  AlF29 = C9*(AlF28+BeF68*s->dpt[3][8])-S9*(AlF18+BeF38*s->dpt[3][8]);
  AlF39 = AlF38+BS98*s->dpt[3][8];
  AlM19_1 = AlM17_1*C9+AlM28_1*S9;
  AlM29_1 = -(AlM17_1*S9-AlM28_1*C9);
  AlM19_2 = AlM17_2*C9+AlM28_2*S9;
  AlM29_2 = -(AlM17_2*S9-AlM28_2*C9);
  AlM19_3 = AlM17_3*C9+AlM28_3*S9;
  AlM29_3 = -(AlM17_3*S9-AlM28_3*C9);
  OpM19_4 = OpM17_4*C9+OpM28_4*S9;
  OpM29_4 = -(OpM17_4*S9-OpM28_4*C9);
  AlM19_4 = C9*(AlM18_4+OpM28_4*s->dpt[3][8])+S9*(AlM28_4-OpM17_4*s->dpt[3][8]);
  AlM29_4 = C9*(AlM28_4-OpM17_4*s->dpt[3][8])-S9*(AlM18_4+OpM28_4*s->dpt[3][8]);
  OpM19_5 = OpM17_5*C9+OpM28_5*S9;
  OpM29_5 = -(OpM17_5*S9-OpM28_5*C9);
  AlM19_5 = C9*(AlM18_5+OpM28_5*s->dpt[3][8])+S9*(AlM28_5-OpM17_5*s->dpt[3][8]);
  AlM29_5 = C9*(AlM28_5-OpM17_5*s->dpt[3][8])-S9*(AlM18_5+OpM28_5*s->dpt[3][8]);
  OpM19_6 = OpM28_6*S9-S7*C9;
  OpM29_6 = OpM28_6*C9+S7*S9;
  AlM19_6 = C9*(AlM18_6+OpM28_6*s->dpt[3][8])+S9*(AlM28_6+s->dpt[3][8]*S7);
  AlM29_6 = C9*(AlM28_6+s->dpt[3][8]*S7)-S9*(AlM18_6+OpM28_6*s->dpt[3][8]);
  OpM19_7 = C8*S9;
  OpM29_7 = C8*C9;
  AlM19_7 = s->dpt[3][8]*C8*C9;
  AlM29_7 = -s->dpt[3][8]*C8*S9;
  AlM19_8 = -s->dpt[3][8]*S9;
  AlM29_8 = -s->dpt[3][8]*C9;
  OM110 = OM19*C10-OM39*S10;
  OM210 = qd[10]+OM29;
  OM310 = OM19*S10+OM39*C10;
  OpF110 = C10*(OpF19-qd[10]*OM39)-S10*(OpF38+qd[10]*OM19);
  OpF310 = C10*(OpF38+qd[10]*OM19)+S10*(OpF19-qd[10]*OM39);
  BS210 = OM110*OM210;
  BS310 = OM110*OM310;
  BS610 = OM210*OM310;
  BS910 = -(OM110*OM110+OM210*OM210);
  BeF310 = BS310+OpF29;
  BeF610 = BS610-OpF110;
  AlF110 = C10*(AlF19+BeF39*s->dpt[3][10])-S10*(AlF39+BS99*s->dpt[3][10]);
  AlF210 = AlF29+BeF69*s->dpt[3][10];
  AlF310 = C10*(AlF39+BS99*s->dpt[3][10])+S10*(AlF19+BeF39*s->dpt[3][10]);
  AlM110_1 = AlM19_1*C10-AlM38_1*S10;
  AlM310_1 = AlM19_1*S10+AlM38_1*C10;
  AlM110_2 = AlM19_2*C10-AlM38_2*S10;
  AlM310_2 = AlM19_2*S10+AlM38_2*C10;
  AlM110_3 = AlM19_3*C10-AlM38_3*S10;
  AlM310_3 = AlM19_3*S10+AlM38_3*C10;
  OpM110_4 = OpM19_4*C10-OpM38_4*S10;
  OpM310_4 = OpM19_4*S10+OpM38_4*C10;
  AlM110_4 = -(AlM38_4*S10-C10*(AlM19_4+OpM29_4*s->dpt[3][10]));
  AlM210_4 = AlM29_4-OpM19_4*s->dpt[3][10];
  AlM310_4 = AlM38_4*C10+S10*(AlM19_4+OpM29_4*s->dpt[3][10]);
  OpM110_5 = OpM19_5*C10-OpM38_5*S10;
  OpM310_5 = OpM19_5*S10+OpM38_5*C10;
  AlM110_5 = -(AlM38_5*S10-C10*(AlM19_5+OpM29_5*s->dpt[3][10]));
  AlM210_5 = AlM29_5-OpM19_5*s->dpt[3][10];
  AlM310_5 = AlM38_5*C10+S10*(AlM19_5+OpM29_5*s->dpt[3][10]);
  OpM110_6 = OpM19_6*C10-OpM38_6*S10;
  OpM310_6 = OpM19_6*S10+OpM38_6*C10;
  AlM110_6 = -(AlM38_6*S10-C10*(AlM19_6+OpM29_6*s->dpt[3][10]));
  AlM210_6 = AlM29_6-OpM19_6*s->dpt[3][10];
  AlM310_6 = AlM38_6*C10+S10*(AlM19_6+OpM29_6*s->dpt[3][10]);
  OpM110_7 = OpM19_7*C10+S10*S8;
  OpM310_7 = OpM19_7*S10-C10*S8;
  AlM110_7 = C10*(AlM19_7+OpM29_7*s->dpt[3][10]);
  AlM210_7 = AlM29_7-OpM19_7*s->dpt[3][10];
  AlM310_7 = S10*(AlM19_7+OpM29_7*s->dpt[3][10]);
  OpM110_8 = C10*C9;
  OpM310_8 = S10*C9;
  AlM110_8 = C10*(AlM19_8-s->dpt[3][10]*S9);
  AlM210_8 = AlM29_8-s->dpt[3][10]*C9;
  AlM310_8 = S10*(AlM19_8-s->dpt[3][10]*S9);
  OM111 = qd[11]+OM110;
  OM211 = OM210*C11+OM310*S11;
  OM311 = -(OM210*S11-OM310*C11);
  OpF211 = C11*(OpF29+qd[11]*OM310)+S11*(OpF310-qd[11]*OM210);
  OpF311 = C11*(OpF310-qd[11]*OM210)-S11*(OpF29+qd[11]*OM310);
  BS211 = OM111*OM211;
  BS311 = OM111*OM311;
  BS611 = OM211*OM311;
  AlF111 = AlF110+BeF310*s->dpt[3][12];
  AlF211 = C11*(AlF210+BeF610*s->dpt[3][12])+S11*(AlF310+BS910*s->dpt[3][12]);
  AlF311 = C11*(AlF310+BS910*s->dpt[3][12])-S11*(AlF210+BeF610*s->dpt[3][12]);
  AlM211_1 = AlM29_1*C11+AlM310_1*S11;
  AlM311_1 = -(AlM29_1*S11-AlM310_1*C11);
  AlM211_2 = AlM29_2*C11+AlM310_2*S11;
  AlM311_2 = -(AlM29_2*S11-AlM310_2*C11);
  AlM211_3 = AlM29_3*C11+AlM310_3*S11;
  AlM311_3 = -(AlM29_3*S11-AlM310_3*C11);
  OpM211_4 = OpM29_4*C11+OpM310_4*S11;
  OpM311_4 = -(OpM29_4*S11-OpM310_4*C11);
  AlM111_4 = AlM110_4+OpM29_4*s->dpt[3][12];
  AlM211_4 = AlM310_4*S11+C11*(AlM210_4-OpM110_4*s->dpt[3][12]);
  AlM311_4 = AlM310_4*C11-S11*(AlM210_4-OpM110_4*s->dpt[3][12]);
  OpM211_5 = OpM29_5*C11+OpM310_5*S11;
  OpM311_5 = -(OpM29_5*S11-OpM310_5*C11);
  AlM111_5 = AlM110_5+OpM29_5*s->dpt[3][12];
  AlM211_5 = AlM310_5*S11+C11*(AlM210_5-OpM110_5*s->dpt[3][12]);
  AlM311_5 = AlM310_5*C11-S11*(AlM210_5-OpM110_5*s->dpt[3][12]);
  OpM211_6 = OpM29_6*C11+OpM310_6*S11;
  OpM311_6 = -(OpM29_6*S11-OpM310_6*C11);
  AlM111_6 = AlM110_6+OpM29_6*s->dpt[3][12];
  AlM211_6 = AlM310_6*S11+C11*(AlM210_6-OpM110_6*s->dpt[3][12]);
  AlM311_6 = AlM310_6*C11-S11*(AlM210_6-OpM110_6*s->dpt[3][12]);
  OpM211_7 = OpM29_7*C11+OpM310_7*S11;
  OpM311_7 = -(OpM29_7*S11-OpM310_7*C11);
  AlM111_7 = AlM110_7+OpM29_7*s->dpt[3][12];
  AlM211_7 = AlM310_7*S11+C11*(AlM210_7-OpM110_7*s->dpt[3][12]);
  AlM311_7 = AlM310_7*C11-S11*(AlM210_7-OpM110_7*s->dpt[3][12]);
  OpM211_8 = OpM310_8*S11-C11*S9;
  OpM311_8 = OpM310_8*C11+S11*S9;
  AlM111_8 = AlM110_8-s->dpt[3][12]*S9;
  AlM211_8 = AlM310_8*S11+C11*(AlM210_8-OpM110_8*s->dpt[3][12]);
  AlM311_8 = AlM310_8*C11-S11*(AlM210_8-OpM110_8*s->dpt[3][12]);
  OpM211_9 = C10*S11;
  OpM311_9 = C10*C11;
  AlM211_9 = s->dpt[3][12]*S10*C11;
  AlM311_9 = -s->dpt[3][12]*S10*S11;
  OM112 = OM111*C12-OM311*S12;
  OM212 = qd[12]+OM211;
  OM312 = OM111*S12+OM311*C12;
  OpF112 = C12*(OpF110-qd[12]*OM311)-S12*(OpF311+qd[12]*OM111);
  OpF312 = C12*(OpF311+qd[12]*OM111)+S12*(OpF110-qd[12]*OM311);
  BS212 = OM112*OM212;
  BS312 = OM112*OM312;
  BS612 = OM212*OM312;
  OpM112_4 = OpM110_4*C12-OpM311_4*S12;
  OpM312_4 = OpM110_4*S12+OpM311_4*C12;
  OpM112_5 = OpM110_5*C12-OpM311_5*S12;
  OpM312_5 = OpM110_5*S12+OpM311_5*C12;
  OpM112_6 = OpM110_6*C12-OpM311_6*S12;
  OpM312_6 = OpM110_6*S12+OpM311_6*C12;
  OpM112_7 = OpM110_7*C12-OpM311_7*S12;
  OpM312_7 = OpM110_7*S12+OpM311_7*C12;
  OpM112_8 = OpM110_8*C12-OpM311_8*S12;
  OpM312_8 = OpM110_8*S12+OpM311_8*C12;
  OpM112_9 = -(OpM311_9*S12+S10*C12);
  OpM312_9 = OpM311_9*C12-S10*S12;
  OpM112_10 = S11*S12;
  OpM312_10 = -S11*C12;

// = = Block_0_1_0_1_0_3 = = 
 
// Forward Kinematics 

  OM113 = OM16*C13-OM36*S13;
  OM213 = qd[13]+OM26;
  OM313 = OM16*S13+OM36*C13;
  OpF113 = C13*(OpF16-qd[13]*OM36)-S13*(OpF35+qd[13]*OM16);
  OpF313 = C13*(OpF35+qd[13]*OM16)+S13*(OpF16-qd[13]*OM36);
  BS213 = OM113*OM213;
  BS313 = OM113*OM313;
  BS513 = -(OM113*OM113+OM313*OM313);
  BS613 = OM213*OM313;
  BeF213 = BS213-OpF313;
  BeF813 = BS613+OpF113;
  AlF113 = C13*(AlF16+BeF26*s->dpt[2][2])-S13*(AlF35+BeF86*s->dpt[2][2]);
  AlF213 = AlF26+BS56*s->dpt[2][2];
  AlF313 = C13*(AlF35+BeF86*s->dpt[2][2])+S13*(AlF16+BeF26*s->dpt[2][2]);
  AlM113_1 = AlM16_1*C13-S13*S5;
  AlM313_1 = AlM16_1*S13+C13*S5;
  AlM113_2 = AlM16_2*C13-AlM35_2*S13;
  AlM313_2 = AlM16_2*S13+AlM35_2*C13;
  AlM113_3 = AlM16_3*C13-AlM35_3*S13;
  AlM313_3 = AlM16_3*S13+AlM35_3*C13;
  OpM113_4 = OpM16_4*C13-S13*S5;
  OpM313_4 = OpM16_4*S13+C13*S5;
  AlM113_4 = -s->dpt[2][2]*(OpM16_4*S13+C13*S5);
  AlM313_4 = s->dpt[2][2]*(OpM16_4*C13-S13*S5);
  OpM113_5 = C13*S6;
  OpM313_5 = S13*S6;
  AlM113_5 = -s->dpt[2][2]*S13*S6;
  AlM313_5 = s->dpt[2][2]*C13*S6;
  AlM113_6 = -s->dpt[2][2]*C13;
  AlM313_6 = -s->dpt[2][2]*S13;
  OM114 = qd[14]+OM113;
  OM214 = OM213*C14+OM313*S14;
  OM314 = -(OM213*S14-OM313*C14);
  OpF214 = C14*(OpF26+qd[14]*OM313)+S14*(OpF313-qd[14]*OM213);
  OpF314 = C14*(OpF313-qd[14]*OM213)-S14*(OpF26+qd[14]*OM313);
  BS214 = OM114*OM214;
  BS314 = OM114*OM314;
  BS614 = OM214*OM314;
  BS914 = -(OM114*OM114+OM214*OM214);
  BeF314 = BS314+OpF214;
  BeF614 = BS614-OpF113;
  AlF114 = AlF113+BeF213*s->dpt[2][18];
  AlF214 = C14*(AlF213+BS513*s->dpt[2][18])+S14*(AlF313+BeF813*s->dpt[2][18]);
  AlF314 = C14*(AlF313+BeF813*s->dpt[2][18])-S14*(AlF213+BS513*s->dpt[2][18]);
  AlM214_1 = AlM26_1*C14+AlM313_1*S14;
  AlM314_1 = -(AlM26_1*S14-AlM313_1*C14);
  AlM214_2 = AlM26_2*C14+AlM313_2*S14;
  AlM314_2 = -(AlM26_2*S14-AlM313_2*C14);
  AlM214_3 = AlM26_3*C14+AlM313_3*S14;
  AlM314_3 = -(AlM26_3*S14-AlM313_3*C14);
  OpM214_4 = OpM26_4*C14+OpM313_4*S14;
  OpM314_4 = -(OpM26_4*S14-OpM313_4*C14);
  AlM114_4 = AlM113_4-OpM313_4*s->dpt[2][18];
  AlM214_4 = S14*(AlM313_4+OpM113_4*s->dpt[2][18]);
  AlM314_4 = C14*(AlM313_4+OpM113_4*s->dpt[2][18]);
  OpM214_5 = OpM313_5*S14+C14*C6;
  OpM314_5 = OpM313_5*C14-S14*C6;
  AlM114_5 = AlM113_5-OpM313_5*s->dpt[2][18];
  AlM214_5 = S14*(AlM313_5+OpM113_5*s->dpt[2][18]);
  AlM314_5 = C14*(AlM313_5+OpM113_5*s->dpt[2][18]);
  OpM214_6 = C13*S14;
  OpM314_6 = C13*C14;
  AlM114_6 = AlM113_6-s->dpt[2][18]*C13;
  AlM214_6 = S14*(AlM313_6-s->dpt[2][18]*S13);
  AlM314_6 = C14*(AlM313_6-s->dpt[2][18]*S13);
  OM115 = OM114*C15+OM214*S15;
  OM215 = -(OM114*S15-OM214*C15);
  OM315 = qd[15]+OM314;
  OpF115 = C15*(OpF113+qd[15]*OM214)+S15*(OpF214-qd[15]*OM114);
  OpF215 = C15*(OpF214-qd[15]*OM114)-S15*(OpF113+qd[15]*OM214);
  BS215 = OM115*OM215;
  BS315 = OM115*OM315;
  BS615 = OM215*OM315;
  BS915 = -(OM115*OM115+OM215*OM215);
  BeF315 = BS315+OpF215;
  BeF615 = BS615-OpF115;
  AlF115 = C15*(AlF114+BeF314*s->dpt[3][20])+S15*(AlF214+BeF614*s->dpt[3][20]);
  AlF215 = C15*(AlF214+BeF614*s->dpt[3][20])-S15*(AlF114+BeF314*s->dpt[3][20]);
  AlF315 = AlF314+BS914*s->dpt[3][20];
  AlM115_1 = AlM113_1*C15+AlM214_1*S15;
  AlM215_1 = -(AlM113_1*S15-AlM214_1*C15);
  AlM115_2 = AlM113_2*C15+AlM214_2*S15;
  AlM215_2 = -(AlM113_2*S15-AlM214_2*C15);
  AlM115_3 = AlM113_3*C15+AlM214_3*S15;
  AlM215_3 = -(AlM113_3*S15-AlM214_3*C15);
  OpM115_4 = OpM113_4*C15+OpM214_4*S15;
  OpM215_4 = -(OpM113_4*S15-OpM214_4*C15);
  AlM115_4 = C15*(AlM114_4+OpM214_4*s->dpt[3][20])+S15*(AlM214_4-OpM113_4*s->dpt[3][20]);
  AlM215_4 = C15*(AlM214_4-OpM113_4*s->dpt[3][20])-S15*(AlM114_4+OpM214_4*s->dpt[3][20]);
  OpM115_5 = OpM113_5*C15+OpM214_5*S15;
  OpM215_5 = -(OpM113_5*S15-OpM214_5*C15);
  AlM115_5 = C15*(AlM114_5+OpM214_5*s->dpt[3][20])+S15*(AlM214_5-OpM113_5*s->dpt[3][20]);
  AlM215_5 = C15*(AlM214_5-OpM113_5*s->dpt[3][20])-S15*(AlM114_5+OpM214_5*s->dpt[3][20]);
  OpM115_6 = OpM214_6*S15-S13*C15;
  OpM215_6 = OpM214_6*C15+S13*S15;
  AlM115_6 = C15*(AlM114_6+OpM214_6*s->dpt[3][20])+S15*(AlM214_6+s->dpt[3][20]*S13);
  AlM215_6 = C15*(AlM214_6+s->dpt[3][20]*S13)-S15*(AlM114_6+OpM214_6*s->dpt[3][20]);
  OpM115_13 = C14*S15;
  OpM215_13 = C14*C15;
  AlM115_13 = s->dpt[3][20]*C14*C15;
  AlM215_13 = -s->dpt[3][20]*C14*S15;
  AlM115_14 = -s->dpt[3][20]*S15;
  AlM215_14 = -s->dpt[3][20]*C15;
  OM116 = OM115*C16-OM315*S16;
  OM216 = qd[16]+OM215;
  OM316 = OM115*S16+OM315*C16;
  OpF116 = C16*(OpF115-qd[16]*OM315)-S16*(OpF314+qd[16]*OM115);
  OpF316 = C16*(OpF314+qd[16]*OM115)+S16*(OpF115-qd[16]*OM315);
  BS216 = OM116*OM216;
  BS316 = OM116*OM316;
  BS616 = OM216*OM316;
  BS916 = -(OM116*OM116+OM216*OM216);
  BeF316 = BS316+OpF215;
  BeF616 = BS616-OpF116;
  AlF116 = C16*(AlF115+BeF315*s->dpt[3][22])-S16*(AlF315+BS915*s->dpt[3][22]);
  AlF216 = AlF215+BeF615*s->dpt[3][22];
  AlF316 = C16*(AlF315+BS915*s->dpt[3][22])+S16*(AlF115+BeF315*s->dpt[3][22]);
  AlM116_1 = AlM115_1*C16-AlM314_1*S16;
  AlM316_1 = AlM115_1*S16+AlM314_1*C16;
  AlM116_2 = AlM115_2*C16-AlM314_2*S16;
  AlM316_2 = AlM115_2*S16+AlM314_2*C16;
  AlM116_3 = AlM115_3*C16-AlM314_3*S16;
  AlM316_3 = AlM115_3*S16+AlM314_3*C16;
  OpM116_4 = OpM115_4*C16-OpM314_4*S16;
  OpM316_4 = OpM115_4*S16+OpM314_4*C16;
  AlM116_4 = -(AlM314_4*S16-C16*(AlM115_4+OpM215_4*s->dpt[3][22]));
  AlM216_4 = AlM215_4-OpM115_4*s->dpt[3][22];
  AlM316_4 = AlM314_4*C16+S16*(AlM115_4+OpM215_4*s->dpt[3][22]);
  OpM116_5 = OpM115_5*C16-OpM314_5*S16;
  OpM316_5 = OpM115_5*S16+OpM314_5*C16;
  AlM116_5 = -(AlM314_5*S16-C16*(AlM115_5+OpM215_5*s->dpt[3][22]));
  AlM216_5 = AlM215_5-OpM115_5*s->dpt[3][22];
  AlM316_5 = AlM314_5*C16+S16*(AlM115_5+OpM215_5*s->dpt[3][22]);
  OpM116_6 = OpM115_6*C16-OpM314_6*S16;
  OpM316_6 = OpM115_6*S16+OpM314_6*C16;
  AlM116_6 = -(AlM314_6*S16-C16*(AlM115_6+OpM215_6*s->dpt[3][22]));
  AlM216_6 = AlM215_6-OpM115_6*s->dpt[3][22];
  AlM316_6 = AlM314_6*C16+S16*(AlM115_6+OpM215_6*s->dpt[3][22]);
  OpM116_13 = OpM115_13*C16+S14*S16;
  OpM316_13 = OpM115_13*S16-S14*C16;
  AlM116_13 = C16*(AlM115_13+OpM215_13*s->dpt[3][22]);
  AlM216_13 = AlM215_13-OpM115_13*s->dpt[3][22];
  AlM316_13 = S16*(AlM115_13+OpM215_13*s->dpt[3][22]);
  OpM116_14 = C15*C16;
  OpM316_14 = C15*S16;
  AlM116_14 = C16*(AlM115_14-s->dpt[3][22]*S15);
  AlM216_14 = AlM215_14-s->dpt[3][22]*C15;
  AlM316_14 = S16*(AlM115_14-s->dpt[3][22]*S15);
  OM117 = qd[17]+OM116;
  OM217 = OM216*C17+OM316*S17;
  OM317 = -(OM216*S17-OM316*C17);
  OpF217 = C17*(OpF215+qd[17]*OM316)+S17*(OpF316-qd[17]*OM216);
  OpF317 = C17*(OpF316-qd[17]*OM216)-S17*(OpF215+qd[17]*OM316);
  BS217 = OM117*OM217;
  BS317 = OM117*OM317;
  BS617 = OM217*OM317;
  AlF117 = AlF116+BeF316*s->dpt[3][24];
  AlF217 = C17*(AlF216+BeF616*s->dpt[3][24])+S17*(AlF316+BS916*s->dpt[3][24]);
  AlF317 = C17*(AlF316+BS916*s->dpt[3][24])-S17*(AlF216+BeF616*s->dpt[3][24]);
  AlM217_1 = AlM215_1*C17+AlM316_1*S17;
  AlM317_1 = -(AlM215_1*S17-AlM316_1*C17);
  AlM217_2 = AlM215_2*C17+AlM316_2*S17;
  AlM317_2 = -(AlM215_2*S17-AlM316_2*C17);
  AlM217_3 = AlM215_3*C17+AlM316_3*S17;
  AlM317_3 = -(AlM215_3*S17-AlM316_3*C17);
  OpM217_4 = OpM215_4*C17+OpM316_4*S17;
  OpM317_4 = -(OpM215_4*S17-OpM316_4*C17);
  AlM117_4 = AlM116_4+OpM215_4*s->dpt[3][24];
  AlM217_4 = AlM316_4*S17+C17*(AlM216_4-OpM116_4*s->dpt[3][24]);
  AlM317_4 = AlM316_4*C17-S17*(AlM216_4-OpM116_4*s->dpt[3][24]);
  OpM217_5 = OpM215_5*C17+OpM316_5*S17;
  OpM317_5 = -(OpM215_5*S17-OpM316_5*C17);
  AlM117_5 = AlM116_5+OpM215_5*s->dpt[3][24];
  AlM217_5 = AlM316_5*S17+C17*(AlM216_5-OpM116_5*s->dpt[3][24]);
  AlM317_5 = AlM316_5*C17-S17*(AlM216_5-OpM116_5*s->dpt[3][24]);
  OpM217_6 = OpM215_6*C17+OpM316_6*S17;
  OpM317_6 = -(OpM215_6*S17-OpM316_6*C17);
  AlM117_6 = AlM116_6+OpM215_6*s->dpt[3][24];
  AlM217_6 = AlM316_6*S17+C17*(AlM216_6-OpM116_6*s->dpt[3][24]);
  AlM317_6 = AlM316_6*C17-S17*(AlM216_6-OpM116_6*s->dpt[3][24]);
  OpM217_13 = OpM215_13*C17+OpM316_13*S17;
  OpM317_13 = -(OpM215_13*S17-OpM316_13*C17);
  AlM117_13 = AlM116_13+OpM215_13*s->dpt[3][24];
  AlM217_13 = AlM316_13*S17+C17*(AlM216_13-OpM116_13*s->dpt[3][24]);
  AlM317_13 = AlM316_13*C17-S17*(AlM216_13-OpM116_13*s->dpt[3][24]);
  OpM217_14 = OpM316_14*S17-S15*C17;
  OpM317_14 = OpM316_14*C17+S15*S17;
  AlM117_14 = AlM116_14-s->dpt[3][24]*S15;
  AlM217_14 = AlM316_14*S17+C17*(AlM216_14-OpM116_14*s->dpt[3][24]);
  AlM317_14 = AlM316_14*C17-S17*(AlM216_14-OpM116_14*s->dpt[3][24]);
  OpM217_15 = C16*S17;
  OpM317_15 = C16*C17;
  AlM217_15 = s->dpt[3][24]*S16*C17;
  AlM317_15 = -s->dpt[3][24]*S16*S17;
  OM118 = OM117*C18-OM317*S18;
  OM218 = qd[18]+OM217;
  OM318 = OM117*S18+OM317*C18;
  OpF118 = C18*(OpF116-qd[18]*OM317)-S18*(OpF317+qd[18]*OM117);
  OpF318 = C18*(OpF317+qd[18]*OM117)+S18*(OpF116-qd[18]*OM317);
  BS218 = OM118*OM218;
  BS318 = OM118*OM318;
  BS618 = OM218*OM318;
  OpM118_4 = OpM116_4*C18-OpM317_4*S18;
  OpM318_4 = OpM116_4*S18+OpM317_4*C18;
  OpM118_5 = OpM116_5*C18-OpM317_5*S18;
  OpM318_5 = OpM116_5*S18+OpM317_5*C18;
  OpM118_6 = OpM116_6*C18-OpM317_6*S18;
  OpM318_6 = OpM116_6*S18+OpM317_6*C18;
  OpM118_13 = OpM116_13*C18-OpM317_13*S18;
  OpM318_13 = OpM116_13*S18+OpM317_13*C18;
  OpM118_14 = OpM116_14*C18-OpM317_14*S18;
  OpM318_14 = OpM116_14*S18+OpM317_14*C18;
  OpM118_15 = -(OpM317_15*S18+S16*C18);
  OpM318_15 = OpM317_15*C18-S16*S18;
  OpM118_16 = S17*S18;
  OpM318_16 = -S17*C18;

// = = Block_0_1_0_1_0_4 = = 
 
// Forward Kinematics 

  OM119 = qd[19]+OM16;
  OM219 = OM26*C19+OM36*S19;
  OM319 = -(OM26*S19-OM36*C19);
  OpF219 = C19*(OpF26+qd[19]*OM36)+S19*(OpF35-qd[19]*OM26);
  OpF319 = C19*(OpF35-qd[19]*OM26)-S19*(OpF26+qd[19]*OM36);
  BS219 = OM119*OM219;
  BS319 = OM119*OM319;
  BS619 = OM219*OM319;
  AlF119 = AlF16+BS16*s->dpt[1][3]+BeF36*s->dpt[3][3];
  AlF219 = C19*(AlF26+BeF46*s->dpt[1][3]+BeF66*s->dpt[3][3])+S19*(AlF35+BS96*s->dpt[3][3]+BeF76*s->dpt[1][3]);
  AlF319 = C19*(AlF35+BS96*s->dpt[3][3]+BeF76*s->dpt[1][3])-S19*(AlF26+BeF46*s->dpt[1][3]+BeF66*s->dpt[3][3]);
  AlM219_1 = AlM26_1*C19+S19*S5;
  AlM319_1 = -(AlM26_1*S19-C19*S5);
  AlM219_2 = AlM26_2*C19+AlM35_2*S19;
  AlM319_2 = -(AlM26_2*S19-AlM35_2*C19);
  AlM219_3 = AlM26_3*C19+AlM35_3*S19;
  AlM319_3 = -(AlM26_3*S19-AlM35_3*C19);
  OpM219_4 = OpM26_4*C19+S19*S5;
  OpM319_4 = -(OpM26_4*S19-C19*S5);
  AlM119_4 = OpM26_4*s->dpt[3][3];
  AlM219_4 = -(OpM26_4*s->dpt[1][3]*S19+C19*(OpM16_4*s->dpt[3][3]-s->dpt[1][3]*S5));
  AlM319_4 = -(OpM26_4*s->dpt[1][3]*C19-S19*(OpM16_4*s->dpt[3][3]-s->dpt[1][3]*S5));
  OpM219_5 = C19*C6;
  OpM319_5 = -S19*C6;
  AlM119_5 = s->dpt[3][3]*C6;
  AlM219_5 = -(s->dpt[1][3]*S19*C6+s->dpt[3][3]*C19*S6);
  AlM319_5 = -(s->dpt[1][3]*C19*C6-s->dpt[3][3]*S19*S6);
  AlM219_6 = s->dpt[1][3]*C19;
  AlM319_6 = -s->dpt[1][3]*S19;
  OM120 = OM119*C20-OM319*S20;
  OM220 = qd[20]+OM219;
  OM320 = OM119*S20+OM319*C20;
  OpF120 = C20*(OpF16-qd[20]*OM319)-S20*(OpF319+qd[20]*OM119);
  OpF320 = C20*(OpF319+qd[20]*OM119)+S20*(OpF16-qd[20]*OM319);
  BS220 = OM120*OM220;
  BS320 = OM120*OM320;
  BS620 = OM220*OM320;
  BS920 = -(OM120*OM120+OM220*OM220);
  BeF320 = BS320+OpF219;
  BeF620 = BS620-OpF120;
  AlF120 = AlF119*C20-AlF319*S20;
  AlF320 = AlF119*S20+AlF319*C20;
  AlM120_1 = AlM16_1*C20-AlM319_1*S20;
  AlM320_1 = AlM16_1*S20+AlM319_1*C20;
  AlM120_2 = AlM16_2*C20-AlM319_2*S20;
  AlM320_2 = AlM16_2*S20+AlM319_2*C20;
  AlM120_3 = AlM16_3*C20-AlM319_3*S20;
  AlM320_3 = AlM16_3*S20+AlM319_3*C20;
  OpM120_4 = OpM16_4*C20-OpM319_4*S20;
  OpM320_4 = OpM16_4*S20+OpM319_4*C20;
  AlM120_4 = AlM119_4*C20-AlM319_4*S20;
  AlM320_4 = AlM119_4*S20+AlM319_4*C20;
  OpM120_5 = -(OpM319_5*S20-C20*S6);
  OpM320_5 = OpM319_5*C20+S20*S6;
  AlM120_5 = AlM119_5*C20-AlM319_5*S20;
  AlM320_5 = AlM119_5*S20+AlM319_5*C20;
  OpM120_6 = -C19*S20;
  OpM320_6 = C19*C20;
  AlM120_6 = -AlM319_6*S20;
  AlM320_6 = AlM319_6*C20;
  OM121 = OM120*C21+OM220*S21;
  OM221 = -(OM120*S21-OM220*C21);
  OM321 = qd[21]+OM320;
  OpF121 = C21*(OpF120+qd[21]*OM220)+S21*(OpF219-qd[21]*OM120);
  OpF221 = C21*(OpF219-qd[21]*OM120)-S21*(OpF120+qd[21]*OM220);
  BS121 = -(OM221*OM221+OM321*OM321);
  BS221 = OM121*OM221;
  BS321 = OM121*OM321;
  BS521 = -(OM121*OM121+OM321*OM321);
  BS621 = OM221*OM321;
  BS921 = -(OM121*OM121+OM221*OM221);
  BeF221 = BS221-OpF320;
  BeF321 = BS321+OpF221;
  BeF421 = BS221+OpF320;
  BeF621 = BS621-OpF121;
  BeF721 = BS321-OpF221;
  BeF821 = BS621+OpF121;
  AlF121 = C21*(AlF120+BeF320*s->dpt[3][32])+S21*(AlF219+BeF620*s->dpt[3][32]);
  AlF221 = C21*(AlF219+BeF620*s->dpt[3][32])-S21*(AlF120+BeF320*s->dpt[3][32]);
  AlF321 = AlF320+BS920*s->dpt[3][32];
  AlM121_1 = AlM120_1*C21+AlM219_1*S21;
  AlM221_1 = -(AlM120_1*S21-AlM219_1*C21);
  AlM121_2 = AlM120_2*C21+AlM219_2*S21;
  AlM221_2 = -(AlM120_2*S21-AlM219_2*C21);
  AlM121_3 = AlM120_3*C21+AlM219_3*S21;
  AlM221_3 = -(AlM120_3*S21-AlM219_3*C21);
  OpM121_4 = OpM120_4*C21+OpM219_4*S21;
  OpM221_4 = -(OpM120_4*S21-OpM219_4*C21);
  AlM121_4 = C21*(AlM120_4+OpM219_4*s->dpt[3][32])+S21*(AlM219_4-OpM120_4*s->dpt[3][32]);
  AlM221_4 = C21*(AlM219_4-OpM120_4*s->dpt[3][32])-S21*(AlM120_4+OpM219_4*s->dpt[3][32]);
  OpM121_5 = OpM120_5*C21+OpM219_5*S21;
  OpM221_5 = -(OpM120_5*S21-OpM219_5*C21);
  AlM121_5 = C21*(AlM120_5+OpM219_5*s->dpt[3][32])+S21*(AlM219_5-OpM120_5*s->dpt[3][32]);
  AlM221_5 = C21*(AlM219_5-OpM120_5*s->dpt[3][32])-S21*(AlM120_5+OpM219_5*s->dpt[3][32]);
  OpM121_6 = OpM120_6*C21+S19*S21;
  OpM221_6 = -(OpM120_6*S21-S19*C21);
  AlM121_6 = C21*(AlM120_6+s->dpt[3][32]*S19)+S21*(AlM219_6-OpM120_6*s->dpt[3][32]);
  AlM221_6 = C21*(AlM219_6-OpM120_6*s->dpt[3][32])-S21*(AlM120_6+s->dpt[3][32]*S19);
  OpM121_19 = C20*C21;
  OpM221_19 = -C20*S21;
  AlM121_19 = -s->dpt[3][32]*C20*S21;
  AlM221_19 = -s->dpt[3][32]*C20*C21;
  AlM221_20 = -s->dpt[3][32]*S21;

// = = Block_0_1_0_2_0_5 = = 
 
// Forward Kinematics 

  OM122 = OM121*C22-OM321*S22;
  OM222 = qd[22]+OM221;
  OM322 = OM121*S22+OM321*C22;
  OpF122 = C22*(OpF121-qd[22]*OM321)-S22*(OpF320+qd[22]*OM121);
  OpF322 = C22*(OpF320+qd[22]*OM121)+S22*(OpF121-qd[22]*OM321);
  BS222 = OM122*OM222;
  BS322 = OM122*OM322;
  BS522 = -(OM122*OM122+OM322*OM322);
  BS622 = OM222*OM322;
  BeF222 = BS222-OpF322;
  BeF822 = BS622+OpF122;
  AlF122 = C22*(AlF121+BS121*s->dpt[1][36]+BeF221*s->dpt[2][36]+BeF321*s->dpt[3][36])-S22*(AlF321+BS921*s->dpt[3][36]+
 BeF721*s->dpt[1][36]+BeF821*s->dpt[2][36]);
  AlF222 = AlF221+BS521*s->dpt[2][36]+BeF421*s->dpt[1][36]+BeF621*s->dpt[3][36];
  AlF322 = C22*(AlF321+BS921*s->dpt[3][36]+BeF721*s->dpt[1][36]+BeF821*s->dpt[2][36])+S22*(AlF121+BS121*s->dpt[1][36]+
 BeF221*s->dpt[2][36]+BeF321*s->dpt[3][36]);
  AlM122_1 = AlM121_1*C22-AlM320_1*S22;
  AlM322_1 = AlM121_1*S22+AlM320_1*C22;
  AlM122_2 = AlM121_2*C22-AlM320_2*S22;
  AlM322_2 = AlM121_2*S22+AlM320_2*C22;
  AlM122_3 = AlM121_3*C22-AlM320_3*S22;
  AlM322_3 = AlM121_3*S22+AlM320_3*C22;
  OpM122_4 = OpM121_4*C22-OpM320_4*S22;
  OpM322_4 = OpM121_4*S22+OpM320_4*C22;
  AlM122_4 = C22*(AlM121_4+OpM221_4*s->dpt[3][36]-OpM320_4*s->dpt[2][36])-S22*(AlM320_4+OpM121_4*s->dpt[2][36]-OpM221_4*
 s->dpt[1][36]);
  AlM222_4 = AlM221_4-OpM121_4*s->dpt[3][36]+OpM320_4*s->dpt[1][36];
  AlM322_4 = C22*(AlM320_4+OpM121_4*s->dpt[2][36]-OpM221_4*s->dpt[1][36])+S22*(AlM121_4+OpM221_4*s->dpt[3][36]-OpM320_4*
 s->dpt[2][36]);
  OpM122_5 = OpM121_5*C22-OpM320_5*S22;
  OpM322_5 = OpM121_5*S22+OpM320_5*C22;
  AlM122_5 = C22*(AlM121_5+OpM221_5*s->dpt[3][36]-OpM320_5*s->dpt[2][36])-S22*(AlM320_5+OpM121_5*s->dpt[2][36]-OpM221_5*
 s->dpt[1][36]);
  AlM222_5 = AlM221_5-OpM121_5*s->dpt[3][36]+OpM320_5*s->dpt[1][36];
  AlM322_5 = C22*(AlM320_5+OpM121_5*s->dpt[2][36]-OpM221_5*s->dpt[1][36])+S22*(AlM121_5+OpM221_5*s->dpt[3][36]-OpM320_5*
 s->dpt[2][36]);
  OpM122_6 = OpM121_6*C22-OpM320_6*S22;
  OpM322_6 = OpM121_6*S22+OpM320_6*C22;
  AlM122_6 = C22*(AlM121_6+OpM221_6*s->dpt[3][36]-OpM320_6*s->dpt[2][36])-S22*(AlM320_6+OpM121_6*s->dpt[2][36]-OpM221_6*
 s->dpt[1][36]);
  AlM222_6 = AlM221_6-OpM121_6*s->dpt[3][36]+OpM320_6*s->dpt[1][36];
  AlM322_6 = C22*(AlM320_6+OpM121_6*s->dpt[2][36]-OpM221_6*s->dpt[1][36])+S22*(AlM121_6+OpM221_6*s->dpt[3][36]-OpM320_6*
 s->dpt[2][36]);
  OpM122_19 = OpM121_19*C22-S20*S22;
  OpM322_19 = OpM121_19*S22+S20*C22;
  AlM122_19 = C22*(AlM121_19+OpM221_19*s->dpt[3][36]-s->dpt[2][36]*S20)-S22*(OpM121_19*s->dpt[2][36]-OpM221_19*
 s->dpt[1][36]);
  AlM222_19 = AlM221_19-OpM121_19*s->dpt[3][36]+s->dpt[1][36]*S20;
  AlM322_19 = C22*(OpM121_19*s->dpt[2][36]-OpM221_19*s->dpt[1][36])+S22*(AlM121_19+OpM221_19*s->dpt[3][36]-s->dpt[2][36]
 *S20);
  OpM122_20 = S21*C22;
  OpM322_20 = S21*S22;
  AlM122_20 = C21*C22*(s->dpt[3][32]+s->dpt[3][36])+S22*(s->dpt[1][36]*C21-s->dpt[2][36]*S21);
  AlM222_20 = AlM221_20-s->dpt[3][36]*S21;
  AlM322_20 = C21*S22*(s->dpt[3][32]+s->dpt[3][36])-C22*(s->dpt[1][36]*C21-s->dpt[2][36]*S21);
  AlM122_21 = -s->dpt[2][36]*C22;
  OM123 = qd[23]+OM122;
  OM223 = OM222*C23+OM322*S23;
  OM323 = -(OM222*S23-OM322*C23);
  OpF223 = C23*(OpF221+qd[23]*OM322)+S23*(OpF322-qd[23]*OM222);
  OpF323 = C23*(OpF322-qd[23]*OM222)-S23*(OpF221+qd[23]*OM322);
  BS223 = OM123*OM223;
  BS523 = -(OM123*OM123+OM323*OM323);
  BeF223 = BS223-OpF323;
  BeF823 = OpF122+OM223*OM323;
  AlF123 = AlF122+BeF222*s->dpt[2][40];
  AlF223 = C23*(AlF222+BS522*s->dpt[2][40])+S23*(AlF322+BeF822*s->dpt[2][40]);
  AlF323 = C23*(AlF322+BeF822*s->dpt[2][40])-S23*(AlF222+BS522*s->dpt[2][40]);
  AlM223_1 = AlM221_1*C23+AlM322_1*S23;
  AlM323_1 = -(AlM221_1*S23-AlM322_1*C23);
  AlM223_2 = AlM221_2*C23+AlM322_2*S23;
  AlM323_2 = -(AlM221_2*S23-AlM322_2*C23);
  AlM223_3 = AlM221_3*C23+AlM322_3*S23;
  AlM323_3 = -(AlM221_3*S23-AlM322_3*C23);
  OpM223_4 = OpM221_4*C23+OpM322_4*S23;
  OpM323_4 = -(OpM221_4*S23-OpM322_4*C23);
  AlM123_4 = AlM122_4-OpM322_4*s->dpt[2][40];
  AlM223_4 = AlM222_4*C23+S23*(AlM322_4+OpM122_4*s->dpt[2][40]);
  AlM323_4 = -(AlM222_4*S23-C23*(AlM322_4+OpM122_4*s->dpt[2][40]));
  OpM223_5 = OpM221_5*C23+OpM322_5*S23;
  OpM323_5 = -(OpM221_5*S23-OpM322_5*C23);
  AlM123_5 = AlM122_5-OpM322_5*s->dpt[2][40];
  AlM223_5 = AlM222_5*C23+S23*(AlM322_5+OpM122_5*s->dpt[2][40]);
  AlM323_5 = -(AlM222_5*S23-C23*(AlM322_5+OpM122_5*s->dpt[2][40]));
  OpM223_6 = OpM221_6*C23+OpM322_6*S23;
  OpM323_6 = -(OpM221_6*S23-OpM322_6*C23);
  AlM123_6 = AlM122_6-OpM322_6*s->dpt[2][40];
  AlM223_6 = AlM222_6*C23+S23*(AlM322_6+OpM122_6*s->dpt[2][40]);
  AlM323_6 = -(AlM222_6*S23-C23*(AlM322_6+OpM122_6*s->dpt[2][40]));
  OpM223_19 = OpM221_19*C23+OpM322_19*S23;
  OpM323_19 = -(OpM221_19*S23-OpM322_19*C23);
  AlM123_19 = AlM122_19-OpM322_19*s->dpt[2][40];
  AlM223_19 = AlM222_19*C23+S23*(AlM322_19+OpM122_19*s->dpt[2][40]);
  AlM323_19 = -(AlM222_19*S23-C23*(AlM322_19+OpM122_19*s->dpt[2][40]));
  OpM223_20 = OpM322_20*S23+C21*C23;
  OpM323_20 = OpM322_20*C23-C21*S23;
  AlM123_20 = AlM122_20-OpM322_20*s->dpt[2][40];
  AlM223_20 = AlM222_20*C23+S23*(AlM322_20+OpM122_20*s->dpt[2][40]);
  AlM323_20 = -(AlM222_20*S23-C23*(AlM322_20+OpM122_20*s->dpt[2][40]));
  OpM223_21 = C22*S23;
  OpM323_21 = C22*C23;
  AlM123_21 = AlM122_21-s->dpt[2][40]*C22;
  AlM223_21 = s->dpt[1][36]*C23-S22*S23*(s->dpt[2][36]+s->dpt[2][40]);
  AlM323_21 = -(s->dpt[1][36]*S23+S22*C23*(s->dpt[2][36]+s->dpt[2][40]));
  OM124 = OM123*C24-OM323*S24;
  OM224 = qd[24]+OM223;
  OM324 = OM123*S24+OM323*C24;
  OpF124 = C24*(OpF122-qd[24]*OM323)-S24*(OpF323+qd[24]*OM123);
  OpF324 = C24*(OpF323+qd[24]*OM123)+S24*(OpF122-qd[24]*OM323);
  BS124 = -(OM224*OM224+OM324*OM324);
  BS224 = OM124*OM224;
  BS324 = OM124*OM324;
  BS524 = -(OM124*OM124+OM324*OM324);
  BS624 = OM224*OM324;
  BeF224 = BS224-OpF324;
  BeF424 = BS224+OpF324;
  BeF724 = BS324-OpF223;
  BeF824 = BS624+OpF124;
  AlF124 = C24*(AlF123+BeF223*s->dpt[2][42])-S24*(AlF323+BeF823*s->dpt[2][42]);
  AlF224 = AlF223+BS523*s->dpt[2][42];
  AlF324 = C24*(AlF323+BeF823*s->dpt[2][42])+S24*(AlF123+BeF223*s->dpt[2][42]);
  AlM124_1 = AlM122_1*C24-AlM323_1*S24;
  AlM324_1 = AlM122_1*S24+AlM323_1*C24;
  AlM124_2 = AlM122_2*C24-AlM323_2*S24;
  AlM324_2 = AlM122_2*S24+AlM323_2*C24;
  AlM124_3 = AlM122_3*C24-AlM323_3*S24;
  AlM324_3 = AlM122_3*S24+AlM323_3*C24;
  OpM124_4 = OpM122_4*C24-OpM323_4*S24;
  OpM324_4 = OpM122_4*S24+OpM323_4*C24;
  AlM124_4 = C24*(AlM123_4-OpM323_4*s->dpt[2][42])-S24*(AlM323_4+OpM122_4*s->dpt[2][42]);
  AlM324_4 = C24*(AlM323_4+OpM122_4*s->dpt[2][42])+S24*(AlM123_4-OpM323_4*s->dpt[2][42]);
  OpM124_5 = OpM122_5*C24-OpM323_5*S24;
  OpM324_5 = OpM122_5*S24+OpM323_5*C24;
  AlM124_5 = C24*(AlM123_5-OpM323_5*s->dpt[2][42])-S24*(AlM323_5+OpM122_5*s->dpt[2][42]);
  AlM324_5 = C24*(AlM323_5+OpM122_5*s->dpt[2][42])+S24*(AlM123_5-OpM323_5*s->dpt[2][42]);
  OpM124_6 = OpM122_6*C24-OpM323_6*S24;
  OpM324_6 = OpM122_6*S24+OpM323_6*C24;
  AlM124_6 = C24*(AlM123_6-OpM323_6*s->dpt[2][42])-S24*(AlM323_6+OpM122_6*s->dpt[2][42]);
  AlM324_6 = C24*(AlM323_6+OpM122_6*s->dpt[2][42])+S24*(AlM123_6-OpM323_6*s->dpt[2][42]);
  OpM124_19 = OpM122_19*C24-OpM323_19*S24;
  OpM324_19 = OpM122_19*S24+OpM323_19*C24;
  AlM124_19 = C24*(AlM123_19-OpM323_19*s->dpt[2][42])-S24*(AlM323_19+OpM122_19*s->dpt[2][42]);
  AlM324_19 = C24*(AlM323_19+OpM122_19*s->dpt[2][42])+S24*(AlM123_19-OpM323_19*s->dpt[2][42]);
  OpM124_20 = OpM122_20*C24-OpM323_20*S24;
  OpM324_20 = OpM122_20*S24+OpM323_20*C24;
  AlM124_20 = C24*(AlM123_20-OpM323_20*s->dpt[2][42])-S24*(AlM323_20+OpM122_20*s->dpt[2][42]);
  AlM324_20 = C24*(AlM323_20+OpM122_20*s->dpt[2][42])+S24*(AlM123_20-OpM323_20*s->dpt[2][42]);
  OpM124_21 = -(OpM323_21*S24+S22*C24);
  OpM324_21 = OpM323_21*C24-S22*S24;
  AlM124_21 = C24*(AlM123_21-OpM323_21*s->dpt[2][42])-S24*(AlM323_21-s->dpt[2][42]*S22);
  AlM324_21 = C24*(AlM323_21-s->dpt[2][42]*S22)+S24*(AlM123_21-OpM323_21*s->dpt[2][42]);
  OpM124_22 = S23*S24;
  OpM324_22 = -S23*C24;
  AlM124_22 = s->dpt[2][42]*S23*C24;
  AlM324_22 = s->dpt[2][42]*S23*S24;
  OM125 = OM124*C25+OM224*S25;
  OM225 = -(OM124*S25-OM224*C25);
  OM325 = qd[25]+OM324;
  OpF125 = C25*(OpF124+qd[25]*OM224)+S25*(OpF223-qd[25]*OM124);
  OpF225 = C25*(OpF223-qd[25]*OM124)-S25*(OpF124+qd[25]*OM224);
  BS125 = -(OM225*OM225+OM325*OM325);
  BS225 = OM125*OM225;
  BS325 = OM125*OM325;
  BS625 = OM225*OM325;
  BeF425 = BS225+OpF324;
  BeF725 = BS325-OpF225;
  AlF125 = C25*(AlF124+BS124*s->dpt[1][44]+BeF224*s->dpt[2][44])+S25*(AlF224+BS524*s->dpt[2][44]+BeF424*s->dpt[1][44]);
  AlF225 = C25*(AlF224+BS524*s->dpt[2][44]+BeF424*s->dpt[1][44])-S25*(AlF124+BS124*s->dpt[1][44]+BeF224*s->dpt[2][44]);
  AlF325 = AlF324+BeF724*s->dpt[1][44]+BeF824*s->dpt[2][44];
  AlM125_1 = AlM124_1*C25+AlM223_1*S25;
  AlM225_1 = -(AlM124_1*S25-AlM223_1*C25);
  AlM125_2 = AlM124_2*C25+AlM223_2*S25;
  AlM225_2 = -(AlM124_2*S25-AlM223_2*C25);
  AlM125_3 = AlM124_3*C25+AlM223_3*S25;
  AlM225_3 = -(AlM124_3*S25-AlM223_3*C25);
  OpM125_4 = OpM124_4*C25+OpM223_4*S25;
  OpM225_4 = -(OpM124_4*S25-OpM223_4*C25);
  AlM125_4 = C25*(AlM124_4-OpM324_4*s->dpt[2][44])+S25*(AlM223_4+OpM324_4*s->dpt[1][44]);
  AlM225_4 = C25*(AlM223_4+OpM324_4*s->dpt[1][44])-S25*(AlM124_4-OpM324_4*s->dpt[2][44]);
  AlM325_4 = AlM324_4+OpM124_4*s->dpt[2][44]-OpM223_4*s->dpt[1][44];
  OpM125_5 = OpM124_5*C25+OpM223_5*S25;
  OpM225_5 = -(OpM124_5*S25-OpM223_5*C25);
  AlM125_5 = C25*(AlM124_5-OpM324_5*s->dpt[2][44])+S25*(AlM223_5+OpM324_5*s->dpt[1][44]);
  AlM225_5 = C25*(AlM223_5+OpM324_5*s->dpt[1][44])-S25*(AlM124_5-OpM324_5*s->dpt[2][44]);
  AlM325_5 = AlM324_5+OpM124_5*s->dpt[2][44]-OpM223_5*s->dpt[1][44];
  OpM125_6 = OpM124_6*C25+OpM223_6*S25;
  OpM225_6 = -(OpM124_6*S25-OpM223_6*C25);
  AlM125_6 = C25*(AlM124_6-OpM324_6*s->dpt[2][44])+S25*(AlM223_6+OpM324_6*s->dpt[1][44]);
  AlM225_6 = C25*(AlM223_6+OpM324_6*s->dpt[1][44])-S25*(AlM124_6-OpM324_6*s->dpt[2][44]);
  AlM325_6 = AlM324_6+OpM124_6*s->dpt[2][44]-OpM223_6*s->dpt[1][44];
  OpM125_19 = OpM124_19*C25+OpM223_19*S25;
  OpM225_19 = -(OpM124_19*S25-OpM223_19*C25);
  AlM125_19 = C25*(AlM124_19-OpM324_19*s->dpt[2][44])+S25*(AlM223_19+OpM324_19*s->dpt[1][44]);
  AlM225_19 = C25*(AlM223_19+OpM324_19*s->dpt[1][44])-S25*(AlM124_19-OpM324_19*s->dpt[2][44]);
  AlM325_19 = AlM324_19+OpM124_19*s->dpt[2][44]-OpM223_19*s->dpt[1][44];
  OpM125_20 = OpM124_20*C25+OpM223_20*S25;
  OpM225_20 = -(OpM124_20*S25-OpM223_20*C25);
  AlM125_20 = C25*(AlM124_20-OpM324_20*s->dpt[2][44])+S25*(AlM223_20+OpM324_20*s->dpt[1][44]);
  AlM225_20 = C25*(AlM223_20+OpM324_20*s->dpt[1][44])-S25*(AlM124_20-OpM324_20*s->dpt[2][44]);
  AlM325_20 = AlM324_20+OpM124_20*s->dpt[2][44]-OpM223_20*s->dpt[1][44];
  OpM125_21 = OpM124_21*C25+OpM223_21*S25;
  OpM225_21 = -(OpM124_21*S25-OpM223_21*C25);
  AlM125_21 = C25*(AlM124_21-OpM324_21*s->dpt[2][44])+S25*(AlM223_21+OpM324_21*s->dpt[1][44]);
  AlM225_21 = C25*(AlM223_21+OpM324_21*s->dpt[1][44])-S25*(AlM124_21-OpM324_21*s->dpt[2][44]);
  AlM325_21 = AlM324_21+OpM124_21*s->dpt[2][44]-OpM223_21*s->dpt[1][44];
  OpM125_22 = OpM124_22*C25+C23*S25;
  OpM225_22 = -(OpM124_22*S25-C23*C25);
  AlM125_22 = OpM324_22*s->dpt[1][44]*S25+C25*(AlM124_22-OpM324_22*s->dpt[2][44]);
  AlM225_22 = OpM324_22*s->dpt[1][44]*C25-S25*(AlM124_22-OpM324_22*s->dpt[2][44]);
  AlM325_22 = AlM324_22+OpM124_22*s->dpt[2][44]-s->dpt[1][44]*C23;
  OpM125_23 = C24*C25;
  OpM225_23 = -C24*S25;
  AlM125_23 = S24*(s->dpt[1][44]*S25-C25*(s->dpt[2][42]+s->dpt[2][44]));
  AlM225_23 = S24*(s->dpt[1][44]*C25+S25*(s->dpt[2][42]+s->dpt[2][44]));
  AlM325_23 = C24*(s->dpt[2][42]+s->dpt[2][44]);
  OM126 = OM125*C26-OM325*S26;
  OM226 = qd[26]+OM225;
  OM326 = OM125*S26+OM325*C26;
  OpF126 = C26*(OpF125-qd[26]*OM325)-S26*(OpF324+qd[26]*OM125);
  OpF326 = C26*(OpF324+qd[26]*OM125)+S26*(OpF125-qd[26]*OM325);
  BS526 = -(OM126*OM126+OM326*OM326);
  BeF226 = OM126*OM226-OpF326;
  BeF826 = OpF126+OM226*OM326;
  AlF126 = C26*(AlF125+BS125*s->dpt[1][46])-S26*(AlF325+BeF725*s->dpt[1][46]);
  AlF226 = AlF225+BeF425*s->dpt[1][46];
  AlF326 = C26*(AlF325+BeF725*s->dpt[1][46])+S26*(AlF125+BS125*s->dpt[1][46]);
  AlM126_1 = AlM125_1*C26-AlM324_1*S26;
  AlM326_1 = AlM125_1*S26+AlM324_1*C26;
  AlM126_2 = AlM125_2*C26-AlM324_2*S26;
  AlM326_2 = AlM125_2*S26+AlM324_2*C26;
  AlM126_3 = AlM125_3*C26-AlM324_3*S26;
  AlM326_3 = AlM125_3*S26+AlM324_3*C26;
  OpM126_4 = OpM125_4*C26-OpM324_4*S26;
  OpM326_4 = OpM125_4*S26+OpM324_4*C26;
  AlM126_4 = AlM125_4*C26-S26*(AlM325_4-OpM225_4*s->dpt[1][46]);
  AlM226_4 = AlM225_4+OpM324_4*s->dpt[1][46];
  AlM326_4 = AlM125_4*S26+C26*(AlM325_4-OpM225_4*s->dpt[1][46]);
  OpM126_5 = OpM125_5*C26-OpM324_5*S26;
  OpM326_5 = OpM125_5*S26+OpM324_5*C26;
  AlM126_5 = AlM125_5*C26-S26*(AlM325_5-OpM225_5*s->dpt[1][46]);
  AlM226_5 = AlM225_5+OpM324_5*s->dpt[1][46];
  AlM326_5 = AlM125_5*S26+C26*(AlM325_5-OpM225_5*s->dpt[1][46]);
  OpM126_6 = OpM125_6*C26-OpM324_6*S26;
  OpM326_6 = OpM125_6*S26+OpM324_6*C26;
  AlM126_6 = AlM125_6*C26-S26*(AlM325_6-OpM225_6*s->dpt[1][46]);
  AlM226_6 = AlM225_6+OpM324_6*s->dpt[1][46];
  AlM326_6 = AlM125_6*S26+C26*(AlM325_6-OpM225_6*s->dpt[1][46]);
  OpM126_19 = OpM125_19*C26-OpM324_19*S26;
  OpM326_19 = OpM125_19*S26+OpM324_19*C26;
  AlM126_19 = AlM125_19*C26-S26*(AlM325_19-OpM225_19*s->dpt[1][46]);
  AlM226_19 = AlM225_19+OpM324_19*s->dpt[1][46];
  AlM326_19 = AlM125_19*S26+C26*(AlM325_19-OpM225_19*s->dpt[1][46]);
  OpM126_20 = OpM125_20*C26-OpM324_20*S26;
  OpM326_20 = OpM125_20*S26+OpM324_20*C26;
  AlM126_20 = AlM125_20*C26-S26*(AlM325_20-OpM225_20*s->dpt[1][46]);
  AlM226_20 = AlM225_20+OpM324_20*s->dpt[1][46];
  AlM326_20 = AlM125_20*S26+C26*(AlM325_20-OpM225_20*s->dpt[1][46]);
  OpM126_21 = OpM125_21*C26-OpM324_21*S26;
  OpM326_21 = OpM125_21*S26+OpM324_21*C26;
  AlM126_21 = AlM125_21*C26-S26*(AlM325_21-OpM225_21*s->dpt[1][46]);
  AlM226_21 = AlM225_21+OpM324_21*s->dpt[1][46];
  AlM326_21 = AlM125_21*S26+C26*(AlM325_21-OpM225_21*s->dpt[1][46]);
  OpM126_22 = OpM125_22*C26-OpM324_22*S26;
  OpM326_22 = OpM125_22*S26+OpM324_22*C26;
  AlM126_22 = AlM125_22*C26-S26*(AlM325_22-OpM225_22*s->dpt[1][46]);
  AlM226_22 = AlM225_22+OpM324_22*s->dpt[1][46];
  AlM326_22 = AlM125_22*S26+C26*(AlM325_22-OpM225_22*s->dpt[1][46]);
  OpM126_23 = OpM125_23*C26-S24*S26;
  OpM326_23 = OpM125_23*S26+S24*C26;
  AlM126_23 = AlM125_23*C26-S26*(AlM325_23-OpM225_23*s->dpt[1][46]);
  AlM226_23 = AlM225_23+s->dpt[1][46]*S24;
  AlM326_23 = AlM125_23*S26+C26*(AlM325_23-OpM225_23*s->dpt[1][46]);
  OpM126_24 = S25*C26;
  OpM326_24 = S25*S26;
  AlM126_24 = S26*(s->dpt[1][44]+s->dpt[1][46]*C25);
  AlM326_24 = -C26*(s->dpt[1][44]+s->dpt[1][46]*C25);
  OM127 = OM126*C27+OM226*S27;
  OM227 = -(OM126*S27-OM226*C27);
  OM327 = qd[27]+OM326;
  OpF127 = C27*(OpF126+qd[27]*OM226)+S27*(OpF225-qd[27]*OM126);
  OpF227 = C27*(OpF225-qd[27]*OM126)-S27*(OpF126+qd[27]*OM226);
  AlF127 = C27*(AlF126+BeF226*s->dpt[2][47])+S27*(AlF226+BS526*s->dpt[2][47]);
  AlF227 = C27*(AlF226+BS526*s->dpt[2][47])-S27*(AlF126+BeF226*s->dpt[2][47]);
  AlF327 = AlF326+BeF826*s->dpt[2][47];
  AlM127_1 = AlM126_1*C27+AlM225_1*S27;
  AlM227_1 = -(AlM126_1*S27-AlM225_1*C27);
  AlM127_2 = AlM126_2*C27+AlM225_2*S27;
  AlM227_2 = -(AlM126_2*S27-AlM225_2*C27);
  AlM127_3 = AlM126_3*C27+AlM225_3*S27;
  AlM227_3 = -(AlM126_3*S27-AlM225_3*C27);
  OpM127_4 = OpM126_4*C27+OpM225_4*S27;
  OpM227_4 = -(OpM126_4*S27-OpM225_4*C27);
  AlM127_4 = AlM226_4*S27+C27*(AlM126_4-OpM326_4*s->dpt[2][47]);
  AlM227_4 = AlM226_4*C27-S27*(AlM126_4-OpM326_4*s->dpt[2][47]);
  AlM327_4 = AlM326_4+OpM126_4*s->dpt[2][47];
  OpM127_5 = OpM126_5*C27+OpM225_5*S27;
  OpM227_5 = -(OpM126_5*S27-OpM225_5*C27);
  AlM127_5 = AlM226_5*S27+C27*(AlM126_5-OpM326_5*s->dpt[2][47]);
  AlM227_5 = AlM226_5*C27-S27*(AlM126_5-OpM326_5*s->dpt[2][47]);
  AlM327_5 = AlM326_5+OpM126_5*s->dpt[2][47];
  OpM127_6 = OpM126_6*C27+OpM225_6*S27;
  OpM227_6 = -(OpM126_6*S27-OpM225_6*C27);
  AlM127_6 = AlM226_6*S27+C27*(AlM126_6-OpM326_6*s->dpt[2][47]);
  AlM227_6 = AlM226_6*C27-S27*(AlM126_6-OpM326_6*s->dpt[2][47]);
  AlM327_6 = AlM326_6+OpM126_6*s->dpt[2][47];
  OpM127_19 = OpM126_19*C27+OpM225_19*S27;
  OpM227_19 = -(OpM126_19*S27-OpM225_19*C27);
  AlM127_19 = AlM226_19*S27+C27*(AlM126_19-OpM326_19*s->dpt[2][47]);
  AlM227_19 = AlM226_19*C27-S27*(AlM126_19-OpM326_19*s->dpt[2][47]);
  AlM327_19 = AlM326_19+OpM126_19*s->dpt[2][47];
  OpM127_20 = OpM126_20*C27+OpM225_20*S27;
  OpM227_20 = -(OpM126_20*S27-OpM225_20*C27);
  AlM127_20 = AlM226_20*S27+C27*(AlM126_20-OpM326_20*s->dpt[2][47]);
  AlM227_20 = AlM226_20*C27-S27*(AlM126_20-OpM326_20*s->dpt[2][47]);
  AlM327_20 = AlM326_20+OpM126_20*s->dpt[2][47];
  OpM127_21 = OpM126_21*C27+OpM225_21*S27;
  OpM227_21 = -(OpM126_21*S27-OpM225_21*C27);
  AlM127_21 = AlM226_21*S27+C27*(AlM126_21-OpM326_21*s->dpt[2][47]);
  AlM227_21 = AlM226_21*C27-S27*(AlM126_21-OpM326_21*s->dpt[2][47]);
  AlM327_21 = AlM326_21+OpM126_21*s->dpt[2][47];
  OpM127_22 = OpM126_22*C27+OpM225_22*S27;
  OpM227_22 = -(OpM126_22*S27-OpM225_22*C27);
  AlM127_22 = AlM226_22*S27+C27*(AlM126_22-OpM326_22*s->dpt[2][47]);
  AlM227_22 = AlM226_22*C27-S27*(AlM126_22-OpM326_22*s->dpt[2][47]);
  AlM327_22 = AlM326_22+OpM126_22*s->dpt[2][47];
  OpM127_23 = OpM126_23*C27+OpM225_23*S27;
  OpM227_23 = -(OpM126_23*S27-OpM225_23*C27);
  AlM127_23 = AlM226_23*S27+C27*(AlM126_23-OpM326_23*s->dpt[2][47]);
  AlM227_23 = AlM226_23*C27-S27*(AlM126_23-OpM326_23*s->dpt[2][47]);
  AlM327_23 = AlM326_23+OpM126_23*s->dpt[2][47];
  OpM127_24 = OpM126_24*C27+C25*S27;
  OpM227_24 = -(OpM126_24*S27-C25*C27);
  AlM127_24 = C27*(AlM126_24-OpM326_24*s->dpt[2][47]);
  AlM227_24 = -S27*(AlM126_24-OpM326_24*s->dpt[2][47]);
  AlM327_24 = AlM326_24+OpM126_24*s->dpt[2][47];
  OpM127_25 = -S26*C27;
  OpM227_25 = S26*S27;
  AlM127_25 = s->dpt[1][46]*S27-s->dpt[2][47]*C26*C27;
  AlM227_25 = s->dpt[1][46]*C27+s->dpt[2][47]*C26*S27;
  AlM327_25 = -s->dpt[2][47]*S26;
  OM128 = qd[28]+OM127;
  OM228 = OM227*C28+OM327*S28;
  OM328 = -(OM227*S28-OM327*C28);

// = = Block_0_1_0_2_0_6 = = 
 
// Forward Kinematics 

  OM129 = OM121*C29-OM321*S29;
  OM229 = qd[29]+OM221;
  OM329 = OM121*S29+OM321*C29;
  OpF129 = C29*(OpF121-qd[29]*OM321)-S29*(OpF320+qd[29]*OM121);
  OpF329 = C29*(OpF320+qd[29]*OM121)+S29*(OpF121-qd[29]*OM321);
  BS229 = OM129*OM229;
  BS329 = OM129*OM329;
  BS529 = -(OM129*OM129+OM329*OM329);
  BS629 = OM229*OM329;
  BeF229 = BS229-OpF329;
  BeF829 = BS629+OpF129;
  AlF129 = C29*(AlF121+BS121*s->dpt[1][37]+BeF221*s->dpt[2][37]+BeF321*s->dpt[3][37])-S29*(AlF321+BS921*s->dpt[3][37]+
 BeF721*s->dpt[1][37]+BeF821*s->dpt[2][37]);
  AlF229 = AlF221+BS521*s->dpt[2][37]+BeF421*s->dpt[1][37]+BeF621*s->dpt[3][37];
  AlF329 = C29*(AlF321+BS921*s->dpt[3][37]+BeF721*s->dpt[1][37]+BeF821*s->dpt[2][37])+S29*(AlF121+BS121*s->dpt[1][37]+
 BeF221*s->dpt[2][37]+BeF321*s->dpt[3][37]);
  AlM129_1 = AlM121_1*C29-AlM320_1*S29;
  AlM329_1 = AlM121_1*S29+AlM320_1*C29;
  AlM129_2 = AlM121_2*C29-AlM320_2*S29;
  AlM329_2 = AlM121_2*S29+AlM320_2*C29;
  AlM129_3 = AlM121_3*C29-AlM320_3*S29;
  AlM329_3 = AlM121_3*S29+AlM320_3*C29;
  OpM129_4 = OpM121_4*C29-OpM320_4*S29;
  OpM329_4 = OpM121_4*S29+OpM320_4*C29;
  AlM129_4 = C29*(AlM121_4+OpM221_4*s->dpt[3][37]-OpM320_4*s->dpt[2][37])-S29*(AlM320_4+OpM121_4*s->dpt[2][37]-OpM221_4*
 s->dpt[1][37]);
  AlM229_4 = AlM221_4-OpM121_4*s->dpt[3][37]+OpM320_4*s->dpt[1][37];
  AlM329_4 = C29*(AlM320_4+OpM121_4*s->dpt[2][37]-OpM221_4*s->dpt[1][37])+S29*(AlM121_4+OpM221_4*s->dpt[3][37]-OpM320_4*
 s->dpt[2][37]);
  OpM129_5 = OpM121_5*C29-OpM320_5*S29;
  OpM329_5 = OpM121_5*S29+OpM320_5*C29;
  AlM129_5 = C29*(AlM121_5+OpM221_5*s->dpt[3][37]-OpM320_5*s->dpt[2][37])-S29*(AlM320_5+OpM121_5*s->dpt[2][37]-OpM221_5*
 s->dpt[1][37]);
  AlM229_5 = AlM221_5-OpM121_5*s->dpt[3][37]+OpM320_5*s->dpt[1][37];
  AlM329_5 = C29*(AlM320_5+OpM121_5*s->dpt[2][37]-OpM221_5*s->dpt[1][37])+S29*(AlM121_5+OpM221_5*s->dpt[3][37]-OpM320_5*
 s->dpt[2][37]);
  OpM129_6 = OpM121_6*C29-OpM320_6*S29;
  OpM329_6 = OpM121_6*S29+OpM320_6*C29;
  AlM129_6 = C29*(AlM121_6+OpM221_6*s->dpt[3][37]-OpM320_6*s->dpt[2][37])-S29*(AlM320_6+OpM121_6*s->dpt[2][37]-OpM221_6*
 s->dpt[1][37]);
  AlM229_6 = AlM221_6-OpM121_6*s->dpt[3][37]+OpM320_6*s->dpt[1][37];
  AlM329_6 = C29*(AlM320_6+OpM121_6*s->dpt[2][37]-OpM221_6*s->dpt[1][37])+S29*(AlM121_6+OpM221_6*s->dpt[3][37]-OpM320_6*
 s->dpt[2][37]);
  OpM129_19 = OpM121_19*C29-S20*S29;
  OpM329_19 = OpM121_19*S29+S20*C29;
  AlM129_19 = C29*(AlM121_19+OpM221_19*s->dpt[3][37]-s->dpt[2][37]*S20)-S29*(OpM121_19*s->dpt[2][37]-OpM221_19*
 s->dpt[1][37]);
  AlM229_19 = AlM221_19-OpM121_19*s->dpt[3][37]+s->dpt[1][37]*S20;
  AlM329_19 = C29*(OpM121_19*s->dpt[2][37]-OpM221_19*s->dpt[1][37])+S29*(AlM121_19+OpM221_19*s->dpt[3][37]-s->dpt[2][37]
 *S20);
  OpM129_20 = S21*C29;
  OpM329_20 = S21*S29;
  AlM129_20 = C21*C29*(s->dpt[3][32]+s->dpt[3][37])+S29*(s->dpt[1][37]*C21-s->dpt[2][37]*S21);
  AlM229_20 = AlM221_20-s->dpt[3][37]*S21;
  AlM329_20 = C21*S29*(s->dpt[3][32]+s->dpt[3][37])-C29*(s->dpt[1][37]*C21-s->dpt[2][37]*S21);
  AlM129_21 = -s->dpt[2][37]*C29;
  OM130 = qd[30]+OM129;
  OM230 = OM229*C30+OM329*S30;
  OM330 = -(OM229*S30-OM329*C30);
  OpF230 = C30*(OpF221+qd[30]*OM329)+S30*(OpF329-qd[30]*OM229);
  OpF330 = C30*(OpF329-qd[30]*OM229)-S30*(OpF221+qd[30]*OM329);
  BS230 = OM130*OM230;
  BS330 = OM130*OM330;
  BS530 = -(OM130*OM130+OM330*OM330);
  BS630 = OM230*OM330;
  BeF230 = BS230-OpF330;
  BeF830 = BS630+OpF129;
  AlF130 = AlF129+BeF229*s->dpt[2][51];
  AlF230 = C30*(AlF229+BS529*s->dpt[2][51])+S30*(AlF329+BeF829*s->dpt[2][51]);
  AlF330 = C30*(AlF329+BeF829*s->dpt[2][51])-S30*(AlF229+BS529*s->dpt[2][51]);
  AlM230_1 = AlM221_1*C30+AlM329_1*S30;
  AlM330_1 = -(AlM221_1*S30-AlM329_1*C30);
  AlM230_2 = AlM221_2*C30+AlM329_2*S30;
  AlM330_2 = -(AlM221_2*S30-AlM329_2*C30);
  AlM230_3 = AlM221_3*C30+AlM329_3*S30;
  AlM330_3 = -(AlM221_3*S30-AlM329_3*C30);
  OpM230_4 = OpM221_4*C30+OpM329_4*S30;
  OpM330_4 = -(OpM221_4*S30-OpM329_4*C30);
  AlM130_4 = AlM129_4-OpM329_4*s->dpt[2][51];
  AlM230_4 = AlM229_4*C30+S30*(AlM329_4+OpM129_4*s->dpt[2][51]);
  AlM330_4 = -(AlM229_4*S30-C30*(AlM329_4+OpM129_4*s->dpt[2][51]));
  OpM230_5 = OpM221_5*C30+OpM329_5*S30;
  OpM330_5 = -(OpM221_5*S30-OpM329_5*C30);
  AlM130_5 = AlM129_5-OpM329_5*s->dpt[2][51];
  AlM230_5 = AlM229_5*C30+S30*(AlM329_5+OpM129_5*s->dpt[2][51]);
  AlM330_5 = -(AlM229_5*S30-C30*(AlM329_5+OpM129_5*s->dpt[2][51]));
  OpM230_6 = OpM221_6*C30+OpM329_6*S30;
  OpM330_6 = -(OpM221_6*S30-OpM329_6*C30);
  AlM130_6 = AlM129_6-OpM329_6*s->dpt[2][51];
  AlM230_6 = AlM229_6*C30+S30*(AlM329_6+OpM129_6*s->dpt[2][51]);
  AlM330_6 = -(AlM229_6*S30-C30*(AlM329_6+OpM129_6*s->dpt[2][51]));
  OpM230_19 = OpM221_19*C30+OpM329_19*S30;
  OpM330_19 = -(OpM221_19*S30-OpM329_19*C30);
  AlM130_19 = AlM129_19-OpM329_19*s->dpt[2][51];
  AlM230_19 = AlM229_19*C30+S30*(AlM329_19+OpM129_19*s->dpt[2][51]);
  AlM330_19 = -(AlM229_19*S30-C30*(AlM329_19+OpM129_19*s->dpt[2][51]));
  OpM230_20 = OpM329_20*S30+C21*C30;
  OpM330_20 = OpM329_20*C30-C21*S30;
  AlM130_20 = AlM129_20-OpM329_20*s->dpt[2][51];
  AlM230_20 = AlM229_20*C30+S30*(AlM329_20+OpM129_20*s->dpt[2][51]);
  AlM330_20 = -(AlM229_20*S30-C30*(AlM329_20+OpM129_20*s->dpt[2][51]));
  OpM230_21 = C29*S30;
  OpM330_21 = C29*C30;
  AlM130_21 = AlM129_21-s->dpt[2][51]*C29;
  AlM230_21 = s->dpt[1][37]*C30-S29*S30*(s->dpt[2][37]+s->dpt[2][51]);
  AlM330_21 = -(s->dpt[1][37]*S30+S29*C30*(s->dpt[2][37]+s->dpt[2][51]));
  OM131 = OM130*C31-OM330*S31;
  OM231 = qd[31]+OM230;
  OM331 = OM130*S31+OM330*C31;
  OpF131 = C31*(OpF129-qd[31]*OM330)-S31*(OpF330+qd[31]*OM130);
  OpF331 = C31*(OpF330+qd[31]*OM130)+S31*(OpF129-qd[31]*OM330);
  BS131 = -(OM231*OM231+OM331*OM331);
  BS231 = OM131*OM231;
  BS331 = OM131*OM331;
  BS531 = -(OM131*OM131+OM331*OM331);
  BS631 = OM231*OM331;
  BeF231 = BS231-OpF331;
  BeF431 = BS231+OpF331;
  BeF731 = BS331-OpF230;
  BeF831 = BS631+OpF131;
  AlF131 = C31*(AlF130+BeF230*s->dpt[2][53])-S31*(AlF330+BeF830*s->dpt[2][53]);
  AlF231 = AlF230+BS530*s->dpt[2][53];
  AlF331 = C31*(AlF330+BeF830*s->dpt[2][53])+S31*(AlF130+BeF230*s->dpt[2][53]);
  AlM131_1 = AlM129_1*C31-AlM330_1*S31;
  AlM331_1 = AlM129_1*S31+AlM330_1*C31;
  AlM131_2 = AlM129_2*C31-AlM330_2*S31;
  AlM331_2 = AlM129_2*S31+AlM330_2*C31;
  AlM131_3 = AlM129_3*C31-AlM330_3*S31;
  AlM331_3 = AlM129_3*S31+AlM330_3*C31;
  OpM131_4 = OpM129_4*C31-OpM330_4*S31;
  OpM331_4 = OpM129_4*S31+OpM330_4*C31;
  AlM131_4 = C31*(AlM130_4-OpM330_4*s->dpt[2][53])-S31*(AlM330_4+OpM129_4*s->dpt[2][53]);
  AlM331_4 = C31*(AlM330_4+OpM129_4*s->dpt[2][53])+S31*(AlM130_4-OpM330_4*s->dpt[2][53]);
  OpM131_5 = OpM129_5*C31-OpM330_5*S31;
  OpM331_5 = OpM129_5*S31+OpM330_5*C31;
  AlM131_5 = C31*(AlM130_5-OpM330_5*s->dpt[2][53])-S31*(AlM330_5+OpM129_5*s->dpt[2][53]);
  AlM331_5 = C31*(AlM330_5+OpM129_5*s->dpt[2][53])+S31*(AlM130_5-OpM330_5*s->dpt[2][53]);
  OpM131_6 = OpM129_6*C31-OpM330_6*S31;
  OpM331_6 = OpM129_6*S31+OpM330_6*C31;
  AlM131_6 = C31*(AlM130_6-OpM330_6*s->dpt[2][53])-S31*(AlM330_6+OpM129_6*s->dpt[2][53]);
  AlM331_6 = C31*(AlM330_6+OpM129_6*s->dpt[2][53])+S31*(AlM130_6-OpM330_6*s->dpt[2][53]);
  OpM131_19 = OpM129_19*C31-OpM330_19*S31;
  OpM331_19 = OpM129_19*S31+OpM330_19*C31;
  AlM131_19 = C31*(AlM130_19-OpM330_19*s->dpt[2][53])-S31*(AlM330_19+OpM129_19*s->dpt[2][53]);
  AlM331_19 = C31*(AlM330_19+OpM129_19*s->dpt[2][53])+S31*(AlM130_19-OpM330_19*s->dpt[2][53]);
  OpM131_20 = OpM129_20*C31-OpM330_20*S31;
  OpM331_20 = OpM129_20*S31+OpM330_20*C31;
  AlM131_20 = C31*(AlM130_20-OpM330_20*s->dpt[2][53])-S31*(AlM330_20+OpM129_20*s->dpt[2][53]);
  AlM331_20 = C31*(AlM330_20+OpM129_20*s->dpt[2][53])+S31*(AlM130_20-OpM330_20*s->dpt[2][53]);
  OpM131_21 = -(OpM330_21*S31+S29*C31);
  OpM331_21 = OpM330_21*C31-S29*S31;
  AlM131_21 = C31*(AlM130_21-OpM330_21*s->dpt[2][53])-S31*(AlM330_21-s->dpt[2][53]*S29);
  AlM331_21 = C31*(AlM330_21-s->dpt[2][53]*S29)+S31*(AlM130_21-OpM330_21*s->dpt[2][53]);
  OpM131_29 = S30*S31;
  OpM331_29 = -S30*C31;
  AlM131_29 = s->dpt[2][53]*S30*C31;
  AlM331_29 = s->dpt[2][53]*S30*S31;
  OM132 = OM131*C32+OM231*S32;
  OM232 = -(OM131*S32-OM231*C32);
  OM332 = qd[32]+OM331;
  OpF132 = C32*(OpF131+qd[32]*OM231)+S32*(OpF230-qd[32]*OM131);
  OpF232 = C32*(OpF230-qd[32]*OM131)-S32*(OpF131+qd[32]*OM231);
  BS132 = -(OM232*OM232+OM332*OM332);
  BS232 = OM132*OM232;
  BeF732 = OM132*OM332-OpF232;
  AlF132 = C32*(AlF131+BS131*s->dpt[1][55]+BeF231*s->dpt[2][55])+S32*(AlF231+BS531*s->dpt[2][55]+BeF431*s->dpt[1][55]);
  AlF232 = C32*(AlF231+BS531*s->dpt[2][55]+BeF431*s->dpt[1][55])-S32*(AlF131+BS131*s->dpt[1][55]+BeF231*s->dpt[2][55]);
  AlF332 = AlF331+BeF731*s->dpt[1][55]+BeF831*s->dpt[2][55];
  AlM132_1 = AlM131_1*C32+AlM230_1*S32;
  AlM232_1 = -(AlM131_1*S32-AlM230_1*C32);
  AlM132_2 = AlM131_2*C32+AlM230_2*S32;
  AlM232_2 = -(AlM131_2*S32-AlM230_2*C32);
  AlM132_3 = AlM131_3*C32+AlM230_3*S32;
  AlM232_3 = -(AlM131_3*S32-AlM230_3*C32);
  OpM132_4 = OpM131_4*C32+OpM230_4*S32;
  OpM232_4 = -(OpM131_4*S32-OpM230_4*C32);
  AlM132_4 = C32*(AlM131_4-OpM331_4*s->dpt[2][55])+S32*(AlM230_4+OpM331_4*s->dpt[1][55]);
  AlM232_4 = C32*(AlM230_4+OpM331_4*s->dpt[1][55])-S32*(AlM131_4-OpM331_4*s->dpt[2][55]);
  AlM332_4 = AlM331_4+OpM131_4*s->dpt[2][55]-OpM230_4*s->dpt[1][55];
  OpM132_5 = OpM131_5*C32+OpM230_5*S32;
  OpM232_5 = -(OpM131_5*S32-OpM230_5*C32);
  AlM132_5 = C32*(AlM131_5-OpM331_5*s->dpt[2][55])+S32*(AlM230_5+OpM331_5*s->dpt[1][55]);
  AlM232_5 = C32*(AlM230_5+OpM331_5*s->dpt[1][55])-S32*(AlM131_5-OpM331_5*s->dpt[2][55]);
  AlM332_5 = AlM331_5+OpM131_5*s->dpt[2][55]-OpM230_5*s->dpt[1][55];
  OpM132_6 = OpM131_6*C32+OpM230_6*S32;
  OpM232_6 = -(OpM131_6*S32-OpM230_6*C32);
  AlM132_6 = C32*(AlM131_6-OpM331_6*s->dpt[2][55])+S32*(AlM230_6+OpM331_6*s->dpt[1][55]);
  AlM232_6 = C32*(AlM230_6+OpM331_6*s->dpt[1][55])-S32*(AlM131_6-OpM331_6*s->dpt[2][55]);
  AlM332_6 = AlM331_6+OpM131_6*s->dpt[2][55]-OpM230_6*s->dpt[1][55];
  OpM132_19 = OpM131_19*C32+OpM230_19*S32;
  OpM232_19 = -(OpM131_19*S32-OpM230_19*C32);
  AlM132_19 = C32*(AlM131_19-OpM331_19*s->dpt[2][55])+S32*(AlM230_19+OpM331_19*s->dpt[1][55]);
  AlM232_19 = C32*(AlM230_19+OpM331_19*s->dpt[1][55])-S32*(AlM131_19-OpM331_19*s->dpt[2][55]);
  AlM332_19 = AlM331_19+OpM131_19*s->dpt[2][55]-OpM230_19*s->dpt[1][55];
  OpM132_20 = OpM131_20*C32+OpM230_20*S32;
  OpM232_20 = -(OpM131_20*S32-OpM230_20*C32);
  AlM132_20 = C32*(AlM131_20-OpM331_20*s->dpt[2][55])+S32*(AlM230_20+OpM331_20*s->dpt[1][55]);
  AlM232_20 = C32*(AlM230_20+OpM331_20*s->dpt[1][55])-S32*(AlM131_20-OpM331_20*s->dpt[2][55]);
  AlM332_20 = AlM331_20+OpM131_20*s->dpt[2][55]-OpM230_20*s->dpt[1][55];
  OpM132_21 = OpM131_21*C32+OpM230_21*S32;
  OpM232_21 = -(OpM131_21*S32-OpM230_21*C32);
  AlM132_21 = C32*(AlM131_21-OpM331_21*s->dpt[2][55])+S32*(AlM230_21+OpM331_21*s->dpt[1][55]);
  AlM232_21 = C32*(AlM230_21+OpM331_21*s->dpt[1][55])-S32*(AlM131_21-OpM331_21*s->dpt[2][55]);
  AlM332_21 = AlM331_21+OpM131_21*s->dpt[2][55]-OpM230_21*s->dpt[1][55];
  OpM132_29 = OpM131_29*C32+C30*S32;
  OpM232_29 = -(OpM131_29*S32-C30*C32);
  AlM132_29 = OpM331_29*s->dpt[1][55]*S32+C32*(AlM131_29-OpM331_29*s->dpt[2][55]);
  AlM232_29 = OpM331_29*s->dpt[1][55]*C32-S32*(AlM131_29-OpM331_29*s->dpt[2][55]);
  AlM332_29 = AlM331_29+OpM131_29*s->dpt[2][55]-s->dpt[1][55]*C30;
  OpM132_30 = C31*C32;
  OpM232_30 = -C31*S32;
  AlM132_30 = S31*(s->dpt[1][55]*S32-C32*(s->dpt[2][53]+s->dpt[2][55]));
  AlM232_30 = S31*(s->dpt[1][55]*C32+S32*(s->dpt[2][53]+s->dpt[2][55]));
  AlM332_30 = C31*(s->dpt[2][53]+s->dpt[2][55]);
  OM133 = OM132*C33-OM332*S33;
  OM233 = qd[33]+OM232;
  OM333 = OM132*S33+OM332*C33;
  OpF133 = C33*(OpF132-qd[33]*OM332)-S33*(OpF331+qd[33]*OM132);
  OpF333 = C33*(OpF331+qd[33]*OM132)+S33*(OpF132-qd[33]*OM332);
  BS533 = -(OM133*OM133+OM333*OM333);
  BeF233 = OM133*OM233-OpF333;
  BeF833 = OpF133+OM233*OM333;
  AlF133 = C33*(AlF132+BS132*s->dpt[1][57])-S33*(AlF332+BeF732*s->dpt[1][57]);
  AlF233 = AlF232+s->dpt[1][57]*(BS232+OpF331);
  AlF333 = C33*(AlF332+BeF732*s->dpt[1][57])+S33*(AlF132+BS132*s->dpt[1][57]);
  AlM133_1 = AlM132_1*C33-AlM331_1*S33;
  AlM333_1 = AlM132_1*S33+AlM331_1*C33;
  AlM133_2 = AlM132_2*C33-AlM331_2*S33;
  AlM333_2 = AlM132_2*S33+AlM331_2*C33;
  AlM133_3 = AlM132_3*C33-AlM331_3*S33;
  AlM333_3 = AlM132_3*S33+AlM331_3*C33;
  OpM133_4 = OpM132_4*C33-OpM331_4*S33;
  OpM333_4 = OpM132_4*S33+OpM331_4*C33;
  AlM133_4 = AlM132_4*C33-S33*(AlM332_4-OpM232_4*s->dpt[1][57]);
  AlM233_4 = AlM232_4+OpM331_4*s->dpt[1][57];
  AlM333_4 = AlM132_4*S33+C33*(AlM332_4-OpM232_4*s->dpt[1][57]);
  OpM133_5 = OpM132_5*C33-OpM331_5*S33;
  OpM333_5 = OpM132_5*S33+OpM331_5*C33;
  AlM133_5 = AlM132_5*C33-S33*(AlM332_5-OpM232_5*s->dpt[1][57]);
  AlM233_5 = AlM232_5+OpM331_5*s->dpt[1][57];
  AlM333_5 = AlM132_5*S33+C33*(AlM332_5-OpM232_5*s->dpt[1][57]);
  OpM133_6 = OpM132_6*C33-OpM331_6*S33;
  OpM333_6 = OpM132_6*S33+OpM331_6*C33;
  AlM133_6 = AlM132_6*C33-S33*(AlM332_6-OpM232_6*s->dpt[1][57]);
  AlM233_6 = AlM232_6+OpM331_6*s->dpt[1][57];
  AlM333_6 = AlM132_6*S33+C33*(AlM332_6-OpM232_6*s->dpt[1][57]);
  OpM133_19 = OpM132_19*C33-OpM331_19*S33;
  OpM333_19 = OpM132_19*S33+OpM331_19*C33;
  AlM133_19 = AlM132_19*C33-S33*(AlM332_19-OpM232_19*s->dpt[1][57]);
  AlM233_19 = AlM232_19+OpM331_19*s->dpt[1][57];
  AlM333_19 = AlM132_19*S33+C33*(AlM332_19-OpM232_19*s->dpt[1][57]);
  OpM133_20 = OpM132_20*C33-OpM331_20*S33;
  OpM333_20 = OpM132_20*S33+OpM331_20*C33;
  AlM133_20 = AlM132_20*C33-S33*(AlM332_20-OpM232_20*s->dpt[1][57]);
  AlM233_20 = AlM232_20+OpM331_20*s->dpt[1][57];
  AlM333_20 = AlM132_20*S33+C33*(AlM332_20-OpM232_20*s->dpt[1][57]);
  OpM133_21 = OpM132_21*C33-OpM331_21*S33;
  OpM333_21 = OpM132_21*S33+OpM331_21*C33;
  AlM133_21 = AlM132_21*C33-S33*(AlM332_21-OpM232_21*s->dpt[1][57]);
  AlM233_21 = AlM232_21+OpM331_21*s->dpt[1][57];
  AlM333_21 = AlM132_21*S33+C33*(AlM332_21-OpM232_21*s->dpt[1][57]);
  OpM133_29 = OpM132_29*C33-OpM331_29*S33;
  OpM333_29 = OpM132_29*S33+OpM331_29*C33;
  AlM133_29 = AlM132_29*C33-S33*(AlM332_29-OpM232_29*s->dpt[1][57]);
  AlM233_29 = AlM232_29+OpM331_29*s->dpt[1][57];
  AlM333_29 = AlM132_29*S33+C33*(AlM332_29-OpM232_29*s->dpt[1][57]);
  OpM133_30 = OpM132_30*C33-S31*S33;
  OpM333_30 = OpM132_30*S33+S31*C33;
  AlM133_30 = AlM132_30*C33-S33*(AlM332_30-OpM232_30*s->dpt[1][57]);
  AlM233_30 = AlM232_30+s->dpt[1][57]*S31;
  AlM333_30 = AlM132_30*S33+C33*(AlM332_30-OpM232_30*s->dpt[1][57]);
  OpM133_31 = S32*C33;
  OpM333_31 = S32*S33;
  AlM133_31 = S33*(s->dpt[1][55]+s->dpt[1][57]*C32);
  AlM333_31 = -C33*(s->dpt[1][55]+s->dpt[1][57]*C32);
  OM134 = OM133*C34+OM233*S34;
  OM234 = -(OM133*S34-OM233*C34);
  OM334 = qd[34]+OM333;
  OpF134 = C34*(OpF133+qd[34]*OM233)+S34*(OpF232-qd[34]*OM133);
  OpF234 = C34*(OpF232-qd[34]*OM133)-S34*(OpF133+qd[34]*OM233);
  AlF134 = C34*(AlF133+BeF233*s->dpt[2][58])+S34*(AlF233+BS533*s->dpt[2][58]);
  AlF234 = C34*(AlF233+BS533*s->dpt[2][58])-S34*(AlF133+BeF233*s->dpt[2][58]);
  AlF334 = AlF333+BeF833*s->dpt[2][58];
  AlM134_1 = AlM133_1*C34+AlM232_1*S34;
  AlM234_1 = -(AlM133_1*S34-AlM232_1*C34);
  AlM134_2 = AlM133_2*C34+AlM232_2*S34;
  AlM234_2 = -(AlM133_2*S34-AlM232_2*C34);
  AlM134_3 = AlM133_3*C34+AlM232_3*S34;
  AlM234_3 = -(AlM133_3*S34-AlM232_3*C34);
  OpM134_4 = OpM133_4*C34+OpM232_4*S34;
  OpM234_4 = -(OpM133_4*S34-OpM232_4*C34);
  AlM134_4 = AlM233_4*S34+C34*(AlM133_4-OpM333_4*s->dpt[2][58]);
  AlM234_4 = AlM233_4*C34-S34*(AlM133_4-OpM333_4*s->dpt[2][58]);
  AlM334_4 = AlM333_4+OpM133_4*s->dpt[2][58];
  OpM134_5 = OpM133_5*C34+OpM232_5*S34;
  OpM234_5 = -(OpM133_5*S34-OpM232_5*C34);
  AlM134_5 = AlM233_5*S34+C34*(AlM133_5-OpM333_5*s->dpt[2][58]);
  AlM234_5 = AlM233_5*C34-S34*(AlM133_5-OpM333_5*s->dpt[2][58]);
  AlM334_5 = AlM333_5+OpM133_5*s->dpt[2][58];
  OpM134_6 = OpM133_6*C34+OpM232_6*S34;
  OpM234_6 = -(OpM133_6*S34-OpM232_6*C34);
  AlM134_6 = AlM233_6*S34+C34*(AlM133_6-OpM333_6*s->dpt[2][58]);
  AlM234_6 = AlM233_6*C34-S34*(AlM133_6-OpM333_6*s->dpt[2][58]);
  AlM334_6 = AlM333_6+OpM133_6*s->dpt[2][58];
  OpM134_19 = OpM133_19*C34+OpM232_19*S34;
  OpM234_19 = -(OpM133_19*S34-OpM232_19*C34);
  AlM134_19 = AlM233_19*S34+C34*(AlM133_19-OpM333_19*s->dpt[2][58]);
  AlM234_19 = AlM233_19*C34-S34*(AlM133_19-OpM333_19*s->dpt[2][58]);
  AlM334_19 = AlM333_19+OpM133_19*s->dpt[2][58];
  OpM134_20 = OpM133_20*C34+OpM232_20*S34;
  OpM234_20 = -(OpM133_20*S34-OpM232_20*C34);
  AlM134_20 = AlM233_20*S34+C34*(AlM133_20-OpM333_20*s->dpt[2][58]);
  AlM234_20 = AlM233_20*C34-S34*(AlM133_20-OpM333_20*s->dpt[2][58]);
  AlM334_20 = AlM333_20+OpM133_20*s->dpt[2][58];
  OpM134_21 = OpM133_21*C34+OpM232_21*S34;
  OpM234_21 = -(OpM133_21*S34-OpM232_21*C34);
  AlM134_21 = AlM233_21*S34+C34*(AlM133_21-OpM333_21*s->dpt[2][58]);
  AlM234_21 = AlM233_21*C34-S34*(AlM133_21-OpM333_21*s->dpt[2][58]);
  AlM334_21 = AlM333_21+OpM133_21*s->dpt[2][58];
  OpM134_29 = OpM133_29*C34+OpM232_29*S34;
  OpM234_29 = -(OpM133_29*S34-OpM232_29*C34);
  AlM134_29 = AlM233_29*S34+C34*(AlM133_29-OpM333_29*s->dpt[2][58]);
  AlM234_29 = AlM233_29*C34-S34*(AlM133_29-OpM333_29*s->dpt[2][58]);
  AlM334_29 = AlM333_29+OpM133_29*s->dpt[2][58];
  OpM134_30 = OpM133_30*C34+OpM232_30*S34;
  OpM234_30 = -(OpM133_30*S34-OpM232_30*C34);
  AlM134_30 = AlM233_30*S34+C34*(AlM133_30-OpM333_30*s->dpt[2][58]);
  AlM234_30 = AlM233_30*C34-S34*(AlM133_30-OpM333_30*s->dpt[2][58]);
  AlM334_30 = AlM333_30+OpM133_30*s->dpt[2][58];
  OpM134_31 = OpM133_31*C34+C32*S34;
  OpM234_31 = -(OpM133_31*S34-C32*C34);
  AlM134_31 = C34*(AlM133_31-OpM333_31*s->dpt[2][58]);
  AlM234_31 = -S34*(AlM133_31-OpM333_31*s->dpt[2][58]);
  AlM334_31 = AlM333_31+OpM133_31*s->dpt[2][58];
  OpM134_32 = -S33*C34;
  OpM234_32 = S33*S34;
  AlM134_32 = s->dpt[1][57]*S34-s->dpt[2][58]*C33*C34;
  AlM234_32 = s->dpt[1][57]*C34+s->dpt[2][58]*C33*S34;
  AlM334_32 = -s->dpt[2][58]*S33;
  OM135 = qd[35]+OM134;
  OM235 = OM234*C35+OM334*S35;
  OM335 = -(OM234*S35-OM334*C35);

// = = Block_0_2_0_1_0_5 = = 
 
// Backward Dynamics 

  FA228 = -(s->frc[2][28]-s->m[28]*(AlF227*C28+AlF327*S28));
  FA328 = -(s->frc[3][28]+s->m[28]*(AlF227*S28-AlF327*C28));
  CF128 = -(s->trq[1][28]-s->In[1][28]*OpF127+OM228*OM328*(s->In[5][28]-s->In[9][28]));
  CF228 = -(s->trq[2][28]-s->In[5][28]*(C28*(OpF227+qd[28]*OM327)+S28*(OpF326-qd[28]*OM227))-OM128*OM328*(s->In[1][28]-
 s->In[9][28]));
  CF328 = -(s->trq[3][28]-s->In[9][28]*(C28*(OpF326-qd[28]*OM227)-S28*(OpF227+qd[28]*OM327))+OM128*OM228*(s->In[1][28]-
 s->In[5][28]));
  FB228_1 = s->m[28]*(AlM227_1*C28+AlM326_1*S28);
  FB328_1 = -s->m[28]*(AlM227_1*S28-AlM326_1*C28);
  FB228_2 = s->m[28]*(AlM227_2*C28+AlM326_2*S28);
  FB328_2 = -s->m[28]*(AlM227_2*S28-AlM326_2*C28);
  FB228_3 = s->m[28]*(AlM227_3*C28+AlM326_3*S28);
  FB328_3 = -s->m[28]*(AlM227_3*S28-AlM326_3*C28);
  FB228_4 = s->m[28]*(AlM227_4*C28+AlM327_4*S28);
  FB328_4 = -s->m[28]*(AlM227_4*S28-AlM327_4*C28);
  CM128_4 = s->In[1][28]*OpM127_4;
  CM228_4 = s->In[5][28]*(OpM227_4*C28+OpM326_4*S28);
  CM328_4 = -s->In[9][28]*(OpM227_4*S28-OpM326_4*C28);
  FB228_5 = s->m[28]*(AlM227_5*C28+AlM327_5*S28);
  FB328_5 = -s->m[28]*(AlM227_5*S28-AlM327_5*C28);
  CM128_5 = s->In[1][28]*OpM127_5;
  CM228_5 = s->In[5][28]*(OpM227_5*C28+OpM326_5*S28);
  CM328_5 = -s->In[9][28]*(OpM227_5*S28-OpM326_5*C28);
  FB228_6 = s->m[28]*(AlM227_6*C28+AlM327_6*S28);
  FB328_6 = -s->m[28]*(AlM227_6*S28-AlM327_6*C28);
  CM128_6 = s->In[1][28]*OpM127_6;
  CM228_6 = s->In[5][28]*(OpM227_6*C28+OpM326_6*S28);
  CM328_6 = -s->In[9][28]*(OpM227_6*S28-OpM326_6*C28);
  FB228_19 = s->m[28]*(AlM227_19*C28+AlM327_19*S28);
  FB328_19 = -s->m[28]*(AlM227_19*S28-AlM327_19*C28);
  CM128_19 = s->In[1][28]*OpM127_19;
  CM228_19 = s->In[5][28]*(OpM227_19*C28+OpM326_19*S28);
  CM328_19 = -s->In[9][28]*(OpM227_19*S28-OpM326_19*C28);
  FB228_20 = s->m[28]*(AlM227_20*C28+AlM327_20*S28);
  FB328_20 = -s->m[28]*(AlM227_20*S28-AlM327_20*C28);
  CM128_20 = s->In[1][28]*OpM127_20;
  CM228_20 = s->In[5][28]*(OpM227_20*C28+OpM326_20*S28);
  CM328_20 = -s->In[9][28]*(OpM227_20*S28-OpM326_20*C28);
  FB228_21 = s->m[28]*(AlM227_21*C28+AlM327_21*S28);
  FB328_21 = -s->m[28]*(AlM227_21*S28-AlM327_21*C28);
  CM128_21 = s->In[1][28]*OpM127_21;
  CM228_21 = s->In[5][28]*(OpM227_21*C28+OpM326_21*S28);
  CM328_21 = -s->In[9][28]*(OpM227_21*S28-OpM326_21*C28);
  FB228_22 = s->m[28]*(AlM227_22*C28+AlM327_22*S28);
  FB328_22 = -s->m[28]*(AlM227_22*S28-AlM327_22*C28);
  CM128_22 = s->In[1][28]*OpM127_22;
  CM228_22 = s->In[5][28]*(OpM227_22*C28+OpM326_22*S28);
  CM328_22 = -s->In[9][28]*(OpM227_22*S28-OpM326_22*C28);
  FB228_23 = s->m[28]*(AlM227_23*C28+AlM327_23*S28);
  FB328_23 = -s->m[28]*(AlM227_23*S28-AlM327_23*C28);
  CM128_23 = s->In[1][28]*OpM127_23;
  CM228_23 = s->In[5][28]*(OpM227_23*C28+OpM326_23*S28);
  CM328_23 = -s->In[9][28]*(OpM227_23*S28-OpM326_23*C28);
  FB228_24 = s->m[28]*(AlM227_24*C28+AlM327_24*S28);
  FB328_24 = -s->m[28]*(AlM227_24*S28-AlM327_24*C28);
  CM128_24 = s->In[1][28]*OpM127_24;
  CM228_24 = s->In[5][28]*(OpM227_24*C28+OpM326_24*S28);
  CM328_24 = -s->In[9][28]*(OpM227_24*S28-OpM326_24*C28);
  FB228_25 = s->m[28]*(AlM227_25*C28+AlM327_25*S28);
  FB328_25 = -s->m[28]*(AlM227_25*S28-AlM327_25*C28);
  CM128_25 = s->In[1][28]*OpM127_25;
  CM228_25 = s->In[5][28]*(OpM227_25*C28+C26*S28);
  CM328_25 = -s->In[9][28]*(OpM227_25*S28-C26*C28);
  CM128_26 = s->In[1][28]*S27;
  CM228_26 = s->In[5][28]*C27*C28;
  CM328_26 = -s->In[9][28]*C27*S28;
  FF127 = -(s->frc[1][27]+s->frc[1][28]-s->m[27]*AlF127-s->m[28]*AlF127);
  FF227 = -(s->frc[2][27]-s->m[27]*AlF227-FA228*C28+FA328*S28);
  FF327 = -(s->frc[3][27]-s->m[27]*AlF327-FA228*S28-FA328*C28);
  CF127 = -(s->trq[1][27]-CF128-s->In[1][27]*OpF127+OM227*OM327*(s->In[5][27]-s->In[9][27]));
  CF227 = -(s->trq[2][27]-s->In[5][27]*OpF227-CF228*C28+CF328*S28-OM127*OM327*(s->In[1][27]-s->In[9][27]));
  CF327 = -(s->trq[3][27]-s->In[9][27]*OpF326-CF228*S28-CF328*C28+OM127*OM227*(s->In[1][27]-s->In[5][27]));
  FM127_1 = AlM127_1*(s->m[27]+s->m[28]);
  FM227_1 = s->m[27]*AlM227_1+FB228_1*C28-FB328_1*S28;
  FM327_1 = s->m[27]*AlM326_1+FB228_1*S28+FB328_1*C28;
  FM127_2 = AlM127_2*(s->m[27]+s->m[28]);
  FM227_2 = s->m[27]*AlM227_2+FB228_2*C28-FB328_2*S28;
  FM327_2 = s->m[27]*AlM326_2+FB228_2*S28+FB328_2*C28;
  FM127_3 = AlM127_3*(s->m[27]+s->m[28]);
  FM227_3 = s->m[27]*AlM227_3+FB228_3*C28-FB328_3*S28;
  FM327_3 = s->m[27]*AlM326_3+FB228_3*S28+FB328_3*C28;
  FM127_4 = AlM127_4*(s->m[27]+s->m[28]);
  FM227_4 = s->m[27]*AlM227_4+FB228_4*C28-FB328_4*S28;
  FM327_4 = s->m[27]*AlM327_4+FB228_4*S28+FB328_4*C28;
  CM127_4 = CM128_4+s->In[1][27]*OpM127_4;
  CM227_4 = s->In[5][27]*OpM227_4+CM228_4*C28-CM328_4*S28;
  CM327_4 = s->In[9][27]*OpM326_4+CM228_4*S28+CM328_4*C28;
  FM127_5 = AlM127_5*(s->m[27]+s->m[28]);
  FM227_5 = s->m[27]*AlM227_5+FB228_5*C28-FB328_5*S28;
  FM327_5 = s->m[27]*AlM327_5+FB228_5*S28+FB328_5*C28;
  CM127_5 = CM128_5+s->In[1][27]*OpM127_5;
  CM227_5 = s->In[5][27]*OpM227_5+CM228_5*C28-CM328_5*S28;
  CM327_5 = s->In[9][27]*OpM326_5+CM228_5*S28+CM328_5*C28;
  FM127_6 = AlM127_6*(s->m[27]+s->m[28]);
  FM227_6 = s->m[27]*AlM227_6+FB228_6*C28-FB328_6*S28;
  FM327_6 = s->m[27]*AlM327_6+FB228_6*S28+FB328_6*C28;
  CM127_6 = CM128_6+s->In[1][27]*OpM127_6;
  CM227_6 = s->In[5][27]*OpM227_6+CM228_6*C28-CM328_6*S28;
  CM327_6 = s->In[9][27]*OpM326_6+CM228_6*S28+CM328_6*C28;
  FM127_19 = AlM127_19*(s->m[27]+s->m[28]);
  FM227_19 = s->m[27]*AlM227_19+FB228_19*C28-FB328_19*S28;
  FM327_19 = s->m[27]*AlM327_19+FB228_19*S28+FB328_19*C28;
  CM127_19 = CM128_19+s->In[1][27]*OpM127_19;
  CM227_19 = s->In[5][27]*OpM227_19+CM228_19*C28-CM328_19*S28;
  CM327_19 = s->In[9][27]*OpM326_19+CM228_19*S28+CM328_19*C28;
  FM127_20 = AlM127_20*(s->m[27]+s->m[28]);
  FM227_20 = s->m[27]*AlM227_20+FB228_20*C28-FB328_20*S28;
  FM327_20 = s->m[27]*AlM327_20+FB228_20*S28+FB328_20*C28;
  CM127_20 = CM128_20+s->In[1][27]*OpM127_20;
  CM227_20 = s->In[5][27]*OpM227_20+CM228_20*C28-CM328_20*S28;
  CM327_20 = s->In[9][27]*OpM326_20+CM228_20*S28+CM328_20*C28;
  FM127_21 = AlM127_21*(s->m[27]+s->m[28]);
  FM227_21 = s->m[27]*AlM227_21+FB228_21*C28-FB328_21*S28;
  FM327_21 = s->m[27]*AlM327_21+FB228_21*S28+FB328_21*C28;
  CM127_21 = CM128_21+s->In[1][27]*OpM127_21;
  CM227_21 = s->In[5][27]*OpM227_21+CM228_21*C28-CM328_21*S28;
  CM327_21 = s->In[9][27]*OpM326_21+CM228_21*S28+CM328_21*C28;
  FM127_22 = AlM127_22*(s->m[27]+s->m[28]);
  FM227_22 = s->m[27]*AlM227_22+FB228_22*C28-FB328_22*S28;
  FM327_22 = s->m[27]*AlM327_22+FB228_22*S28+FB328_22*C28;
  CM127_22 = CM128_22+s->In[1][27]*OpM127_22;
  CM227_22 = s->In[5][27]*OpM227_22+CM228_22*C28-CM328_22*S28;
  CM327_22 = s->In[9][27]*OpM326_22+CM228_22*S28+CM328_22*C28;
  FM127_23 = AlM127_23*(s->m[27]+s->m[28]);
  FM227_23 = s->m[27]*AlM227_23+FB228_23*C28-FB328_23*S28;
  FM327_23 = s->m[27]*AlM327_23+FB228_23*S28+FB328_23*C28;
  CM127_23 = CM128_23+s->In[1][27]*OpM127_23;
  CM227_23 = s->In[5][27]*OpM227_23+CM228_23*C28-CM328_23*S28;
  CM327_23 = s->In[9][27]*OpM326_23+CM228_23*S28+CM328_23*C28;
  FM127_24 = AlM127_24*(s->m[27]+s->m[28]);
  FM227_24 = s->m[27]*AlM227_24+FB228_24*C28-FB328_24*S28;
  FM327_24 = s->m[27]*AlM327_24+FB228_24*S28+FB328_24*C28;
  CM127_24 = CM128_24+s->In[1][27]*OpM127_24;
  CM227_24 = s->In[5][27]*OpM227_24+CM228_24*C28-CM328_24*S28;
  CM327_24 = s->In[9][27]*OpM326_24+CM228_24*S28+CM328_24*C28;
  FM127_25 = AlM127_25*(s->m[27]+s->m[28]);
  FM227_25 = s->m[27]*AlM227_25+FB228_25*C28-FB328_25*S28;
  CM127_25 = CM128_25+s->In[1][27]*OpM127_25;
  CM227_25 = s->In[5][27]*OpM227_25+CM228_25*C28-CM328_25*S28;
  CM327_25 = s->In[9][27]*C26+CM228_25*S28+CM328_25*C28;
  CM2726_328 = CM228_26*S28+CM328_26*C28;
  CM327_27 = s->In[9][27]+s->In[5][28]*S28*S28+s->In[9][28]*C28*C28;
  FA126 = -(s->frc[1][26]-s->m[26]*(AlF126+BeF226*s->l[2][26]));
  FA326 = -(s->frc[3][26]-s->m[26]*(AlF326+BeF826*s->l[2][26]));
  FF126 = FA126+FF127*C27-FF227*S27;
  FF226 = -(s->frc[2][26]-s->m[26]*(AlF226+BS526*s->l[2][26])-FF127*S27-FF227*C27);
  FF326 = FA326+FF327;
  CF126 = -(s->trq[1][26]-s->In[1][26]*OpF126-CF127*C27+CF227*S27-FA326*s->l[2][26]-FF327*s->dpt[2][47]+OM226*OM326*(
 s->In[5][26]-s->In[9][26]));
  CF226 = -(s->trq[2][26]-s->In[5][26]*OpF225-CF127*S27-CF227*C27-OM126*OM326*(s->In[1][26]-s->In[9][26]));
  CF326 = -(s->trq[3][26]-CF327-s->In[9][26]*OpF326+FA126*s->l[2][26]+OM126*OM226*(s->In[1][26]-s->In[5][26])+
 s->dpt[2][47]*(FF127*C27-FF227*S27));
  FB126_1 = s->m[26]*AlM126_1;
  FB326_1 = s->m[26]*AlM326_1;
  FM126_1 = FB126_1+FM127_1*C27-FM227_1*S27;
  FM226_1 = s->m[26]*AlM225_1+FM127_1*S27+FM227_1*C27;
  FM326_1 = FB326_1+FM327_1;
  CM126_1 = FB326_1*s->l[2][26]+FM327_1*s->dpt[2][47];
  CM326_1 = -(FB126_1*s->l[2][26]+s->dpt[2][47]*(FM127_1*C27-FM227_1*S27));
  FB126_2 = s->m[26]*AlM126_2;
  FB326_2 = s->m[26]*AlM326_2;
  FM126_2 = FB126_2+FM127_2*C27-FM227_2*S27;
  FM226_2 = s->m[26]*AlM225_2+FM127_2*S27+FM227_2*C27;
  FM326_2 = FB326_2+FM327_2;
  CM126_2 = FB326_2*s->l[2][26]+FM327_2*s->dpt[2][47];
  CM326_2 = -(FB126_2*s->l[2][26]+s->dpt[2][47]*(FM127_2*C27-FM227_2*S27));
  FB126_3 = s->m[26]*AlM126_3;
  FB326_3 = s->m[26]*AlM326_3;
  FM126_3 = FB126_3+FM127_3*C27-FM227_3*S27;
  FM226_3 = s->m[26]*AlM225_3+FM127_3*S27+FM227_3*C27;
  FM326_3 = FB326_3+FM327_3;
  CM126_3 = FB326_3*s->l[2][26]+FM327_3*s->dpt[2][47];
  CM326_3 = -(FB126_3*s->l[2][26]+s->dpt[2][47]*(FM127_3*C27-FM227_3*S27));
  FB126_4 = s->m[26]*(AlM126_4-OpM326_4*s->l[2][26]);
  FB326_4 = s->m[26]*(AlM326_4+OpM126_4*s->l[2][26]);
  FM126_4 = FB126_4+FM127_4*C27-FM227_4*S27;
  FM226_4 = s->m[26]*AlM226_4+FM127_4*S27+FM227_4*C27;
  FM326_4 = FB326_4+FM327_4;
  CM126_4 = s->In[1][26]*OpM126_4+CM127_4*C27-CM227_4*S27+FB326_4*s->l[2][26]+FM327_4*s->dpt[2][47];
  CM226_4 = s->In[5][26]*OpM225_4+CM127_4*S27+CM227_4*C27;
  CM326_4 = CM327_4+s->In[9][26]*OpM326_4-FB126_4*s->l[2][26]-s->dpt[2][47]*(FM127_4*C27-FM227_4*S27);
  FB126_5 = s->m[26]*(AlM126_5-OpM326_5*s->l[2][26]);
  FB326_5 = s->m[26]*(AlM326_5+OpM126_5*s->l[2][26]);
  FM126_5 = FB126_5+FM127_5*C27-FM227_5*S27;
  FM226_5 = s->m[26]*AlM226_5+FM127_5*S27+FM227_5*C27;
  FM326_5 = FB326_5+FM327_5;
  CM126_5 = s->In[1][26]*OpM126_5+CM127_5*C27-CM227_5*S27+FB326_5*s->l[2][26]+FM327_5*s->dpt[2][47];
  CM226_5 = s->In[5][26]*OpM225_5+CM127_5*S27+CM227_5*C27;
  CM326_5 = CM327_5+s->In[9][26]*OpM326_5-FB126_5*s->l[2][26]-s->dpt[2][47]*(FM127_5*C27-FM227_5*S27);
  FB126_6 = s->m[26]*(AlM126_6-OpM326_6*s->l[2][26]);
  FB326_6 = s->m[26]*(AlM326_6+OpM126_6*s->l[2][26]);
  FM126_6 = FB126_6+FM127_6*C27-FM227_6*S27;
  FM226_6 = s->m[26]*AlM226_6+FM127_6*S27+FM227_6*C27;
  FM326_6 = FB326_6+FM327_6;
  CM126_6 = s->In[1][26]*OpM126_6+CM127_6*C27-CM227_6*S27+FB326_6*s->l[2][26]+FM327_6*s->dpt[2][47];
  CM226_6 = s->In[5][26]*OpM225_6+CM127_6*S27+CM227_6*C27;
  CM326_6 = CM327_6+s->In[9][26]*OpM326_6-FB126_6*s->l[2][26]-s->dpt[2][47]*(FM127_6*C27-FM227_6*S27);
  FB126_19 = s->m[26]*(AlM126_19-OpM326_19*s->l[2][26]);
  FB326_19 = s->m[26]*(AlM326_19+OpM126_19*s->l[2][26]);
  FM126_19 = FB126_19+FM127_19*C27-FM227_19*S27;
  FM226_19 = s->m[26]*AlM226_19+FM127_19*S27+FM227_19*C27;
  FM326_19 = FB326_19+FM327_19;
  CM126_19 = s->In[1][26]*OpM126_19+CM127_19*C27-CM227_19*S27+FB326_19*s->l[2][26]+FM327_19*s->dpt[2][47];
  CM226_19 = s->In[5][26]*OpM225_19+CM127_19*S27+CM227_19*C27;
  CM326_19 = CM327_19+s->In[9][26]*OpM326_19-FB126_19*s->l[2][26]-s->dpt[2][47]*(FM127_19*C27-FM227_19*S27);
  FB126_20 = s->m[26]*(AlM126_20-OpM326_20*s->l[2][26]);
  FB326_20 = s->m[26]*(AlM326_20+OpM126_20*s->l[2][26]);
  FM126_20 = FB126_20+FM127_20*C27-FM227_20*S27;
  FM226_20 = s->m[26]*AlM226_20+FM127_20*S27+FM227_20*C27;
  FM326_20 = FB326_20+FM327_20;
  CM126_20 = s->In[1][26]*OpM126_20+CM127_20*C27-CM227_20*S27+FB326_20*s->l[2][26]+FM327_20*s->dpt[2][47];
  CM226_20 = s->In[5][26]*OpM225_20+CM127_20*S27+CM227_20*C27;
  CM326_20 = CM327_20+s->In[9][26]*OpM326_20-FB126_20*s->l[2][26]-s->dpt[2][47]*(FM127_20*C27-FM227_20*S27);
  FB126_21 = s->m[26]*(AlM126_21-OpM326_21*s->l[2][26]);
  FB326_21 = s->m[26]*(AlM326_21+OpM126_21*s->l[2][26]);
  FM126_21 = FB126_21+FM127_21*C27-FM227_21*S27;
  FM226_21 = s->m[26]*AlM226_21+FM127_21*S27+FM227_21*C27;
  FM326_21 = FB326_21+FM327_21;
  CM126_21 = s->In[1][26]*OpM126_21+CM127_21*C27-CM227_21*S27+FB326_21*s->l[2][26]+FM327_21*s->dpt[2][47];
  CM226_21 = s->In[5][26]*OpM225_21+CM127_21*S27+CM227_21*C27;
  CM326_21 = CM327_21+s->In[9][26]*OpM326_21-FB126_21*s->l[2][26]-s->dpt[2][47]*(FM127_21*C27-FM227_21*S27);
  FB126_22 = s->m[26]*(AlM126_22-OpM326_22*s->l[2][26]);
  FB326_22 = s->m[26]*(AlM326_22+OpM126_22*s->l[2][26]);
  FM126_22 = FB126_22+FM127_22*C27-FM227_22*S27;
  FM226_22 = s->m[26]*AlM226_22+FM127_22*S27+FM227_22*C27;
  FM326_22 = FB326_22+FM327_22;
  CM126_22 = s->In[1][26]*OpM126_22+CM127_22*C27-CM227_22*S27+FB326_22*s->l[2][26]+FM327_22*s->dpt[2][47];
  CM226_22 = s->In[5][26]*OpM225_22+CM127_22*S27+CM227_22*C27;
  CM326_22 = CM327_22+s->In[9][26]*OpM326_22-FB126_22*s->l[2][26]-s->dpt[2][47]*(FM127_22*C27-FM227_22*S27);
  FB126_23 = s->m[26]*(AlM126_23-OpM326_23*s->l[2][26]);
  FB326_23 = s->m[26]*(AlM326_23+OpM126_23*s->l[2][26]);
  FM126_23 = FB126_23+FM127_23*C27-FM227_23*S27;
  FM226_23 = s->m[26]*AlM226_23+FM127_23*S27+FM227_23*C27;
  FM326_23 = FB326_23+FM327_23;
  CM126_23 = s->In[1][26]*OpM126_23+CM127_23*C27-CM227_23*S27+FB326_23*s->l[2][26]+FM327_23*s->dpt[2][47];
  CM226_23 = s->In[5][26]*OpM225_23+CM127_23*S27+CM227_23*C27;
  CM326_23 = CM327_23+s->In[9][26]*OpM326_23-FB126_23*s->l[2][26]-s->dpt[2][47]*(FM127_23*C27-FM227_23*S27);
  FB126_24 = s->m[26]*(AlM126_24-OpM326_24*s->l[2][26]);
  FB326_24 = s->m[26]*(AlM326_24+OpM126_24*s->l[2][26]);
  FM126_24 = FB126_24+FM127_24*C27-FM227_24*S27;
  FM326_24 = FB326_24+FM327_24;
  CM126_24 = s->In[1][26]*OpM126_24+CM127_24*C27-CM227_24*S27+FB326_24*s->l[2][26]+FM327_24*s->dpt[2][47];
  CM226_24 = s->In[5][26]*C25+CM127_24*S27+CM227_24*C27;
  CM326_24 = CM327_24+s->In[9][26]*OpM326_24-FB126_24*s->l[2][26]-s->dpt[2][47]*(FM127_24*C27-FM227_24*S27);
  CM2625_227 = CM127_25*S27+CM227_25*C27;
  CM226_26 = s->In[5][26]+C27*(s->In[5][27]*C27+CM228_26*C28-CM328_26*S28)+S27*S27*(s->In[1][27]+s->In[1][28]);
  FA125 = -(s->frc[1][25]-s->m[25]*(AlF125+BS125*s->l[1][25]+s->l[2][25]*(BS225-OpF324)+s->l[3][25]*(BS325+OpF225)));
  FA225 = -(s->frc[2][25]-s->m[25]*(AlF225+BeF425*s->l[1][25]-s->l[2][25]*(OM125*OM125+OM325*OM325)+s->l[3][25]*(BS625-
 OpF125)));
  FA325 = -(s->frc[3][25]-s->m[25]*(AlF325+BeF725*s->l[1][25]+s->l[2][25]*(BS625+OpF125)-s->l[3][25]*(OM125*OM125+OM225*
 OM225)));
  FF125 = FA125+FF126*C26+FF326*S26;
  FF225 = FA225+FF226;
  FF325 = FA325-FF126*S26+FF326*C26;
  CF125 = -(s->trq[1][25]-s->In[1][25]*OpF125-CF126*C26-CF326*S26+FA225*s->l[3][25]-FA325*s->l[2][25]+OM225*OM325*(
 s->In[5][25]-s->In[9][25]));
  CF225 = -(s->trq[2][25]-CF226-s->In[5][25]*OpF225-FA125*s->l[3][25]+FA325*s->l[1][25]-OM125*OM325*(s->In[1][25]-
 s->In[9][25])-s->dpt[1][46]*(FF126*S26-FF326*C26));
  CF325 = -(s->trq[3][25]-s->In[9][25]*OpF324+CF126*S26-CF326*C26+FA125*s->l[2][25]-FA225*s->l[1][25]-FF226*
 s->dpt[1][46]+OM125*OM225*(s->In[1][25]-s->In[5][25]));
  FB125_1 = s->m[25]*AlM125_1;
  FB225_1 = s->m[25]*AlM225_1;
  FB325_1 = s->m[25]*AlM324_1;
  FM125_1 = FB125_1+FM126_1*C26+FM326_1*S26;
  FM225_1 = FB225_1+FM226_1;
  FM325_1 = FB325_1-FM126_1*S26+FM326_1*C26;
  CM125_1 = CM126_1*C26+CM326_1*S26-FB225_1*s->l[3][25]+FB325_1*s->l[2][25];
  CM225_1 = FB125_1*s->l[3][25]-FB325_1*s->l[1][25]+s->dpt[1][46]*(FM126_1*S26-FM326_1*C26);
  CM325_1 = -(CM126_1*S26-CM326_1*C26+FB125_1*s->l[2][25]-FB225_1*s->l[1][25]-FM226_1*s->dpt[1][46]);
  FB125_2 = s->m[25]*AlM125_2;
  FB225_2 = s->m[25]*AlM225_2;
  FB325_2 = s->m[25]*AlM324_2;
  FM125_2 = FB125_2+FM126_2*C26+FM326_2*S26;
  FM225_2 = FB225_2+FM226_2;
  FM325_2 = FB325_2-FM126_2*S26+FM326_2*C26;
  CM125_2 = CM126_2*C26+CM326_2*S26-FB225_2*s->l[3][25]+FB325_2*s->l[2][25];
  CM225_2 = FB125_2*s->l[3][25]-FB325_2*s->l[1][25]+s->dpt[1][46]*(FM126_2*S26-FM326_2*C26);
  CM325_2 = -(CM126_2*S26-CM326_2*C26+FB125_2*s->l[2][25]-FB225_2*s->l[1][25]-FM226_2*s->dpt[1][46]);
  FB125_3 = s->m[25]*AlM125_3;
  FB225_3 = s->m[25]*AlM225_3;
  FB325_3 = s->m[25]*AlM324_3;
  FM125_3 = FB125_3+FM126_3*C26+FM326_3*S26;
  FM225_3 = FB225_3+FM226_3;
  FM325_3 = FB325_3-FM126_3*S26+FM326_3*C26;
  CM125_3 = CM126_3*C26+CM326_3*S26-FB225_3*s->l[3][25]+FB325_3*s->l[2][25];
  CM225_3 = FB125_3*s->l[3][25]-FB325_3*s->l[1][25]+s->dpt[1][46]*(FM126_3*S26-FM326_3*C26);
  CM325_3 = -(CM126_3*S26-CM326_3*C26+FB125_3*s->l[2][25]-FB225_3*s->l[1][25]-FM226_3*s->dpt[1][46]);
  FB125_4 = s->m[25]*(AlM125_4+OpM225_4*s->l[3][25]-OpM324_4*s->l[2][25]);
  FB225_4 = s->m[25]*(AlM225_4-OpM125_4*s->l[3][25]+OpM324_4*s->l[1][25]);
  FB325_4 = s->m[25]*(AlM325_4+OpM125_4*s->l[2][25]-OpM225_4*s->l[1][25]);
  FM125_4 = FB125_4+FM126_4*C26+FM326_4*S26;
  FM225_4 = FB225_4+FM226_4;
  FM325_4 = FB325_4-FM126_4*S26+FM326_4*C26;
  CM125_4 = s->In[1][25]*OpM125_4+CM126_4*C26+CM326_4*S26-FB225_4*s->l[3][25]+FB325_4*s->l[2][25];
  CM225_4 = CM226_4+s->In[5][25]*OpM225_4+FB125_4*s->l[3][25]-FB325_4*s->l[1][25]+s->dpt[1][46]*(FM126_4*S26-FM326_4*C26
 );
  CM325_4 = s->In[9][25]*OpM324_4-CM126_4*S26+CM326_4*C26-FB125_4*s->l[2][25]+FB225_4*s->l[1][25]+FM226_4*s->dpt[1][46];
  FB125_5 = s->m[25]*(AlM125_5+OpM225_5*s->l[3][25]-OpM324_5*s->l[2][25]);
  FB225_5 = s->m[25]*(AlM225_5-OpM125_5*s->l[3][25]+OpM324_5*s->l[1][25]);
  FB325_5 = s->m[25]*(AlM325_5+OpM125_5*s->l[2][25]-OpM225_5*s->l[1][25]);
  FM125_5 = FB125_5+FM126_5*C26+FM326_5*S26;
  FM225_5 = FB225_5+FM226_5;
  FM325_5 = FB325_5-FM126_5*S26+FM326_5*C26;
  CM125_5 = s->In[1][25]*OpM125_5+CM126_5*C26+CM326_5*S26-FB225_5*s->l[3][25]+FB325_5*s->l[2][25];
  CM225_5 = CM226_5+s->In[5][25]*OpM225_5+FB125_5*s->l[3][25]-FB325_5*s->l[1][25]+s->dpt[1][46]*(FM126_5*S26-FM326_5*C26
 );
  CM325_5 = s->In[9][25]*OpM324_5-CM126_5*S26+CM326_5*C26-FB125_5*s->l[2][25]+FB225_5*s->l[1][25]+FM226_5*s->dpt[1][46];
  FB125_6 = s->m[25]*(AlM125_6+OpM225_6*s->l[3][25]-OpM324_6*s->l[2][25]);
  FB225_6 = s->m[25]*(AlM225_6-OpM125_6*s->l[3][25]+OpM324_6*s->l[1][25]);
  FB325_6 = s->m[25]*(AlM325_6+OpM125_6*s->l[2][25]-OpM225_6*s->l[1][25]);
  FM125_6 = FB125_6+FM126_6*C26+FM326_6*S26;
  FM225_6 = FB225_6+FM226_6;
  FM325_6 = FB325_6-FM126_6*S26+FM326_6*C26;
  CM125_6 = s->In[1][25]*OpM125_6+CM126_6*C26+CM326_6*S26-FB225_6*s->l[3][25]+FB325_6*s->l[2][25];
  CM225_6 = CM226_6+s->In[5][25]*OpM225_6+FB125_6*s->l[3][25]-FB325_6*s->l[1][25]+s->dpt[1][46]*(FM126_6*S26-FM326_6*C26
 );
  CM325_6 = s->In[9][25]*OpM324_6-CM126_6*S26+CM326_6*C26-FB125_6*s->l[2][25]+FB225_6*s->l[1][25]+FM226_6*s->dpt[1][46];
  FB125_19 = s->m[25]*(AlM125_19+OpM225_19*s->l[3][25]-OpM324_19*s->l[2][25]);
  FB225_19 = s->m[25]*(AlM225_19-OpM125_19*s->l[3][25]+OpM324_19*s->l[1][25]);
  FB325_19 = s->m[25]*(AlM325_19+OpM125_19*s->l[2][25]-OpM225_19*s->l[1][25]);
  FM125_19 = FB125_19+FM126_19*C26+FM326_19*S26;
  FM225_19 = FB225_19+FM226_19;
  FM325_19 = FB325_19-FM126_19*S26+FM326_19*C26;
  CM125_19 = s->In[1][25]*OpM125_19+CM126_19*C26+CM326_19*S26-FB225_19*s->l[3][25]+FB325_19*s->l[2][25];
  CM225_19 = CM226_19+s->In[5][25]*OpM225_19+FB125_19*s->l[3][25]-FB325_19*s->l[1][25]+s->dpt[1][46]*(FM126_19*S26-
 FM326_19*C26);
  CM325_19 = s->In[9][25]*OpM324_19-CM126_19*S26+CM326_19*C26-FB125_19*s->l[2][25]+FB225_19*s->l[1][25]+FM226_19*
 s->dpt[1][46];
  FB125_20 = s->m[25]*(AlM125_20+OpM225_20*s->l[3][25]-OpM324_20*s->l[2][25]);
  FB225_20 = s->m[25]*(AlM225_20-OpM125_20*s->l[3][25]+OpM324_20*s->l[1][25]);
  FB325_20 = s->m[25]*(AlM325_20+OpM125_20*s->l[2][25]-OpM225_20*s->l[1][25]);
  FM125_20 = FB125_20+FM126_20*C26+FM326_20*S26;
  FM225_20 = FB225_20+FM226_20;
  FM325_20 = FB325_20-FM126_20*S26+FM326_20*C26;
  CM125_20 = s->In[1][25]*OpM125_20+CM126_20*C26+CM326_20*S26-FB225_20*s->l[3][25]+FB325_20*s->l[2][25];
  CM225_20 = CM226_20+s->In[5][25]*OpM225_20+FB125_20*s->l[3][25]-FB325_20*s->l[1][25]+s->dpt[1][46]*(FM126_20*S26-
 FM326_20*C26);
  CM325_20 = s->In[9][25]*OpM324_20-CM126_20*S26+CM326_20*C26-FB125_20*s->l[2][25]+FB225_20*s->l[1][25]+FM226_20*
 s->dpt[1][46];
  FB125_21 = s->m[25]*(AlM125_21+OpM225_21*s->l[3][25]-OpM324_21*s->l[2][25]);
  FB225_21 = s->m[25]*(AlM225_21-OpM125_21*s->l[3][25]+OpM324_21*s->l[1][25]);
  FB325_21 = s->m[25]*(AlM325_21+OpM125_21*s->l[2][25]-OpM225_21*s->l[1][25]);
  FM125_21 = FB125_21+FM126_21*C26+FM326_21*S26;
  FM225_21 = FB225_21+FM226_21;
  FM325_21 = FB325_21-FM126_21*S26+FM326_21*C26;
  CM125_21 = s->In[1][25]*OpM125_21+CM126_21*C26+CM326_21*S26-FB225_21*s->l[3][25]+FB325_21*s->l[2][25];
  CM225_21 = CM226_21+s->In[5][25]*OpM225_21+FB125_21*s->l[3][25]-FB325_21*s->l[1][25]+s->dpt[1][46]*(FM126_21*S26-
 FM326_21*C26);
  CM325_21 = s->In[9][25]*OpM324_21-CM126_21*S26+CM326_21*C26-FB125_21*s->l[2][25]+FB225_21*s->l[1][25]+FM226_21*
 s->dpt[1][46];
  FB125_22 = s->m[25]*(AlM125_22+OpM225_22*s->l[3][25]-OpM324_22*s->l[2][25]);
  FB225_22 = s->m[25]*(AlM225_22-OpM125_22*s->l[3][25]+OpM324_22*s->l[1][25]);
  FB325_22 = s->m[25]*(AlM325_22+OpM125_22*s->l[2][25]-OpM225_22*s->l[1][25]);
  FM125_22 = FB125_22+FM126_22*C26+FM326_22*S26;
  FM225_22 = FB225_22+FM226_22;
  FM325_22 = FB325_22-FM126_22*S26+FM326_22*C26;
  CM125_22 = s->In[1][25]*OpM125_22+CM126_22*C26+CM326_22*S26-FB225_22*s->l[3][25]+FB325_22*s->l[2][25];
  CM225_22 = CM226_22+s->In[5][25]*OpM225_22+FB125_22*s->l[3][25]-FB325_22*s->l[1][25]+s->dpt[1][46]*(FM126_22*S26-
 FM326_22*C26);
  CM325_22 = s->In[9][25]*OpM324_22-CM126_22*S26+CM326_22*C26-FB125_22*s->l[2][25]+FB225_22*s->l[1][25]+FM226_22*
 s->dpt[1][46];
  FB125_23 = s->m[25]*(AlM125_23+OpM225_23*s->l[3][25]-s->l[2][25]*S24);
  FB225_23 = s->m[25]*(AlM225_23-OpM125_23*s->l[3][25]+s->l[1][25]*S24);
  FB325_23 = s->m[25]*(AlM325_23+OpM125_23*s->l[2][25]-OpM225_23*s->l[1][25]);
  FM125_23 = FB125_23+FM126_23*C26+FM326_23*S26;
  FM225_23 = FB225_23+FM226_23;
  FM325_23 = FB325_23-FM126_23*S26+FM326_23*C26;
  CM125_23 = s->In[1][25]*OpM125_23+CM126_23*C26+CM326_23*S26-FB225_23*s->l[3][25]+FB325_23*s->l[2][25];
  CM225_23 = CM226_23+s->In[5][25]*OpM225_23+FB125_23*s->l[3][25]-FB325_23*s->l[1][25]+s->dpt[1][46]*(FM126_23*S26-
 FM326_23*C26);
  CM325_23 = s->In[9][25]*S24-CM126_23*S26+CM326_23*C26-FB125_23*s->l[2][25]+FB225_23*s->l[1][25]+FM226_23*s->dpt[1][46];
  FB125_24 = s->m[25]*s->l[3][25]*C25;
  FB225_24 = -s->m[25]*s->l[3][25]*S25;
  FB325_24 = -s->m[25]*(s->dpt[1][44]+s->l[1][25]*C25-s->l[2][25]*S25);
  CM325_24 = -(CM126_24*S26-CM326_24*C26+FB125_24*s->l[2][25]-FB225_24*s->l[1][25]-s->dpt[1][46]*(FM127_24*S27+FM227_24*
 C27));
  CM325_25 = s->In[9][25]+s->m[25]*s->l[1][25]*s->l[1][25]+s->m[25]*s->l[2][25]*s->l[2][25]+s->dpt[1][46]*(s->m[26]*
 s->dpt[1][46]+FM127_25*S27+FM227_25*C27)+C26*(CM327_25+s->In[9][26]*C26+s->m[26]*s->l[2][26]*s->l[2][26]*C26-s->dpt[2][47]*(
 FM127_25*C27-FM227_25*S27))-S26*(CM127_25*C27-CM227_25*S27+s->dpt[2][47]*(s->m[27]*AlM327_25+FB228_25*S28+FB328_25*C28)-S26*
 (s->In[1][26]+s->m[26]*s->l[2][26]*s->l[2][26]));
  FA124 = -(s->frc[1][24]-s->m[24]*(AlF124+BS124*s->l[1][24]+BeF224*s->l[2][24]+s->l[3][24]*(BS324+OpF223)));
  FA224 = -(s->frc[2][24]-s->m[24]*(AlF224+BS524*s->l[2][24]+BeF424*s->l[1][24]+s->l[3][24]*(BS624-OpF124)));
  FA324 = -(s->frc[3][24]-s->m[24]*(AlF324+BeF724*s->l[1][24]+BeF824*s->l[2][24]-s->l[3][24]*(OM124*OM124+OM224*OM224)));
  FF124 = FA124+FF125*C25-FF225*S25;
  FF324 = FA324+FF325;
  CF124 = -(s->trq[1][24]-s->In[1][24]*OpF124-CF125*C25+CF225*S25+FA224*s->l[3][24]-FA324*s->l[2][24]-FF325*
 s->dpt[2][44]+OM224*OM324*(s->In[5][24]-s->In[9][24]));
  CF224 = -(s->trq[2][24]-s->In[5][24]*OpF223-CF125*S25-CF225*C25-FA124*s->l[3][24]+FA324*s->l[1][24]+FF325*
 s->dpt[1][44]-OM124*OM324*(s->In[1][24]-s->In[9][24]));
  CF324 = -(s->trq[3][24]-CF325-s->In[9][24]*OpF324+FA124*s->l[2][24]-FA224*s->l[1][24]+OM124*OM224*(s->In[1][24]-
 s->In[5][24])-s->dpt[1][44]*(FF125*S25+FF225*C25)+s->dpt[2][44]*(FF125*C25-FF225*S25));
  FB124_1 = s->m[24]*AlM124_1;
  FB224_1 = s->m[24]*AlM223_1;
  FB324_1 = s->m[24]*AlM324_1;
  FM124_1 = FB124_1+FM125_1*C25-FM225_1*S25;
  FM324_1 = FB324_1+FM325_1;
  CM124_1 = CM125_1*C25-CM225_1*S25-FB224_1*s->l[3][24]+FB324_1*s->l[2][24]+FM325_1*s->dpt[2][44];
  CM224_1 = CM125_1*S25+CM225_1*C25+FB124_1*s->l[3][24]-FB324_1*s->l[1][24]-FM325_1*s->dpt[1][44];
  CM324_1 = CM325_1-FB124_1*s->l[2][24]+FB224_1*s->l[1][24]+s->dpt[1][44]*(FM125_1*S25+FM225_1*C25)-s->dpt[2][44]*(
 FM125_1*C25-FM225_1*S25);
  FB124_2 = s->m[24]*AlM124_2;
  FB224_2 = s->m[24]*AlM223_2;
  FB324_2 = s->m[24]*AlM324_2;
  FM124_2 = FB124_2+FM125_2*C25-FM225_2*S25;
  FM324_2 = FB324_2+FM325_2;
  CM124_2 = CM125_2*C25-CM225_2*S25-FB224_2*s->l[3][24]+FB324_2*s->l[2][24]+FM325_2*s->dpt[2][44];
  CM224_2 = CM125_2*S25+CM225_2*C25+FB124_2*s->l[3][24]-FB324_2*s->l[1][24]-FM325_2*s->dpt[1][44];
  CM324_2 = CM325_2-FB124_2*s->l[2][24]+FB224_2*s->l[1][24]+s->dpt[1][44]*(FM125_2*S25+FM225_2*C25)-s->dpt[2][44]*(
 FM125_2*C25-FM225_2*S25);
  FB124_3 = s->m[24]*AlM124_3;
  FB224_3 = s->m[24]*AlM223_3;
  FB324_3 = s->m[24]*AlM324_3;
  FM124_3 = FB124_3+FM125_3*C25-FM225_3*S25;
  FM324_3 = FB324_3+FM325_3;
  CM124_3 = CM125_3*C25-CM225_3*S25-FB224_3*s->l[3][24]+FB324_3*s->l[2][24]+FM325_3*s->dpt[2][44];
  CM224_3 = CM125_3*S25+CM225_3*C25+FB124_3*s->l[3][24]-FB324_3*s->l[1][24]-FM325_3*s->dpt[1][44];
  CM324_3 = CM325_3-FB124_3*s->l[2][24]+FB224_3*s->l[1][24]+s->dpt[1][44]*(FM125_3*S25+FM225_3*C25)-s->dpt[2][44]*(
 FM125_3*C25-FM225_3*S25);
  FB124_4 = s->m[24]*(AlM124_4+OpM223_4*s->l[3][24]-OpM324_4*s->l[2][24]);
  FB224_4 = s->m[24]*(AlM223_4-OpM124_4*s->l[3][24]+OpM324_4*s->l[1][24]);
  FB324_4 = s->m[24]*(AlM324_4+OpM124_4*s->l[2][24]-OpM223_4*s->l[1][24]);
  FM124_4 = FB124_4+FM125_4*C25-FM225_4*S25;
  FM324_4 = FB324_4+FM325_4;
  CM124_4 = s->In[1][24]*OpM124_4+CM125_4*C25-CM225_4*S25-FB224_4*s->l[3][24]+FB324_4*s->l[2][24]+FM325_4*s->dpt[2][44];
  CM224_4 = s->In[5][24]*OpM223_4+CM125_4*S25+CM225_4*C25+FB124_4*s->l[3][24]-FB324_4*s->l[1][24]-FM325_4*s->dpt[1][44];
  CM324_4 = CM325_4+s->In[9][24]*OpM324_4-FB124_4*s->l[2][24]+FB224_4*s->l[1][24]+s->dpt[1][44]*(FM125_4*S25+FM225_4*C25
 )-s->dpt[2][44]*(FM125_4*C25-FM225_4*S25);
  FB124_5 = s->m[24]*(AlM124_5+OpM223_5*s->l[3][24]-OpM324_5*s->l[2][24]);
  FB224_5 = s->m[24]*(AlM223_5-OpM124_5*s->l[3][24]+OpM324_5*s->l[1][24]);
  FB324_5 = s->m[24]*(AlM324_5+OpM124_5*s->l[2][24]-OpM223_5*s->l[1][24]);
  FM124_5 = FB124_5+FM125_5*C25-FM225_5*S25;
  FM324_5 = FB324_5+FM325_5;
  CM124_5 = s->In[1][24]*OpM124_5+CM125_5*C25-CM225_5*S25-FB224_5*s->l[3][24]+FB324_5*s->l[2][24]+FM325_5*s->dpt[2][44];
  CM224_5 = s->In[5][24]*OpM223_5+CM125_5*S25+CM225_5*C25+FB124_5*s->l[3][24]-FB324_5*s->l[1][24]-FM325_5*s->dpt[1][44];
  CM324_5 = CM325_5+s->In[9][24]*OpM324_5-FB124_5*s->l[2][24]+FB224_5*s->l[1][24]+s->dpt[1][44]*(FM125_5*S25+FM225_5*C25
 )-s->dpt[2][44]*(FM125_5*C25-FM225_5*S25);
  FB124_6 = s->m[24]*(AlM124_6+OpM223_6*s->l[3][24]-OpM324_6*s->l[2][24]);
  FB224_6 = s->m[24]*(AlM223_6-OpM124_6*s->l[3][24]+OpM324_6*s->l[1][24]);
  FB324_6 = s->m[24]*(AlM324_6+OpM124_6*s->l[2][24]-OpM223_6*s->l[1][24]);
  FM124_6 = FB124_6+FM125_6*C25-FM225_6*S25;
  FM324_6 = FB324_6+FM325_6;
  CM124_6 = s->In[1][24]*OpM124_6+CM125_6*C25-CM225_6*S25-FB224_6*s->l[3][24]+FB324_6*s->l[2][24]+FM325_6*s->dpt[2][44];
  CM224_6 = s->In[5][24]*OpM223_6+CM125_6*S25+CM225_6*C25+FB124_6*s->l[3][24]-FB324_6*s->l[1][24]-FM325_6*s->dpt[1][44];
  CM324_6 = CM325_6+s->In[9][24]*OpM324_6-FB124_6*s->l[2][24]+FB224_6*s->l[1][24]+s->dpt[1][44]*(FM125_6*S25+FM225_6*C25
 )-s->dpt[2][44]*(FM125_6*C25-FM225_6*S25);
  FB124_19 = s->m[24]*(AlM124_19+OpM223_19*s->l[3][24]-OpM324_19*s->l[2][24]);
  FB224_19 = s->m[24]*(AlM223_19-OpM124_19*s->l[3][24]+OpM324_19*s->l[1][24]);
  FB324_19 = s->m[24]*(AlM324_19+OpM124_19*s->l[2][24]-OpM223_19*s->l[1][24]);
  FM124_19 = FB124_19+FM125_19*C25-FM225_19*S25;
  FM324_19 = FB324_19+FM325_19;
  CM124_19 = s->In[1][24]*OpM124_19+CM125_19*C25-CM225_19*S25-FB224_19*s->l[3][24]+FB324_19*s->l[2][24]+FM325_19*
 s->dpt[2][44];
  CM224_19 = s->In[5][24]*OpM223_19+CM125_19*S25+CM225_19*C25+FB124_19*s->l[3][24]-FB324_19*s->l[1][24]-FM325_19*
 s->dpt[1][44];
  CM324_19 = CM325_19+s->In[9][24]*OpM324_19-FB124_19*s->l[2][24]+FB224_19*s->l[1][24]+s->dpt[1][44]*(FM125_19*S25+
 FM225_19*C25)-s->dpt[2][44]*(FM125_19*C25-FM225_19*S25);
  FB124_20 = s->m[24]*(AlM124_20+OpM223_20*s->l[3][24]-OpM324_20*s->l[2][24]);
  FB224_20 = s->m[24]*(AlM223_20-OpM124_20*s->l[3][24]+OpM324_20*s->l[1][24]);
  FB324_20 = s->m[24]*(AlM324_20+OpM124_20*s->l[2][24]-OpM223_20*s->l[1][24]);
  FM124_20 = FB124_20+FM125_20*C25-FM225_20*S25;
  FM324_20 = FB324_20+FM325_20;
  CM124_20 = s->In[1][24]*OpM124_20+CM125_20*C25-CM225_20*S25-FB224_20*s->l[3][24]+FB324_20*s->l[2][24]+FM325_20*
 s->dpt[2][44];
  CM224_20 = s->In[5][24]*OpM223_20+CM125_20*S25+CM225_20*C25+FB124_20*s->l[3][24]-FB324_20*s->l[1][24]-FM325_20*
 s->dpt[1][44];
  CM324_20 = CM325_20+s->In[9][24]*OpM324_20-FB124_20*s->l[2][24]+FB224_20*s->l[1][24]+s->dpt[1][44]*(FM125_20*S25+
 FM225_20*C25)-s->dpt[2][44]*(FM125_20*C25-FM225_20*S25);
  FB124_21 = s->m[24]*(AlM124_21+OpM223_21*s->l[3][24]-OpM324_21*s->l[2][24]);
  FB224_21 = s->m[24]*(AlM223_21-OpM124_21*s->l[3][24]+OpM324_21*s->l[1][24]);
  FB324_21 = s->m[24]*(AlM324_21+OpM124_21*s->l[2][24]-OpM223_21*s->l[1][24]);
  FM124_21 = FB124_21+FM125_21*C25-FM225_21*S25;
  FM324_21 = FB324_21+FM325_21;
  CM124_21 = s->In[1][24]*OpM124_21+CM125_21*C25-CM225_21*S25-FB224_21*s->l[3][24]+FB324_21*s->l[2][24]+FM325_21*
 s->dpt[2][44];
  CM224_21 = s->In[5][24]*OpM223_21+CM125_21*S25+CM225_21*C25+FB124_21*s->l[3][24]-FB324_21*s->l[1][24]-FM325_21*
 s->dpt[1][44];
  CM324_21 = CM325_21+s->In[9][24]*OpM324_21-FB124_21*s->l[2][24]+FB224_21*s->l[1][24]+s->dpt[1][44]*(FM125_21*S25+
 FM225_21*C25)-s->dpt[2][44]*(FM125_21*C25-FM225_21*S25);
  FB124_22 = s->m[24]*(AlM124_22-OpM324_22*s->l[2][24]+s->l[3][24]*C23);
  FB224_22 = -s->m[24]*(OpM124_22*s->l[3][24]-OpM324_22*s->l[1][24]);
  FB324_22 = s->m[24]*(AlM324_22+OpM124_22*s->l[2][24]-s->l[1][24]*C23);
  FM124_22 = FB124_22+FM125_22*C25-FM225_22*S25;
  FM324_22 = FB324_22+FM325_22;
  CM124_22 = s->In[1][24]*OpM124_22+CM125_22*C25-CM225_22*S25-FB224_22*s->l[3][24]+FB324_22*s->l[2][24]+FM325_22*
 s->dpt[2][44];
  CM224_22 = s->In[5][24]*C23+CM125_22*S25+CM225_22*C25+FB124_22*s->l[3][24]-FB324_22*s->l[1][24]-FM325_22*s->dpt[1][44];
  CM324_22 = CM325_22+s->In[9][24]*OpM324_22-FB124_22*s->l[2][24]+FB224_22*s->l[1][24]+s->dpt[1][44]*(FM125_22*S25+
 FM225_22*C25)-s->dpt[2][44]*(FM125_22*C25-FM225_22*S25);
  FB124_23 = -s->m[24]*S24*(s->dpt[2][42]+s->l[2][24]);
  FB224_23 = s->m[24]*(s->l[1][24]*S24-s->l[3][24]*C24);
  FB324_23 = s->m[24]*C24*(s->dpt[2][42]+s->l[2][24]);
  CM224_23 = CM125_23*S25+CM225_23*C25+FB124_23*s->l[3][24]-FB324_23*s->l[1][24]-FM325_23*s->dpt[1][44];
  CM224_24 = s->In[5][24]+s->m[24]*s->l[1][24]*s->l[1][24]+s->m[24]*s->l[3][24]*s->l[3][24]-s->dpt[1][44]*(FB325_24-
 FM126_24*S26+FM326_24*C26)+C25*(CM226_24+s->In[5][25]*C25+FB125_24*s->l[3][25]-FB325_24*s->l[1][25]+s->dpt[1][46]*(FM126_24*
 S26-FM326_24*C26))+S25*(s->In[1][25]*S25+CM126_24*C26+CM326_24*S26-FB225_24*s->l[3][25]+FB325_24*s->l[2][25]);
  FA123 = -(s->frc[1][23]-s->m[23]*(AlF123+BeF223*s->l[2][23]-s->l[1][23]*(OM223*OM223+OM323*OM323)));
  FA223 = -(s->frc[2][23]-s->m[23]*(AlF223+BS523*s->l[2][23]+s->l[1][23]*(BS223+OpF323)));
  FA323 = -(s->frc[3][23]-s->m[23]*(AlF323+BeF823*s->l[2][23]-s->l[1][23]*(OpF223-OM123*OM323)));
  FF123 = FA123+FF124*C24+FF324*S24;
  FF223 = FA223+FA224+FF125*S25+FF225*C25;
  FF323 = FA323-FF124*S24+FF324*C24;
  CF123 = -(s->trq[1][23]-s->In[1][23]*OpF122-CF124*C24-CF324*S24-FA323*s->l[2][23]+OM223*OM323*(s->In[5][23]-
 s->In[9][23])+s->dpt[2][42]*(FF124*S24-FF324*C24));
  CF223 = -(s->trq[2][23]-CF224-s->In[5][23]*OpF223+FA323*s->l[1][23]-OM123*OM323*(s->In[1][23]-s->In[9][23]));
  CF323 = -(s->trq[3][23]-s->In[9][23]*OpF323+CF124*S24-CF324*C24+FA123*s->l[2][23]-FA223*s->l[1][23]+OM123*OM223*(
 s->In[1][23]-s->In[5][23])+s->dpt[2][42]*(FF124*C24+FF324*S24));
  FB123_1 = s->m[23]*AlM122_1;
  FB223_1 = s->m[23]*AlM223_1;
  FB323_1 = s->m[23]*AlM323_1;
  FM123_1 = FB123_1+FM124_1*C24+FM324_1*S24;
  FM223_1 = FB223_1+FB224_1+FM125_1*S25+FM225_1*C25;
  FM323_1 = FB323_1-FM124_1*S24+FM324_1*C24;
  CM123_1 = CM124_1*C24+CM324_1*S24+FB323_1*s->l[2][23]-s->dpt[2][42]*(FM124_1*S24-FM324_1*C24);
  CM223_1 = CM224_1-FB323_1*s->l[1][23];
  CM323_1 = -(CM124_1*S24-CM324_1*C24+FB123_1*s->l[2][23]-FB223_1*s->l[1][23]+s->dpt[2][42]*(FM124_1*C24+FM324_1*S24));
  FB123_2 = s->m[23]*AlM122_2;
  FB223_2 = s->m[23]*AlM223_2;
  FB323_2 = s->m[23]*AlM323_2;
  FM123_2 = FB123_2+FM124_2*C24+FM324_2*S24;
  FM223_2 = FB223_2+FB224_2+FM125_2*S25+FM225_2*C25;
  FM323_2 = FB323_2-FM124_2*S24+FM324_2*C24;
  CM123_2 = CM124_2*C24+CM324_2*S24+FB323_2*s->l[2][23]-s->dpt[2][42]*(FM124_2*S24-FM324_2*C24);
  CM223_2 = CM224_2-FB323_2*s->l[1][23];
  CM323_2 = -(CM124_2*S24-CM324_2*C24+FB123_2*s->l[2][23]-FB223_2*s->l[1][23]+s->dpt[2][42]*(FM124_2*C24+FM324_2*S24));
  FB123_3 = s->m[23]*AlM122_3;
  FB223_3 = s->m[23]*AlM223_3;
  FB323_3 = s->m[23]*AlM323_3;
  FM123_3 = FB123_3+FM124_3*C24+FM324_3*S24;
  FM223_3 = FB223_3+FB224_3+FM125_3*S25+FM225_3*C25;
  FM323_3 = FB323_3-FM124_3*S24+FM324_3*C24;
  CM123_3 = CM124_3*C24+CM324_3*S24+FB323_3*s->l[2][23]-s->dpt[2][42]*(FM124_3*S24-FM324_3*C24);
  CM223_3 = CM224_3-FB323_3*s->l[1][23];
  CM323_3 = -(CM124_3*S24-CM324_3*C24+FB123_3*s->l[2][23]-FB223_3*s->l[1][23]+s->dpt[2][42]*(FM124_3*C24+FM324_3*S24));
  FB123_4 = s->m[23]*(AlM123_4-OpM323_4*s->l[2][23]);
  FB223_4 = s->m[23]*(AlM223_4+OpM323_4*s->l[1][23]);
  FB323_4 = s->m[23]*(AlM323_4+OpM122_4*s->l[2][23]-OpM223_4*s->l[1][23]);
  FM123_4 = FB123_4+FM124_4*C24+FM324_4*S24;
  FM223_4 = FB223_4+FB224_4+FM125_4*S25+FM225_4*C25;
  FM323_4 = FB323_4-FM124_4*S24+FM324_4*C24;
  CM123_4 = s->In[1][23]*OpM122_4+CM124_4*C24+CM324_4*S24+FB323_4*s->l[2][23]-s->dpt[2][42]*(FM124_4*S24-FM324_4*C24);
  CM223_4 = CM224_4+s->In[5][23]*OpM223_4-FB323_4*s->l[1][23];
  CM323_4 = s->In[9][23]*OpM323_4-CM124_4*S24+CM324_4*C24-FB123_4*s->l[2][23]+FB223_4*s->l[1][23]-s->dpt[2][42]*(FM124_4
 *C24+FM324_4*S24);
  FB123_5 = s->m[23]*(AlM123_5-OpM323_5*s->l[2][23]);
  FB223_5 = s->m[23]*(AlM223_5+OpM323_5*s->l[1][23]);
  FB323_5 = s->m[23]*(AlM323_5+OpM122_5*s->l[2][23]-OpM223_5*s->l[1][23]);
  FM123_5 = FB123_5+FM124_5*C24+FM324_5*S24;
  FM223_5 = FB223_5+FB224_5+FM125_5*S25+FM225_5*C25;
  FM323_5 = FB323_5-FM124_5*S24+FM324_5*C24;
  CM123_5 = s->In[1][23]*OpM122_5+CM124_5*C24+CM324_5*S24+FB323_5*s->l[2][23]-s->dpt[2][42]*(FM124_5*S24-FM324_5*C24);
  CM223_5 = CM224_5+s->In[5][23]*OpM223_5-FB323_5*s->l[1][23];
  CM323_5 = s->In[9][23]*OpM323_5-CM124_5*S24+CM324_5*C24-FB123_5*s->l[2][23]+FB223_5*s->l[1][23]-s->dpt[2][42]*(FM124_5
 *C24+FM324_5*S24);
  FB123_6 = s->m[23]*(AlM123_6-OpM323_6*s->l[2][23]);
  FB223_6 = s->m[23]*(AlM223_6+OpM323_6*s->l[1][23]);
  FB323_6 = s->m[23]*(AlM323_6+OpM122_6*s->l[2][23]-OpM223_6*s->l[1][23]);
  FM123_6 = FB123_6+FM124_6*C24+FM324_6*S24;
  FM223_6 = FB223_6+FB224_6+FM125_6*S25+FM225_6*C25;
  FM323_6 = FB323_6-FM124_6*S24+FM324_6*C24;
  CM123_6 = s->In[1][23]*OpM122_6+CM124_6*C24+CM324_6*S24+FB323_6*s->l[2][23]-s->dpt[2][42]*(FM124_6*S24-FM324_6*C24);
  CM223_6 = CM224_6+s->In[5][23]*OpM223_6-FB323_6*s->l[1][23];
  CM323_6 = s->In[9][23]*OpM323_6-CM124_6*S24+CM324_6*C24-FB123_6*s->l[2][23]+FB223_6*s->l[1][23]-s->dpt[2][42]*(FM124_6
 *C24+FM324_6*S24);
  FB123_19 = s->m[23]*(AlM123_19-OpM323_19*s->l[2][23]);
  FB223_19 = s->m[23]*(AlM223_19+OpM323_19*s->l[1][23]);
  FB323_19 = s->m[23]*(AlM323_19+OpM122_19*s->l[2][23]-OpM223_19*s->l[1][23]);
  FM123_19 = FB123_19+FM124_19*C24+FM324_19*S24;
  FM223_19 = FB223_19+FB224_19+FM125_19*S25+FM225_19*C25;
  FM323_19 = FB323_19-FM124_19*S24+FM324_19*C24;
  CM123_19 = s->In[1][23]*OpM122_19+CM124_19*C24+CM324_19*S24+FB323_19*s->l[2][23]-s->dpt[2][42]*(FM124_19*S24-FM324_19*
 C24);
  CM223_19 = CM224_19+s->In[5][23]*OpM223_19-FB323_19*s->l[1][23];
  CM323_19 = s->In[9][23]*OpM323_19-CM124_19*S24+CM324_19*C24-FB123_19*s->l[2][23]+FB223_19*s->l[1][23]-s->dpt[2][42]*(
 FM124_19*C24+FM324_19*S24);
  FB123_20 = s->m[23]*(AlM123_20-OpM323_20*s->l[2][23]);
  FB223_20 = s->m[23]*(AlM223_20+OpM323_20*s->l[1][23]);
  FB323_20 = s->m[23]*(AlM323_20+OpM122_20*s->l[2][23]-OpM223_20*s->l[1][23]);
  FM123_20 = FB123_20+FM124_20*C24+FM324_20*S24;
  FM223_20 = FB223_20+FB224_20+FM125_20*S25+FM225_20*C25;
  FM323_20 = FB323_20-FM124_20*S24+FM324_20*C24;
  CM123_20 = s->In[1][23]*OpM122_20+CM124_20*C24+CM324_20*S24+FB323_20*s->l[2][23]-s->dpt[2][42]*(FM124_20*S24-FM324_20*
 C24);
  CM223_20 = CM224_20+s->In[5][23]*OpM223_20-FB323_20*s->l[1][23];
  CM323_20 = s->In[9][23]*OpM323_20-CM124_20*S24+CM324_20*C24-FB123_20*s->l[2][23]+FB223_20*s->l[1][23]-s->dpt[2][42]*(
 FM124_20*C24+FM324_20*S24);
  FB123_21 = s->m[23]*(AlM123_21-OpM323_21*s->l[2][23]);
  FB223_21 = s->m[23]*(AlM223_21+OpM323_21*s->l[1][23]);
  FB323_21 = s->m[23]*(AlM323_21-OpM223_21*s->l[1][23]-s->l[2][23]*S22);
  FM123_21 = FB123_21+FM124_21*C24+FM324_21*S24;
  FM223_21 = FB223_21+FB224_21+FM125_21*S25+FM225_21*C25;
  FM323_21 = FB323_21-FM124_21*S24+FM324_21*C24;
  CM123_21 = CM124_21*C24+CM324_21*S24-s->dpt[2][42]*(FM124_21*S24-FM324_21*C24)-s->In[1][23]*S22+FB323_21*s->l[2][23];
  CM223_21 = CM224_21+s->In[5][23]*OpM223_21-FB323_21*s->l[1][23];
  CM323_21 = s->In[9][23]*OpM323_21-CM124_21*S24+CM324_21*C24-FB123_21*s->l[2][23]+FB223_21*s->l[1][23]-s->dpt[2][42]*(
 FM124_21*C24+FM324_21*S24);
  FB323_22 = -s->m[23]*s->l[1][23]*C23;
  CM123_22 = CM124_22*C24+CM324_22*S24+FB323_22*s->l[2][23]-s->dpt[2][42]*(FM124_22*S24-FM324_22*C24);
  CM123_23 = s->In[1][23]+s->m[23]*s->l[2][23]*s->l[2][23]+s->dpt[2][42]*(C24*(FB324_23+FM325_23)-S24*(FB124_23+FM125_23
 *C25-FM225_23*S25))+C24*(s->In[1][24]*C24+CM125_23*C25-CM225_23*S25-FB224_23*s->l[3][24]+FB324_23*s->l[2][24]+FM325_23*
 s->dpt[2][44])+S24*(CM325_23+s->In[9][24]*S24-FB124_23*s->l[2][24]+FB224_23*s->l[1][24]+s->dpt[1][44]*(FM125_23*S25+FM225_23
 *C25)-s->dpt[2][44]*(FM125_23*C25-FM225_23*S25));
  FA122 = -(s->frc[1][22]-s->m[22]*(AlF122+BeF222*s->l[2][22]-s->l[1][22]*(OM222*OM222+OM322*OM322)+s->l[3][22]*(BS322+
 OpF221)));
  FA222 = -(s->frc[2][22]-s->m[22]*(AlF222+BS522*s->l[2][22]+s->l[1][22]*(BS222+OpF322)+s->l[3][22]*(BS622-OpF122)));
  FA322 = -(s->frc[3][22]-s->m[22]*(AlF322+BeF822*s->l[2][22]+s->l[1][22]*(BS322-OpF221)-s->l[3][22]*(OM122*OM122+OM222*
 OM222)));
  FF122 = FA122+FF123;
  FF222 = FA222+FF223*C23-FF323*S23;
  FF322 = FA322+FF223*S23+FF323*C23;
  CF122 = -(s->trq[1][22]-CF123-s->In[1][22]*OpF122+FA222*s->l[3][22]-FA322*s->l[2][22]+OM222*OM322*(s->In[5][22]-
 s->In[9][22])-s->dpt[2][40]*(FF223*S23+FF323*C23));
  CF222 = -(s->trq[2][22]-s->In[5][22]*OpF221-CF223*C23+CF323*S23-FA122*s->l[3][22]+FA322*s->l[1][22]-OM122*OM322*(
 s->In[1][22]-s->In[9][22]));
  CF322 = -(s->trq[3][22]-s->In[9][22]*OpF322-CF223*S23-CF323*C23+FA122*s->l[2][22]-FA222*s->l[1][22]+FF123*
 s->dpt[2][40]+OM122*OM222*(s->In[1][22]-s->In[5][22]));
  FB122_1 = s->m[22]*AlM122_1;
  FB222_1 = s->m[22]*AlM221_1;
  FB322_1 = s->m[22]*AlM322_1;
  FM122_1 = FB122_1+FM123_1;
  FM222_1 = FB222_1+FM223_1*C23-FM323_1*S23;
  FM322_1 = FB322_1+FM223_1*S23+FM323_1*C23;
  CM122_1 = CM123_1-FB222_1*s->l[3][22]+FB322_1*s->l[2][22]+s->dpt[2][40]*(FM223_1*S23+FM323_1*C23);
  CM222_1 = CM223_1*C23-CM323_1*S23+FB122_1*s->l[3][22]-FB322_1*s->l[1][22];
  CM322_1 = CM223_1*S23+CM323_1*C23-FB122_1*s->l[2][22]+FB222_1*s->l[1][22]-FM123_1*s->dpt[2][40];
  FB122_2 = s->m[22]*AlM122_2;
  FB222_2 = s->m[22]*AlM221_2;
  FB322_2 = s->m[22]*AlM322_2;
  FM122_2 = FB122_2+FM123_2;
  FM222_2 = FB222_2+FM223_2*C23-FM323_2*S23;
  FM322_2 = FB322_2+FM223_2*S23+FM323_2*C23;
  CM122_2 = CM123_2-FB222_2*s->l[3][22]+FB322_2*s->l[2][22]+s->dpt[2][40]*(FM223_2*S23+FM323_2*C23);
  CM222_2 = CM223_2*C23-CM323_2*S23+FB122_2*s->l[3][22]-FB322_2*s->l[1][22];
  CM322_2 = CM223_2*S23+CM323_2*C23-FB122_2*s->l[2][22]+FB222_2*s->l[1][22]-FM123_2*s->dpt[2][40];
  FB122_3 = s->m[22]*AlM122_3;
  FB222_3 = s->m[22]*AlM221_3;
  FB322_3 = s->m[22]*AlM322_3;
  FM122_3 = FB122_3+FM123_3;
  FM222_3 = FB222_3+FM223_3*C23-FM323_3*S23;
  FM322_3 = FB322_3+FM223_3*S23+FM323_3*C23;
  CM122_3 = CM123_3-FB222_3*s->l[3][22]+FB322_3*s->l[2][22]+s->dpt[2][40]*(FM223_3*S23+FM323_3*C23);
  CM222_3 = CM223_3*C23-CM323_3*S23+FB122_3*s->l[3][22]-FB322_3*s->l[1][22];
  CM322_3 = CM223_3*S23+CM323_3*C23-FB122_3*s->l[2][22]+FB222_3*s->l[1][22]-FM123_3*s->dpt[2][40];
  FB122_4 = s->m[22]*(AlM122_4+OpM221_4*s->l[3][22]-OpM322_4*s->l[2][22]);
  FB222_4 = s->m[22]*(AlM222_4-OpM122_4*s->l[3][22]+OpM322_4*s->l[1][22]);
  FB322_4 = s->m[22]*(AlM322_4+OpM122_4*s->l[2][22]-OpM221_4*s->l[1][22]);
  FM122_4 = FB122_4+FM123_4;
  FM222_4 = FB222_4+FM223_4*C23-FM323_4*S23;
  FM322_4 = FB322_4+FM223_4*S23+FM323_4*C23;
  CM122_4 = CM123_4+s->In[1][22]*OpM122_4-FB222_4*s->l[3][22]+FB322_4*s->l[2][22]+s->dpt[2][40]*(FM223_4*S23+FM323_4*C23
 );
  CM222_4 = s->In[5][22]*OpM221_4+CM223_4*C23-CM323_4*S23+FB122_4*s->l[3][22]-FB322_4*s->l[1][22];
  CM322_4 = s->In[9][22]*OpM322_4+CM223_4*S23+CM323_4*C23-FB122_4*s->l[2][22]+FB222_4*s->l[1][22]-FM123_4*s->dpt[2][40];
  FB122_5 = s->m[22]*(AlM122_5+OpM221_5*s->l[3][22]-OpM322_5*s->l[2][22]);
  FB222_5 = s->m[22]*(AlM222_5-OpM122_5*s->l[3][22]+OpM322_5*s->l[1][22]);
  FB322_5 = s->m[22]*(AlM322_5+OpM122_5*s->l[2][22]-OpM221_5*s->l[1][22]);
  FM122_5 = FB122_5+FM123_5;
  FM222_5 = FB222_5+FM223_5*C23-FM323_5*S23;
  FM322_5 = FB322_5+FM223_5*S23+FM323_5*C23;
  CM122_5 = CM123_5+s->In[1][22]*OpM122_5-FB222_5*s->l[3][22]+FB322_5*s->l[2][22]+s->dpt[2][40]*(FM223_5*S23+FM323_5*C23
 );
  CM222_5 = s->In[5][22]*OpM221_5+CM223_5*C23-CM323_5*S23+FB122_5*s->l[3][22]-FB322_5*s->l[1][22];
  CM322_5 = s->In[9][22]*OpM322_5+CM223_5*S23+CM323_5*C23-FB122_5*s->l[2][22]+FB222_5*s->l[1][22]-FM123_5*s->dpt[2][40];
  FB122_6 = s->m[22]*(AlM122_6+OpM221_6*s->l[3][22]-OpM322_6*s->l[2][22]);
  FB222_6 = s->m[22]*(AlM222_6-OpM122_6*s->l[3][22]+OpM322_6*s->l[1][22]);
  FB322_6 = s->m[22]*(AlM322_6+OpM122_6*s->l[2][22]-OpM221_6*s->l[1][22]);
  FM122_6 = FB122_6+FM123_6;
  FM222_6 = FB222_6+FM223_6*C23-FM323_6*S23;
  FM322_6 = FB322_6+FM223_6*S23+FM323_6*C23;
  CM122_6 = CM123_6+s->In[1][22]*OpM122_6-FB222_6*s->l[3][22]+FB322_6*s->l[2][22]+s->dpt[2][40]*(FM223_6*S23+FM323_6*C23
 );
  CM222_6 = s->In[5][22]*OpM221_6+CM223_6*C23-CM323_6*S23+FB122_6*s->l[3][22]-FB322_6*s->l[1][22];
  CM322_6 = s->In[9][22]*OpM322_6+CM223_6*S23+CM323_6*C23-FB122_6*s->l[2][22]+FB222_6*s->l[1][22]-FM123_6*s->dpt[2][40];
  FB122_19 = s->m[22]*(AlM122_19+OpM221_19*s->l[3][22]-OpM322_19*s->l[2][22]);
  FB222_19 = s->m[22]*(AlM222_19-OpM122_19*s->l[3][22]+OpM322_19*s->l[1][22]);
  FB322_19 = s->m[22]*(AlM322_19+OpM122_19*s->l[2][22]-OpM221_19*s->l[1][22]);
  FM122_19 = FB122_19+FM123_19;
  FM222_19 = FB222_19+FM223_19*C23-FM323_19*S23;
  FM322_19 = FB322_19+FM223_19*S23+FM323_19*C23;
  CM122_19 = CM123_19+s->In[1][22]*OpM122_19-FB222_19*s->l[3][22]+FB322_19*s->l[2][22]+s->dpt[2][40]*(FM223_19*S23+
 FM323_19*C23);
  CM222_19 = s->In[5][22]*OpM221_19+CM223_19*C23-CM323_19*S23+FB122_19*s->l[3][22]-FB322_19*s->l[1][22];
  CM322_19 = s->In[9][22]*OpM322_19+CM223_19*S23+CM323_19*C23-FB122_19*s->l[2][22]+FB222_19*s->l[1][22]-FM123_19*
 s->dpt[2][40];
  FB122_20 = s->m[22]*(AlM122_20-OpM322_20*s->l[2][22]+s->l[3][22]*C21);
  FB222_20 = s->m[22]*(AlM222_20-OpM122_20*s->l[3][22]+OpM322_20*s->l[1][22]);
  FB322_20 = s->m[22]*(AlM322_20+OpM122_20*s->l[2][22]-s->l[1][22]*C21);
  FM122_20 = FB122_20+FM123_20;
  FM222_20 = FB222_20+FM223_20*C23-FM323_20*S23;
  FM322_20 = FB322_20+FM223_20*S23+FM323_20*C23;
  CM122_20 = CM123_20+s->In[1][22]*OpM122_20-FB222_20*s->l[3][22]+FB322_20*s->l[2][22]+s->dpt[2][40]*(FM223_20*S23+
 FM323_20*C23);
  CM222_20 = s->In[5][22]*C21+CM223_20*C23-CM323_20*S23+FB122_20*s->l[3][22]-FB322_20*s->l[1][22];
  CM322_20 = s->In[9][22]*OpM322_20+CM223_20*S23+CM323_20*C23-FB122_20*s->l[2][22]+FB222_20*s->l[1][22]-FM123_20*
 s->dpt[2][40];
  FB122_21 = s->m[22]*(AlM122_21-s->l[2][22]*C22);
  FB222_21 = s->m[22]*(s->dpt[1][36]+s->l[1][22]*C22+s->l[3][22]*S22);
  FB322_21 = -s->m[22]*S22*(s->dpt[2][36]+s->l[2][22]);
  CM222_21 = CM223_21*C23-CM323_21*S23+FB122_21*s->l[3][22]-FB322_21*s->l[1][22];
  CM222_22 = s->In[5][22]+s->m[22]*s->l[1][22]*s->l[1][22]+s->m[22]*s->l[3][22]*s->l[3][22]+C23*(CM224_22+s->In[5][23]*
 C23-FB323_22*s->l[1][23])+S23*(s->m[23]*s->l[2][23]*s->l[2][23]*S23+CM124_22*S24-CM324_22*C24+s->dpt[2][42]*(FM124_22*C24+
 FM324_22*S24)+S23*(s->In[9][23]+s->m[23]*s->l[1][23]*s->l[1][23]));

// = = Block_0_2_0_1_0_6 = = 
 
// Backward Dynamics 

  FA235 = -(s->frc[2][35]-s->m[35]*(AlF234*C35+AlF334*S35));
  FA335 = -(s->frc[3][35]+s->m[35]*(AlF234*S35-AlF334*C35));
  CF135 = -(s->trq[1][35]-s->In[1][35]*OpF134+OM235*OM335*(s->In[5][35]-s->In[9][35]));
  CF235 = -(s->trq[2][35]-s->In[5][35]*(C35*(OpF234+qd[35]*OM334)+S35*(OpF333-qd[35]*OM234))-OM135*OM335*(s->In[1][35]-
 s->In[9][35]));
  CF335 = -(s->trq[3][35]-s->In[9][35]*(C35*(OpF333-qd[35]*OM234)-S35*(OpF234+qd[35]*OM334))+OM135*OM235*(s->In[1][35]-
 s->In[5][35]));
  FB235_1 = s->m[35]*(AlM234_1*C35+AlM333_1*S35);
  FB335_1 = -s->m[35]*(AlM234_1*S35-AlM333_1*C35);
  FB235_2 = s->m[35]*(AlM234_2*C35+AlM333_2*S35);
  FB335_2 = -s->m[35]*(AlM234_2*S35-AlM333_2*C35);
  FB235_3 = s->m[35]*(AlM234_3*C35+AlM333_3*S35);
  FB335_3 = -s->m[35]*(AlM234_3*S35-AlM333_3*C35);
  FB235_4 = s->m[35]*(AlM234_4*C35+AlM334_4*S35);
  FB335_4 = -s->m[35]*(AlM234_4*S35-AlM334_4*C35);
  CM135_4 = s->In[1][35]*OpM134_4;
  CM235_4 = s->In[5][35]*(OpM234_4*C35+OpM333_4*S35);
  CM335_4 = -s->In[9][35]*(OpM234_4*S35-OpM333_4*C35);
  FB235_5 = s->m[35]*(AlM234_5*C35+AlM334_5*S35);
  FB335_5 = -s->m[35]*(AlM234_5*S35-AlM334_5*C35);
  CM135_5 = s->In[1][35]*OpM134_5;
  CM235_5 = s->In[5][35]*(OpM234_5*C35+OpM333_5*S35);
  CM335_5 = -s->In[9][35]*(OpM234_5*S35-OpM333_5*C35);
  FB235_6 = s->m[35]*(AlM234_6*C35+AlM334_6*S35);
  FB335_6 = -s->m[35]*(AlM234_6*S35-AlM334_6*C35);
  CM135_6 = s->In[1][35]*OpM134_6;
  CM235_6 = s->In[5][35]*(OpM234_6*C35+OpM333_6*S35);
  CM335_6 = -s->In[9][35]*(OpM234_6*S35-OpM333_6*C35);
  FB235_19 = s->m[35]*(AlM234_19*C35+AlM334_19*S35);
  FB335_19 = -s->m[35]*(AlM234_19*S35-AlM334_19*C35);
  CM135_19 = s->In[1][35]*OpM134_19;
  CM235_19 = s->In[5][35]*(OpM234_19*C35+OpM333_19*S35);
  CM335_19 = -s->In[9][35]*(OpM234_19*S35-OpM333_19*C35);
  FB235_20 = s->m[35]*(AlM234_20*C35+AlM334_20*S35);
  FB335_20 = -s->m[35]*(AlM234_20*S35-AlM334_20*C35);
  CM135_20 = s->In[1][35]*OpM134_20;
  CM235_20 = s->In[5][35]*(OpM234_20*C35+OpM333_20*S35);
  CM335_20 = -s->In[9][35]*(OpM234_20*S35-OpM333_20*C35);
  FB235_21 = s->m[35]*(AlM234_21*C35+AlM334_21*S35);
  FB335_21 = -s->m[35]*(AlM234_21*S35-AlM334_21*C35);
  CM135_21 = s->In[1][35]*OpM134_21;
  CM235_21 = s->In[5][35]*(OpM234_21*C35+OpM333_21*S35);
  CM335_21 = -s->In[9][35]*(OpM234_21*S35-OpM333_21*C35);
  FB235_29 = s->m[35]*(AlM234_29*C35+AlM334_29*S35);
  FB335_29 = -s->m[35]*(AlM234_29*S35-AlM334_29*C35);
  CM135_29 = s->In[1][35]*OpM134_29;
  CM235_29 = s->In[5][35]*(OpM234_29*C35+OpM333_29*S35);
  CM335_29 = -s->In[9][35]*(OpM234_29*S35-OpM333_29*C35);
  FB235_30 = s->m[35]*(AlM234_30*C35+AlM334_30*S35);
  FB335_30 = -s->m[35]*(AlM234_30*S35-AlM334_30*C35);
  CM135_30 = s->In[1][35]*OpM134_30;
  CM235_30 = s->In[5][35]*(OpM234_30*C35+OpM333_30*S35);
  CM335_30 = -s->In[9][35]*(OpM234_30*S35-OpM333_30*C35);
  FB235_31 = s->m[35]*(AlM234_31*C35+AlM334_31*S35);
  FB335_31 = -s->m[35]*(AlM234_31*S35-AlM334_31*C35);
  CM135_31 = s->In[1][35]*OpM134_31;
  CM235_31 = s->In[5][35]*(OpM234_31*C35+OpM333_31*S35);
  CM335_31 = -s->In[9][35]*(OpM234_31*S35-OpM333_31*C35);
  FB235_32 = s->m[35]*(AlM234_32*C35+AlM334_32*S35);
  FB335_32 = -s->m[35]*(AlM234_32*S35-AlM334_32*C35);
  CM135_32 = s->In[1][35]*OpM134_32;
  CM235_32 = s->In[5][35]*(OpM234_32*C35+C33*S35);
  CM335_32 = -s->In[9][35]*(OpM234_32*S35-C33*C35);
  CM135_33 = s->In[1][35]*S34;
  CM235_33 = s->In[5][35]*C34*C35;
  CM335_33 = -s->In[9][35]*C34*S35;
  FF134 = -(s->frc[1][34]+s->frc[1][35]-s->m[34]*AlF134-s->m[35]*AlF134);
  FF234 = -(s->frc[2][34]-s->m[34]*AlF234-FA235*C35+FA335*S35);
  FF334 = -(s->frc[3][34]-s->m[34]*AlF334-FA235*S35-FA335*C35);
  CF134 = -(s->trq[1][34]-CF135-s->In[1][34]*OpF134+OM234*OM334*(s->In[5][34]-s->In[9][34]));
  CF234 = -(s->trq[2][34]-s->In[5][34]*OpF234-CF235*C35+CF335*S35-OM134*OM334*(s->In[1][34]-s->In[9][34]));
  CF334 = -(s->trq[3][34]-s->In[9][34]*OpF333-CF235*S35-CF335*C35+OM134*OM234*(s->In[1][34]-s->In[5][34]));
  FM134_1 = AlM134_1*(s->m[34]+s->m[35]);
  FM234_1 = s->m[34]*AlM234_1+FB235_1*C35-FB335_1*S35;
  FM334_1 = s->m[34]*AlM333_1+FB235_1*S35+FB335_1*C35;
  FM134_2 = AlM134_2*(s->m[34]+s->m[35]);
  FM234_2 = s->m[34]*AlM234_2+FB235_2*C35-FB335_2*S35;
  FM334_2 = s->m[34]*AlM333_2+FB235_2*S35+FB335_2*C35;
  FM134_3 = AlM134_3*(s->m[34]+s->m[35]);
  FM234_3 = s->m[34]*AlM234_3+FB235_3*C35-FB335_3*S35;
  FM334_3 = s->m[34]*AlM333_3+FB235_3*S35+FB335_3*C35;
  FM134_4 = AlM134_4*(s->m[34]+s->m[35]);
  FM234_4 = s->m[34]*AlM234_4+FB235_4*C35-FB335_4*S35;
  FM334_4 = s->m[34]*AlM334_4+FB235_4*S35+FB335_4*C35;
  CM134_4 = CM135_4+s->In[1][34]*OpM134_4;
  CM234_4 = s->In[5][34]*OpM234_4+CM235_4*C35-CM335_4*S35;
  CM334_4 = s->In[9][34]*OpM333_4+CM235_4*S35+CM335_4*C35;
  FM134_5 = AlM134_5*(s->m[34]+s->m[35]);
  FM234_5 = s->m[34]*AlM234_5+FB235_5*C35-FB335_5*S35;
  FM334_5 = s->m[34]*AlM334_5+FB235_5*S35+FB335_5*C35;
  CM134_5 = CM135_5+s->In[1][34]*OpM134_5;
  CM234_5 = s->In[5][34]*OpM234_5+CM235_5*C35-CM335_5*S35;
  CM334_5 = s->In[9][34]*OpM333_5+CM235_5*S35+CM335_5*C35;
  FM134_6 = AlM134_6*(s->m[34]+s->m[35]);
  FM234_6 = s->m[34]*AlM234_6+FB235_6*C35-FB335_6*S35;
  FM334_6 = s->m[34]*AlM334_6+FB235_6*S35+FB335_6*C35;
  CM134_6 = CM135_6+s->In[1][34]*OpM134_6;
  CM234_6 = s->In[5][34]*OpM234_6+CM235_6*C35-CM335_6*S35;
  CM334_6 = s->In[9][34]*OpM333_6+CM235_6*S35+CM335_6*C35;
  FM134_19 = AlM134_19*(s->m[34]+s->m[35]);
  FM234_19 = s->m[34]*AlM234_19+FB235_19*C35-FB335_19*S35;
  FM334_19 = s->m[34]*AlM334_19+FB235_19*S35+FB335_19*C35;
  CM134_19 = CM135_19+s->In[1][34]*OpM134_19;
  CM234_19 = s->In[5][34]*OpM234_19+CM235_19*C35-CM335_19*S35;
  CM334_19 = s->In[9][34]*OpM333_19+CM235_19*S35+CM335_19*C35;
  FM134_20 = AlM134_20*(s->m[34]+s->m[35]);
  FM234_20 = s->m[34]*AlM234_20+FB235_20*C35-FB335_20*S35;
  FM334_20 = s->m[34]*AlM334_20+FB235_20*S35+FB335_20*C35;
  CM134_20 = CM135_20+s->In[1][34]*OpM134_20;
  CM234_20 = s->In[5][34]*OpM234_20+CM235_20*C35-CM335_20*S35;
  CM334_20 = s->In[9][34]*OpM333_20+CM235_20*S35+CM335_20*C35;
  FM134_21 = AlM134_21*(s->m[34]+s->m[35]);
  FM234_21 = s->m[34]*AlM234_21+FB235_21*C35-FB335_21*S35;
  FM334_21 = s->m[34]*AlM334_21+FB235_21*S35+FB335_21*C35;
  CM134_21 = CM135_21+s->In[1][34]*OpM134_21;
  CM234_21 = s->In[5][34]*OpM234_21+CM235_21*C35-CM335_21*S35;
  CM334_21 = s->In[9][34]*OpM333_21+CM235_21*S35+CM335_21*C35;
  FM134_29 = AlM134_29*(s->m[34]+s->m[35]);
  FM234_29 = s->m[34]*AlM234_29+FB235_29*C35-FB335_29*S35;
  FM334_29 = s->m[34]*AlM334_29+FB235_29*S35+FB335_29*C35;
  CM134_29 = CM135_29+s->In[1][34]*OpM134_29;
  CM234_29 = s->In[5][34]*OpM234_29+CM235_29*C35-CM335_29*S35;
  CM334_29 = s->In[9][34]*OpM333_29+CM235_29*S35+CM335_29*C35;
  FM134_30 = AlM134_30*(s->m[34]+s->m[35]);
  FM234_30 = s->m[34]*AlM234_30+FB235_30*C35-FB335_30*S35;
  FM334_30 = s->m[34]*AlM334_30+FB235_30*S35+FB335_30*C35;
  CM134_30 = CM135_30+s->In[1][34]*OpM134_30;
  CM234_30 = s->In[5][34]*OpM234_30+CM235_30*C35-CM335_30*S35;
  CM334_30 = s->In[9][34]*OpM333_30+CM235_30*S35+CM335_30*C35;
  FM134_31 = AlM134_31*(s->m[34]+s->m[35]);
  FM234_31 = s->m[34]*AlM234_31+FB235_31*C35-FB335_31*S35;
  FM334_31 = s->m[34]*AlM334_31+FB235_31*S35+FB335_31*C35;
  CM134_31 = CM135_31+s->In[1][34]*OpM134_31;
  CM234_31 = s->In[5][34]*OpM234_31+CM235_31*C35-CM335_31*S35;
  CM334_31 = s->In[9][34]*OpM333_31+CM235_31*S35+CM335_31*C35;
  FM134_32 = AlM134_32*(s->m[34]+s->m[35]);
  FM234_32 = s->m[34]*AlM234_32+FB235_32*C35-FB335_32*S35;
  CM134_32 = CM135_32+s->In[1][34]*OpM134_32;
  CM234_32 = s->In[5][34]*OpM234_32+CM235_32*C35-CM335_32*S35;
  CM334_32 = s->In[9][34]*C33+CM235_32*S35+CM335_32*C35;
  CM3433_335 = CM235_33*S35+CM335_33*C35;
  CM334_34 = s->In[9][34]+s->In[5][35]*S35*S35+s->In[9][35]*C35*C35;
  FA133 = -(s->frc[1][33]-s->m[33]*(AlF133+BeF233*s->l[2][33]));
  FA333 = -(s->frc[3][33]-s->m[33]*(AlF333+BeF833*s->l[2][33]));
  FF133 = FA133+FF134*C34-FF234*S34;
  FF233 = -(s->frc[2][33]-s->m[33]*(AlF233+BS533*s->l[2][33])-FF134*S34-FF234*C34);
  FF333 = FA333+FF334;
  CF133 = -(s->trq[1][33]-s->In[1][33]*OpF133-CF134*C34+CF234*S34-FA333*s->l[2][33]-FF334*s->dpt[2][58]+OM233*OM333*(
 s->In[5][33]-s->In[9][33]));
  CF233 = -(s->trq[2][33]-s->In[5][33]*OpF232-CF134*S34-CF234*C34-OM133*OM333*(s->In[1][33]-s->In[9][33]));
  CF333 = -(s->trq[3][33]-CF334-s->In[9][33]*OpF333+FA133*s->l[2][33]+OM133*OM233*(s->In[1][33]-s->In[5][33])+
 s->dpt[2][58]*(FF134*C34-FF234*S34));
  FB133_1 = s->m[33]*AlM133_1;
  FB333_1 = s->m[33]*AlM333_1;
  FM133_1 = FB133_1+FM134_1*C34-FM234_1*S34;
  FM233_1 = s->m[33]*AlM232_1+FM134_1*S34+FM234_1*C34;
  FM333_1 = FB333_1+FM334_1;
  CM133_1 = FB333_1*s->l[2][33]+FM334_1*s->dpt[2][58];
  CM333_1 = -(FB133_1*s->l[2][33]+s->dpt[2][58]*(FM134_1*C34-FM234_1*S34));
  FB133_2 = s->m[33]*AlM133_2;
  FB333_2 = s->m[33]*AlM333_2;
  FM133_2 = FB133_2+FM134_2*C34-FM234_2*S34;
  FM233_2 = s->m[33]*AlM232_2+FM134_2*S34+FM234_2*C34;
  FM333_2 = FB333_2+FM334_2;
  CM133_2 = FB333_2*s->l[2][33]+FM334_2*s->dpt[2][58];
  CM333_2 = -(FB133_2*s->l[2][33]+s->dpt[2][58]*(FM134_2*C34-FM234_2*S34));
  FB133_3 = s->m[33]*AlM133_3;
  FB333_3 = s->m[33]*AlM333_3;
  FM133_3 = FB133_3+FM134_3*C34-FM234_3*S34;
  FM233_3 = s->m[33]*AlM232_3+FM134_3*S34+FM234_3*C34;
  FM333_3 = FB333_3+FM334_3;
  CM133_3 = FB333_3*s->l[2][33]+FM334_3*s->dpt[2][58];
  CM333_3 = -(FB133_3*s->l[2][33]+s->dpt[2][58]*(FM134_3*C34-FM234_3*S34));
  FB133_4 = s->m[33]*(AlM133_4-OpM333_4*s->l[2][33]);
  FB333_4 = s->m[33]*(AlM333_4+OpM133_4*s->l[2][33]);
  FM133_4 = FB133_4+FM134_4*C34-FM234_4*S34;
  FM233_4 = s->m[33]*AlM233_4+FM134_4*S34+FM234_4*C34;
  FM333_4 = FB333_4+FM334_4;
  CM133_4 = s->In[1][33]*OpM133_4+CM134_4*C34-CM234_4*S34+FB333_4*s->l[2][33]+FM334_4*s->dpt[2][58];
  CM233_4 = s->In[5][33]*OpM232_4+CM134_4*S34+CM234_4*C34;
  CM333_4 = CM334_4+s->In[9][33]*OpM333_4-FB133_4*s->l[2][33]-s->dpt[2][58]*(FM134_4*C34-FM234_4*S34);
  FB133_5 = s->m[33]*(AlM133_5-OpM333_5*s->l[2][33]);
  FB333_5 = s->m[33]*(AlM333_5+OpM133_5*s->l[2][33]);
  FM133_5 = FB133_5+FM134_5*C34-FM234_5*S34;
  FM233_5 = s->m[33]*AlM233_5+FM134_5*S34+FM234_5*C34;
  FM333_5 = FB333_5+FM334_5;
  CM133_5 = s->In[1][33]*OpM133_5+CM134_5*C34-CM234_5*S34+FB333_5*s->l[2][33]+FM334_5*s->dpt[2][58];
  CM233_5 = s->In[5][33]*OpM232_5+CM134_5*S34+CM234_5*C34;
  CM333_5 = CM334_5+s->In[9][33]*OpM333_5-FB133_5*s->l[2][33]-s->dpt[2][58]*(FM134_5*C34-FM234_5*S34);
  FB133_6 = s->m[33]*(AlM133_6-OpM333_6*s->l[2][33]);
  FB333_6 = s->m[33]*(AlM333_6+OpM133_6*s->l[2][33]);
  FM133_6 = FB133_6+FM134_6*C34-FM234_6*S34;
  FM233_6 = s->m[33]*AlM233_6+FM134_6*S34+FM234_6*C34;
  FM333_6 = FB333_6+FM334_6;
  CM133_6 = s->In[1][33]*OpM133_6+CM134_6*C34-CM234_6*S34+FB333_6*s->l[2][33]+FM334_6*s->dpt[2][58];
  CM233_6 = s->In[5][33]*OpM232_6+CM134_6*S34+CM234_6*C34;
  CM333_6 = CM334_6+s->In[9][33]*OpM333_6-FB133_6*s->l[2][33]-s->dpt[2][58]*(FM134_6*C34-FM234_6*S34);
  FB133_19 = s->m[33]*(AlM133_19-OpM333_19*s->l[2][33]);
  FB333_19 = s->m[33]*(AlM333_19+OpM133_19*s->l[2][33]);
  FM133_19 = FB133_19+FM134_19*C34-FM234_19*S34;
  FM233_19 = s->m[33]*AlM233_19+FM134_19*S34+FM234_19*C34;
  FM333_19 = FB333_19+FM334_19;
  CM133_19 = s->In[1][33]*OpM133_19+CM134_19*C34-CM234_19*S34+FB333_19*s->l[2][33]+FM334_19*s->dpt[2][58];
  CM233_19 = s->In[5][33]*OpM232_19+CM134_19*S34+CM234_19*C34;
  CM333_19 = CM334_19+s->In[9][33]*OpM333_19-FB133_19*s->l[2][33]-s->dpt[2][58]*(FM134_19*C34-FM234_19*S34);
  FB133_20 = s->m[33]*(AlM133_20-OpM333_20*s->l[2][33]);
  FB333_20 = s->m[33]*(AlM333_20+OpM133_20*s->l[2][33]);
  FM133_20 = FB133_20+FM134_20*C34-FM234_20*S34;
  FM233_20 = s->m[33]*AlM233_20+FM134_20*S34+FM234_20*C34;
  FM333_20 = FB333_20+FM334_20;
  CM133_20 = s->In[1][33]*OpM133_20+CM134_20*C34-CM234_20*S34+FB333_20*s->l[2][33]+FM334_20*s->dpt[2][58];
  CM233_20 = s->In[5][33]*OpM232_20+CM134_20*S34+CM234_20*C34;
  CM333_20 = CM334_20+s->In[9][33]*OpM333_20-FB133_20*s->l[2][33]-s->dpt[2][58]*(FM134_20*C34-FM234_20*S34);
  FB133_21 = s->m[33]*(AlM133_21-OpM333_21*s->l[2][33]);
  FB333_21 = s->m[33]*(AlM333_21+OpM133_21*s->l[2][33]);
  FM133_21 = FB133_21+FM134_21*C34-FM234_21*S34;
  FM233_21 = s->m[33]*AlM233_21+FM134_21*S34+FM234_21*C34;
  FM333_21 = FB333_21+FM334_21;
  CM133_21 = s->In[1][33]*OpM133_21+CM134_21*C34-CM234_21*S34+FB333_21*s->l[2][33]+FM334_21*s->dpt[2][58];
  CM233_21 = s->In[5][33]*OpM232_21+CM134_21*S34+CM234_21*C34;
  CM333_21 = CM334_21+s->In[9][33]*OpM333_21-FB133_21*s->l[2][33]-s->dpt[2][58]*(FM134_21*C34-FM234_21*S34);
  FB133_29 = s->m[33]*(AlM133_29-OpM333_29*s->l[2][33]);
  FB333_29 = s->m[33]*(AlM333_29+OpM133_29*s->l[2][33]);
  FM133_29 = FB133_29+FM134_29*C34-FM234_29*S34;
  FM233_29 = s->m[33]*AlM233_29+FM134_29*S34+FM234_29*C34;
  FM333_29 = FB333_29+FM334_29;
  CM133_29 = s->In[1][33]*OpM133_29+CM134_29*C34-CM234_29*S34+FB333_29*s->l[2][33]+FM334_29*s->dpt[2][58];
  CM233_29 = s->In[5][33]*OpM232_29+CM134_29*S34+CM234_29*C34;
  CM333_29 = CM334_29+s->In[9][33]*OpM333_29-FB133_29*s->l[2][33]-s->dpt[2][58]*(FM134_29*C34-FM234_29*S34);
  FB133_30 = s->m[33]*(AlM133_30-OpM333_30*s->l[2][33]);
  FB333_30 = s->m[33]*(AlM333_30+OpM133_30*s->l[2][33]);
  FM133_30 = FB133_30+FM134_30*C34-FM234_30*S34;
  FM233_30 = s->m[33]*AlM233_30+FM134_30*S34+FM234_30*C34;
  FM333_30 = FB333_30+FM334_30;
  CM133_30 = s->In[1][33]*OpM133_30+CM134_30*C34-CM234_30*S34+FB333_30*s->l[2][33]+FM334_30*s->dpt[2][58];
  CM233_30 = s->In[5][33]*OpM232_30+CM134_30*S34+CM234_30*C34;
  CM333_30 = CM334_30+s->In[9][33]*OpM333_30-FB133_30*s->l[2][33]-s->dpt[2][58]*(FM134_30*C34-FM234_30*S34);
  FB133_31 = s->m[33]*(AlM133_31-OpM333_31*s->l[2][33]);
  FB333_31 = s->m[33]*(AlM333_31+OpM133_31*s->l[2][33]);
  FM133_31 = FB133_31+FM134_31*C34-FM234_31*S34;
  FM333_31 = FB333_31+FM334_31;
  CM133_31 = s->In[1][33]*OpM133_31+CM134_31*C34-CM234_31*S34+FB333_31*s->l[2][33]+FM334_31*s->dpt[2][58];
  CM233_31 = s->In[5][33]*C32+CM134_31*S34+CM234_31*C34;
  CM333_31 = CM334_31+s->In[9][33]*OpM333_31-FB133_31*s->l[2][33]-s->dpt[2][58]*(FM134_31*C34-FM234_31*S34);
  CM3332_234 = CM134_32*S34+CM234_32*C34;
  CM233_33 = s->In[5][33]+C34*(s->In[5][34]*C34+CM235_33*C35-CM335_33*S35)+S34*S34*(s->In[1][34]+s->In[1][35]);
  FA132 = -(s->frc[1][32]-s->m[32]*(AlF132+s->l[2][32]*(BS232-OpF331)));
  FA332 = -(s->frc[3][32]-s->m[32]*(AlF332+s->l[2][32]*(OpF132+OM232*OM332)));
  FF132 = FA132+FF133*C33+FF333*S33;
  FF232 = -(s->frc[2][32]-FF233-s->m[32]*(AlF232-s->l[2][32]*(OM132*OM132+OM332*OM332)));
  FF332 = FA332-FF133*S33+FF333*C33;
  CF132 = -(s->trq[1][32]-s->In[1][32]*OpF132-CF133*C33-CF333*S33-FA332*s->l[2][32]+OM232*OM332*(s->In[5][32]-
 s->In[9][32]));
  CF232 = -(s->trq[2][32]-CF233-s->In[5][32]*OpF232-OM132*OM332*(s->In[1][32]-s->In[9][32])-s->dpt[1][57]*(FF133*S33-
 FF333*C33));
  CF332 = -(s->trq[3][32]-s->In[9][32]*OpF331+CF133*S33-CF333*C33+FA132*s->l[2][32]-FF233*s->dpt[1][57]+OM132*OM232*(
 s->In[1][32]-s->In[5][32]));
  FB132_1 = s->m[32]*AlM132_1;
  FB332_1 = s->m[32]*AlM331_1;
  FM132_1 = FB132_1+FM133_1*C33+FM333_1*S33;
  FM232_1 = FM233_1+s->m[32]*AlM232_1;
  FM332_1 = FB332_1-FM133_1*S33+FM333_1*C33;
  CM321_233 = s->dpt[1][57]*(FM133_1*S33-FM333_1*C33);
  CM132_1 = CM133_1*C33+CM333_1*S33+FB332_1*s->l[2][32];
  CM332_1 = -(CM133_1*S33-CM333_1*C33+FB132_1*s->l[2][32]-FM233_1*s->dpt[1][57]);
  FB132_2 = s->m[32]*AlM132_2;
  FB332_2 = s->m[32]*AlM331_2;
  FM132_2 = FB132_2+FM133_2*C33+FM333_2*S33;
  FM232_2 = FM233_2+s->m[32]*AlM232_2;
  FM332_2 = FB332_2-FM133_2*S33+FM333_2*C33;
  CM322_233 = s->dpt[1][57]*(FM133_2*S33-FM333_2*C33);
  CM132_2 = CM133_2*C33+CM333_2*S33+FB332_2*s->l[2][32];
  CM332_2 = -(CM133_2*S33-CM333_2*C33+FB132_2*s->l[2][32]-FM233_2*s->dpt[1][57]);
  FB132_3 = s->m[32]*AlM132_3;
  FB332_3 = s->m[32]*AlM331_3;
  FM132_3 = FB132_3+FM133_3*C33+FM333_3*S33;
  FM232_3 = FM233_3+s->m[32]*AlM232_3;
  FM332_3 = FB332_3-FM133_3*S33+FM333_3*C33;
  CM323_233 = s->dpt[1][57]*(FM133_3*S33-FM333_3*C33);
  CM132_3 = CM133_3*C33+CM333_3*S33+FB332_3*s->l[2][32];
  CM332_3 = -(CM133_3*S33-CM333_3*C33+FB132_3*s->l[2][32]-FM233_3*s->dpt[1][57]);
  FB132_4 = s->m[32]*(AlM132_4-OpM331_4*s->l[2][32]);
  FB332_4 = s->m[32]*(AlM332_4+OpM132_4*s->l[2][32]);
  FM132_4 = FB132_4+FM133_4*C33+FM333_4*S33;
  FM232_4 = FM233_4+s->m[32]*AlM232_4;
  FM332_4 = FB332_4-FM133_4*S33+FM333_4*C33;
  CM132_4 = s->In[1][32]*OpM132_4+CM133_4*C33+CM333_4*S33+FB332_4*s->l[2][32];
  CM232_4 = CM233_4+s->In[5][32]*OpM232_4+s->dpt[1][57]*(FM133_4*S33-FM333_4*C33);
  CM332_4 = s->In[9][32]*OpM331_4-CM133_4*S33+CM333_4*C33-FB132_4*s->l[2][32]+FM233_4*s->dpt[1][57];
  FB132_5 = s->m[32]*(AlM132_5-OpM331_5*s->l[2][32]);
  FB332_5 = s->m[32]*(AlM332_5+OpM132_5*s->l[2][32]);
  FM132_5 = FB132_5+FM133_5*C33+FM333_5*S33;
  FM232_5 = FM233_5+s->m[32]*AlM232_5;
  FM332_5 = FB332_5-FM133_5*S33+FM333_5*C33;
  CM132_5 = s->In[1][32]*OpM132_5+CM133_5*C33+CM333_5*S33+FB332_5*s->l[2][32];
  CM232_5 = CM233_5+s->In[5][32]*OpM232_5+s->dpt[1][57]*(FM133_5*S33-FM333_5*C33);
  CM332_5 = s->In[9][32]*OpM331_5-CM133_5*S33+CM333_5*C33-FB132_5*s->l[2][32]+FM233_5*s->dpt[1][57];
  FB132_6 = s->m[32]*(AlM132_6-OpM331_6*s->l[2][32]);
  FB332_6 = s->m[32]*(AlM332_6+OpM132_6*s->l[2][32]);
  FM132_6 = FB132_6+FM133_6*C33+FM333_6*S33;
  FM232_6 = FM233_6+s->m[32]*AlM232_6;
  FM332_6 = FB332_6-FM133_6*S33+FM333_6*C33;
  CM132_6 = s->In[1][32]*OpM132_6+CM133_6*C33+CM333_6*S33+FB332_6*s->l[2][32];
  CM232_6 = CM233_6+s->In[5][32]*OpM232_6+s->dpt[1][57]*(FM133_6*S33-FM333_6*C33);
  CM332_6 = s->In[9][32]*OpM331_6-CM133_6*S33+CM333_6*C33-FB132_6*s->l[2][32]+FM233_6*s->dpt[1][57];
  FB132_19 = s->m[32]*(AlM132_19-OpM331_19*s->l[2][32]);
  FB332_19 = s->m[32]*(AlM332_19+OpM132_19*s->l[2][32]);
  FM132_19 = FB132_19+FM133_19*C33+FM333_19*S33;
  FM232_19 = FM233_19+s->m[32]*AlM232_19;
  FM332_19 = FB332_19-FM133_19*S33+FM333_19*C33;
  CM132_19 = s->In[1][32]*OpM132_19+CM133_19*C33+CM333_19*S33+FB332_19*s->l[2][32];
  CM232_19 = CM233_19+s->In[5][32]*OpM232_19+s->dpt[1][57]*(FM133_19*S33-FM333_19*C33);
  CM332_19 = s->In[9][32]*OpM331_19-CM133_19*S33+CM333_19*C33-FB132_19*s->l[2][32]+FM233_19*s->dpt[1][57];
  FB132_20 = s->m[32]*(AlM132_20-OpM331_20*s->l[2][32]);
  FB332_20 = s->m[32]*(AlM332_20+OpM132_20*s->l[2][32]);
  FM132_20 = FB132_20+FM133_20*C33+FM333_20*S33;
  FM232_20 = FM233_20+s->m[32]*AlM232_20;
  FM332_20 = FB332_20-FM133_20*S33+FM333_20*C33;
  CM132_20 = s->In[1][32]*OpM132_20+CM133_20*C33+CM333_20*S33+FB332_20*s->l[2][32];
  CM232_20 = CM233_20+s->In[5][32]*OpM232_20+s->dpt[1][57]*(FM133_20*S33-FM333_20*C33);
  CM332_20 = s->In[9][32]*OpM331_20-CM133_20*S33+CM333_20*C33-FB132_20*s->l[2][32]+FM233_20*s->dpt[1][57];
  FB132_21 = s->m[32]*(AlM132_21-OpM331_21*s->l[2][32]);
  FB332_21 = s->m[32]*(AlM332_21+OpM132_21*s->l[2][32]);
  FM132_21 = FB132_21+FM133_21*C33+FM333_21*S33;
  FM232_21 = FM233_21+s->m[32]*AlM232_21;
  FM332_21 = FB332_21-FM133_21*S33+FM333_21*C33;
  CM132_21 = s->In[1][32]*OpM132_21+CM133_21*C33+CM333_21*S33+FB332_21*s->l[2][32];
  CM232_21 = CM233_21+s->In[5][32]*OpM232_21+s->dpt[1][57]*(FM133_21*S33-FM333_21*C33);
  CM332_21 = s->In[9][32]*OpM331_21-CM133_21*S33+CM333_21*C33-FB132_21*s->l[2][32]+FM233_21*s->dpt[1][57];
  FB132_29 = s->m[32]*(AlM132_29-OpM331_29*s->l[2][32]);
  FB332_29 = s->m[32]*(AlM332_29+OpM132_29*s->l[2][32]);
  FM132_29 = FB132_29+FM133_29*C33+FM333_29*S33;
  FM232_29 = FM233_29+s->m[32]*AlM232_29;
  FM332_29 = FB332_29-FM133_29*S33+FM333_29*C33;
  CM132_29 = s->In[1][32]*OpM132_29+CM133_29*C33+CM333_29*S33+FB332_29*s->l[2][32];
  CM232_29 = CM233_29+s->In[5][32]*OpM232_29+s->dpt[1][57]*(FM133_29*S33-FM333_29*C33);
  CM332_29 = s->In[9][32]*OpM331_29-CM133_29*S33+CM333_29*C33-FB132_29*s->l[2][32]+FM233_29*s->dpt[1][57];
  FB132_30 = s->m[32]*(AlM132_30-s->l[2][32]*S31);
  FB332_30 = s->m[32]*(AlM332_30+OpM132_30*s->l[2][32]);
  FM132_30 = FB132_30+FM133_30*C33+FM333_30*S33;
  FM232_30 = FM233_30+s->m[32]*AlM232_30;
  FM332_30 = FB332_30-FM133_30*S33+FM333_30*C33;
  CM132_30 = s->In[1][32]*OpM132_30+CM133_30*C33+CM333_30*S33+FB332_30*s->l[2][32];
  CM232_30 = CM233_30+s->In[5][32]*OpM232_30+s->dpt[1][57]*(FM133_30*S33-FM333_30*C33);
  CM332_30 = s->In[9][32]*S31-CM133_30*S33+CM333_30*C33-FB132_30*s->l[2][32]+FM233_30*s->dpt[1][57];
  FB332_31 = -s->m[32]*(s->dpt[1][55]-s->l[2][32]*S32);
  CM3231_333 = -(CM133_31*S33-CM333_31*C33-s->dpt[1][57]*(FM134_31*S34+FM234_31*C34));
  CM332_32 = s->In[9][32]+s->m[32]*s->l[2][32]*s->l[2][32]+s->dpt[1][57]*(s->m[33]*s->dpt[1][57]+FM134_32*S34+FM234_32*
 C34)+C33*(CM334_32+s->In[9][33]*C33+s->m[33]*s->l[2][33]*s->l[2][33]*C33-s->dpt[2][58]*(FM134_32*C34-FM234_32*S34))-S33*(
 CM134_32*C34-CM234_32*S34+s->dpt[2][58]*(s->m[34]*AlM334_32+FB235_32*S35+FB335_32*C35)-S33*(s->In[1][33]+s->m[33]*
 s->l[2][33]*s->l[2][33]));
  FA131 = -(s->frc[1][31]-s->m[31]*(AlF131+BS131*s->l[1][31]+BeF231*s->l[2][31]+s->l[3][31]*(BS331+OpF230)));
  FA231 = -(s->frc[2][31]-s->m[31]*(AlF231+BS531*s->l[2][31]+BeF431*s->l[1][31]+s->l[3][31]*(BS631-OpF131)));
  FA331 = -(s->frc[3][31]-s->m[31]*(AlF331+BeF731*s->l[1][31]+BeF831*s->l[2][31]-s->l[3][31]*(OM131*OM131+OM231*OM231)));
  FF131 = FA131+FF132*C32-FF232*S32;
  FF331 = FA331+FF332;
  CF131 = -(s->trq[1][31]-s->In[1][31]*OpF131-CF132*C32+CF232*S32+FA231*s->l[3][31]-FA331*s->l[2][31]-FF332*
 s->dpt[2][55]+OM231*OM331*(s->In[5][31]-s->In[9][31]));
  CF231 = -(s->trq[2][31]-s->In[5][31]*OpF230-CF132*S32-CF232*C32-FA131*s->l[3][31]+FA331*s->l[1][31]+FF332*
 s->dpt[1][55]-OM131*OM331*(s->In[1][31]-s->In[9][31]));
  CF331 = -(s->trq[3][31]-CF332-s->In[9][31]*OpF331+FA131*s->l[2][31]-FA231*s->l[1][31]+OM131*OM231*(s->In[1][31]-
 s->In[5][31])-s->dpt[1][55]*(FF132*S32+FF232*C32)+s->dpt[2][55]*(FF132*C32-FF232*S32));
  FB131_1 = s->m[31]*AlM131_1;
  FB231_1 = s->m[31]*AlM230_1;
  FB331_1 = s->m[31]*AlM331_1;
  FM131_1 = FB131_1+FM132_1*C32-FM232_1*S32;
  FM331_1 = FB331_1+FM332_1;
  CM131_1 = CM132_1*C32-CM321_233*S32-FB231_1*s->l[3][31]+FB331_1*s->l[2][31]+FM332_1*s->dpt[2][55];
  CM231_1 = CM132_1*S32+CM321_233*C32+FB131_1*s->l[3][31]-FB331_1*s->l[1][31]-FM332_1*s->dpt[1][55];
  CM331_1 = CM332_1-FB131_1*s->l[2][31]+FB231_1*s->l[1][31]+s->dpt[1][55]*(FM132_1*S32+FM232_1*C32)-s->dpt[2][55]*(
 FM132_1*C32-FM232_1*S32);
  FB131_2 = s->m[31]*AlM131_2;
  FB231_2 = s->m[31]*AlM230_2;
  FB331_2 = s->m[31]*AlM331_2;
  FM131_2 = FB131_2+FM132_2*C32-FM232_2*S32;
  FM331_2 = FB331_2+FM332_2;
  CM131_2 = CM132_2*C32-CM322_233*S32-FB231_2*s->l[3][31]+FB331_2*s->l[2][31]+FM332_2*s->dpt[2][55];
  CM231_2 = CM132_2*S32+CM322_233*C32+FB131_2*s->l[3][31]-FB331_2*s->l[1][31]-FM332_2*s->dpt[1][55];
  CM331_2 = CM332_2-FB131_2*s->l[2][31]+FB231_2*s->l[1][31]+s->dpt[1][55]*(FM132_2*S32+FM232_2*C32)-s->dpt[2][55]*(
 FM132_2*C32-FM232_2*S32);
  FB131_3 = s->m[31]*AlM131_3;
  FB231_3 = s->m[31]*AlM230_3;
  FB331_3 = s->m[31]*AlM331_3;
  FM131_3 = FB131_3+FM132_3*C32-FM232_3*S32;
  FM331_3 = FB331_3+FM332_3;
  CM131_3 = CM132_3*C32-CM323_233*S32-FB231_3*s->l[3][31]+FB331_3*s->l[2][31]+FM332_3*s->dpt[2][55];
  CM231_3 = CM132_3*S32+CM323_233*C32+FB131_3*s->l[3][31]-FB331_3*s->l[1][31]-FM332_3*s->dpt[1][55];
  CM331_3 = CM332_3-FB131_3*s->l[2][31]+FB231_3*s->l[1][31]+s->dpt[1][55]*(FM132_3*S32+FM232_3*C32)-s->dpt[2][55]*(
 FM132_3*C32-FM232_3*S32);
  FB131_4 = s->m[31]*(AlM131_4+OpM230_4*s->l[3][31]-OpM331_4*s->l[2][31]);
  FB231_4 = s->m[31]*(AlM230_4-OpM131_4*s->l[3][31]+OpM331_4*s->l[1][31]);
  FB331_4 = s->m[31]*(AlM331_4+OpM131_4*s->l[2][31]-OpM230_4*s->l[1][31]);
  FM131_4 = FB131_4+FM132_4*C32-FM232_4*S32;
  FM331_4 = FB331_4+FM332_4;
  CM131_4 = s->In[1][31]*OpM131_4+CM132_4*C32-CM232_4*S32-FB231_4*s->l[3][31]+FB331_4*s->l[2][31]+FM332_4*s->dpt[2][55];
  CM231_4 = s->In[5][31]*OpM230_4+CM132_4*S32+CM232_4*C32+FB131_4*s->l[3][31]-FB331_4*s->l[1][31]-FM332_4*s->dpt[1][55];
  CM331_4 = CM332_4+s->In[9][31]*OpM331_4-FB131_4*s->l[2][31]+FB231_4*s->l[1][31]+s->dpt[1][55]*(FM132_4*S32+FM232_4*C32
 )-s->dpt[2][55]*(FM132_4*C32-FM232_4*S32);
  FB131_5 = s->m[31]*(AlM131_5+OpM230_5*s->l[3][31]-OpM331_5*s->l[2][31]);
  FB231_5 = s->m[31]*(AlM230_5-OpM131_5*s->l[3][31]+OpM331_5*s->l[1][31]);
  FB331_5 = s->m[31]*(AlM331_5+OpM131_5*s->l[2][31]-OpM230_5*s->l[1][31]);
  FM131_5 = FB131_5+FM132_5*C32-FM232_5*S32;
  FM331_5 = FB331_5+FM332_5;
  CM131_5 = s->In[1][31]*OpM131_5+CM132_5*C32-CM232_5*S32-FB231_5*s->l[3][31]+FB331_5*s->l[2][31]+FM332_5*s->dpt[2][55];
  CM231_5 = s->In[5][31]*OpM230_5+CM132_5*S32+CM232_5*C32+FB131_5*s->l[3][31]-FB331_5*s->l[1][31]-FM332_5*s->dpt[1][55];
  CM331_5 = CM332_5+s->In[9][31]*OpM331_5-FB131_5*s->l[2][31]+FB231_5*s->l[1][31]+s->dpt[1][55]*(FM132_5*S32+FM232_5*C32
 )-s->dpt[2][55]*(FM132_5*C32-FM232_5*S32);
  FB131_6 = s->m[31]*(AlM131_6+OpM230_6*s->l[3][31]-OpM331_6*s->l[2][31]);
  FB231_6 = s->m[31]*(AlM230_6-OpM131_6*s->l[3][31]+OpM331_6*s->l[1][31]);
  FB331_6 = s->m[31]*(AlM331_6+OpM131_6*s->l[2][31]-OpM230_6*s->l[1][31]);
  FM131_6 = FB131_6+FM132_6*C32-FM232_6*S32;
  FM331_6 = FB331_6+FM332_6;
  CM131_6 = s->In[1][31]*OpM131_6+CM132_6*C32-CM232_6*S32-FB231_6*s->l[3][31]+FB331_6*s->l[2][31]+FM332_6*s->dpt[2][55];
  CM231_6 = s->In[5][31]*OpM230_6+CM132_6*S32+CM232_6*C32+FB131_6*s->l[3][31]-FB331_6*s->l[1][31]-FM332_6*s->dpt[1][55];
  CM331_6 = CM332_6+s->In[9][31]*OpM331_6-FB131_6*s->l[2][31]+FB231_6*s->l[1][31]+s->dpt[1][55]*(FM132_6*S32+FM232_6*C32
 )-s->dpt[2][55]*(FM132_6*C32-FM232_6*S32);
  FB131_19 = s->m[31]*(AlM131_19+OpM230_19*s->l[3][31]-OpM331_19*s->l[2][31]);
  FB231_19 = s->m[31]*(AlM230_19-OpM131_19*s->l[3][31]+OpM331_19*s->l[1][31]);
  FB331_19 = s->m[31]*(AlM331_19+OpM131_19*s->l[2][31]-OpM230_19*s->l[1][31]);
  FM131_19 = FB131_19+FM132_19*C32-FM232_19*S32;
  FM331_19 = FB331_19+FM332_19;
  CM131_19 = s->In[1][31]*OpM131_19+CM132_19*C32-CM232_19*S32-FB231_19*s->l[3][31]+FB331_19*s->l[2][31]+FM332_19*
 s->dpt[2][55];
  CM231_19 = s->In[5][31]*OpM230_19+CM132_19*S32+CM232_19*C32+FB131_19*s->l[3][31]-FB331_19*s->l[1][31]-FM332_19*
 s->dpt[1][55];
  CM331_19 = CM332_19+s->In[9][31]*OpM331_19-FB131_19*s->l[2][31]+FB231_19*s->l[1][31]+s->dpt[1][55]*(FM132_19*S32+
 FM232_19*C32)-s->dpt[2][55]*(FM132_19*C32-FM232_19*S32);
  FB131_20 = s->m[31]*(AlM131_20+OpM230_20*s->l[3][31]-OpM331_20*s->l[2][31]);
  FB231_20 = s->m[31]*(AlM230_20-OpM131_20*s->l[3][31]+OpM331_20*s->l[1][31]);
  FB331_20 = s->m[31]*(AlM331_20+OpM131_20*s->l[2][31]-OpM230_20*s->l[1][31]);
  FM131_20 = FB131_20+FM132_20*C32-FM232_20*S32;
  FM331_20 = FB331_20+FM332_20;
  CM131_20 = s->In[1][31]*OpM131_20+CM132_20*C32-CM232_20*S32-FB231_20*s->l[3][31]+FB331_20*s->l[2][31]+FM332_20*
 s->dpt[2][55];
  CM231_20 = s->In[5][31]*OpM230_20+CM132_20*S32+CM232_20*C32+FB131_20*s->l[3][31]-FB331_20*s->l[1][31]-FM332_20*
 s->dpt[1][55];
  CM331_20 = CM332_20+s->In[9][31]*OpM331_20-FB131_20*s->l[2][31]+FB231_20*s->l[1][31]+s->dpt[1][55]*(FM132_20*S32+
 FM232_20*C32)-s->dpt[2][55]*(FM132_20*C32-FM232_20*S32);
  FB131_21 = s->m[31]*(AlM131_21+OpM230_21*s->l[3][31]-OpM331_21*s->l[2][31]);
  FB231_21 = s->m[31]*(AlM230_21-OpM131_21*s->l[3][31]+OpM331_21*s->l[1][31]);
  FB331_21 = s->m[31]*(AlM331_21+OpM131_21*s->l[2][31]-OpM230_21*s->l[1][31]);
  FM131_21 = FB131_21+FM132_21*C32-FM232_21*S32;
  FM331_21 = FB331_21+FM332_21;
  CM131_21 = s->In[1][31]*OpM131_21+CM132_21*C32-CM232_21*S32-FB231_21*s->l[3][31]+FB331_21*s->l[2][31]+FM332_21*
 s->dpt[2][55];
  CM231_21 = s->In[5][31]*OpM230_21+CM132_21*S32+CM232_21*C32+FB131_21*s->l[3][31]-FB331_21*s->l[1][31]-FM332_21*
 s->dpt[1][55];
  CM331_21 = CM332_21+s->In[9][31]*OpM331_21-FB131_21*s->l[2][31]+FB231_21*s->l[1][31]+s->dpt[1][55]*(FM132_21*S32+
 FM232_21*C32)-s->dpt[2][55]*(FM132_21*C32-FM232_21*S32);
  FB131_29 = s->m[31]*(AlM131_29-OpM331_29*s->l[2][31]+s->l[3][31]*C30);
  FB231_29 = -s->m[31]*(OpM131_29*s->l[3][31]-OpM331_29*s->l[1][31]);
  FB331_29 = s->m[31]*(AlM331_29+OpM131_29*s->l[2][31]-s->l[1][31]*C30);
  FM131_29 = FB131_29+FM132_29*C32-FM232_29*S32;
  FM331_29 = FB331_29+FM332_29;
  CM131_29 = s->In[1][31]*OpM131_29+CM132_29*C32-CM232_29*S32-FB231_29*s->l[3][31]+FB331_29*s->l[2][31]+FM332_29*
 s->dpt[2][55];
  CM231_29 = s->In[5][31]*C30+CM132_29*S32+CM232_29*C32+FB131_29*s->l[3][31]-FB331_29*s->l[1][31]-FM332_29*s->dpt[1][55];
  CM331_29 = CM332_29+s->In[9][31]*OpM331_29-FB131_29*s->l[2][31]+FB231_29*s->l[1][31]+s->dpt[1][55]*(FM132_29*S32+
 FM232_29*C32)-s->dpt[2][55]*(FM132_29*C32-FM232_29*S32);
  FB131_30 = -s->m[31]*S31*(s->dpt[2][53]+s->l[2][31]);
  FB231_30 = s->m[31]*(s->l[1][31]*S31-s->l[3][31]*C31);
  FB331_30 = s->m[31]*C31*(s->dpt[2][53]+s->l[2][31]);
  CM231_30 = CM132_30*S32+CM232_30*C32+FB131_30*s->l[3][31]-FB331_30*s->l[1][31]-FM332_30*s->dpt[1][55];
  CM231_31 = s->In[5][31]+s->m[31]*s->l[1][31]*s->l[1][31]+s->m[31]*s->l[3][31]*s->l[3][31]-s->dpt[1][55]*(FB332_31-
 FM133_31*S33+FM333_31*C33)+C32*(CM233_31+s->In[5][32]*C32+s->dpt[1][57]*(FM133_31*S33-FM333_31*C33))+S32*(s->In[1][32]*S32+
 CM133_31*C33+CM333_31*S33+FB332_31*s->l[2][32]);
  FA130 = -(s->frc[1][30]-s->m[30]*(AlF130+BeF230*s->l[2][30]-s->l[1][30]*(OM230*OM230+OM330*OM330)+s->l[3][30]*(BS330+
 OpF230)));
  FA230 = -(s->frc[2][30]-s->m[30]*(AlF230+BS530*s->l[2][30]+s->l[1][30]*(BS230+OpF330)+s->l[3][30]*(BS630-OpF129)));
  FA330 = -(s->frc[3][30]-s->m[30]*(AlF330+BeF830*s->l[2][30]+s->l[1][30]*(BS330-OpF230)-s->l[3][30]*(OM130*OM130+OM230*
 OM230)));
  FF130 = FA130+FF131*C31+FF331*S31;
  FF230 = FA230+FA231+FF132*S32+FF232*C32;
  FF330 = FA330-FF131*S31+FF331*C31;
  CF130 = -(s->trq[1][30]-s->In[1][30]*OpF129-CF131*C31-CF331*S31+FA230*s->l[3][30]-FA330*s->l[2][30]+OM230*OM330*(
 s->In[5][30]-s->In[9][30])+s->dpt[2][53]*(FF131*S31-FF331*C31));
  CF230 = -(s->trq[2][30]-CF231-s->In[5][30]*OpF230-FA130*s->l[3][30]+FA330*s->l[1][30]-OM130*OM330*(s->In[1][30]-
 s->In[9][30]));
  CF330 = -(s->trq[3][30]-s->In[9][30]*OpF330+CF131*S31-CF331*C31+FA130*s->l[2][30]-FA230*s->l[1][30]+OM130*OM230*(
 s->In[1][30]-s->In[5][30])+s->dpt[2][53]*(FF131*C31+FF331*S31));
  FB130_1 = s->m[30]*AlM129_1;
  FB230_1 = s->m[30]*AlM230_1;
  FB330_1 = s->m[30]*AlM330_1;
  FM130_1 = FB130_1+FM131_1*C31+FM331_1*S31;
  FM230_1 = FB230_1+FB231_1+FM132_1*S32+FM232_1*C32;
  FM330_1 = FB330_1-FM131_1*S31+FM331_1*C31;
  CM130_1 = CM131_1*C31+CM331_1*S31-FB230_1*s->l[3][30]+FB330_1*s->l[2][30]-s->dpt[2][53]*(FM131_1*S31-FM331_1*C31);
  CM230_1 = CM231_1+FB130_1*s->l[3][30]-FB330_1*s->l[1][30];
  CM330_1 = -(CM131_1*S31-CM331_1*C31+FB130_1*s->l[2][30]-FB230_1*s->l[1][30]+s->dpt[2][53]*(FM131_1*C31+FM331_1*S31));
  FB130_2 = s->m[30]*AlM129_2;
  FB230_2 = s->m[30]*AlM230_2;
  FB330_2 = s->m[30]*AlM330_2;
  FM130_2 = FB130_2+FM131_2*C31+FM331_2*S31;
  FM230_2 = FB230_2+FB231_2+FM132_2*S32+FM232_2*C32;
  FM330_2 = FB330_2-FM131_2*S31+FM331_2*C31;
  CM130_2 = CM131_2*C31+CM331_2*S31-FB230_2*s->l[3][30]+FB330_2*s->l[2][30]-s->dpt[2][53]*(FM131_2*S31-FM331_2*C31);
  CM230_2 = CM231_2+FB130_2*s->l[3][30]-FB330_2*s->l[1][30];
  CM330_2 = -(CM131_2*S31-CM331_2*C31+FB130_2*s->l[2][30]-FB230_2*s->l[1][30]+s->dpt[2][53]*(FM131_2*C31+FM331_2*S31));
  FB130_3 = s->m[30]*AlM129_3;
  FB230_3 = s->m[30]*AlM230_3;
  FB330_3 = s->m[30]*AlM330_3;
  FM130_3 = FB130_3+FM131_3*C31+FM331_3*S31;
  FM230_3 = FB230_3+FB231_3+FM132_3*S32+FM232_3*C32;
  FM330_3 = FB330_3-FM131_3*S31+FM331_3*C31;
  CM130_3 = CM131_3*C31+CM331_3*S31-FB230_3*s->l[3][30]+FB330_3*s->l[2][30]-s->dpt[2][53]*(FM131_3*S31-FM331_3*C31);
  CM230_3 = CM231_3+FB130_3*s->l[3][30]-FB330_3*s->l[1][30];
  CM330_3 = -(CM131_3*S31-CM331_3*C31+FB130_3*s->l[2][30]-FB230_3*s->l[1][30]+s->dpt[2][53]*(FM131_3*C31+FM331_3*S31));
  FB130_4 = s->m[30]*(AlM130_4+OpM230_4*s->l[3][30]-OpM330_4*s->l[2][30]);
  FB230_4 = s->m[30]*(AlM230_4-OpM129_4*s->l[3][30]+OpM330_4*s->l[1][30]);
  FB330_4 = s->m[30]*(AlM330_4+OpM129_4*s->l[2][30]-OpM230_4*s->l[1][30]);
  FM130_4 = FB130_4+FM131_4*C31+FM331_4*S31;
  FM230_4 = FB230_4+FB231_4+FM132_4*S32+FM232_4*C32;
  FM330_4 = FB330_4-FM131_4*S31+FM331_4*C31;
  CM130_4 = s->In[1][30]*OpM129_4+CM131_4*C31+CM331_4*S31-FB230_4*s->l[3][30]+FB330_4*s->l[2][30]-s->dpt[2][53]*(FM131_4
 *S31-FM331_4*C31);
  CM230_4 = CM231_4+s->In[5][30]*OpM230_4+FB130_4*s->l[3][30]-FB330_4*s->l[1][30];
  CM330_4 = s->In[9][30]*OpM330_4-CM131_4*S31+CM331_4*C31-FB130_4*s->l[2][30]+FB230_4*s->l[1][30]-s->dpt[2][53]*(FM131_4
 *C31+FM331_4*S31);
  FB130_5 = s->m[30]*(AlM130_5+OpM230_5*s->l[3][30]-OpM330_5*s->l[2][30]);
  FB230_5 = s->m[30]*(AlM230_5-OpM129_5*s->l[3][30]+OpM330_5*s->l[1][30]);
  FB330_5 = s->m[30]*(AlM330_5+OpM129_5*s->l[2][30]-OpM230_5*s->l[1][30]);
  FM130_5 = FB130_5+FM131_5*C31+FM331_5*S31;
  FM230_5 = FB230_5+FB231_5+FM132_5*S32+FM232_5*C32;
  FM330_5 = FB330_5-FM131_5*S31+FM331_5*C31;
  CM130_5 = s->In[1][30]*OpM129_5+CM131_5*C31+CM331_5*S31-FB230_5*s->l[3][30]+FB330_5*s->l[2][30]-s->dpt[2][53]*(FM131_5
 *S31-FM331_5*C31);
  CM230_5 = CM231_5+s->In[5][30]*OpM230_5+FB130_5*s->l[3][30]-FB330_5*s->l[1][30];
  CM330_5 = s->In[9][30]*OpM330_5-CM131_5*S31+CM331_5*C31-FB130_5*s->l[2][30]+FB230_5*s->l[1][30]-s->dpt[2][53]*(FM131_5
 *C31+FM331_5*S31);
  FB130_6 = s->m[30]*(AlM130_6+OpM230_6*s->l[3][30]-OpM330_6*s->l[2][30]);
  FB230_6 = s->m[30]*(AlM230_6-OpM129_6*s->l[3][30]+OpM330_6*s->l[1][30]);
  FB330_6 = s->m[30]*(AlM330_6+OpM129_6*s->l[2][30]-OpM230_6*s->l[1][30]);
  FM130_6 = FB130_6+FM131_6*C31+FM331_6*S31;
  FM230_6 = FB230_6+FB231_6+FM132_6*S32+FM232_6*C32;
  FM330_6 = FB330_6-FM131_6*S31+FM331_6*C31;
  CM130_6 = s->In[1][30]*OpM129_6+CM131_6*C31+CM331_6*S31-FB230_6*s->l[3][30]+FB330_6*s->l[2][30]-s->dpt[2][53]*(FM131_6
 *S31-FM331_6*C31);
  CM230_6 = CM231_6+s->In[5][30]*OpM230_6+FB130_6*s->l[3][30]-FB330_6*s->l[1][30];
  CM330_6 = s->In[9][30]*OpM330_6-CM131_6*S31+CM331_6*C31-FB130_6*s->l[2][30]+FB230_6*s->l[1][30]-s->dpt[2][53]*(FM131_6
 *C31+FM331_6*S31);
  FB130_19 = s->m[30]*(AlM130_19+OpM230_19*s->l[3][30]-OpM330_19*s->l[2][30]);
  FB230_19 = s->m[30]*(AlM230_19-OpM129_19*s->l[3][30]+OpM330_19*s->l[1][30]);
  FB330_19 = s->m[30]*(AlM330_19+OpM129_19*s->l[2][30]-OpM230_19*s->l[1][30]);
  FM130_19 = FB130_19+FM131_19*C31+FM331_19*S31;
  FM230_19 = FB230_19+FB231_19+FM132_19*S32+FM232_19*C32;
  FM330_19 = FB330_19-FM131_19*S31+FM331_19*C31;
  CM130_19 = s->In[1][30]*OpM129_19+CM131_19*C31+CM331_19*S31-FB230_19*s->l[3][30]+FB330_19*s->l[2][30]-s->dpt[2][53]*(
 FM131_19*S31-FM331_19*C31);
  CM230_19 = CM231_19+s->In[5][30]*OpM230_19+FB130_19*s->l[3][30]-FB330_19*s->l[1][30];
  CM330_19 = s->In[9][30]*OpM330_19-CM131_19*S31+CM331_19*C31-FB130_19*s->l[2][30]+FB230_19*s->l[1][30]-s->dpt[2][53]*(
 FM131_19*C31+FM331_19*S31);
  FB130_20 = s->m[30]*(AlM130_20+OpM230_20*s->l[3][30]-OpM330_20*s->l[2][30]);
  FB230_20 = s->m[30]*(AlM230_20-OpM129_20*s->l[3][30]+OpM330_20*s->l[1][30]);
  FB330_20 = s->m[30]*(AlM330_20+OpM129_20*s->l[2][30]-OpM230_20*s->l[1][30]);
  FM130_20 = FB130_20+FM131_20*C31+FM331_20*S31;
  FM230_20 = FB230_20+FB231_20+FM132_20*S32+FM232_20*C32;
  FM330_20 = FB330_20-FM131_20*S31+FM331_20*C31;
  CM130_20 = s->In[1][30]*OpM129_20+CM131_20*C31+CM331_20*S31-FB230_20*s->l[3][30]+FB330_20*s->l[2][30]-s->dpt[2][53]*(
 FM131_20*S31-FM331_20*C31);
  CM230_20 = CM231_20+s->In[5][30]*OpM230_20+FB130_20*s->l[3][30]-FB330_20*s->l[1][30];
  CM330_20 = s->In[9][30]*OpM330_20-CM131_20*S31+CM331_20*C31-FB130_20*s->l[2][30]+FB230_20*s->l[1][30]-s->dpt[2][53]*(
 FM131_20*C31+FM331_20*S31);
  FB130_21 = s->m[30]*(AlM130_21+OpM230_21*s->l[3][30]-OpM330_21*s->l[2][30]);
  FB230_21 = s->m[30]*(AlM230_21+OpM330_21*s->l[1][30]+s->l[3][30]*S29);
  FB330_21 = s->m[30]*(AlM330_21-OpM230_21*s->l[1][30]-s->l[2][30]*S29);
  FM130_21 = FB130_21+FM131_21*C31+FM331_21*S31;
  FM230_21 = FB230_21+FB231_21+FM132_21*S32+FM232_21*C32;
  FM330_21 = FB330_21-FM131_21*S31+FM331_21*C31;
  CM130_21 = CM131_21*C31+CM331_21*S31-s->dpt[2][53]*(FM131_21*S31-FM331_21*C31)-s->In[1][30]*S29-FB230_21*s->l[3][30]+
 FB330_21*s->l[2][30];
  CM230_21 = CM231_21+s->In[5][30]*OpM230_21+FB130_21*s->l[3][30]-FB330_21*s->l[1][30];
  CM330_21 = s->In[9][30]*OpM330_21-CM131_21*S31+CM331_21*C31-FB130_21*s->l[2][30]+FB230_21*s->l[1][30]-s->dpt[2][53]*(
 FM131_21*C31+FM331_21*S31);
  FB130_29 = s->m[30]*(s->l[2][30]*S30+s->l[3][30]*C30);
  FB230_29 = -s->m[30]*s->l[1][30]*S30;
  FB330_29 = -s->m[30]*s->l[1][30]*C30;
  CM130_29 = CM131_29*C31+CM331_29*S31-FB230_29*s->l[3][30]+FB330_29*s->l[2][30]-s->dpt[2][53]*(FM131_29*S31-FM331_29*
 C31);
  CM130_30 = s->In[1][30]+s->m[30]*s->l[2][30]*s->l[2][30]+s->m[30]*s->l[3][30]*s->l[3][30]+s->dpt[2][53]*(C31*(FB331_30
 +FM332_30)-S31*(FB131_30+FM132_30*C32-FM232_30*S32))+C31*(s->In[1][31]*C31+CM132_30*C32-CM232_30*S32-FB231_30*s->l[3][31]+
 FB331_30*s->l[2][31]+FM332_30*s->dpt[2][55])+S31*(CM332_30+s->In[9][31]*S31-FB131_30*s->l[2][31]+FB231_30*s->l[1][31]+
 s->dpt[1][55]*(FM132_30*S32+FM232_30*C32)-s->dpt[2][55]*(FM132_30*C32-FM232_30*S32));
  FA129 = -(s->frc[1][29]-s->m[29]*(AlF129+BeF229*s->l[2][29]-s->l[1][29]*(OM229*OM229+OM329*OM329)+s->l[3][29]*(BS329+
 OpF221)));
  FA229 = -(s->frc[2][29]-s->m[29]*(AlF229+BS529*s->l[2][29]+s->l[1][29]*(BS229+OpF329)+s->l[3][29]*(BS629-OpF129)));
  FA329 = -(s->frc[3][29]-s->m[29]*(AlF329+BeF829*s->l[2][29]+s->l[1][29]*(BS329-OpF221)-s->l[3][29]*(OM129*OM129+OM229*
 OM229)));
  FF129 = FA129+FF130;
  FF229 = FA229+FF230*C30-FF330*S30;
  FF329 = FA329+FF230*S30+FF330*C30;
  CF129 = -(s->trq[1][29]-CF130-s->In[1][29]*OpF129+FA229*s->l[3][29]-FA329*s->l[2][29]+OM229*OM329*(s->In[5][29]-
 s->In[9][29])-s->dpt[2][51]*(FF230*S30+FF330*C30));
  CF229 = -(s->trq[2][29]-s->In[5][29]*OpF221-CF230*C30+CF330*S30-FA129*s->l[3][29]+FA329*s->l[1][29]-OM129*OM329*(
 s->In[1][29]-s->In[9][29]));
  CF329 = -(s->trq[3][29]-s->In[9][29]*OpF329-CF230*S30-CF330*C30+FA129*s->l[2][29]-FA229*s->l[1][29]+FF130*
 s->dpt[2][51]+OM129*OM229*(s->In[1][29]-s->In[5][29]));
  FB129_1 = s->m[29]*AlM129_1;
  FB229_1 = s->m[29]*AlM221_1;
  FB329_1 = s->m[29]*AlM329_1;
  FM129_1 = FB129_1+FM130_1;
  FM229_1 = FB229_1+FM230_1*C30-FM330_1*S30;
  FM329_1 = FB329_1+FM230_1*S30+FM330_1*C30;
  CM129_1 = CM130_1-FB229_1*s->l[3][29]+FB329_1*s->l[2][29]+s->dpt[2][51]*(FM230_1*S30+FM330_1*C30);
  CM229_1 = CM230_1*C30-CM330_1*S30+FB129_1*s->l[3][29]-FB329_1*s->l[1][29];
  CM329_1 = CM230_1*S30+CM330_1*C30-FB129_1*s->l[2][29]+FB229_1*s->l[1][29]-FM130_1*s->dpt[2][51];
  FB129_2 = s->m[29]*AlM129_2;
  FB229_2 = s->m[29]*AlM221_2;
  FB329_2 = s->m[29]*AlM329_2;
  FM129_2 = FB129_2+FM130_2;
  FM229_2 = FB229_2+FM230_2*C30-FM330_2*S30;
  FM329_2 = FB329_2+FM230_2*S30+FM330_2*C30;
  CM129_2 = CM130_2-FB229_2*s->l[3][29]+FB329_2*s->l[2][29]+s->dpt[2][51]*(FM230_2*S30+FM330_2*C30);
  CM229_2 = CM230_2*C30-CM330_2*S30+FB129_2*s->l[3][29]-FB329_2*s->l[1][29];
  CM329_2 = CM230_2*S30+CM330_2*C30-FB129_2*s->l[2][29]+FB229_2*s->l[1][29]-FM130_2*s->dpt[2][51];
  FB129_3 = s->m[29]*AlM129_3;
  FB229_3 = s->m[29]*AlM221_3;
  FB329_3 = s->m[29]*AlM329_3;
  FM129_3 = FB129_3+FM130_3;
  FM229_3 = FB229_3+FM230_3*C30-FM330_3*S30;
  FM329_3 = FB329_3+FM230_3*S30+FM330_3*C30;
  CM129_3 = CM130_3-FB229_3*s->l[3][29]+FB329_3*s->l[2][29]+s->dpt[2][51]*(FM230_3*S30+FM330_3*C30);
  CM229_3 = CM230_3*C30-CM330_3*S30+FB129_3*s->l[3][29]-FB329_3*s->l[1][29];
  CM329_3 = CM230_3*S30+CM330_3*C30-FB129_3*s->l[2][29]+FB229_3*s->l[1][29]-FM130_3*s->dpt[2][51];
  FB129_4 = s->m[29]*(AlM129_4+OpM221_4*s->l[3][29]-OpM329_4*s->l[2][29]);
  FB229_4 = s->m[29]*(AlM229_4-OpM129_4*s->l[3][29]+OpM329_4*s->l[1][29]);
  FB329_4 = s->m[29]*(AlM329_4+OpM129_4*s->l[2][29]-OpM221_4*s->l[1][29]);
  FM129_4 = FB129_4+FM130_4;
  FM229_4 = FB229_4+FM230_4*C30-FM330_4*S30;
  FM329_4 = FB329_4+FM230_4*S30+FM330_4*C30;
  CM129_4 = CM130_4+s->In[1][29]*OpM129_4-FB229_4*s->l[3][29]+FB329_4*s->l[2][29]+s->dpt[2][51]*(FM230_4*S30+FM330_4*C30
 );
  CM229_4 = s->In[5][29]*OpM221_4+CM230_4*C30-CM330_4*S30+FB129_4*s->l[3][29]-FB329_4*s->l[1][29];
  CM329_4 = s->In[9][29]*OpM329_4+CM230_4*S30+CM330_4*C30-FB129_4*s->l[2][29]+FB229_4*s->l[1][29]-FM130_4*s->dpt[2][51];
  FB129_5 = s->m[29]*(AlM129_5+OpM221_5*s->l[3][29]-OpM329_5*s->l[2][29]);
  FB229_5 = s->m[29]*(AlM229_5-OpM129_5*s->l[3][29]+OpM329_5*s->l[1][29]);
  FB329_5 = s->m[29]*(AlM329_5+OpM129_5*s->l[2][29]-OpM221_5*s->l[1][29]);
  FM129_5 = FB129_5+FM130_5;
  FM229_5 = FB229_5+FM230_5*C30-FM330_5*S30;
  FM329_5 = FB329_5+FM230_5*S30+FM330_5*C30;
  CM129_5 = CM130_5+s->In[1][29]*OpM129_5-FB229_5*s->l[3][29]+FB329_5*s->l[2][29]+s->dpt[2][51]*(FM230_5*S30+FM330_5*C30
 );
  CM229_5 = s->In[5][29]*OpM221_5+CM230_5*C30-CM330_5*S30+FB129_5*s->l[3][29]-FB329_5*s->l[1][29];
  CM329_5 = s->In[9][29]*OpM329_5+CM230_5*S30+CM330_5*C30-FB129_5*s->l[2][29]+FB229_5*s->l[1][29]-FM130_5*s->dpt[2][51];
  FB129_6 = s->m[29]*(AlM129_6+OpM221_6*s->l[3][29]-OpM329_6*s->l[2][29]);
  FB229_6 = s->m[29]*(AlM229_6-OpM129_6*s->l[3][29]+OpM329_6*s->l[1][29]);
  FB329_6 = s->m[29]*(AlM329_6+OpM129_6*s->l[2][29]-OpM221_6*s->l[1][29]);
  FM129_6 = FB129_6+FM130_6;
  FM229_6 = FB229_6+FM230_6*C30-FM330_6*S30;
  FM329_6 = FB329_6+FM230_6*S30+FM330_6*C30;
  CM129_6 = CM130_6+s->In[1][29]*OpM129_6-FB229_6*s->l[3][29]+FB329_6*s->l[2][29]+s->dpt[2][51]*(FM230_6*S30+FM330_6*C30
 );
  CM229_6 = s->In[5][29]*OpM221_6+CM230_6*C30-CM330_6*S30+FB129_6*s->l[3][29]-FB329_6*s->l[1][29];
  CM329_6 = s->In[9][29]*OpM329_6+CM230_6*S30+CM330_6*C30-FB129_6*s->l[2][29]+FB229_6*s->l[1][29]-FM130_6*s->dpt[2][51];
  FB129_19 = s->m[29]*(AlM129_19+OpM221_19*s->l[3][29]-OpM329_19*s->l[2][29]);
  FB229_19 = s->m[29]*(AlM229_19-OpM129_19*s->l[3][29]+OpM329_19*s->l[1][29]);
  FB329_19 = s->m[29]*(AlM329_19+OpM129_19*s->l[2][29]-OpM221_19*s->l[1][29]);
  FM129_19 = FB129_19+FM130_19;
  FM229_19 = FB229_19+FM230_19*C30-FM330_19*S30;
  FM329_19 = FB329_19+FM230_19*S30+FM330_19*C30;
  CM129_19 = CM130_19+s->In[1][29]*OpM129_19-FB229_19*s->l[3][29]+FB329_19*s->l[2][29]+s->dpt[2][51]*(FM230_19*S30+
 FM330_19*C30);
  CM229_19 = s->In[5][29]*OpM221_19+CM230_19*C30-CM330_19*S30+FB129_19*s->l[3][29]-FB329_19*s->l[1][29];
  CM329_19 = s->In[9][29]*OpM329_19+CM230_19*S30+CM330_19*C30-FB129_19*s->l[2][29]+FB229_19*s->l[1][29]-FM130_19*
 s->dpt[2][51];
  FB129_20 = s->m[29]*(AlM129_20-OpM329_20*s->l[2][29]+s->l[3][29]*C21);
  FB229_20 = s->m[29]*(AlM229_20-OpM129_20*s->l[3][29]+OpM329_20*s->l[1][29]);
  FB329_20 = s->m[29]*(AlM329_20+OpM129_20*s->l[2][29]-s->l[1][29]*C21);
  FM129_20 = FB129_20+FM130_20;
  FM229_20 = FB229_20+FM230_20*C30-FM330_20*S30;
  FM329_20 = FB329_20+FM230_20*S30+FM330_20*C30;
  CM129_20 = CM130_20+s->In[1][29]*OpM129_20-FB229_20*s->l[3][29]+FB329_20*s->l[2][29]+s->dpt[2][51]*(FM230_20*S30+
 FM330_20*C30);
  CM229_20 = s->In[5][29]*C21+CM230_20*C30-CM330_20*S30+FB129_20*s->l[3][29]-FB329_20*s->l[1][29];
  CM329_20 = s->In[9][29]*OpM329_20+CM230_20*S30+CM330_20*C30-FB129_20*s->l[2][29]+FB229_20*s->l[1][29]-FM130_20*
 s->dpt[2][51];
  FB129_21 = s->m[29]*(AlM129_21-s->l[2][29]*C29);
  FB229_21 = s->m[29]*(s->dpt[1][37]+s->l[1][29]*C29+s->l[3][29]*S29);
  FB329_21 = -s->m[29]*S29*(s->dpt[2][37]+s->l[2][29]);
  CM229_21 = CM230_21*C30-CM330_21*S30+FB129_21*s->l[3][29]-FB329_21*s->l[1][29];
  CM229_29 = s->In[5][29]+s->m[29]*s->l[1][29]*s->l[1][29]+s->m[29]*s->l[3][29]*s->l[3][29]+C30*(CM231_29+s->In[5][30]*
 C30+FB130_29*s->l[3][30]-FB330_29*s->l[1][30])+S30*(s->In[9][30]*S30+CM131_29*S31-CM331_29*C31+FB130_29*s->l[2][30]-FB230_29
 *s->l[1][30]+s->dpt[2][53]*(FM131_29*C31+FM331_29*S31));

// = = Block_0_2_0_2_0_2 = = 
 
// Backward Dynamics 

  FA112 = -(s->frc[1][12]-s->m[12]*(AlF111*C12-AlF311*S12-s->l[1][12]*(OM212*OM212+OM312*OM312)+s->l[2][12]*(BS212-
 OpF312)+s->l[3][12]*(BS312+OpF211)));
  FA212 = -(s->frc[2][12]-s->m[12]*(AlF211+s->l[1][12]*(BS212+OpF312)-s->l[2][12]*(OM112*OM112+OM312*OM312)+s->l[3][12]*
 (BS612-OpF112)));
  FA312 = -(s->frc[3][12]-s->m[12]*(AlF111*S12+AlF311*C12+s->l[1][12]*(BS312-OpF211)+s->l[2][12]*(BS612+OpF112)-
 s->l[3][12]*(OM112*OM112+OM212*OM212)));
  CF112 = -(s->trq[1][12]-s->In[1][12]*OpF112-s->In[2][12]*OpF211-s->In[3][12]*OpF312+FA212*s->l[3][12]-FA312*
 s->l[2][12]-OM212*(s->In[3][12]*OM112+s->In[6][12]*OM212+s->In[9][12]*OM312)+OM312*(s->In[2][12]*OM112+s->In[5][12]*OM212+
 s->In[6][12]*OM312));
  CF212 = -(s->trq[2][12]-s->In[2][12]*OpF112-s->In[5][12]*OpF211-s->In[6][12]*OpF312-FA112*s->l[3][12]+FA312*
 s->l[1][12]+OM112*(s->In[3][12]*OM112+s->In[6][12]*OM212+s->In[9][12]*OM312)-OM312*(s->In[1][12]*OM112+s->In[2][12]*OM212+
 s->In[3][12]*OM312));
  CF312 = -(s->trq[3][12]-s->In[3][12]*OpF112-s->In[6][12]*OpF211-s->In[9][12]*OpF312+FA112*s->l[2][12]-FA212*
 s->l[1][12]-OM112*(s->In[2][12]*OM112+s->In[5][12]*OM212+s->In[6][12]*OM312)+OM212*(s->In[1][12]*OM112+s->In[2][12]*OM212+
 s->In[3][12]*OM312));
  FB112_1 = s->m[12]*(AlM110_1*C12-AlM311_1*S12);
  FB212_1 = s->m[12]*AlM211_1;
  FB312_1 = s->m[12]*(AlM110_1*S12+AlM311_1*C12);
  CM112_1 = -(FB212_1*s->l[3][12]-FB312_1*s->l[2][12]);
  CM212_1 = FB112_1*s->l[3][12]-FB312_1*s->l[1][12];
  CM312_1 = -(FB112_1*s->l[2][12]-FB212_1*s->l[1][12]);
  FB112_2 = s->m[12]*(AlM110_2*C12-AlM311_2*S12);
  FB212_2 = s->m[12]*AlM211_2;
  FB312_2 = s->m[12]*(AlM110_2*S12+AlM311_2*C12);
  CM112_2 = -(FB212_2*s->l[3][12]-FB312_2*s->l[2][12]);
  CM212_2 = FB112_2*s->l[3][12]-FB312_2*s->l[1][12];
  CM312_2 = -(FB112_2*s->l[2][12]-FB212_2*s->l[1][12]);
  FB112_3 = s->m[12]*(AlM110_3*C12-AlM311_3*S12);
  FB212_3 = s->m[12]*AlM211_3;
  FB312_3 = s->m[12]*(AlM110_3*S12+AlM311_3*C12);
  CM112_3 = -(FB212_3*s->l[3][12]-FB312_3*s->l[2][12]);
  CM212_3 = FB112_3*s->l[3][12]-FB312_3*s->l[1][12];
  CM312_3 = -(FB112_3*s->l[2][12]-FB212_3*s->l[1][12]);
  FB112_4 = s->m[12]*(AlM111_4*C12-AlM311_4*S12+OpM211_4*s->l[3][12]-OpM312_4*s->l[2][12]);
  FB212_4 = s->m[12]*(AlM211_4-OpM112_4*s->l[3][12]+OpM312_4*s->l[1][12]);
  FB312_4 = s->m[12]*(AlM111_4*S12+AlM311_4*C12+OpM112_4*s->l[2][12]-OpM211_4*s->l[1][12]);
  CM112_4 = s->In[1][12]*OpM112_4+s->In[2][12]*OpM211_4+s->In[3][12]*OpM312_4-FB212_4*s->l[3][12]+FB312_4*s->l[2][12];
  CM212_4 = s->In[2][12]*OpM112_4+s->In[5][12]*OpM211_4+s->In[6][12]*OpM312_4+FB112_4*s->l[3][12]-FB312_4*s->l[1][12];
  CM312_4 = s->In[3][12]*OpM112_4+s->In[6][12]*OpM211_4+s->In[9][12]*OpM312_4-FB112_4*s->l[2][12]+FB212_4*s->l[1][12];
  FB112_5 = s->m[12]*(AlM111_5*C12-AlM311_5*S12+OpM211_5*s->l[3][12]-OpM312_5*s->l[2][12]);
  FB212_5 = s->m[12]*(AlM211_5-OpM112_5*s->l[3][12]+OpM312_5*s->l[1][12]);
  FB312_5 = s->m[12]*(AlM111_5*S12+AlM311_5*C12+OpM112_5*s->l[2][12]-OpM211_5*s->l[1][12]);
  CM112_5 = s->In[1][12]*OpM112_5+s->In[2][12]*OpM211_5+s->In[3][12]*OpM312_5-FB212_5*s->l[3][12]+FB312_5*s->l[2][12];
  CM212_5 = s->In[2][12]*OpM112_5+s->In[5][12]*OpM211_5+s->In[6][12]*OpM312_5+FB112_5*s->l[3][12]-FB312_5*s->l[1][12];
  CM312_5 = s->In[3][12]*OpM112_5+s->In[6][12]*OpM211_5+s->In[9][12]*OpM312_5-FB112_5*s->l[2][12]+FB212_5*s->l[1][12];
  FB112_6 = s->m[12]*(AlM111_6*C12-AlM311_6*S12+OpM211_6*s->l[3][12]-OpM312_6*s->l[2][12]);
  FB212_6 = s->m[12]*(AlM211_6-OpM112_6*s->l[3][12]+OpM312_6*s->l[1][12]);
  FB312_6 = s->m[12]*(AlM111_6*S12+AlM311_6*C12+OpM112_6*s->l[2][12]-OpM211_6*s->l[1][12]);
  CM112_6 = s->In[1][12]*OpM112_6+s->In[2][12]*OpM211_6+s->In[3][12]*OpM312_6-FB212_6*s->l[3][12]+FB312_6*s->l[2][12];
  CM212_6 = s->In[2][12]*OpM112_6+s->In[5][12]*OpM211_6+s->In[6][12]*OpM312_6+FB112_6*s->l[3][12]-FB312_6*s->l[1][12];
  CM312_6 = s->In[3][12]*OpM112_6+s->In[6][12]*OpM211_6+s->In[9][12]*OpM312_6-FB112_6*s->l[2][12]+FB212_6*s->l[1][12];
  FB112_7 = s->m[12]*(AlM111_7*C12-AlM311_7*S12+OpM211_7*s->l[3][12]-OpM312_7*s->l[2][12]);
  FB212_7 = s->m[12]*(AlM211_7-OpM112_7*s->l[3][12]+OpM312_7*s->l[1][12]);
  FB312_7 = s->m[12]*(AlM111_7*S12+AlM311_7*C12+OpM112_7*s->l[2][12]-OpM211_7*s->l[1][12]);
  CM112_7 = s->In[1][12]*OpM112_7+s->In[2][12]*OpM211_7+s->In[3][12]*OpM312_7-FB212_7*s->l[3][12]+FB312_7*s->l[2][12];
  CM212_7 = s->In[2][12]*OpM112_7+s->In[5][12]*OpM211_7+s->In[6][12]*OpM312_7+FB112_7*s->l[3][12]-FB312_7*s->l[1][12];
  CM312_7 = s->In[3][12]*OpM112_7+s->In[6][12]*OpM211_7+s->In[9][12]*OpM312_7-FB112_7*s->l[2][12]+FB212_7*s->l[1][12];
  FB112_8 = s->m[12]*(AlM111_8*C12-AlM311_8*S12+OpM211_8*s->l[3][12]-OpM312_8*s->l[2][12]);
  FB212_8 = s->m[12]*(AlM211_8-OpM112_8*s->l[3][12]+OpM312_8*s->l[1][12]);
  FB312_8 = s->m[12]*(AlM111_8*S12+AlM311_8*C12+OpM112_8*s->l[2][12]-OpM211_8*s->l[1][12]);
  CM112_8 = s->In[1][12]*OpM112_8+s->In[2][12]*OpM211_8+s->In[3][12]*OpM312_8-FB212_8*s->l[3][12]+FB312_8*s->l[2][12];
  CM212_8 = s->In[2][12]*OpM112_8+s->In[5][12]*OpM211_8+s->In[6][12]*OpM312_8+FB112_8*s->l[3][12]-FB312_8*s->l[1][12];
  CM312_8 = s->In[3][12]*OpM112_8+s->In[6][12]*OpM211_8+s->In[9][12]*OpM312_8-FB112_8*s->l[2][12]+FB212_8*s->l[1][12];
  FB112_9 = -s->m[12]*(AlM311_9*S12-OpM211_9*s->l[3][12]+OpM312_9*s->l[2][12]);
  FB212_9 = s->m[12]*(AlM211_9-OpM112_9*s->l[3][12]+OpM312_9*s->l[1][12]);
  FB312_9 = s->m[12]*(AlM311_9*C12+OpM112_9*s->l[2][12]-OpM211_9*s->l[1][12]);
  CM112_9 = s->In[1][12]*OpM112_9+s->In[2][12]*OpM211_9+s->In[3][12]*OpM312_9-FB212_9*s->l[3][12]+FB312_9*s->l[2][12];
  CM212_9 = s->In[2][12]*OpM112_9+s->In[5][12]*OpM211_9+s->In[6][12]*OpM312_9+FB112_9*s->l[3][12]-FB312_9*s->l[1][12];
  CM312_9 = s->In[3][12]*OpM112_9+s->In[6][12]*OpM211_9+s->In[9][12]*OpM312_9-FB112_9*s->l[2][12]+FB212_9*s->l[1][12];
  FB112_10 = s->m[12]*(s->dpt[3][12]*C12-OpM312_10*s->l[2][12]+s->l[3][12]*C11);
  FB212_10 = -s->m[12]*(OpM112_10*s->l[3][12]-OpM312_10*s->l[1][12]);
  FB312_10 = s->m[12]*(OpM112_10*s->l[2][12]+s->dpt[3][12]*S12-s->l[1][12]*C11);
  CM112_10 = s->In[1][12]*OpM112_10+s->In[2][12]*C11+s->In[3][12]*OpM312_10-FB212_10*s->l[3][12]+FB312_10*s->l[2][12];
  CM212_10 = s->In[2][12]*OpM112_10+s->In[5][12]*C11+s->In[6][12]*OpM312_10+FB112_10*s->l[3][12]-FB312_10*s->l[1][12];
  CM312_10 = s->In[3][12]*OpM112_10+s->In[6][12]*C11+s->In[9][12]*OpM312_10-FB112_10*s->l[2][12]+FB212_10*s->l[1][12];
  FB112_11 = -s->m[12]*s->l[2][12]*S12;
  FB212_11 = s->m[12]*(s->l[1][12]*S12-s->l[3][12]*C12);
  FB312_11 = s->m[12]*s->l[2][12]*C12;
  CM212_11 = s->In[2][12]*C12+s->In[6][12]*S12+FB112_11*s->l[3][12]-FB312_11*s->l[1][12];
  CM212_12 = s->In[5][12]+s->m[12]*s->l[1][12]*s->l[1][12]+s->m[12]*s->l[3][12]*s->l[3][12];
  FA111 = -(s->frc[1][11]-s->m[11]*(AlF111-s->l[1][11]*(OM211*OM211+OM311*OM311)+s->l[2][11]*(BS211-OpF311)+s->l[3][11]*
 (BS311+OpF211)));
  FA211 = -(s->frc[2][11]-s->m[11]*(AlF211+s->l[1][11]*(BS211+OpF311)-s->l[2][11]*(OM111*OM111+OM311*OM311)+s->l[3][11]*
 (BS611-OpF110)));
  FA311 = -(s->frc[3][11]-s->m[11]*(AlF311+s->l[1][11]*(BS311-OpF211)+s->l[2][11]*(BS611+OpF110)-s->l[3][11]*(OM111*
 OM111+OM211*OM211)));
  FF111 = FA111+FA112*C12+FA312*S12;
  FF211 = FA211+FA212;
  FF311 = FA311-FA112*S12+FA312*C12;
  CF111 = -(s->trq[1][11]-s->In[1][11]*OpF110-s->In[2][11]*OpF211-s->In[3][11]*OpF311-CF112*C12-CF312*S12+FA211*
 s->l[3][11]-FA311*s->l[2][11]-OM211*(s->In[3][11]*OM111+s->In[6][11]*OM211+s->In[9][11]*OM311)+OM311*(s->In[2][11]*OM111+
 s->In[5][11]*OM211+s->In[6][11]*OM311));
  CF211 = -(s->trq[2][11]-CF212-s->In[2][11]*OpF110-s->In[5][11]*OpF211-s->In[6][11]*OpF311-FA111*s->l[3][11]+FA311*
 s->l[1][11]+OM111*(s->In[3][11]*OM111+s->In[6][11]*OM211+s->In[9][11]*OM311)-OM311*(s->In[1][11]*OM111+s->In[2][11]*OM211+
 s->In[3][11]*OM311));
  CF311 = -(s->trq[3][11]-s->In[3][11]*OpF110-s->In[6][11]*OpF211-s->In[9][11]*OpF311+CF112*S12-CF312*C12+FA111*
 s->l[2][11]-FA211*s->l[1][11]-OM111*(s->In[2][11]*OM111+s->In[5][11]*OM211+s->In[6][11]*OM311)+OM211*(s->In[1][11]*OM111+
 s->In[2][11]*OM211+s->In[3][11]*OM311));
  FB111_1 = s->m[11]*AlM110_1;
  FB211_1 = s->m[11]*AlM211_1;
  FB311_1 = s->m[11]*AlM311_1;
  FM111_1 = FB111_1+FB112_1*C12+FB312_1*S12;
  FM211_1 = FB211_1+FB212_1;
  FM311_1 = FB311_1-FB112_1*S12+FB312_1*C12;
  CM111_1 = CM112_1*C12+CM312_1*S12-FB211_1*s->l[3][11]+FB311_1*s->l[2][11];
  CM211_1 = CM212_1+FB111_1*s->l[3][11]-FB311_1*s->l[1][11];
  CM311_1 = -(CM112_1*S12-CM312_1*C12+FB111_1*s->l[2][11]-FB211_1*s->l[1][11]);
  FB111_2 = s->m[11]*AlM110_2;
  FB211_2 = s->m[11]*AlM211_2;
  FB311_2 = s->m[11]*AlM311_2;
  FM111_2 = FB111_2+FB112_2*C12+FB312_2*S12;
  FM211_2 = FB211_2+FB212_2;
  FM311_2 = FB311_2-FB112_2*S12+FB312_2*C12;
  CM111_2 = CM112_2*C12+CM312_2*S12-FB211_2*s->l[3][11]+FB311_2*s->l[2][11];
  CM211_2 = CM212_2+FB111_2*s->l[3][11]-FB311_2*s->l[1][11];
  CM311_2 = -(CM112_2*S12-CM312_2*C12+FB111_2*s->l[2][11]-FB211_2*s->l[1][11]);
  FB111_3 = s->m[11]*AlM110_3;
  FB211_3 = s->m[11]*AlM211_3;
  FB311_3 = s->m[11]*AlM311_3;
  FM111_3 = FB111_3+FB112_3*C12+FB312_3*S12;
  FM211_3 = FB211_3+FB212_3;
  FM311_3 = FB311_3-FB112_3*S12+FB312_3*C12;
  CM111_3 = CM112_3*C12+CM312_3*S12-FB211_3*s->l[3][11]+FB311_3*s->l[2][11];
  CM211_3 = CM212_3+FB111_3*s->l[3][11]-FB311_3*s->l[1][11];
  CM311_3 = -(CM112_3*S12-CM312_3*C12+FB111_3*s->l[2][11]-FB211_3*s->l[1][11]);
  FB111_4 = s->m[11]*(AlM111_4+OpM211_4*s->l[3][11]-OpM311_4*s->l[2][11]);
  FB211_4 = s->m[11]*(AlM211_4-OpM110_4*s->l[3][11]+OpM311_4*s->l[1][11]);
  FB311_4 = s->m[11]*(AlM311_4+OpM110_4*s->l[2][11]-OpM211_4*s->l[1][11]);
  FM111_4 = FB111_4+FB112_4*C12+FB312_4*S12;
  FM211_4 = FB211_4+FB212_4;
  FM311_4 = FB311_4-FB112_4*S12+FB312_4*C12;
  CM111_4 = s->In[1][11]*OpM110_4+s->In[2][11]*OpM211_4+s->In[3][11]*OpM311_4+CM112_4*C12+CM312_4*S12-FB211_4*
 s->l[3][11]+FB311_4*s->l[2][11];
  CM211_4 = CM212_4+s->In[2][11]*OpM110_4+s->In[5][11]*OpM211_4+s->In[6][11]*OpM311_4+FB111_4*s->l[3][11]-FB311_4*
 s->l[1][11];
  CM311_4 = s->In[3][11]*OpM110_4+s->In[6][11]*OpM211_4+s->In[9][11]*OpM311_4-CM112_4*S12+CM312_4*C12-FB111_4*
 s->l[2][11]+FB211_4*s->l[1][11];
  FB111_5 = s->m[11]*(AlM111_5+OpM211_5*s->l[3][11]-OpM311_5*s->l[2][11]);
  FB211_5 = s->m[11]*(AlM211_5-OpM110_5*s->l[3][11]+OpM311_5*s->l[1][11]);
  FB311_5 = s->m[11]*(AlM311_5+OpM110_5*s->l[2][11]-OpM211_5*s->l[1][11]);
  FM111_5 = FB111_5+FB112_5*C12+FB312_5*S12;
  FM211_5 = FB211_5+FB212_5;
  FM311_5 = FB311_5-FB112_5*S12+FB312_5*C12;
  CM111_5 = s->In[1][11]*OpM110_5+s->In[2][11]*OpM211_5+s->In[3][11]*OpM311_5+CM112_5*C12+CM312_5*S12-FB211_5*
 s->l[3][11]+FB311_5*s->l[2][11];
  CM211_5 = CM212_5+s->In[2][11]*OpM110_5+s->In[5][11]*OpM211_5+s->In[6][11]*OpM311_5+FB111_5*s->l[3][11]-FB311_5*
 s->l[1][11];
  CM311_5 = s->In[3][11]*OpM110_5+s->In[6][11]*OpM211_5+s->In[9][11]*OpM311_5-CM112_5*S12+CM312_5*C12-FB111_5*
 s->l[2][11]+FB211_5*s->l[1][11];
  FB111_6 = s->m[11]*(AlM111_6+OpM211_6*s->l[3][11]-OpM311_6*s->l[2][11]);
  FB211_6 = s->m[11]*(AlM211_6-OpM110_6*s->l[3][11]+OpM311_6*s->l[1][11]);
  FB311_6 = s->m[11]*(AlM311_6+OpM110_6*s->l[2][11]-OpM211_6*s->l[1][11]);
  FM111_6 = FB111_6+FB112_6*C12+FB312_6*S12;
  FM211_6 = FB211_6+FB212_6;
  FM311_6 = FB311_6-FB112_6*S12+FB312_6*C12;
  CM111_6 = s->In[1][11]*OpM110_6+s->In[2][11]*OpM211_6+s->In[3][11]*OpM311_6+CM112_6*C12+CM312_6*S12-FB211_6*
 s->l[3][11]+FB311_6*s->l[2][11];
  CM211_6 = CM212_6+s->In[2][11]*OpM110_6+s->In[5][11]*OpM211_6+s->In[6][11]*OpM311_6+FB111_6*s->l[3][11]-FB311_6*
 s->l[1][11];
  CM311_6 = s->In[3][11]*OpM110_6+s->In[6][11]*OpM211_6+s->In[9][11]*OpM311_6-CM112_6*S12+CM312_6*C12-FB111_6*
 s->l[2][11]+FB211_6*s->l[1][11];
  FB111_7 = s->m[11]*(AlM111_7+OpM211_7*s->l[3][11]-OpM311_7*s->l[2][11]);
  FB211_7 = s->m[11]*(AlM211_7-OpM110_7*s->l[3][11]+OpM311_7*s->l[1][11]);
  FB311_7 = s->m[11]*(AlM311_7+OpM110_7*s->l[2][11]-OpM211_7*s->l[1][11]);
  FM111_7 = FB111_7+FB112_7*C12+FB312_7*S12;
  FM211_7 = FB211_7+FB212_7;
  FM311_7 = FB311_7-FB112_7*S12+FB312_7*C12;
  CM111_7 = s->In[1][11]*OpM110_7+s->In[2][11]*OpM211_7+s->In[3][11]*OpM311_7+CM112_7*C12+CM312_7*S12-FB211_7*
 s->l[3][11]+FB311_7*s->l[2][11];
  CM211_7 = CM212_7+s->In[2][11]*OpM110_7+s->In[5][11]*OpM211_7+s->In[6][11]*OpM311_7+FB111_7*s->l[3][11]-FB311_7*
 s->l[1][11];
  CM311_7 = s->In[3][11]*OpM110_7+s->In[6][11]*OpM211_7+s->In[9][11]*OpM311_7-CM112_7*S12+CM312_7*C12-FB111_7*
 s->l[2][11]+FB211_7*s->l[1][11];
  FB111_8 = s->m[11]*(AlM111_8+OpM211_8*s->l[3][11]-OpM311_8*s->l[2][11]);
  FB211_8 = s->m[11]*(AlM211_8-OpM110_8*s->l[3][11]+OpM311_8*s->l[1][11]);
  FB311_8 = s->m[11]*(AlM311_8+OpM110_8*s->l[2][11]-OpM211_8*s->l[1][11]);
  FM111_8 = FB111_8+FB112_8*C12+FB312_8*S12;
  FM211_8 = FB211_8+FB212_8;
  FM311_8 = FB311_8-FB112_8*S12+FB312_8*C12;
  CM111_8 = s->In[1][11]*OpM110_8+s->In[2][11]*OpM211_8+s->In[3][11]*OpM311_8+CM112_8*C12+CM312_8*S12-FB211_8*
 s->l[3][11]+FB311_8*s->l[2][11];
  CM211_8 = CM212_8+s->In[2][11]*OpM110_8+s->In[5][11]*OpM211_8+s->In[6][11]*OpM311_8+FB111_8*s->l[3][11]-FB311_8*
 s->l[1][11];
  CM311_8 = s->In[3][11]*OpM110_8+s->In[6][11]*OpM211_8+s->In[9][11]*OpM311_8-CM112_8*S12+CM312_8*C12-FB111_8*
 s->l[2][11]+FB211_8*s->l[1][11];
  FB111_9 = s->m[11]*(OpM211_9*s->l[3][11]-OpM311_9*s->l[2][11]);
  FB211_9 = s->m[11]*(AlM211_9+OpM311_9*s->l[1][11]+s->l[3][11]*S10);
  FB311_9 = s->m[11]*(AlM311_9-OpM211_9*s->l[1][11]-s->l[2][11]*S10);
  CM111_9 = CM112_9*C12+CM312_9*S12-s->In[1][11]*S10+s->In[2][11]*OpM211_9+s->In[3][11]*OpM311_9-FB211_9*s->l[3][11]+
 FB311_9*s->l[2][11];
  CM211_9 = CM212_9-s->In[2][11]*S10+s->In[5][11]*OpM211_9+s->In[6][11]*OpM311_9+FB111_9*s->l[3][11]-FB311_9*s->l[1][11];
  CM311_9 = -(s->In[3][11]*S10-s->In[6][11]*OpM211_9-s->In[9][11]*OpM311_9+CM112_9*S12-CM312_9*C12+FB111_9*s->l[2][11]-
 FB211_9*s->l[1][11]);
  FB111_10 = s->m[11]*(s->dpt[3][12]+s->l[2][11]*S11+s->l[3][11]*C11);
  FB211_10 = -s->m[11]*s->l[1][11]*S11;
  FB311_10 = -s->m[11]*s->l[1][11]*C11;
  CM111_10 = s->In[2][11]*C11-s->In[3][11]*S11+CM112_10*C12+CM312_10*S12-FB211_10*s->l[3][11]+FB311_10*s->l[2][11];
  CM111_11 = s->In[1][11]+s->m[11]*s->l[2][11]*s->l[2][11]+s->m[11]*s->l[3][11]*s->l[3][11]+C12*(s->In[1][12]*C12+
 s->In[3][12]*S12-FB212_11*s->l[3][12]+FB312_11*s->l[2][12])+S12*(s->In[3][12]*C12+s->In[9][12]*S12-FB112_11*s->l[2][12]+
 FB212_11*s->l[1][12]);
  FA110 = -(s->frc[1][10]-s->m[10]*(AlF110+BeF310*s->l[3][10]-s->l[1][10]*(OM210*OM210+OM310*OM310)+s->l[2][10]*(BS210-
 OpF310)));
  FA210 = -(s->frc[2][10]-s->m[10]*(AlF210+BeF610*s->l[3][10]+s->l[1][10]*(BS210+OpF310)-s->l[2][10]*(OM110*OM110+OM310*
 OM310)));
  FA310 = -(s->frc[3][10]-s->m[10]*(AlF310+BS910*s->l[3][10]+s->l[1][10]*(BS310-OpF29)+s->l[2][10]*(BS610+OpF110)));
  FF110 = FA110+FF111;
  FF210 = FA210+FF211*C11-FF311*S11;
  FF310 = FA310+FF211*S11+FF311*C11;
  CF110 = -(s->trq[1][10]-CF111-s->In[1][10]*OpF110-s->In[2][10]*OpF29-s->In[3][10]*OpF310+FA210*s->l[3][10]-FA310*
 s->l[2][10]-OM210*(s->In[3][10]*OM110+s->In[6][10]*OM210+s->In[9][10]*OM310)+OM310*(s->In[2][10]*OM110+s->In[5][10]*OM210+
 s->In[6][10]*OM310)+s->dpt[3][12]*(FF211*C11-FF311*S11));
  CF210 = -(s->trq[2][10]-s->In[2][10]*OpF110-s->In[5][10]*OpF29-s->In[6][10]*OpF310-CF211*C11+CF311*S11-FA110*
 s->l[3][10]+FA310*s->l[1][10]-FF111*s->dpt[3][12]+OM110*(s->In[3][10]*OM110+s->In[6][10]*OM210+s->In[9][10]*OM310)-OM310*(
 s->In[1][10]*OM110+s->In[2][10]*OM210+s->In[3][10]*OM310));
  CF310 = -(s->trq[3][10]-s->In[3][10]*OpF110-s->In[6][10]*OpF29-s->In[9][10]*OpF310-CF211*S11-CF311*C11+FA110*
 s->l[2][10]-FA210*s->l[1][10]-OM110*(s->In[2][10]*OM110+s->In[5][10]*OM210+s->In[6][10]*OM310)+OM210*(s->In[1][10]*OM110+
 s->In[2][10]*OM210+s->In[3][10]*OM310));
  FB110_1 = s->m[10]*AlM110_1;
  FB210_1 = s->m[10]*AlM29_1;
  FB310_1 = s->m[10]*AlM310_1;
  FM110_1 = FB110_1+FM111_1;
  FM210_1 = FB210_1+FM211_1*C11-FM311_1*S11;
  FM310_1 = FB310_1+FM211_1*S11+FM311_1*C11;
  CM110_1 = CM111_1-FB210_1*s->l[3][10]+FB310_1*s->l[2][10]-s->dpt[3][12]*(FM211_1*C11-FM311_1*S11);
  CM210_1 = CM211_1*C11-CM311_1*S11+FB110_1*s->l[3][10]-FB310_1*s->l[1][10]+FM111_1*s->dpt[3][12];
  CM310_1 = CM211_1*S11+CM311_1*C11-FB110_1*s->l[2][10]+FB210_1*s->l[1][10];
  FB110_2 = s->m[10]*AlM110_2;
  FB210_2 = s->m[10]*AlM29_2;
  FB310_2 = s->m[10]*AlM310_2;
  FM110_2 = FB110_2+FM111_2;
  FM210_2 = FB210_2+FM211_2*C11-FM311_2*S11;
  FM310_2 = FB310_2+FM211_2*S11+FM311_2*C11;
  CM110_2 = CM111_2-FB210_2*s->l[3][10]+FB310_2*s->l[2][10]-s->dpt[3][12]*(FM211_2*C11-FM311_2*S11);
  CM210_2 = CM211_2*C11-CM311_2*S11+FB110_2*s->l[3][10]-FB310_2*s->l[1][10]+FM111_2*s->dpt[3][12];
  CM310_2 = CM211_2*S11+CM311_2*C11-FB110_2*s->l[2][10]+FB210_2*s->l[1][10];
  FB110_3 = s->m[10]*AlM110_3;
  FB210_3 = s->m[10]*AlM29_3;
  FB310_3 = s->m[10]*AlM310_3;
  FM110_3 = FB110_3+FM111_3;
  FM210_3 = FB210_3+FM211_3*C11-FM311_3*S11;
  FM310_3 = FB310_3+FM211_3*S11+FM311_3*C11;
  CM110_3 = CM111_3-FB210_3*s->l[3][10]+FB310_3*s->l[2][10]-s->dpt[3][12]*(FM211_3*C11-FM311_3*S11);
  CM210_3 = CM211_3*C11-CM311_3*S11+FB110_3*s->l[3][10]-FB310_3*s->l[1][10]+FM111_3*s->dpt[3][12];
  CM310_3 = CM211_3*S11+CM311_3*C11-FB110_3*s->l[2][10]+FB210_3*s->l[1][10];
  FB110_4 = s->m[10]*(AlM110_4+OpM29_4*s->l[3][10]-OpM310_4*s->l[2][10]);
  FB210_4 = s->m[10]*(AlM210_4-OpM110_4*s->l[3][10]+OpM310_4*s->l[1][10]);
  FB310_4 = s->m[10]*(AlM310_4+OpM110_4*s->l[2][10]-OpM29_4*s->l[1][10]);
  FM110_4 = FB110_4+FM111_4;
  FM210_4 = FB210_4+FM211_4*C11-FM311_4*S11;
  FM310_4 = FB310_4+FM211_4*S11+FM311_4*C11;
  CM110_4 = CM111_4+s->In[1][10]*OpM110_4+s->In[2][10]*OpM29_4+s->In[3][10]*OpM310_4-FB210_4*s->l[3][10]+FB310_4*
 s->l[2][10]-s->dpt[3][12]*(FM211_4*C11-FM311_4*S11);
  CM210_4 = s->In[2][10]*OpM110_4+s->In[5][10]*OpM29_4+s->In[6][10]*OpM310_4+CM211_4*C11-CM311_4*S11+FB110_4*s->l[3][10]
 -FB310_4*s->l[1][10]+FM111_4*s->dpt[3][12];
  CM310_4 = s->In[3][10]*OpM110_4+s->In[6][10]*OpM29_4+s->In[9][10]*OpM310_4+CM211_4*S11+CM311_4*C11-FB110_4*s->l[2][10]
 +FB210_4*s->l[1][10];
  FB110_5 = s->m[10]*(AlM110_5+OpM29_5*s->l[3][10]-OpM310_5*s->l[2][10]);
  FB210_5 = s->m[10]*(AlM210_5-OpM110_5*s->l[3][10]+OpM310_5*s->l[1][10]);
  FB310_5 = s->m[10]*(AlM310_5+OpM110_5*s->l[2][10]-OpM29_5*s->l[1][10]);
  FM110_5 = FB110_5+FM111_5;
  FM210_5 = FB210_5+FM211_5*C11-FM311_5*S11;
  FM310_5 = FB310_5+FM211_5*S11+FM311_5*C11;
  CM110_5 = CM111_5+s->In[1][10]*OpM110_5+s->In[2][10]*OpM29_5+s->In[3][10]*OpM310_5-FB210_5*s->l[3][10]+FB310_5*
 s->l[2][10]-s->dpt[3][12]*(FM211_5*C11-FM311_5*S11);
  CM210_5 = s->In[2][10]*OpM110_5+s->In[5][10]*OpM29_5+s->In[6][10]*OpM310_5+CM211_5*C11-CM311_5*S11+FB110_5*s->l[3][10]
 -FB310_5*s->l[1][10]+FM111_5*s->dpt[3][12];
  CM310_5 = s->In[3][10]*OpM110_5+s->In[6][10]*OpM29_5+s->In[9][10]*OpM310_5+CM211_5*S11+CM311_5*C11-FB110_5*s->l[2][10]
 +FB210_5*s->l[1][10];
  FB110_6 = s->m[10]*(AlM110_6+OpM29_6*s->l[3][10]-OpM310_6*s->l[2][10]);
  FB210_6 = s->m[10]*(AlM210_6-OpM110_6*s->l[3][10]+OpM310_6*s->l[1][10]);
  FB310_6 = s->m[10]*(AlM310_6+OpM110_6*s->l[2][10]-OpM29_6*s->l[1][10]);
  FM110_6 = FB110_6+FM111_6;
  FM210_6 = FB210_6+FM211_6*C11-FM311_6*S11;
  FM310_6 = FB310_6+FM211_6*S11+FM311_6*C11;
  CM110_6 = CM111_6+s->In[1][10]*OpM110_6+s->In[2][10]*OpM29_6+s->In[3][10]*OpM310_6-FB210_6*s->l[3][10]+FB310_6*
 s->l[2][10]-s->dpt[3][12]*(FM211_6*C11-FM311_6*S11);
  CM210_6 = s->In[2][10]*OpM110_6+s->In[5][10]*OpM29_6+s->In[6][10]*OpM310_6+CM211_6*C11-CM311_6*S11+FB110_6*s->l[3][10]
 -FB310_6*s->l[1][10]+FM111_6*s->dpt[3][12];
  CM310_6 = s->In[3][10]*OpM110_6+s->In[6][10]*OpM29_6+s->In[9][10]*OpM310_6+CM211_6*S11+CM311_6*C11-FB110_6*s->l[2][10]
 +FB210_6*s->l[1][10];
  FB110_7 = s->m[10]*(AlM110_7+OpM29_7*s->l[3][10]-OpM310_7*s->l[2][10]);
  FB210_7 = s->m[10]*(AlM210_7-OpM110_7*s->l[3][10]+OpM310_7*s->l[1][10]);
  FB310_7 = s->m[10]*(AlM310_7+OpM110_7*s->l[2][10]-OpM29_7*s->l[1][10]);
  FM110_7 = FB110_7+FM111_7;
  FM210_7 = FB210_7+FM211_7*C11-FM311_7*S11;
  FM310_7 = FB310_7+FM211_7*S11+FM311_7*C11;
  CM110_7 = CM111_7+s->In[1][10]*OpM110_7+s->In[2][10]*OpM29_7+s->In[3][10]*OpM310_7-FB210_7*s->l[3][10]+FB310_7*
 s->l[2][10]-s->dpt[3][12]*(FM211_7*C11-FM311_7*S11);
  CM210_7 = s->In[2][10]*OpM110_7+s->In[5][10]*OpM29_7+s->In[6][10]*OpM310_7+CM211_7*C11-CM311_7*S11+FB110_7*s->l[3][10]
 -FB310_7*s->l[1][10]+FM111_7*s->dpt[3][12];
  CM310_7 = s->In[3][10]*OpM110_7+s->In[6][10]*OpM29_7+s->In[9][10]*OpM310_7+CM211_7*S11+CM311_7*C11-FB110_7*s->l[2][10]
 +FB210_7*s->l[1][10];
  FB110_8 = s->m[10]*(AlM110_8-OpM310_8*s->l[2][10]-s->l[3][10]*S9);
  FB210_8 = s->m[10]*(AlM210_8-OpM110_8*s->l[3][10]+OpM310_8*s->l[1][10]);
  FB310_8 = s->m[10]*(AlM310_8+OpM110_8*s->l[2][10]+s->l[1][10]*S9);
  FM110_8 = FB110_8+FM111_8;
  FM210_8 = FB210_8+FM211_8*C11-FM311_8*S11;
  FM310_8 = FB310_8+FM211_8*S11+FM311_8*C11;
  CM110_8 = CM111_8+s->In[1][10]*OpM110_8-s->In[2][10]*S9+s->In[3][10]*OpM310_8-FB210_8*s->l[3][10]+FB310_8*s->l[2][10]-
 s->dpt[3][12]*(FM211_8*C11-FM311_8*S11);
  CM210_8 = s->In[2][10]*OpM110_8-s->In[5][10]*S9+s->In[6][10]*OpM310_8+CM211_8*C11-CM311_8*S11+FB110_8*s->l[3][10]-
 FB310_8*s->l[1][10]+FM111_8*s->dpt[3][12];
  CM310_8 = s->In[3][10]*OpM110_8-s->In[6][10]*S9+s->In[9][10]*OpM310_8+CM211_8*S11+CM311_8*C11-FB110_8*s->l[2][10]+
 FB210_8*s->l[1][10];
  FB110_9 = -s->m[10]*s->l[2][10]*C10;
  FB210_9 = s->m[10]*(s->l[1][10]*C10+s->l[3][10]*S10);
  FB310_9 = -s->m[10]*s->l[2][10]*S10;
  CM210_9 = CM211_9*C11-CM311_9*S11+s->dpt[3][12]*(FB111_9+FB112_9*C12+FB312_9*S12)-s->In[2][10]*S10+s->In[6][10]*C10+
 FB110_9*s->l[3][10]-FB310_9*s->l[1][10];
  CM210_10 = s->In[5][10]+s->m[10]*s->l[1][10]*s->l[1][10]+s->m[10]*s->l[3][10]*s->l[3][10]+s->dpt[3][12]*(FB111_10+
 FB112_10*C12+FB312_10*S12)+C11*(CM212_10+s->In[5][11]*C11-s->In[6][11]*S11+FB111_10*s->l[3][11]-FB311_10*s->l[1][11])-S11*(
 s->In[6][11]*C11-s->In[9][11]*S11-CM112_10*S12+CM312_10*C12-FB111_10*s->l[2][11]+FB211_10*s->l[1][11]);
  FA19 = -(s->frc[1][9]-s->m[9]*(AlF19+BeF39*s->l[3][9]-s->l[1][9]*(OM29*OM29+OM39*OM39)+s->l[2][9]*(BS29-OpF38)));
  FA29 = -(s->frc[2][9]-s->m[9]*(AlF29+BeF69*s->l[3][9]+s->l[1][9]*(BS29+OpF38)-s->l[2][9]*(OM19*OM19+OM39*OM39)));
  FA39 = -(s->frc[3][9]-s->m[9]*(AlF39+BS99*s->l[3][9]+s->l[1][9]*(BS39-OpF29)+s->l[2][9]*(BS69+OpF19)));
  FF19 = FA19+FF110*C10+FF310*S10;
  FF29 = FA29+FF210;
  CF19 = -(s->trq[1][9]-s->In[1][9]*OpF19-s->In[2][9]*OpF29-s->In[3][9]*OpF38-CF110*C10-CF310*S10+FA29*s->l[3][9]-FA39*
 s->l[2][9]+FF210*s->dpt[3][10]-OM29*(s->In[3][9]*OM19+s->In[6][9]*OM29+s->In[9][9]*OM39)+OM39*(s->In[2][9]*OM19+s->In[5][9]*
 OM29+s->In[6][9]*OM39));
  CF29 = -(s->trq[2][9]-CF210-s->In[2][9]*OpF19-s->In[5][9]*OpF29-s->In[6][9]*OpF38-FA19*s->l[3][9]+FA39*s->l[1][9]+OM19
 *(s->In[3][9]*OM19+s->In[6][9]*OM29+s->In[9][9]*OM39)-OM39*(s->In[1][9]*OM19+s->In[2][9]*OM29+s->In[3][9]*OM39)-
 s->dpt[3][10]*(FF110*C10+FF310*S10));
  CF39 = -(s->trq[3][9]-s->In[3][9]*OpF19-s->In[6][9]*OpF29-s->In[9][9]*OpF38+CF110*S10-CF310*C10+FA19*s->l[2][9]-FA29*
 s->l[1][9]-OM19*(s->In[2][9]*OM19+s->In[5][9]*OM29+s->In[6][9]*OM39)+OM29*(s->In[1][9]*OM19+s->In[2][9]*OM29+s->In[3][9]*
 OM39));
  FB19_1 = s->m[9]*AlM19_1;
  FB29_1 = s->m[9]*AlM29_1;
  FB39_1 = s->m[9]*AlM38_1;
  FM19_1 = FB19_1+FM110_1*C10+FM310_1*S10;
  FM29_1 = FB29_1+FM210_1;
  CM19_1 = CM110_1*C10+CM310_1*S10-FB29_1*s->l[3][9]+FB39_1*s->l[2][9]-FM210_1*s->dpt[3][10];
  CM29_1 = CM210_1+FB19_1*s->l[3][9]-FB39_1*s->l[1][9]+s->dpt[3][10]*(FM110_1*C10+FM310_1*S10);
  CM39_1 = -(CM110_1*S10-CM310_1*C10+FB19_1*s->l[2][9]-FB29_1*s->l[1][9]);
  FB19_2 = s->m[9]*AlM19_2;
  FB29_2 = s->m[9]*AlM29_2;
  FB39_2 = s->m[9]*AlM38_2;
  FM19_2 = FB19_2+FM110_2*C10+FM310_2*S10;
  FM29_2 = FB29_2+FM210_2;
  CM19_2 = CM110_2*C10+CM310_2*S10-FB29_2*s->l[3][9]+FB39_2*s->l[2][9]-FM210_2*s->dpt[3][10];
  CM29_2 = CM210_2+FB19_2*s->l[3][9]-FB39_2*s->l[1][9]+s->dpt[3][10]*(FM110_2*C10+FM310_2*S10);
  CM39_2 = -(CM110_2*S10-CM310_2*C10+FB19_2*s->l[2][9]-FB29_2*s->l[1][9]);
  FB19_3 = s->m[9]*AlM19_3;
  FB29_3 = s->m[9]*AlM29_3;
  FB39_3 = s->m[9]*AlM38_3;
  FM19_3 = FB19_3+FM110_3*C10+FM310_3*S10;
  FM29_3 = FB29_3+FM210_3;
  CM19_3 = CM110_3*C10+CM310_3*S10-FB29_3*s->l[3][9]+FB39_3*s->l[2][9]-FM210_3*s->dpt[3][10];
  CM29_3 = CM210_3+FB19_3*s->l[3][9]-FB39_3*s->l[1][9]+s->dpt[3][10]*(FM110_3*C10+FM310_3*S10);
  CM39_3 = -(CM110_3*S10-CM310_3*C10+FB19_3*s->l[2][9]-FB29_3*s->l[1][9]);
  FB19_4 = s->m[9]*(AlM19_4+OpM29_4*s->l[3][9]-OpM38_4*s->l[2][9]);
  FB29_4 = s->m[9]*(AlM29_4-OpM19_4*s->l[3][9]+OpM38_4*s->l[1][9]);
  FB39_4 = s->m[9]*(AlM38_4+OpM19_4*s->l[2][9]-OpM29_4*s->l[1][9]);
  FM19_4 = FB19_4+FM110_4*C10+FM310_4*S10;
  FM29_4 = FB29_4+FM210_4;
  CM19_4 = s->In[1][9]*OpM19_4+s->In[2][9]*OpM29_4+s->In[3][9]*OpM38_4+CM110_4*C10+CM310_4*S10-FB29_4*s->l[3][9]+FB39_4*
 s->l[2][9]-FM210_4*s->dpt[3][10];
  CM29_4 = CM210_4+s->In[2][9]*OpM19_4+s->In[5][9]*OpM29_4+s->In[6][9]*OpM38_4+FB19_4*s->l[3][9]-FB39_4*s->l[1][9]+
 s->dpt[3][10]*(FM110_4*C10+FM310_4*S10);
  CM39_4 = s->In[3][9]*OpM19_4+s->In[6][9]*OpM29_4+s->In[9][9]*OpM38_4-CM110_4*S10+CM310_4*C10-FB19_4*s->l[2][9]+FB29_4*
 s->l[1][9];
  FB19_5 = s->m[9]*(AlM19_5+OpM29_5*s->l[3][9]-OpM38_5*s->l[2][9]);
  FB29_5 = s->m[9]*(AlM29_5-OpM19_5*s->l[3][9]+OpM38_5*s->l[1][9]);
  FB39_5 = s->m[9]*(AlM38_5+OpM19_5*s->l[2][9]-OpM29_5*s->l[1][9]);
  FM19_5 = FB19_5+FM110_5*C10+FM310_5*S10;
  FM29_5 = FB29_5+FM210_5;
  CM19_5 = s->In[1][9]*OpM19_5+s->In[2][9]*OpM29_5+s->In[3][9]*OpM38_5+CM110_5*C10+CM310_5*S10-FB29_5*s->l[3][9]+FB39_5*
 s->l[2][9]-FM210_5*s->dpt[3][10];
  CM29_5 = CM210_5+s->In[2][9]*OpM19_5+s->In[5][9]*OpM29_5+s->In[6][9]*OpM38_5+FB19_5*s->l[3][9]-FB39_5*s->l[1][9]+
 s->dpt[3][10]*(FM110_5*C10+FM310_5*S10);
  CM39_5 = s->In[3][9]*OpM19_5+s->In[6][9]*OpM29_5+s->In[9][9]*OpM38_5-CM110_5*S10+CM310_5*C10-FB19_5*s->l[2][9]+FB29_5*
 s->l[1][9];
  FB19_6 = s->m[9]*(AlM19_6+OpM29_6*s->l[3][9]-OpM38_6*s->l[2][9]);
  FB29_6 = s->m[9]*(AlM29_6-OpM19_6*s->l[3][9]+OpM38_6*s->l[1][9]);
  FB39_6 = s->m[9]*(AlM38_6+OpM19_6*s->l[2][9]-OpM29_6*s->l[1][9]);
  FM19_6 = FB19_6+FM110_6*C10+FM310_6*S10;
  FM29_6 = FB29_6+FM210_6;
  CM19_6 = s->In[1][9]*OpM19_6+s->In[2][9]*OpM29_6+s->In[3][9]*OpM38_6+CM110_6*C10+CM310_6*S10-FB29_6*s->l[3][9]+FB39_6*
 s->l[2][9]-FM210_6*s->dpt[3][10];
  CM29_6 = CM210_6+s->In[2][9]*OpM19_6+s->In[5][9]*OpM29_6+s->In[6][9]*OpM38_6+FB19_6*s->l[3][9]-FB39_6*s->l[1][9]+
 s->dpt[3][10]*(FM110_6*C10+FM310_6*S10);
  CM39_6 = s->In[3][9]*OpM19_6+s->In[6][9]*OpM29_6+s->In[9][9]*OpM38_6-CM110_6*S10+CM310_6*C10-FB19_6*s->l[2][9]+FB29_6*
 s->l[1][9];
  FB19_7 = s->m[9]*(AlM19_7+OpM29_7*s->l[3][9]+s->l[2][9]*S8);
  FB29_7 = s->m[9]*(AlM29_7-OpM19_7*s->l[3][9]-s->l[1][9]*S8);
  FB39_7 = s->m[9]*(OpM19_7*s->l[2][9]-OpM29_7*s->l[1][9]);
  FM19_7 = FB19_7+FM110_7*C10+FM310_7*S10;
  FM29_7 = FB29_7+FM210_7;
  CM19_7 = s->In[1][9]*OpM19_7+s->In[2][9]*OpM29_7-s->In[3][9]*S8+CM110_7*C10+CM310_7*S10-FB29_7*s->l[3][9]+FB39_7*
 s->l[2][9]-FM210_7*s->dpt[3][10];
  CM29_7 = CM210_7+s->In[2][9]*OpM19_7+s->In[5][9]*OpM29_7-s->In[6][9]*S8+FB19_7*s->l[3][9]-FB39_7*s->l[1][9]+
 s->dpt[3][10]*(FM110_7*C10+FM310_7*S10);
  CM39_7 = s->In[3][9]*OpM19_7+s->In[6][9]*OpM29_7-s->In[9][9]*S8-CM110_7*S10+CM310_7*C10-FB19_7*s->l[2][9]+FB29_7*
 s->l[1][9];
  FB19_8 = s->m[9]*(AlM19_8-s->l[3][9]*S9);
  FB29_8 = s->m[9]*(AlM29_8-s->l[3][9]*C9);
  FB39_8 = s->m[9]*(s->l[1][9]*S9+s->l[2][9]*C9);
  CM39_8 = s->In[3][9]*C9-s->In[6][9]*S9-CM110_8*S10+CM310_8*C10-FB19_8*s->l[2][9]+FB29_8*s->l[1][9];
  CM39_9 = s->In[9][9]+s->m[9]*s->l[1][9]*s->l[1][9]+s->m[9]*s->l[2][9]*s->l[2][9]-C10*(s->In[3][10]*S10-s->In[9][10]*
 C10-CM211_9*S11-CM311_9*C11+FB110_9*s->l[2][10]-FB210_9*s->l[1][10])-S10*(CM111_9-s->In[1][10]*S10+s->In[3][10]*C10-FB210_9*
 s->l[3][10]+FB310_9*s->l[2][10]-s->dpt[3][12]*(C11*(FB211_9+FB212_9)-S11*(FB311_9-FB112_9*S12+FB312_9*C12)));
  FA18 = -(s->frc[1][8]-s->m[8]*(AlF18+BeF38*s->l[3][8]-s->l[1][8]*(OM28*OM28+OM38*OM38)+s->l[2][8]*(BS28-OpF38)));
  FA28 = -(s->frc[2][8]-s->m[8]*(AlF28+BeF68*s->l[3][8]+s->l[1][8]*(BS28+OpF38)-s->l[2][8]*(OM18*OM18+OM38*OM38)));
  FA38 = -(s->frc[3][8]-s->m[8]*(AlF38+BS98*s->l[3][8]+s->l[1][8]*(BS38-OpF28)+s->l[2][8]*(BS68+OpF17)));
  FF18 = FA18+FF19*C9-FF29*S9;
  FF28 = FA28+FF19*S9+FF29*C9;
  FF38 = FA38+FA39-FF110*S10+FF310*C10;
  CF18 = -(s->trq[1][8]-s->In[1][8]*OpF17-s->In[2][8]*OpF28-s->In[3][8]*OpF38-CF19*C9+CF29*S9+FA28*s->l[3][8]-FA38*
 s->l[2][8]-OM28*(s->In[3][8]*OM18+s->In[6][8]*OM28+s->In[9][8]*OM38)+OM38*(s->In[2][8]*OM18+s->In[5][8]*OM28+s->In[6][8]*
 OM38)+s->dpt[3][8]*(FF19*S9+FF29*C9));
  CF28 = -(s->trq[2][8]-s->In[2][8]*OpF17-s->In[5][8]*OpF28-s->In[6][8]*OpF38-CF19*S9-CF29*C9-FA18*s->l[3][8]+FA38*
 s->l[1][8]+OM18*(s->In[3][8]*OM18+s->In[6][8]*OM28+s->In[9][8]*OM38)-OM38*(s->In[1][8]*OM18+s->In[2][8]*OM28+s->In[3][8]*
 OM38)-s->dpt[3][8]*(FF19*C9-FF29*S9));
  CF38 = -(s->trq[3][8]-CF39-s->In[3][8]*OpF17-s->In[6][8]*OpF28-s->In[9][8]*OpF38+FA18*s->l[2][8]-FA28*s->l[1][8]-OM18*
 (s->In[2][8]*OM18+s->In[5][8]*OM28+s->In[6][8]*OM38)+OM28*(s->In[1][8]*OM18+s->In[2][8]*OM28+s->In[3][8]*OM38));
  FB18_1 = s->m[8]*AlM17_1;
  FB28_1 = s->m[8]*AlM28_1;
  FB38_1 = s->m[8]*AlM38_1;
  FM18_1 = FB18_1+FM19_1*C9-FM29_1*S9;
  FM28_1 = FB28_1+FM19_1*S9+FM29_1*C9;
  FM38_1 = FB38_1+FB39_1-FM110_1*S10+FM310_1*C10;
  CM18_1 = CM19_1*C9-CM29_1*S9-FB28_1*s->l[3][8]+FB38_1*s->l[2][8]-s->dpt[3][8]*(FM19_1*S9+FM29_1*C9);
  CM28_1 = CM19_1*S9+CM29_1*C9+FB18_1*s->l[3][8]-FB38_1*s->l[1][8]+s->dpt[3][8]*(FM19_1*C9-FM29_1*S9);
  CM38_1 = CM39_1-FB18_1*s->l[2][8]+FB28_1*s->l[1][8];
  FB18_2 = s->m[8]*AlM17_2;
  FB28_2 = s->m[8]*AlM28_2;
  FB38_2 = s->m[8]*AlM38_2;
  FM18_2 = FB18_2+FM19_2*C9-FM29_2*S9;
  FM28_2 = FB28_2+FM19_2*S9+FM29_2*C9;
  FM38_2 = FB38_2+FB39_2-FM110_2*S10+FM310_2*C10;
  CM18_2 = CM19_2*C9-CM29_2*S9-FB28_2*s->l[3][8]+FB38_2*s->l[2][8]-s->dpt[3][8]*(FM19_2*S9+FM29_2*C9);
  CM28_2 = CM19_2*S9+CM29_2*C9+FB18_2*s->l[3][8]-FB38_2*s->l[1][8]+s->dpt[3][8]*(FM19_2*C9-FM29_2*S9);
  CM38_2 = CM39_2-FB18_2*s->l[2][8]+FB28_2*s->l[1][8];
  FB18_3 = s->m[8]*AlM17_3;
  FB28_3 = s->m[8]*AlM28_3;
  FB38_3 = s->m[8]*AlM38_3;
  FM18_3 = FB18_3+FM19_3*C9-FM29_3*S9;
  FM28_3 = FB28_3+FM19_3*S9+FM29_3*C9;
  FM38_3 = FB38_3+FB39_3-FM110_3*S10+FM310_3*C10;
  CM18_3 = CM19_3*C9-CM29_3*S9-FB28_3*s->l[3][8]+FB38_3*s->l[2][8]-s->dpt[3][8]*(FM19_3*S9+FM29_3*C9);
  CM28_3 = CM19_3*S9+CM29_3*C9+FB18_3*s->l[3][8]-FB38_3*s->l[1][8]+s->dpt[3][8]*(FM19_3*C9-FM29_3*S9);
  CM38_3 = CM39_3-FB18_3*s->l[2][8]+FB28_3*s->l[1][8];
  FB18_4 = s->m[8]*(AlM18_4+OpM28_4*s->l[3][8]-OpM38_4*s->l[2][8]);
  FB28_4 = s->m[8]*(AlM28_4-OpM17_4*s->l[3][8]+OpM38_4*s->l[1][8]);
  FB38_4 = s->m[8]*(AlM38_4+OpM17_4*s->l[2][8]-OpM28_4*s->l[1][8]);
  FM18_4 = FB18_4+FM19_4*C9-FM29_4*S9;
  FM28_4 = FB28_4+FM19_4*S9+FM29_4*C9;
  FM38_4 = FB38_4+FB39_4-FM110_4*S10+FM310_4*C10;
  CM18_4 = s->In[1][8]*OpM17_4+s->In[2][8]*OpM28_4+s->In[3][8]*OpM38_4+CM19_4*C9-CM29_4*S9-FB28_4*s->l[3][8]+FB38_4*
 s->l[2][8]-s->dpt[3][8]*(FM19_4*S9+FM29_4*C9);
  CM28_4 = s->In[2][8]*OpM17_4+s->In[5][8]*OpM28_4+s->In[6][8]*OpM38_4+CM19_4*S9+CM29_4*C9+FB18_4*s->l[3][8]-FB38_4*
 s->l[1][8]+s->dpt[3][8]*(FM19_4*C9-FM29_4*S9);
  CM38_4 = CM39_4+s->In[3][8]*OpM17_4+s->In[6][8]*OpM28_4+s->In[9][8]*OpM38_4-FB18_4*s->l[2][8]+FB28_4*s->l[1][8];
  FB18_5 = s->m[8]*(AlM18_5+OpM28_5*s->l[3][8]-OpM38_5*s->l[2][8]);
  FB28_5 = s->m[8]*(AlM28_5-OpM17_5*s->l[3][8]+OpM38_5*s->l[1][8]);
  FB38_5 = s->m[8]*(AlM38_5+OpM17_5*s->l[2][8]-OpM28_5*s->l[1][8]);
  FM18_5 = FB18_5+FM19_5*C9-FM29_5*S9;
  FM28_5 = FB28_5+FM19_5*S9+FM29_5*C9;
  FM38_5 = FB38_5+FB39_5-FM110_5*S10+FM310_5*C10;
  CM18_5 = s->In[1][8]*OpM17_5+s->In[2][8]*OpM28_5+s->In[3][8]*OpM38_5+CM19_5*C9-CM29_5*S9-FB28_5*s->l[3][8]+FB38_5*
 s->l[2][8]-s->dpt[3][8]*(FM19_5*S9+FM29_5*C9);
  CM28_5 = s->In[2][8]*OpM17_5+s->In[5][8]*OpM28_5+s->In[6][8]*OpM38_5+CM19_5*S9+CM29_5*C9+FB18_5*s->l[3][8]-FB38_5*
 s->l[1][8]+s->dpt[3][8]*(FM19_5*C9-FM29_5*S9);
  CM38_5 = CM39_5+s->In[3][8]*OpM17_5+s->In[6][8]*OpM28_5+s->In[9][8]*OpM38_5-FB18_5*s->l[2][8]+FB28_5*s->l[1][8];
  FB18_6 = s->m[8]*(AlM18_6+OpM28_6*s->l[3][8]-OpM38_6*s->l[2][8]);
  FB28_6 = s->m[8]*(AlM28_6+OpM38_6*s->l[1][8]+s->l[3][8]*S7);
  FB38_6 = s->m[8]*(AlM38_6-OpM28_6*s->l[1][8]-s->l[2][8]*S7);
  FM18_6 = FB18_6+FM19_6*C9-FM29_6*S9;
  FM28_6 = FB28_6+FM19_6*S9+FM29_6*C9;
  FM38_6 = FB38_6+FB39_6-FM110_6*S10+FM310_6*C10;
  CM18_6 = CM19_6*C9-CM29_6*S9-s->dpt[3][8]*(FM19_6*S9+FM29_6*C9)-s->In[1][8]*S7+s->In[2][8]*OpM28_6+s->In[3][8]*OpM38_6
 -FB28_6*s->l[3][8]+FB38_6*s->l[2][8];
  CM28_6 = CM19_6*S9+CM29_6*C9+s->dpt[3][8]*(FM19_6*C9-FM29_6*S9)-s->In[2][8]*S7+s->In[5][8]*OpM28_6+s->In[6][8]*OpM38_6
 +FB18_6*s->l[3][8]-FB38_6*s->l[1][8];
  CM38_6 = CM39_6-s->In[3][8]*S7+s->In[6][8]*OpM28_6+s->In[9][8]*OpM38_6-FB18_6*s->l[2][8]+FB28_6*s->l[1][8];
  FB18_7 = s->m[8]*(s->l[2][8]*S8+s->l[3][8]*C8);
  FB28_7 = -s->m[8]*s->l[1][8]*S8;
  FB38_7 = -s->m[8]*s->l[1][8]*C8;
  CM18_7 = s->In[2][8]*C8-s->In[3][8]*S8+CM19_7*C9-CM29_7*S9-FB28_7*s->l[3][8]+FB38_7*s->l[2][8]-s->dpt[3][8]*(FM19_7*S9
 +FM29_7*C9);
  CM18_8 = s->In[1][8]+s->m[8]*s->l[2][8]*s->l[2][8]+s->m[8]*s->l[3][8]*s->l[3][8]-s->dpt[3][8]*(C9*(FB29_8+FM210_8)+S9*
 (FB19_8+FM110_8*C10+FM310_8*S10))+C9*(s->In[1][9]*C9-s->In[2][9]*S9+CM110_8*C10+CM310_8*S10-FB29_8*s->l[3][9]+FB39_8*
 s->l[2][9]-FM210_8*s->dpt[3][10])-S9*(CM210_8+s->In[2][9]*C9-s->In[5][9]*S9+FB19_8*s->l[3][9]-FB39_8*s->l[1][9]+
 s->dpt[3][10]*(FM110_8*C10+FM310_8*S10));
  FA17 = -(s->frc[1][7]-s->m[7]*(AlF17+BeF27*s->l[2][7]-s->l[1][7]*(OM27*OM27+OM37*OM37)+s->l[3][7]*(BS37+OpF26)));
  FA27 = -(s->frc[2][7]-s->m[7]*(AlF27+BS57*s->l[2][7]+s->l[1][7]*(BS27+OpF37)+s->l[3][7]*(BS67-OpF17)));
  FA37 = -(s->frc[3][7]-s->m[7]*(AlF37+BeF87*s->l[2][7]+s->l[1][7]*(BS37-OpF26)-s->l[3][7]*(OM17*OM17+OM27*OM27)));
  FF17 = FA17+FF18;
  FF37 = FA37+FF28*S8+FF38*C8;
  CF17 = -(s->trq[1][7]-CF18-s->In[1][7]*OpF17-s->In[2][7]*OpF26-s->In[3][7]*OpF37+FA27*s->l[3][7]-FA37*s->l[2][7]-OM27*
 (s->In[3][7]*OM17+s->In[6][7]*OM27+s->In[9][7]*OM37)+OM37*(s->In[2][7]*OM17+s->In[5][7]*OM27+s->In[6][7]*OM37)-s->dpt[2][6]*
 (FF28*S8+FF38*C8));
  CF27 = -(s->trq[2][7]-s->In[2][7]*OpF17-s->In[5][7]*OpF26-s->In[6][7]*OpF37-CF28*C8+CF38*S8-FA17*s->l[3][7]+FA37*
 s->l[1][7]+OM17*(s->In[3][7]*OM17+s->In[6][7]*OM27+s->In[9][7]*OM37)-OM37*(s->In[1][7]*OM17+s->In[2][7]*OM27+s->In[3][7]*
 OM37));
  CF37 = -(s->trq[3][7]-s->In[3][7]*OpF17-s->In[6][7]*OpF26-s->In[9][7]*OpF37-CF28*S8-CF38*C8+FA17*s->l[2][7]-FA27*
 s->l[1][7]+FF18*s->dpt[2][6]-OM17*(s->In[2][7]*OM17+s->In[5][7]*OM27+s->In[6][7]*OM37)+OM27*(s->In[1][7]*OM17+s->In[2][7]*
 OM27+s->In[3][7]*OM37));
  FB17_1 = s->m[7]*AlM17_1;
  FB27_1 = s->m[7]*AlM26_1;
  FB37_1 = s->m[7]*AlM37_1;
  FM17_1 = FB17_1+FM18_1;
  FM37_1 = FB37_1+FM28_1*S8+FM38_1*C8;
  CM17_1 = CM18_1-FB27_1*s->l[3][7]+FB37_1*s->l[2][7]+s->dpt[2][6]*(FM28_1*S8+FM38_1*C8);
  CM27_1 = CM28_1*C8-CM38_1*S8+FB17_1*s->l[3][7]-FB37_1*s->l[1][7];
  CM37_1 = CM28_1*S8+CM38_1*C8-FB17_1*s->l[2][7]+FB27_1*s->l[1][7]-FM18_1*s->dpt[2][6];
  FB17_2 = s->m[7]*AlM17_2;
  FB27_2 = s->m[7]*AlM26_2;
  FB37_2 = s->m[7]*AlM37_2;
  FM17_2 = FB17_2+FM18_2;
  FM37_2 = FB37_2+FM28_2*S8+FM38_2*C8;
  CM17_2 = CM18_2-FB27_2*s->l[3][7]+FB37_2*s->l[2][7]+s->dpt[2][6]*(FM28_2*S8+FM38_2*C8);
  CM27_2 = CM28_2*C8-CM38_2*S8+FB17_2*s->l[3][7]-FB37_2*s->l[1][7];
  CM37_2 = CM28_2*S8+CM38_2*C8-FB17_2*s->l[2][7]+FB27_2*s->l[1][7]-FM18_2*s->dpt[2][6];
  FB17_3 = s->m[7]*AlM17_3;
  FB27_3 = s->m[7]*AlM26_3;
  FB37_3 = s->m[7]*AlM37_3;
  FM17_3 = FB17_3+FM18_3;
  FM37_3 = FB37_3+FM28_3*S8+FM38_3*C8;
  CM17_3 = CM18_3-FB27_3*s->l[3][7]+FB37_3*s->l[2][7]+s->dpt[2][6]*(FM28_3*S8+FM38_3*C8);
  CM27_3 = CM28_3*C8-CM38_3*S8+FB17_3*s->l[3][7]-FB37_3*s->l[1][7];
  CM37_3 = CM28_3*S8+CM38_3*C8-FB17_3*s->l[2][7]+FB27_3*s->l[1][7]-FM18_3*s->dpt[2][6];
  FB17_4 = s->m[7]*(AlM17_4+OpM26_4*s->l[3][7]-OpM37_4*s->l[2][7]);
  FB27_4 = -s->m[7]*(OpM17_4*s->l[3][7]-OpM37_4*s->l[1][7]);
  FB37_4 = s->m[7]*(AlM37_4+OpM17_4*s->l[2][7]-OpM26_4*s->l[1][7]);
  FM17_4 = FB17_4+FM18_4;
  FM37_4 = FB37_4+FM28_4*S8+FM38_4*C8;
  CM17_4 = CM18_4+s->In[1][7]*OpM17_4+s->In[2][7]*OpM26_4+s->In[3][7]*OpM37_4-FB27_4*s->l[3][7]+FB37_4*s->l[2][7]+
 s->dpt[2][6]*(FM28_4*S8+FM38_4*C8);
  CM27_4 = s->In[2][7]*OpM17_4+s->In[5][7]*OpM26_4+s->In[6][7]*OpM37_4+CM28_4*C8-CM38_4*S8+FB17_4*s->l[3][7]-FB37_4*
 s->l[1][7];
  CM37_4 = s->In[3][7]*OpM17_4+s->In[6][7]*OpM26_4+s->In[9][7]*OpM37_4+CM28_4*S8+CM38_4*C8-FB17_4*s->l[2][7]+FB27_4*
 s->l[1][7]-FM18_4*s->dpt[2][6];
  FB17_5 = s->m[7]*(AlM17_5-OpM37_5*s->l[2][7]+s->l[3][7]*C6);
  FB27_5 = -s->m[7]*(OpM17_5*s->l[3][7]-OpM37_5*s->l[1][7]);
  FB37_5 = s->m[7]*(AlM37_5+OpM17_5*s->l[2][7]-s->l[1][7]*C6);
  FM17_5 = FB17_5+FM18_5;
  FM37_5 = FB37_5+FM28_5*S8+FM38_5*C8;
  CM17_5 = CM18_5+s->In[1][7]*OpM17_5+s->In[2][7]*C6+s->In[3][7]*OpM37_5-FB27_5*s->l[3][7]+FB37_5*s->l[2][7]+
 s->dpt[2][6]*(FM28_5*S8+FM38_5*C8);
  CM27_5 = s->In[2][7]*OpM17_5+s->In[5][7]*C6+s->In[6][7]*OpM37_5+CM28_5*C8-CM38_5*S8+FB17_5*s->l[3][7]-FB37_5*
 s->l[1][7];
  CM37_5 = s->In[3][7]*OpM17_5+s->In[6][7]*C6+s->In[9][7]*OpM37_5+CM28_5*S8+CM38_5*C8-FB17_5*s->l[2][7]+FB27_5*
 s->l[1][7]-FM18_5*s->dpt[2][6];
  FB17_6 = s->m[7]*(AlM17_6-s->l[2][7]*C7);
  FB27_6 = s->m[7]*(s->l[1][7]*C7+s->l[3][7]*S7);
  FB37_6 = s->m[7]*(AlM37_6-s->l[2][7]*S7);
  CM27_6 = CM28_6*C8-CM38_6*S8-s->In[2][7]*S7+s->In[6][7]*C7+FB17_6*s->l[3][7]-FB37_6*s->l[1][7];
  CM27_7 = s->In[5][7]+s->m[7]*s->l[1][7]*s->l[1][7]+s->m[7]*s->l[3][7]*s->l[3][7]+C8*(s->In[5][8]*C8-s->In[6][8]*S8+
 CM19_7*S9+CM29_7*C9+FB18_7*s->l[3][8]-FB38_7*s->l[1][8]+s->dpt[3][8]*(FM19_7*C9-FM29_7*S9))-S8*(CM39_7+s->In[6][8]*C8-
 s->In[9][8]*S8-FB18_7*s->l[2][8]+FB28_7*s->l[1][8]);

// = = Block_0_2_0_2_0_3 = = 
 
// Backward Dynamics 

  FA118 = -(s->frc[1][18]-s->m[18]*(AlF117*C18-AlF317*S18-s->l[1][18]*(OM218*OM218+OM318*OM318)+s->l[2][18]*(BS218-
 OpF318)+s->l[3][18]*(BS318+OpF217)));
  FA218 = -(s->frc[2][18]-s->m[18]*(AlF217+s->l[1][18]*(BS218+OpF318)-s->l[2][18]*(OM118*OM118+OM318*OM318)+s->l[3][18]*
 (BS618-OpF118)));
  FA318 = -(s->frc[3][18]-s->m[18]*(AlF117*S18+AlF317*C18+s->l[1][18]*(BS318-OpF217)+s->l[2][18]*(BS618+OpF118)-
 s->l[3][18]*(OM118*OM118+OM218*OM218)));
  CF118 = -(s->trq[1][18]-s->In[1][18]*OpF118-s->In[2][18]*OpF217-s->In[3][18]*OpF318+FA218*s->l[3][18]-FA318*
 s->l[2][18]-OM218*(s->In[3][18]*OM118+s->In[6][18]*OM218+s->In[9][18]*OM318)+OM318*(s->In[2][18]*OM118+s->In[5][18]*OM218+
 s->In[6][18]*OM318));
  CF218 = -(s->trq[2][18]-s->In[2][18]*OpF118-s->In[5][18]*OpF217-s->In[6][18]*OpF318-FA118*s->l[3][18]+FA318*
 s->l[1][18]+OM118*(s->In[3][18]*OM118+s->In[6][18]*OM218+s->In[9][18]*OM318)-OM318*(s->In[1][18]*OM118+s->In[2][18]*OM218+
 s->In[3][18]*OM318));
  CF318 = -(s->trq[3][18]-s->In[3][18]*OpF118-s->In[6][18]*OpF217-s->In[9][18]*OpF318+FA118*s->l[2][18]-FA218*
 s->l[1][18]-OM118*(s->In[2][18]*OM118+s->In[5][18]*OM218+s->In[6][18]*OM318)+OM218*(s->In[1][18]*OM118+s->In[2][18]*OM218+
 s->In[3][18]*OM318));
  FB118_1 = s->m[18]*(AlM116_1*C18-AlM317_1*S18);
  FB218_1 = s->m[18]*AlM217_1;
  FB318_1 = s->m[18]*(AlM116_1*S18+AlM317_1*C18);
  CM118_1 = -(FB218_1*s->l[3][18]-FB318_1*s->l[2][18]);
  CM218_1 = FB118_1*s->l[3][18]-FB318_1*s->l[1][18];
  CM318_1 = -(FB118_1*s->l[2][18]-FB218_1*s->l[1][18]);
  FB118_2 = s->m[18]*(AlM116_2*C18-AlM317_2*S18);
  FB218_2 = s->m[18]*AlM217_2;
  FB318_2 = s->m[18]*(AlM116_2*S18+AlM317_2*C18);
  CM118_2 = -(FB218_2*s->l[3][18]-FB318_2*s->l[2][18]);
  CM218_2 = FB118_2*s->l[3][18]-FB318_2*s->l[1][18];
  CM318_2 = -(FB118_2*s->l[2][18]-FB218_2*s->l[1][18]);
  FB118_3 = s->m[18]*(AlM116_3*C18-AlM317_3*S18);
  FB218_3 = s->m[18]*AlM217_3;
  FB318_3 = s->m[18]*(AlM116_3*S18+AlM317_3*C18);
  CM118_3 = -(FB218_3*s->l[3][18]-FB318_3*s->l[2][18]);
  CM218_3 = FB118_3*s->l[3][18]-FB318_3*s->l[1][18];
  CM318_3 = -(FB118_3*s->l[2][18]-FB218_3*s->l[1][18]);
  FB118_4 = s->m[18]*(AlM117_4*C18-AlM317_4*S18+OpM217_4*s->l[3][18]-OpM318_4*s->l[2][18]);
  FB218_4 = s->m[18]*(AlM217_4-OpM118_4*s->l[3][18]+OpM318_4*s->l[1][18]);
  FB318_4 = s->m[18]*(AlM117_4*S18+AlM317_4*C18+OpM118_4*s->l[2][18]-OpM217_4*s->l[1][18]);
  CM118_4 = s->In[1][18]*OpM118_4+s->In[2][18]*OpM217_4+s->In[3][18]*OpM318_4-FB218_4*s->l[3][18]+FB318_4*s->l[2][18];
  CM218_4 = s->In[2][18]*OpM118_4+s->In[5][18]*OpM217_4+s->In[6][18]*OpM318_4+FB118_4*s->l[3][18]-FB318_4*s->l[1][18];
  CM318_4 = s->In[3][18]*OpM118_4+s->In[6][18]*OpM217_4+s->In[9][18]*OpM318_4-FB118_4*s->l[2][18]+FB218_4*s->l[1][18];
  FB118_5 = s->m[18]*(AlM117_5*C18-AlM317_5*S18+OpM217_5*s->l[3][18]-OpM318_5*s->l[2][18]);
  FB218_5 = s->m[18]*(AlM217_5-OpM118_5*s->l[3][18]+OpM318_5*s->l[1][18]);
  FB318_5 = s->m[18]*(AlM117_5*S18+AlM317_5*C18+OpM118_5*s->l[2][18]-OpM217_5*s->l[1][18]);
  CM118_5 = s->In[1][18]*OpM118_5+s->In[2][18]*OpM217_5+s->In[3][18]*OpM318_5-FB218_5*s->l[3][18]+FB318_5*s->l[2][18];
  CM218_5 = s->In[2][18]*OpM118_5+s->In[5][18]*OpM217_5+s->In[6][18]*OpM318_5+FB118_5*s->l[3][18]-FB318_5*s->l[1][18];
  CM318_5 = s->In[3][18]*OpM118_5+s->In[6][18]*OpM217_5+s->In[9][18]*OpM318_5-FB118_5*s->l[2][18]+FB218_5*s->l[1][18];
  FB118_6 = s->m[18]*(AlM117_6*C18-AlM317_6*S18+OpM217_6*s->l[3][18]-OpM318_6*s->l[2][18]);
  FB218_6 = s->m[18]*(AlM217_6-OpM118_6*s->l[3][18]+OpM318_6*s->l[1][18]);
  FB318_6 = s->m[18]*(AlM117_6*S18+AlM317_6*C18+OpM118_6*s->l[2][18]-OpM217_6*s->l[1][18]);
  CM118_6 = s->In[1][18]*OpM118_6+s->In[2][18]*OpM217_6+s->In[3][18]*OpM318_6-FB218_6*s->l[3][18]+FB318_6*s->l[2][18];
  CM218_6 = s->In[2][18]*OpM118_6+s->In[5][18]*OpM217_6+s->In[6][18]*OpM318_6+FB118_6*s->l[3][18]-FB318_6*s->l[1][18];
  CM318_6 = s->In[3][18]*OpM118_6+s->In[6][18]*OpM217_6+s->In[9][18]*OpM318_6-FB118_6*s->l[2][18]+FB218_6*s->l[1][18];
  FB118_13 = s->m[18]*(AlM117_13*C18-AlM317_13*S18+OpM217_13*s->l[3][18]-OpM318_13*s->l[2][18]);
  FB218_13 = s->m[18]*(AlM217_13-OpM118_13*s->l[3][18]+OpM318_13*s->l[1][18]);
  FB318_13 = s->m[18]*(AlM117_13*S18+AlM317_13*C18+OpM118_13*s->l[2][18]-OpM217_13*s->l[1][18]);
  CM118_13 = s->In[1][18]*OpM118_13+s->In[2][18]*OpM217_13+s->In[3][18]*OpM318_13-FB218_13*s->l[3][18]+FB318_13*
 s->l[2][18];
  CM218_13 = s->In[2][18]*OpM118_13+s->In[5][18]*OpM217_13+s->In[6][18]*OpM318_13+FB118_13*s->l[3][18]-FB318_13*
 s->l[1][18];
  CM318_13 = s->In[3][18]*OpM118_13+s->In[6][18]*OpM217_13+s->In[9][18]*OpM318_13-FB118_13*s->l[2][18]+FB218_13*
 s->l[1][18];
  FB118_14 = s->m[18]*(AlM117_14*C18-AlM317_14*S18+OpM217_14*s->l[3][18]-OpM318_14*s->l[2][18]);
  FB218_14 = s->m[18]*(AlM217_14-OpM118_14*s->l[3][18]+OpM318_14*s->l[1][18]);
  FB318_14 = s->m[18]*(AlM117_14*S18+AlM317_14*C18+OpM118_14*s->l[2][18]-OpM217_14*s->l[1][18]);
  CM118_14 = s->In[1][18]*OpM118_14+s->In[2][18]*OpM217_14+s->In[3][18]*OpM318_14-FB218_14*s->l[3][18]+FB318_14*
 s->l[2][18];
  CM218_14 = s->In[2][18]*OpM118_14+s->In[5][18]*OpM217_14+s->In[6][18]*OpM318_14+FB118_14*s->l[3][18]-FB318_14*
 s->l[1][18];
  CM318_14 = s->In[3][18]*OpM118_14+s->In[6][18]*OpM217_14+s->In[9][18]*OpM318_14-FB118_14*s->l[2][18]+FB218_14*
 s->l[1][18];
  FB118_15 = -s->m[18]*(AlM317_15*S18-OpM217_15*s->l[3][18]+OpM318_15*s->l[2][18]);
  FB218_15 = s->m[18]*(AlM217_15-OpM118_15*s->l[3][18]+OpM318_15*s->l[1][18]);
  FB318_15 = s->m[18]*(AlM317_15*C18+OpM118_15*s->l[2][18]-OpM217_15*s->l[1][18]);
  CM118_15 = s->In[1][18]*OpM118_15+s->In[2][18]*OpM217_15+s->In[3][18]*OpM318_15-FB218_15*s->l[3][18]+FB318_15*
 s->l[2][18];
  CM218_15 = s->In[2][18]*OpM118_15+s->In[5][18]*OpM217_15+s->In[6][18]*OpM318_15+FB118_15*s->l[3][18]-FB318_15*
 s->l[1][18];
  CM318_15 = s->In[3][18]*OpM118_15+s->In[6][18]*OpM217_15+s->In[9][18]*OpM318_15-FB118_15*s->l[2][18]+FB218_15*
 s->l[1][18];
  FB118_16 = s->m[18]*(s->dpt[3][24]*C18-OpM318_16*s->l[2][18]+s->l[3][18]*C17);
  FB218_16 = -s->m[18]*(OpM118_16*s->l[3][18]-OpM318_16*s->l[1][18]);
  FB318_16 = s->m[18]*(OpM118_16*s->l[2][18]+s->dpt[3][24]*S18-s->l[1][18]*C17);
  CM118_16 = s->In[1][18]*OpM118_16+s->In[2][18]*C17+s->In[3][18]*OpM318_16-FB218_16*s->l[3][18]+FB318_16*s->l[2][18];
  CM218_16 = s->In[2][18]*OpM118_16+s->In[5][18]*C17+s->In[6][18]*OpM318_16+FB118_16*s->l[3][18]-FB318_16*s->l[1][18];
  CM318_16 = s->In[3][18]*OpM118_16+s->In[6][18]*C17+s->In[9][18]*OpM318_16-FB118_16*s->l[2][18]+FB218_16*s->l[1][18];
  FB118_17 = -s->m[18]*s->l[2][18]*S18;
  FB218_17 = s->m[18]*(s->l[1][18]*S18-s->l[3][18]*C18);
  FB318_17 = s->m[18]*s->l[2][18]*C18;
  CM218_17 = s->In[2][18]*C18+s->In[6][18]*S18+FB118_17*s->l[3][18]-FB318_17*s->l[1][18];
  CM218_18 = s->In[5][18]+s->m[18]*s->l[1][18]*s->l[1][18]+s->m[18]*s->l[3][18]*s->l[3][18];
  FA117 = -(s->frc[1][17]-s->m[17]*(AlF117-s->l[1][17]*(OM217*OM217+OM317*OM317)+s->l[2][17]*(BS217-OpF317)+s->l[3][17]*
 (BS317+OpF217)));
  FA217 = -(s->frc[2][17]-s->m[17]*(AlF217+s->l[1][17]*(BS217+OpF317)-s->l[2][17]*(OM117*OM117+OM317*OM317)+s->l[3][17]*
 (BS617-OpF116)));
  FA317 = -(s->frc[3][17]-s->m[17]*(AlF317+s->l[1][17]*(BS317-OpF217)+s->l[2][17]*(BS617+OpF116)-s->l[3][17]*(OM117*
 OM117+OM217*OM217)));
  FF117 = FA117+FA118*C18+FA318*S18;
  FF217 = FA217+FA218;
  FF317 = FA317-FA118*S18+FA318*C18;
  CF117 = -(s->trq[1][17]-s->In[1][17]*OpF116-s->In[2][17]*OpF217-s->In[3][17]*OpF317-CF118*C18-CF318*S18+FA217*
 s->l[3][17]-FA317*s->l[2][17]-OM217*(s->In[3][17]*OM117+s->In[6][17]*OM217+s->In[9][17]*OM317)+OM317*(s->In[2][17]*OM117+
 s->In[5][17]*OM217+s->In[6][17]*OM317));
  CF217 = -(s->trq[2][17]-CF218-s->In[2][17]*OpF116-s->In[5][17]*OpF217-s->In[6][17]*OpF317-FA117*s->l[3][17]+FA317*
 s->l[1][17]+OM117*(s->In[3][17]*OM117+s->In[6][17]*OM217+s->In[9][17]*OM317)-OM317*(s->In[1][17]*OM117+s->In[2][17]*OM217+
 s->In[3][17]*OM317));
  CF317 = -(s->trq[3][17]-s->In[3][17]*OpF116-s->In[6][17]*OpF217-s->In[9][17]*OpF317+CF118*S18-CF318*C18+FA117*
 s->l[2][17]-FA217*s->l[1][17]-OM117*(s->In[2][17]*OM117+s->In[5][17]*OM217+s->In[6][17]*OM317)+OM217*(s->In[1][17]*OM117+
 s->In[2][17]*OM217+s->In[3][17]*OM317));
  FB117_1 = s->m[17]*AlM116_1;
  FB217_1 = s->m[17]*AlM217_1;
  FB317_1 = s->m[17]*AlM317_1;
  FM117_1 = FB117_1+FB118_1*C18+FB318_1*S18;
  FM217_1 = FB217_1+FB218_1;
  FM317_1 = FB317_1-FB118_1*S18+FB318_1*C18;
  CM117_1 = CM118_1*C18+CM318_1*S18-FB217_1*s->l[3][17]+FB317_1*s->l[2][17];
  CM217_1 = CM218_1+FB117_1*s->l[3][17]-FB317_1*s->l[1][17];
  CM317_1 = -(CM118_1*S18-CM318_1*C18+FB117_1*s->l[2][17]-FB217_1*s->l[1][17]);
  FB117_2 = s->m[17]*AlM116_2;
  FB217_2 = s->m[17]*AlM217_2;
  FB317_2 = s->m[17]*AlM317_2;
  FM117_2 = FB117_2+FB118_2*C18+FB318_2*S18;
  FM217_2 = FB217_2+FB218_2;
  FM317_2 = FB317_2-FB118_2*S18+FB318_2*C18;
  CM117_2 = CM118_2*C18+CM318_2*S18-FB217_2*s->l[3][17]+FB317_2*s->l[2][17];
  CM217_2 = CM218_2+FB117_2*s->l[3][17]-FB317_2*s->l[1][17];
  CM317_2 = -(CM118_2*S18-CM318_2*C18+FB117_2*s->l[2][17]-FB217_2*s->l[1][17]);
  FB117_3 = s->m[17]*AlM116_3;
  FB217_3 = s->m[17]*AlM217_3;
  FB317_3 = s->m[17]*AlM317_3;
  FM117_3 = FB117_3+FB118_3*C18+FB318_3*S18;
  FM217_3 = FB217_3+FB218_3;
  FM317_3 = FB317_3-FB118_3*S18+FB318_3*C18;
  CM117_3 = CM118_3*C18+CM318_3*S18-FB217_3*s->l[3][17]+FB317_3*s->l[2][17];
  CM217_3 = CM218_3+FB117_3*s->l[3][17]-FB317_3*s->l[1][17];
  CM317_3 = -(CM118_3*S18-CM318_3*C18+FB117_3*s->l[2][17]-FB217_3*s->l[1][17]);
  FB117_4 = s->m[17]*(AlM117_4+OpM217_4*s->l[3][17]-OpM317_4*s->l[2][17]);
  FB217_4 = s->m[17]*(AlM217_4-OpM116_4*s->l[3][17]+OpM317_4*s->l[1][17]);
  FB317_4 = s->m[17]*(AlM317_4+OpM116_4*s->l[2][17]-OpM217_4*s->l[1][17]);
  FM117_4 = FB117_4+FB118_4*C18+FB318_4*S18;
  FM217_4 = FB217_4+FB218_4;
  FM317_4 = FB317_4-FB118_4*S18+FB318_4*C18;
  CM117_4 = s->In[1][17]*OpM116_4+s->In[2][17]*OpM217_4+s->In[3][17]*OpM317_4+CM118_4*C18+CM318_4*S18-FB217_4*
 s->l[3][17]+FB317_4*s->l[2][17];
  CM217_4 = CM218_4+s->In[2][17]*OpM116_4+s->In[5][17]*OpM217_4+s->In[6][17]*OpM317_4+FB117_4*s->l[3][17]-FB317_4*
 s->l[1][17];
  CM317_4 = s->In[3][17]*OpM116_4+s->In[6][17]*OpM217_4+s->In[9][17]*OpM317_4-CM118_4*S18+CM318_4*C18-FB117_4*
 s->l[2][17]+FB217_4*s->l[1][17];
  FB117_5 = s->m[17]*(AlM117_5+OpM217_5*s->l[3][17]-OpM317_5*s->l[2][17]);
  FB217_5 = s->m[17]*(AlM217_5-OpM116_5*s->l[3][17]+OpM317_5*s->l[1][17]);
  FB317_5 = s->m[17]*(AlM317_5+OpM116_5*s->l[2][17]-OpM217_5*s->l[1][17]);
  FM117_5 = FB117_5+FB118_5*C18+FB318_5*S18;
  FM217_5 = FB217_5+FB218_5;
  FM317_5 = FB317_5-FB118_5*S18+FB318_5*C18;
  CM117_5 = s->In[1][17]*OpM116_5+s->In[2][17]*OpM217_5+s->In[3][17]*OpM317_5+CM118_5*C18+CM318_5*S18-FB217_5*
 s->l[3][17]+FB317_5*s->l[2][17];
  CM217_5 = CM218_5+s->In[2][17]*OpM116_5+s->In[5][17]*OpM217_5+s->In[6][17]*OpM317_5+FB117_5*s->l[3][17]-FB317_5*
 s->l[1][17];
  CM317_5 = s->In[3][17]*OpM116_5+s->In[6][17]*OpM217_5+s->In[9][17]*OpM317_5-CM118_5*S18+CM318_5*C18-FB117_5*
 s->l[2][17]+FB217_5*s->l[1][17];
  FB117_6 = s->m[17]*(AlM117_6+OpM217_6*s->l[3][17]-OpM317_6*s->l[2][17]);
  FB217_6 = s->m[17]*(AlM217_6-OpM116_6*s->l[3][17]+OpM317_6*s->l[1][17]);
  FB317_6 = s->m[17]*(AlM317_6+OpM116_6*s->l[2][17]-OpM217_6*s->l[1][17]);
  FM117_6 = FB117_6+FB118_6*C18+FB318_6*S18;
  FM217_6 = FB217_6+FB218_6;
  FM317_6 = FB317_6-FB118_6*S18+FB318_6*C18;
  CM117_6 = s->In[1][17]*OpM116_6+s->In[2][17]*OpM217_6+s->In[3][17]*OpM317_6+CM118_6*C18+CM318_6*S18-FB217_6*
 s->l[3][17]+FB317_6*s->l[2][17];
  CM217_6 = CM218_6+s->In[2][17]*OpM116_6+s->In[5][17]*OpM217_6+s->In[6][17]*OpM317_6+FB117_6*s->l[3][17]-FB317_6*
 s->l[1][17];
  CM317_6 = s->In[3][17]*OpM116_6+s->In[6][17]*OpM217_6+s->In[9][17]*OpM317_6-CM118_6*S18+CM318_6*C18-FB117_6*
 s->l[2][17]+FB217_6*s->l[1][17];
  FB117_13 = s->m[17]*(AlM117_13+OpM217_13*s->l[3][17]-OpM317_13*s->l[2][17]);
  FB217_13 = s->m[17]*(AlM217_13-OpM116_13*s->l[3][17]+OpM317_13*s->l[1][17]);
  FB317_13 = s->m[17]*(AlM317_13+OpM116_13*s->l[2][17]-OpM217_13*s->l[1][17]);
  FM117_13 = FB117_13+FB118_13*C18+FB318_13*S18;
  FM217_13 = FB217_13+FB218_13;
  FM317_13 = FB317_13-FB118_13*S18+FB318_13*C18;
  CM117_13 = s->In[1][17]*OpM116_13+s->In[2][17]*OpM217_13+s->In[3][17]*OpM317_13+CM118_13*C18+CM318_13*S18-FB217_13*
 s->l[3][17]+FB317_13*s->l[2][17];
  CM217_13 = CM218_13+s->In[2][17]*OpM116_13+s->In[5][17]*OpM217_13+s->In[6][17]*OpM317_13+FB117_13*s->l[3][17]-FB317_13
 *s->l[1][17];
  CM317_13 = s->In[3][17]*OpM116_13+s->In[6][17]*OpM217_13+s->In[9][17]*OpM317_13-CM118_13*S18+CM318_13*C18-FB117_13*
 s->l[2][17]+FB217_13*s->l[1][17];
  FB117_14 = s->m[17]*(AlM117_14+OpM217_14*s->l[3][17]-OpM317_14*s->l[2][17]);
  FB217_14 = s->m[17]*(AlM217_14-OpM116_14*s->l[3][17]+OpM317_14*s->l[1][17]);
  FB317_14 = s->m[17]*(AlM317_14+OpM116_14*s->l[2][17]-OpM217_14*s->l[1][17]);
  FM117_14 = FB117_14+FB118_14*C18+FB318_14*S18;
  FM217_14 = FB217_14+FB218_14;
  FM317_14 = FB317_14-FB118_14*S18+FB318_14*C18;
  CM117_14 = s->In[1][17]*OpM116_14+s->In[2][17]*OpM217_14+s->In[3][17]*OpM317_14+CM118_14*C18+CM318_14*S18-FB217_14*
 s->l[3][17]+FB317_14*s->l[2][17];
  CM217_14 = CM218_14+s->In[2][17]*OpM116_14+s->In[5][17]*OpM217_14+s->In[6][17]*OpM317_14+FB117_14*s->l[3][17]-FB317_14
 *s->l[1][17];
  CM317_14 = s->In[3][17]*OpM116_14+s->In[6][17]*OpM217_14+s->In[9][17]*OpM317_14-CM118_14*S18+CM318_14*C18-FB117_14*
 s->l[2][17]+FB217_14*s->l[1][17];
  FB117_15 = s->m[17]*(OpM217_15*s->l[3][17]-OpM317_15*s->l[2][17]);
  FB217_15 = s->m[17]*(AlM217_15+OpM317_15*s->l[1][17]+s->l[3][17]*S16);
  FB317_15 = s->m[17]*(AlM317_15-OpM217_15*s->l[1][17]-s->l[2][17]*S16);
  CM117_15 = CM118_15*C18+CM318_15*S18-s->In[1][17]*S16+s->In[2][17]*OpM217_15+s->In[3][17]*OpM317_15-FB217_15*
 s->l[3][17]+FB317_15*s->l[2][17];
  CM217_15 = CM218_15-s->In[2][17]*S16+s->In[5][17]*OpM217_15+s->In[6][17]*OpM317_15+FB117_15*s->l[3][17]-FB317_15*
 s->l[1][17];
  CM317_15 = -(s->In[3][17]*S16-s->In[6][17]*OpM217_15-s->In[9][17]*OpM317_15+CM118_15*S18-CM318_15*C18+FB117_15*
 s->l[2][17]-FB217_15*s->l[1][17]);
  FB117_16 = s->m[17]*(s->dpt[3][24]+s->l[2][17]*S17+s->l[3][17]*C17);
  FB217_16 = -s->m[17]*s->l[1][17]*S17;
  FB317_16 = -s->m[17]*s->l[1][17]*C17;
  CM117_16 = s->In[2][17]*C17-s->In[3][17]*S17+CM118_16*C18+CM318_16*S18-FB217_16*s->l[3][17]+FB317_16*s->l[2][17];
  CM117_17 = s->In[1][17]+s->m[17]*s->l[2][17]*s->l[2][17]+s->m[17]*s->l[3][17]*s->l[3][17]+C18*(s->In[1][18]*C18+
 s->In[3][18]*S18-FB218_17*s->l[3][18]+FB318_17*s->l[2][18])+S18*(s->In[3][18]*C18+s->In[9][18]*S18-FB118_17*s->l[2][18]+
 FB218_17*s->l[1][18]);
  FA116 = -(s->frc[1][16]-s->m[16]*(AlF116+BeF316*s->l[3][16]-s->l[1][16]*(OM216*OM216+OM316*OM316)+s->l[2][16]*(BS216-
 OpF316)));
  FA216 = -(s->frc[2][16]-s->m[16]*(AlF216+BeF616*s->l[3][16]+s->l[1][16]*(BS216+OpF316)-s->l[2][16]*(OM116*OM116+OM316*
 OM316)));
  FA316 = -(s->frc[3][16]-s->m[16]*(AlF316+BS916*s->l[3][16]+s->l[1][16]*(BS316-OpF215)+s->l[2][16]*(BS616+OpF116)));
  FF116 = FA116+FF117;
  FF216 = FA216+FF217*C17-FF317*S17;
  FF316 = FA316+FF217*S17+FF317*C17;
  CF116 = -(s->trq[1][16]-CF117-s->In[1][16]*OpF116-s->In[2][16]*OpF215-s->In[3][16]*OpF316+FA216*s->l[3][16]-FA316*
 s->l[2][16]-OM216*(s->In[3][16]*OM116+s->In[6][16]*OM216+s->In[9][16]*OM316)+OM316*(s->In[2][16]*OM116+s->In[5][16]*OM216+
 s->In[6][16]*OM316)+s->dpt[3][24]*(FF217*C17-FF317*S17));
  CF216 = -(s->trq[2][16]-s->In[2][16]*OpF116-s->In[5][16]*OpF215-s->In[6][16]*OpF316-CF217*C17+CF317*S17-FA116*
 s->l[3][16]+FA316*s->l[1][16]-FF117*s->dpt[3][24]+OM116*(s->In[3][16]*OM116+s->In[6][16]*OM216+s->In[9][16]*OM316)-OM316*(
 s->In[1][16]*OM116+s->In[2][16]*OM216+s->In[3][16]*OM316));
  CF316 = -(s->trq[3][16]-s->In[3][16]*OpF116-s->In[6][16]*OpF215-s->In[9][16]*OpF316-CF217*S17-CF317*C17+FA116*
 s->l[2][16]-FA216*s->l[1][16]-OM116*(s->In[2][16]*OM116+s->In[5][16]*OM216+s->In[6][16]*OM316)+OM216*(s->In[1][16]*OM116+
 s->In[2][16]*OM216+s->In[3][16]*OM316));
  FB116_1 = s->m[16]*AlM116_1;
  FB216_1 = s->m[16]*AlM215_1;
  FB316_1 = s->m[16]*AlM316_1;
  FM116_1 = FB116_1+FM117_1;
  FM216_1 = FB216_1+FM217_1*C17-FM317_1*S17;
  FM316_1 = FB316_1+FM217_1*S17+FM317_1*C17;
  CM116_1 = CM117_1-FB216_1*s->l[3][16]+FB316_1*s->l[2][16]-s->dpt[3][24]*(FM217_1*C17-FM317_1*S17);
  CM216_1 = CM217_1*C17-CM317_1*S17+FB116_1*s->l[3][16]-FB316_1*s->l[1][16]+FM117_1*s->dpt[3][24];
  CM316_1 = CM217_1*S17+CM317_1*C17-FB116_1*s->l[2][16]+FB216_1*s->l[1][16];
  FB116_2 = s->m[16]*AlM116_2;
  FB216_2 = s->m[16]*AlM215_2;
  FB316_2 = s->m[16]*AlM316_2;
  FM116_2 = FB116_2+FM117_2;
  FM216_2 = FB216_2+FM217_2*C17-FM317_2*S17;
  FM316_2 = FB316_2+FM217_2*S17+FM317_2*C17;
  CM116_2 = CM117_2-FB216_2*s->l[3][16]+FB316_2*s->l[2][16]-s->dpt[3][24]*(FM217_2*C17-FM317_2*S17);
  CM216_2 = CM217_2*C17-CM317_2*S17+FB116_2*s->l[3][16]-FB316_2*s->l[1][16]+FM117_2*s->dpt[3][24];
  CM316_2 = CM217_2*S17+CM317_2*C17-FB116_2*s->l[2][16]+FB216_2*s->l[1][16];
  FB116_3 = s->m[16]*AlM116_3;
  FB216_3 = s->m[16]*AlM215_3;
  FB316_3 = s->m[16]*AlM316_3;
  FM116_3 = FB116_3+FM117_3;
  FM216_3 = FB216_3+FM217_3*C17-FM317_3*S17;
  FM316_3 = FB316_3+FM217_3*S17+FM317_3*C17;
  CM116_3 = CM117_3-FB216_3*s->l[3][16]+FB316_3*s->l[2][16]-s->dpt[3][24]*(FM217_3*C17-FM317_3*S17);
  CM216_3 = CM217_3*C17-CM317_3*S17+FB116_3*s->l[3][16]-FB316_3*s->l[1][16]+FM117_3*s->dpt[3][24];
  CM316_3 = CM217_3*S17+CM317_3*C17-FB116_3*s->l[2][16]+FB216_3*s->l[1][16];
  FB116_4 = s->m[16]*(AlM116_4+OpM215_4*s->l[3][16]-OpM316_4*s->l[2][16]);
  FB216_4 = s->m[16]*(AlM216_4-OpM116_4*s->l[3][16]+OpM316_4*s->l[1][16]);
  FB316_4 = s->m[16]*(AlM316_4+OpM116_4*s->l[2][16]-OpM215_4*s->l[1][16]);
  FM116_4 = FB116_4+FM117_4;
  FM216_4 = FB216_4+FM217_4*C17-FM317_4*S17;
  FM316_4 = FB316_4+FM217_4*S17+FM317_4*C17;
  CM116_4 = CM117_4+s->In[1][16]*OpM116_4+s->In[2][16]*OpM215_4+s->In[3][16]*OpM316_4-FB216_4*s->l[3][16]+FB316_4*
 s->l[2][16]-s->dpt[3][24]*(FM217_4*C17-FM317_4*S17);
  CM216_4 = s->In[2][16]*OpM116_4+s->In[5][16]*OpM215_4+s->In[6][16]*OpM316_4+CM217_4*C17-CM317_4*S17+FB116_4*
 s->l[3][16]-FB316_4*s->l[1][16]+FM117_4*s->dpt[3][24];
  CM316_4 = s->In[3][16]*OpM116_4+s->In[6][16]*OpM215_4+s->In[9][16]*OpM316_4+CM217_4*S17+CM317_4*C17-FB116_4*
 s->l[2][16]+FB216_4*s->l[1][16];
  FB116_5 = s->m[16]*(AlM116_5+OpM215_5*s->l[3][16]-OpM316_5*s->l[2][16]);
  FB216_5 = s->m[16]*(AlM216_5-OpM116_5*s->l[3][16]+OpM316_5*s->l[1][16]);
  FB316_5 = s->m[16]*(AlM316_5+OpM116_5*s->l[2][16]-OpM215_5*s->l[1][16]);
  FM116_5 = FB116_5+FM117_5;
  FM216_5 = FB216_5+FM217_5*C17-FM317_5*S17;
  FM316_5 = FB316_5+FM217_5*S17+FM317_5*C17;
  CM116_5 = CM117_5+s->In[1][16]*OpM116_5+s->In[2][16]*OpM215_5+s->In[3][16]*OpM316_5-FB216_5*s->l[3][16]+FB316_5*
 s->l[2][16]-s->dpt[3][24]*(FM217_5*C17-FM317_5*S17);
  CM216_5 = s->In[2][16]*OpM116_5+s->In[5][16]*OpM215_5+s->In[6][16]*OpM316_5+CM217_5*C17-CM317_5*S17+FB116_5*
 s->l[3][16]-FB316_5*s->l[1][16]+FM117_5*s->dpt[3][24];
  CM316_5 = s->In[3][16]*OpM116_5+s->In[6][16]*OpM215_5+s->In[9][16]*OpM316_5+CM217_5*S17+CM317_5*C17-FB116_5*
 s->l[2][16]+FB216_5*s->l[1][16];
  FB116_6 = s->m[16]*(AlM116_6+OpM215_6*s->l[3][16]-OpM316_6*s->l[2][16]);
  FB216_6 = s->m[16]*(AlM216_6-OpM116_6*s->l[3][16]+OpM316_6*s->l[1][16]);
  FB316_6 = s->m[16]*(AlM316_6+OpM116_6*s->l[2][16]-OpM215_6*s->l[1][16]);
  FM116_6 = FB116_6+FM117_6;
  FM216_6 = FB216_6+FM217_6*C17-FM317_6*S17;
  FM316_6 = FB316_6+FM217_6*S17+FM317_6*C17;
  CM116_6 = CM117_6+s->In[1][16]*OpM116_6+s->In[2][16]*OpM215_6+s->In[3][16]*OpM316_6-FB216_6*s->l[3][16]+FB316_6*
 s->l[2][16]-s->dpt[3][24]*(FM217_6*C17-FM317_6*S17);
  CM216_6 = s->In[2][16]*OpM116_6+s->In[5][16]*OpM215_6+s->In[6][16]*OpM316_6+CM217_6*C17-CM317_6*S17+FB116_6*
 s->l[3][16]-FB316_6*s->l[1][16]+FM117_6*s->dpt[3][24];
  CM316_6 = s->In[3][16]*OpM116_6+s->In[6][16]*OpM215_6+s->In[9][16]*OpM316_6+CM217_6*S17+CM317_6*C17-FB116_6*
 s->l[2][16]+FB216_6*s->l[1][16];
  FB116_13 = s->m[16]*(AlM116_13+OpM215_13*s->l[3][16]-OpM316_13*s->l[2][16]);
  FB216_13 = s->m[16]*(AlM216_13-OpM116_13*s->l[3][16]+OpM316_13*s->l[1][16]);
  FB316_13 = s->m[16]*(AlM316_13+OpM116_13*s->l[2][16]-OpM215_13*s->l[1][16]);
  FM116_13 = FB116_13+FM117_13;
  FM216_13 = FB216_13+FM217_13*C17-FM317_13*S17;
  FM316_13 = FB316_13+FM217_13*S17+FM317_13*C17;
  CM116_13 = CM117_13+s->In[1][16]*OpM116_13+s->In[2][16]*OpM215_13+s->In[3][16]*OpM316_13-FB216_13*s->l[3][16]+FB316_13
 *s->l[2][16]-s->dpt[3][24]*(FM217_13*C17-FM317_13*S17);
  CM216_13 = s->In[2][16]*OpM116_13+s->In[5][16]*OpM215_13+s->In[6][16]*OpM316_13+CM217_13*C17-CM317_13*S17+FB116_13*
 s->l[3][16]-FB316_13*s->l[1][16]+FM117_13*s->dpt[3][24];
  CM316_13 = s->In[3][16]*OpM116_13+s->In[6][16]*OpM215_13+s->In[9][16]*OpM316_13+CM217_13*S17+CM317_13*C17-FB116_13*
 s->l[2][16]+FB216_13*s->l[1][16];
  FB116_14 = s->m[16]*(AlM116_14-OpM316_14*s->l[2][16]-s->l[3][16]*S15);
  FB216_14 = s->m[16]*(AlM216_14-OpM116_14*s->l[3][16]+OpM316_14*s->l[1][16]);
  FB316_14 = s->m[16]*(AlM316_14+OpM116_14*s->l[2][16]+s->l[1][16]*S15);
  FM116_14 = FB116_14+FM117_14;
  FM216_14 = FB216_14+FM217_14*C17-FM317_14*S17;
  FM316_14 = FB316_14+FM217_14*S17+FM317_14*C17;
  CM116_14 = CM117_14+s->In[1][16]*OpM116_14-s->In[2][16]*S15+s->In[3][16]*OpM316_14-FB216_14*s->l[3][16]+FB316_14*
 s->l[2][16]-s->dpt[3][24]*(FM217_14*C17-FM317_14*S17);
  CM216_14 = s->In[2][16]*OpM116_14-s->In[5][16]*S15+s->In[6][16]*OpM316_14+CM217_14*C17-CM317_14*S17+FB116_14*
 s->l[3][16]-FB316_14*s->l[1][16]+FM117_14*s->dpt[3][24];
  CM316_14 = s->In[3][16]*OpM116_14-s->In[6][16]*S15+s->In[9][16]*OpM316_14+CM217_14*S17+CM317_14*C17-FB116_14*
 s->l[2][16]+FB216_14*s->l[1][16];
  FB116_15 = -s->m[16]*s->l[2][16]*C16;
  FB216_15 = s->m[16]*(s->l[1][16]*C16+s->l[3][16]*S16);
  FB316_15 = -s->m[16]*s->l[2][16]*S16;
  CM216_15 = CM217_15*C17-CM317_15*S17+s->dpt[3][24]*(FB117_15+FB118_15*C18+FB318_15*S18)-s->In[2][16]*S16+s->In[6][16]*
 C16+FB116_15*s->l[3][16]-FB316_15*s->l[1][16];
  CM216_16 = s->In[5][16]+s->m[16]*s->l[1][16]*s->l[1][16]+s->m[16]*s->l[3][16]*s->l[3][16]+s->dpt[3][24]*(FB117_16+
 FB118_16*C18+FB318_16*S18)+C17*(CM218_16+s->In[5][17]*C17-s->In[6][17]*S17+FB117_16*s->l[3][17]-FB317_16*s->l[1][17])-S17*(
 s->In[6][17]*C17-s->In[9][17]*S17-CM118_16*S18+CM318_16*C18-FB117_16*s->l[2][17]+FB217_16*s->l[1][17]);
  FA115 = -(s->frc[1][15]-s->m[15]*(AlF115+BeF315*s->l[3][15]-s->l[1][15]*(OM215*OM215+OM315*OM315)+s->l[2][15]*(BS215-
 OpF314)));
  FA215 = -(s->frc[2][15]-s->m[15]*(AlF215+BeF615*s->l[3][15]+s->l[1][15]*(BS215+OpF314)-s->l[2][15]*(OM115*OM115+OM315*
 OM315)));
  FA315 = -(s->frc[3][15]-s->m[15]*(AlF315+BS915*s->l[3][15]+s->l[1][15]*(BS315-OpF215)+s->l[2][15]*(BS615+OpF115)));
  FF115 = FA115+FF116*C16+FF316*S16;
  FF215 = FA215+FF216;
  CF115 = -(s->trq[1][15]-s->In[1][15]*OpF115-s->In[2][15]*OpF215-s->In[3][15]*OpF314-CF116*C16-CF316*S16+FA215*
 s->l[3][15]-FA315*s->l[2][15]+FF216*s->dpt[3][22]-OM215*(s->In[3][15]*OM115+s->In[6][15]*OM215+s->In[9][15]*OM315)+OM315*(
 s->In[2][15]*OM115+s->In[5][15]*OM215+s->In[6][15]*OM315));
  CF215 = -(s->trq[2][15]-CF216-s->In[2][15]*OpF115-s->In[5][15]*OpF215-s->In[6][15]*OpF314-FA115*s->l[3][15]+FA315*
 s->l[1][15]+OM115*(s->In[3][15]*OM115+s->In[6][15]*OM215+s->In[9][15]*OM315)-OM315*(s->In[1][15]*OM115+s->In[2][15]*OM215+
 s->In[3][15]*OM315)-s->dpt[3][22]*(FF116*C16+FF316*S16));
  CF315 = -(s->trq[3][15]-s->In[3][15]*OpF115-s->In[6][15]*OpF215-s->In[9][15]*OpF314+CF116*S16-CF316*C16+FA115*
 s->l[2][15]-FA215*s->l[1][15]-OM115*(s->In[2][15]*OM115+s->In[5][15]*OM215+s->In[6][15]*OM315)+OM215*(s->In[1][15]*OM115+
 s->In[2][15]*OM215+s->In[3][15]*OM315));
  FB115_1 = s->m[15]*AlM115_1;
  FB215_1 = s->m[15]*AlM215_1;
  FB315_1 = s->m[15]*AlM314_1;
  FM115_1 = FB115_1+FM116_1*C16+FM316_1*S16;
  FM215_1 = FB215_1+FM216_1;
  CM115_1 = CM116_1*C16+CM316_1*S16-FB215_1*s->l[3][15]+FB315_1*s->l[2][15]-FM216_1*s->dpt[3][22];
  CM215_1 = CM216_1+FB115_1*s->l[3][15]-FB315_1*s->l[1][15]+s->dpt[3][22]*(FM116_1*C16+FM316_1*S16);
  CM315_1 = -(CM116_1*S16-CM316_1*C16+FB115_1*s->l[2][15]-FB215_1*s->l[1][15]);
  FB115_2 = s->m[15]*AlM115_2;
  FB215_2 = s->m[15]*AlM215_2;
  FB315_2 = s->m[15]*AlM314_2;
  FM115_2 = FB115_2+FM116_2*C16+FM316_2*S16;
  FM215_2 = FB215_2+FM216_2;
  CM115_2 = CM116_2*C16+CM316_2*S16-FB215_2*s->l[3][15]+FB315_2*s->l[2][15]-FM216_2*s->dpt[3][22];
  CM215_2 = CM216_2+FB115_2*s->l[3][15]-FB315_2*s->l[1][15]+s->dpt[3][22]*(FM116_2*C16+FM316_2*S16);
  CM315_2 = -(CM116_2*S16-CM316_2*C16+FB115_2*s->l[2][15]-FB215_2*s->l[1][15]);
  FB115_3 = s->m[15]*AlM115_3;
  FB215_3 = s->m[15]*AlM215_3;
  FB315_3 = s->m[15]*AlM314_3;
  FM115_3 = FB115_3+FM116_3*C16+FM316_3*S16;
  FM215_3 = FB215_3+FM216_3;
  CM115_3 = CM116_3*C16+CM316_3*S16-FB215_3*s->l[3][15]+FB315_3*s->l[2][15]-FM216_3*s->dpt[3][22];
  CM215_3 = CM216_3+FB115_3*s->l[3][15]-FB315_3*s->l[1][15]+s->dpt[3][22]*(FM116_3*C16+FM316_3*S16);
  CM315_3 = -(CM116_3*S16-CM316_3*C16+FB115_3*s->l[2][15]-FB215_3*s->l[1][15]);
  FB115_4 = s->m[15]*(AlM115_4+OpM215_4*s->l[3][15]-OpM314_4*s->l[2][15]);
  FB215_4 = s->m[15]*(AlM215_4-OpM115_4*s->l[3][15]+OpM314_4*s->l[1][15]);
  FB315_4 = s->m[15]*(AlM314_4+OpM115_4*s->l[2][15]-OpM215_4*s->l[1][15]);
  FM115_4 = FB115_4+FM116_4*C16+FM316_4*S16;
  FM215_4 = FB215_4+FM216_4;
  CM115_4 = s->In[1][15]*OpM115_4+s->In[2][15]*OpM215_4+s->In[3][15]*OpM314_4+CM116_4*C16+CM316_4*S16-FB215_4*
 s->l[3][15]+FB315_4*s->l[2][15]-FM216_4*s->dpt[3][22];
  CM215_4 = CM216_4+s->In[2][15]*OpM115_4+s->In[5][15]*OpM215_4+s->In[6][15]*OpM314_4+FB115_4*s->l[3][15]-FB315_4*
 s->l[1][15]+s->dpt[3][22]*(FM116_4*C16+FM316_4*S16);
  CM315_4 = s->In[3][15]*OpM115_4+s->In[6][15]*OpM215_4+s->In[9][15]*OpM314_4-CM116_4*S16+CM316_4*C16-FB115_4*
 s->l[2][15]+FB215_4*s->l[1][15];
  FB115_5 = s->m[15]*(AlM115_5+OpM215_5*s->l[3][15]-OpM314_5*s->l[2][15]);
  FB215_5 = s->m[15]*(AlM215_5-OpM115_5*s->l[3][15]+OpM314_5*s->l[1][15]);
  FB315_5 = s->m[15]*(AlM314_5+OpM115_5*s->l[2][15]-OpM215_5*s->l[1][15]);
  FM115_5 = FB115_5+FM116_5*C16+FM316_5*S16;
  FM215_5 = FB215_5+FM216_5;
  CM115_5 = s->In[1][15]*OpM115_5+s->In[2][15]*OpM215_5+s->In[3][15]*OpM314_5+CM116_5*C16+CM316_5*S16-FB215_5*
 s->l[3][15]+FB315_5*s->l[2][15]-FM216_5*s->dpt[3][22];
  CM215_5 = CM216_5+s->In[2][15]*OpM115_5+s->In[5][15]*OpM215_5+s->In[6][15]*OpM314_5+FB115_5*s->l[3][15]-FB315_5*
 s->l[1][15]+s->dpt[3][22]*(FM116_5*C16+FM316_5*S16);
  CM315_5 = s->In[3][15]*OpM115_5+s->In[6][15]*OpM215_5+s->In[9][15]*OpM314_5-CM116_5*S16+CM316_5*C16-FB115_5*
 s->l[2][15]+FB215_5*s->l[1][15];
  FB115_6 = s->m[15]*(AlM115_6+OpM215_6*s->l[3][15]-OpM314_6*s->l[2][15]);
  FB215_6 = s->m[15]*(AlM215_6-OpM115_6*s->l[3][15]+OpM314_6*s->l[1][15]);
  FB315_6 = s->m[15]*(AlM314_6+OpM115_6*s->l[2][15]-OpM215_6*s->l[1][15]);
  FM115_6 = FB115_6+FM116_6*C16+FM316_6*S16;
  FM215_6 = FB215_6+FM216_6;
  CM115_6 = s->In[1][15]*OpM115_6+s->In[2][15]*OpM215_6+s->In[3][15]*OpM314_6+CM116_6*C16+CM316_6*S16-FB215_6*
 s->l[3][15]+FB315_6*s->l[2][15]-FM216_6*s->dpt[3][22];
  CM215_6 = CM216_6+s->In[2][15]*OpM115_6+s->In[5][15]*OpM215_6+s->In[6][15]*OpM314_6+FB115_6*s->l[3][15]-FB315_6*
 s->l[1][15]+s->dpt[3][22]*(FM116_6*C16+FM316_6*S16);
  CM315_6 = s->In[3][15]*OpM115_6+s->In[6][15]*OpM215_6+s->In[9][15]*OpM314_6-CM116_6*S16+CM316_6*C16-FB115_6*
 s->l[2][15]+FB215_6*s->l[1][15];
  FB115_13 = s->m[15]*(AlM115_13+OpM215_13*s->l[3][15]+s->l[2][15]*S14);
  FB215_13 = s->m[15]*(AlM215_13-OpM115_13*s->l[3][15]-s->l[1][15]*S14);
  FB315_13 = s->m[15]*(OpM115_13*s->l[2][15]-OpM215_13*s->l[1][15]);
  FM115_13 = FB115_13+FM116_13*C16+FM316_13*S16;
  FM215_13 = FB215_13+FM216_13;
  CM115_13 = s->In[1][15]*OpM115_13+s->In[2][15]*OpM215_13-s->In[3][15]*S14+CM116_13*C16+CM316_13*S16-FB215_13*
 s->l[3][15]+FB315_13*s->l[2][15]-FM216_13*s->dpt[3][22];
  CM215_13 = CM216_13+s->In[2][15]*OpM115_13+s->In[5][15]*OpM215_13-s->In[6][15]*S14+FB115_13*s->l[3][15]-FB315_13*
 s->l[1][15]+s->dpt[3][22]*(FM116_13*C16+FM316_13*S16);
  CM315_13 = s->In[3][15]*OpM115_13+s->In[6][15]*OpM215_13-s->In[9][15]*S14-CM116_13*S16+CM316_13*C16-FB115_13*
 s->l[2][15]+FB215_13*s->l[1][15];
  FB115_14 = s->m[15]*(AlM115_14-s->l[3][15]*S15);
  FB215_14 = s->m[15]*(AlM215_14-s->l[3][15]*C15);
  FB315_14 = s->m[15]*(s->l[1][15]*S15+s->l[2][15]*C15);
  CM315_14 = s->In[3][15]*C15-s->In[6][15]*S15-CM116_14*S16+CM316_14*C16-FB115_14*s->l[2][15]+FB215_14*s->l[1][15];
  CM315_15 = s->In[9][15]+s->m[15]*s->l[1][15]*s->l[1][15]+s->m[15]*s->l[2][15]*s->l[2][15]-C16*(s->In[3][16]*S16-
 s->In[9][16]*C16-CM217_15*S17-CM317_15*C17+FB116_15*s->l[2][16]-FB216_15*s->l[1][16])-S16*(CM117_15-s->In[1][16]*S16+
 s->In[3][16]*C16-FB216_15*s->l[3][16]+FB316_15*s->l[2][16]-s->dpt[3][24]*(C17*(FB217_15+FB218_15)-S17*(FB317_15-FB118_15*S18
 +FB318_15*C18)));
  FA114 = -(s->frc[1][14]-s->m[14]*(AlF114+BeF314*s->l[3][14]-s->l[1][14]*(OM214*OM214+OM314*OM314)+s->l[2][14]*(BS214-
 OpF314)));
  FA214 = -(s->frc[2][14]-s->m[14]*(AlF214+BeF614*s->l[3][14]+s->l[1][14]*(BS214+OpF314)-s->l[2][14]*(OM114*OM114+OM314*
 OM314)));
  FA314 = -(s->frc[3][14]-s->m[14]*(AlF314+BS914*s->l[3][14]+s->l[1][14]*(BS314-OpF214)+s->l[2][14]*(BS614+OpF113)));
  FF114 = FA114+FF115*C15-FF215*S15;
  FF214 = FA214+FF115*S15+FF215*C15;
  FF314 = FA314+FA315-FF116*S16+FF316*C16;
  CF114 = -(s->trq[1][14]-s->In[1][14]*OpF113-s->In[2][14]*OpF214-s->In[3][14]*OpF314-CF115*C15+CF215*S15+FA214*
 s->l[3][14]-FA314*s->l[2][14]-OM214*(s->In[3][14]*OM114+s->In[6][14]*OM214+s->In[9][14]*OM314)+OM314*(s->In[2][14]*OM114+
 s->In[5][14]*OM214+s->In[6][14]*OM314)+s->dpt[3][20]*(FF115*S15+FF215*C15));
  CF214 = -(s->trq[2][14]-s->In[2][14]*OpF113-s->In[5][14]*OpF214-s->In[6][14]*OpF314-CF115*S15-CF215*C15-FA114*
 s->l[3][14]+FA314*s->l[1][14]+OM114*(s->In[3][14]*OM114+s->In[6][14]*OM214+s->In[9][14]*OM314)-OM314*(s->In[1][14]*OM114+
 s->In[2][14]*OM214+s->In[3][14]*OM314)-s->dpt[3][20]*(FF115*C15-FF215*S15));
  CF314 = -(s->trq[3][14]-CF315-s->In[3][14]*OpF113-s->In[6][14]*OpF214-s->In[9][14]*OpF314+FA114*s->l[2][14]-FA214*
 s->l[1][14]-OM114*(s->In[2][14]*OM114+s->In[5][14]*OM214+s->In[6][14]*OM314)+OM214*(s->In[1][14]*OM114+s->In[2][14]*OM214+
 s->In[3][14]*OM314));
  FB114_1 = s->m[14]*AlM113_1;
  FB214_1 = s->m[14]*AlM214_1;
  FB314_1 = s->m[14]*AlM314_1;
  FM114_1 = FB114_1+FM115_1*C15-FM215_1*S15;
  FM214_1 = FB214_1+FM115_1*S15+FM215_1*C15;
  FM314_1 = FB314_1+FB315_1-FM116_1*S16+FM316_1*C16;
  CM114_1 = CM115_1*C15-CM215_1*S15-FB214_1*s->l[3][14]+FB314_1*s->l[2][14]-s->dpt[3][20]*(FM115_1*S15+FM215_1*C15);
  CM214_1 = CM115_1*S15+CM215_1*C15+FB114_1*s->l[3][14]-FB314_1*s->l[1][14]+s->dpt[3][20]*(FM115_1*C15-FM215_1*S15);
  CM314_1 = CM315_1-FB114_1*s->l[2][14]+FB214_1*s->l[1][14];
  FB114_2 = s->m[14]*AlM113_2;
  FB214_2 = s->m[14]*AlM214_2;
  FB314_2 = s->m[14]*AlM314_2;
  FM114_2 = FB114_2+FM115_2*C15-FM215_2*S15;
  FM214_2 = FB214_2+FM115_2*S15+FM215_2*C15;
  FM314_2 = FB314_2+FB315_2-FM116_2*S16+FM316_2*C16;
  CM114_2 = CM115_2*C15-CM215_2*S15-FB214_2*s->l[3][14]+FB314_2*s->l[2][14]-s->dpt[3][20]*(FM115_2*S15+FM215_2*C15);
  CM214_2 = CM115_2*S15+CM215_2*C15+FB114_2*s->l[3][14]-FB314_2*s->l[1][14]+s->dpt[3][20]*(FM115_2*C15-FM215_2*S15);
  CM314_2 = CM315_2-FB114_2*s->l[2][14]+FB214_2*s->l[1][14];
  FB114_3 = s->m[14]*AlM113_3;
  FB214_3 = s->m[14]*AlM214_3;
  FB314_3 = s->m[14]*AlM314_3;
  FM114_3 = FB114_3+FM115_3*C15-FM215_3*S15;
  FM214_3 = FB214_3+FM115_3*S15+FM215_3*C15;
  FM314_3 = FB314_3+FB315_3-FM116_3*S16+FM316_3*C16;
  CM114_3 = CM115_3*C15-CM215_3*S15-FB214_3*s->l[3][14]+FB314_3*s->l[2][14]-s->dpt[3][20]*(FM115_3*S15+FM215_3*C15);
  CM214_3 = CM115_3*S15+CM215_3*C15+FB114_3*s->l[3][14]-FB314_3*s->l[1][14]+s->dpt[3][20]*(FM115_3*C15-FM215_3*S15);
  CM314_3 = CM315_3-FB114_3*s->l[2][14]+FB214_3*s->l[1][14];
  FB114_4 = s->m[14]*(AlM114_4+OpM214_4*s->l[3][14]-OpM314_4*s->l[2][14]);
  FB214_4 = s->m[14]*(AlM214_4-OpM113_4*s->l[3][14]+OpM314_4*s->l[1][14]);
  FB314_4 = s->m[14]*(AlM314_4+OpM113_4*s->l[2][14]-OpM214_4*s->l[1][14]);
  FM114_4 = FB114_4+FM115_4*C15-FM215_4*S15;
  FM214_4 = FB214_4+FM115_4*S15+FM215_4*C15;
  FM314_4 = FB314_4+FB315_4-FM116_4*S16+FM316_4*C16;
  CM114_4 = s->In[1][14]*OpM113_4+s->In[2][14]*OpM214_4+s->In[3][14]*OpM314_4+CM115_4*C15-CM215_4*S15-FB214_4*
 s->l[3][14]+FB314_4*s->l[2][14]-s->dpt[3][20]*(FM115_4*S15+FM215_4*C15);
  CM214_4 = s->In[2][14]*OpM113_4+s->In[5][14]*OpM214_4+s->In[6][14]*OpM314_4+CM115_4*S15+CM215_4*C15+FB114_4*
 s->l[3][14]-FB314_4*s->l[1][14]+s->dpt[3][20]*(FM115_4*C15-FM215_4*S15);
  CM314_4 = CM315_4+s->In[3][14]*OpM113_4+s->In[6][14]*OpM214_4+s->In[9][14]*OpM314_4-FB114_4*s->l[2][14]+FB214_4*
 s->l[1][14];
  FB114_5 = s->m[14]*(AlM114_5+OpM214_5*s->l[3][14]-OpM314_5*s->l[2][14]);
  FB214_5 = s->m[14]*(AlM214_5-OpM113_5*s->l[3][14]+OpM314_5*s->l[1][14]);
  FB314_5 = s->m[14]*(AlM314_5+OpM113_5*s->l[2][14]-OpM214_5*s->l[1][14]);
  FM114_5 = FB114_5+FM115_5*C15-FM215_5*S15;
  FM214_5 = FB214_5+FM115_5*S15+FM215_5*C15;
  FM314_5 = FB314_5+FB315_5-FM116_5*S16+FM316_5*C16;
  CM114_5 = s->In[1][14]*OpM113_5+s->In[2][14]*OpM214_5+s->In[3][14]*OpM314_5+CM115_5*C15-CM215_5*S15-FB214_5*
 s->l[3][14]+FB314_5*s->l[2][14]-s->dpt[3][20]*(FM115_5*S15+FM215_5*C15);
  CM214_5 = s->In[2][14]*OpM113_5+s->In[5][14]*OpM214_5+s->In[6][14]*OpM314_5+CM115_5*S15+CM215_5*C15+FB114_5*
 s->l[3][14]-FB314_5*s->l[1][14]+s->dpt[3][20]*(FM115_5*C15-FM215_5*S15);
  CM314_5 = CM315_5+s->In[3][14]*OpM113_5+s->In[6][14]*OpM214_5+s->In[9][14]*OpM314_5-FB114_5*s->l[2][14]+FB214_5*
 s->l[1][14];
  FB114_6 = s->m[14]*(AlM114_6+OpM214_6*s->l[3][14]-OpM314_6*s->l[2][14]);
  FB214_6 = s->m[14]*(AlM214_6+OpM314_6*s->l[1][14]+s->l[3][14]*S13);
  FB314_6 = s->m[14]*(AlM314_6-OpM214_6*s->l[1][14]-s->l[2][14]*S13);
  FM114_6 = FB114_6+FM115_6*C15-FM215_6*S15;
  FM214_6 = FB214_6+FM115_6*S15+FM215_6*C15;
  FM314_6 = FB314_6+FB315_6-FM116_6*S16+FM316_6*C16;
  CM114_6 = CM115_6*C15-CM215_6*S15-s->dpt[3][20]*(FM115_6*S15+FM215_6*C15)-s->In[1][14]*S13+s->In[2][14]*OpM214_6+
 s->In[3][14]*OpM314_6-FB214_6*s->l[3][14]+FB314_6*s->l[2][14];
  CM214_6 = CM115_6*S15+CM215_6*C15+s->dpt[3][20]*(FM115_6*C15-FM215_6*S15)-s->In[2][14]*S13+s->In[5][14]*OpM214_6+
 s->In[6][14]*OpM314_6+FB114_6*s->l[3][14]-FB314_6*s->l[1][14];
  CM314_6 = CM315_6-s->In[3][14]*S13+s->In[6][14]*OpM214_6+s->In[9][14]*OpM314_6-FB114_6*s->l[2][14]+FB214_6*s->l[1][14];
  FB114_13 = s->m[14]*(s->l[2][14]*S14+s->l[3][14]*C14);
  FB214_13 = -s->m[14]*s->l[1][14]*S14;
  FB314_13 = -s->m[14]*s->l[1][14]*C14;
  CM114_13 = s->In[2][14]*C14-s->In[3][14]*S14+CM115_13*C15-CM215_13*S15-FB214_13*s->l[3][14]+FB314_13*s->l[2][14]-
 s->dpt[3][20]*(FM115_13*S15+FM215_13*C15);
  CM114_14 = s->In[1][14]+s->m[14]*s->l[2][14]*s->l[2][14]+s->m[14]*s->l[3][14]*s->l[3][14]-s->dpt[3][20]*(C15*(FB215_14
 +FM216_14)+S15*(FB115_14+FM116_14*C16+FM316_14*S16))+C15*(s->In[1][15]*C15-s->In[2][15]*S15+CM116_14*C16+CM316_14*S16-
 FB215_14*s->l[3][15]+FB315_14*s->l[2][15]-FM216_14*s->dpt[3][22])-S15*(CM216_14+s->In[2][15]*C15-s->In[5][15]*S15+FB115_14*
 s->l[3][15]-FB315_14*s->l[1][15]+s->dpt[3][22]*(FM116_14*C16+FM316_14*S16));
  FA113 = -(s->frc[1][13]-s->m[13]*(AlF113+BeF213*s->l[2][13]-s->l[1][13]*(OM213*OM213+OM313*OM313)+s->l[3][13]*(BS313+
 OpF26)));
  FA213 = -(s->frc[2][13]-s->m[13]*(AlF213+BS513*s->l[2][13]+s->l[1][13]*(BS213+OpF313)+s->l[3][13]*(BS613-OpF113)));
  FA313 = -(s->frc[3][13]-s->m[13]*(AlF313+BeF813*s->l[2][13]+s->l[1][13]*(BS313-OpF26)-s->l[3][13]*(OM113*OM113+OM213*
 OM213)));
  FF113 = FA113+FF114;
  FF313 = FA313+FF214*S14+FF314*C14;
  CF113 = -(s->trq[1][13]-CF114-s->In[1][13]*OpF113-s->In[2][13]*OpF26-s->In[3][13]*OpF313+FA213*s->l[3][13]-FA313*
 s->l[2][13]-OM213*(s->In[3][13]*OM113+s->In[6][13]*OM213+s->In[9][13]*OM313)+OM313*(s->In[2][13]*OM113+s->In[5][13]*OM213+
 s->In[6][13]*OM313)-s->dpt[2][18]*(FF214*S14+FF314*C14));
  CF213 = -(s->trq[2][13]-s->In[2][13]*OpF113-s->In[5][13]*OpF26-s->In[6][13]*OpF313-CF214*C14+CF314*S14-FA113*
 s->l[3][13]+FA313*s->l[1][13]+OM113*(s->In[3][13]*OM113+s->In[6][13]*OM213+s->In[9][13]*OM313)-OM313*(s->In[1][13]*OM113+
 s->In[2][13]*OM213+s->In[3][13]*OM313));
  CF313 = -(s->trq[3][13]-s->In[3][13]*OpF113-s->In[6][13]*OpF26-s->In[9][13]*OpF313-CF214*S14-CF314*C14+FA113*
 s->l[2][13]-FA213*s->l[1][13]+FF114*s->dpt[2][18]-OM113*(s->In[2][13]*OM113+s->In[5][13]*OM213+s->In[6][13]*OM313)+OM213*(
 s->In[1][13]*OM113+s->In[2][13]*OM213+s->In[3][13]*OM313));
  FB113_1 = s->m[13]*AlM113_1;
  FB213_1 = s->m[13]*AlM26_1;
  FB313_1 = s->m[13]*AlM313_1;
  FM113_1 = FB113_1+FM114_1;
  FM313_1 = FB313_1+FM214_1*S14+FM314_1*C14;
  CM113_1 = CM114_1-FB213_1*s->l[3][13]+FB313_1*s->l[2][13]+s->dpt[2][18]*(FM214_1*S14+FM314_1*C14);
  CM213_1 = CM214_1*C14-CM314_1*S14+FB113_1*s->l[3][13]-FB313_1*s->l[1][13];
  CM313_1 = CM214_1*S14+CM314_1*C14-FB113_1*s->l[2][13]+FB213_1*s->l[1][13]-FM114_1*s->dpt[2][18];
  FB113_2 = s->m[13]*AlM113_2;
  FB213_2 = s->m[13]*AlM26_2;
  FB313_2 = s->m[13]*AlM313_2;
  FM113_2 = FB113_2+FM114_2;
  FM313_2 = FB313_2+FM214_2*S14+FM314_2*C14;
  CM113_2 = CM114_2-FB213_2*s->l[3][13]+FB313_2*s->l[2][13]+s->dpt[2][18]*(FM214_2*S14+FM314_2*C14);
  CM213_2 = CM214_2*C14-CM314_2*S14+FB113_2*s->l[3][13]-FB313_2*s->l[1][13];
  CM313_2 = CM214_2*S14+CM314_2*C14-FB113_2*s->l[2][13]+FB213_2*s->l[1][13]-FM114_2*s->dpt[2][18];
  FB113_3 = s->m[13]*AlM113_3;
  FB213_3 = s->m[13]*AlM26_3;
  FB313_3 = s->m[13]*AlM313_3;
  FM113_3 = FB113_3+FM114_3;
  FM313_3 = FB313_3+FM214_3*S14+FM314_3*C14;
  CM113_3 = CM114_3-FB213_3*s->l[3][13]+FB313_3*s->l[2][13]+s->dpt[2][18]*(FM214_3*S14+FM314_3*C14);
  CM213_3 = CM214_3*C14-CM314_3*S14+FB113_3*s->l[3][13]-FB313_3*s->l[1][13];
  CM313_3 = CM214_3*S14+CM314_3*C14-FB113_3*s->l[2][13]+FB213_3*s->l[1][13]-FM114_3*s->dpt[2][18];
  FB113_4 = s->m[13]*(AlM113_4+OpM26_4*s->l[3][13]-OpM313_4*s->l[2][13]);
  FB213_4 = -s->m[13]*(OpM113_4*s->l[3][13]-OpM313_4*s->l[1][13]);
  FB313_4 = s->m[13]*(AlM313_4+OpM113_4*s->l[2][13]-OpM26_4*s->l[1][13]);
  FM113_4 = FB113_4+FM114_4;
  FM313_4 = FB313_4+FM214_4*S14+FM314_4*C14;
  CM113_4 = CM114_4+s->In[1][13]*OpM113_4+s->In[2][13]*OpM26_4+s->In[3][13]*OpM313_4-FB213_4*s->l[3][13]+FB313_4*
 s->l[2][13]+s->dpt[2][18]*(FM214_4*S14+FM314_4*C14);
  CM213_4 = s->In[2][13]*OpM113_4+s->In[5][13]*OpM26_4+s->In[6][13]*OpM313_4+CM214_4*C14-CM314_4*S14+FB113_4*s->l[3][13]
 -FB313_4*s->l[1][13];
  CM313_4 = s->In[3][13]*OpM113_4+s->In[6][13]*OpM26_4+s->In[9][13]*OpM313_4+CM214_4*S14+CM314_4*C14-FB113_4*s->l[2][13]
 +FB213_4*s->l[1][13]-FM114_4*s->dpt[2][18];
  FB113_5 = s->m[13]*(AlM113_5-OpM313_5*s->l[2][13]+s->l[3][13]*C6);
  FB213_5 = -s->m[13]*(OpM113_5*s->l[3][13]-OpM313_5*s->l[1][13]);
  FB313_5 = s->m[13]*(AlM313_5+OpM113_5*s->l[2][13]-s->l[1][13]*C6);
  FM113_5 = FB113_5+FM114_5;
  FM313_5 = FB313_5+FM214_5*S14+FM314_5*C14;
  CM113_5 = CM114_5+s->In[1][13]*OpM113_5+s->In[2][13]*C6+s->In[3][13]*OpM313_5-FB213_5*s->l[3][13]+FB313_5*s->l[2][13]+
 s->dpt[2][18]*(FM214_5*S14+FM314_5*C14);
  CM213_5 = s->In[2][13]*OpM113_5+s->In[5][13]*C6+s->In[6][13]*OpM313_5+CM214_5*C14-CM314_5*S14+FB113_5*s->l[3][13]-
 FB313_5*s->l[1][13];
  CM313_5 = s->In[3][13]*OpM113_5+s->In[6][13]*C6+s->In[9][13]*OpM313_5+CM214_5*S14+CM314_5*C14-FB113_5*s->l[2][13]+
 FB213_5*s->l[1][13]-FM114_5*s->dpt[2][18];
  FB113_6 = s->m[13]*(AlM113_6-s->l[2][13]*C13);
  FB213_6 = s->m[13]*(s->l[1][13]*C13+s->l[3][13]*S13);
  FB313_6 = s->m[13]*(AlM313_6-s->l[2][13]*S13);
  CM213_6 = CM214_6*C14-CM314_6*S14-s->In[2][13]*S13+s->In[6][13]*C13+FB113_6*s->l[3][13]-FB313_6*s->l[1][13];
  CM213_13 = s->In[5][13]+s->m[13]*s->l[1][13]*s->l[1][13]+s->m[13]*s->l[3][13]*s->l[3][13]+C14*(s->In[5][14]*C14-
 s->In[6][14]*S14+CM115_13*S15+CM215_13*C15+FB114_13*s->l[3][14]-FB314_13*s->l[1][14]+s->dpt[3][20]*(FM115_13*C15-FM215_13*
 S15))-S14*(CM315_13+s->In[6][14]*C14-s->In[9][14]*S14-FB114_13*s->l[2][14]+FB214_13*s->l[1][14]);

// = = Block_0_2_0_2_0_4 = = 
 
// Backward Dynamics 

  FA121 = -(s->frc[1][21]-s->m[21]*(AlF121+BS121*s->l[1][21]+BeF221*s->l[2][21]+BeF321*s->l[3][21]));
  FA221 = -(s->frc[2][21]-s->m[21]*(AlF221+BS521*s->l[2][21]+BeF421*s->l[1][21]+BeF621*s->l[3][21]));
  FA321 = -(s->frc[3][21]-s->m[21]*(AlF321+BS921*s->l[3][21]+BeF721*s->l[1][21]+BeF821*s->l[2][21]));
  FF121 = FA121+FF122*C22+FF129*C29+FF322*S22+FF329*S29;
  FF221 = FA221+FF222+FF229;
  CF121 = -(s->trq[1][21]-s->In[1][21]*OpF121-s->In[2][21]*OpF221-s->In[3][21]*OpF320-CF122*C22-CF129*C29-CF322*S22-
 CF329*S29+FA221*s->l[3][21]-FA321*s->l[2][21]+FF222*s->dpt[3][36]+FF229*s->dpt[3][37]-OM221*(s->In[3][21]*OM121+s->In[6][21]
 *OM221+s->In[9][21]*OM321)+OM321*(s->In[2][21]*OM121+s->In[5][21]*OM221+s->In[6][21]*OM321)+s->dpt[2][36]*(FF122*S22-FF322*
 C22)+s->dpt[2][37]*(FF129*S29-FF329*C29));
  CF221 = -(s->trq[2][21]-CF222-CF229-s->In[2][21]*OpF121-s->In[5][21]*OpF221-s->In[6][21]*OpF320-FA121*s->l[3][21]+
 FA321*s->l[1][21]+OM121*(s->In[3][21]*OM121+s->In[6][21]*OM221+s->In[9][21]*OM321)-OM321*(s->In[1][21]*OM121+s->In[2][21]*
 OM221+s->In[3][21]*OM321)-s->dpt[1][36]*(FF122*S22-FF322*C22)-s->dpt[1][37]*(FF129*S29-FF329*C29)-s->dpt[3][36]*(FF122*C22+
 FF322*S22)-s->dpt[3][37]*(FF129*C29+FF329*S29));
  CF321 = -(s->trq[3][21]-s->In[3][21]*OpF121-s->In[6][21]*OpF221-s->In[9][21]*OpF320+CF122*S22+CF129*S29-CF322*C22-
 CF329*C29+FA121*s->l[2][21]-FA221*s->l[1][21]-FF222*s->dpt[1][36]-FF229*s->dpt[1][37]-OM121*(s->In[2][21]*OM121+s->In[5][21]
 *OM221+s->In[6][21]*OM321)+OM221*(s->In[1][21]*OM121+s->In[2][21]*OM221+s->In[3][21]*OM321)+s->dpt[2][36]*(FF122*C22+FF322*
 S22)+s->dpt[2][37]*(FF129*C29+FF329*S29));
  FB121_1 = s->m[21]*AlM121_1;
  FB221_1 = s->m[21]*AlM221_1;
  FB321_1 = s->m[21]*AlM320_1;
  FM121_1 = FB121_1+FM122_1*C22+FM129_1*C29+FM322_1*S22+FM329_1*S29;
  FM221_1 = FB221_1+FM222_1+FM229_1;
  CM121_1 = CM122_1*C22+CM129_1*C29+CM322_1*S22+CM329_1*S29-FB221_1*s->l[3][21]+FB321_1*s->l[2][21]-FM222_1*
 s->dpt[3][36]-FM229_1*s->dpt[3][37]-s->dpt[2][36]*(FM122_1*S22-FM322_1*C22)-s->dpt[2][37]*(FM129_1*S29-FM329_1*C29);
  CM221_1 = CM222_1+CM229_1+FB121_1*s->l[3][21]-FB321_1*s->l[1][21]+s->dpt[1][36]*(FM122_1*S22-FM322_1*C22)+
 s->dpt[1][37]*(FM129_1*S29-FM329_1*C29)+s->dpt[3][36]*(FM122_1*C22+FM322_1*S22)+s->dpt[3][37]*(FM129_1*C29+FM329_1*S29);
  CM321_1 = -(CM122_1*S22+CM129_1*S29-CM322_1*C22-CM329_1*C29+FB121_1*s->l[2][21]-FB221_1*s->l[1][21]-FM222_1*
 s->dpt[1][36]-FM229_1*s->dpt[1][37]+s->dpt[2][36]*(FM122_1*C22+FM322_1*S22)+s->dpt[2][37]*(FM129_1*C29+FM329_1*S29));
  FB121_2 = s->m[21]*AlM121_2;
  FB221_2 = s->m[21]*AlM221_2;
  FB321_2 = s->m[21]*AlM320_2;
  FM121_2 = FB121_2+FM122_2*C22+FM129_2*C29+FM322_2*S22+FM329_2*S29;
  FM221_2 = FB221_2+FM222_2+FM229_2;
  CM121_2 = CM122_2*C22+CM129_2*C29+CM322_2*S22+CM329_2*S29-FB221_2*s->l[3][21]+FB321_2*s->l[2][21]-FM222_2*
 s->dpt[3][36]-FM229_2*s->dpt[3][37]-s->dpt[2][36]*(FM122_2*S22-FM322_2*C22)-s->dpt[2][37]*(FM129_2*S29-FM329_2*C29);
  CM221_2 = CM222_2+CM229_2+FB121_2*s->l[3][21]-FB321_2*s->l[1][21]+s->dpt[1][36]*(FM122_2*S22-FM322_2*C22)+
 s->dpt[1][37]*(FM129_2*S29-FM329_2*C29)+s->dpt[3][36]*(FM122_2*C22+FM322_2*S22)+s->dpt[3][37]*(FM129_2*C29+FM329_2*S29);
  CM321_2 = -(CM122_2*S22+CM129_2*S29-CM322_2*C22-CM329_2*C29+FB121_2*s->l[2][21]-FB221_2*s->l[1][21]-FM222_2*
 s->dpt[1][36]-FM229_2*s->dpt[1][37]+s->dpt[2][36]*(FM122_2*C22+FM322_2*S22)+s->dpt[2][37]*(FM129_2*C29+FM329_2*S29));
  FB121_3 = s->m[21]*AlM121_3;
  FB221_3 = s->m[21]*AlM221_3;
  FB321_3 = s->m[21]*AlM320_3;
  FM121_3 = FB121_3+FM122_3*C22+FM129_3*C29+FM322_3*S22+FM329_3*S29;
  FM221_3 = FB221_3+FM222_3+FM229_3;
  CM121_3 = CM122_3*C22+CM129_3*C29+CM322_3*S22+CM329_3*S29-FB221_3*s->l[3][21]+FB321_3*s->l[2][21]-FM222_3*
 s->dpt[3][36]-FM229_3*s->dpt[3][37]-s->dpt[2][36]*(FM122_3*S22-FM322_3*C22)-s->dpt[2][37]*(FM129_3*S29-FM329_3*C29);
  CM221_3 = CM222_3+CM229_3+FB121_3*s->l[3][21]-FB321_3*s->l[1][21]+s->dpt[1][36]*(FM122_3*S22-FM322_3*C22)+
 s->dpt[1][37]*(FM129_3*S29-FM329_3*C29)+s->dpt[3][36]*(FM122_3*C22+FM322_3*S22)+s->dpt[3][37]*(FM129_3*C29+FM329_3*S29);
  CM321_3 = -(CM122_3*S22+CM129_3*S29-CM322_3*C22-CM329_3*C29+FB121_3*s->l[2][21]-FB221_3*s->l[1][21]-FM222_3*
 s->dpt[1][36]-FM229_3*s->dpt[1][37]+s->dpt[2][36]*(FM122_3*C22+FM322_3*S22)+s->dpt[2][37]*(FM129_3*C29+FM329_3*S29));
  FB121_4 = s->m[21]*(AlM121_4+OpM221_4*s->l[3][21]-OpM320_4*s->l[2][21]);
  FB221_4 = s->m[21]*(AlM221_4-OpM121_4*s->l[3][21]+OpM320_4*s->l[1][21]);
  FB321_4 = s->m[21]*(AlM320_4+OpM121_4*s->l[2][21]-OpM221_4*s->l[1][21]);
  FM121_4 = FB121_4+FM122_4*C22+FM129_4*C29+FM322_4*S22+FM329_4*S29;
  FM221_4 = FB221_4+FM222_4+FM229_4;
  CM121_4 = s->In[1][21]*OpM121_4+s->In[2][21]*OpM221_4+s->In[3][21]*OpM320_4+CM122_4*C22+CM129_4*C29+CM322_4*S22+
 CM329_4*S29-FB221_4*s->l[3][21]+FB321_4*s->l[2][21]-FM222_4*s->dpt[3][36]-FM229_4*s->dpt[3][37]-s->dpt[2][36]*(FM122_4*S22-
 FM322_4*C22)-s->dpt[2][37]*(FM129_4*S29-FM329_4*C29);
  CM221_4 = CM222_4+CM229_4+s->In[2][21]*OpM121_4+s->In[5][21]*OpM221_4+s->In[6][21]*OpM320_4+FB121_4*s->l[3][21]-
 FB321_4*s->l[1][21]+s->dpt[1][36]*(FM122_4*S22-FM322_4*C22)+s->dpt[1][37]*(FM129_4*S29-FM329_4*C29)+s->dpt[3][36]*(FM122_4*
 C22+FM322_4*S22)+s->dpt[3][37]*(FM129_4*C29+FM329_4*S29);
  CM321_4 = s->In[3][21]*OpM121_4+s->In[6][21]*OpM221_4+s->In[9][21]*OpM320_4-CM122_4*S22-CM129_4*S29+CM322_4*C22+
 CM329_4*C29-FB121_4*s->l[2][21]+FB221_4*s->l[1][21]+FM222_4*s->dpt[1][36]+FM229_4*s->dpt[1][37]-s->dpt[2][36]*(FM122_4*C22+
 FM322_4*S22)-s->dpt[2][37]*(FM129_4*C29+FM329_4*S29);
  FB121_5 = s->m[21]*(AlM121_5+OpM221_5*s->l[3][21]-OpM320_5*s->l[2][21]);
  FB221_5 = s->m[21]*(AlM221_5-OpM121_5*s->l[3][21]+OpM320_5*s->l[1][21]);
  FB321_5 = s->m[21]*(AlM320_5+OpM121_5*s->l[2][21]-OpM221_5*s->l[1][21]);
  FM121_5 = FB121_5+FM122_5*C22+FM129_5*C29+FM322_5*S22+FM329_5*S29;
  FM221_5 = FB221_5+FM222_5+FM229_5;
  CM121_5 = s->In[1][21]*OpM121_5+s->In[2][21]*OpM221_5+s->In[3][21]*OpM320_5+CM122_5*C22+CM129_5*C29+CM322_5*S22+
 CM329_5*S29-FB221_5*s->l[3][21]+FB321_5*s->l[2][21]-FM222_5*s->dpt[3][36]-FM229_5*s->dpt[3][37]-s->dpt[2][36]*(FM122_5*S22-
 FM322_5*C22)-s->dpt[2][37]*(FM129_5*S29-FM329_5*C29);
  CM221_5 = CM222_5+CM229_5+s->In[2][21]*OpM121_5+s->In[5][21]*OpM221_5+s->In[6][21]*OpM320_5+FB121_5*s->l[3][21]-
 FB321_5*s->l[1][21]+s->dpt[1][36]*(FM122_5*S22-FM322_5*C22)+s->dpt[1][37]*(FM129_5*S29-FM329_5*C29)+s->dpt[3][36]*(FM122_5*
 C22+FM322_5*S22)+s->dpt[3][37]*(FM129_5*C29+FM329_5*S29);
  CM321_5 = s->In[3][21]*OpM121_5+s->In[6][21]*OpM221_5+s->In[9][21]*OpM320_5-CM122_5*S22-CM129_5*S29+CM322_5*C22+
 CM329_5*C29-FB121_5*s->l[2][21]+FB221_5*s->l[1][21]+FM222_5*s->dpt[1][36]+FM229_5*s->dpt[1][37]-s->dpt[2][36]*(FM122_5*C22+
 FM322_5*S22)-s->dpt[2][37]*(FM129_5*C29+FM329_5*S29);
  FB121_6 = s->m[21]*(AlM121_6+OpM221_6*s->l[3][21]-OpM320_6*s->l[2][21]);
  FB221_6 = s->m[21]*(AlM221_6-OpM121_6*s->l[3][21]+OpM320_6*s->l[1][21]);
  FB321_6 = s->m[21]*(AlM320_6+OpM121_6*s->l[2][21]-OpM221_6*s->l[1][21]);
  FM121_6 = FB121_6+FM122_6*C22+FM129_6*C29+FM322_6*S22+FM329_6*S29;
  FM221_6 = FB221_6+FM222_6+FM229_6;
  CM121_6 = s->In[1][21]*OpM121_6+s->In[2][21]*OpM221_6+s->In[3][21]*OpM320_6+CM122_6*C22+CM129_6*C29+CM322_6*S22+
 CM329_6*S29-FB221_6*s->l[3][21]+FB321_6*s->l[2][21]-FM222_6*s->dpt[3][36]-FM229_6*s->dpt[3][37]-s->dpt[2][36]*(FM122_6*S22-
 FM322_6*C22)-s->dpt[2][37]*(FM129_6*S29-FM329_6*C29);
  CM221_6 = CM222_6+CM229_6+s->In[2][21]*OpM121_6+s->In[5][21]*OpM221_6+s->In[6][21]*OpM320_6+FB121_6*s->l[3][21]-
 FB321_6*s->l[1][21]+s->dpt[1][36]*(FM122_6*S22-FM322_6*C22)+s->dpt[1][37]*(FM129_6*S29-FM329_6*C29)+s->dpt[3][36]*(FM122_6*
 C22+FM322_6*S22)+s->dpt[3][37]*(FM129_6*C29+FM329_6*S29);
  CM321_6 = s->In[3][21]*OpM121_6+s->In[6][21]*OpM221_6+s->In[9][21]*OpM320_6-CM122_6*S22-CM129_6*S29+CM322_6*C22+
 CM329_6*C29-FB121_6*s->l[2][21]+FB221_6*s->l[1][21]+FM222_6*s->dpt[1][36]+FM229_6*s->dpt[1][37]-s->dpt[2][36]*(FM122_6*C22+
 FM322_6*S22)-s->dpt[2][37]*(FM129_6*C29+FM329_6*S29);
  FB121_19 = s->m[21]*(AlM121_19+OpM221_19*s->l[3][21]-s->l[2][21]*S20);
  FB221_19 = s->m[21]*(AlM221_19-OpM121_19*s->l[3][21]+s->l[1][21]*S20);
  FB321_19 = s->m[21]*(OpM121_19*s->l[2][21]-OpM221_19*s->l[1][21]);
  FM121_19 = FB121_19+FM122_19*C22+FM129_19*C29+FM322_19*S22+FM329_19*S29;
  FM221_19 = FB221_19+FM222_19+FM229_19;
  CM121_19 = s->In[1][21]*OpM121_19+s->In[2][21]*OpM221_19+s->In[3][21]*S20+CM122_19*C22+CM129_19*C29+CM322_19*S22+
 CM329_19*S29-FB221_19*s->l[3][21]+FB321_19*s->l[2][21]-FM222_19*s->dpt[3][36]-FM229_19*s->dpt[3][37]-s->dpt[2][36]*(FM122_19
 *S22-FM322_19*C22)-s->dpt[2][37]*(FM129_19*S29-FM329_19*C29);
  CM221_19 = CM222_19+CM229_19+s->In[2][21]*OpM121_19+s->In[5][21]*OpM221_19+s->In[6][21]*S20+FB121_19*s->l[3][21]-
 FB321_19*s->l[1][21]+s->dpt[1][36]*(FM122_19*S22-FM322_19*C22)+s->dpt[1][37]*(FM129_19*S29-FM329_19*C29)+s->dpt[3][36]*(
 FM122_19*C22+FM322_19*S22)+s->dpt[3][37]*(FM129_19*C29+FM329_19*S29);
  CM321_19 = s->In[3][21]*OpM121_19+s->In[6][21]*OpM221_19+s->In[9][21]*S20-CM122_19*S22-CM129_19*S29+CM322_19*C22+
 CM329_19*C29-FB121_19*s->l[2][21]+FB221_19*s->l[1][21]+FM222_19*s->dpt[1][36]+FM229_19*s->dpt[1][37]-s->dpt[2][36]*(FM122_19
 *C22+FM322_19*S22)-s->dpt[2][37]*(FM129_19*C29+FM329_19*S29);
  FB121_20 = s->m[21]*C21*(s->dpt[3][32]+s->l[3][21]);
  FB221_20 = s->m[21]*(AlM221_20-s->l[3][21]*S21);
  FB321_20 = -s->m[21]*(s->l[1][21]*C21-s->l[2][21]*S21);
  CM321_20 = s->In[3][21]*S21+s->In[6][21]*C21-CM122_20*S22-CM129_20*S29+CM322_20*C22+CM329_20*C29-FB121_20*s->l[2][21]+
 FB221_20*s->l[1][21]+FM222_20*s->dpt[1][36]+FM229_20*s->dpt[1][37]-s->dpt[2][36]*(FM122_20*C22+FM322_20*S22)-s->dpt[2][37]*(
 FM129_20*C29+FM329_20*S29);
  CM321_21 = s->In[9][21]+s->m[21]*s->l[1][21]*s->l[1][21]+s->m[21]*s->l[2][21]*s->l[2][21]+s->dpt[1][36]*(FB222_21+
 FM223_21*C23-FM323_21*S23)+s->dpt[1][37]*(FB229_21+FM230_21*C30-FM330_21*S30)-s->dpt[2][36]*(C22*(FB122_21+FM123_21)+S22*(
 FB322_21+FM223_21*S23+FM323_21*C23))-s->dpt[2][37]*(C29*(FB129_21+FM130_21)+S29*(FB329_21+FM230_21*S30+FM330_21*C30))+C22*(
 s->In[9][22]*C22+CM223_21*S23+CM323_21*C23-FB122_21*s->l[2][22]+FB222_21*s->l[1][22]-FM123_21*s->dpt[2][40])-S22*(CM123_21-
 s->In[1][22]*S22-FB222_21*s->l[3][22]+FB322_21*s->l[2][22]+s->dpt[2][40]*(FM223_21*S23+FM323_21*C23))+C29*(s->In[9][29]*C29+
 CM230_21*S30+CM330_21*C30-FB129_21*s->l[2][29]+FB229_21*s->l[1][29]-FM130_21*s->dpt[2][51])-S29*(CM130_21-s->In[1][29]*S29-
 FB229_21*s->l[3][29]+FB329_21*s->l[2][29]+s->dpt[2][51]*(FM230_21*S30+FM330_21*C30));
  FA120 = -(s->frc[1][20]-s->m[20]*(AlF120+BeF320*s->l[3][20]-s->l[1][20]*(OM220*OM220+OM320*OM320)+s->l[2][20]*(BS220-
 OpF320)));
  FA220 = -(s->frc[2][20]-s->m[20]*(AlF219+BeF620*s->l[3][20]+s->l[1][20]*(BS220+OpF320)-s->l[2][20]*(OM120*OM120+OM320*
 OM320)));
  FA320 = -(s->frc[3][20]-s->m[20]*(AlF320+BS920*s->l[3][20]+s->l[1][20]*(BS320-OpF219)+s->l[2][20]*(BS620+OpF120)));
  FF120 = FA120+FF121*C21-FF221*S21;
  FF320 = FA320+FA321-FF122*S22-FF129*S29+FF322*C22+FF329*C29;
  CF120 = -(s->trq[1][20]-s->In[1][20]*OpF120-s->In[2][20]*OpF219-s->In[3][20]*OpF320-CF121*C21+CF221*S21+FA220*
 s->l[3][20]-FA320*s->l[2][20]-OM220*(s->In[3][20]*OM120+s->In[6][20]*OM220+s->In[9][20]*OM320)+OM320*(s->In[2][20]*OM120+
 s->In[5][20]*OM220+s->In[6][20]*OM320)+s->dpt[3][32]*(FF121*S21+FF221*C21));
  CF220 = -(s->trq[2][20]-s->In[2][20]*OpF120-s->In[5][20]*OpF219-s->In[6][20]*OpF320-CF121*S21-CF221*C21-FA120*
 s->l[3][20]+FA320*s->l[1][20]+OM120*(s->In[3][20]*OM120+s->In[6][20]*OM220+s->In[9][20]*OM320)-OM320*(s->In[1][20]*OM120+
 s->In[2][20]*OM220+s->In[3][20]*OM320)-s->dpt[3][32]*(FF121*C21-FF221*S21));
  CF320 = -(s->trq[3][20]-CF321-s->In[3][20]*OpF120-s->In[6][20]*OpF219-s->In[9][20]*OpF320+FA120*s->l[2][20]-FA220*
 s->l[1][20]-OM120*(s->In[2][20]*OM120+s->In[5][20]*OM220+s->In[6][20]*OM320)+OM220*(s->In[1][20]*OM120+s->In[2][20]*OM220+
 s->In[3][20]*OM320));
  FB120_1 = s->m[20]*AlM120_1;
  FB220_1 = s->m[20]*AlM219_1;
  FB320_1 = s->m[20]*AlM320_1;
  FM120_1 = FB120_1+FM121_1*C21-FM221_1*S21;
  FM320_1 = FB320_1+FB321_1-FM122_1*S22-FM129_1*S29+FM322_1*C22+FM329_1*C29;
  CM120_1 = CM121_1*C21-CM221_1*S21-FB220_1*s->l[3][20]+FB320_1*s->l[2][20]-s->dpt[3][32]*(FM121_1*S21+FM221_1*C21);
  CM220_1 = CM121_1*S21+CM221_1*C21+FB120_1*s->l[3][20]-FB320_1*s->l[1][20]+s->dpt[3][32]*(FM121_1*C21-FM221_1*S21);
  CM320_1 = CM321_1-FB120_1*s->l[2][20]+FB220_1*s->l[1][20];
  FB120_2 = s->m[20]*AlM120_2;
  FB220_2 = s->m[20]*AlM219_2;
  FB320_2 = s->m[20]*AlM320_2;
  FM120_2 = FB120_2+FM121_2*C21-FM221_2*S21;
  FM320_2 = FB320_2+FB321_2-FM122_2*S22-FM129_2*S29+FM322_2*C22+FM329_2*C29;
  CM120_2 = CM121_2*C21-CM221_2*S21-FB220_2*s->l[3][20]+FB320_2*s->l[2][20]-s->dpt[3][32]*(FM121_2*S21+FM221_2*C21);
  CM220_2 = CM121_2*S21+CM221_2*C21+FB120_2*s->l[3][20]-FB320_2*s->l[1][20]+s->dpt[3][32]*(FM121_2*C21-FM221_2*S21);
  CM320_2 = CM321_2-FB120_2*s->l[2][20]+FB220_2*s->l[1][20];
  FB120_3 = s->m[20]*AlM120_3;
  FB220_3 = s->m[20]*AlM219_3;
  FB320_3 = s->m[20]*AlM320_3;
  FM120_3 = FB120_3+FM121_3*C21-FM221_3*S21;
  FM320_3 = FB320_3+FB321_3-FM122_3*S22-FM129_3*S29+FM322_3*C22+FM329_3*C29;
  CM120_3 = CM121_3*C21-CM221_3*S21-FB220_3*s->l[3][20]+FB320_3*s->l[2][20]-s->dpt[3][32]*(FM121_3*S21+FM221_3*C21);
  CM220_3 = CM121_3*S21+CM221_3*C21+FB120_3*s->l[3][20]-FB320_3*s->l[1][20]+s->dpt[3][32]*(FM121_3*C21-FM221_3*S21);
  CM320_3 = CM321_3-FB120_3*s->l[2][20]+FB220_3*s->l[1][20];
  FB120_4 = s->m[20]*(AlM120_4+OpM219_4*s->l[3][20]-OpM320_4*s->l[2][20]);
  FB220_4 = s->m[20]*(AlM219_4-OpM120_4*s->l[3][20]+OpM320_4*s->l[1][20]);
  FB320_4 = s->m[20]*(AlM320_4+OpM120_4*s->l[2][20]-OpM219_4*s->l[1][20]);
  FM120_4 = FB120_4+FM121_4*C21-FM221_4*S21;
  FM320_4 = FB320_4+FB321_4-FM122_4*S22-FM129_4*S29+FM322_4*C22+FM329_4*C29;
  CM120_4 = s->In[1][20]*OpM120_4+s->In[2][20]*OpM219_4+s->In[3][20]*OpM320_4+CM121_4*C21-CM221_4*S21-FB220_4*
 s->l[3][20]+FB320_4*s->l[2][20]-s->dpt[3][32]*(FM121_4*S21+FM221_4*C21);
  CM220_4 = s->In[2][20]*OpM120_4+s->In[5][20]*OpM219_4+s->In[6][20]*OpM320_4+CM121_4*S21+CM221_4*C21+FB120_4*
 s->l[3][20]-FB320_4*s->l[1][20]+s->dpt[3][32]*(FM121_4*C21-FM221_4*S21);
  CM320_4 = CM321_4+s->In[3][20]*OpM120_4+s->In[6][20]*OpM219_4+s->In[9][20]*OpM320_4-FB120_4*s->l[2][20]+FB220_4*
 s->l[1][20];
  FB120_5 = s->m[20]*(AlM120_5+OpM219_5*s->l[3][20]-OpM320_5*s->l[2][20]);
  FB220_5 = s->m[20]*(AlM219_5-OpM120_5*s->l[3][20]+OpM320_5*s->l[1][20]);
  FB320_5 = s->m[20]*(AlM320_5+OpM120_5*s->l[2][20]-OpM219_5*s->l[1][20]);
  FM120_5 = FB120_5+FM121_5*C21-FM221_5*S21;
  FM320_5 = FB320_5+FB321_5-FM122_5*S22-FM129_5*S29+FM322_5*C22+FM329_5*C29;
  CM120_5 = s->In[1][20]*OpM120_5+s->In[2][20]*OpM219_5+s->In[3][20]*OpM320_5+CM121_5*C21-CM221_5*S21-FB220_5*
 s->l[3][20]+FB320_5*s->l[2][20]-s->dpt[3][32]*(FM121_5*S21+FM221_5*C21);
  CM220_5 = s->In[2][20]*OpM120_5+s->In[5][20]*OpM219_5+s->In[6][20]*OpM320_5+CM121_5*S21+CM221_5*C21+FB120_5*
 s->l[3][20]-FB320_5*s->l[1][20]+s->dpt[3][32]*(FM121_5*C21-FM221_5*S21);
  CM320_5 = CM321_5+s->In[3][20]*OpM120_5+s->In[6][20]*OpM219_5+s->In[9][20]*OpM320_5-FB120_5*s->l[2][20]+FB220_5*
 s->l[1][20];
  FB120_6 = s->m[20]*(AlM120_6-OpM320_6*s->l[2][20]+s->l[3][20]*S19);
  FB220_6 = s->m[20]*(AlM219_6-OpM120_6*s->l[3][20]+OpM320_6*s->l[1][20]);
  FB320_6 = s->m[20]*(AlM320_6+OpM120_6*s->l[2][20]-s->l[1][20]*S19);
  CM120_6 = s->In[1][20]*OpM120_6+s->In[2][20]*S19+s->In[3][20]*OpM320_6+CM121_6*C21-CM221_6*S21-FB220_6*s->l[3][20]+
 FB320_6*s->l[2][20]-s->dpt[3][32]*(FM121_6*S21+FM221_6*C21);
  CM220_6 = s->In[2][20]*OpM120_6+s->In[5][20]*S19+s->In[6][20]*OpM320_6+CM121_6*S21+CM221_6*C21+FB120_6*s->l[3][20]-
 FB320_6*s->l[1][20]+s->dpt[3][32]*(FM121_6*C21-FM221_6*S21);
  CM320_6 = CM321_6+s->In[3][20]*OpM120_6+s->In[6][20]*S19+s->In[9][20]*OpM320_6-FB120_6*s->l[2][20]+FB220_6*s->l[1][20];
  FB120_19 = -s->m[20]*s->l[2][20]*S20;
  FB220_19 = s->m[20]*(s->l[1][20]*S20-s->l[3][20]*C20);
  FB320_19 = s->m[20]*s->l[2][20]*C20;
  CM220_19 = s->In[2][20]*C20+s->In[6][20]*S20+CM121_19*S21+CM221_19*C21+FB120_19*s->l[3][20]-FB320_19*s->l[1][20]+
 s->dpt[3][32]*(FM121_19*C21-FM221_19*S21);
  CM220_20 = s->In[5][20]+s->m[20]*s->l[1][20]*s->l[1][20]+s->m[20]*s->l[3][20]*s->l[3][20]+s->dpt[3][32]*(C21*(FB121_20
 +FM122_20*C22+FM129_20*C29+FM322_20*S22+FM329_20*S29)-S21*(FB221_20+FM222_20+FM229_20))+C21*(CM222_20+CM229_20+s->In[2][21]*
 S21+s->In[5][21]*C21+FB121_20*s->l[3][21]-FB321_20*s->l[1][21]+s->dpt[1][36]*(FM122_20*S22-FM322_20*C22)+s->dpt[1][37]*(
 FM129_20*S29-FM329_20*C29)+s->dpt[3][36]*(FM122_20*C22+FM322_20*S22)+s->dpt[3][37]*(FM129_20*C29+FM329_20*S29))+S21*(
 s->In[1][21]*S21+s->In[2][21]*C21+CM122_20*C22+CM129_20*C29+CM322_20*S22+CM329_20*S29-FB221_20*s->l[3][21]+FB321_20*
 s->l[2][21]-FM222_20*s->dpt[3][36]-FM229_20*s->dpt[3][37]-s->dpt[2][36]*(FM122_20*S22-FM322_20*C22)-s->dpt[2][37]*(FM129_20*
 S29-FM329_20*C29));
  FA119 = -(s->frc[1][19]-s->m[19]*(AlF119-s->l[1][19]*(OM219*OM219+OM319*OM319)+s->l[2][19]*(BS219-OpF319)+s->l[3][19]*
 (BS319+OpF219)));
  FA219 = -(s->frc[2][19]-s->m[19]*(AlF219+s->l[1][19]*(BS219+OpF319)-s->l[2][19]*(OM119*OM119+OM319*OM319)+s->l[3][19]*
 (BS619-OpF16)));
  FA319 = -(s->frc[3][19]-s->m[19]*(AlF319+s->l[1][19]*(BS319-OpF219)+s->l[2][19]*(BS619+OpF16)-s->l[3][19]*(OM119*OM119
 +OM219*OM219)));
  FF119 = FA119+FF120*C20+FF320*S20;
  FF219 = FA219+FA220+FF121*S21+FF221*C21;
  FF319 = FA319-FF120*S20+FF320*C20;
  CF119 = -(s->trq[1][19]-s->In[1][19]*OpF16-s->In[2][19]*OpF219-s->In[3][19]*OpF319-CF120*C20-CF320*S20+FA219*
 s->l[3][19]-FA319*s->l[2][19]-OM219*(s->In[3][19]*OM119+s->In[6][19]*OM219+s->In[9][19]*OM319)+OM319*(s->In[2][19]*OM119+
 s->In[5][19]*OM219+s->In[6][19]*OM319));
  CF219 = -(s->trq[2][19]-CF220-s->In[2][19]*OpF16-s->In[5][19]*OpF219-s->In[6][19]*OpF319-FA119*s->l[3][19]+FA319*
 s->l[1][19]+OM119*(s->In[3][19]*OM119+s->In[6][19]*OM219+s->In[9][19]*OM319)-OM319*(s->In[1][19]*OM119+s->In[2][19]*OM219+
 s->In[3][19]*OM319));
  CF319 = -(s->trq[3][19]-s->In[3][19]*OpF16-s->In[6][19]*OpF219-s->In[9][19]*OpF319+CF120*S20-CF320*C20+FA119*
 s->l[2][19]-FA219*s->l[1][19]-OM119*(s->In[2][19]*OM119+s->In[5][19]*OM219+s->In[6][19]*OM319)+OM219*(s->In[1][19]*OM119+
 s->In[2][19]*OM219+s->In[3][19]*OM319));
  FB119_1 = s->m[19]*AlM16_1;
  FB219_1 = s->m[19]*AlM219_1;
  FB319_1 = s->m[19]*AlM319_1;
  FM119_1 = FB119_1+FM120_1*C20+FM320_1*S20;
  FM219_1 = FB219_1+FB220_1+FM121_1*S21+FM221_1*C21;
  FM319_1 = FB319_1-FM120_1*S20+FM320_1*C20;
  CM119_1 = CM120_1*C20+CM320_1*S20-FB219_1*s->l[3][19]+FB319_1*s->l[2][19];
  CM219_1 = CM220_1+FB119_1*s->l[3][19]-FB319_1*s->l[1][19];
  CM319_1 = -(CM120_1*S20-CM320_1*C20+FB119_1*s->l[2][19]-FB219_1*s->l[1][19]);
  FB119_2 = s->m[19]*AlM16_2;
  FB219_2 = s->m[19]*AlM219_2;
  FB319_2 = s->m[19]*AlM319_2;
  FM119_2 = FB119_2+FM120_2*C20+FM320_2*S20;
  FM219_2 = FB219_2+FB220_2+FM121_2*S21+FM221_2*C21;
  FM319_2 = FB319_2-FM120_2*S20+FM320_2*C20;
  CM119_2 = CM120_2*C20+CM320_2*S20-FB219_2*s->l[3][19]+FB319_2*s->l[2][19];
  CM219_2 = CM220_2+FB119_2*s->l[3][19]-FB319_2*s->l[1][19];
  CM319_2 = -(CM120_2*S20-CM320_2*C20+FB119_2*s->l[2][19]-FB219_2*s->l[1][19]);
  FB119_3 = s->m[19]*AlM16_3;
  FB219_3 = s->m[19]*AlM219_3;
  FB319_3 = s->m[19]*AlM319_3;
  FM119_3 = FB119_3+FM120_3*C20+FM320_3*S20;
  FM219_3 = FB219_3+FB220_3+FM121_3*S21+FM221_3*C21;
  FM319_3 = FB319_3-FM120_3*S20+FM320_3*C20;
  CM119_3 = CM120_3*C20+CM320_3*S20-FB219_3*s->l[3][19]+FB319_3*s->l[2][19];
  CM219_3 = CM220_3+FB119_3*s->l[3][19]-FB319_3*s->l[1][19];
  CM319_3 = -(CM120_3*S20-CM320_3*C20+FB119_3*s->l[2][19]-FB219_3*s->l[1][19]);
  FB119_4 = s->m[19]*(AlM119_4+OpM219_4*s->l[3][19]-OpM319_4*s->l[2][19]);
  FB219_4 = s->m[19]*(AlM219_4-OpM16_4*s->l[3][19]+OpM319_4*s->l[1][19]);
  FB319_4 = s->m[19]*(AlM319_4+OpM16_4*s->l[2][19]-OpM219_4*s->l[1][19]);
  FM219_4 = FB219_4+FB220_4+FM121_4*S21+FM221_4*C21;
  FM319_4 = FB319_4-FM120_4*S20+FM320_4*C20;
  CM119_4 = s->In[1][19]*OpM16_4+s->In[2][19]*OpM219_4+s->In[3][19]*OpM319_4+CM120_4*C20+CM320_4*S20-FB219_4*s->l[3][19]
 +FB319_4*s->l[2][19];
  CM219_4 = CM220_4+s->In[2][19]*OpM16_4+s->In[5][19]*OpM219_4+s->In[6][19]*OpM319_4+FB119_4*s->l[3][19]-FB319_4*
 s->l[1][19];
  CM319_4 = s->In[3][19]*OpM16_4+s->In[6][19]*OpM219_4+s->In[9][19]*OpM319_4-CM120_4*S20+CM320_4*C20-FB119_4*s->l[2][19]
 +FB219_4*s->l[1][19];
  FB119_5 = s->m[19]*(AlM119_5+OpM219_5*s->l[3][19]-OpM319_5*s->l[2][19]);
  FB219_5 = s->m[19]*(AlM219_5+OpM319_5*s->l[1][19]-s->l[3][19]*S6);
  FB319_5 = s->m[19]*(AlM319_5-OpM219_5*s->l[1][19]+s->l[2][19]*S6);
  FM219_5 = FB219_5+FB220_5+FM121_5*S21+FM221_5*C21;
  FM319_5 = FB319_5-FM120_5*S20+FM320_5*C20;
  CM119_5 = s->In[1][19]*S6+s->In[2][19]*OpM219_5+s->In[3][19]*OpM319_5+CM120_5*C20+CM320_5*S20-FB219_5*s->l[3][19]+
 FB319_5*s->l[2][19];
  CM219_5 = CM220_5+s->In[2][19]*S6+s->In[5][19]*OpM219_5+s->In[6][19]*OpM319_5+FB119_5*s->l[3][19]-FB319_5*s->l[1][19];
  CM319_5 = s->In[3][19]*S6+s->In[6][19]*OpM219_5+s->In[9][19]*OpM319_5-CM120_5*S20+CM320_5*C20-FB119_5*s->l[2][19]+
 FB219_5*s->l[1][19];
  FB119_6 = -s->m[19]*(s->l[2][19]*C19-s->l[3][19]*S19);
  FB219_6 = s->m[19]*C19*(s->dpt[1][3]+s->l[1][19]);
  FB319_6 = s->m[19]*(AlM319_6-s->l[1][19]*S19);
  CM119_6 = s->In[2][19]*S19+s->In[3][19]*C19+CM120_6*C20+CM320_6*S20-FB219_6*s->l[3][19]+FB319_6*s->l[2][19];
  CM119_19 = s->In[1][19]+s->m[19]*s->l[2][19]*s->l[2][19]+s->m[19]*s->l[3][19]*s->l[3][19]+C20*(s->In[1][20]*C20+
 s->In[3][20]*S20+CM121_19*C21-CM221_19*S21-FB220_19*s->l[3][20]+FB320_19*s->l[2][20]-s->dpt[3][32]*(FM121_19*S21+FM221_19*
 C21))+S20*(CM321_19+s->In[3][20]*C20+s->In[9][20]*S20-FB120_19*s->l[2][20]+FB220_19*s->l[1][20]);

// = = Block_0_2_0_3_0_1 = = 
 
// Backward Dynamics 

  FA16 = -(s->frc[1][6]-s->m[6]*(AlF16+BS16*s->l[1][6]+BeF26*s->l[2][6]+BeF36*s->l[3][6]));
  FA26 = -(s->frc[2][6]-s->m[6]*(AlF26+BS56*s->l[2][6]+BeF46*s->l[1][6]+BeF66*s->l[3][6]));
  FA36 = -(s->frc[3][6]-s->m[6]*(AlF35+BS96*s->l[3][6]+BeF76*s->l[1][6]+BeF86*s->l[2][6]));
  FF16 = FA16+FF119+FF113*C13+FF17*C7+FF313*S13+FF37*S7;
  FF26 = FA213+FA26+FA27+FF214*C14+FF219*C19+FF28*C8-FF314*S14-FF319*S19-FF38*S8;
  FF36 = FA36-FF113*S13-FF17*S7+FF219*S19+FF313*C13+FF319*C19+FF37*C7;
  CF16 = -(s->trq[1][6]-CF119-s->In[1][6]*OpF16-s->In[2][6]*OpF26-s->In[3][6]*OpF35-CF113*C13-CF17*C7-CF313*S13-CF37*S7+
 FA26*s->l[3][6]-FA36*s->l[2][6]-OM26*(s->In[3][6]*OM16+s->In[6][6]*OM26+s->In[9][6]*OM36)+OM36*(s->In[2][6]*OM16+s->In[5][6]
 *OM26+s->In[6][6]*OM36)+s->dpt[2][1]*(FF17*S7-FF37*C7)+s->dpt[2][2]*(FF113*S13-FF313*C13)+s->dpt[3][3]*(FF219*C19-FF319*S19)
 );
  CF26 = -(s->trq[2][6]-CF213-CF27-s->In[2][6]*OpF16-s->In[5][6]*OpF26-s->In[6][6]*OpF35-CF219*C19+CF319*S19-FA16*
 s->l[3][6]+FA36*s->l[1][6]-FF119*s->dpt[3][3]+OM16*(s->In[3][6]*OM16+s->In[6][6]*OM26+s->In[9][6]*OM36)-OM36*(s->In[1][6]*
 OM16+s->In[2][6]*OM26+s->In[3][6]*OM36)+s->dpt[1][3]*(FF219*S19+FF319*C19));
  CF36 = -(s->trq[3][6]-s->In[3][6]*OpF16-s->In[6][6]*OpF26-s->In[9][6]*OpF35+CF113*S13+CF17*S7-CF219*S19-CF313*C13-
 CF319*C19-CF37*C7+FA16*s->l[2][6]-FA26*s->l[1][6]-OM16*(s->In[2][6]*OM16+s->In[5][6]*OM26+s->In[6][6]*OM36)+OM26*(
 s->In[1][6]*OM16+s->In[2][6]*OM26+s->In[3][6]*OM36)-s->dpt[1][3]*(FF219*C19-FF319*S19)+s->dpt[2][1]*(FF17*C7+FF37*S7)+
 s->dpt[2][2]*(FF113*C13+FF313*S13));
  FB16_1 = s->m[6]*AlM16_1;
  FB26_1 = s->m[6]*AlM26_1;
  FB36_1 = s->m[6]*S5;
  FM16_1 = FB16_1+FM119_1+FM113_1*C13+FM17_1*C7+FM313_1*S13+FM37_1*S7;
  FM26_1 = FB213_1+FB26_1+FB27_1+FM214_1*C14+FM219_1*C19+FM28_1*C8-FM314_1*S14-FM319_1*S19-FM38_1*S8;
  FM36_1 = FB36_1-FM113_1*S13-FM17_1*S7+FM219_1*S19+FM313_1*C13+FM319_1*C19+FM37_1*C7;
  CM16_1 = CM119_1+CM113_1*C13+CM17_1*C7+CM313_1*S13+CM37_1*S7-FB26_1*s->l[3][6]+FB36_1*s->l[2][6]-s->dpt[2][1]*(FM17_1*
 S7-FM37_1*C7)-s->dpt[2][2]*(FM113_1*S13-FM313_1*C13)-s->dpt[3][3]*(FM219_1*C19-FM319_1*S19);
  CM26_1 = CM213_1+CM27_1+CM219_1*C19-CM319_1*S19+FB16_1*s->l[3][6]-FB36_1*s->l[1][6]+FM119_1*s->dpt[3][3]-s->dpt[1][3]*
 (FM219_1*S19+FM319_1*C19);
  CM36_1 = CM219_1*S19+CM319_1*C19+s->dpt[1][3]*(FM219_1*C19-FM319_1*S19)-CM113_1*S13-CM17_1*S7+CM313_1*C13+CM37_1*C7-
 FB16_1*s->l[2][6]+FB26_1*s->l[1][6]-s->dpt[2][1]*(FM17_1*C7+FM37_1*S7)-s->dpt[2][2]*(FM113_1*C13+FM313_1*S13);
  FB16_2 = s->m[6]*AlM16_2;
  FB26_2 = s->m[6]*AlM26_2;
  FB36_2 = s->m[6]*AlM35_2;
  FM16_2 = FB16_2+FM119_2+FM113_2*C13+FM17_2*C7+FM313_2*S13+FM37_2*S7;
  FM26_2 = FB213_2+FB26_2+FB27_2+FM214_2*C14+FM219_2*C19+FM28_2*C8-FM314_2*S14-FM319_2*S19-FM38_2*S8;
  CM16_2 = CM119_2+CM113_2*C13+CM17_2*C7+CM313_2*S13+CM37_2*S7-FB26_2*s->l[3][6]+FB36_2*s->l[2][6]-s->dpt[2][1]*(FM17_2*
 S7-FM37_2*C7)-s->dpt[2][2]*(FM113_2*S13-FM313_2*C13)-s->dpt[3][3]*(FM219_2*C19-FM319_2*S19);
  CM26_2 = CM213_2+CM27_2+CM219_2*C19-CM319_2*S19+FB16_2*s->l[3][6]-FB36_2*s->l[1][6]+FM119_2*s->dpt[3][3]-s->dpt[1][3]*
 (FM219_2*S19+FM319_2*C19);
  CM36_2 = CM219_2*S19+CM319_2*C19+s->dpt[1][3]*(FM219_2*C19-FM319_2*S19)-CM113_2*S13-CM17_2*S7+CM313_2*C13+CM37_2*C7-
 FB16_2*s->l[2][6]+FB26_2*s->l[1][6]-s->dpt[2][1]*(FM17_2*C7+FM37_2*S7)-s->dpt[2][2]*(FM113_2*C13+FM313_2*S13);
  FB16_3 = s->m[6]*AlM16_3;
  FB26_3 = s->m[6]*AlM26_3;
  FB36_3 = s->m[6]*AlM35_3;
  FM16_3 = FB16_3+FM119_3+FM113_3*C13+FM17_3*C7+FM313_3*S13+FM37_3*S7;
  FM26_3 = FB213_3+FB26_3+FB27_3+FM214_3*C14+FM219_3*C19+FM28_3*C8-FM314_3*S14-FM319_3*S19-FM38_3*S8;
  CM16_3 = CM119_3+CM113_3*C13+CM17_3*C7+CM313_3*S13+CM37_3*S7-FB26_3*s->l[3][6]+FB36_3*s->l[2][6]-s->dpt[2][1]*(FM17_3*
 S7-FM37_3*C7)-s->dpt[2][2]*(FM113_3*S13-FM313_3*C13)-s->dpt[3][3]*(FM219_3*C19-FM319_3*S19);
  CM26_3 = CM213_3+CM27_3+CM219_3*C19-CM319_3*S19+FB16_3*s->l[3][6]-FB36_3*s->l[1][6]+FM119_3*s->dpt[3][3]-s->dpt[1][3]*
 (FM219_3*S19+FM319_3*C19);
  CM36_3 = CM219_3*S19+CM319_3*C19+s->dpt[1][3]*(FM219_3*C19-FM319_3*S19)-CM113_3*S13-CM17_3*S7+CM313_3*C13+CM37_3*C7-
 FB16_3*s->l[2][6]+FB26_3*s->l[1][6]-s->dpt[2][1]*(FM17_3*C7+FM37_3*S7)-s->dpt[2][2]*(FM113_3*C13+FM313_3*S13);
  FB16_4 = s->m[6]*(OpM26_4*s->l[3][6]-s->l[2][6]*S5);
  FB26_4 = -s->m[6]*(OpM16_4*s->l[3][6]-s->l[1][6]*S5);
  FB36_4 = s->m[6]*(OpM16_4*s->l[2][6]-OpM26_4*s->l[1][6]);
  CM16_4 = CM119_4+s->In[1][6]*OpM16_4+s->In[2][6]*OpM26_4+s->In[3][6]*S5+CM113_4*C13+CM17_4*C7+CM313_4*S13+CM37_4*S7-
 FB26_4*s->l[3][6]+FB36_4*s->l[2][6]-s->dpt[2][1]*(FM17_4*S7-FM37_4*C7)-s->dpt[2][2]*(FM113_4*S13-FM313_4*C13)-s->dpt[3][3]*(
 FM219_4*C19-FM319_4*S19);
  CM26_4 = CM213_4+CM27_4+s->In[2][6]*OpM16_4+s->In[5][6]*OpM26_4+s->In[6][6]*S5+CM219_4*C19-CM319_4*S19+FB16_4*
 s->l[3][6]-FB36_4*s->l[1][6]-s->dpt[1][3]*(FM219_4*S19+FM319_4*C19)+s->dpt[3][3]*(FB119_4+FM120_4*C20+FM320_4*S20);
  CM36_4 = s->In[3][6]*OpM16_4+s->In[6][6]*OpM26_4+s->In[9][6]*S5-CM113_4*S13-CM17_4*S7+CM219_4*S19+CM313_4*C13+CM319_4*
 C19+CM37_4*C7-FB16_4*s->l[2][6]+FB26_4*s->l[1][6]+s->dpt[1][3]*(FM219_4*C19-FM319_4*S19)-s->dpt[2][1]*(FM17_4*C7+FM37_4*S7)-
 s->dpt[2][2]*(FM113_4*C13+FM313_4*S13);
  FB16_5 = s->m[6]*s->l[3][6]*C6;
  FB26_5 = -s->m[6]*s->l[3][6]*S6;
  FB36_5 = -s->m[6]*(s->l[1][6]*C6-s->l[2][6]*S6);
  CM36_5 = s->In[3][6]*S6+s->In[6][6]*C6-CM113_5*S13-CM17_5*S7+CM219_5*S19+CM313_5*C13+CM319_5*C19+CM37_5*C7-FB16_5*
 s->l[2][6]+FB26_5*s->l[1][6]+s->dpt[1][3]*(FM219_5*C19-FM319_5*S19)-s->dpt[2][1]*(FM17_5*C7+FM37_5*S7)-s->dpt[2][2]*(FM113_5
 *C13+FM313_5*S13);
  CM36_6 = s->In[9][6]+s->m[6]*s->l[1][6]*s->l[1][6]+s->m[6]*s->l[2][6]*s->l[2][6]+s->dpt[1][3]*(C19*(FB219_6+FB220_6+
 FM121_6*S21+FM221_6*C21)-S19*(FB319_6+C20*(FB320_6+FB321_6-FM122_6*S22-FM129_6*S29+FM322_6*C22+FM329_6*C29)-S20*(FB120_6+
 FM121_6*C21-FM221_6*S21)))-s->dpt[2][1]*(C7*(FB17_6+FM18_6)+S7*(FB37_6+FM28_6*S8+FM38_6*C8))-s->dpt[2][2]*(C13*(FB113_6+
 FM114_6)+S13*(FB313_6+FM214_6*S14+FM314_6*C14))-C13*(s->In[3][13]*S13-s->In[9][13]*C13-CM214_6*S14-CM314_6*C14+FB113_6*
 s->l[2][13]-FB213_6*s->l[1][13]+FM114_6*s->dpt[2][18])-S13*(CM114_6-s->In[1][13]*S13+s->In[3][13]*C13-FB213_6*s->l[3][13]+
 FB313_6*s->l[2][13]+s->dpt[2][18]*(FM214_6*S14+FM314_6*C14))+C19*(s->In[6][19]*S19+s->In[9][19]*C19-CM120_6*S20+CM320_6*C20-
 FB119_6*s->l[2][19]+FB219_6*s->l[1][19])+S19*(CM220_6+s->In[5][19]*S19+s->In[6][19]*C19+FB119_6*s->l[3][19]-FB319_6*
 s->l[1][19])-C7*(s->In[3][7]*S7-s->In[9][7]*C7-CM28_6*S8-CM38_6*C8+FB17_6*s->l[2][7]-FB27_6*s->l[1][7]+FM18_6*s->dpt[2][6])-
 S7*(CM18_6-s->In[1][7]*S7+s->In[3][7]*C7-FB27_6*s->l[3][7]+FB37_6*s->l[2][7]+s->dpt[2][6]*(FM28_6*S8+FM38_6*C8));
  FF5_16 = FF16*C6-FF26*S6;
  FF5_26 = FF16*S6+FF26*C6;
  CF5_26 = CF16*S6+CF26*C6;
  FM51_16 = FM16_1*C6-FM26_1*S6;
  FM51_26 = FM16_1*S6+FM26_1*C6;
  CM51_26 = CM16_1*S6+CM26_1*C6;
  FM52_26 = FM16_2*S6+FM26_2*C6;
  CM52_26 = CM16_2*S6+CM26_2*C6;
  CM53_26 = CM16_3*S6+CM26_3*C6;
  CM54_26 = CM16_4*S6+CM26_4*C6;
  CM55_26 = C6*(CM213_5+CM27_5+s->In[2][6]*S6+s->In[5][6]*C6+CM219_5*C19-CM319_5*S19+FB16_5*s->l[3][6]-FB36_5*s->l[1][6]
 -s->dpt[1][3]*(FM219_5*S19+FM319_5*C19)+s->dpt[3][3]*(FB119_5+FM120_5*C20+FM320_5*S20))+S6*(CM119_5+s->In[1][6]*S6+
 s->In[2][6]*C6+CM113_5*C13+CM17_5*C7+CM313_5*S13+CM37_5*S7-FB26_5*s->l[3][6]+FB36_5*s->l[2][6]-s->dpt[2][1]*(FM17_5*S7-
 FM37_5*C7)-s->dpt[2][2]*(FM113_5*S13-FM313_5*C13)-s->dpt[3][3]*(FM219_5*C19-FM319_5*S19));
  FF4_15 = FF36*S5+FF5_16*C5;
  FF4_35 = FF36*C5-FF5_16*S5;
  CF4_15 = CF36*S5+C5*(CF16*C6-CF26*S6);
  FM41_15 = FM36_1*S5+FM51_16*C5;
  FM41_35 = FM36_1*C5-FM51_16*S5;
  CM41_15 = CM36_1*S5+C5*(CM16_1*C6-CM26_1*S6);
  FM42_35 = C5*(FB36_2-FM113_2*S13-FM17_2*S7+FM219_2*S19+FM313_2*C13+FM319_2*C19+FM37_2*C7)-S5*(FM16_2*C6-FM26_2*S6);
  CM42_15 = CM36_2*S5+C5*(CM16_2*C6-CM26_2*S6);
  CM43_15 = CM36_3*S5+C5*(CM16_3*C6-CM26_3*S6);
  CM44_15 = CM36_4*S5+C5*(CM16_4*C6-CM26_4*S6);
  FF3_24 = -(FF4_35*S4-FF5_26*C4);
  FF3_34 = FF4_35*C4+FF5_26*S4;
  FM31_24 = -(FM41_35*S4-FM51_26*C4);
  FM31_34 = FM41_35*C4+FM51_26*S4;
  FM32_24 = -(FM42_35*S4-FM52_26*C4);
  FM32_34 = FM42_35*C4+FM52_26*S4;
  FM33_34 = C4*(C5*(FB36_3-FM113_3*S13-FM17_3*S7+FM219_3*S19+FM313_3*C13+FM319_3*C19+FM37_3*C7)-S5*(FM16_3*C6-FM26_3*S6)
 )+S4*(FM16_3*S6+FM26_3*C6);

// = = Block_0_3_0_0_0_0 = = 
 
// Symbolic Outputs  

  M[1][1] = FM41_15;
  M[1][2] = FM31_24;
  M[1][3] = FM31_34;
  M[1][4] = CM41_15;
  M[1][5] = CM51_26;
  M[1][6] = CM36_1;
  M[1][7] = CM27_1;
  M[1][8] = CM18_1;
  M[1][9] = CM39_1;
  M[1][10] = CM210_1;
  M[1][11] = CM111_1;
  M[1][12] = CM212_1;
  M[1][13] = CM213_1;
  M[1][14] = CM114_1;
  M[1][15] = CM315_1;
  M[1][16] = CM216_1;
  M[1][17] = CM117_1;
  M[1][18] = CM218_1;
  M[1][19] = CM119_1;
  M[1][20] = CM220_1;
  M[1][21] = CM321_1;
  M[1][22] = CM222_1;
  M[1][23] = CM123_1;
  M[1][24] = CM224_1;
  M[1][25] = CM325_1;
  M[1][29] = CM229_1;
  M[1][30] = CM130_1;
  M[1][31] = CM231_1;
  M[1][32] = CM332_1;
  M[2][1] = FM31_24;
  M[2][2] = FM32_24;
  M[2][3] = FM32_34;
  M[2][4] = CM42_15;
  M[2][5] = CM52_26;
  M[2][6] = CM36_2;
  M[2][7] = CM27_2;
  M[2][8] = CM18_2;
  M[2][9] = CM39_2;
  M[2][10] = CM210_2;
  M[2][11] = CM111_2;
  M[2][12] = CM212_2;
  M[2][13] = CM213_2;
  M[2][14] = CM114_2;
  M[2][15] = CM315_2;
  M[2][16] = CM216_2;
  M[2][17] = CM117_2;
  M[2][18] = CM218_2;
  M[2][19] = CM119_2;
  M[2][20] = CM220_2;
  M[2][21] = CM321_2;
  M[2][22] = CM222_2;
  M[2][23] = CM123_2;
  M[2][24] = CM224_2;
  M[2][25] = CM325_2;
  M[2][29] = CM229_2;
  M[2][30] = CM130_2;
  M[2][31] = CM231_2;
  M[2][32] = CM332_2;
  M[3][1] = FM31_34;
  M[3][2] = FM32_34;
  M[3][3] = FM33_34;
  M[3][4] = CM43_15;
  M[3][5] = CM53_26;
  M[3][6] = CM36_3;
  M[3][7] = CM27_3;
  M[3][8] = CM18_3;
  M[3][9] = CM39_3;
  M[3][10] = CM210_3;
  M[3][11] = CM111_3;
  M[3][12] = CM212_3;
  M[3][13] = CM213_3;
  M[3][14] = CM114_3;
  M[3][15] = CM315_3;
  M[3][16] = CM216_3;
  M[3][17] = CM117_3;
  M[3][18] = CM218_3;
  M[3][19] = CM119_3;
  M[3][20] = CM220_3;
  M[3][21] = CM321_3;
  M[3][22] = CM222_3;
  M[3][23] = CM123_3;
  M[3][24] = CM224_3;
  M[3][25] = CM325_3;
  M[3][29] = CM229_3;
  M[3][30] = CM130_3;
  M[3][31] = CM231_3;
  M[3][32] = CM332_3;
  M[4][1] = CM41_15;
  M[4][2] = CM42_15;
  M[4][3] = CM43_15;
  M[4][4] = CM44_15;
  M[4][5] = CM54_26;
  M[4][6] = CM36_4;
  M[4][7] = CM27_4;
  M[4][8] = CM18_4;
  M[4][9] = CM39_4;
  M[4][10] = CM210_4;
  M[4][11] = CM111_4;
  M[4][12] = CM212_4;
  M[4][13] = CM213_4;
  M[4][14] = CM114_4;
  M[4][15] = CM315_4;
  M[4][16] = CM216_4;
  M[4][17] = CM117_4;
  M[4][18] = CM218_4;
  M[4][19] = CM119_4;
  M[4][20] = CM220_4;
  M[4][21] = CM321_4;
  M[4][22] = CM222_4;
  M[4][23] = CM123_4;
  M[4][24] = CM224_4;
  M[4][25] = CM325_4;
  M[4][26] = CM226_4;
  M[4][27] = CM327_4;
  M[4][28] = CM128_4;
  M[4][29] = CM229_4;
  M[4][30] = CM130_4;
  M[4][31] = CM231_4;
  M[4][32] = CM332_4;
  M[4][33] = CM233_4;
  M[4][34] = CM334_4;
  M[4][35] = CM135_4;
  M[5][1] = CM51_26;
  M[5][2] = CM52_26;
  M[5][3] = CM53_26;
  M[5][4] = CM54_26;
  M[5][5] = CM55_26;
  M[5][6] = CM36_5;
  M[5][7] = CM27_5;
  M[5][8] = CM18_5;
  M[5][9] = CM39_5;
  M[5][10] = CM210_5;
  M[5][11] = CM111_5;
  M[5][12] = CM212_5;
  M[5][13] = CM213_5;
  M[5][14] = CM114_5;
  M[5][15] = CM315_5;
  M[5][16] = CM216_5;
  M[5][17] = CM117_5;
  M[5][18] = CM218_5;
  M[5][19] = CM119_5;
  M[5][20] = CM220_5;
  M[5][21] = CM321_5;
  M[5][22] = CM222_5;
  M[5][23] = CM123_5;
  M[5][24] = CM224_5;
  M[5][25] = CM325_5;
  M[5][26] = CM226_5;
  M[5][27] = CM327_5;
  M[5][28] = CM128_5;
  M[5][29] = CM229_5;
  M[5][30] = CM130_5;
  M[5][31] = CM231_5;
  M[5][32] = CM332_5;
  M[5][33] = CM233_5;
  M[5][34] = CM334_5;
  M[5][35] = CM135_5;
  M[6][1] = CM36_1;
  M[6][2] = CM36_2;
  M[6][3] = CM36_3;
  M[6][4] = CM36_4;
  M[6][5] = CM36_5;
  M[6][6] = CM36_6;
  M[6][7] = CM27_6;
  M[6][8] = CM18_6;
  M[6][9] = CM39_6;
  M[6][10] = CM210_6;
  M[6][11] = CM111_6;
  M[6][12] = CM212_6;
  M[6][13] = CM213_6;
  M[6][14] = CM114_6;
  M[6][15] = CM315_6;
  M[6][16] = CM216_6;
  M[6][17] = CM117_6;
  M[6][18] = CM218_6;
  M[6][19] = CM119_6;
  M[6][20] = CM220_6;
  M[6][21] = CM321_6;
  M[6][22] = CM222_6;
  M[6][23] = CM123_6;
  M[6][24] = CM224_6;
  M[6][25] = CM325_6;
  M[6][26] = CM226_6;
  M[6][27] = CM327_6;
  M[6][28] = CM128_6;
  M[6][29] = CM229_6;
  M[6][30] = CM130_6;
  M[6][31] = CM231_6;
  M[6][32] = CM332_6;
  M[6][33] = CM233_6;
  M[6][34] = CM334_6;
  M[6][35] = CM135_6;
  M[7][1] = CM27_1;
  M[7][2] = CM27_2;
  M[7][3] = CM27_3;
  M[7][4] = CM27_4;
  M[7][5] = CM27_5;
  M[7][6] = CM27_6;
  M[7][7] = CM27_7;
  M[7][8] = CM18_7;
  M[7][9] = CM39_7;
  M[7][10] = CM210_7;
  M[7][11] = CM111_7;
  M[7][12] = CM212_7;
  M[8][1] = CM18_1;
  M[8][2] = CM18_2;
  M[8][3] = CM18_3;
  M[8][4] = CM18_4;
  M[8][5] = CM18_5;
  M[8][6] = CM18_6;
  M[8][7] = CM18_7;
  M[8][8] = CM18_8;
  M[8][9] = CM39_8;
  M[8][10] = CM210_8;
  M[8][11] = CM111_8;
  M[8][12] = CM212_8;
  M[9][1] = CM39_1;
  M[9][2] = CM39_2;
  M[9][3] = CM39_3;
  M[9][4] = CM39_4;
  M[9][5] = CM39_5;
  M[9][6] = CM39_6;
  M[9][7] = CM39_7;
  M[9][8] = CM39_8;
  M[9][9] = CM39_9;
  M[9][10] = CM210_9;
  M[9][11] = CM111_9;
  M[9][12] = CM212_9;
  M[10][1] = CM210_1;
  M[10][2] = CM210_2;
  M[10][3] = CM210_3;
  M[10][4] = CM210_4;
  M[10][5] = CM210_5;
  M[10][6] = CM210_6;
  M[10][7] = CM210_7;
  M[10][8] = CM210_8;
  M[10][9] = CM210_9;
  M[10][10] = CM210_10;
  M[10][11] = CM111_10;
  M[10][12] = CM212_10;
  M[11][1] = CM111_1;
  M[11][2] = CM111_2;
  M[11][3] = CM111_3;
  M[11][4] = CM111_4;
  M[11][5] = CM111_5;
  M[11][6] = CM111_6;
  M[11][7] = CM111_7;
  M[11][8] = CM111_8;
  M[11][9] = CM111_9;
  M[11][10] = CM111_10;
  M[11][11] = CM111_11;
  M[11][12] = CM212_11;
  M[12][1] = CM212_1;
  M[12][2] = CM212_2;
  M[12][3] = CM212_3;
  M[12][4] = CM212_4;
  M[12][5] = CM212_5;
  M[12][6] = CM212_6;
  M[12][7] = CM212_7;
  M[12][8] = CM212_8;
  M[12][9] = CM212_9;
  M[12][10] = CM212_10;
  M[12][11] = CM212_11;
  M[12][12] = CM212_12;
  M[13][1] = CM213_1;
  M[13][2] = CM213_2;
  M[13][3] = CM213_3;
  M[13][4] = CM213_4;
  M[13][5] = CM213_5;
  M[13][6] = CM213_6;
  M[13][13] = CM213_13;
  M[13][14] = CM114_13;
  M[13][15] = CM315_13;
  M[13][16] = CM216_13;
  M[13][17] = CM117_13;
  M[13][18] = CM218_13;
  M[14][1] = CM114_1;
  M[14][2] = CM114_2;
  M[14][3] = CM114_3;
  M[14][4] = CM114_4;
  M[14][5] = CM114_5;
  M[14][6] = CM114_6;
  M[14][13] = CM114_13;
  M[14][14] = CM114_14;
  M[14][15] = CM315_14;
  M[14][16] = CM216_14;
  M[14][17] = CM117_14;
  M[14][18] = CM218_14;
  M[15][1] = CM315_1;
  M[15][2] = CM315_2;
  M[15][3] = CM315_3;
  M[15][4] = CM315_4;
  M[15][5] = CM315_5;
  M[15][6] = CM315_6;
  M[15][13] = CM315_13;
  M[15][14] = CM315_14;
  M[15][15] = CM315_15;
  M[15][16] = CM216_15;
  M[15][17] = CM117_15;
  M[15][18] = CM218_15;
  M[16][1] = CM216_1;
  M[16][2] = CM216_2;
  M[16][3] = CM216_3;
  M[16][4] = CM216_4;
  M[16][5] = CM216_5;
  M[16][6] = CM216_6;
  M[16][13] = CM216_13;
  M[16][14] = CM216_14;
  M[16][15] = CM216_15;
  M[16][16] = CM216_16;
  M[16][17] = CM117_16;
  M[16][18] = CM218_16;
  M[17][1] = CM117_1;
  M[17][2] = CM117_2;
  M[17][3] = CM117_3;
  M[17][4] = CM117_4;
  M[17][5] = CM117_5;
  M[17][6] = CM117_6;
  M[17][13] = CM117_13;
  M[17][14] = CM117_14;
  M[17][15] = CM117_15;
  M[17][16] = CM117_16;
  M[17][17] = CM117_17;
  M[17][18] = CM218_17;
  M[18][1] = CM218_1;
  M[18][2] = CM218_2;
  M[18][3] = CM218_3;
  M[18][4] = CM218_4;
  M[18][5] = CM218_5;
  M[18][6] = CM218_6;
  M[18][13] = CM218_13;
  M[18][14] = CM218_14;
  M[18][15] = CM218_15;
  M[18][16] = CM218_16;
  M[18][17] = CM218_17;
  M[18][18] = CM218_18;
  M[19][1] = CM119_1;
  M[19][2] = CM119_2;
  M[19][3] = CM119_3;
  M[19][4] = CM119_4;
  M[19][5] = CM119_5;
  M[19][6] = CM119_6;
  M[19][19] = CM119_19;
  M[19][20] = CM220_19;
  M[19][21] = CM321_19;
  M[19][22] = CM222_19;
  M[19][23] = CM123_19;
  M[19][24] = CM224_19;
  M[19][25] = CM325_19;
  M[19][26] = CM226_19;
  M[19][27] = CM327_19;
  M[19][28] = CM128_19;
  M[19][29] = CM229_19;
  M[19][30] = CM130_19;
  M[19][31] = CM231_19;
  M[19][32] = CM332_19;
  M[19][33] = CM233_19;
  M[19][34] = CM334_19;
  M[19][35] = CM135_19;
  M[20][1] = CM220_1;
  M[20][2] = CM220_2;
  M[20][3] = CM220_3;
  M[20][4] = CM220_4;
  M[20][5] = CM220_5;
  M[20][6] = CM220_6;
  M[20][19] = CM220_19;
  M[20][20] = CM220_20;
  M[20][21] = CM321_20;
  M[20][22] = CM222_20;
  M[20][23] = CM123_20;
  M[20][24] = CM224_20;
  M[20][25] = CM325_20;
  M[20][26] = CM226_20;
  M[20][27] = CM327_20;
  M[20][28] = CM128_20;
  M[20][29] = CM229_20;
  M[20][30] = CM130_20;
  M[20][31] = CM231_20;
  M[20][32] = CM332_20;
  M[20][33] = CM233_20;
  M[20][34] = CM334_20;
  M[20][35] = CM135_20;
  M[21][1] = CM321_1;
  M[21][2] = CM321_2;
  M[21][3] = CM321_3;
  M[21][4] = CM321_4;
  M[21][5] = CM321_5;
  M[21][6] = CM321_6;
  M[21][19] = CM321_19;
  M[21][20] = CM321_20;
  M[21][21] = CM321_21;
  M[21][22] = CM222_21;
  M[21][23] = CM123_21;
  M[21][24] = CM224_21;
  M[21][25] = CM325_21;
  M[21][26] = CM226_21;
  M[21][27] = CM327_21;
  M[21][28] = CM128_21;
  M[21][29] = CM229_21;
  M[21][30] = CM130_21;
  M[21][31] = CM231_21;
  M[21][32] = CM332_21;
  M[21][33] = CM233_21;
  M[21][34] = CM334_21;
  M[21][35] = CM135_21;
  M[22][1] = CM222_1;
  M[22][2] = CM222_2;
  M[22][3] = CM222_3;
  M[22][4] = CM222_4;
  M[22][5] = CM222_5;
  M[22][6] = CM222_6;
  M[22][19] = CM222_19;
  M[22][20] = CM222_20;
  M[22][21] = CM222_21;
  M[22][22] = CM222_22;
  M[22][23] = CM123_22;
  M[22][24] = CM224_22;
  M[22][25] = CM325_22;
  M[22][26] = CM226_22;
  M[22][27] = CM327_22;
  M[22][28] = CM128_22;
  M[23][1] = CM123_1;
  M[23][2] = CM123_2;
  M[23][3] = CM123_3;
  M[23][4] = CM123_4;
  M[23][5] = CM123_5;
  M[23][6] = CM123_6;
  M[23][19] = CM123_19;
  M[23][20] = CM123_20;
  M[23][21] = CM123_21;
  M[23][22] = CM123_22;
  M[23][23] = CM123_23;
  M[23][24] = CM224_23;
  M[23][25] = CM325_23;
  M[23][26] = CM226_23;
  M[23][27] = CM327_23;
  M[23][28] = CM128_23;
  M[24][1] = CM224_1;
  M[24][2] = CM224_2;
  M[24][3] = CM224_3;
  M[24][4] = CM224_4;
  M[24][5] = CM224_5;
  M[24][6] = CM224_6;
  M[24][19] = CM224_19;
  M[24][20] = CM224_20;
  M[24][21] = CM224_21;
  M[24][22] = CM224_22;
  M[24][23] = CM224_23;
  M[24][24] = CM224_24;
  M[24][25] = CM325_24;
  M[24][26] = CM226_24;
  M[24][27] = CM327_24;
  M[24][28] = CM128_24;
  M[25][1] = CM325_1;
  M[25][2] = CM325_2;
  M[25][3] = CM325_3;
  M[25][4] = CM325_4;
  M[25][5] = CM325_5;
  M[25][6] = CM325_6;
  M[25][19] = CM325_19;
  M[25][20] = CM325_20;
  M[25][21] = CM325_21;
  M[25][22] = CM325_22;
  M[25][23] = CM325_23;
  M[25][24] = CM325_24;
  M[25][25] = CM325_25;
  M[25][26] = CM2625_227;
  M[25][27] = CM327_25;
  M[25][28] = CM128_25;
  M[26][4] = CM226_4;
  M[26][5] = CM226_5;
  M[26][6] = CM226_6;
  M[26][19] = CM226_19;
  M[26][20] = CM226_20;
  M[26][21] = CM226_21;
  M[26][22] = CM226_22;
  M[26][23] = CM226_23;
  M[26][24] = CM226_24;
  M[26][25] = CM2625_227;
  M[26][26] = CM226_26;
  M[26][27] = CM2726_328;
  M[26][28] = CM128_26;
  M[27][4] = CM327_4;
  M[27][5] = CM327_5;
  M[27][6] = CM327_6;
  M[27][19] = CM327_19;
  M[27][20] = CM327_20;
  M[27][21] = CM327_21;
  M[27][22] = CM327_22;
  M[27][23] = CM327_23;
  M[27][24] = CM327_24;
  M[27][25] = CM327_25;
  M[27][26] = CM2726_328;
  M[27][27] = CM327_27;
  M[28][4] = CM128_4;
  M[28][5] = CM128_5;
  M[28][6] = CM128_6;
  M[28][19] = CM128_19;
  M[28][20] = CM128_20;
  M[28][21] = CM128_21;
  M[28][22] = CM128_22;
  M[28][23] = CM128_23;
  M[28][24] = CM128_24;
  M[28][25] = CM128_25;
  M[28][26] = CM128_26;
  M[28][28] = s->In[1][28];
  M[29][1] = CM229_1;
  M[29][2] = CM229_2;
  M[29][3] = CM229_3;
  M[29][4] = CM229_4;
  M[29][5] = CM229_5;
  M[29][6] = CM229_6;
  M[29][19] = CM229_19;
  M[29][20] = CM229_20;
  M[29][21] = CM229_21;
  M[29][29] = CM229_29;
  M[29][30] = CM130_29;
  M[29][31] = CM231_29;
  M[29][32] = CM332_29;
  M[29][33] = CM233_29;
  M[29][34] = CM334_29;
  M[29][35] = CM135_29;
  M[30][1] = CM130_1;
  M[30][2] = CM130_2;
  M[30][3] = CM130_3;
  M[30][4] = CM130_4;
  M[30][5] = CM130_5;
  M[30][6] = CM130_6;
  M[30][19] = CM130_19;
  M[30][20] = CM130_20;
  M[30][21] = CM130_21;
  M[30][29] = CM130_29;
  M[30][30] = CM130_30;
  M[30][31] = CM231_30;
  M[30][32] = CM332_30;
  M[30][33] = CM233_30;
  M[30][34] = CM334_30;
  M[30][35] = CM135_30;
  M[31][1] = CM231_1;
  M[31][2] = CM231_2;
  M[31][3] = CM231_3;
  M[31][4] = CM231_4;
  M[31][5] = CM231_5;
  M[31][6] = CM231_6;
  M[31][19] = CM231_19;
  M[31][20] = CM231_20;
  M[31][21] = CM231_21;
  M[31][29] = CM231_29;
  M[31][30] = CM231_30;
  M[31][31] = CM231_31;
  M[31][32] = CM3231_333;
  M[31][33] = CM233_31;
  M[31][34] = CM334_31;
  M[31][35] = CM135_31;
  M[32][1] = CM332_1;
  M[32][2] = CM332_2;
  M[32][3] = CM332_3;
  M[32][4] = CM332_4;
  M[32][5] = CM332_5;
  M[32][6] = CM332_6;
  M[32][19] = CM332_19;
  M[32][20] = CM332_20;
  M[32][21] = CM332_21;
  M[32][29] = CM332_29;
  M[32][30] = CM332_30;
  M[32][31] = CM3231_333;
  M[32][32] = CM332_32;
  M[32][33] = CM3332_234;
  M[32][34] = CM334_32;
  M[32][35] = CM135_32;
  M[33][4] = CM233_4;
  M[33][5] = CM233_5;
  M[33][6] = CM233_6;
  M[33][19] = CM233_19;
  M[33][20] = CM233_20;
  M[33][21] = CM233_21;
  M[33][29] = CM233_29;
  M[33][30] = CM233_30;
  M[33][31] = CM233_31;
  M[33][32] = CM3332_234;
  M[33][33] = CM233_33;
  M[33][34] = CM3433_335;
  M[33][35] = CM135_33;
  M[34][4] = CM334_4;
  M[34][5] = CM334_5;
  M[34][6] = CM334_6;
  M[34][19] = CM334_19;
  M[34][20] = CM334_20;
  M[34][21] = CM334_21;
  M[34][29] = CM334_29;
  M[34][30] = CM334_30;
  M[34][31] = CM334_31;
  M[34][32] = CM334_32;
  M[34][33] = CM3433_335;
  M[34][34] = CM334_34;
  M[35][4] = CM135_4;
  M[35][5] = CM135_5;
  M[35][6] = CM135_6;
  M[35][19] = CM135_19;
  M[35][20] = CM135_20;
  M[35][21] = CM135_21;
  M[35][29] = CM135_29;
  M[35][30] = CM135_30;
  M[35][31] = CM135_31;
  M[35][32] = CM135_32;
  M[35][33] = CM135_33;
  M[35][35] = s->In[1][35];
  c[1] = FF4_15;
  c[2] = FF3_24;
  c[3] = FF3_34;
  c[4] = CF4_15;
  c[5] = CF5_26;
  c[6] = CF36;
  c[7] = CF27;
  c[8] = CF18;
  c[9] = CF39;
  c[10] = CF210;
  c[11] = CF111;
  c[12] = CF212;
  c[13] = CF213;
  c[14] = CF114;
  c[15] = CF315;
  c[16] = CF216;
  c[17] = CF117;
  c[18] = CF218;
  c[19] = CF119;
  c[20] = CF220;
  c[21] = CF321;
  c[22] = CF222;
  c[23] = CF123;
  c[24] = CF224;
  c[25] = CF325;
  c[26] = CF226;
  c[27] = CF327;
  c[28] = CF128;
  c[29] = CF229;
  c[30] = CF130;
  c[31] = CF231;
  c[32] = CF332;
  c[33] = CF233;
  c[34] = CF334;
  c[35] = CF135;

// ====== END Task 0 ====== 


}
 

