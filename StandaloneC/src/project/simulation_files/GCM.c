//---------------------------
// Nicolas Van der Noot & Allan Barrea
//
// Creation : 06/03/2013
// Last update : 08/07/2014

// Initialize the GCM (Ground Contact Model)
//
//---------------------------

#include "simu_def.h"


// The force sensor of each foot is placed on the ground, so z = 0.0 will
// produce a flat foot at the right distance from the ankle joint.

// z = f(x,y) for the left foot
double z_left_foot(double x, double y)
{
    return 0.0;
}

// z = f(x,y) for the right foot
double z_right_foot(double x, double y)
{
    return 0.0;
}

// height of the ground
double get_ground_height(double x, double y, double tsim, MBSdataStruct *MBSdata)
{
    return 0.0;
}

/*
 * Compliant feet (two rigid plates connected by a spring)
 */
#if defined(COMP_FEET)

// Initializes Mohamad's GCM with compliant feet
void init_GCM(MBSdataStruct *MBSdata)
{
    int i,j,k;
    double mesh_resolution_x, mesh_resolution_y;
    double X1_prox,X2_prox,Y1_prox,Y2_prox;
    double X1_dist,X2_dist,Y1_dist,Y2_dist;
    int m_prox,n_prox,Msize_prox;
    int m_dist,n_dist,Msize_dist;
    double x_prox,y_prox;
    double x_dist,y_dist;

    // A mesh grid of contact points under the sole of the robot should be  
    // defined. Contact surface can have a 3D shape and is not necessarily a flat  
    // surface. The mesh grid can be finer in more critical areas of the sole, which
    // have more contact with the ground.
 
    // WARNING ! the values here (mesh_resolution_x/y and also X1...Y2) must be coherent with
    // what is written in simu_variables.m/.txt !

    mesh_resolution_x = 0.01; // [m]
    mesh_resolution_y = 0.01; // [m]

    // [X1 X2] and [Y1 Y2] introduce possible contact range of foot
    // WARNING ! the mesh should be "exact" w.r.t. the mesh resolution !
    X1_prox =-0.06;
    X2_prox = 0.08;
    Y1_prox =-0.045;
    Y2_prox = 0.045;

    X1_dist = 0.0;
    X2_dist = 0.05;
    Y1_dist =-0.045;
    Y2_dist = 0.045;
 
    // x, y and z contains position coordinates of all grid points which can be 
    // in contact with the ground. These points can be placed in arbitrary positions 
    // depending on the application.
    // Mesh size is number of all points under each foot.
    m_prox     = (int)((X2_prox-X1_prox)/mesh_resolution_x)+1;
    n_prox     = (int)((Y2_prox-Y1_prox)/mesh_resolution_y)+1;
    Msize_prox = m_prox*n_prox;

    MBSdata->user_IO->Msize_GCM_prox = Msize_prox;

    m_dist     = (int)((X2_dist-X1_dist)/mesh_resolution_x)+1;
    n_dist     = (int)((Y2_dist-Y1_dist)/mesh_resolution_y)+1;
    Msize_dist = m_dist*n_dist;

    MBSdata->user_IO->Msize_GCM_dist = Msize_dist;

    // rn is mesh points in body coordinate system. rn shows the shape of the sole
    for(i=1; i<=m_prox; i++)
    {
        for(j=1; j<=n_prox; j++)
        {
            x_prox = X1_prox + mesh_resolution_x*(i-1);
            y_prox = Y1_prox + mesh_resolution_y*(j-1);

            k = j+(i-1)*n_prox;

            MBSdata->user_IO->rn_left_prox_x[k] = x_prox;
            MBSdata->user_IO->rn_right_prox_x[k] = x_prox;

            MBSdata->user_IO->rn_left_prox_y[k] = y_prox;
            MBSdata->user_IO->rn_right_prox_y[k] = y_prox;

            MBSdata->user_IO->rn_left_prox_z[k] = z_left_foot(x_prox, y_prox);
            MBSdata->user_IO->rn_right_prox_z[k] = z_right_foot(x_prox, y_prox);
        }
    }

    for(i=1; i<=m_dist; i++)
    {
        for(j=1; j<=n_dist; j++)
        {
            x_dist = X1_dist + mesh_resolution_x*(i-1);
            y_dist = Y1_dist + mesh_resolution_y*(j-1);

            k = j+(i-1)*n_dist;

            MBSdata->user_IO->rn_left_dist_x[k]  = x_dist;
            MBSdata->user_IO->rn_right_dist_x[k] = x_dist;

            MBSdata->user_IO->rn_left_dist_y[k]  = y_dist;
            MBSdata->user_IO->rn_right_dist_y[k] = y_dist;

            MBSdata->user_IO->rn_left_dist_z[k]  = z_left_foot(x_dist, y_dist);
            MBSdata->user_IO->rn_right_dist_z[k] = z_right_foot(x_dist, y_dist);
        }
    }
}

/*
 * Short feet (a single 14 cm-long plate)
 */
#elif defined(SHORT_FEET)

// Initializes Mohamad's GCM with short feet from Luca
void init_GCM(MBSdataStruct *MBSdata)
{
    int i,j,k;
    double mesh_resolution_x, mesh_resolution_y;
    double X1_prox,X2_prox,Y1_prox,Y2_prox;
    int m_prox,n_prox,Msize_prox;
    double x_prox,y_prox;

    // A mesh grid of contact points under the sole of the robot should be  
    // defined. Contact surface can have a 3D shape and is not necessarily a flat  
    // surface. The mesh grid can be finer in more critical areas of the sole, which
    // have more contact with the ground.
 
    // WARNING ! the values here (mesh_resolution_x/y and also X1...Y2) must be coherent with
    // what is written in simu_variables.m/.txt !

    mesh_resolution_x = 0.01; // [m]
    mesh_resolution_y = 0.01; // [m]

    // [X1 X2] and [Y1 Y2] introduce possible contact range of foot
    // WARNING ! the mesh should be "exact" w.r.t. the mesh resolution !
    X1_prox =-0.06;
    X2_prox = 0.08;
    Y1_prox =-0.045;
    Y2_prox = 0.045;

 
    // x, y and z contains position coordinates of all grid points which can be 
    // in contact with the ground. These points can be placed in arbitrary positions 
    // depending on the application.
    // Mesh size is number of all points under each foot.
    m_prox     = (int)((X2_prox-X1_prox)/mesh_resolution_x)+1;
    n_prox     = (int)((Y2_prox-Y1_prox)/mesh_resolution_y)+1;
    Msize_prox = m_prox*n_prox;

    MBSdata->user_IO->Msize_GCM_prox = Msize_prox;

    // rn is mesh points in body coordinate system. rn shows the shape of the sole
    for(i=1; i<=m_prox; i++)
    {
        for(j=1; j<=n_prox; j++)
        {
            x_prox = X1_prox + mesh_resolution_x*(i-1);
            y_prox = Y1_prox + mesh_resolution_y*(j-1);

            k = j+(i-1)*n_prox;

            MBSdata->user_IO->rn_left_prox_x[k]  = x_prox;
            MBSdata->user_IO->rn_right_prox_x[k] = x_prox;

            MBSdata->user_IO->rn_left_prox_y[k]  = y_prox;
            MBSdata->user_IO->rn_right_prox_y[k] = y_prox;

            MBSdata->user_IO->rn_left_prox_z[k]  = z_left_foot(x_prox, y_prox);
            MBSdata->user_IO->rn_right_prox_z[k] = z_right_foot(x_prox, y_prox);
        }
    }
}

/*
 * Initial feet (rigid 19 cm-long, rounded)
 */
#else

// Initializes Mohamad's GCM with real CoMan shape
void init_GCM(MBSdataStruct *MBSdata)
{
    // user variables
    UserIOStruct *uvs;
    
	int i;
    
    uvs = MBSdata->user_IO;
		
	// Nb of points under each foot
	uvs->Msize_GCM = 200;

	uvs->rn_left_x[1]  = 0.087379;
    uvs->rn_left_x[2]  = 0.082;
    uvs->rn_left_x[3]  = 0.07769;
    uvs->rn_left_x[4]  = 0.068;
    uvs->rn_left_x[5]  = -0.028039;
    uvs->rn_left_x[6]  = -0.034442;
    uvs->rn_left_x[7]  = -0.040934;
    uvs->rn_left_x[8]  = -0.046234;
    uvs->rn_left_x[9]  = -0.05;
    uvs->rn_left_x[10]  = -0.05498;
    uvs->rn_left_x[11]  = -0.05749;
    uvs->rn_left_x[12]  = -0.06;
    uvs->rn_left_x[13]  = -0.06;
    uvs->rn_left_x[14]  = -0.057892;
    uvs->rn_left_x[15]  = -0.055896;
    uvs->rn_left_x[16]  = -0.052973;
    uvs->rn_left_x[17]  = -0.046158;
    uvs->rn_left_x[18]  = -0.040599;
    uvs->rn_left_x[19]  = -0.035039;
    uvs->rn_left_x[20]  = 0.1055;
    uvs->rn_left_x[21]  = 0.11188;
    uvs->rn_left_x[22]  = 0.11826;
    uvs->rn_left_x[23]  = 0.12195;
    uvs->rn_left_x[24]  = 0.12453;
    uvs->rn_left_x[25]  = 0.12716;
    uvs->rn_left_x[26]  = 0.12882;
    uvs->rn_left_x[27]  = 0.12986;
    uvs->rn_left_x[28]  = 0.12986;
    uvs->rn_left_x[29]  = 0.1283;
    uvs->rn_left_x[30]  = 0.12686;
    uvs->rn_left_x[31]  = 0.1241;
    uvs->rn_left_x[32]  = 0.12133;
    uvs->rn_left_x[33]  = 0.11855;
    uvs->rn_left_x[34]  = 0.11313;
    uvs->rn_left_x[35]  = 0.10808;
    uvs->rn_left_x[36]  = 0.10302;
    uvs->rn_left_x[37]  = 0.095199;
    uvs->rn_left_x[38]  = 0.087379;
    uvs->rn_left_x[39]  = -0.06;
    uvs->rn_left_x[40]  = -0.06;
    uvs->rn_left_x[41]  = -0.06;
    uvs->rn_left_x[42]  = -0.06;
    uvs->rn_left_x[43]  = -0.050007;
    uvs->rn_left_x[44]  = -0.050007;
    uvs->rn_left_x[45]  = -0.050007;
    uvs->rn_left_x[46]  = -0.050007;
    uvs->rn_left_x[47]  = -0.050007;
    uvs->rn_left_x[48]  = -0.050007;
    uvs->rn_left_x[49]  = -0.040015;
    uvs->rn_left_x[50]  = -0.040015;
    uvs->rn_left_x[51]  = -0.040015;
    uvs->rn_left_x[52]  = -0.040015;
    uvs->rn_left_x[53]  = -0.040015;
    uvs->rn_left_x[54]  = -0.040015;
    uvs->rn_left_x[55]  = -0.040015;
    uvs->rn_left_x[56]  = -0.040015;
    uvs->rn_left_x[57]  = -0.030022;
    uvs->rn_left_x[58]  = -0.030022;
    uvs->rn_left_x[59]  = -0.030022;
    uvs->rn_left_x[60]  = -0.030022;
    uvs->rn_left_x[61]  = -0.030022;
    uvs->rn_left_x[62]  = -0.030022;
    uvs->rn_left_x[63]  = -0.030022;
    uvs->rn_left_x[64]  = -0.030022;
    uvs->rn_left_x[65]  = -0.030022;
    uvs->rn_left_x[66]  = -0.020029;
    uvs->rn_left_x[67]  = -0.020029;
    uvs->rn_left_x[68]  = -0.020029;
    uvs->rn_left_x[69]  = -0.020029;
    uvs->rn_left_x[70]  = -0.020029;
    uvs->rn_left_x[71]  = -0.020029;
    uvs->rn_left_x[72]  = -0.020029;
    uvs->rn_left_x[73]  = -0.020029;
    uvs->rn_left_x[74]  = -0.020029;
    uvs->rn_left_x[75]  = -0.020029;
    uvs->rn_left_x[76]  = -0.010037;
    uvs->rn_left_x[77]  = -0.010037;
    uvs->rn_left_x[78]  = -0.010037;
    uvs->rn_left_x[79]  = -0.010037;
    uvs->rn_left_x[80]  = -0.010037;
    uvs->rn_left_x[81]  = -0.010037;
    uvs->rn_left_x[82]  = -0.010037;
    uvs->rn_left_x[83]  = -0.010037;
    uvs->rn_left_x[84]  = -0.010037;
    uvs->rn_left_x[85]  = -0.010037;
    uvs->rn_left_x[86]  = -4.4211e-05;
    uvs->rn_left_x[87]  = -4.4211e-05;
    uvs->rn_left_x[88]  = -4.4211e-05;
    uvs->rn_left_x[89]  = -4.4211e-05;
    uvs->rn_left_x[90]  = -4.4211e-05;
    uvs->rn_left_x[91]  = -4.4211e-05;
    uvs->rn_left_x[92]  = -4.4211e-05;
    uvs->rn_left_x[93]  = -4.4211e-05;
    uvs->rn_left_x[94]  = -4.4211e-05;
    uvs->rn_left_x[95]  = -4.4211e-05;
    uvs->rn_left_x[96]  = 0.0099484;
    uvs->rn_left_x[97]  = 0.0099484;
    uvs->rn_left_x[98]  = 0.0099484;
    uvs->rn_left_x[99]  = 0.0099484;
    uvs->rn_left_x[100]  = 0.0099484;
    uvs->rn_left_x[101]  = 0.0099484;
    uvs->rn_left_x[102]  = 0.0099484;
    uvs->rn_left_x[103]  = 0.0099484;
    uvs->rn_left_x[104]  = 0.0099484;
    uvs->rn_left_x[105]  = 0.0099484;
    uvs->rn_left_x[106]  = 0.019941;
    uvs->rn_left_x[107]  = 0.019941;
    uvs->rn_left_x[108]  = 0.019941;
    uvs->rn_left_x[109]  = 0.019941;
    uvs->rn_left_x[110]  = 0.019941;
    uvs->rn_left_x[111]  = 0.019941;
    uvs->rn_left_x[112]  = 0.019941;
    uvs->rn_left_x[113]  = 0.019941;
    uvs->rn_left_x[114]  = 0.019941;
    uvs->rn_left_x[115]  = 0.019941;
    uvs->rn_left_x[116]  = 0.029934;
    uvs->rn_left_x[117]  = 0.029934;
    uvs->rn_left_x[118]  = 0.029934;
    uvs->rn_left_x[119]  = 0.029934;
    uvs->rn_left_x[120]  = 0.029934;
    uvs->rn_left_x[121]  = 0.029934;
    uvs->rn_left_x[122]  = 0.029934;
    uvs->rn_left_x[123]  = 0.029934;
    uvs->rn_left_x[124]  = 0.029934;
    uvs->rn_left_x[125]  = 0.029934;
    uvs->rn_left_x[126]  = 0.039926;
    uvs->rn_left_x[127]  = 0.039926;
    uvs->rn_left_x[128]  = 0.039926;
    uvs->rn_left_x[129]  = 0.039926;
    uvs->rn_left_x[130]  = 0.039926;
    uvs->rn_left_x[131]  = 0.039926;
    uvs->rn_left_x[132]  = 0.039926;
    uvs->rn_left_x[133]  = 0.039926;
    uvs->rn_left_x[134]  = 0.039926;
    uvs->rn_left_x[135]  = 0.039926;
    uvs->rn_left_x[136]  = 0.049919;
    uvs->rn_left_x[137]  = 0.049919;
    uvs->rn_left_x[138]  = 0.049919;
    uvs->rn_left_x[139]  = 0.049919;
    uvs->rn_left_x[140]  = 0.049919;
    uvs->rn_left_x[141]  = 0.049919;
    uvs->rn_left_x[142]  = 0.049919;
    uvs->rn_left_x[143]  = 0.049919;
    uvs->rn_left_x[144]  = 0.049919;
    uvs->rn_left_x[145]  = 0.049919;
    uvs->rn_left_x[146]  = 0.059912;
    uvs->rn_left_x[147]  = 0.059912;
    uvs->rn_left_x[148]  = 0.059912;
    uvs->rn_left_x[149]  = 0.059912;
    uvs->rn_left_x[150]  = 0.059912;
    uvs->rn_left_x[151]  = 0.059912;
    uvs->rn_left_x[152]  = 0.059912;
    uvs->rn_left_x[153]  = 0.059912;
    uvs->rn_left_x[154]  = 0.059912;
    uvs->rn_left_x[155]  = 0.059912;
    uvs->rn_left_x[156]  = 0.069904;
    uvs->rn_left_x[157]  = 0.069904;
    uvs->rn_left_x[158]  = 0.069904;
    uvs->rn_left_x[159]  = 0.069904;
    uvs->rn_left_x[160]  = 0.069904;
    uvs->rn_left_x[161]  = 0.069904;
    uvs->rn_left_x[162]  = 0.069904;
    uvs->rn_left_x[163]  = 0.069904;
    uvs->rn_left_x[164]  = 0.069904;
    uvs->rn_left_x[165]  = 0.079897;
    uvs->rn_left_x[166]  = 0.079897;
    uvs->rn_left_x[167]  = 0.079897;
    uvs->rn_left_x[168]  = 0.079897;
    uvs->rn_left_x[169]  = 0.079897;
    uvs->rn_left_x[170]  = 0.079897;
    uvs->rn_left_x[171]  = 0.079897;
    uvs->rn_left_x[172]  = 0.079897;
    uvs->rn_left_x[173]  = 0.079897;
    uvs->rn_left_x[174]  = 0.089889;
    uvs->rn_left_x[175]  = 0.089889;
    uvs->rn_left_x[176]  = 0.089889;
    uvs->rn_left_x[177]  = 0.089889;
    uvs->rn_left_x[178]  = 0.089889;
    uvs->rn_left_x[179]  = 0.089889;
    uvs->rn_left_x[180]  = 0.089889;
    uvs->rn_left_x[181]  = 0.089889;
    uvs->rn_left_x[182]  = 0.099882;
    uvs->rn_left_x[183]  = 0.099882;
    uvs->rn_left_x[184]  = 0.099882;
    uvs->rn_left_x[185]  = 0.099882;
    uvs->rn_left_x[186]  = 0.099882;
    uvs->rn_left_x[187]  = 0.099882;
    uvs->rn_left_x[188]  = 0.099882;
    uvs->rn_left_x[189]  = 0.099882;
    uvs->rn_left_x[190]  = 0.10987;
    uvs->rn_left_x[191]  = 0.10987;
    uvs->rn_left_x[192]  = 0.10987;
    uvs->rn_left_x[193]  = 0.10987;
    uvs->rn_left_x[194]  = 0.10987;
    uvs->rn_left_x[195]  = 0.10987;
    uvs->rn_left_x[196]  = 0.11987;
    uvs->rn_left_x[197]  = 0.11987;
    uvs->rn_left_x[198]  = 0.11987;
    uvs->rn_left_x[199]  = 0.11987;
    uvs->rn_left_x[200]  = 0.12986;

    uvs->rn_right_x[1] = 0.087379;
    uvs->rn_right_x[2] = 0.082;
    uvs->rn_right_x[3] = 0.07769;
    uvs->rn_right_x[4] = 0.068;
    uvs->rn_right_x[5] = -0.028039;
    uvs->rn_right_x[6] = -0.034442;
    uvs->rn_right_x[7] = -0.040934;
    uvs->rn_right_x[8] = -0.046234;
    uvs->rn_right_x[9] = -0.05;
    uvs->rn_right_x[10] = -0.05498;
    uvs->rn_right_x[11] = -0.05749;
    uvs->rn_right_x[12] = -0.06;
    uvs->rn_right_x[13] = -0.06;
    uvs->rn_right_x[14] = -0.057892;
    uvs->rn_right_x[15] = -0.055896;
    uvs->rn_right_x[16] = -0.052973;
    uvs->rn_right_x[17] = -0.046158;
    uvs->rn_right_x[18] = -0.040599;
    uvs->rn_right_x[19] = -0.035039;
    uvs->rn_right_x[20] = 0.1055;
    uvs->rn_right_x[21] = 0.11188;
    uvs->rn_right_x[22] = 0.11826;
    uvs->rn_right_x[23] = 0.12195;
    uvs->rn_right_x[24] = 0.12453;
    uvs->rn_right_x[25] = 0.12716;
    uvs->rn_right_x[26] = 0.12882;
    uvs->rn_right_x[27] = 0.12986;
    uvs->rn_right_x[28] = 0.12986;
    uvs->rn_right_x[29] = 0.1283;
    uvs->rn_right_x[30] = 0.12686;
    uvs->rn_right_x[31] = 0.1241;
    uvs->rn_right_x[32] = 0.12133;
    uvs->rn_right_x[33] = 0.11855;
    uvs->rn_right_x[34] = 0.11313;
    uvs->rn_right_x[35] = 0.10808;
    uvs->rn_right_x[36] = 0.10302;
    uvs->rn_right_x[37] = 0.095199;
    uvs->rn_right_x[38] = 0.087379;
    uvs->rn_right_x[39] = -0.06;
    uvs->rn_right_x[40] = -0.06;
    uvs->rn_right_x[41] = -0.06;
    uvs->rn_right_x[42] = -0.06;
    uvs->rn_right_x[43] = -0.050007;
    uvs->rn_right_x[44] = -0.050007;
    uvs->rn_right_x[45] = -0.050007;
    uvs->rn_right_x[46] = -0.050007;
    uvs->rn_right_x[47] = -0.050007;
    uvs->rn_right_x[48] = -0.050007;
    uvs->rn_right_x[49] = -0.040015;
    uvs->rn_right_x[50] = -0.040015;
    uvs->rn_right_x[51] = -0.040015;
    uvs->rn_right_x[52] = -0.040015;
    uvs->rn_right_x[53] = -0.040015;
    uvs->rn_right_x[54] = -0.040015;
    uvs->rn_right_x[55] = -0.040015;
    uvs->rn_right_x[56] = -0.040015;
    uvs->rn_right_x[57] = -0.030022;
    uvs->rn_right_x[58] = -0.030022;
    uvs->rn_right_x[59] = -0.030022;
    uvs->rn_right_x[60] = -0.030022;
    uvs->rn_right_x[61] = -0.030022;
    uvs->rn_right_x[62] = -0.030022;
    uvs->rn_right_x[63] = -0.030022;
    uvs->rn_right_x[64] = -0.030022;
    uvs->rn_right_x[65] = -0.030022;
    uvs->rn_right_x[66] = -0.020029;
    uvs->rn_right_x[67] = -0.020029;
    uvs->rn_right_x[68] = -0.020029;
    uvs->rn_right_x[69] = -0.020029;
    uvs->rn_right_x[70] = -0.020029;
    uvs->rn_right_x[71] = -0.020029;
    uvs->rn_right_x[72] = -0.020029;
    uvs->rn_right_x[73] = -0.020029;
    uvs->rn_right_x[74] = -0.020029;
    uvs->rn_right_x[75] = -0.020029;
    uvs->rn_right_x[76] = -0.010037;
    uvs->rn_right_x[77] = -0.010037;
    uvs->rn_right_x[78] = -0.010037;
    uvs->rn_right_x[79] = -0.010037;
    uvs->rn_right_x[80] = -0.010037;
    uvs->rn_right_x[81] = -0.010037;
    uvs->rn_right_x[82] = -0.010037;
    uvs->rn_right_x[83] = -0.010037;
    uvs->rn_right_x[84] = -0.010037;
    uvs->rn_right_x[85] = -0.010037;
    uvs->rn_right_x[86] = -4.4211e-05;
    uvs->rn_right_x[87] = -4.4211e-05;
    uvs->rn_right_x[88] = -4.4211e-05;
    uvs->rn_right_x[89] = -4.4211e-05;
    uvs->rn_right_x[90] = -4.4211e-05;
    uvs->rn_right_x[91] = -4.4211e-05;
    uvs->rn_right_x[92] = -4.4211e-05;
    uvs->rn_right_x[93] = -4.4211e-05;
    uvs->rn_right_x[94] = -4.4211e-05;
    uvs->rn_right_x[95] = -4.4211e-05;
    uvs->rn_right_x[96] = 0.0099484;
    uvs->rn_right_x[97] = 0.0099484;
    uvs->rn_right_x[98] = 0.0099484;
    uvs->rn_right_x[99] = 0.0099484;
    uvs->rn_right_x[100] = 0.0099484;
    uvs->rn_right_x[101] = 0.0099484;
    uvs->rn_right_x[102] = 0.0099484;
    uvs->rn_right_x[103] = 0.0099484;
    uvs->rn_right_x[104] = 0.0099484;
    uvs->rn_right_x[105] = 0.0099484;
    uvs->rn_right_x[106] = 0.019941;
    uvs->rn_right_x[107] = 0.019941;
    uvs->rn_right_x[108] = 0.019941;
    uvs->rn_right_x[109] = 0.019941;
    uvs->rn_right_x[110] = 0.019941;
    uvs->rn_right_x[111] = 0.019941;
    uvs->rn_right_x[112] = 0.019941;
    uvs->rn_right_x[113] = 0.019941;
    uvs->rn_right_x[114] = 0.019941;
    uvs->rn_right_x[115] = 0.019941;
    uvs->rn_right_x[116] = 0.029934;
    uvs->rn_right_x[117] = 0.029934;
    uvs->rn_right_x[118] = 0.029934;
    uvs->rn_right_x[119] = 0.029934;
    uvs->rn_right_x[120] = 0.029934;
    uvs->rn_right_x[121] = 0.029934;
    uvs->rn_right_x[122] = 0.029934;
    uvs->rn_right_x[123] = 0.029934;
    uvs->rn_right_x[124] = 0.029934;
    uvs->rn_right_x[125] = 0.029934;
    uvs->rn_right_x[126] = 0.039926;
    uvs->rn_right_x[127] = 0.039926;
    uvs->rn_right_x[128] = 0.039926;
    uvs->rn_right_x[129] = 0.039926;
    uvs->rn_right_x[130] = 0.039926;
    uvs->rn_right_x[131] = 0.039926;
    uvs->rn_right_x[132] = 0.039926;
    uvs->rn_right_x[133] = 0.039926;
    uvs->rn_right_x[134] = 0.039926;
    uvs->rn_right_x[135] = 0.039926;
    uvs->rn_right_x[136] = 0.049919;
    uvs->rn_right_x[137] = 0.049919;
    uvs->rn_right_x[138] = 0.049919;
    uvs->rn_right_x[139] = 0.049919;
    uvs->rn_right_x[140] = 0.049919;
    uvs->rn_right_x[141] = 0.049919;
    uvs->rn_right_x[142] = 0.049919;
    uvs->rn_right_x[143] = 0.049919;
    uvs->rn_right_x[144] = 0.049919;
    uvs->rn_right_x[145] = 0.049919;
    uvs->rn_right_x[146] = 0.059912;
    uvs->rn_right_x[147] = 0.059912;
    uvs->rn_right_x[148] = 0.059912;
    uvs->rn_right_x[149] = 0.059912;
    uvs->rn_right_x[150] = 0.059912;
    uvs->rn_right_x[151] = 0.059912;
    uvs->rn_right_x[152] = 0.059912;
    uvs->rn_right_x[153] = 0.059912;
    uvs->rn_right_x[154] = 0.059912;
    uvs->rn_right_x[155] = 0.059912;
    uvs->rn_right_x[156] = 0.069904;
    uvs->rn_right_x[157] = 0.069904;
    uvs->rn_right_x[158] = 0.069904;
    uvs->rn_right_x[159] = 0.069904;
    uvs->rn_right_x[160] = 0.069904;
    uvs->rn_right_x[161] = 0.069904;
    uvs->rn_right_x[162] = 0.069904;
    uvs->rn_right_x[163] = 0.069904;
    uvs->rn_right_x[164] = 0.069904;
    uvs->rn_right_x[165] = 0.079897;
    uvs->rn_right_x[166] = 0.079897;
    uvs->rn_right_x[167] = 0.079897;
    uvs->rn_right_x[168] = 0.079897;
    uvs->rn_right_x[169] = 0.079897;
    uvs->rn_right_x[170] = 0.079897;
    uvs->rn_right_x[171] = 0.079897;
    uvs->rn_right_x[172] = 0.079897;
    uvs->rn_right_x[173] = 0.079897;
    uvs->rn_right_x[174] = 0.089889;
    uvs->rn_right_x[175] = 0.089889;
    uvs->rn_right_x[176] = 0.089889;
    uvs->rn_right_x[177] = 0.089889;
    uvs->rn_right_x[178] = 0.089889;
    uvs->rn_right_x[179] = 0.089889;
    uvs->rn_right_x[180] = 0.089889;
    uvs->rn_right_x[181] = 0.089889;
    uvs->rn_right_x[182] = 0.099882;
    uvs->rn_right_x[183] = 0.099882;
    uvs->rn_right_x[184] = 0.099882;
    uvs->rn_right_x[185] = 0.099882;
    uvs->rn_right_x[186] = 0.099882;
    uvs->rn_right_x[187] = 0.099882;
    uvs->rn_right_x[188] = 0.099882;
    uvs->rn_right_x[189] = 0.099882;
    uvs->rn_right_x[190] = 0.10987;
    uvs->rn_right_x[191] = 0.10987;
    uvs->rn_right_x[192] = 0.10987;
    uvs->rn_right_x[193] = 0.10987;
    uvs->rn_right_x[194] = 0.10987;
    uvs->rn_right_x[195] = 0.10987;
    uvs->rn_right_x[196] = 0.11987;
    uvs->rn_right_x[197] = 0.11987;
    uvs->rn_right_x[198] = 0.11987;
    uvs->rn_right_x[199] = 0.11987;
    uvs->rn_right_x[200] = 0.12986;

    uvs->rn_left_y[1] = 0.039554;
    uvs->rn_left_y[2] = 0.04177;
    uvs->rn_left_y[3] = 0.043;
    uvs->rn_left_y[4] = 0.045;
    uvs->rn_left_y[5] = 0.045;
    uvs->rn_left_y[6] = 0.044334;
    uvs->rn_left_y[7] = 0.043354;
    uvs->rn_left_y[8] = 0.041164;
    uvs->rn_left_y[9] = 0.0387;
    uvs->rn_left_y[10] = 0.032816;
    uvs->rn_left_y[11] = 0.0289;
    uvs->rn_left_y[12] = 0.022984;
    uvs->rn_left_y[13] = -0.019198;
    uvs->rn_left_y[14] = -0.025738;
    uvs->rn_left_y[15] = -0.030759;
    uvs->rn_left_y[16] = -0.035204;
    uvs->rn_left_y[17] = -0.040847;
    uvs->rn_left_y[18] = -0.04319;
    uvs->rn_left_y[19] = -0.045;
    uvs->rn_left_y[20] = -0.045;
    uvs->rn_left_y[21] = -0.0436;
    uvs->rn_left_y[22] = -0.040817;
    uvs->rn_left_y[23] = -0.038052;
    uvs->rn_left_y[24] = -0.034944;
    uvs->rn_left_y[25] = -0.030833;
    uvs->rn_left_y[26] = -0.025986;
    uvs->rn_left_y[27] = -0.020581;
    uvs->rn_left_y[28] = -0.00709;
    uvs->rn_left_y[29] = -0.001312;
    uvs->rn_left_y[30] = 0.004467;
    uvs->rn_left_y[31] = 0.01004;
    uvs->rn_left_y[32] = 0.015613;
    uvs->rn_left_y[33] = 0.0193;
    uvs->rn_left_y[34] = 0.024782;
    uvs->rn_left_y[35] = 0.02849;
    uvs->rn_left_y[36] = 0.031871;
    uvs->rn_left_y[37] = 0.036222;
    uvs->rn_left_y[38] = 0.039554;
    uvs->rn_left_y[39] = 0.015;
    uvs->rn_left_y[40] = 0.005;
    uvs->rn_left_y[41] = -0.005;
    uvs->rn_left_y[42] = -0.015;
    uvs->rn_left_y[43] = 0.025;
    uvs->rn_left_y[44] = 0.015;
    uvs->rn_left_y[45] = 0.005;
    uvs->rn_left_y[46] = -0.005;
    uvs->rn_left_y[47] = -0.015;
    uvs->rn_left_y[48] = -0.025;
    uvs->rn_left_y[49] = 0.035;
    uvs->rn_left_y[50] = 0.025;
    uvs->rn_left_y[51] = 0.015;
    uvs->rn_left_y[52] = 0.005;
    uvs->rn_left_y[53] = -0.005;
    uvs->rn_left_y[54] = -0.015;
    uvs->rn_left_y[55] = -0.025;
    uvs->rn_left_y[56] = -0.035;
    uvs->rn_left_y[57] = 0.035;
    uvs->rn_left_y[58] = 0.025;
    uvs->rn_left_y[59] = 0.015;
    uvs->rn_left_y[60] = 0.005;
    uvs->rn_left_y[61] = -0.005;
    uvs->rn_left_y[62] = -0.015;
    uvs->rn_left_y[63] = -0.025;
    uvs->rn_left_y[64] = -0.035;
    uvs->rn_left_y[65] = -0.045;
    uvs->rn_left_y[66] = 0.045;
    uvs->rn_left_y[67] = 0.035;
    uvs->rn_left_y[68] = 0.025;
    uvs->rn_left_y[69] = 0.015;
    uvs->rn_left_y[70] = 0.005;
    uvs->rn_left_y[71] = -0.005;
    uvs->rn_left_y[72] = -0.015;
    uvs->rn_left_y[73] = -0.025;
    uvs->rn_left_y[74] = -0.035;
    uvs->rn_left_y[75] = -0.045;
    uvs->rn_left_y[76] = 0.045;
    uvs->rn_left_y[77] = 0.035;
    uvs->rn_left_y[78] = 0.025;
    uvs->rn_left_y[79] = 0.015;
    uvs->rn_left_y[80] = 0.005;
    uvs->rn_left_y[81] = -0.005;
    uvs->rn_left_y[82] = -0.015;
    uvs->rn_left_y[83] = -0.025;
    uvs->rn_left_y[84] = -0.035;
    uvs->rn_left_y[85] = -0.045;
    uvs->rn_left_y[86] = 0.045;
    uvs->rn_left_y[87] = 0.035;
    uvs->rn_left_y[88] = 0.025;
    uvs->rn_left_y[89] = 0.015;
    uvs->rn_left_y[90] = 0.005;
    uvs->rn_left_y[91] = -0.005;
    uvs->rn_left_y[92] = -0.015;
    uvs->rn_left_y[93] = -0.025;
    uvs->rn_left_y[94] = -0.035;
    uvs->rn_left_y[95] = -0.045;
    uvs->rn_left_y[96] = 0.045;
    uvs->rn_left_y[97] = 0.035;
    uvs->rn_left_y[98] = 0.025;
    uvs->rn_left_y[99] = 0.015;
    uvs->rn_left_y[100] = 0.005;
    uvs->rn_left_y[101] = -0.005;
    uvs->rn_left_y[102] = -0.015;
    uvs->rn_left_y[103] = -0.025;
    uvs->rn_left_y[104] = -0.035;
    uvs->rn_left_y[105] = -0.045;
    uvs->rn_left_y[106] = 0.045;
    uvs->rn_left_y[107] = 0.035;
    uvs->rn_left_y[108] = 0.025;
    uvs->rn_left_y[109] = 0.015;
    uvs->rn_left_y[110] = 0.005;
    uvs->rn_left_y[111] = -0.005;
    uvs->rn_left_y[112] = -0.015;
    uvs->rn_left_y[113] = -0.025;
    uvs->rn_left_y[114] = -0.035;
    uvs->rn_left_y[115] = -0.045;
    uvs->rn_left_y[116] = 0.045;
    uvs->rn_left_y[117] = 0.035;
    uvs->rn_left_y[118] = 0.025;
    uvs->rn_left_y[119] = 0.015;
    uvs->rn_left_y[120] = 0.005;
    uvs->rn_left_y[121] = -0.005;
    uvs->rn_left_y[122] = -0.015;
    uvs->rn_left_y[123] = -0.025;
    uvs->rn_left_y[124] = -0.035;
    uvs->rn_left_y[125] = -0.045;
    uvs->rn_left_y[126] = 0.045;
    uvs->rn_left_y[127] = 0.035;
    uvs->rn_left_y[128] = 0.025;
    uvs->rn_left_y[129] = 0.015;
    uvs->rn_left_y[130] = 0.005;
    uvs->rn_left_y[131] = -0.005;
    uvs->rn_left_y[132] = -0.015;
    uvs->rn_left_y[133] = -0.025;
    uvs->rn_left_y[134] = -0.035;
    uvs->rn_left_y[135] = -0.045;
    uvs->rn_left_y[136] = 0.045;
    uvs->rn_left_y[137] = 0.035;
    uvs->rn_left_y[138] = 0.025;
    uvs->rn_left_y[139] = 0.015;
    uvs->rn_left_y[140] = 0.005;
    uvs->rn_left_y[141] = -0.005;
    uvs->rn_left_y[142] = -0.015;
    uvs->rn_left_y[143] = -0.025;
    uvs->rn_left_y[144] = -0.035;
    uvs->rn_left_y[145] = -0.045;
    uvs->rn_left_y[146] = 0.045;
    uvs->rn_left_y[147] = 0.035;
    uvs->rn_left_y[148] = 0.025;
    uvs->rn_left_y[149] = 0.015;
    uvs->rn_left_y[150] = 0.005;
    uvs->rn_left_y[151] = -0.005;
    uvs->rn_left_y[152] = -0.015;
    uvs->rn_left_y[153] = -0.025;
    uvs->rn_left_y[154] = -0.035;
    uvs->rn_left_y[155] = -0.045;
    uvs->rn_left_y[156] = 0.035;
    uvs->rn_left_y[157] = 0.025;
    uvs->rn_left_y[158] = 0.015;
    uvs->rn_left_y[159] = 0.005;
    uvs->rn_left_y[160] = -0.005;
    uvs->rn_left_y[161] = -0.015;
    uvs->rn_left_y[162] = -0.025;
    uvs->rn_left_y[163] = -0.035;
    uvs->rn_left_y[164] = -0.045;
    uvs->rn_left_y[165] = 0.035;
    uvs->rn_left_y[166] = 0.025;
    uvs->rn_left_y[167] = 0.015;
    uvs->rn_left_y[168] = 0.005;
    uvs->rn_left_y[169] = -0.005;
    uvs->rn_left_y[170] = -0.015;
    uvs->rn_left_y[171] = -0.025;
    uvs->rn_left_y[172] = -0.035;
    uvs->rn_left_y[173] = -0.045;
    uvs->rn_left_y[174] = 0.025;
    uvs->rn_left_y[175] = 0.015;
    uvs->rn_left_y[176] = 0.005;
    uvs->rn_left_y[177] = -0.005;
    uvs->rn_left_y[178] = -0.015;
    uvs->rn_left_y[179] = -0.025;
    uvs->rn_left_y[180] = -0.035;
    uvs->rn_left_y[181] = -0.045;
    uvs->rn_left_y[182] = 0.025;
    uvs->rn_left_y[183] = 0.015;
    uvs->rn_left_y[184] = 0.005;
    uvs->rn_left_y[185] = -0.005;
    uvs->rn_left_y[186] = -0.015;
    uvs->rn_left_y[187] = -0.025;
    uvs->rn_left_y[188] = -0.035;
    uvs->rn_left_y[189] = -0.045;
    uvs->rn_left_y[190] = 0.015;
    uvs->rn_left_y[191] = 0.005;
    uvs->rn_left_y[192] = -0.005;
    uvs->rn_left_y[193] = -0.015;
    uvs->rn_left_y[194] = -0.025;
    uvs->rn_left_y[195] = -0.035;
    uvs->rn_left_y[196] = 0.005;
    uvs->rn_left_y[197] = -0.005;
    uvs->rn_left_y[198] = -0.015;
    uvs->rn_left_y[199] = -0.025;
    uvs->rn_left_y[200] = -0.015;

    uvs->rn_right_y[1] = -0.039554;
    uvs->rn_right_y[2] = -0.04177;
    uvs->rn_right_y[3] = -0.043;
    uvs->rn_right_y[4] = -0.045;
    uvs->rn_right_y[5] = -0.045;
    uvs->rn_right_y[6] = -0.044334;
    uvs->rn_right_y[7] = -0.043354;
    uvs->rn_right_y[8] = -0.041164;
    uvs->rn_right_y[9] = -0.0387;
    uvs->rn_right_y[10] = -0.032816;
    uvs->rn_right_y[11] = -0.0289;
    uvs->rn_right_y[12] = -0.022984;
    uvs->rn_right_y[13] = 0.019198;
    uvs->rn_right_y[14] = 0.025738;
    uvs->rn_right_y[15] = 0.030759;
    uvs->rn_right_y[16] = 0.035204;
    uvs->rn_right_y[17] = 0.040847;
    uvs->rn_right_y[18] = 0.04319;
    uvs->rn_right_y[19] = 0.045;
    uvs->rn_right_y[20] = 0.045;
    uvs->rn_right_y[21] = 0.0436;
    uvs->rn_right_y[22] = 0.040817;
    uvs->rn_right_y[23] = 0.038052;
    uvs->rn_right_y[24] = 0.034944;
    uvs->rn_right_y[25] = 0.030833;
    uvs->rn_right_y[26] = 0.025986;
    uvs->rn_right_y[27] = 0.020581;
    uvs->rn_right_y[28] = 0.00709;
    uvs->rn_right_y[29] = 0.001312;
    uvs->rn_right_y[30] = -0.004467;
    uvs->rn_right_y[31] = -0.01004;
    uvs->rn_right_y[32] = -0.015613;
    uvs->rn_right_y[33] = -0.0193;
    uvs->rn_right_y[34] = -0.024782;
    uvs->rn_right_y[35] = -0.02849;
    uvs->rn_right_y[36] = -0.031871;
    uvs->rn_right_y[37] = -0.036222;
    uvs->rn_right_y[38] = -0.039554;
    uvs->rn_right_y[39] = -0.015;
    uvs->rn_right_y[40] = -0.005;
    uvs->rn_right_y[41] = 0.005;
    uvs->rn_right_y[42] = 0.015;
    uvs->rn_right_y[43] = -0.025;
    uvs->rn_right_y[44] = -0.015;
    uvs->rn_right_y[45] = -0.005;
    uvs->rn_right_y[46] = 0.005;
    uvs->rn_right_y[47] = 0.015;
    uvs->rn_right_y[48] = 0.025;
    uvs->rn_right_y[49] = -0.035;
    uvs->rn_right_y[50] = -0.025;
    uvs->rn_right_y[51] = -0.015;
    uvs->rn_right_y[52] = -0.005;
    uvs->rn_right_y[53] = 0.005;
    uvs->rn_right_y[54] = 0.015;
    uvs->rn_right_y[55] = 0.025;
    uvs->rn_right_y[56] = 0.035;
    uvs->rn_right_y[57] = -0.035;
    uvs->rn_right_y[58] = -0.025;
    uvs->rn_right_y[59] = -0.015;
    uvs->rn_right_y[60] = -0.005;
    uvs->rn_right_y[61] = 0.005;
    uvs->rn_right_y[62] = 0.015;
    uvs->rn_right_y[63] = 0.025;
    uvs->rn_right_y[64] = 0.035;
    uvs->rn_right_y[65] = 0.045;
    uvs->rn_right_y[66] = -0.045;
    uvs->rn_right_y[67] = -0.035;
    uvs->rn_right_y[68] = -0.025;
    uvs->rn_right_y[69] = -0.015;
    uvs->rn_right_y[70] = -0.005;
    uvs->rn_right_y[71] = 0.005;
    uvs->rn_right_y[72] = 0.015;
    uvs->rn_right_y[73] = 0.025;
    uvs->rn_right_y[74] = 0.035;
    uvs->rn_right_y[75] = 0.045;
    uvs->rn_right_y[76] = -0.045;
    uvs->rn_right_y[77] = -0.035;
    uvs->rn_right_y[78] = -0.025;
    uvs->rn_right_y[79] = -0.015;
    uvs->rn_right_y[80] = -0.005;
    uvs->rn_right_y[81] = 0.005;
    uvs->rn_right_y[82] = 0.015;
    uvs->rn_right_y[83] = 0.025;
    uvs->rn_right_y[84] = 0.035;
    uvs->rn_right_y[85] = 0.045;
    uvs->rn_right_y[86] = -0.045;
    uvs->rn_right_y[87] = -0.035;
    uvs->rn_right_y[88] = -0.025;
    uvs->rn_right_y[89] = -0.015;
    uvs->rn_right_y[90] = -0.005;
    uvs->rn_right_y[91] = 0.005;
    uvs->rn_right_y[92] = 0.015;
    uvs->rn_right_y[93] = 0.025;
    uvs->rn_right_y[94] = 0.035;
    uvs->rn_right_y[95] = 0.045;
    uvs->rn_right_y[96] = -0.045;
    uvs->rn_right_y[97] = -0.035;
    uvs->rn_right_y[98] = -0.025;
    uvs->rn_right_y[99] = -0.015;
    uvs->rn_right_y[100] = -0.005;
    uvs->rn_right_y[101] = 0.005;
    uvs->rn_right_y[102] = 0.015;
    uvs->rn_right_y[103] = 0.025;
    uvs->rn_right_y[104] = 0.035;
    uvs->rn_right_y[105] = 0.045;
    uvs->rn_right_y[106] = -0.045;
    uvs->rn_right_y[107] = -0.035;
    uvs->rn_right_y[108] = -0.025;
    uvs->rn_right_y[109] = -0.015;
    uvs->rn_right_y[110] = -0.005;
    uvs->rn_right_y[111] = 0.005;
    uvs->rn_right_y[112] = 0.015;
    uvs->rn_right_y[113] = 0.025;
    uvs->rn_right_y[114] = 0.035;
    uvs->rn_right_y[115] = 0.045;
    uvs->rn_right_y[116] = -0.045;
    uvs->rn_right_y[117] = -0.035;
    uvs->rn_right_y[118] = -0.025;
    uvs->rn_right_y[119] = -0.015;
    uvs->rn_right_y[120] = -0.005;
    uvs->rn_right_y[121] = 0.005;
    uvs->rn_right_y[122] = 0.015;
    uvs->rn_right_y[123] = 0.025;
    uvs->rn_right_y[124] = 0.035;
    uvs->rn_right_y[125] = 0.045;
    uvs->rn_right_y[126] = -0.045;
    uvs->rn_right_y[127] = -0.035;
    uvs->rn_right_y[128] = -0.025;
    uvs->rn_right_y[129] = -0.015;
    uvs->rn_right_y[130] = -0.005;
    uvs->rn_right_y[131] = 0.005;
    uvs->rn_right_y[132] = 0.015;
    uvs->rn_right_y[133] = 0.025;
    uvs->rn_right_y[134] = 0.035;
    uvs->rn_right_y[135] = 0.045;
    uvs->rn_right_y[136] = -0.045;
    uvs->rn_right_y[137] = -0.035;
    uvs->rn_right_y[138] = -0.025;
    uvs->rn_right_y[139] = -0.015;
    uvs->rn_right_y[140] = -0.005;
    uvs->rn_right_y[141] = 0.005;
    uvs->rn_right_y[142] = 0.015;
    uvs->rn_right_y[143] = 0.025;
    uvs->rn_right_y[144] = 0.035;
    uvs->rn_right_y[145] = 0.045;
    uvs->rn_right_y[146] = -0.045;
    uvs->rn_right_y[147] = -0.035;
    uvs->rn_right_y[148] = -0.025;
    uvs->rn_right_y[149] = -0.015;
    uvs->rn_right_y[150] = -0.005;
    uvs->rn_right_y[151] = 0.005;
    uvs->rn_right_y[152] = 0.015;
    uvs->rn_right_y[153] = 0.025;
    uvs->rn_right_y[154] = 0.035;
    uvs->rn_right_y[155] = 0.045;
    uvs->rn_right_y[156] = -0.035;
    uvs->rn_right_y[157] = -0.025;
    uvs->rn_right_y[158] = -0.015;
    uvs->rn_right_y[159] = -0.005;
    uvs->rn_right_y[160] = 0.005;
    uvs->rn_right_y[161] = 0.015;
    uvs->rn_right_y[162] = 0.025;
    uvs->rn_right_y[163] = 0.035;
    uvs->rn_right_y[164] = 0.045;
    uvs->rn_right_y[165] = -0.035;
    uvs->rn_right_y[166] = -0.025;
    uvs->rn_right_y[167] = -0.015;
    uvs->rn_right_y[168] = -0.005;
    uvs->rn_right_y[169] = 0.005;
    uvs->rn_right_y[170] = 0.015;
    uvs->rn_right_y[171] = 0.025;
    uvs->rn_right_y[172] = 0.035;
    uvs->rn_right_y[173] = 0.045;
    uvs->rn_right_y[174] = -0.025;
    uvs->rn_right_y[175] = -0.015;
    uvs->rn_right_y[176] = -0.005;
    uvs->rn_right_y[177] = 0.005;
    uvs->rn_right_y[178] = 0.015;
    uvs->rn_right_y[179] = 0.025;
    uvs->rn_right_y[180] = 0.035;
    uvs->rn_right_y[181] = 0.045;
    uvs->rn_right_y[182] = -0.025;
    uvs->rn_right_y[183] = -0.015;
    uvs->rn_right_y[184] = -0.005;
    uvs->rn_right_y[185] = 0.005;
    uvs->rn_right_y[186] = 0.015;
    uvs->rn_right_y[187] = 0.025;
    uvs->rn_right_y[188] = 0.035;
    uvs->rn_right_y[189] = 0.045;
    uvs->rn_right_y[190] = -0.015;
    uvs->rn_right_y[191] = -0.005;
    uvs->rn_right_y[192] = 0.005;
    uvs->rn_right_y[193] = 0.015;
    uvs->rn_right_y[194] = 0.025;
    uvs->rn_right_y[195] = 0.035;
    uvs->rn_right_y[196] = -0.005;
    uvs->rn_right_y[197] = 0.005;
    uvs->rn_right_y[198] = 0.015;
    uvs->rn_right_y[199] = 0.025;
    uvs->rn_right_y[200] = 0.015;

	for(i=1; i<=uvs->Msize_GCM; i++)
	{
		uvs->rn_left_z[i]  = 0.0;
		uvs->rn_right_z[i] = 0.0;
	}
}

#endif
