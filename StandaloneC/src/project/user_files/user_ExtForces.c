//--------------------------- 
// UCL-CEREM-MBS 
// 
// @version MBsysLab_s 1.7.a 
// 
// Creation : 2006 
// Last update : 01/10/2008 
//--------------------------- 

#include "simu_def.h"

#ifdef SIMBODY
#include "simbody_functions.h"
#include "simbody_cpp_functions.h"
#endif

double* user_ExtForces(double PxF[4], double RxF[4][4], 
					   double VxF[4], double OMxF[4], 
					   double AxF[4], double OMPxF[4], 
					   MBSdataStruct *MBSdata, double tsim, int ixF)
{
	int i;
	double Fx=0.0, Fy=0.0, Fz=0.0;
	double Mx=0.0, My=0.0, Mz=0.0;
	double dxF[3+1] ={0.0, 0.0, 0.0, 0.0}; // +1 because indexes begin at 1

	#ifdef SIMBODY
	SimbodyBodiesStruct *simbodyBodies;
	#endif

	double *SWr = MBSdata->SWr[ixF];
	int idpt = 0;
        
    // user variables
    UserIOStruct *uvs;
    
    uvs = MBSdata->user_IO;

	idpt = MBSdata->xfidpt[ixF];

	dxF[1] = MBSdata->dpt[1][idpt];
	dxF[2] = MBSdata->dpt[2][idpt];
	dxF[3] = MBSdata->dpt[3][idpt];



	#ifdef SIMBODY
	
	//compute all the forces at once (arbitrary for 1st external force) (else compute same thing for each force sensors)
	if(ixF == 1)   
	{
		// 1) Simbody receives kinematics from Robotran
		update_simbody_kinematics(MBSdata->user_IO->simbodyStruct->simbodyBodies, MBSdata);

		// 2) Simbody computes contact force
		loop_Simbody(MBSdata->user_IO->simbodyStruct);
	}

	// 3) Simbody sends contact force to robotran dynamics
	simbodyBodies = uvs->simbodyStruct->simbodyBodies;	
	for(i=0; i<simbodyBodies->nb_contact_bodies; i++)
	{
	 	if (ixF == simbodyBodies->F_sensor_Robotran_index[i])
	 	{
	 		Fx += simbodyBodies->force_bodies[i][0]; 
	 		Fy += simbodyBodies->force_bodies[i][1]; 
	 		Fz += simbodyBodies->force_bodies[i][2]; 

	 		Mx += simbodyBodies->torque_bodies[i][0]; 
	 		My += simbodyBodies->torque_bodies[i][1]; 
	 		Mz += simbodyBodies->torque_bodies[i][2];  		

	 		break;
	 	}
	}
	#endif
	

	// initializes the terms of Swr
	SWr[1] = Fx;
	SWr[2] = Fy;
	SWr[3] = Fz;
	SWr[4] = Mx;
	SWr[5] = My;
	SWr[6] = Mz;
	SWr[7] = dxF[1];
	SWr[8] = dxF[2];
	SWr[9] = dxF[3];

	// ---- If the force sensor considered is located in a foot ---- //
	#ifdef COMP_FEET
	if((ixF == RFOOT_FSENS_ID) || (ixF == LFOOT_FSENS_ID) || (ixF == RFOOT_DIST_FSENS_ID) || (ixF == LFOOT_DIST_FSENS_ID))
	#else
	if((ixF == RFOOT_FSENS_ID) || (ixF == LFOOT_FSENS_ID))
	#endif	
	{
		#ifdef GCM_MT
		// Compute thing resultant forces and moments from the ground
		ground_mesh_model(PxF, RxF, VxF, OMxF, MBSdata, tsim, ixF, dxF, SWr);
		#endif

		// Limit external forces
		for (i=1; i<=3; i++)
		{
			SWr[i] = (SWr[i] >  MAX_EXT_FORCES) ?  MAX_EXT_FORCES : SWr[i];
			SWr[i] = (SWr[i] < -MAX_EXT_FORCES) ? -MAX_EXT_FORCES : SWr[i];
		}

		// Limit external moments
		for (i=4; i<=6; i++)
		{
			SWr[i] = (SWr[i] >  MAX_EXT_MOMENTS) ?  MAX_EXT_MOMENTS : SWr[i];
			SWr[i] = (SWr[i] < -MAX_EXT_MOMENTS) ? -MAX_EXT_MOMENTS : SWr[i];
		}

		// Write the results in the internal variables (for HG model)
        switch (ixF)
        {
            case RFOOT_FSENS_ID:
                uvs->GRF_r[1] = SWr[1];
                uvs->GRF_r[2] = SWr[2];
                uvs->GRF_r[3] = SWr[3];
                
                uvs->GRM_r[1] = SWr[4];
                uvs->GRM_r[2] = SWr[5];
                uvs->GRM_r[3] = SWr[6];
                break;
                
            case LFOOT_FSENS_ID:
                uvs->GRF_l[1] = SWr[1];
                uvs->GRF_l[2] = SWr[2];
                uvs->GRF_l[3] = SWr[3];
                
                uvs->GRM_l[1] = SWr[4];
                uvs->GRM_l[2] = SWr[5];
                uvs->GRM_l[3] = SWr[6];
                break;

            #ifdef COMP_FEET
    		case LFOOT_DIST_FSENS_ID:
    			uvs->GRF_l_dist[1] = SWr[1];
                uvs->GRF_l_dist[2] = SWr[2];
                uvs->GRF_l_dist[3] = SWr[3];
                
                uvs->GRM_l_dist[1] = SWr[4];
                uvs->GRM_l_dist[2] = SWr[5];
                uvs->GRM_l_dist[3] = SWr[6];
                break;

            case RFOOT_DIST_FSENS_ID:
    			uvs->GRF_r_dist[1] = SWr[1];
                uvs->GRF_r_dist[2] = SWr[2];
                uvs->GRF_r_dist[3] = SWr[3];
                
                uvs->GRM_r_dist[1] = SWr[4];
                uvs->GRM_r_dist[2] = SWr[5];
                uvs->GRM_r_dist[3] = SWr[6];
                break;
            #endif
                
            default:
                break;
        }
    }
	return SWr;
}


// Contact Model of foot written by Mohamad Mosadeghzad  
// mohamad.mosadeghzad@iit.it or m_mzad83@yahoo.com  
// This Contact model is written to increase accuracy and speed of  
// simulation. A mesh grid of contact points under sole of robot should be  
// defined. Contact surface can have a 3D shape and is not necessarily flat  
// surface. Mesh grid can be finer in more critical area of sole which has more  
// contact with ground.
//
// This ground contact model has been modified by Allan Barrea & Nicolas Van der Noot.
// We generalized it to be able to define two different foot shapes (for the left and
// right foot).
//
// WARNING! the reaction force of the ground is always in the Z direction
// but it should be normal to the ground --> so, this model is only valid
// for an horizontal ground.
void ground_mesh_model(double PxF[4], double RxF[4][4], 
					   double VxF[4], double OMxF[4],
					   MBSdataStruct *MBSdata, double tsim,
					   int ixF, double *dxF, double *SWr)
{
    // user variables
    UserIOStruct *uvs = MBSdata->user_IO;
    
	int i,j;

	int Msize;	// Mesh size = nb of points under one foot

	// --- Mohamad's model ---
	double K	  = K_GCM; // stiffness
	double D	  = D_GCM; // damping
	double mu_grf = MU_GCM; // ground friction coeff. (Fx=mu*Fz)
	// --- Geyer's model ---
	double k_gz     = K_GZ_GCM; // stiffness
	double v_gz_max = V_GZ_MAX_GCM;

	double Fx,Fy,Fz;
	double Mx,My,Mz;

	double *rn_x,*rn_y,*rn_z;
	double *temp_grfx,*temp_grfy;
	int *flag_grfx,*flag_grfy;

	double fr, Alpha, dz, threshold; // additional local variables

	// "rn" is mesh points in body coordinate system; it shows the shape of the sole.
	// "temp_grfx_left/right" and "temp_grfy_left/right" are used to store the value of initial X/Y contact point with the ground.
	// "flag_grfx_left/right" and "flag_grfy_left/right" are used to store the initial X/Y contact point with the ground.

	double **rn,**r0,**r,**v;
	double *FTx,*FTy,*FTz,**T;
	double **RxF_transpose;

    Msize = 0;

    rn_x = NULL;
    rn_y = NULL;
    rn_z = NULL;

    temp_grfx = NULL;
    temp_grfy = NULL;

    flag_grfx = NULL;
    flag_grfy = NULL;
    
	// Fetching variables related to the foot indicated by ixF
	switch (ixF)
    {
    	case LFOOT_FSENS_ID:
    		#if defined(COMP_FEET) || defined(SHORT_FEET)
    		Msize = uvs->Msize_GCM_prox;

    		rn_x = uvs->rn_left_prox_x;
			rn_y = uvs->rn_left_prox_y;
			rn_z = uvs->rn_left_prox_z;

			temp_grfx = uvs->temp_grfx_left_prox;
			temp_grfy = uvs->temp_grfy_left_prox;

			flag_grfx = uvs->flag_grfx_left_prox;
			flag_grfy = uvs->flag_grfy_left_prox;
			#else
			Msize = uvs->Msize_GCM;

    		rn_x = uvs->rn_left_x;
			rn_y = uvs->rn_left_y;
			rn_z = uvs->rn_left_z;

			temp_grfx = uvs->temp_grfx_left;
			temp_grfy = uvs->temp_grfy_left;

			flag_grfx = uvs->flag_grfx_left;
			flag_grfy = uvs->flag_grfy_left;
			#endif
    		break;

    	case RFOOT_FSENS_ID:
    		#if defined(COMP_FEET) || defined(SHORT_FEET)
    		Msize = uvs->Msize_GCM_prox;

    		rn_x = uvs->rn_right_prox_x;
			rn_y = uvs->rn_right_prox_y;
			rn_z = uvs->rn_right_prox_z;

			temp_grfx = uvs->temp_grfx_right_prox;
			temp_grfy = uvs->temp_grfy_right_prox;

			flag_grfx = uvs->flag_grfx_right_prox;
			flag_grfy = uvs->flag_grfy_right_prox;    		
			#else
			Msize = uvs->Msize_GCM;

    		rn_x = uvs->rn_right_x;
			rn_y = uvs->rn_right_y;
			rn_z = uvs->rn_right_z;

			temp_grfx = uvs->temp_grfx_right;
			temp_grfy = uvs->temp_grfy_right;

			flag_grfx = uvs->flag_grfx_right;
			flag_grfy = uvs->flag_grfy_right;	
			#endif
    		break;

    	#ifdef COMP_FEET
    	case LFOOT_DIST_FSENS_ID:
    		Msize = uvs->Msize_GCM_dist;

    		rn_x = uvs->rn_left_dist_x;
			rn_y = uvs->rn_left_dist_y;
			rn_z = uvs->rn_left_dist_z;

			temp_grfx = uvs->temp_grfx_left_dist;
			temp_grfy = uvs->temp_grfy_left_dist;

			flag_grfx = uvs->flag_grfx_left_dist;
			flag_grfy = uvs->flag_grfy_left_dist;
    		break;

    	case RFOOT_DIST_FSENS_ID:
    		Msize = uvs->Msize_GCM_dist;

    		rn_x = uvs->rn_right_dist_x;
			rn_y = uvs->rn_right_dist_y;
			rn_z = uvs->rn_right_dist_z;

			temp_grfx = uvs->temp_grfx_right_dist;
			temp_grfy = uvs->temp_grfy_right_dist;

			flag_grfx = uvs->flag_grfx_right_dist;
			flag_grfy = uvs->flag_grfy_right_dist;
    		break;
    	#endif
    
    	default:
    		break;
    }

	// Allocating local variables for the ground reaction forces computation
	rn  = dmatrix(1,3,1,Msize);	// rn is position vector of mesh points in body coordinate system (attached to the foot)
	r0  = dmatrix(1,3,1,Msize);	// r0 is position vector of mesh points in a frame attached to the body but aligned with the inertial frame
	r   = dmatrix(1,3,1,Msize);	// r is position vector of mesh points in global coordinate system (inertial frame)
	v   = dmatrix(1,3,1,Msize);	// v is velocity vector of mesh points in global coordinate system (inertial frame)
	FTx = dvector(1,Msize);		// Fx
	FTy = dvector(1,Msize);		// Fy
	FTz = dvector(1,Msize);		// Fz
	T	= dmatrix(1,3,1,Msize);	// Torques in the ankle due to each mesh point
	RxF_transpose = dmatrix(1,3,1,3); // Transpose of the rotation matrix RxF

	// Initializing local variables
	for(i=1; i<=Msize; i++)
	{
		rn[1][i] = rn_x[i];
		rn[2][i] = rn_y[i];
		rn[3][i] = rn_z[i];

		v[1][i] = 0.0;
		v[2][i] = 0.0;
		v[3][i] = 0.0;

		FTx[i] = 0.0;
		FTy[i] = 0.0;
		FTz[i] = 0.0;

		T[1][i] = 0.0;
		T[2][i] = 0.0;
		T[3][i] = 0.0;
	}

	// --- getting absolute positions of mesh points (i.e. in inertial frame) ---
	
	// RxF is the rotational matrix to pass from inertial frame to body-attached frame
	// r0 = RxF_transpose*rn (matrix product)

	// Transposing RxF
	for(i=1; i<=3 ;i++)
	{
		for(j=1; j<=3 ;j++)
		{
			RxF_transpose[i][j] = RxF[j][i];
		}
	}

	// Matrix product
	for(i=1; i<=Msize; i++)
	{
		r0[1][i] = RxF_transpose[1][1]*rn[1][i] + RxF_transpose[1][2]*rn[2][i] + RxF_transpose[1][3]*rn[3][i];
		r0[2][i] = RxF_transpose[2][1]*rn[1][i] + RxF_transpose[2][2]*rn[2][i] + RxF_transpose[2][3]*rn[3][i];
		r0[3][i] = RxF_transpose[3][1]*rn[1][i] + RxF_transpose[3][2]*rn[2][i] + RxF_transpose[3][3]*rn[3][i];
	}

	// r = r0 + PxF
	for(i=1; i<=Msize; i++)
	{
		r[1][i] = r0[1][i] + PxF[1];
		r[2][i] = r0[2][i] + PxF[2];
		r[3][i] = r0[3][i] + PxF[3];
	}

	// --- getting absolute velocities of mesh points (i.e. in inertial frame) ---
	// u_dot = u_circle + omega x u (velocity formula in a mobile frame),
	// where u is the position in the body-attached frame
	// here: v = VxF + OMxF x r0
	for(i=1; i<=Msize; i++)
	{
		v[1][i] = - OMxF[3]*r0[2][i] + OMxF[2]*r0[3][i] + VxF[1];
		v[2][i] =   OMxF[3]*r0[1][i] - OMxF[1]*r0[3][i] + VxF[2];
		v[3][i] = - OMxF[2]*r0[1][i] + OMxF[1]*r0[2][i] + VxF[3];
	}

		

	// Now, we know the absolute position and velocity of each grid point (i.e.
	// in the inertial frame). It is now time to compute the force for each of
	// them.
	for(i=1; i<=Msize; i++) // for each point in the mesh grid
	{
		// Ground profile as a function of x, y and time (tsim) can be added here
		threshold = get_ground_height(r[1][i], r[2][i], tsim, MBSdata);

		dz = r[3][i] - threshold;

		if (dz <= 0) // if contact with the ground
		{
			/*// --- Mohamad's model ---
			// Should APPLY DAMPING IN BOTH DIRECTIONS, BOTH WHEN penetration and when the foot is going out of the ground.  
			// apply the linear model for the normal force: (NB: could be extended into non linear model --> Geyer !)
			FTz[i]= -K*(r[3][i] - threshold) - D*v[3][i];//
			// The force is applied only to push the robot, not to pull it toward the ground.
			//FTz[i] = (FTz[i] < 0) ? 0.0 : FTz[i];//*/

			// --- Geyer's model ---
			if (v[3][i] < v_gz_max)
			{
				FTz[i] = -k_gz * dz * (1-v[3][i]/v_gz_max);
			}
			else
			{
				FTz[i] = 0.0;
			}

			//----------------------------------------------------------                     
			// Horizontal friction force in X direction:    
			if(flag_grfx[i] == 0)
			{
				temp_grfx[i] = r[1][i]; // store the initial point of contact.     
				flag_grfx[i] = 1;
			}
  
			FTx[i] = -D*v[1][i] - K*(r[1][i] - temp_grfx[i]); // friction force can be positive or negative.     
  
			//----------------------------------------------------------     
			// Horizontal friction force in Y direction:  
			if(flag_grfy[i] == 0)
			{
				temp_grfy[i] = r[2][i]; // store the initial point of contact.     
				flag_grfy[i] = 1;
			}
  
			FTy[i] = -D*v[2][i] - K*(r[2][i] - temp_grfy[i]); // friction force can be positive or negative.     

			// check if each contact point is slipping or not
			fr=sqrt(FTx[i]*FTx[i]+FTy[i]*FTy[i]);

			if(fr > mu_grf*FTz[i])
			{
				fr = mu_grf*FTz[i];
				Alpha = atan2(v[2][i],v[1][i]);
				FTx[i] = -fr*cos(Alpha);
				FTy[i] = -fr*sin(Alpha);
				flag_grfx[i] = 0;
				flag_grfy[i] = 0;
			}
		}
		else
		{
			FTx[i]=0;
			FTy[i]=0;
			FTz[i]=0;
			flag_grfx[i]=0;
			flag_grfy[i]=0;
		} // END if contact with the ground
	} // END for loop for all Mesh points of contact (Msize)

	Fx=0.0; Fy=0.0; Fz=0.0;
	Mx=0.0; My=0.0; Mz=0.0;

	// bring back all forces to the force sensor
	for(i=1; i<=Msize; i++)
	{
		Fx += FTx[i];
		Fy += FTy[i];
		Fz += FTz[i];
	}
  
	// Calculate resultant Torque relative to Position of sensor F
	// Cross Product of r0 and FT (T = r0 x FT)
	for(i=1; i<=Msize; i++)
	{
		T[1][i] = - r0[3][i]*FTy[i] + r0[2][i]*FTz[i];  
		T[2][i] =   r0[3][i]*FTx[i] - r0[1][i]*FTz[i];  
		T[3][i] = - r0[2][i]*FTx[i] + r0[1][i]*FTy[i];
	}

	// calculating Total Torque
	for(i=1; i<=Msize; i++)
	{
		Mx += T[1][i];
		My += T[2][i];  
		Mz += T[3][i];
	}

	// Storing results in SWr
	SWr[1] = Fx;
	SWr[2] = Fy;
	SWr[3] = Fz;
	SWr[4] = Mx;
	SWr[5] = My;
	SWr[6] = Mz;
	SWr[7] = dxF[1];
	SWr[8] = dxF[2];
	SWr[9] = dxF[3];

	// Deallocating memory
	#ifdef STANDALONE
	free_dmatrix(rn,1,1);
	free_dmatrix(r0,1,1);
	free_dmatrix(r,1,1);
	free_dmatrix(v,1,1);
	free_dvector(FTx,1);
	free_dvector(FTy,1);
	free_dvector(FTz,1);
	free_dmatrix(T,1,1);
	free_dmatrix(RxF_transpose,1,1);
	#else
	free_dmatrix(rn,1,3,1,Msize);
	free_dmatrix(r0,1,3,1,Msize);
	free_dmatrix(r,1,3,1,Msize);
	free_dmatrix(v,1,3,1,Msize);
	free_dvector(FTx,1,Msize);
	free_dvector(FTy,1,Msize);
	free_dvector(FTz,1,Msize);
	free_dmatrix(T,1,3,1,Msize);
	free_dmatrix(RxF_transpose,1,3,1,3);
	#endif
}
