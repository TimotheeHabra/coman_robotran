/*
 * Function related to the integrator (Runge Kutta order 4, from Numerical Recipes)
 */

#include "integrator.h"


/*
 * Computes the derivatives of the system to integrate
 * x = t (i.e. "t" is denoted by "x" here = independent variable)
 */
void derivs(double x, double y[], double dydx[], LocalDataStruct *lds,MBSdataStruct *s)
{
	int i;

	#ifdef ODN
	int j;
	#endif

	s->tsim = x;

	// Update state variables
	for(i=1;i<=s->nqu;i++)
	{
		s->q[s->qu[i]] = y[i-INDEX_0_FLAG_INTEG];
		s->qd[s->qu[i]] = y[i+s->nqu-INDEX_0_FLAG_INTEG];
	}
	for(i=1;i<=s->Nux;i++)
	{
		s->ux[i] = y[i+2*s->nqu-INDEX_0_FLAG_INTEG];
	}

	// Direct Dynamics computation (or with Accelred)

	#if defined(DIRDYNARED)

	i = dirdynared(lds,s);
	if(i<0)
	{
		printf("Loop closing Error : NR iteration overrun !\n");
	} 

	#elif defined(ACCELRED)

	i = accelred(s->qddu,s,s->tsim);

	#elif defined(ODN)

	if (s->nqc>0) user_DrivenJoints(s,s->tsim);

	for(i=1;i<=s->nbody;i++)
	{
		for(j=1;j<=3;j++)
		{
			s->frc[j][i]=0.0;
			s->trq[j][i]=0.0;
		}
	}

	if(s->Nxfrc > 0)
	{
		extforces(s->frc,s->trq,s,s->tsim);
	} 

	s->Qq = user_JointForces(s,s->tsim);

	mbs_dirdyna_odn(s, s->tsim);

	#endif

	// User Derivatives
	if(s->Nux>0)
	{
		user_Derivative(s);
	} 
	
	// Update state vector
	for(i=1;i<=s->nqu;i++)
	{
		dydx[i-INDEX_0_FLAG_INTEG] = s->qd[s->qu[i]];
		#ifdef ODN
		dydx[i+s->nqu-INDEX_0_FLAG_INTEG] = s->qdd[i];
		#else
		dydx[i+s->nqu-INDEX_0_FLAG_INTEG] = s->qddu[i];
		#endif
	}
	for(i=1;i<=s->Nux;i++)
	{
		dydx[i+2*s->nqu-INDEX_0_FLAG_INTEG] = s->uxd[i];
	}
}
