/*===========================================================================*
  *
  *  UserModelStruct.c
  *	
  *  (c) Universite catholique de Louvain
  *      D�partement de M�canique 
  *      Unit� de Production M�canique et Machines 
  *      2, Place du Levant 
  *      1348 Louvain-la-Neuve 
  *  http://www.robotran.be// 
  *  
 /*===========================================================================*/

#include "MBSdataStruct.h"

#include "lut.h"

#ifndef STANDALONE
UserModelStruct * loadUserModel(mxArray *usm_ptr)
#else
UserModelStruct * loadUserModel()
#endif
{
	return NULL;
}

void freeUserModel(UserModelStruct *ums)
{

}
