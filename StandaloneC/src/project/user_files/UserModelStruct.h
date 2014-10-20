//---------------------------
// C-code automatically generated from Gen_mds_user project
//
//
// Last update : Mon Oct 20 16:27:37 2014
//---------------------------



#ifndef USERMODELSTRUCT_h
#define USERMODELSTRUCT_h

#include "lut.h"
#include "useful_functions.h"
#include "mbs_xml_reader.h"

// ============================================================ //


typedef struct UserModelStruct 
{
    struct Actuator{
        int* Motor;
        // index of the corresponding values in MBSdataStruct->ux/uxd/ux0
    } Actuator;
 
} UserModelStruct;

UserModelStruct* init_UserModelStruct();
void free_UserModelStruct(UserModelStruct* ums);
void load_UserModelStruct(MDS_gen_strct* gen, UserModelStruct* ums);
// ============================================================ //
 
# endif
