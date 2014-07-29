//---------------------------
//
// Nicolas Van der Noot
//
// Creation : 25/07/2014
// Last update : 25/07/2014
//---------------------------

#include "controller_interface.h"

//inputs and outputs structure
static Ctrl_Inputs  *ivs;
static Ctrl_Outputs *ovs;

/*
 * Interface of the controller: initialization
 */
void controller_init_interface(MBSdataStruct *MBSdata)
{
    // io structures initialization
    ivs = (Ctrl_Inputs*)  malloc(sizeof(Ctrl_Inputs));
    ovs = (Ctrl_Outputs*) malloc(sizeof(Ctrl_Outputs));

	robotran_ctrl_init_interface(MBSdata, ivs);
}

/*
 * Interface of the controller: loop
 */
void controller_loop_interface(MBSdataStruct *MBSdata)
{
	robotran_ctrl_loop_interface(MBSdata, ivs, ovs);
}

void controller_close_interface(MBSdataStruct *MBSdata)
{
    // io structures release memory
    free(ivs);
    free(ovs);
}

