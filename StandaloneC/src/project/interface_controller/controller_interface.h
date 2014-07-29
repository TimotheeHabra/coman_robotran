
#ifndef _CONTROLLER_INTERFACE_H_
#define _CONTROLLER_INTERFACE_H_

#include "simu_def.h"

void controller_inputs(MBSdataStruct *MBSdata, Ctrl_Inputs *ivs);
void controller_outputs(MBSdataStruct *MBSdata, Ctrl_Outputs *ovs);

void robotran_ctrl_init_interface(MBSdataStruct *MBSdata, Ctrl_Inputs  *ivs);
void robotran_ctrl_loop_interface(MBSdataStruct *MBSdata, Ctrl_Inputs  *ivs, Ctrl_Outputs *ovs);

#endif
