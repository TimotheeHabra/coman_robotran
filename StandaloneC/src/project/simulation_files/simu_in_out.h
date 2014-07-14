/*
 * Header of the simulation output configuration
 *
 * author: Nicolas Van der Noot
 */

#ifndef _OUTPUT_SIMU_H_
#define _OUTPUT_SIMU_H_

#include "MBSdataStruct.h"

/*
 * File used to let the user define the inputs and the outputs of a Standalone simulation.
 * This file and the 'simu_in_out.c' ones must be filled according to the user need.
 *
 * Here, we provide an example for an optimization process:
 *       input:  a vector to evaluate for the optimization ('double *optiInputs')
 *       output: the fitness (objective function) computed by the user during the simulation (double fitness)
 */

// ---- MODIFICATIONS HERE ---- //

// input structure -> user can fill according to its needs
typedef struct InputSimu {
    double *optiInputs;
} InputSimu;

// output structure -> user can fill according to its needs
typedef struct OutputSimu {
    double fitness;
} OutputSimu;

// ---- AVOID MODIFYING ---- //


// functions prototype
InputSimu* create_InputSimu();
void free_InputSimu(InputSimu *inputSimu);

OutputSimu* create_OutputSimu(MBSdataStruct *MBSdata);
void free_OutputSimu(OutputSimu* outputSimu);

#endif
