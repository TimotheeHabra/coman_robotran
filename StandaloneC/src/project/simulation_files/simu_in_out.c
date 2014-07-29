/*
 * Simulation output configuration
 *
 * author: Nicolas Van der Noot
 */

#include "simu_in_out.h"

 /*
 * File used to let the user define the inputs and the outputs of a Standalone simulation.
 * This file and the 'simu_in_out.h' ones must be filled according to the user need.
 *
 * Here, we provide an example for an optimization process:
 *       input:  a vector to evaluate for the optimization ('double *optiInputs')
 *       output: the fitness (objective function) computed by the user during the simulation (double fitness)
 */

/*
 * Create and initialize the input structure -> user can modify
 */
InputSimu* create_InputSimu()
{
    InputSimu *inputSimu;

    inputSimu = (InputSimu*) malloc(sizeof(InputSimu));

    inputSimu->optiInputs = NULL;
    
    return inputSimu;
}

/*
 * free the input structure -> user can modify
 */
void free_InputSimu(InputSimu *inputSimu)
{
    free(inputSimu);
}

/*
 * Create and initialize the output structure -> user can modify
 */
OutputSimu* create_OutputSimu(MBSdataStruct *MBSdata)
{
    OutputSimu *outputSimu;

    outputSimu = (OutputSimu*) malloc(sizeof(OutputSimu));

    outputSimu->fitness = 0.0;

    return outputSimu;
}

/*
 * free the output structure -> user can modify
 */
void free_OutputSimu(OutputSimu* outputSimu)
{
    free(outputSimu);
}
