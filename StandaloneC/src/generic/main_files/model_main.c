/*
 * Top-level function of the model.
 * 1. initialization
 * 2. loop
 * 3. end of the simulation
 *
 * author: Nicolas Van der Noot
 */

#include "main_simulation.h"

#ifdef OPTI
#include "opti_def.h"
#include "simu_def.h"
#endif


/*
 * Main function
 */
#ifndef OPTI

int main(int argc, char const *argv[])
{    
    // variables declaration
    InputSimu *inputSimu;
	OutputSimu *outputSimu;

    // input structure initialization
    inputSimu  = create_InputSimu();

    // main loop
    outputSimu = model_eval(inputSimu);

    // print fitness
    #ifdef PRINT_REPORT
    printf("fitness: %f\n", outputSimu->fitness);
    #endif

    // free memory
    free_InputSimu(inputSimu);
    free_OutputSimu(outputSimu);
	
	return 0;
}
#endif

/*
 * Main loop
 */
OutputSimu* model_eval(InputSimu *inputSimu)
{
    // varaibles declaration
    Loop_arguments *loop_arguments;
    OutputSimu  *outputSimu;
    
    // initialization
	loop_arguments = init_simulation();

    // optimization init
    #ifdef OPTI
	opti_init(inputSimu, loop_arguments->MBSdata);
    #endif

    // loop
    outputSimu = loop_simulation(loop_arguments);

    // end of the simulation
    finish_simulation(loop_arguments);

    return outputSimu;
}
