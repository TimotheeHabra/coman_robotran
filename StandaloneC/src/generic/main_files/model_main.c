/*
 * Top-level function of the model.
 * 1. initialization
 * 2. loop
 * 3. end of the simulation
 *
 * author: Nicolas Van der Noot
 */

#include "main_simulation.h"

/*
 * Main function
 */
#ifdef CXX
int C_main(int argc, char const *argv[])
#else
int main(int argc, char const *argv[])
#endif
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

    // loop
    outputSimu = loop_simulation(loop_arguments);

    // end of the simulation
    finish_simulation(loop_arguments);

    return outputSimu;
}
