/*
 * 
 *
 * author: Nicolas Van der Noot
 */

#include "plot_save_vectors.h"

/*
 * Read a .txt file and returns the corresponding vector (with its size)
 */ 
double* read_vector(char *fileName, int *size_vector)
{
	// variables declaration
	int i, size_vec;
	double cur_value;
	double *vec;
	char line[FGETS_MAX_SIZE];
	FILE* file;

	// initialization
	size_vec = 0;

	file = fopen(fileName, "r");

	if (file == NULL)
	{
		printf("No file: %s\n", fileName);
    	exit(1);
	}

	// size of the vector
	while (fgets(line, FGETS_MAX_SIZE, file) != NULL)
	{
		size_vec++;		
	}

	*size_vector = size_vec;

	// vector allocation
	vec = (double*) malloc(size_vec*sizeof(double));

	for (i=0; i<size_vec; i++)
	{
		vec[i] = 0.0;
	}

	// restart the file reading
	i = 0;
	rewind(file);
	
	// read all the vector values
	while (fgets(line, FGETS_MAX_SIZE, file) != NULL)
	{
		sscanf(line, "%lf", &cur_value);

		vec[i] = cur_value;
		i++;
	}

	// close file
	fclose(file);

	// return vector
	return vec;
}
