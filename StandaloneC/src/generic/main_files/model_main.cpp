/*
 * Top-level function of the model if using C++ code.
 *
 * author: Nicolas Van der Noot
 */

#ifdef CXX 

extern "C" int C_main(int, char**); 

int main(int argc, char** argv) 
{ 
	return C_main(argc, argv); 
}

#endif
