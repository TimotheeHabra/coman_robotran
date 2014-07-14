/*
 * Functions used to generate 'ControllersStruct.c', 'ControllersStruct.h'
 * 'user_sf_IO.c' and 'user_sf_IO.h'
 *
 * author: Nicolas Van der Noot
 */
#include "print_c_files.h"
#include "useful_functions.h"

#include <time.h>
#include <string.h>

/*
 * Returns the current time as a String
 *
 * source: http://en.wikipedia.org/wiki/C_date_and_time_functions
 */
char* get_time_machine()
{
	time_t cur_t;
    char* t_string;
 
    // obtain current time as seconds elapsed since the Epoch
    cur_t = time(NULL);
 
    if (cur_t == ((time_t)-1))
    {
        (void) fprintf(stderr, "Failure to compute the current time.");
        return NULL;
    }
 
    // convert to local time format
    t_string = ctime(&cur_t);
 
    if (t_string == NULL)
    {
        (void) fprintf(stderr, "Failure to convert the current time.");
        return NULL;
    }

    return t_string;
}

/*
 * Generates 'ControllersStruct.c' and 'ControllersStruct.h'
 */
void print_c_ctrl_variables(int nb_ctrl, int *nb_var_ctrl, char ***varNameTab, char ***typeTab, 
	int **varSizeTab, int ***twoVarSizeTab, char **namesCtrl, char *fileoutC, char *fileoutH)
{
	int i, j, k;

	int found_single_tab, found_multi_tab;

	FILE *fidC, *fidH;

	// File declaration
    fidC = NULL; // internal filename

    // Opening file
    fidC = fopen(fileoutC, "wt"); 

    // Fill the file
    if(fidC == NULL)
    {
        printf("error: cannot open file '%s'\n", fileoutC);
        exit(1);
    }  

    // File declaration
    fidH = NULL; // internal filename

    // Opening file
    fidH = fopen(fileoutH, "wt"); 

    // Fill the file
    if(fidH == NULL)
    {
        printf("error: cannot open file '%s'\n", fileoutH);
        exit(1);
    }  

    // -- ControllerStruct.c -- //

    fprintf(fidC, "//---------------------------\n");
	fprintf(fidC, "// Nicolas Van der Noot\n");
	fprintf(fidC, "//\n");
	fprintf(fidC, "// Creation : 24-Jan-2014\n");
	fprintf(fidC, "// Last update : %s", get_time_machine());
	fprintf(fidC, "//---------------------------\n\n");
	fprintf(fidC, "#include <stdlib.h>\n\n");
	fprintf(fidC, "#include \"ControllersStruct.h\"\n");
	fprintf(fidC, "\n\n");

	fprintf(fidC,"// ---- Controllers initialization ---- //\n \n");

	// init
	for(k=0; k<nb_ctrl; k++) // loop on all the controllers
	{	    
	    fprintf(fidC,"// %s\n", namesCtrl[k]);
	    fprintf(fidC,"%s * init_%s(void)\n", namesCtrl[k], namesCtrl[k]);
	    fprintf(fidC,"{\n");
	    fprintf(fidC,"    %s *cvs;\n\n", namesCtrl[k]);
	    
	    found_single_tab = 0;
	    found_multi_tab = 0;

	    for(i=0; i<nb_var_ctrl[k]; i++) // loop on this controller variables
	    {
	    	if (!varSizeTab[k][i])
	    	{
	    		found_multi_tab = 1;
	    		break;
	    	}
	    	else if (varSizeTab[k][i] > 1)
	    	{
	    		found_single_tab = 1;
	    	}
	    }

	    if (found_multi_tab)
	    {	    	
	        fprintf(fidC,"    int i, j;\n\n");
	    }
	    else if (found_single_tab)
	    {
	        fprintf(fidC,"    int i;\n\n");
	    }
	    
	    fprintf(fidC,"    cvs = (%s*) malloc(sizeof(%s));\n\n", namesCtrl[k], namesCtrl[k]); 

	    for(i=0; i<nb_var_ctrl[k]; i++) // loop on this controller variables
	    {
	    	if (!varSizeTab[k][i]) // 2-entries tabular
	    	{
	    		fprintf(fidC,"    for (i=0;i<%d;i++)\n", twoVarSizeTab[k][i][0]);
				fprintf(fidC,"    {\n");
				fprintf(fidC,"        for (j=0;j<%d;j++)\n", twoVarSizeTab[k][i][1]);
				fprintf(fidC,"        {\n");

				if ( !strcmp(typeTab[k][i], "int") )
	    		{
	    			fprintf(fidC,"            cvs->%s[i][j] = 0;\n", varNameTab[k][i]);
	    		}
	    		else if ( !strcmp(typeTab[k][i], "double") )
	    		{
	    			fprintf(fidC,"            cvs->%s[i][j] = 0.0;\n", varNameTab[k][i]);
	    		}
	    		else
	    		{
	    			fprintf(fidC,"            cvs->%s[i][j] = init_%s();\n", varNameTab[k][i], typeTab[k][i]);
	    		}

				fprintf(fidC,"        }\n");
				fprintf(fidC,"    }\n\n");
	    	}
	    	else if (varSizeTab[k][i] == 1) // single variables
	    	{
	    		if ( !strcmp(typeTab[k][i], "int") )
	    		{
	    			fprintf(fidC,"    cvs->%s = 0;\n\n", varNameTab[k][i]); 
	    		}
	    		else if ( !strcmp(typeTab[k][i], "double") )
	    		{
	    			fprintf(fidC,"    cvs->%s = 0.0;\n\n", varNameTab[k][i]); 
	    		}
	    		else
	    		{
	    			fprintf(fidC,"    cvs->%s = init_%s();\n\n", varNameTab[k][i], typeTab[k][i]);
	    		}
	    	}
	    	else if (varSizeTab[k][i] > 1) // single-entry tabular
	    	{
	    		fprintf(fidC,"    for (i=0;i<%d;i++)\n", varSizeTab[k][i]);
				fprintf(fidC,"    {\n");

				if ( !strcmp(typeTab[k][i], "int") )
	    		{
	    			fprintf(fidC,"        cvs->%s[i] = 0;\n", varNameTab[k][i]);
				}
	    		else if ( !strcmp(typeTab[k][i], "double") )
	    		{
	    			fprintf(fidC,"        cvs->%s[i] = 0.0;\n", varNameTab[k][i]);
	    		}
	    		else
	    		{
	    			fprintf(fidC,"        cvs->%s[i] = init_%s();\n", varNameTab[k][i], typeTab[k][i]);
	    		}

				fprintf(fidC,"    }\n\n");
	    	}	    	
	    }

	    fprintf(fidC,"    return cvs;\n");
	    fprintf(fidC,"}\n\n");
	}

	// free
	fprintf(fidC,"// ---- Controllers: free ---- //\n\n");
	for(k=0; k<nb_ctrl; k++) // loop on all the controllers
	{	
	    fprintf(fidC,"// %s\n", namesCtrl[k]);
	    fprintf(fidC,"void free_%s(%s *cvs)\n", namesCtrl[k], namesCtrl[k]);
	    fprintf(fidC,"{\n");

	    found_single_tab = 0;
	    found_multi_tab = 0;

	    for(i=0; i<nb_var_ctrl[k]; i++) // loop on this controller variables
	    {
	    	if ( ( strcmp(typeTab[k][i], "int") ) && ( strcmp(typeTab[k][i], "double") ) ) // structure
	    	{
		    	if (!varSizeTab[k][i])
		    	{
		    		found_multi_tab = 1;
		    		break;
		    	}
		    	else if (varSizeTab[k][i] > 1)
		    	{
		    		found_single_tab = 1;
		    	}
		    }
	    }

	    if (found_multi_tab)
	    {	    	
	        fprintf(fidC,"    int i, j;\n\n");
	    }
	    else if (found_single_tab)
	    {
	        fprintf(fidC,"    int i;\n\n");
	    }

		for(i=0; i<nb_var_ctrl[k]; i++) // loop on this controller variables
	    {
	    	if ( ( strcmp(typeTab[k][i], "int") ) && ( strcmp(typeTab[k][i], "double") ) ) // structure
	    	{
	    		if (!varSizeTab[k][i]) // 2-entries tabular
		    	{
		    		fprintf(fidC,"    for (i=0;i<%d;i++)\n", twoVarSizeTab[k][i][0]);
					fprintf(fidC,"    {\n");
					fprintf(fidC,"        for (j=0;j<%d;j++)\n", twoVarSizeTab[k][i][1]);
					fprintf(fidC,"        {\n");
		    		fprintf(fidC,"            free_%s(cvs->%s[i][j]);\n", typeTab[k][i], varNameTab[k][i]);
					fprintf(fidC,"        }\n");
					fprintf(fidC,"    }\n\n");
		    	}
		    	else if (varSizeTab[k][i] == 1) // single variables
		    	{
		    		fprintf(fidC,"    free_%s(cvs->%s);\n\n", typeTab[k][i], varNameTab[k][i]);
		    	}
		    	else if (varSizeTab[k][i] > 1) // single-entry tabular
		    	{
		    		fprintf(fidC,"    for (i=0;i<%d;i++)\n", varSizeTab[k][i]);
					fprintf(fidC,"    {\n");
		    		fprintf(fidC,"        free_%s(cvs->%s[i]);\n", typeTab[k][i], varNameTab[k][i]);
					fprintf(fidC,"    }\n\n");
		    	}
	    	}
	    }

	    fprintf(fidC,"    free(cvs);\n");
	    fprintf(fidC,"}\n\n");
	}

	fclose(fidC);
	printf(">> 'ControllerStruct.c' created\n");


	// -- ControllerStruct.h -- //

	fprintf(fidH,"//---------------------------\n");
	fprintf(fidH,"// Nicolas Van der Noot\n");
	fprintf(fidH,"//\n");
	fprintf(fidH,"// Creation : 19-Sep-2013\n");
	fprintf(fidH, "// Last update : %s", get_time_machine());
	fprintf(fidH,"//---------------------------\n\n");
	fprintf(fidH,"#ifndef ControllerStruct_h\n");
	fprintf(fidH,"#define ControllerStruct_h\n");
	fprintf(fidH,"\n\n");
	fprintf(fidH,"// ---- Structures definitions (typedef) ---- //\n\n");


	for(k=0; k<nb_ctrl; k++) // loop on all the controllers
	{	  
	    fprintf(fidH,"// %sStruc\n", namesCtrl[k]);
	    fprintf(fidH,"typedef struct %s\n", namesCtrl[k]);
	    fprintf(fidH,"{\n");

	    for(i=0; i<nb_var_ctrl[k]; i++) // loop on this controller variables
	    {
	    	if ( ( !strcmp(typeTab[k][i], "int") ) || ( !strcmp(typeTab[k][i], "double") ) )
	    	{
	    		if (!varSizeTab[k][i]) // 2-entries tabular
		    	{
		    		fprintf(fidH,"    %s %s[%d][%d];\n", typeTab[k][i], varNameTab[k][i], twoVarSizeTab[k][i][0], twoVarSizeTab[k][i][1]);
		    	}
		    	else if (varSizeTab[k][i] == 1) // single variables
		    	{
		    		fprintf(fidH,"    %s %s;\n", typeTab[k][i], varNameTab[k][i]);
		    	}
		    	else if (varSizeTab[k][i] > 1) // single-entry tabular (vector)
		    	{
		    		fprintf(fidH,"    %s %s[%d];\n", typeTab[k][i], varNameTab[k][i], varSizeTab[k][i]);
		    	}
	    	}
	    	else
	    	{
	    		if (!varSizeTab[k][i]) // 2-entries tabular
		    	{
		    		fprintf(fidH,"    struct %s *%s[%d][%d];\n", typeTab[k][i], varNameTab[k][i], twoVarSizeTab[k][i][0], twoVarSizeTab[k][i][1]);
		    	}
		    	else if (varSizeTab[k][i] == 1) // single variables
		    	{
		    		fprintf(fidH,"    struct %s *%s;\n", typeTab[k][i], varNameTab[k][i]);
		    	}
		    	else if (varSizeTab[k][i] > 1) // single-entry tabular (vector)
		    	{
		    		fprintf(fidH,"    struct %s *%s[%d];\n", typeTab[k][i], varNameTab[k][i], varSizeTab[k][i]);
		    	}
	    	}

	    	if (varSizeTab[k][i] <= -1) // pointers
	    	{
	    		fprintf(fidH,"    %s ", typeTab[k][i]);
	    		for(j=varSizeTab[k][i]; j<0; j++)
	    		{
	    			fprintf(fidH,"*");
	    		}
	    		fprintf(fidH,"%s;\n", varNameTab[k][i]);
	    	}		    	
	    }
	    fprintf(fidH,"\n} %s;\n\n\n", namesCtrl[k]);
	}

	fprintf(fidH,"// ---- Init and free functions: declarations ---- //\n\n");

	for(k=0; k<nb_ctrl; k++) // loop on all the controllers
	{	
		fprintf(fidH,"%s * init_%s(void);\n", namesCtrl[k], namesCtrl[k]);
    	fprintf(fidH,"void free_%s(%s *cvs);\n\n", namesCtrl[k], namesCtrl[k]);
	}

	fprintf(fidH,"/*--------------------*/\n");
	fprintf(fidH,"#endif\n\n");

	fclose(fidH);
	printf(">> 'ControllerStruct.h' created\n");
}

/*
 * Generates 'user_sf_IO.c' and 'user_sf_IO.h'
 */
void print_c_simu_variables(int *nb_var_simu, char ***varNameTab, char ***typeTab, 
	int **varSizeTab, int ***twoVarSizeTab, char *fileoutC, char *fileoutH)
{
	int i, j, k;
	int found_single_tab, found_multi_tab;

	FILE *fidC, *fidH; 

	// File declaration
    fidC = NULL; // internal filename

    // Opening file
    fidC = fopen(fileoutC, "wt"); 

    // Fill the file
    if(fidC == NULL)
    {
        printf("error: cannot open file '%s'\n", fileoutC);
        exit(1);
    }  

    // File declaration
    fidH = NULL; // internal filename

    // Opening file
    fidH = fopen(fileoutH, "wt"); 

    // Fill the file
    if(fidH == NULL)
    {
        printf("error: cannot open file '%s'\n", fileoutH);
        exit(1);
    }  

    // -- user_sf_IO.c -- //

    // header
	fprintf(fidC, "/*===========================================================================*\n");
	fprintf(fidC, " *\n");
	fprintf(fidC, " *  user_sf_IO.c\n");
	fprintf(fidC, " * \n");
	fprintf(fidC, " *  Generation date: %s\n", get_time_machine());
	fprintf(fidC, " * \n");
	fprintf(fidC, " *  (c) Universite catholique de Louvain\n");
	fprintf(fidC, " *      Departement de Mecanique \n");
	fprintf(fidC, " *      Unite de Production Mecanique et Machines \n");
	fprintf(fidC, " *      2, Place du Levant \n");
	fprintf(fidC, " *      1348 Louvain-la-Neuve \n");
	fprintf(fidC, " *  http://www.robotran.be// \n");
	fprintf(fidC, " *  \n");
	fprintf(fidC, "/*===========================================================================*/\n\n");

	// begin file
    fprintf(fidC, "#include \"MBSfun.h\" \n");
    fprintf(fidC, "#include \"user_sf_IO.h\" \n");
    fprintf(fidC, "#include \"sfdef.h\" \n");
    fprintf(fidC, "#include \"userDef.h\"\n");
    fprintf(fidC, "#include \"ControllersStruct.h\"\n\n");

	fprintf(fidC, "\n");
	fprintf(fidC, "UserIOStruct * initUserIO(MBSdataStruct *s)\n");
	fprintf(fidC, "{\n");
	fprintf(fidC, "    UserIOStruct *uvs;\n\n");

	found_single_tab = 0;
	found_multi_tab = 0;

    for(i=0; i<3; i++) // lopp on all the simulation variables (except the structures)
	{
		for(j=0; j<nb_var_simu[i]; j++) // loop on all these variables
		{
			if (!varSizeTab[i][j])
			{
				found_multi_tab = 1;
	    		break;
			}
			else if (varSizeTab[i][j] > 1) 
			{
				found_single_tab = 1;
			}
		}
	}

	if (found_multi_tab)
    {	    	
        fprintf(fidC,"    int i, j;\n");
    }
    else if (found_single_tab)
    {
        fprintf(fidC,"    int i;\n");
    }

	fprintf(fidC, "\n");
	fprintf(fidC, "    uvs = (UserIOStruct*) malloc(sizeof(UserIOStruct));\n");

	for(i=0; i<3; i++) // lopp on all the simulation variables (except the structures)
	{
		for(j=0; j<nb_var_simu[i]; j++) // loop on all these variables
		{
			fprintf(fidC, "\n    // %s //\n", varNameTab[i][j]);


			if (!varSizeTab[i][j]) // 2-entries tabular
	    	{
	    		fprintf(fidC,"    for (i=1;i<=%d;i++)\n", twoVarSizeTab[i][j][0]);
				fprintf(fidC,"    {\n");
				fprintf(fidC,"        for (j=1;j<=%d;j++)\n", twoVarSizeTab[i][j][1]);
				fprintf(fidC,"        {\n");

				if ( !strcmp(typeTab[i][j], "int") )
	    		{
	    			fprintf(fidC,"            uvs->%s[i][j] = 0;\n", varNameTab[i][j]);
	    		}
	    		else if ( !strcmp(typeTab[i][j], "double") )
	    		{
	    			fprintf(fidC,"            uvs->%s[i][j] = 0.0;\n", varNameTab[i][j]);
	    		}
	    		else
	    		{
	    			fprintf(fidC,"            uvs->%s[i][j] = init_%s();\n", varNameTab[i][j], typeTab[i][j]);
	    		}

				fprintf(fidC,"        }\n");
				fprintf(fidC,"    }\n");
	    	}
	    	else if (varSizeTab[i][j] == 1) // single variables
			{
				if ( !strcmp(typeTab[i][j], "int") )
				{
					fprintf(fidC, "    uvs->%s = 0;\n", varNameTab[i][j]);
				}
				else
				{
					fprintf(fidC, "    uvs->%s = 0.0;\n", varNameTab[i][j]);
				}
			}
			else if (varSizeTab[i][j] > 1) // vector (single-entry tabular)
			{
				fprintf(fidC,"    for (i=1;i<=%d;i++)\n", varSizeTab[i][j]);
                fprintf(fidC,"    {\n");
                if ( !strcmp(typeTab[i][j], "int") )
				{
                	fprintf(fidC,"        uvs->%s[i] = 0;\n", varNameTab[i][j]);
                }
                else
                {
                	fprintf(fidC,"        uvs->%s[i] = 0.0;\n", varNameTab[i][j]);
                }
                fprintf(fidC,"    }\n");
			}
		}
	}

	for(j=0; j<nb_var_simu[3]; j++) // loop on all the variables of the strucures part
	{
		fprintf(fidC,"\n");
	    fprintf(fidC,"    // %s //\n", varNameTab[3][j]);

	    if (!varSizeTab[3][j]) // 2-entries tabular
    	{
    		fprintf(fidC,"    for (i=1;i<=%d;i++)\n", twoVarSizeTab[3][j][0]);
			fprintf(fidC,"    {\n");
			fprintf(fidC,"        for (j=1;j<=%d;j++)\n", twoVarSizeTab[3][j][1]);
			fprintf(fidC,"        {\n");
			fprintf(fidC,"            uvs->%s[i][j] = init_%s();\n", varNameTab[3][j], typeTab[3][j]);
			fprintf(fidC,"        }\n");
			fprintf(fidC,"    }\n");
    	}
	    else if (varSizeTab[3][j] == 1) // single variables
		{
			fprintf(fidC,"    uvs->%s = init_%s();\n", varNameTab[3][j], typeTab[3][j]);
		}
		else if (varSizeTab[3][j] > 1) // vector (single-entry tabular)
		{
			fprintf(fidC,"    for (i=1;i<=%d;i++)\n", varSizeTab[3][j]);
			fprintf(fidC,"    {\n");
			fprintf(fidC,"        uvs->%s[i] = init_%s();\n", varNameTab[3][j], typeTab[3][j]);
			fprintf(fidC,"    }\n");
		}
	}

	// stop simulation
	fprintf(fidC,"    // stop simulation //\n");
	fprintf(fidC,"    uvs->stop_simu = 0;\n");
	fprintf(fidC,"\n");

	// Simbody
	fprintf(fidC,"    // simbodyStruct //\n");
	fprintf(fidC,"    #ifdef SIMBODY\n");
	fprintf(fidC,"    uvs->simbodyStruct = init_SimbodyStruct();\n");
	fprintf(fidC,"    #endif\n");
	fprintf(fidC,"\n");

	fprintf(fidC,"    return uvs;\n");
	fprintf(fidC,"}\n");
	fprintf(fidC,"\n\n");
	fprintf(fidC,"void freeUserIO(UserIOStruct *uvs, MBSdataStruct *s)\n");
	fprintf(fidC,"{\n"); 

	found_single_tab = 0;
    found_multi_tab  = 0;

    for(j=0; j<nb_var_simu[3]; j++) // loop on all the structure instances
    {
    	if ( ( strcmp(typeTab[3][j], "int") ) && ( strcmp(typeTab[3][j], "double") ) ) // structure
    	{
	    	if (!varSizeTab[3][j])
	    	{
	    		found_multi_tab = 1;
	    		break;
	    	}
	    	else if (varSizeTab[3][j] > 1)
	    	{
	    		found_single_tab = 1;
	    	}
	    }
    }

    if (found_multi_tab)
    {	    	
        fprintf(fidC,"    int i, j;\n");
    }
    else if (found_single_tab)
    {
        fprintf(fidC,"    int i;\n");
    }

	for(j=0; j<nb_var_simu[3]; j++) // loop on all the variables of the strucures part
	{
		fprintf(fidC,"\n");
        fprintf(fidC,"    // %s: %s //\n", typeTab[3][j], varNameTab[3][j]);


        if (!varSizeTab[3][j]) // 2-entries tabular
    	{
    		fprintf(fidC,"    for (i=1;i<=%d;i++)\n", twoVarSizeTab[3][j][0]);
			fprintf(fidC,"    {\n");
			fprintf(fidC,"        for (j=1;j<=%d;j++)\n", twoVarSizeTab[3][j][1]);
			fprintf(fidC,"        {\n");
    		fprintf(fidC,"            free_%s(uvs->%s[i][j]);\n", typeTab[3][j], varNameTab[3][j]);
			fprintf(fidC,"        }\n");
			fprintf(fidC,"    }\n");
    	}
    	else if (varSizeTab[3][j] == 1) // single variables
		{
			fprintf(fidC,"    free_%s(uvs->%s);\n", typeTab[3][j], varNameTab[3][j]);
		}
		else if (varSizeTab[3][j] > 1) // vector (single-entry tabular)
		{
			fprintf(fidC,"    for (i=1;i<=%d;i++)\n", varSizeTab[3][j]);
			fprintf(fidC,"    {\n");
			fprintf(fidC,"        free_%s(uvs->%s[i]);\n", typeTab[3][j], varNameTab[3][j]);
			fprintf(fidC,"    }\n");
		}        
	}

	// Simbody
	fprintf(fidC,"\n");
	fprintf(fidC,"    // SimbodyStruct: simbodyStruct //\n");
	fprintf(fidC,"    #ifdef SIMBODY\n");
	fprintf(fidC,"    free_SimbodyStruct(uvs->simbodyStruct);\n");
	fprintf(fidC,"    #endif\n");
	fprintf(fidC,"\n");

	fprintf(fidC,"    free(uvs);\n");
	fprintf(fidC,"}\n\n");

    fclose(fidC);
	printf(">> 'user_sf_IO.c' created\n");


	// -- user_sf_IO.h -- //

	// header
	fprintf(fidH, "/*===========================================================================*\n");
	fprintf(fidH, " *\n");
	fprintf(fidH, " *  user_sf_IO.h\n");
	fprintf(fidH, " * \n");
	fprintf(fidH, " *  Generation date: %s\n", get_time_machine());
	fprintf(fidH, " * \n");
	fprintf(fidH, " *  (c) Universite catholique de Louvain\n");
	fprintf(fidH, " *      Departement de Mecanique \n");
	fprintf(fidH, " *      Unite de Production Mecanique et Machines \n");
	fprintf(fidH, " *      2, Place du Levant \n");
	fprintf(fidH, " *      1348 Louvain-la-Neuve \n");
	fprintf(fidH, " *  http://www.robotran.be// \n");
	fprintf(fidH, " *  \n");
	fprintf(fidH, "/*===========================================================================*/\n\n");

	// Begin file
	fprintf(fidH, "#ifndef UsersfIO_h\n");
	fprintf(fidH, "#define UsersfIO_h\n");
	fprintf(fidH, "/*--------------------*/\n \n");

   
    fprintf(fidH, "#include \"userDef.h\"\n");
    fprintf(fidH, "#include \"ControllersStruct.h\"\n");
    fprintf(fidH, " \n");

	fprintf(fidH, "typedef struct UserIOStruct \n");
	fprintf(fidH, "{\n");

	for(i=0; i<3; i++) // lopp on all the simulation variables (except the structures)
	{
		for(j=0; j<nb_var_simu[i]; j++) // loop on all these variables
		{
			if (!varSizeTab[i][j]) // 2-entries tabular
	    	{
	    		fprintf(fidH,"    %s %s[%d+1][%d+1];\n", typeTab[i][j], varNameTab[i][j], twoVarSizeTab[i][j][0], twoVarSizeTab[i][j][1]);
	    	}
	    	else if (varSizeTab[i][j] == 1) // single variables
			{
				fprintf(fidH, "    %s %s;\n", typeTab[i][j], varNameTab[i][j]);
			}
			else if (varSizeTab[i][j]  > 1) // single-entry tabular (vector)
			{
				fprintf(fidH, "    %s %s[%d+1];\n", typeTab[i][j], varNameTab[i][j], varSizeTab[i][j]);
			}
			else if (varSizeTab[i][j] <= -1) // pointers
	    	{
	    		fprintf(fidH,"    %s ", typeTab[i][j]);
	    		for(k=varSizeTab[i][j]; k<0; k++)
	    		{
	    			fprintf(fidH,"*");
	    		}
	    		fprintf(fidH,"%s;\n", varNameTab[i][j]);
	    	}
		}
	}

	for (j=0; j<nb_var_simu[3]; j++) // loop on all the variables of the strucures part
	{
		if (!varSizeTab[3][j]) // 2-entries tabular
		{
			fprintf(fidH,"    struct %s *%s[%d+1][%d+1];\n", typeTab[3][j], varNameTab[3][j], twoVarSizeTab[3][j][0], twoVarSizeTab[3][j][1]);
		}
		else if (varSizeTab[3][j] == 1) // single variables
		{
			fprintf(fidH, "    struct %s *%s;\n", typeTab[3][j], varNameTab[3][j]);
		}
		else if (varSizeTab[3][j] > 1) // single-entry tabular (vector)
		{
			fprintf(fidH, "    struct %s *%s[%d+1];\n", typeTab[3][j], varNameTab[3][j], varSizeTab[3][j]);
		}
		else if (varSizeTab[3][j] <= -1) // pointers
    	{
    		fprintf(fidH,"    %s ", typeTab[3][j]);
    		for(j=varSizeTab[3][j]; j<0; j++)
    		{
    			fprintf(fidH,"*");
    		}
    		fprintf(fidH,"%s;\n", varNameTab[3][j]);
    	}
	}

	// stop simulation
	fprintf(fidH,"\n");
	fprintf(fidH,"    // stop simulation //\n");
	fprintf(fidH,"    int stop_simu;\n");
	fprintf(fidH,"\n");

	// Simbody
	fprintf(fidH, "    #ifdef SIMBODY\n");
	fprintf(fidH, "    SimbodyStruct *simbodyStruct;\n");
	fprintf(fidH, "    #endif\n");
	fprintf(fidH, "\n");

	fprintf(fidH, "} UserIOStruct;\n");

	fprintf(fidH, "\n");
	fprintf(fidH, "/*--------------------*/\n");
	fprintf(fidH, "#endif\n");

	fclose(fidH);
	printf(">> 'user_sf_IO.h' created\n");
}
