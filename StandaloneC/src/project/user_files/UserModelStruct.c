//---------------------------
// C-code automatically generated from Gen_mds_user project
//
//
// Last update : Thu Oct 23 11:42:04 2014
//---------------------------




#include "UserModelStruct.h"

// ============================================================ //


UserModelStruct* init_UserModelStruct() 
{
    UserModelStruct* ums;
    ums = (UserModelStruct*)malloc(sizeof(UserModelStruct));
    ums->Actuator.Motor = get_int_vec(58+1);
    ums->Actuator.Motor[0] = 58+1;
 
    return ums;
}

void free_UserModelStruct(UserModelStruct* ums) 
{
    free_int_vec(ums->Actuator.Motor);
    free(ums);
}

 void load_UserModelStruct(MDS_gen_strct* gen, UserModelStruct* ums) 
{

    int ind;
    int ind_state_value = 0;

    for(ind=0; ind<gen->user_models->user_model_list[0]->parameter_list[0]->n_value; ind++)
    {
        ums->Actuator.Motor[ind] = ind_state_value;
        ind_state_value++;
    }
 
}

// ============================================================ //
 
