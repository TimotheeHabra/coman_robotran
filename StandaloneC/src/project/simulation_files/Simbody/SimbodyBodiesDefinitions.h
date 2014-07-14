/*
 * Simbody bodies definition
 *
 * authors: Alexandra Zobova & Timothee Habra
 */

#ifdef SIMBODY

// number of bodies with contact
#define NB_CONTACT_BODIES 2

// S sensors
#define S_RFOOTS 15   
#define S_LFOOTS 27

// F sensors
#define RFOOT_FSENS_ID 1
#define LFOOT_FSENS_ID 2

// sensors tabulars
#define S_SENSORS_ARRAY {S_LFOOTS, S_RFOOTS}
#define F_SENSORS_ARRAY {LFOOT_FSENS_ID, RFOOT_FSENS_ID}

// path to the .obj mesh files
#define BODIES_OBJ_PATH PROJECT_ABS_PATH,"/src/project/simulation_files/Simbody/mesh_obj"

#endif
