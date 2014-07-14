/*
 * Contact properties definitions 
 *
 * authors: Alexandra Zobova & Timothee Habra
 */

#if defined(SIMBODY) & defined(__cplusplus)

#include "simbody_cpp_functions.h"
#include "SimbodyStruct.h"
#include <string>

/*
 * Defines bodies meshes for contact
 */
int fill_bodies_contact_properties(ContactPropertiesStruct* BodyContProp, int NumberofBodies)
{
	// -- Left foot -- //

	ContactPropertiesStruct* CurBodyContProp;
	
	 CurBodyContProp = &(BodyContProp[0]); // Left leg
	// set all the mechanical parameters of the contact
	CurBodyContProp->ud = 0.9;   // dynamic   dry friction coefficient
	CurBodyContProp->us = 1.1;   // static;  it is required ud < us
	CurBodyContProp->uv = 0;     // viscous (force/velocity)
	CurBodyContProp->c  = 0.1;  //0.1/100.; // dissipation (1/v)
	CurBodyContProp->k  = 1e5;   //1e5/20.;  // stiffness (pascals)
	CurBodyContProp->thickness = 0.001/5;

	CurBodyContProp->ScaleFactor = 0.03937;
	
	CurBodyContProp->Transform[0] = 0;
	CurBodyContProp->Transform[1] = -0.0726;
	CurBodyContProp->Transform[2] = -0.0932+0.1;

	CurBodyContProp->Rotation[0] = 0; 
	CurBodyContProp->Rotation[1] = 0;
	CurBodyContProp->Rotation[2] = -180;

	sprintf(CurBodyContProp->FileName, "fCOLLADAsimpleL.obj"); // file should be in the folder src/project/simulation_files/Simbody/mesh_obj
	

	// -- Right foot -- //

	CurBodyContProp = &(BodyContProp[1]); // Right leg

	// set all the mechanical parameters of the contact
	CurBodyContProp->ud = 0.9;   // dynamic   dry friction coefficient
	CurBodyContProp->us = 1.1;   // static;  it is required ud < us
	CurBodyContProp->uv = 0;     // viscous (force/velocity)
	CurBodyContProp->c  = 0.1;  //0.1/100.; // dissipation (1/v)
	CurBodyContProp->k  = 1e5;  //1e5/20.;  // stiffness (pascals)
	CurBodyContProp->thickness = 0.001/5;

	CurBodyContProp->ScaleFactor = 0.03937;
	
	CurBodyContProp->Transform[0] = 0;
	CurBodyContProp->Transform[1] = 0.0726;
	CurBodyContProp->Transform[2] = -0.0932+0.1;

	CurBodyContProp->Rotation[0] = 0; 
	CurBodyContProp->Rotation[1] = 0;
	CurBodyContProp->Rotation[2] = -180;

	CurBodyContProp->Geometry = 1; // 1 means a mesh

	sprintf(CurBodyContProp->FileName, "fCOLLADAsimpleR.obj"); // file should be in the folder src/project/simulation_files/Simbody/mesh_obj

	return 0;	
}

/*
 * Defines ground meshes for contact
 */
int fill_ground_contact_properties(ContactPropertiesStruct* BodyContProp)
{
	// set all the mechanical parameters of the contact
	BodyContProp->ud = 0.9;   // dynamic   dry friction coefficient
	BodyContProp->us = 1.1;   // static;  it is required ud < us
	BodyContProp->uv = 0;     // viscous (force/velocity)
	BodyContProp->c  = 0.1;  // dissipation (1/v)
	BodyContProp->k  = 1e5;   // stiffness (pascals)
	BodyContProp->thickness = 0.001;
	
	sprintf(BodyContProp->FileName, "ground_mine.obj");  // file should be in the folder src/project/simulation_files/Simbody/mesh_obj

	BodyContProp->Geometry = 0; // 0 means OneHalfSpace  z+; 1 means a mesh

	if (BodyContProp->Geometry)
	{
		BodyContProp->ScaleFactor = 1;
		
		BodyContProp->Transform[0] = -2.5;
		BodyContProp->Transform[1] = -8;
		BodyContProp->Transform[2] = -0.1;

		BodyContProp->Rotation[0] = 90;
		BodyContProp->Rotation[1] = 0;
		BodyContProp->Rotation[2] = 0;
	}
		
	return 0;
}
#endif
