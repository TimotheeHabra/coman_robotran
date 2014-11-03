#
# author: Nicolas Van der Noot
#
# Creation:    Feb 27 2014
# Last update: May 18 2014
#
# Main CMakeLists, used to configure the project
#


## ---- FLAGS ----  ##


# compile with C++
option (FLAG_CXX_PROJECT  
        "Use C++" OFF) 

# let the use choose the compiler
option (FLAG_COMPILER_PROPERTIES  
        "Modify compilers properties" ON) 

# use dirdynared
option (FLAG_DIRDYNARED 
        "Dirdynared" ON) 

# use odn
option (FLAG_ODN_METHOD 
        "Odn" OFF) 

# use accelred
option (FLAG_ACCELRED 
        "Accelred" OFF) 

# print messages in the terminal...
option (FLAG_PRINT_REPORT  
        "Print Report" ON) 

# write the .anim file and the debug .txt vectors
option (FLAG_WRITE_OUTPUT_FILES 
        "Write Files" OFF) 
 
# automatic generation of the 'ControllersStruct.h/.c' and 'user_sf_IO.h/.c' files
option (FLAG_GENERATE_VARSTATE 
        "Generate ControllersStruct, user_sf_IO" ON) 

# run the simulation in real-time
option (FLAG_REAL_TIME 
        "Real time" ON) 

# use the SDL library to plot real-time graphs and to handle the user inputs (keyboard...)
option (FLAG_SDL_INTERACTIONS 
        "SDL" ON) 

# use JNI (Java Native Interface) to see the simulation in real-time
option (FLAG_JAVA_VISUALIZATION 
        "JNI" ON)

# run the simulation in real-time
option (FLAG_GROUND_CONTACT_MODEL 
        "Use custom-made GCM" ON) 

# run the simulation in real-time
option (FLAG_STOP_SIMULATION 
        "Stop simulation" ON) 

# Simbody
option (FLAG_SIMBODY 
        "Simbody" OFF) 

# 3D model
option (FLAG_3D_MODEL 
        "3D model" OFF) 

# Release flag
option (FLAG_RELEASE_VERSION 
        "Release flag" ON) 

# Adaptive time step
option (FLAG_ADAPTIVE_TIME_STEP 
        "Adaptive time step" OFF) 


## ---- COMAN VERSIONS ----  ##

# One of the following lines must be uncommented,
# all the others must be commented.
#
# Basic_version  : Rigid rounded feet (on the ground)
# Short_feet     : Rigid short rectangular feet
# Compliant_feet : Compliant feet (two rigid plates connected by a spring)
# Long arms      : CoMan with long arms ans hands
# SkyWalker      : Rigid rounded feet hung with two ropes

#set(COMAN_VERSION "Basic_version")
#set(COMAN_VERSION "Short_feet")
#set(COMAN_VERSION "Compliant_feet")
set(COMAN_VERSION "Long_arms")
#set(COMAN_VERSION "SkyWalker")

## ---- FLAGS DEPENDENCIES ----  ##

if (NOT FLAG_CXX_PROJECT)
	set(FLAG_SIMBODY OFF)
endif (NOT FLAG_CXX_PROJECT)

if (NOT FLAG_REAL_TIME)
	set(FLAG_SDL_INTERACTIONS OFF)
	set(FLAG_JAVA_VISUALIZATION OFF)
endif (NOT FLAG_REAL_TIME)

if (FLAG_DIRDYNARED)
    set(FLAG_ODN_METHOD OFF)
    set(FLAG_ACCELRED OFF)
endif (FLAG_DIRDYNARED)

if (FLAG_ODN_METHOD)
    set(FLAG_ACCELRED OFF)
endif (FLAG_ODN_METHOD)

if (${COMAN_VERSION} STREQUAL "SkyWalker")
	set(FLAG_3D_MODEL ON)
endif (${COMAN_VERSION} STREQUAL "SkyWalker")



## ---- PROJECT MAIN CONFIGURATIONS ---- ##

# CMake minimum version
cmake_minimum_required(VERSION 2.8.7)

# compiler preferences
if (FLAG_COMPILER_PROPERTIES)
	set( CMAKE_C_COMPILER gcc-4.4 )
	set( CMAKE_CXX_COMPILER g++-4.4 )
	set(CMAKE_C_FLAGS_RELEASE "-O3")
	set(CMAKE_CXX_FLAGS_RELEASE "-O3")
endif (FLAG_COMPILER_PROPERTIES)

# project name
project (STANDALONE_C)

# message to display the project name
message(STATUS "Processing ${PROJECT_NAME}")

# define the location of all the 'Find<module>.cmake' files
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/src/other/conf)

if (FLAG_CXX_PROJECT)
	add_definitions( -DCXX )
endif (FLAG_CXX_PROJECT)

if(FLAG_ADAPTIVE_TIME_STEP)
	add_definitions( -DADAPTIVE_TIME_STEP )
endif (FLAG_ADAPTIVE_TIME_STEP)


# for Unix: display all the warnings, except the ones related to /* -- */
if (UNIX)
	set(CMAKE_C_FLAGS "-g -Wall -Wextra -Wno-comment -Wno-unused-parameter")
	set(CMAKE_CXX_FLAGS "-g -Wall -Wextra -Wno-comment -Wno-unused-parameter")
endif (UNIX)

# use Release version and not the Debug one (faster)
if (FLAG_RELEASE_VERSION)
	set(CMAKE_BUILD_TYPE Release)
else (FLAG_RELEASE_VERSION)
	set(CMAKE_BUILD_TYPE Debug)
endif (FLAG_RELEASE_VERSION)

## ---- COMAN VERSIONS SYNCHRONIZATION ---- ##

if (FLAG_3D_MODEL)
        set(MODEL_STANDALONE "Model_standalone_3D")
else (FLAG_3D_MODEL)
        set(MODEL_STANDALONE "Model_standalone_2D")
endif (FLAG_3D_MODEL)

# if different mbs, synchronize version
file(GLOB MBS_SRC "${PROJECT_SOURCE_DIR}/src/other/CoMan_versions/${COMAN_VERSION}/${MODEL_STANDALONE}/dataR/*.mbs") 
file(READ ${MBS_SRC} STRING1)

file(GLOB MBS_DST "${PROJECT_SOURCE_DIR}/../dataR/*.mbs")
if(EXISTS MBS_DST) 
	file(READ ${MBS_DST} STRING2)
else(EXISTS MBS_DST) 
	set(STRING2 " ")
endif(EXISTS MBS_DST) 

if(NOT (STRING1 STREQUAL STRING2))
	file(WRITE ${PROJECT_SOURCE_DIR}/../dataR/copy_check "this file is used to check if the mbs has changed")
	file(COPY ${PROJECT_SOURCE_DIR}/src/other/CoMan_versions/${COMAN_VERSION}/Symbolic_files/ DESTINATION ${PROJECT_SOURCE_DIR}/..)
	file(COPY ${PROJECT_SOURCE_DIR}/src/other/CoMan_versions/${COMAN_VERSION}/${MODEL_STANDALONE}/dataR/ DESTINATION ${PROJECT_SOURCE_DIR}/../dataR)
endif(NOT (STRING1 STREQUAL STRING2))	

## ---- AUTOMATIC GENERATION  ('ControllersStruct.h/.c' and 'user_sf_IO.h/.c')  ---- ##

if (FLAG_GENERATE_VARSTATE)

	# ControllersStruct.h/.c
	set(CONTROLLERSSTRCT_H ${PROJECT_SOURCE_DIR}/src/project/controller_files/ControllersStruct.h)
	set(CONTROLLERSSTRCT_C ${PROJECT_SOURCE_DIR}/src/project/controller_files/ControllersStruct.c)

	# user_sf_IO.h/.c
	set(USER_SF_IO_H ${PROJECT_SOURCE_DIR}/src/project/user_files/user_sf_IO.h)
	set(USER_SF_IO_C ${PROJECT_SOURCE_DIR}/src/project/user_files/user_sf_IO.c)

	# input .txt files
	set(CONTROL_VAR_TXT ${PROJECT_SOURCE_DIR}/src/project/varState/control_variables.txt)
	set(SIMU_VAR_TXT ${PROJECT_SOURCE_DIR}/src/project/varState/simu_variables.txt)
	
	# checking files
	set(CHECK_CTRL ${PROJECT_SOURCE_DIR}/src/project/varState/update_check/ctrl_check.txt)
	set(CHECK_SIMU ${PROJECT_SOURCE_DIR}/src/project/varState/update_check/simu_check.txt)

	# re-generate ControllersStruct.h/.c
	add_custom_command (
	  OUTPUT ${CHECK_CTRL}
	  COMMAND Gen_state_var ctrl
	  DEPENDS ${CONTROL_VAR_TXT}
	)

	# re-generate user_sf_IO.h/.c
	add_custom_command (
	  OUTPUT ${CHECK_SIMU}
	  COMMAND Gen_state_var simu
	  DEPENDS ${SIMU_VAR_TXT}
	)

	# check if re-generation needed
	add_custom_target(CheckGeneration ALL 
		DEPENDS ${CHECK_CTRL} ${CHECK_SIMU})
	
endif (FLAG_GENERATE_VARSTATE)

message(STATUS "CMake generator: ${CMAKE_GENERATOR}")
 


## ---- AUTOMATIC GENERATION  ('user_all_id.h' and 'UserModelStruct.h/.c')  ---- ##
## change for mds_user

	# UserModelStruct.h/.c
	set(USER_MODELS_H ${PROJECT_SOURCE_DIR}/src/project/user_files/UserModelStruct.h)
	set(USER_MODELS_C ${PROJECT_SOURCE_DIR}/src/project/user_files/UserModelStruct.c)
	set(USER_ALL_ID_H ${PROJECT_SOURCE_DIR}/src/project/user_files/user_all_id.h)

	# input .mbs file
	file(GLOB MBS_FILE "${PROJECT_SOURCE_DIR}/../dataR/*.mbs")
	
	# checking files
	set(CHECK_USER_MODELS ${CMAKE_CURRENT_BINARY_DIR}/src/project/user_files/update_check/user_models_check.txt) # these files are actually not created.
	set(CHECK_USER_ALL_ID ${CMAKE_CURRENT_BINARY_DIR}/src/project/user_files/update_check/user_all_id_check.txt) # can't use .c/.h as they are deleted at 'clean'

	# re-generate user_models.h/.c
	add_custom_command (
	  OUTPUT ${CHECK_USER_MODELS}
	  COMMAND Gen_mds_user user_models
	  DEPENDS ${MBS_FILE}
	)
	
	# re-generate user_all_id.h/.c
	add_custom_command (
	  OUTPUT ${CHECK_USER_ALL_ID}
	  COMMAND Gen_mds_user user_all_id
	  DEPENDS ${MBS_FILE}
	)
	
	# check if re-generation needed
	add_custom_target(CheckGeneration_2 ALL 
		DEPENDS ${CHECK_USER_ALL_ID} ${CHECK_USER_MODELS})



## ---- PROHIBITED LIST ---- ##

# Project name in Robotran
file(GLOB_RECURSE ROBOTRAN_ROJECT_NAME "${PROJECT_SOURCE_DIR}/src/other/CoMan_versions/${COMAN_VERSION}/${MODEL_STANDALONE}/dataR/*.mbs")
get_filename_component(ROBOTRAN_ROJECT_NAME ${ROBOTRAN_ROJECT_NAME} NAME_WE)

message(STATUS "Robotran project name: ${ROBOTRAN_ROJECT_NAME}")

# These files will not be compiled or included in the project
set(CUR_ITEM "")

set(PROHIBITION_LIST 
	"mbs_invdyna_${ROBOTRAN_ROJECT_NAME}.c" 
	"mbs_invdyna_${ROBOTRAN_ROJECT_NAME}.h")

if (FLAG_DIRDYNARED)

	set(PROHIBITION_LIST ${PROHIBITION_LIST} 
		"mbs_accelred_${ROBOTRAN_ROJECT_NAME}.c" 
		"mbs_dirdyna_odn_${ROBOTRAN_ROJECT_NAME}.c" 
		"mbs_accelred_${ROBOTRAN_ROJECT_NAME}.h" 
		"mbs_dirdyna_odn_${ROBOTRAN_ROJECT_NAME}.h")

else (FLAG_DIRDYNARED)

    if (FLAG_ODN_METHOD)

        set(PROHIBITION_LIST ${PROHIBITION_LIST} 
        	"mbs_dirdyna_${ROBOTRAN_ROJECT_NAME}.c" 
        	"mbs_accelred_${ROBOTRAN_ROJECT_NAME}.c" 
        	"mbs_dirdyna_${ROBOTRAN_ROJECT_NAME}.h" 
        	"mbs_accelred_${ROBOTRAN_ROJECT_NAME}.h")

    else (FLAG_ODN_METHOD)

        set(PROHIBITION_LIST ${PROHIBITION_LIST} 
        	"mbs_dirdyna_${ROBOTRAN_ROJECT_NAME}.c" 
        	"mbs_dirdyna_odn_${ROBOTRAN_ROJECT_NAME}.c" 
        	"mbs_dirdyna_${ROBOTRAN_ROJECT_NAME}.h" 
        	"mbs_dirdyna_odn_${ROBOTRAN_ROJECT_NAME}.h")

    endif (FLAG_ODN_METHOD)    	

endif (FLAG_DIRDYNARED)


## ---- CUSTOM FUNCTIONS ---- ##


## -- Prohibited list -- ##

# function to return 1 if the element arg0 is present in list 'PROHIBITION_LIST' 
# -> arg1 == 1 if arg0 is part of the prohibited list (and opposite)
function(prohibited_file arg0 arg1)
	set(${arg1} 0 PARENT_SCOPE)
	foreach(LIST_ITEM ${PROHIBITION_LIST})
		if (${LIST_ITEM} STREQUAL ${arg0})
    		set(${arg1} 1 PARENT_SCOPE)
    	endif (${LIST_ITEM} STREQUAL ${arg0})
	endforeach(LIST_ITEM)
endfunction()


## -- Basic listing functions -- ##

# function to list all directories containing ".h" and ".hpp" files,
# recursing into the folder arg1
# -> arg0 contains the list at the end
function(list_include_directories arg0 arg1)
    file(GLOB_RECURSE SOURCE_FILES_TMP RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    		"${PROJECT_SOURCE_DIR}/${arg1}/*.h"
    		"${PROJECT_SOURCE_DIR}/${arg1}/*.hpp")
    set(CUR_DIR_LIST "")
    foreach(FILE_PATH ${SOURCE_FILES_TMP})
    	get_filename_component(CUR_ITEM ${FILE_PATH} NAME)
    	get_filename_component(CUR_PATH ${FILE_PATH} PATH)
    	prohibited_file(CUR_ITEM PROHIBITED_FILE)
    	if (NOT PROHIBITED_FILE)
    		set(CUR_DIR_LIST ${CUR_DIR_LIST} ${CUR_PATH})
    	endif (NOT PROHIBITED_FILE)
    endforeach(FILE_PATH)
    list(REMOVE_DUPLICATES CUR_DIR_LIST) 
    set(${arg0} ${CUR_DIR_LIST} PARENT_SCOPE)
endfunction()

# function to list all files whose form equals arg1,
# recursing into the folder arg2
# -> arg0 contains the list at the end
function(list_source_files arg0 arg1 arg2)
    file(GLOB_RECURSE SOURCE_FILES_TMP  
    		"${PROJECT_SOURCE_DIR}/${arg2}/${arg1}")
    foreach(SRC_FILE ${SOURCE_FILES_TMP})
    	get_filename_component(CUR_ITEM ${SRC_FILE} NAME)
    	prohibited_file(CUR_ITEM PROHIBITED_FILE)
    	if (NOT PROHIBITED_FILE)
    		set(SOURCES ${SOURCES} ${SRC_FILE})
    	endif (NOT PROHIBITED_FILE)
    endforeach(SRC_FILE)
    set(${arg0} ${SOURCES} PARENT_SCOPE)
endfunction()

# list all files (.c, .cc, .cpp, .h and .hpp) 
# recursively contained into the folder arg1
# -> arg0 contains the list at the end
function(list_folder_files arg0 arg1)
	set(CUR_FULL_LIST "")
	list_source_files(CUR_LIST "*.c" ${arg1})
	set(CUR_FULL_LIST ${CUR_LIST})
	list_source_files(CUR_LIST "*.cc" ${arg1})
	set(CUR_FULL_LIST ${CUR_FULL_LIST} ${CUR_LIST})
	list_source_files(CUR_LIST "*.cpp" ${arg1})
	set(CUR_FULL_LIST ${CUR_FULL_LIST} ${CUR_LIST})
	list_source_files(CUR_LIST "*.h" ${arg1})
	set(CUR_FULL_LIST ${CUR_FULL_LIST} ${CUR_LIST})
	list_source_files(CUR_LIST "*.hpp" ${arg1})
	set(CUR_FULL_LIST ${CUR_FULL_LIST} ${CUR_LIST})
	set(${arg0} ${CUR_FULL_LIST} PARENT_SCOPE)
endfunction()

# function to list all special files,
# recursing into sub-directories
# -> arg0 contains the list at the end
function(list_special_files arg0)
    file(GLOB_RECURSE SOURCE_FILES_TMP
            "${PROJECT_SOURCE_DIR}/src/project/*.mbsdata"
            "${PROJECT_SOURCE_DIR}/src/project/*/control_variables.txt"
            "${PROJECT_SOURCE_DIR}/src/project/*/simu_variables.txt")
    set(${arg0} ${SOURCE_FILES_TMP} PARENT_SCOPE)
endfunction()


## -- Listing functions combined -- ##

# list_include_directories into 2 folders: arg1 and arg2
# -> arg0 contains the list at the end
function(list_include_directories_2 arg0 arg1 arg2)
	set(CUR_FULL_LIST "")
	list_include_directories(CUR_LIST ${arg1})
	set(CUR_FULL_LIST ${CUR_LIST})	
	list_include_directories(CUR_LIST ${arg2})
	set(CUR_FULL_LIST ${CUR_FULL_LIST} ${CUR_LIST})
	set(${arg0} ${CUR_FULL_LIST} PARENT_SCOPE)
endfunction()

# list_include_directories into 2 folders: arg2 and arg3
# (arg 1 is still the requested file form)
# -> arg0 contains the list at the end
function(list_source_files_2 arg0 arg1 arg2 arg3)
	set(CUR_FULL_LIST "")
	list_source_files(CUR_LIST ${arg1} ${arg2})
	set(CUR_FULL_LIST ${CUR_LIST})	
	list_source_files(CUR_LIST ${arg1} ${arg3})
	set(CUR_FULL_LIST ${CUR_FULL_LIST} ${CUR_LIST})
	set(${arg0} ${CUR_FULL_LIST} PARENT_SCOPE)
endfunction()

# list_include_directories into 3 folders: arg2, arg3 and arg4
# (arg 1 is still the requested file form)
# -> arg0 contains the list at the end
function(list_source_files_3 arg0 arg1 arg2 arg3 arg4)
	set(CUR_FULL_LIST "")
	list_source_files(CUR_LIST ${arg1} ${arg2})
	set(CUR_FULL_LIST ${CUR_LIST})	
	list_source_files(CUR_LIST ${arg1} ${arg3})
	set(CUR_FULL_LIST ${CUR_FULL_LIST} ${CUR_LIST})
	list_source_files(CUR_LIST ${arg1} ${arg4})
	set(CUR_FULL_LIST ${CUR_FULL_LIST} ${CUR_LIST})
	set(${arg0} ${CUR_FULL_LIST} PARENT_SCOPE)
endfunction()

# list_include_directories into 1 folder (arg3),
# but looking for 2 forms: arg1 and arg2
# -> arg0 contains the list at the end
function(list_source_files_4 arg0 arg1 arg2 arg3)
	set(CUR_FULL_LIST "")
	list_source_files(CUR_LIST ${arg1} ${arg3})
	set(CUR_FULL_LIST ${CUR_LIST})	
	list_source_files(CUR_LIST ${arg2} ${arg3})
	set(CUR_FULL_LIST ${CUR_FULL_LIST} ${CUR_LIST})
	set(${arg0} ${CUR_FULL_LIST} PARENT_SCOPE)
endfunction()


## ---- FIND PATHS AND ADD DEFINITIONS ---- ##

# main source project path (to print in 'cmake_config.h') 
set (PROJECT_ABS_PATH ${PROJECT_SOURCE_DIR})
set (BINARY_ABS_PATH ${PROJECT_BINARY_DIR})

# not a Simulink simulation (Standalone)
add_definitions( -DSTANDALONE )
add_definitions( -DCMEX )

# load a xml file to intialize the dynamics structure
add_definitions( -DLOAD_XML )

# Ground Contact Model activated
if (FLAG_GROUND_CONTACT_MODEL)
	add_definitions( -DGCM_MT )
endif (FLAG_GROUND_CONTACT_MODEL)

# stop the simulation
if (FLAG_STOP_SIMULATION)
	add_definitions( -DSTOP_SIMU )
endif (FLAG_STOP_SIMULATION)

# dirdynared or accelred
if (FLAG_DIRDYNARED)
	add_definitions( -DDIRDYNARED )    
else (FLAG_DIRDYNARED)
    if (FLAG_ODN_METHOD)
        add_definitions( -DODN )
    else (FLAG_ODN_METHOD)
        add_definitions( -DACCELRED )
    endif (FLAG_ODN_METHOD)	
endif (FLAG_DIRDYNARED)

# flag for Unix systems (used in some source files)
if (UNIX)
	add_definitions( -DUNIX )
endif (UNIX)

# write the .anim file and the debug .txt vectors
if (FLAG_WRITE_OUTPUT_FILES)
	add_definitions( -DWRITE_FILES )
endif (FLAG_WRITE_OUTPUT_FILES)

# print messages in the terminal...
if (FLAG_PRINT_REPORT)
	add_definitions( -DPRINT_REPORT )
endif (FLAG_PRINT_REPORT)

# short feet flag
if (${COMAN_VERSION} STREQUAL "Short_feet")
	add_definitions( -DSHORT_FEET )
endif (${COMAN_VERSION} STREQUAL "Short_feet")

# compliant feet flag
if (${COMAN_VERSION} STREQUAL "Compliant_feet")
	add_definitions( -DCOMP_FEET )
endif (${COMAN_VERSION} STREQUAL "Compliant_feet")

# long arms flag
if (${COMAN_VERSION} STREQUAL "Long_arms")
	add_definitions( -DLONG_ARMS )
	add_definitions( -DSHORT_FEET )
endif (${COMAN_VERSION} STREQUAL "Long_arms")



## ---- EXTERNAL LIBRARIES ---- ##

# Libxml2 (always required)
find_package(Libxml2 REQUIRED)
if (LIBXML2_FOUND)
	include_directories (${LIBXML2_INCLUDE_PATH})	
	message( "\n  >> Libxml2 found")
else (LIBXML2_FOUND)
	message( "\n  >> Libxml2 not found, modify src/other/conf/FindLibxml2.cmake")
endif(LIBXML2_FOUND)

message("      -> INC : ${LIBXML2_INCLUDE_PATH}")
message("      -> LIB : ${LIBXML2_LIBRARIES}\n")

# GSL library
find_package(GSL REQUIRED)
if (GSL_FOUND)
	include_directories(${GSL_INCLUDE_DIR})
	message( "\n >> GSL found")
else(GSL_FOUND)
	message( "\n >> GSL not found, modify src/other/conf/FindGSL.cmake")
endif(GSL_FOUND)

message("      -> INC : ${GSL_INCLUDE_DIR}")
message("      -> LIB : ${GSL_LIBRARIES}\n")

# real-time features
if (FLAG_REAL_TIME)

	# real-time flag
	add_definitions( -DREAL_TIME )

	# SDL
	if (FLAG_SDL_INTERACTIONS)

		add_definitions( -DSDL )

		find_package(SDL2 REQUIRED)
		if (SDL2_FOUND)
			include_directories (${SDL2_INCLUDE_PATH})
			message( "\n  >> SDL 2.0 found")		
		else (SDL2_FOUND)
			message( "\n>> SDL 2.0 not found, modify src/other/conf/FindSDL2.cmake")		
		endif (SDL2_FOUND)

		message("      -> INC : ${SDL2_INCLUDE_PATH}")
		message("      -> LIB : ${SDL2_LIBRARIES}\n")

		# SDL_ttf
		find_package(SDL2_ttf REQUIRED)
		if (SDL2TTF_FOUND)				
			include_directories (${SDL2TTF_INCLUDE_PATH})
			message( "\n  >> SDL 2.0 ttf found")				
		else (SDL2TTF_FOUND)
			message( "\n  >> SDL 2.0 ttf not found, modify src/other/conf/FindSDL2_ttf.cmake")
		endif (SDL2TTF_FOUND)

		message("      -> INC : ${SDL2TTF_INCLUDE_PATH}")
		message("      -> LIB : ${SDL2TTF_LIBRARIES}\n")

	endif (FLAG_SDL_INTERACTIONS)

	# JNI (Java Native Interface)
	if (FLAG_JAVA_VISUALIZATION)

		add_definitions( -DJNI )

		find_package(JNI REQUIRED)

		if (JNI_FOUND)
			include_directories (${JNI_INCLUDE_PATH})
			message( "\n  >> JNI found")
		else (JNI_FOUND)
			message( "\n  >> JNI not found, modify src/other/conf/FindJNI.cmake")
		endif (JNI_FOUND)

		message("      -> INC : ${JNI_INCLUDE_PATH}")
		message("      -> LIB : ${JNI_LIBRARIES}\n")

		# Java 3D
		find_package(J3D REQUIRED)

		if (J3D_FOUND)
			# path to some Java 3D files (to print in 'cmake_config.h') 
			set (J3D_ABS_PATH ${J3D_PATH})
			message( "\n  >> Java 3D found")
		else (J3D_FOUND)
			message( "\n  >> Java 3D not found, modify src/other/conf/FindJ3D.cmake")
		endif (J3D_FOUND)

		message("      -> PATH : ${J3D_PATH}\n")

	endif (FLAG_JAVA_VISUALIZATION)

endif (FLAG_REAL_TIME)

# Simbody
if (FLAG_SIMBODY)
	add_definitions( -DSIMBODY )

	find_package(Simbody REQUIRED)
	if (SIMBODY_FOUND)
			include_directories (${SIMBODY_INCLUDE_PATH})
			message( "\n  >> Simbody found")		
		else (SIMBODY_FOUND)
			message( "\n>> Simbody not found, modify src/other/conf/FindSimbody.cmake")		
		endif (SIMBODY_FOUND)

		message("      -> INC : ${SIMBODY_INCLUDE_PATH}")
		message("      -> LIB : ${SIMBODY_LIBRARIES}\n")
endif (FLAG_SIMBODY)


# configure a header file to pass some of the CMake settings
# to the source code
configure_file (
  "${PROJECT_SOURCE_DIR}/src/other/conf/cmake_config.h.in"
  "${PROJECT_BINARY_DIR}/src/other/conf/cmake_config.h"
  )
# 'cmake_config.h.in' is in the 'conf' folder
include_directories (${PROJECT_BINARY_DIR}/src/other/conf)



## ---- WINDOWS DLL FILES ---- ##

# copy all the dll (except 'jvm.dll') used for Windows
# these dll files are copied in the Executable directory (Debug or Release)
if (WIN32)
	file(COPY ${PROJECT_SOURCE_DIR}/src/other/win64_include_lib/dll/ DESTINATION ${CMAKE_BINARY_DIR}/Debug)
	file(COPY ${PROJECT_SOURCE_DIR}/src/other/win64_include_lib/dll/ DESTINATION ${CMAKE_BINARY_DIR}/Release)
endif (WIN32)



## ---- ADDITIONAL CMAKELISTS.TXT ---- ##

# add locations of other CMakeLists.txt
add_subdirectory (src/)
add_subdirectory (src/other/varState/)

## change for user_models
add_subdirectory (src/other/user_models/)
