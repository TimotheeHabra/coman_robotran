#
# author: Nicolas Van der Noot
# Feb 27 2014
#
# CMakeLists used to compile the main sources files of the project
#

# list all source files
list_source_files_3(SOURCE_FILES_C   "*.c"   "../symbolicR" "src/project" "src/generic")
list_source_files_3(SOURCE_FILES_CPP "*.cpp" "../symbolicR" "src/project" "src/generic")
list_source_files_3(HEADER_FILES_H   "*.h"   "../symbolicR" "src/project" "src/generic")
list_source_files_3(HEADER_FILES_HPP "*.hpp" "../symbolicR" "src/project" "src/generic")

set(HEADER_FILES ${HEADER_FILES_H} ${HEADER_FILES_HPP})

# list all source files from the 'generic' folder
list_folder_files(SRC_GEN_INT "src/generic/integrator")
list_folder_files(SRC_GEN_JNI "src/generic/jni")
list_folder_files(SRC_GEN_MAIN "src/generic/main_files")
list_folder_files(SRC_GEN_XML "src/generic/mbs_load_xml")
list_folder_files(SRC_GEN_OTH "src/generic/other")
list_folder_files(SRC_GEN_RT "src/generic/real_time")
list_folder_files(SRC_GEN_ROB "src/generic/robotran")
list_folder_files(SRC_GEN_SDL "src/generic/sdl")
list_folder_files(SRC_GEN_SYM "src/generic/Simbody")

# list all source files from the 'project' folder
list_folder_files(SRC_PRO_CTRL "src/project/controller_files")
list_folder_files(SRC_PRO_PRO "src/project/project_files")
list_folder_files(SRC_PRO_SIMU "src/project/simulation_files")
list_folder_files(SRC_PRO_USR "src/project/user_files")
list_folder_files(SRC_PRO_SYM "../symbolicR")
list_special_files(SPECIAL_FILES)

# directories for the 'generic' folder
source_group("src\\generic\\integrator" FILES ${SRC_GEN_INT})
source_group("src\\generic\\jni" FILES ${SRC_GEN_JNI})
source_group("src\\generic\\main_files" FILES ${SRC_GEN_MAIN})
source_group("src\\generic\\mbs_load_xml" FILES ${SRC_GEN_XML})
source_group("src\\generic\\other" FILES ${SRC_GEN_OTH})
source_group("src\\generic\\real_time" FILES ${SRC_GEN_RT})
source_group("src\\generic\\robotran" FILES ${SRC_GEN_ROB})
source_group("src\\generic\\sdl" FILES ${SRC_GEN_SDL})
source_group("src\\generic\\symbody" FILES ${SRC_GEN_SYM})

# directories for the 'project' folder
source_group("src\\project\\controller_files" FILES ${SRC_PRO_CTRL})
source_group("src\\project\\project_files" FILES ${SRC_PRO_PRO})
source_group("src\\project\\simulation_files" FILES ${SRC_PRO_SIMU})
source_group("src\\project\\user_files" FILES ${SRC_PRO_USR})
source_group("src\\project\\symbolicR" FILES ${SRC_PRO_SYM})
source_group("src\\project\\special_files" FILES ${SPECIAL_FILES}) 


# list all directories where the header files are located (except the external ones)
list_include_directories_2(FULL_INCLUDE_DIRECTORIES "src/generic" "src/project")

# include the corresponding directories
include_directories(${FULL_INCLUDE_DIRECTORIES})

set(SOURCE_FILES ${SOURCE_FILES_C})

if (FLAG_CXX_PROJECT)
	set(SOURCE_FILES ${SOURCE_FILES} ${SOURCE_FILES_CPP})
endif (FLAG_CXX_PROJECT)

# name of the executable: 'Exec'
set (Executable Exec)


# put the executable in the root of the build folder
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR} )

# generate the executable
add_executable (${Executable} ${SOURCE_FILES} ${HEADER_FILES} ${SPECIAL_FILES})

# math external library (for Unix): linkage
if (UNIX)
	target_link_libraries (${Executable} m)
endif (UNIX)

# Libxml2 external libray: linkage
target_link_libraries (${Executable} ${LIBXML2_LIBRARIES})

# GSL external libray: linkage
target_link_libraries (${Executable} ${GSL_LIBRARIES})

if (FLAG_REAL_TIME)

	# SDL external library: linkage
	if (FLAG_SDL_INTERACTIONS)
		target_link_libraries (${Executable} ${SDL2_LIBRARIES})
		target_link_libraries (${Executable} ${SDL2TTF_LIBRARIES})
	endif (FLAG_SDL_INTERACTIONS)

	# JNI external library: linkage
	if (FLAG_JAVA_VISUALIZATION)
		target_link_libraries (${Executable} ${JNI_LIBRARIES})
	endif (FLAG_JAVA_VISUALIZATION)

endif (FLAG_REAL_TIME)

# Simbody
if (FLAG_SIMBODY)
	target_link_libraries (${Executable} ${SIMBODY_LIBRARIES})
endif(FLAG_SIMBODY)


# files compiled as C++ files
if (FLAG_CXX_PROJECT)
	set_source_files_properties(${SOURCE_FILES_CPP} PROPERTIES LANGUAGE CXX )
endif (FLAG_CXX_PROJECT)

