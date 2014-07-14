#
# author: Nicolas Van der Noot
# Feb 27 2014
#
# CMakeLists used to plot the vectors saved in the fodler 'vectors'
# in post-process, using the same SDL interface as the one used 
# in the Real-time version
#

# CMake minimum version
cmake_minimum_required(VERSION 2.8.7)

# project name
project (PLOT_POST_PROCESS)

# message to display the project name
message(STATUS "Processing ${PROJECT_NAME}")

# use Release version and not the Debug one (faster)
set(CMAKE_BUILD_TYPE Release)

# for Unix: display all the warnings, except the ones related to /* -- */
if (UNIX)
  set(CMAKE_C_FLAGS "-g -Wall -Wno-comment")
endif (UNIX)

# definitions
add_definitions( -DSDL )
add_definitions( -DREAL_TIME )
add_definitions( -DSTANDALONE )
add_definitions( -DCMEX )
add_definitions( -DLOAD_XML )

# flag for Unix systems (used in some source files)
if (UNIX)
    add_definitions( -DUNIX )
endif (UNIX)

# function to list all source files in the src/ directory, 
# recursing into sub-directories
function(list_gen_source_files HEADER_FILES)
    file(GLOB_RECURSE SOURCE_FILES_TMP 
        "${PROJECT_SOURCE_DIR}/src/*.c" 
        "${PROJECT_SOURCE_DIR}/../../generic/sdl/plot_sdl.c"
        "${PROJECT_SOURCE_DIR}/../../generic/useful/useful_functions.c" 
        "${PROJECT_SOURCE_DIR}/../../generic/real_time/time_functions.c"
        "${PROJECT_SOURCE_DIR}/../../generic/real_time/real_time.c" 
        "${PROJECT_SOURCE_DIR}/../../project/project_files/events_config.c")
    set(SOURCE_FILES ${SOURCE_FILES_TMP} PARENT_SCOPE)
endfunction()

# function to list all paths to header files in the src/ directory, 
# recursing into sub-directories (includes)
function(list_gen_include_directories FULL_INCLUDE_DIRECTORIES)
    file(GLOB_RECURSE FULL_INCLUDE_DIRECTORIES_TMP RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} 
        "${PROJECT_SOURCE_DIR}/src/*.h" 
        "${PROJECT_SOURCE_DIR}/../../generic/*.h" 
        "${PROJECT_SOURCE_DIR}/../../project/*.h" 
        "${PROJECT_SOURCE_DIR}/../../../../symbolicR/*.h")
    set(DIR_LIST "")
    foreach(FILE_PATH ${FULL_INCLUDE_DIRECTORIES_TMP})
        get_filename_component(DIR_PATH ${FILE_PATH} PATH)
        set(DIR_LIST ${DIR_LIST} ${DIR_PATH})
    endforeach(FILE_PATH)
    list(REMOVE_DUPLICATES DIR_LIST) 
    set(FULL_INCLUDE_DIRECTORIES ${DIR_LIST} PARENT_SCOPE)
endfunction()


# main source project path (to print in 'cmake_config.h') 
set (CUR_PROJECT_ABS_PATH ${PROJECT_SOURCE_DIR})
set (PROJECT_ABS_PATH "${PROJECT_SOURCE_DIR}/../../..")
set (BINARY_ABS_PATH ${PROJECT_BINARY_DIR})

# configure a header file to pass some of the CMake settings
# to the source code
configure_file (
  "${PROJECT_SOURCE_DIR}/src/conf/cmake_config.h.in"
  "${PROJECT_BINARY_DIR}/src/conf/cmake_config.h"
  )
# 'cmake_config.h.in' is in the 'conf' folder
include_directories (${PROJECT_BINARY_DIR}/src/conf)


# define the location of all the 'Find<module>.cmake' files
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../conf)

# SDL libraries
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

# Libxml2 (required for include folders)
find_package(Libxml2 REQUIRED)
if (LIBXML2_FOUND)
    include_directories (${LIBXML2_INCLUDE_PATH})   
    message( "\n  >> Libxml2 found")
else (LIBXML2_FOUND)
    message( "\n  >> Libxml2 not found, modify src/other/conf/FindLibxml2.cmake")
endif(LIBXML2_FOUND)

message("      -> INC : ${LIBXML2_INCLUDE_PATH}")
message("      -> LIB : ${LIBXML2_LIBRARIES}\n")


# copy all the dll (except 'jvm.dll') used for Windows
# these dll files are copied in the Executable directory (Debug or Release)
if (NOT UNIX)
    file(COPY ${PROJECT_ABS_PATH}/src/other/win64_include_lib/dll/ DESTINATION ${CMAKE_BINARY_DIR}/Debug)
    file(COPY ${PROJECT_ABS_PATH}/src/other/win64_include_lib/dll/ DESTINATION ${CMAKE_BINARY_DIR}/Release)
endif (NOT UNIX)
 
# list source files to compile
list_gen_source_files(SOURCE_FILES)

# list include directories (to find headers)
list_gen_include_directories(FULL_INCLUDE_DIRECTORIES)

# include these directories
include_directories (${FULL_INCLUDE_DIRECTORIES})

# name of the executable: 'Gen_state_var'
set (Executable Plot_post_process)

# generate the executable
add_executable (${Executable} ${SOURCE_FILES})

target_link_libraries (${Executable} ${SDL2_LIBRARIES})
target_link_libraries (${Executable} ${SDL2TTF_LIBRARIES})

# math external library (for Unix): linkage
if (UNIX)
	target_link_libraries (${Executable} m)
endif (UNIX)
