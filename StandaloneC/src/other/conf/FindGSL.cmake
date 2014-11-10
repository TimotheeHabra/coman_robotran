##
## Try to find gnu scientific library GSL
## (see http://www.gnu.org/software/gsl/)
## Once run this will define:
##
## GSL_FOUND = system has GSL lib
##
## GSL_LIBRARIES = full path to the libraries
## on Unix/Linux with additional linker flags from "gsl-config --libs"
##
## CMAKE_GSL_CXX_FLAGS = Unix compiler flags for GSL, essentially "`gsl-config --cxxflags`"
##
## GSL_INCLUDE_DIR = where to find headers
##
## GSL_LINK_DIRECTORIES = link directories, useful for rpath on Unix
## GSL_EXE_LINKER_FLAGS = rpath on Unix
##
## Felix Woelk 07/2004
## minor corrections Jan Woetzel
##
## www.mip.informatik.uni-kiel.de
## --------------------------------
##
## Included in robotran standalone by Aubain Verle and Timothee Habra Sept 14.
## Some changes to account for different include dirs.


## ---- WINDOWS ---- ##

IF(WIN32)

# possible paths for: 'gsl_.h'
SET(TRIAL_PATHS_INC
  ${PROJECT_SOURCE_DIR}/src/other/win64_include_lib/include/gsl   #/gsl
)

# possible paths for: 'GSL.lib'
SET(TRIAL_PATHS_LIB_GSL
  ${PROJECT_SOURCE_DIR}/src/other/win64_include_lib/lib
)

FIND_PATH(GSL_INCLUDE_DIR gsl_version.h ${TRIAL_PATHS_INC})

FIND_LIBRARY(GSL_LIBRARIES_GSL gsl.lib ${TRIAL_PATHS_LIB_GSL})
FIND_LIBRARY(GSL_LIBRARIES_GSLCBLAS cblas.lib ${TRIAL_PATHS_LIB_GSL})

IF (GSL_LIBRARIES_GSL AND GSL_LIBRARIES_GSLCBLAS)
	SET(GSL_LIBRARIES ${GSL_LIBRARIES_GSL} ${GSL_LIBRARIES_GSLCBLAS})
ENDIF (GSL_LIBRARIES_GSL AND GSL_LIBRARIES_GSLCBLAS)

## ---------------- ##


ELSE(WIN32)

## ---- LINUX ---- ##

IF(UNIX)

SET(GSL_CONFIG_PREFER_PATH 
	"$ENV{GSL_HOME}/bin" CACHE STRING "preferred path to GSL (gsl-config)"
)

FIND_PROGRAM(GSL_CONFIG gsl-config ${GSL_CONFIG_PREFER_PATH} $ENV{LIB_DIR}/bin /usr/local/bin/ /usr/bin/ )

# MESSAGE("DBG GSL_CONFIG ${GSL_CONFIG}")
IF (GSL_CONFIG)

	# set CXXFLAGS to be fed into CXX_FLAGS by the user:
	SET(GSL_CXX_FLAGS "`${GSL_CONFIG} --cflags`")

	# set INCLUDE_DIRS to prefix+include
	EXEC_PROGRAM(${GSL_CONFIG}
	ARGS --prefix
	OUTPUT_VARIABLE GSL_PREFIX)

	SET(GSL_INCLUDE_DIR 
		${GSL_PREFIX}/include CACHE STRING INTERNAL
	)

	# set link libraries and link flags
	EXEC_PROGRAM(${GSL_CONFIG}
	ARGS --libs
	OUTPUT_VARIABLE GSL_LIBRARIES)

	## extract link dirs for rpath
	EXEC_PROGRAM(${GSL_CONFIG}
	ARGS --libs
	OUTPUT_VARIABLE GSL_CONFIG_LIBS )

	## split off the link dirs (for rpath)
	## use regular expression to match wildcard equivalent "-L*<endchar>"
	## with <endchar> is a space or a semicolon
	STRING(REGEX MATCHALL 
		"[-][L]([^ ;])+"
		GSL_LINK_DIRECTORIES_WITH_PREFIX
		"${GSL_CONFIG_LIBS}" 
	)

	# MESSAGE("DBG GSL_LINK_DIRECTORIES_WITH_PREFIX=${GSL_LINK_DIRECTORIES_WITH_PREFIX}")
	## remove prefix -L because we need the pure directory for LINK_DIRECTORIES
	IF (GSL_LINK_DIRECTORIES_WITH_PREFIX)
		STRING(REGEX REPLACE "[-][L]" "" GSL_LINK_DIRECTORIES ${GSL_LINK_DIRECTORIES_WITH_PREFIX} )
	ENDIF (GSL_LINK_DIRECTORIES_WITH_PREFIX)

	SET(GSL_EXE_LINKER_FLAGS 
		"-Wl,-rpath,${GSL_LINK_DIRECTORIES}" 
		CACHE STRING INTERNAL
	)

	# MESSAGE("DBG GSL_LINK_DIRECTORIES=${GSL_LINK_DIRECTORIES}")
	# MESSAGE("DBG GSL_EXE_LINKER_FLAGS=${GSL_EXE_LINKER_FLAGS}")
	# ADD_DEFINITIONS("-DHAVE_GSL")
	# SET(GSL_DEFINITIONS "-DHAVE_GSL")
	MARK_AS_ADVANCED(
	GSL_CXX_FLAGS
	GSL_INCLUDE_DIR
	GSL_LIBRARIES
	GSL_LINK_DIRECTORIES
	GSL_DEFINITIONS
)

ELSE(GSL_CONFIG)
	IF (GSL_FIND_REQUIRED)
		MESSAGE(FATAL_ERROR "Could not find gsl-config. Please set it manually. GSL_CONFIG=${GSL_CONFIG}")
	ELSE (GSL_FIND_REQUIRED)
		MESSAGE(STATUS "Could not find GSL")
		# TODO: Avoid cmake complaints if GSL is not found
	ENDIF (GSL_FIND_REQUIRED)
ENDIF(GSL_CONFIG)


## ----------------- ##

ENDIF(UNIX)

ENDIF(WIN32)

# flag put to 1 if all required files are found
IF(GSL_LIBRARIES)
	IF(GSL_INCLUDE_DIR OR GSL_CXX_FLAGS)
		SET(GSL_FOUND 1)
		# MESSAGE(STATUS "Using GSL from ${GSL_PREFIX}")
	ENDIF(GSL_INCLUDE_DIR OR GSL_CXX_FLAGS)
ENDIF(GSL_LIBRARIES) 