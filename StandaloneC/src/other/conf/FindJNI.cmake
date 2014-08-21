#
# author: Nicolas Van der Noot
# Feb 27 2014
#
# This file finds the include folders (header files) and the libraries
# related to JNI (Java Native Interface)
#
# JNI_FOUND :        1 if all required files found (0 otherwise)
# JNI_INCLUDE_PATH : include paths (for the header files) -> for compilation
# JNI_LIBRARIES :    libraries -> for linkage
#


IF (UNIX)

IF (APPLE)
  

## ---- MAC OS ---- ##

# possible paths for: 'jni.h'
SET(TRIAL_PATHS_INC_JNI
  /Library/Java/JavaVirtualMachines/jdk1.7.0_40.jdk/Contents/Home/include
  /Library/Java/JavaVirtualMachines/jdk1.7.0_41.jdk/Contents/Home/include
  /Library/Java/JavaVirtualMachines/jdk1.7.0_42.jdk/Contents/Home/include
  /Library/Java/JavaVirtualMachines/jdk1.7.0_43.jdk/Contents/Home/include
  /Library/Java/JavaVirtualMachines/jdk1.7.0_44.jdk/Contents/Home/include
  /Library/Java/JavaVirtualMachines/jdk1.7.0_45.jdk/Contents/Home/include
  /Library/Java/JavaVirtualMachines/jdk1.7.0_46.jdk/Contents/Home/include
  /Library/Java/JavaVirtualMachines/jdk1.7.0_47.jdk/Contents/Home/include
  /Library/Java/JavaVirtualMachines/jdk1.7.0_48.jdk/Contents/Home/include
  /Library/Java/JavaVirtualMachines/jdk1.7.0_49.jdk/Contents/Home/include
  /Library/Java/JavaVirtualMachines/jdk1.7.0_50.jdk/Contents/Home/include
  /Library/Java/JavaVirtualMachines/jdk1.7.0_51.jdk/Contents/Home/include
  /Library/Java/JavaVirtualMachines/jdk1.7.0_52.jdk/Contents/Home/include
  /Library/Java/JavaVirtualMachines/jdk1.7.0_53.jdk/Contents/Home/include
  /Library/Java/JavaVirtualMachines/jdk1.7.0_54.jdk/Contents/Home/include
  /Library/Java/JavaVirtualMachines/jdk1.7.0_55.jdk/Contents/Home/include
  /Library/Java/JavaVirtualMachines/jdk1.7.0_56.jdk/Contents/Home/include
  /Library/Java/JavaVirtualMachines/jdk1.7.0_57.jdk/Contents/Home/include
  /Library/Java/JavaVirtualMachines/jdk1.7.0_58.jdk/Contents/Home/include
  /Library/Java/JavaVirtualMachines/jdk1.7.0_59.jdk/Contents/Home/include
  /Library/Java/JavaVirtualMachines/jdk1.7.0_60.jdk/Contents/Home/include
  /Library/Java/JavaVirtualMachines/jdk1.7.0_61.jdk/Contents/Home/include
  /Library/Java/JavaVirtualMachines/jdk1.7.0_62.jdk/Contents/Home/include
  /Library/Java/JavaVirtualMachines/jdk1.7.0_63.jdk/Contents/Home/include
  /Library/Java/JavaVirtualMachines/jdk1.7.0_64.jdk/Contents/Home/include
  /Library/Java/JavaVirtualMachines/jdk1.7.0_65.jdk/Contents/Home/include
  /Library/Java/JavaVirtualMachines/jdk1.7.0_66.jdk/Contents/Home/include
  /Library/Java/JavaVirtualMachines/jdk1.7.0_67.jdk/Contents/Home/include
) 

# possible paths for: 'jni_md.h'
SET(TRIAL_PATHS_INC_JNI_MD
  /Library/Java/JavaVirtualMachines/jdk1.7.0_40.jdk/Contents/Home/include/darwin
  /Library/Java/JavaVirtualMachines/jdk1.7.0_41.jdk/Contents/Home/include/darwin
  /Library/Java/JavaVirtualMachines/jdk1.7.0_42.jdk/Contents/Home/include/darwin
  /Library/Java/JavaVirtualMachines/jdk1.7.0_43.jdk/Contents/Home/include/darwin
  /Library/Java/JavaVirtualMachines/jdk1.7.0_44.jdk/Contents/Home/include/darwin
  /Library/Java/JavaVirtualMachines/jdk1.7.0_45.jdk/Contents/Home/include/darwin
  /Library/Java/JavaVirtualMachines/jdk1.7.0_46.jdk/Contents/Home/include/darwin
  /Library/Java/JavaVirtualMachines/jdk1.7.0_47.jdk/Contents/Home/include/darwin
  /Library/Java/JavaVirtualMachines/jdk1.7.0_48.jdk/Contents/Home/include/darwin
  /Library/Java/JavaVirtualMachines/jdk1.7.0_49.jdk/Contents/Home/include/darwin
  /Library/Java/JavaVirtualMachines/jdk1.7.0_50.jdk/Contents/Home/include/darwin
  /Library/Java/JavaVirtualMachines/jdk1.7.0_51.jdk/Contents/Home/include/darwin
  /Library/Java/JavaVirtualMachines/jdk1.7.0_52.jdk/Contents/Home/include/darwin
  /Library/Java/JavaVirtualMachines/jdk1.7.0_53.jdk/Contents/Home/include/darwin
  /Library/Java/JavaVirtualMachines/jdk1.7.0_54.jdk/Contents/Home/include/darwin
  /Library/Java/JavaVirtualMachines/jdk1.7.0_55.jdk/Contents/Home/include/darwin
  /Library/Java/JavaVirtualMachines/jdk1.7.0_56.jdk/Contents/Home/include/darwin
  /Library/Java/JavaVirtualMachines/jdk1.7.0_57.jdk/Contents/Home/include/darwin
  /Library/Java/JavaVirtualMachines/jdk1.7.0_58.jdk/Contents/Home/include/darwin
  /Library/Java/JavaVirtualMachines/jdk1.7.0_59.jdk/Contents/Home/include/darwin
  /Library/Java/JavaVirtualMachines/jdk1.7.0_60.jdk/Contents/Home/include/darwin
  /Library/Java/JavaVirtualMachines/jdk1.7.0_61.jdk/Contents/Home/include/darwin
  /Library/Java/JavaVirtualMachines/jdk1.7.0_62.jdk/Contents/Home/include/darwin
  /Library/Java/JavaVirtualMachines/jdk1.7.0_63.jdk/Contents/Home/include/darwin
  /Library/Java/JavaVirtualMachines/jdk1.7.0_64.jdk/Contents/Home/include/darwin
  /Library/Java/JavaVirtualMachines/jdk1.7.0_65.jdk/Contents/Home/include/darwin
  /Library/Java/JavaVirtualMachines/jdk1.7.0_66.jdk/Contents/Home/include/darwin
  /Library/Java/JavaVirtualMachines/jdk1.7.0_67.jdk/Contents/Home/include/darwin
) 

# possible paths for: 'libjvm.dylib'
SET(TRIAL_PATHS_LIB
  /Library/Java/JavaVirtualMachines/jdk1.7.0_40.jdk/Contents/Home/jre/lib/server
  /Library/Java/JavaVirtualMachines/jdk1.7.0_41.jdk/Contents/Home/jre/lib/server
  /Library/Java/JavaVirtualMachines/jdk1.7.0_42.jdk/Contents/Home/jre/lib/server
  /Library/Java/JavaVirtualMachines/jdk1.7.0_43.jdk/Contents/Home/jre/lib/server
  /Library/Java/JavaVirtualMachines/jdk1.7.0_44.jdk/Contents/Home/jre/lib/server
  /Library/Java/JavaVirtualMachines/jdk1.7.0_45.jdk/Contents/Home/jre/lib/server
  /Library/Java/JavaVirtualMachines/jdk1.7.0_46.jdk/Contents/Home/jre/lib/server
  /Library/Java/JavaVirtualMachines/jdk1.7.0_47.jdk/Contents/Home/jre/lib/server
  /Library/Java/JavaVirtualMachines/jdk1.7.0_48.jdk/Contents/Home/jre/lib/server
  /Library/Java/JavaVirtualMachines/jdk1.7.0_49.jdk/Contents/Home/jre/lib/server
  /Library/Java/JavaVirtualMachines/jdk1.7.0_50.jdk/Contents/Home/jre/lib/server
  /Library/Java/JavaVirtualMachines/jdk1.7.0_51.jdk/Contents/Home/jre/lib/server
  /Library/Java/JavaVirtualMachines/jdk1.7.0_52.jdk/Contents/Home/jre/lib/server
  /Library/Java/JavaVirtualMachines/jdk1.7.0_53.jdk/Contents/Home/jre/lib/server
  /Library/Java/JavaVirtualMachines/jdk1.7.0_54.jdk/Contents/Home/jre/lib/server
  /Library/Java/JavaVirtualMachines/jdk1.7.0_55.jdk/Contents/Home/jre/lib/server
  /Library/Java/JavaVirtualMachines/jdk1.7.0_56.jdk/Contents/Home/jre/lib/server
  /Library/Java/JavaVirtualMachines/jdk1.7.0_57.jdk/Contents/Home/jre/lib/server
  /Library/Java/JavaVirtualMachines/jdk1.7.0_58.jdk/Contents/Home/jre/lib/server
  /Library/Java/JavaVirtualMachines/jdk1.7.0_59.jdk/Contents/Home/jre/lib/server
  /Library/Java/JavaVirtualMachines/jdk1.7.0_60.jdk/Contents/Home/jre/lib/server
  /Library/Java/JavaVirtualMachines/jdk1.7.0_61.jdk/Contents/Home/jre/lib/server
  /Library/Java/JavaVirtualMachines/jdk1.7.0_62.jdk/Contents/Home/jre/lib/server
  /Library/Java/JavaVirtualMachines/jdk1.7.0_63.jdk/Contents/Home/jre/lib/server
  /Library/Java/JavaVirtualMachines/jdk1.7.0_64.jdk/Contents/Home/jre/lib/server
  /Library/Java/JavaVirtualMachines/jdk1.7.0_65.jdk/Contents/Home/jre/lib/server
  /Library/Java/JavaVirtualMachines/jdk1.7.0_66.jdk/Contents/Home/jre/lib/server
  /Library/Java/JavaVirtualMachines/jdk1.7.0_67.jdk/Contents/Home/jre/lib/server
) 

FIND_PATH(JNI_INCLUDE_JNI jni.h ${TRIAL_PATHS_INC_JNI})
FIND_PATH(JNI_INCLUDE_JNI_MD jni_md.h ${TRIAL_PATHS_INC_JNI_MD})

FIND_LIBRARY(JNI_LIBRARIES libjvm.dylib ${TRIAL_PATHS_LIB})

SET(JNI_INCLUDE_PATH ${JNI_INCLUDE_JNI} ${JNI_INCLUDE_JNI_MD})

## ---------------- ##


ELSE (APPLE)
  

## ---- LINUX ---- ## 

# possible paths for: 'jni.h'
SET(TRIAL_PATHS_INC_JNI
  /usr/lib/jvm/java-7-openjdk-amd64/include
  /usr/lib/jvm/java-6-sun/include
) 

# possible paths for: 'jni_md.h'
SET(TRIAL_PATHS_INC_JNI_MD
  /usr/lib/jvm/java-7-openjdk-amd64/include/linux
  /usr/lib/jvm/java-6-sun/include/linux
) 

# possible paths for: 'libjvm.so'
SET(TRIAL_PATHS_LIB
  /usr/lib/jvm/java-7-openjdk-amd64/jre/lib/amd64/server
  /usr/lib/jvm/java-6-sun/jre/lib/amd64/server
) 

FIND_PATH(JNI_INCLUDE_JNI jni.h ${TRIAL_PATHS_INC_JNI})
FIND_PATH(JNI_INCLUDE_JNI_MD jni_md.h ${TRIAL_PATHS_INC_JNI_MD})

FIND_LIBRARY(JNI_LIBRARIES libjvm.so ${TRIAL_PATHS_LIB})

SET(JNI_INCLUDE_PATH ${JNI_INCLUDE_JNI} ${JNI_INCLUDE_JNI_MD})

## --------------- ##


ENDIF (APPLE)

ELSE (UNIX)


## ---- WINDOWS ---- ##

# possible paths for: 'jni.h'
SET(TRIAL_PATHS_INC_JNI
  C:/Program\ Files/Java/jdk1.7.0_40/include
  C:/Program\ Files/Java/jdk1.7.0_41/include
  C:/Program\ Files/Java/jdk1.7.0_42/include
  C:/Program\ Files/Java/jdk1.7.0_43/include
  C:/Program\ Files/Java/jdk1.7.0_44/include
  C:/Program\ Files/Java/jdk1.7.0_45/include
  C:/Program\ Files/Java/jdk1.7.0_46/include
  C:/Program\ Files/Java/jdk1.7.0_47/include
  C:/Program\ Files/Java/jdk1.7.0_48/include
  C:/Program\ Files/Java/jdk1.7.0_49/include
  C:/Program\ Files/Java/jdk1.7.0_50/include
  C:/Program\ Files/Java/jdk1.7.0_51/include
  C:/Program\ Files/Java/jdk1.7.0_52/include
  C:/Program\ Files/Java/jdk1.7.0_53/include
  C:/Program\ Files/Java/jdk1.7.0_54/include
  C:/Program\ Files/Java/jdk1.7.0_55/include
  C:/Program\ Files/Java/jdk1.7.0_56/include
  C:/Program\ Files/Java/jdk1.7.0_57/include
  C:/Program\ Files/Java/jdk1.7.0_58/include
  C:/Program\ Files/Java/jdk1.7.0_59/include
  C:/Program\ Files/Java/jdk1.7.0_60/include
  C:/Program\ Files/Java/jdk1.7.0_61/include
  C:/Program\ Files/Java/jdk1.7.0_62/include
  C:/Program\ Files/Java/jdk1.7.0_63/include
  C:/Program\ Files/Java/jdk1.7.0_64/include
  C:/Program\ Files/Java/jdk1.7.0_65/include
  C:/Program\ Files/Java/jdk1.7.0_66/include
  C:/Program\ Files/Java/jdk1.7.0_67/include
  C:/Program\ Files/Java/jdk1.8.0/include
) 

# possible paths for: 'jni_md.h'
SET(TRIAL_PATHS_INC_JNI_MD
  C:/Program\ Files/Java/jdk1.7.0_40/include/win32
  C:/Program\ Files/Java/jdk1.7.0_41/include/win32
  C:/Program\ Files/Java/jdk1.7.0_42/include/win32
  C:/Program\ Files/Java/jdk1.7.0_43/include/win32
  C:/Program\ Files/Java/jdk1.7.0_44/include/win32
  C:/Program\ Files/Java/jdk1.7.0_45/include/win32
  C:/Program\ Files/Java/jdk1.7.0_46/include/win32
  C:/Program\ Files/Java/jdk1.7.0_47/include/win32
  C:/Program\ Files/Java/jdk1.7.0_48/include/win32
  C:/Program\ Files/Java/jdk1.7.0_49/include/win32
  C:/Program\ Files/Java/jdk1.7.0_50/include/win32
  C:/Program\ Files/Java/jdk1.7.0_51/include/win32
  C:/Program\ Files/Java/jdk1.7.0_52/include/win32
  C:/Program\ Files/Java/jdk1.7.0_53/include/win32
  C:/Program\ Files/Java/jdk1.7.0_54/include/win32
  C:/Program\ Files/Java/jdk1.7.0_55/include/win32
  C:/Program\ Files/Java/jdk1.7.0_56/include/win32
  C:/Program\ Files/Java/jdk1.7.0_57/include/win32
  C:/Program\ Files/Java/jdk1.7.0_58/include/win32
  C:/Program\ Files/Java/jdk1.7.0_59/include/win32
  C:/Program\ Files/Java/jdk1.7.0_60/include/win32
  C:/Program\ Files/Java/jdk1.7.0_61/include/win32
  C:/Program\ Files/Java/jdk1.7.0_62/include/win32
  C:/Program\ Files/Java/jdk1.7.0_63/include/win32
  C:/Program\ Files/Java/jdk1.7.0_64/include/win32
  C:/Program\ Files/Java/jdk1.7.0_65/include/win32
  C:/Program\ Files/Java/jdk1.7.0_66/include/win32
  C:/Program\ Files/Java/jdk1.7.0_67/include/win32
  C:/Program\ Files/Java/jdk1.8.0/include/win32
) 

# possible paths for: 'jvm.lib'
SET(TRIAL_PATHS_LIB
  C:/Program\ Files/Java/jdk1.7.0_40/lib
  C:/Program\ Files/Java/jdk1.7.0_41/lib
  C:/Program\ Files/Java/jdk1.7.0_42/lib
  C:/Program\ Files/Java/jdk1.7.0_43/lib
  C:/Program\ Files/Java/jdk1.7.0_44/lib
  C:/Program\ Files/Java/jdk1.7.0_45/lib
  C:/Program\ Files/Java/jdk1.7.0_46/lib
  C:/Program\ Files/Java/jdk1.7.0_47/lib
  C:/Program\ Files/Java/jdk1.7.0_48/lib
  C:/Program\ Files/Java/jdk1.7.0_49/lib
  C:/Program\ Files/Java/jdk1.7.0_50/lib
  C:/Program\ Files/Java/jdk1.7.0_51/lib
  C:/Program\ Files/Java/jdk1.7.0_52/lib
  C:/Program\ Files/Java/jdk1.7.0_53/lib
  C:/Program\ Files/Java/jdk1.7.0_54/lib
  C:/Program\ Files/Java/jdk1.7.0_55/lib
  C:/Program\ Files/Java/jdk1.7.0_56/lib
  C:/Program\ Files/Java/jdk1.7.0_57/lib
  C:/Program\ Files/Java/jdk1.7.0_58/lib
  C:/Program\ Files/Java/jdk1.7.0_59/lib
  C:/Program\ Files/Java/jdk1.7.0_60/lib
  C:/Program\ Files/Java/jdk1.7.0_61/lib
  C:/Program\ Files/Java/jdk1.7.0_62/lib
  C:/Program\ Files/Java/jdk1.7.0_63/lib
  C:/Program\ Files/Java/jdk1.7.0_64/lib
  C:/Program\ Files/Java/jdk1.7.0_65/lib
  C:/Program\ Files/Java/jdk1.7.0_66/lib
  C:/Program\ Files/Java/jdk1.7.0_67/lib
  C:/Program\ Files/Java/jdk1.8.0/lib
) 

FIND_PATH(JNI_INCLUDE_JNI jni.h ${TRIAL_PATHS_INC_JNI})
FIND_PATH(JNI_INCLUDE_JNI_MD jni_md.h ${TRIAL_PATHS_INC_JNI_MD})

FIND_LIBRARY(JNI_LIBRARIES jvm.lib ${TRIAL_PATHS_LIB})

SET(JNI_INCLUDE_PATH ${JNI_INCLUDE_JNI} ${JNI_INCLUDE_JNI_MD})

## ----------------- ##


ENDIF (UNIX)


# flag put to 1 if all required files are found
IF(JNI_INCLUDE_PATH AND JNI_LIBRARIES)
  SET(JNI_FOUND 1)
ELSE(JNI_INCLUDE_PATH AND JNI_LIBRARIES)
  SET(JNI_FOUND 0)
ENDIF(JNI_INCLUDE_PATH AND JNI_LIBRARIES)

# outputs
MARK_AS_ADVANCED(
  JNI_FOUND
  JNI_INCLUDE_PATH
  JNI_LIBRARIES 
)
