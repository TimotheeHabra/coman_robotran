#
# author: Nicolas Van der Noot
# Feb 27 2014
#
# This file finds the libraries
# related to Java 3D
#
# J3D_FOUND : 1 if all required files found (0 otherwise)
# J3D_PATH  : libraries -> for linkage or during the execution (Windows)
#


IF (UNIX)

IF (APPLE)
  

## ---- MAC OS ---- ##

# possible paths for: 'j3dcore.jar'
SET(TRIAL_PATHS_DLL
  /Library/Java/JavaVirtualMachines/jdk1.7.0_40.jdk/Contents/Home/jre/lib/ext
  /Library/Java/JavaVirtualMachines/jdk1.7.0_41.jdk/Contents/Home/jre/lib/ext
  /Library/Java/JavaVirtualMachines/jdk1.7.0_42.jdk/Contents/Home/jre/lib/ext
  /Library/Java/JavaVirtualMachines/jdk1.7.0_43.jdk/Contents/Home/jre/lib/ext
  /Library/Java/JavaVirtualMachines/jdk1.7.0_44.jdk/Contents/Home/jre/lib/ext
  /Library/Java/JavaVirtualMachines/jdk1.7.0_45.jdk/Contents/Home/jre/lib/ext
  /Library/Java/JavaVirtualMachines/jdk1.7.0_46.jdk/Contents/Home/jre/lib/ext
  /Library/Java/JavaVirtualMachines/jdk1.7.0_47.jdk/Contents/Home/jre/lib/ext
  /Library/Java/JavaVirtualMachines/jdk1.7.0_48.jdk/Contents/Home/jre/lib/ext
  /Library/Java/JavaVirtualMachines/jdk1.7.0_49.jdk/Contents/Home/jre/lib/ext
  /Library/Java/JavaVirtualMachines/jdk1.7.0_50.jdk/Contents/Home/jre/lib/ext
  /Library/Java/JavaVirtualMachines/jdk1.7.0_51.jdk/Contents/Home/jre/lib/ext
  /Library/Java/JavaVirtualMachines/jdk1.7.0_52.jdk/Contents/Home/jre/lib/ext
  /Library/Java/JavaVirtualMachines/jdk1.7.0_53.jdk/Contents/Home/jre/lib/ext
  /Library/Java/JavaVirtualMachines/jdk1.7.0_54.jdk/Contents/Home/jre/lib/ext
  /Library/Java/JavaVirtualMachines/jdk1.7.0_55.jdk/Contents/Home/jre/lib/ext
  /Library/Java/JavaVirtualMachines/jdk1.7.0_56.jdk/Contents/Home/jre/lib/ext
  /Library/Java/JavaVirtualMachines/jdk1.7.0_57.jdk/Contents/Home/jre/lib/ext
  /Library/Java/JavaVirtualMachines/jdk1.7.0_58.jdk/Contents/Home/jre/lib/ext
  /Library/Java/JavaVirtualMachines/jdk1.7.0_59.jdk/Contents/Home/jre/lib/ext
  /Library/Java/JavaVirtualMachines/jdk1.7.0_60.jdk/Contents/Home/jre/lib/ext
  /Library/Java/JavaVirtualMachines/jdk1.7.0_61.jdk/Contents/Home/jre/lib/ext
  /Library/Java/JavaVirtualMachines/jdk1.7.0_62.jdk/Contents/Home/jre/lib/ext
  /Library/Java/JavaVirtualMachines/jdk1.7.0_63.jdk/Contents/Home/jre/lib/ext
  /Library/Java/JavaVirtualMachines/jdk1.7.0_64.jdk/Contents/Home/jre/lib/ext
  /Library/Java/JavaVirtualMachines/jdk1.7.0_65.jdk/Contents/Home/jre/lib/ext
  /Library/Java/JavaVirtualMachines/jdk1.7.0_66.jdk/Contents/Home/jre/lib/ext
  /Library/Java/JavaVirtualMachines/jdk1.7.0_67.jdk/Contents/Home/jre/lib/ext
) 

FIND_PATH(J3D_PATH j3dcore.jar ${TRIAL_PATHS_DLL})

## ---------------- ##


ELSE (APPLE)
  

## ---- LINUX ---- ## 

# possible paths for: 'libj3dcore-ogl.so'
SET(TRIAL_PATHS_DLL
  /usr/lib/jvm/java-7-openjdk-amd64/jre/bin
  /usr/lib/jvm/java-6-sun/jre/bin
) 

FIND_PATH(J3D_PATH libj3dcore-ogl.so ${TRIAL_PATHS_DLL})

## --------------- ##


ENDIF (APPLE)

ELSE (UNIX)


## ---- WINDOWS ---- ##

# possible paths for: 'j3dcore-ogl.dll'
SET(TRIAL_PATHS_DLL
  C:/Program\ Files/Java/jdk1.7.0_40/jre/bin
  C:/Program\ Files/Java/jdk1.7.0_41/jre/bin
  C:/Program\ Files/Java/jdk1.7.0_42/jre/bin
  C:/Program\ Files/Java/jdk1.7.0_43/jre/bin
  C:/Program\ Files/Java/jdk1.7.0_44/jre/bin
  C:/Program\ Files/Java/jdk1.7.0_45/jre/bin
  C:/Program\ Files/Java/jdk1.7.0_46/jre/bin
  C:/Program\ Files/Java/jdk1.7.0_47/jre/bin
  C:/Program\ Files/Java/jdk1.7.0_48/jre/bin
  C:/Program\ Files/Java/jdk1.7.0_49/jre/bin
  C:/Program\ Files/Java/jdk1.7.0_50/jre/bin
  C:/Program\ Files/Java/jdk1.7.0_51/jre/bin
  C:/Program\ Files/Java/jdk1.7.0_52/jre/bin
  C:/Program\ Files/Java/jdk1.7.0_53/jre/bin
  C:/Program\ Files/Java/jdk1.7.0_54/jre/bin
  C:/Program\ Files/Java/jdk1.7.0_55/jre/bin
  C:/Program\ Files/Java/jdk1.7.0_56/jre/bin
  C:/Program\ Files/Java/jdk1.7.0_57/jre/bin
  C:/Program\ Files/Java/jdk1.7.0_58/jre/bin
  C:/Program\ Files/Java/jdk1.7.0_59/jre/bin
  C:/Program\ Files/Java/jdk1.7.0_60/jre/bin
  C:/Program\ Files/Java/jdk1.7.0_61/jre/bin
  C:/Program\ Files/Java/jdk1.7.0_62/jre/bin
  C:/Program\ Files/Java/jdk1.7.0_63/jre/bin
  C:/Program\ Files/Java/jdk1.7.0_64/jre/bin
  C:/Program\ Files/Java/jdk1.7.0_65/jre/bin
  C:/Program\ Files/Java/jdk1.7.0_66/jre/bin
  C:/Program\ Files/Java/jdk1.7.0_67/jre/bin
  C:/Program\ Files/Java/jdk1.8.0/jre/bin
  C:/Program\ Files/Java/Java3D/1.5.2/bin
) 

FIND_PATH(J3D_PATH j3dcore-ogl.dll ${TRIAL_PATHS_DLL})

## ----------------- ##


ENDIF (UNIX)


# flag put to 1 if all required files are found
IF(J3D_PATH)
  SET(J3D_FOUND 1)
ELSE(J3D_PATH)
  SET(J3D_FOUND 0)
ENDIF(J3D_PATH)

# outputs
MARK_AS_ADVANCED(
  J3D_FOUND
  J3D_PATH 
)
