# - Find XERCES
# Find the native XERCES includes and library
# This module defines
#  XERCES_INCLUDE_DIR, where to find acml.h, etc.
#  XERCES_LIBRARIES, the libraries needed to use XERCES.
#  XERCES_FOUND, If false, do not try to use XERCES.
# also defined, but not for general use are
#  XERCES_LIBRARY, where to find the XERCES library.

FIND_PATH(XERCES_INCLUDE_DIR 
	/xercesc/util/XercesVersion.hpp
	"${XERCES_DIR}\\include"
	"C:/Program\ Files\ (x86)/CodeSynthesis\ XSD\ 4.0/include"
      "C:/Programmi/CodeSynthesis\ XSD\ 3.3/include" 
 )
 
MESSAGE(STATUS "XERCES Include Dir ${XERCES_INCLUDE_DIR}")
 
FIND_LIBRARY(XERCES_LIBRARY
  NAMES  
      xerces-c
      xerces-c_2
	  xerces-c_3
	  xerces-c_3_1
	  xerces-c_3_1_vc100
  PATHS 
    "${XERCES_DIR}\\lib"
    /usr/local/lib
    /usr/lib 
    /usr/lib/x86_64-linux-gnu/ 
	"C:/Program\ Files\ (x86)/CodeSynthesis\ XSD\ 4.0/bin"
	"C:/Program\ Files\ (x86)/CodeSynthesis\ XSD\ 4.0/lib64/vc-12.0"
	"C:/Programmi/CodeSynthesis\ XSD\ 3.3/bin"
	"C:/Programmi/CodeSynthesis\ XSD\ 3.3/lib/vc-10.0"
  )

  MESSAGE(STATUS "XERCES Lib Dir ${XERCES_LIBRARY}")
  
IF (XERCES_LIBRARY AND XERCES_INCLUDE_DIR)
    SET(XERCES_LIBRARIES ${XERCES_LIBRARY})
    SET(XERCES_FOUND "YES")
ELSE (XERCES_LIBRARY AND XERCES_INCLUDE_DIR)
  SET(XERCES_FOUND "NO")
ENDIF (XERCES_LIBRARY AND XERCES_INCLUDE_DIR)

IF (XERCES_FOUND)
   IF (NOT XERCES_FIND_QUIETLY)
      MESSAGE(STATUS "Found XERCES: ${XERCES_LIBRARIES}")
   ENDIF (NOT XERCES_FIND_QUIETLY)
ELSE (XERCES_FOUND)
   IF (XERCES_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find XERCES library")
   ENDIF (XERCES_FIND_REQUIRED)
ENDIF (XERCES_FOUND)

# Deprecated declarations.
GET_FILENAME_COMPONENT (NATIVE_XERCES_LIB_PATH ${XERCES_LIBRARY} PATH)

MARK_AS_ADVANCED(
  XERCES_LIBRARY
  XERCES_INCLUDE_DIR
  )
