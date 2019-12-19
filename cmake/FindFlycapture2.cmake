# Locate the Assimp library
#
# This module defines the following variables:
#
# Flycapture2_LIBRARY      the name of the library;
# Flycapture2_INCLUDE_DIR  where to find include files.
# Flycapture2_FOUND        true if both the Flycapture2_LIBRARY and Flycapture2_INCLUDE_DIR have been found.
#
######################################################################
# PointGrey Camera
if(WIN32)
  if(NOT CMAKE_CL_64)
    set( PROJ_PG_PATH "C:/Program Files (x86)/Point Grey Research/FlyCapture2")
    set( LIB_PATH "lib")
  else()
    set( PROJ_PG_PATH "C:/Program Files/Point Grey Research/FlyCapture2")
    set( LIB_PATH "lib64")
  endif()
endif()

set(Flycapture2_RUNTIME_DIR "vs2015")
SET( Flycapture2_INCLUDE_DIR   "${PROJ_PG_PATH}/include")
SET( Flycapture2_LIBRARY 
  debug "${PROJ_PG_PATH}/${LIB_PATH}/${Flycapture2_RUNTIME_DIR}/FlyCapture2d_v140.lib"
  optimized "${PROJ_PG_PATH}/${LIB_PATH}/${Flycapture2_RUNTIME_DIR}/FlyCapture2_v140.lib")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Flycapture2 DEFAULT_MSG
                                  Flycapture2_LIBRARY Flycapture2_INCLUDE_DIR)

