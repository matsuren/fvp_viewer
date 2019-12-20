######################################################################
# The project name
cmake_minimum_required (VERSION 3.0)
cmake_policy(SET CMP0012 NEW)
get_filename_component(ProjectId ${CMAKE_SOURCE_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})
message(Project Id : ${ProjectId})
project (${ProjectId})
include_directories(${CMAKE_SOURCE_DIR})
message(------project name : ${ProjectId} ---------)

######################################################################
# Visual Studio Options
if(WIN32)
  # Displays warnings and disables compiler warning C4996 and C4819 ...
  add_definitions (/W3 /wd4996 /wd4819)
  # Multiprocessor compilation
  set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} /MP")
  # Enable handle addresses larger than 2 gigabytes
  if(NOT CMAKE_CL_64)
    SET(CMAKE_EXE_LINKER_FLAGS "/LARGEADDRESSAWARE ${CMAKE_EXE_LINKER_FLAGS}")
  endif(NOT CMAKE_CL_64)
endif(WIN32)

######################################################################
# Options
option (USE_STATIC "Use static library" OFF)
# set(USE_STATIC OFF CACHE BOOL "use opencv static libs")

######################################################################
# Static link
IF(USE_STATIC)
  if(WIN32)
    message (STATUS " *** Static link.... ***")
    set(CMAKE_EXE_LINKER_FLAGS
      "${CMAKE_EXE_LINKER_FLAGS} /NODEFAULTLIB:msvcrt.lib")

    #We statically link to reduce dependancies
    foreach(flag_var CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO)
      if(${flag_var} MATCHES "/MD")
        string(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
      endif(${flag_var} MATCHES "/MD")
      if(${flag_var} MATCHES "/MDd")
        string(REGEX REPLACE "/MDd" "/MTd" ${flag_var} "${${flag_var}}")
      endif(${flag_var} MATCHES "/MDd")
    endforeach(flag_var)
  endif(WIN32)
ENDIF(USE_STATIC)

######################################################################
# CMake Options
set( CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake )

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

######################################################################
# OpenGL 
find_package( GLFW3 REQUIRED )
find_package( OpenGL REQUIRED )
list( APPEND PROJ_LIBS ${OPENGL_gl_LIBRARY} glfw)

include_directories( ${PROJ_INCLUDE} )

######################################################################
# Assimp
find_package(Assimp REQUIRED)
include_directories(${ASSIMP_INCLUDE_DIRS})
link_directories(${ASSIMP_LIBRARY_DIRS})
list( APPEND PROJ_LIBS ${ASSIMP_LIBRARIES} )

######################################################################
# OpenCV 3.1
if(USE_STATIC)
  set(OpenCV_STATIC ON)
else()
  set(OpenCV_STATIC OFF)
endif(USE_STATIC)
find_package( OpenCV REQUIRED)
# Please set "target_link_libraries( ${PROJ_NAME}  ${OpenCV_LIBS})"

######################################################################
# PointGrey Camera Flycapture2
find_package(Flycapture2 REQUIRED)
include_directories(${Flycapture2_INCLUDE_DIR})
list( APPEND PROJ_LIBS ${Flycapture2_LIBRARY} )

######################################################################
# Recurse into the subdirectories. 
add_subdirectory( viewer )

