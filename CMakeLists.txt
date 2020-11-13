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




# ##############################################################################
# Spinnaker SDK
if(WIN32)
  set(Spinnaker_LIBS debug "Spinnakerd_v140.lib" optimized "Spinnaker_v140.lib")
  set(Spinnaker_INCLUDE_DIR
      "C:/Program Files/FLIR Systems/Spinnaker/include")
  link_directories(
    "C:/Program Files/FLIR Systems/Spinnaker/lib64/vs2015")
endif(WIN32)
if(UNIX)
  add_definitions(-DLINUX)
  set(Spinnaker_LIBS "libSpinnaker.so")
  set(Spinnaker_INCLUDE_DIR "/usr/include/spinnaker")
  link_directories("/usr/lib")
endif(UNIX)
list( APPEND PROJ_LIBS ${Spinnaker_LIBS} )


# ##############################################################################
# spdlog 
find_package(spdlog REQUIRED)
list( APPEND PROJ_LIBS spdlog::spdlog)

# ##############################################################################
#  rplidar SDK
set(rplidar True)
if(rplidar)
    if(WIN32)
      set(rplidar_LIBS 
      debug "C:/dev/rplidar_sdk/sdk/workspaces/vc14/x64/Debug/rplidar_driver.lib" 
      optimized "C:/dev/rplidar_sdk/sdk/workspaces/vc14/x64/Release/rplidar_driver.lib")
      set(rplidar_INCLUDE_DIR
          "C:/dev/rplidar_sdk/sdk/sdk/include" "C:/dev/rplidar_sdk/sdk/sdk/src")
    endif(WIN32)
endif(rplidar)
list( APPEND PROJ_LIBS ${rplidar_LIBS} )


# ##############################################################################
# thirdparty header only libraries
include_directories(${CMAKE_SOURCE_DIR}/thirdparty/glm)
include_directories(${CMAKE_SOURCE_DIR}/thirdparty/cereal/include)


######################################################################
# Recurse into the subdirectories. 
add_subdirectory( fvp_core )

