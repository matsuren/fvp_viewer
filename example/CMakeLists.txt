# ##############################################################################
# CMakeLists based on https://github.com/Bagira80/More-Modern-CMake
cmake_minimum_required(VERSION 3.13) # More-Modern-CMake
project(fvp_example)

# ##############################################################################
# OpenCV 3.1
find_package(OpenCV REQUIRED)

# ##############################################################################
# spdlog
find_package(spdlog REQUIRED)

# ##############################################################################
# Project
add_executable(simple_example)
set(EXAMPLE_SOURCES PRIVATE "simple_example.cpp")
target_sources(simple_example ${EXAMPLE_SOURCES})
target_link_libraries(
  simple_example
  PUBLIC fvp_core
  PRIVATE spdlog::spdlog ${OpenCV_LIBS} Thirdparty::HeaderOnly)
target_include_directories(simple_example PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
# directory structure
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${EXAMPLE_SOURCES})


################################################################################
## with sensors
################################################################################
if(WITH_SENSORS)
  # ############################################################################
  # Spinnaker SDK
  if(WIN32)
    set(Spinnaker_LIBS debug "Spinnakerd_v140.lib" optimized
                       "Spinnaker_v140.lib")
    set(Spinnaker_INCLUDE_DIR "C:/Program Files/FLIR Systems/Spinnaker/include")
    link_directories("C:/Program Files/FLIR Systems/Spinnaker/lib64/vs2015")
  endif(WIN32)
  if(UNIX)
    add_definitions(-DLINUX)
    set(Spinnaker_LIBS "libSpinnaker.so")
    set(Spinnaker_INCLUDE_DIR "/usr/include/spinnaker")
    link_directories("/usr/lib")
  endif(UNIX)

  # ############################################################################
  # rplidar SDK
  set(rplidar True)
  if(rplidar)
    if(WIN32)
      set(rplidar_LIBS
          debug
          "C:/dev/rplidar_sdk/sdk/workspaces/vc14/x64/Debug/rplidar_driver.lib"
          optimized
          "C:/dev/rplidar_sdk/sdk/workspaces/vc14/x64/Release/rplidar_driver.lib"
      )
      set(rplidar_INCLUDE_DIR "C:/dev/rplidar_sdk/sdk/sdk/include"
                              "C:/dev/rplidar_sdk/sdk/sdk/src")
    endif(WIN32)
  endif(rplidar)

  # ############################################################################
  # Example with sensor
  add_executable(sensor_example)
  set(SENSOR_SOURCES
      PRIVATE "sensor_example.cpp" "sensors/sensor_manager.cpp" "sensors/file_lrf.cpp"
      "sensors/rplidar_lrf.cpp" "sensors/spincamera.cpp" "sensors/urg_lrf.cpp")
  set(SENSOR_HEADERS
      PRIVATE
      "FpsDisplayer.hpp"
      "sensors/sensor_manager.hpp"
      "sensors/file_lrf.hpp"
      "sensors/rplidar_lrf.hpp"
      "sensors/spincamera.hpp"
      "sensors/spinmanager.hpp"
      "sensors/urg_lrf.hpp")
  target_sources(sensor_example ${SENSOR_SOURCES} ${SENSOR_HEADERS})
  target_link_libraries(
    sensor_example
    PUBLIC fvp_core
    PUBLIC spdlog::spdlog ${OpenCV_LIBS} ${rplidar_LIBS} ${Spinnaker_LIBS}
          Thirdparty::HeaderOnly urg_lib)
  target_include_directories(
    sensor_example PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${Spinnaker_INCLUDE_DIR}
                          ${rplidar_INCLUDE_DIR})
  # directory structure
  source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${SENSOR_SOURCES}
                                                         ${SENSOR_HEADERS})
endif(WITH_SENSORS)
