# ##############################################################################
# CMakeLists based on https://github.com/Bagira80/More-Modern-CMake
cmake_minimum_required(VERSION 3.13) # More-Modern-CMake
project(fvp_viewer)

# Always use '-fPIC'/'-fPIE' option.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Add warning level if you want
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 14)

if(MSVC)
  # Build using multicore
  add_definitions(/MP)
  # Second, for multi-config builds (e.g. msvc)
  foreach(OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG}
        ${CMAKE_BINARY_DIR}/${OUTPUTCONFIG})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_BINARY_DIR}/lib)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_BINARY_DIR}/lib)
  endforeach(OUTPUTCONFIG CMAKE_CONFIGURATION_TYPES)
endif()

option(WITH_SENSORS "ON: Use sensors. OFF: Use only sample images." ON)
if(WITH_SENSORS)
  option(WITH_LRF "ON: Use LRF in indoor environment" ON)
endif()

# ##############################################################################
add_subdirectory(thirdparty)
add_subdirectory(src)
add_subdirectory(example)
