# ##############################################################################
# CMakeLists based on https://github.com/Bagira80/More-Modern-CMake
cmake_minimum_required(VERSION 3.13) # More-Modern-CMake
cmake_policy(SET CMP0012 NEW)
get_filename_component(ProjectId ${CMAKE_SOURCE_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})
message(Project Id : ${ProjectId})
project(${ProjectId})
message(------project name : ${ProjectId} ---------)

# Always use '-fPIC'/'-fPIE' option.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Add warning level if you want
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 11)

# Build using multicore
if(MSVC)
  add_definitions(/MP)
endif()

# ##############################################################################
add_subdirectory(thirdparty)
add_subdirectory(fvp_core)
add_subdirectory(example)
