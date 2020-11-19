# ##############################################################################
# CMakeLists based on https://github.com/Bagira80/More-Modern-CMake
cmake_minimum_required(VERSION 3.13) # More-Modern-CMake
project(fvp_viewer)

# Always use '-fPIC'/'-fPIE' option.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Add warning level if you want
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 14)

# Build using multicore
if(MSVC)
  add_definitions(/MP)
endif()

# ##############################################################################
add_subdirectory(thirdparty)
add_subdirectory(src)
add_subdirectory(example)
