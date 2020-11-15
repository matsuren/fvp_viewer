# ##############################################################################
# spdlog
find_package(spdlog REQUIRED)

# ##############################################################################
# OpenGL
find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)

# ##############################################################################
# Assimp
find_package(Assimp REQUIRED)

# ##############################################################################
# OpenCV
find_package(OpenCV REQUIRED)

# ##############################################################################
# Project
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
add_library(fvp_core STATIC) # SHARED
target_include_directories(
  fvp_core
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/glad/include
  PRIVATE ${ASSIMP_INCLUDE_DIRS})

set(FVP_SOURCES
    PRIVATE
    "fvp/config.cpp"
    "fvp/fvp_system.cpp"
    "fvp/gl_camera.cpp"
    "fvp/gl_data_manager.cpp"
    "fvp/gl_model_manager.cpp"
    "models/dome.cpp"
    "models/drawable.cpp"
    "models/mesh.cpp"
    "models/plane.cpp"
    "glad/src/glad.c"
    "calib/ocam_functions.cpp"
    "glslcookbook/glslprogram.cpp"
    "glslcookbook/glutils.cpp")
set(FVP_HEADERS
    PUBLIC
    "fvp/config.hpp"
    "fvp/fvp_system.hpp"
    "fvp/gl_camera.hpp"
    "fvp/gl_data_manager.hpp"
    "fvp/gl_model_manager.hpp"
    PRIVATE
    "models/dome.hpp"
    "models/drawable.hpp"
    "models/mesh.hpp"
    "models/plane.hpp"
    "calib/ocam_functions.hpp"
    "glad/include/KHR/khrplatform.h"
    "glad/include/glad/glad.h"
    "glslcookbook/cookbookogl.h"
    "glslcookbook/glslprogram.h"
    "glslcookbook/glutils.h")
target_sources(fvp_core ${FVP_SOURCES} ${FVP_HEADERS})
target_link_libraries(
  fvp_core PUBLIC ${OpenCV_LIBS} ${OPENGL_gl_LIBRARY} spdlog::spdlog
                  Thirdparty::HeaderOnly ${ASSIMP_LIBRARIES} glfw)
target_link_directories(fvp_core PUBLIC ${ASSIMP_LIBRARY_DIRS})
target_compile_features(fvp_core PUBLIC cxx_std_14)

# directory structure
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${FVP_SOURCES}
                                                       ${FVP_HEADERS})
