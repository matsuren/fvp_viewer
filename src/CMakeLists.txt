IF(POLICY CMP0012)
    CMAKE_POLICY(SET CMP0012 NEW)
ENDIF()

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
  PUBLIC "${fvp_viewer_SOURCE_DIR}/include"
  PRIVATE ${ASSIMP_INCLUDE_DIRS})

#message("fvp_viewer_SOURCE_DIR:${fvp_viewer_SOURCE_DIR}")

set(FVP_SOURCES
    "fvp/config.cpp"
    "fvp/fvp_system.cpp"
    "fvp/gl_camera.cpp"
    "fvp/gl_data_manager.cpp"
    "fvp/gl_model_manager.cpp"
    "models/dome.cpp"
    "models/drawable.cpp"
    "models/mesh.cpp"
    "models/plane.cpp"
    "glad/glad.c"
    "calib/ocam_functions.cpp"
    "glslcookbook/glslprogram.cpp"
    "glslcookbook/glutils.cpp")
set(FVP_HEADERS
    "${fvp_viewer_SOURCE_DIR}/include/fvp/config.hpp"
    "${fvp_viewer_SOURCE_DIR}/include/fvp/fvp_system.hpp"
    "${fvp_viewer_SOURCE_DIR}/include/fvp/gl_camera.hpp"
    "${fvp_viewer_SOURCE_DIR}/include/fvp/gl_data_manager.hpp"
    "${fvp_viewer_SOURCE_DIR}/include/fvp/gl_model_manager.hpp"
    "${fvp_viewer_SOURCE_DIR}/include/models/dome.hpp"
    "${fvp_viewer_SOURCE_DIR}/include/models/drawable.hpp"
    "${fvp_viewer_SOURCE_DIR}/include/models/mesh.hpp"
    "${fvp_viewer_SOURCE_DIR}/include/models/plane.hpp"
    "${fvp_viewer_SOURCE_DIR}/include/calib/ocam_functions.hpp"
    "${fvp_viewer_SOURCE_DIR}/include/KHR/khrplatform.h"
    "${fvp_viewer_SOURCE_DIR}/include/glad/glad.h"
    "${fvp_viewer_SOURCE_DIR}/include/glslcookbook/cookbookogl.h"
    "${fvp_viewer_SOURCE_DIR}/include/glslcookbook/glslprogram.h"
    "${fvp_viewer_SOURCE_DIR}/include/glslcookbook/glutils.h")
target_sources(fvp_core PRIVATE ${FVP_SOURCES} PUBLIC ${FVP_HEADERS})
target_link_libraries(
  fvp_core PUBLIC ${OpenCV_LIBS} ${OPENGL_gl_LIBRARY} spdlog::spdlog
                  Thirdparty::HeaderOnly ${ASSIMP_LIBRARIES} glfw)
target_link_directories(fvp_core PUBLIC ${ASSIMP_LIBRARY_DIRS})
target_compile_features(fvp_core PUBLIC cxx_std_14)

# directory structure
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${FVP_SOURCES})
source_group(TREE "${fvp_viewer_SOURCE_DIR}/include" FILES ${FVP_HEADERS})
