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
  PRIVATE ${ASSIMP_INCLUDE_DIRS}
         )
target_sources(
  fvp_core
  PRIVATE "fvp/config.cpp"
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
          "glslcookbook/glutils.cpp"
  PUBLIC "fvp/config.hpp" 
         "fvp/fvp_sytem.hpp"
         "fvp/gl_camera.hpp" 
         "fvp/gl_data_manager.hpp"
         "fvp/gl_model_manager.hpp"
  PRIVATE "models/dome.hpp"
          "models/drawable.hpp"
          "models/mesh.hpp"
          "models/plane.hpp"
          "calib/ocam_functions.hpp"
          "glad/include/KHR/khrplatform.h"
          "glad/include/glad/glad.h"
          "glslcookbook/cookbookogl.h"
          "glslcookbook/glslprogram.h"
          "glslcookbook/glutils.h")
target_link_libraries(
  fvp_core PUBLIC ${OpenCV_LIBS} ${OPENGL_gl_LIBRARY} spdlog::spdlog
                  Thirdparty::HeaderOnly ${ASSIMP_LIBRARIES} glfw)
target_link_directories(fvp_core PUBLIC ${ASSIMP_LIBRARY_DIRS})

# directory structure
set(source_lists ${PROJ_SOURCES} ${PROJ_HEADERS})
foreach(source IN LISTS source_lists)
  get_filename_component(source_path "${source}" PATH)
  string(REPLACE "/" "\\" source_path_msvc "${source_path}")
  source_group("${source_path_msvc}" FILES "${source}")
endforeach()
