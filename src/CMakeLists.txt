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
if(NOT DEFINED CMAKE_DEBUG_POSTFIX)
  set(CMAKE_DEBUG_POSTFIX "-d")
endif()

# ##############################################################################
# Project
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
#add_library(fvp_core STATIC) # SHARED
add_library(fvp_core SHARED) # STATIC
target_include_directories(
        fvp_core
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
        $<INSTALL_INTERFACE:include/fvp_core>
        PRIVATE ${ASSIMP_INCLUDE_DIRS})

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
        "${CMAKE_CURRENT_SOURCE_DIR}/../include/fvp/config.hpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/../include/fvp/fvp_system.hpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/../include/fvp/gl_camera.hpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/../include/fvp/gl_data_manager.hpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/../include/fvp/gl_model_manager.hpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/../include/models/dome.hpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/../include/models/drawable.hpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/../include/models/mesh.hpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/../include/models/plane.hpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/../include/calib/ocam_functions.hpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/../include/KHR/khrplatform.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/../include/glad/glad.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/../include/glslcookbook/cookbookogl.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/../include/glslcookbook/glslprogram.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/../include/glslcookbook/glutils.h")
target_sources(fvp_core PRIVATE ${FVP_SOURCES} PRIVATE ${FVP_HEADERS} INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include/fvp/fvp_system.hpp>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include/fvp/config.hpp>
        $<INSTALL_INTERFACE:include/fvp_core/fvp/fvp_system.hpp>
        $<INSTALL_INTERFACE:include/fvp_core/fvp/config.hpp>
        )
target_link_libraries(
        fvp_core PRIVATE ${OpenCV_LIBS} ${OPENGL_gl_LIBRARY} spdlog::spdlog
        Thirdparty::HeaderOnly ${ASSIMP_LIBRARIES} glfw)
target_link_directories(fvp_core PRIVATE ${ASSIMP_LIBRARY_DIRS})
target_compile_features(fvp_core PUBLIC cxx_std_14)

# directory structure
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${FVP_SOURCES})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../include" FILES ${FVP_HEADERS})

# install
install(TARGETS fvp_core
        EXPORT fvp_core-config
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        )
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../include/" DESTINATION include/fvp_core)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/glm/glm/" DESTINATION include/fvp_core/glm)
install(EXPORT fvp_core-config DESTINATION share/fvp_core/cmake)

# This makes the project importable from the build directory
export(TARGETS fvp_core FILE fvp_core-config.cmake)