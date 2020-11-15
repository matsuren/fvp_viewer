# ##############################################################################
# The project name Setting the variables
get_filename_component(PROJ_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(REPLACE " " "_" PROJ_NAME ${PROJ_NAME})
message("\n*--- project name : ${PROJ_NAME} ---*")

# ##############################################################################
# spdlog
find_package(spdlog REQUIRED)

# ##############################################################################
# OpenGL
find_package(GLFW3 REQUIRED)
find_package(OpenGL REQUIRED)

# ##############################################################################
# Assimp
find_package(Assimp REQUIRED)

# ##############################################################################
# OpenCV
find_package(OpenCV REQUIRED)

# Add the files
file(
  GLOB_RECURSE PROJ_SOURCES
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  *.cpp *.cc *.c)
file(
  GLOB_RECURSE PROJ_HEADERS
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  *.hpp *.h)

# ##############################################################################
# Project
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
add_library(fvp_core STATIC) # SHARED
target_sources(
  fvp_core
  PRIVATE ${PROJ_SOURCES}
  PUBLIC ${PROJ_HEADERS})
target_include_directories(
  fvp_core
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/glad/include
         ${ASSIMP_INCLUDE_DIRS})
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
