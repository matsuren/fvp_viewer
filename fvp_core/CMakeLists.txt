######################################################################
# The project name
#Setting the variables
get_filename_component(PROJ_NAME ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" PROJ_NAME ${PROJ_NAME})
message( "\n*--- project name : ${PROJ_NAME} ---*")
include_directories(${CMAKE_CURRENT_LIST_DIR})

#Setting the output folder
# First for the generic no-config case (e.g. with mingw)
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib )
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib )
# Second, for multi-config builds (e.g. msvc)
foreach( OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES} )
    string( TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG )
    set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_BINARY_DIR}/${OUTPUTCONFIG} )  
    set( CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_BINARY_DIR}/lib )
    set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_BINARY_DIR}/lib )
endforeach( OUTPUTCONFIG CMAKE_CONFIGURATION_TYPES )


######################################################################
# OpenGL 
find_package( GLFW3 REQUIRED )
find_package( OpenGL REQUIRED )
list( APPEND PROJ_LIBS ${OPENGL_gl_LIBRARY} glfw)
include_directories(${CMAKE_CURRENT_LIST_DIR}/glad/include)

######################################################################
# Assimp
find_package(Assimp REQUIRED)
include_directories(${ASSIMP_INCLUDE_DIRS})
link_directories(${ASSIMP_LIBRARY_DIRS})
list( APPEND PROJ_LIBS ${ASSIMP_LIBRARIES} )

######################################################################
# OpenCV 3.1
if(USE_STATIC)
  set(OpenCV_STATIC ON)
else()
  set(OpenCV_STATIC OFF)
endif(USE_STATIC)
find_package( OpenCV REQUIRED)
# Please set "target_link_libraries( ${PROJ_NAME}  ${OpenCV_LIBS})"


#Add the files
file( GLOB_RECURSE PROJ_SOURCES RELATIVE ${CMAKE_CURRENT_LIST_DIR} *.cpp *.cc *.c)
file( GLOB_RECURSE PROJ_HEADERS RELATIVE ${CMAKE_CURRENT_LIST_DIR}  *.hpp *.h)

# URG dependency library
include_directories(${CMAKE_CURRENT_LIST_DIR}/urg)
set(URG_LIBS setupapi.lib ws2_32.lib comctl32.lib)

######################################################################
# Project
add_executable( ${PROJ_NAME} ${PROJ_SOURCES}  ${PROJ_HEADERS})
include_directories( ${PROJ_INCLUDES} 
  "${CMAKE_CURRENT_LIST_DIR}/sensors" "${CMAKE_CURRENT_LIST_DIR}/glslcookbook"
  "${CMAKE_CURRENT_LIST_DIR}/models" "${CMAKE_CURRENT_LIST_DIR}/utils"
   ${Spinnaker_INCLUDE_DIR} ${rplidar_INCLUDE_DIR}
  )
target_link_libraries( ${PROJ_NAME} ${PROJ_LIBS} ${OpenCV_LIBS} ${URG_LIBS})
# directory structure    
set(source_lists ${PROJ_SOURCES} ${PROJ_HEADERS})
foreach(source IN LISTS source_lists)
  get_filename_component(source_path "${source}" PATH)
  string(REPLACE "/" "\\" source_path_msvc "${source_path}")
  source_group("${source_path_msvc}" FILES "${source}")
endforeach()
