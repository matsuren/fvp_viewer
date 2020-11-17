cmake_minimum_required(VERSION 3.0.2)
project(fvp_viewer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(fvp_core REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fvp_viewer
#  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS} "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

## Declare a C++ executable
add_executable(fvp_node src/fvp_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(fvp_node
  fvp_core
  ${catkin_LIBRARIES} ${OpenCV_LIBS}
)

add_executable(ros_practice_cam src/ros_practice_cam.cpp)
target_link_libraries(ros_practice_cam
        ${catkin_LIBRARIES} ${OpenCV_LIBS}
        )
add_executable(ros_practice_lrf src/ros_practice_lrf.cpp)
target_link_libraries(ros_practice_lrf
        ${catkin_LIBRARIES} ${OpenCV_LIBS}
        )