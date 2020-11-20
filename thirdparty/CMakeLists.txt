cmake_minimum_required(VERSION 3.13)

# Thirdparty header only libraries
add_library(Thirdparty::HeaderOnly INTERFACE IMPORTED GLOBAL)
target_include_directories(
  Thirdparty::HeaderOnly SYSTEM
  INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/glm
            ${CMAKE_CURRENT_SOURCE_DIR}/cereal/include)

if(WITH_SENSORS)
  # ############################################################################
  # URG
  add_library(urg_lib OBJECT)
  target_sources(
    urg_lib
    PRIVATE "urg_library/current/samples/cpp/Connection_information.cpp"
            "urg_library/current/src/ticks.cpp"
            "urg_library/current/src/Urg_driver.cpp"
            "urg_library/current/src/urg_connection.c"
            "urg_library/current/src/urg_debug.c"
            "urg_library/current/src/urg_ring_buffer.c"
            "urg_library/current/src/urg_sensor.c"
            "urg_library/current/src/urg_serial.c"
            "urg_library/current/src/urg_serial_utils_windows.c"
            "urg_library/current/src/urg_tcpclient.c"
            "urg_library/current/src/urg_utils.c"
    PUBLIC "urg_library/current/samples/cpp/Connection_information.h"
           "urg_library/current/include/cpp/Urg_driver.h"
    PRIVATE "urg_library/current/include/cpp/Lidar.h"
            "urg_library/current/include/cpp/detect_os.h"
            "urg_library/current/include/cpp/math_utilities.h"
            "urg_library/current/include/cpp/ticks.h"
    PRIVATE "urg_library/current/include/c/urg_connection.h"
            "urg_library/current/include/c/urg_debug.h"
            "urg_library/current/include/c/urg_detect_os.h"
            "urg_library/current/include/c/urg_errno.h"
            "urg_library/current/include/c/urg_ring_buffer.h"
            "urg_library/current/include/c/urg_sensor.h"
            "urg_library/current/include/c/urg_serial.h"
            "urg_library/current/include/c/urg_serial_utils.h"
            "urg_library/current/include/c/urg_tcpclient.h"
            "urg_library/current/include/c/urg_utils.h")

  target_link_libraries(urg_lib PRIVATE setupapi.lib ws2_32.lib comctl32.lib)
  target_include_directories(
    urg_lib
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/urg_library/current/include/cpp"
           "${CMAKE_CURRENT_SOURCE_DIR}/urg_library/current/include/c"
           "${CMAKE_CURRENT_SOURCE_DIR}/urg_library/current/samples/cpp")
endif(WITH_SENSORS)
