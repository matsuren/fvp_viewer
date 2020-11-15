cmake_minimum_required(VERSION 3.13)

# Thirdparty header only libraries
add_library(Thirdparty::HeaderOnly INTERFACE IMPORTED GLOBAL)
target_include_directories(
  Thirdparty::HeaderOnly SYSTEM
  INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/glm
            ${CMAKE_CURRENT_SOURCE_DIR}/cereal/include)
