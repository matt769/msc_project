cmake_minimum_required(VERSION 2.8)
project(communications)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)
SET(CMAKE_CXX_FLAGS "-pthread -Wall")

add_library(communications STATIC communications.cpp)
add_executable(example_basic_control basic_control.cpp)
target_link_libraries(example_basic_control communications)

