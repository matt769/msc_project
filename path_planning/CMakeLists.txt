cmake_minimum_required(VERSION 2.8)
project(pathplanning)

set(CMAKE_CXX_STANDARD 11)
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()
SET(CMAKE_CXX_FLAGS "-Wall")

find_package(Eigen3 REQUIRED)

include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/../map
)

add_library(PathPlanning STATIC astar.cpp)

link_directories(${CMAKE_SOURCE_DIR}/../map/)

add_executable(tests tests.cpp)
target_link_libraries(tests
        PathPlanning
        GridMap
        ${EIGEN3_LIBS}
        )

add_executable(tests2 tests2.cpp)
target_link_libraries(tests2
        PathPlanning
        GridMap
        MockMap
        ${EIGEN3_LIBS}
        )