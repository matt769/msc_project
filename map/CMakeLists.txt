cmake_minimum_required(VERSION 2.8)
project(map)

set(CMAKE_CXX_STANDARD 11)
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()
SET(CMAKE_CXX_FLAGS "-Wall")

find_package(Eigen3 REQUIRED)

include_directories(
        ${EIGEN3_INCLUDE_DIR}
)

add_library(GridMap STATIC GridMap.cpp OverlappingGridMap.cpp)
add_library(MockMap STATIC MockMap.cpp)

add_executable(example_map example_map.cpp)
target_link_libraries(example_map
        GridMap
        ${EIGEN3_LIBS}
)

add_executable(test test.cpp)
target_link_libraries(test
        GridMap
	    MockMap
        ${EIGEN3_LIBS}
        )


add_executable(test_overlap test_overlap.cpp)
target_link_libraries(test_overlap
        GridMap
        ${EIGEN3_LIBS}
        )
