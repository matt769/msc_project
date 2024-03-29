cmake_minimum_required(VERSION 2.8)
project(Full)


IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")


# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()




find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
         ${OpenCV_INCLUDE_DIRS}
         ../../ORB_SLAM2_PRJ/
         ../../ORB_SLAM2_PRJ/include
         ../../ORB_SLAM2_PRJ/Thirdparty/DBoW2/DBoW2
         ../../ORB_SLAM2_PRJ/Thirdparty/g2o/g2o
         ${EIGEN3_INCLUDE_DIR}
         ${Pangolin_INCLUDE_DIRS}
         /usr/include/flycapture
         ~/ws/FlyCap/flycapture.2.13.3.31_armhf/include
         ${CMAKE_SOURCE_DIR}/../communication
        ${CMAKE_SOURCE_DIR}/../map
        ${CMAKE_SOURCE_DIR}/../path_planning
)


link_directories(${CMAKE_SOURCE_DIR}/../communication/
        ${CMAKE_SOURCE_DIR}/../map/
        ${CMAKE_SOURCE_DIR}/../path_planning/
        )


add_executable(full full.cpp camera.cpp transformations.cpp)
target_link_libraries(full
        ${CMAKE_CURRENT_SOURCE_DIR}/../../ORB_SLAM2_PRJ/lib/libORB_SLAM2_PRJ.so
        ${Pangolin_LIBRARIES}
        ${OpenCV_LIBS}
        flycapture
        communications
        GridMap
        PathPlanning
        ${EIGEN3_LIBS}
        )

