
cmake_minimum_required(VERSION 3.6)

project(AngleSolver)

#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(Angle OBJECT AngleSolver.cpp)

target_link_libraries(Angle ${OpenCV_LIBS})

