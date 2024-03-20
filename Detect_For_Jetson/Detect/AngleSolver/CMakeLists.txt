
cmake_minimum_required(VERSION 3.6)

project(AngleSolver)

set(CMAKE_BUILD_TYPE "Release")

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_library(Angle OBJECT AngleSolver.cpp)

target_link_libraries(Angle ${OpenCV_LIBS})

