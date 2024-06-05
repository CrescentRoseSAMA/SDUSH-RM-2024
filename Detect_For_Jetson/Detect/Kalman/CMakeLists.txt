cmake_minimum_required(VERSION 3.6)

set(CMAKE_BUILD_TYPE "Release")

project(Kalman)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(Kalman OBJECT Kalman.cpp)

target_link_libraries(Kalman ${OpenCV_LIBS})