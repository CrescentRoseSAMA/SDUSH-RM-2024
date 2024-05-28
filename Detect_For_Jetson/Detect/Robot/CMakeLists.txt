cmake_minimum_required(VERSION 3.6)

project(Robot)

set(CMAKE_BUILD_TYPE "Release")

find_package(OpenCV REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(Robot OBJECT Robot.cpp)

target_link_libraries(Robot ${OpenCV_LIBS})

