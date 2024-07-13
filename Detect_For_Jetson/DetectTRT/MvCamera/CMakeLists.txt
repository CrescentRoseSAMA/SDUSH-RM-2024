cmake_minimum_required(VERSION 3.6)
project(MvCamera)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_DIRS})

add_library(MvCamera OBJECT MvCamera.cpp)

target_link_libraries(MvCamera ${OpenCV_LIBS} MVSDK)
