cmake_minimum_required(VERSION 3.0)

project(Utils)

set(CMAKE_BUILD_TYPE "Debug")

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_library(Utils OBJECT Utils.cpp)

target_link_libraries(Utils ${OpenCV_LIBS})