cmake_minimum_required(VERSION 3.0)

project(Serial)
set(CMAKE_BUILD_TYPE "Release")

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_library(Serial OBJECT Serial.cpp)

target_link_libraries(Serial ${OpenCV_LIBS})

