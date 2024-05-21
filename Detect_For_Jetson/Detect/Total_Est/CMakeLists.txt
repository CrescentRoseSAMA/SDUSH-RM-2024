cmake_minimum_required(VERSION 3.0)

project(Total_Estimate)

set(CMAKE_BUILD_TYPE "Release")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})


add_library(Total_Est OBJECT Total_Est.cpp)

target_link_libraries(Total_Est ${OpenCV_LIBS})
