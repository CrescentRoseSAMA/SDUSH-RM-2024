cmake_minimum_required(VERSION 3.6)

project(TRT_Frame)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")

find_package(TensorRT REQUIRED)

find_package(OpenCV REQUIRED)

find_package(FMT REQUIRED)
include_directories(/usr/local/cuda-11.8/targets/x86_64-linux/include/device_launch_parameters.h)

include_directories(${CUDA_INCLUDE_DIRS})

include_directories(${TensorRT_INCLUDE_DIRS})

include_directories(${OpenCV_INCLUDE_DIRS})

add_library(TRTFrame OBJECT TRTFrame.cpp ${TensorRT_SOURCE})

target_link_libraries(TRTFrame ${TensorRT_LIBS} ${OpenCV_LIBS} ${CUDA_LIBRARIES} nvonnxparser cuda pthread) 

