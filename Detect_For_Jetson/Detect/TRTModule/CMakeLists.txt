cmake_minimum_required(VERSION 3.6)

project(trt)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_BUILD_TYPE "Release")

find_package(TensorRT REQUIRED)

find_package(OpenCV REQUIRED)

find_package(FMT REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS})

include_directories(${TensorRT_INCLUDE_DIRS})

include_directories(${OpenCV_INCLUDE_DIRS})

add_library(Module OBJECT TRTModule.cpp ${TensorRT_SOURCE})

target_link_libraries(Module ${TensorRT_LIBS} ${OpenCV_LIBS} ${CUDA_LIBRARIES} fmt::fmt nvonnxparser cuda pthread) 

