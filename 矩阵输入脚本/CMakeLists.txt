cmake_minimum_required(VERSION 3.6)
project(Mv_Camera)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_DIRS})

add_executable(Matrix_Input_Script  main.cpp Mv_Camera.cpp)

target_link_libraries(Matrix_Input_Script ${OpenCV_LIBS} MVSDK)
