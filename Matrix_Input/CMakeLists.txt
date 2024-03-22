cmake_minimum_required(VERSION 3.6)
project(Mv_Camera)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_DIRS})
include_directories("../Detect_For_Jetson/Detect/MvCamera")

add_executable(Matrix_Input_Script  ../Detect_For_Jetson/Detect/MvCamera/MvCamera.cpp main.cpp)

target_link_libraries(Matrix_Input_Script ${OpenCV_LIBS} MVSDK)
