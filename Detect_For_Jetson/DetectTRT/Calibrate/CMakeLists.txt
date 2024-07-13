cmake_minimum_required(VERSION 3.6)

project(Catch&Calibrate)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(Photoshot Catch&Calibrate.cpp Camera.cpp Camera_Calibrate.cpp)

target_link_libraries(Photoshot ${OpenCV_LIBS} MVSDK)
