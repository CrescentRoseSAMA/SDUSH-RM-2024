cmake_minimum_required(VERSION 3.6)

set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
project(Detect)

add_executable(Detect Detect.cpp)

include_directories(Serial)
add_subdirectory(Serial)
target_link_libraries(Detect Serial)

include_directories(AngleSolver)
add_subdirectory(AngleSolver)
target_link_libraries(Detect Angle)

include_directories(MvCamera)
add_subdirectory(MvCamera)
target_link_libraries(Detect MvCamera)

include_directories(TRTFrame)
add_subdirectory(TRTFrame)
target_link_libraries(Detect TRTFrame)

include_directories(Utils)
add_subdirectory(Utils)
target_link_libraries(Detect Utils)

include_directories(Kalman)
add_subdirectory(Kalman)
target_link_libraries(Detect Kalman)

include_directories(Robot)
add_subdirectory(Robot)
target_link_libraries(Detect Robot)