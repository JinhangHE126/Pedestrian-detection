cmake_minimum_required(VERSION 2.14)
project(faceLightClient)
# 添加ROS依赖项
find_package(roslib REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp)
include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -O3 -march=native -Wall")

include_directories(include)
link_directories(lib)

add_executable(faceLightClient main.cpp)
target_link_libraries(faceLightClient libfaceLight_SDK_arm64.so ${catkin_LIBRARIES} stdc++ c m)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
