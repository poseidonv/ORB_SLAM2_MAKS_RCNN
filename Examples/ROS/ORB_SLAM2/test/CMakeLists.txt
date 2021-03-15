cmake_minimum_required(VERSION 2.8)
project(useEigen)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3")

# 添加Eigen头文件
#include_directories("/usr/include/eigen3")
find_package(Eigen3 REQUIRED)
set(LIBS ${EIGEN3_INCLUDE_DIRS})
message("lib: ---${LIBS}---")
include_directories(${EIGEN3_INCLUDE_DIRS})
add_executable(eigenMatrix eigenMatrix.cpp)
