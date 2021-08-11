cmake_minimum_required(VERSION 3.19)
project(daa_lidar_algo)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -pthread -fopenmp")#
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
message(${EIGEN3_INCLUDE_DIRS})
include_directories(./include)
add_library( daa_lidar_algo_lib SHARED main.cpp pointcloud_common.cpp)
#add_executable(daa_lidar_algo main.cpp pointcloud_common.cpp pointcloud_common.h)
