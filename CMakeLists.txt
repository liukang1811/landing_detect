cmake_minimum_required(VERSION 2.8.3)
project(pre_takeoff_warning)
set(DCMAKE_BUILD_TYPE =Debug )
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -pthread -fopenmp")#
set(CMAKE_PREFIX_PATH  "/opt/ros/melodic/")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
find_package(PCL REQUIRED)
include_directories(include ${PCL_INCLUDE_DIRS})
FIND_PACKAGE(OpenMP)
IF(OPENMP_FOUND)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS"${CMAKE_EXE_LINKER_FLAGS}${OpenMP_EXE_LINKER_FLAGS}")
  MESSAGE(STATUS "Found OpenMP")
ENDIF()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

INCLUDE_DIRECTORIES(
        ${catkin_INCLUDE_DIRS}
        include/
        ./pre_takeoff_warning_algo/include
)

link_directories(src/)

add_executable(
                   pre_takeoff_warning
                   src/main.cpp
                   src/motion_state.cpp
                  ./pre_takeoff_warning_algo/pointcloud_common.cpp
        pre_takeoff_warning_algo/LandingDetection.cpp pre_takeoff_warning_algo/include/LandingDetection.h)
 target_link_libraries(pre_takeoff_warning
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )
add_dependencies(pre_takeoff_warning lidar_detection_generate_messages_cpp)

