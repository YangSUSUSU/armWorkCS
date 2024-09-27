cmake_minimum_required(VERSION 3.0.2)
project(trajectory_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED 
  roscpp
)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES trajectory_planner
 CATKIN_DEPENDS eigen_conversions roscpp  
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(trajectory_planner
  src/traj_planner_base.cpp
  src/traj_cp_planner.cpp
  src/traj_s_planner.cpp
  src/sVelocityPlanner.cpp
  src/interpolation.cpp
)

