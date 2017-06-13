cmake_minimum_required(VERSION 2.8.3)
project(robot_fake_interface)

set(CMAKE_BUILD_TYPE RelWithDebInfo) #RelWithDebInfo

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  prm_planner
  roscpp
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES robot_fake_interface
    CATKIN_DEPENDS prm_planner roscpp
    DEPENDS Boost
)

###########
## Build ##
###########

ADD_DEFINITIONS(
    -std=c++11
    -D__cplusplus=201103L #for eclipse
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Declare a C++ library
add_library(robot_fake_interface
   src/robot_fake_interface.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(robot_fake_interface
   ${catkin_LIBRARIES}
)

