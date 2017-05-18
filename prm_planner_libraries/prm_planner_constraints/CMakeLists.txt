cmake_minimum_required(VERSION 2.8.3)
project(prm_planner_constraints)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  ais_log
  ais_definitions
  eigen_conversions
  prm_planner_robot
)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)

find_package(orocos_kdl REQUIRED)

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
  LIBRARIES prm_planner_constraints
  CATKIN_DEPENDS roscpp ais_log ais_definitions eigen_conversions prm_planner_robot
  DEPENDS orocos_kdl
)

###########
## Build ##
###########

if(CMAKE_COMPILER_IS_GNUCXX AND CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "-O3")        ## Optimize
endif()

ADD_DEFINITIONS(
    -std=c++11
    -D__cplusplus=201103L #for eclipse
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  include
)

## Declare a C++ library
add_library(prm_planner_constraints
    src/constraint.cpp
    src/constraint_xyzrpy.cpp
    src/constraint_horizontal.cpp
    src/constraint_upright.cpp
    src/constraint_downwards.cpp

    src/constraint_vertical.cpp
    
    src/constraint_z_up.cpp
    src/constraint_y_up.cpp
    src/constraint_y_down.cpp

    src/constraint_factory.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(prm_planner_constraints
   ${catkin_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
)
