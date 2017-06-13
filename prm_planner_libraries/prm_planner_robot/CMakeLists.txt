cmake_minimum_required(VERSION 2.8.3)
project(prm_planner_robot)

set(CMAKE_BUILD_TYPE RelWithDebInfo) #RelWithDebInfo

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  roscpp
  spline_library
  urdf
  ais_definitions
  ais_log
  ais_util
  eigen_conversions
  actionlib
)

find_package(lbr_vrep_interface QUIET)

find_package(Boost REQUIRED COMPONENTS chrono system thread)
find_package(orocos_kdl REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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
    LIBRARIES prm_planner_robot
    CATKIN_DEPENDS kdl_parser roscpp spline_library urdf ais_definitions ais_log ais_util eigen_conversions
    DEPENDS Eigen Boost orocos_kdl
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
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(prm_planner_robot
    src/trajectory.cpp
    src/robot_arm.cpp
    src/path.cpp
    src/gripper_interface.cpp
    src/robot_interface.cpp
    src/kinematics.cpp
)

set(LIBRARIES
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES} 
   ${orocos_kdl_LIBRARIES} 
   -lrt
)

if (${lbr_vrep_interface_FOUND}) 
    include_directories(${lbr_vrep_interface_INCLUDE_DIRS})
    add_definitions(-DFOUND_VREP_INTERFACE)
    set(LIBRARIES ${LIBRARIES} ${lbr_vrep_interface_LIBRARIES})
endif()

# Specify libraries to link a library or executable target against
target_link_libraries(prm_planner_robot
   ${LIBRARIES}
)

