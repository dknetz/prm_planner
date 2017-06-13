cmake_minimum_required(VERSION 2.8.3)
project(prm_kinematics_pr2)

set(CMAKE_BUILD_TYPE Release) #RelWithDebInfo

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  prm_planner
  roscpp
)

find_package(pr2_arm_kinematics QUIET)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)
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
    LIBRARIES prm_kinematics_pr2
    CATKIN_DEPENDS prm_planner roscpp
    DEPENDS Boost orocos_kdl
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
  ${Boost_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${pr2_arm_kinematics_INCLUDE_DIRS}
  include
)


if (${pr2_arm_kinematics_FOUND})
## Declare a C++ library
add_library(prm_kinematics_pr2
   src/pr2_kinematics.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(prm_kinematics_kdl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(prm_kinematics_kdl_node src/kuka_robot_executers_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(prm_kinematics_kdl_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(prm_kinematics_pr2
   ${catkin_LIBRARIES} 
   ${Boost_LIBRARIES} 
   ${orocos_kdl_LIBRARIES} 
   ${pr2_arm_kinematics_LIBRARIES} 
)
endif()
