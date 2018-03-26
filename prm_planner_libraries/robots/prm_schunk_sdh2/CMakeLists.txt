cmake_minimum_required(VERSION 2.8.3)
project(prm_schunk_sdh2)

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

#get sdh driver includes and library, if available
find_package(sdh2_hand QUIET)
if (${sdh2_hand_FOUND})
    set(catkin_INCLUDE_DIRS "${catkin_INCLUDE_DIRS};${sdh2_hand_INCLUDE_DIRS}")
    set(catkin_LIBRARIES "${catkin_LIBRARIES};${sdh2_hand_LIBRARIES}")
    add_definitions(-DFOUND_SDH)
endif()

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
    LIBRARIES prm_schunk_sdh2
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
add_library(prm_schunk_sdh2
   src/schunk_sdh2_gripper_interface.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(prm_schunk_sdh2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(prm_schunk_sdh2_node src/kuka_robot_executers_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(prm_schunk_sdh2_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(prm_schunk_sdh2
   ${catkin_LIBRARIES}
)

