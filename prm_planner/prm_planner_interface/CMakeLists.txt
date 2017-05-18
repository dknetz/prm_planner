cmake_minimum_required(VERSION 2.8.3)
project(prm_planner_interface)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  prm_planner
  roscpp
)

#build neurobots interface
find_package(robot_interface_definition QUIET)

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
    LIBRARIES prm_planner_interface
    CATKIN_DEPENDS prm_planner roscpp
    #DEPENDS system_lib
)

###########
## Build ##
###########


if(${robot_interface_definition_FOUND})
    add_definitions(
        -std=c++11
        -D__cplusplus=201103L #for eclipse
    )
    
    add_definitions(-DBUILD_NEUROBOTS_INTERFACE)
    
## Specify additional locations of header files
    ## Your package locations should be listed before other locations
    # include_directories(include)
    include_directories(
      include
      ${catkin_INCLUDE_DIRS}
      ${robot_interface_definition_INCLUDE_DIRS}
    )

    #neurobots planner interface
    add_library(prm_planner_interface src/prm_planner_interface.cpp)
    
    # Specify libraries to link a library or executable target against
    target_link_libraries(prm_planner_interface
       ${catkin_LIBRARIES}
       ${robot_interface_definition_LIBRARIES}
    )
endif()
