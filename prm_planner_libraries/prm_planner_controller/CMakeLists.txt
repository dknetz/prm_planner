cmake_minimum_required(VERSION 2.8.3)
project(prm_planner_controller)

set(CMAKE_BUILD_TYPE RelWithDebInfo) #RelWithDebInfo

set(BUILD_EXAMPLES_AND_TEST true)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  prm_planner_robot
  prm_planner_constraints
  ais_util
  ais_ros
  ais_definitions
  ais_point_cloud
  roscpp
  fcl_wrapper
)

find_package(lbr_vrep_interface QUIET)

find_package(Boost REQUIRED COMPONENTS chrono system thread)
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
  LIBRARIES controller_basics velocity_controller torque_controller
  CATKIN_DEPENDS prm_planner_robot
      prm_planner_constraints
      roscpp
      ais_util
      ais_ros
      ais_definitions
      ais_point_cloud
      fcl_wrapper
  DEPENDS Boost orocos_kdl
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
add_library(controller_basics
    src/jacobian_multi_solver.cpp
    src/controller.cpp
)

add_library(velocity_controller
    src/velocity_controller.cpp
)

add_library(torque_controller
    src/torque_controller.cpp
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
target_link_libraries(controller_basics
    ${LIBRARIES}
)

target_link_libraries(velocity_controller
    controller_basics
)

target_link_libraries(torque_controller
    controller_basics
)

#examples/test
if (BUILD_EXAMPLES_AND_TEST)
    add_executable(example_velocity_controller
        test/example_velocity_controller.cpp
    )
    target_link_libraries(example_velocity_controller
        velocity_controller
    )
    
#    find_package(kuka_robot_interfaces REQUIRED)
    find_package(vrep_interface REQUIRED)
    include_directories(${kuka_robot_interfaces_INCLUDE_DIRS} ${vrep_interface_INCLUDE_DIRS})
    
    #velocity test controller
    add_executable(test_velocity_controller
        test/test_velocity_controller.cpp
    )
    
    target_link_libraries(test_velocity_controller
        controller_basics
 #       ${kuka_robot_interfaces_LIBRARIES}
        ${vrep_interface_LIBRARIES}
    )
    
    #torque test controller
    add_executable(test_torque_controller
        test/test_torque_controller.cpp
    )
    
    target_link_libraries(test_torque_controller
        controller_basics
  #      ${kuka_robot_interfaces_LIBRARIES}
        ${vrep_interface_LIBRARIES}
    )
endif()

