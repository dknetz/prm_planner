cmake_minimum_required(VERSION 2.8.3)
project(fcl_wrapper)

set(CMAKE_BUILD_TYPE RelWithDebInfo) #RelWithDebInfo

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    ais_definitions
    ais_util
    ais_ros
    urdf)

# System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system chrono thread)
find_package(octomap REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(PkgConfig REQUIRED)

#ros < jade
find_package(fcl QUIET)

if (${fcl_FOUND})
    link_directories(${fcl_LIBRARY_DIRS})
    string(REPLACE "fcl;" "" fcl_LIBRARIES "${fcl_LIBRARIES}")
else() #else kinetic
    pkg_check_modules(fcl fcl)
endif()

pkg_check_modules(ASSIMP assimp)

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
    INCLUDE_DIRS include ${ASSIMP_INCLUDE_DIRS} ${fcl_INCLUDE_DIRS}
    LIBRARIES fcl_wrapper
    CATKIN_DEPENDS roscpp ais_definitions ais_util ais_ros urdf
    DEPENDS Boost octomap orocos_kdl ${ASSIMP_LIBRARIES} ${fcl_LIBRARIES}
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

set(SOURCE_FILES
    src/robot_model/robot_joint.cpp
    src/robot_model/robot_link.cpp
    src/robot_model/robot_model.cpp
    src/robot_model/robot_state.cpp
    src/robot_model/geometry.cpp
    src/robot_model/mesh.cpp
    src/robot_model/box.cpp
    src/robot_model/cylinder.cpp
    src/robot_model/sphere.cpp
    
    src/collision_detection/collection.cpp
    src/collision_detection/collision_matrix.cpp
    src/collision_detection/fcl_wrapper.cpp
    src/collision_detection/octomap.cpp
    src/collision_detection/physical_object.cpp
    src/collision_detection/box.cpp
    src/collision_detection/sphere.cpp
)

set(LIBRARIES
    ${catkin_LIBRARIES} 
    ${Boost_LIBRARIES} 
    ${OCTOMAP_LIBRARIES} 
    ${orocos_kdl_LIBRARIES}
    ${ASSIMP_LIBRARIES}
    ${fcl_LIBRARIES}
    -lrt
)

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
    include 
    ${catkin_INCLUDE_DIRS} 
    ${OCTOMAP_INCLUDE_DIRS} 
    ${Boost_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${ASSIMP_INCLUDE_DIRS}
    ${fcl_INCLUDE_DIRS}
)

add_library(fcl_wrapper ${SOURCE_FILES})
target_link_libraries(fcl_wrapper ${LIBRARIES})

add_executable(sample_allowed_collisions src/sample_allowed_collisions.cpp)
target_link_libraries(sample_allowed_collisions ${LIBRARIES} fcl_wrapper)
