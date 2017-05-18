cmake_minimum_required(VERSION 2.8.3)
project(ais_point_cloud)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

#to find Flann
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ais_definitions
  ais_log
  ais_util
  cv_bridge
)

##############################################################################
# PCL
##############################################################################
FIND_PACKAGE(PCL REQUIRED COMPONENTS common filters)
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS} )

##############################################################################
# Flann
##############################################################################
FIND_PACKAGE(Flann REQUIRED)
INCLUDE_DIRECTORIES( ${FLANN_INCLUDE_DIRS} )

##############################################################################
# OpenCV
##############################################################################
#find_package(OpenCV 3 REQUIRED)
#include_directories(BEFORE ${OpenCV_INCLUDE_DIRS})

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
    INCLUDE_DIRS include ${FLANN_INCLUDE_DIRS}
    LIBRARIES ais_point_cloud
    CATKIN_DEPENDS ais_log ais_util cv_bridge
    DEPENDS PCL ${FLANN_LIBRARIES}
)

###########
## Build ##
###########
ADD_DEFINITIONS(
    -std=c++11
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(ais_point_cloud
   src/rgbd_image.cpp
   src/easy_kd_tree.cpp
   src/point_cloud_helpers.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(ais_point_cloud
   ${catkin_LIBRARIES} ${PCL_LIBRARIES}  ${FLANN_LIBRARIES} #${OpenCV_LIBRARIES}
)

