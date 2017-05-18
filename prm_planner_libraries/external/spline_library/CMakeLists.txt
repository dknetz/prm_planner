cmake_minimum_required(VERSION 2.8.3)
project(spline_library) 

find_package(catkin REQUIRED)

catkin_package(
	INCLUDE_DIRS include
#  LIBRARIES spline_library
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a C++ library
# add_library(spline_library
#   src/${PROJECT_NAME}/spline_library.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(spline_library ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(spline_library_node src/spline_library_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(spline_library_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(spline_library_node
#   ${catkin_LIBRARIES}
# )
