cmake_minimum_required(VERSION 2.8.3)
project(ais_ros)

set(CMAKE_BUILD_TYPE RelWithDebInfo) #RelWithDebInfo

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  eigen_conversions
  roscpp
  sensor_msgs
  geometry_msgs
  std_srvs
  tf
  tf_conversions
  cmake_modules 
  ais_definitions
  ais_log
  ais_util
  ais_point_cloud
  pluginlib
)

SET(CMAKE_BUILD_TYPE Release)
 
find_package(pcl_ros QUIET)
if (${pcl_ros_FOUND})
    ##############################################################################
    # BOOST
    ##############################################################################
    set(Boost_USE_MULTITHREADED ON)
    FIND_PACKAGE( Boost COMPONENTS filesystem signals system thread REQUIRED )
    INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} ) 
    
    ##############################################################################
    # Eigen
    ##############################################################################
    find_package(Eigen3 REQUIRED)
    include_directories(${EIGEN_INCLUDE_DIRS})
    
    ##############################################################################
    # OpenMP
    ##############################################################################
    FIND_PACKAGE( OpenMP REQUIRED)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    
    ##############################################################################
    # PCL
    ##############################################################################
    FIND_PACKAGE(PCL 1.7 REQUIRED common)
    INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS} )
    
    ##############################################################################
    # OpenCV
    ##############################################################################
    #find_package(OpenCV 3 REQUIRED)
    #include_directories(BEFORE ${OpenCV_INCLUDE_DIRS})
    
    ##############################################################################
    # DEFINITIONS
    ##############################################################################
    ADD_DEFINITIONS(
        -std=c++11
    )
    
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
      LIBRARIES ais_ros 
      CATKIN_DEPENDS eigen_conversions
      			cv_bridge
      			roscpp
      			sensor_msgs
      			geometry_msgs
      			std_srvs
      			tf
      			tf_conversions
      			pcl_ros 
    			ais_definitions
    			ais_log 
    			ais_util
    			ais_point_cloud
    			pluginlib
      DEPENDS Eigen PCL 
    )
    
    ###########
    ## Build ##
    ###########
    
    set(SOURCE_FILES
        src/parameter_server.cpp
        src/ros_camera_info.cpp
        src/ros_time.cpp
        src/ros_base_interface.cpp
    )
    
    ## Specify additional locations of header files
    ## Your package locations should be listed before other locations
    include_directories(include ${catkin_INCLUDE_DIRS} ${pcl_ros_INCLUDE_DIRS})
    
    ## Declare a cpp library
    add_library(ais_ros
       ${SOURCE_FILES}
    )
    
    ## Declare a cpp executable
    # add_executable(ais_ros_node src/ais_ros_node.cpp)
    
    ## Add cmake target dependencies of the executable/library
    ## as an example, message headers may need to be generated before nodes
    # add_dependencies(ais_ros_node ais_ros_generate_messages_cpp)
    
    ## Specify libraries to link a library or executable target against
    target_link_libraries(ais_ros
       	${catkin_LIBRARIES}
       	${pcl_ros_LIBRARIES}
    	-lboost_filesystem -lboost_signals -lboost_system -lboost_thread
    	#${XERCESC_LIBRARY}
    	${PCL_LIBRARIES}
    	${OpenCV_LIBRARIES}
    )
else()
    message(WARNING "Cannot build ais_ros because of missing package pcl_ros")
    catkin_package()
endif()
