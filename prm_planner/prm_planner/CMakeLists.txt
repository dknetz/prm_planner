cmake_minimum_required(VERSION 2.8.3)
project(prm_planner)

set(CMAKE_BUILD_TYPE RelWithDebInfo) #RelWithDebInfo

include(cmake/config.cmake)
include(cmake/macros.cmake)

find_package(catkin REQUIRED 
    roscpp
    geometric_shapes
    ais_definitions
    ais_util
    ais_ros
    ais_point_cloud
    octomap_msgs
    geometry_msgs
    nav_msgs
    eigen_conversions
    visualization_msgs
    interactive_markers
    prm_planner_robot
    prm_planner_constraints
    prm_planner_controller
    prm_planner_msgs    
    fcl_wrapper
    actionlib
    dynamic_reconfigure
)

find_package(vrep_interface QUIET)
find_package(Boost REQUIRED COMPONENTS chrono system thread)
find_package(octomap REQUIRED)
find_package(orocos_kdl REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

INCLUDE(cmake/FindSSE.cmake)
FindSSE ()
IF(SSE3_FOUND)
    IF(SSSE3_FOUND)
            SET(CXX_DFLAGS -msse3 -mssse3)
    ENDIF(SSSE3_FOUND)
ENDIF(SSE3_FOUND)

IF(SSE4_2_FOUND)
        SET(CXX_DFLAGS -msse4.2 -mpopcnt)
ENDIF(SSE4_2_FOUND)
ADD_DEFINITIONS(${CXX_DFLAGS})

generate_dynamic_reconfigure_options(
        cfg/PRMPlanner.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES prm_planner_lib #prm_planner_cached_database_lib prm_planner_prm_astar_lib prm_planner_rrt_lib
    CATKIN_DEPENDS  
        roscpp
        geometric_shapes
        ais_definitions
        ais_util
        ais_ros
        ais_point_cloud
        nav_msgs
        eigen_conversions
        interactive_markers
        prm_planner_robot
        prm_planner_constraints
        prm_planner_controller
        fcl_wrapper
        prm_planner_msgs
        actionlib
    DEPENDS Boost octomap orocos_kdl OpenMP
)

if(CMAKE_COMPILER_IS_GNUCXX AND CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "-O3")        ## Optimize
endif()

###########
## Build ##
###########

ADD_DEFINITIONS(
    -std=c++11
    -D__cplusplus=201103L #for eclipse
)

set(SOURCE_FILES
    src/prm_planner.cpp
    
    src/collision_detection/collision_detector.cpp
    
    src/controllers/simulation_robot_controller.cpp
    src/controllers/simulation_velocity_controller_n.cpp
    
    src/execution/executer.cpp
    src/execution/follow_joint_trajectory_executer.cpp
    src/execution/robot_executer.cpp
    
    src/objects/graspable_object.cpp
    src/objects/object_manager.cpp
    src/objects/dropping_region.cpp
    
    src/planners/path_planner.cpp
    
    src/problem_definitions/problem_definition.cpp
    src/problem_definitions/problem_definition_manager.cpp
    src/problem_definitions/approaching_problem_definition.cpp
    
    src/robot/trajectory_validator.cpp
    src/robot/robot.cpp
    src/robot/robot_state_publisher.cpp
    src/robot/feasibility_checker.cpp
    
    src/util/parameter_server.cpp
    
    src/visualization/interactive_marker.cpp
    src/visualization/object_interactive_marker.cpp
    src/visualization/robot_arm_interactive_marker.cpp
    src/visualization/task_pose_interactive_marker.cpp
    src/visualization/robot_trajectory_visualizer.cpp
    
    src/planners/rrt/rrt.cpp
    src/planners/rrt/rrt_node.cpp
    
    src/planners/prm/prma_star.cpp
    src/planners/prm/prm.cpp
    src/planners/prm/prm_view.cpp
    src/planners/prm/prm_node.cpp
    src/planners/prm/prm_edge.cpp
    src/planners/prm/heuristic.cpp
    
    src/planners/database_cache/database_cached_planner.cpp
    src/planners/database_cache/path_database.cpp
    
    src/environment/planning_scene.cpp
    src/environment/planning_object.cpp
)

set(LIBRARIES
    ${catkin_LIBRARIES} 
    ${Boost_LIBRARIES} 
    ${OCTOMAP_LIBRARIES} 
    ${orocos_kdl_LIBRARIES} 
    -lrt) #for boost chrono
 
if (${vrep_interface_FOUND}) 
    #include_directories(${vrep_interface_INCLUDE_DIRS})
    add_definitions(-DFOUND_VREP_INTERFACE)
    set(LIBRARIES ${LIBRARIES} ${vrep_interface_LIBRARIES})
endif()
    
include_directories(
    include 
    ${catkin_INCLUDE_DIRS} 
    ${OCTOMAP_INCLUDE_DIRS} 
    ${Boost_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${vrep_interface_INCLUDE_DIRS}
)
    
if(BUILD_CONTROLLER_DEBUG)
    add_definitions(-DBUILD_CONTROLLER_DEBUG)
endif()

#Build libs
#          name                    path
#add_module(prm_astar               planners/prm)
#add_module(rrt                     planners/rrt)
#add_module(cached_database         planners/database_cache)

add_library(prm_planner_lib ${SOURCE_FILES})
target_link_libraries(prm_planner_lib ${LIBRARIES})

#MAIN
set(LIBRARIES_FOR_BIN
        prm_planner_lib
#        prm_planner_cached_database_lib
#        prm_planner_prm_astar_lib
#        prm_planner_rrt_lib
)

#Build main
add_executable(prm_planner src/main.cpp)
target_link_libraries(prm_planner ${LIBRARIES_FOR_BIN})
add_dependencies(prm_planner prm_planner_gencfg)

add_executable(prm_planner_execute_path src/main_execute_path.cpp)
target_link_libraries(prm_planner_execute_path ${LIBRARIES_FOR_BIN})
add_dependencies(prm_planner_execute_path prm_planner_gencfg)

add_executable(sample_cached_database src/planners/database_cache/main_sample_database.cpp)
target_link_libraries(sample_cached_database ${LIBRARIES_FOR_BIN})

add_executable(visualize_cached_database src/planners/database_cache/main_visualize_database.cpp)
target_link_libraries(visualize_cached_database ${LIBRARIES_FOR_BIN})

#Build new prm binary 
add_executable(new_prm src/main_new_prm.cpp)
target_link_libraries(new_prm ${LIBRARIES_FOR_BIN})
