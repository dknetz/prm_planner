cmake_minimum_required(VERSION 2.8.3)
project(prm_planner_msgs)

find_package(catkin REQUIRED geometry_msgs std_msgs genmsg actionlib_msgs actionlib)

# Generate actions in the 'action' folder
add_action_files(
	FILES Goal.action
)

add_service_files(
    FILES SetState.srv 
        GetImage.srv
        ModifyPlanningScene.srv
        SetObjectPoseType.srv
)

add_message_files(
    FILES CollisionObject.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
	actionlib_msgs
)

catkin_package()

