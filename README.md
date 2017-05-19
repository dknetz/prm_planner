# Probabilistic Roadmap Planner
The probabilistic roadmap (PRM) planner uses a graph of task poses to find a path between the start and goal poses. It can be used to plan a manipulator trajectory between a start joint pose and a goal task pose using constraints. Addionally, the planner is able to perform simple grasping and dropping actions directly without an external library. Finally, the planner is implemented as a ROS package. It is capable to execute the trajectory, if a FollowJointTrajectory interface is available or you provide a corresponding interface to your robot. 

# Installation
Assuming you already created a ROS workspace you need to checkout this repository and build it. It was tested with ROS Jade and Kinetic under Ubuntu 14.04 and 16.04, respectively.

Instructions:
- Checkout into the src folder
- Build the workspace with catkin_make
