# Examples
This folder contains multiple examples to show, how the planner can be used:

### prm_planner_simple_iiwa
A simple example to show the basic functionalities of the planner. It opens rviz and displays an interactive marker, which can be used to specify the goal. You start planning and execution by right-clicking the marker. Additionally two boxes are available to simulate obstacles.
```roslaunch prm_planner_simple_iiwa example.launch```

### prm_planner_simple_iiwa_api
The example shows how to call the planner using a ROS action client. 
```roslaunch prm_planner_simple_iiwa_api example_ros_action_server.launch``` 

The second example calls the planner directly.
```roslaunch prm_planner_simple_iiwa_api example_api.launch``` 

### prm_planner_simple_pr2
A simple example using the PR2 robot to show the basic functionalities of the planner. It opens rviz and displays an interactive marker, which can be used to specify the goal of the right arm. You start planning and execution by right-clicking the marker. Additionally two boxes are available to simulate obstacles. Additionally, this example uses an alternative kinematics plugin.
```roslaunch prm_planner_simple_pr2 example.launch```

### prm_planner_example_configs
This package contains some common config files to specify robot and planning related data. The config files are used in all example packages