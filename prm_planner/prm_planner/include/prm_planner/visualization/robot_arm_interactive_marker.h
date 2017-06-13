/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jan 15, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: interactive_marker.h
 */

#ifndef INTERACTIVE_MARKER_H_
#define INTERACTIVE_MARKER_H_

#include <ais_definitions/class.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <prm_planner/visualization/interactive_marker.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <unordered_map>

namespace prm_planner
{

FORWARD_DECLARE(Robot);
FORWARD_DECLARE(PRMPlanner);
FORWARD_DECLARE(Constraint);
FORWARD_DECLARE(RobotPoseVisualizer);

class RobotArmInteractiveMarker: public InteractiveMarker
{
public:
	/**
	 * Creates an interactive marker to define a goal in task space.
	 * The marker uses the inverse kinematics of KDL to find a possible
	 * joint state vector for the robot. These vectors are used to
	 * visualize the robot in rviz using a RobotModel. The RobotModel
	 * will be provided on the parameter server (name:
	 * {robot_description_name}_{name}, where name one of the arguments
	 * of the constructor.
	 *
	 * @robot: the robot model
	 * @planner: PRMPlanner instance
	 * @constraint: the current constraint
	 * @name: the name of the RobotModel. If the old robot description was
	 * 		named 'robot_description' and you use 'goal' as name, the parameter
	 * 		for the robot model will be available as 'robot_description_goal'.
	 * 		Furthermore the interactive marker will be published on the topic
	 * 		{private_namespace}/{name}.
	 */
	RobotArmInteractiveMarker(boost::shared_ptr<Robot> robot,
			boost::shared_ptr<PRMPlanner> planner,
			boost::shared_ptr<Constraint> constraint,
			const std::string& name);
	virtual ~RobotArmInteractiveMarker();

	void getJoints(std::unordered_map<std::string, double>& joints);

	virtual void update(const ros::TimerEvent& e);

private:
	void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
	void initMarkerServer();
	void initMenu();

	ros::NodeHandle m_nodeHandle;
	ros::Timer m_timer;
	tf::TransformBroadcaster m_transformBroadcaster;

	boost::shared_ptr<Robot> m_robot;
	geometry_msgs::Pose m_oldPose;
	KDL::ChainIkSolverVel_pinv m_ikVel;
	KDL::ChainFkSolverPos_recursive m_fk;
	KDL::ChainIkSolverPos_NR_JL* m_ik;
	KDL::JntArray m_joints;
	RobotPoseVisualizer* m_poseVisualizer;

	interactive_markers::InteractiveMarkerServer* m_server;
	interactive_markers::MenuHandler m_menuHandler;
	visualization_msgs::InteractiveMarker m_interactiveMarker;
	visualization_msgs::InteractiveMarkerFeedback::_menu_entry_id_type m_menuEntryPlanAndExecute;
	visualization_msgs::InteractiveMarkerFeedback::_menu_entry_id_type m_menuEntryReset;
	visualization_msgs::InteractiveMarkerFeedback::_menu_entry_id_type m_menuEntryRandomValid;
	visualization_msgs::InteractiveMarkerFeedback::_menu_entry_id_type m_menuEntrySavePose;
	visualization_msgs::InteractiveMarkerFeedback::_menu_entry_id_type m_menuEntryPrintPose;

	boost::shared_ptr<Constraint> m_constraint;
	bool m_firstValid;
	const std::string c_name;
};

} /* namespace prm_planner */

#endif /* INTERACTIVE_MARKER_H_ */
