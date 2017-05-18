/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Aug 15, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: task_pose_interactive_marker.h
 */

#ifndef H9E60FC1A_78B2_45C4_B86D_8A489337FC7F
#define H9E60FC1A_78B2_45C4_B86D_8A489337FC7F

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <prm_planner/visualization/interactive_marker.h>
#include <ros/ros.h>

namespace prm_planner
{

class Robot;
class PRMPlanner;
class Constraint;

class TaskPoseInteractiveMarker: public InteractiveMarker
{
public:
	TaskPoseInteractiveMarker(boost::shared_ptr<Robot> robot,
			boost::shared_ptr<PRMPlanner> planner,
			boost::shared_ptr<Constraint> constraint);
	virtual ~TaskPoseInteractiveMarker();

	virtual void update();

private:
	void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
	void initMarkerServer();
	void initMenu();

	ros::NodeHandle m_nodeHandle;
	interactive_markers::InteractiveMarkerServer* m_server;
	interactive_markers::MenuHandler m_menuHandler;
	boost::shared_ptr<Robot> m_robot;
	geometry_msgs::Pose m_oldPose;
	visualization_msgs::InteractiveMarker m_interactiveMarker;
	visualization_msgs::InteractiveMarkerFeedback::_menu_entry_id_type m_menuEntryPlanAndExecute;
	visualization_msgs::InteractiveMarkerFeedback::_menu_entry_id_type m_menuEntryReset;
	visualization_msgs::InteractiveMarkerFeedback::_menu_entry_id_type m_menuEntryRandomValid;
	boost::shared_ptr<Constraint> m_constraint;
};

} /* namespace prm_planner */

#endif /* H9E60FC1A_78B2_45C4_B86D_8A489337FC7F */
