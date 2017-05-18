/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Aug 15, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: task_pose_interactive_marker.cpp
 */

#include <ais_definitions/class.h>
#include <ais_log/log.h>
#include <boost/shared_ptr.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <interactive_markers/interactive_marker_server.h>
#include <prm_planner_constraints/constraint.h>
#include <prm_planner/prm_planner.h>
#include <prm_planner/problem_definitions/problem_definition_manager.h>
#include <prm_planner/problem_definitions/problem_definition.h>
#include <prm_planner/visualization/task_pose_interactive_marker.h>
#include <Eigen/Geometry>

namespace prm_planner
{

TaskPoseInteractiveMarker::TaskPoseInteractiveMarker(boost::shared_ptr<Robot> robot,
		boost::shared_ptr<PRMPlanner> planner,
		boost::shared_ptr<Constraint> constraint) :
				InteractiveMarker(planner),
				m_robot(robot),
				m_nodeHandle("/prm_planner"),
				m_server(new interactive_markers::InteractiveMarkerServer(m_nodeHandle.getNamespace() + "/goal_pose")),
				m_constraint(constraint)
{
	initMarkerServer();
}

TaskPoseInteractiveMarker::~TaskPoseInteractiveMarker()
{
	DELETE_VAR(m_server);
}

void TaskPoseInteractiveMarker::update()
{
}

void TaskPoseInteractiveMarker::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
	boost::shared_ptr<ProblemDefinition> pd = ProblemDefinitionManager::getInstance()->getProblemDefinition();

	switch (feedback->event_type)
	{
		case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
			{
			if (feedback->menu_entry_id == m_menuEntryPlanAndExecute)
			{
				Eigen::Affine3d pose;
				tf::poseMsgToEigen(feedback->pose, pose);

				//why do we get the answer in another frame than the marker?
				Eigen::Affine3d t = m_planner->getTransformation(feedback->header.frame_id, m_interactiveMarker.header.frame_id);
				pose = t * pose;

				m_planner->planAndExecute(pose, pd->getFrame());
			}
			else if (feedback->menu_entry_id == m_menuEntryReset)
			{
				Eigen::Affine3d pose = pd->getCurrentTaskPose();
				geometry_msgs::Pose tfPose;
				tf::poseEigenToMsg(pose, tfPose);
				m_server->setPose(m_interactiveMarker.name, tfPose);
				m_menuHandler.reApply(*m_server);
				m_server->applyChanges();
			}
			else if (feedback->menu_entry_id == m_menuEntryRandomValid)
			{
				Eigen::Affine3d pose = pd->samplePose();
				geometry_msgs::Pose tfPose;
				tf::poseEigenToMsg(pose, tfPose);
				m_server->setPose(m_interactiveMarker.name, tfPose);
				m_menuHandler.reApply(*m_server);
				m_server->applyChanges();
			}
			break;
		}
		case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
			//				{
//				KDL::Frame x;
//				Eigen::Affine3d pose;
//				tf::poseMsgToEigen(feedback->pose, pose);
//
//				//why do we get the answer in another frame than the marker?
//				Eigen::Affine3d t = m_planner->getTransformation(feedback->header.frame_id, m_interactiveMarker.header.frame_id);
//				pose = t * pose;
//
//				if (!m_firstValid && !m_constraint->isValidPose(pose))
//				{
//					m_constraint->findValidPose(pose);
//				}
//
//				tf::transformEigenToKDL(pose, x);
//
//				KDL::JntArray solution;
//				bool valid = m_constraint->isValidPose(pose);
//				if (valid)
//				{
//					if (m_ik->CartToJnt(m_joints, x, solution) != 0)
//					{
//						m_server->setPose(m_interactiveMarker.name, m_oldPose);
//						m_menuHandler.reApply(*m_server);
//						m_server->applyChanges();
//					}
//					else
//					{
//	//					LOG_INFO(std::setprecision(12) << solution.data.transpose());
//
//						geometry_msgs::Pose p;
//						tf::poseEigenToMsg(pose, p);
//
//						if (!m_firstValid)
//						{
//							m_server->setPose(m_interactiveMarker.name, p);
//							m_menuHandler.reApply(*m_server);
//							m_server->applyChanges();
//						}
//						m_firstValid = true;
//						m_oldPose = p;
//						m_joints = solution;
//					}
//				}
//				else if (!valid && m_firstValid)
//				{
//					m_server->setPose(m_interactiveMarker.name, m_oldPose);
//					m_menuHandler.reApply(*m_server);
//					m_server->applyChanges();
//				}
//				break;
//			}
		default:
			return;
	}
}

void TaskPoseInteractiveMarker::initMarkerServer()
{
	boost::shared_ptr<ProblemDefinition> pd = ProblemDefinitionManager::getInstance()->getProblemDefinition();
	std::string frame = pd->getFrame();

	// create an interactive marker for our server
	m_interactiveMarker.header.frame_id = frame;
	m_interactiveMarker.header.stamp = ros::Time::now();
	m_interactiveMarker.name = "goal_pose_marker";
	m_interactiveMarker.scale = 0.3;
	m_interactiveMarker.description = "6-DOF Control";
	tf::poseEigenToMsg(pd->getCurrentTaskPose(), m_interactiveMarker.pose);

	// create a red sphere marker
	visualization_msgs::Marker sphereMarker;
	sphereMarker.type = visualization_msgs::Marker::SPHERE;
	sphereMarker.scale.x = 0.05;
	sphereMarker.scale.y = 0.05;
	sphereMarker.scale.z = 0.05;
	sphereMarker.color = ais_util::Color::red().toROSMsg();

	visualization_msgs::InteractiveMarkerControl sphereControl;
	sphereControl.always_visible = true;
	sphereControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
	sphereControl.markers.push_back(sphereMarker);

	m_interactiveMarker.controls.push_back(sphereControl);

	visualization_msgs::InteractiveMarkerControl control;
	control.orientation.w = 1;
	control.orientation.x = 1;
	control.orientation.y = 0;
	control.orientation.z = 0;
	control.name = "rotate_x";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	m_interactiveMarker.controls.push_back(control);
	control.name = "move_x";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	m_interactiveMarker.controls.push_back(control);

	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 1;
	control.orientation.z = 0;
	control.name = "rotate_z";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	m_interactiveMarker.controls.push_back(control);
	control.name = "move_z";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	m_interactiveMarker.controls.push_back(control);

	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 0;
	control.orientation.z = 1;
	control.name = "rotate_y";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	m_interactiveMarker.controls.push_back(control);
	control.name = "move_y";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	m_interactiveMarker.controls.push_back(control);

	// add the interactive marker to our collection &
	// tell the server to call processFeedback() when feedback arrives for it
	m_server->insert(m_interactiveMarker, boost::bind(&TaskPoseInteractiveMarker::processFeedback, this, _1));

	initMenu();

	// 'commit' changes and send to all clients
	m_server->applyChanges();
}

void TaskPoseInteractiveMarker::initMenu()
{
	m_menuEntryPlanAndExecute = m_menuHandler.insert("Plan and execute", boost::bind(&TaskPoseInteractiveMarker::processFeedback, this, _1));
	m_menuEntryReset = m_menuHandler.insert("Set to current robot pose", boost::bind(&TaskPoseInteractiveMarker::processFeedback, this, _1));
	m_menuEntryRandomValid = m_menuHandler.insert("Set to random valid", boost::bind(&TaskPoseInteractiveMarker::processFeedback, this, _1));
	m_menuHandler.apply(*m_server, m_interactiveMarker.name);
}

} /* namespace prm_planner */
