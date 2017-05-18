/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jun 9, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: roadmap_planner.cpp
 */

#ifdef BUILD_NEUROBOTS_INTERFACE

#include <ais_log/log.h>
#include <prm_planner_interface/prm_planner_interface.h>
#include <prm_planner_msgs/SetState.h>

#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>

namespace prm_planner
{

PRMPlannerInterface::PRMPlannerInterface(const std::string& plannerNamespace) :
				c_namespace(plannerNamespace),
				m_actionClient(NULL)
{
	ros::NodeHandle n(plannerNamespace);

	m_actionClient = new actionlib::SimpleActionClient<prm_planner_msgs::GoalAction>(plannerNamespace + "/goals", true);
	if (!m_actionClient->waitForServer(ros::Duration(60)))
	{
		LOG_ERROR("Something is wrong: Is the PRM Planner in the namespace " << plannerNamespace << " running?");
	}
	else {
		LOG_INFO("Connected to PRM Planner " << plannerNamespace);
	}

	m_setState = n.serviceClient<prm_planner_msgs::SetState>("set_state");
}

PRMPlannerInterface::~PRMPlannerInterface()
{
	DELETE_VAR(m_actionClient);
}

bool PRMPlannerInterface::plan(const Eigen::Affine3d& goal)
{
	prm_planner_msgs::GoalGoal goalMsg;
	tf::poseEigenToMsg(goal, goalMsg.goal);
	goalMsg.action = prm_planner_msgs::GoalGoal::ACTION_MOVE;
	m_actionClient->sendGoal(goalMsg);
	m_actionClient->waitForResult();
	return m_actionClient->getResult()->success;
}

bool PRMPlannerInterface::execute()
{
	return true;
}

bool PRMPlannerInterface::plan(const std::vector<std::string>& params)
{
	//for now we use the objects from the planner
	prm_planner_msgs::GoalGoal goalMsg;
	goalMsg.action = prm_planner_msgs::GoalGoal::ACTION_CUSTOM;
	goalMsg.str = params;
	m_actionClient->sendGoal(goalMsg);
	m_actionClient->waitForResult();
	return m_actionClient->getResult()->success;
}

bool PRMPlannerInterface::planRel(const Eigen::Affine3d& goal)
{
	//for now we use the objects from the planner
	prm_planner_msgs::GoalGoal goalMsg;
	goalMsg.action = prm_planner_msgs::GoalGoal::ACTION_MOVE_REL;
	goalMsg.robot = ""; //we only have one robot, so leave the robot variable empty
	tf::poseEigenToMsg(goal, goalMsg.goal);
	m_actionClient->sendGoal(goalMsg);
	m_actionClient->waitForResult();
	return m_actionClient->getResult()->success;
}

bool PRMPlannerInterface::grasp(const std::string& object)
{
	prm_planner_msgs::GoalGoal goalMsg;
	goalMsg.object_name = object;
	goalMsg.action = prm_planner_msgs::GoalGoal::ACTION_GRASP;
	m_actionClient->sendGoal(goalMsg);
	m_actionClient->waitForResult();
	return m_actionClient->getResult()->success;
}

bool PRMPlannerInterface::drop(const std::string& object,
		const Eigen::Affine3d& pose)
{
	prm_planner_msgs::GoalGoal goalMsg;
	goalMsg.object_name = object;
	tf::poseEigenToMsg(pose, goalMsg.goal);
	goalMsg.action = prm_planner_msgs::GoalGoal::ACTION_DROP;
	m_actionClient->sendGoal(goalMsg);
	m_actionClient->waitForResult();
	return m_actionClient->getResult()->success;
}

bool PRMPlannerInterface::activate()
{
	prm_planner_msgs::SetState srv;
	srv.request.active = true;
	return m_setState.call(srv);
}

bool PRMPlannerInterface::deactivate()
{
	prm_planner_msgs::SetState srv;
	srv.request.active = false;
	return m_setState.call(srv);
}

#endif

} /* namespace prm_planner */

