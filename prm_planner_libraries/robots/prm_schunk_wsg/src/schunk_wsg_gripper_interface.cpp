/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Sep 15, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: gripper_sdh.cpp
 */

#include <prm_schunk_wsg/schunk_wsg_gripper_interface.h>
#include <ais_log/log.h>
#include <pluginlib/class_list_macros.h>

namespace prm_schunk_wsg
{

SchunkWSGGripperInterface::SchunkWSGGripperInterface() :
				prm_planner::GripperInterface()
{
#ifdef FOUND_WSG
	m_actionClient = NULL;
#endif
}

SchunkWSGGripperInterface::~SchunkWSGGripperInterface()
{
#ifdef FOUND_WSG
	DELETE_VAR(m_actionClient);
#endif
}

bool SchunkWSGGripperInterface::open()
{
#ifdef FOUND_WSG
	wsg_gripper::GripperCommandGoal goal;
	goal.command_code = wsg_gripper::GripperCommandGoal::MOVE_WITHOUT_FORCE_CONTROL;
	goal.speed = 50.0;
	goal.width = 110.0;

	m_actionClient->sendGoalAndWait(goal);
	return true;
#else
	LOG_ERROR("You need to compile the WSG Gripper Interface with available wsg_gripper package");
	return false;
#endif
}

void SchunkWSGGripperInterface::init(GripperInterfaceParameters& parameters)
{
#ifdef FOUND_WSG
	GripperInterface::init(parameters);
	LOG_INFO("Waiting for WSG");
	m_actionClient = new actionlib::SimpleActionClient<wsg_gripper::GripperCommandAction>(parameters.topic);
	LOG_INFO("Connected");
	m_actionClient->waitForServer();
#else
	LOG_ERROR("You need to compile the WSG Gripper Interface with available wsg_gripper package");
#endif
}

bool SchunkWSGGripperInterface::close()
{
#ifdef FOUND_WSG
	wsg_gripper::GripperCommandGoal goal;
	goal.command_code = wsg_gripper::GripperCommandGoal::GRASP_A_PART;
	goal.speed = 30.0;
	goal.width = 50.0;

	m_actionClient->sendGoalAndWait(goal);
	return true;
#else
	LOG_ERROR("You need to compile the WSG Gripper Interface with available wsg_gripper package");
	return false;
#endif
}

} /* namespace prm_schunk_wsg */

PLUGINLIB_EXPORT_CLASS(prm_schunk_wsg::SchunkWSGGripperInterface, prm_planner::GripperInterface)
