/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Sep 15, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: gripper_sdh.cpp
 */

#include <schunk_wsg/schunk_wsg_gripper_interface.h>

#include <ais_log/log.h>
#include <sdh2_hand/SDHAction.h>
#include <pluginlib/class_list_macros.h>

namespace schunk_wsg
{

SchunkWSGGripperInterface::SchunkWSGGripperInterface() :
				prm_planner::GripperInterface(),
				m_actionClient(NULL)
{
}

SchunkWSGGripperInterface::~SchunkWSGGripperInterface()
{
	DELETE_VAR(m_actionClient);
}

bool SchunkWSGGripperInterface::open()
{
	wsg_gripper::GripperCommandGoal goal;
	goal.command_code = wsg_gripper::GripperCommandGoal::MOVE_WITHOUT_FORCE_CONTROL;
	goal.speed = 50.0;
	goal.width = 110.0;

	m_actionClient->sendGoalAndWait(goal);
	return true;
}

void SchunkWSGGripperInterface::init(GripperInterfaceParameters& parameters)
{
	GripperInterface::init(parameters);
	LOG_INFO("Waiting for WSG");
	m_actionClient = new actionlib::SimpleActionClient<wsg_gripper::GripperCommandAction>(parameters.topic);
	LOG_INFO("Connected");
	m_actionClient->waitForServer();
}

bool SchunkWSGGripperInterface::close()
{
	wsg_gripper::GripperCommandGoal goal;
	goal.command_code = wsg_gripper::GripperCommandGoal::GRASP_A_PART;
	goal.speed = 30.0;
	goal.width = 50.0;

	m_actionClient->sendGoalAndWait(goal);
	return true;
}

} /* namespace schunk_wsg */

PLUGINLIB_EXPORT_CLASS(schunk_wsg::SchunkWSGGripperInterface, prm_planner::GripperInterface)
