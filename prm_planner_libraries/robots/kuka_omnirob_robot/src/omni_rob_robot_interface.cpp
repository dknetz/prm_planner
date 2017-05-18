/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Oct 10, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: omni_rob_robot_interface.cpp
 */

#include <kuka_omnirob_robot/omni_rob_robot_interface.h>

#include <ais_log/log.h>
#include <pluginlib/class_list_macros.h>

namespace kuka_omnirob_robot
{

OmniRobRobotInterface::OmniRobRobotInterface()
{
}

OmniRobRobotInterface::~OmniRobRobotInterface()
{
}

bool OmniRobRobotInterface::start()
{
	LOG_ERROR("Not implemented yet");
	return true;
}

bool OmniRobRobotInterface::read(const ros::Time time,
		const ros::Duration period)
{
	LOG_ERROR("Not implemented yet");
	return true;
}

bool OmniRobRobotInterface::write(const ros::Time time,
		const ros::Duration period)
{
	LOG_ERROR("Not implemented yet");
	return true;
}

bool OmniRobRobotInterface::stop()
{
	LOG_ERROR("Not implemented yet");
	return true;
}

} /* namespace kuka_omnirob_robot */

PLUGINLIB_EXPORT_CLASS(kuka_omnirob_robot::OmniRobRobotInterface, prm_planner::RobotInterface)
