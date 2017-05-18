/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Oct 10, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: robot_interface.cpp
 */

#include <ais_log/log.h>
#include <prm_planner_robot/robot_interface.h>

namespace prm_planner
{

pluginlib::ClassLoader<RobotInterface>* RobotInterface::s_loader = NULL;

RobotInterface::JointData::JointData() :
				cmdPos(0),
				cmdVel(0),
				cmdTorque(0),
				desiredPos(0),
				pos(0),
				vel(0),
				torque(0),
				lastPos(0),
				isRealJoint(true),
				mode(Velocity)
{
}

RobotInterface::RobotInterface()
{
}

RobotInterface::~RobotInterface()
{
}

boost::shared_ptr<RobotInterface> prm_planner::RobotInterface::load(const std::string& package,
		const std::string& library)
{
	//create instance if not already available
	if (s_loader == NULL)
		s_loader = new pluginlib::ClassLoader<RobotInterface>("prm_planner", "prm_planner::RobotInterface");

	boost::shared_ptr<RobotInterface> interface;

	try
	{
		interface = s_loader->createInstance(library);
	}
	catch (pluginlib::PluginlibException& ex)
	{
		LOG_FATAL("The plugin failed to load for some reason. Error: " << ex.what());
		exit(2);
	}

	return interface;
}

void RobotInterface::getJointState(KDL::JntArray& jointState,
		const std::vector<std::string>& names)
{
	jointState.resize(names.size());
	for (size_t i = 0; i < names.size(); ++i)
	{
		jointState(i) = m_data[names[i]].pos;
	}
}

void RobotInterface::setJointPositionCommand(const KDL::JntArray& jointState,
		const std::vector<std::string>& names)
{
	assert(jointState.rows() == names.size());

	for (size_t i = 0; i < names.size(); ++i)
	{
		JointData& j = m_data[names[i]];
		j.cmdPos = jointState(i);
		j.mode = JointData::Position;
	}
}

void RobotInterface::setJointVelocityCommand(const KDL::JntArray& velocities,
		const std::vector<std::string>& names)
{
	assert(velocities.rows() == names.size());

	for (size_t i = 0; i < names.size(); ++i)
	{
		JointData& j = m_data[names[i]];
		j.cmdVel = velocities(i);
		j.mode = JointData::Velocity;
	}
}

void RobotInterface::setJointTorqueCommand(const KDL::JntArray& torques,
		const std::vector<std::string>& names)
{
	assert(torques.rows() == names.size());

	for (size_t i = 0; i < names.size(); ++i)
	{
		JointData& j = m_data[names[i]];
		j.cmdTorque = torques(i);
		j.mode = JointData::Torque;
	}
}

} /* namespace prm_planner */

