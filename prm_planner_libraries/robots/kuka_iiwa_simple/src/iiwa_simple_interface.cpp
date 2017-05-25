/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Oct 10, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: iiwa_v_rep_interface.cpp
 */

#include <kuka_iiwa_simple/iiwa_simple_interface.h>

#include <ais_log/log.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/JointState.h>

namespace kuka_iiwa_simple
{

IiwaSimpleInterface::IiwaSimpleInterface()
{
	ros::NodeHandle n;
	std::vector<std::string> names;
	std::vector<double> values;
	std::string jointTopic;
	if (n.getParam("iiwa_simple_interface/start_joint_names", names) && n.getParam("iiwa_simple_interface/start_joint_values", values))
	{
		for (size_t i = 0; i < names.size(); ++i)
		{
			m_data[names[i]].pos = values[i];
		}
	}
	else
	{
		LOG_INFO("No start joints given, assuming 0");
	}

	if (!n.getParam("iiwa_simple_interface/joint_topic", jointTopic))
	{
		LOG_FATAL("Please provide joint state topic");
		exit(18934);
	}

	m_jointPub = n.advertise<sensor_msgs::JointState>(jointTopic, 1);
}

IiwaSimpleInterface::~IiwaSimpleInterface()
{
}

bool IiwaSimpleInterface::start()
{
	return true;
}

bool IiwaSimpleInterface::read(const ros::Time time,
		const ros::Duration period)
{
	//apply commands
	for (auto& it : m_data)
	{
		if (it.second.mode == JointData::Velocity)
		{
			it.second.pos += period.toSec() * it.second.cmdVel;
		}
		else if (it.second.mode == JointData::Torque)
		{
			LOG_ERROR("Not implemented");
		}
		else
		{
			it.second.pos = it.second.cmdPos;
		}
	}

	sensor_msgs::JointState js;
	js.header.stamp = ros::Time::now();
	for (auto& it : m_data)
	{
		js.name.push_back(it.first);
		js.position.push_back(it.second.pos);
		js.velocity.push_back(it.second.vel);
		js.effort.push_back(it.second.torque);
	}

	m_jointPub.publish(js);

	return true;
}

bool IiwaSimpleInterface::write(const ros::Time time,
		const ros::Duration period)
{
	return true;
}

bool IiwaSimpleInterface::stop()
{
	return true;
}

} /* namespace kuka_iiwa_vrep */

PLUGINLIB_EXPORT_CLASS(kuka_iiwa_simple::IiwaSimpleInterface, prm_planner::RobotInterface)

