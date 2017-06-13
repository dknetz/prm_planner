/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Oct 10, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: iiwa_v_rep_interface.cpp
 */

#include <ais_log/log.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/JointState.h>
#include <robot_fake_interface/robot_fake_interface.h>

namespace robot_fake_interface
{

RobotFakeInterface::RobotFakeInterface()
{
	ros::NodeHandle n;
	std::vector<double> values;
	std::string jointTopic;
	if (n.getParam("robot_fake_interface/start_joint_names", m_chainNames) && n.getParam("robot_fake_interface/start_joint_values", values))
	{
		for (size_t i = 0; i < m_chainNames.size(); ++i)
		{
			m_data[m_chainNames[i]].pos = values[i];
		}
	}
	else
	{
		LOG_INFO("No start joints given, assuming 0");
	}

	if (!n.getParam("robot_fake_interface/joint_topic", jointTopic))
	{
		LOG_FATAL("Please provide joint state topic");
		exit(18934);
	}

	m_jointPub = n.advertise<sensor_msgs::JointState>(jointTopic, 1);
	m_jointSub = n.subscribe(jointTopic, 1, &RobotFakeInterface::receiveJointState, this);
}

RobotFakeInterface::~RobotFakeInterface()
{
}

bool RobotFakeInterface::start()
{
	return true;
}

bool RobotFakeInterface::read(const ros::Time time,
		const ros::Duration period)
{
	sensor_msgs::JointState js;
	js.header.stamp = ros::Time::now();

	//apply commands and build message for publisher
	for (auto& it : m_data)
	{
		if (std::find(m_chainNames.begin(), m_chainNames.end(), it.first) == m_chainNames.end())
			continue;

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

		js.name.push_back(it.first);
		js.position.push_back(it.second.pos);
		js.velocity.push_back(it.second.vel);
		js.effort.push_back(it.second.torque);
	}

	m_jointPub.publish(js);

	return true;
}

bool RobotFakeInterface::write(const ros::Time time,
		const ros::Duration period)
{
	return true;
}

bool RobotFakeInterface::stop()
{
	return true;
}

void RobotFakeInterface::receiveJointState(const sensor_msgs::JointStateConstPtr& jointState)
{
	for (size_t i = 0; i < jointState->name.size(); ++i)
	{
		if (std::find(m_chainNames.begin(), m_chainNames.end(), jointState->name[i]) != m_chainNames.end())
			continue;

		m_data[jointState->name[i]].pos = jointState->position[i];
	}
}

} /* namespace robot_fake_interface */

PLUGINLIB_EXPORT_CLASS(robot_fake_interface::RobotFakeInterface, prm_planner::RobotInterface)

