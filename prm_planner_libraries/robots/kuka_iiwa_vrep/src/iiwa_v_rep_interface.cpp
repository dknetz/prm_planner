/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Oct 10, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: iiwa_v_rep_interface.cpp
 */

#include <kuka_iiwa_vrep/iiwa_v_rep_interface.h>

#include <ais_log/log.h>
#include <pluginlib/class_list_macros.h>

namespace kuka_iiwa_vrep
{

IiwaVRepInterface::IiwaVRepInterface()
#ifdef FOUND_VREP
:
				m_vrep(NULL),
				m_firstRead(true)
#endif
{
}

IiwaVRepInterface::~IiwaVRepInterface()
{
}

bool IiwaVRepInterface::start()
{
#ifdef FOUND_VREP
	m_vrep = vrep_interface::VRepInterface::getInstance();
	if (!m_vrep->checkConnection())
		return false;

	//get all handles
	m_vrepData[IIWA_1_JOINT].vrepHandle = m_vrep->getJointHandle(IIWA_1_JOINT);
	m_vrepData[IIWA_2_JOINT].vrepHandle = m_vrep->getJointHandle(IIWA_2_JOINT);
	m_vrepData[IIWA_3_JOINT].vrepHandle = m_vrep->getJointHandle(IIWA_3_JOINT);
	m_vrepData[IIWA_4_JOINT].vrepHandle = m_vrep->getJointHandle(IIWA_4_JOINT);
	m_vrepData[IIWA_5_JOINT].vrepHandle = m_vrep->getJointHandle(IIWA_5_JOINT);
	m_vrepData[IIWA_6_JOINT].vrepHandle = m_vrep->getJointHandle(IIWA_6_JOINT);
	m_vrepData[IIWA_7_JOINT].vrepHandle = m_vrep->getJointHandle(IIWA_7_JOINT);

//	//set velocities to zero
//	for (auto& it : m_vrepData)
//	{
//		m_vrep->setJointVelocity(it.second.vrepHandle, 0);
//	}

	m_vrep->activateSyncMode();

	return true;

#else
	LOG_FATAL("You cannot use this interface because it wasn't compiled with VREP support!");
	exit(-3);
#endif
}

bool IiwaVRepInterface::read(const ros::Time time,
		const ros::Duration period)
{
#ifdef FOUND_VREP
	if (!m_vrep->checkConnection())
		return false;

	for (auto& it : m_data)
	{
		VrepData& vrepData = m_vrepData[it.first];

		//get joint position
		if (!m_vrep->getJointPosition(vrepData.vrepHandle, it.second.pos))
			return false;

		//compute velocity
		double posDiff = it.second.pos - it.second.lastPos;
		ros::Duration timeDiff = time - it.second.lastTime;

		//the first time we set a velocity of 0 because we don't
		//have no old data to compute the difference
		it.second.vel = m_firstRead ? 0.0 : (posDiff / timeDiff.toSec());
		it.second.lastPos = it.second.pos;
		it.second.lastTime = time;

		//get joint effort
		if (!m_vrep->getJointTorque(vrepData.vrepHandle, it.second.torque))
			return false;
	}

	m_firstRead = true;

	return true;
#else
	LOG_FATAL("You cannot use this interface because it wasn't compiled with VREP support!");
	exit(-3);
#endif
}

bool IiwaVRepInterface::write(const ros::Time time,
		const ros::Duration period)
{
#ifdef FOUND_VREP
	if (!m_vrep->checkConnection())
		return false;

	//we send all commands at the same time
	m_vrep->pauseCommunication();

	for (auto& it : m_data)
	{
		VrepData& vrepData = m_vrepData[it.first];
		if (it.second.mode == JointData::Velocity)
			m_vrep->setJointVelocity(vrepData.vrepHandle, it.second.cmdVel);
		else if (it.second.mode == JointData::Torque)
			m_vrep->setJointTorque(vrepData.vrepHandle, it.second.cmdTorque);
		else
			m_vrep->setJointPosition(vrepData.vrepHandle, it.second.cmdPos);
	}

	//send the data
	m_vrep->startCommunication();

	//do a simulation step
	m_vrep->step();

	return true;
#else
	LOG_FATAL("You cannot use this interface because it wasn't compiled with VREP support!");
	exit(-3);
#endif
}

bool IiwaVRepInterface::stop()
{
#ifdef FOUND_VREP
	return m_vrep->stopSimulation();
#else
	LOG_FATAL("You cannot use this interface because it wasn't compiled with VREP support!");
	exit(-3);
#endif
}

} /* namespace kuka_iiwa_vrep */

PLUGINLIB_EXPORT_CLASS(kuka_iiwa_vrep::IiwaVRepInterface, prm_planner::RobotInterface)

