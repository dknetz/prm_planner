/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Oct 10, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: iiwa_robot_interface.cpp
 */

#ifdef FOUND_IIWA_FRI

#ifdef FOUND_VREP
#include <vrep_interface/v_rep_iiwa.h>
#else
#define IIWA_1_JOINT "iiwa_1_joint"
#define IIWA_2_JOINT "iiwa_2_joint"
#define IIWA_3_JOINT "iiwa_3_joint"
#define IIWA_4_JOINT "iiwa_4_joint"
#define IIWA_5_JOINT "iiwa_5_joint"
#define IIWA_6_JOINT "iiwa_6_joint"
#define IIWA_7_JOINT "iiwa_7_joint"
#endif

#include <kuka_iiwa_robot/iiwa_robot_interface.h>

#include <ais_log/log.h>
#include <pluginlib/class_list_macros.h>

#define IIWA_IP "192.168.42.138" //192.168.42.138
#define IIWA_PORT 30200

namespace fri = KUKA::FRI;

namespace kuka_iiwa_robot
{

IiwaRobotInterface::IiwaRobotInterface() :
				m_udpConnection(NULL),
				m_client(NULL),
				m_clientApplication(NULL),
				m_firstRead(true)
{
}

IiwaRobotInterface::~IiwaRobotInterface()
{
	DELETE_ARRAY(m_clientApplication);
	DELETE_ARRAY(m_client);
	DELETE_ARRAY(m_udpConnection);
}

bool IiwaRobotInterface::start()
{
	m_udpConnection = new fri::UdpConnection;
	m_client = new Client(m_data);
	m_clientApplication = new fri::ClientApplication(*m_udpConnection, *m_client);
	m_clientApplication->connect(IIWA_PORT, IIWA_IP);
	m_thread = boost::thread(boost::bind(&IiwaRobotInterface::run, this));

	return false;
}

bool IiwaRobotInterface::read(const ros::Time time,
		const ros::Duration period)
{
	const fri::LBRState& state = m_client->robotState();

	const double* joints = state.getMeasuredJointPosition();
	const double* torques = state.getMeasuredTorque();

	//set position and torque (directly retrieved from robot)
	m_data[IIWA_1_JOINT].pos = joints[0];
	m_data[IIWA_1_JOINT].torque = torques[0];

	m_data[IIWA_2_JOINT].pos = joints[1];
	m_data[IIWA_2_JOINT].torque = torques[1];

	m_data[IIWA_3_JOINT].pos = joints[2];
	m_data[IIWA_3_JOINT].torque = torques[2];

	m_data[IIWA_4_JOINT].pos = joints[3];
	m_data[IIWA_4_JOINT].torque = torques[3];

	m_data[IIWA_5_JOINT].pos = joints[4];
	m_data[IIWA_5_JOINT].torque = torques[4];

	m_data[IIWA_6_JOINT].pos = joints[5];
	m_data[IIWA_6_JOINT].torque = torques[5];

	m_data[IIWA_7_JOINT].pos = joints[6];
	m_data[IIWA_7_JOINT].torque = torques[6];

	//compute the velocity based on the positions
	for (auto& it : m_data)
	{
		double posDiff = it.second.pos - it.second.lastPos;
		ros::Duration timeDiff = time - it.second.lastTime;

		//the first time we set a velocity of 0 because we don't
		//have no old data to compute the difference
		it.second.vel = m_firstRead ? 0.0 : (posDiff / timeDiff.toSec());
		it.second.lastPos = it.second.pos;
		it.second.lastTime = time;
	}

	m_firstRead = false;

//	LOG_INFO(state.getConnectionQuality());

//	if (!m_clientApplication->step())
//			return false;

	return true;
}

prm_planner::RobotInterface::JointData::Mode IiwaRobotInterface::setCmd(double& cmd,
		JointData& jd)
{
	if (jd.mode == RobotInterface::JointData::Position)
	{
		cmd = jd.cmdPos;
		return RobotInterface::JointData::Position;
	}
	else
	if (jd.mode == RobotInterface::JointData::Torque)
	{
		cmd = jd.cmdTorque;
		return RobotInterface::JointData::Torque;
	}
	else
	{
		LOG_ERROR("The iiwa interface only supports position and torque commands!");
		return RobotInterface::JointData::Unknown;
	}
}

void IiwaRobotInterface::run()
{
	ros::Rate r(1000);
	while (ros::ok())
	{
		m_clientApplication->step();
		r.sleep();
	}
}

bool IiwaRobotInterface::write(const ros::Time time,
		const ros::Duration period)
{
	double cmd[7];
	RobotInterface::JointData::Mode mode = RobotInterface::JointData::Unknown;

	mode = setCmd(cmd[0], m_data[IIWA_1_JOINT]);
	mode = setCmd(cmd[1], m_data[IIWA_2_JOINT]);
	mode = setCmd(cmd[2], m_data[IIWA_3_JOINT]);
	mode = setCmd(cmd[3], m_data[IIWA_4_JOINT]);
	mode = setCmd(cmd[4], m_data[IIWA_5_JOINT]);
	mode = setCmd(cmd[5], m_data[IIWA_6_JOINT]);
	mode = setCmd(cmd[6], m_data[IIWA_7_JOINT]);

	m_client->setCMD(cmd, mode);

	return true;
}

bool IiwaRobotInterface::stop()
{
	if (m_clientApplication)
		m_clientApplication->disconnect();
	return true;
}

IiwaRobotInterface::Client::Client(Data& data) :
				m_mode(RobotInterface::JointData::Unknown)
{
}

void IiwaRobotInterface::Client::command()
{
	boost::mutex::scoped_lock lock(m_mutex);
	LOG_INFO("called");

	const fri::LBRState& state = robotState();

	if (m_mode == RobotInterface::JointData::Position)
		robotCommand().setJointPosition(m_cmd);
	else if (m_mode == RobotInterface::JointData::Torque)
	{
		const double* currentState = state.getMeasuredJointPosition();
		robotCommand().setJointPosition(currentState);
		robotCommand().setTorque(m_cmd);
	}
	else
		LOG_ERROR("Unknown command!");
}

void IiwaRobotInterface::Client::onStateChange(fri::ESessionState oldState,
		fri::ESessionState newState)
{
	LBRClient::onStateChange(oldState, newState);
	LOG_INFO("State change from " << oldState << " to " << newState);
}

void IiwaRobotInterface::Client::setCMD(double* cmd,
		RobotInterface::JointData::Mode& mode)
{
	boost::mutex::scoped_lock lock(m_mutex);
	for (size_t i = 0; i < 7; ++i)
		m_cmd[i] = cmd[i];

	m_mode = mode;
}

} /* namespace kuka_iiwa_robot */

PLUGINLIB_EXPORT_CLASS(kuka_iiwa_robot::IiwaRobotInterface, prm_planner::RobotInterface)

#endif
