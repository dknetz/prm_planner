/*
 * velocity_controller.cpp
 *
 *  Created on: Aug 22, 2016
 *      Author: kuhnerd
 */

#include <ais_log/log.h>
#include <prm_planner_controller/controller.h>
#include <prm_planner_robot/robot_arm.h>

namespace prm_planner
{

Controller::Controller(boost::shared_ptr<RobotArm> arm) :
				m_robotArm(arm),
				m_jointNames(arm->getChainJointNames())
{
	for (auto& it : m_jointNames)
	{
		m_robotState[it] = 0;
	}
}

Controller::~Controller()
{
}

void Controller::getTime(ros::Time& now)
{
	struct timespec ts = { 0, 0 };
	clock_gettime(CLOCK_REALTIME, &ts);
	now.sec = ts.tv_sec;
	now.nsec = ts.tv_nsec;
}

bool Controller::updateFromValues(const Eigen::Affine3d& startPose,
		const Eigen::Affine3d& goalPose,
		const KDL::JntArray& startJoints)
{
	LOG_ERROR("Don't call this method directly");
	return false;
}

KDL::JntArray Controller::getCurrentJointPosition()
{
	return m_q;
}

fcl_robot_model::RobotState Controller::getCurrentRobotState()
{
	if (m_q.rows() > 0)
	{
		int i = 0;
		for (auto& it : m_jointNames)
		{
			m_robotState[it] = m_q(i++);
		}
	}

	return m_robotState;
}

void Controller::publish()
{
}

void Controller::lock()
{
}

void Controller::unlock()
{
}

const ArmJointPath& Controller::getJointPositions() const
{
	LOG_FATAL("Don't call this method directly");
	exit(-1);
//	return std::vector<KDL::JntArray>();
}

void Controller::init()
{
}

bool Controller::writeData(const std::string& baseFileName)
{
	return true;
}

} /* namespace prm_planner */

