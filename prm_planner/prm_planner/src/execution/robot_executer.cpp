/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Oct 10, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: robot_executer.cpp
 */

#include <ais_definitions/class.h>
#include <ais_ros/ros_base_interface.h>
#include <prm_planner/controllers/helpers.h>
#include <prm_planner/execution/robot_executer.h>
#include <prm_planner/problem_definitions/problem_definition.h>
#include <prm_planner/problem_definitions/problem_definition_manager.h>
#include <prm_planner/robot/robot.h>
#include <prm_planner_constraints/constraint.h>
#include <prm_planner_controller/velocity_controller.h>

namespace prm_planner
{

RobotExecuter::RobotExecuter()
{
	ProblemDefinitionManager* pdm = ProblemDefinitionManager::getInstance();
	boost::shared_ptr<Constraint> constraint = pdm->getConstraint();

	ControllerParameters params = getControllerParametersFromParameterServer(m_robot->getParameters().controllerConfig);
//	params.optimizeJointRangeInNullspace = true;

	if (params.type == "VelocityController7DOF")
	{
		m_controller = new VelocityController<7>(params,
				constraint, m_robot, pdm->getProblemDefinition()->getFrame(),
				ParameterServer::octomapResolution);
	}
	else if (params.type == "VelocityController7Plus3DOF")
	{
		m_controller = new VelocityController<10>(params,
				constraint, m_robot, pdm->getProblemDefinition()->getFrame(),
				ParameterServer::octomapResolution);
	}
	else
	{
		LOG_FATAL("Unknown controller type");
	}
}

RobotExecuter::~RobotExecuter()
{
	interrupt();
	m_threadController.join();
	m_threadPathUpdater.join();
	DELETE_VAR(m_controller);
}

bool RobotExecuter::isGoalReached() const
{
	EXECUTER_LOCK();
	return m_pathSegments.size() == 0 && m_controller->isGoalReached();
}

bool RobotExecuter::hasErrors() const
{
	return !m_controller->isSuccess();
}

double RobotExecuter::getPathLength()
{
	return m_controller->getPathLength();
}

double RobotExecuter::getExecutedPathLength()
{
	return m_controller->getExecutionPathLength();
}

void RobotExecuter::stopMotion()
{
	EXECUTER_LOCK();
	m_pathSegments.clear();
	m_currentPathSegment = 0;
	m_controller->reset();
}

void RobotExecuter::init()
{
	m_controller->init();

	m_threadController = boost::thread(&RobotExecuter::runController, this);
	m_threadPathUpdater = boost::thread(&RobotExecuter::runPathUpdater, this);
}

void RobotExecuter::reset()
{
	m_controller->reset();
}

void RobotExecuter::publish()
{
	m_controller->publish();
}

void RobotExecuter::lock()
{
	m_controller->lock();
}

void RobotExecuter::unlock()
{
	m_controller->unlock();
}

void RobotExecuter::interrupt()
{
	m_threadController.interrupt();
	m_threadPathUpdater.interrupt();
}

void RobotExecuter::runController()
{
	ros::Time now, last;

	Controller::getTime(now);
	last = now;
	ros::Duration dt = now - last;

	//TODO: getting frequency is not optimal yet
	ControllerParameters params = getControllerParametersFromParameterServer(m_robot->getParameters().controllerConfig);

	ros::Rate r(params.frequency);

	while (!boost::this_thread::interruption_requested())
	{
		last = now;
		Controller::getTime(now);
		dt = now - last;

		if (m_controller->update(now, dt)) //returns true on error
		{
			m_executedTrajectoryMutex.lock();
			writeFile();
			m_executedTrajectoryMutex.unlock();
		}
		else
		{
			m_executedTrajectoryMutex.lock();
			m_executedTrajectory.push_back(m_controller->getCurrentJointPosition());
			m_executedTrajectoryMutex.unlock();
		}

		r.sleep();
	}
}

void RobotExecuter::runPathUpdater()
{
	ros::Rate r(100);

	while (!boost::this_thread::interruption_requested())
	{
		m_mutex.lock();

		//if we have no new paths we can continue
		if (m_pathSegments.empty())
		{
			m_mutex.unlock();
		}
		else
		{
			m_mutex.unlock();

			//check if the current goal is reached
			bool finished = m_controller->isGoalReached();

			//if so, go on to the next path.
			//we also send a new path, if no path was send so far,
			//i.e., m_currentPathSegment is 0
			if (finished || m_currentPathSegment == 0)
			{
				writeFile();

				m_mutex.lock();
				SubPath pathSegments = m_pathSegments;
				auto& path = pathSegments[m_currentPathSegment];
				m_mutex.unlock();

				//check if the next path is special command
				if (path->isSpecialCommand())
				{
					handleSpecialCommand(pathSegments, m_currentPathSegment.load());
				}
				//send new paths
				else
				{
					sendPath(pathSegments, m_currentPathSegment, m_controller);
				}

				//increment the counter again to get the
				//next command
				++m_currentPathSegment;

				//finished?
				if (m_currentPathSegment == pathSegments.size())
				{
					m_mutex.lock();
					m_pathSegments.clear();
					m_currentPathSegment = 0;
					m_mutex.unlock();
					continue;
				}
			}
		}

		r.sleep();

	}
}

void RobotExecuter::writeFile()
{
	m_executedTrajectoryMutex.lock();
	static int k = 0;
	if (m_executedTrajectory.empty())
	{
		return;
	}
	LOG_INFO("Writing executed trajectory with id " << k);
	writeTrajectory(std::string("/tmp/exec_traj_") + std::to_string(k) + ".traj");
	m_controller->writeData(std::string("/tmp/exec_controller_") + std::to_string(k++));
	m_executedTrajectory.clear();
	m_executedTrajectoryMutex.unlock();
}

} /* namespace prm_planner */

