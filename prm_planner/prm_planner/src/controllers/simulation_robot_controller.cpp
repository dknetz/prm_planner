/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jul 21, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: simulation_robot_controller.cpp
 */

#include <ais_ros/ros_base_interface.h>
#include <ais_util/stop_watch.h>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <prm_planner/controllers/helpers.h>
#include <prm_planner/controllers/simulation_robot_controller.h>
#include <prm_planner/controllers/simulation_velocity_controller.h>
#include <prm_planner/planners/prm/prm_edge.h>
#include <prm_planner/planners/prm/prm_node.h>
#include <prm_planner/problem_definitions/problem_definition.h>
#include <prm_planner/problem_definitions/problem_definition_manager.h>
#include <prm_planner_controller/controller.h>
#include <prm_planner/robot/robot.h>
#include <prm_planner/collision_detection/collision_detector.h>
#include <fcl_wrapper/robot_model/robot_model.h>

namespace prm_planner
{

SimulationRobotController::SimulationRobotController(const PRMEdge* edge,
		boost::shared_ptr<Robot> robot,
		boost::shared_ptr<ProblemDefinition> pd) :
				m_edge(edge),
				m_pd(pd),
				m_robot(robot)
{
	ControllerParameters params = getControllerParametersFromParameterServer(m_robot->getParameters().controllerConfig);

	if (params.type == "VelocityController7DOF")
	{
		m_controller = new SimulationVelocityControllerN<7>(params, m_robot, m_pd);
	}
	else if (params.type == "VelocityController7Plus3DOF")
	{
		m_controller = new SimulationVelocityControllerN<10>(params, m_robot, m_pd);
	}
}

SimulationRobotController::SimulationRobotController(const Eigen::Affine3d& startPose,
		const Eigen::Affine3d& goalPose,
		KDL::JntArray& startJointState,
		boost::shared_ptr<Robot> robot,
		boost::shared_ptr<ProblemDefinition> pd) :
				m_edge(NULL),
				m_pd(pd),
				m_robot(robot)
{
	ControllerParameters params = getControllerParametersFromParameterServer(m_robot->getParameters().controllerConfig);

	if (params.type == "VelocityController7DOF")
	{
		m_controller = new SimulationVelocityControllerN<7>(startPose, goalPose, startJointState, params, m_robot, m_pd);
	}
	else if (params.type == "VelocityController7Plus3DOF")
	{
		m_controller = new SimulationVelocityControllerN<10>(startPose, goalPose, startJointState, params, m_robot, m_pd);
	}
}

SimulationRobotController::SimulationRobotController(boost::shared_ptr<Robot> robot,
		boost::shared_ptr<ProblemDefinition> pd) :
				m_edge(NULL),
				m_pd(pd),
				m_robot(robot)
{
	ControllerParameters params = getControllerParametersFromParameterServer(m_robot->getParameters().controllerConfig);

	if (params.type == "VelocityController7DOF")
	{
		m_controller = new SimulationVelocityControllerN<7>(params, m_robot, m_pd);
	}
	else if (params.type == "VelocityController7Plus3DOF")
	{
		m_controller = new SimulationVelocityControllerN<10>(params, m_robot, m_pd);
	}
}

SimulationRobotController::~SimulationRobotController()
{
	DELETE_VAR(m_controller);
}

bool SimulationRobotController::canControl(const double maxWaitTime,
		const double dt)
{
	ros::Time now = ros::Time(0);
	ros::Time start = ros::Time::now();
	ros::Duration deltaT(dt);
	double lastCollisionCheck = 0;

	bool useCollisionDetection = m_cd.get() != NULL;

//	struct timespec startTime, nowTime;
//	clock_gettime( CLOCK_THREAD_CPUTIME_ID, &startTime); //we use CLOCK_THREAD_CPUTIME_ID to get thread time

	int counter = 0;
	while (++counter < maxWaitTime)
	{
//		clock_gettime( CLOCK_THREAD_CPUTIME_ID, &nowTime);
//		double diff = (nowTime.tv_sec - startTime.tv_sec) + (nowTime.tv_nsec - startTime.tv_nsec) / 1000000000.0;
//		if (diff > maxWaitTime)
//			break;

		now += deltaT;
		lastCollisionCheck += deltaT.toSec();

		bool goalReached = true;

		//update controllers
		if (!m_controller->update(now, deltaT))
		{
			return false;
		}

		//check collision after x sec and at the beginning
		if (useCollisionDetection && lastCollisionCheck > 0.3)
		{
			m_cd->robot->setRobotState(m_controller->getCurrentRobotState());
		}

		//check collisions
		if (lastCollisionCheck > 0.3)
		{
			if (useCollisionDetection && m_cd->fcl->checkCollisions())
			{
//				fcl_collision_detection::FCLWrapper::CollisionsVector col;
//				m_cd->fcl->getCollisions(col);
//				LOG_INFO("found col" << now.toSec() );
//				for (auto& it: col) {
//					LOG_INFO(it.first->getName() << " " << it.second->getName());
//				}
				return false;
			}
			lastCollisionCheck = 0;
		}

//		static int printed = 0;
//		LOG_INFO_COND(printed++ < 10, "COLLISION AVOIDANCE DEACTIVATED!!!");

//check if goal is reached
		goalReached = m_controller->isGoalReached();

		if (goalReached)
		{
			//check collisions in goal state
			if (useCollisionDetection)
			{
				m_cd->robot->setRobotState(m_controller->getCurrentRobotState());

				if (m_cd->fcl->checkCollisions())
				{
					return false;
				}
			}

			return true;
		}
	}

	return false;
}

bool SimulationRobotController::updateFromEdge(const PRMNode* startNode,
		KDL::JntArray& startJoint)
{
	if (m_edge == NULL)
	{
		LOG_ERROR("edge is NULL!");
		return false;
	}

	const PRMNode* n1;
	const PRMNode* n2;

	if (startNode->getId() == m_edge->getNode1()->getId())
	{
		n1 = m_edge->getNode1();
		n2 = m_edge->getNode2();
	}
	else
	{
		n1 = m_edge->getNode2();
		n2 = m_edge->getNode1();
	}

	Eigen::Affine3d pStart = n1->getPose(), pGoal = n2->getPose();

	return m_controller->updateFromValues(pStart, pGoal, startJoint);
}

bool SimulationRobotController::updateFromValues(const Eigen::Affine3d& startPose,
		const Eigen::Affine3d& goalPose,
		KDL::JntArray& startJoint)
{
	return m_controller->updateFromValues(startPose, goalPose, startJoint);
}

bool SimulationRobotController::updateFromRobotPath(const std::vector<boost::shared_ptr<Path>> path,
		const int currentIndex,
		const bool adjustRuntimes)
{
	return m_controller->updateFromPath(path[currentIndex]);
}

void SimulationRobotController::getFinalJointState(KDL::JntArray& finalJointState)
{
	finalJointState = m_controller->getCurrentJointPosition();
}

void SimulationRobotController::getJointPath(ArmJointPath& jointPath) const
		{
	jointPath = m_controller->getJointPositions();
}

void SimulationRobotController::setCollisionDetection(boost::shared_ptr<CollisionDetector>& cd)
{
	m_cd = cd;
}

void SimulationRobotController::reset()
{
	m_controller->reset();
}

} /* namespace prm_planner */

