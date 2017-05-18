/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Feb 5, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: trajectory_validator.cpp
 */

#include <ais_log/log.h>
#include <prm_planner/controllers/simulation_velocity_controller.h>
#include <prm_planner/planners/path_planner.h>
#include <prm_planner/robot/robot.h>
#include <prm_planner/robot/trajectory_validator.h>

namespace prm_planner
{

TrajectoryValidator::TrajectoryValidator(boost::shared_ptr<VelocityController7DOF> controller,
		boost::shared_ptr<PathPlanner> planner,
		boost::shared_ptr<Robot> robot) :
				m_controller(controller),
				m_planner(planner),
				m_robot(robot),
				m_runThread(true)
{
}

TrajectoryValidator::~TrajectoryValidator()
{
	m_runThread = false;
	m_thread.join();
}

void TrajectoryValidator::startValidation()
{
	//start thread
	m_thread = boost::thread(&TrajectoryValidator::update, this);
}

void TrajectoryValidator::update()
{
//	ros::Rate r(100);
//	while (m_controller->isRunning() && m_runThread)
//	{
//		if (m_controller->isTrajectoryAvailable() && !m_controller->isGoalReached())
//		{
//			m_simulationController.reset(
//					new SimulationVelocityController7DOF(m_controller, m_controller->c_parameters, m_robot));
//
//			PlannerErrorCodes state = m_simulationController->canControl(0.25, 0);
//			if (state != PlannerSuccess)
//			{
//				LOG_INFO("problem detected: " << errorToStr(state));
//			}
//		}
//
//		r.sleep();
//	}
}

} /* namespace prm_planner */

