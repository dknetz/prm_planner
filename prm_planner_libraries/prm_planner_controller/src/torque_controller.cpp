/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Oct 28, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: torque_controller.cpp
 */

#include <prm_planner_constraints/constraint.h>
#include <prm_planner_controller/torque_controller.h>

namespace prm_planner
{

TorqueController::TorqueController(const ControllerParameters& parameters,
		const boost::shared_ptr<Constraint> constraint,
		boost::shared_ptr<RobotArm> robotArm,
		const std::string& planningFrame,
		const double octomapResolution) :
				Controller(robotArm)
{
}

TorqueController::~TorqueController()
{
}

bool TorqueController::updateFromPath(const boost::shared_ptr<Path> path)
{
	return true;
}

void TorqueController::initRobotControl()
{
}

bool TorqueController::isGoalReached() const
{
	return true;
}

bool TorqueController::isSuccess() const
{
	return true;
}

bool TorqueController::isTrajectoryAvailable() const
{
	return true;
}

void TorqueController::reset()
{
}

double TorqueController::getPathLength()
{
	return 0.0;
}

double TorqueController::getExecutionPathLength()
{
	return 0.0;
}

double TorqueController::getPredictedExecutionTime() const
{
	return 0.0;
}

void TorqueController::setPredictedExecutionTime(double predictedExecutionTime)
{
}

bool TorqueController::update(const ros::Time& now,
		const ros::Duration& dt)
{
	return true;
}

bool TorqueController::computeJacobian(Matrix6xN& jacobian)
{
}

} /* namespace prm_planner */
