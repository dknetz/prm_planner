/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Feb 5, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: trajectory_validator.h
 */

#ifndef TRAJECTORY_VALIDATOR_H_
#define TRAJECTORY_VALIDATOR_H_
#include <boost/thread/thread.hpp>

namespace prm_planner
{

class VelocityController7DOF;
class SimulationVelocityController7DOF;
class PathPlanner;
class Robot;

class TrajectoryValidator
{
public:
	TrajectoryValidator(boost::shared_ptr<VelocityController7DOF> controller,
			boost::shared_ptr<PathPlanner> planner,
			boost::shared_ptr<Robot> robot);
	virtual ~TrajectoryValidator();

	void startValidation();

private:
	void update();

private:
	boost::thread m_thread;
	boost::shared_ptr<VelocityController7DOF> m_controller;
	boost::shared_ptr<PathPlanner> m_planner;
	boost::shared_ptr<SimulationVelocityController7DOF> m_simulationController;
	boost::shared_ptr<Robot> m_robot;
	boost::atomic<bool> m_runThread;
};

} /* namespace prm_planner */

#endif /* TRAJECTORY_VALIDATOR_H_ */
