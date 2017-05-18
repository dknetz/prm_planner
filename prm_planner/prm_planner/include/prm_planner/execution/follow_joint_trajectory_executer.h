/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Oct 10, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: follow_joint_trajectory_executer.h
 */

#ifndef H2AC8D05F_FD0F_4CEF_AD49_A9A1A4054786
#define H2AC8D05F_FD0F_4CEF_AD49_A9A1A4054786

#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/thread.hpp>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <prm_planner/execution/executer.h>

namespace prm_planner
{

FORWARD_DECLARE(Controller);
FORWARD_DECLARE(RobotArm);
FORWARD_DECLARE(SimulationRobotController);

class FollowJointTrajectoryExecuter: public Executer
{
public:
	FollowJointTrajectoryExecuter();
	virtual ~FollowJointTrajectoryExecuter();

	virtual bool executePath(const boost::shared_ptr<Path> path);
	virtual bool executePreprocessedPathMap(const boost::shared_ptr<Path>& path);

	virtual void stopMotion();

	virtual bool isGoalReached() const;
	virtual bool hasErrors() const;

	virtual double getPathLength();
	virtual double getExecutedPathLength();

	virtual void publish();

	virtual void lock();
	virtual void unlock();
	virtual void interrupt();

	virtual void init();
	virtual void reset();

protected:
	virtual void run();

	/**
	 * Generates the ROS messages given the computed paths.
	 * It removes way-points to reduce the amount to a suitable
	 * number.
	 * @goalMessages contains all ROS messages for each arm (which were defined in paths)
	 */
	void generateGoalMessage(ArmJointPath& path,
			control_msgs::FollowJointTrajectoryGoal& goalMessage);

private:
	boost::shared_ptr<SimulationRobotController> m_controller;
	boost::thread m_thread;
	boost::atomic_bool m_goalReached;
	boost::atomic_bool m_newDataReceived;
};

} /* namespace prm_planner */

#endif /* H2AC8D05F_FD0F_4CEF_AD49_A9A1A4054786 */
