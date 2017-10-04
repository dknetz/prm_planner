/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Oct 10, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: robot_executer.h
 */

#ifndef H378F1FC2_E698_49CD_B824_18FA1092B8E9
#define H378F1FC2_E698_49CD_B824_18FA1092B8E9

#include <boost/atomic.hpp>
#include <boost/thread/thread.hpp>
#include <prm_planner/execution/executer.h>

namespace prm_planner
{

class Controller;

/**
 * RobotExecuter uses a controller to directly
 * move the arm with a given RobotInterface instance.
 */
class RobotExecuter: public Executer
{
public:
	RobotExecuter();
	virtual ~RobotExecuter();

	virtual bool isGoalReached() const;
	virtual bool hasErrors() const;

	virtual double getPathLength();
	virtual double getExecutedPathLength();

	virtual void stopMotion();

	virtual void init();
	virtual void reset();

	virtual void publish();

	virtual void lock();
	virtual void unlock();
	virtual void interrupt();

protected:
	/**
	 * This method runs in a thread and periodically
	 * calls controllers update method
	 */
	virtual void runController();

	/**
	 * This method runs in a thread and periodically
	 * checks, if the current goal is reached and sends
	 * new paths to the controllers if required.
	 */
	virtual void runPathUpdater();

	void writeFile();

protected:
	boost::thread m_threadController, m_threadPathUpdater;
	Controller* m_controller;
};

} /* namespace prm_planner */

#endif /* H378F1FC2_E698_49CD_B824_18FA1092B8E9 */
