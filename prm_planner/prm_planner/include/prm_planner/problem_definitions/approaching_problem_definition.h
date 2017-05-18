/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jul 22, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: single_arm_problem_definition.h
 */

#ifndef HEDADD34C_78E6_4875_81B6_30E73FDB6EA8
#define HEDADD34C_78E6_4875_81B6_30E73FDB6EA8

#include <prm_planner/problem_definitions/problem_definition.h>

namespace prm_planner
{

class RobotArm;

class ApproachingProblemDefinition: public ProblemDefinition
{
public:
	ApproachingProblemDefinition();
	virtual ~ApproachingProblemDefinition();

	/**
	 * Given the name, it returns the problem definition of
	 * an arm. To get these problem definitions you need to
	 * specify it in the configuration of the planner.
	 * They can be used to approach a pre-target pose or
	 * control a single arm.
	 *
	 * @return: NULL pointer, if there is no problem definition
	 */
	boost::shared_ptr<ProblemDefinition> createApproachingProblemDefinition();

	virtual void update(const boost::shared_ptr<PlanningScene>& planningScene);

	virtual void publish();

protected:
	virtual void initPlanner();

protected:
	boost::shared_ptr<ProblemDefinition> m_approachingProblemDefinition;
	boost::shared_ptr<PathPlanner> m_approachingPathPlanner;
};

} /* namespace prm_planner */

#endif /* HEDADD34C_78E6_4875_81B6_30E73FDB6EA8 */
