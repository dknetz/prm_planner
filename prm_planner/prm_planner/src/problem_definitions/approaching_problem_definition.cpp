/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jul 22, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: single_arm_problem_definition.cpp
 */

#include <prm_planner/visualization/robot_arm_interactive_marker.h>
#include <ais_log/log.h>
#include <ais_ros/ros_base_interface.h>
#include <boost/shared_ptr.hpp>
#include <eigen_conversions/eigen_kdl.h>
#include <prm_planner/objects/object_manager.h>
#include <prm_planner/planners/prm/prma_star.h>
#include <prm_planner_constraints/constraint.h>
#include <prm_planner/prm_planner.h>
#include <prm_planner/problem_definitions/approaching_problem_definition.h>
#include <prm_planner/robot/robot.h>
#include <prm_planner/util/parameter_server.h>
#include <prm_planner/visualization/object_interactive_marker.h>
#include <prm_planner_constraints/constraint_factory.h>
#include <prm_planner_robot/robot_arm.h>

namespace prm_planner
{

ApproachingProblemDefinition::ApproachingProblemDefinition()
{
}

ApproachingProblemDefinition::~ApproachingProblemDefinition()
{
}

void ApproachingProblemDefinition::initPlanner()
{
	ProblemDefinition::initPlanner();

	if (!c_config.isSingleArmProblemDefinition)
	{
		if (ParameterServer::robotConfigs[m_robot->c_robotName].arms[m_robot->getName()].hasSingleArmPlanner)
		{
			boost::shared_ptr<ProblemDefinition> pd = createApproachingProblemDefinition();
			m_approachingPathPlanner.reset(new PRMAStar(pd, ParameterServer::maxPlanningTime));
		}
	}
}

boost::shared_ptr<ProblemDefinition> ApproachingProblemDefinition::createApproachingProblemDefinition()
{
	//check if the problem definition is already
	//available
	if (m_approachingProblemDefinition.get() != NULL)
	{
		//if so return this pd
		return m_approachingProblemDefinition;
	}
	//otherwise create a new one
	else
	{
		//Create a copy of the robot to be able to
		//to plan independent to the other planners.
		//We only activate the corresponding arm of
		//the robot.
//		boost::shared_ptr<Robot> arm(new Robot(*m_robot));

		//build config
		parameters::ArmConfig ac = ParameterServer::robotConfigs[m_robot->c_robotName].arms[m_robot->getName()];
		parameters::ProblemConfig pc;
		pc.plannerType = ac.singleArmPlannerType;
		pc.constraint = ac.singleArmConstraint;
		pc.planningFrame = ac.singleArmPlanningFrame;
		pc.name = m_robot->getName() + "_single";
		pc.prmConfig = ac.singleArmPRMConfig;
		pc.rrtConfig = ac.singleArmRRTConfig;
		pc.rootFrame = c_config.rootFrame;
		pc.isSingleArmProblemDefinition = true;//set it to true, otherwise there will be a recursion!!!

		//create problem definition
		boost::shared_ptr<ProblemDefinition> pd(new ProblemDefinition());
		pd->init(m_robot, ConstraintFactory::create(pc.constraint), m_plannerInterface, pc);
		m_approachingProblemDefinition = pd;

		return pd;
	}
}

void ApproachingProblemDefinition::update(const boost::shared_ptr<prm_planner::PlanningScene>& planningScene)
{
	ProblemDefinition::update(planningScene);

	PD_WRITE_LOCK();
	if (m_approachingPathPlanner.get() != NULL)
		m_approachingPathPlanner->update(planningScene);
}

void ApproachingProblemDefinition::publish()
{
	ProblemDefinition::publish();

	PD_READ_LOCK();
	if (m_approachingPathPlanner.get() != NULL)
		m_approachingPathPlanner->publish();
}

} /* namespace prm_planner */
