/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jul 22, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: problem_definition_manager.cpp
 */

#include <ais_definitions/macros.h>
#include <prm_planner/prm_planner.h>
#include <prm_planner/problem_definitions/approaching_problem_definition.h>
#include <prm_planner_constraints/constraint_factory.h>
#include <prm_planner/problem_definitions/problem_definition_manager.h>
#include <prm_planner/robot/robot.h>
#include <prm_planner/util/parameter_server.h>
#include <prm_planner_constraints/constraint.h>

namespace prm_planner
{

SINGLETON_SOURCE(ProblemDefinitionManager)

ProblemDefinitionManager::ProblemDefinitionManager()
{
}

ProblemDefinitionManager::~ProblemDefinitionManager()
{
}

void ProblemDefinitionManager::init(const std::string& type,
		const boost::shared_ptr<PRMPlanner> planner)
{
	m_planner = planner;

	std::unordered_map<std::string, parameters::ProblemConfig>& problems = ParameterServer::problemConfigs;

	if (!CHECK_MAP(problems, type))
	{
		LOG_FATAL("Unknown problem definitions " << type);
		exit(-1);
	}

	const parameters::ProblemConfig& problem = problems[type];

	LOG_DEBUG("Setup robot");
	m_robot.reset(new Robot(problem.robotConfig));
	m_constraint = ConstraintFactory::create(problem.constraint);

	LOG_DEBUG("Loading problem definition");

	//load default problem definition for single arm configurations
	if (problem.pluginPackage == "prm_planner" && problem.pluginClass == "prm_planner::SingleArmProblemDefinition")
	{
		m_problemDefinition.reset(new ApproachingProblemDefinition());
	}
	//otherwise load the corresponding problem definition from a plugin
	else
	{
		m_problemDefinition = ProblemDefinition::load(problem.pluginPackage, problem.pluginClass);

		if (m_problemDefinition.get() == NULL)
		{
			LOG_FATAL("Loading the problem definition was not possible: ");
			LOG_FATAL("  pluginClass = " << problem.pluginClass);
			LOG_FATAL("  pluginPackage = " << problem.pluginPackage);
			exit(1432);
		}
	}

	m_problemDefinition->init(m_robot, m_constraint, planner, problem);
}

const boost::shared_ptr<Constraint>& ProblemDefinitionManager::getConstraint() const
{
	return m_constraint;
}

const boost::shared_ptr<Robot>& ProblemDefinitionManager::getRobot() const
{
	return m_robot;
}

boost::shared_ptr<ProblemDefinition> ProblemDefinitionManager::getProblemDefinition()
{
	if (m_problemDefinition.get() == NULL)
	{
		LOG_FATAL("No problem definition available. Please call init() before using this method");
		return boost::shared_ptr<ProblemDefinition>();
	}

	return m_problemDefinition;
}

} /* namespace prm_planner */

