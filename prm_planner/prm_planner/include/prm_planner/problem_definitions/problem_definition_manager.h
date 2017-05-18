/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jul 22, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: problem_definition_manager.h
 */

#ifndef HBAC98EE0_A62D_475E_AA14_2DF4C05EA027
#define HBAC98EE0_A62D_475E_AA14_2DF4C05EA027

#include <ais_definitions/class.h>
#include <iosfwd>

namespace prm_planner
{

FORWARD_DECLARE(ApproachingProblemDefinition);
FORWARD_DECLARE(ProblemDefinition);
FORWARD_DECLARE(PRMPlanner);
FORWARD_DECLARE(Robot);
FORWARD_DECLARE(Constraint);

class ProblemDefinitionManager
{
SINGLETON_HEADER(ProblemDefinitionManager)

public:
	typedef boost::shared_ptr<ApproachingProblemDefinition> SingleArmPD;

public:
	virtual ~ProblemDefinitionManager();

	/**
	 * Initializes the problem definitions.
	 * It needs to be called once to initialize the problem definitions
	 *
	 * @type [in]: the name of the problem
	 * @planner [in]: the planner interface
	 */
	void init(const std::string& type,
			const boost::shared_ptr<PRMPlanner> planner);

	/**
	 * Returns the problem definition
	 */
	boost::shared_ptr<ProblemDefinition> getProblemDefinition();

	/**
	 * Returns the contstraint currently used in the problem
	 * definition
	 */
	const boost::shared_ptr<Constraint>& getConstraint() const;

	/**
	 * Returns the robot instance, which actually communicates
	 * with the robot. You should generate copies of it, if
	 * you want to use the robot instance in e.g., planning.
	 */
	const boost::shared_ptr<Robot>& getRobot() const;

private:
	boost::shared_ptr<ProblemDefinition> m_problemDefinition;
	SingleArmPD m_singleArmProblemDefinition;
	boost::shared_ptr<Robot> m_robot;
	boost::shared_ptr<Constraint> m_constraint;
	boost::shared_ptr<PRMPlanner> m_planner;
};

} /* namespace prm_planner */

#endif /* HBAC98EE0_A62D_475E_AA14_2DF4C05EA027 */
