/*
 * constraint_factory.cpp
 *
 *  Created on: Feb 21, 2016
 *      Author: kuhnerd
 */

#include <ais_log/log.h>
#include <prm_planner_constraints/constraint_factory.h>
#include <prm_planner_constraints/constraints.h>

namespace prm_planner
{

ConstraintFactory::ConstraintFactory()
{
}

ConstraintFactory::~ConstraintFactory()
{
}

boost::shared_ptr<Constraint> ConstraintFactory::create(const std::string& name)
{
	if (name == "horizontal")
	{
		return boost::shared_ptr<Constraint>(new ConstraintHorizontal);
	}
	else if (name == "vertical")
	{
		return boost::shared_ptr<Constraint>(new ConstraintVertical);
	}
	else if (name == "upright")
	{
		return boost::shared_ptr<Constraint>(new ConstraintUpright);
	}
	else if (name == "downwards")
	{
		return boost::shared_ptr<Constraint>(new ConstraintDownwards);
	}
	else if (name == "z_up")
	{
		return boost::shared_ptr<Constraint>(new ConstraintZUp);
	}
	else if (name == "y_up")
	{
		return boost::shared_ptr<Constraint>(new ConstraintYUp);
	}
	else if (name == "y_down")
	{
		return boost::shared_ptr<Constraint>(new ConstraintYDown);
	}
	else if (name == "xyzrpy")
	{
		return boost::shared_ptr<Constraint>(new ConstraintXYZRPY);
	}
	else
	{
		LOG_FATAL("Unknown constraint type: " << name);
		exit(-1);
	}
}

} /* namespace prm_planner */
