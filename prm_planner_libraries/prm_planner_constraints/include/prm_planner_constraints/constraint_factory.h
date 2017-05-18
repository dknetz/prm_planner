/*
 * constraint_factory.h
 *
 *  Created on: Feb 21, 2016
 *      Author: kuhnerd
 */

#ifndef TEST_ELASTIC_ROADMAPS_ELASTIC_ROADMAP_PLANNER_INCLUDE_ELASTIC_ROADMAP_PLANNER_CONSTRAINTS_CONSTRAINT_FACTORY_H_
#define TEST_ELASTIC_ROADMAPS_ELASTIC_ROADMAP_PLANNER_INCLUDE_ELASTIC_ROADMAP_PLANNER_CONSTRAINTS_CONSTRAINT_FACTORY_H_
#include <boost/shared_ptr.hpp>

namespace prm_planner
{

class Constraint;

class ConstraintFactory
{
public:
	static boost::shared_ptr<Constraint> create(const std::string& name);

private:
	ConstraintFactory();
	virtual ~ConstraintFactory();
};

} /* namespace prm_planner */

#endif /* TEST_ELASTIC_ROADMAPS_ELASTIC_ROADMAP_PLANNER_INCLUDE_ELASTIC_ROADMAP_PLANNER_CONSTRAINTS_CONSTRAINT_FACTORY_H_ */
