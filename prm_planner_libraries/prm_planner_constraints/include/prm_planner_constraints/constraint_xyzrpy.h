/*
 * constraint_xyzrpy.h
 *
 *  Created on: Feb 10, 2016
 *      Author: kuhnerd
 */

#ifndef ELASTIC_ROADMAP_TEST_ELASTIC_ROADMAP_PLANNER_INCLUDE_ELASTIC_ROADMAP_PLANNER_CONSTRAINTS_CONSTRAINT_XYZRPY_H_
#define ELASTIC_ROADMAP_TEST_ELASTIC_ROADMAP_PLANNER_INCLUDE_ELASTIC_ROADMAP_PLANNER_CONSTRAINTS_CONSTRAINT_XYZRPY_H_

#include <prm_planner_constraints/constraint.h>

namespace prm_planner
{

class ConstraintXYZRPY: public Constraint
{
public:
	ConstraintXYZRPY();
	virtual ~ConstraintXYZRPY();

	virtual void computeError(const Trajectory::Pose& goal,
			const Trajectory::Pose& start,
			Vector6d& result);
	virtual bool isGoalReached(const Trajectory::Pose& goal,
			const Trajectory::Pose& start,
			const double thresholdGoalReachedPos,
			const double thresholdGoalReachedAng) const;
};

} /* namespace prm_planner */

#endif /* ELASTIC_ROADMAP_TEST_ELASTIC_ROADMAP_PLANNER_INCLUDE_ELASTIC_ROADMAP_PLANNER_CONSTRAINTS_CONSTRAINT_XYZRPY_H_ */
