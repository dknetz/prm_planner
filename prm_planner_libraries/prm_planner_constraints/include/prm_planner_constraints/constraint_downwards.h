/*
 * constraint_downwards.h
 *
 *  Created on: Mar 1, 2016
 *      Author: kuhnerd
 */

#ifndef ELASTIC_ROADMAP_TEST_ELASTIC_ROADMAP_PLANNER_INCLUDE_ELASTIC_ROADMAP_PLANNER_CONSTRAINTS_CONSTRAINT_DOWNWARDS_H_
#define ELASTIC_ROADMAP_TEST_ELASTIC_ROADMAP_PLANNER_INCLUDE_ELASTIC_ROADMAP_PLANNER_CONSTRAINTS_CONSTRAINT_DOWNWARDS_H_

#include <prm_planner_constraints/constraint.h>

namespace prm_planner
{

class ConstraintDownwards: public Constraint
{
public:
	ConstraintDownwards();
	virtual ~ConstraintDownwards();

	virtual void computeError(const Trajectory::Pose& goal,
			const Trajectory::Pose& start,
			Vector6d& result);
	virtual bool isGoalReached(const Trajectory::Pose& goal,
			const Trajectory::Pose& start,
			const double thresholdGoalReachedPos,
			const double thresholdGoalReachedAng) const;

	virtual bool isValidOrientation(const Eigen::Affine3d& pose);
	virtual void findValidOrientation(Eigen::Affine3d& pose);
	virtual void findNearestValidOrientation(Eigen::Affine3d& pose);
};

} /* namespace prm_planner */

#endif /* ELASTIC_ROADMAP_TEST_ELASTIC_ROADMAP_PLANNER_INCLUDE_ELASTIC_ROADMAP_PLANNER_CONSTRAINTS_CONSTRAINT_DOWNWARDS_H_ */
