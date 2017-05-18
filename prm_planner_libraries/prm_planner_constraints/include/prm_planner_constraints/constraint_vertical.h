/*
 * constraint_vertical.h
 *
 *  Created on: Aug 12, 2016
 *      Author: kuhnerd
 */

#ifndef PRM_PLANNER_PRM_PLANNER_INCLUDE_PRM_PLANNER_CONSTRAINTS_CONSTRAINT_VERTICAL_H_
#define PRM_PLANNER_PRM_PLANNER_INCLUDE_PRM_PLANNER_CONSTRAINTS_CONSTRAINT_VERTICAL_H_

#include <prm_planner_constraints/constraint.h>

namespace prm_planner
{

/**
 * Constraint is used to hold, e.g., a tray
 */
class ConstraintVertical: public Constraint
{
public:
	ConstraintVertical();
	virtual ~ConstraintVertical();

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

#endif /* PRM_PLANNER_PRM_PLANNER_INCLUDE_PRM_PLANNER_CONSTRAINTS_CONSTRAINT_VERTICAL_H_ */
