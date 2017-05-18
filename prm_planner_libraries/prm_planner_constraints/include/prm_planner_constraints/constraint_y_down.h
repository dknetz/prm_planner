/*
 * constraint_y_down.h
 *
 *  Created on: Apr 26, 2017
 *      Author: kuhnerd
 */

#ifndef PRM_PLANNER_LIBRARIES_PRM_PLANNER_CONSTRAINTS_INCLUDE_PRM_PLANNER_CONSTRAINTS_CONSTRAINT_Y_DOWN_H_
#define PRM_PLANNER_LIBRARIES_PRM_PLANNER_CONSTRAINTS_INCLUDE_PRM_PLANNER_CONSTRAINTS_CONSTRAINT_Y_DOWN_H_

#include <prm_planner_constraints/constraint.h>

namespace prm_planner
{

class ConstraintYDown: public Constraint
{
public:
	ConstraintYDown();
	virtual ~ConstraintYDown();

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

#endif /* PRM_PLANNER_LIBRARIES_PRM_PLANNER_CONSTRAINTS_INCLUDE_PRM_PLANNER_CONSTRAINTS_CONSTRAINT_Y_DOWN_H_ */
