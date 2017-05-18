/*
 * constraint_y_up.h
 *
 *  Created on: Aug 12, 2016
 *      Author: kuhnerd
 */

#ifndef HAE9BC8CC_9A18_451A_89BF_EF723407DC475
#define HAE9BC8CC_9A18_451A_89BF_EF723407DC475

#include <prm_planner_constraints/constraint.h>

namespace prm_planner
{

class ConstraintYUp: public Constraint
{
public:
	ConstraintYUp();
	virtual ~ConstraintYUp();

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

#endif /* HAE9BC8CC_9A18_451A_89BF_EF723407DC475 */
