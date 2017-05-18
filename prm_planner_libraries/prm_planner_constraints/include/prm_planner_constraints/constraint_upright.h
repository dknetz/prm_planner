/*
 * ConstraintUpright.h
 *
 *  Created on: Feb 21, 2016
 *      Author: kuhnerd
 */

#ifndef TEST_ELASTIC_ROADMAPS_ELASTIC_ROADMAP_PLANNER_SRC_CONSTRAINTS_CONSTRAINTUPRIGHT_H_
#define TEST_ELASTIC_ROADMAPS_ELASTIC_ROADMAP_PLANNER_SRC_CONSTRAINTS_CONSTRAINTUPRIGHT_H_


#include <prm_planner_constraints/constraint.h>

namespace prm_planner
{

class ConstraintUpright: public Constraint
{
public:
	ConstraintUpright();
	virtual ~ConstraintUpright();

	virtual void computeError(const Trajectory::Pose& goal,
			const Trajectory::Pose& start,
			Vector6d& result);
	virtual bool isGoalReached(const Trajectory::Pose& goal,
			const Trajectory::Pose& start,
			const double thresholdGoalReachedPos,
			const double thresholdGoalReachedAng) const;

	virtual bool isValidOrientation(const Eigen::Affine3d& pose);
	virtual void findValidOrientation(Eigen::Affine3d& pose);
};

} /* namespace prm_planner */
#endif /* TEST_ELASTIC_ROADMAPS_ELASTIC_ROADMAP_PLANNER_SRC_CONSTRAINTS_CONSTRAINTUPRIGHT_H_ */
