/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Feb 19, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: constraint_horizontal.h
 */

#ifndef CONSTRAINT_HORIZONTAL_H_
#define CONSTRAINT_HORIZONTAL_H_

#include <prm_planner_constraints/constraint.h>

namespace prm_planner
{

class ConstraintHorizontal: public Constraint
{
public:
	ConstraintHorizontal();
	virtual ~ConstraintHorizontal();

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

#endif /* CONSTRAINT_HORIZONTAL_H_ */
