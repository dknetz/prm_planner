/*
 * constraint_xyzrpy.cpp
 *
 *  Created on: Feb 10, 2016
 *      Author: kuhnerd
 */

#include <ais_log/log.h>
#include <prm_planner_constraints/constraint_xyzrpy.h>

namespace prm_planner
{

ConstraintXYZRPY::ConstraintXYZRPY()
{
}

ConstraintXYZRPY::~ConstraintXYZRPY()
{
}

void ConstraintXYZRPY::computeError(const Trajectory::Pose& goal,
		const Trajectory::Pose& start,
		Vector6d& result)
{
	getLinearDiff(goal, start, result);
	getAngularDiff(goal, start, result);
}

bool ConstraintXYZRPY::isGoalReached(const Trajectory::Pose& goal,
		const Trajectory::Pose& start,
		const double thresholdGoalReachedPos,
		const double thresholdGoalReachedAng) const
{
	Eigen::Vector3d error;
	getLinearDiff(goal, start, error);
	double angularError = start.quaternion.angularDistance(goal.quaternion);
//	LOG_INFO(error.norm());
//	LOG_INFO(angularError);
	return error.norm() < thresholdGoalReachedPos && angularError < thresholdGoalReachedAng;
}

} /* namespace prm_planner */
