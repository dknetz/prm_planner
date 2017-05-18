/*
 * ConstraintUpright.cpp
 *
 *  Created on: Feb 21, 2016
 *      Author: kuhnerd
 */

#include <ais_log/log.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl/frames.hpp>
#include <prm_planner_constraints/constraint_upright.h>

namespace prm_planner
{

ConstraintUpright::ConstraintUpright()
{
}

ConstraintUpright::~ConstraintUpright()
{
}

void ConstraintUpright::computeError(const Trajectory::Pose& goal,
		const Trajectory::Pose& start,
		Vector6d& result)
{
	getLinearDiff(goal, start, result);
	getAngularDiff(goal, start, result);
}

bool ConstraintUpright::isGoalReached(const Trajectory::Pose& goal,
		const Trajectory::Pose& start,
		const double thresholdGoalReachedPos,
		const double thresholdGoalReachedAng) const
{
	Eigen::Vector3d error;
	getLinearDiff(goal, start, error);
	double angularError = start.quaternion.angularDistance(goal.quaternion);
	return error.norm() < thresholdGoalReachedPos && angularError < thresholdGoalReachedAng;
}

bool ConstraintUpright::isValidOrientation(const Eigen::Affine3d& pose)
{
	static const double eps = 0.01;
	KDL::Frame frame;
	tf::transformEigenToKDL(pose, frame);
	double alpha, beta, gamma;
	frame.M.GetEulerZYX(alpha, beta, gamma);
	return (beta > -eps && beta < eps && gamma > -eps && gamma < eps);
}

void ConstraintUpright::findValidOrientation(Eigen::Affine3d& pose)
{
	static std::default_random_engine re;
	static std::uniform_real_distribution<double> unif(-M_PI, M_PI);
	KDL::Frame frame;
	frame.p.x(pose(0, 3));
	frame.p.y(pose(1, 3));
	frame.p.z(pose(2, 3));
	frame.M = KDL::Rotation::EulerZYX(unif(re), 0, 0);
	tf::transformKDLToEigen(frame, pose);
}

} /* namespace prm_planner */
