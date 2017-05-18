/*
 * constraint_vertical.cpp
 *
 *  Created on: Aug 12, 2016
 *      Author: kuhnerd
 */

#include <eigen_conversions/eigen_kdl.h>
#include <prm_planner_constraints/constraint_vertical.h>
#include <ais_log/log.h>

namespace prm_planner
{

ConstraintVertical::ConstraintVertical()
{
}

ConstraintVertical::~ConstraintVertical()
{
}

void ConstraintVertical::computeError(const Trajectory::Pose& goal,
		const Trajectory::Pose& start,
		Vector6d& result)
{
	getLinearDiff(goal, start, result);
	getAngularDiff(goal, start, result);
}

bool ConstraintVertical::isGoalReached(const Trajectory::Pose& goal,
		const Trajectory::Pose& start,
		const double thresholdGoalReachedPos,
		const double thresholdGoalReachedAng) const
{
	Vector3d error;
	getLinearDiff(goal, start, error);
	double angularError = start.quaternion.angularDistance(goal.quaternion);
	return error.norm() < thresholdGoalReachedPos && angularError < thresholdGoalReachedAng;
}

bool ConstraintVertical::isValidOrientation(const Eigen::Affine3d& pose)
{
	static const double eps = 0.01;
	static const Eigen::Vector3d xExpected(0, 0, -1);
	Eigen::Vector3d xAxis = pose.linear().col(0);
	return (fabs(xAxis.dot(xExpected) - 1.0) < eps);
}

void ConstraintVertical::findValidOrientation(Eigen::Affine3d& pose)
{
	static std::default_random_engine re;
	static std::uniform_real_distribution<double> unif(-M_PI, M_PI);

	Eigen::Matrix3d rot;
	rot.col(0) = Eigen::Vector3d(0, 0, -1);
	rot.col(1) = Eigen::Vector3d(0, 1, 0);
	rot.col(2) = Eigen::Vector3d(1, 0, 0);
	rot = Eigen::AngleAxisd(unif(re), Eigen::Vector3d::UnitZ()) * rot;

	pose.linear() = rot;
}

void ConstraintVertical::findNearestValidOrientation(Eigen::Affine3d& pose)
{
	Eigen::Vector3d x(0, 0, -1);
	Eigen::Matrix3d rotation = pose.rotation();
	Eigen::Vector3d rotY, rotZ;
	rotY = rotation.col(1);
	rotZ = x.cross(rotY);
	rotY = rotZ.cross(x);

	rotation.col(0) = x;
	rotation.col(1) = rotY.normalized();
	rotation.col(2) = rotZ.normalized();

	pose.linear() = rotation;
}

} /* namespace prm_planner */
