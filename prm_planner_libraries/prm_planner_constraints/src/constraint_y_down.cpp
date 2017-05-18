/*
 * constraint_y_down.cpp
 *
 *  Created on: Apr 26, 2017
 *      Author: kuhnerd
 */

#include <prm_planner_constraints/constraint_y_down.h>
#include <eigen_conversions/eigen_kdl.h>
#include <ais_log/log.h>

namespace prm_planner
{

ConstraintYDown::ConstraintYDown()
{
}

ConstraintYDown::~ConstraintYDown()
{
}

void ConstraintYDown::computeError(const Trajectory::Pose& goal,
		const Trajectory::Pose& start,
		Vector6d& result)
{
	getLinearDiff(goal, start, result);
	getAngularDiff(goal, start, result);
}

bool ConstraintYDown::isGoalReached(const Trajectory::Pose& goal,
		const Trajectory::Pose& start,
		const double thresholdGoalReachedPos,
		const double thresholdGoalReachedAng) const
{
	Vector3d error;
	getLinearDiff(goal, start, error);
	double angularError = start.quaternion.angularDistance(goal.quaternion);
	return error.norm() < thresholdGoalReachedPos && angularError < thresholdGoalReachedAng;
}

bool ConstraintYDown::isValidOrientation(const Eigen::Affine3d& pose)
{
	static const double eps = 0.01;
	static const Eigen::Vector3d yExpected(0, -1, 0);
	Eigen::Vector3d yAxis = pose.linear().col(1);
	return (fabs(yAxis.dot(yExpected) - 1.0) < eps);
}

void ConstraintYDown::findValidOrientation(Eigen::Affine3d& pose)
{
	static std::default_random_engine re;
	static std::uniform_real_distribution<double> unif(-M_PI, M_PI);

	Eigen::Matrix3d rot;
	rot.col(0) = Eigen::Vector3d(0, 1, 0);
	rot.col(1) = Eigen::Vector3d(0, 0, -1);
	rot.col(2) = Eigen::Vector3d(-1, 0, 0);
	rot = Eigen::AngleAxisd(unif(re), Eigen::Vector3d::UnitY()) * rot;

	pose.linear() = rot;
}

void ConstraintYDown::findNearestValidOrientation(Eigen::Affine3d& pose)
{
	Eigen::Vector3d y(0, 0, -1);
	Eigen::Matrix3d rotation = pose.rotation();
	Eigen::Vector3d rotZ, rotX;
	rotX = rotation.col(0);
	rotZ = rotX.cross(y);
	rotX = y.cross(rotZ);

	rotation.col(0) = rotX.normalized();
	rotation.col(1) = y;
	rotation.col(2) = rotZ.normalized();

	pose.linear() = rotation;
}

} /* namespace prm_planner */
