/*
 * constraint_z_up.cpp
 *
 *  Created on: Aug 12, 2016
 *      Author: kuhnerd
 */

#include <eigen_conversions/eigen_kdl.h>
#include <prm_planner_constraints/constraint_z_up.h>
#include <ais_log/log.h>

namespace prm_planner
{

ConstraintZUp::ConstraintZUp()
{
}

ConstraintZUp::~ConstraintZUp()
{
}

void ConstraintZUp::computeError(const Trajectory::Pose& goal,
		const Trajectory::Pose& start,
		Vector6d& result)
{
	getLinearDiff(goal, start, result);
	getAngularDiff(goal, start, result);
}

bool ConstraintZUp::isGoalReached(const Trajectory::Pose& goal,
		const Trajectory::Pose& start,
		const double thresholdGoalReachedPos,
		const double thresholdGoalReachedAng) const
{
	Vector3d error;
	getLinearDiff(goal, start, error);
	double angularError = start.quaternion.angularDistance(goal.quaternion);
	return error.norm() < thresholdGoalReachedPos && angularError < thresholdGoalReachedAng;
}

bool ConstraintZUp::isValidOrientation(const Eigen::Affine3d& pose)
{
	static const double eps = 0.01;
	static const Eigen::Vector3d zExpected(0, 0, 1);
	Eigen::Vector3d zAxis = pose.linear().col(2);
	return (fabs(zAxis.dot(zExpected) - 1.0) < eps);
}

void ConstraintZUp::findValidOrientation(Eigen::Affine3d& pose)
{
	static std::default_random_engine re;
	static std::uniform_real_distribution<double> unif(-M_PI, M_PI);

	Eigen::Matrix3d rot;
	rot.col(0) = Eigen::Vector3d(1, 0, 0);
	rot.col(1) = Eigen::Vector3d(0, 1, 0);
	rot.col(2) = Eigen::Vector3d(0, 0, 1);
	rot = Eigen::AngleAxisd(unif(re), Eigen::Vector3d::UnitZ()) * rot;

	pose.linear() = rot;
}

void ConstraintZUp::findNearestValidOrientation(Eigen::Affine3d& pose)
{
	Eigen::Vector3d z(0, 0, 1);
	Eigen::Matrix3d rotation = pose.rotation();
	Eigen::Vector3d rotY, rotX;
	rotY = rotation.col(1);
	rotX = rotY.cross(z);
	rotY = z.cross(rotX);

	rotation.col(0) = rotX.normalized();
	rotation.col(1) = rotY.normalized();
	rotation.col(2) = z;

	pose.linear() = rotation;
}

} /* namespace prm_planner */
