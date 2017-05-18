/*
 * constraint_downwards.cpp
 *
 *  Created on: Mar 1, 2016
 *      Author: kuhnerd
 */

#include <ais_log/log.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl/frames.hpp>
#include <prm_planner_constraints/constraint_downwards.h>

namespace prm_planner
{

ConstraintDownwards::ConstraintDownwards()
{
}

ConstraintDownwards::~ConstraintDownwards()
{
}

void ConstraintDownwards::computeError(const Trajectory::Pose& goal,
		const Trajectory::Pose& start,
		Vector6d& result)
{
	getLinearDiff(goal, start, result);
	getAngularDiff(goal, start, result);

//	KDL::Rotation r1 = KDL::Rotation::RPY(goal(3, 0), goal(4, 0), goal(5, 0));
//	KDL::Rotation r2 = KDL::Rotation::RPY(M_PI, 0, start(5, 0));
//
//	KDL::Vector angError = (r1.UnitX() * r2.UnitX() +
//			r1.UnitY() * r2.UnitY() +
//			r1.UnitZ() * r2.UnitZ());
//
//	result(3, 0) = angError.x();
//	result(4, 0) = angError.y();
//	result(5, 0) = angError.z();
}

bool ConstraintDownwards::isGoalReached(const Trajectory::Pose& goal,
		const Trajectory::Pose& start,
		const double thresholdGoalReachedPos,
		const double thresholdGoalReachedAng) const
{
	Vector3d error;
	getLinearDiff(goal, start, error);

//	Eigen::Quaterniond diff = goal.quaternion * start.quaternion.inverse();
//	KDL::Rotation rpy = KDL::Rotation::Quaternion(diff.x(), diff.y(), diff.z(), diff.w());
//	double r, p, y;
//	rpy.GetRPY(r, p, y);
//
//	KDL::Vector angError(r, p, y);

	double angularError = start.quaternion.angularDistance(goal.quaternion);
//	LOG_INFO(error.norm() << " " << angError.Norm() << " " << angularError);
	return error.norm() < thresholdGoalReachedPos && angularError < thresholdGoalReachedAng;
}

bool ConstraintDownwards::isValidOrientation(const Eigen::Affine3d& pose)
{
	static const double eps = 0.01;
	KDL::Frame frame;
	tf::transformEigenToKDL(pose, frame);
	double alpha, beta, gamma;
	frame.M.GetEulerZYX(alpha, beta, gamma);
	return (beta > -eps && beta < eps && fabs(gamma) > M_PI - eps && fabs(gamma) < M_PI + eps);
}

void ConstraintDownwards::findValidOrientation(Eigen::Affine3d& pose)
{
	static std::default_random_engine re;
	static std::uniform_real_distribution<double> unif(-M_PI, M_PI);
	KDL::Frame frame;
	frame.p.x(pose(0, 3));
	frame.p.y(pose(1, 3));
	frame.p.z(pose(2, 3));
	frame.M = KDL::Rotation::EulerZYX(unif(re), 0, M_PI);
	tf::transformKDLToEigen(frame, pose);
}

void ConstraintDownwards::findNearestValidOrientation(Eigen::Affine3d& pose)
{
	Eigen::Matrix3d rotation = pose.rotation();
	Eigen::Vector3d rotX, rotY, rotZ, unitZ;
	rotY = rotation.col(1);
	unitZ = -Eigen::Vector3d::UnitZ();

	rotX = rotY.cross(unitZ); //compute x
	rotY = unitZ.cross(rotX); //adjust y

	rotation.col(0) = rotX.normalized();
	rotation.col(1) = rotY.normalized();
	rotation.col(2) = unitZ;

	pose.linear() = rotation;
}

} /* namespace prm_planner */
