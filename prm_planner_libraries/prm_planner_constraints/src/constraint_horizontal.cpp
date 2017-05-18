/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Feb 19, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: constraint_horizontal.cpp
 */

#include <ais_log/log.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl/frames.hpp>
#include <prm_planner_constraints/constraint_horizontal.h>

namespace prm_planner
{

ConstraintHorizontal::ConstraintHorizontal()
{
}

ConstraintHorizontal::~ConstraintHorizontal()
{
}

void ConstraintHorizontal::computeError(const Trajectory::Pose& goal,
		const Trajectory::Pose& start,
		Vector6d& result)
{
	getLinearDiff(goal, start, result);
	getAngularDiff(goal, start, result);
////	LOG_INFO(start);
////	LOG_INFO(goal);
//	KDL::Rotation r1 = KDL::Rotation::RPY(start(3, 0), start(4, 0), start(5, 0));
//	KDL::Rotation r2 = KDL::Rotation::RPY(-M_PI_2, 0, goal(5, 0)); //-M_PI_2, 0
//
//	KDL::Vector angError = (r1.UnitX() * r2.UnitX() +
//			r1.UnitY() * r2.UnitY() +
//			r1.UnitZ() * r2.UnitZ());
//
////	LOG_INFO(angError(0) << " " << angError(1) << " " << angError(2));
//
//	result(3, 0) = angError.x();
//	result(4, 0) = angError.y();
//	result(5, 0) = angError.z();
}

bool ConstraintHorizontal::isGoalReached(const Trajectory::Pose& goal,
		const Trajectory::Pose& start,
		const double thresholdGoalReachedPos,
		const double thresholdGoalReachedAng) const
{
	Vector3d error;
	getLinearDiff(goal, start, error);
	double angularError = start.quaternion.angularDistance(goal.quaternion);
	return error.norm() < thresholdGoalReachedPos && angularError < thresholdGoalReachedAng;
}

bool ConstraintHorizontal::isValidOrientation(const Eigen::Affine3d& pose)
{
	static const double eps = 0.01;
	KDL::Frame frame;
	tf::transformEigenToKDL(pose, frame);
	double alpha, beta, gamma;
	frame.M.GetEulerZYX(alpha, beta, gamma);
	return (beta > -eps && beta < eps && gamma > -M_PI_2 - eps && gamma < -M_PI_2 + eps);
}

void ConstraintHorizontal::findValidOrientation(Eigen::Affine3d& pose)
{
	static std::default_random_engine re;
	static std::uniform_real_distribution<double> unif(-M_PI, M_PI);
	KDL::Frame frame;
	frame.p.x(pose(0, 3));
	frame.p.y(pose(1, 3));
	frame.p.z(pose(2, 3));
	frame.M = KDL::Rotation::EulerZYX(unif(re), 0, -M_PI_2);
	tf::transformKDLToEigen(frame, pose);
}

void ConstraintHorizontal::findNearestValidOrientation(Eigen::Affine3d& pose)
{
	Eigen::Matrix3d rotation = pose.rotation();
	Eigen::Vector3d rotX, rotY, rotZ, unitY;
	rotY = rotation.col(1);
	unitY = -Eigen::Vector3d::UnitZ();
	double angle = acos(rotY.normalized().dot(unitY));

	//we assume that the y axis is pointing upwards. If the y axis is pointing downwards, we also flip x
	rotZ = rotation.col(2);	//get z
	rotX = unitY.cross(rotZ); //compute z
	rotZ = rotX.cross(unitY); //adjust x

	rotation.col(0) = rotX.normalized();
	rotation.col(1) = unitY;
	rotation.col(2) = rotZ.normalized();

	pose.linear() = rotation;
}

} /* namespace prm_planner */
