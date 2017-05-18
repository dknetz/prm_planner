/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Feb 10, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: constraint.cpp
 */

#include <prm_planner_constraints/constraint.h>
#include <Eigen/Geometry>
#include <random>

namespace prm_planner
{

Constraint::Constraint()
{
}

Constraint::~Constraint()
{
}

void Constraint::computeDifference(const Trajectory::Pose& goal,
		const Trajectory::Pose& start,
		Vector6d& result)
{
	computeError(goal, start, result);
}

bool Constraint::findValidPose(Eigen::Affine3d& pose)
{
	findValidPosition(pose);
	findValidOrientation(pose);
	return true;
}

bool Constraint::isValidPose(const Eigen::Affine3d& pose)
{
	return isValidPosition(pose) && isValidOrientation(pose);
}

void Constraint::samplePose(Eigen::Affine3d& pose)
{
	static std::random_device r; //seed
	static std::default_random_engine generator(r());
	static std::uniform_real_distribution<double> distributionAngle(-M_PI, M_PI);
	static std::uniform_real_distribution<double> distributionPos(-100.0, 100.0);

	Eigen::Matrix3d m;
	m = Eigen::AngleAxisd(distributionAngle(generator), Eigen::Vector3d::UnitZ())
			* Eigen::AngleAxisd(distributionAngle(generator), Eigen::Vector3d::UnitY())
			* Eigen::AngleAxisd(distributionAngle(generator), Eigen::Vector3d::UnitZ());

	Eigen::Vector3d pos(distributionPos(generator), distributionPos(generator), distributionPos(generator));

	pose.linear() = m;
	pose.translation() = pos;

	//make it a valid pose
	findValidPose(pose);
}

bool Constraint::isValidOrientation(const Eigen::Affine3d& pose)
{
	return true;
}

bool Constraint::isValidPosition(const Eigen::Affine3d& pose)
{
	return true;
}

void Constraint::findValidPosition(Eigen::Affine3d& pose)
{
}

void Constraint::findValidOrientation(Eigen::Affine3d& pose)
{
}

void Constraint::findNearestValidPose(Eigen::Affine3d& pose)
{
	findNearestValidPosition(pose);
	findNearestValidOrientation(pose);
}

void Constraint::findNearestValidPosition(Eigen::Affine3d& pose)
{
}

void Constraint::findNearestValidOrientation(Eigen::Affine3d& pose)
{
}

} /* namespace prm_planner */

