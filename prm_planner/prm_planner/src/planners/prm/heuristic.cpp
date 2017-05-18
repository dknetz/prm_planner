/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jun 21, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: heuristic.cpp
 */

#include <prm_planner/planners/prm/heuristic.h>
#include <Eigen/Core>

namespace prm_planner
{

Heuristic::Heuristic()
{
}

Heuristic::~Heuristic()
{
}

double Heuristic::euclideanDistEndEffector(const PRMNode* n1,
		const PRMNode* n2)
{
	return (n1->getPosition() - n2->getPosition()).norm();
}

double Heuristic::frameDistEndEffector(const PRMNode* n1,
		const PRMNode* n2)
{
	const Eigen::Matrix3d& r1 = n1->getPose().linear();
	const Eigen::Matrix3d& r2 = n2->getPose().linear();
	const Eigen::Vector3d& p1 = n1->getPosition();
	const Eigen::Vector3d& p2 = n2->getPosition();

	double dx = ((r1.col(0) + p1) - (r2.col(0) + p2)).norm();
	double dy = ((r1.col(1) + p1) - (r2.col(1) + p2)).norm();
	double dz = ((r1.col(2) + p1) - (r2.col(2) + p2)).norm();

	return sqrt(dx * dx + dy * dy + dz * dz);
}

} /* namespace prm_planner */
