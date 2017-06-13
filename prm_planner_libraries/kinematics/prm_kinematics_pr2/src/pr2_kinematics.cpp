/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Oct 10, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: iiwa_robot_interface.cpp
 */

#include <pluginlib/class_list_macros.h>
#include <prm_planner/robot/robot.h>
#include <prm_kinematics_pr2/pr2_kinematics.h>

#include <eigen_conversions/eigen_kdl.h>
namespace prm_kinematics_pr2
{

PR2Kinematics::PR2Kinematics()
{
}

PR2Kinematics::~PR2Kinematics()
{
}

void PR2Kinematics::init(const prm_planner::RobotArm* robot)
{
	KDL::Chain chain = robot->getChain();
	m_fkSolver.reset(new KDL::ChainFkSolverPos_recursive(chain));
	m_ikSolver.reset(new pr2_arm_kinematics::PR2ArmIKSolver(robot->getUrdf(), robot->getRootLink(), robot->getTipLink(), 0.01, 0));
	m_jacobianSolver.reset(new KDL::ChainJntToJacSolver(chain));
}

boost::shared_ptr<prm_planner::Kinematics> PR2Kinematics::getCopy(const prm_planner::RobotArm* robot)
{
	boost::shared_ptr<Kinematics> k(new PR2Kinematics);
	k->init(robot);
	return k;
}

bool PR2Kinematics::getJacobian(const KDL::JntArray& jointPosition,
		KDL::Jacobian& jacobian,
		int segmentNR)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_jacobianSolver->JntToJac(jointPosition, jacobian, segmentNR) >= 0;
}

bool PR2Kinematics::getFK(const KDL::JntArray& jointPosition,
		KDL::Frame& pose,
		int segmentNR)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_fkSolver->JntToCart(jointPosition, pose, segmentNR) >= 0;
}

bool PR2Kinematics::getIK(const Eigen::Affine3d& pose,
		const KDL::JntArray& init,
		KDL::JntArray& jointPosition)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	KDL::Frame x;
	tf::transformEigenToKDL(pose, x);
	return m_ikSolver->CartToJntSearch(init, x, jointPosition, 1.0, M_PI) >= 0;
}

} /* namespace prm_kinematics_kdl */

PLUGINLIB_EXPORT_CLASS(prm_kinematics_pr2::PR2Kinematics, prm_planner::Kinematics)

