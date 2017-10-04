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

#include <prm_kinematics_kdl/kdl_kinematics.h>

#include <ais_log/log.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

namespace prm_kinematics_kdl
{

KDLKinematics::KDLKinematics()
{
}

KDLKinematics::~KDLKinematics()
{
}

void KDLKinematics::init(const prm_planner::RobotArm* robot)
{
	m_chain = robot->getChain();
	m_fkSolver.reset(new KDL::ChainFkSolverPos_recursive(m_chain));
	m_ikVelSolver.reset(new KDL::ChainIkSolverVel_pinv(m_chain));
	m_ikSolver.reset(new KDL::ChainIkSolverPos_NR_JL(m_chain, robot->getChainLimitMin(), robot->getChainLimitMax(), *m_fkSolver, *m_ikVelSolver));
	m_jacobianSolver.reset(new KDL::ChainJntToJacSolver(m_chain));
}

boost::shared_ptr<prm_planner::Kinematics> KDLKinematics::getCopy(const prm_planner::RobotArm* robot)
{
	boost::shared_ptr<Kinematics> k(new KDLKinematics);
	k->init(robot);
	return k;
}

bool KDLKinematics::getJacobian(const KDL::JntArray& jointPosition,
		KDL::Jacobian& jacobian,
		int segmentNR)
{
	KDL::ChainJntToJacSolver solver(m_chain); //no mutex required
	return solver.JntToJac(jointPosition, jacobian, segmentNR) >= 0;
}

bool KDLKinematics::getFK(const KDL::JntArray& jointPosition,
		KDL::Frame& pose,
		int segmentNR)
{
	KDL::ChainFkSolverPos_recursive fk(m_chain); //create it here, it shouldn't take too long
	return fk.JntToCart(jointPosition, pose, segmentNR) >= 0;
}

bool KDLKinematics::getIK(const Eigen::Affine3d& pose,
		const KDL::JntArray& init,
		KDL::JntArray& jointPosition)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	KDL::Frame x;
	tf::transformEigenToKDL(pose, x);
	return m_ikSolver->CartToJnt(init, x, jointPosition) >= 0;
}

} /* namespace prm_kinematics_kdl */

PLUGINLIB_EXPORT_CLASS(prm_kinematics_kdl::KDLKinematics, prm_planner::Kinematics)

