/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Oct 10, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: iiwa_robot_interface.h
 */

#ifndef HF0FDEC6B_B2B2_45DA_A8B0_494EFAF39215
#define HF0FDEC6B_B2B2_45DA_A8B0_494EFAF39215
#include <boost/thread/pthread/recursive_mutex.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <pr2_arm_kinematics/pr2_arm_ik_solver.h>
#include <prm_planner_robot/kinematics.h>

namespace prm_kinematics_pr2
{

class PR2Kinematics: public prm_planner::Kinematics
{
public:
	PR2Kinematics();
	virtual ~PR2Kinematics();

	virtual void init(const prm_planner::RobotArm* robot);

	virtual boost::shared_ptr<prm_planner::Kinematics> getCopy(const prm_planner::RobotArm* robot);

	virtual bool getJacobian(const KDL::JntArray& jointPosition,
			KDL::Jacobian& jacobian,
			int segmentNR = -1);

	virtual bool getFK(const KDL::JntArray& jointPosition,
			KDL::Frame& pose,
			int segmentNR = -1);

	virtual bool getIK(const Eigen::Affine3d& pose,
			const KDL::JntArray& init,
			KDL::JntArray& jointPosition);

private:
	mutable boost::recursive_mutex m_mutex;

	boost::shared_ptr<pr2_arm_kinematics::PR2ArmIKSolver> m_ikSolver;
	boost::shared_ptr<KDL::ChainFkSolverPos_recursive> m_fkSolver;
	boost::shared_ptr<KDL::ChainJntToJacSolver> m_jacobianSolver;
};

} /* namespace prm_kinematics_kdl */

#endif /* HF0FDEC6B_B2B2_45DA_A8B0_494EFAF39215 */
