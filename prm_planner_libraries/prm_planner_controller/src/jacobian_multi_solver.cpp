/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Feb 26, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: jacobian_multi_solver.cpp
 */

#include <prm_planner_controller/jacobian_multi_solver.h>

namespace prm_planner
{

JacobianMultiSolver::JacobianMultiSolver(const KDL::Chain& _chain) :
				chain(_chain),
				locked_joints_(chain.getNrOfJoints(), false),
				nr_of_unlocked_joints_(chain.getNrOfJoints())
{
}

JacobianMultiSolver::~JacobianMultiSolver()
{
}

int JacobianMultiSolver::setLockedJoints(const std::vector<bool> locked_joints)
{
	if (locked_joints.size() != locked_joints_.size())
		return -1;
	locked_joints_ = locked_joints;
	nr_of_unlocked_joints_ = 0;
	for (unsigned int i = 0; i < locked_joints_.size(); i++)
	{
		if (!locked_joints_[i])
			nr_of_unlocked_joints_++;
	}

	return 0;
}

const char* JacobianMultiSolver::strError(const int error) const
		{
	if (E_JAC_FAILED == error)
		return "Jac Failed";
	else
		return "No error";
}

void JacobianMultiSolver::setChain(const KDL::Chain& _chain)
{
	chain = _chain;
	locked_joints_.resize(chain.getNrOfJoints());
	std::fill(locked_joints_.begin(), locked_joints_.end(), false);
	nr_of_unlocked_joints_ = chain.getNrOfJoints();
}

} /* namespace prm_planner */

