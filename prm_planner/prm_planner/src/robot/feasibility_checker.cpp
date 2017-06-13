/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Mar 20, 2017
 *      Author: kuhnerd
 * 	  Filename: feasibility_checker.cpp
 */

#include <ais_log/log.h>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <fcl_wrapper/collision_detection/fcl_wrapper.h>
#include <fcl_wrapper/robot_model/robot_model.h>
#include <fcl_wrapper/robot_model/robot_state.h>
#include <prm_planner/collision_detection/collision_detector.h>
#include <prm_planner/robot/feasibility_checker.h>
#include <prm_planner/robot/robot.h>

namespace prm_planner
{

FeasibilityChecker::FeasibilityChecker(boost::shared_ptr<Robot> robot) :
				m_robot(robot)
{
}

FeasibilityChecker::~FeasibilityChecker()
{
}

bool FeasibilityChecker::check(const Eigen::Affine3d& pose,
		boost::shared_ptr<CollisionDetector> cd,
		bool printCollisions)
{
	KDL::JntArray joints;
	const bool useCD = cd.get() != NULL;

	int counter = 0;
	bool printed = false;
	while (counter++ < 10)
	{
		//get random joint state
		KDL::JntArray randomJointState;
		m_robot->sampleValidChainJointState(randomJointState);

		//check ik and collisions
		if (m_robot->getIKWithInit(randomJointState, pose, joints))
		{
			if (useCD)
			{
				//setup robot for collision check
				if (useCD)
				{
					int i = 0;
					fcl_robot_model::RobotState state;
					for (auto& it : m_robot->getChainJointNames())
					{
						state[it] = joints(i++);
					}
					cd->robot->setRobotState(state);
				}

				if (!cd->fcl->checkCollisions())
				{
					return true;
				}
				else
				{
					if (printCollisions && !printed)
					{
						fcl_collision_detection::FCLWrapper::CollisionsVector collisions;
						cd->fcl->getCollisions(collisions);
						for (auto& it : collisions)
						{
							LOG_WARNING("Collision between: " << it.first << " and " << it.second);
						}

						printed = true;
					}
				}
			}
			else
			{
				return true;
			}
		}
	}

	return false;
}

} /* namespace prm_planner */
