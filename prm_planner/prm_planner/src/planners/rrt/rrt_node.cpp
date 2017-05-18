/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jul 4, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: rrt_node.cpp
 */

#include <ais_definitions/class.h>
#include <prm_planner/controllers/simulation_robot_controller.h>
#include <prm_planner/planners/rrt/rrt_node.h>

namespace prm_planner
{

RRTNode::RRTNode(const Eigen::Affine3d& taskPose) :
				m_pose(taskPose),
				m_parent(NULL),
				m_controller(NULL),
				m_controllerSuccess(false)
{
}

RRTNode::RRTNode(const KDL::JntArray& joints,
		const Eigen::Affine3d& taskPose) :
				m_joints(joints),
				m_pose(taskPose),
				m_parent(NULL),
				m_controller(NULL),
				m_controllerSuccess(false)
{
}

RRTNode::~RRTNode()
{
	DELETE_VAR(m_controller);
}

} /* namespace prm_planner */

