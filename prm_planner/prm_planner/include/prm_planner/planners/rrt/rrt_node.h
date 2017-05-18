/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jul 4, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: rrt_node.h
 */

#ifndef H87E7DDAB_5BA8_42A9_A0B5_C286415158CA
#define H87E7DDAB_5BA8_42A9_A0B5_C286415158CA

#include <kdl/jntarray.hpp>
#include <Eigen/Geometry>

namespace prm_planner
{

class SimulationRobotController;

class RRTNode
{
public:
	RRTNode(const Eigen::Affine3d& taskPose);
	RRTNode(const KDL::JntArray& joints,
			const Eigen::Affine3d& taskPose);
	virtual ~RRTNode();

	RRTNode* m_parent;
	Eigen::Affine3d m_pose;
	KDL::JntArray m_joints;
	SimulationRobotController* m_controller; //controller which moves the arm to this node
	bool m_controllerSuccess;
};

} /* namespace prm_planner */

#endif /* H87E7DDAB_5BA8_42A9_A0B5_C286415158CA */
