/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jan 5, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: robot.h
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include <boost/shared_ptr.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <prm_planner_robot/defines.h>
#include <prm_planner_robot/robot_arm.h>
#include <ros/ros.h>
#include <urdf/model.h>

namespace prm_planner
{

/**
 * A robot consists of one or multiple arms.
 * You can activate or deactivate arms to plan
 * in single or multiarm setups.
 */
class Robot: public RobotArm
{
public:
	Robot(const std::string robotName);

	/**
	 * Copy constructor, only generates a flat
	 * copy of the underlying arm interfaces
	 */
//	Robot(const Robot& other);

	virtual ~Robot();

public:
	const std::string c_robotName;
};

} /* namespace prm_planner */

#endif /* ROBOT_H_ */
