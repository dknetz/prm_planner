/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Oct 10, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: omni_rob_robot_interface.h
 */

#ifndef H4C095A54_6CE3_4B49_B423_6676D56DFB9C
#define H4C095A54_6CE3_4B49_B423_6676D56DFB9C

#include <prm_planner_robot/robot_interface.h>

namespace kuka_omnirob_robot
{

class OmniRobRobotInterface: public prm_planner::RobotInterface
{
public:
	OmniRobRobotInterface();
	virtual ~OmniRobRobotInterface();

	virtual bool start();
	virtual bool read(const ros::Time time = ros::Time(0),
			const ros::Duration period = ros::Duration(0));
	virtual bool write(const ros::Time time = ros::Time(0),
			const ros::Duration period = ros::Duration(0));
	virtual bool stop();
};

} /* namespace kuka_omnirob_robot */

#endif /* H4C095A54_6CE3_4B49_B423_6676D56DFB9C */
