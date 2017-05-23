/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Oct 10, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: iiwa_v_rep_interface.h
 */

#ifndef H20A551E2_AB04_46AA_8DD3_4590AD2as5gs
#define H20A551E2_AB04_46AA_8DD3_4590AD2as5gs

#include <prm_planner_robot/robot_interface.h>

namespace kuka_iiwa_simple
{

class IiwaSimpleInterface: public prm_planner::RobotInterface
{
public:
	IiwaSimpleInterface();
	virtual ~IiwaSimpleInterface();

	virtual bool start();
	virtual bool read(const ros::Time time = ros::Time(0),
			const ros::Duration period = ros::Duration(0));
	virtual bool write(const ros::Time time = ros::Time(0),
			const ros::Duration period = ros::Duration(0));
	virtual bool stop();

private:
	ros::Publisher m_jointPub;
};

} /* namespace kuka_iiwa_simple */

#endif /* H20A551E2_AB04_46AA_8DD3_4590AD2as5gs */
