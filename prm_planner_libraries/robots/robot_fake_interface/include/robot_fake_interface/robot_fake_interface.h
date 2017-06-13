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
#include <sensor_msgs/JointState.h>

namespace robot_fake_interface
{

class RobotFakeInterface: public prm_planner::RobotInterface
{
public:
	RobotFakeInterface();
	virtual ~RobotFakeInterface();

	virtual bool start();
	virtual bool read(const ros::Time time = ros::Time(0),
			const ros::Duration period = ros::Duration(0));
	virtual bool write(const ros::Time time = ros::Time(0),
			const ros::Duration period = ros::Duration(0));
	virtual bool stop();

private:
	virtual void receiveJointState(const sensor_msgs::JointStateConstPtr& jointState);

private:
	std::vector<std::string> m_chainNames;
	ros::Publisher m_jointPub;
	ros::Subscriber m_jointSub;
};

} /* namespace robot_fake_interface */

#endif /* H20A551E2_AB04_46AA_8DD3_4590AD2as5gs */
