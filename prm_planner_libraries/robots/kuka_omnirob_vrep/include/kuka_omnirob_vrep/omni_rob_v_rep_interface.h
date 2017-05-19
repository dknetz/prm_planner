/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Oct 10, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: omni_rob_v_rep_interface.h
 */

#ifndef H9F18194C_6817_4EE0_82EA_BBCB68588782
#define H9F18194C_6817_4EE0_82EA_BBCB68588782

#include <prm_planner_robot/robot_interface.h>
#include <ros/ros.h>

#ifdef FOUND_VREP
#include <boost/thread/pthread/mutex.hpp>
#include <nav_msgs/Odometry.h>
#include <vrep_interface/v_rep_interface.h>

#define OMNIROB_LBR_BASE_JOINT_ROT1 "global_rot1_joint"
#define OMNIROB_LBR_BASE_JOINT_LIN "global_lin_joint"
#define OMNIROB_LBR_BASE_JOINT_ROT2 "global_rot2_joint"
#define OMNIROB_LBR_1_JOINT "lbr_1_joint"
#define OMNIROB_LBR_2_JOINT "lbr_2_joint"
#define OMNIROB_LBR_3_JOINT "lbr_3_joint"
#define OMNIROB_LBR_4_JOINT "lbr_4_joint"
#define OMNIROB_LBR_5_JOINT "lbr_5_joint"
#define OMNIROB_LBR_6_JOINT "lbr_6_joint"
#define OMNIROB_LBR_7_JOINT "lbr_7_joint"
#endif

namespace kuka_omnirob_vrep
{

class OmniRobVRepInterface: public prm_planner::RobotInterface
{
public:
	OmniRobVRepInterface();
	virtual ~OmniRobVRepInterface();

	virtual bool start();
	virtual bool read(const ros::Time time = ros::Time(0),
			const ros::Duration period = ros::Duration(0));
	virtual bool write(const ros::Time time = ros::Time(0),
			const ros::Duration period = ros::Duration(0));
	virtual bool stop();

#ifdef FOUND_VREP
private:
	void callbackOdom(nav_msgs::OdometryConstPtr odom);

private:
	struct VrepData
	{
		VrepData() :
						vrepHandle(-1),
						vrepDirection(1.0)
		{
		}

		double vrepDirection; //used to compensate for other frames in vrep
		simxInt vrepHandle;
	};

	typedef std::unordered_map<std::string, VrepData> VrepDataType;

	/**
	 * vrep interface instance. Don't delete it, because
	 * it is managed by a singleton class
	 */
	vrep_interface::VRepInterface* m_vrep;

	VrepDataType m_vrepData;

	bool m_firstRead;

	/**
	 * The handles for the wheels
	 */
	simxInt m_handleLeftFrontWheel;
	simxInt m_handleRightFrontWheel;
	simxInt m_handleLeftRearWheel;
	simxInt m_handleRightRearWheel;

	/**
	 * We receive odometry information via ROS to
	 * be able to also include the platform as a set
	 * of three additional joints
	 */
	boost::mutex m_odomMutex;
	ros::Subscriber m_subOdom;
	bool m_hasOdom;
	nav_msgs::OdometryConstPtr m_odom;
#endif
};

} /* namespace kuka_omnirob_vrep */

#endif /* H9F18194C_6817_4EE0_82EA_BBCB68588782 */
