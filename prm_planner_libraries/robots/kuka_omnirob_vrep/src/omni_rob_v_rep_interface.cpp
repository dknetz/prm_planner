/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Oct 10, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: omni_rob_v_rep_interface.cpp
 */

#include <kuka_omnirob_vrep/omni_rob_v_rep_interface.h>

#include <ais_log/log.h>
#include <pluginlib/class_list_macros.h>

#ifdef FOUND_VREP
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#endif

namespace kuka_omnirob_vrep
{

OmniRobVRepInterface::OmniRobVRepInterface()
#ifdef FOUND_VREP
:
				m_vrep(NULL),
				m_firstRead(true),
				m_hasOdom(false),
				m_handleLeftFrontWheel(-1),
				m_handleRightFrontWheel(-1),
				m_handleLeftRearWheel(-1),
				m_handleRightRearWheel(-1)
#endif
{
}

OmniRobVRepInterface::~OmniRobVRepInterface()
{
}

bool OmniRobVRepInterface::start()
{
#ifdef FOUND_VREP
	m_vrep = vrep_interface::VRepInterface::getInstance();
	if (!m_vrep->checkConnection())
		return false;

	//check if there is a odom topic. If so,
	//we can move the platform with three additional
	//joints
	m_odom = ros::topic::waitForMessage<nav_msgs::Odometry>("/vrep/omnirob/odom", ros::Duration(2));
	if (m_odom.get() != NULL)
	{
		m_hasOdom = true;
		ros::NodeHandle n;
		m_subOdom = n.subscribe("/vrep/omnirob/odom", 1, &OmniRobVRepInterface::callbackOdom, this);
		JointData& rot1 = m_data[OMNIROB_LBR_BASE_JOINT_ROT1];
		JointData& lin = m_data[OMNIROB_LBR_BASE_JOINT_LIN];
		JointData& rot2 = m_data[OMNIROB_LBR_BASE_JOINT_ROT2];

		rot1.pos = 0;
		rot1.isRealJoint = false;

		lin.pos = 0;
		lin.isRealJoint = false;

		rot2.pos = 0;
		rot2.isRealJoint = false;

		//get handles of the wheels
		m_handleLeftRearWheel = m_vrep->getHandle("Omnirob_RLwheel_motor");
		m_handleLeftFrontWheel = m_vrep->getHandle("Omnirob_FLwheel_motor");
		m_handleRightRearWheel = m_vrep->getHandle("Omnirob_RRwheel_motor");
		m_handleRightFrontWheel = m_vrep->getHandle("Omnirob_FRwheel_motor");

		//set velocity to 0
		m_vrep->setJointVelocity(m_handleLeftRearWheel, 0);
		m_vrep->setJointVelocity(m_handleLeftFrontWheel, 0);
		m_vrep->setJointVelocity(m_handleRightRearWheel, 0);
		m_vrep->setJointVelocity(m_handleRightFrontWheel, 0);
	}

	//get all handles
	m_vrepData[OMNIROB_LBR_1_JOINT].vrepHandle = m_vrep->getJointHandle(OMNIROB_LBR_1_JOINT);
	m_vrepData[OMNIROB_LBR_2_JOINT].vrepHandle = m_vrep->getJointHandle(OMNIROB_LBR_2_JOINT);
	m_vrepData[OMNIROB_LBR_3_JOINT].vrepHandle = m_vrep->getJointHandle(OMNIROB_LBR_3_JOINT);
	m_vrepData[OMNIROB_LBR_4_JOINT].vrepHandle = m_vrep->getJointHandle(OMNIROB_LBR_4_JOINT);
	m_vrepData[OMNIROB_LBR_5_JOINT].vrepHandle = m_vrep->getJointHandle(OMNIROB_LBR_5_JOINT);
	m_vrepData[OMNIROB_LBR_6_JOINT].vrepHandle = m_vrep->getJointHandle(OMNIROB_LBR_6_JOINT);
	m_vrepData[OMNIROB_LBR_7_JOINT].vrepHandle = m_vrep->getJointHandle(OMNIROB_LBR_7_JOINT);

	//adapt directions because of mis-matching joint directions in vrep
	m_vrepData[OMNIROB_LBR_2_JOINT].vrepDirection = -1.0;
	m_vrepData[OMNIROB_LBR_6_JOINT].vrepDirection = -1.0;

	//set velocities to zero
	for (auto& it : m_vrepData)
	{
		m_vrep->setJointVelocity(it.second.vrepHandle, 0);
	}

	m_vrep->activateSyncMode();

	return true;

#else
	LOG_FATAL("You cannot use this interface because it wasn't compiled with VREP support!");
	exit(-3);
#endif
}

bool OmniRobVRepInterface::read(const ros::Time time,
		const ros::Duration period)
{
#ifdef FOUND_VREP
	if (!m_vrep->checkConnection())
		return false;

	for (auto& it : m_data)
	{
		if (!it.second.isRealJoint)
			continue;

		VrepData& vrepData = m_vrepData[it.first];

		//get joint position
		if (!m_vrep->getJointPosition(vrepData.vrepHandle, it.second.pos))
			return false;

		//compute velocity
		double posDiff = it.second.pos - it.second.lastPos;
		ros::Duration timeDiff = time - it.second.lastTime;

		//the first time we set a velocity of 0 because we don't
		//have no old data to compute the difference
		it.second.vel = m_firstRead ? 0.0 : (posDiff / timeDiff.toSec());
		it.second.lastPos = it.second.pos;
		it.second.lastTime = time;

		//get joint effort
		if (!m_vrep->getJointTorque(vrepData.vrepHandle, it.second.torque))
			return false;
	}

	//convert odom to virtual joints
	if (m_hasOdom)
	{
		Eigen::Affine3d pose;
		Eigen::Matrix<double, 6, 1> twist;

		tf::poseMsgToEigen(m_odom->pose.pose, pose);
		tf::twistMsgToEigen(m_odom->twist.twist, twist);
		tf::Pose tfpose;
		tf::poseMsgToTF(m_odom->pose.pose, tfpose);

		double x = m_odom->pose.pose.position.x;
		double y = m_odom->pose.pose.position.y;
		double theta = tf::getYaw(tfpose.getRotation());

		//first rotation
		double alpha = atan2(y, x);

		//second rotation
		double beta = theta - alpha;
		if (beta > M_PI)
			beta = -(2.0 * M_PI - beta);
		else if (beta < -M_PI)
			beta += 2.0 * M_PI;

		JointData& rot1 = m_data[OMNIROB_LBR_BASE_JOINT_ROT1];
		JointData& lin = m_data[OMNIROB_LBR_BASE_JOINT_LIN];
		JointData& rot2 = m_data[OMNIROB_LBR_BASE_JOINT_ROT2];

		//compute velocities
		rot1.lastPos = rot1.pos;
		rot1.pos = alpha;
		double posDiff = rot1.pos - rot1.lastPos;
		ros::Duration timeDiff = time - rot1.lastTime;
		rot1.vel = m_firstRead ? 0.0 : (posDiff / timeDiff.toSec());
		rot1.lastTime = time;

		lin.lastPos = lin.pos;
		lin.pos = Eigen::Vector2d(x, y).norm();
		posDiff = lin.pos - lin.lastPos;
		timeDiff = time - lin.lastTime;
		lin.vel = m_firstRead ? 0.0 : (posDiff / timeDiff.toSec());
		lin.lastTime = time;

		rot2.lastPos = rot2.pos;
		rot2.pos = beta;
		posDiff = rot2.pos - rot2.lastPos;
		timeDiff = time - rot2.lastTime;
		rot2.vel = m_firstRead ? 0.0 : (posDiff / timeDiff.toSec());
		rot2.lastTime = time;
	}

	m_firstRead = true;

	return true;
#else
	LOG_FATAL("You cannot use this interface because it wasn't compiled with VREP support!");
	exit(-3);
#endif
}

bool OmniRobVRepInterface::write(const ros::Time time,
		const ros::Duration period)
{
#ifdef FOUND_VREP
	if (!m_vrep->checkConnection())
		return false;

	//we send all commands at the same time
	m_vrep->pauseCommunication();

	for (auto& it : m_data)
	{
		VrepData& vrepData = m_vrepData[it.first];
		if (it.second.mode == JointData::Velocity)
			m_vrep->setJointVelocity(vrepData.vrepHandle, it.second.cmdVel);
		else
			m_vrep->setJointVelocity(vrepData.vrepHandle, it.second.cmdPos);
	}

	//convert odom to virtual joints
	if (m_hasOdom)
	{
		JointData& rot1 = m_data[OMNIROB_LBR_BASE_JOINT_ROT1];
		JointData& lin = m_data[OMNIROB_LBR_BASE_JOINT_LIN];
		JointData& rot2 = m_data[OMNIROB_LBR_BASE_JOINT_ROT2];

		double alpha = rot1.pos;
		double alphaVel = rot1.cmdVel;
		double t = lin.pos;
		double tVel = lin.cmdVel;
		double beta = rot2.pos;
		double betaVel = rot2.cmdVel;

		Eigen::Vector3d omega(0, 0, alphaVel);
		Eigen::Vector3d tNorm(cos(alpha), sin(alpha), 0);
		Eigen::Vector3d vAngular = omega.cross(tNorm * t);
		Eigen::Vector3d vT = tNorm * tVel;
		Eigen::Vector3d v = vAngular + vT;

		Eigen::Matrix2d rot;
		rot << cos(beta), -sin(beta), sin(beta), cos(beta);
		Eigen::Vector2d vel = rot * v.head(2);

		static const double lenghtBetweenLeftAndRight = 0.5;
		static const double lengthBetweenFrontAndBack = 0.7;
		static const double invWheelRadius = 1.0 / 0.25;

		double flWheelSpeed;
		double frWheelSpeed;
		double rrWheelSpeed;
		double rlWheelSpeed;

		frWheelSpeed = +invWheelRadius * (+vel.y() + vel.x() - 0.5 * (lenghtBetweenLeftAndRight + lengthBetweenFrontAndBack) * betaVel);
		flWheelSpeed = -invWheelRadius * (-vel.y() + vel.x() + 0.5 * (lenghtBetweenLeftAndRight + lengthBetweenFrontAndBack) * betaVel);
		rlWheelSpeed = -invWheelRadius * (+vel.y() + vel.x() + 0.5 * (lenghtBetweenLeftAndRight + lengthBetweenFrontAndBack) * betaVel);
		rrWheelSpeed = +invWheelRadius * (-vel.y() + vel.x() - 0.5 * (lenghtBetweenLeftAndRight + lengthBetweenFrontAndBack) * betaVel);

		m_vrep->setJointVelocity(m_handleLeftRearWheel, rlWheelSpeed);
		m_vrep->setJointVelocity(m_handleLeftFrontWheel, flWheelSpeed);
		m_vrep->setJointVelocity(m_handleRightRearWheel, rrWheelSpeed);
		m_vrep->setJointVelocity(m_handleRightFrontWheel, frWheelSpeed);
	}

	//send the data
	m_vrep->startCommunication();

	//do a simulation step
	m_vrep->step();

	return true;
#else
	LOG_FATAL("You cannot use this interface because it wasn't compiled with VREP support!");
	exit(-3);
#endif
}

bool OmniRobVRepInterface::stop()
{
#ifdef FOUND_VREP
	return m_vrep->stopSimulation();
#else
	LOG_FATAL("You cannot use this interface because it wasn't compiled with VREP support!");
	exit(-3);
#endif
}

#ifdef FOUND_VREP
void OmniRobVRepInterface::callbackOdom(nav_msgs::OdometryConstPtr odom)
{
	boost::mutex::scoped_lock lock(m_odomMutex);
	m_odom = odom;
}
#endif

} /* namespace kuka_omnirob_vrep */

PLUGINLIB_EXPORT_CLASS(kuka_omnirob_vrep::OmniRobVRepInterface, prm_planner::RobotInterface)
