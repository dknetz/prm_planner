/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jan 5, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: robot.cpp
 */

#include <ais_log/log.h>
#include <ais_definitions/macros.h>
#include <ais_definitions/exception.h>
#include <eigen_conversions/eigen_kdl.h>
#include <visualization_msgs/MarkerArray.h>
#include <exception>
#include <random>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <prm_planner/robot/robot.h>
#include <prm_planner/util/parameter_server.h>
#include <prm_planner/util/defines.h>
#include <chrono>

namespace prm_planner
{

Robot::Robot(const std::string robotName) :
				RobotArm(ParameterServer::robotConfigs[robotName].arms.begin()->second.arm, true),
				c_robotName(robotName)
{
	parameters::RobotConfig config = ParameterServer::robotConfigs[c_robotName];
	parameters::ArmConfig armConfig = config.arms.begin()->second;

	//set hand
	if (armConfig.hasHand)
	{
		boost::shared_ptr<GripperInterface> gripper = GripperInterface::load(armConfig.hand.interfacePackage, armConfig.hand.interfaceClass);

		GripperInterface::GripperInterfaceParameters params;
		params.graspFrame = armConfig.arm.tfPrefix + "/" + armConfig.arm.tipLink;
		params.jointStateTopic = armConfig.hand.jointStateTopic;
		params.jointNames = armConfig.hand.jointNames;
		params.name = armConfig.hand.name;
		params.topic = armConfig.hand.topic;
		params.dropPreDistance = armConfig.hand.dropPreDistance;
		params.graspPreDistance = armConfig.hand.graspPreDistance;
		params.graspRadius = armConfig.hand.graspRadius;
		params.graspPostHeight = armConfig.hand.graspPostHeight;

		gripper->init(params);
		setGripper(gripper);
	}
}

//Robot::Robot(const Robot& other) :
////				RobotArm(other.c_armConfig, false), //just a copy of the robot model without a real robot connection
//				c_robotName(other.c_robotName)
//{
//}

Robot::~Robot()
{
}

} /* namespace prm_planner */

