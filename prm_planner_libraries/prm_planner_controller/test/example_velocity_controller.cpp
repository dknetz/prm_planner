/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Aug 19, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: example.cpp
 */

#include <prm_planner_constraints/constraint.h>
#include <prm_planner_constraints/constraint_factory.h>
#include <prm_planner_controller/velocity_controller.h>
#include <prm_planner_robot/defines.h>
#include <prm_planner_robot/path.h>
#include <prm_planner_robot/robot_arm.h>
#include <ros/ros.h>

using namespace prm_planner;

int main(int argc,
		char** argv)
{
	//init ros
	ros::init(argc, argv, "controller_example");
	ros::NodeHandle n;

	//setup parameters
	ControllerParameters parameters;
	parameters.frequency = 250;  //hz
	parameters.maxVelocity = 0.7;
	parameters.k =
	{	10, 10, 10, 3, 3, 3};
	parameters.lambda = 1.7;
	parameters.leastSquareDampingLambdaMax = 0.04;
	parameters.leastSquareDampingEpsilon = 0.04;
	parameters.debug = false;
	parameters.thresholdGoalReachedPos = 0.015;
	parameters.thresholdGoalReachedAng = 0.02;
	parameters.collisionAvoidanceUse = false;
	parameters.collisionAvoidanceRatioDoCollisionChecks = 1.0; //100% collision checks
	parameters.collisionAvoidanceVel0 = 3.0;
	parameters.collisionAvoidanceDistance1 = 0.2;
	parameters.collisionAvoidanceDistance2 = 0.5;
	parameters.collisionDetectionStopDistance = 0.07;
	parameters.collisionAvoidanceMaxJointVel = 0.7;
	parameters.maxTaskVelPos = 0.05;
	parameters.maxTaskVelAng = 0.2;
	parameters.jointRangeNullspaceWeightStd = 0.3;
	parameters.jointRangeNullspaceWeightDefault = 0.3;

	//setup robot parameters
	const std::string name = "iiwa";
	const std::string rootLink = "iiwa_0_link";
	const std::string tipLink = "iiwa_7_link";
	const std::string tfPrefix = "iiwa";
	const std::string robotDescription = "iiwa/robot_description";

	//other parameters
	const std::string planningFrame = "iiwa/base_link";

	//setup constraint
	boost::shared_ptr<Constraint> constraint = ConstraintFactory::create("xyzrpy"); //use exact pose

	//robot
	RobotArmConfig config;
	config.name = name;
	config.rootLink = rootLink;
	config.tipLink = tipLink;
	config.tfPrefix = tfPrefix;
	config.robotDescriptionParam = robotDescription;
	config.interfacePackage = "kuka_robot_interfaces";
	config.interfaceClass = "kuka_robot_interfaces::IiwaRobotInterface";
	config.executionInterface = ArmExecutionMode::HardwareInterface;
	boost::shared_ptr<RobotArm> robot(new RobotArm(config, true));

	//create path
	boost::shared_ptr<Path> path;
	Path::Waypoint wp;
	wp.id = 0;
	wp.pose = Eigen::Affine3d::Identity(); //set the right poses here
	path->append(wp, true); //add the waypoint to the path and clean the path

	//setup controller
	VelocityController<7> controller(parameters, constraint, robot, planningFrame);
	controller.init();
	controller.updateFromPath(path);

	//initialize time
	ros::Time now, last;
	Controller::getTime(now);
	last = now;
	ros::Duration dt = now - last;

	ros::Rate r(parameters.frequency);

	while (!controller.isGoalReached())
	{
		last = now;
		Controller::getTime(now);
		dt = now - last;

		controller.update(now, dt);

		r.sleep();
	}

	//start ros spinner
	ros::AsyncSpinner spinner(0);
	spinner.start();

	//wait until user presses ctrl+c
	ros::waitForShutdown();

	return 0;
}

