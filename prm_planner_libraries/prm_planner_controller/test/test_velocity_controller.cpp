/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Oct 28, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: test_torque_controller.cpp
 */

#include <ais_util/math.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <prm_planner_controller/jacobian_multi_solver.h>
#include <prm_planner_controller/torque_controller.h>
#include <prm_planner_robot/defines.h>
#include <prm_planner_robot/robot_arm.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <vrep_interface/v_rep_interface.h>
#include <vrep_interface/v_rep_iiwa.h>
#include <fstream>

using namespace prm_planner;

struct Pose {
	Eigen::Vector3d position;
	Eigen::Quaterniond orientation;

	KDL::Frame toKDL() {
		KDL::Frame f;
		f.p[0] = position.x();
		f.p[1] = position.y();
		f.p[2] = position.z();
		f.M = KDL::Rotation::Quaternion(orientation.x(), orientation.y(), orientation.z(), orientation.w());
		return f;
	}
};

void KDL2Pose(KDL::Frame& frame, Pose& pose) {
	pose.position = Eigen::Vector3d(frame.p[0], frame.p[1], frame.p[2]);
	frame.M.GetQuaternion(pose.orientation.x(), pose.orientation.y(), pose.orientation.z(), pose.orientation.w());
}

void computeDiff(Pose& start,
		Pose& goal,
		Eigen::Matrix<double, 6, 1>& diff)
{
	Eigen::Quaterniond diffQ = goal.orientation * start.orientation.inverse();
	KDL::Rotation rpy = KDL::Rotation::Quaternion(diffQ.x(), diffQ.y(), diffQ.z(), diffQ.w());
	rpy.GetRPY(diff(3), diff(4), diff(5));
	diff(0) = goal.position.x() - start.position.x();
	diff(1) = goal.position.y() - start.position.y();
	diff(2) = goal.position.z() - start.position.z();
}

int main(int argc,
		char** argv)
{
	//init ros
	ros::init(argc, argv, "controller_example");
	ros::NodeHandle n;
//	ros::Publisher pub = n.advertise<sensor_msgs::JointState>("/joint_states", 10);
//	kuka_robot_interfaces::IiwaVRepInterface robotInterface;
//	if (!robotInterface.start())
//	{
//		LOG_FATAL("Cannot connect to V-Rep!");
//		return 1;
//	}

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
	const std::string robotDescription = "robot_description";
	const std::vector<std::string> names = { IIWA_1_JOINT, IIWA_2_JOINT, IIWA_3_JOINT, IIWA_4_JOINT, IIWA_5_JOINT, IIWA_6_JOINT, IIWA_7_JOINT };

	//other parameters
	const std::string planningFrame = "base_link";

	//file
	std::ofstream file("/tmp/path.txt");
	std::ofstream fileGT("/tmp/path_gt.txt");

	//robot
	RobotArmConfig config;
	config.name = name;
	config.rootLink = rootLink;
	config.tipLink = tipLink;
	config.tfPrefix = tfPrefix;
	config.robotDescriptionParam = robotDescription;
	config.interfacePackage = "kuka_robot_interfaces";
	config.interfaceClass = "kuka_robot_interfaces::IiwaVRepInterface";
	config.executionInterface = ArmExecutionMode::HardwareInterface;
	boost::shared_ptr<RobotArm> robot(new RobotArm(config, true));

	KDL::Chain chain = robot->getChain();

	KDL::JntArray q(7);
	q(0) = 0;
	q(1) = DEG2RAD(45.0);
	q(2) = 0;
	q(3) = DEG2RAD(-90.0);
	q(4) = 0;
	q(5) = DEG2RAD(-45.0);
	q(6) = 0;

	JacobianMultiSolver solver(chain);
	KDL::ChainFkSolverPos_recursive fksolver(chain);
	KDL::ChainIkSolverVel_pinv iksolverv(chain); //Inverse velocity solver
	KDL::ChainIkSolverPos_NR iksolver(chain, fksolver, iksolverv, 100, 1e-6); //Maximum 100 iterations, stop at accuracy 1e-6

	std::vector<Eigen::Matrix<double, 6, 7>> j;

	KDL::Frame kdlGoal, kdlGoalOld;
	KDL::Frame kdlCurrent;

	Pose goal, goalOld, current;

	fksolver.JntToCart(q, kdlGoal);

	KDL::JntArray cmdVel(7);

	bool setInitPosition = true;

	ros::Time t = ros::Time::now();
	ros::Time tOld = t;
	ros::Duration dt(0);

	//compute initial position
	kdlGoal.p[0] = 0.12 * cos(t.toSec()) + 0.7;
	kdlGoal.p[1] = 0.5 * sin(2.0 * t.toSec()) / 2;
	iksolver.CartToJnt(q, kdlGoal, q);
	KDL2Pose(kdlGoal, goal);
	goalOld = goal;

	//set this pose
	vrep_interface::VRepInterface::getInstance()->stopSimulation();
	sleep(1);
	robot->sendChainJointPosition(q);
	vrep_interface::VRepInterface::getInstance()->startSimulation();

	//get state
	robot->receiveData(t, dt);
	q = robot->getKDLChainJointState();

	//gains
	Eigen::Matrix<double, 6, 6> K;
	K.setIdentity();
	K(0, 0) = K(1, 1) = K(2, 2) = 10;
	K(3, 3) = K(4, 4) = K(5, 5) = 5;

	Eigen::Matrix<double, 6, 1> xD;

	//dummy
	tOld = t;
	t = ros::Time::now();
	dt = t - tOld;
	robot->receiveData(t, dt);
	q = robot->getKDLChainJointState();

	const double velTraj = 0.5;

	ros::Rate r(100);
	while (ros::ok())
	{
		//update time
		t = ros::Time::now();
		dt = t - tOld;

		//get data
		robot->receiveData(t, dt);
		q = robot->getKDLChainJointState();

		//get eef pose
		fksolver.JntToCart(q, kdlCurrent);
		KDL2Pose(kdlCurrent, current);

		//set new goal pose
		kdlGoal.p[0] = 0.15 * cos(t.toSec() * velTraj) + 0.7;
		kdlGoal.p[1] = 0.5 * sin(2.0 * t.toSec() * velTraj) / 2;
		KDL2Pose(kdlGoal, goal);

		//compute desired velocity
		computeDiff(goalOld, goal, xD);
		goalOld = goal;

		//divide by dt to get speed
		xD /= dt.toSec();

		//compute error
		Eigen::Matrix<double, 6, 1> e;
		computeDiff(current, goal, e);

		solver.GetAllJacobians<7>(q, j);
		Eigen::Matrix<double, 6, 7> arm = j.back();
		Eigen::Matrix<double, 7, 6> pinv;
		computeDampedLeastSquarePseudoInverse(arm, pinv, parameters);

		cmdVel.data = pinv * (xD + K * e);

		//write data
		robot->sendVelocity(t, dt, cmdVel);

		tOld = t;

		file << current.position.x() << " " << current.position.y() << std::endl;
		fileGT << goal.position.x() << " " << goal.position.y() << std::endl;

		r.sleep();
	}

	return 0;
}

