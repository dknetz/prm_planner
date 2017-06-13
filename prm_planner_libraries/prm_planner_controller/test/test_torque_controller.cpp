/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Oct 28, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: test_velocity_controller.cpp
 */

#include <ais_util/math.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chaindynparam.hpp>
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

typedef Eigen::Matrix<double, 6, 1> CartVec;
typedef Eigen::Matrix<double, 7, 1> JointVec;

KDL::ChainFkSolverPos_recursive* fksolver;
KDL::ChainIdSolver_RNE* gravitySolver;
KDL::ChainIdSolver_RNE* coriolisSolver;
KDL::ChainDynParam* dynamicsSolver;
JointVec VOld;
bool first = true;

struct Pose
{
	Eigen::Vector3d position;
	Eigen::Quaterniond orientation;

	KDL::Frame toKDL()
	{
		KDL::Frame f;
		f.p[0] = position.x();
		f.p[1] = position.y();
		f.p[2] = position.z();
		f.M = KDL::Rotation::Quaternion(orientation.x(), orientation.y(), orientation.z(), orientation.w());
		return f;
	}
};

void KDL2Pose(KDL::Frame& frame,
		Pose& pose)
{
	pose.position = Eigen::Vector3d(frame.p[0], frame.p[1], frame.p[2]);
	frame.M.GetQuaternion(pose.orientation.x(), pose.orientation.y(), pose.orientation.z(), pose.orientation.w());
}

//see http://free-cad.sourceforge.net/SrcDocu/d4/d44/frames_8inl_source.html
void computeDiff(Eigen::Matrix<double, 6, 1>& start,
		Eigen::Matrix<double, 6, 1>& goal,
		Eigen::Matrix<double, 6, 1>& diff)
{
	diff = goal - start;
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

double getHeightOfCenterOfMass(KDL::JntArray& jointState,
		KDL::Vector& cog,
		int linkI)
{
	KDL::Frame pose;
	fksolver->JntToCart(jointState, pose, linkI);
	return (pose * cog).z();
}

int main(int argc,
		char** argv)
{
	//init ros
	ros::init(argc, argv, "controller_example");
	ros::NodeHandle n;

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
	const std::vector<std::string> names = { IIWA_1_JOINT, IIWA_2_JOINT, IIWA_3_JOINT, IIWA_4_JOINT, IIWA_5_JOINT, IIWA_6_JOINT, IIWA_7_JOINT };

	//other parameters
	const std::string planningFrame = "base_link";

	//file
	std::ofstream file("/tmp/path.txt");
	std::ofstream fileGT("/tmp/path_gt.txt");
	std::ofstream fileErrors("/tmp/path_errors.txt");
	std::ofstream fileErrorsDot("/tmp/path_errors_dot.txt");

	//robot
	RobotArmConfig config;
	config.name = name;
	config.rootLink = rootLink;
	config.tipLink = tipLink;
	config.tfPrefix = tfPrefix;
	config.robotDescriptionParam = robotDescription;
	config.interfacePackage = "kuka_robot_interfaces";
//	config.interfaceClass = "kuka_robot_interfaces::IiwaVRepInterface";
	config.interfaceClass = "kuka_robot_interfaces::IiwaRobotInterface";
	config.executionInterface = ArmExecutionMode::HardwareInterface;
	boost::shared_ptr<RobotArm> robot(new RobotArm(config, true));

	//get chain
	KDL::Chain chain = robot->getChain();

	//constants and parameters
	KDL::JntArray velNull(7);
	velNull.data.setZero();
	bool setInitPosition = true;
	const double velTraj = 0.5;
	int counter = 0;
	const double timestep = 0.005;

	//gains
	Eigen::Matrix<double, 6, 6> kp, kv;
	kp.setIdentity();
	kv.setIdentity();
	kp(0, 0) = kp(1, 1) = kp(2, 2) = 20;
	kp(3, 3) = kp(4, 4) = kp(5, 5) = 10;
	kv(0, 0) = kv(1, 1) = kv(2, 2) = 15;
	kv(3, 3) = kv(4, 4) = kv(5, 5) = 7.5;

	//joint position and derivatives
	KDL::JntArray q(7);
	KDL::JntArray qOld(7);
	KDL::JntArray qDot(7);
	KDL::JntArray qDotOld(7);
	KDL::JntArray qDotDot(7);

	//time related
	ros::Time t = ros::Time::now();
	ros::Time tStart = t;
	ros::Time tOld = t;
	ros::Duration dt(0);

	//solvers
	JacobianMultiSolver solver(chain);
	fksolver = new KDL::ChainFkSolverPos_recursive(chain);
//	gravitySolver = new KDL::ChainIdSolver_RNE(chain, KDL::Vector(0, 0, -9.81));
//	coriolisSolver = new KDL::ChainIdSolver_RNE(chain, KDL::Vector::Zero());
	dynamicsSolver = new KDL::ChainDynParam(chain, KDL::Vector(0, 0, -9.81));
	KDL::ChainIkSolverVel_pinv iksolverv(chain); //Inverse velocity solver
	KDL::ChainIkSolverPos_NR iksolver(chain, *fksolver, iksolverv, 100, 1e-6); //Maximum 100 iterations, stop at accuracy 1e-6

	Pose goal, goalOld;					//goal eef pose
	KDL::Frame kdlGoal, kdlGoalOld;
	Pose current, currentOld;			//current eef pose
	KDL::Frame kdlCurrent;
	KDL::JntSpaceInertiaMatrix H(7); 	//joint space inertia matrix
	Eigen::Matrix<double, 6, 1> vel;	//eef velocity
	Eigen::Matrix<double, 6, 6> M;		//cartesian space inertia matrix
	Eigen::Matrix<double, 7, 1> G, C;	//gravity and coriolis forces (Eigen)
	KDL::JntArray gravity(7);			//gravity (KDL)
	KDL::JntArray coriolis(7);			//corriolis (KDL)
	std::vector<KDL::Jacobian> j;		//jacobians for all links
	KDL::JntArray cmdTorque(7);			//command torque vector
	KDL::Wrenches wrenches(chain.segments.size(), KDL::Wrench::Zero());
	Eigen::Matrix<double, 6, 1> xDVelocity = Eigen::Matrix<double, 6, 1>::Zero();
	Eigen::Matrix<double, 6, 1> xDVelocityOld = xDVelocity;
	Eigen::Matrix<double, 6, 1> xDAcceleration = Eigen::Matrix<double, 6, 1>::Zero();

	//initialize position
	q(0) = 0;
	q(1) = DEG2RAD(45.0);
	q(2) = 0;
	q(3) = DEG2RAD(-90.0);
	q(4) = 0;
	q(5) = DEG2RAD(-45.0);
	q(6) = 0;
	qOld = q;
	qDot.data.setZero();
	qDotOld.data.setZero();
	qDotDot.data.setZero();

	//get forward kinematic position for q
	fksolver->JntToCart(q, kdlGoal);

	//compute initial position
	kdlGoal.p[0] = 0.12 * cos(t.toSec()) + 0.7;
	kdlGoal.p[1] = 0.5 * sin(2.0 * t.toSec()) / 2;
	iksolver.CartToJnt(q, kdlGoal, q);
	KDL2Pose(kdlGoal, goal);
	goalOld = goal;

	//set this pose
	if (config.interfaceClass == "kuka_robot_interfaces::IiwaVRepInterface")
		vrep_interface::VRepInterface::getInstance()->stopSimulation();
	sleep(1);
	robot->sendChainJointPosition(q);

	if (config.interfaceClass == "kuka_robot_interfaces::IiwaVRepInterface")
		vrep_interface::VRepInterface::getInstance()->startSimulation();

	//get state
	robot->receiveData(t, dt);
	q = robot->getKDLChainJointState();

	//dummy
	tOld = t;
	t = ros::Time::now();
	dt = t - tOld;
	robot->receiveData(t, dt);
	q = robot->getKDLChainJointState();

//	KDL::JntArray qGoal = q;

	ros::Rate r(250);
	while (ros::ok())
	{
		//update time
//		t += ros::Duration(timestep); //ros::Time::now();
		t = ros::Time::now();
		dt = t - tOld;
		double _dt = dt.toSec();

		//======= get data
		//of joints
		robot->receiveData(t, dt);
		q = robot->getKDLChainJointState();
//		LOG_INFO(q.data);

		//and eef
		fksolver->JntToCart(q, kdlCurrent);
		KDL2Pose(kdlCurrent, current);

		//update goal
		//set new goal pose
//		kdlGoal.p[0] = 0.15 * cos(t.toSec() * velTraj) + 0.7;
//		kdlGoal.p[1] = 0.5 * sin(2.0 * t.toSec() * velTraj) / 2;
//		KDL2Pose(kdlGoal, goal);

		//======= compute velocities
		//start condition: assume zero velocity
		if (counter == 0)
		{
			currentOld = current;
			goalOld = goal;
			qOld = q;
		}

		//joint
		qDot.data = (q.data - qOld.data) / _dt;

		//eef
		computeDiff(currentOld, current, vel); //velocity
		vel /= _dt;

		//goal
		computeDiff(goalOld, goal, xDVelocity);
		xDVelocity /= _dt;

		//======= compute acceleration
		//start condition: assume zero acceleration
		if (counter <= 1)
		{
			qDotOld = qDot;
			xDVelocityOld = xDVelocity;
		}

		//joint
		qDotDot.data = (qDot.data - qDotOld.data) / _dt;

		//goal
		computeDiff(xDVelocityOld, xDVelocity, xDAcceleration);
		xDAcceleration /= _dt;

		//======= compute error
		Eigen::Matrix<double, 6, 1> e, eDot;
		computeDiff(current, goal, e);
		computeDiff(vel, xDVelocity, eDot);

		//======= compute jacobian and pseudo inverse
		solver.GetAllJacobians<7>(q, j);
		KDL::Jacobian armKDL = j.back();
		Eigen::Matrix<double, 6, 7> eefJ = armKDL.data;
		Eigen::Matrix<double, 7, 6> pinv;
		computeDampedLeastSquarePseudoInverse(eefJ, pinv, parameters);

		//compute dynamics
		dynamicsSolver->JntToMass(q, H);
		dynamicsSolver->JntToGravity(q, gravity);
		dynamicsSolver->JntToCoriolis(q, qDot, coriolis);
		M = (eefJ * H.data.inverse() * eefJ.transpose()).inverse();
		G = gravity.data;
		C = coriolis.data;

		/*arm.transpose() * ((pinv.transpose() * M.data * pinv) * (xDAcceleration + kp * e + kv * eDot)) +*/
		//H.data * (velNull.data + 15 * (qGoal.data - q.data))
		cmdTorque.data = eefJ.transpose() * (M * (xDAcceleration + kp * e + kv * eDot));// + C + G; //arm.transpose() * M * xDAcceleration +

//		LOG_INFO(cmdTorque.data.transpose());

		//write data
		robot->sendTorque(t, dt, cmdTorque);

		tOld = t;
		goalOld = goal;
		currentOld = current;
		xDVelocityOld = xDVelocity;
		qOld = q;
		qDotOld = qDot;
		first = false;
		++counter;

		file << current.position.x() << " " << current.position.y() << std::endl;
		fileGT << goal.position.x() << " " << goal.position.y() << std::endl;
		fileErrors << (t - tStart).toSec() << " " << e(0) << " " << e(1) << " "<< e(2) << " "<< e(3) << " "<< e(4) << " "<< e(5) << std::endl;
		fileErrorsDot << (t - tStart).toSec() << " " << eDot(0) << " " << eDot(1) << " "<< eDot(2) << " "<< eDot(3) << " "<< eDot(4) << " "<< eDot(5) << std::endl;

		r.sleep();
	}

	delete fksolver;

	return 0;
}

