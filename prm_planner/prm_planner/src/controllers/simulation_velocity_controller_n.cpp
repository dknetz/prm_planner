/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jan 13, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: elastic_roadmap_edge_controller.cpp
 */

#include <ais_util/stop_watch.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl/jntarray.hpp>
#include <omp.h>
#include <prm_planner/controllers/simulation_velocity_controller.h>
#include <prm_planner/problem_definitions/problem_definition.h>
#include <prm_planner_constraints/constraint_factory.h>
#include <prm_planner_constraints/constraint_xyzrpy.h>
#include <prm_planner_robot/path.h>
#include <prm_planner_robot/robot_arm.h>
#include <prm_planner_robot/kinematics.h>
#include <prm_planner/util/parameter_server.h>
#include <iosfwd>

namespace prm_planner
{

template<int Dim, class Type>
SimulationVelocityControllerN<Dim, Type>::SimulationVelocityControllerN(const ControllerParameters& parameters,
		boost::shared_ptr<RobotArm> robotArm,
		boost::shared_ptr<ProblemDefinition> pd) :
				VelocityController<Dim, Type>(parameters, pd->getConstraint(), robotArm,
						pd->getFrame(),
						ParameterServer::octomapResolution)
{
	this->template init();

	this->m_now = this->m_start = ros::Time(0);
	this->m_dt = ros::Duration(1.0 / this->c_parameters.frequency);
}

template<int Dim, class Type>
SimulationVelocityControllerN<Dim, Type>::SimulationVelocityControllerN(const Eigen::Affine3d& start,
		const Eigen::Affine3d& goal,
		const KDL::JntArray& qStart,
		const ControllerParameters& parameters,
		boost::shared_ptr<RobotArm> robotArm,
		boost::shared_ptr<ProblemDefinition> pd) :
				VelocityController<Dim, Type>(parameters, pd->getConstraint(), robotArm,
						pd->getFrame(),
						ParameterServer::octomapResolution),
				m_startPose(start),
				m_goalPose(goal)
{
	this->m_q = qStart;
	this->template init();

	this->m_now = this->m_start = ros::Time(0);
	this->m_dt = ros::Duration(1.0 / this->c_parameters.frequency);

	updateFromValues(m_startPose, m_goalPose, qStart);
}
template<int Dim, class Type>
SimulationVelocityControllerN<Dim, Type>::SimulationVelocityControllerN(
		const boost::shared_ptr<SimulationVelocityControllerN<Dim, Type>> controller,
		const ControllerParameters& parameters,
		boost::shared_ptr<RobotArm> robotArm,
		boost::shared_ptr<ProblemDefinition> pd) :
				VelocityController<Dim, Type>(parameters,
						pd->getConstraint(),
						robotArm,
						pd->getFrame(),
						ParameterServer::octomapResolution)
{
	this->m_success = true;

	controller->lock();
	this->m_q = controller->m_q;
	this->template init();
	this->m_x = controller->m_x;
	this->m_x0 = controller->m_x0;
	this->m_xGoal = controller->m_xGoal;
	this->m_xPredicted = controller->m_xPredicted;
	this->m_xPredictedOld = controller->m_xPredictedOld;
	this->m_error = controller->m_error;
	this->m_xDesiredDot = controller->m_xDesiredDot;
	this->m_start = controller->m_start;
	this->m_now = controller->m_now;
	this->s_maxVelocityLimit = controller->s_maxVelocityLimit;
	this->m_counter = controller->m_counter;
	this->m_path = controller->m_path;
	this->m_trajectory = controller->m_trajectory;
//	this->m_kdtree = controller->m_kdtree;
	controller->unlock();
}

template<int Dim, class Type>
SimulationVelocityControllerN<Dim, Type>::~SimulationVelocityControllerN()
{
}

template<int Dim, class Type>
bool SimulationVelocityControllerN<Dim, Type>::update(const ros::Time& now,
		const ros::Duration& dt)
{
	if (this->m_path->size() == 1)
	{
		return true;
	}

	if (!this->m_trajectory->isValid())
	{
		LOG_INFO("trajectory is invalid");
		return false;
	}

	this->m_dt = dt;
//	m_dt = ros::Duration(0.1);
	this->m_now = now;

	const int nrOfJoints = this->m_robotArm->getChain().getNrOfJoints();
	KDL::JntArray cmd(nrOfJoints);

	//get data
	this->m_x.reset(this->m_kinematics, this->m_q); //this->m_robotArm->getFkSolver()

	//store trajectory for visualization and follow joint trajectory
	//interface
	const Path::Waypoint& currentPathWP = this->m_path->operator [](this->m_currentWaypoint);
	TrajectoryWaypoint wp;
	wp.positions = this->m_q;
	wp.taskPosition = this->m_x.x.head(3);
	wp.taskOrientation = this->m_x.quaternion;
	wp.timeFromStart = this->m_now;
	if (this->m_currentWaypoint >= 0 && this->m_currentWaypoint < this->m_path->size())
	{
		wp.velocityAng = currentPathWP.maxAngularVel;
		wp.velocityLin = currentPathWP.maxTranslationalVel;
	}
	else
	{
		wp.velocityAng = -1.0;
		wp.velocityLin = -1.0;
	}
	m_jointPositions.push_back(wp);

	//compute Jacobian
//	ais_util::StopWatch::getInstance()->start("jacobian");
	if (this->m_q.rows() == 0)
	{
		LOG_ERROR("You cannot use the controller with 0 sized joint states! You need to"
				"set an initial state.");
		return false;
	}

	//get jacobian
	this->m_kinematics->getJacobian(this->m_q, this->m_kdlJacobian);
	this->m_jacobian = this->m_kdlJacobian.data;

//	ais_util::StopWatch::getInstance()->stopPrint("jacobian");

//and inverse
//	ais_util::StopWatch::getInstance()->start("pinv");
	computeDampedLeastSquarePseudoInverse(this->m_jacobian, this->m_pseudoInverse, this->c_parameters);
//	ais_util::StopWatch::getInstance()->stopPrint("pinv");

	//compute where the robot should be
	this->template computePrediction();

	//compute error
	this->template computeError();

	//compute desired velocity
	this->c_constraint->computeDifference(this->m_xPredicted, this->m_xPredictedOld, this->m_xDesiredDot);
	this->m_xDesiredDot /= this->m_dt.toSec();

//...collision avoidance
//	if (c_parameters.collisionAvoidanceUse)
//	{
//		updateObjectDistances();
////			addNullSpaceCollisionAvoidance();
//
//		if (checkCollision())
//		{
//			//ws->stopPrint("check");
//			return false;
//		}
//	}

	//compute command
//	LOG_INFO(this->m_x.x.transpose());
//	LOG_INFO(this->m_xPredicted.x.transpose());
//	LOG_INFO(this->m_q.data.transpose());
//	LOG_INFO(this->m_now.toSec());
	cmd.data = this->m_pseudoInverse * (this->m_xDesiredDot + this->m_k * this->m_error);

	//add nullspace joint range optimization if needed
	if (this->c_parameters.optimizeJointRangeInNullspace)
	{
		this->m_N = this->m_I - this->m_pseudoInverse * this->m_jacobian; //compute null space projector
		this->m_qNullSpace.fill(0);
		this->addNullSpaceMiddleJointRange();
		Type weight = this->computeJointRangeWeight();
		cmd.data += weight * this->m_qNullSpaceJointRange;
	}

	if (!this->template isValidCommand(cmd, this->m_dt.toSec(), 0.04))
	{
		return false;
	}

//	if(isGoalReached(m_x)) {
//		LOG_INFO(m_c);
//		LOG_INFO(m_dt.toSec());
//		LOG_INFO(m_now.toSec());
//	}

//	if (isGoalReached(m_x)) {
//		LOG_INFO("goal reached");
//		Trajectory::Pose p1(m_startPose);
//		Trajectory::Pose p2(m_goalPose);
//		Vector6d error;
//		c_constraint->computeError(p1, p2, error);
//
//		m_jacobianSolver->GetAllJacobians<Dim>(m_qStart, m_jacobians);
//			m_jacobian = m_jacobians.back();
//
//			computeDampedLeastSquarePseudoInverse(m_jacobian, m_pseudoInverse, c_parameters);
//
//			LOG_INFO(error.transpose());
//			LOG_INFO(m_q.data.transpose());
//			KDL::JntArray test;
//			test.data = m_pseudoInverse * error;
//			LOG_INFO((m_q.data + test.data).transpose());
//	}

	this->m_q.data += cmd.data * this->m_dt.toSec();

	return true;
}

template<int Dim, class Type>
KDL::JntArray SimulationVelocityControllerN<Dim, Type>::getJointStateFromRobot()
{
	return KDL::JntArray(Dim);
}

template<int Dim, class Type>
bool SimulationVelocityControllerN<Dim, Type>::updateFromValues(const Eigen::Affine3d& startPose,
		const Eigen::Affine3d& goalPose,
		const KDL::JntArray& startJoints)
{
	this->m_success = true;
	this->m_now = this->m_start = ros::Time(0);

//	KDL::JntArray jIk;
//	if (!m_robotArm->getIK(goalPose, jIk))
//	{
//		return false;
//	}

	m_startPose = this->m_transformationPlanningFrameToArm * startPose; //transform it into arm frame
	m_goalPose = this->m_transformationPlanningFrameToArm * goalPose; //transform it into arm frame;

	std::vector<Path::Waypoint> waypoints(2);
	waypoints[0].id = 0;
	waypoints[0].pose = m_startPose;
	waypoints[1].id = 1;
	waypoints[1].pose = m_goalPose;

	this->m_path.reset(new Path(this->c_planningFrame));
	this->m_path->setWaypoints(waypoints);

	//same waypoints
	if (this->m_path->size() == 1)
	{
		return true;
	}

	this->m_trajectory.reset(new Trajectory(this->m_path, Vector6d::Zero(), this->c_parameters.maxTaskVelPos,
			this->c_parameters.maxTaskVelAng, this->m_now, this->c_armFrame));

	if (!this->m_trajectory->isValid())
	{
		LOG_INFO_COND(VERB, waypoints[0].pose.matrix());
		LOG_INFO_COND(VERB, waypoints[1].pose.matrix());
		return false;
	}

	this->m_q = startJoints;

	this->m_x = this->m_x0 = this->m_xPredicted = this->m_xPredictedOld = Trajectory::Pose(waypoints[0].pose);
	this->m_xGoal = Trajectory::Pose(waypoints[1].pose);
	m_jointPositions.clear();

//	LOG_INFO("Init");
//	LOG_INFO(this->m_x.x.transpose());
//	LOG_INFO(this->m_xGoal.x.transpose());
//	LOG_INFO(this->m_q.data.transpose());

	return true;
}

template<int Dim, class Type>
bool SimulationVelocityControllerN<Dim, Type>::updateFromPath(const boost::shared_ptr<Path> path)
{
	this->m_success = true;
	this->m_now = this->m_start = ros::Time(0);
	this->m_path = path;

	//same waypoints
	if (this->m_path->size() == 1)
	{
		return true;
	}

	this->m_trajectory.reset(new Trajectory(this->m_path, Vector6d::Zero(), this->c_parameters.maxTaskVelPos,
			this->c_parameters.maxTaskVelAng, this->m_now, this->c_armFrame));

	if (!this->m_trajectory->isValid())
	{
		return false;
	}

	Path::Waypoint& front = this->m_path->front();
	this->m_q = front.jointPose;
	this->m_x = this->m_x0 = this->m_xPredicted = this->m_xPredictedOld = Trajectory::Pose(front.pose);
	this->m_xGoal = Trajectory::Pose(this->m_path->back().pose);
	m_jointPositions.clear();

	return true;
}

template<int Dim, class Type>
const ArmJointPath& SimulationVelocityControllerN<Dim, Type>::getJointPositions() const
{
	return m_jointPositions;
}

template class SimulationVelocityControllerN<7, double> ;
template class SimulationVelocityControllerN<10, double> ;

} /* namespace prm_planner */

