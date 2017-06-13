/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Oct 10, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: follow_joint_trajectory_executer.cpp
 */

#include <ais_ros/ros_base_interface.h>
#include <prm_planner/controllers/helpers.h>
#include <prm_planner/controllers/simulation_robot_controller.h>
#include <prm_planner/controllers/simulation_velocity_controller.h>
#include <prm_planner/execution/follow_joint_trajectory_executer.h>
#include <prm_planner/problem_definitions/problem_definition.h>
#include <prm_planner/problem_definitions/problem_definition_manager.h>
#include <prm_planner/robot/robot.h>
#include <prm_planner_constraints/constraint.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace prm_planner
{

FollowJointTrajectoryExecuter::FollowJointTrajectoryExecuter() :
				m_goalReached(false),
				m_newDataReceived(false)
{
	boost::shared_ptr<ProblemDefinition> pd = ProblemDefinitionManager::getInstance()->getProblemDefinition();
	boost::shared_ptr<Robot> robot = pd->getRobot();

	//create controller
	m_controller.reset(new SimulationRobotController(robot, pd));
}

FollowJointTrajectoryExecuter::~FollowJointTrajectoryExecuter()
{
	interrupt();
	m_thread.join();
}

bool FollowJointTrajectoryExecuter::executePath(const boost::shared_ptr<Path> path)
{
	LOG_INFO("Execute Task Path");

	EXECUTER_LOCK();
	m_goalReached = false;
	m_newDataReceived = true;
	return Executer::executePath(path);

}

bool FollowJointTrajectoryExecuter::executePreprocessedPathMap(const boost::shared_ptr<Path>& path)
{
	LOG_INFO("Execute Preprocessed Path");

	EXECUTER_LOCK();
	bool result = Executer::executePreprocessedPathMap(path);
	m_goalReached = false;
	m_newDataReceived = true;
	return result;
}

void FollowJointTrajectoryExecuter::stopMotion()
{
	EXECUTER_LOCK();
	m_robot->stopMotion();
	m_goalReached = true;
	m_pathSegments.clear();
}

bool FollowJointTrajectoryExecuter::isGoalReached() const
{
	return m_goalReached;
}

bool FollowJointTrajectoryExecuter::hasErrors() const
{
	EXECUTER_LOCK();
	return m_pathSegments.empty() && !m_goalReached;
}

double FollowJointTrajectoryExecuter::getPathLength()
{
	LOG_ERROR("Not implemented!");
	return -1.0;
}

double FollowJointTrajectoryExecuter::getExecutedPathLength()
{
	LOG_ERROR("Not implemented!");
	return -1.0;
}

void FollowJointTrajectoryExecuter::publish()
{
}

void FollowJointTrajectoryExecuter::lock()
{
	m_mutex.lock();
}

void FollowJointTrajectoryExecuter::unlock()
{
	m_mutex.unlock();
}

void FollowJointTrajectoryExecuter::interrupt()
{
	m_thread.interrupt();
}

void FollowJointTrajectoryExecuter::init()
{
	m_thread = boost::thread(&FollowJointTrajectoryExecuter::run, this);
}

void FollowJointTrajectoryExecuter::reset()
{
	m_controller->reset();
}

void FollowJointTrajectoryExecuter::run()
{
	ros::Rate r(200);
	while (!boost::this_thread::interruption_requested())
	{
		m_mutex.lock();
		int size = m_pathSegments.size();
		m_mutex.unlock();

		//we have a new path, so execute it
		if (m_newDataReceived && size > 0)
		{
			m_newDataReceived = false;

			//get current arm poses
			KDL::JntArray jointPose = m_robot->getKDLChainJointState();

			//get some further information
			m_mutex.lock();
			SubPath pathSegments = m_pathSegments;
			m_mutex.unlock();

			if (pathSegments.empty())
				LOG_FATAL("is empty!!!!!!");

//			LOG_INFO("Got "<<pathSegments.begin()->second.size());

//wait for result
			while (!m_goalReached && !boost::this_thread::interruption_requested())
			{
				//we got a new path, so we stop current execution here
				if (m_newDataReceived)
					break;

				const int currentIndex = m_currentPathSegment.load();

				boost::shared_ptr<Path>& currentPath = pathSegments[currentIndex];

				//check if it is a special command
				if (currentPath->isSpecialCommand())
				{
					LOG_INFO("special command");
					handleSpecialCommand(pathSegments, currentIndex);
				}
				//send a new path to the controller
				else
				{
					LOG_INFO("path command");

					//update start joint pose to the last joint pose
					//or the current joint pose. We take the current
					//pose if the first sub path is currently handled,
					//otherwise we take the final position of the last
					//sub path, which was no special command
					currentPath->front().jointPose = jointPose;

					//the third parameter is used to specify
					//whether the controller should estimate
					//the runtime of all robot arm controllers.
					//It only has to be done if there is more
					//than one robot, because there synchronisation
					//might be needed.
					m_controller->updateFromRobotPath(pathSegments, currentIndex, pathSegments.size() > 1);

					//check motion and compute trajectory
					if (m_controller->canControl(500000, 0.01))
					{
						//get trajectory
						ArmJointPath path;
						m_controller->getJointPath(path);

						//get final joint pose, which are needed for the next sub path
						m_controller->getFinalJointState(jointPose);

						//generate messages
						control_msgs::FollowJointTrajectoryGoal goal;
						generateGoalMessage(path, goal);

						//execute trajectories
						m_robot->sendTrajectory(goal);

						//wait until goal is reached
						bool finished = true;
						ros::Rate r2(100);
						do
						{
							//first check, if one of the controllers doesn't reached its goal
							finished = m_robot->isTrajectoryExecutionFinished();

							//we got a new path, so we stop current execution here
							if (m_newDataReceived)
								break;

							r2.sleep();
						}
						while (!boost::this_thread::interruption_requested() && !finished);
					}
					else
					{
						m_mutex.lock();
						m_goalReached = false;
						m_pathSegments.clear();
						m_mutex.unlock();
						LOG_ERROR("Cannot execute trajectory!");
						break;
					}
				}

				//if we got new data, we will not increment the counters,
				//because it was set to 0 in the meantime
				if (!m_newDataReceived)
				{
					//increment the counter again to get the
					//next command
					++m_currentPathSegment;

					//reset some variables to initial state
					if (m_currentPathSegment == pathSegments.size())
					{
						m_mutex.lock();
						m_goalReached = true;
						m_pathSegments.clear();
						m_mutex.unlock();
					}
				}
			}
		}

		r.sleep();
	}
}

void FollowJointTrajectoryExecuter::generateGoalMessage(ArmJointPath& path,
		control_msgs::FollowJointTrajectoryGoal& goalMessage)
{
	//variables
	Eigen::Vector3d pos;
	Eigen::Quaterniond orientation;
	KDL::JntArray joints;
	double currentTime;
	const double newWaypointLin = 0.03;
	const double newWaypointAng = 0.05;

	//no motion required
	if (path.size() <= 1)
		return;

	//get parameters
	parameters::ControllerConfig controllerParams = ParameterServer::controllerConfigs[m_robot->getParameters().controllerConfig];

	//create ROS message
	goalMessage.trajectory.joint_names = m_robot->getChainJointNames();
	trajectory_msgs::JointTrajectoryPoint jointState;

	//old poses
	TrajectoryWaypoint& wpStart = path.front();
	Eigen::Vector3d oldPos = wpStart.taskPosition;
	Eigen::Quaterniond oldOrientation = wpStart.taskOrientation;
	KDL::JntArray oldJoints = wpStart.positions;
	double oldTime = wpStart.timeFromStart.toSec();

	//start state
	convert(wpStart.positions, jointState.positions);
	jointState.velocities = std::vector<double>(wpStart.positions.rows(), wpStart.velocityAng);
	jointState.time_from_start.fromSec(oldTime);
	goalMessage.trajectory.points.push_back(jointState);

	//states between start and goal
	for (size_t i = 1; i < path.size() - 1; ++i)
	{
		//get current position and orientation
		TrajectoryWaypoint& wp = path[i];
		pos = wp.taskPosition;
		orientation = wp.taskOrientation;
		joints = wp.positions;
		currentTime = wp.timeFromStart.toSec();

		//compute difference to old pose
		double distAngular = (orientation.angularDistance(oldOrientation));
		double distPos = (pos - oldPos).norm();

		double velAng = 1.0;
		double velLin = 1.0;

//		if (wp.velocityAng > 0)
//			velAng = wp.velocityAng / controllerParams.maxTaskVelAng;
//
//		if (wp.velocityLin > 0)
//			velLin = wp.velocityLin / controllerParams.maxTaskVelPos;

		const double newWaypointThresholdAngular = velAng * newWaypointAng;
		const double newWaypointThresholdPos = velLin * newWaypointLin;

		if (distAngular > newWaypointThresholdAngular || distPos > newWaypointThresholdPos)
		{
			double diff = (currentTime - oldTime);
			KDL::JntArray vel(wp.positions.rows());
			if (diff == 0)
				vel.data.setZero();
			else
				vel.data = (joints.data - oldJoints.data) / diff;

			//set message
			convert(wp.positions, jointState.positions);
//				convert(vel, jointState.velocities);
			jointState.velocities = std::vector<double>(vel.rows(), wp.velocityAng);
			jointState.time_from_start.fromSec(currentTime);
			goalMessage.trajectory.points.push_back(jointState);

			//set old to current
			oldPos = pos;
			oldOrientation = orientation;
			oldTime = currentTime;
			oldJoints = joints;
		}
	}

	//goal state
	TrajectoryWaypoint& wpGoal = path.back();
	convert(wpGoal.positions, jointState.positions);
	jointState.velocities = std::vector<double>(wpGoal.positions.rows(), wpGoal.velocityAng);
	jointState.time_from_start.fromSec(wpGoal.timeFromStart.toSec());
	goalMessage.trajectory.points.push_back(jointState);

	LOG_DEBUG("sending " << goalMessage.trajectory.points.size() << " of " << path.size() << " points");
}

} /* namespace prm_planner */
