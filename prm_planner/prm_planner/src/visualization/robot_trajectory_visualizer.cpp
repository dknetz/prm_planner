/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Aug 12, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: robot_trajectory_visualizer.cpp
 */

#include <ais_definitions/class.h>
#include <prm_planner/robot/robot.h>
#include <prm_planner/visualization/robot_trajectory_visualizer.h>
#include <prm_planner_robot/path.h>
#include <ros/ros.h>

namespace prm_planner
{

RobotTrajectoryVisualizer::RobotTrajectoryVisualizer(boost::shared_ptr<Robot> robot,
		const std::string& type) :
				m_robot(robot),
				m_type(type),
				m_counter(0),
				m_nodeHandle("~")
{
	std::hash<std::string> hash;
	m_uniqueName = hash(m_nodeHandle.getNamespace());

	initRobotModel(m_robot->getRobotDescription(), m_nodeHandle.getNamespace() + m_robot->getRobotDescription() + "_" + m_type, m_type,
			m_robot, m_statePublisher, m_nodeHandle);
}

RobotTrajectoryVisualizer::~RobotTrajectoryVisualizer()
{
	m_thread.interrupt();
	m_thread.join();

	DELETE_VAR(m_statePublisher);
}

void RobotTrajectoryVisualizer::start()
{
	m_thread.interrupt();
	m_thread.join();
	m_thread = boost::thread(&RobotTrajectoryVisualizer::run, this);
}

void RobotTrajectoryVisualizer::setTrajectory(ArmJointPath& trajectory)
{
	boost::mutex::scoped_lock lock(m_mutex);
	m_trajectory = trajectory;
	m_counter = 0;
}

void RobotTrajectoryVisualizer::setTrajectory(boost::shared_ptr<Path>& path)
{
	ArmJointPath traj;
	for (auto& it : *path)
	{
		if (it.trajectory.empty())
		{
			continue;
		}
		traj.insert(traj.end(), it.trajectory.begin(), it.trajectory.end());
	}

	setTrajectory(traj);
}

void RobotTrajectoryVisualizer::initRobotModel(const std::string& robotDescriptionParam,
		const std::string& robotDescriptionParamNew,
		const std::string& prependString,
		boost::shared_ptr<RobotArm> robot,
		RobotStatePublisher*& statePublisher,
		ros::NodeHandle& myNodeHandle)
{
	ros::NodeHandle n;
	std::string desc;

	//publish robot model
	n.getParam(robotDescriptionParam, desc);

	const KDL::Tree& tree = robot->getRobot();
	for (auto& it : tree.getSegments())
	{
		boost::replace_all(desc, "name=\"" + it.second.segment.getName(), "name=\"" + it.second.segment.getName() + "_" + prependString + "_" + std::to_string(m_uniqueName));
		boost::replace_all(desc, "link=\"" + it.second.segment.getName(), "link=\"" + it.second.segment.getName() + "_" + prependString + "_" + std::to_string(m_uniqueName));
		boost::replace_all(desc, "name=\"" + it.second.segment.getJoint().getName(),
				"name=\"" + it.second.segment.getJoint().getName() + "_" + prependString + "_" + std::to_string(m_uniqueName));
	}

	statePublisher = new RobotStatePublisher(tree);

	myNodeHandle.setParam(robotDescriptionParamNew, desc);
}

void RobotTrajectoryVisualizer::update()
{
	int i = 0;
	static tf::Transform baseTF = tf::Transform::getIdentity();

	if (m_counter >= m_trajectory.size())
	{
		m_counter = 0;
	}

	std::unordered_map<std::string, double> joints;

	//get gripper state
	if (m_robot->getGripper().get() != NULL)
	{
		joints = m_robot->getGripper()->getJoints();
	}

	//use trajectory
	if (!m_trajectory.empty() && m_counter < m_trajectory.size())
	{
		for (auto& it : m_robot->getChainJointNames())
		{
			joints[it] = m_trajectory[m_counter].positions.data(i++, 0);
		}

		++m_counter;
	}
	//use current state
	else
	{
		KDL::JntArray current = m_robot->getKDLChainJointState();
		for (auto& it : m_robot->getChainJointNames())
		{
			joints[it] = current.data(i++, 0);
		}
	}

	//publish joint states of the robot
	m_statePublisher->publishTransforms(joints, ros::Time::now(), m_robot->getName(), "_" + m_type + "_" + std::to_string(m_uniqueName));
	m_statePublisher->publishFixedTransforms(m_robot->getName(), "_" + m_type + "_" + std::to_string(m_uniqueName));

	//publish static identy transformation between robot
	//and fake robot to be able to show the fake robot in rviz
	m_br.sendTransform(tf::StampedTransform(baseTF, ros::Time::now(), m_robot->getRootFrame(), m_robot->getRootFrame() + "_" + m_type + "_" + std::to_string(m_uniqueName)));
}

void RobotTrajectoryVisualizer::run()
{
	ros::Rate r(40);

	while (!boost::this_thread::interruption_requested())
	{
		update();
		r.sleep();
	}
}

} /* namespace prm_planner */

