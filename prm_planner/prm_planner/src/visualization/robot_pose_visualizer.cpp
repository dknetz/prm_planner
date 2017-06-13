/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Jun 6, 2017
 *      Author: kuhnerd
 * 	  Filename: robot_pose_visualizer.cpp
 */

#include <ais_definitions/class.h>
#include <prm_planner/robot/robot.h>
#include <prm_planner/visualization/robot_pose_visualizer.h>
#include <prm_planner_robot/path.h>
#include <ros/ros.h>

namespace prm_planner
{

RobotPoseVisualizer::RobotPoseVisualizer(boost::shared_ptr<Robot> robot,
		const std::string& type) :
				m_robot(robot),
				m_type(type),
				m_nodeHandle("~")
{
	std::hash<std::string> hash;
	m_uniqueName = hash(m_nodeHandle.getNamespace());

	initRobotModel(m_robot->getRobotDescription(), m_nodeHandle.getNamespace() + m_robot->getRobotDescription() + "_" + m_type, m_type,
			m_robot, m_statePublisher, m_nodeHandle);
}

RobotPoseVisualizer::~RobotPoseVisualizer()
{
	m_thread.interrupt();
	m_thread.join();

	DELETE_VAR(m_statePublisher);
}

void RobotPoseVisualizer::start()
{
	m_thread.interrupt();
	m_thread.join();
	m_thread = boost::thread(&RobotPoseVisualizer::run, this);
}

void RobotPoseVisualizer::setPose(KDL::JntArray& jointPose)
{
	boost::mutex::scoped_lock lock(m_mutex);
	m_pose = jointPose;
}

void RobotPoseVisualizer::initRobotModel(const std::string& robotDescriptionParam,
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
		boost::replace_all(desc, "name=\"" + it.second.segment.getName(),
				"name=\"" + it.second.segment.getName() + "_" + prependString + "_" + std::to_string(m_uniqueName));
		boost::replace_all(desc, "link=\"" + it.second.segment.getName(),
				"link=\"" + it.second.segment.getName() + "_" + prependString + "_" + std::to_string(m_uniqueName));
		boost::replace_all(desc, "name=\"" + it.second.segment.getJoint().getName(),
				"name=\"" + it.second.segment.getJoint().getName() + "_" + prependString + "_" + std::to_string(m_uniqueName));
	}

	statePublisher = new RobotStatePublisher(tree);

	myNodeHandle.setParam(robotDescriptionParamNew, desc);
}

void RobotPoseVisualizer::update()
{
//	int i = 0;
//	static tf::Transform baseTF = tf::Transform::getIdentity();
//
//	std::unordered_map<std::string, double> joints;
//
//	//get robot data
//	KDL::JntArray current = m_robot->getKDLJointState();
//	for (auto& it : m_robot->getJointNames())
//	{
//		joints[it] = current.data(i++, 0);
//	}
//
//	//get gripper data
//	if (m_robot->getGripper().get() != NULL)
//	{
//		auto gripperJoints = m_robot->getGripper()->getJoints();
//		for (auto& it : gripperJoints)
//		{
//			joints[it.first] = it.second;
//		}
//	}
//
//	//set joint pose
//	i = 0;
//	for (auto& it : m_robot->getChain().segments)
//	{
//		joints[it.getJoint().getName()] = m_pose.data(i++, 0);
//	}
//
//	//publish joint states of the robot
//	m_statePublisher->publishTransforms(joints, ros::Time::now(), m_robot->getName(), "_" + m_type + "_" + std::to_string(m_uniqueName));
//	m_statePublisher->publishFixedTransforms(m_robot->getName(), "_" + m_type + "_" + std::to_string(m_uniqueName));
//
//	//publish static identy transformation between robot
//	//and fake robot to be able to show the fake robot in rviz
//	m_br.sendTransform(
//			tf::StampedTransform(baseTF, ros::Time::now(), m_robot->getRootFrame(),
//					m_robot->getRootFrame() + "_" + m_type + "_" + std::to_string(m_uniqueName)));
}

void RobotPoseVisualizer::run()
{
	ros::Rate r(40);

	while (!boost::this_thread::interruption_requested())
	{
		update();
		r.sleep();
	}
}

} /* namespace prm_planner */
